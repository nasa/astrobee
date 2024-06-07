#!/usr/bin/env python3

# Copyright (c) 2024, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
Utilities for detecting and correcting bad pixels.
"""

import abc
import itertools
import json
import multiprocessing as mp
import multiprocessing.pool
import os
import pathlib
import queue
import threading
import time
from abc import abstractmethod
from dataclasses import dataclass
from typing import (
    Any,
    Callable,
    Dict,
    Generator,
    Generic,
    Iterable,
    Iterator,
    List,
    Optional,
    Tuple,
    Type,
    TypeVar,
)

import cv2
import matplotlib
import numpy as np
import rosbag
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

# pylint: disable=c-extension-no-member  # because pylint can't find cv2 members
# pylint: disable=line-too-long  # let black handle this

# Type aliases
RgbChannel = str  # "R", "G", or "B"
CoordArray = Tuple[np.ndarray, ...]  # 1D arrays: y coords, x coords
ArrayShape = Tuple[int, ...]  # nrows, ncols
RgbImage = np.ndarray  # RGB image with array shape (H, W, 3), dtype=np.uint8.
BayerImage = np.ndarray  # Raw Bayer image with array shape (H, W), dtype=np.uint8
BayerImagePatch = BayerImage  # A really small BayerImage ;)
ImageMask = np.ndarray  # An image with array shape (H, W), dtype=bool
Kernel = np.ndarray  # An NxN kernel with N odd
MonoImage = np.ndarray  # Image with array shape (H, W), dtype=np.uint8
FloatImage = np.ndarray  # Image with array shape (H, W), dtype=np.float64
CountImage = np.ndarray  # Image with array shape (H, W), dtype=np.uint32
SignedImage = np.ndarray  # Image with array shape (H, W), dtype=np.int16
BayerImageFilter = Callable[[BayerImage], BayerImage]
JsonObject = Dict[
    str, Any
]  # A mapping object compatible with json.dump() / json.load()
Self = Any  # Older mypy we're stuck on can't handle real Self type
ImageStatsT = TypeVar("ImageStatsT")  # subtype of ImageStats
T = TypeVar("T")

# https://github.com/nasa/astrobee/blob/develop/hardware/is_camera/src/camera.cc#L522
# https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
NAVCAM_BAYER_CONVENTION = "GRBG"

# Pixels are considered bad if rms_err > RMS_ERROR_THRESHOLD.
RMS_ERROR_THRESHOLD = 40

# Kernels representing 8 nearest same-color neighbors of a pixel depending on its color
RB_KERNEL = np.array(
    [
        [1, 0, 1, 0, 1],
        [0, 0, 0, 0, 0],
        [1, 0, 0, 0, 1],
        [0, 0, 0, 0, 0],
        [1, 0, 1, 0, 1],
    ],
    dtype=np.uint8,
)
G_KERNEL = np.array(
    [
        [0, 0, 1, 0, 0],
        [0, 1, 0, 1, 0],
        [1, 0, 0, 0, 1],
        [0, 1, 0, 1, 0],
        [0, 0, 1, 0, 0],
    ],
    dtype=np.uint8,
)

MF_TIMING = bool(os.environ.get("MEDIAN_FILTER_TIMING"))

# Registers the corrector class and accumulator class for making a `corrector_type` instance.
CORRECTOR_REGISTRY: Dict[
    str, "Tuple[Type[BadPixelCorrector], Type[ImageStatsAccumulator]]"
] = {}


def get_bayer_patch(bayer_convention: str) -> BayerImagePatch:
    "Return 2 x 2 `bayer_convention` patch."
    return np.array(list(bayer_convention)).reshape((2, 2))


def get_pixel_color(bayer_convention: str, y: int, x: int) -> RgbChannel:
    "Return the color of pixel `y`, `x` according to `bayer_convention`."
    return get_bayer_patch(bayer_convention)[y % 2, x % 2]


def get_bayer_mask(
    im_shape: ArrayShape, channel: RgbChannel, bayer_convention: str
) -> ImageMask:
    """
    Return a boolean mask with shape `im_shape` and True values where the Bayer pattern agrees
    with `color`, assuming `bayer_convention`.
    """
    patch = get_bayer_patch(bayer_convention) == channel
    patch_y = np.arange(im_shape[0]) % 2
    patch_x = np.arange(im_shape[1]) % 2
    grid_x, grid_y = np.meshgrid(patch_x, patch_y)
    return patch[grid_y, grid_x]


def get_bayer_neighbor_kernel(channel: RgbChannel) -> Kernel:
    """
    Return the kernel representing the 8 nearest same-color neighbors for a pixel with color
    `channel`.
    """
    return G_KERNEL if channel == "G" else RB_KERNEL


def neighbor_offsets_from_kernel(kernel: Kernel) -> CoordArray:
    "Return pixels set in `kernel` as a CoordArray."
    width, height = kernel.shape
    radius = width // 2
    assert width == 2 * radius + 1
    assert width == height
    y, x = np.nonzero(kernel)
    return (y - radius, x - radius)


def pairwise(it: Iterable[T]) -> Iterator[Tuple[T, T]]:
    "Return pairs of consecutive entries from `it`. Back-ports itertools.pairwise() to Python 3.8."
    a, b = itertools.tee(it)
    next(b)
    return zip(a, b)


def median_filter(
    im: MonoImage,
    kernel: Kernel,
    out: Optional[MonoImage] = None,
    compute_mask: Optional[np.ndarray] = None,
) -> MonoImage:
    """
    Return the result of applying a median filter to `im`.

    :param kernel: Include pixels set in `kernel` as neighbors for median calculation.
    :param out: If specified, write output to `out` rather than allocating a new array.
    :param compute_mask: If specified, only compute/set entries of `out` that are nonzero in `compute_mask`.
    """
    # pylint: disable=too-many-locals,too-many-branches,too-many-statements
    times = []
    if MF_TIMING:
        times.append(time.time())  # 0
    y_off, x_off = neighbor_offsets_from_kernel(kernel)
    h, w = im.shape  # not worrying about multi-channel images
    yvec = np.arange(h)
    xvec = np.arange(w)
    x0, y0 = np.meshgrid(xvec, yvec)
    if MF_TIMING:
        times.append(time.time())  # 1

    if out is None:
        out = np.zeros(im.shape, dtype=np.uint8)
    else:
        assert out.shape == im.shape
        assert out.dtype == np.uint8

    if compute_mask is None:
        compute_mask = np.ones(im.shape, dtype=bool)
    else:
        assert compute_mask.shape == im.shape

    # edges - slower due to use of np.ma.median() to exclude out-of-bounds neighbors
    edge_mask = np.zeros(im.shape, dtype=bool)
    edge_mask[:2, :] = compute_mask[:2, :]
    edge_mask[-2:, :] = compute_mask[-2:, :]
    edge_mask[:, :2] = compute_mask[:, :2]
    edge_mask[:, -2:] = compute_mask[:, -2:]
    edge_y, edge_x = np.nonzero(edge_mask)
    y = y0[np.newaxis, edge_y, edge_x] + y_off[:, np.newaxis, np.newaxis]
    x = x0[np.newaxis, edge_y, edge_x] + x_off[:, np.newaxis, np.newaxis]
    if MF_TIMING:
        times.append(time.time())  # 2
    valid = (0 <= y) & (y < h) & (0 <= x) & (x < w)
    if MF_TIMING:
        times.append(time.time())  # 3

    neighbor_vals = np.ma.array(np.zeros(x.shape), mask=~valid, dtype=np.uint8)
    if MF_TIMING:
        times.append(time.time())  # 4
    neighbor_vals[valid] = im[y[valid], x[valid]]
    if MF_TIMING:
        times.append(time.time())  # 5
    edge_out = np.ma.median(neighbor_vals, axis=0, overwrite_input=True)
    out[edge_y, edge_x] = np.clip(edge_out, 0, 255).astype(np.uint8)
    if MF_TIMING:
        times.append(time.time())  # 6

    # middle - runs faster using np.median()
    middle_mask = np.zeros(im.shape, dtype=bool)
    middle_mask[2:-2, 2:-2] = compute_mask[2:-2, 2:-2]
    middle_y, middle_x = np.nonzero(middle_mask)
    y = y0[np.newaxis, middle_y, middle_x] + y_off[:, np.newaxis, np.newaxis]
    x = x0[np.newaxis, middle_y, middle_x] + x_off[:, np.newaxis, np.newaxis]
    if MF_TIMING:
        times.append(time.time())  # 7

    neighbor_vals = im[y, x]
    if MF_TIMING:
        times.append(time.time())  # 8
    middle_out = np.median(neighbor_vals, axis=0, overwrite_input=True)
    if MF_TIMING:
        times.append(time.time())  # 9
    out[middle_y, middle_x] = middle_out

    if MF_TIMING:
        for i, (ti, ti_next) in enumerate(pairwise(times)):
            print(
                f"[{i}] {100 * (ti_next - ti) / (times[-1] - times[0]):5.1f}% ", end=""
            )
        print(f"[{len(times) - 1}] elapsed: {times[-1] - times[0]:.3f}s")

    return out


def estimate_from_bayer_neighbors(
    im: BayerImage,
    bayer_convention: str = NAVCAM_BAYER_CONVENTION,
    use_median: bool = True,
) -> BayerImage:
    """
    Return an image the same shape as `im` where each pixel is replaced with an estimate of its
    value based on averaging its same-color Bayer neighbors.
    """
    result = np.zeros(im.shape, dtype=np.uint8)
    for channel in "GRB":
        kernel = get_bayer_neighbor_kernel(channel)
        mask = get_bayer_mask(im.shape, channel, bayer_convention).astype(np.uint8)
        mask_ind = mask != 0
        if use_median:
            median_filter(im, kernel, out=result, compute_mask=mask)
        else:
            numer = cv2.filter2D(
                src=im * mask,
                ddepth=cv2.CV_16U,
                kernel=kernel,
                borderType=cv2.BORDER_CONSTANT,
            )
            denom = cv2.filter2D(
                src=mask,
                ddepth=cv2.CV_8U,
                kernel=kernel,
                borderType=cv2.BORDER_CONSTANT,
            )
            result[mask_ind] = numer[mask_ind] // denom[mask_ind]
    return result


class ImageStats(
    abc.ABC
):  # pylint: disable=too-few-public-methods # just an abstract parent class
    "Statistics over a set of images."


@dataclass
class DebugStats(ImageStats):  # pylint:disable=too-many-instance-attributes
    "Statistics from bad pixel analysis over a set of images."

    count: int
    "Number of images used to generate stats."

    shape: Tuple[int, ...]
    "Image shape: (height, width)"

    mean: FloatImage
    "Mean of actual value"

    stdev: FloatImage
    "Standard deviation of actual value"

    mean_err: FloatImage
    "Mean of actual - estimated"

    rms_err: FloatImage
    "RMS of actual - estimated"

    stdev_err: FloatImage
    "Standard deviation of actual - estimated"

    unsat_mean_err: FloatImage
    "Mean of actual - estimated, calculated from unsaturated samples only"

    slope: FloatImage
    "Value m from least-squares fit to linear model: estimated = m * actual + b"

    intercept: FloatImage
    "Value b from model"

    unsat_count: CountImage
    "Count of unsaturated samples"


@dataclass
class BiasStats(ImageStats):  # pylint:disable=too-many-instance-attributes
    "Statistics needed for BiasCorrector."

    unsat_mean_err: FloatImage
    "Mean of actual - estimated, calculated from unsaturated samples only"


class ImageSourcePaths:  # pylint: disable=too-few-public-methods # use __iter__()
    "A source of images read from paths. Lazy loading avoids having all images in memory."

    def __init__(self, paths: List[str]):
        """
        :param paths: Paths to read images from.
        """
        self.paths = paths

    def __iter__(self) -> Generator[BayerImage, None, None]:
        "Lazily yield images. This method gives instances of this class type Iterable[BayerImage]."
        for path in self.paths:
            im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            yield im


class ImageSourceBagPaths:  # pylint: disable=too-few-public-methods # use __iter__()
    "A source of images read from bags."

    def __init__(
        self,
        bag_paths: List[str],
        topic: str = "/hw/cam_nav_bayer",
    ):
        """
        :param bag_paths: ROS bag paths to read images from.
        :param topic: Topic that relevant images are on.
        """
        self.bag_paths = bag_paths
        self.topic = topic

    def __iter__(self) -> Generator[BayerImage, None, None]:
        "Lazily yield images. This method gives instances of this class type Iterable[BayerImage]."
        bridge = CvBridge()
        for path in self.bag_paths:
            with rosbag.Bag(path, "r") as bag:
                for _, msg, _ in bag.read_messages([self.topic]):
                    im = bridge.imgmsg_to_cv2(msg, "bayer_grbg8")
                    yield im


def image_stats_read(
    images: Iterable[BayerImage],
    input_queue: "queue.Queue[Optional[BayerImage]]",
    num_workers: int,
) -> None:
    """
    Target for reader thread. Put (lazily read) `images` into `input_queue`, then send
    `num_workers` None values to signal completion to all workers.
    """
    for im in images:
        input_queue.put(im)
    for _ in range(num_workers):
        # Each None entry signals end of input to one worker
        input_queue.put(None)


def image_stats_accum(
    args: Tuple[
        ArrayShape,
        "queue.Queue[Optional[BayerImage]]",
        Optional[BayerImageFilter],
        "Type[ImageStatsAccumulator[ImageStatsT]]",
    ],
) -> "ImageStatsAccumulator[ImageStatsT]":
    """
    Target for accumulator thread. Create an ImageStatsAccumulator with `shape`. Process incoming
    images on `input_queue` until a None entry is received, then return the accumulator.
    """
    shape, input_queue, preprocess, accum_class = args
    acc = accum_class(shape)
    while True:
        im = input_queue.get()
        if im is None:
            return acc
        if preprocess is not None:
            im = preprocess(im)
        acc.add(im)
    return acc


class ImageStatsAccumulator(abc.ABC, Generic[ImageStatsT]):
    "Abstract intermediate accumulated stats for generating a final ImageStats."

    def __init__(self, shape: ArrayShape):
        self.shape = shape

    @abstractmethod
    def add(self, im: BayerImage) -> None:
        "Add `im` to accumulated stats."

    @abstractmethod
    def merge(self, acc: Self) -> None:
        "Merge `acc` into accumulated stats."

    @abstractmethod
    def finish_stats(self) -> Optional[ImageStatsT]:
        "Return output stats based on accumulated values."

    @classmethod
    def get_image_stats(
        cls,
        images: Iterable[BayerImage],
        preprocess: Optional[BayerImageFilter] = None,
    ) -> Optional[ImageStatsT]:
        """
        Return image stats for bad pixel detection computed from `images`, or return None if there
        were not enough `images`.
        """
        for i, im in enumerate(images):
            if preprocess is not None:
                im = preprocess(im)
            if i == 0:
                acc = cls(im.shape)
            acc.add(im)
        return acc.finish_stats()

    @classmethod
    def get_image_stats_parallel(
        cls,
        images: Iterable[BayerImage],
        preprocess: Optional[BayerImageFilter] = None,
        num_workers: int = mp.cpu_count(),
    ) -> Optional[ImageStatsT]:
        """
        Return image stats for bad pixel detection computed from `images`, using `num_workers`
        accumulator workers, or return None if there were not enough `images`.
        """
        images_iterator = iter(images)
        try:
            image0 = next(images_iterator)
        except StopIteration:
            print("get_image_stats_parallel(): got no images, not generating stats")
            return None
        images = itertools.chain([image0], images_iterator)
        shape = image0.shape
        del image0

        input_queue: "queue.Queue[Optional[BayerImage]]" = queue.Queue(1)
        threading.Thread(
            target=image_stats_read, args=(images, input_queue, num_workers)
        ).start()

        with multiprocessing.pool.ThreadPool(num_workers) as pool:
            accums = pool.imap_unordered(
                image_stats_accum,  # type: ignore  # can't figure out why mypy dislikes this
                [(shape, input_queue, preprocess, cls) for _ in range(num_workers)],
            )
            for i, acc in enumerate(accums):
                if i == 0:
                    merge_acc = acc
                else:
                    merge_acc.merge(acc)
        return merge_acc.finish_stats()  # type: ignore  # mypy gives nonsensical error


class DebugStatsAccumulator(
    ImageStatsAccumulator[DebugStats]
):  # pylint: disable=too-many-instance-attributes
    "Intermediate accumulated stats for generating a final DebugStats."

    def __init__(self, shape: ArrayShape):
        super().__init__(shape)
        self.sum_actual = np.zeros(shape, dtype=np.float64)
        self.sum_actual_sq = np.zeros(shape, dtype=np.float64)
        self.sum_est = np.zeros(shape, dtype=np.float64)
        self.sum_err_sq = np.zeros(shape, dtype=np.float64)
        self.n = 0

        self.unsat_sum_actual = np.zeros(shape, dtype=np.float64)
        self.unsat_sum_actual_sq = np.zeros(shape, dtype=np.float64)
        self.unsat_sum_est = np.zeros(shape, dtype=np.float64)
        self.unsat_sum_est_sq = np.zeros(shape, dtype=np.float64)
        self.unsat_sum_err_sq = np.zeros(shape, dtype=np.float64)
        self.unsat_sum_prod = np.zeros(shape, dtype=np.float64)
        self.unsat_n = np.zeros(shape, dtype=np.uint32)

    def add(self, im: BayerImage) -> None:
        "Add `im` to accumulated stats."
        # These values are calculated for all pixels.
        est = estimate_from_bayer_neighbors(im, NAVCAM_BAYER_CONVENTION)
        self.sum_actual += im
        self.sum_actual_sq += np.square(im.astype(np.float64))
        self.sum_est += est
        self.sum_err_sq += np.square(im.astype(np.float64) - est)
        self.n += 1

        # A pixel is considered unsaturated for statistics purposes if both the pixel's actual and
        # estimated values are unsaturated.
        unsat = (im < 255) & (est < 255)
        # unsat = (im < 255).astype(np.uint8)
        # cv2.erode(unsat, kernel=np.ones((5, 5)), dst=unsat, borderType=cv2.BORDER_CONSTANT)
        unsat_ok = unsat != 0

        # These values are calculated for unsaturated pixels only.
        self.unsat_sum_actual[unsat_ok] += im[unsat_ok]
        self.unsat_sum_actual_sq[unsat_ok] += np.square(im[unsat_ok].astype(np.float64))
        self.unsat_sum_est[unsat_ok] += est[unsat_ok]
        self.unsat_sum_est_sq[unsat_ok] += np.square(est[unsat_ok].astype(np.float64))
        self.unsat_sum_err_sq[unsat_ok] += np.square(
            im[unsat_ok].astype(np.float64) - est[unsat_ok]
        )
        self.unsat_sum_prod[unsat_ok] += np.multiply(
            im[unsat_ok].astype(np.float64), est[unsat_ok]
        )
        self.unsat_n[unsat == 1] += 1

    def get_sum_fields(self) -> Tuple[Any, ...]:
        "Return array fields of self that accumulate sums."
        return (
            self.sum_actual,
            self.sum_actual_sq,
            self.sum_est,
            self.sum_err_sq,
            self.unsat_sum_actual,
            self.unsat_sum_actual_sq,
            self.unsat_sum_est,
            self.unsat_sum_est_sq,
            self.unsat_sum_err_sq,
            self.unsat_sum_prod,
            self.unsat_n,
        )

    def merge(self, acc: "DebugStatsAccumulator") -> None:
        "Merge `acc` into accumulated stats."
        for self_field, acc_field in zip(self.get_sum_fields(), acc.get_sum_fields()):
            self_field += acc_field
        # Can't be handled like the others because it's a primitive value
        self.n += acc.n

    def finish_stats(self) -> Optional[DebugStats]:
        "Return output stats based on accumulated values, or None if there were not enough images."
        if self.n < 2:
            print(
                f"DebugStatsAccumulator: expected >= 2 images, got {self.n}, not generating stats"
            )
            return None

        # standard least-squares linear regression
        unsat_ok = self.unsat_n != 0
        denom = self.unsat_n * self.unsat_sum_actual_sq - np.square(
            self.unsat_sum_actual
        )
        denom_ok = denom != 0

        slope = np.ones(self.shape, dtype=np.float64)
        slope[denom_ok] = (
            self.unsat_n[denom_ok] * self.unsat_sum_prod[denom_ok]
            - self.unsat_sum_actual[denom_ok] * self.unsat_sum_est[denom_ok]
        ) / denom[denom_ok]
        intercept = np.zeros(self.shape, dtype=np.float64)
        intercept[unsat_ok] = (
            self.unsat_sum_est[unsat_ok]
            - slope[unsat_ok] * self.unsat_sum_actual[unsat_ok]
        ) / self.unsat_n[unsat_ok]

        unsat_mean_err = np.zeros(self.shape, dtype=np.float64)
        unsat_mean_err[unsat_ok] = (
            self.unsat_sum_actual[unsat_ok] - self.unsat_sum_est[unsat_ok]
        ) / self.unsat_n[unsat_ok]

        # mypy doesn't like expressions below because np.sqrt() can blow up for negative inputs, but these
        # inputs should be guaranteed positive by construction.
        return DebugStats(
            count=self.n,
            shape=self.shape,
            mean=self.sum_actual / self.n,
            stdev=np.sqrt((self.sum_actual_sq / self.n) - np.square(self.sum_actual / self.n)),  # type: ignore
            mean_err=(self.sum_actual - self.sum_est) / self.n,
            rms_err=np.sqrt(self.sum_err_sq / self.n),  # type: ignore
            stdev_err=np.sqrt(  # type: ignore
                (self.sum_err_sq / self.n)
                - np.square((self.sum_est - self.sum_actual) / self.n)
            ),
            unsat_mean_err=unsat_mean_err,
            slope=slope,
            intercept=intercept,
            unsat_count=self.unsat_n,
        )


class BiasStatsAccumulator(
    ImageStatsAccumulator[BiasStats]
):  # pylint: disable=too-many-instance-attributes
    "Intermediate accumulated stats for generating a final BiasStats."

    def __init__(self, shape: ArrayShape):
        super().__init__(shape)
        self.n = 0
        self.unsat_n = np.zeros(shape, dtype=np.uint16)
        self.unsat_sum_err = np.zeros(shape, dtype=np.float64)

    def add(self, im: BayerImage) -> None:
        "Add `im` to accumulated stats."
        est = estimate_from_bayer_neighbors(im, NAVCAM_BAYER_CONVENTION)
        # A pixel is considered unsaturated for statistics purposes if both the pixel's actual and
        # estimated values are unsaturated.
        self.n += 1
        unsat = (im < 255) & (est < 255)
        unsat_ok = unsat != 0
        self.unsat_n[unsat_ok] += 1
        self.unsat_sum_err[unsat_ok] += im[unsat_ok].astype(np.float64) - est[unsat_ok]

    def merge(self, acc: Self) -> None:
        "Merge `acc` into accumulated stats."
        self.n += acc.n
        self.unsat_n += acc.unsat_n
        self.unsat_sum_err += acc.unsat_sum_err

    def finish_stats(self) -> Optional[BiasStats]:
        "Return output stats based on accumulated values, or None if there were not enough images."
        if self.n < 1:
            print(
                f"BiasStatsAccumulator: expected >= 1 image, got {self.n}, not generating stats"
            )
            return None

        unsat_ok = self.unsat_n != 0
        unsat_mean_err = np.zeros(self.shape, dtype=np.int16)
        unsat_mean_err[unsat_ok] = (
            self.unsat_sum_err[unsat_ok] / self.unsat_n[unsat_ok]
        ).astype(np.int16)

        return BiasStats(
            unsat_mean_err=unsat_mean_err,
        )


def plot_image_stats1(
    ax: matplotlib.axes.Axes,
    im: np.ndarray,
    title: str,
    colorbar: bool = True,
    crop_center: float = 1.0,
    **kwargs,
) -> None:
    """
    Plot `im` on axes `ax` with `title`.

    :param colorbar: Whether to plot a colorbar beside the image.
    :param crop_center: If < 1.0, crop the image to the specified proportion before plotting.
    :param kwargs: Unrecognized arguments are passed through to plt.imshow().
    """
    if crop_center < 1.0:
        h_in, w_in = im.shape[:2]
        h = int(h_in * crop_center)
        w = int(w_in * crop_center)
        x = (w_in - w) // 2
        y = (h_in - h) // 2
        if len(im.shape) == 2:
            im = im[y : (y + h), x : (x + w)]
        else:
            im = im[y : (y + h), x : (x + w), :]
    obj = ax.imshow(im, **kwargs)
    ax.axis("off")
    if colorbar:
        plt.colorbar(obj, ax=ax, fraction=0.035, pad=0.04)
    ax.set_title(title)


def plot_image_stats(stats: DebugStats) -> None:
    "Plot a standard set of images representing `stats`."
    fig, axes = plt.subplots(6, 1)

    plot_image_stats1(axes[0], stats.mean, "mean actual")
    plot_image_stats1(axes[1], stats.mean_err, "mean err (actual - estimated)")
    plot_image_stats1(axes[2], stats.rms_err, "RMS err (actual - estimated)")
    plot_image_stats1(axes[3], stats.slope, "Slope (actual vs. estimated)")
    plot_image_stats1(axes[4], stats.intercept, "Intercept (actual when estimated = 0)")
    plot_image_stats1(axes[5], stats.unsat_count, "Count of unsaturated frames")

    fig.set_size_inches((96, 240))


def get_bad_pixel_coords(stats: DebugStats) -> CoordArray:
    "Return coordinates of bad pixels given image stats."
    # Arbitrary criterion, may reconsider
    return np.nonzero(stats.rms_err >= RMS_ERROR_THRESHOLD)


class BadPixelCorrector(abc.ABC):
    "Abstract filter for correcting bad pixels."

    def __init__(self, note: str):
        self.note = note

    @abstractmethod
    def __call__(self, im: BayerImage) -> BayerImage:
        "Return the result of correcting `im`."

    @classmethod
    def from_json_object(cls, obj: JsonObject, path: pathlib.Path) -> Self:
        "Return an instance of `cls` constructed from `obj`."
        subclass, _ = cls.get_classes(obj["type"])
        return subclass.from_json_object(obj, path)

    @abstractmethod
    def to_json_object(self, path: pathlib.Path) -> JsonObject:
        "Return the JsonObject representation of the corrector."

    @classmethod
    def load(cls, path: pathlib.Path) -> Self:
        "Return an instance of `cls` loaded from JSON file at `path`."
        with path.open("r", encoding="utf-8") as stream:
            return cls.from_json_object(json.load(stream), path)

    def save(self, path: pathlib.Path) -> None:
        "Write corrector JSON representation to `path`."
        with path.open("w", encoding="utf-8") as stream:
            json.dump(self.to_json_object(path), stream, separators=(",", ":"))
        print(f"Wrote to {path}")

    @classmethod
    @abstractmethod
    def from_image_stats(cls, stats: ImageStats, note: str = "") -> Self:
        "Return an instance of `cls` constructed from `stats`."

    @staticmethod
    def get_classes(
        corrector_type: str,
    ) -> "Tuple[Type[BadPixelCorrector], Type[ImageStatsAccumulator]]":
        "Return the corrector class and accumulator class for making a `corrector_type` instance."
        return CORRECTOR_REGISTRY[corrector_type]


@dataclass
class NeighborBadPixel:
    "Represents a bad pixel to be corrected."

    y: int
    "Y coordinate of the bad pixel"

    x: int
    "X coordinate of the bad pixel"

    y_neighbors: np.ndarray
    "Y coordinates of neighbor pixels to use for interpolation (1D array)."

    x_neighbors: np.ndarray
    "X coordinates of neighbor pixels to use for interpolation (1D array)."

    order: int
    "Determines order to fill in bad pixels. Earlier order filled sooner."


class NeighborMeanCorrector(
    BadPixelCorrector
):  # pylint:disable=too-few-public-methods # use __call__()
    "Filter for replacing bad pixel values with the mean of their neighbors."

    def __init__(self, image_shape: ArrayShape, bad_coords: CoordArray, note: str = ""):
        """
        :param image_shape: The shape of images that will be corrected.
        :param bad_coords: Bad pixel coords represented as tuple (y, x) of 1D arrays
        """
        super().__init__(note)
        self.image_shape = image_shape
        self.bad_pixels: List[NeighborBadPixel] = []
        self._init_filter(bad_coords)

    def _init_filter(self, bad_coords: CoordArray) -> None:
        "Initialize self.bad_pixels for interpolation."
        bad_image = np.zeros(self.image_shape, dtype=bool)
        bad_image[tuple(bad_coords)] = True
        for y, x in zip(*bad_coords):
            kernel = get_bayer_neighbor_kernel(
                get_pixel_color(NAVCAM_BAYER_CONVENTION, y=y, x=x)
            )
            y_offsets, x_offsets = neighbor_offsets_from_kernel(kernel)
            y_neighbors = y_offsets + y
            x_neighbors = x_offsets + x

            ny, nx = self.image_shape
            in_bounds = (
                (0 <= y_neighbors)
                & (y_neighbors < ny)
                & (0 <= x_neighbors)
                & (x_neighbors < nx)
            )
            y_neighbors = y_neighbors[in_bounds]
            x_neighbors = x_neighbors[in_bounds]

            # pylint:disable-next=singleton-comparison # this is an array operation
            not_bad = bad_image[y_neighbors, x_neighbors] != True
            if not_bad.any():
                y_neighbors = y_neighbors[not_bad]
                x_neighbors = x_neighbors[not_bad]
                order = 0
            else:
                # If all neighbors are bad, fill them first, then use
                # them to fill this pixel.
                order = 1

            self.bad_pixels.append(
                NeighborBadPixel(
                    y=int(y),
                    x=int(x),
                    y_neighbors=y_neighbors,
                    x_neighbors=x_neighbors,
                    order=order,
                )
            )
        self.bad_pixels = sorted(self.bad_pixels, key=lambda pix: pix.order)

    def __call__(self, im: np.ndarray) -> np.ndarray:
        "Return the result of correcting `im`."
        result = im.copy()
        for pix in self.bad_pixels:
            n = len(pix.y_neighbors)
            result[pix.y, pix.x] = im[pix.y_neighbors, pix.x_neighbors].sum() / n
        return result

    @classmethod
    def from_json_object(
        cls, obj: JsonObject, path: pathlib.Path
    ) -> "NeighborMeanCorrector":
        "Return an instance of `cls` constructed from `obj`."
        assert obj["type"] == "NeighborMeanCorrector"
        assert obj["version"] == "1"
        return cls(
            image_shape=(obj["height"], obj["width"]),
            bad_coords=(obj["y"], obj["x"]),
            note=obj["note"],
        )

    def to_json_object(self, path: pathlib.Path) -> JsonObject:
        "Return the JsonObject representation of the corrector."
        bad_coord_tuples = [(pix.y, pix.x) for pix in self.bad_pixels]
        y, x = list(zip(*bad_coord_tuples))
        height, width = self.image_shape
        return {
            "type": "NeighborMeanCorrector",
            "version": "1",
            "implementation": "https://github.com/nasa/astrobee/tree/develop/localization/camera/src/camera/pixel_utils.py",
            "width": width,
            "height": height,
            "x": x,
            "y": y,
            "note": self.note,
        }

    @classmethod
    def from_image_stats(cls, stats: ImageStats, note: str = "") -> Self:
        "Return a NeighborMeanCorrector constructed from `stats`."
        assert isinstance(stats, DebugStats)
        bad_coords = get_bad_pixel_coords(stats)
        return cls(
            image_shape=stats.shape,
            bad_coords=bad_coords,
            note=note,
        )


class BiasCorrector(
    BadPixelCorrector
):  # pylint:disable=too-few-public-methods # use __call__()
    """
    Filter for bias correction of pixel values. Two steps are applied:
    1) All pixels are corrected according to: v_out = v_in - bias
    2) At pixel locations where the original v_in was saturated, v_out is replaced with the
       mean of its eight closest same-color Bayer neighbors.
    """

    def __init__(self, bias: SignedImage, note: str = ""):
        """
        :param bias: Bias represented as int8.
        """
        super().__init__(note)
        assert bias.dtype == np.int16
        self.bias = bias

    def __call__(self, im: BayerImage) -> BayerImage:
        "Return the result of correcting `im`."
        result = self.apply_bias(im)
        self.correct_saturated_from_neighbors(im, result)
        return result

    def apply_bias(self, im: BayerImage) -> BayerImage:
        "Return the result of applying the affine correction to `im`."
        assert im.dtype == np.uint8
        result16 = im.astype(np.int16)
        result16 -= self.bias
        return result16.clip(0, 255).astype(np.uint8)

    def correct_saturated_from_neighbors(
        self, im: BayerImage, result: BayerImage
    ) -> None:
        """
        Apply an in-place correction to `result` at locations where `im` has saturated pixels.
        The correction replaces each value in `result` with the mean of its 8 closest same-color
        neighbors.
        """
        sat = im == 255
        # This could probably be done more efficiently because saturated pixels are very sparse.
        est = estimate_from_bayer_neighbors(
            result, NAVCAM_BAYER_CONVENTION, use_median=False
        )
        result[sat] = est[sat]

    @classmethod
    def from_image_stats(cls, stats: ImageStats, note: str = "") -> Self:
        "Return a BiasCorrector constructed from `stats`."
        assert isinstance(stats, (BiasStats, DebugStats))
        return cls(
            bias=np.clip(stats.unsat_mean_err, -32768, 32767).astype(np.int16),
            note=note,
        )

    @staticmethod
    def write_png(path: pathlib.Path, im: SignedImage) -> None:
        "Write `im` (-256..255, np.int16) to `path` in low 9 bits of 16-bit PNG format."
        cv2.imwrite(str(path), (im + 256).view(np.uint16))

    @staticmethod
    def read_png(path: pathlib.Path) -> SignedImage:
        "Return the image (-256..255, np.int16) read from low 9 bits of 16-bit PNG at `path`."
        result = cv2.imread(str(path), cv2.IMREAD_ANYDEPTH)
        assert result.dtype == np.uint16
        return result.astype(np.int16) - 256

    @classmethod
    def from_json_object(cls, obj: JsonObject, path: pathlib.Path) -> "BiasCorrector":
        "Return an instance of `cls` constructed from `obj`."
        assert obj["type"] == "BiasCorrector"
        assert obj["version"] == "1"
        bias = cls.read_png(path.parent / obj["bias"])
        note = obj["note"]
        return cls(bias=bias, note=note)

    def to_json_object(self, path: pathlib.Path) -> JsonObject:
        "Return the JsonObject representation of the corrector."
        bias_path = path.parent / (path.stem + "_bias.png")
        self.write_png(bias_path, self.bias)
        return {
            "type": "BiasCorrector",
            "version": "1",
            "implementation": "https://github.com/nasa/astrobee/tree/develop/localization/camera/src/camera/pixel_utils.py",
            "bias": str(bias_path.name),
            "note": self.note,
        }

    def assert_equal(self, other: "BiasCorrector") -> None:
        "Raise AssertionError if `self` and `other` are not equal."
        np.testing.assert_equal(self.bias, other.bias)


def coords_union(coords1: CoordArray, coords2: CoordArray) -> CoordArray:
    "Return the union of two coordinate arrays, eliminating duplicates."
    y1, x1 = coords1
    y2, x2 = coords2

    # concatenate
    result0 = (
        np.concatenate((y1, y2)),
        np.concatenate((x1, x2)),
    )

    # remove duplicates
    union_set = set(zip(*result0))
    y, x = zip(*list(union_set))

    return np.array(y), np.array(x)


def plot_mean_vs_rms_err(
    ax: matplotlib.axes.Axes, stats: DebugStats, title: str
) -> None:
    "Plot `stats.mean_err` vs. `stats.rms_err`."
    fields = {
        "RMS error": stats.rms_err,
        "Mean error": stats.mean_err,
    }
    x, y = fields.values()
    hist, xedges, yedges = np.histogram2d(x.flatten(), y.flatten(), bins=300)
    plt.jet()
    disp = np.log(hist.T + 1)
    disp[disp != 0] += 5  # type: ignore # improve visibility of single points
    ax.imshow(
        disp,
        interpolation="lanczos",
        origin="low",
        extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
    )
    xlabel, ylabel = fields.keys()
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)


def get_levels(stats: DebugStats) -> np.ndarray:
    "Return relative mean levels for RGB channels in `stats` as a 3-vector."
    levels = np.zeros(3)
    for i, channel in enumerate("RGB"):
        mask = get_bayer_mask(stats.shape, channel, NAVCAM_BAYER_CONVENTION)
        levels[i] = stats.mean[mask != 0].sum()
        if channel == "G":
            levels[i] *= 0.5  # Correct for twice as many G pixels
    levels /= levels.min()
    return levels


def fix_levels(im: RgbImage, levels: np.ndarray, scale: float) -> RgbImage:
    """
    Return `im` white-balanced to make RGB `levels` even, and optional contrast stretch with `scale`.
    :param levels: Levels returned by `get_levels()`.
    :param scale: Specify scale factor > 1 if contrast stretch is desired (usually helps).
    """
    coeff = scale / levels
    return np.clip(np.multiply(im, coeff[np.newaxis, np.newaxis, :]), 0, 255).astype(
        np.uint8
    )


def label_with_circles(im: RgbImage, coords: CoordArray, radius: int = 7) -> RgbImage:
    """
    Return the result of labeling `im` with white circles at `coords` of specified `radius`.
    """
    im_copy = im.copy()
    white = (255, 255, 255)
    for y, x in zip(*coords):
        cv2.circle(im_copy, center=(x, y), radius=radius, color=white, thickness=1)
    return im_copy


def plot_image_correction_example(
    im_bayer: BayerImage,
    corrector: BadPixelCorrector,
    levels: np.ndarray,
    scale: float = 1.8,
    bad_coords: Optional[CoordArray] = None,
):
    """
    Plot how an image looks before and after correction, zooming in on the center. (This creates
    a new figure with two subplots.)
    """
    corr_bayer = corrector(im_bayer)
    im = fix_levels(cv2.cvtColor(im_bayer, cv2.COLOR_BayerGB2RGB), levels, scale=scale)
    corr = fix_levels(
        cv2.cvtColor(corr_bayer, cv2.COLOR_BayerGB2RGB), levels, scale=scale
    )

    if bad_coords is not None:
        im = label_with_circles(im, bad_coords)
        corr = label_with_circles(corr, bad_coords)

    images_to_plot = {
        "Raw image": im,
        "Corrected image": corr,
    }
    fig, axes = plt.subplots(1, len(images_to_plot))
    for i, (title, image) in enumerate(images_to_plot.items()):
        plot_image_stats1(axes[i], image, title, colorbar=False, crop_center=0.15)
    return fig


def plot_bayer_crop_example(ax: matplotlib.axes.Axes, im_bayer: BayerImage) -> None:
    "Plot a zoomed-in example of what raw Bayer imagery looks like."
    fig, ax = plt.subplots()
    plot_image_stats1(
        ax, im_bayer, "bayer", colorbar=False, crop_center=0.15, cmap="gray"
    )
    fig.set_size_inches((100, 75))


CORRECTOR_REGISTRY = {
    "BiasCorrector": (BiasCorrector, BiasStatsAccumulator),
    "NeighborMeanCorrector": (NeighborMeanCorrector, DebugStatsAccumulator),
}

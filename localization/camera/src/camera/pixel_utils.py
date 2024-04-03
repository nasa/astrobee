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
import base64
import glob
import itertools
import json
import multiprocessing as mp
import multiprocessing.pool
import pathlib
import queue
import threading
from abc import abstractmethod
from dataclasses import dataclass
from typing import Any, Callable, Dict, Generator, Iterable, List, Optional, Tuple

import cv2
import matplotlib
import numpy as np
import rosbag
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

# pylint: disable=c-extension-no-member  # because pylint can't find cv2 members

# Type aliases
RgbChannel = str  # "R", "G", or "B"
CoordArray = Tuple[np.ndarray, ...]  # 1D arrays: y coords, x coords
ArrayShape = Tuple[int, ...]  # nrows, ncols
RgbImage = np.ndarray  # RGB image with array shape (H, W, 3), dtype=np.uint8.
BayerImage = np.ndarray  # Raw Bayer image with array shape (H, W), dtype=np.uint8
BayerImagePatch = BayerImage  # A really small BayerImage ;)
ImageMask = np.ndarray  # An image with array shape (H, W), dtype=np.bool
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

# https://github.com/nasa/astrobee/blob/develop/hardware/is_camera/src/camera.cc#L522
# https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
NAVCAM_BAYER_CONVENTION = "GRBG"

# Pixels are considered bad if rms_err > RMS_ERROR_THRESHOLD.
RMS_ERROR_THRESHOLD = 40

PNG_BASE64_PREFIX = "png:base64:"

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


def median_filter(im: MonoImage, kernel: Kernel) -> MonoImage:
    """
    Return the result of applying a median filter to `im`.

    :param kernel: Include pixels set in kernel as neighbors for median calculation.
    """
    y_off, x_off = neighbor_offsets_from_kernel(kernel)
    h, w = im.shape  # not worrying about multi-channel images
    yvec = np.arange(h)
    xvec = np.arange(w)
    x0, y0 = np.meshgrid(xvec, yvec)

    y = y0[np.newaxis, :, :] + y_off[:, np.newaxis, np.newaxis]
    x = x0[np.newaxis, :, :] + x_off[:, np.newaxis, np.newaxis]
    valid = (0 <= y) & (y < h) & (0 <= x) & (x < w)

    neighbor_vals = np.ma.array(np.zeros(x.shape), mask=~valid)
    neighbor_vals[valid] = im[y[valid], x[valid]]
    result = np.ma.median(neighbor_vals, axis=0, overwrite_input=True)
    return result


def estimate_from_bayer_neighbors(
    im: BayerImage, bayer_convention: str, use_median: bool = True
) -> BayerImage:
    """
    Return an image the same shape as `im` where each pixel is replaced with an estimate of its
    value based on averaging its same-color Bayer neighbors.
    """
    result = np.zeros(im.shape, dtype=np.float64)
    for channel in "GRB":
        kernel = get_bayer_neighbor_kernel(channel)
        mask = get_bayer_mask(im.shape, channel, bayer_convention).astype(np.uint8)
        mask_ind = mask != 0
        if use_median:
            result[mask_ind] = median_filter(im, kernel)[mask_ind]
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


@dataclass
class ImageStats:  # pylint:disable=too-many-instance-attributes
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


class ImageStatsAccumulator:  # pylint: disable=too-many-instance-attributes
    "Represents intermediate accumulated stats for generating a final ImageStats."

    def __init__(self, shape: ArrayShape):
        self.shape = shape
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
        self.sum_actual_sq += np.square(im, dtype=np.float64)
        self.sum_est += est
        self.sum_err_sq += np.square(im - est, dtype=np.float64)
        self.n += 1

        # A pixel is considered unsaturated for statistics purposes if both the pixel's actual and
        # estimated values are unsaturated.
        unsat = (im < 255) & (est < 255)
        unsat_ok = unsat != 0

        # These values are calculated for unsaturated pixels only.
        self.unsat_sum_actual[unsat_ok] += im[unsat_ok]
        self.unsat_sum_actual_sq[unsat_ok] += np.square(im[unsat_ok], dtype=np.float64)
        self.unsat_sum_est[unsat_ok] += est[unsat_ok]
        self.unsat_sum_est_sq[unsat_ok] += np.square(est[unsat_ok], dtype=np.float64)
        self.unsat_sum_err_sq[unsat_ok] += np.square(
            im[unsat_ok] - est[unsat_ok], dtype=np.float64
        )
        self.unsat_sum_prod[unsat_ok] += np.multiply(
            im[unsat_ok], est[unsat_ok], dtype=np.float64
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

    def merge(self, acc: "ImageStatsAccumulator") -> None:
        "Merge `acc` into accumulated stats."
        for self_field, acc_field in zip(self.get_sum_fields(), acc.get_sum_fields()):
            self_field += acc_field
        # Can't be handled like the others because it's a primitive value
        self.n += acc.n

    def get_stats(self) -> ImageStats:
        "Return output stats based on accumulated values."
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
        return ImageStats(
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


def get_image_stats(
    images: Iterable[BayerImage], preprocess: Optional[BayerImageFilter] = None
) -> ImageStats:
    "Return image stats for bad pixel detection computed from `images`."
    for i, im in enumerate(images):
        if preprocess is not None:
            im = preprocess(im)
        if i == 0:
            acc = ImageStatsAccumulator(im.shape)
        acc.add(im)
    return acc.get_stats()


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
        ArrayShape, "queue.Queue[Optional[BayerImage]]", Optional[BayerImageFilter]
    ],
) -> ImageStatsAccumulator:
    """
    Target for accumulator thread. Create an ImageStatsAccumulator with `shape`. Process incoming
    images on `input_queue` until a None entry is received, then return the accumulator.
    """
    shape, input_queue, preprocess = args
    acc = ImageStatsAccumulator(shape)
    while True:
        im = input_queue.get()
        if im is None:
            return acc
        if preprocess is not None:
            im = preprocess(im)
        acc.add(im)
    return acc


def get_image_stats_parallel(
    images: Iterable[BayerImage],
    preprocess: Optional[BayerImageFilter] = None,
    num_workers: int = mp.cpu_count(),
) -> ImageStats:
    """
    Return image stats for bad pixel detection computed from `images`, using `num_workers`
    accumulator workers.
    """
    images, images_copy = itertools.tee(images)
    shape = next(images_copy).shape
    del images_copy

    input_queue: "queue.Queue[Optional[BayerImage]]" = queue.Queue(1)
    threading.Thread(
        target=image_stats_read, args=(images, input_queue, num_workers)
    ).start()

    with multiprocessing.pool.ThreadPool(num_workers) as pool:
        accums = pool.imap_unordered(
            image_stats_accum,
            [(shape, input_queue, preprocess) for _ in range(num_workers)],
        )
        for i, acc in enumerate(accums):
            if i == 0:
                merge_acc = acc
            else:
                merge_acc.merge(acc)
    return merge_acc.get_stats()


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


def plot_image_stats(stats: ImageStats) -> None:
    "Plot a standard set of images representing `stats`."
    fig, axes = plt.subplots(6, 1)

    plot_image_stats1(axes[0], stats.mean, "mean actual")
    plot_image_stats1(axes[1], stats.mean_err, "mean err (actual - estimated)")
    plot_image_stats1(axes[2], stats.rms_err, "RMS err (actual - estimated)")
    plot_image_stats1(axes[3], stats.slope, "Slope (actual vs. estimated)")
    plot_image_stats1(axes[4], stats.intercept, "Intercept (actual when estimated = 0)")
    plot_image_stats1(axes[5], stats.unsat_count, "Count of unsaturated frames")

    fig.set_size_inches((96, 240))


def get_bad_pixel_coords(stats: ImageStats) -> CoordArray:
    "Return coordinates of bad pixels given image stats."
    # Arbitrary criterion, may reconsider
    return np.nonzero(stats.rms_err >= RMS_ERROR_THRESHOLD)


class BadPixelCorrector(abc.ABC):
    "Abstract filter for correcting bad pixels."

    @abstractmethod
    def __call__(self, im: BayerImage) -> BayerImage:
        "Return the result of correcting `im`."

    @classmethod
    @abstractmethod
    def from_json_object(cls, obj: JsonObject) -> Self:
        "Return an instance of `cls` constructed from `obj`."

    @abstractmethod
    def to_json_object(self) -> JsonObject:
        "Return the JsonObject representation of the corrector."

    @classmethod
    def load(cls, path: pathlib.Path) -> Self:
        "Return an instance of `cls` loaded from JSON file at `path`."
        with path.open("r", encoding="utf-8") as stream:
            return cls.from_json_object(json.load(stream))

    def save(self, path: pathlib.Path) -> None:
        "Write corrector JSON representation to `path`."
        with path.open("w", encoding="utf-8") as stream:
            json.dump(self.to_json_object(), stream, separators=(",", ":"))
        print(f"Wrote to {path}")


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

    def __init__(self, image_shape: ArrayShape, bad_coords: CoordArray):
        """
        :param image_shape: The shape of images that will be corrected.
        :param bad_coords: Bad pixel coords represented as tuple (y, x) of 1D arrays
        """
        self.image_shape = image_shape
        self.bad_pixels: List[NeighborBadPixel] = []
        self._init_filter(bad_coords)

    def _init_filter(self, bad_coords: CoordArray) -> None:
        "Initialize self.bad_pixels for interpolation."
        bad_image = np.zeros(self.image_shape, dtype=np.bool)
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
    def from_json_object(cls, obj: JsonObject) -> "NeighborMeanCorrector":
        "Return an instance of `cls` constructed from `obj`."
        assert obj["type"] == "NeighborMeanCorrector"
        assert obj["version"] == "1"
        return cls(
            image_shape=(obj["height"], obj["width"]), bad_coords=(obj["y"], obj["x"])
        )

    def to_json_object(self) -> JsonObject:
        "Return the JsonObject representation of the corrector."
        bad_coord_tuples = [(pix.y, pix.x) for pix in self.bad_pixels]
        y, x = list(zip(*bad_coord_tuples))
        height, width = self.image_shape
        return {
            "type": "NeighborMeanCorrector",
            "version": "1",
            "width": width,
            "height": height,
            "x": x,
            "y": y,
        }


class BiasCorrector(
    BadPixelCorrector
):  # pylint:disable=too-few-public-methods # use __call__()
    """
    Filter for bias correction of pixel values. Two steps are applied:
    1) All pixels are corrected according to: v_out = v_in - bias
    2) At pixel locations where the original v_in was saturated, v_out is replaced with the
       mean of its eight closest same-color Bayer neighbors.
    """

    def __init__(self, bias: SignedImage):
        """
        :param bias: Bias represented as int8.
        """
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

    @staticmethod
    def from_image_stats(stats: ImageStats) -> "BiasCorrector":
        "Return a BiasCorrector constructed from `stats`."
        return BiasCorrector(
            bias=np.clip(stats.unsat_mean_err, -32768, 32767).astype(np.int16)
        )

    @staticmethod
    def to_png_base64(im: SignedImage) -> str:
        "Return the png:base64 JSON encoded from `im`."
        return PNG_BASE64_PREFIX + base64.b64encode(
            cv2.imencode(".png", im.view(np.uint16))[1].tobytes()
        ).decode("ascii")

    @staticmethod
    def from_png_base64(buf: str) -> SignedImage:
        "Return the SignedImage decoded from png:base64 JSON `buf`."
        if not buf.startswith(PNG_BASE64_PREFIX):
            raise ValueError(
                f"Expected png:base64 JSON field to start with '{PNG_BASE64_PREFIX}'"
            )
        data = buf[len(PNG_BASE64_PREFIX) :]
        result = cv2.imdecode(
            np.frombuffer(base64.b64decode(data.encode("ascii")), dtype=np.uint8),
            cv2.IMREAD_ANYDEPTH,
        )
        assert result.dtype == np.uint16
        return result.astype(np.int16)

    @classmethod
    def from_json_object(cls, obj: JsonObject) -> "BiasCorrector":
        "Return an instance of `cls` constructed from `obj`."
        assert obj["type"] == "BiasCorrector"
        assert obj["version"] == "1"
        return cls(
            bias=cls.from_png_base64(obj["bias"]),
        )

    def to_json_object(self) -> JsonObject:
        "Return the JsonObject representation of the corrector."
        return {
            "type": "BiasCorrector",
            "version": "1",
            "bias": self.to_png_base64(self.bias),
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


def plot_mean_vs_rms_error(stats: ImageStats) -> None:
    "Plot mean vs. RMS error."
    plt.scatter(stats.rms_err.flatten(), stats.mean_err.flatten())
    plt.xlabel("RMS err")
    plt.ylabel("Mean err")


def get_levels(stats: ImageStats) -> np.ndarray:
    "Return relative mean levels for RGB channels in `stats` as a 3-vector."
    levels = np.zeros(3)
    for i, channel in enumerate("RGB"):
        mask = get_bayer_mask(stats.mean.shape, channel, NAVCAM_BAYER_CONVENTION)
        levels[i] = stats.mean[mask != 0].sum()
        if channel == "G":
            levels[i] *= 0.5  # Correct for twice as many G pixels
    levels /= levels.min()
    return levels


def fix_levels(im: RgbImage, levels: np.ndarray, scale: float) -> RgbImage:
    """
    Return `im` adjusted to make RGB `levels` even, and optional contrast stretch with `scale`.
    :param levels: Levels returned by `get_levels()`.
    :param scale: Specify scale factor > 1 if contrast stretch is desired (usually helps).
    """
    inv = 1.0 / levels
    return np.clip(
        np.multiply(im, scale * inv[np.newaxis, np.newaxis, :]), 0, 255
    ).astype(np.uint8)


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
    corrector: NeighborMeanCorrector,
    levels: np.ndarray,
    scale: float = 1.8,
    bad_coords: Optional[CoordArray] = None,
) -> None:
    """
    Plot how an image looks before and after correction, zooming in on the center.
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
        "input": im,
        "corrected": corr,
    }
    fig, axes = plt.subplots(1, len(images_to_plot))
    for i, (title, image) in enumerate(images_to_plot.items()):
        plot_image_stats1(axes[i], image, title, colorbar=False, crop_center=0.15)
    fig.set_size_inches((30, 40 * len(images_to_plot)))
    plt.tight_layout()


def plot_bayer_crop_example(ax: matplotlib.axes.Axes, im_bayer: BayerImage) -> None:
    "Plot a zoomed-in example of what raw Bayer imagery looks like."
    fig, ax = plt.subplots()
    plot_image_stats1(
        ax, im_bayer, "bayer", colorbar=False, crop_center=0.15, cmap="gray"
    )
    fig.set_size_inches((100, 75))


def process_images() -> None:
    """
    Build up a corrector through multiple rounds of analyze/correct. Can probably do this
    in one pass now that we use median_filter().
    """

    # limit number of images to run faster
    limit_images = 1000

    paths = glob.glob("images/*.png")[:limit_images]

    # This subset of frames is dark enough that we can think of them as dark frames; can reduce artifacts
    # first_frame = "images/1703871209207275120.png"
    # last_frame = "images/1703871226255847821.png"
    # paths = [path for path in paths if first_frame <= path <= last_frame]

    stats = get_image_stats(ImageSourcePaths(paths))
    plot_image_stats(stats)

    bad_coords = get_bad_pixel_coords(stats)
    corrector = NeighborMeanCorrector(stats.mean.shape, bad_coords)

    stats2 = get_image_stats(ImageSourcePaths(paths), preprocess=corrector)
    plot_image_stats(stats2)

    bad_coords2 = coords_union(bad_coords, get_bad_pixel_coords(stats2))
    corrector2 = NeighborMeanCorrector(stats.mean.shape, bad_coords2)
    stats3 = get_image_stats(ImageSourcePaths(paths), preprocess=corrector2)
    plot_image_stats(stats3)

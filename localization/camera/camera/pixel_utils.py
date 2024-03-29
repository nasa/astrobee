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

import glob
from dataclasses import dataclass
from typing import Callable, Generator, Iterable, List, Optional, Tuple

import cv2
import matplotlib
import numpy as np
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
BayerImageFilter = Callable[[BayerImage], BayerImage]

# https://github.com/nasa/astrobee/blob/develop/hardware/is_camera/src/camera.cc#L522
# https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
NAVCAM_BAYER_CONVENTION = "GRBG"

# Pixels are considered saturated during analysis if >= SAT_THRESHOLD.
SAT_THRESHOLD = 256  # Value >255 effectively disables saturation check for now

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
    result = np.ma.median(neighbor_vals, axis=0)
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

    slope: FloatImage
    "Value m from: actual = m * estimated + b"

    intercept: FloatImage
    "Value b from: actual = m * estimated + b"

    unsat_count: CountImage
    "Count of frames in which pixel and neighbors were unsaturated"


class ImageSourcePaths:  # pylint: disable=too-few-public-methods # use __iter__()
    "A source of images read from paths. Lazy loading avoids having all images in memory."

    def __init__(self, paths: List[str], preprocess: Optional[BayerImageFilter] = None):
        """
        :param paths: Paths to read images from.
        :param preprocess: If specified, apply this filter to incoming images before analysis.
        """
        self.paths = paths
        self.preprocess = preprocess

    def __iter__(self) -> Generator[BayerImage, None, None]:
        "Lazily yield images. This method gives instances of this class type Iterable[BayerImage]."
        for path in self.paths:
            im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if self.preprocess is not None:
                im = self.preprocess(im)
            yield im


def get_image_stats(images: Iterable[BayerImage]) -> ImageStats:
    """
    Compute image stats for bad pixel detection from `images`.
    """

    box55 = np.ones(
        (5, 5), dtype=np.uint8
    )  # overkill: better to use bayer-aware kernel here
    for i, im in enumerate(images):
        if i == 0:
            sum_actual = np.zeros(im.shape, dtype=np.float64)
            sum_actual_sq = np.zeros(im.shape, dtype=np.float64)
            sum_est = np.zeros(im.shape, dtype=np.float64)
            sum_est_sq = np.zeros(im.shape, dtype=np.float64)
            sum_err_sq = np.zeros(im.shape, dtype=np.float64)
            sum_prod = np.zeros(im.shape, dtype=np.float64)
            unsat_count = np.zeros(im.shape, dtype=np.uint32)
        est = estimate_from_bayer_neighbors(im, NAVCAM_BAYER_CONVENTION)
        unsat0 = (im < SAT_THRESHOLD).astype(np.uint8)
        unsat = cv2.erode(unsat0, kernel=box55, borderType=cv2.BORDER_REPLICATE)
        valid = unsat != 0
        sum_actual[valid] += im[valid]
        sum_actual_sq[valid] += np.square(im[valid], dtype=np.float64)
        sum_est[valid] += est[valid]
        sum_est_sq[valid] += np.square(est[valid], dtype=np.float64)
        sum_err_sq[valid] += np.square(im[valid] - est[valid], dtype=np.float64)
        sum_prod[valid] += np.multiply(im[valid], est[valid], dtype=np.float64)
        unsat_count[unsat == 1] += 1

    n = unsat_count

    # standard least-squares linear regression
    denom = n * sum_est_sq - np.square(sum_est)
    nz = denom != 0
    slope = np.ones(sum_prod.shape, dtype=np.float64)
    slope[nz] = (n[nz] * sum_prod[nz] - sum_est[nz] * sum_actual[nz]) / denom[nz]
    intercept = (sum_actual - slope * sum_est) / n

    # Type-checking errors below because np.sqrt() can blow up for negative inputs, but these
    # inputs should be guaranteed positive by construction.
    return ImageStats(
        mean=sum_actual / n,
        stdev=np.sqrt((sum_actual_sq / n) - np.square(sum_actual / n)),  # type: ignore
        mean_err=(sum_actual - sum_est) / n,
        rms_err=np.sqrt(sum_err_sq / n),  # type: ignore
        stdev_err=np.sqrt((sum_err_sq / n) - np.square((sum_est - sum_actual) / n)),  # type: ignore
        slope=slope,
        intercept=intercept,
        unsat_count=unsat_count,
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


@dataclass
class BadPixel:
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


class BadPixelCorrector:  # pylint:disable=too-few-public-methods # use __call__()
    "A processor that can correct bad pixels in images."

    def __init__(self, image_shape: ArrayShape, bad_coords: CoordArray):
        """
        :param image_shape: The shape of images that will be corrected.
        :param bad_coords: Bad pixel coords represented as tuple (y, x) of 1D arrays
        """
        self.image_shape = image_shape
        self.bad_pixels: List[BadPixel] = []
        self._init_filter(bad_coords)

    def _init_filter(self, bad_coords: CoordArray) -> None:
        "Initialize self.bad_pixels for interpolation."
        bad_image = np.zeros(self.image_shape, dtype=np.bool)
        bad_image[bad_coords] = True
        for y, x in zip(*bad_coords):
            color = get_pixel_color(NAVCAM_BAYER_CONVENTION, y=y, x=x)
            kernel = get_bayer_neighbor_kernel(color)
            y_offsets, x_offsets = neighbor_offsets_from_kernel(kernel)
            y_neighbors0 = y_offsets + y
            x_neighbors0 = x_offsets + x

            ny, nx = self.image_shape
            in_bounds = (
                (0 <= y_neighbors0)
                & (y_neighbors0 < ny)
                & (0 <= x_neighbors0)
                & (x_neighbors0 < nx)
            )
            y_neighbors1 = y_neighbors0[in_bounds]
            x_neighbors1 = x_neighbors0[in_bounds]

            # pylint:disable-next=singleton-comparison # this is an array operation
            not_bad = bad_image[y_neighbors1, x_neighbors1] != True
            y_neighbors2 = y_neighbors1
            x_neighbors2 = x_neighbors1
            if not_bad.any():
                y_neighbors2 = y_neighbors1[not_bad]
                x_neighbors2 = x_neighbors1[not_bad]
                order = 0
            else:
                # If all neighbors are bad, fill them first, then use
                # them to fill this pixel.
                order = 1

            if y_neighbors2.size == 0:
                print(
                    f"neighbors: {not_bad} {y_neighbors0} {x_neighbors0} {y_neighbors1} {x_neighbors1}"
                )
                raise RuntimeError()

            self.bad_pixels.append(
                BadPixel(
                    y=y,
                    x=x,
                    y_neighbors=y_neighbors2,
                    x_neighbors=x_neighbors2,
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
    im_path: str,
    stats: ImageStats,
    corrector: BadPixelCorrector,
    bad_coords: CoordArray,
) -> None:
    """
    Plot how an image looks before and after correction, zooming in on the center.
    This is copy/paste from a Jupyter notebook, needs to be refactored a bit.
    """
    im_bayer = cv2.imread(im_path, cv2.IMREAD_GRAYSCALE)
    corr_bayer = corrector(im_bayer)
    im = cv2.cvtColor(im_bayer, cv2.COLOR_BayerGB2RGB)
    corr = cv2.cvtColor(corr_bayer, cv2.COLOR_BayerGB2RGB)
    levels = get_levels(stats)
    scale = 1.4
    corr_fixed = fix_levels(corr, levels, scale=scale)
    fig, ax = plt.subplots()
    plot_image_stats1(ax, corr_fixed, "raw", colorbar=False, crop_center=0.2)
    # plt.imshow(corr_fixed)
    fig.set_size_inches((100, 75))

    # plot with and without fix
    fig, axes = plt.subplots(2, 1)
    plot_image_stats1(
        axes[0], fix_levels(im, levels, scale), "raw", colorbar=False, crop_center=0.2
    )
    plot_image_stats1(
        axes[1],
        fix_levels(corr, levels, scale),
        "corr",
        colorbar=False,
        crop_center=0.2,
    )
    fig.set_size_inches((100, 75))

    im_c = label_with_circles(fix_levels(im, levels, scale), bad_coords)
    corr_c = label_with_circles(fix_levels(corr, levels, scale), bad_coords)

    # plot with and without fix -- labeled with circles
    fig, axes = plt.subplots(2, 1)
    plot_image_stats1(axes[0], im_c, "raw labeled", colorbar=False, crop_center=0.15)
    plot_image_stats1(axes[1], corr_c, "corr labeled", colorbar=False, crop_center=0.15)
    fig.set_size_inches((100, 150))


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

    stats = get_image_stats(ImageSourcePaths(paths, preprocess=None))
    plot_image_stats(stats)

    bad_coords = get_bad_pixel_coords(stats)
    corrector = BadPixelCorrector(stats.mean.shape, bad_coords)

    stats2 = get_image_stats(ImageSourcePaths(paths, preprocess=corrector))
    plot_image_stats(stats2)

    bad_coords2 = coords_union(bad_coords, get_bad_pixel_coords(stats2))
    corrector2 = BadPixelCorrector(stats.mean.shape, bad_coords2)
    stats3 = get_image_stats(ImageSourcePaths(paths, preprocess=corrector2))
    plot_image_stats(stats3)

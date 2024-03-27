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
Test pixel_utils.py. This is currently copy/paste from a Jupyter notebook.
Some of it can probably be turned into real unit tests. Other tests require
manual review of output.
"""

import numpy as np
from matplotlib import pyplot as plt

import pixel_utils as pu


def test_get_pixel_color(y: int, x: int, expected: pu.RgbChannel):
    "Test get_pixel_color()."
    c = pu.get_pixel_color(pu.NAVCAM_BAYER_CONVENTION, y, x)
    assert c == expected
    print(f"test_get_pixel_color: y={y} x={x} expected={expected} result={c}")


def test_get_bayer_mask():
    "Test get_bayer_mask()."
    test_image = np.zeros((6, 8, 3), dtype=np.uint8)
    for channel_num, channel in enumerate("RGB"):
        test_image[:, :, channel_num] = 255 * pu.get_bayer_mask(
            test_image.shape, channel, pu.NAVCAM_BAYER_CONVENTION
        )
    plt.imshow(test_image)
    # Visually inspect that the color pattern matches our convention according to
    # https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html


def test_neighbor_offsets_from_kernel():
    "Test neighbor_offsets_from_kernel()."
    print(pu.neighbor_offsets_from_kernel(pu.RB_KERNEL))
    print(pu.neighbor_offsets_from_kernel(pu.G_KERNEL))


def test_median_filter():
    "Test median_filter()."
    rng = np.random.default_rng()

    # start with grey image
    h, w = 6, 8
    im = (128 * np.ones((h, w))).astype(np.int16)

    # add some random noise
    noise_max = 10
    noise = rng.integers(low=-noise_max, high=noise_max, size=im.shape, dtype=np.int16)
    im += noise

    # add some 'hot pixels'. neighboring bad pixels should be ok, assuming enough other pixels
    hot_y = np.array([0, 0, 1, 5, 5])
    hot_x = np.array([0, 3, 4, 1, 7])
    im[hot_y, hot_x] = 255

    # set kernel to be 1x1 box kernel with missing center pixel
    kernel = np.ones((3, 3), dtype=np.uint8)
    kernel[1, 1] = 0

    # apply median filter
    corr = pu.median_filter(im, kernel)

    # all pixels should be in the original noise range
    assert np.all((128 - noise_max <= corr) & (corr < 128 + noise_max))

    fig, axes = plt.subplots(1, 2)
    axes[0].imshow(im, vmin=0, vmax=255)
    axes[1].imshow(corr, vmin=0, vmax=255)
    fig.set_size_inches((20, 8))


def get_bayer_test_image() -> pu.BayerImage:
    """
    Return a test image in raw Bayer format where all pixels with the same filter color have the
    same value, but different colors have different values. Handy for certain tests.
    """
    result = np.zeros((6, 8), dtype=np.uint8)
    rgb_values = [150, 100, 50]  # arbitrary constant color value for each channel
    for channel_num, channel in enumerate("RGB"):
        mask = pu.get_bayer_mask(result.shape, channel, pu.NAVCAM_BAYER_CONVENTION)
        result[mask] = rgb_values[channel_num]
    return result


def test_estimate_from_bayer_neighbors():
    "Test estimate_from_bayer_neighbors()."
    test2_in = get_bayer_test_image()
    test2_out = pu.estimate_from_bayer_neighbors(test2_in, pu.NAVCAM_BAYER_CONVENTION)

    assert np.all(test2_in == test2_out)

    fig, axes = plt.subplots(1, 2)

    im = axes[0].imshow(test2_in)
    plt.colorbar(im, ax=axes[0])
    axes[0].set_title("input")

    im = axes[1].imshow(test2_out)
    plt.colorbar(im, ax=axes[1])
    axes[1].set_title("estimated output (should be the same)")

    fig.set_size_inches((20, 8))


def test_get_bad_pixel_coords(stats: pu.ImageStats) -> None:
    "Test get_bad_pixel_coords()."
    bad_coords = pu.get_bad_pixel_coords(stats)
    disp = np.zeros(stats.rms_err.shape)
    disp[bad_coords] = stats.rms_err[bad_coords]

    fig, _ = plt.subplots()
    plt.jet()
    plt.imshow(disp)
    plt.colorbar(fraction=0.035, pad=0.04)
    fig.set_size_inches((80, 60))
    print(np.count_nonzero(disp))


def test_bad_pixel_corrector():
    "Test BadPixelCorrector."
    # Choose R, G, B pixels. The last pair are (blue) same-color neighbors.
    bad_coords = (np.array([0, 1, 3, 5]), np.array([1, 3, 2, 2]))

    test2_in = get_bayer_test_image()

    corrupted = test2_in.copy()
    corrupted[bad_coords] = 0

    corrector = pu.BadPixelCorrector(corrupted.shape, bad_coords)
    corrected = corrector(corrupted)

    fig, axes = plt.subplots(1, 3)

    obj = axes[0].imshow(test2_in)
    plt.colorbar(obj, ax=axes[0], fraction=0.035, pad=0.04)
    axes[0].set_title("Input")

    obj = axes[1].imshow(corrupted)
    plt.colorbar(obj, ax=axes[1], fraction=0.035, pad=0.04)
    axes[1].set_title("Corrupted")

    obj = axes[2].imshow(corrected)
    plt.colorbar(obj, ax=axes[2], fraction=0.035, pad=0.04)
    axes[2].set_title("Corrected")

    fig.set_size_inches((30, 10))

    assert (test2_in == corrected).all()


def main():
    "Main testing function."
    test_get_pixel_color(y=1, x=0, expected="B")
    test_get_pixel_color(y=0, x=3, expected="R")
    test_get_bayer_mask()
    test_neighbor_offsets_from_kernel()
    test_median_filter()
    test_estimate_from_bayer_neighbors()
    # test_get_bad_pixel_coords(stats)  # figure out what stats to test this one
    test_bad_pixel_corrector()

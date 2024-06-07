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
Test pixel_utils.py.
"""

import os
import pathlib
import tempfile
import unittest
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from matplotlib import pyplot as plt

from bag_processing import pixel_utils as pu

# Coordinates of hot pixels in test data
HOT_Y = np.array([0, 0, 1, 5, 5])
HOT_X = np.array([0, 3, 4, 1, 7])


def get_rgb_bayer_pattern() -> pu.RgbImage:
    "Return a small RGB image visualizing the NavCam Bayer pattern."
    test_image = np.zeros((6, 8, 3), dtype=np.uint8)
    for channel_num, channel in enumerate("RGB"):
        test_image[:, :, channel_num] = 255 * pu.get_bayer_mask(
            test_image.shape, channel, pu.NAVCAM_BAYER_CONVENTION
        )
    return test_image


def get_channel_rgb(channel: pu.RgbChannel) -> np.ndarray:
    "Return RGB values for `channel`."
    if channel == "R":
        return np.array([255, 0, 0])
    if channel == "G":
        return np.array([0, 255, 0])
    if channel == "B":
        return np.array([0, 0, 255])
    raise RuntimeError("Never reach this point")


def get_hot_pixel_test_images(
    gradient: bool = False,
) -> Tuple[pu.MonoImage, pu.MonoImage]:
    """
    Return a pair of images (truth, observed) where truth is the "actual" image and observed has
    been degraded with hot pixels. Useful for testing correction algorithms that try to
    reconstruct truth from observed.
    """
    h, w = 6, 8
    if gradient:
        # Start with gradient image
        x_grid, _ = np.meshgrid(np.arange(w), np.arange(h))
        truth = (50 + 10 * x_grid).astype(np.int16)
    else:
        # Start with constant gray image
        truth = np.ones((h, w)) * 128

    # Add some 'hot pixels' that we will correct. For a median filter, fairly good correction should
    # be possible as long as the majority of 8-connected neighbors of each pixel are valid.
    observed = truth.copy()
    observed[HOT_Y, HOT_X] = 255

    return truth, observed


def get_median_filtered(observed: pu.MonoImage):
    "Return the result of correcting `observed` with a median filter and an 8-connected kernel."
    # set kernel to be 1x1 box kernel with missing center pixel
    kernel = np.ones((3, 3), dtype=np.uint8)
    kernel[1, 1] = 0

    # apply median filter
    return pu.median_filter(observed, kernel)


def get_bayer_test_image() -> pu.BayerImage:
    """
    Return a test image in raw Bayer format where all pixels with the same filter color have the
    same value, but different colors have different values. Handy for testing that processing
    steps are respecting the Bayer color structure.
    """
    result = np.zeros((6, 8), dtype=np.uint8)
    rgb_values = [255, 128, 0]  # arbitrary constant color value for each channel
    for channel_num, channel in enumerate("RGB"):
        mask = pu.get_bayer_mask(result.shape, channel, pu.NAVCAM_BAYER_CONVENTION)
        result[mask] = rgb_values[channel_num]
    return result


def get_test_images_for_stats(gradient: bool = False) -> List[pu.BayerImage]:
    """
    Return a minimal set of test images for get_image_stats().
    """
    _, observed = get_hot_pixel_test_images(gradient)
    hot = observed == 255

    in1 = observed - 1
    in1[hot] = 255
    in2 = observed + 1
    in2[hot] = 255
    return [in1, in2]


@dataclass
class AffineTestData:
    """
    Test data for slope/intercept calculation. Each image in data.images should satisfy:
      - est[data.y, data.x] = data.m * image[data.y, data.x] + data.b
    Assuming:
      - est = estimate_from_bayer_neighbors(image)
      - As with real images, pixel values won't follow the model past saturation (255).
    """

    images: List[pu.BayerImage]
    y: np.ndarray
    x: np.ndarray
    m: np.ndarray
    b: np.ndarray
    vals: np.ndarray

    @staticmethod
    def get() -> "AffineTestData":
        "Return test data."
        shape = (6, 8)
        y = np.array([0, 4, 2, 1, 5])
        x = np.array([0, 2, 5, 7, 7])
        m = np.array([1.1, 0.9, 1.2, 0.8, 1.25], dtype=np.float64)
        b = np.array([0, -5, -8, -10, -100], dtype=np.float64)
        images = []
        vals = np.arange(256, dtype=np.uint8)
        for val in vals:
            img = np.full(shape, val, dtype=np.uint8)
            img[y, x] = np.clip((val - b) / m, 0, 255).astype(np.uint8)
            images.append(img)
        return AffineTestData(images=images, y=y, x=x, m=m, b=b, vals=vals)


class TestPixelUtils(unittest.TestCase):  # pylint: disable=too-many-public-methods
    "TestCase for testing pixel_utils."

    def get_pixel_color_case(self, y: int, x: int, expected: pu.RgbChannel) -> None:
        "One case for test_get_pixel_color()."
        c = pu.get_pixel_color(pu.NAVCAM_BAYER_CONVENTION, y, x)
        self.assertEqual(c, expected)

    def test_get_pixel_color(self) -> None:
        "Test get_pixel_color()."
        self.get_pixel_color_case(y=1, x=0, expected="B")
        self.get_pixel_color_case(y=0, x=3, expected="R")

    def test_get_bayer_mask(self) -> None:
        "Test get_bayer_mask()."
        test_image = get_rgb_bayer_pattern()

        h, w, _ = test_image.shape
        for patch_y in range(0, h, 2):
            for patch_x in range(0, w, 2):
                patch = test_image[patch_y : (patch_y + 2), patch_x : (patch_x + 2)]
                np.testing.assert_equal(
                    patch[0, 0, :], get_channel_rgb(pu.NAVCAM_BAYER_CONVENTION[0])
                )
                np.testing.assert_equal(
                    patch[0, 1, :], get_channel_rgb(pu.NAVCAM_BAYER_CONVENTION[1])
                )
                np.testing.assert_equal(
                    patch[1, 0, :], get_channel_rgb(pu.NAVCAM_BAYER_CONVENTION[2])
                )
                np.testing.assert_equal(
                    patch[1, 1, :], get_channel_rgb(pu.NAVCAM_BAYER_CONVENTION[3])
                )

    def demo_get_bayer_mask(self) -> None:
        "Demo get_bayer_mask()."
        _, ax = plt.subplots()
        ax.imshow(get_rgb_bayer_pattern())
        out_path = "output/demo_get_bayer_mask.png"
        plt.savefig(out_path)
        print()
        print("== demo_get_bayer_mask")
        print(f"Wrote to: {out_path}")
        print(
            "You can visually verify that the color pattern matches our convention according to:"
        )
        print("  https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html")
        print(
            "  https://github.com/nasa/astrobee/blob/develop/hardware/is_camera/src/camera.cc#L522 [GRBG convention]"
        )

    def test_neighbor_offsets_from_kernel(self) -> None:
        "Test neighbor_offsets_from_kernel()."
        x_rb, y_rb = pu.neighbor_offsets_from_kernel(pu.RB_KERNEL)
        np.testing.assert_equal(x_rb, np.array([-2, -2, -2, 0, 0, 2, 2, 2]))
        np.testing.assert_equal(y_rb, np.array([-2, 0, 2, -2, 2, -2, 0, 2]))
        x_g, y_g = pu.neighbor_offsets_from_kernel(pu.G_KERNEL)
        np.testing.assert_equal(x_g, np.array([-2, -1, -1, 0, 0, 1, 1, 2]))
        np.testing.assert_equal(y_g, np.array([0, -1, 1, -2, 2, -1, 1, 0]))

    def test_median_filter(self) -> None:
        "Test median_filter()."
        truth, observed = get_hot_pixel_test_images(gradient=True)
        corrected = get_median_filtered(observed)
        # Corrected pixels should match ground truth, up to max error bounded by the slope of the
        # image gradient in truth.
        np.testing.assert_allclose(corrected, truth, atol=10, rtol=9999)

        truth, observed = get_hot_pixel_test_images(gradient=False)
        corrected = get_median_filtered(observed)
        # With no gradient, the match should be exact up to roundoff
        np.testing.assert_allclose(corrected, truth)

    def demo_median_filter(self) -> None:
        "Demo median_filter()."
        truth, observed = get_hot_pixel_test_images(gradient=True)
        filtered = get_median_filtered(observed)

        images_to_plot = {
            "truth": truth,
            "observed": observed,
            "filtered": filtered,
        }
        fig, axes = plt.subplots(len(images_to_plot), 1)
        plt.jet()
        for i, (title, im) in enumerate(images_to_plot.items()):
            axes[i].imshow(im, vmin=0, vmax=255)
            axes[i].set_title(title)
        fig.set_size_inches((8, 20))
        out_path = "output/demo_median_filter.png"
        plt.savefig(out_path)
        print()
        print("== demo_median_filter")
        print(f"Wrote to: {out_path}")
        print("You should see that 'filtered' closely matches 'truth'.")

    def test_estimate_from_bayer_neighbors(self) -> None:
        "Test estimate_from_bayer_neighbors()."
        observed = get_bayer_test_image()
        estimated = pu.estimate_from_bayer_neighbors(
            observed, pu.NAVCAM_BAYER_CONVENTION
        )
        # In this case, since all pixels of the same color have the same value in the input image,
        # running the filter shouldn't change anything. However, if the filter doesn't respect the
        # Bayer same-color structure, it will change the values.
        np.testing.assert_allclose(observed, estimated)

    def demo_estimate_from_bayer_neighbors(self) -> None:
        "Test estimate_from_bayer_neighbors()."
        observed = get_bayer_test_image()
        estimated = pu.estimate_from_bayer_neighbors(
            observed, pu.NAVCAM_BAYER_CONVENTION
        )

        images_to_plot = {
            "observed": observed,
            "estimated": estimated,
        }
        fig, axes = plt.subplots(len(images_to_plot), 1)
        plt.jet()
        for i, (title, im) in enumerate(images_to_plot.items()):
            axes[i].imshow(im, vmin=0, vmax=255)
            axes[i].set_title(title)
        fig.set_size_inches((8, 20))
        out_path = "output/demo_estimate_from_bayer_neighbors.png"
        plt.savefig(out_path)
        print()
        print("== demo_estimate_from_bayer_neighbors")
        print(f"Wrote to: {out_path}")
        print("You should see that 'estimated' equals 'observed'.")

    def test_get_image_stats(self) -> None:
        "Test get_image_stats()."
        _, observed = get_hot_pixel_test_images()
        test_images = get_test_images_for_stats()
        stats = pu.DebugStatsAccumulator.get_image_stats(test_images)
        assert stats is not None
        hot = observed == 255
        expected_hot_mean_err = 255 - 128
        expected_hot_rms_err = np.sqrt(np.mean(np.square([255 - 127, 255 - 129])))
        np.testing.assert_allclose(stats.mean, observed)
        np.testing.assert_allclose(stats.stdev[hot], 0)
        np.testing.assert_allclose(stats.stdev[~hot], np.std([-1, 1]))
        np.testing.assert_allclose(stats.mean_err[hot], expected_hot_mean_err)
        np.testing.assert_allclose(stats.mean_err[~hot], 0)
        np.testing.assert_allclose(stats.rms_err[hot], expected_hot_rms_err)
        np.testing.assert_allclose(stats.rms_err[~hot], 0)
        np.testing.assert_allclose(stats.stdev_err[hot], np.std([-1, 1]))
        np.testing.assert_allclose(stats.stdev_err[~hot], 0)
        # stats.slope and stats.intercept tested in test_slope_intercept()

    def test_get_bad_pixel_coords(self) -> None:
        "Test get_bad_pixel_coords()."
        test_images = get_test_images_for_stats()
        stats = pu.DebugStatsAccumulator.get_image_stats(test_images)
        assert stats is not None
        bad_y, bad_x = pu.get_bad_pixel_coords(stats)
        np.testing.assert_equal(bad_y, HOT_Y)
        np.testing.assert_equal(bad_x, HOT_X)

    def test_neighbor_mean_corrector(self) -> None:
        "Test NeighborMeanCorrector."
        truth, _ = get_hot_pixel_test_images()
        test_images = get_test_images_for_stats()
        # test_images are [truth - 1, truth + 1] (both corrupted with hot pixels)
        m1, p1 = test_images
        stats = pu.DebugStatsAccumulator.get_image_stats(test_images)
        assert stats is not None
        bad_coords = pu.get_bad_pixel_coords(stats)
        corrector = pu.NeighborMeanCorrector(stats.mean.shape, bad_coords)
        np.testing.assert_allclose(corrector(m1), truth - 1)
        np.testing.assert_allclose(corrector(p1), truth + 1)

    def plot_neighbor_mean_corrector(
        self, images_to_plot: Dict[str, np.ndarray]
    ) -> None:
        "Plot for demo_neighbor_mean_corrector()."
        fig, axes = plt.subplots(3, 1)
        plt.jet()
        for i, (title, im) in enumerate(images_to_plot.items()):
            obj = axes[i].imshow(im, vmin=0, vmax=255)
            plt.colorbar(obj, ax=axes[i], fraction=0.035, pad=0.04)
            axes[i].set_title(title)
        fig.set_size_inches((10, 30))
        out_path = "output/demo_neighbor_mean_corrector.png"
        plt.savefig(out_path)
        print()
        print("== demo_neighbor_mean_corrector")
        print(f"Wrote to: {out_path}")
        print("You should see that 'corrected' closely matches 'truth'.")

    def demo_neighbor_mean_corrector(self) -> None:
        "Demo NeighborMeanCorrector."
        truth, observed = get_hot_pixel_test_images(gradient=True)
        test_images = get_test_images_for_stats(gradient=True)
        stats = pu.DebugStatsAccumulator.get_image_stats(test_images)
        assert stats is not None
        bad_coords = pu.get_bad_pixel_coords(stats)
        corrector = pu.NeighborMeanCorrector(stats.mean.shape, bad_coords)
        corrected = corrector(observed)

        images_to_plot = {
            "truth": truth,
            "observed": observed,
            "corrected": corrected,
        }
        self.plot_neighbor_mean_corrector(images_to_plot)

    def demo_all(self) -> None:
        "Call all demo_x() methods."
        pathlib.Path("output").mkdir(exist_ok=True)
        self.demo_get_bayer_mask()
        self.demo_median_filter()
        self.demo_estimate_from_bayer_neighbors()
        self.demo_neighbor_mean_corrector()

    def check_nbp_equal(self, a: pu.NeighborBadPixel, b: pu.NeighborBadPixel) -> None:
        "Test that `a` and `b` are equal."
        self.assertEqual(a.y, b.y)
        self.assertEqual(a.x, b.x)
        np.testing.assert_equal(a.y_neighbors, b.y_neighbors)
        np.testing.assert_equal(a.x_neighbors, b.x_neighbors)
        self.assertEqual(a.order, b.order)

    def check_nbp_list_equal(
        self, a: List[pu.NeighborBadPixel], b: List[pu.NeighborBadPixel]
    ) -> None:
        "Test that `a` and `b` are equal."
        for a_val, b_val in zip(a, b):
            self.check_nbp_equal(a_val, b_val)

    def test_neighbor_mean_corrector_save_load(self) -> None:
        "Test NeighborMeanCorrector save() and load()."
        test_images = get_test_images_for_stats(gradient=True)
        stats = pu.DebugStatsAccumulator.get_image_stats(test_images)
        assert stats is not None
        bad_coords = pu.get_bad_pixel_coords(stats)
        corrector = pu.NeighborMeanCorrector(stats.mean.shape, bad_coords)
        temp_fd, temp_name = tempfile.mkstemp(
            "test_neighbor_mean_corrector_save_load.json"
        )
        os.close(temp_fd)
        temp_path = pathlib.Path(temp_name)
        corrector.save(temp_path)
        corrector_copy = pu.NeighborMeanCorrector.load(temp_path)
        self.assertEqual(corrector.image_shape, corrector_copy.image_shape)
        self.check_nbp_list_equal(corrector.bad_pixels, corrector_copy.bad_pixels)

    def test_get_image_stats_parallel(self) -> None:
        "Test get_image_stats_parallel()."
        test_images = get_test_images_for_stats(gradient=True)
        stats = pu.DebugStatsAccumulator.get_image_stats(test_images)
        assert stats is not None
        stats2 = pu.DebugStatsAccumulator.get_image_stats_parallel(
            test_images, num_workers=2
        )
        assert stats2 is not None
        np.testing.assert_allclose(stats.mean, stats2.mean)

    def test_bias_corrector_save_load(self) -> None:
        "Test BiasCorrector save() and load()."
        test_images = get_test_images_for_stats(gradient=True)
        stats = pu.BiasStatsAccumulator.get_image_stats(test_images)
        assert stats is not None
        corrector = pu.BiasCorrector.from_image_stats(stats)
        temp_fd, temp_name = tempfile.mkstemp("test_bias_corrector_save_load.json")
        os.close(temp_fd)
        temp_path = pathlib.Path(temp_name)
        corrector.save(temp_path)
        corrector_copy = pu.BiasCorrector.load(temp_path)
        corrector.assert_equal(corrector_copy)

    def test_slope_intercept(self) -> None:
        "Test DebugStatsAccumulator slope/intercept calculations."
        data = AffineTestData.get()
        stats = pu.DebugStatsAccumulator.get_image_stats(data.images)
        assert stats is not None
        np.testing.assert_allclose(
            stats.slope[data.y, data.x], data.m, rtol=999, atol=1e-3
        )
        np.testing.assert_allclose(
            stats.intercept[data.y, data.x], data.b, rtol=999, atol=1
        )


def main():
    "Main testing function."
    if os.environ.get("TEST_PIXEL_UTILS_NO_DEMO") is None:
        TestPixelUtils().demo_all()
        print()
        print("== Unit testing")
    unittest.main()


if __name__ == "__main__":
    main()

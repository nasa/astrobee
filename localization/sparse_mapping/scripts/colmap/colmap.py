# This module contains helper functions to read colmap databases and models
# as well as call colmap functions

import os
import re
import sys
import tempfile

import argparse
import collections
import sqlite3
import numpy as np
import struct
import subprocess

_MAX_IMAGE_ID = 2**31 - 1

# Access colmap sqlite database with cameras, images and matches
class COLMAPDatabase(sqlite3.Connection):
    @staticmethod
    def connect(database_path):
        return sqlite3.connect(database_path, factory=COLMAPDatabase)

    def __init__(self, *args, **kwargs):
        super(COLMAPDatabase, self).__init__(*args, **kwargs)

    def __image_ids_to_pair_id(image_id1, image_id2):
        if image_id1 > image_id2:
            image_id1, image_id2 = image_id2, image_id1
        return image_id1 * _MAX_IMAGE_ID + image_id2

    def cameras(self):
        cameras = []
        rows = self.execute("SELECT * FROM cameras")
        for r in rows:
            cameras.append(r)
        return cameras

    def images(self):
        images = []
        rows = self.execute("SELECT image_id, name FROM images")
        for r in rows:
            images.append(r)
        return images

    def image_id(self, image_name):
        images = []
        rows = self.execute(
            "SELECT image_id FROM images WHERE images.name = '%s'" % (image_name)
        )
        if rows == None:
            return None
        return rows.fetchall()[0][0]

    def num_matches(self, image1, image2):
        if image1 == image2:
            return 0
        rows = self.execute(
            "SELECT rows FROM matches WHERE pair_id = %d"
            % (self.__image_ids_to_pair_id(image1, image2))
        )
        return next(rows)[0]


_FILE_PATH = os.path.dirname(os.path.realpath(__file__))

# creates a colmap project ini file based on a template, filling in key arguments (use in a "with:" block)
class ColmapProjectConfig:
    def __init__(
        self,
        database_path,
        image_path,
        output_path,
        image_list=None,
        ini_file="mapper.ini",
        input_path=None,
    ):
        self.database_path = database_path
        self.image_path = image_path
        self.output_path = output_path
        self.image_list = image_list
        self.ini_file = ini_file
        self.input_path = input_path

    def file_name(self):
        return self.config.name

    def __enter__(self):
        self.config = tempfile.NamedTemporaryFile(delete=False)
        if self.image_list:
            self.image_config = tempfile.NamedTemporaryFile(delete=False)
            with open(self.image_config.name, "w") as f:
                for image in self.image_list:
                    f.write(image + "\n")

        with open(os.path.join(_FILE_PATH, self.ini_file), "r") as f:
            content = f.read()
            content = re.sub("DATABASE_PATH", self.database_path, content)
            content = re.sub("IMAGE_PATH", self.image_path, content)
            content = re.sub("OUTPUT_PATH", self.output_path, content)
            if self.input_path != None:
                content = re.sub("INPUT_PATH", self.input_path, content)
            image_list_set = (
                "image_list_path = %s" % (self.image_config.name)
                if self.image_list
                else ""
            )
            content = re.sub("IMAGE_LIST_SET", image_list_set, content)
            with open(self.config.name, "w") as cfg:
                cfg.write(content)

        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        os.remove(self.config.name)
        if self.image_list:
            os.remove(self.image_config.name)


CameraModel = collections.namedtuple(
    "CameraModel", ["model_id", "model_name", "num_params"]
)
Camera = collections.namedtuple("Camera", ["id", "model", "width", "height", "params"])
BaseImage = collections.namedtuple(
    "Image", ["id", "qvec", "tvec", "camera_id", "name", "xys", "point3D_ids"]
)
Point3D = collections.namedtuple(
    "Point3D", ["id", "xyz", "rgb", "error", "image_ids", "point2D_idxs"]
)


class Image(BaseImage):
    def qvec2rotmat(self):
        return qvec2rotmat(self.qvec)


def _read_next_bytes(fid, num_bytes, format_char_sequence, endian_character="<"):
    """Read and unpack the next bytes from a binary file.
    :param fid:
    :param num_bytes: Sum of combination of {2, 4, 8}, e.g. 2, 6, 16, 30, etc.
    :param format_char_sequence: List of {c, e, f, d, h, H, i, I, l, L, q, Q}.
    :param endian_character: Any of {@, =, <, >, !}
    :return: Tuple of read and unpacked values.
    """
    data = fid.read(num_bytes)
    return struct.unpack(endian_character + format_char_sequence, data)


def _skip_next_bytes(fid, num_bytes):
    fid.seek(num_bytes, 1)


# snippets taken from colmap scripts/python/read_write_model.py
# this class reads a colmap model model and exposes fields of interest (mainly just the images)
class Model:
    def __init__(self, fname):
        self.filename = fname
        self.__read_images_binary(os.path.join(fname, "images.bin"))
        self.__analyze_model()

    def __str__(self):
        return "%d images, %g mean track length, %g mean reprojection error" % (
            self.num_images,
            self.mean_track_length,
            self.mean_reprojection_error,
        )

    def __read_images_binary(self, path_to_model_file):
        """
        see: src/colmap/scene/reconstruction.cc
            void Reconstruction::ReadImagesBinary(const std::string& path)
            void Reconstruction::WriteImagesBinary(const std::string& path)
        """
        images = {}
        with open(path_to_model_file, "rb") as fid:
            num_reg_images = _read_next_bytes(fid, 8, "Q")[0]
            for _ in range(num_reg_images):
                binary_image_properties = _read_next_bytes(
                    fid, num_bytes=64, format_char_sequence="idddddddi"
                )
                image_id = binary_image_properties[0]
                qvec = np.array(binary_image_properties[1:5])
                tvec = np.array(binary_image_properties[5:8])
                camera_id = binary_image_properties[8]
                binary_image_name = b""
                current_char = _read_next_bytes(fid, 1, "c")[0]
                while current_char != b"\x00":  # look for the ASCII 0 entry
                    binary_image_name += current_char
                    current_char = _read_next_bytes(fid, 1, "c")[0]
                image_name = binary_image_name.decode("utf-8")
                num_points2D = _read_next_bytes(
                    fid, num_bytes=8, format_char_sequence="Q"
                )[0]
                _skip_next_bytes(
                    fid, num_bytes=24 * num_points2D
                )  # faster to skip, we don't care
                xys = None
                point3D_ids = None
                # x_y_id_s = read_next_bytes(
                #    fid,
                #    num_bytes=24 * num_points2D,
                #    format_char_sequence="ddq" * num_points2D,
                # )
                # xys = np.column_stack(
                #    [
                #        tuple(map(float, x_y_id_s[0::3])),
                #        tuple(map(float, x_y_id_s[1::3])),
                #    ]
                # )
                # point3D_ids = np.array(tuple(map(int, x_y_id_s[2::3])))
                images[image_id] = Image(
                    id=image_id,
                    qvec=qvec,
                    tvec=tvec,
                    camera_id=camera_id,
                    name=image_name,
                    xys=xys,
                    point3D_ids=point3D_ids,
                )
        self.images = images

    def __analyze_model(self):
        cmd = subprocess.Popen(
            "colmap model_analyzer --path %s" % (self.filename),
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        output = cmd.communicate()[1]
        if cmd.returncode != 0:
            raise Exception("Model %s could not be analyzed." % (self.filename))
        result = dict()
        m = re.search("Registered images: (\d+)", str(output))
        self.num_images = int(m.groups()[0])
        m = re.search("Mean track length: ([-+]?(?:\d*\.*\d+))", str(output))
        self.mean_track_length = float(m.groups()[0])
        m = re.search("Mean reprojection error: (.+)px", str(output))
        self.mean_reprojection_error = float(m.groups()[0])

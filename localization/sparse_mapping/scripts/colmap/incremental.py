import argparse
import heapq
import os
import re
import shutil
import sys
import tempfile

import numpy as np

import colmap

# pass a colmap model with features detected and match, with images nested in subdirectories.
# builds submodels for each image subdirectory, then attempts to intelligently merge them.


class IncrementalMapper:
    def __init__(self, database_path, image_path, output_path):
        self.database_path = database_path
        self.image_path = image_path
        self.output_path = output_path

        self.db = colmap.COLMAPDatabase.connect(database_path)
        self.images = self.list_images()

    # builds nested dictionary of image folder structure
    def list_images(self):
        image_list = self.db.images()
        images = dict()
        for img in image_list:
            parts = img[1].split(os.sep)
            d = images
            for p in parts[:-2]:
                if p not in d:
                    d[p] = dict()
                d = d[p]
            if parts[-2] not in d:
                d[parts[-2]] = list()
            d[parts[-2]].append(img)
        return images

    def images_to_paths(self, images):
        paths = []
        for img in images:
            if type(img) is dict:
                for (d, i) in images.items():
                    paths.extend(self.images_to_paths(i))
            else:
                paths.append(img[1])
        return paths

    def build_map(self, image_list, output_path):
        if os.path.exists(output_path):
            print("Model already exists in %s, skipping rebuild." % (output_path))
            return
        try:
            os.makedirs(output_path, exist_ok=True)
        except:
            print("Could not create directory in %s." % (output_path))
            sys.exit(1)
        with colmap.ColmapProjectConfig(
            self.database_path,
            self.image_path,
            output_path,
            image_list,
            input_path=output_path,
        ) as cfg:
            os.system(
                "colmap mapper --project_path %s > %s/colmap.out 2>&1"
                % (cfg.file_name(), output_path)
            )

    def create_leaf_models(self, images, subdir="leaf"):
        if type(images) is dict:
            for (d, i) in images.items():
                self.create_leaf_models(i, os.path.join(subdir, d))
        else:
            path = os.path.join(self.output_path, subdir)
            self.build_map(self.images_to_paths(images), path)
            for fname in os.listdir(path):
                model = colmap.Model(os.path.join(path, fname))
                print("  Model %s: %s" % (os.path.join(subdir, fname), model))

    def get_best_overlap(self, base_model, models):
        best = None
        best_matches = 0
        for m in models:
            total = 0
            for img1 in base_model.images.values():
                for img2 in m.images.values():
                    total += self.db.num_matches(img1.id, img2.id)
            if total > best_matches:
                best_matches = total
                best = m
        return (best, best_matches)

    # Merge two models by adding overlapping images to one with the most overlapping features, then
    # merging with colmap's model_merger, adding points and bundle adjusting. Worked but results did not seem great
    def merge_models(self, model1, model2):
        print("Merging: %s and %s" % (model1.filename, model2.filename))
        image_choices = []
        for img2 in model2.images.values():
            matches = 0
            for img1 in model1.images.values():
                matches += self.db.num_matches(img1.id, img2.id)
            image_choices.append((img2.name, matches))
        best_matches = heapq.nlargest(10, image_choices, lambda x: x[1])
        images = list(map(lambda x: x.name, model1.images.values()))
        images.extend(list(map(lambda x: x[0], best_matches)))
        print("Adding top %d images." % (len(best_matches)))
        with colmap.ColmapProjectConfig(
            self.database_path,
            self.image_path,
            model1.filename,
            images,
            ini_file="merge.ini",
            input_path=model1.filename,
        ) as cfg:
            os.system(
                "colmap mapper --project_path %s >> %s/colmap_add.out 2>&1"
                % (cfg.file_name(), model1.filename)
            )
        print("Results: %s" % (colmap.Model(model1.filename)))
        os.mkdir(model1.filename + "_merged")
        os.system(
            "colmap model_merger --input_path1 %s --input_path2 %s --output_path %s"
            % (model2.filename, model1.filename, model1.filename + "_merged")
        )
        print("Merged maps: %s" % (colmap.Model(model1.filename + "_merged")))
        os.mkdir(model1.filename + "_points")
        with colmap.ColmapProjectConfig(
            self.database_path,
            self.image_path,
            model1.filename + "_points",
            images,
            ini_file="merge.ini",
            input_path=model1.filename + "_merged",
        ) as cfg:
            os.system(
                "colmap mapper --project_path %s >> %s/colmap_points.out 2>&1"
                % (cfg.file_name(), model1.filename)
            )
        # os.system('colmap mapper --database_path %s --image_path %s --input_path %s --output_path %s >> %s/colmap_points.out 2>&1' % (self.database_path, self.image_path, model1.filename + '_merged', model1.filename + '_points', model1.filename))
        print("Points: %s" % (colmap.Model(model1.filename + "_points")))
        os.mkdir(model1.filename + "_ba")
        os.system(
            "colmap bundle_adjuster --input_path %s --output_path %s >> %s/colmap_ba.out 2>&1"
            % (model1.filename + "_points", model1.filename + "_ba", model1.filename)
        )
        print("BA: %s" % (colmap.Model(model1.filename + "_ba")))
        sys.exit(1)
        return colmap.Model(model1.filename)

    def merge_models_recursive(self, images, subdir=""):
        # get models at leaf nodes
        if type(images) is not dict:
            models = []
            path = os.path.join(self.output_path, "leaf", subdir)
            for fname in os.listdir(path):
                model = colmap.Model(os.path.join(path, fname))
                if model.num_images > 5 and model.mean_reprojection_error < 1.5:
                    models.append(model)
            return models

        # sort models in folder by number of images, start with biggest model
        models = []
        for (d, i) in images.items():
            models.extend(self.merge_models_recursive(i, os.path.join(subdir, d)))
        models.sort(key=lambda x: x.num_images, reverse=True)

        if len(models) <= 1:
            return models
        print("Merging models in %s..." % (subdir))
        model = models.pop(0)
        print(
            "Starting with %s: %s"
            % (model.filename[len(self.output_path) + 5 :], model)
        )
        merged_path = os.path.join(self.output_path, "merged", subdir)
        if os.path.exists(merged_path):
            # this was failing to delete the top level folder for me with errno 26. didn't understand why.
            shutil.rmtree(merged_path, ignore_errors=True)
            if os.path.exists(merged_path):
                os.rmdir(merged_path)
        shutil.copytree(model.filename, merged_path)
        model = colmap.Model(merged_path)

        while len(models) > 0:
            (to_merge, shared_features) = self.get_best_overlap(model, models)
            models.remove(to_merge)
            print(
                "Merging %s with %d shared features: %s"
                % (
                    to_merge.filename[len(self.output_path) + 5 :],
                    shared_features,
                    to_merge,
                )
            )
            model = self.merge_models(model, to_merge)
            print("Result: %s" % (model))
        return [model]

    def list_subdirs(self, images, subdir=""):
        if type(images) is dict:
            result = []
            for (d, i) in images.items():
                result.extend(self.list_subdirs(i, os.path.join(subdir, d)))
            return result
        else:
            return [(subdir, self.images_to_paths(images))]

    def get_best_overlap_images(self, cur_images, subdirs):
        best = None
        best_matches = 0
        for i in range(len(subdirs)):
            total = 0
            for img1 in subdirs[i][1]:
                img1_id = self.db.image_id(img1)
                for img2 in cur_images:
                    img2_id = self.db.image_id(img2)
                    total += self.db.num_matches(img1_id, img2_id)
            if total > best_matches:
                best_matches = total
                best = i
        return (i, best_matches)

    # add images to model a subdirectory at a time, save models separately. Idea is to see which images mapping fails and remove them.
    # seems to be much slower than mapping with all the images at once.
    def incremental_add(self):
        subdirs = self.list_subdirs(self.images)
        (best_idx, best_matches) = self.get_best_overlap_images([], subdirs)
        # start with directory with most images
        max_id = max(enumerate(subdirs), key=lambda x: len(x[1][1]))[0]
        (data_source, images) = subdirs.pop(max_id)
        map_num = 0
        map_path = os.path.join(self.output_path, str(map_num))
        print(
            "Building initial map %s from %s with %d images..."
            % (str(map_num), data_source, len(images))
        )
        self.build_map(images, map_path)
        best_count = 0
        best = None
        for fname in os.listdir(map_path):
            if fname.endswith(".out"):
                continue
            model = colmap.Model(os.path.join(map_path, fname))
            print("  %s: %s" % (fname, model))
            count = len(model.images.keys())
            if count > best_count:
                best_count = count
                best = os.path.join(map_path, fname)
        map_path = best

        while len(subdirs) > 0:
            map_num += 1
            (best_idx, best_matches) = self.get_best_overlap_images(images, subdirs)
            (data_source, new_images) = subdirs.pop(best_idx)
            images.extend(new_images)
            next_map_path = os.path.join(self.output_path, str(map_num))
            print(
                "Building map %s, adding %s with %d images and %d overlapping features."
                % (str(map_num), data_source, len(new_images), best_matches)
            )
            os.mkdir(next_map_path)
            with colmap.ColmapProjectConfig(
                self.database_path,
                self.image_path,
                next_map_path,
                images,
                ini_file="merge.ini",
                input_path=map_path,
            ) as cfg:
                os.system(
                    "colmap mapper --project_path %s > %s/colmap.out 2>&1"
                    % (cfg.file_name(), next_map_path)
                )
            print("  %s" % (colmap.Model(next_map_path)))

    def incremental_map(self):
        self.create_leaf_models(self.images)
        self.merge_models_recursive(self.images)


def main():
    parser = argparse.ArgumentParser(
        description="Incrementally build a map with colmap from a nested directory of images."
    )
    parser.add_argument("--database_path", required=True, help="Colmap database file.")
    parser.add_argument(
        "--output_path", required=True, help="Path to write output model."
    )
    parser.add_argument("--image_path", required=True, help="Colmap image directory.")
    args = parser.parse_args()
    if not os.path.exists(args.database_path):
        print("Database not found.")
        return 1
    if os.path.exists(args.output_path):
        print("Output model already exists.")
        return 1
    try:
        os.mkdir(args.output_path)
    except:
        print("Could not create output directory.")
        return 1

    mapper = IncrementalMapper(args.database_path, args.image_path, args.output_path)
    mapper.incremental_add()

    return 0


if __name__ == "__main__":
    sys.exit(main())

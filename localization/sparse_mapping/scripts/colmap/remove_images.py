# Deletes a folder of images from an existing colmap model
import os
import sys
import tempfile

import argparse

import colmap

def main():
    parser = argparse.ArgumentParser(description="Deletes a directory of images from a given colmap model.")
    parser.add_argument("--input_path", required=True, help="Model to delete from")
    parser.add_argument("--output_path", required=True, help="Model to output to")
    parser.add_argument("--image_path", required=True, help="Base path for images in model")
    parser.add_argument("--remove_dir", required=True, help="Directory within image_path with images to remove")
    args = parser.parse_args()
    if not os.path.exists(args.input_path):
        print("Input not found.")
        return 1
    if not os.path.exists(args.output_path):
        try:
            os.mkdir(args.output_path)
        except:
            print("Output directory does not exist.")
            return 1

    try:
        tmp = tempfile.NamedTemporaryFile(mode='w+', delete=False)
        for root, dirs, files in os.walk(args.remove_dir):
            for f in files:
                tmp.write(os.path.relpath(os.path.join(root, f), args.image_path) + '\n')
        tmp.close()
        os.system("colmap image_deleter --input_path %s --output_path %s --image_names_path %s" % (args.input_path, args.output_path, tmp.name))
    finally:
        tmp.close()
        os.unlink(tmp.name)

    return 0

if __name__ == "__main__":
    sys.exit(main())

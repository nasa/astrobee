#!/usr/bin/env python
# Copyright (c) 2017, United States Government, as represented by the
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

'''
Undistort the images, create a theia map, then convert it to a sparse
map as understood by astrobee software, and rebuild it with distorted
images and SURF features, while reoptimizing the cameras. The
Theia build_reconstruction tool must be in the PATH.
'''

import sys, os, argparse, subprocess, re, shutil, glob

def which(program):
    '''Find if a program is in the PATH'''
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None

def run_cmd(cmd):
    '''
    Run a command.
    '''
    print(" ".join(cmd) + "\n")
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    process.wait()    
    if (process.returncode != 0):
        print("Failed execution of: " + " ".join(cmd))
        sys.exit(1)

def sanity_checks(undistort_image_path, import_map_path, build_map_path, args):

    # Check if the environment was set
    for var in ['ASTROBEE_RESOURCE_DIR', 'ASTROBEE_CONFIG_DIR', 'ASTROBEE_WORLD', 'ASTROBEE_ROBOT']:
        if var not in os.environ:
            raise Exception("Must set " + var)

    if not os.path.exists(undistort_image_path):
        raise Exception("Executable does not exist: " + undistort_image_path)
    
    if not os.path.exists(import_map_path):
        raise Exception("Executable does not exist: " + import_map_path)
    
    if not os.path.exists(build_map_path):
        raise Exception("Executable does not exist: " + build_map_path)

    if args.image_list == "":
        raise Exception("The path to the output map was not specified.")

    if args.output_map == "":
        raise Exception("The path to the output map was not specified.")

    if args.work_dir == "":
        raise Exception("The path to the work directory was not specified.")

    if not os.path.exists(args.theia_flags):
        raise Exception("Cannot find the Theia flags file: " + args.theia_flags)

    if which('build_reconstruction') is None:
        raise Exception("Cannot find the 'build_reconstruction' program in PATH.")
    
    if which('export_to_nvm_file') is None:
        raise Exception("Cannot find the 'export_to_nvm_file' program in PATH.")
    
def process_args(args):
    '''
    Set up the parser and parse the args.
    '''
    
    # Extract some paths before the args are parsed
    src_path = os.path.dirname(args[0])
    exec_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(src_path))))
    undistort_image_path = os.path.join(exec_path, "build/devel/lib/camera/undistort_image")
    import_map_path = os.path.join(exec_path, "build/devel/lib/sparse_mapping/import_map")
    build_map_path = os.path.join(exec_path, "build/devel/lib/sparse_mapping/build_map")

    parser = argparse.ArgumentParser(description="Parameters for the geometry mapper.")
    parser.add_argument("--theia_flags", dest = "theia_flags", default = "",
                        help = "The flags to pass to Theia. If not specified, use " +
                        "localization/sparse_mapping/theia_flags.txt in the astrobee repo.")
    parser.add_argument("--image_list", dest = "image_list", default = "",
                        help = "The list of distorted (original) nav cam images to use, " +
                        "one per line.")
    parser.add_argument("--output_map", dest = "output_map", default = "",
                        help = "The resulting output map.")
    parser.add_argument('--skip_rebuilding', dest = 'skip_rebuilding', action = 'store_true',
                        help = 'Do not rebuild the map after importing it from Theia.')
    parser.add_argument("--work_dir", dest = "work_dir", default = "",
                        help = "A temporary work directory to be deleted by the user later.")

    args = parser.parse_args()

    if args.theia_flags == "":
        args.theia_flags = os.path.dirname(src_path) + "/theia_flags.txt"
    
    sanity_checks(undistort_image_path, import_map_path, build_map_path, args)
    
    return (src_path, undistort_image_path, import_map_path, build_map_path, args)

def mkdir_p(path):
    if path == "":
        return # this can happen when path is os.path.dirname("myfile.txt")
    try:
        os.makedirs(path)
    except OSError:
        if os.path.isdir(path):
            pass
        else:
            raise Exception("Could not make directory " + path + \
                            " as a file with this name exists.")

# Theia likes its images undistorted and in one directory
def gen_undist_image_list(work_dir, dist_image_list):
    undist_dir = work_dir + "/undist"
    mkdir_p(undist_dir)

    # Wipe any preexisting images to not confuse theia later
    count = 0
    for image in glob.glob(undist_dir + "/*.jpg"):
        if count == 0:
            print("Wiping old images in: " + undist_dir)
        count += 1
        os.remove(image)

    undist_image_list = work_dir + "/undistorted_list.txt"
    print("Writing: " + undist_image_list)
    
    count = 10000 # to have the created images show up nicely
    undist_images = []
    with open(dist_image_list, "r") as dfh:
        dist_image_files = dfh.readlines()
        with open(undist_image_list, "w") as ufh:
            for image in dist_image_files:
                base_image = str(count) + ".jpg"
                undist_images.append(base_image)
                undist_image = undist_dir + "/" + base_image
                ufh.write(undist_image + "\n")
                count += 1

    return undist_image_list, undist_dir, undist_images

def gen_theia_calib_file(work_dir, undist_images, undist_intrinsics_file):
    
    calib_file = work_dir + "/" + "theia_calibration.json";

    # Parse the intrinsics
    with open(undist_intrinsics_file, "r") as fh:
        for line in fh:
            if len(line) == 0:
                continue
            if line[0] == "#":
                continue

            intrinsics = line.split()
            if len(intrinsics) < 5:
                raise Exception("Expecting 5 intrinsics numbers in: " + undist_intrinsics_file)
            
            width        = intrinsics[0]
            height       = intrinsics[1]
            focal_length = intrinsics[2]
            opt_ctr_x    = intrinsics[3]
            opt_ctr_y    = intrinsics[4]
            
            break
                
    print("Writing: " + calib_file)
    with open(calib_file, "w") as fh:
        fh.write("{\n")
        fh.write("\"priors\" : [\n")

        num_images = len(undist_images)
        for it in range(num_images):
            image = undist_images[it]
            fh.write("{\"CameraIntrinsicsPrior\" : {\n");
            fh.write("\"image_name\" : \"" + image + "\",\n");
            fh.write("\"width\" : " + str(width) + ",\n");
            fh.write("\"height\" : " + str(height) + ",\n");
            fh.write("\"camera_intrinsics_type\" : \"PINHOLE\",\n");
            fh.write("\"focal_length\" : " + str(focal_length) + ",\n");
            fh.write("\"principal_point\" : [" + str(opt_ctr_x) + ", " + str(opt_ctr_y) + "]\n");
            
            if it < num_images - 1:
                fh.write("}},\n")
            else:
                fh.write("}}\n")
        
        fh.write("]\n")
        fh.write("}\n")
        
    return calib_file

        
if __name__ == "__main__":
    
    (src_path, undistort_image_path, import_map_path, build_map_path, args) \
               = process_args(sys.argv)

    mkdir_p(args.work_dir)
    undist_image_list, undist_dir, undist_images \
                       = gen_undist_image_list(args.work_dir, args.image_list)

    # Undistort the images and crop to a central region
    undist_intrinsics_file = args.work_dir + "/undistorted_intrinsics.txt"
    cmd = [undistort_image_path, "-image_list", args.image_list,
           "--undistorted_crop_win", "1100 776",
           "-output_list", undist_image_list,
           "-undistorted_intrinsics", undist_intrinsics_file]
    run_cmd(cmd)
    
    calib_file   = gen_theia_calib_file(args.work_dir, undist_images, undist_intrinsics_file)
    recon_file   = args.work_dir + "/run"
    matching_dir = args.work_dir + "/matches"
    
    # Wipe old data
    for old_recon in glob.glob(recon_file + "*"):
        print("Deleting old reconstruction: " + old_recon)
        os.remove(old_recon)

    count = 0
    for old_matches in glob.glob(matching_dir + "/*"):
        if count == 0:
            print("Wiping old matches in: " + matching_dir)
        count += 1
        os.remove(old_matches)
    
    cmd =  ["build_reconstruction", "--flagfile", args.theia_flags,
            "--images", undist_dir + "/*jpg",
            "--calibration_file", calib_file,
            "--output_reconstruction", recon_file, 
            "--matching_working_directory", matching_dir,
            "--intrinsics_to_optimize", "NONE"]
    run_cmd(cmd)
    
    nvm_file = recon_file + ".nvm"
    cmd = ["export_to_nvm_file",
           "-input_reconstruction_file", recon_file + "-0",
           '-output_nvm_file', nvm_file]
    run_cmd(cmd)

    cmd = [import_map_path,
           '-input_map', nvm_file,
           '-output_map', args.output_map,
           '-undistorted_images_list', undist_image_list,
           '-distorted_images_list', args.image_list]
    run_cmd(cmd)
    
    if not args.skip_rebuilding:
        cmd = [build_map_path,
               '-output_map', args.output_map,
               '-rebuild', '-rebuild_refloat_cameras',
               '-rebuild_detector', 'SURF',
               '--min_valid_angle 1.0' # to avoid features only seen in close-by images
               ]
    run_cmd(cmd)
    

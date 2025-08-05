#!/usr/bin/python3
import argparse
import subprocess
import os
import shutil
import rosbag
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion

parser = argparse.ArgumentParser(
    description="Filters through ground truth bag to pull pose postions"
)
parser.add_argument(
    "--directory", "-d", type=str, help="full path to the directory that holds all the differnt bags from the issac activity"
)

parser.add_argument(
    "--robot", "-r", type=str, help="put the first letter of the robot your working with(queen, bumble, honey)"
)

args = parser.parse_args()

directory = args.directory
robo = args.robot
file_exstension = "_groundtruth.bag"
#change to your path to colmap
colmap = "/srv/novus_1/cmason9/bin/colmap"
error_bags = []
#function to call colmap commands 
def run_colmap_command(command_args):
    """Executes a COLMAP command """
    full_command = [colmap] + command_args
    subprocess.run(full_command, check=True, text=True, capture_output=True)

for bags in os.listdir(directory):
    full_path = directory + "/" + bags

    #assumes that each directory is named after the bag with splice ex:bag_name_22
    for splice in os.listdir(full_path):
            counter = 0
            bag_directory = directory + "/" + bags + "/" + splice
            mapping_pose = []
            images_path = bag_directory+ "/bag_images_" + splice
            timestamps = []
            for images in os.listdir(images_path):
                chars_ignore ="jpg."
                edit_name = str.maketrans("","",chars_ignore)
                image_time = images.translate(edit_name)
                timestamps.append(image_time)

            bag_path = bag_directory + "/" + splice
            bag = bag_path + file_exstension
            input_bag = rosbag.Bag(bag, 'r')

            for time in timestamps:
                for topic, msg, t in input_bag.read_messages(topics="/sparse_mapping/pose"):
                    image_time = int(time)
                    bag_time = int(t.to_nsec()/100)
                    tolerance =5.0 #seconds

                    if abs(image_time - bag_time) <= tolerance:

                        point_x = msg.pose.position.x
                        point_y = msg.pose.position.y
                        point_z = msg.pose.position.z
                        ori_x = msg.pose.orientation.x
                        ori_y = msg.pose.orientation.y
                        ori_z = msg.pose.orientation.z
                        ori_w = msg.pose.orientation.w
                        image = time[:10] + "." + time[10:] + ".jpg"
                        info = (ori_w, ori_x, ori_y, ori_z, point_x, point_y, point_z, "1", image)
                        mapping_pose.append(info)
    #check if input directory exists, if not create new one
            input_directory = bag_directory + "/input"
            os.makedirs(input_directory, exist_ok=True)
            shutil.move(images_path, input_directory)
            images_path = input_directory + "/bag_images_" + splice

            #image.txt
            file_name =  input_directory + "/" + "image.txt"
            with open(file_name, "w", newline="") as file:
                for tup in mapping_pose:
                    counter += 1
                    counter_str = str(counter)
 
                    line = " ".join([str(data) for data in tup])
                    file.write(counter_str + " " + line + "\n\n")
 
            #cameras.txt file
            if robo.casefold() == "b".casefold():
                #bumble
                camera_data = [
                    "1 SIMPLE_PINHOLE  608.8073 0.0 632.53684 0.998693",
                    "2 PINHOLE 0.0 607.61439 549.08386",
                    "3 SIMPLE_RADIAL 0.0 0.0 1.0"
                    ]
            elif robo.casefold() == "q".casefold():
                #queen
                camera_data = [
                    "1 SIMPLE_PINHOLE  604.19903 0.0 588.79562 1.00201",
                    "2 PINHOLE 0.0 602.67924 509.73835",
                    "3 SIMPLE_RADIAL 0.0 0.0 1.0"
                    ]
            elif robo.casefold() == "h".casefold()::
                #honey
                camera_data = [
                    "1 SIMPLE_PINHOLE  610.12594 0.0 628.35064 0.998575",
                    "2 PINHOLE 0.0 610.12594 514.89592",
                    "3 SIMPLE_RADIAL 0.0 0.0 1.0"
                    ]
 
            processed_list =[line.split() for line in camera_data]
 
 
            camera_file = input_directory + "/"  + "cameras.txt"
            with open(camera_file, "w", newline="") as file:
                for row in processed_list:
                    output =" ".join(row)
                    file.write(output +'\n')
        
            #3D point file
            point_file = input_directory + "/" + "points3D.txt"
        
            with open(point_file, 'w',newline="") as file:
                pass

            input_bag.close()

            database_path = input_directory + "/database.dq"
            # runs the commands needed to generate a dense model from a sparse model
            try:
                run_colmap_command([
                "feature_extractor",
                "--database_path", database_path,
                "--image_path", images_path
                ])
                
                run_colmap_command([
                "exhaustive_matcher",
                "--database_path", database_path
                ])

                sparse_path = input_directory + "/sparse"
                os.makedirs(sparse_path, exist_ok=True)

                run_colmap_command([
                "mapper",
                "--database_path", database_path,
                "--image_path", images_path,
                "--output_path", sparse_path
                ])
                
                tri_path = input_directory + "/triangulator"
                os.makedirs(tri_path, exist_ok=True)

                sparse_path = sparse_path + "/0"

                run_colmap_command([
                "point_triangulator",
                "--database_path", database_path,
                "--image_path", images_path,
                "--input_path", sparse_path,
                "--output_path", tri_path
                ])

                dense_path = input_directory + "/dense"
                os.makedirs(dense_path, exist_ok=True)

                run_colmap_command([
                "image_undistorter",
                "--image_path", images_path,
                "--input_path", tri_path,
                "--output_path", dense_path,
                ])

                run_colmap_command([
                "patch_match_stereo",
                "--workspace_path", dense_path,
                ])

                run_colmap_command([
                "stereo_fusion",
                "--workspace_path", dense_path,
                "--output_path", os.path.join(dense_path, "fused.ply")
                ])
            # if bag causes errors adds it to list and continues to the next value 
            except:
                error_bag = bags + ": " + splice
                error_bags.append(error_bag)
                continue

print("These bags had errors:")
print(error_bags)

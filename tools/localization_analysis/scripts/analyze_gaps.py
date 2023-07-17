import rosbag
import rospy
import argparse
import os
import glob
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math
import rospy
import tf
from geometry_msgs.msg import Quaternion

matplotlib.use("pdf")
from matplotlib.backends.backend_pdf import PdfPages

def quat_to_rpy(quaternion):
    # Convert Quaternion to RPY
    quaternion_tuple = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    rpy = tf.transformations.euler_from_quaternion(quaternion_tuple)
    return rpy

def find_bag(gt_bag, bag_dir):
    for root, dirs, files in os.walk(bag_dir):
        for file_name in files:
            search_for = gt_bag
            if file_name == search_for:
                file_path = os.path.join(root, file_name)
                return file_path
    return ""


def plot_histogram(data, name, ax, log_scale = False, bucket_size = 5, ylim = (0, 30), xlim = (5, 280), color = 'blue'):
    ax.hist(data, bins=np.arange(min(data), max(data) + bucket_size, bucket_size),
                edgecolor='black', alpha = 0.5, label = name, color = color)  # 'auto' selects the number of bins automatically
    ax.set_xlabel('Gap sizes (s)')
    ax.set_ylabel('Frequency')
    ax.legend()
    ax.set_xlim(xlim)
    if log_scale:
        ax.set_title('Gap size distribution (Log Scale)')
        ax.set_yscale('log')    
    else:
        ax.set_title('Gap size distribution')
        ax.set_ylim(ylim)

def calculate_rmse(bag1_file, bag2_file, topic):
    # Open the bags
    bag1 = rosbag.Bag(bag1_file)
    bag2 = rosbag.Bag(bag2_file)

    # Get the timestamps from both bags
    bag1_timestamps = {ts.to_nsec(): msg for _, msg, ts in bag1.read_messages(topics=[topic])}
    bag2_timestamps = {ts.to_nsec(): msg for _, msg, ts in bag2.read_messages(topics=[topic])}

    # Find the common timestamps in both bags
    common_timestamps = set(bag1_timestamps.keys()).intersection(bag2_timestamps.keys())
    if (len(common_timestamps)) < 20:
        return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=float)

    # Extract x, y, z positions for messages with the same timestamp
    positions1 = []
    positions2 = []

    # Extract r, p, y positions for messages with the same timestamp
    orientations1 = []
    orientations2 = []

    for timestamp in common_timestamps:
        positions1.append((bag1_timestamps[timestamp].pose.position.x,
                           bag1_timestamps[timestamp].pose.position.y,
                           bag1_timestamps[timestamp].pose.position.z))

        positions2.append((bag2_timestamps[timestamp].pose.position.x,
                           bag2_timestamps[timestamp].pose.position.y,
                           bag2_timestamps[timestamp].pose.position.z))

        rpy1 = quat_to_rpy(bag1_timestamps[timestamp].pose.orientation)
        rpy2 = quat_to_rpy(bag2_timestamps[timestamp].pose.orientation)
        orientations1.append(rpy1)
        orientations2.append(rpy2)

    # Convert positions to numpy arrays
    positions1 = np.array(positions1)
    positions2 = np.array(positions2)
    orientations1 = np.array(orientations1)
    orientations2 = np.array(orientations2)

    # # Calculate the squared differences between positions
    # squared_diff = (positions1 - positions2) ** 2

    # # Calculate the RMSE for each position (x, y, z)
    # rmse = np.sqrt(np.mean(squared_diff, axis=0))

    # Close the bags
    bag1.close()
    bag2.close()

    return positions1, positions2, orientations1, orientations2

def find_message_gaps(bag_file, topic, min_gap, start_time, end_time):
    bag = rosbag.Bag(bag_file)

    gap_list = []
    prev_time = start_time

    try:
        for topic, msg, timestamp in bag.read_messages(topics=[topic]):
            current_time = msg.header.stamp.to_sec()

            if prev_time is not None and current_time - prev_time > min_gap:
                gap_list.append((prev_time, current_time))

            prev_time = current_time

        # Check for gap at the end
        if end_time - prev_time > min_gap:
            gap_list.append((current_time, end_time))
    except Exception as e:
        print(f"Error reading {bag_file}")
        print(e)
        bag.close()
        return None

    bag.close()
    return gap_list

def find_all_gaps(directory, orig_dir, min_gap_size, verbose):
    all_gaps_surf = []
    all_gaps_brisk = []
    brisk_positions_for_rmse = np.empty((0, 3), dtype=float)
    surf_positions_for_rmse = np.empty((0, 3), dtype=float)
    brisk_oris_for_rmse = np.empty((0, 3), dtype=float)
    surf_oris_for_rmse = np.empty((0, 3), dtype=float)
    total_duration = 0
    brisk_bag_files = glob.glob(os.path.join(directory, '*brisk.bag'))
    for brisk_bag_file in brisk_bag_files:
        bag_file = brisk_bag_file.replace("_brisk.bag", "_groundtruth.bag").split("/")[-1]
        bag_file = find_bag(bag_file, orig_dir)
        if verbose:
            print(f"Processing {bag_file}")
        try:
            start_time = rosbag.Bag(bag_file).get_start_time()
            end_time = rosbag.Bag(bag_file).get_end_time()
            gaps_brisk = find_message_gaps(brisk_bag_file, '/sparse_mapping/pose', min_gap_size, start_time, end_time)
            gaps_surf = find_message_gaps(brisk_bag_file.replace("brisk.bag", "surf.bag"), '/sparse_mapping/pose', min_gap_size, start_time, end_time)
            if gaps_brisk is None or gaps_surf is None:
                continue
            brisk_poses, surf_poses, brisk_oris, surf_oris =  (calculate_rmse(brisk_bag_file, brisk_bag_file.replace("brisk.bag", "surf.bag"), '/sparse_mapping/pose'))
            # print(brisk_poses.shape, brisk_positions_for_rmse.shape)
            brisk_positions_for_rmse = np.concatenate((brisk_positions_for_rmse, brisk_poses))
            surf_positions_for_rmse = np.concatenate((surf_positions_for_rmse, surf_poses))
            brisk_oris_for_rmse = np.concatenate((brisk_oris_for_rmse, brisk_oris))
            surf_oris_for_rmse = np.concatenate((surf_oris_for_rmse, surf_oris))
            total_duration += end_time - start_time
            for gap in gaps_brisk:
                all_gaps_brisk.append(gap[1] - gap[0])
            for gap in gaps_surf:
                all_gaps_surf.append(gap[1] - gap[0])
        except Exception as e:
            print(f"Error processing {bag_file}")
            print(e)
    # Create a new PDF file
    with PdfPages('gap_analysis.pdf') as pdf:
        fig, (ax1, ax2) = plt.subplots(1,2)

        plot_histogram(all_gaps_brisk, "BRISK", ax1)
        plot_histogram(all_gaps_surf, "SURF", ax2, color = 'orange')
        pdf.savefig()

        fig3, ax3 = plt.subplots()
        plot_histogram(all_gaps_brisk, "BRISK", ax3)
        plot_histogram(all_gaps_surf, "SURF", ax3, color = 'orange')

        # Save the figure to the PDF file
        pdf.savefig()

        fig3, ax3 = plt.subplots()
        plot_histogram(all_gaps_brisk, "BRISK (Log Scale)", ax3, log_scale=True)
        plot_histogram(all_gaps_surf, "SURF (Log Scale)", ax3, log_scale=True, color = 'orange')

        # Save the figure to the PDF file
        pdf.savefig()

        # Create a figure without axes
        fig = plt.figure()

        surf_coverage = (total_duration - sum(all_gaps_surf)) / total_duration * 100
        brisk_coverage = (total_duration - sum(all_gaps_brisk)) / total_duration * 100

        # Calculate the squared differences between positions
        squared_diff = (brisk_positions_for_rmse - surf_positions_for_rmse) ** 2
        squared_diff_rpy = (brisk_oris_for_rmse - surf_oris_for_rmse) ** 2

        # Calculate the RMSE for each position (x, y, z)
        rmse = np.sqrt(np.mean(squared_diff, axis=0))
        rmse_rpy = np.sqrt(np.mean(squared_diff_rpy, axis=0))

        # Add text directly to the figure
        plt.text(0.5, 0.4, 'SURF Coverage: {}%'.format(surf_coverage), ha='center', va='center', fontsize=12)
        plt.text(0.5, 0.5, 'BRISK Coverage: {}%'.format(brisk_coverage), ha='center', va='center', fontsize=12)
        plt.text(0.5, 0.6, 'XYZ RMSE: {}'.format(rmse), ha='center', va='center', fontsize=12)
        plt.text(0.5, 0.7, 'RPY RMSE: {}'.format(rmse_rpy), ha='center', va='center', fontsize=12)

        # Remove axes and ticks
        plt.axis('off')

        # Add the axes to the PDF file
        pdf.savefig()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("-d", "--matched_dir", help="Directory of matched bags.")
    parser.add_argument("-g", "--orig_dir", help="Directory of original bags.")
    parser.add_argument("-m", "--min_gap_size", help="Min size to count as a gap.")
    parser.add_argument("-v", "--verbose", action="store_true", help="Print verbose output.")
    args = parser.parse_args()
    find_all_gaps(args.matched_dir, args.orig_dir, float(args.min_gap_size), args.verbose)


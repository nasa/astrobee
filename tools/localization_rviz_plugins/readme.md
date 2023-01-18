\page localizationrvizplugins Localization Rviz Plugins

# Package Overview
The localization rviz plugins package provides various plugins for use to display information on various localization components such as the graph localizer, imu augmentor, and sparse mapping results.  Plugins have the advantage of allowing many customizations to visualize information.  Additionally, whereas rviz provides visualization_msgs, plugins can subscibe to any message and prevent the creation of many visualization only messages that may live in functional code. 
The plugins in this package are spit between displays and panels, where displays provide 3D visualizations and panels display color coded text and numerical information.

## Usage
In the RVIZ Displays panel, select "Add" and select the desired plugin under the `localization_rviz_plugins` section. It should now appear in the Displays panel and any options should be available.
Ensure that the localization_rviz_plugins package is built and the required setup.bash script is sourced for the Astrobee repo, otherwise RVIZ will not be able to find the custom plugins.
Also ensure that the required Astrobee environment variables are set as described in the astrobee/readme.md file.

## Plugins
## Depth Odometry Display 
The depth odometry display publishes source and target point clouds and correspondence points between the clouds. It also publishes a correspondence image to the `/depth_odom/single_correspondence_image` topic if image space matching is used.

## Imu Augmentor Display
The imu augmentor display draws imu augmentor poses.  This is useful when comparing with graph localizer poses and sparse mapping poses, as ideally these are all alligned.

## Localization Graph Display 
The localization graph display draws the full history of poses in the latest graph localization message.  It also draws imu estimates between poses as arrows, and publishes optical flow feature track images using the feature tracks in the localizer.

## Localization Graph Panel 
The localization graph panel displays lots of information about the most recent graph localizer, including factor counts, factor information, timing, imu information, and more.

## Pose Display
The pose display draws a history of poses. This is useful when comparing a pose topic such as sparse mapping poses or groundtruth poses with localization.

## Sparse Mapping Display
The sparse mapping display loads a sparse map and publishes it as a point cloud to the `sparse_mapping/map_cloud` topic.

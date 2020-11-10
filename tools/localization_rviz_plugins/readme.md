\page localizationrvizplugins Localization Rviz Plugins

# Package Overview
The localization rviz plugins package provides various plugins for use to display information on various localization components such as the graph localizer, imu augmentor, and sparse mapping results.  Plugins have the advantage of allowing many customizations to visualize information.  Additionally, whereas rviz provides visualization\_msgs, plugins can subscibe to any message and prevent the creation of many visualization only messages that may live in functional code. 
The plugins in this package are spit between displays and panels, where displays provide 3D visualizations and panels display color coded text and numerical information.

## Plugins
## Localization Graph Display 
The localization graph display draws the full history of poses in the latest graph localization message.  It also draws imu estimates between poses as arrows, and publishes optical flow feature track images using the feature tracks in the localizer.

## Localization Graph Panel 
The localization graph panel displays lots of information about the most recent graph localizer, including factor counts, factor information, timing, imu information, and more.

## Imu Augmentor Display
The imu augmentor display draws imu augmentor poses.  This is useful when comparing with graph localizer poses and sparse mapping poses, as ideally these are all alligned.

## Sparse Mapping Display
The sparse mapping display draws sparse mapping poses. These are useful as a groundtruth indicate to see when localization drift occurs.

## Imu Measurement Display
This draws imu acceleration measurements as arrows.

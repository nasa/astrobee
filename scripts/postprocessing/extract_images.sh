#!/bin/bash

#
# Extract images from a set of bags in the current directory
#
# The images are extracted into a directory 'images' sibling to the current directory
# with subdirs corresponding to the timestamp of the bag start and the camera name
#

fsw_build=${HOME}/build/amd64/ff_develop

extract_imgs=${fsw_build}/devel/lib/localization_node/extract_image_bag

for b in *.bag
do
    ts=${b%%_*}

    echo "processing bag with timestamp = $ts"

    has_nav=`rosbag info $b | grep nav_cam | wc -l`
    if [ $has_nav -eq 1 ]
    then
	echo "  extracting nav_cam images..."
	nav_dir=../images/${ts}/nav_cam
	mkdir -p $nav_dir
	$extract_imgs -image_topic=/mgt/img_sampler/nav_cam/image_record -output_directory=$nav_dir -output_format "%04i.jpg" $b
    fi

    
    has_dock=`rosbag info $b | grep dock_cam | wc -l`
    if [ $has_dock -eq 1 ]
    then
	echo "  extracting dock_cam images..."
	dock_dir=../images/${ts}/dock_cam
	mkdir -p $dock_dir
	$extract_imgs -image_topic=/mgt/img_sampler/dock_cam/image_record -output_directory=$dock_dir -output_format "%04i.jpg" $b
    fi

done

    

\page build_map_from_multiple_bags Build map from multiple bags  
Several tools exist to ease the process of building a new map from a set of bagfiles. For more options/instructions for each tool, 
use   
`rosrun package_name tool_name.py -h`. 


## Steps
### 1. Convert bags from bayer to rgb if necessary  
Assuming the bags are all in the same directory, from that directory use:   
`rosrun bag_processing convert_all_bayer_bags.py`  

The original bayer bags can now be removed or moved elsewhere.

### 2. Splice bags using splice tool
For each bagfile, use the splice tool with:    
`rosrun bag_processing splice_bag.py bag_name`  

The mapping pipeline struggles with in place rotations, so try to splice a bagfile before and after the rotation if necessary.
If the rotation contains enough simultaneous translational movement, do not splice around the rotation since splicing could result in different segments of a movement no longer matching to each other when they are later merged if there is not enough overlap in their images.

If a bag contains multiple movements that contain sufficient overlap, these can be spliced to make the mapping process more efficient (smaller bags are faster to create maps for) and allow for more modular sanity checks (if movements are spliced and one fails to map well, the other may still be usable).

After splicing, the original bags can be removed or moved elsewhere.

### 3. Build individual SURF maps for each spliced bag
Run    
`rosrun sparse_mapping make_surf_maps.py -r robot_name -w world_name`

This builds individual maps in parallel for each spliced bag in the directory. The mapping process first removes low movement images from a bagfile to prevent triangulation issues and make the mapping process more efficient, then builds a surf map (detects image features, matches features, runs incremental bundle adjustment using successive images, runs a final round of bundle adjustment on the whole map).

View the map poses using  
`rosrun sparse_mapping nvm_visualize -only_poses`
to ensure the maps were built successfully.


### 4. Merge SURF maps
A common merge strategy for a set of movements is to identify the longest movement that covers the most of an area and incrementally merge in maps that have sufficient overlap with this. Use:    
`rosrun sparse_mapping merge_maps larger_map_name smaller_map_name -output_map combined_map_name -num_image_overlaps_at_endpoints 1000000`

First merge the new movements together using the above strategy, then optionally merge the resultant combined new movements map into an already existing map (that has some overlap with the new combined movements maps). 

View the resulting map poses using  
`rosrun sparse_mapping nvm_visualize -only_poses`
to ensure the maps were built successfully.

### 5. Register SURF map using provided world points
Since mapping is perfomed using monocular images, no set scale is provied and the map needs to be registered using real world points before being used for localization. Additionally, providing known point locations for images in the map can improve the accuracy of the map creation. 

First copy the map that should be registered then run:    
`rosrun sparse_mapping build_map -registration registration_points.pto registration_points.txt -output_map registered_map_name` 

The creation of registration points is detailed in `build_map.md`, images from the map are used to manually select feature locations and 3D points for these features are selected using a 3D model of the mapped environment provided externally.

### 6. Build BRISK map for localization 
Use:     
`rosrun sparse_mapping make_brisk_map.py surf_map_name -r robot_name -w world_name -d map_directory`  
to rebuild the SURF map with BRISK features for use with localization. 


### 7. Verify BRISK map using localization
Use:     
`rosrun sparse_mapping run_graph_bag_and_plot_results bag_name brisk_map_name config_path --generate-image-features -r robot_config -w world_name`  
to test the map using localization. 

The bags used here should not have been used for map creation but should contain data in the area of the map. If they contain IMU data full localization results will be generated, otherwise only pose estimates using the bag images registered with the provided map will be plotted. Make sure to include the `--generate-image-features` option to generate new map matches with the provided map. 

This script creates a pdf plotting localization accuracy and map-based pose estimates. If the map is well made, the localization and pose estimates should be relatively smooth and consistent. Jumps in these tend to indicate that portions of the map may not be well aligned.

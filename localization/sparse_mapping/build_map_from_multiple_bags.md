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

### 3. Build individual maps for each spliced bag
Run    
`rosrun localization_analysis make_surf_maps.py`

This builds individual maps in parallel for each spliced bag in the directory. The mapping process first removes low movement images from a bagfile to prevent triangulation issues and make the mapping process more efficient, then builds a surf map (detects image features, matches features, runs incremental bundle adjustment using successive images, runs a final round of bundle adjustment on the whole map).


### 4. Merge maps
A common merge strategy for a set of movements is to identify the longest movement that covers the most of an area and incrementally merge in maps that have sufficient overlap with this. Use:    
`rosrun sparse_mapping merge_maps larger_map_name smaller_map_name -output_map combined_map_name -num_image_overlaps_at_endpoints 1000000`

Sometimes an existing map is available that may contain areas not covered in the new bags. In this case, first merge the new movements together using the above strategy, then merge the resultant combined new movements map into the existing map. 

### 5. Register map against real world
Since mapping is perfomed using monocular images, no set scale is provied and the map needs to be registered using defined real world points before using for localization. 

First copy the map that should be registered then run:    
`rosrun sparse_mapping build_map -registration registration_points.pto registration_points.txt -output_map registered_map_name` 

The creation of registration points is detailed in `build_map.md`, images from the map are used to manually select feature locations and 3D points for these features are selected using a 3D model of the mapped environment provided externally.

### 6. Verify the resulting map
Use:     
`rosrun localization_analysis run_graph_bag_and_plot_results bag_name map_name config_path --generate-image-features -r robot_config -w world_name`  
to test localization with the map. 

The bags used here should not have been used for map creation but should contain data in the area of the map. They additionally need IMU data, if this is not available image registration can be testing using the sparse_mapping_pose_adder tool. Make sure to include the `--generate-image-features` option since image features in the bag are recorded using matches with whatever map was used when the bag was recorded. 

This script creates a pdf plotting localization accuracy and map-based pose estimates. If the map is well made, localization pose estimates should be relatively smooth and consistent. Jumps in poses tend to indicate the portions of the map may not be well aligned.

Tools to generate a heatmap visualization in RViz (Gazebo) of the features distribution (or map coverage) for a given map or trajectory contained in a bag.
If this process is used to observe the coverage in a given bag at a given activity, Step 1 can be skipped.

Steps:
1. Generate a bag from all the images in the map of interest. 
Inputs:  dir/where/jpg are
Tool: ~/coverage_analysis/img_to_bag.py
Outputs: bagOfTheMap.bag
 
Move all the image files used to generate the map to a single directory
Run "python img_to_bag.py /dir/where/jpgs/are /dir/where/bag/should/go/bagOfTheMap.bag", bagOfTheMap.bag = 20210518_cabanel.bag
If this gives an error, run rosdep update.

2. Run the localization node to find ML landmarks
Inputs: activity_name, activity_date, bagOfInterest.bag
Tools: roslaunch, rosservice, ~/coverage_analysis/activity_db_generator.py, rosbag play
Outputs: activity_database_file with recommended naming convention "YYYYMMDD_map_activity_db.csv" (The coordinates in the CSV output file are in the Astrobee's body inertial frame already).

Follow the instructions in https://github.com/nasa/astrobee/blob/master/localization/sparse_mapping/build_map.md for "Verify localization against 
a sparse map on a local machine"

Make sure your environment variables are correct, and that the symbolic link in ~/astrobee/astrobee/resources/maps/iss.map points to the map of interest. 
To do this, first check iss.map is pointing to your map of interest by running in a terminal "ls -lah~/astrobee/astrobee/resources/maps/iss.map". 
If it is not pointing to the map of interest, move the current map, and make it point to the correct one by running in a terminal
 "mv ~/astrobee/astrobee/resources/maps/iss.map ~/astrobee/astrobee/resources/maps/iss_backup.map"
 "ln -s ~/map/location/<mapOfInterest>.map ~/astrobee/astrobee/resources/maps/iss.map"
Run in a terminal 
 "roslaunch astrobee astrobee.launch mlp:=local llp:=disabled nodes:=framestore,localization_node world:=iss".
In another terminal enable localization: 
 "rosservice call /loc/ml/enable true" and wait for it to return "success: True".
In this same terminal, run 
"python ~/coverage_analysis/activity_db_generator.py <activity date> <map name> <activity name> <location to save>", where
activity date = 20210726
map name      = dali
activity name = KiboEventRehRun1
location to save = /dir/where/database/file/will/be/
The final file will be /dir/where/database/file/will/be/20210726_dali_KiboEventRehRun1_db.csv

After starting activity_DatabaseGenerator.py, run in another terminal the bag just created: 
"rosbag play bagOfInterest.bag /mgt/img_sampler/nav_cam/image_record:=/hw/cam_nav"
If necessary, once the database of ML features and robot poses is generated change back the map as it was by running in a terminal
"rm ~/astrobee/astrobee/resources/maps/iss.map; mv ~/astrobee/astrobee/resources/maps/iss_backup.map ~/astrobee/astrobee/resources/maps/iss.map"

After the process has finished, Ctrl-C all the terminals.

3. Generate a coverage database
Inputs:  activity_database_file 
Tools:   ~/coverage_analysis/coverage_analyzer.py
Outputs: activity_coverage_database_file. Temporary files: output_features_database.csv, output_features_database_nonrepeat.csv,  

This step first extracts from activity_database.csv only the ML feature poses into a temporary file called, for example, 20210518_cabanel_features_db.csv or 20210726_dali_KiboEvRhRun1_features_db.csv. 
It then identifies repeated ML features and copies only one of the repeated ML features into a temporary file called, for example, 20210518_cabanel_features_db_nonrepeat.csv or 20210726_dali_KiboEvRhRun1_features_db_nonrepeat.csv
These repeated features are unique, however it should be kept in mind that multiple cameras may be able to see the same feature. For visualization purposes is faster to have unique features.
The next step generates the list of cube centers of size defined by grid_size along the different walls in the JEM (Overhead, Aft, Forward, Deck).
These walls are divided into the 8 bays of the JEM, where 1 corresponds to the bay closest to the entry node and 8 to the bay closest to the airlock. 
If a feature is within the volume of the cube centered at the pose being searched, a counter is increased and the final number of matches and the center of the cube is saved in the activity_coverage_database.csv file called, for example, 
20210518_cabanel_coverage_db.csv or 20210726_dali_KiboEvRhRun1_coverage_db.csv
The format of this file consist of Number-of-ML-Features-found-within-the-cube-defined-by X-Y-Z-Pose-of-the-center-of-the-cube-checked.

Run 
"python ~/coverage_analysis/coverage_analyzer.py <activity_database_file>"  where 
activity_database_file = /dir/where/database/file/is/20210726_dali_KiboEvRhRun1_db.csv, 
database_fileOut       = /dir/where/results/are/going/to/be/placed/20210726_dali_KiboEvRhRun1_features_db.csv,
feat_only_fileOut      = /dir/where/results/are/going/to/be/placed/20210726_dali_KiboEvRhRun1_features_db_nonrepeat.csv, and
activity_coverage_database_file = /dir/where/results/are/going/to/be/placed/20210518_cabanel_coverage_db.csv or /dir/where/results/are/going/to/be/placed/20210726_dali_KiboEvRhRun1_db.csv

At this point, the user has a few options:
1. Generate a 3D heatmap visualization in Gazebo of the studied map's general coverage of the JEM
   This is accomplished following Step 4.
2. Generate a 3D animation and visualization in Gazebo of the studied robot's trajectory coverage at each pose of the trajectory
   This is accomplished following Step 5.
3. Generate a statistics report in PDF of the general and detailed coverage of the studied map or robot's trajectory's coverage
   This is accomplished following Step 6.

4. Generate Walls Heatmap visualization in Gazebo
Inputs: activity_coverage_database_file 
Tools: ~/coverage_painter.py, roslaunch
Outputs: 30cm^3 grid painted as shown in rviz (Gazebo) 

In a terminal, run 
  "roslaunch astrobee sim.launch dds:=false speed:=0.75 rviz:=true" to bring RViz up. 
If the Rviz display looks too cluttered, most topics can be turned off except "Registration".
In another terminal run 
  "python ~/coverage_anaylisis/coverage_painter.py <activity_coverage_database_file> map_coverage" where

activity_coverage_database_file = /dir/where/results/are/going/to/be/placed/20210518_cabanel_coverage_db.csv
    
This publishes cubes representing the map coverage in 20210518_cabanel_coverage_db.csv according to the following color scheme:
red    if  0 registered ML features found in any cube are present in this cube
orange if  1-10 registered ML features found in any cube are present in this cube
yellow if 11-20 registered ML features found in any cube are present in this cube
green  if 21-40 registered ML features found in any cube are present in this cube
blue   if 40+ registered ML features found in any cube are present in this cube

5. Generate Robot's Trajectory Heatmap visualization in RViz
Inputs: activity_database_file
Tools: ~/coverage_painter.py, roslaunch
Outputs: Animated trajectory consisting of each of the trajectory poses colored coded according to their coverage level 

In a terminal, run "roslaunch astrobee sim.launch dds:=false speed:=0.75 rviz:=true" to bring RViz up. 
If the Rviz display looks too cluttered, most topics can be turned off except "Registration".
In another terminal run 
  "python ~/coverage/anaylisis/coverage_painter.py <activity_database_file> robot_coverage" where

activity_database_file = /dir/where/database/file/is/20210726_dali_KiboEvRhRun1_db.csv, 

This will publish each pose contained in the trajectory and will color code them according to the following color scheme:
red    if  0 registered ML features found in any cube are present in this cube
orange if  1-10 registered ML features found in any cube are present in this cube
yellow if 11-20 registered ML features found in any cube are present in this cube
green  if 21-40 registered ML features found in any cube are present in this cube
blue   if 40+ registered ML features found in any cube are present in this cube

6. Generate stats report
Inputs: activity_coverage_database_file = /dir/where/map-coverage/file/is/20210518_cabanel_coverage_db.csv or /dir/where/robot-trajectory-coverage/file/is/20210726_dali_KiboEvRhRun1_coverage_db.csv
Tools: ~/coverage_stats_reporter.py
Outputs: CSV and PDF Stats reports, saved at the input file's same location 
(CSV: /dir/where/map-coverage/file/is/20210707_dali_coverage_db_stats.csv or /dir/where/robot-trajector-coverage/file/is/20210726_dali_KiboEvRhRun1_coverage_db_stats.csv)
(PDF: /dir/where/map-coverage/file/is/20210707_dali_coverage_db_stats_report.pdf or /dir/where/robot-trajector-coverage/file/is/20210726_dali_KiboEvRhRun1_coverage_db_stats_report.pdf)

Generates a statistics report in CSV and PDF of the coverage on each wall (Overhead, Aft, Forward, Deck) and airlock based on the number of registered ML features found on every cube analyzed.
The report consists of two images: 1) the general map or bag coverage distribution and the 2) the detailed map or bag coverage distribution.
It provides the coverage percentage based on the number of registered ML features at each wall distributed across five ranges: 0ML, 1-10ML, 11-20ML, 21-40ML, and 40+ML, as well as the total number of registered ML features in 
the analized wall, and the percentage of the total number of ML features analyzed within the JEM walls and airlock.
In a terminal run
 "python ~/coverage_analysis/coverage_stats_reporter.py <activity_coverage_database_file>" where 
activity_coverage_database_file = /dir/where/map-coverage/file/is/20210518_cabanel_coverage_db.csv or /dir/where/robot-trajectory-coverage/file/is/20210726_dali_KiboEvRhRun1_coverage_db.csv


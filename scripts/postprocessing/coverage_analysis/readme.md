# **COVERAGE ANALYSIS README**

### Tools to generate a heatmap visualization in RViz (Gazebo) of the features distribution (or map coverage) for a given map or trajectory contained in a bag. If this process is used to observe the coverage in a given bag at a given activity, Step 1 can be skipped.

##  Steps: 

## 1. Generate a bag from all the images in the map of interest.
- Inputs: dir/where/jpg/are 
- Tool: ~/coverage\_analysis/img\_to\_bag.py
- Outputs: bagOfTheMap.bag

Move all the image files used to generate the map to a single directory. 
Run  
`python img_to_bag.py /dir/where/jpgs/are/dir/where/bag/should/go/bagOfTheMap.bag`  
where  
bagOfTheMap.bag = 20210518_cabanel.bag  
If this gives an error, run rosdep update.

## 2. Find ML landmarks 
- Inputs: bagOfInterest.bag, mapToTestAgainst.map
- Tools: run_map_matcher.cc
- Outputs: output.bag

In a terminal run  
`rosrun  localization_analysis run_map_matcher --bagfile </dir/to/bagOfInterest.bag>  --map-file </dir/to/mapToTestAgainst.map> -o /dir/where/to/save/output.bag -c <dir/to/astrobee/config>`  

## 3.  Generate pose/ML features database.
- Inputs: activity\_name, activity\_date, bagOfInterest.bag 
- Tools: ~/coverage\_analysis/activity\_db\_generator.py
- Outputs: activity\_database\_file with recommended naming convention "YYYYMMDD\_map\_activity\_db.csv" (The coordinates in the CSV output file
are in the Astrobee's body inertial frame already).

Run in a terminal  
`python ~/coverage_analysis/activity_db_generator.py <activity date> <map name> <activity name> <location to save> <bag_name>`
where  
activity date = 20210726  
map name = dali  
activity name = KiboEventRehRun1  
location to save = /dir/to/database/file/db.csv
The final file will be called /dir/to/database/file/20210726_dali_KiboEventRehRun1_db.csv  

## 4. Generate a coverage database 
- Inputs: activity\_database\_file 
- Tools: ~/coverage\_analysis/coverage_analyzer.py 
- Outputs: activity\_coverage\_database_file. 
- Temporary files: output\_features\_database.csv, output\_features\_database\_nonrepeat.csv

This step first extracts from activity\_database.csv only the ML feature poses into a temporary file called, for example, 20210518\_cabanel\_features_db.csv or 20210726\_dali\_KiboEvRhRun1\_features_db.csv.  

It then identifies repeated ML features and copies only one of the repeated ML features into a temporary file called, for example, 20210518\_cabanel\_features\_db\_nonrepeat.csv or 20210726\_dali\_KiboEvRhRun1\_features\_db\_nonrepeat.csv These repeated features are unique, however it should be kept in mind that multiple cameras may be able to see the same feature. For visualization purposes is faster to have unique features.  

The next step generates the list of cube centers of size defined by grid\_size along the different walls in the JEM (Overhead, Aft, Forward, Deck). These walls are divided into the 8 bays of the JEM, where 1 corresponds to the bay closest to the entry node and 8 to the bay closest to the airlock.  

If a feature is within the volume of the cube centered at the pose being searched, a counter is increased and the final number of matches and the center of the cube is saved in the activity\_coverage\_database.csv file called, for example, 20210518\_cabanel\_coverage\_db.csv or 20210726\_dali\_KiboEvRhRun1\_coverage\_db.csv.   

The format of this file consist of Number-of-ML-Features-found-within-the-cube-defined-by X-Y-Z-Pose-of-the-center-of-the-cube-checked.

Run in a terminal  
`python ~/coverage_analysis/coverage_analyzer.py <activity_database_file>`  
where   
activity\_database\_file = /dir/where/database/file/is/20210726\_dali\_KiboEvRhRun1\_db.csv,  
database\_fileOut = /dir/where/results/are/going/to/be/placed/20210726\_dali\_KiboEvRhRun1\_features\_db.csv,  
feat\_only\_fileOut = /dir/where/results/are/going/to/be/placed/20210726\_dali\_KiboEvRhRun1\_features\_db\_nonrepeat.csv,  
activity\_coverage\_database\_file = /dir/where/results/are/going/to/be/placed/20210518\_cabanel\_coverage\_db.csv  
or  
activity\_coverage\_database\_file = /dir/where/results/are/going/to/be/placed/20210726\_dali\_KiboEvRhRun1\_db.csv

At this point, the user has a few options: 
- **1.** Generate a 3D heatmap visualization in Gazebo of the studied map's general coverage of the JEM  
This is accomplished following Step 4. 
- **2.** Generate a 3D animation and visualization in Gazebo of the studied robot's trajectory coverage at each pose of the trajectory  
This is accomplished following Step 5. 
- **3.** Generate a statistics report in PDF of the general and detailed coverage of the studied map or robot's trajectory's coverage  
This is accomplished following Step 6.

## 5. Generate Walls Heatmap visualization in Gazebo 
- Inputs: activity\_coverage\_database\_file 
- Tools: ~/coverage_painter.py, roslaunch
- Outputs: 30cm^3 grid painted as shown in rviz (Gazebo)

In a terminal, run   
`roslaunch astrobee sim.launch dds:=false speed:=0.75 rviz:=true`  
to bring RViz up. If the Rviz display looks too cluttered, most topics can be turned off except "Registration". In another terminal run  
`python ~/coverage_anaylisis/coverage_painter.py <activity_coverage_database_file> map_coverage`  
where  
activity\_coverage\_database\_file = /dir/where/results/are/going/to/be/placed/20210518\_cabanel\_coverage\_db.csv  

This publishes cubes representing the map coverage in 20210518_cabanel_coverage_db.csv according to the following color scheme: 
- red if 0 registered ML features found in any cube
- orange if 1-10 registered ML features found in any cube 
- yellow if 11-20 registered ML features found in any cube
- green if 21-40 registered ML features found in any cube 
- blue if 40+ registered ML features found in any cube 

## 6. Generate Robot's Trajectory Heatmap visualization in RViz 
- Inputs: activity\_database\_file 
- Tools: ~/coverage\_painter.py, roslaunch 
- Outputs: Animated trajectory consisting of each of the trajectory poses colored coded according to their coverage level

In a terminal, run  
`roslaunch astrobee sim.launch dds:=false speed:=0.75 rviz:=true`  
to bring RViz up. If the Rviz display looks too cluttered, most topics can be turned off except "Registration". In another terminal run 
`python ~/coverage/anaylisis/coverage_painter.py <activity_database_file> robot_coverage`  
where  
activity\_database\_file = /dir/where/database/file/is/20210726\_dali\_KiboEvRhRun1\_db.csv  

This will publish each pose contained in the trajectory and will color code them according to the following color scheme: 
- red if 0 registered ML features found in any cube
- orange if 1-10 registered ML features found in any cube 
- yellow if 11-20 registered ML features found in any cube
- green if 21-40 registered ML features found in any cube 
- blue if 40+ registered ML features found in any cube

## 7. Generate stats report 
- Inputs: activity\_coverage\_database\_file = /dir/where/map-coverage/file/is/20210518\_cabanel\_coverage\_db.csv or /dir/where/robot-trajectory-coverage/file/is/20210726\_dali\_KiboEvRhRun1\_coverage\_db.csv
- Tools: ~/coverage\_stats\_reporter.py
- Outputs: CSV and PDF Stats reports, saved at the input file's same location  
1) CSV: /dir/where/map-coverage/file/is/20210707\_dali\_coverage\_db\_stats.csv or /dir/where/robot-trajector-coverage/file/is/20210726\_dali\_KiboEvRhRun1\_coverage\_db\_stats.csv  
2) PDF: /dir/where/map-coverage/file/is/20210707\_dali\_coverage\_db\_stats\_report.pdf or /dir/where/robot-trajector-coverage/file/is/20210726\_dali\_KiboEvRhRun1\_coverage\_db\_stats\_report.pdf  

Generates a statistics report in CSV and PDF of the coverage on each wall (Overhead, Aft, Forward, Deck) and airlock based on the number of registered ML features found on every cube analyzed. 
The report consists of two images:  
- the general map or bag coverage distribution and the
- the detailed map or bag coverage distribution.  

It provides the coverage percentage based on the number of registered ML features at each wall distributed across five ranges: 0ML, 1-10ML, 11-20ML, 21-40ML, and 40+ML, as well as the total number of registered ML features in the analized wall, and the percentage of the total number of ML features analyzed within the JEM walls and airlock. 
In a terminal run  
`python ~/coverage_analysis/coverage_stats_reporter.py <activity_coverage_database_file>`  
where  
activity\_coverage\_database\_file = /dir/where/map-coverage/file/is/20210518\_cabanel\_coverage\_db.csv  
or  
activity\_coverage\_database\_file = /dir/where/robot-trajectory-coverage/file/is/20210726\_dali\_KiboEvRhRun1\_coverage\_db.csv



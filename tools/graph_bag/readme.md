\page graphbag Graph Bag

# Package Overview
The graph bag package provides several tools for measuring localization performance as described below.

# Tools
## GraphBag
Graph bag simulates localization using a saved bagfile.  Rather than relying on rosbag play, it loads measurements directly and greatly decreases runtime.  To accurately simulate measurement delays and drops, the LiveMeasurementSimulator class is provided along with a config file to provide delays and minimum spacing between measurements.  Graph bag saves results to a new bag file that can be processed by the plot\_results\_main.py script into a pdf showing information such as poses estiamtes, velocity estimates, bias estiates, covariances, and more.

## BagImuFilterer
The bag imu filterer enables the testing of imu filters written in c++.  It parses a bag file and loads a c++ imu filter, then saves the filtered data to a new bag file.  The filtered data can be plotted and analyzed by the imu\_analyzer script.

#Scripts
## bag\_sweep
The bag sweep tool runs the graph bag tool in parallel on multiple bag files.  It takes config file with bagnames, map names, and robot configs and produces pdfs and result bagfiles for each entry.
Example config file:
/home/bag\_name.bag /home/map\_name.map /mgt/img\_sampler/nav\_cam/image\_record /home/astrobee/astrobee config/robots/bumble.config iss false
/home/bag\_name\_2.bag /home/map\_name.map /mgt/img\_sampler/nav\_cam/image\_record /home/astrobee/astrobee config/robots/bumble.config iss false

Example bag sweep command:
rosrun graph\_bag bag\_sweep.py /home/bag\_sweep\_config.csv /home/output\_dir 

## check\_bags\_for\_gaps
This is a simple tool to check for large gaps in imu or image data in a bagfile.

## imu\_analyzer\_main
The imu analyzer tool plots imu data and also filtered imu data.  It can use python filtering tools to lowpass filter imu data at a certain cutoff frequency, or it can use a provided bag file with filtered imu data and compare this against a bag file with unfiltered data.  Data is plotted in its raw form and also after being passed through an FFT to show its frequency response.

## merge\_bags
Merge bags looks for bag files in the current directory with a provided name prefix and merges these into a single bag file.  Bags are assumed to be numbered and are merged in numerical order.

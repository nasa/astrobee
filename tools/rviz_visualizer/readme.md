\page rvizvisualizer RViz Visualizer

This helper node publishes various topics that aid
in visualization using rviz. It currently publishes
a model of the granite table and the rviz tf2 transform.

## Getting RVIZ Overhead Cam Display
![RVIZ](../images/mobility/overhead_cropped.png)

ROS can project the RVIZ display into the camera frame of the overhead camera and overlay it on the image in real time.

### Prereqs 

You will need the Kumar Robotics version of the flea3 driver (https://github.com/KumarRobotics/flea3, https://github.com/KumarRobotics/camera_base). This needs to be built with the catkin build system. 

	mkdir -p catkin_ws/src && cd catkin_ws/src 
	git clone https://github.com/KumarRobotics/flea3 && git clone https://github.com/KumarRobotics/camera_base 
	cd .. && source /opt/ros/kinetic/setup.bash && catkin init 
	catkin config -DCMAKE_BUILD_TYPE=Release 
	catkin build --jobs 12 
	echo source `pwd`/devel/setup.bash --extend >> ~/.bashrc  

You will also need the jsk-visiualization plugins. (Don't try to build these from source, it's doable, but a mess)

	sudo apt-get install ros-kinetic-jsk-visualization

### Setup

The camera_info directory inside rviz_visualizer needs to be copied to ~/.ros
The rules.d/40-pgr.rules file needs to be copied to /etc/udev/rules.d/ and the user needs to be added to the plugdev group.

### Launching 
	roslaunch rviz_visualizer cam.launch

![RVIZ](../images/mobility/display2_marked_up.png)
After launching, add the CameraOverlay to the rviz window

![RVIZ](../images/mobility/overhead_marked_up.png)
Select the camera topic and set alpha to 1

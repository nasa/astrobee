----
## Running the Simulator 

To run the simulator headless run: 

    roslaunch astrobee sim.launch

Add the following commands at the end of the launch command if you want any of the options. 

To open an rviz terminal

    rviz:=true  

To open the gazebo gui 

    sviz:=true

To view the GNC visualizer 

    gviz:=true

To launch the astrobee on a different namespace 

    ns:=name

----
## File Layout

The simulation directory holds all things related to running a simulation of Astrobee. There are two main subdirectories, astrobee description  and astrobee gazebo. 

----
## Astrobee Desicription

This folder includes everything used to describe astrobee as it is loaded in simulation. The config files are configs used for common astrobee namespaces in Rviz. They load an rviz view of most of the relevant information being published by the gazebo simulator. 

The urdf's are used to load everything connected to astrobee in simulation. These urdf's also include which plugins are connected to them. Those plugins are found in the astrobee_gazebo subdirectory and will be described later on. 

* Astrobee - links to all the other urdf's that will be loaded whenever an astrobee is spawned. 
    - astrobee_body - the inertial, visual, and collsiion properties of the astrobee body
    - model_flashlight - all the information for the forward and aft flashlight models
    - model_laser - a model for the laser. 
    - model_perching_arm - this is currently commented out as the arm does not receive control comands and negatively effects the control of astrobee as it moves in zero gravity. 
    - model_pmc - loads the model pmc plugin. 
    - model_truth - loads the truth plugin. 
    - sensor_doc_cam, haz_cam, nav_cam, and perch_cam - all load their respective camera plugins. The URDF's also describe the image width, height, depth, and field of view. (note the haz and perch cam (depth cameras) are currently commented out becuase they are extremly inefficient and are significantly slowing down the simulator)
    - sensor_imu - loads the IMU plugin and defines its update rate.

----
## Astrobee Gazebo

Everything in this folder is directly related to the gazebo simulator. 

### Launch

Start_simulation is called when a new simulation is started, it loads the gazebo world and server. 

Spawn_astrobee loads an astrobee into the gazebo world. This launch file can be used to spawn a second or third astrobee into the world on different name spaces once the simulator has been started. 

### Media, Models, and Worlds

These directories include all the meshes and models used in the simulator. From what the world looks like, to what the robot looks like. All of that can be found in one of these directories. 

### Src

This is the directory with all the plugins for the gazebo simulator. 

**astrobee_gazebo.cc**

This is the class used for common traits all gazebo plugins have. Each plugin inherits either a freeflyerSensorPlugin or a freeflyerModelPlugin. Those both inherit from freeflyer nodelet so they produce heartbeats like all other freeflyer nodes. Each plugin also has it's own queue thread for callbacks, allowing the plugins to subscribe and publish to multiple topics if necessary without any callback issues between plugins all running on the gzserver. 

The extrinsics for each sensor is also setup based on the tf2 transform being published for the frame attached to the sensor in it's urdf. If a frame isn't specified, the sensor name will be used to attempt to find a proper transform. For most sensors the pose from tf2 is directly transfered as the sensor pose. However there is a discrepency between the pose used for the flight software cameras and the pose used by gazebo for cameras. In the flight software a camera frame is defined by z pointing into the camera frame and x pointing the the right. In gazebo a camera is defined with z pointing up and x pointing into the camera frame. To transform from flight software to gazebo a rotation about x is needed followed by a rotation about z. 

**eps plugin**

Eventually this will model the battery levels. Right now it is not used. 

**flashlight plugin**

To adjust the flashlight models set the brightness to a value between 0-200, there is a forward and aft flashlight

    rosservice call /namespace/hw/light_forward "brightness: 0"
    rosservice call /namespace/hw/light_aft "brightness: 200"

**laser plugin**

To toggle the laser model:
    
    rosservice call /namespace/hw/laser/enable "enabled: true"

**perching arm plugin**

Not currently in use or functional. Eventually will be used to control the perching arm. 

**pmc plugin**

This plugin subscribes to the PMC Command topic. When a new PMC command is recieved, those blower states are sent to a simulink GNC model. That will then calculate the force and torque on the model produced from the desired blower commands. The relative force and torque are then applied to the body. 

**truth plugin**

This model plugin publishes the truth values of the pose, twist and accelecration values. 

**doc cam plugin**

This is a rgb camera that points backwards on the robot. The plugin copies the image data from the gazebo camera sensor into the image message and publishes it to the doc camera topic. 

**haz cam plugin**

This is a depth camera that points forwards on the robot. The plugin copies the image data from the gazebo depth camera sensor into the image message and publishes it to the haz camera topic. 

It also produces point clouds. This is currently extremely inefficient and so the depth cameras are not currently being used. 

**imu plugin**

The IMU plugin publishes to the IMU topic. On every update of the gazebo world, it gets the angular velocity and linear acceleration of the imu sensor and publishes this information. 

**nav cam plugin**

This is a rgb camera that points forwards on the robot. The plugin copies the image data from the gazebo camera sensor into the image message and publishes it to the nav camera topic. 

**perch cam plugin**

This is a depth camera that points backwards on the robot. The plugin copies the image data from the gazebo depth camera sensor into the image message and publishes it to the perch camera topic. 

It also produces point clouds. This is currently extremely inefficient and so the depth cameras are not currently being used. 

**sparse map plugin**

The sparse map responds to the ml localization enable service, to switch between localization modes. It publishes camera registration and feature messages for localization. It builds a point cloud with a defined number of features (defined in the sparse map config file). These are random points that would exist in a full point cloud produced from the nav cam. These points are then turned into individual features in the world frame. Those features, along with their pixel position in the nav came and the nav cam pose is all published as a feature message. A timer is used to delay between the registration and feature messages being published (this delay can also be changed in the sparse map config file). 





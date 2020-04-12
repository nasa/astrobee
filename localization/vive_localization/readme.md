\defgroup vive_localization Vive Localization
\ingroup localization

This is the high-level tool for post-processing raw HTC vive data captured by the low-level driver and recorded to a rosbag. It fuses light capture measurements to track the pose of a robot over time.

1. Calibration (optional) - A PnP solution is first used to separately calculate the robot trajectory in the master and slave lighthouse frames. Then, Kabsch is used to find the transform from the slave to the master lighthouse in order to minimize the squared error between the trajectories.  
2. Solving - PnP is used to estimate the the robot trajectory in the world frame by taking the average solution across all lighthouses that saw the robot at that instant. 
3. Refinement (optional) - The PnP solution is passed into ceres solver, and the trajectory is refined in such a way that minimizes the residual error between predicted and measured light angles. The trajectory can also be smoothed, but this is disabled by default.
4. Registration (optional) - Kabsch is used to find the projection of the trajectory into the world frame in such a way that aligns it to the  solution from another localization sytem. In our case, this is the vision-based EKF.
5. Projection - The solution is projected from the master lighthouse (vive frame) to the world frame, using the transform given by the registration phase.

# Calibration

In order to calibrate the system, you must first collect a rosbag containing the raw Vive data hw/vive/* and output from the Astrobee localization system loc/pose. Let's assume this bag is located at /path/to/test.bag.

First, examine the bag or use the vive_tool to determine your lighthouse and tracker serial numbers.

Second, declare the locations of the trackers with respect to the body-frame of the robot in your robot config file. If you are using the standard Vive brackets, you can just copy and paste the transforms below:

    robot_vive_extrinsics = {
        {
            -- port
            serial = "LHR-08DE963B", pose = transform(
                vec3(0, -0.1397, -0.1397),
                quat4(1.0, 0.0, 0.0, 0.0))
        },{
            -- starboard
            serial = "LHR-1FC0DEF4", pose = transform(
                vec3(0,  0.1397, -0.1397),
                quat4(0.0, 1.0, 0.0, 0.0))
        }
    }

Third, declare the lighthouses in your system in the world config file. It doesn't really matter what transforms you choose, because you will be solving for them as part of the calibration phase.

    world_vive_lighthouses = {
        {
            -- back
            serial = "2935772913", pose = transform(
                vec3(-1.3027, -0.364447, 1.72141),
                quat4(-0.580631, 0.495535, -0.265315, 0.589))
        },{
            -- front
            serial = "668207140", pose = transform(
                vec3(0, 0, 0),
                quat4(0, 0, 0, 1))
        }
    }

Fourth, declare the registration transform in your world config file. It doesn't really matter what transform you choose, because you will be solving for them as part of the registration phase.

    world_vive_registration = transform(
        vec3(0.634971, 1.17579, -1.63761),
        quat4(0.321928, -0.355711, 0.436771, 0.7609629))

Fifth, turn on all optional steps in the solver stage. make sure the following variables are set in astrobee/localization/vive_localization.config. 

    calibrate = true
    refine = true
    register = true

Finally, post-process the bag using the following command:

    roslaunch vive_localization vive_localization.launch bag:=/path/to/test.bag rviz:=true

Once the command has run check in the GUI to make sure that there is a good match between the EKF (red) and Vive (green) trajectories, and that the master and slave lighthouses look correct. If this is the case, then follow the instructions written to the console and update the lighthouse and registration information in your world config file.

# Tracking (offline)

To do this you will need a calibrated system and a rosbag containing the raw Vive data /hw/vive/*. This bag may also optional have output from the Astrobee localization system at /loc/pose.

First, turn off the calibration and registration steps of the solver. By doing this you use the default values in your world config, which maximizes accuracy. In other words, make sure the following variables are set in astrobee/localization/vive_localization.config. 

    calibrate = false
    refine = true
    register = false

Next, post-process the bag using the following command:

    roslaunch vive_localization vive_localization.launch bag:=/path/to/test.bag rviz:=true

Once the command has run check in the GUI to make sure that the Vive (green) trajectory looks sensible. If the bag file included EKF data, you will also see a red trajectory. By default, a CSV file comparing the Vive to EKF results for pose only will be written to ~/.ros/performance.csv. You can change this location with the argument out:=/path/to/file.csv.

    roslaunch vive_localization vive_localization.launch bag:=/path/to/test.bag out:=/path/to/performance.csv rviz:=true

# Notes

There are a number of additional options in the astrobee/localization/vive_localization.config file.
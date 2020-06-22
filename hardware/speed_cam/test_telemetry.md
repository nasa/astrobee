\page speed_cam_test_telemetry Test : Telemetry

The speed camera provides a velocity estimate, along with raw measurements: optical flow, inertial, and camera images. The data is broadcast to the topics `hw/speed_cam/speed`, `hw/speed_cam/optical_flow` `hw/speed_cam/imu` and `speed_cam/camera_image`, optionally prefixed by the name space for the robot on which the driver is being run. 

Although data is pushed at a constant rate on the serial bus, the driver only forwards those messages to topics with at least one subscriber.

Assuming the robot is running with name space `/`, telemetry can be displayed using the following command:

    rostopic echo /hw/speed_cam/optical_flow
    rostopic echo /hw/speed_cam/speed
    rostopic echo /hw/speed_cam/imu

Images can be viewed using the rqt image viewer

    rqt_image_view /hw/speed_cam/camera_image

If the flight software stack is not running, the `speed_cam_tool` can be used as a debugging interface to configure and query the device. To do this, first run the tool in the following way:

    rosrun speed_cam speed_cam_tool

Follow the instructions to print out the data of your choice. Note that if a camera image is requested, the data will be written to a PGM file that can be opened in any modern image viewer.
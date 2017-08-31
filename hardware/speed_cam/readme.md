\defgroup speed_cam Speedcam driver
\ingroup hw

The speedcam is a dedicated hardware component for estimating the velocity of a freeflyer, and is comprised of a suite of sensors and a dedicated Cortex-M4 processor. The device itself is a modified PX4FLOW, in which the ultrasonic distance is replaced with with a Teraranger laser distance sensor.

The device communicates with the LLP over USB. More specifically, the speedcam uses an FDTI chip as a serial over USB. The data packets are encoded using the standard mavlink protocol, and decoded by flight software with an appropriate driver, which was forked off the px-ros-pkg project.

In order to keep the speed camera computation loop time consistent, data is pushed at a constant rate up to the driver code. The driver decodes all messages, but only publishes messages if their corresponding topic has at least one subscriber.

\defgroup eps_driver_test_payload_power Test : Payload power
\ingroup eps_driver

The EPS subsystem is responsible for enabling and disabling power to payloads being carried by the astrobee. There are four payload bays -- two on the top side and two on the underside of the robot. 

Payload power is configured by a service call to the topic `hw/eps/configure_payload_power`, which is optionally prefixed by the name space for the robot on which the EPS is being run. This service call takes the form shown in \ref ff_hw_msgs_ConfigurePayloadPower. For example, assuming the robot is running on name space `/`, to turn power on to the payload with index 0, type the following on the command line:

    rosservice call /hw/eps/configure_payload_power { payload: 0, state: 1 }

If the flight software stack is not running, the `eps_tool` can be used as a debugging interface to configure power to the payload. To do this, first run the tool in the following way:

    rosrun eps_driver eps_tool

Then, select option 12 and follow the instructions.
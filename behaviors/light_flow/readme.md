\defgroup signal Signal behavior
\ingroup beh

Astrobee has 44 individually addressable LEDs on each PMC.  To control the LEDs, patterns specified by the LightFlow programming language are interpreted into messages containing the RGB value of each LED.  These messages are sent over ROS to a hardware driver that formats the message into a buffer, which is sent over I2C.  

# Internal behavior

After receiving a ```ff_msgs::SignalState``` message, the behavior node will first process the ID to make sure that it has knowledge of the corresponding LightFlow pattern (the class that holds the patterns is called `LightFlowStorage` inside of the `lightflow_storage.py` file).  Once the pattern is confirmed to exist and there is not a pattern running, the behavior node begins to interpret the LightFlow and send messages in real time.  To send messages at a constant update rate, the behavior node determines the time at which each message needs to be sent.  The behavior node then figures out the correct color for each LED, waits until the correct time to send the message, and sends it.  
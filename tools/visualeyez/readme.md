\page visualeyez Visualeyez marker tracking system

## Overview

The Visualeyez tracking system provides the ability to continuously track active LED markers in 3D space. Refer to the image below. The tracker (top left) is a bar containing three cameras. When an LED is in view of all three cameras, the system is able to solve for its position within a reference frame centered on the tracker. The VZSoft software collects raw measurements from the tracker (over a serial line) and plots the data in a GUI for the user.

![alt text](../images/tools/visualeyez.png "The Visualeyez System")

The GUI also provides the ability to stream measurements to third party applications. The company provides Matlab libraries for collecting this data (see ```matlab/vz```. We have written a small Matlab script, called ```astrobee_ros``` to collect and stream this data over UDP to a remote server. In our system configuration the Visualeyez tracker, VZSoft and Matlab script run on a PC called *memric*, and the data is streamed to another PC called *spheresgoat*.

Clearly, the server (*spheresgoat*) must also be able to interpret these data packets. For the sake of simplicity we have written a ROS node called ```visualeyez_bridge```, which listens for the UDP packets and forwards the data as ROS messages. This makes it much easier to interpret and plot the data.

Knowing the absolute position of the markers is only the first part of the problem. For the tracking system to be truly useful, we need to be able to transform the marker positions from the *visualeyez* reference frame to the *world* reference frame. The *grounding LEDs* and their world coordinates -- measured accurately by a theodolite -- are provided in the ```visualeyez.config```  LUA config file, along with the declaration of *target LED* groupings.

The excerpt below from the LUA config file instructs the ```visualeyez_server``` that LEDS 1-4 on TCM1 are rigidly attached to the environment, and their world coordinates are known. One rigid body is declared with name `honey`, which is comprised of 24 LEDs connected to TCM2. The coordinate for each LED is an initial  estimate of its position in the world frame when the body and world frames are aligned (origins and alignment equal). The actual positions are solved for through a calibration procedure, which will be explained later.

    grounding = {
      { tcm=1, led=1, x=0.9140106314862838, y=-3.219100563007645, z=-0.6207210724524649 },
      { tcm=1, led=2, x=0.1835635587737848, y=-3.219100563007645, z=-0.2319006270806013 },
      { tcm=1, led=3, x=1.043904870248541,  y=-3.219100563007645, z= 0.6628505519526655 },
      { tcm=1, led=4, x=1.110810070079649,  y=-2.050167430479354, z=-0.2681018912672751 } 
    };

    targets = {
      {
        name = "honey",
        markers = {
          { tcm=2, led=1,  x=0.15282164,  y=0.1529461,  z=-0.12371832 },
          { tcm=2, led=2,  x=0.15282164,  y=0.1529461,  z=-0.10805668 },
          { tcm=2, led=3,  x=0.15282164,  y=0.1393825,  z=-0.117475 },
          { tcm=2, led=4,  x=0.15282164,  y=0.1529461,  z=0.10805668 },
          { tcm=2, led=5,  x=0.15282164,  y=0.1529461,  z=0.12371832 },
          { tcm=2, led=6,  x=0.15282164,  y=0.1393825,  z=0.117475 },
          { tcm=2, led=7,  x=0.15282164,  y=-0.1529461, z=-0.12371832 },
          { tcm=2, led=8,  x=0.15282164,  y=-0.1529461, z=-0.10805668 },
          { tcm=2, led=9,  x=0.15282164,  y=-0.1393825, z=-0.117475 },
          { tcm=2, led=10, x=0.15282164,  y=-0.1529461, z=0.10805668 },
          { tcm=2, led=11, x=0.15282164,  y=-0.1529461, z=0.12371832 },
          { tcm=2, led=12, x=0.15282164,  y=-0.1393825, z=0.117475 },
          { tcm=2, led=13, x=-0.15282164, y=0.1529461,  z=-0.12371832 },
          { tcm=2, led=14, x=-0.15282164, y=0.1529461,  z=-0.10805668 },
          { tcm=2, led=15, x=-0.15282164, y=0.1393825,  z=-0.117475 },
          { tcm=2, led=16, x=-0.15282164, y=0.1529461,  z=0.10805668 },
          { tcm=2, led=17, x=-0.15282164, y=0.1529461,  z=0.12371832 },
          { tcm=2, led=18, x=-0.15282164, y=0.1393825,  z=0.117475 },
          { tcm=2, led=19, x=-0.15282164, y=-0.1529461, z=-0.12371832 },
          { tcm=2, led=20, x=-0.15282164, y=-0.1529461, z=-0.10805668 },
          { tcm=2, led=21, x=-0.15282164, y=-0.1393825, z=-0.117475 },
          { tcm=2, led=22, x=-0.15282164, y=-0.1529461, z=0.10805668 },
          { tcm=2, led=23, x=-0.15282164, y=-0.1529461, z=0.12371832 },
          { tcm=2, led=24, x=-0.15282164, y=-0.1393825, z=0.117475 }
        }
      }
    }

Once the calibration procedure has been carried out the results are saved to a file called ```calibration.bin```. This file is simply a serialized ROS message containing the best estimate of the world position of each of the targets LEDs when the target's body frame and the world frame are aligned.

## Operating guidelines

### Configuring memric

Firstly, log onto *memric* and make sure you have copied the contents of the the ```./tools/visualeyez/matlab``` folder in FSW to the Desktop. You will also need to obtain the the tarballs ```vz.tar.gz``` an ```tcp_udp_ip.tar.gz``` from ```volar.ndc.nasa.gov``` before running the script:

    /home/p-free-flyer/free-flyer/FSW/files/vz.tar.gz
    /home/p-free-flyer/free-flyer/FSW/files/tcp_udp_ip.tar.gz`

Secondly, turn on the Visualeyez tracker and that it is plugged into the serial port on *memric*. An orange light indicates that it is powered on.

Thirdly, power all visualeyez TCMs by plugging in the white 14.8v lithium ion batteries and checking the small black switch is in the on position. A single green LED indicates that the TCM is operating correctly. Always make sure you unplug the batteries when you are finished, or when the TCM buzzers sound. This is because the TCMs continue to drain the battery even if their switch is turned off. If the battery falls below a 8v its cells become unstable.

Now, run the Visualeyez VZSoft 2 software: ```C:\Program Files (x86)\Phoneix Technologies\VZ Soft\Vzsoftv2.exe```

When the GUI launches it should look something like the screen shot below. The first thing you'll want to is enable all TCMs and LEDs specified in your LUA file. In the example configuration given in the previous section you'll want TCM1 LED1-4 and TCM2 LED1-24 enabled.

![alt text](../images/tools/vz_leds.png "")

Verify that your TCMs are set to wireless (tetherless) operation:

![alt text](../images/tools/vz_tcm.png "")

Then, verify that you are listening for data ```COM8```.

![alt text](../images/tools/vz_source.png "")

Then, check that your operating options match those given below:

![alt text](../images/tools/vz_operating.png "")

Then, check that your sample timings are correct. This is probably the trickiest part of setting up the system to track correctly. One important thing to note before we proceed is that the LEDs can be damaged if you configure their sample period below 2.5ms.

![alt text](../images/tools/vz_sampling.png "")

Check that your exposure settings match those given below:

![alt text](../images/tools/vz_exposure.png "")

Check that your capture process settings match those given below:

![alt text](../images/tools/vz_capture.png "")

Finally, check that your stream settings match those given below:

![alt text](../images/tools/vz_stream.png "")

You are now ready to track! If you click the green circular button at the bottom of the GUI you should start to see red dots appear on the two sets of axes. Note that you can use ctrl plus the mouse button to zoom.

![alt text](../images/tools/vz_track.png "")

The final step it to launch Matlab R2014b 32-bit and run the astrobee_ros script in the folder you copied to **memric**.

![alt text](../images/tools/matlab.png "")

### Configuring spheresgoat

Follow the instruction to checkout, build and source flight software on *spheresgoat*. Then launch the ```visualeyez_bridge``` and ```visualeyez_server``` on spheresgoat using the following command:

    roslaunch visualeyez visualeyez.launch rviz:=true

The system starts in tracking mode by default. A snapshot of the tracking test is shown below The green dots represent static grounding LED positions in the world frame. The red dots are the target LEDs, which move as a rigid group. The set of axes in the center of the red LEDs shows the pose of the body frame with respect to the world frame. If calibration has not been carried out, you will notice that the estimate jumps frantically.

![alt text](../images/tools/tracking.png "The tracking GUI")

### Running the visualeyez ROS tools on another machine (optional)

The NASA firewall prevents **memric** UDP packets from being forwarded to any laptop on the wireless. We'll therefore have to setup an SSH tunnel over which to forward the UDP packets. First, you'll need to set up your laptop to always use **m** as a proxy for **spheresgoat**. You should have this in your ***~/.ssh/config*** directory:

	Host m
	Hostname m.ndc.nasa.gov
	 
	Host spheresgoat
	ProxyCommand ssh -q m nc spheresgoat.ndc.nasa.gov 22

Then, in one terminal set up an SSH tunnel to forward TCP packets to and from port 11000 on spheresgoat to port 11000 on your local machine. This command also starts ***socat** to receive UDP packets on **spheresgoat**  port 9090 and forward them to TCP port 11000. Note that this command must be started BEFORE you launch ROS otherwise you will have the connection refused.

	ssh -L 11000:spheresgoat:11000 spheresgoat "socat -u udp-recvfrom:9090 tcp-listen:11000,fork"

Then, in another terminal window on your local machine type:

	socat -u tcp:localhost:11000,fork udp-sendto:localhost:9090

This will pull TCP packets off the SSH tunnel and repackage them as UDP packets. If visualeyez is running, you should see lots of binary output when you type this command on your local machine:

	nc -ul 9090

If you are having issues with ports I find it extremely useful to be able to to check which ports are currently active:

	netstat -plun
	netstat -pltn

### Calibration procedure

The calibration procedure makes two key assumptions:

1. The center of rotation (gimbal origin) is the same as the body frame origin.
2. The first measurement taken corresponds to a roll, pitch and yaw of zero.

You will first need to follow the instructions in the previous two sections to start pushing data from *memric* and consuming it on *spheresgoat*.

It is critical that you now ensure the freeflyer has exactly zero roll, zero pitch and zero yaw before starting the calibration procedure. You can piggy-back of the gantry zeroing procedure, or perform a manual adjustment.

Once the freeflyer is aligned you can use the ```visualeyez_tool``` to switch ```visualeyez_server``` to calibration mode. 

    rosrun visualeyez visualeyez_tool -n honey -w calibration.bin -s 100

This command instructs the system to calibrate the markers for target "honey" and write the results to ```calibration.bin```. The `-s 100` argument instructs the calibration algorithm to run only when at least 100 sample measurements are received for each marker. If things go well you should see the following output on the command line.

    Recording:
    Calibrating...
    0065 0000 0119 0040 0114 0066 0312 0161 0114 0180 0225 0191 0197 0172 0240 0145 0173 0355 0394 0195 0164

This output tells us that we have seen LED1 65 times, LED2 zero times, etc. It will keep updating dynamically with time. Your GUI will switch to blue markers to show they are measurements. Note that only the subset of markers currently in view will be plotted on the GUI.

![alt text](../images/tools/calibration.png "The calibration GUI")

At this point you will now need to rotate the gimbal slowly around all three degrees of freedom, so that each LED comes in view for at least 100 samples. When this threshold is met, you will see the following output.

    Saving...
    Done!

At this point the system will automatically switch back to tracking mode, using the reference marker positions solved for in the calibration procedure. If all went well the red dots should move smoothly in response to orientation changes.

## Performance evaluation

### Experimental design

As with all localization systems, in order to measure the performance of the Visualeyez tracking system one must compare it against a "ground truth" produced by another tracking system with a stated accuracy at least one order of magnitude higher than the predicted accuracy of the system under test.

To measure the performance of the Visualeyez system we will use a Leica TotalStation TPS1200 theodolite to track a 360 degree prism mounted rigidly on the underside of the freeflyer, while in motion. This robotic theodolite is able to track the center of the prism at 8Hz with at most 3mm error.

Refer to the image below. The prism is mounted in the area underneath the freeflyer in place of a payload. A 25th LED is placed underneath the prism by a known distance *h*. This enables us to estimate the body-frame position of the prism center by simply subtracting *h* from the LED25 calibrated position. If we transform this coordinate to the world frame using the TF2 transform, we can compare it directly to the output data from the total station.

![alt text](../images/tools/gantry.png "The gantry")

The first step is to place the theodolite at a fixed position within view of all grounding markers, but sufficiently far from the freeflyer to avoid sharp freeflyer accelerations from affecting its ability to track the prism. The markers are then surveyed, and used jointly to set the grounding coordinates, but also to build the localization map (see \ref localization).

The Visualeyez LEDs are then fixed to the grounding targets, and calibration is performed to learn the reference positions of the target markers (see the previous section). Once this is complete we can use the visualeyez server to track the pose of the freeflyer in the world frame.

The theodolite is then configured to autonomously track and record the position of the prism at 8Hz as the freeflyer is moved by the gantry. We simultaneously record the TF2 transform produced by our tracking system. 

After the experiment completes we use the recorded TF2 transforms to estimate the trajectory of the prism, which is simply POS(LED25) - (0,0,h) transformed from the body frame to the world frame using the TF2 transforms. We use cross-correlation to determine the time offset between the theodolite (ground truth) and the Visualeyez data (system under test), and compare error statistics over the Leica time indices.
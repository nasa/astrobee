\page picoflexx PicoFlexx ROS Driver

The PMD PicoFlexx camera is a time of flight sensor that returns depth images, rather than RGB images. The sensor also comes factory calibrated with camera intrinsics and distortion parameters burned into its ROM. Using this information it is possible to project the depth measurements into a point cloud, which is represented in the optical axis. In order to project this into another frame (ISS, body) camera extrinsics are required.

The objective of this page is to document an experiment to test the amount of computational resources (CPU load) required to run the PicoFlexx on our target architecture. For the sake of repeatability, this page provides description of the hardware and software setup, and includes attachments that may be of use to the reader.

# Experimental setup

## Hardware

The hardware setup is pictured below. It is essentially an Inforce IFC6501 system on module (SOM) plugged into a development board. This board is connected by a USB 2.0 hub to two PMD PicoFlexx sensors, and through Ethernet to a host PC (Ubuntu 16.04 64bit VM). The host PC acts as a DHCP server, NAT router, DNS forwarder and debugger.

![alt text](../images/hw/pico-setup.jpg "The experimental setup")

The SOM is fitted with a cooling solution (heatsink and fan) that roughly mimics what is on the actual platform. Thermal testing has shown that by default all cores are run at maximum speed and under high load the CPU frequency switches three of the four cores offline when a thermal barrier is reached. For this reason the following script is run on boot to lower the maximum operating frequency of all four cores to 1.96GHz (they may be throttled lower than this in response to demand). 

	#!/bin/bash

	freq='2.0GHz'

	sys_base=/sys/devices/system/cpu

	for c in 0 1 2 3; do
	  online=$(cat ${sys_base}/cpu${c}/online)
	  if [[ "$online" -eq "1" ]]; then
	    echo "setting cpu frequency for cpu" ${c}
	    cpufreq-set -c ${c} -u ${freq}
	  else
	    echo "cpu ${c} is off, skipping"
	  fi
	done


## Software

### USB patch for Inforce 3.10 kernel

The default kernel that is shipped with the SOM has a bug in the USB core drivers that prevents the PicoFlexx from being woken up after suspending. Up until now the solution had been to plug a USB 1.1 device into the hub to prevent autosuspend from occurring completely. An alternate software-only solution is to set kernel parameter ''usbcore.autosuspend=-1''. However, in order for this to propagate to the hub, a kernel patch is required. 

The kernel patch can be found here: [0060-usb2514b-hub-fix.patch](https://babelfish.arc.nasa.gov/trac/freeflyer/attachment/wiki/pmd_picoflexx/0060-usb2514b-hub-fix.patch). It is included in the kernel build scripts, along with other patches, in the ''./scripts/inforce'' folder of the flight software repository. If you want to build the kernel from scratch, you should use these scripts, otherwise just download a pre-built kernel image: [boot.img](https://babelfish.arc.nasa.gov/trac/freeflyer/attachment/wiki/ifc6540/kernel_ifc6540_astrobee_mlp_ftdi_overlayfs_usbfix.img). Note this kernel image also includes overlayfs and FTDI serial drivers.

### Custom PicoFlexx SDK compilation

Although PMD now distributes an 32 bit armhf linux binary SDK, there are currently five major issues with it:

* It is a black box, so we can't see what it's doing or debug it.
* It depends on libuvc, which is not packaged with the SDK.
* It is not tuned for our architecture.
* It is compiled with gcc-4.9, and is not compatible with gcc-4.8.
* The default cmake package library path doesn't add the correct directories.

The PMD PicoFlexx SDK (called ''libroyale'') is covered by an NDA, and so the source code cannot be uploaded to this wiki. The code itself depends on a ''lispectre'' static library, which implements the time of flight algorithms. We have cross-compiled the SDK for the freeflyer, which is available here: [libroyale-2.3.0.0-LINUX-arm-32Bit-FFSW.zip](https://babelfish.arc.nasa.gov/trac/freeflyer/attachment/wiki/pmd_picoflexx/libroyale-2.3.0.0-LINUX-arm-32Bit-FFSW.zip). Although we have stripped debug symbols from the binary products, we recommend not distributing this zip file outside of the FreeFlyer project until written approval from PMD is received.

If you wish to compile the PicoFlexx SDK from source, you will need to get hold of the `spectre-releases-v2.6.5.zip` and `project-royale-extern-v2.3.0.zip` source tarballs. You will need to put these zipfiles along with the [0001-spectre.patch](https://babelfish.arc.nasa.gov/trac/freeflyer/attachment/wiki/pmd_picoflexx/0001-spectre.patch) and [0002-royale.patch](https://babelfish.arc.nasa.gov/trac/freeflyer/attachment/wiki/pmd_picoflexx/0002-royale.patch) patches, and then run the following script. This script requires that you have a valid freeflyer cross-compile toolchain, ie. arm-linux-gnueabihf-gcc-4.8 and ~/arm_trusty_chroot.

	#!/bin/bash

	# Install dependencies
	sudo apt-get install libusb-dev cppcheck valgrind -y

	# Unzip code
	unzip spectre-releases-v2.6.5.zip -d spectre
	unzip project-royale-extern-v2.3.0.zip -d royale

	# Compile spectre
	pushd spectre
	patch -p1 < ../0001-spectre.patch
	mkdir build
	pushd build
	cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain/arm_linux.cmake -DARM_COMPILER_TOOLS=/this/dir/doesnt/matter -DSPECTRE_SYSTEM_TARGET=arm -DBUILD_SHARED_LIBS=OFF -DSPECTRE_BUILD_SHARED=false -DCMAKE_INSTALL_PREFIX=install -DCMAKE_BUILD_TYPE=Release ..
	make -j6 install
	popd
	popd

	# Compile royale
	pushd royale
	patch -p1 < ../0002-royale.patch
	mkdir build
	pushd build
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/arm_linux.cmake -DROOTFS=~/arm_trusty_chroot -DPROCESSING_SPECTRE_HEADER_DIR=$PWD/../../spectre/build/install/inc -DPROCESSING_SPECTRE_SUBFOLDER=$PWD/../../spectre/build/install/lib -DCMAKE_BUILD_TYPE=Release ..
	make -j6 install
	mv install libroyale-2.3.0.0-LINUX-arm-32Bit-FFSW
	zip -ry ../../libroyale-2.3.0.0-LINUX-arm-32Bit-FFSW.zip libroyale-2.3.0.0-LINUX-arm-32Bit-FFSW
	popd
	popd

If all runs successfully, you should have a ``libroyale-2.3.0.0-LINUX-arm-32Bit-FFSW.zip`` product in the base folder, which is a binary SDK for the PicoFlexx camera custom-built for the IFC6501 platform.

# Experiments

## Performance : stock versus custom SDK

An experiment was conducted to verify that our custom compiled SDK has a similar performance to the stock SDK when run on the IFC6501. The way I chose to do this was to run [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) on a 16.04 minimal Ubuntu root file system, and access the camera through the [pico_flexx_driver](https://github.com/code-iai/pico_flexx_driver). In an effort to separate out the effect of ROS messaging overhead from libroyale computation I wrote a small nodelet called [pico_listener](https://babelfish.arc.nasa.gov/trac/freeflyer/attachment/wiki/pmd_picoflexx/pico_listener.tar.gz) that subscribes to a point cloud published by the driver from within the same nodelet manager. 

On the target (terminal 1) - to start the camera(s) :

	ROS_IP=10.42.0.95 roslaunch pico_listener pico_listener.launch          # Case: single camera
	ROS_IP=10.42.0.95 roslaunch pico_listener pico_listener_pair.launch     # Case: dual cameras


On the target (terminal 2) - to record load :

	sar 1 30 > data_log.txt

On the host - to configure the camera(s) :

	ROS_MASTER_URI=http://10.42.0.95:11311 ROS_IP=10.42.0.1 rosrun rqt_reconfigure rqt_reconfigure

I recorded load measurements at 1Hz for 30 seconds, and the results are summarized in the diagram below. Results are included for both the single (left column) and dual (right column) camera setups, and for both the stock (top row) and custom-built (bottom row). The x-axis on each graph shows the sample frequency of the camera(s) and the y-axis shows the load as a percentage of *all* CPUs. Since we have four CPUs, a load of 25% is equivalent to saturating one of the four CPUs. Similarly, a load of 50% is equivalent to saturating two CPUs.

There are a couple of interesting results from this experiment:

* The custom SDK performance matches the stock SDK performance well, given the confidence intervals.
* At the lowest setting (5Hz) running each camera costs approximately 12% of system resources. In other words, two cameras will saturate one of the four cores available.
* Oddly enough, running at 15Hz or 35Hz consumes similar resources, but 25% consumes substantially more. Whether this is because they are closer to perfect powers of two is yet to be determined.

It is also important to note that, although it is not shown in the results, the output of the pico_flexx_driver showed the point cloud callback rate to be within 1% of the target rate. For this reason, frame dropping cannot be used as a justification for the plateau in load we see from 15Hz onwards.

![alt text](../images/hw/pico-results.jpg "Performance results")

The raw data for this experiment, as well as the octave code used to generate the plots, is included in [matlab-results.tar.gz](https://babelfish.arc.nasa.gov/trac/freeflyer/attachment/wiki/pmd_picoflexx/matlab-results.tar.gz).

## Performance : pico_flexx_driver vs pico_driver

The PicoFlexx SDK provides a C++ interface for querying point clouds from the camera. What remains to be done is to extract these point clouds, convert them to an appropriate ROS message type and publish them on the messaging backbone. The ''de faco'' method for doing this is through the [pico_flexx_driver](https://github.com/code-iai/pico_flexx_driver). However, this driver performs extra work -- it allows for dynamic reconfiguration of the camera and also publishes the depth images.

Here we compare the performance of the pico_flexx_driver to that of a light-weight driver written for the FreeFlyer flight software stack. For both drivers we used the same SDK (our custom build) and the ROS Kinetic test environment described in the previous section. We limit our analysis to just the 5Hz mode, because this is the only feasible configuration -- all other moved are too resource-intensive for our application. The results are shown below.

![alt text](../images/hw/pico-drivers.jpg "Comparison to third party ROS drivers")

Our results show that we are likely to gain a 1% improvement in resource usage by running our pico_driver rather than the pico_flexx_driver. However, the pico_driver does need to be updated to distinguish between multiple cameras; it currently just picks the first one that it finds on the bus.

## Performance : optimizing the pico_driver

The current version of pico_driver performed an iterative copy of all points from the royale SDK to a point cloud, and then a message conversion from the PointCloud to a PointCloud2 data type. I added the improvements mentioned in the section below, and optimized the code to use a memcpy rather than an iterative copy, by exploiting the fact that the PointCloud2 data type accommodates complex data structures. This yielded a small improvement in speed for low rates, but a massive improvement for higher rates. The results are shown in the graph below:

![alt text](../images/hw/pico-optimization.jpg "Optimizing the ROS drivers")

# Setup on a laptop

In order to test the PicoFlexx on a laptop, several configuration steps are necessary. The cameras.config file located in ../freeflyer/astrobee/config/ must be changed with the correct camera uuid:
	       picoflexx = {
		  api_key = "",
		  devices = {
		    {
		      name = "perch_cam",               -- frame
		      topic = "perch",                  -- frame
LINE TO CHANGE -----> device = "0005-4804-0050-1421",    -- camera uuid ("" : automatic)
		      exposure = 0,                     -- exposure time (0: automatic)
		      mode = "MODE_9_5FPS_2000",        -- use case
		      required = true                   -- is camera required
		    },{
		      name = "haz_cam",                 -- frame
		      topic = "haz",                    -- frame
		      device = robot_haz_cam_device,    -- camera uuid ("" : automatic)
		      exposure = 0,                     -- exposure time (0: automatic)
		      mode = "MODE_9_5FPS_2000",        -- use case
		      required = false                   -- is camera required
		    },{
		      name = "test_cam",                -- special name
		      topic = "test",                   -- frame
		      device = "0005-4805-0050-1520",   -- camera uuid ("" : automatic)
		      exposure = 0,                     -- exposure time (0: automatic)
		      mode = "MODE_5_45FPS_500",        -- use case
		      required = false                  -- is camera required
		    }
		  }
		}

To find the camera uuid, run the following executable once the camera is connected to your laptop:
		 ../freeflyer_build/native/devel/lib/pico_driver/pico_tool

If the exectuable returns the message "Could not find any cameras", make sure you give sufficient permissions to the user of the hardware by typing the following command:
			sudo chmod 666 /PATH/TO/THE/CAMERA

To find the camera, I needed to run the command lsusb which returned:
			Bus 001 Device 024: ID 1c28:c012  

Which means that, in my case, the /PATH/TO/THE/CAMERA is /dev/bus/usb/001/024

Once the camera is found via the pico_tool executable, you can start using the camera. An example of how to see some imagery can be to run the following commands:
	roslaunch astrobee granite.launch mlp:=local llp:=disabled nodes:=pico_driver,framestore

And, in another terminal:	rosrun rviz rviz

Finally, click the "Add" button in the bottom left corner of the display panel, select "By Topic", and choose /hw/depth_perch/depth_image/Image


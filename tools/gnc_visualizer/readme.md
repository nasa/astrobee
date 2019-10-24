<!--\defgroup gncvisualizer GNC Visualizer
\ingroup tools-->

# GNC Visualizer

This tool visualizes the GNC status of the robot. For using the tool,
see the menu bar when the tool is open. There are many useful hotkeys,
indicated in the menu bar.

You may use one of the following two communication methods:

## ROS Mode

This is the default communication method. Use this if you are running on simulation
or inside the robot local network. It requires rospy dependencies.

You may use the `--comm` argument to set the `ros` value. Example:

```shell
    python ./scripts/visualizer.py --comm ros
```

You may also add `--gantry`, `--granite`, `--bag`, and `--sim` to start the
appropriate roslaunch file as well. Another argument, `--plan`, lets you
specify a plan file that can be started by pressing `p`. Please be advised,
these arguments only work in ROS mode.

## DDS Mode

This communication method allows you to read data from DDS topics. Use this
option if you want to get telemetry from a DDS enabled network (e.g. Astrobee
on orbit, space-like ground networks, dds_ros_bridge on simulation). Note
that many menu options are disabled when using this communication method.

You may use the `--comm` argument to set the `dds` value. Example:

```shell
    python ./scripts/visualizer.py --comm dds
```

This action will read the file located in `./scripts/communications/config.ini`.
It contains default values to run on simulation with DDS enabled. You may
edit this file in order to add peers or change the participant name.

Instead of modifyng the config file, you may set these values by adding `--robot_name`,
and `--use_ip`. Example:

```shell
    python ./scripts/visualizer.py --comm dds --use_ip 10.42.0.37 --robot_name Bumble
```

Please be advised this tool will not read data from DDS topics until you manually
set `ekf`, `gnc`, and `pmc` states to a value higher than 0 using the Ground Data
System (Advanced Tab).

Finally, **make sure your firewall is not blocking DDS communication**.

## Dependencies

This tool makes use of standard python modules pre-installed in Ubuntu.

If you are planning to use this tool along with the Astrobee Robot Software,
please follow the installation instructions in order to make sure you have the
correct dependencies installed.

On the other hand, if you want to use this software as a standalone tool
**in DDS mode**, you may copy the whole `gnc_visualizer` folder to the
destination computer and install missing dependencies. Depending on your
system, you may need to install the following:

* pyqtgraph
* PyQt4 and/or PyQt5
* numpy
* rticonnextdds_connector (required for DDS mode)

Any other usage is not recommended.

### Installing PIP

Install pip using `apt` or `yum`. Otherwise, try the following:

(Alternative)
```shell
    wget https://bootstrap.pypa.io/get-pip.py
    python get-pip.py
    python -m pip install --upgrade pip setuptools wheel
```

### Installing the RTI connector (DDS Only)

* Note: RTI Connext DDS Connector is under the RTI license.

Install the RTI software

```shell
    # Use pip2 instead if needed
    pip install rticonnextdds_connector
```

### Installing QT in standalone mode

* Note: If you follow the Astrobee install (Ubuntu only), this should not be
needed.

```shell
    # Use pip2 instead if needed
    pip install pyqtgraph

    # CentOS 7
    yum -y install PyQt4

    # Ubuntu 16.04
    apt-cache search pyqt
    sudo apt-get install python-qt4
```

## Platform support

This software has been tested on:

* Ubuntu 16.04 (Python 2.7.12)
* Kubuntu 16.04 (Python 2.7.12)
* CentOS 7 (Python 2.7.5)

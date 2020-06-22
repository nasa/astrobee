\page gncvisualizer  GNC Visualizer

This tool visualizes the GNC status of the robot. For using the tool,
see the menu bar when the tool is open. There are many useful hotkeys,
indicated in the menu bar.

You may use one of the following two communication methods:

## ROS Mode

This is the default communication method. Use this if you are running on
simulation or inside the robot local network. **It requires rospy dependencies**.

You may use the `--comm` argument to set the `ros` value. Example:

```shell
    python ./scripts/visualizer.py --comm ros
```

You may also add `--gantry`, `--granite`, `--bag`, and `--sim` to start the
appropriate roslaunch file as well. Another argument, `--plan`, lets you
specify a plan file that can be started by pressing `p`. Please be advised,
**these arguments only work in ROS mode**.

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
edit this file in order to add peers, change the participant name, set the DDS
domain and/or add a public IP address for TReK communications.

Instead of modifying the configuration file, you may set these values by adding
`--robot_name`, `--use_ip`, `--domain` and/or `--public_ip`. When used, these
arguments will take precedence over the configuration file. If an argument
is invalid or not present the script will fall back to the configuration file.
Example:

```shell
    # Override initial_peers and participant name but keep domain and public IP
    # same as the configuration file
    python visualizer.py --comm dds --use_ip 10.42.0.37 --robot_name Bumble
```

Please be advised this tool will not read data from DDS topics until you manually
set `ekf`, `gnc`, and `pmc` states to a value higher than 0 using the Ground Data
System (Engineering Tab).

Additionally, **make sure your firewall is not blocking DDS communications**.

### On-orbit activities

For **on-orbit activities**, when running **inside of a TRek environment**,
you may use the `--public_ip` argument to state the `ON_BOARD_PROXY_IP`,
`--use_ip` for the `GROUND_PROXY_IP` and `--robot_name` to set the agent.
Example:

```shell
    # Override participant name, initial peers and public IP, but keep the
    # default domain from configuration file
    python visualizer.py --comm dds --robot_name Bumble --use_ip 0.0.0.0 \
      --public_ip 0.0.0.0
```

_**Note: Setting the `--public_ip` argument outside of a TReK environment may
prevent the program from getting telemetry.**_

For **on-orbit activities**, when running outside of a TRek environment but
**using the DDS2 relay server**, you may set the `--use_ip` argument to the
DDS2 IP address and `--domain` to one of the DDS_GROUND_DOMAIN IDs. Example:

```shell
    # Override participant name, initial peers and domain, but keep the default
    # public_ip (should be empty) from the configuration file
    python visualizer.py --comm dds --robot_name Bumble --use_ip 0.0.0.0 \
      --domain 29
```

## Dependencies

This tool makes use of standard python modules present in most Linux and Windows
implementations.

### If used along with the Astrobee Robot Software

If you followed the installation instructions for this repository, you should
already have all the needed dependencies to run this tool **in ROS mode**.

In order to run this tool in **DDS mode** you need an additional dependency:

* rticonnextdds_connector

### If using as a standalone tool

You may use this program as a standalone tool **only in DDS mode**. You may copy
the whole `gnc_visualizer` folder to the destination computer and install
missing dependencies.

Depending on your system, you may need to install the following packages:

* pyqtgraph
* PyQt4 and/or PyQt5
* numpy
* rticonnextdds_connector

Any other usage is not recommended.

## Installing dependencies

Here you will find suggestions on how to install some dependencies needed
for this tool usage.

### Installing Python

Make sure **Python 2.7.x** is installed on your computer. You may get it from
your package manager or from [the official website](https://www.python.org/downloads/)

If you followed the Astrobee install, Python should be already installed

### Installing PIP

You may install pip using `apt` or `yum`. Otherwise, try the following:

(Alternative)
```shell
    wget https://bootstrap.pypa.io/get-pip.py # Or download it from a browser
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

* Note: If you followed the Astrobee install (Ubuntu only), this should not be
needed.

```shell
    # Use pip2 instead if needed
    pip install pyqtgraph

    ## Use one of the following depending on your platform

    # CentOS 7
    yum -y install PyQt4

    # Ubuntu 16.04
    apt-cache search pyqt
    sudo apt-get install python-qt4

    # Windows 10
    #
    # Download the wheel package
    # One possible location: https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyqt4
    # Make sure you download a cp27 version
    #
    # Or build it from source code: https://www.riverbankcomputing.com/software/pyqt/download
    #
    # Then install it (use pip2 if needed):
    pip install PyQt4-[...].whl
```

## Platform support

This software has been tested on:

* Ubuntu 16.04 (Python 2.7.12)
* Kubuntu 16.04 (Python 2.7.12)
* CentOS 7 (Python 2.7.5)
* Windows 10 (Python 2.7.17)

\subpage gnc_visualizer_dds
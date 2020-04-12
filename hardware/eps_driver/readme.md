\defgroup eps_driver EPS Driver
\ingroup hw

The Electrical Power Subsystem (EPS) is responsible for controlling power to the
various components of the avionics stack. The EPS is controller from the
low-level processor (LLP) over i2c, by default on /dev/i2c1 with address 0x40.

The eps_driver is a proxy library that converts human-readable function calls
into i2c read and write transactions to and from the EPS. The eps_driver_node
uses this proxy library to extract telemetry data, which it broadcasts to ROS.
The eps_driver_node also offers a few ROS services that enable remote users to
call a subset of commands on the EPS.

The eps_driver_tool is a command-line interface to the eps_driver. It allows the
user (or scripts) to do the following:

* Turn on an off power to various avionics components
* Configure status LEDs to be on, off or flash
* View battery information
* View charger information
* View housekeeping information
* View and clear fault information
* View docking and power states
* View the build string, software version and serial number
* Ring the buzzer
* Clear a terminate state
* Reboot the EPS
* Enter the EPS bootloader

The tool itself has build-in documentation. To see it, run the tool without specifying any flags:

    eps_driver_tool

Most commands take the following form, where `<system>` is one of -led, -hk,
-power, -state, -fault, -battery, -string or -charge. The flag -list lists all
possible indexes and values for the system. The flag -set sets a value for one
or more indexes shown as `[ <index> ... ]` below.

    eps_driver_tool <system> [ -list | -get | -set <val> ] [ <index> ... ]

Every system supports -list, however only some may support -get or -set. Certain
commands may also have other flags. To see the documentation for a specific
section, simply add its flag. For example, to see the -led documentation type
the following:

    eps_driver_tool -led

You might now know the index or values that you can use. So, take for example
the power subsystem. Let's say you want to get a lit of all possible power
channels that you can turn on and off:

    eps_driver_tool -power -list

You should see something like this:

    Indexes
     - Low-level processor (llp)
     - Mid-level processor (mlp)
     - High-level processor (hlp)
     - Universal serial bus (usb)
     - Auxiliary (aux)
     - Ethernet (eth)
     - Fan (fan)
     - Speaker (spk)
     - Payload 1 - top-front (pay1)
     - Payload 2 - bottom-front (pay2)
     - Payload 3 - top-aft (pay3)
     - Payload 4 - bottom-aft (pay4)
     - PMC 1 - right / port (pmc1)
     - PMC 2 - left / stbd (pmc2)
     - LED Status 1A (s1a)
     - LED Status 1B (s1b)
     - LED Status 1C (s1c)
     - LED Status 2A (s2a)
     - LED Status 2B (s2b)
     - LED Status 2C (s2c)
     - LED Microphone (mic)
     - LED Camera (cam)
     - LED Streaming (str)
    Values
     - Turn off (disable, off, false, 0)
     - Turn on (enable, on, true, 1)

The format of each line above is NAME (index). You might notice that any index
or value might have multiple keywords. This is just for convenience, and it
means the following two commands are analogous:

    eps_driver_tool -power -set on pmc1 pmc2
    eps_driver_tool -power -set enable pmc1 pmc2

# Usage examples

Flash the camera LED ("cam") at a medium speed of 1Hz ("medium")

    eps_driver_tool -led -set medium cam

To turn on all LEDs ("on") just

    eps_driver_tool -led -set on

Get the EPS software version ("sw") and build time ("build") strings

    eps_driver_tool -string -get sw build

Ring the buzzer at 1500 Hz for two seconds

    eps_driver_tool -buzz -freq 1500 2

Toggle power to both propulsion systems

    eps_driver_tool -power -set off pmc1 pmc2
    eps_driver_tool -power -set on  pmc1 pmc2

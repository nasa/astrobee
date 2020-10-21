\page adding_a_fault.md Adding a fault to the Astrobee FSW

# Introduction

The fault detection system on Astrobee is distributed. This means that each node is responsible for detecting, asserting, and clearing their own faults. Each node can respond to a fault locally but the system monitor is in charge of issuing the system wide fault response. The system monitor is also responsible for detecting and triggering faults when nodes in the system freeze or die. A fault response can be any command that Astrobee accepts. For faults that are only a warning, the fault response should be the no operation or noOp command. Also, a fault can be blocking. This means that incoming mobility commands will be rejected when a blocking command is triggered. Once the blocking fault is cleared, mobility commands will be accepted again. In order to add a fault to the system, you must add it to the fault config files and the node responsible for detecting it. Please read the following sections on how to do this. 

# Add Fault to the FMECA Spreadsheet

The master spreadsheet of faults is called IRG-FF042-01-Astrobee-FMECA.xlsx and it is located in the doc folder. Please open the spreadsheet in your favorite spreadsheet editor. Faults are sorted by type. For example, heartbeat faults are grouped together, initialization faults are grouped together, imu faults are grouped together, etc. Please find the group your fault corresponds to and add your fault to the end of the group. If your fault doesn't fit into any of the groups, please add your fault to the end of the table. Please try to fill out as many columns as possible. The columns that you must fill out if you want the fault to show up in the software are Fault ID, Subsystem Name, Failure Mechanism, Warning, Blocking, Node Name, Response Command, and Key. The Command Arguments column is only required if the fault response command takes arguments and the Timeout and Misses columns are only required for heartbeat faults. If you have any questions on how to fill out the table, please contact Katie Hamilton. After you have added of all your faults, you will want to save the spreadsheet as normal and also save the spreadsheet as a csv file. You get a warning that the formatting cannot be saved in csv format. This is okay and you should be able to continue to save it as a csv file.

# Fault Config Files Generation

There is a script that will generate the fault config files by parsing the FMECA spreadsheet. This script assumes the steps in the previous section were completed and the FMECA spreadsheet was saved as a csv file in the doc folder. If it was not saved in the doc folder, please set the FMECA_CSV_PATH environment variable to folder were it was saved. Then run the following line:
 
  # if needed, set the path to the FMECA csv file
  export FMECA_CSV_PATH=path/to/fmeca/csv/file
  ./scripts/build/genFaultTable.py

After running the script, please make sure your fault id doesn't show up in the output of faults not added to the fault table. If it does, please make sure you filled out all of the required columns correctly.

# Add to Node

The freeflyer nodelet base class offers functions to help assert and clear faults. The base class also reads in the nodelet's faults. If you have added a heartbeat fault, the base class and the system monitor will take care of asserting the fault for you. All other faults, will need to be asserted when the fault occurs and cleared when the fault goes away. To assert and clear a fault, you will need to use the correct fault key enumeration value. Please see shared/ff_util/include/ff_util/ff_faults.h for the list of fault keys. If you are not sure which fault belongs to your nodelet, please see astrobee/config/faults.config for the list of nodelets and their corresponding faults. Please contact Katie Hamilton if you have any questions about faults.

\page adding_a_command Adding a ground command to the Astrobee FSW

# Introduction

Adding a ground command to the flight software is a pretty involved task so
please make sure you have a full understanding of the system before you attempt
it. There are four parts to adding a command: adding the command to the command
spreedsheet, adding the command to the plan schema, generating the fsw and
Android files, and adding the command to the executive. Please make sure to
complete all four parts to ensure your command is fully intergrated into the
system.

# Update the Commands Spreadsheet

The command spreadsheet can be found in freeflyer_docs/FlightSoftware and is
named commands.xml. This spreadsheet keeps track of the ground commands the fsw
accepts, which commands can run while other commands are running, the operating
and mobility states the commands are accepted in, whether the command is accept
in a plan or not, and the status of the command in the fsw. Please make sure to
add your command to each of the relevant sheets. If you have questions, please
contact Katie Hamilton.

# Add Command to the Plan Schema

The plan schema can be found in astrobee/commands. Please add your command to
the freeFlyerPlanSchema.json file. Make sure to follow the json format for a
command.

# File Generation

When adding a command, some fsw and GDS files need to be updated. There are
scripts that will update everything for you. After adding your command to the
plan schema, please follow the next three subsections to generate the files.

## Setup

The scripts for auto-generating source code files from the command dictionary
depend on the `xgds_planner2` library, which can be installed as follows:

    sudo pip install git+https://github.com/trey0/xgds_planner2.git@just_xpjson

## Generate FSW files

To generate the fsw files and copy them to correct location, please run the
following line from the top level directory of astrobee:

    ./scripts/build/processCommandSchema.sh

After running this script, please make sure your changes compile. To do this,
navigate to the folder you build the fsw in, rebuild the cache, and then compile
the code. Don't forget to commit and push your changes. If you want your command
incorporated into GDS, please let whoever is in charge of GDS know that the
command constants idl has changed.

## Generate Android files

Make sure you have the Astrobee Android repo. If you don't, please see the
install instructions on how to get it. Navigate to the astrobee api scripts
folder in the astrobee Android repo (astrobee_api/scripts). These scripts assume
that the android repo is located in the fsw submodules folder. If it is located
outside of the fsw, please set the $SOURCE_PATH environment variable to the fsw
location. Also if you added a new enum command type, you will need to add an
include line to both the genBaseRobot and genBaseRobotImpl scripts. To generate
the android files, please run the following in the scripts folder:

    cd astrobee_api/scripts
    # if needed, set the source path
    export SOURCE_PATH=path/to/astrobee
    ./processCommandSchema.sh

The script will copy the generated files to the correct locations. Please make
sure that the astrobee_api builds with no errors. To do this, go to the top
level astrobee api directory and build the code:

    cd astrobee_api/
    ./gradlew build

Once everything compiles, make sure you commit and push your changes.


# Add to Executive

The last step is to add the command to the executive. Please make sure you have 
a full understanding of the executive and operating state classes before adding
the command. If you have questions, please contact Katie Hamilton.

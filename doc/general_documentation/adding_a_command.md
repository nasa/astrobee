\page adding_a_command.md Adding a ground command to the Astrobee FSW

# Introduction

Adding a ground command to the flight software is a pretty involved task so please make sure you have a full understanding of the system before you attempt it. There are four parts to adding a command: adding the command to the command spreedsheet, adding the command to the plan schema, generating the fsw and Android files, and adding the command to the executive. Please make sure to complete all four parts to ensure your command is fully intergrated into the system.

# Update the Commands Spreadsheet

The command spreadsheet can be found in freeflyer_docs/FlightSoftware and is named commands.xml. This spreadsheet keeps track of the ground commands the fsw accepts, which commands can run while other commands are running, the operating and mobility states the commands are accepted in, whether the command is accept in a plan or not, and the status of the command in the fsw. Please make sure to add your command to each of the relevant sheets. If you have questions, please contact Katie Hamilton.

# Add Command to the Plan Schema

The plan schema can be found in astrobee/commands. Please add your command to the freeFlyerPlanSchema.json file. Make sure to folloow the json format for a command.

# File Generation

When adding a command, some fsw and GDS files need to be updated. There are scripts that will update everything for you. After adding your command to the plan schema, please follow the next three subsections to generate the files.

## Setup

To use the scripts, you will need to checkout the xgds2 repo and install some python and geocam tools.

    cd astrobee/commands/
    git clone https://github.com/xgds/xgds_planner2.git
    sudo apt-get install python-pip python-iso8601
    sudo pip install git+https://github.com/geocam/geocamUtilWeb

In order to generate the Android files, the xpjson planner parser needs to be changed to keep the parent field for each command parameter. To do this, open the astrobee/commands/xgds_planner2/xgds_planner2/xpjson.py file in your preferred  text editor. Find the KEEP_PARAM_PARENT parameter (around line 114) and set it to true. Then save and close the file.

## Generate FSW files

To generate the fsw files and copy them to correct location, please run the following line from the top level directory of astrobee:

    ./scripts/build/processCommandSchema.sh

After running this script, please make sure your changes compile. To do this, navigate to the folder you build the fsw in, rebuild the cache, and then compile the code. Don't forget to commit and push your changes. If you want your command incorporated into GDS, please let whoever is in charge of GDS know that the command constants idl has changed.

## Generate Android files

Make sure you have the Astrobee Android repo checked out. If you don't, please see the install instructions on how to do this. Navigate to the astrobee api scripts folder in the astrobee Android repo (astrobee_api/scripts). These scripts assume that the fsw is located in $HOME/astrobee. If the fsw is located somewhere else, please set the $SOURCE_PATH environment variable to the fsw location. Also if you added a new enum command type, you will need to add an include line to both the genBaseRobot and genBaseRobotImpl scripts. To generate the android files, please run the following in the scripts folder:

    cd astrobee_api/scripts
    # if needed, set the source path
    export SOURCE_PATH=path/to/astrobee
    ./processCommandSchema.sh

The script will copy the generated files to the correct locations. Please make sure that the astrobee_api builds with no errors. To do this, go to the top level astrobee api directory and build the code:

    cd astrobee_api/
    ./gradlew build

Once everything compiles, make sure you commit and push your changes.

# Add to Executive

The last step is to add the command to the executive. Please make sure you have a full understanding of the executive and operating state classes before adding the command. If you have questions, please contact Katie Hamilton.

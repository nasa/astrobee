# Setting up a new robot

## Create a new "custom" image for the robot

Follow instructions in [Generating custom overlays](../submodules/platform/readme.md)

## Calibrate the sensors

Follow instructions in [calibrate readme](../scripts/calibrate/readme.md).

## Create a new robot config

Use one existing robot configuration (in `astrobee\config\robots`) as a template.

Adjust the following  parameters in the new config:
  - local IPs
  - Picoflexx cameras ID
  - Calibration parameters (obtained from the previous step)
  - Agent name

If the new robot will become the default robot on the granite table, its name can be configured to do so in the `astrobee\launch\granite.launch`.

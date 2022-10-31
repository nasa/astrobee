\page picoflexx_python Pico Flexx Python utilities

These Python utilities enable us to process logged
`ff_msgs/PicoflexxIntermediateData` messages on the
`/hw/depth_haz/extended` topic and split the data to regenerate both
point cloud messages normally logged on the `/hw/depth_haz/points` topic
and amplitude image messages logged on the
`/hw/depth_haz/extended/amplitude_int` topic. (Those are the
HazCam-specific topics; to use  PerchCam, specify `--cam=perch`.)

As a result, once these tools are fully validated, we will no longer
need to log the latter two topics, providing a major savings in data
volume. To quantify the impact, we studied the longest `delayed` bag
from the SoundSee-Data-3 Astrobee ISS activity, running an ISAAC-style
survey. Out of a total bag size of 20.9 GB, the top four messages by
data volume were NavCam images (8.6 GB), HazCam point clouds (6.3 GB),
HazCam extended messages (5.0 GB), and HazCam amplitude images (0.6
GB). By no longer logging point clouds and amplitude images, we could
save up to 6.9 GB (33%). These savings impact not just storage space on
the Astrobee MLP but also CPU load required to log the data, which
experience shows can be significant.

## System requirements and installation

So far, these tools were tested only under Ubuntu 18.04 and Python 2.7
on the host astrobeast.ndc.nasa.gov. This host already had the necessary
dependencies pre-installed by the admins in the ARC-TI Systems Group,
so no installation steps were needed, other than checking out the scripts
from the repository.

Some minor changes would probably be needed to support Python 3.x. If
the tools will be used on other hosts that aren't pre-configured, this
section should be expanded to document how to install the dependencies.

## Usage

Using these tools is a two-step process:

1. Prep once for each Pico Flexx serial number: Using a prior bag that
   contains point clouds, extract and save xyz coefficients that are
   required to recover point clouds from extended messages. Optionally,
   if the prior bag contains all three Pico message types, you can also
   use it to do a dry run of the split operation and check the quality
   of the reconstructed data.

2. Production: A single command performs the split operation, writing a
   new bag with the recovered point clouds and amplitude images.

This usage example shows how to do the prep:

    scripts=$HOME/astrobee/src/hardware/pico_driver/scripts
    # example bag archived at hivemind.ndc.nasa.gov:/home/data-processing/freeflyer/2022-01-03_100/robot_data/SN003/bags
    inbag=20220103_1429_soundsee_isaac_isaac_survey15mins.bag
    $scripts/pico_write_xyz_coeff.py $inbag bumble_haz_xyz_coeff.npy
    # optional: dry run and check data quality
    $scripts/pico_split_extended.py $inbag bumble_haz_xyz_coeff.npy haz_split.bag
    $scripts/pico_check_split_extended.py $inbag haz_split.bag

And this example excerpts the one step needed in production:

    $scripts/pico_split_extended.py $inbag bumble_xyz_coeff.npy haz_split.bag

## Getting valid xyz coefficients

A file of xyz coefficients is required to recover point clouds from
extended messages.

Using the pico_write_xyz_coeff.py script, the xyz coefficients are
effectively derived from the camera intrinsic parameters stored in the
camera firmware. These parameters are unique to each Pico Flexx serial
number and calibrated at the factory. By convention, when saving the
coefficients, the output file name should document which robot and which
camera (haz or perch) the coefficients are for. During reconstruction,
be careful to use the correct coefficients file for your robot and
camera.

The method we are using to recover the coefficients requires a bag of
sample data recorded from the sensor, and it can only produce
coefficients for a given sensor pixel if at least one frame of the
input bag has valid depth data for that pixel. This means that:

- The bag needs to be recorded with a scene that is suitable for the
  Pico Flexx (objects within the right distance range, objects not too
  dark or specular, etc.).

- If the bag is recorded during a normal Astrobee activity in the
  complicated ISS environment, you will effectively never see a static
  scene that has all valid pixels. Therefore, it's better to process a
  bag that contains lots of Pico Flexx frames with varying scenery, in
  order to have the best chance of filling in the occasional dropout
  pixels.

- We've seen in practice that even if the guidelines above are followed,
  there may be many pixels (~3%) near the corners of the image that
  never get any valid depth returns. This doesn't cause anything to
  crash and isn't a big deal *except* that if we use the resulting
  coefficients, then even if future Pico Flexx sensor frames have valid
  depth returns for those pixels, we won't be able to reconstruct the
  corresponding xyz points due to the missing coefficients. The relevant
  entries in the reconstructed xyz output will be set to the no-data
  value, as if there was not a valid depth return. Note that for dense
  3D mapping purposes, the pixels near the corners were already being
  ignored anyway, because they were considered suspect due to
  limitations of the camera calibration and other issues.

- In a controlled lab setting, the ideal bag to collect to maximize
  valid depth returns for calibrating the coefficients would probably
  have the camera staring at a blank white surface just a bit farther
  away than the minimum depth range of the camera (e.g., 20 cm).

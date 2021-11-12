\page calibration Calibration

# Package Overview
# Camera Target Based Intrinsics Calibration 
#TODO(rsoussan): Go over some common process to make target detections, run calibration, and view results!

# Tools

## create\_undistorted\_images 
Generates undistorted images from a set of distorted images and provided camera calibration parameters.
See 'rosrun calibration create\_undistorted\_images -h'
for further details and usage instructions.

## run\_camera\_target\_based\_intrinsics\_calibrator
Runs the intrinsics calibrator using a set of target detection files and initial estimates
for the camera intrinsics and distortion values.  Support various distortion models.
See 'rosrun calibration run\_camera\_target\_based\_intrinsics\_calibrator -h'
for further details and usage instructions.

# Scripts

## calibrate\_intrinsics\_and\_save\_results.py  
Runs camera intrinsic calibration using provided target detections and a config file with camera
parameters (including initial estimate for camera intrinsics and distortion values).
See 'rosrun calibration calibrate\_intrinsics\_and\_save\_results.py -h'
for further details and usage instructions.

## copy\_calibration\_params\_to\_config.py      
Helper script that copies calibration parameters from the output of the 
calibration pipeline and writes these to the camera config file.
See 'rosrun calibration copy\_calibration\_params\_to\_config.py -h'
for further details and usage instructions.

## get\_bags\_with\_topic.py    
Helper script that generates a list of bag files in a directory
with the provided topic. 
See 'rosrun calibration get\_bags\_with\_topic.py -h'
for further details and usage instructions.

## make\_error\_histograms.py
Generates a histogram of errors using the output errors from 
the calibration pipeline.
See 'rosrun calibration make\_error\_histograms.py -h'
for further details and usage instructions.

## save\_images\_with\_target\_detections.py
Generates target detection files for use with the calibration 
pipeline from a set of bagfiles containing images of target detections. 
See 'rosrun calibration save\_images\_with\_target\_detections.py -h'
for further details and usage instructions.

## view\_all\_detections.py
Generates an image containing all images space detections of a target
for a set of target detection files.
See 'rosrun calibration view\_all\_detections.py -h'
for further details and usage instructions.

\page calibration Calibration

# Package Overview
# Camera Target Based Intrinsics Calibration 
Running the camera target based intrinsics calibrator involves generating target detection files
from a set of bagfiles, optionally generating an initial estimate for the camera intrinsics and distortion values, 
the optimizing to find more accurate camera intrinsics parameters.
Several camera distortion models (fov, rad, radtan) are supported for calibration.
Various helper scripts are provided to assist generating the target images, running the calibrator, 
and viewing the results.
While error reprojection images and histograms are generated to assist with viewing the results of the calibration, 
please rely mostly on the output covariances from the intrinsics calibrator as a confidence metric for the solved
parameters.
Target detection images should ideally span the entire image and be viewed from various depths and angles to more
accurately calibrate the camera parameters.

## Example
rosrun calibration save\_images\_with\_target\_detections -d ~/bag\_files -o target\_detections -t ~/target.yaml
rosrun calibration view\_all\_detections.py -d target\_detections
feh detection\_image.jpg 
<p align="center">
<img src="./doc/images/detection_image.jpg" width="330">
</p>
This shows all of the target detections extracting from the selected bag files.  Ideally these span the entire image.
rosrun calibration calibrate\_intrinsics\_and\_save\_results.py target\_detections ~/astrobee/src/astrobee/config config/robots/bumble.config -u -p -i target\_detections -d fov
The calibrated resuls are saved in the calibrated\_params.txt file while more verbose output is saved to the calibration\_output.txtfile.  
Ensure that the covariances for each calibration parameter are small in the calibration\_output.txt file.  If some are larger, this indicates the calibration problem is not properly constrained.  The easiest solution to this is adding more target detection images to the calibration pipeline to cover more of the image as shown in the previously mentioned detection\_image.jpg if there are sparseregions in that image.  Additionally, if a higher parameter distortion model is used (i.e. rad or ratan), consider setting some calibration parameters to fixed by toggling calibrate\_parameter\_name in the camera\_target\_based\_intrinsics\_calibrator.config file.  Known degeneracies can occur when calibrating principal points at the same time as target poses, for example.
The output calibrated\_reprojection\_from\_all\_targets\_absolute\_image.png displays reprojection errors after the calibration procedure has completed.  
<p align="center">
<img src="./doc/images/calibrated_reprojection_from_all_targets_absolute_image.png" width="330">
</p>

This gives another indicator of calibration performance in addition to parameter covariances, although if target detections are not adequately provided the calibration can overfit to the target data, so this should only be used as an aid while primary focus shoud be on the output parameter covariances.
# todo: show how to use undistort images, show how to plot error histrogram, mention these can be enabled directly in the calibrate_intrinsics script!!!!
The new calibrated params are updated to the robot config file (in this case bumble.config).  Since -u is selected and -i is directed to the target\_detections directory, each image containing a target image is undistorted with the new calibration parameters.  Finally since -p is selected, a histogram of errors is saved for each reprojection error included in the calibration procedure.
Further options for calibration are located in the camera\_target\_based\_intrinsics\_calibrator.config file. 
## 
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

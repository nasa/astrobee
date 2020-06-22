\page opticalflow Optical Flow

This nodelet tracks optical flow features using the Lucas Kanade algorithm.
The features used are `Pretty Good Features to Track`, and more features are detected
when the number remaining is low. We try to maintain at least fifty features.

Note that we first reduce the resolution of the image, to speed up computation.

# Inputs

* `/hw/nav_cam`

# Outputs

* `/localization/optical_flow/features`
* `/localization/optical_flow/registration`

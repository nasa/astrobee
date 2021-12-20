\page visioncommon Vision Common

# Package Overview

### Distortion Models 
Field of view (fov), radial (rad), and radial tangential (radtan) distortion models are provided along with an identity distortion model.  These are use for example as template parameters for image projection functions in the camera utilities files.  The models are templated on the value type (i.e. double, jet) to allow their usage with Ceres-based optimization packages. 

### Feature Detection And Matching
Brisk, surf, and lucas kanade optical flow feature detector and matchers are provided and inherit the base FeatureDetectorAndMatcher class so they can be used polymorphically by other classes and functions.

### Utilities
The utilities files provide helper functions for the distortion models and templated implementations based on the distortion type of image space projections, ransac pnp, and a reprojection pose estimate function that utilities ransac pnp to find an initial pose estimate for a set of 3d and corresponding image points before performing nonliner optimization to refine this pose estimate.


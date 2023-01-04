\page factoradders Factor Adders

# Package Overview
The factor adders package provides a set of factor adders for various measurement types for use with a graph optimizer. See the GraphOptimizer package for more information on their usage.

## FactorAdders
### LocFactorAdder
The LocFactorAdder takes map-based image feature measurements and generates either LocProjectionFactors or LocPoseFactors, depeneding on if the LocProjectionFactors suffered cheirality errors or not.

### ProjectionFactorAdder
The ProjectionFactorAdder creates bundle-adjustment ProjectionFactors for tracked image features. (Not currently used).

### RotationFactorAdder
The RotationFactorAdder generates a relative rotation using tracked image features in two successive images.  (Not currently used).

### SmartProjectionCumulativeFactorAdder
The SmartProjectionCumulativeFactorAdder generates visual odometry smart factors using image feature tracks.  It contains options for the minimum disparity allowed for feature tracks, minimum separation in image space for added feature tracks, whether to use a rotation-only factor if created smart factors suffer from cheirality errors, and more. It can generate factors using maximally spaced measurements in time to include a longer history of measurements while including only a maximum number of total measurements or only include measurements from a given set with a set spacing (toggle these with the use\_allowed\_timestamps option).  

### StandstillFactorAdder
The StandstillFactorAdder creates a zero velocity prior and zero tranform between factor for successive CombinedNavState nodes when standstill is detected.  Standstill detection checks for a minimum average disparity for image feature tracks over time.

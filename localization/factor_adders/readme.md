\page factoradders Factor Adders

# Package Overview
The factor adders package provides a set of factor adders that add factors to a graph optimizer given a range of timestamps. 

## FactorAdder
Base class for adding factors given a timestamp range.

## MeasurementBasedFactorAdder
Extends the FactorAdder and stores measurements in a buffer before using these for factor creation.

## SingleMeasurementBasedFactorAdder
MeasurementBasedFactorAdder that adds factors using single measurements at a time for measurements in a provided time range. 

## LocFactorAdder
SingleMeasurementBasedFactorAdder that takes map-based image feature measurements and generates either LocProjectionFactors or LocPoseFactors. Optionally adds loc pose factors as a fallback for projection factors that suffer cheirality errors. Uses a pose node adder for required node creation.

### VOSmartProjectionFactorAdder
Generates visual odometry smart factors using image feature tracks.  Downsamples measurements as desired for smart factors. See gtsam::SmartProjectionPoseFactor for more information on smart factors. Uses a pose node adder for required node creation.

### StandstillFactorAdder
Creates a zero velocity prior and a zero tranform between factor for successive nodes when standstill is detected. Uses a pose and velocity node adder for required node creation. 

\page graphfactors Graph Factors

# Package Overview
The graph factors package provides a set of graph factors for use with a GTSAM graph optimizer. See the GraphOptimizer package for more information on their usage.

## Factors

### LocPoseFactor
The LocPoseFactor is simply a gtsam::PriorFactor\<gtsam::Pose3\> that enables the differention of a pose prior from a localization map-based image feature factor.

### LocProjectionFactor
The LocProjectionFactor is almost a direct copy of the gtsam::ProjectionFactor except it does not optimize for the 3D feature point location.

### PoseRotationFactor
The PoseRotationFactor constrains two gtsam::Pose3 nodes using their relative rotation.

### PointToHandrailEndpointFactor.h
The PointToHandrailEndpointFactor constrains a gtsam::Pose3 using a handrail endpoint detection in the sensor frame compared with the closest handrail endpoint from a know handrail. 

### PointToLineFactor.h
The PointToLineFactor constrains a gtsam::Pose3 using a point detection in the sensor frame compared with a line in the world frame.

### PointToLineSegmentFactor.h
The PointToLineSegmentFactor constrains a gtsam::Pose3 using a point detection in the sensor frame compared with a line segment in the world frame.  This factor contains a discontinuity in the Jacobian as there is zero error along the line segment axis if the point is between line segment endpoints and non-zero error otherwise.  An option to use a SILU (Sigmoid Linear Unit) approximation is provided for this case.

###PointToPlaneFactor.h
The PointToPlaneFactor constrains a gtsam::Pose3 using a point detection in the sensor frame compared with a plane in the world frame.

### RobustSmartProjectionFactor
The RobustSmartProjectionFactor adds to the gtsam::SmartProjectionFactor by providing a robust huber kernel.  Additionally, it fixes some issues in the SmartProjectionFactor allowing for a rotation-only fallback when using the JacobianSVD option and allows for proper serialization of the factor.

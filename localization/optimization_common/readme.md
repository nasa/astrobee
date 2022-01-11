\page optimizationcommon Optimization Common

# Package Overview

### Residuals
Provides various error functions (point to point, point to plane, reprojection, etc.) for usage in Ceres-based nonlinear optimization problems.

### SE3 Local Parameterization 
Provides a local parameterization for the SE3 Lie Group that performs on-manifold updates using the axis-angle Lie Algebra.  Axis-angle updates are provided with an array in compact axis-angle format (where the norm of the 3d array is the angle and the normalized 3d array is the axis) and applied using right multiplication on the given SE3 element, mimicking the right multiplicative updates used by the GTSAM library.

### Utilities 
Provides helper functions to convert between minimal and non-minimal representations for optimization variables along with other optimization helper functions.

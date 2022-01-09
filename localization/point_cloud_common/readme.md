\page pointcloudcommon Point Cloud Common

# Package Overview

### PointToPlaneICP 
Performs point to plane ICP using the PCL library.  Supports nonsymmetric and symmetric costs along with optional correspondence rejection when a symmetric cost is used.  Also has options for downsampling the input point clouds and performing coarse to fine ICP.

### PointCloudWithKnownCorrespondencesAligner 
Performs nonliner optimization with a robust huber cost to align two sets of corresponding 3d points (point to point cost).  This is useful when correspondences between two point clouds are known and do not need to be reestimated iteratively as is done in ICP, resulting in an often much faster aligment.  Supports performing an initial guess using the Umeyama algorithm.  Additionally supports using a point to plane or symmetric point to plane cost, requiring surface normals in addition to 3d correspondences as needed.

### Utilities 
Provides various utilities for point cloud operations such as normal estimation, downsampling, valid point filtering, covariance estimation, and more.

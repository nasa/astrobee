\defgroup localization_manager Localization Manager
\ingroup localization

Depending on the context, flight software might switch localization systems. For example, in free-flight sparse mapping is used to localize the robot. However, the accuracy of this method (several centimeters) is not high enough to ensure reliable docking. Thus, when approaching the dock, flight software will switch to another localization mechanism (marker tracking), which localizes by visually tracking targets on the dock itself with a different camera. The **localization manager** is responsible for carrying out a controlled switch between the two localization systems, continually monitoring the health of the active localization system, and falling back to a safe localization system automatically when there is a problem.

The localization manager supports the following localization pipelines:

1. None (NO) - Do not localize.
2. Sparse mapping (ML) - Localize by finding correspondences between features seen from the navigation camera and those in a 3D map, build a priori. This relies on messages produced by the sparse_mapping node.
3. Marker tracking (AR) - Localize by detecting augmented reality (AR) markers on a target that has a known pose with respect to the world. This relies on messages produced by the marker_tracking node and optical flow.
4. Handrail localization (HR) - Localize with respect to a specific handrail detected from a point cloud, with an unknwon pose with respect to the world. This relies on messages produced by the handrail_detect node.
5. Ground truth (GT) - Localize using pose and twist feedback from a ground truth localization system, such as the HTC Vive or Gazebo simulator.  This relies on either the vive node (real experiement) or the Gazebo simulator.

Refer to the image below. Each localization system can be thought of as a pipeline that converts raw measurements from a sensor into features. Since there is a delay in this pipeline, a registration pulse is sent as soon as the image is received, and this is used in the update step of the filter to perform a delay-aware correction of the state estimate. The **set_input** service on the EKF is essentially a multiplexer, which passes only one pipeline's features into the filter at any point.

![alt text](../images/localization-manager.png "The localization manager")

In order to perform a safe switch, the localization manager must run through a fixed sequence of steps:

1. Power up the new pipeline.
2. Check that measurements are being produced by the required sensor.
3. Check that sufficient features are being found by the pipeline.
4. Switch to the new pipeline.
5. Check that the pipeline is stable for a period.
6. Switch off the old pipeline.
7. Monitor the current pipeline.

If step [5] above fails the localization manager falls back to the previous pipeline so that the platform remains stable.

If step [7] above fails the localization manager automatically switches back to the fallback pipeline, which is set to sparse mapping.



\defgroup localization_manager Localization Manager
\ingroup localization
Depending on the context, flight software might switch localization systems. For example, in free-flight visual landmarks are used to localize the robot. However, the accuracy of this method (several centimeters) is not high enough to ensure reliable docking. Thus, when approaching the dock, flight software will switch to another localization mechanism (AR target), which localizes by visually tracking targets on the dock itself with a different camera. The **localization manager** is responsible for carrying out a controlled switch between the two localization systems. 

Refer to the image below. Each localization system can be thought of as a pipeline that converts raw measurements from a sensor into features. Since there is a delay in this pipeline, a registration is performed when the measurement occurs, and this is used in the update step of the filter to perform a delay-aware update of the state estimate. The **set_input** service on the EKF is essentially a multiplexer, which passes only one pipeline's features into the filter at any point.

![alt text](../images/localization-manager.png "The localization manager")

In order to perform a safe switch, the localization manager has to run through a fixed sequence of steps:

1. Power up the new localization pipeline.
2. Check that measurements are being produced by the driver.
3. Check that sufficient features are being found by the feature detector.
4. Switch to the new localization system.
5. Check that the new localization system remains stable for a period before considering the pipeline as usable.
6. Switch off the old localization system.

In the case where step [5] above fails, one must fall back to the previous localization system so that the platform remains controllable. One key assumption is that the platform is stationary when the switch is initiated.

# Adding a new localization system


\page localizationmeasurements Localization Measurements

# Package Overview
Localization measurements provides an inheritance class for sensor data used by the localizer.  Each measurement has a timestamp, and a measurement fully encapsulates the data provided by a sensor message.  Information used by multiple measurements are given their own objects for modularity.  Whenever new sensor information will be used by the localizer, it is recommended to add a xxxx_measurement object if one does not already exist.  
Measurements are converted from ros msgs to measurement objects in the measurements\_conversion file.  Since this may happen in multiple places (for instance the localizer and also an offline localization tool), it is recommened to add conversion here to be used by other packages.

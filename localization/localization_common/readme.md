\page localizationcommon Localization Common 

# Package Overview
Localization common provides several objects that are quite useful for various localization functions, along with various helper functions.  Many of these helper functions make converting between gtsam and ros types required for ros messages easier, along with wrapping some parameter loading functions to provide a cleaner parameter loading interface.  

# Important Classes
## CombinedNavState
CombinedNavState extends the gtsam::NavState to include imu biases and a timestamp.  This is the state that is estimated by the localizer.

## Timer and Averager
The timer and averager classes provide easy ways to profile data and save statistics.  Each maintains a rolling average to avoid overflow and optionally logs data to a log file. 

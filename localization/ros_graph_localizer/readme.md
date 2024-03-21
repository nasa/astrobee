\page rosgraphlocalizer Ros Graph Localizer

# Package Overview
Ros wrapper for the graph localizer. The graph localizer wrapper takes ros messages, convers these to localization measurements, and passes these to the graph localizer. The ros graph localizer nodelet handles live subscribing and publishing to ros topics.

## RosGraphLocalizerWrapper
Contains the GraphLocalizer. Converts ROS messages to localization measurements and provides these to the localizer.

## RosGraphVIONodelet
Subscribes to ROS messages for online use and publishes GraphVIO messages and TFs. Contains a RosGraphLocalizerWrapper and passes messages to this.

# Inputs
* `/graph_vio/state`
* `/loc/ml/features`

# Outputs
* `graph_loc/graph`
* `graph_loc/state`

\page interactive_marker_teleop Interactive Marker Teleop

The interactive marker tool overlays a control in rviz to command robot movements and display feedback. 

1. Run the tool
```rosrun interactive_marker_teleop interactive_marker_teleop```
2. Add to rviz using the `Add` button in the displays panel and choose `By Topic`, then `InteractiveMarkers`

Move the marker to a desired location and right click the marker to initiate commands from the dropdown menu. Feedback can be found in the console if the command is not valid.

Clicking the marker can be a challenge if there are objects in the way. Disabling "Debug/zones", for example, will help.

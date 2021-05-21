\page total_station Total Station

# Doing a Survey with the Total Station

This will tell you how to just operate the Total Station device. When
you use it in practice, you'll want to set your reference to the -1,
-1, -1 corner on the aluminum frame in granite. Then you'll want to
survey the following:

 * The corners of the black squares on well-distributed set of AR
   targets, so that the AR tags can be in the global coordinate frame.

 * The body of the overhead camera, so that the overhead's position
   estimate is accurate to the global coordinate frame.

After you have surveyed the above and anything else of interest to
you, you should add the vector [-1, -1, -1] to all measurements as we
traditionally like the origin to be the center of the beacon volume in
granite lab.

 1. Get the Leica Total Station which is in the red box just outside the Garage (not roverscape).
 2. Set it up in the granite lab. Place down some carpet so the legs don't slip. Place and screw down the top unit to the stand. There is a bubble level on side of the top unit. Zero it by adjusting the 3 turn knobs.
 3. Power on the unit by pressing and holding the "prog" button.
 4. Ensure that the battery is charged, otherwise use the charger to charge it.
 5. You are now ready to start a survey.
 6. Press "1" to Survey
   a. Press Enter
   a. F2 for new Job
   a. Write job name, then hit 'Enter'
   a. Use CE to erase
   a. Use F6 to switch into letter mode
   a. Press F1 to store solution
   a. Press F1 to continue
   a. May need to press F1 again
   a. For the Reflector option, switch to Reflectorless mode (otherwise there will be an error about a missing prism).
 7. Press "F3" to Setup Coordinate System
   a. Select method, change to "Local Resection".
   a. Select stn ht from, change to "Target 1 Ht Diff".
   a. Select station ID, enter the value "FRONTOFGRANITE" (or something shorter).
 8. Press "F1" to continue and survey the origin.
   a. Select "Point ID", enter the value "ORIGIN" (or some other value).
   a. Move the sights to look at a corner bolt of the aluminum frame (if in the granite lab). This will be the origin of the output coordinate system.
   a. Use the knobs to zero the cross hairs perfectly on the bolt. You might also need to change the focus of the sights.
   a. Once aligned correctly, press "F2" to get a distance measurement.
   a. Press "F3 to record the measurement.
 9. Now you are ready to survey the Y axis reference. Z axis will be aligned with gravity, pointing towards the sky.
   a. Select "Point ID", enter the value "YREF" (or some other value).
   a. Move the sights to look at the opposite corner bolt of the aluminum frame.
   a. Once aligned correctly, use "F1" to record distance and save the point. If you get an error about point distribution, you will have to redo the ORIGIN and YREF points as further apart (use at least a few meters among them).
 10. Press "F1" to set.
 11. Press "F4" ok.
 12. You are now ready to survey things. Keep on centering the cross-hairs on new points and pressing F3 to record it. Survey a well-distributed set of AR tag corners on room walls and survey for example the top left, top right, bot left corners of each of them. Each AR tag has an ID printed on it, that can be associated with each measurement.

Once you are done you need to export your project to the GSI format so you can read it back in.

 * Keep hitting ESC until you are back to the home screen.
 * Press "4" to convert
 * Press "1" to export
 * Select Job, change to what ever is your job.
 * Select Format File, change to "gsi16_cartesian.frt".
 * Select File Name, write value for what you want the output file to be.
 * Press "F1" to save.
 * Press "F4" to close out.

Turn the nob on the side of the station where it says "CF Card" and pull the card out. Hook it up to a USB reader and get the measurement file. Here is a sample data file:

*110001+0000000000ORIGIN 81...0+0000000000000000 82...0+0000000000000000 83...0+0000000000000000 
*110002+000000000000YREF 81...0+0000000000000000 82...0+0000000000006417 83...0-0000000000000022 
*110003+0000000000000003 81...0+0000000000000004 82...0+0000000000006535 83...0-0000000000000786 

What this means is that the points are in Cartesian coordinates with x,y,z measured in millimeters (note that some values are negative). After removing the redundant text and converting to meters, this looks like:

Point 1 (Origin): 0.0    0.0      0.0
Point 2 (YREF)  : 0.0    6.417    -0.022
Point 3         : 0.004  6.535    -0.786     

The program 

  localization/sparse_mapping/tools/parse_total_station.py

can be used to convert from the Total Station format to an xyz format
that can be used for sparse map registration. 

When you are done, shut off the machine:

 * Keep hitting ESC until you are back to the home screen.
 * Press USER and PROG at the same time.
 * Press "F6" to confirm.

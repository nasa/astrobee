\page data_bagger Data Bagger

This node manages recording for the Astrobee robot. This node is mainly used with GDS where different profiles can be easily loaded and managed.

It distinguishes between two types of recording, immediate and delayed.
 * Immediate recording: enabled for the entire duration where the flight software is on. The topics it records are defined in the config file "management/data_bagger.config" and can't be customized during flight.
 * Delayed recording: used for data collection, the topics it records can be customized and the recording can start and stop at any time.

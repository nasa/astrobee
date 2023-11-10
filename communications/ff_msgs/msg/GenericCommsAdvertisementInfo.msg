# timestamp in header should be time of whatever triggered the advertisement
std_msgs/Header header

#topic on which to republish
string output_topic

#whether should be republished as a latching topic
bool latching

#standard ROS meanings for message descriptions
# eg std_msgs/Int32
string data_type
# eg da5909fbe378aeaf85e547e830cc1bb7
string md5_sum
# eg a definition such as this file
string definition


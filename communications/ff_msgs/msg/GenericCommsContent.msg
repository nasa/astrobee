# timestamp in header should be time of receipt of original message
std_msgs/Header header

#topic on which to republish
string output_topic

#md5sum of msg type, repeated from advertisement
string type_md5_sum

#serialized content of the message
#definition, etc. assumed to be known from preceding advertisement
uint8[] msg


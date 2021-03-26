######################################################
# A message containing basic REALM sparse cloud data #
######################################################

# (required) Header containing message meta data
std_msgs/Header header

# (required) Vector of unique point ids
uint32[] point_ids

# (required) Data of the sparse cloud structured as cv::Mat with row(i) = x,y,z
sensor_msgs/Image points
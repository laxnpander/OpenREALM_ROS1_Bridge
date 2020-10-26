##################################################
# A message containing basic REALM depthmap data #
##################################################

# (required) Header containing message meta data
std_msgs/Header header

# (required) Camera_model containing pinhole parameters of capturing cam
realm_msgs/Pinhole camera_model

# (required) Pose during acquisition
geometry_msgs/Pose pose

# (required) Depth information
sensor_msgs/Image data
#!/usr/bin/env bash

# enable ROS
source /opt/ros/kinetic/setup.bash
source ${HOME}/catkin_ws/devel/setup.bash

# record the bag
kinect_res=$2
node_name=$3

rosbag record --output-prefix $1 -b 0 \
/tf /tf_static \
-e "/deepgrasp/kinect2_(left|right|middle)/${kinect_res}/(camera_info|image_color_rect/compressed|image_depth_rect/compressed)" \
__name:=${node_name}

# debug
# echo $1 $2

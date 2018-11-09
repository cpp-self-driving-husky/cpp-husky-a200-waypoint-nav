#!/bin/bash

if [ $# -gt 0 ]; then
	FILE_PREFIX=$1
else
	FILE_PREFIX="localize"
fi

FILE_NAME="$FILE_PREFIX"
echo $FILE_NAME

rosbag record -o $FILE_NAME /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /outdoor_waypoint_nav/odometry/gps /odometry/filtered /imu/data /imu/data_raw /navsat/fix /gps/fix /cmd_vel /move_base/status /tf /tf_static


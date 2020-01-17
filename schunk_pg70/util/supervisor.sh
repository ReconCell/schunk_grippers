#!/bin/bash
# PATH=$PATH:/usr/local/bin

source /home/ubuntu/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

echo $(/opt/ros/kinetic/bin/rospack find rpigpio_ros)
# /bin/echo $PATH
export ROS_MASTER_URI=http://localhost:11311/
/opt/ros/kinetic/bin/roslaunch schunk_pg70 pg70_rs232_control.launch
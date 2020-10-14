#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting test" 

cd ~/catkin_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash

catkin build mrs_lib --catkin-make-args tests
rostest mrs_lib geometry_utils_test.launch -t --results-filename=mrs_lib.test --results-base-dir=/tmp
catkin_test_results /tmp

echo "Test ended"

#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting test" 

cd ~/catkin_ws
source /opt/ros/$ROS_DISTRO/setup.bash

catkin build mrs_lib --catkin-make-args tests

source /opt/ros/$ROS_DISTRO/setup.bash

rostest mrs_lib geometry_utils_test.launch -t --results-filename=mrs_lib.test --results-base-dir=/tmp/mrs_lib
catkin_test_results /tmp/mrs_lib

echo "Test ended"

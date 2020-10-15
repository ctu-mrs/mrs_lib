#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting test" 

cd ~/catkin_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash

catkin build mrs_lib # it has to be fully build normally before building with --catkin-make-args tests
catkin build mrs_lib --catkin-make-args tests
TEST_RESULT_PATH=$(realpath /tmp/$RANDOM)
mkdir -p $TEST_RESULT_PATH
rostest mrs_lib geometry_utils_test.launch --results-filename=mrs_lib.test --results-base-dir="$TEST_RESULT_PATH"
catkin_test_results "$TEST_RESULT_PATH"

echo "Test ended"

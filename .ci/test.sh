#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting test"

cd ~/mrs_workspace
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/mrs_workspace/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311

roscore &

sleep 2

PACKAGE="mrs_lib"
VERBOSE=0

[ "$VERBOSE" = "0" ] && TEXT_OUTPUT=""
[ "$VERBOSE" = "1" ] && TEXT_OUTPUT="-t"

# build the package
catkin build $PACKAGE # it has to be fully built normally before building with --catkin-make-args tests
catkin build $PACKAGE --catkin-make-args tests

# folder for test results
TEST_RESULT_PATH=$(realpath /tmp/$RANDOM)
mkdir -p $TEST_RESULT_PATH

# run the test
rostest --reuse-master $PACKAGE mrs_lib.test $TEXT_OUTPUT --results-filename=$PACKAGE.test --results-base-dir="$TEST_RESULT_PATH"

# evaluate the test results
catkin_test_results "$TEST_RESULT_PATH"

echo "Test ended"

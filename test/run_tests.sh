#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

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

export ROS_MASTER_URI=http://localhost:11311

roscore &

# run the test
rostest --reuse-master $PACKAGE mrs_lib.test $TEXT_OUTPUT --results-filename=$PACKAGE.test --results-base-dir="$TEST_RESULT_PATH"

# evaluate the test results
echo test result path is $TEST_RESULT_PATH
catkin_test_results "$TEST_RESULT_PATH"
success=$?

echo ""
if [ $success = "0" ]; then
  echo -e "\033[0;32mSuccess! \033[1;"
else
  echo -e "\033[0;31mFailed some tests! \033[1;"
fi

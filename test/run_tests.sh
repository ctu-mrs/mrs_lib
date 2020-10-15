#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

VERBOSE=0

[ "$VERBOSE" = "0" ] && TEXT_OUTPUT=""
[ "$VERBOSE" = "1" ] && TEXT_OUTPUT="-t"

catkin build mrs_lib # it has to be fully build normally before building with --catkin-make-args tests
catkin build mrs_lib --catkin-make-args tests
TEST_RESULT_PATH=$(realpath /tmp/$RANDOM)
mkdir -p $TEST_RESULT_PATH
rostest mrs_lib geometry_utils.test $TEXT_OUTPUT --results-filename=mrs_lib.test --results-base-dir="$TEST_RESULT_PATH"
catkin_test_results "$TEST_RESULT_PATH"

echo ""
echo "Success? $?"

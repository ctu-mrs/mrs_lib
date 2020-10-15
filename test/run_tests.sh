#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

PACKAGE="mrs_lib"
VERBOSE=0

[ "$VERBOSE" = "0" ] && TEXT_OUTPUT=""
[ "$VERBOSE" = "1" ] && TEXT_OUTPUT="-t"

# build the package
catkin build $PACKAGE # it has to be fully build normally before building with --catkin-make-args tests
catkin build $PACKAGE --catkin-make-args tests

# folder for test results
TEST_RESULT_PATH=$(realpath /tmp/$RANDOM)
mkdir -p $TEST_RESULT_PATH

# run the test
rostest $PACKAGE mrs_lib.test $TEXT_OUTPUT --results-filename=$PACKAGE.test --results-base-dir="$TEST_RESULT_PATH"

# evaluate the test results
catkin_test_results "$TEST_RESULT_PATH"

echo ""
echo "Success? $?"

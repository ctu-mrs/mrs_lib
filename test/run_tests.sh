#!/bin/bash

catkin build mrs_lib --catkin-make-args tests
rostest mrs_lib geometry_utils_test.launch -t --results-filename=mrs_lib.test --results-base-dir=/tmp/mrs_lib
catkin_test_results /tmp/mrs_lib

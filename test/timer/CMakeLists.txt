mrs_lib_add_simple_labeled_gtest(ros_wrappers test_timer "timer_test.cpp")
set_tests_properties(test_timer
  PROPERTIES
    TIMEOUT 120
)

mrs_lib_add_simple_labeled_gtest(coro test_coro_cancellation "cancellation_test.cpp")
mrs_lib_add_simple_labeled_gtest(coro test_coro "coro_test.cpp")

set_tests_properties(test_coro
  PROPERTIES
    TIMEOUT 300
)

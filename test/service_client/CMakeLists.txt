find_package(std_srvs REQUIRED)

mrs_lib_add_simple_labeled_gtest(ros_wrappers test_service_client "service_client_test.cpp")

target_link_libraries(test_service_client
  ${std_srvs_TARGETS}
)

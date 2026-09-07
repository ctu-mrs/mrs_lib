find_package(std_srvs REQUIRED)

mrs_lib_add_simple_labeled_gtest(ros_wrappers test_service_server "service_server_test.cpp")

target_link_libraries(test_service_server
  ${std_srvs_TARGETS}
)

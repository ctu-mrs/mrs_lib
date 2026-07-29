add_library(example_coro_service_component SHARED)
target_sources(example_coro_service_component
  PRIVATE
    "coro_service.cpp"
)

target_link_libraries(example_coro_service_component
  PRIVATE
    mrs_lib::mrs_lib
    rclcpp_components::component
    ${std_srvs_TARGETS}
)

rclcpp_components_register_node(example_coro_service_component
  PLUGIN mrs_lib_examples::CoroServiceExample
  EXECUTABLE example_coro_service
  EXECUTOR rclcpp::executors::SingleThreadedExecutor
)

rclcpp_components_register_node(example_coro_service_component
  PLUGIN mrs_lib_examples::CoroServiceExampleServer
  EXECUTABLE example_coro_service_server
)

install(
  TARGETS example_coro_service_component
)

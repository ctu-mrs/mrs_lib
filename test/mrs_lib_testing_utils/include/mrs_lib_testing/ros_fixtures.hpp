#ifndef MRS_LIB_TESTING_ROS_FIXTURES_HPP_
#define MRS_LIB_TESTING_ROS_FIXTURES_HPP_


#include <gtest/gtest.h>

#include <rclcpp/utilities.hpp>


namespace mrs_lib_testing
{

  class RosInitAndShutdownFixture : public ::testing::Test
  {
  public:
    RosInitAndShutdownFixture()
    {
      rclcpp::init(0, nullptr);
    }

    ~RosInitAndShutdownFixture()
    {
      rclcpp::shutdown();
    }

    RosInitAndShutdownFixture(const RosInitAndShutdownFixture&) = delete;
    RosInitAndShutdownFixture(RosInitAndShutdownFixture&&) = delete;
    RosInitAndShutdownFixture& operator=(const RosInitAndShutdownFixture&) = delete;
    RosInitAndShutdownFixture& operator=(RosInitAndShutdownFixture&&) = delete;
  };

} // namespace mrs_lib_testing

#endif // MRS_LIB_TESTING_ROS_FIXTURES_HPP_

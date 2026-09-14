#ifndef MRS_LIB_TESTING_ROS_FIXTURES_HPP_
#define MRS_LIB_TESTING_ROS_FIXTURES_HPP_


#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include <rclcpp/executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
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

  template <typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
  class RosExecutorFixture : public RosInitAndShutdownFixture
  {
  public:
    template <typename... ExecutorArgs>
    RosExecutorFixture(ExecutorArgs&&... args)
        : executor_(std::make_shared<ExecutorT>(std::forward<ExecutorArgs>(args)...)), spin_thread_([this]() { executor_->spin(); })
    {
      while (!executor_->is_spinning())
      {
        using namespace std::chrono_literals;
        std::cout << "Waiting for executor to start...\n" << std::flush;
        std::this_thread::sleep_for(1ms);
      }
    }

    ~RosExecutorFixture()
    {
      executor_->cancel();

      if (spin_thread_.joinable())
      {
        spin_thread_.join();
      }
    }

    RosExecutorFixture(const RosExecutorFixture&) = delete;
    RosExecutorFixture(RosExecutorFixture&&) = delete;
    RosExecutorFixture& operator=(const RosExecutorFixture&) = delete;
    RosExecutorFixture& operator=(RosExecutorFixture&&) = delete;

    rclcpp::Executor& get_executor()
    {
      return *executor_;
    }

  private:
    std::shared_ptr<rclcpp::Executor> executor_;

    std::jthread spin_thread_;
  };

} // namespace mrs_lib_testing

#endif // MRS_LIB_TESTING_ROS_FIXTURES_HPP_

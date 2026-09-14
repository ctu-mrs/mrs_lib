#include "mrs_lib/subscriber.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>

#include "mrs_lib/publisher_handler.h"

#include "mrs_lib_testing/ros_fixtures.hpp"

namespace
{

  namespace example
  {

    // DOCS: BEGIN EXAMPLE
    class ExampleNode : public rclcpp::Node
    {
    public:
      ExampleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
          : Node("example_node", options),
            logger_(*this),
            subscriber_(mrs_lib::SubscriberOptions{.node_interfaces = *this}, "example_topic", std::bind_front(&ExampleNode::message_callback, this))
      {
      }

      [[nodiscard]] int64_t get_last_val() const
      {
        return last_val_;
      }

    private:
      void message_callback(std::shared_ptr<const std_msgs::msg::Int64> msg)
      {
        logger_.info("Message received with payload: {}", msg->data);
        last_val_ = msg->data;
      }

      mrs_lib::Logger logger_;

      std::atomic<int64_t> last_val_ = 0;

      mrs_lib::Subscriber<std_msgs::msg::Int64> subscriber_;
    };
    // DOCS: END EXAMPLE

    class SubscriberExampleTest : public mrs_lib_testing::RosExecutorFixture<>
    {
    };


    TEST_F(SubscriberExampleTest, Example)
    {
      using namespace std::chrono_literals;

      auto node = std::make_shared<ExampleNode>();
      get_executor().add_node(node);

      auto publisher = node->create_publisher<std_msgs::msg::Int64>("example_topic", 1);

      {
        std_msgs::msg::Int64 msg;
        msg.data = 42;
        publisher->publish(msg);
      }

      std::this_thread::sleep_for(10ms);

      EXPECT_EQ(node->get_last_val(), 42);

      get_executor().remove_node(node);
    }

  } // namespace example


  using namespace std::chrono_literals;

  class SubscriberTest : public ::mrs_lib_testing::RosExecutorFixture<>
  {

  public:
    void SetUp() override
    {
      node_ = std::make_shared<rclcpp::Node>("test_subscriber", rclcpp::NodeOptions().use_intra_process_comms(false));

      get_executor().add_node(node_);
    }

    void TearDown() override
    {
      get_executor().remove_node(node_);
    }

    void message_callback(std::shared_ptr<const std_msgs::msg::Int64> msg)
    {
      RCLCPP_INFO(node_->get_logger(), "message received");

      if (msg->data == num_to_send)
      {
        num_received++;
      }
    }

    void timeout_callback(std::string_view, rclcpp::Time)
    {
      RCLCPP_INFO(node_->get_logger(), "timeout triggered");

      timeout_triggered_ = true;
    }

    rclcpp::Node::SharedPtr node_;

    int num_received = 0;

    std::atomic<bool> timeout_triggered_ = false;

    int num_to_send = 1234;
  };

  template <typename MessageType>
  bool wait_for_publisher(const mrs_lib::Subscriber<MessageType>& subscriber, std::shared_ptr<rclcpp::Clock> clock)
  {
    rclcpp::Rate rate(1.0, std::move(clock));

    for (int i = 0; i < 10; i++)
    {

      if (subscriber.get_publisher_count() > 0)
      {
        break;
      }

      rate.sleep();
    }

    return subscriber.get_publisher_count() > 0;
  }

  TEST_F(SubscriberTest, Polled)
  {

    auto clock = node_->get_clock();

    RCLCPP_INFO(node_->get_logger(), "creating publisher");

    // | ---------------- create publisher handler ---------------- |

    mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

    // | ------------------- create a subscriber ------------------ |

    RCLCPP_INFO(node_->get_logger(), "creating subscriber");

    mrs_lib::SubscriberOptions sh_opts{.node_interfaces = *node_};

    auto sh_int_ = mrs_lib::Subscriber<std_msgs::msg::Int64>(sh_opts, "/topic1", mrs_lib::PolledSubscriberTag{});

    // | ---------------------- start testing --------------------- |

    RCLCPP_INFO(node_->get_logger(), "initialized");

    if (!wait_for_publisher(sh_int_, clock))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
      FAIL();
    }

    clock->sleep_for(1s);

    std_msgs::msg::Int64 data;
    data.data = num_to_send;

    RCLCPP_INFO(node_->get_logger(), "publishing");

    ph_int.publish(data);

    clock->sleep_for(1s);

    auto msg_opt = sh_int_.get_message();
    ASSERT_TRUE(msg_opt.has_value());

    auto msg = msg_opt.value();

    EXPECT_EQ(num_to_send, msg->data);

    RCLCPP_INFO(node_->get_logger(), "finished");
  }

  TEST_F(SubscriberTest, Callback)
  {

    auto clock = node_->get_clock();

    RCLCPP_INFO(node_->get_logger(), "creating publisher");

    // | ---------------- create publisher handler ---------------- |

    mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

    // | ------------------- create a subscriber ------------------ |

    RCLCPP_INFO(node_->get_logger(), "creating subscriber");

    mrs_lib::SubscriberOptions sh_opts{.node_interfaces = *node_};

    auto sh_int_ = mrs_lib::Subscriber<std_msgs::msg::Int64>(sh_opts, "/topic1", [this](auto&& msg) { message_callback(msg); });

    // | ---------------------- start testing --------------------- |

    RCLCPP_INFO(node_->get_logger(), "initialized");

    if (!wait_for_publisher(sh_int_, clock))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
      FAIL();
    }

    clock->sleep_for(1s);

    std_msgs::msg::Int64 data;
    data.data = num_to_send;

    RCLCPP_INFO(node_->get_logger(), "publishing");

    ph_int.publish(data);

    clock->sleep_for(1s);

    EXPECT_GE(num_received, 1);

    RCLCPP_INFO(node_->get_logger(), "finished");
  }

  TEST_F(SubscriberTest, PolledTimeout)
  {
    auto clock = node_->get_clock();

    RCLCPP_INFO(node_->get_logger(), "creating publisher");

    // | ---------------- create publisher handler ---------------- |

    mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

    // | ------------------- create a subscriber ------------------ |

    RCLCPP_INFO(node_->get_logger(), "creating subscriber");

    mrs_lib::SubscriberOptions sh_opts{
        .node_interfaces = *node_,
        .no_message_timeout = 500ms,
        .timeout_callback = [this](std::string_view name, rclcpp::Time time) { timeout_callback(name, time); },
    };

    auto sh_int_ = mrs_lib::Subscriber<std_msgs::msg::Int64>(sh_opts, "/topic1", mrs_lib::PolledSubscriberTag{});

    // | ---------------------- start testing --------------------- |

    RCLCPP_INFO(node_->get_logger(), "initialized");

    if (!wait_for_publisher(sh_int_, clock))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
      FAIL();
    }

    RCLCPP_INFO(node_->get_logger(), "publishing");

    {
      rclcpp::Rate rate(100.0, clock);

      for (int i = 0; i < 100; i++)
      {

        std_msgs::msg::Int64 data;
        data.data = num_to_send;

        ph_int.publish(data);

        EXPECT_FALSE(timeout_triggered_);

        rate.sleep();
      }
    }

    RCLCPP_INFO(node_->get_logger(), "stopping publisher");

    clock->sleep_for(1s);

    EXPECT_TRUE(timeout_triggered_);

    RCLCPP_INFO(node_->get_logger(), "finished");
  }

  TEST_F(SubscriberTest, CallbackTimeout)
  {
    auto clock = node_->get_clock();

    RCLCPP_INFO(node_->get_logger(), "creating publisher");

    // | ---------------- create publisher handler ---------------- |

    mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

    // | ------------------- create a subscriber ------------------ |

    RCLCPP_INFO(node_->get_logger(), "creating subscriber");

    mrs_lib::SubscriberOptions sh_opts{
        .node_interfaces = *node_,
        .no_message_timeout = 500ms,
        .timeout_callback = [this](std::string_view name, rclcpp::Time time) { timeout_callback(name, time); },
    };

    auto sh_int_ = mrs_lib::Subscriber<std_msgs::msg::Int64>(sh_opts, "/topic1", [this](auto&& msg) { message_callback(msg); });

    // | ---------------------- start testing --------------------- |

    RCLCPP_INFO(node_->get_logger(), "initialized");

    if (!wait_for_publisher(sh_int_, clock))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
      FAIL();
    }

    RCLCPP_INFO(node_->get_logger(), "publishing");

    {
      rclcpp::Rate rate(100.0, clock);

      for (int i = 0; i < 100; i++)
      {

        std_msgs::msg::Int64 data;
        data.data = num_to_send;

        ph_int.publish(data);

        EXPECT_FALSE(timeout_triggered_);

        rate.sleep();
      }
    }

    RCLCPP_INFO(node_->get_logger(), "stopping publisher");

    clock->sleep_for(1s);

    EXPECT_TRUE(timeout_triggered_);

    RCLCPP_INFO(node_->get_logger(), "finished");
  }

} // namespace

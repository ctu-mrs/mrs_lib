#include "mrs_lib/publisher.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>

#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include <std_msgs/msg/int64.hpp>

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
            publisher_(
                mrs_lib::PublisherOptions{
                    .node_interfaces = *this,
                },
                "example_topic")
      {
      }

      void work()
      {
        auto val = next_val_.fetch_add(1);
        logger_.info("Publishing value: {}", val);

        std_msgs::msg::Int64 msg;
        msg.data = val;

        publisher_.publish(msg);
      }

    private:
      mrs_lib::Logger logger_;

      std::atomic<int64_t> next_val_ = 1;

      mrs_lib::Publisher<std_msgs::msg::Int64> publisher_;
    };
    // DOCS: END EXAMPLE

    class PublisherExampleTest : public mrs_lib_testing::RosExecutorFixture<>
    {
    };


    TEST_F(PublisherExampleTest, Example)
    {
      using namespace std::chrono_literals;

      auto node = std::make_shared<ExampleNode>();
      get_executor().add_node(node);

      std::atomic<size_t> received_count = 0;
      std::atomic<int64_t> latest_val = 0;

      std::function callback = [&](std::shared_ptr<const std_msgs::msg::Int64> msg) {
        ++received_count;
        latest_val = msg->data;
      };

      auto publisher = node->create_subscription<std_msgs::msg::Int64>("example_topic", rclcpp::SystemDefaultsQoS(), callback);

      for (size_t i = 0; i < 10; ++i)
      {
        node->work();
        std::this_thread::sleep_for(10ms);

        EXPECT_EQ(received_count, i + 1);
        EXPECT_EQ(latest_val, static_cast<int64_t>(i + 1));
      }

      get_executor().remove_node(node);
    }

  } // namespace example

  using namespace std::chrono_literals;

  class PublisherTest : public mrs_lib_testing::RosExecutorFixture<>
  {

  public:
    void SetUp() override
    {
      node_ = std::make_shared<rclcpp::Node>("test_publisher", rclcpp::NodeOptions().use_intra_process_comms(false));
      get_executor().add_node(node_);
    }

    void TearDown() override
    {
      get_executor().remove_node(node_);
    }

    void message_callback(std::shared_ptr<const std_msgs::msg::Int64> msg)
    {
      RCLCPP_INFO(node_->get_logger(), "message received");

      if (msg->data == val_to_send_)
      {
        received_count_++;
      }
    }

    rclcpp::Node::SharedPtr node_;

    int received_count_ = 0;
    int val_to_send_ = 1234;
  };


  template <typename MessageType>
  bool wait_for_publisher(const rclcpp::Subscription<MessageType>& subscriber, std::shared_ptr<rclcpp::Clock> clock)
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


  TEST_F(PublisherTest, Publish)
  {
    auto clock = node_->get_clock();

    RCLCPP_INFO(node_->get_logger(), "creating publisher");

    // | ---------------- create publisher handler ---------------- |

    auto publisher = mrs_lib::Publisher<std_msgs::msg::Int64>(
        mrs_lib::PublisherOptions{
            .node_interfaces = *node_,
        },
        "/topic1");

    // | ------------------- create a subscriber ------------------ |

    RCLCPP_INFO(node_->get_logger(), "creating subscriber");

    std::function<void(std::shared_ptr<const std_msgs::msg::Int64>)> bound_callback = std::bind_front(&PublisherTest::message_callback, this);

    auto subscriber = node_->create_subscription<std_msgs::msg::Int64>("/topic1", 100, bound_callback);

    // | ---------------------- start testing --------------------- |

    RCLCPP_INFO(node_->get_logger(), "initialized");

    if (!wait_for_publisher(*subscriber, clock))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
      FAIL();
    }

    std_msgs::msg::Int64 data;
    data.data = val_to_send_;

    {
      rclcpp::Rate rate(100.0, clock);

      for (int i = 0; i < 10; i++)
      {

        RCLCPP_INFO(node_->get_logger(), "publishing");

        publisher.publish(data);

        rate.sleep();
      }
    }

    clock->sleep_for(100ms);

    EXPECT_EQ(received_count_, 10);

    RCLCPP_INFO(node_->get_logger(), "finished");
  }

  TEST_F(PublisherTest, Throttling)
  {
    auto clock = node_->get_clock();

    RCLCPP_INFO(node_->get_logger(), "creating publisher");

    // | ---------------- create publisher handler ---------------- |

    auto publisher = mrs_lib::Publisher<std_msgs::msg::Int64>(
        mrs_lib::PublisherOptions{
            .node_interfaces = *node_,
            .throttle_duration = 100ms,
        },
        "/topic1");

    // | ------------------- create a subscriber ------------------ |

    RCLCPP_INFO(node_->get_logger(), "creating subscriber");

    std::function<void(std::shared_ptr<const std_msgs::msg::Int64>)> bound_callback = std::bind_front(&PublisherTest::message_callback, this);

    auto subscriber = node_->create_subscription<std_msgs::msg::Int64>("/topic1", 100, bound_callback);

    // | ---------------------- start testing --------------------- |

    RCLCPP_INFO(node_->get_logger(), "initialized");


    if (!wait_for_publisher(*subscriber, clock))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
      FAIL();
    }

    std_msgs::msg::Int64 data;
    data.data = val_to_send_;

    {
      rclcpp::Rate rate(100, clock);

      for (int i = 0; i < 100; i++)
      {

        RCLCPP_INFO(node_->get_logger(), "publishing");

        publisher.publish(data);

        rate.sleep();
      }
    }

    clock->sleep_for(100ms);

    if (received_count_ < 9 || received_count_ > 11)
    {
      RCLCPP_ERROR(node_->get_logger(), "did not received the correct number of messages, want 9 >= %d <= 11", received_count_);
      FAIL();
    }

    RCLCPP_INFO(node_->get_logger(), "finished");
  }
} // namespace

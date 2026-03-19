#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

#include <chrono>
#include <mutex>
#include <vector>

using namespace mrs_lib::errorgraph;
using namespace std::chrono_literals;

class ErrorPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("error_publisher_test_node");
    clock_ = node_->get_clock();

    // Subscribe to the errors topic published by ErrorPublisher
    sub_ = node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>("~/errors", 10, [this](const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) {
      std::scoped_lock lck(mtx_);
      received_msgs_.push_back(*msg);
    });
  }

  void TearDown() override
  {
    publisher_.reset();
    sub_.reset();
    node_.reset();
  }

  /// \brief Create the ErrorPublisher with a fast publish rate for testing.
  void createPublisher(const rclcpp::Rate& rate = rclcpp::Rate(50.0))
  {
    publisher_ = std::make_unique<ErrorPublisher>(node_, clock_, "test_node", "test_component", rate);
  }

  /// \brief Spin the node until at least `count` messages are received or timeout.
  bool waitForMessages(size_t count, double timeout_s = 3.0)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(timeout_s))
    {
      rclcpp::spin_some(node_);
      {
        std::scoped_lock lck(mtx_);
        if (received_msgs_.size() >= count)
          return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  /// \brief Get a copy of received messages.
  std::vector<mrs_msgs::msg::ErrorgraphElement> getReceivedMessages()
  {
    std::scoped_lock lck(mtx_);
    return received_msgs_;
  }

  /// \brief Clear received messages.
  void clearReceivedMessages()
  {
    std::scoped_lock lck(mtx_);
    received_msgs_.clear();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<ErrorPublisher> publisher_;
  rclcpp::Subscription<mrs_msgs::msg::ErrorgraphElement>::SharedPtr sub_;
  std::mutex mtx_;
  std::vector<mrs_msgs::msg::ErrorgraphElement> received_msgs_;
};

/* TEST: publishes messages with correct source node //{ */

TEST_F(ErrorPublisherTest, publishes_with_correct_source_node)
{
  createPublisher();
  publisher_->addOneshotError("some error");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  ASSERT_GE(msgs.size(), 1);

  // Find a message that contains our error (first message may be empty if timer fires before addOneshotError)
  bool found = false;
  for (const auto& msg : msgs)
  {
    EXPECT_EQ(msg.source_node.node, "test_node");
    EXPECT_EQ(msg.source_node.component, "test_component");
    if (!msg.errors.empty())
      found = true;
  }
  EXPECT_TRUE(found);
}

//}

/* TEST: publishes no-error message when no errors added //{ */

TEST_F(ErrorPublisherTest, publishes_empty_errors_when_none_added)
{
  createPublisher();

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  ASSERT_GE(msgs.size(), 1);
  // At least the first message should have an empty errors list (no errors were added)
  EXPECT_TRUE(msgs[0].errors.empty());
}

//}

/* TEST: addGeneralError appears in published message //{ */

TEST_F(ErrorPublisherTest, add_general_error_appears_in_message)
{
  createPublisher();
  publisher_->addGeneralError(0, "sensor_failure");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  bool found = false;
  for (const auto& msg : msgs)
  {
    for (const auto& error : msg.errors)
    {
      if (error.type == "sensor_failure")
      {
        found = true;
        break;
      }
    }
  }
  EXPECT_TRUE(found);
}

//}

/* TEST: addGeneralError deduplicates by id //{ */

TEST_F(ErrorPublisherTest, add_general_error_deduplicates_by_id)
{
  createPublisher(rclcpp::Rate(2.0)); // Slow publish rate so errors accumulate
  publisher_->addGeneralError(1, "first_description");
  publisher_->addGeneralError(1, "updated_description");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  // Find a message with errors
  for (const auto& msg : msgs)
  {
    if (msg.errors.empty())
      continue;

    // Should have exactly one error with id 1, not two
    int count = 0;
    for (const auto& error : msg.errors)
    {
      if (error.type == "first_description" || error.type == "updated_description")
        count++;
    }
    // The id-based dedup should keep only one entry
    EXPECT_EQ(count, 1);
    // And it should be the updated one
    bool has_updated = false;
    for (const auto& error : msg.errors)
    {
      if (error.type == "updated_description")
        has_updated = true;
    }
    EXPECT_TRUE(has_updated);
    break;
  }
}

//}

/* TEST: addGeneralError keeps different ids separate //{ */

TEST_F(ErrorPublisherTest, add_general_error_different_ids_kept_separate)
{
  createPublisher(rclcpp::Rate(2.0));
  publisher_->addGeneralError(1, "error_one");
  publisher_->addGeneralError(2, "error_two");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  for (const auto& msg : msgs)
  {
    if (msg.errors.size() >= 2)
    {
      bool found_one = false, found_two = false;
      for (const auto& error : msg.errors)
      {
        if (error.type == "error_one")
          found_one = true;
        if (error.type == "error_two")
          found_two = true;
      }
      EXPECT_TRUE(found_one);
      EXPECT_TRUE(found_two);
      break;
    }
  }
}

//}

/* TEST: addOneshotError does not deduplicate //{ */

TEST_F(ErrorPublisherTest, add_oneshot_error_no_dedup)
{
  createPublisher(rclcpp::Rate(2.0));
  publisher_->addOneshotError("duplicate_error");
  publisher_->addOneshotError("duplicate_error");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  for (const auto& msg : msgs)
  {
    int count = 0;
    for (const auto& error : msg.errors)
    {
      if (error.type == "duplicate_error")
        count++;
    }
    if (count > 0)
    {
      // Both oneshot errors should be present
      EXPECT_EQ(count, 2);
      break;
    }
  }
}

//}

/* TEST: addWaitingForNodeError appears with correct type and node_id //{ */

TEST_F(ErrorPublisherTest, add_waiting_for_node_error)
{
  createPublisher(rclcpp::Rate(2.0));
  node_id_t waited_node{"other_node", "other_component"};
  publisher_->addWaitingForNodeError(waited_node);

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  bool found = false;
  for (const auto& msg : msgs)
  {
    for (const auto& error : msg.errors)
    {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE)
      {
        EXPECT_EQ(error.waited_for_node.node, "other_node");
        EXPECT_EQ(error.waited_for_node.component, "other_component");
        found = true;
      }
    }
  }
  EXPECT_TRUE(found);
}

//}

/* TEST: addWaitingForNodeError deduplicates by node_id //{ */

TEST_F(ErrorPublisherTest, add_waiting_for_node_error_deduplicates)
{
  createPublisher(rclcpp::Rate(2.0));
  node_id_t waited_node{"dep_node", "dep_comp"};
  publisher_->addWaitingForNodeError(waited_node);
  publisher_->addWaitingForNodeError(waited_node);

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  for (const auto& msg : msgs)
  {
    int count = 0;
    for (const auto& error : msg.errors)
    {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE && error.waited_for_node.node == "dep_node")
        count++;
    }
    if (count > 0)
    {
      EXPECT_EQ(count, 1);
      break;
    }
  }
}

//}

/* TEST: addWaitingForTopicError appears with correct type and topic //{ */

TEST_F(ErrorPublisherTest, add_waiting_for_topic_error)
{
  createPublisher(rclcpp::Rate(2.0));
  publisher_->addWaitingForTopicError("/some/topic");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  bool found = false;
  for (const auto& msg : msgs)
  {
    for (const auto& error : msg.errors)
    {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_TOPIC)
      {
        EXPECT_EQ(error.waited_for_topic, "/some/topic");
        found = true;
      }
    }
  }
  EXPECT_TRUE(found);
}

//}

/* TEST: addWaitingForTopicError deduplicates by topic name //{ */

TEST_F(ErrorPublisherTest, add_waiting_for_topic_error_deduplicates)
{
  createPublisher(rclcpp::Rate(2.0));
  publisher_->addWaitingForTopicError("/dup/topic");
  publisher_->addWaitingForTopicError("/dup/topic");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  for (const auto& msg : msgs)
  {
    int count = 0;
    for (const auto& error : msg.errors)
    {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_TOPIC && error.waited_for_topic == "/dup/topic")
        count++;
    }
    if (count > 0)
    {
      EXPECT_EQ(count, 1);
      break;
    }
  }
}

//}

/* TEST: errors are cleared after publish //{ */

TEST_F(ErrorPublisherTest, errors_cleared_after_publish)
{
  createPublisher(rclcpp::Rate(50.0)); // Fast rate
  publisher_->addOneshotError("transient_error");

  // Wait for first message with the error
  ASSERT_TRUE(waitForMessages(1));

  // Clear and wait for more messages — subsequent ones should be empty
  clearReceivedMessages();
  ASSERT_TRUE(waitForMessages(2));

  auto msgs = getReceivedMessages();
  // At least one of the subsequent messages should have no errors
  bool found_empty = false;
  for (const auto& msg : msgs)
  {
    if (msg.errors.empty())
    {
      found_empty = true;
      break;
    }
  }
  EXPECT_TRUE(found_empty);
}

//}

/* TEST: addGeneralError with enum type //{ */

TEST_F(ErrorPublisherTest, add_general_error_with_enum)
{
  enum class TestError : uint8_t
  {
    SENSOR_FAIL = 0,
    COMM_FAIL = 1,
  };

  createPublisher(rclcpp::Rate(2.0));
  publisher_->addGeneralError(TestError::SENSOR_FAIL, "sensor_failed");
  publisher_->addGeneralError(TestError::COMM_FAIL, "comm_failed");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  bool found_sensor = false, found_comm = false;
  for (const auto& msg : msgs)
  {
    for (const auto& error : msg.errors)
    {
      if (error.type == "sensor_failed")
        found_sensor = true;
      if (error.type == "comm_failed")
        found_comm = true;
    }
  }
  EXPECT_TRUE(found_sensor);
  EXPECT_TRUE(found_comm);
}

//}

/* TEST: addGeneralError with enum deduplicates same as integer id //{ */

TEST_F(ErrorPublisherTest, add_general_error_enum_deduplicates)
{
  enum class TestError : uint8_t
  {
    MY_ERROR = 5,
  };

  createPublisher(rclcpp::Rate(2.0));
  publisher_->addGeneralError(TestError::MY_ERROR, "old_desc");
  publisher_->addGeneralError(TestError::MY_ERROR, "new_desc");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  for (const auto& msg : msgs)
  {
    if (msg.errors.empty())
      continue;
    bool has_old = false, has_new = false;
    for (const auto& error : msg.errors)
    {
      if (error.type == "old_desc")
        has_old = true;
      if (error.type == "new_desc")
        has_new = true;
    }
    if (has_new || has_old)
    {
      EXPECT_FALSE(has_old);
      EXPECT_TRUE(has_new);
      break;
    }
  }
}

//}

/* TEST: mixed error types in single publish //{ */

TEST_F(ErrorPublisherTest, mixed_error_types_single_publish)
{
  createPublisher(rclcpp::Rate(2.0));
  publisher_->addGeneralError(0, "general_err");
  publisher_->addOneshotError("oneshot_err");
  publisher_->addWaitingForNodeError(node_id_t{"waited_node", "waited_comp"});
  publisher_->addWaitingForTopicError("/waited/topic");

  ASSERT_TRUE(waitForMessages(1));

  auto msgs = getReceivedMessages();
  bool found_general = false, found_oneshot = false, found_node = false, found_topic = false;
  for (const auto& msg : msgs)
  {
    for (const auto& error : msg.errors)
    {
      if (error.type == "general_err")
        found_general = true;
      if (error.type == "oneshot_err")
        found_oneshot = true;
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE)
        found_node = true;
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_TOPIC)
        found_topic = true;
    }
  }
  EXPECT_TRUE(found_general);
  EXPECT_TRUE(found_oneshot);
  EXPECT_TRUE(found_node);
  EXPECT_TRUE(found_topic);
}

//}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

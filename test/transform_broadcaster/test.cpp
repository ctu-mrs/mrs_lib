#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/transform_broadcaster.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_msgs/msg/tf_message.hpp>

#include <atomic>
#include <thread>

using namespace std::chrono_literals;

class Test : public ::testing::Test
{

protected:
  /* SetUpTestCase() //{ */

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  //}

  /* TearDownTestCase() //{ */

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  //}

  /* initialize() //{ */

  void initialize(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
  {

    node_ = std::make_shared<rclcpp::Node>("test_publisher_handler", node_options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    finished_future_ = finished_promise_.get_future();

    main_thread_ = std::thread(&Test::spin, this);
  }

  //}

  /* spin() //{ */

  void spin()
  {

    RCLCPP_INFO(node_->get_logger(), "starting spinning");

    executor_->spin();

    RCLCPP_INFO(node_->get_logger(), "stopped spinning");
  }

  //}

  /* despin() //{ */

  void despin()
  {
    executor_->cancel();

    main_thread_.join();
  }

  //}

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  std::promise<bool> finished_promise_;
  std::future<bool> finished_future_;

  mrs_lib::TransformBroadcaster tf_broadcaster_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

/* TEST_F(Test, test_exception) //{ */

TEST_F(Test, test_exception)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  clock->sleep_for(1s);

  geometry_msgs::msg::TransformStamped tf;

  tf.header.frame_id = "from";
  tf.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);

  tf.child_frame_id = "to";

  EXPECT_ANY_THROW(tf_broadcaster_.sendTransform(tf));

  despin();

  clock->sleep_for(1s);
}

//}

/* TEST_F(Test, test_broadcast) //{ */

TEST_F(Test, test_broadcast)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  tf_broadcaster_ = mrs_lib::TransformBroadcaster(node_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  clock->sleep_for(1s);

  // | ------------------------- publish ------------------------ |

  geometry_msgs::msg::TransformStamped tf;

  tf.header.frame_id = "from";
  tf.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);

  tf.child_frame_id = "to";

  for (int i = 0; i < 100; i++)
  {

    if (i % 10 == 0)
    {
      tf.header.stamp = rclcpp::Time(10 + i / 10, 0, RCL_ROS_TIME);
    }

    tf_broadcaster_.sendTransform(tf);

    clock->sleep_for(0.01s);
  }

  // check for the transsform

  try
  {
    const geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("from", "to", rclcpp::Time(0));

    EXPECT_TRUE(true);
  }
  catch (tf2::TransformException& ex)
  {

    RCLCPP_WARN_STREAM(node_->get_logger(), "" << ex.what());

    EXPECT_FALSE(true);
  }

  despin();

  clock->sleep_for(1s);
}

//}

/* TEST_F(Test, test_broadcast_vector) //{ */

TEST_F(Test, test_broadcast_vector)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  tf_broadcaster_ = mrs_lib::TransformBroadcaster(node_);

  std::atomic<int> n_messages = 0;
  std::atomic<int> n_transforms = 0;

  // other tests or a running system may publish on /tf too
  auto sub = node_->create_subscription<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicListenerQoS(), [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    if (msg->transforms.empty() || msg->transforms.front().header.frame_id != "test_broadcast_vector")
    {
      return;
    }
    n_messages++;
    n_transforms += msg->transforms.size();
  });

  clock->sleep_for(1s);

  // | ------------------------- publish ------------------------ |

  std::vector<geometry_msgs::msg::TransformStamped> tfs;

  for (const auto& child : {"a", "b", "c"})
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "test_broadcast_vector";
    tf.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
    tf.child_frame_id = child;
    tf.transform.rotation.w = 1.0;
    tfs.push_back(tf);
  }

  // all three transforms go out in a single message
  tf_broadcaster_.sendTransform(tfs);

  clock->sleep_for(1s);

  EXPECT_EQ(n_messages, 1);
  EXPECT_EQ(n_transforms, 3);

  // a repeated timestamp is skipped, the rest still goes out in one message
  tfs.at(1).header.stamp = rclcpp::Time(11, 0, RCL_ROS_TIME);
  tfs.at(2).header.stamp = rclcpp::Time(11, 0, RCL_ROS_TIME);

  tf_broadcaster_.sendTransform(tfs);

  clock->sleep_for(1s);

  EXPECT_EQ(n_messages, 2);
  EXPECT_EQ(n_transforms, 5);

  // nothing passes the check, so nothing is published
  tf_broadcaster_.sendTransform(tfs);

  clock->sleep_for(1s);

  EXPECT_EQ(n_messages, 2);
  EXPECT_EQ(n_transforms, 5);

  despin();

  clock->sleep_for(1s);
}

//}

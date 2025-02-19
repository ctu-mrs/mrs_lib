#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>

#include <thread>

using namespace std::chrono_literals;

class Test : public ::testing::Test {

public:
  /* callback1() //{ */

  void callback1(const std_msgs::msg::Int64::ConstSharedPtr msg) {

    RCLCPP_INFO(node_->get_logger(), "message received");

    if (msg->data == num_to_send) {
      num_received++;
    }
  }

  //}

  /* timeoutCallback() //{ */

  void timeoutCallback([[maybe_unused]] const std::string& topic_name, [[maybe_unused]] const rclcpp::Time& last_msg) {

    timeouted_ = true;
  }

  //}

protected:
  /* SetUpTestCase() //{ */

  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
  }

  //}

  /* TearDownTestCase() //{ */

  static void TearDownTestCase() {
    rclcpp::shutdown();
  }

  //}

  /* initialize() //{ */

  void initialize(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) {

    node_ = std::make_shared<rclcpp::Node>("test_publisher_handler", node_options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    finished_future_ = finished_promise_.get_future();

    main_thread_ = std::thread(&Test::spin, this);
  }

  //}

  /* spin() //{ */

  void spin() {

    RCLCPP_INFO(node_->get_logger(), "starting spinning");

    executor_->spin();

    RCLCPP_INFO(node_->get_logger(), "stopped spinning");
  }

  //}

  /* despin() //{ */

  void despin() {
    executor_->cancel();

    main_thread_.join();
  }

  //}

  rclcpp::Node::SharedPtr                              node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  int num_received = 0;

  std::atomic<bool> timeouted_ = false;

  int num_to_send = 1234;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;
};

/* TEST_F(Test, polled) //{ */

TEST_F(Test, polled) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "creating publisher");

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  RCLCPP_INFO(node_->get_logger(), "creating subscriber");

  mrs_lib::SubscriberHandlerOptions sh_opts(node_);

  sh_opts.autostart = true;

  mrs_lib::SubscriberHandler<std_msgs::msg::Int64> sh_int_;

  sh_int_ = mrs_lib::SubscriberHandler<std_msgs::msg::Int64>(sh_opts, "/topic1");

  // | ---------------------- start testing --------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  {
    rclcpp::Rate rate(1.0, clock);

    for (int i = 0; i < 10; i++) {

      if (sh_int_.getNumPublishers() > 0) {
        break;
      }

      rate.sleep();
    }
  }

  if (sh_int_.getNumPublishers() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
    EXPECT_TRUE(false);
  }

  clock->sleep_for(1s);

  std_msgs::msg::Int64 data;
  data.data = num_to_send;

  RCLCPP_INFO(node_->get_logger(), "publishing");

  ph_int.publish(data);

  clock->sleep_for(1s);

  EXPECT_TRUE(sh_int_.hasMsg());

  auto msg = sh_int_.getMsg();

  EXPECT_EQ(num_to_send, msg->data);

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();
}

//}

/* TEST_F(Test, callback) //{ */

TEST_F(Test, callback) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "creating publisher");

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  RCLCPP_INFO(node_->get_logger(), "creating subscriber");

  mrs_lib::SubscriberHandlerOptions sh_opts(node_);

  sh_opts.autostart = true;

  mrs_lib::SubscriberHandler<std_msgs::msg::Int64> sh_int_;

  sh_int_ = mrs_lib::SubscriberHandler<std_msgs::msg::Int64>(sh_opts, "/topic1", std::bind(&Test::callback1, this, std::placeholders::_1));

  // | ---------------------- start testing --------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  {
    rclcpp::Rate rate(1.0, clock);

    for (int i = 0; i < 10; i++) {

      if (sh_int_.getNumPublishers() > 0) {
        break;
      }

      rate.sleep();
    }
  }

  if (sh_int_.getNumPublishers() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
    EXPECT_TRUE(false);
  }

  clock->sleep_for(1s);

  std_msgs::msg::Int64 data;
  data.data = num_to_send;

  RCLCPP_INFO(node_->get_logger(), "publishing");

  ph_int.publish(data);

  clock->sleep_for(1s);

  EXPECT_GE(num_received, 1);

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();
}

//}

/* TEST_F(Test, timeout) //{ */

TEST_F(Test, timeout) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "creating publisher");

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  RCLCPP_INFO(node_->get_logger(), "creating subscriber");

  mrs_lib::SubscriberHandlerOptions sh_opts(node_);

  sh_opts.autostart          = true;
  sh_opts.no_message_timeout = rclcpp::Duration(std::chrono::duration<double>(0.5));

  mrs_lib::SubscriberHandler<std_msgs::msg::Int64> sh_int_;

  sh_int_ = mrs_lib::SubscriberHandler<std_msgs::msg::Int64>(sh_opts, "/topic1",
                                                             std::bind(&Test::timeoutCallback, this, std::placeholders::_1, std::placeholders::_2));

  // | ---------------------- start testing --------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  {
    rclcpp::Rate rate(0.1, clock);

    for (int i = 0; i < 10; i++) {

      if (sh_int_.getNumPublishers() > 0) {
        break;
      }

      rate.sleep();
    }
  }

  if (sh_int_.getNumPublishers() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
    EXPECT_TRUE(false);
  }

  RCLCPP_INFO(node_->get_logger(), "publishing");

  {
    rclcpp::Rate rate(100.0, clock);

    for (int i = 0; i < 100; i++) {

      std_msgs::msg::Int64 data;
      data.data = num_to_send;

      ph_int.publish(data);

      EXPECT_FALSE(timeouted_);

      rate.sleep();
    }
  }

  RCLCPP_INFO(node_->get_logger(), "stopping publisher");

  clock->sleep_for(1s);

  EXPECT_TRUE(timeouted_);

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();
}

//}

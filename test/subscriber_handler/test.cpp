#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>

#include <cstdlib>
#include <future>
#include <thread>

using namespace std::chrono_literals;

class Test : public ::testing::Test
{

public:
  /* callback1() //{ */

  void callback1(const std_msgs::msg::Int64::ConstSharedPtr msg)
  {

    RCLCPP_INFO(node_->get_logger(), "message received");

    if (msg->data == num_to_send)
    {
      num_received++;
    }
  }

  //}

  /* timeoutCallback() //{ */

  void timeoutCallback([[maybe_unused]] const std::string& topic_name, [[maybe_unused]] const rclcpp::Time& last_msg)
  {

    timeouted_ = true;
  }

  //}

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

  int num_received = 0;

  std::atomic<bool> timeouted_ = false;

  int num_to_send = 1234;

  std::promise<bool> finished_promise_;
  std::future<bool> finished_future_;
};

/* TEST_F(Test, polled) //{ */

TEST_F(Test, polled)
{

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

    for (int i = 0; i < 10; i++)
    {

      if (sh_int_.getNumPublishers() > 0)
      {
        break;
      }

      rate.sleep();
    }
  }

  if (sh_int_.getNumPublishers() == 0)
  {
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

TEST_F(Test, callback)
{

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

    for (int i = 0; i < 10; i++)
    {

      if (sh_int_.getNumPublishers() > 0)
      {
        break;
      }

      rate.sleep();
    }
  }

  if (sh_int_.getNumPublishers() == 0)
  {
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

TEST_F(Test, timeout)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "creating publisher");

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  RCLCPP_INFO(node_->get_logger(), "creating subscriber");

  mrs_lib::SubscriberHandlerOptions sh_opts(node_);

  sh_opts.autostart = true;
  sh_opts.no_message_timeout = rclcpp::Duration(std::chrono::duration<double>(0.5));

  mrs_lib::SubscriberHandler<std_msgs::msg::Int64> sh_int_;

  sh_int_ = mrs_lib::SubscriberHandler<std_msgs::msg::Int64>(sh_opts, "/topic1",
                                                             std::bind(&Test::timeoutCallback, this, std::placeholders::_1, std::placeholders::_2));

  // | ---------------------- start testing --------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  {
    rclcpp::Rate rate(0.1, clock);

    for (int i = 0; i < 10; i++)
    {

      if (sh_int_.getNumPublishers() > 0)
      {
        break;
      }

      rate.sleep();
    }
  }

  if (sh_int_.getNumPublishers() == 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
    EXPECT_TRUE(false);
  }

  RCLCPP_INFO(node_->get_logger(), "publishing");

  {
    rclcpp::Rate rate(100.0, clock);

    for (int i = 0; i < 100; i++)
    {

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

/* TEST_F(Test, default_timeout_callback_before_start) //{ */

TEST_F(Test, default_timeout_callback_before_start)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ------------------- create a subscriber ------------------ |

  mrs_lib::SubscriberHandlerOptions sh_opts(node_);

  sh_opts.autostart = false;
  sh_opts.no_message_timeout = rclcpp::Duration(std::chrono::duration<double>(0.1));

  mrs_lib::SubscriberHandler<std_msgs::msg::Int64> sh_int_(sh_opts, "/topic1");

  // | ---------------------- start testing --------------------- |

  // the timeout must not touch the missing subscription before start() and after stop()
  clock->sleep_for(500ms);

  sh_int_.start();

  clock->sleep_for(300ms);

  sh_int_.stop();

  sh_int_.setNoMessageTimeout(rclcpp::Duration(std::chrono::duration<double>(0.1)));

  clock->sleep_for(300ms);

  SUCCEED();

  despin();
}

//}

/* TEST_F(Test, custom_timeout_callback_calls_handler) //{ */

TEST_F(Test, custom_timeout_callback_calls_handler)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  mrs_lib::SubscriberHandler<std_msgs::msg::Int64> sh_int_;

  std::atomic<bool> probed = false;
  std::promise<void> probe_done;

  mrs_lib::SubscriberHandlerOptions sh_opts(node_);

  sh_opts.autostart = false;
  sh_opts.threadsafe = true;
  sh_opts.no_message_timeout = rclcpp::Duration(std::chrono::duration<double>(0.2));

  // runs under the TimeoutManager lock: let a message reach data_callback() first, then call the handler
  sh_opts.timeout_callback = [&]([[maybe_unused]] const std::string& topic_name, [[maybe_unused]] const rclcpp::Time& last_msg) {
    if (probed.exchange(true))
      return;

    std_msgs::msg::Int64 data;
    data.data = num_to_send;
    ph_int.publish(data);

    std::this_thread::sleep_for(200ms);

    sh_int_.lastMsgTime();

    probe_done.set_value();
  };

  sh_int_ = mrs_lib::SubscriberHandler<std_msgs::msg::Int64>(sh_opts, "/topic1");

  sh_int_.start();

  for (int i = 0; i < 50 && sh_int_.getNumPublishers() == 0; i++)
  {
    clock->sleep_for(100ms);
  }

  ASSERT_GT(sh_int_.getNumPublishers(), 0u) << "failed to connect publisher and subscriber";

  // | ---------------------- start testing --------------------- |

  if (probe_done.get_future().wait_for(5s) != std::future_status::ready)
  {
    // the threads are deadlocked, so the test cannot clean up
    ADD_FAILURE() << "the timeout callback deadlocked with data_callback()";
    std::_Exit(1);
  }

  clock->sleep_for(300ms);

  EXPECT_TRUE(sh_int_.hasMsg());

  despin();
}

//}

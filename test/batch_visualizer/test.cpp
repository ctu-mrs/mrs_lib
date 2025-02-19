#include <cmath>

#include <cstddef>
#include <gtest/gtest.h>

#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/batch_visualizer.h>

#include <string>
#include <thread>
#include <random>

using namespace std::chrono_literals;
using namespace mrs_lib;
using namespace mrs_lib::geometry;

class Test : public ::testing::Test {

public:
  /* callback1() //{ */

  void callback1(visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    if (msg.get() != nullptr) {
      RCLCPP_INFO(node_->get_logger(), "Message received");
      received_msg_ = msg;
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "Message empty");
    }
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

    node_ = std::make_shared<rclcpp::Node>("test_batch_visualizer", node_options);

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

  void test_batch_visualizer_init(const std::string& topic_name, const std::string& ns_name, const double point_scale = 0.02, const double line_scale = 0.04) {
    EXPECT_EQ(received_msg_->markers.size(), 3);

    EXPECT_EQ(received_msg_->markers[0].header.frame_id, ns_name);
    EXPECT_EQ(received_msg_->markers[0].ns, topic_name);
    EXPECT_EQ(received_msg_->markers[0].id, 8);
    EXPECT_EQ(received_msg_->markers[0].points.size(), 0);
    EXPECT_EQ(received_msg_->markers[0].colors.size(), 0);
    EXPECT_DOUBLE_EQ(received_msg_->markers[0].scale.x, point_scale);
    EXPECT_DOUBLE_EQ(received_msg_->markers[0].scale.y, point_scale);

    EXPECT_EQ(received_msg_->markers[1].header.frame_id, ns_name);
    EXPECT_EQ(received_msg_->markers[1].ns, topic_name);
    EXPECT_EQ(received_msg_->markers[1].id, 5);
    EXPECT_EQ(received_msg_->markers[1].points.size(), 0);
    EXPECT_EQ(received_msg_->markers[1].colors.size(), 0);
    EXPECT_DOUBLE_EQ(received_msg_->markers[1].scale.x, line_scale);

    EXPECT_EQ(received_msg_->markers[2].header.frame_id, ns_name);
    EXPECT_EQ(received_msg_->markers[2].ns, topic_name);
    EXPECT_EQ(received_msg_->markers[2].id, 11);
    EXPECT_EQ(received_msg_->markers[2].points.size(), 0);
    EXPECT_EQ(received_msg_->markers[2].colors.size(), 0);
  }

  rclcpp::Node::SharedPtr                              node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;

  double range_min_ = -20;
  double range_max_ = 20;

  // std::mt19937                           generator_(0);
  // std::uniform_real_distribution<double> rand_dbl_(range_min_, range_max_);
  // std::uniform_real_distribution<double> rand_percent_(0.0, 1.0);

  visualization_msgs::msg::MarkerArray::ConstSharedPtr received_msg_;
};

/* TEST_F(Test, batch_visualize_init) //{ */

TEST_F(Test, batch_visualize_init) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ------------------- create a subscriber ------------------ |

  RCLCPP_INFO(node_->get_logger(), "Creating subscriber");
  const std::string topic_name = "batch_points";

  const std::function<void(visualization_msgs::msg::MarkerArray::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

  auto sub = node_->create_subscription<visualization_msgs::msg::MarkerArray>(topic_name, 10, callback1_ptr);

  // | ---------------------- start testing --------------------- |

  // test if the object is clean just after init
  BatchVisualizer bv;
  std::string ns_name = "map";
  bv = BatchVisualizer(node_, topic_name, ns_name);

  {
    for (int i = 0; i < 10; i++) {

      if (sub->get_publisher_count() > 0) {
        RCLCPP_INFO(node_->get_logger(), "Connected publisher and subscriber");
        break;
      }

      clock->sleep_for(1s);
    }
  }

  if (sub->get_publisher_count() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to connect publisher and subscriber");
    despin();
    return;
  }

  bv.publish();
  // RCLCPP_INFO(node_->get_logger(), "Received msg cjk")
  EXPECT_TRUE(received_msg_.get() != nullptr);
  // EXPECT_TRUE(received_msg_);
  // test_batch_visualizer_init(topic_name, ns_name);
  // EXPECT_EQ(received_msg_->markers.size(), 3);

  // EXPECT_EQ(received_msg_->markers[0].header.frame_id, ns_name);
  // EXPECT_EQ(received_msg_->markers[0].ns, topic_name);
  // EXPECT_EQ(received_msg_->markers[0].id, 8);
  // EXPECT_EQ(received_msg_->markers[0].points.size(), 0);
  // EXPECT_EQ(received_msg_->markers[0].colors.size(), 0);
  // EXPECT_DOUBLE_EQ(received_msg_->markers[0].scale.x, 0.02);
  // EXPECT_DOUBLE_EQ(received_msg_->markers[0].scale.y, 0.02);

  // EXPECT_EQ(received_msg_->markers[1].header.frame_id, ns_name);
  // EXPECT_EQ(received_msg_->markers[1].ns, topic_name);
  // EXPECT_EQ(received_msg_->markers[1].id, 5);
  // EXPECT_EQ(received_msg_->markers[1].points.size(), 0);
  // EXPECT_EQ(received_msg_->markers[1].colors.size(), 0);
  // EXPECT_DOUBLE_EQ(received_msg_->markers[1].scale.x, 0.04);

  // EXPECT_EQ(received_msg_->markers[2].header.frame_id, ns_name);
  // EXPECT_EQ(received_msg_->markers[2].ns, topic_name);
  // EXPECT_EQ(received_msg_->markers[2].id, 11);
  // EXPECT_EQ(received_msg_->markers[2].points.size(), 0);
  // EXPECT_EQ(received_msg_->markers[2].colors.size(), 0);

  // bv.setPointsScale(0.5);
  // bv.setParentFrame("cat");
  // bv.setLinesScale(10.0);
  // bv.publish();
  // // EXPECT_TRUE(received_msg_);
  // test_batch_visualizer_init(topic_name, "cat", 0.5, 10.0);

  despin();

  clock->sleep_for(1s);
}

//}
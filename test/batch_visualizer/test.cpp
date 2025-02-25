#include <chrono>
#include <cmath>

#include <cstddef>
#include <filesystem>
#include <gtest/gtest.h>

#include <std_msgs/msg/color_rgba.hpp>
#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/batch_visualizer.h>

#include <string>
#include <thread>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

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

    auto clock = node_->get_clock();

    // | ------------------- create a subscriber ------------------ |

    RCLCPP_INFO(node_->get_logger(), "Creating subscriber");

    const std::function<void(visualization_msgs::msg::MarkerArray::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

    auto sub = node_->create_subscription<visualization_msgs::msg::MarkerArray>(viz_topic_name_, 10, callback1_ptr);

    // | ---------------------- start testing --------------------- |

    bv_ = BatchVisualizer(node_, viz_topic_name_, viz_ns_name_);
    for (int i = 0; i < 10; i++) {

      if (sub->get_publisher_count() > 0) {
        RCLCPP_INFO(node_->get_logger(), "Connected publisher and subscriber");
        break;
      }

      clock->sleep_for(100ms);
    }

    if (sub->get_publisher_count() == 0) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to connect publisher and subscriber");
      despin();
      return;
    }
    // clean the init msg published on object construction
    for (int i = 0; i < 10; i++) {
      if (received_msg_) {
        RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
        break;
      }
      clock->sleep_for(100ms);
    }
    EXPECT_TRUE(received_msg_);
    received_msg_.reset();
    RCLCPP_INFO(node_->get_logger(), "Cleaned the init message received from BatchVisualizer at object creation");
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

  void test_batch_visualizer_init(const std::string& viz_topic_name_, const std::string& viz_ns_name_, const double point_scale = 0.02, const double line_scale = 0.04) {
    EXPECT_EQ(received_msg_->markers.size(), 3);

    EXPECT_EQ(received_msg_->markers[0].header.frame_id, viz_ns_name_);
    EXPECT_EQ(received_msg_->markers[0].ns, viz_topic_name_ + "_points");
    EXPECT_EQ(received_msg_->markers[0].id, 8);
    EXPECT_EQ(received_msg_->markers[0].points.size(), 1);
    EXPECT_EQ(received_msg_->markers[0].colors.size(), 1);
    EXPECT_DOUBLE_EQ(received_msg_->markers[0].scale.x, point_scale);
    EXPECT_DOUBLE_EQ(received_msg_->markers[0].scale.y, point_scale);

    EXPECT_EQ(received_msg_->markers[1].header.frame_id, viz_ns_name_);
    EXPECT_EQ(received_msg_->markers[1].ns, viz_topic_name_ + "_lines");
    EXPECT_EQ(received_msg_->markers[1].id, 5);
    EXPECT_EQ(received_msg_->markers[1].points.size(), 2);
    EXPECT_EQ(received_msg_->markers[1].colors.size(), 2);
    EXPECT_DOUBLE_EQ(received_msg_->markers[1].scale.x, line_scale);

    EXPECT_EQ(received_msg_->markers[2].header.frame_id, viz_ns_name_);
    EXPECT_EQ(received_msg_->markers[2].ns, viz_topic_name_ + "_triangles");
    EXPECT_EQ(received_msg_->markers[2].id, 11);
    EXPECT_EQ(received_msg_->markers[2].points.size(), 3);
    EXPECT_EQ(received_msg_->markers[2].colors.size(), 3);
  }

  std::tuple<double, double, double> point_to_tuple(const geometry_msgs::msg::Point& point) {
      return std::make_tuple(point.x, point.y, point.z);
  }

  std::tuple<double, double, double, double> color_to_tuple(const std_msgs::msg::ColorRGBA& color) {
      return std::make_tuple(color.r, color.g, color.b, color.a);
  }

  std::vector<std::tuple<double, double, double>> make_vector_tuple(const std::vector<geometry_msgs::msg::Point>& points) {
    std::vector<std::tuple<double, double, double>> output;
    output.reserve(points.size());

    std::transform(points.begin(), points.end(), std::back_inserter(output), [](const auto& p) { return std::make_tuple(p.x, p.y, p.z); });
    return output;
  }

  std::vector<std::tuple<double, double, double, double>> make_vector_tuple(const std::vector<std_msgs::msg::ColorRGBA>& colors) {
    std::vector<std::tuple<double, double, double, double>> output;
    output.reserve(colors.size());

    std::transform(colors.begin(), colors.end(), std::back_inserter(output), [](const auto& p) { return std::make_tuple(p.r, p.g, p.b, p.a); });
    return output;
  }

  static bool compare_tuple3(const std::tuple<double, double, double>& lhs, const std::tuple<double, double, double>& rhs) {
  const double epsilon = 1e-4;  // Tolerance for floating-point comparison
    
    return std::abs(std::get<0>(lhs) - std::get<0>(rhs)) < epsilon && std::abs(std::get<1>(lhs) - std::get<1>(rhs)) < epsilon && std::abs(std::get<2>(lhs) - std::get<2>(rhs)) < epsilon;
  }

  static bool compare_tuple4(const std::tuple<double, double, double, double>& lhs, const std::tuple<double, double, double, double>& rhs) {
  const double epsilon = 1e-4;  // Tolerance for floating-point comparison
    
    return std::abs(std::get<0>(lhs) - std::get<0>(rhs)) < epsilon
     && std::abs(std::get<1>(lhs) - std::get<1>(rhs)) < epsilon
     && std::abs(std::get<2>(lhs) - std::get<2>(rhs)) < epsilon
     && std::abs(std::get<3>(lhs) - std::get<3>(rhs)) < epsilon;
  }

  rclcpp::Node::SharedPtr                              node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;

  double range_min_ = -20.0;
  double range_max_ = 20.0;

  std::mt19937                           generator_{10};
  std::uniform_real_distribution<> rand_dbl_{range_min_, range_max_};
  std::uniform_real_distribution<> rand_percent_{0.0, 1.0};

  visualization_msgs::msg::MarkerArray::ConstSharedPtr received_msg_;
  BatchVisualizer bv_;
  std::string viz_ns_name_ = "map";
  std::string viz_topic_name_ = "batch_viz";
  int batch_size_ = 100;
};

TEST_F(Test, batch_visualize_init) {

  // use_intra_process_comms() reduces connection latency when subscribers and publishers belong to the same context
  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);
  bv_.publish();
  // the callback is not called instantly
  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }
  // test if the object is clean just after init
  EXPECT_TRUE(received_msg_);
  test_batch_visualizer_init(viz_topic_name_, viz_ns_name_);
  RCLCPP_INFO(node_->get_logger(), "Tested the init values");
  received_msg_.reset();

  EXPECT_FALSE(received_msg_);
  bv_.setParentFrame("cat");
  bv_.publish();
  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    bv_.publish();
    clock->sleep_for(100ms);
  }
  EXPECT_TRUE(received_msg_);
  test_batch_visualizer_init(viz_topic_name_, "cat");
  RCLCPP_INFO(node_->get_logger(), "Tested the 'cat' in 'ns'");
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

// TEST_F(Test, batch_visualize_points) {

//   initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

//   auto clock = node_->get_clock();

//   // | ------------------- create a subscriber ------------------ |

//   RCLCPP_INFO(node_->get_logger(), "Creating subscriber");
//   const std::string viz_topic_name_ = "batch_viz";

//   const std::function<void(visualization_msgs::msg::MarkerArray::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

//   auto sub = node_->create_subscription<visualization_msgs::msg::MarkerArray>(viz_topic_name_, 10, callback1_ptr);

//   // | ---------------------- start testing --------------------- |

//   BatchVisualizer bv_;
//   std::string viz_ns_name_ = "map";
//   bv_ = BatchVisualizer(node_, viz_topic_name_, viz_ns_name_);

//   {
//     for (int i = 0; i < 10; i++) {

//       if (sub->get_publisher_count() > 0) {
//         RCLCPP_INFO(node_->get_logger(), "Connected publisher and subscriber");
//         break;
//       }

//       clock->sleep_for(100ms);
//     }
//   }

//   if (sub->get_publisher_count() == 0) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to connect publisher and subscriber");
//     despin();
//     return;
//   }
//   // clean the init msg published on object construction
//   for (int i = 0; i < 10; i++) {
//     if (received_msg_) {
//       RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
//       break;
//     }
//     clock->sleep_for(100ms);
//   }
//   EXPECT_TRUE(received_msg_);
//   received_msg_.reset();

//   RCLCPP_INFO(node_->get_logger(), "Generating random points");
//   std::vector<std::tuple<double, double, double>> points;
//   std::vector<std::tuple<double, double, double, double>> colors;
//   for (int i = 0; i < 100; i++) {
//     double          x = rand_dbl_(generator_);
//     double          y = rand_dbl_(generator_);
//     double          z = rand_dbl_(generator_);
//     Eigen::Vector3d point(x, y, z);
//     double          r = (x - range_min_) / (range_max_ - range_min_);
//     double          g = (y - range_min_) / (range_max_ - range_min_);
//     double          b = (z - range_min_) / (range_max_ - range_min_);
//     double          a = rand_percent_(generator_);
//     bv_.addPoint(point, r, g, b, a, rclcpp::Duration{std::chrono::nanoseconds(1s)});
//     points.emplace_back(std::make_tuple(x, y, z));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//   }
//   bv_.publish();

//   for (int i = 0; i < 10; i++) {
//     if (received_msg_) {
//       RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
//       break;
//     }
//     clock->sleep_for(100ms);
//   }

//   EXPECT_TRUE(received_msg_);

//   EXPECT_EQ(received_msg_->markers[0].points.size(), points.size());
//   EXPECT_EQ(received_msg_->markers[0].colors.size(), colors.size());

//   auto tmp_pts = make_vector_tuple(received_msg_->markers[0].points);
//   EXPECT_TRUE(std::equal(tmp_pts.begin(), tmp_pts.end(), points.begin(), points.end(), compare_tuple3));
//   auto tmp_colors = make_vector_tuple(received_msg_->markers[0].colors);
//   EXPECT_TRUE(std::equal(tmp_colors.begin(), tmp_colors.end(), colors.begin(), colors.end(), compare_tuple4));

//   EXPECT_EQ(received_msg_->markers[1].points.size(), 2);
//   EXPECT_EQ(received_msg_->markers[1].colors.size(), 2);
//   EXPECT_EQ(received_msg_->markers[2].points.size(), 3);
//   EXPECT_EQ(received_msg_->markers[2].colors.size(), 3);

//   received_msg_.reset();

//   despin();

//   clock->sleep_for(1s);
// }

// TEST_F(Test, batch_visualize_lines) {

//   initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

//   auto clock = node_->get_clock();

//   // | ------------------- create a subscriber ------------------ |

//   RCLCPP_INFO(node_->get_logger(), "Creating subscriber");
//   const std::string viz_topic_name_ = "batch_viz";

//   const std::function<void(visualization_msgs::msg::MarkerArray::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

//   auto sub = node_->create_subscription<visualization_msgs::msg::MarkerArray>(viz_topic_name_, 10, callback1_ptr);

//   // | ---------------------- start testing --------------------- |

//   BatchVisualizer bv_;
//   std::string viz_ns_name_ = "map";
//   bv_ = BatchVisualizer(node_, viz_topic_name_, viz_ns_name_);
//   int elem_count = 100;

//   {
//     for (int i = 0; i < 10; i++) {

//       if (sub->get_publisher_count() > 0) {
//         RCLCPP_INFO(node_->get_logger(), "Connected publisher and subscriber");
//         break;
//       }

//       clock->sleep_for(100ms);
//     }
//   }

//   if (sub->get_publisher_count() == 0) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to connect publisher and subscriber");
//     despin();
//     return;
//   }
//   // clean the init msg published on object construction
//   for (int i = 0; i < 10; i++) {
//     if (received_msg_) {
//       RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
//       break;
//     }
//     clock->sleep_for(100ms);
//   }
//   EXPECT_TRUE(received_msg_);
//   received_msg_.reset();

//   RCLCPP_INFO(node_->get_logger(), "Generating random rays");
//   std::vector<std::tuple<double, double, double>> points;
//   std::vector<std::tuple<double, double, double, double>> colors;
//   for (int i = 0; i < elem_count; i++) {
//     double          x1 = rand_dbl_(generator_);
//     double          y1 = rand_dbl_(generator_);
//     double          z1 = rand_dbl_(generator_);
//     Eigen::Vector3d point1(x1, y1, z1);
//     double          x2 = rand_dbl_(generator_);
//     double          y2 = rand_dbl_(generator_);
//     double          z2 = rand_dbl_(generator_);
//     Eigen::Vector3d point2(x2, y2, z2);
//     Ray             ray = Ray::twopointCast(point1, point2);
//     double          r   = ((x1 * x2) - range_min_) / (range_max_ - range_min_);
//     double          g   = ((y1 * y2) - range_min_) / (range_max_ - range_min_);
//     double          b   = ((z1 * z2) - range_min_) / (range_max_ - range_min_);
//     double          a   = rand_percent_(generator_);
//     bv_.addRay(ray, r, g, b, a, rclcpp::Duration{std::chrono::nanoseconds(1s)});
//     points.emplace_back(std::make_tuple(x1, y1, z1));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//     points.emplace_back(std::make_tuple(x2, y2, z2));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//   }
//   bv_.publish();

//   for (int i = 0; i < 10; i++) {
//     if (received_msg_) {
//       RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
//       break;
//     }
//     clock->sleep_for(100ms);
//   }

//   EXPECT_TRUE(received_msg_);

//   EXPECT_EQ(received_msg_->markers[1].points.size(), points.size());
//   EXPECT_EQ(received_msg_->markers[1].colors.size(), colors.size());

//   auto tmp_pts = make_vector_tuple(received_msg_->markers[1].points);
//   EXPECT_TRUE(std::equal(tmp_pts.begin(), tmp_pts.end(), points.begin(), points.end(), compare_tuple3));
//   auto tmp_colors = make_vector_tuple(received_msg_->markers[1].colors);
//   EXPECT_TRUE(std::equal(tmp_colors.begin(), tmp_colors.end(), colors.begin(), colors.end(), compare_tuple4));

//   EXPECT_EQ(received_msg_->markers[0].points.size(), 1);
//   EXPECT_EQ(received_msg_->markers[0].colors.size(), 1);
//   EXPECT_EQ(received_msg_->markers[2].points.size(), 3);
//   EXPECT_EQ(received_msg_->markers[2].colors.size(), 3);

//   received_msg_.reset();

//   RCLCPP_INFO(node_->get_logger(), "Generating random rays starting from origin");
//   for (int i = 0; i < elem_count; i++) {
//     double          x1 = rand_dbl_(generator_);
//     double          y1 = rand_dbl_(generator_);
//     double          z1 = rand_dbl_(generator_);
//     Eigen::Vector3d point(x1, y1, z1);
//     Ray             ray = Ray::twopointCast(Eigen::Vector3d::Zero(), point);
//     double          r   = ((x1 * 0.0) - range_min_) / (range_max_ - range_min_);
//     double          g   = ((y1 * 0.0) - range_min_) / (range_max_ - range_min_);
//     double          b   = ((z1 * 0.0) - range_min_) / (range_max_ - range_min_);
//     double          a   = rand_percent_(generator_);
//     bv_.addRay(ray, r, g, b, a, rclcpp::Duration{std::chrono::nanoseconds(1s)});
//     points.emplace_back(std::make_tuple(0.0, 0.0, 0.0));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//     points.emplace_back(std::make_tuple(x1, y1, z1));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//   }
//   bv_.publish();

//   for (int i = 0; i < 10; i++) {
//     if (received_msg_) {
//       RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
//       break;
//     }
//     clock->sleep_for(100ms);
//   }

//   EXPECT_TRUE(received_msg_);

//   EXPECT_EQ(received_msg_->markers[1].points.size(), points.size());
//   EXPECT_EQ(received_msg_->markers[1].colors.size(), colors.size());

//   tmp_pts = make_vector_tuple(received_msg_->markers[1].points);
//   EXPECT_TRUE(std::equal(tmp_pts.begin(), tmp_pts.end(), points.begin(), points.end(), compare_tuple3));
//   tmp_colors = make_vector_tuple(received_msg_->markers[1].colors);
//   EXPECT_TRUE(std::equal(tmp_colors.begin(), tmp_colors.end(), colors.begin(), colors.end(), compare_tuple4));

//   EXPECT_EQ(received_msg_->markers[0].points.size(), 1);
//   EXPECT_EQ(received_msg_->markers[0].colors.size(), 1);
//   EXPECT_EQ(received_msg_->markers[2].points.size(), 3);
//   EXPECT_EQ(received_msg_->markers[2].colors.size(), 3);

//   received_msg_.reset();

//   despin();

//   clock->sleep_for(1s);
// }

// TEST_F(Test, batch_visualize_triangles) {

//   initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

//   auto clock = node_->get_clock();

//   // | ------------------- create a subscriber ------------------ |

//   RCLCPP_INFO(node_->get_logger(), "Creating subscriber");
//   const std::string viz_topic_name_ = "batch_viz";

//   const std::function<void(visualization_msgs::msg::MarkerArray::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

//   auto sub = node_->create_subscription<visualization_msgs::msg::MarkerArray>(viz_topic_name_, 10, callback1_ptr);

//   // | ---------------------- start testing --------------------- |

//   BatchVisualizer bv_;
//   std::string viz_ns_name_ = "map";
//   bv_ = BatchVisualizer(node_, viz_topic_name_, viz_ns_name_);
//   int elem_count = 100;

//   {
//     for (int i = 0; i < 10; i++) {

//       if (sub->get_publisher_count() > 0) {
//         RCLCPP_INFO(node_->get_logger(), "Connected publisher and subscriber");
//         break;
//       }

//       clock->sleep_for(100ms);
//     }
//   }

//   if (sub->get_publisher_count() == 0) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to connect publisher and subscriber");
//     despin();
//     return;
//   }
//   // clean the init msg published on object construction
//   for (int i = 0; i < 10; i++) {
//     if (received_msg_) {
//       RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
//       break;
//     }
//     clock->sleep_for(100ms);
//   }
//   EXPECT_TRUE(received_msg_);
//   received_msg_.reset();

//   RCLCPP_INFO(node_->get_logger(), "Generating random rays");
//   std::vector<std::tuple<double, double, double>> points;
//   std::vector<std::tuple<double, double, double, double>> colors;
//   for (int i = 0; i < elem_count; i++) {
//     double          x1 = rand_dbl_(generator_);
//     double          y1 = rand_dbl_(generator_);
//     double          z1 = rand_dbl_(generator_);
//     Eigen::Vector3d point1(x1, y1, z1);
//     double          x2 = rand_dbl_(generator_);
//     double          y2 = rand_dbl_(generator_);
//     double          z2 = rand_dbl_(generator_);
//     Eigen::Vector3d point2(x2, y2, z2);
//     Ray             ray = Ray::twopointCast(point1, point2);
//     double          r   = ((x1 * x2) - range_min_) / (range_max_ - range_min_);
//     double          g   = ((y1 * y2) - range_min_) / (range_max_ - range_min_);
//     double          b   = ((z1 * z2) - range_min_) / (range_max_ - range_min_);
//     double          a   = rand_percent_(generator_);
//     bv_.addRay(ray, r, g, b, a, rclcpp::Duration{std::chrono::nanoseconds(1s)});
//     points.emplace_back(std::make_tuple(x1, y1, z1));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//     points.emplace_back(std::make_tuple(x2, y2, z2));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//   }
//   bv_.publish();

//   for (int i = 0; i < 10; i++) {
//     if (received_msg_) {
//       RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
//       break;
//     }
//     clock->sleep_for(100ms);
//   }

//   EXPECT_TRUE(received_msg_);

//   EXPECT_EQ(received_msg_->markers[1].points.size(), points.size());
//   EXPECT_EQ(received_msg_->markers[1].colors.size(), colors.size());

//   auto tmp_pts = make_vector_tuple(received_msg_->markers[1].points);
//   EXPECT_TRUE(std::equal(tmp_pts.begin(), tmp_pts.end(), points.begin(), points.end(), compare_tuple3));
//   auto tmp_colors = make_vector_tuple(received_msg_->markers[1].colors);
//   EXPECT_TRUE(std::equal(tmp_colors.begin(), tmp_colors.end(), colors.begin(), colors.end(), compare_tuple4));

//   EXPECT_EQ(received_msg_->markers[0].points.size(), 1);
//   EXPECT_EQ(received_msg_->markers[0].colors.size(), 1);
//   EXPECT_EQ(received_msg_->markers[2].points.size(), 3);
//   EXPECT_EQ(received_msg_->markers[2].colors.size(), 3);

//   received_msg_.reset();

//   RCLCPP_INFO(node_->get_logger(), "Generating random rays starting from origin");
//   for (int i = 0; i < elem_count; i++) {
//     double          x1 = rand_dbl_(generator_);
//     double          y1 = rand_dbl_(generator_);
//     double          z1 = rand_dbl_(generator_);
//     Eigen::Vector3d point(x1, y1, z1);
//     Ray             ray = Ray::twopointCast(Eigen::Vector3d::Zero(), point);
//     double          r   = ((x1 * 0.0) - range_min_) / (range_max_ - range_min_);
//     double          g   = ((y1 * 0.0) - range_min_) / (range_max_ - range_min_);
//     double          b   = ((z1 * 0.0) - range_min_) / (range_max_ - range_min_);
//     double          a   = rand_percent_(generator_);
//     bv_.addRay(ray, r, g, b, a, rclcpp::Duration{std::chrono::nanoseconds(1s)});
//     points.emplace_back(std::make_tuple(0.0, 0.0, 0.0));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//     points.emplace_back(std::make_tuple(x1, y1, z1));
//     colors.emplace_back(std::make_tuple(r, g, b, a));
//   }
//   bv_.publish();

//   for (int i = 0; i < 10; i++) {
//     if (received_msg_) {
//       RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
//       break;
//     }
//     clock->sleep_for(100ms);
//   }

//   EXPECT_TRUE(received_msg_);

//   EXPECT_EQ(received_msg_->markers[1].points.size(), points.size());
//   EXPECT_EQ(received_msg_->markers[1].colors.size(), colors.size());

//   tmp_pts = make_vector_tuple(received_msg_->markers[1].points);
//   EXPECT_TRUE(std::equal(tmp_pts.begin(), tmp_pts.end(), points.begin(), points.end(), compare_tuple3));
//   tmp_colors = make_vector_tuple(received_msg_->markers[1].colors);
//   EXPECT_TRUE(std::equal(tmp_colors.begin(), tmp_colors.end(), colors.begin(), colors.end(), compare_tuple4));

//   EXPECT_EQ(received_msg_->markers[0].points.size(), 1);
//   EXPECT_EQ(received_msg_->markers[0].colors.size(), 1);
//   EXPECT_EQ(received_msg_->markers[2].points.size(), 3);
//   EXPECT_EQ(received_msg_->markers[2].colors.size(), 3);

//   received_msg_.reset();

//   despin();

//   clock->sleep_for(1s);
// }
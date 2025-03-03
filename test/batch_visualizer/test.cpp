#include <chrono>
#include <cmath>

#include <cstddef>
#include <filesystem>
#include <gtest/gtest.h>

#include <std_msgs/msg/color_rgba.hpp>
#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_msgs/msg/trajectory_reference.hpp>
#include <mrs_msgs/msg/reference.hpp>

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

    sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(viz_topic_name_, 10, callback1_ptr);

    // | ---------------------- start testing --------------------- |

    bv_ = BatchVisualizer(node_, viz_topic_name_, viz_ns_name_);
    for (int i = 0; i < 10; i++) {

      if (sub_->get_publisher_count() > 0) {
        RCLCPP_INFO(node_->get_logger(), "Connected publisher and subscriber");
        break;
      }

      clock->sleep_for(100ms);
    }

    if (sub_->get_publisher_count() == 0) {
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

  void test_batch_visualizer_msg(const int marker_idx, const std::vector<std::tuple<double, double, double>>& points, const std::vector<std::tuple<double, double, double, double>>& colors, std::size_t point_marker_vec_size = 1, std::size_t line_marker_vec_size = 2, std::size_t triangle_marker_vec_size = 3) {

    EXPECT_EQ(received_msg_->markers[0].points.size(), point_marker_vec_size);
    EXPECT_EQ(received_msg_->markers[0].colors.size(), point_marker_vec_size);

    EXPECT_EQ(received_msg_->markers[1].points.size(), line_marker_vec_size);
    EXPECT_EQ(received_msg_->markers[1].colors.size(), line_marker_vec_size);

    EXPECT_EQ(received_msg_->markers[2].points.size(), triangle_marker_vec_size);
    EXPECT_EQ(received_msg_->markers[2].colors.size(), triangle_marker_vec_size);

    auto tmp_pts = make_vector_tuple(received_msg_->markers[marker_idx].points);
    EXPECT_TRUE(std::equal(tmp_pts.begin(), tmp_pts.end(), points.begin(), points.end(), compare_tuple3));
    auto tmp_colors = make_vector_tuple(received_msg_->markers[marker_idx].colors);
    EXPECT_TRUE(std::equal(tmp_colors.begin(), tmp_colors.end(), colors.begin(), colors.end(), compare_tuple4));
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
  const double epsilon = 1e-6;  // Tolerance for floating-point comparison
    
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

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr received_msg_;
  BatchVisualizer bv_;
  std::string viz_ns_name_ = "map";
  std::string viz_topic_name_ = "batch_viz";
  int batch_size_ = 4;
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
    clock->sleep_for(100ms);
  }
  EXPECT_TRUE(received_msg_);
  test_batch_visualizer_init(viz_topic_name_, "cat");
  RCLCPP_INFO(node_->get_logger(), "Tested the 'cat' in 'ns'");
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_points) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);
  bv_.clearBuffers();

  RCLCPP_INFO(node_->get_logger(), "Generating random points");
  std::vector<std::tuple<double, double, double>> points;
  std::vector<std::tuple<double, double, double, double>> colors;
  for (int i = 0; i < 100; i++) {
    double          x = rand_dbl_(generator_);
    double          y = rand_dbl_(generator_);
    double          z = rand_dbl_(generator_);
    Eigen::Vector3d point(x, y, z);
    double          r = (x - range_min_) / (range_max_ - range_min_);
    double          g = (y - range_min_) / (range_max_ - range_min_);
    double          b = (z - range_min_) / (range_max_ - range_min_);
    double          a = rand_percent_(generator_);
    bv_.addPoint(point, r, g, b, a, rclcpp::Duration{std::chrono::nanoseconds(1s)});
    points.emplace_back(std::make_tuple(x, y, z));
    colors.emplace_back(std::make_tuple(r, g, b, a));
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);
  test_batch_visualizer_msg(0, points, colors, points.size());

  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_lines) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);
  bv_.clearBuffers();

  RCLCPP_INFO(node_->get_logger(), "Generating random rays");
  std::vector<std::tuple<double, double, double>> points;
  std::vector<std::tuple<double, double, double, double>> colors;
  for (int i = 0; i < batch_size_; i++) {
    double          x1 = rand_dbl_(generator_);
    double          y1 = rand_dbl_(generator_);
    double          z1 = rand_dbl_(generator_);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl_(generator_);
    double          y2 = rand_dbl_(generator_);
    double          z2 = rand_dbl_(generator_);
    Eigen::Vector3d point2(x2, y2, z2);
    Ray             ray = Ray::twopointCast(point1, point2);
    double          r   = ((x1 * x2) - range_min_) / (range_max_ - range_min_);
    double          g   = ((y1 * y2) - range_min_) / (range_max_ - range_min_);
    double          b   = ((z1 * z2) - range_min_) / (range_max_ - range_min_);
    double          a   = rand_percent_(generator_);
    bv_.addRay(ray, r, g, b, a, rclcpp::Duration{std::chrono::nanoseconds(1s)});
    points.emplace_back(std::make_tuple(x1, y1, z1));
    colors.emplace_back(std::make_tuple(r, g, b, a));
    points.emplace_back(std::make_tuple(x2, y2, z2));
    colors.emplace_back(std::make_tuple(r, g, b, a));
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);
  test_batch_visualizer_msg(1, points, colors, 1, points.size());

  bv_.clearBuffers();
  received_msg_.reset();
  points.clear();
  colors.clear();

  EXPECT_FALSE(received_msg_);

  RCLCPP_INFO(node_->get_logger(), "Generating random rays starting from origin");
  for (int i = 0; i < batch_size_; i++) {
    double          x1 = rand_dbl_(generator_);
    double          y1 = rand_dbl_(generator_);
    double          z1 = rand_dbl_(generator_);
    Eigen::Vector3d point(x1, y1, z1);
    Ray             ray = Ray::twopointCast(Eigen::Vector3d::Zero(), point);
    double          r   = ((x1 * 0.0) - range_min_) / (range_max_ - range_min_);
    double          g   = ((y1 * 0.0) - range_min_) / (range_max_ - range_min_);
    double          b   = ((z1 * 0.0) - range_min_) / (range_max_ - range_min_);
    double          a   = rand_percent_(generator_);
    bv_.addRay(ray, r, g, b, a, rclcpp::Duration{std::chrono::nanoseconds(1s)});
    points.emplace_back(std::make_tuple(0.0, 0.0, 0.0));
    colors.emplace_back(std::make_tuple(r, g, b, a));
    points.emplace_back(std::make_tuple(x1, y1, z1));
    colors.emplace_back(std::make_tuple(r, g, b, a));
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);
  test_batch_visualizer_msg(1, points, colors, 1, points.size());

  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_triangles) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  std::vector<std::tuple<double, double, double>> points;
  std::vector<std::tuple<double, double, double, double>> colors;

  EXPECT_FALSE(received_msg_);
  bv_.clearBuffers();

  RCLCPP_INFO(node_->get_logger(), "Generating random filled triangles");
  for (int i = 0; i < batch_size_; i++) {
    double          x1 = rand_dbl_(generator_);
    double          y1 = rand_dbl_(generator_);
    double          z1 = rand_dbl_(generator_);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl_(generator_);
    double          y2 = rand_dbl_(generator_);
    double          z2 = rand_dbl_(generator_);
    Eigen::Vector3d point2(x2, y2, z2);
    double          x3 = rand_dbl_(generator_);
    double          y3 = rand_dbl_(generator_);
    double          z3 = rand_dbl_(generator_);
    Eigen::Vector3d point3(x3, y3, z3);
    double          r = x1 * x2 * x3;
    double          g = y1 * y2 * y3;
    double          b = z1 * z2 * z3;
    double          a = rand_percent_(generator_);
    Triangle        tri(point1, point2, point3);
    bv_.addTriangle(tri, r, g, b, a, true, rclcpp::Duration{std::chrono::nanoseconds(1s)});
    points.emplace_back(std::make_tuple(x1, y1, z1));
    points.emplace_back(std::make_tuple(x2, y2, z2));
    points.emplace_back(std::make_tuple(x3, y3, z3));
    colors.emplace_back(std::make_tuple(r, g, b, a));
    colors.emplace_back(std::make_tuple(r, g, b, a));
    colors.emplace_back(std::make_tuple(r, g, b, a));
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);
  test_batch_visualizer_msg(2, points, colors, 1, 2, points.size());

  bv_.clearBuffers();
  received_msg_.reset();
  points.clear();
  colors.clear();

  EXPECT_FALSE(received_msg_);
  RCLCPP_INFO(node_->get_logger(), "Generating random outlined triangles");
  for (int i = 0; i < batch_size_; i++) {
    double          x1 = rand_dbl_(generator_);
    double          y1 = rand_dbl_(generator_);
    double          z1 = rand_dbl_(generator_);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl_(generator_);
    double          y2 = rand_dbl_(generator_);
    double          z2 = rand_dbl_(generator_);
    Eigen::Vector3d point2(x2, y2, z2);
    double          x3 = rand_dbl_(generator_);
    double          y3 = rand_dbl_(generator_);
    double          z3 = rand_dbl_(generator_);
    Eigen::Vector3d point3(x3, y3, z3);
    double          r = x1 * x2 * x3;
    double          g = y1 * y2 * y3;
    double          b = z1 * z2 * z3;
    double          a = rand_percent_(generator_);
    Triangle        tri(point1, point2, point3);
    bv_.addTriangle(tri, r, g, b, a, false, rclcpp::Duration{std::chrono::nanoseconds(1s)});

    points.emplace_back(std::make_tuple(x1, y1, z1));
    points.emplace_back(std::make_tuple(x2, y2, z2));
    colors.emplace_back(std::make_tuple(r, g, b, a));
    colors.emplace_back(std::make_tuple(r, g, b, a));

    points.emplace_back(std::make_tuple(x2, y2, z2));
    points.emplace_back(std::make_tuple(x3, y3, z3));
    colors.emplace_back(std::make_tuple(r, g, b, a));
    colors.emplace_back(std::make_tuple(r, g, b, a));

    points.emplace_back(std::make_tuple(x3, y3, z3));
    points.emplace_back(std::make_tuple(x1, y1, z1));
    colors.emplace_back(std::make_tuple(r, g, b, a));
    colors.emplace_back(std::make_tuple(r, g, b, a));
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);
  test_batch_visualizer_msg(1, points, colors, 1, points.size());

  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_rectangles) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);

  RCLCPP_INFO(node_->get_logger(), "Generating random filled triangles");
  for (int i = 0; i < batch_size_; i++) {
    double          x1 = rand_dbl_(generator_);
    double          y1 = rand_dbl_(generator_);
    double          z1 = rand_dbl_(generator_);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl_(generator_);
    double          y2 = rand_dbl_(generator_);
    double          z2 = rand_dbl_(generator_);
    Eigen::Vector3d point2(x2, y2, z2);
    double          plane_x = rand_dbl_(generator_);
    double          plane_y = rand_dbl_(generator_);
    double          plane_z = rand_dbl_(generator_);
    Eigen::Vector3d plane_anchor(plane_x, plane_y, plane_z);
    double          side_length2 = rand_dbl_(generator_) / 2.0;
    Eigen::Vector3d point3       = point2 + side_length2 * (((point2 - point1).cross(plane_anchor - point1)).normalized());
    Eigen::Vector3d point4       = point1 + side_length2 * (((point2 - point1).cross(plane_anchor - point1)).normalized());

    double    r = x1 * x2;
    double    g = y1 * y2;
    double    b = z1 * z2;
    double    a = rand_percent_(generator_);
    Rectangle rect(point1, point2, point3, point4);
    bv_.addRectangle(rect, r, g, b, a, true, rclcpp::Duration{std::chrono::nanoseconds(1s)});   // draw colored faces
    bv_.addRectangle(rect, 0, 0, 0, 1, false, rclcpp::Duration{std::chrono::nanoseconds(1s)});  // draw outlines
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);

  bv_.clearBuffers();
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_cuboid) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);

  RCLCPP_INFO(node_->get_logger(), "Generating random cubes");
  for (int i = 0; i < batch_size_; i++) {
    double             x1 = rand_dbl_(generator_);
    double             y1 = rand_dbl_(generator_);
    double             z1 = rand_dbl_(generator_);
    Eigen::Vector3d    center(x1, y1, z1);
    double             s = 0.4 * rand_dbl_(generator_);
    Eigen::Vector3d    scale(s, s, s);
    double             x2          = rand_dbl_(generator_);
    double             y2          = rand_dbl_(generator_);
    double             z2          = rand_dbl_(generator_);
    Eigen::Quaterniond orientation = mrs_lib::geometry::quaternionFromEuler(x2, y2, z2);

    double r = (x1 - range_min_) / (range_max_ - range_min_);
    double g = (y1 - range_min_) / (range_max_ - range_min_);
    double b = (z1 - range_min_) / (range_max_ - range_min_);
    double a = rand_percent_(generator_);
    Cuboid cub(center, scale, orientation);

    bv_.addCuboid(cub, r, g, b, a, true, rclcpp::Duration{std::chrono::nanoseconds(1s)});   // draw colored faces
    bv_.addCuboid(cub, 0, 0, 0, 1, false, rclcpp::Duration{std::chrono::nanoseconds(1s)});  // draw outlines
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);

  bv_.clearBuffers();
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_ellipse) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);

  RCLCPP_INFO(node_->get_logger(), "Generating random ellipses");
  for (int i = 0; i < batch_size_; i++) {
    double             x1 = rand_dbl_(generator_);
    double             y1 = rand_dbl_(generator_);
    double             z1 = rand_dbl_(generator_);
    Eigen::Vector3d    center(x1, y1, z1);
    double             major       = 0.4 * rand_dbl_(generator_);
    double             minor       = 0.1 * rand_dbl_(generator_);
    double             x2          = rand_dbl_(generator_);
    double             y2          = rand_dbl_(generator_);
    double             z2          = rand_dbl_(generator_);
    Eigen::Quaterniond orientation = mrs_lib::geometry::quaternionFromEuler(x2, y2, z2);

    double  r = (x1 - range_min_) / (range_max_ - range_min_);
    double  g = (y1 - range_min_) / (range_max_ - range_min_);
    double  b = (z1 - range_min_) / (range_max_ - range_min_);
    double  a = rand_percent_(generator_);
    Ellipse el(center, orientation, major, minor);

    bv_.addEllipse(el, r, g, b, a, true, 64, rclcpp::Duration{std::chrono::nanoseconds(1s)});           // colored face
    bv_.addEllipse(el, 0.0, 0.0, 0.0, 1.0, false, 64, rclcpp::Duration{std::chrono::nanoseconds(1s)});  // black outline
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);

  bv_.clearBuffers();
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_cylinders) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);

  RCLCPP_INFO(node_->get_logger(), "Generating random cylinders");
  for (int i = 0; i < batch_size_; i++) {
    double             x1 = rand_dbl_(generator_);
    double             y1 = rand_dbl_(generator_);
    double             z1 = rand_dbl_(generator_);
    Eigen::Vector3d    center(x1, y1, z1);
    double             x2          = rand_dbl_(generator_);
    double             y2          = rand_dbl_(generator_);
    double             z2          = rand_dbl_(generator_);
    Eigen::Quaterniond orientation = mrs_lib::geometry::quaternionFromEuler(x2, y2, z2);

    double radius = 0.3 * rand_dbl_(generator_);
    double height = rand_dbl_(generator_);

    double r = (x1 - range_min_) / (range_max_ - range_min_);
    double g = (y1 - range_min_) / (range_max_ - range_min_);
    double b = (z1 - range_min_) / (range_max_ - range_min_);
    double a = rand_percent_(generator_);

    Cylinder cyl(center, radius, height, orientation);
    bv_.addCylinder(cyl, r, g, b, a, true, true, 12, rclcpp::Duration{std::chrono::nanoseconds(1s)});
    bv_.addCylinder(cyl, 0, 0, 0, 1, false, false, 12, rclcpp::Duration{std::chrono::nanoseconds(1s)});
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);

  bv_.clearBuffers();
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_cones) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);

  RCLCPP_INFO(node_->get_logger(), "Generating random cones");
  for (int i = 0; i < batch_size_; i++) {
    double          x1 = rand_dbl_(generator_);
    double          y1 = rand_dbl_(generator_);
    double          z1 = rand_dbl_(generator_);
    Eigen::Vector3d origin(x1, y1, z1);
    double          x2 = rand_dbl_(generator_);
    double          y2 = rand_dbl_(generator_);
    double          z2 = rand_dbl_(generator_);
    Eigen::Vector3d direction(x2, y2, z2);

    double angle  = 0.02 * rand_dbl_(generator_);
    double height = rand_dbl_(generator_);

    double r = (x1 - range_min_) / (range_max_ - range_min_);
    double g = (y1 - range_min_) / (range_max_ - range_min_);
    double b = (z1 - range_min_) / (range_max_ - range_min_);
    double a = rand_percent_(generator_);

    Cone cone(origin, angle, height, direction);
    bv_.addCone(cone, r, g, b, a, true, true, 12, rclcpp::Duration{std::chrono::nanoseconds(1s)});
    bv_.addCone(cone, 0, 0, 0, 1, false, false, 12, rclcpp::Duration{std::chrono::nanoseconds(1s)});
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);

  bv_.clearBuffers();
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_trajectories) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);

  RCLCPP_INFO(node_->get_logger(), "Generating random trajectories");
  for (int i = 0; i < batch_size_; i++) {
    mrs_msgs::msg::TrajectoryReference traj;
    for (int n = 0; n < 10; n++) {
      double              x = rand_dbl_(generator_);
      double              y = rand_dbl_(generator_);
      double              z = rand_dbl_(generator_);
      mrs_msgs::msg::Reference ref;
      ref.position.x = x;
      ref.position.y = y;
      ref.position.z = z;
      traj.points.push_back(ref);
    }

    bv_.addTrajectory(traj, 0, 1, 1, 1, true, rclcpp::Duration{std::chrono::nanoseconds(1s)});
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);

  bv_.clearBuffers();
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}

TEST_F(Test, batch_visualize_rays_and_triangles) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto clock = node_->get_clock();

  EXPECT_FALSE(received_msg_);

  RCLCPP_INFO(node_->get_logger(), "Generating random rays and triangles");
  for (int i = 0; i < batch_size_; i++) {
    double          x1 = rand_dbl_(generator_);
    double          y1 = rand_dbl_(generator_);
    double          z1 = rand_dbl_(generator_);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl_(generator_);
    double          y2 = rand_dbl_(generator_);
    double          z2 = rand_dbl_(generator_);
    Eigen::Vector3d point2(x2, y2, z2);
    double          x3 = rand_dbl_(generator_);
    double          y3 = rand_dbl_(generator_);
    double          z3 = rand_dbl_(generator_);
    Eigen::Vector3d point3(3 * x3, 3 * y3, 3 * z3);
    Triangle        tri(point1, point2, point3);

    double          rx1 = rand_dbl_(generator_);
    double          ry1 = rand_dbl_(generator_);
    double          rz1 = rand_dbl_(generator_);
    Eigen::Vector3d ray_point1(4 * rx1, 4 * ry1, 4 * rz1);
    double          rx2 = rand_dbl_(generator_);
    double          ry2 = rand_dbl_(generator_);
    double          rz2 = rand_dbl_(generator_);
    Eigen::Vector3d ray_point2(4 * rx2, 4 * ry2, 4 * rz2);
    Ray             ray = Ray::twopointCast(ray_point1, ray_point2);
    bv_.addRay(ray, 1.0, 0.5, 0.0, 1.0, rclcpp::Duration{std::chrono::nanoseconds(1s)});

    if (!tri.intersectionRay(ray).has_value()) {
      bv_.addTriangle(tri, 1.0, 0.0, 0.0, 1.0, true, rclcpp::Duration{std::chrono::nanoseconds(1s)});
    } else {
      bv_.addTriangle(tri, 0.0, 1.0, 0.3, 1.0, true, rclcpp::Duration{std::chrono::nanoseconds(1s)});
      bv_.addPoint(tri.intersectionRay(ray).value(), 1, 1, 0, 1, rclcpp::Duration{std::chrono::nanoseconds(1s)});
    }
  }
  bv_.publish();

  for (int i = 0; i < 10; i++) {
    if (received_msg_) {
      RCLCPP_INFO(node_->get_logger(), "Msg found at itr: %i", i);
      break;
    }
    clock->sleep_for(100ms);
  }

  EXPECT_TRUE(received_msg_);

  bv_.clearBuffers();
  received_msg_.reset();

  despin();

  clock->sleep_for(1s);
}
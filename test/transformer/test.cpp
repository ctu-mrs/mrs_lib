#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/transform_broadcaster.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <mrs_lib/transformer.h>

#include <iostream>

#include <thread>

using namespace std::chrono_literals;

using namespace mrs_lib;
using namespace std;
using namespace mrs_lib::geometry;

class Test : public ::testing::Test {

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

    tf_broadcaster_ = mrs_lib::TransformBroadcaster(node_);

    local2local               = anax_t(3.2, vec3_t::UnitZ());
    local2local.translation() = 10 * vec3_t::Random();

    local2fcu               = anax_t(1.2, vec3_t::UnitZ()) * anax_t(0.2, vec3_t::UnitX());
    local2fcu.translation() = vec3_t::Random();

    local2utm               = anax_t(2.2, vec3_t::UnitZ()) * anax_t(0.1, vec3_t::UnitX());
    local2utm.translation() = vec3_t::Random();

    fcu2cam               = anax_t(1.54, vec3_t::UnitZ()) * anax_t(1.54, vec3_t::UnitX()) * anax_t(1.54, vec3_t::UnitY());
    fcu2cam.translation() = vec3_t(0.2, 0, 0);
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

  //{ publish_transform helper function
  //
  void publish_transforms(const rclcpp::Time& t = rclcpp::Time(0)) {

    geometry_msgs::msg::TransformStamped tf;

    rclcpp::Time time;

    std::cout << "kocour" << std::endl;

    if (t.seconds() <= 0) {

      std::cout << "c" << std::endl;

      time = node_->get_clock()->now();
    } else {

      std::cout << "d" << std::endl;

      time = t;
    }

    tf                 = tf2::eigenToTransform(local2local);
    tf.child_frame_id  = "uav3/local_origin";
    tf.header.frame_id = "uav66/local_origin";
    tf.header.stamp    = time;
    tf_broadcaster_.sendTransform(tf);

    tf                 = tf2::eigenToTransform(local2fcu);
    tf.child_frame_id  = "uav66/local_origin";
    tf.header.frame_id = "uav66/fcu";
    tf.header.stamp    = time;
    tf_broadcaster_.sendTransform(tf);

    tf                 = tf2::eigenToTransform(local2utm.inverse());
    tf.child_frame_id  = "uav66/utm_origin";
    tf.header.frame_id = "uav66/local_origin";
    tf.header.stamp    = time;
    tf_broadcaster_.sendTransform(tf);

    tf                 = tf2::eigenToTransform(fcu2cam);
    tf.child_frame_id  = "uav66/fcu";
    tf.header.frame_id = "uav66/camera";
    tf.header.stamp    = time;
    tf_broadcaster_.sendTransform(tf);
  }

  //}

  /* //{ wait_for_tf helper function*/
  std::optional<geometry_msgs::msg::TransformStamped> wait_for_tf(const std::string& from, const std::string& to, mrs_lib::Transformer& tfr,
                                                                  const rclcpp::Time& publish_t = rclcpp::Time(0),
                                                                  const rclcpp::Time& stamp     = rclcpp::Time(0)) {

    std::cout << "Waiting for transformation from " << from << " to " << to << ".\n";
    std::optional<geometry_msgs::msg::TransformStamped> tf_opt;

    // get the transform
    for (int it = 0; it < 10 && rclcpp::ok(); it++) {

      if (publish_t == rclcpp::Time(0)) {

        publish_transforms(node_->get_clock()->now());
      } else {

        publish_transforms(publish_t);
      }

      tf_opt = tfr.getTransform(from, to, stamp);

      if (tf_opt.has_value())
        break;

      node_->get_clock()->sleep_for(std::chrono::duration<double>(0.05));
    }

    std::cout << "... got the transform" << std::endl;

    return tf_opt;
  }

  //}

  rclcpp::Node::SharedPtr                              node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;

  mrs_lib::TransformBroadcaster tf_broadcaster_;

  Eigen::Affine3d local2local;
  Eigen::Affine3d local2fcu;
  Eigen::Affine3d local2utm;
  Eigen::Affine3d fcu2cam;
};

/* TEST_F(Test, main_test) //{ */

TEST_F(Test, main_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);

  tfr.setDefaultPrefix("uav66");

  const std::string from = "fcu";
  const std::string to   = "local_origin";

  const auto tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    RCLCPP_INFO(node_->get_logger(), "got the transform");

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << rclcpp::Time(tf.header.stamp).seconds()
              << std::endl;
    /* std::cout << tf << std::endl; */

    const auto tf_inv = Transformer::inverse(tf);
    std::cout << "from: " << Transformer::frame_from(tf_inv) << ", to: " << Transformer::frame_to(tf_inv)
              << ", stamp: " << rclcpp::Time(tf_inv.header.stamp).seconds() << std::endl;
    /* std::cout << tf_inv << std::endl; */

    if (fabs(tf_inv.transform.translation.x - local2fcu.translation().x()) > 1e-6 ||
        fabs(tf_inv.transform.translation.y - local2fcu.translation().y()) > 1e-6 ||
        fabs(tf_inv.transform.translation.z - local2fcu.translation().z()) > 1e-6) {

      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                                   "translation does not match (gt: " << local2fcu.translation().transpose() << ")");
      result *= 0;
    }

    const Eigen::Isometry3d etf = tf2::transformToEigen(tf_inv);
    const quat_t            etf_rot(etf.rotation());
    const double            angle_diff = etf_rot.angularDistance(quat_t(local2fcu.rotation()));

    if (angle_diff > 1e-6) {
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "translation does not match (by angle: " << angle_diff << ")");
      result *= 0;
    }

  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "missing tf");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

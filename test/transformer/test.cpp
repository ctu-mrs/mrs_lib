#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/transform_broadcaster.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <mrs_lib/transformer.h>

#include <mrs_lib/geometry/conversions_eigen.h>

#include <iostream>

#include <thread>

#include <mrs_lib/gps_conversions.h>

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

    if (t.seconds() <= 0) {
      time = node_->get_clock()->now();
    } else {
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

      if (publish_t.seconds() == rclcpp::Time(0).seconds()) {

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

  /* latlon test help functions //{ */

  bool compare_gt_latlon(const geometry_msgs::msg::Point& tv, const geometry_msgs::msg::Point& rv, const char utm_zone[10], const bool ll2local);

  bool compare_gt_latlon([[maybe_unused]] const quat_t& tv, [[maybe_unused]] const quat_t& rv, [[maybe_unused]] const char utm_zone[10],
                         [[maybe_unused]] const bool ll2local);

  template <typename T>
  bool trytransform_latlon(const T& tv, mrs_lib::Transformer& tfr, const char utm_zone[10], const bool ll2local, const bool expect_ok);

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
    std::cout << geometry_msgs::msg::to_yaml(tf_inv) << std::endl;

    if (fabs(tf_inv.transform.translation.x - local2fcu.translation().x()) > 1e-6 ||
        fabs(tf_inv.transform.translation.y - local2fcu.translation().y()) > 1e-6 ||
        fabs(tf_inv.transform.translation.z - local2fcu.translation().z()) > 1e-6) {

      RCLCPP_ERROR_STREAM(node_->get_logger(), "translation does not match (gt: " << local2fcu.translation().transpose() << ")");
      result *= 0;
    }

    const Eigen::Isometry3d etf = tf2::transformToEigen(tf_inv);
    const quat_t            etf_rot(etf.rotation());
    const double            angle_diff = etf_rot.angularDistance(quat_t(local2fcu.rotation()));

    if (angle_diff > 1e-6) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "translation does not match (by angle: " << angle_diff << ")");
      result *= 0;
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "missing tf");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, tf_times_test) //{ */

TEST_F(Test, tf_times_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);

  const rclcpp::Time t1    = rclcpp::Time(1);
  const std::string  from  = "uav1/fcu";
  const rclcpp::Time t2    = rclcpp::Time(2);
  const std::string  to    = "uav2/fcu";
  const std::string  fixed = "common_origin";

  // create and publish the transformations
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id      = fixed;
  tf.transform.rotation.w = 1;
  Eigen::Isometry3d common2uav1, common2uav2;

  // UAV1 is at position [-1,0,2] at time t1
  tf.header.stamp           = t1;
  tf.child_frame_id         = from;
  common2uav1.translation() = vec3_t(-1, 0, 2);
  tf.transform.translation  = mrs_lib::geometry::fromEigenVec(common2uav1.translation());
  tf_broadcaster_.sendTransform(tf);

  // UAV2 is at position [-10,0,3] at time t1
  tf.header.stamp           = t1;
  tf.child_frame_id         = to;
  common2uav2.translation() = vec3_t(-10, 0, 3);
  tf.transform.translation  = mrs_lib::geometry::fromEigenVec(common2uav2.translation());
  tf_broadcaster_.sendTransform(tf);

  // UAV1 is at position [3,0,2] at time t2
  tf.header.stamp           = t2;
  tf.child_frame_id         = from;
  common2uav1.translation() = vec3_t(3, 0, 2);
  tf.transform.translation  = mrs_lib::geometry::fromEigenVec(common2uav1.translation());
  tf_broadcaster_.sendTransform(tf);

  // UAV2 is at position [60,0,3] at time t2
  tf.header.stamp           = t2;
  tf.child_frame_id         = to;
  common2uav2.translation() = vec3_t(60, 0, 3);
  tf.transform.translation  = mrs_lib::geometry::fromEigenVec(common2uav2.translation());
  tf_broadcaster_.sendTransform(tf);

  std::cout << "Waiting for transformation from " << from << " to " << to << " through " << fixed << ".\n";

  tfr.setLookupTimeout(rclcpp::Duration(chrono::duration<double>(2.0)));

  // where was UAV1 at time t1 relative to position of UAV2 at time t2 assuming common_frame is fixed??
  const std::optional<geometry_msgs::msg::TransformStamped> tf_opt = tfr.getTransform(from, t1, to, t2, fixed);

  if (tf_opt.has_value()) {
    RCLCPP_INFO(node_->get_logger(), "got the transform");

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << rclcpp::Time(tf.header.stamp).seconds()
              << std::endl;

    // the expected transformation is a translation [-61, 0, -1]
    const vec3_t tr = toEigen(tf.transform.translation);

    if (fabs(tr.x() - -61) > 1e-6 || fabs(tr.y() - 0) > 1e-6 || fabs(tr.z() - -1) > 1e-6) {
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                                   "translation does not match (gt: -61, 0, -1), actual: " << tr.transpose() << ")");
      result *= 0;
    }

    const Eigen::Isometry3d etf = tf2::transformToEigen(tf);
    const quat_t            etf_rot(etf.rotation());
    const double            angle_diff = etf_rot.angularDistance(quat_t::Identity());

    if (angle_diff > 1e-6) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "rotation does not match (by angle: " << angle_diff << ")");
      result *= 0;
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "missing tf");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, eigen_vector3d_test) //{ */

TEST_F(Test, eigen_vector3d_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from   = "uav66/camera";
  const std::string to     = "fcu";
  const auto        tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    RCLCPP_INFO(node_->get_logger(), "got the transform");

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << rclcpp::Time(tf.header.stamp).seconds()
              << std::endl;
    std::cout << geometry_msgs::msg::to_yaml(tf) << std::endl;

    std::vector<Eigen::Vector3d> test_vectors = {{1, 0, 0}, {0, -1, 666}, {0, 0, 0}, {0, 1.1, -1}, {0, 0, 18}, {3, 0, 0}};

    for (const auto& tv : test_vectors) {

      // | ------------- normal transform (as a vector) ------------- |
      {
        const auto rv = tfr.transform(tv, tf);
        if (!rv) {
          RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        } else {
          // transform it as a *vector*, not as a point!
          const vec3_t gt        = fcu2cam.inverse().rotation() * tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6) {
            RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                                         "transformed vector [" << gt.transpose() << "] with value of [" << rv.value().transpose()
                                                                << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }

      // | ----------------------- as a vector ---------------------- |
      {
        const auto rv = tfr.transformAsVector(tv, tf);
        if (!rv) {
          RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        } else {
          // transform it as a *vector*, not as a point!
          const vec3_t gt        = fcu2cam.inverse().rotation() * tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6) {
            RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                                         "transformed vector [" << gt.transpose() << "] with value of [" << rv.value().transpose()
                                                                << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }

      // | ----------------------- as a point ----------------------- |
      {
        const auto rv = tfr.transformAsPoint(tv, tf);
        if (!rv) {
          RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        } else {
          // transform it as a *vector*, not as a point!
          const vec3_t gt        = fcu2cam.inverse() * tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6) {
            RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                                         "transformed vector [" << gt.transpose() << "] with value of [" << rv.value().transpose()
                                                                << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }

      // | ----------------------- as a vector ---------------------- |
      {
        const auto rv = tfr.transformAsVector(mrs_lib::Transformer::frame_from(tf), tv, mrs_lib::Transformer::frame_to(tf), tf.header.stamp);
        if (!rv) {
          RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        } else {
          // transform it as a *vector*, not as a point!
          const vec3_t gt        = fcu2cam.inverse().rotation() * tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6) {
            RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                                         "transformed vector [" << gt.transpose() << "] with value of [" << rv.value().transpose()
                                                                << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }

      // | ----------------------- as a point ----------------------- |
      {
        const auto rv = tfr.transformAsPoint(mrs_lib::Transformer::frame_from(tf), tv, mrs_lib::Transformer::frame_to(tf), tf.header.stamp);
        if (!rv) {
          RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        } else {
          // transform it as a *vector*, not as a point!
          const vec3_t gt        = fcu2cam.inverse() * tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6) {
            RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                                         "transformed vector [" << gt.transpose() << "] with value of [" << rv.value().transpose()
                                                                << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "missing tf");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, transform_single) //{ */

TEST_F(Test, transform_single) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);
  tfr.setDefaultPrefix("uav66");

  const rclcpp::Time t = node_->get_clock()->now();
  publish_transforms(t);

  const std::string from = "uav66/camera";
  const std::string to   = "fcu";

  wait_for_tf(from, to, tfr, t);

  // test transforming a geometry_msgs::msg::PointStamped message
  {
    geometry_msgs::msg::PointStamped pt;
    pt.point.x         = 666;
    pt.point.y         = 667;
    pt.point.z         = 668;
    pt.header.frame_id = from;
    pt.header.stamp    = t;
    const vec3_t tv(pt.point.x, pt.point.y, pt.point.z);

    const auto rv_opt = tfr.transformSingle(pt, to);

    if (!rv_opt) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to transform vector [" << tv.transpose() << "]");
      result *= 0;
    } else {
      const vec3_t rv(rv_opt.value().point.x, rv_opt.value().point.y, rv_opt.value().point.z);
      const vec3_t gt        = fcu2cam.inverse() * tv;
      const vec3_t vect_diff = rv - gt;
      if (vect_diff.norm() > 1e-6) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "transformed vector [" << gt.transpose() << "] with value of [" << rv.transpose()
                                                                        << "] does not match the expected value of [" << gt.transpose() << "]");
        result *= 0;
      }
    }
  }

  // test transforming a headerless message
  {
    const quat_t                   tv(anax_t(2, vec3_t::UnitZ()) * anax_t(-0, vec3_t::UnitX()));
    geometry_msgs::msg::Quaternion q;
    q.x                               = tv.x();
    q.y                               = tv.y();
    q.z                               = tv.z();
    q.w                               = tv.w();
    geometry_msgs::msg::Quaternion cq = q;

    const auto rv_opt = tfr.transformSingle(from, cq, to, t);
    if (!rv_opt) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to transform vector [" << tv.vec() << ", " << tv.w() << "]");
      result *= 0;
    } else {
      const quat_t rv(rv_opt.value().w, rv_opt.value().x, rv_opt.value().y, rv_opt.value().z);
      const quat_t gt(fcu2cam.inverse().rotation() * tv);
      const double ang_diff = rv.angularDistance(gt);
      if (ang_diff > 1e-6) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Transformed quaternion doesn't match (angular difference is " << ang_diff << ")");
        result *= 0;
      }
    }
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, mrs_reference_test) //{ */

TEST_F(Test, mrs_reference_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);

  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from   = "uav66/fcu";
  const std::string to     = "camera";
  const auto        tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    RCLCPP_INFO(node_->get_logger(), "got the transform");

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << rclcpp::Time(tf.header.stamp).seconds()
              << std::endl;
    std::cout << geometry_msgs::msg::to_yaml(tf) << std::endl;

    const std::vector<Eigen::Vector3d> test_points = {{1, 0, 0}, {0, -1, 666}, {0, 0, 0}, {0, 1.1, -1}, {0, 0, 18}, {3, 0, 0}};

    std::vector<std::pair<mrs_msgs::msg::ReferenceStamped, mrs_msgs::msg::ReferenceStamped>> test_refs;
    mrs_msgs::msg::ReferenceStamped                                                          orig, tfd;

    orig.header.frame_id = "camera";
    orig.header.stamp    = node_->get_clock()->now();
    tfd.header.frame_id  = "fcu";
    tfd.header.stamp     = orig.header.stamp;

    for (const auto& vec : test_points) {
      orig.reference.position.x = vec.x();
      orig.reference.position.y = vec.y();
      orig.reference.position.z = vec.z();
      orig.reference.heading    = std::atan2(vec.y(), vec.z());

      // transform it as a *point*, not as a vector!
      const vec3_t gt          = fcu2cam * vec;
      tfd.reference.position.x = gt.x();
      tfd.reference.position.y = gt.y();
      tfd.reference.position.z = gt.z();
      tfd.reference.heading    = headingFromRot(fcu2cam.rotation() * quaternionFromHeading(orig.reference.heading));

      test_refs.push_back({orig, tfd});
    }

    for (const auto& tv : test_refs) {

      const auto rv_opt = tfr.transform(tv.first, tf);

      if (!rv_opt) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Failed to transform reference");
        result *= 0;
      } else {

        const auto            rv           = rv_opt.value();
        const auto            gt           = tv.second;
        const Eigen::Vector3d vect_diff    = mrs_lib::geometry::toEigen(rv.reference.position) - mrs_lib::geometry::toEigen(gt.reference.position);
        const double          heading_diff = std::abs(rv.reference.heading - gt.reference.heading);

        if (vect_diff.norm() > 1e-6 || heading_diff > 1e-6) {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "transformed reference does not match the expected value");
          result *= 0;
        }
      }
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "missing tf");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, quaternion_test) //{ */

TEST_F(Test, quaternion_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from   = "local_origin";
  const std::string to     = "fcu";
  const auto        tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    RCLCPP_INFO(node_->get_logger(), "got the transform");

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << std_msgs::msg::to_yaml(tf.header)
              << std::endl;
    std::cout << geometry_msgs::msg::to_yaml(tf) << std::endl;

    std::vector<geometry_msgs::msg::Quaternion> tsts;
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));

    for (const auto& tv : tsts) {

      const auto rv = tfr.transform(tv, tf);

      if (!rv) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Failed to transform quaternion");
        result *= 0;
      } else {

        quat_t orig, tfd;
        tf2::fromMsg(tv, orig);
        tf2::fromMsg(rv.value(), tfd);
        const quat_t gt(local2fcu.rotation() * orig);
        const double angle_diff = gt.angularDistance(tfd);

        if (angle_diff > 1e-6) {
          RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "transformed quaternion does not match the expected value");
          result *= 0;
        }
      }
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "missing tf");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, opencv_test) //{ */

#ifdef OPENCV_SPECIALIZATION
TEST_F(Test, opencv_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  ROS_INFO("[%s]: Testing the cv::Point3<T> transformation", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_opencv_test");
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from   = "local_origin";
  const std::string to     = "fcu";
  const auto        tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {
    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    const cv::Point3d tv(1, 2, 3);
    const auto        rv = tfr.transform(tv, tf);
    if (!rv) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform point [" << tv.x << ", " << tv.y << ", " << tv.z << "]");
      result *= 0;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: missing tf", ros::this_node::getName().c_str());
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}
#endif

//}

/* TEST_F(Test, latlon_test) //{ */

bool Test::compare_gt_latlon(const geometry_msgs::msg::Point& tv, const geometry_msgs::msg::Point& rv, const char utm_zone[10], const bool ll2local) {

  vec3_t gt;

  if (ll2local) {
    // convert LAT-LON to UTM
    Eigen::Vector3d utm;
    mrs_lib::UTM(tv.x, tv.y, &(utm.x()), &(utm.y()));
    // copy the height from the input
    utm.z()            = tv.z;
    const vec3_t local = local2utm.inverse() * utm;
    gt                 = local;
  } else {
    const vec3_t    utm = local2utm * mrs_lib::geometry::toEigen(tv);
    Eigen::Vector3d latlon;
    mrs_lib::UTMtoLL(utm.y(), utm.x(), utm_zone, latlon.x(), latlon.y());
    latlon.z() = utm.z();
    gt         = latlon;
  }

  const Eigen::Vector3d vect_diff = mrs_lib::geometry::toEigen(rv) - gt;

  if (vect_diff.norm() > 1e-6) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "<< Transformed [" << geometry_msgs::msg::to_yaml(tv) << "] with value of [" << geometry_msgs::msg::to_yaml(rv)
                                                                << "] does not match the expected value of [" << gt.transpose() << "]");
    return false;
  }

  std::cout << "<< Expected success happened\n";
  return true;
}

bool Test::compare_gt_latlon([[maybe_unused]] const quat_t& tv, [[maybe_unused]] const quat_t& rv, [[maybe_unused]] const char utm_zone[10],
                             [[maybe_unused]] const bool ll2local) {

  RCLCPP_ERROR(node_->get_logger(), "this should never happen");
  return false;
}

template <typename T>
bool Test::trytransform_latlon(const T& tv, mrs_lib::Transformer& tfr, const char utm_zone[10], const bool ll2local, const bool expect_ok) {

  const std::string local  = "local_origin";
  const std::string ll     = mrs_lib::LATLON_ORIGIN;
  const auto        tf_opt = ll2local ? wait_for_tf(ll, local, tfr) : wait_for_tf(local, ll, tfr);

  if (!tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "missing tf");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "got the transform");

  const auto tf = tf_opt.value();
  std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << rclcpp::Time(tf.header.stamp).seconds()
            << std::endl;

  const auto rv_opt = tfr.transform(tv, tf);
  if (!rv_opt) {
    if (!expect_ok) {
      std::cout << "<< Expected failure happened\n";
      return true;
    }

    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to transform message");
    return false;
  }

  if (rv_opt && !expect_ok) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Transformed message - that should not be possible!");
    return false;
  }

  const auto rv = rv_opt.value();
  return compare_gt_latlon(tv, rv, utm_zone, ll2local);
}

TEST_F(Test, latlon_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "Testing transformation from/to latlon");

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const double lat = 1;
  const double lon = 2;
  double       utm_x, utm_y;
  char         utm_zone_[10];

  mrs_lib::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_);

  geometry_msgs::msg::Point tv;
  tv.x = 5;
  tv.y = 6;
  tv.z = 7;

  const quat_t tq(1, 0, 0, 0);

  // try transforming a vector from latlon to local frame and expect a failure
  RCLCPP_INFO(node_->get_logger(), "Testing transformation of vector from local to latlon, expecting failure >>");
  result = result && trytransform_latlon(tv, tfr, utm_zone_, false, false);

  // try transforming a vector from local to latlon frame and expect everything OK
  RCLCPP_INFO(node_->get_logger(), "Testing transformation of vector from latlon to local, expecting all OK >>");
  result = result && trytransform_latlon(tv, tfr, utm_zone_, true, true);

  // try transforming a quaternion from local to latlon frame and expect failure because there's no UTMtoLL method for quaternion
  RCLCPP_INFO(node_->get_logger(), "Testing transformation of quaternion from latlon to local, expecting failure >>");
  result = result && trytransform_latlon(tq, tfr, utm_zone_, true, false);

  // after latlon is set, lookup from utm to latlon should be OK
  RCLCPP_INFO(node_->get_logger(), "Setting lattitude and longitude to Transformer, UTM->latlon should be fine now.");
  tfr.setLatLon(1, 2);

  // try transforming a vector from latlon to local frame and expect everything OK
  RCLCPP_INFO(node_->get_logger(), "Testing transformation of vector from local to latlon, expecting all OK >>");
  result = result && trytransform_latlon(tv, tfr, utm_zone_, false, true);

  // try transforming a vector from local to latlon frame and expect everything OK
  RCLCPP_INFO(node_->get_logger(), "Testing transformation of vector from latlon to local, expecting all OK >>");
  result = result && trytransform_latlon(tv, tfr, utm_zone_, true, true);

  // try transforming a quaternion from latlon to local frame and expect failure because there's no UTMtoLL method for quaternion
  RCLCPP_INFO(node_->get_logger(), "Testing transformation of quaternion from local to latlon, expecting failure >>");
  result = result && trytransform_latlon(tq, tfr, utm_zone_, false, false);

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, eigen_decomposed_test) //{ */

TEST_F(Test, eigen_decomposed_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "Testing the Eigen translation and rotation extraction");

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from   = "camera";
  const std::string to     = "fcu";
  const auto        tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    RCLCPP_INFO(node_->get_logger(), "got the transform");

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << rclcpp::Time(tf.header.stamp).seconds()
              << std::endl;
    std::cout << geometry_msgs::msg::to_yaml(tf) << std::endl;

    /* std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> test_vectors = {{{1,0,0},{0,-1,0}},{{0,1,0},{0,0,-1}},{{0,0,1},{1,0,0}}}; */

    const auto               etf  = tf2::transformToEigen(tf);
    const Eigen::Vector3d    tran = etf.translation();
    const Eigen::Quaterniond rot(etf.rotation());

    const vec3_t gt        = fcu2cam.inverse().translation();
    const vec3_t vect_diff = tran - gt;

    if (vect_diff.norm() > 1e-6) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "translation vector [" << tran.transpose() << "] does not match the expected value of [" << gt.transpose() << "]");
      result *= 0;
    }

    const quat_t rot_template(fcu2cam.inverse().rotation());
    if (rot_template.angularDistance(rot) > 1e-6) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "rotation quaternion \
          [" << rot.x()
             << "," << rot.y() << "," << rot.z() << "," << rot.w() << "]\
          does not match the expected value of \
          [" << rot_template.x()
             << "," << rot_template.y() << "," << rot_template.z() << "," << rot_template.w() << "]");
      result *= 0;
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "missing tf");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, retry_latest_test) //{ */

TEST_F(Test, retry_latest_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);
  tfr.setDefaultPrefix("uav66");

  const rclcpp::Time t = node_->get_clock()->now();
  publish_transforms(t);

  const std::string from   = "camera";
  const std::string to     = "local_origin";
  auto              tf_opt = wait_for_tf(from, to, tfr, t, t - rclcpp::Duration(1000, 0));

  if (tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "got the transform (that should not happen)");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tfr.retryLookupNewest(true);
  tf_opt = wait_for_tf(from, to, tfr, t, t - rclcpp::Duration(1000, 0));

  if (!tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "didn't get the transformation!");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, prefix_test) //{ */

TEST_F(Test, prefix_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);

  const rclcpp::Time t = node_->get_clock()->now();
  publish_transforms(t);

  auto tf_opt = wait_for_tf("camera", "local_origin", tfr);
  if (tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "got the transform (that should not happen)");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tf_opt = wait_for_tf("uav3/local_origin", "uav66/local_origin", tfr);
  if (!tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "didn't get the transformation!");
    result *= 0;
  }

  tfr.setDefaultPrefix("uav66");
  tf_opt = std::nullopt;
  tf_opt = wait_for_tf("camera", "local_origin", tfr);
  if (!tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "didn't get the transformation!");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tf_opt = wait_for_tf("uav66/camera", "uav66/local_origin", tfr);
  if (!tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "didn't get the transformation!");
    result *= 0;
  }

  tfr.setDefaultPrefix("");
  tf_opt = std::nullopt;
  tf_opt = wait_for_tf("camera", "uav66/local_origin", tfr);
  if (tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "got the transform (that should not happen)");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

/* TEST_F(Test, empty_frame_test) //{ */

TEST_F(Test, default_frame_test) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  int result = 1;

  auto tfr = mrs_lib::Transformer(node_);

  tfr.setDefaultPrefix("uav66");
  tfr.setLookupTimeout(rclcpp::Duration(std::chrono::duration<double>(0.1)));

  const rclcpp::Time t = node_->get_clock()->now();
  publish_transforms(t);

  auto tf_opt = tfr.getTransform("camera", "");
  if (tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "got the transform (that should not happen)");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tf_opt = tfr.getTransform("", "fcu");
  if (tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "got the transform (that should not happen)");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tf_opt = tfr.getTransform("", "");
  if (tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "got the transform (that should not happen)");
    result *= 0;
  }

  RCLCPP_INFO(node_->get_logger(), "Setting the default frame name");
  tfr.setDefaultFrame("uav66/fcu");
  tf_opt = std::nullopt;
  tf_opt = tfr.getTransform("", "");
  if (!tf_opt.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "didn't get the transformation!");
    result *= 0;
  }

  EXPECT_TRUE(result);

  despin();
}

//}

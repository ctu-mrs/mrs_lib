#include <mrs_lib/msg_extractor.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gtest/gtest.h>

/* TEST(TESTSuite, point_position) //{ */

TEST(TESTSuite, point_position) {

  geometry_msgs::msg::Point msg;

  msg.x = 1;
  msg.y = 2;
  msg.z = 3;

  auto [x, y, z] = mrs_lib::getXYZ(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);
}

//}

/* TEST(TESTSuite, point_sharedptr_position) //{ */

TEST(TESTSuite, point_sharedptr_position) {

  geometry_msgs::msg::Point::SharedPtr msg = std::make_shared<geometry_msgs::msg::Point>();

  msg->x = 1;
  msg->y = 2;
  msg->z = 3;

  auto [x, y, z] = mrs_lib::getXYZ(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);
}

//}

/* TEST(TESTSuite, vector3_position) //{ */

TEST(TESTSuite, vector_position) {

  geometry_msgs::msg::Vector3 msg;

  msg.x = 1;
  msg.y = 2;
  msg.z = 3;

  auto [x, y, z] = mrs_lib::getXYZ(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);
}

//}

/* TEST(TESTSuite, vector3_sharedptr_position) //{ */

TEST(TESTSuite, vector3_sharedptr_position) {

  geometry_msgs::msg::Vector3::SharedPtr msg = std::make_shared<geometry_msgs::msg::Vector3>();

  msg->x = 1;
  msg->y = 2;
  msg->z = 3;

  auto [x, y, z] = mrs_lib::getXYZ(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);
}

//}

/* TEST(TESTSuite, pose) //{ */

TEST(TESTSuite, pose) {

  geometry_msgs::msg::Pose msg;

  msg.position.x = 1;
  msg.position.y = 2;
  msg.position.z = 3;

  msg.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setHeading(0.1);

  auto [x, y, z] = mrs_lib::getPosition(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);

  double heading = mrs_lib::getHeading(msg);

  EXPECT_NEAR(heading, 0.1, 1e-3);

  msg.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setYaw(0.2);

  double yaw = mrs_lib::getYaw(msg);

  EXPECT_NEAR(yaw, 0.2, 1e-3);
}

//}

/* TEST(TESTSuite, pose_shared_ptr) //{ */

TEST(TESTSuite, pose_shared_ptr) {

  geometry_msgs::msg::Pose::SharedPtr msg = std::make_shared<geometry_msgs::msg::Pose>();

  msg->position.x = 1;
  msg->position.y = 2;
  msg->position.z = 3;

  msg->orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setHeading(0.1);

  auto [x, y, z] = mrs_lib::getPosition(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);

  double heading = mrs_lib::getHeading(msg);

  EXPECT_NEAR(heading, 0.1, 1e-3);

  msg->orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setYaw(0.2);

  double yaw = mrs_lib::getYaw(msg);

  EXPECT_NEAR(yaw, 0.2, 1e-3);
}

//}

/* TEST(TESTSuite, pose_with_covariance) //{ */

TEST(TESTSuite, pose_with_covariance) {

  geometry_msgs::msg::PoseWithCovariance msg;

  msg.pose.position.x = 1;
  msg.pose.position.y = 2;
  msg.pose.position.z = 3;

  msg.pose.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setHeading(0.1);

  auto [x, y, z] = mrs_lib::getPosition(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);

  double heading = mrs_lib::getHeading(msg);

  EXPECT_NEAR(heading, 0.1, 1e-3);

  msg.pose.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setYaw(0.2);

  double yaw = mrs_lib::getYaw(msg);

  EXPECT_NEAR(yaw, 0.2, 1e-3);
}

//}

/* TEST(TESTSuite, pose_with_covariance_shared_ptr) //{ */

TEST(TESTSuite, pose_with_covariance_shared_ptr) {

  geometry_msgs::msg::PoseWithCovariance::SharedPtr msg = std::make_shared<geometry_msgs::msg::PoseWithCovariance>();

  msg->pose.position.x = 1;
  msg->pose.position.y = 2;
  msg->pose.position.z = 3;

  msg->pose.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setHeading(0.1);

  auto [x, y, z] = mrs_lib::getPosition(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);

  double heading = mrs_lib::getHeading(msg);

  EXPECT_NEAR(heading, 0.1, 1e-3);

  msg->pose.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setYaw(0.2);

  double yaw = mrs_lib::getYaw(msg);

  EXPECT_NEAR(yaw, 0.2, 1e-3);
}

//}

/* TEST(TESTSuite, twist) //{ */

TEST(TESTSuite, twist) {

  geometry_msgs::msg::Twist msg;

  msg.linear.x = 1;
  msg.linear.y = 2;
  msg.linear.z = 3;

  auto [x, y, z] = mrs_lib::getVelocity(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);
}

//}

/* TEST(TESTSuite, twist_ptr) //{ */

TEST(TESTSuite, twist_ptr) {

  std::shared_ptr<geometry_msgs::msg::Twist> msg = std::make_shared<geometry_msgs::msg::Twist>();

  msg->linear.x = 1;
  msg->linear.y = 2;
  msg->linear.z = 3;

  auto [x, y, z] = mrs_lib::getVelocity(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);
}

//}

/* TEST(TESTSuite, twist_with_covariance) //{ */

TEST(TESTSuite, twist_with_covariance) {

  geometry_msgs::msg::TwistWithCovariance msg;

  msg.twist.linear.x = 1;
  msg.twist.linear.y = 2;
  msg.twist.linear.z = 3;

  auto [x, y, z] = mrs_lib::getVelocity(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);
}

//}

/* TEST(TESTSuite, twist_with_covariance_ptr) //{ */

TEST(TESTSuite, twist_with_covariance_ptr) {

  std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> msg = std::make_shared<geometry_msgs::msg::TwistWithCovariance>();

  msg->twist.linear.x = 1;
  msg->twist.linear.y = 2;
  msg->twist.linear.z = 3;

  auto [x, y, z] = mrs_lib::getVelocity(msg);

  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(z, 3);
}

//}

/* TEST(TESTSuite, odometry) //{ */

TEST(TESTSuite, odometry) {

  nav_msgs::msg::Odometry msg;

  msg.pose.pose.position.x = 1;
  msg.pose.pose.position.y = 2;
  msg.pose.pose.position.z = 3;

  msg.twist.twist.linear.x = 4;
  msg.twist.twist.linear.y = 5;
  msg.twist.twist.linear.z = 6;

  msg.pose.pose.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setHeading(0.1);

  {
    auto [x, y, z] = mrs_lib::getPosition(msg);

    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, 2);
    EXPECT_EQ(z, 3);
  }

  {
    auto [x, y, z] = mrs_lib::getVelocity(msg);

    EXPECT_EQ(x, 4);
    EXPECT_EQ(y, 5);
    EXPECT_EQ(z, 6);
  }

  {
    double heading = mrs_lib::getHeading(msg);

    EXPECT_NEAR(heading, 0.1, 1e-3);
  }

  msg.pose.pose.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setYaw(0.2);

  {
    double yaw = mrs_lib::getYaw(msg);

    EXPECT_NEAR(yaw, 0.2, 1e-3);
  }
}

//}

/* TEST(TESTSuite, odometry_ptr) //{ */

TEST(TESTSuite, odometry_ptr) {

  nav_msgs::msg::Odometry::SharedPtr msg = std::make_shared<nav_msgs::msg::Odometry>();

  msg->pose.pose.position.x = 1;
  msg->pose.pose.position.y = 2;
  msg->pose.pose.position.z = 3;

  msg->twist.twist.linear.x = 4;
  msg->twist.twist.linear.y = 5;
  msg->twist.twist.linear.z = 6;

  msg->pose.pose.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setHeading(0.1);

  {
    auto [x, y, z] = mrs_lib::getPosition(msg);

    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, 2);
    EXPECT_EQ(z, 3);
  }

  {
    auto [x, y, z] = mrs_lib::getVelocity(msg);

    EXPECT_EQ(x, 4);
    EXPECT_EQ(y, 5);
    EXPECT_EQ(z, 6);
  }

  {
    double heading = mrs_lib::getHeading(msg);

    EXPECT_NEAR(heading, 0.1, 1e-3);
  }

  msg->pose.pose.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setYaw(0.2);

  {
    double yaw = mrs_lib::getYaw(msg);

    EXPECT_NEAR(yaw, 0.2, 1e-3);
  }
}

//}

/* TEST(TESTSuite, tracker_command) //{ */

TEST(TESTSuite, tracker_command) {

  mrs_msgs::msg::TrackerCommand msg;

  msg.position.x = 1;
  msg.position.y = 2;
  msg.position.z = 3;

  msg.velocity.x = 4;
  msg.velocity.y = 5;
  msg.velocity.z = 6;

  {
    auto [x, y, z] = mrs_lib::getPosition(msg);

    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, 2);
    EXPECT_EQ(z, 3);
  }

  {
    auto [x, y, z] = mrs_lib::getVelocity(msg);

    EXPECT_EQ(x, 4);
    EXPECT_EQ(y, 5);
    EXPECT_EQ(z, 6);
  }

  {
    msg.use_heading     = true;
    msg.use_orientation = false;

    msg.heading = 0.1;

    double heading = mrs_lib::getHeading(msg);

    EXPECT_NEAR(heading, 0.1, 1e-3);
  }

  {
    msg.use_heading     = false;
    msg.use_orientation = true;

    msg.orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setHeading(0.15);

    double heading = mrs_lib::getHeading(msg);

    EXPECT_NEAR(heading, 0.15, 1e-3);
  }
}

//}

/* TEST(TESTSuite, tracker_command_ptr) //{ */

TEST(TESTSuite, tracker_command_ptr) {

  mrs_msgs::msg::TrackerCommand::SharedPtr msg = std::make_shared<mrs_msgs::msg::TrackerCommand>();

  msg->position.x = 1;
  msg->position.y = 2;
  msg->position.z = 3;

  msg->velocity.x = 4;
  msg->velocity.y = 5;
  msg->velocity.z = 6;

  msg->heading = 0.1;

  {
    auto [x, y, z] = mrs_lib::getPosition(msg);

    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, 2);
    EXPECT_EQ(z, 3);
  }

  {
    auto [x, y, z] = mrs_lib::getVelocity(msg);

    EXPECT_EQ(x, 4);
    EXPECT_EQ(y, 5);
    EXPECT_EQ(z, 6);
  }

  {
    msg->use_heading     = true;
    msg->use_orientation = false;

    msg->heading = 0.1;

    double heading = mrs_lib::getHeading(msg);

    EXPECT_NEAR(heading, 0.1, 1e-3);
  }

  {
    msg->use_heading     = false;
    msg->use_orientation = true;

    msg->orientation = mrs_lib::AttitudeConverter(0.1, 0.2, 0.3).setHeading(0.15);

    double heading = mrs_lib::getHeading(msg);

    EXPECT_NEAR(heading, 0.15, 1e-3);
  }
}

//}

/* TEST(TESTSuite, reference) //{ */

TEST(TESTSuite, reference) {

  mrs_msgs::msg::Reference msg;

  msg.position.x = 1;
  msg.position.y = 2;
  msg.position.z = 3;

  {
    auto [x, y, z] = mrs_lib::getPosition(msg);

    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, 2);
    EXPECT_EQ(z, 3);
  }

  {
    msg.heading = 0.1;

    double heading = mrs_lib::getHeading(msg);

    EXPECT_EQ(heading, 0.1);
  }
}

//}

/* TEST(TESTSuite, reference_ptr) //{ */

TEST(TESTSuite, reference_ptr) {

  mrs_msgs::msg::Reference::SharedPtr msg = std::make_shared<mrs_msgs::msg::Reference>();

  msg->position.x = 1;
  msg->position.y = 2;
  msg->position.z = 3;

  {
    auto [x, y, z] = mrs_lib::getPosition(msg);

    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, 2);
    EXPECT_EQ(z, 3);
  }

  {
    msg->heading = 0.1;

    double heading = mrs_lib::getHeading(msg);

    EXPECT_EQ(heading, 0.1);
  }
}

//}

/* TEST(TESTSuite, reference_stamped) //{ */

TEST(TESTSuite, reference_stamped) {

  mrs_msgs::msg::ReferenceStamped msg;

  msg.reference.position.x = 1;
  msg.reference.position.y = 2;
  msg.reference.position.z = 3;

  {
    auto [x, y, z] = mrs_lib::getPosition(msg);

    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, 2);
    EXPECT_EQ(z, 3);
  }

  {
    msg.reference.heading = 0.1;

    double heading = mrs_lib::getHeading(msg);

    EXPECT_EQ(heading, 0.1);
  }
}

//}

/* TEST(TESTSuite, reference_stamped_ptr) //{ */

TEST(TESTSuite, reference_stamped_ptr) {

  mrs_msgs::msg::ReferenceStamped::SharedPtr msg = std::make_shared<mrs_msgs::msg::ReferenceStamped>();

  msg->reference.position.x = 1;
  msg->reference.position.y = 2;
  msg->reference.position.z = 3;

  {
    auto [x, y, z] = mrs_lib::getPosition(msg);

    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, 2);
    EXPECT_EQ(z, 3);
  }

  {
    msg->reference.heading = 0.1;

    double heading = mrs_lib::getHeading(msg);

    EXPECT_EQ(heading, 0.1);
  }
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

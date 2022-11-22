#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/twist_with_covariance.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace
{
double calcTwistNorm(const geometry_msgs::msg::Twist & twist)
{
  return tier4_autoware_utils::calcNorm(twist.linear);
}

double calcTwistNorm(const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance)
{
  return tier4_autoware_utils::calcNorm(twist_with_covariance.twist.linear);
}

Eigen::Vector2d getVector2d(const geometry_msgs::msg::Twist & twist)
{
  return Eigen::Vector2d{twist.linear.x, twist.linear.y};
}

Eigen::Vector2d getVector2d(const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance)
{
  return getVector2d(twist_with_covariance.twist);
}

geometry_msgs::msg::Twist createTwist(const Eigen::Vector2d & vector)
{
  auto linear = tier4_autoware_utils::createVector3(vector.x(), vector.y(), 0.0);
  auto angular = tier4_autoware_utils::createVector3(0.0, 0.0, 0.0);
  return tier4_autoware_utils::createTwist(linear, angular);
}

geometry_msgs::msg::TwistWithCovariance createTwistWithCovariance(
  const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::TwistWithCovariance twist_with_covariance;
  twist_with_covariance.twist = twist;
  return twist_with_covariance;
}

geometry_msgs::msg::TwistWithCovariance createTwistWithCovariance(
  const geometry_msgs::msg::Vector3 & velocity, geometry_msgs::msg::Vector3 & angular)
{
  return createTwistWithCovariance(tier4_autoware_utils::createTwist(velocity, angular));
}

geometry_msgs::msg::Vector3 getTransformedVelocity(
  const geometry_msgs::msg::Vector3 velocity,
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform)
{
  geometry_msgs::msg::Vector3Stamped velocity_stamped{};
  velocity_stamped.vector = velocity;
  geometry_msgs::msg::Vector3Stamped transformed_velocity_stamped{};
  tf2::doTransform(velocity_stamped, transformed_velocity_stamped, *transform);
  return velocity_stamped.vector;
}
}  // namespace

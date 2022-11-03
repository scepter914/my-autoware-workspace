
#include "my_utils/my_utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <geometry_msgs/msg/twist_with_covariance.hpp>

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

Eigen::Vector2d toVector2d(const geometry_msgs::msg::Twist & twist)
{
  return Eigen::Vector2d{twist.linear.x, twist.linear.y};
}

Eigen::Vector2d toVector2d(const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance)
{
  return toVector2d(twist_with_covariance.twist);
}

geometry_msgs::msg::Twist toTwist(const Eigen::Vector2d & vector)
{
  geometry_msgs::msg::Twist twist;
  twist.linear = tier4_autoware_utils::createVector3(vector(0), vector(1), 0.0);
  return twist;
}

geometry_msgs::msg::Twist toTwistWithCovariance(const Eigen::Vector2d & vector)
{
  return tier4_autoware_utils::createTwist(
    tier4_autoware_utils::createVector3(vector.x, vector.y, 0.0),
    tier4_autoware_utils::createVector3(0.0, 0.0, 0.0));
}
}  // namespace

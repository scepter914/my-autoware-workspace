
#include "my_utils/my_utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <geometry_msgs/msg/twist_with_covariance.hpp>

namespace
{
inline double calcTwistNorm(const geometry_msgs::msg::Twist & twist)
{
  return calcTwistNorm(twist.linear);
}

inline double calcTwistNorm(const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance)
{
  return calcTwistNorm(twist_with_covariance.twist.linear);
}

inline geometry_msgs::msg::TwistWithCovariance createTwistWithCovariance(
  const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::TwistWithCovariance twist_with_covariance;
  twist_with_covariance.twist = twist;
  return twist_with_covariance;
}

inline geometry_msgs::msg::TwistWithCovariance createTwistWithCovariance(
  const geometry_msgs::msg::Vector3 & velocity, geometry_msgs::msg::Vector3 & angular)
{
  return createTwistWithCovariance(tier4_autoware_utils::createTwist(velocity, angular));
}
}  // namespace

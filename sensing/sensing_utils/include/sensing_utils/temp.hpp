#include <algorithm>
#include <boost/geometry.hpp>
#include <vector>

#include "perception_utils/geometry.hpp"
#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

namespace perception_utils
{

}  // namespace perception_utils

namespace tier4_autoware_utils
{
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::Point2d;

LinearRing2d toLinearRing2d(DetectedObject object)
{
  return transformVector(
    createRectangle(object.shape.dimensions.x, object.shape.dimensions.y),
    tier4_autoware_utils::pose2transform(object.kinematics.pose_with_covariance.pose));
}

LinearRing2d createRectangle(const float x, const float y)
{
  const double x_front = x / 2.0;
  const double x_rear = x / 2.0;
  const double y_left = y / 2.0;
  const double y_right = y / 2.0;

  LinearRing2d box{};
  box.push_back(Point2d{x_front, y_left});
  box.push_back(Point2d{x_front, y_right});
  box.push_back(Point2d{x_rear, y_right});
  box.push_back(Point2d{x_rear, y_left});
  box.push_back(Point2d{x_front, y_left});

  return box;
}

// to radar fusion
/*
LinearRing2d getLinearRing2dWithMargin(DetectedObject object, float margin_x, float margin_y)
{
  auto p = object.shape.dimensions;
  linear_ring = transformVector(
    createRectangle(p.x + margin_x, p.y + margin_y),
    tier4_autoware_utils::pose2transform(object.kinematics.pose_with_covariance.pose));
  return linear_ring;
*/

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovariance;

inline double getTwistNorm(const TwistWithCovariance & twist_with_covariance)
{
  return getTwistNorm(twist_with_covariance.twist);
}

inline double getTwistNorm(const Twist & twist)
{
  auto v = twist.linear;
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/// Judge whether twist covariance is available.
inline bool isTwistCovarianceAvailable(const TwistWithCovariance & twist_with_covariance)
{
  using IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  auto covariance = twist_with_covariance.covariance;
  if (covariance[IDX::X_X] == 0.0 && covariance[IDX::Y_Y] == 0.0 && covariance[IDX::Z_Z] == 0.0) {
    return false;
  } else {
    return true;
  }
}

inline TwistWithCovariance toTwistWithCovariance(const Twist & twist)
{
  TwistWithCovariance twist_with_covariance;
  twist_with_covariance.twist = twist;
  return twist_with_covariance;
}

inline TwistWithCovariance toTwistWithCovariance(const geometry_msgs::msg::Vector3 & velocity)
{
  TwistWithCovariance twist_with_covariance;
  twist_with_covariance.twist.linear = velocity;
  return twist_with_covariance;
}

inline Twist toTwist(const geometry_msgs::msg::Vector3 & velocity)
{
  Twist twist;
  twist.linear = velocity;
  return twist;
}
}  // namespace tier4_autoware_utils

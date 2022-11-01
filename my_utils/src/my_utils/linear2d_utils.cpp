
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "autoware_auto_perception_msgs/msg/detected_object.hpp"

namespace
{

using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::Point2d;

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

LinearRing2d toLinearRing2d(
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return transformVector(
      createRectangle(shape.dimensions.x, shape.dimensions.y),
      tier4_autoware_utils::pose2transform(pose));
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    // TODO
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    // TODO
  }
}

LinearRing2d toLinearRing2d(const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  return toLinearRing2d(object.kinematics.pose_with_covariance.pose, object.shape);
}

LinearRing2d getLinearRing2dWithMargin(
  autoware_auto_perception_msgs::msg::DetectedObject & object, float margin_x, float margin_y)
{
  auto p = object.shape.dimensions;
  auto linear_ring = transformVector(
    createRectangle(p.x + margin_x, p.y + margin_y),
    tier4_autoware_utils::pose2transform(object.kinematics.pose_with_covariance.pose));
  return linear_ring;
}
}  // namespace

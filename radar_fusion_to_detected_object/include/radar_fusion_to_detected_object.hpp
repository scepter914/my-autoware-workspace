/*
 * Copyright 2022 TIER IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RADAR_FUSION_TO_DETECTED_OBJECT__RADAR_FUSION_TO_DETECTED_OBJECT_HPP__
#define RADAR_FUSION_TO_DETECTED_OBJECT__RADAR_FUSION_TO_DETECTED_OBJECT_HPP__

#include "rclcpp/logger.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "std_msgs/msg/header.hpp"

#include <memory>
#include <string>
#include <vector>

namespace radar_fusion_to_detected_object
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovariance;
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::Point2d;

class RadarFusionToDetectedObject
{
public:
  explicit RadarFusionToDetectedObject(const rclcpp::Logger & logger) : logger_(logger) {}

  struct Param
  {
    // Radar fusion param
    double bounding_box_margin{};
    double split_threshold_velocity{};

    // Weight param for velocity estimation
    double velocity_weight_average{};
    double velocity_weight_median{};
    double velocity_weight_target_value_average{};
    double velocity_weight_target_value_top{};

    // Parameters for fixing object information
    bool convert_doppler_to_twist{};
    double threshold_probability{};
  };

  struct RadarInput
  {
    std_msgs::msg::Header header{};
    PoseWithCovariance pose_with_covariance{};
    TwistWithCovariance twist_with_covariance{};
    double target_value{};
  };

  struct Input
  {
    std::vector<RadarInput> radars;
    DetectedObjects objects{};
  };

  struct Output
  {
    DetectedObjects objects{};
  };

  void setParam(const Param & param);
  Output update(const Input & input);

private:
  rclcpp::Logger logger_;
  Param param_{};
  std::vector<RadarInput> filterRadarWithinObject(
    const DetectedObject & object, const std::vector<RadarInput> & radars);
  std::vector<DetectedObject> splitObject(
    const DetectedObject & object, const std::vector<RadarInput> & radars);
  TwistWithCovariance estimateTwist(
    const DetectedObject & object, const std::vector<RadarInput> & radars);
  bool isQualified(const DetectedObject & object, const std::vector<RadarInput> & radars);
  TwistWithCovariance convertDopplerToTwist(
    const DetectedObject & object, const TwistWithCovariance & twist_with_covariance);
  Twist addTwist(const Twist & twist_1, const Twist & twist_2);
  Twist scaleTwist(const Twist & twist, const double scale);
  double getTwistNorm(const Twist & twist);
  Twist sumTwist(const std::vector<Twist> & twists);
};
}  // namespace radar_fusion_to_detected_object

namespace tier4_autoware_utils
{
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::Point2d;

inline LinearRing2d createObject2d(const Point2d object_size, const double margin = 0.0)
{
  const double x_front = object_size.x() / 2.0 + margin;
  const double x_rear = -object_size.x() / 2.0 - margin;
  const double y_left = object_size.y() / 2.0 + margin;
  const double y_right = -object_size.y() / 2.0 - margin;

  LinearRing2d box{};
  box.push_back(Point2d{x_front, y_left});
  box.push_back(Point2d{x_front, y_right});
  box.push_back(Point2d{x_rear, y_right});
  box.push_back(Point2d{x_rear, y_left});
  box.push_back(Point2d{x_front, y_left});

  return box;
}
}  // namespace tier4_autoware_utils

#endif  // RADAR_FUSION_TO_DETECTED_OBJECT__RADAR_FUSION_TO_DETECTED_OBJECT_HPP__

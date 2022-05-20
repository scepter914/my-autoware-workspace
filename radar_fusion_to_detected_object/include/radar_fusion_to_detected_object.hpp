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
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovariance;

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
    double velocity_weight_median{};
    double velocity_weight_average{};
    double velocity_weight_target_value_average{};
    double velocity_weight_top_target_value{};

    // Parameters for fixing object information
    bool convert_doppler_to_twist{};
    double threshold_probability{};
  };

  struct RadarInput
  {
    std_msgs::msg::Header header;
    PoseWithCovariance pose_with_covariance;
    TwistWithCovariance twist_with_covariance;
    double target_value;
  };

  struct Input
  {
    std::vector<RadarInput> radars;
    DetectedObjects objects;
  };

  struct Output
  {
    DetectedObjects objects;
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
    const DetectedObject & object, std::vector<RadarInput> & radars);
  bool isQualified(const DetectedObject & object);
  TwistWithCovariance convertDopplerToTwist(
    DetectedObject & object, TwistWithCovariance & twist_with_covariance);
  Twist addTwist(Twist & twist_1, Twist & twist_2);
  Twist scaleTwist(Twist & twist, double scale);
  Twist scaleTwist(Twist & twist, double scale);

}  // namespace radar_fusion_to_detected_object

#endif  // RADAR_FUSION_TO_DETECTED_OBJECT__RADAR_FUSION_TO_DETECTED_OBJECT_HPP__

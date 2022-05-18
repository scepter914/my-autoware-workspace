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

#include "radar_fusion_to_detected_object.hpp"
#include "rclcpp/logger.hpp"

#include <memory>
#include <string>
#include <vector>

/*
namespace radar_fusion_to_detected_object
{
class RadarFusionToDetectedObject
{
public:
  explicit RadarFusionToDetectedObject(const rclcpp::Logger & logger) : logger_(logger) {}

  struct Input
  {
    int data{};
  };

  struct Output
  {
    int data{};
  };

  struct Param
  {
    int data{};
  };

  void setParam(const Param & param) { param_ = param; }

  Output update(const Input & input);

private:
  rclcpp::Logger logger_;
  Param param_{};
};

}  // namespace radar_fusion_to_detected_object

#endif  // RADAR_FUSION_TO_DETECTED_OBJECT__RADAR_FUSION_TO_DETECTED_OBJECT_HPP__

/*
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <geometry_msgs/PoseWithCovariance.h>

#pragma once

namespace radar_fusion_to_3dbbox
{
struct RadarFusionTo3dbboxParam
{
  // weight param
  double velocity_weight_median;
  double velocity_weight_average;
  double velocity_weight_confidence_average;
  double velocity_weight_top_confidence;

  // other param
  double eps;
  double bounding_box_margin;
  double threshold_high_confidence;
  double threshold_low_confidence;
  bool with_twist_reliable;
};

struct RadarInput
{
  geometry_msgs::Pose pose;
  double doppler_velocity;
  double confidence;
  std_msgs::Header header;
};

struct RadarFusionInput
{
  std::vector<RadarInput> radars;
  autoware_perception_msgs::DynamicObjectWithFeatureArray objects;
};

struct RadarFusionOutput
{
  autoware_perception_msgs::DynamicObjectWithFeatureArray objects;
};

class RadarFusionTo3dbbox
{
public:
  void setParam(const RadarFusionTo3dbboxParam & param);
  RadarFusionOutput update(const RadarFusionInput & input_);

private:
  RadarFusionTo3dbboxParam param_;
  std::vector<RadarInput> filterRadarWithinObject(
    const autoware_perception_msgs::DynamicObjectWithFeature & object,
    const std::vector<RadarInput> & radars);
  double estimateVelocity(std::vector<RadarInput> & radars);
  autoware_perception_msgs::DynamicObjectWithFeature mergeDoppler(
    const autoware_perception_msgs::DynamicObjectWithFeature & object, const double velocity,
    const double yaw);
  autoware_perception_msgs::DynamicObjectWithFeatureArray fuseRadarTo3dbbox(
    const autoware_perception_msgs::DynamicObjectWithFeature & object,
    std::vector<RadarInput> & radars);
};

}  // namespace radar_fusion_to_3dbbox
*/

#endif  // RADAR_FUSION_TO_DETECTED_OBJECT__RADAR_FUSION_TO_DETECTED_OBJECT_HPP__

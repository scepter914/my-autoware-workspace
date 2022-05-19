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

#include "radar_fusion_to_detected_object.hpp"

#include "autoware_utils/geometry/boost_geometry.h"
#include "autoware_utils/geometry/geometry.h"

#include <boost/geometry.hpp>

#include <algorithm>
#include <iostream>
#include <numeric>

namespace radar_fusion_to_detected_object
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::TwistWithCovariance;

void RadarFusionToDetectedObject::setParam(const RadarFusionToDetectedObjectParam & param)
{
  // Radar fusion param
  param_.bounding_box_margin = param.bounding_box_margin;
  param_.split_threshold_velocity = param.split_threshold_velocity;

  // normalize weight param
  double sum_weight = param.velocity_weight_median + param.velocity_weight_average +
                      param.velocity_weight_target_value_average +
                      param.velocity_weight_top_target_value;
  if (sum_weight < 0.01) {
    param_.velocity_weight_median = 1.0;
    param_.velocity_weight_average = 0.0;
    param_.velocity_weight_target_value_average = 0.0;
    param_.velocity_weight_top_target_value = 0.0;
  } else {
    param_.velocity_weight_median = param.velocity_weight_median / sum_weight;
    param_.velocity_weight_average = param.velocity_weight_average / sum_weight;
    param_.velocity_weight_target_value_average =
      param.velocity_weight_target_value_average / sum_weight;
    param_.velocity_weight_top_target_value = param.velocity_weight_top_target_value / sum_weight;
  }

  // Parameters for fixing object information
  param_.threshold_probability = param.threshold_probability;
  param_.convert_doppler_to_twist = param.convert_doppler_to_twist;
}

RadarFusionToDetectedObject::Output RadarFusionToDetectedObject::update(
  const RadarFusionToDetectedObject::Input & input)
{
  RadarFusionToDetectedObject::Output output;
  output_.objects.header = input_.objects.header;

  for (const auto & object : input_.objects.objects) {
    // Link between 3d bounding box and radar data
    std::vector<RadarInput> radars_within_object = filterRadarWithinObject(object, input_.radars);

    // Split the object going in a different direction
    std::vector<DetectedObject> split_objects = splitObject(object, radars_within_object);

    for (auto & split_object : split_objects) {
      std::vector<RadarInput> radars_within_split_object;
      if (split_objects.size() == 1) {
        // If object is not split, radar data within object is same
        radars_within_split_object = radars_within_object;
      } else {
        // If object is split, then filter radar again
        radars_within_split_object = filterRadarWithinObject(object, radars_within_object);
      }

      // Estimate twist of object
      split_object.kinematics.has_twist = true;
      split_object.kinematics.twist_with_covariance =
        estimateTwist(split_object, radars_within_split_object);

      // Delete objects with low probability
      if (isQualified(split_object)) {
        output_.objects.objects.emplaced_back(split_object);
      }
    }
  }
  return output;
}

bool RadarFusionToDetectedObject::isQualified(const DetectedObject & object)
{
  if (split_object.classification[0].probability > param_.threshold_probability) {
    return true;
  } else {
    if (!radars_within_object.empty()) {
      return true;
    } else {
      return false;
    }
  }
}

}  // namespace radar_fusion_to_detected_object

/*

std::vector<RadarInput> RadarFusionToDetectedObject::filterRadarWithinObject(
  const autoware_perception_msgs::DynamicObjectWithFeature & object,
  const std::vector<RadarInput> & radars)
{
  std::vector<RadarInput> filtered_radars;
  autoware_utils::Point2d object_size{
    object.object.shape.dimensions.x, object.object.shape.dimensions.y};
  autoware_utils::LinearRing2d object_box =
    autoware_utils::createObject2d(object_size, param_.bounding_box_margin);
  object_box = autoware_utils::transformVector(
    object_box, autoware_utils::pose2transform(object.object.state.pose_covariance.pose));
  ROS_DEBUG(
    "object box 0 x: %f, y: %f, object box 1 x: %f, y: %f", object_box.at(0).x(),
    object_box.at(0).y(), object_box.at(1).x(), object_box.at(1).y());
  for (const auto & ra : radars) {
    autoware_utils::Point2d radar_point{ra.pose.position.x, ra.pose.position.y};
    if (boost::geometry::within(radar_point, object_box)) {
      filtered_radars.push_back(ra);
    }
  }
  // remove the radar data whose frame_id is lower than the most
  // [TODO] frame id filter
  // std::string frame_id = radars.at(0).header.frame_id;
  return filtered_radars;
}

double RadarFusionToDetectedObject::estimateVelocity(std::vector<RadarInput> & radars)
{
  double estimated_velocity;
  if (radars.empty()) {
    estimated_velocity = 0.0;
    return estimated_velocity;
  }

  // calculate median
  double doppler_median = 0.0;
  auto ascending_func = [](const RadarInput & a, const RadarInput & b) {
    return a.doppler_velocity < b.doppler_velocity;
  };
  std::sort(radars.begin(), radars.end(), ascending_func);

  if (radars.size() % 2 == 1) {
    int median_index = (radars.size() - 1) / 2;
    doppler_median = radars.at(median_index).doppler_velocity;
  } else {
    int median_index = radars.size() / 2;
    doppler_median =
      (radars.at(median_index - 1).doppler_velocity + radars.at(median_index).doppler_velocity) *
      0.5;
  }

  // calculate average
  double doppler_average = 0.0;
  if (param_.velocity_weight_average > 0.0) {
    auto add_v_func = [](const double & a, RadarInput & b) { return a + b.doppler_velocity; };
    doppler_average =
      std::accumulate(std::begin(radars), std::end(radars), 0.0, add_v_func) / radars.size();
  }

  // calculate top target_value
  double doppler_top_target_value = 0.0;
  if (param_.velocity_weight_top_target_value > 0.0) {
    auto comp_func = [](const RadarInput & a, const RadarInput & b) {
      return a.target_value < b.target_value;
    };
    auto iter = std::max_element(std::begin(radars), std::end(radars), comp_func);
    doppler_top_target_value = iter->target_value;
  }

  // calculate target_value * average
  double doppler_target_value_average = 0.0;
  if (param_.velocity_weight_target_value_average > 0.0) {
    auto add_target_value_func = [](const double & a, RadarInput & b) { return a + b.target_value;
}; double sum_target_value = std::accumulate(std::begin(radars), std::end(radars), 0.0,
add_target_value_func); auto add_target_value_vel_func = [](const double & a, RadarInput & b) {
      return a + b.target_value * b.doppler_velocity;
    };
    doppler_target_value_average =
      std::accumulate(std::begin(radars), std::end(radars), 0.0, add_target_value_vel_func) /
      sum_target_value;
  }

  // estimate doppler velocity with cost weight
  estimated_velocity = param_.velocity_weight_median * doppler_median +
                       param_.velocity_weight_average * doppler_average +
                       param_.velocity_weight_target_value_average * doppler_target_value_average
+ param_.velocity_weight_top_target_value * doppler_top_target_value;

  // Convert doppler velocity to twist
  if (param_.convert_doppler_to_twist) {
     twist_with_covariance = convertDopplerToTwist(output_object, twist_with_covariance)
  }
  return estimated_velocity;
}

autoware_perception_msgs::DynamicObjectWithFeature RadarFusionToDetectedObject::mergeDoppler(
  const autoware_perception_msgs::DynamicObjectWithFeature & object, const double velocity,
  const double yaw)
{
  autoware_perception_msgs::DynamicObjectWithFeature output;
  output = object;
  output.object.state.twist_reliable = param_.with_twist_reliable;
  output.object.state.twist_covariance.twist.linear.x = velocity * std::cos(yaw);
  output.object.state.twist_covariance.twist.linear.y = velocity * std::sin(yaw);

  return output;
}

autoware_perception_msgs::DynamicObjectWithFeatureArray
RadarFusionToDetectedObject::fuseRadarTo3dbbox( const
autoware_perception_msgs::DynamicObjectWithFeature & object, std::vector<RadarInput> &
radars_within_object)
{


if (!radars_within_object.empty()) {
  output_objects.feature_objects.emplace_back(mergeDoppler(object, velocity, yaw));
} else {
  // for debug
  auto object_debug = mergeDoppler(object, velocity, yaw);
  object_debug.object.semantic.target_value = 0.1;
  output_objects.feature_objects.emplace_back(object_debug);
}
}
// if (3d bbox is low target_value && not radar point in 3d bbox) then no-detection
// output is empty
return output_objects;
}

}  // namespace radar_fusion_to_3dbbox

*/

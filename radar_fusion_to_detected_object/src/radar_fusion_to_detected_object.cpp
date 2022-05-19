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

#include <autoware_radar_utils/autoware_radar_utils.hpp>
#include <radar_fusion_to_3dbbox.hpp>

#include <boost/geometry.hpp>

#include <autoware_utils/geometry/boost_geometry.h>
#include <autoware_utils/geometry/geometry.h>

#include <algorithm>
#include <iostream>
#include <numeric>

namespace radar_fusion_to_detected_object
{
RadarFusionToDetectedObject::Output RadarFusionToDetectedObject::update(
  const RadarFusionToDetectedObject::Input & input)
{
  RadarFusionToDetectedObject::Output output;

  // Sample
  output.data = input.data + param_.data;

  return output;
}

}  // namespace radar_fusion_to_detected_object

/*
namespace radar_fusion_to_3dbbox
{
void RadarFusionToDetectedObject::setParam(const RadarFusionToDetectedObjectParam & param)
{
  // other param
  param_.eps = 0.0001;
  param_.with_twist_reliable = param.with_twist_reliable;
  param_.bounding_box_margin = param.bounding_box_margin;
  param_.threshold_high_confidence = param.threshold_high_confidence;
  param_.threshold_low_confidence = param.threshold_low_confidence;

  // normalize weight param
  double sum_weight = param.velocity_weight_median + param.velocity_weight_average +
                      param.velocity_weight_confidence_average +
                      param.velocity_weight_top_confidence;
  if (sum_weight < param_.eps) {
    param_.velocity_weight_median = 1.0;
    param_.velocity_weight_average = 0.0;
    param_.velocity_weight_confidence_average = 0.0;
    param_.velocity_weight_top_confidence = 0.0;
  } else {
    param_.velocity_weight_median = param.velocity_weight_median / sum_weight;
    param_.velocity_weight_average = param.velocity_weight_average / sum_weight;
    param_.velocity_weight_confidence_average =
      param.velocity_weight_confidence_average / sum_weight;
    param_.velocity_weight_top_confidence = param.velocity_weight_top_confidence / sum_weight;
  }
}

RadarFusionOutput RadarFusionToDetectedObject::update(const RadarFusionInput & input_)
{
  RadarFusionOutput output_;
  output_.objects.header = input_.objects.header;
  for (const auto & ob : input_.objects.feature_objects) {
    std::vector<RadarInput> radar_within_objects = filterRadarWithinObject(ob, input_.radars);
    auto object_with_doppler = fuseRadarTo3dbbox(ob, radar_within_objects);
    if (!object_with_doppler.feature_objects.empty()) {
      for (const auto & ob_d : object_with_doppler.feature_objects) {
        output_.objects.feature_objects.emplace_back(ob_d);
      }
    }
  }
  return output_;
}

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
  if (param_.velocity_weight_average > param_.eps) {
    auto add_v_func = [](const double & a, RadarInput & b) { return a + b.doppler_velocity; };
    doppler_average =
      std::accumulate(std::begin(radars), std::end(radars), 0.0, add_v_func) / radars.size();
  }

  // calculate top confidence
  double doppler_top_confidence = 0.0;
  if (param_.velocity_weight_top_confidence > param_.eps) {
    auto comp_func = [](const RadarInput & a, const RadarInput & b) {
      return a.confidence < b.confidence;
    };
    auto iter = std::max_element(std::begin(radars), std::end(radars), comp_func);
    doppler_top_confidence = iter->confidence;
  }

  // calculate confidence * average
  double doppler_confidence_average = 0.0;
  if (param_.velocity_weight_confidence_average > param_.eps) {
    auto add_confidence_func = [](const double & a, RadarInput & b) { return a + b.confidence; };
    double sum_confidence =
      std::accumulate(std::begin(radars), std::end(radars), 0.0, add_confidence_func);
    auto add_confidence_vel_func = [](const double & a, RadarInput & b) {
      return a + b.confidence * b.doppler_velocity;
    };
    doppler_confidence_average =
      std::accumulate(std::begin(radars), std::end(radars), 0.0, add_confidence_vel_func) /
      sum_confidence;
  }

  // estimate doppler velocity with cost weight
  estimated_velocity = param_.velocity_weight_median * doppler_median +
                       param_.velocity_weight_average * doppler_average +
                       param_.velocity_weight_confidence_average * doppler_confidence_average +
                       param_.velocity_weight_top_confidence * doppler_top_confidence;

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
  autoware_perception_msgs::DynamicObjectWithFeatureArray output_objects;
  double yaw = 0.0;
  double velocity = estimateVelocity(radars_within_object);
  ROS_DEBUG(
    "ob_x: %f, vel: %f [m/s], yaw: %f [rad], Radar's size: %d",
    object.object.state.pose_covariance.pose.position.x, velocity, yaw,
    int(radars_within_object.size()));
  if (object.object.semantic.confidence > param_.threshold_high_confidence) {
    output_objects.feature_objects.emplace_back(mergeDoppler(object, velocity, yaw));
  } else if (object.object.semantic.confidence > param_.threshold_low_confidence) {
    //  TODO split_object
    // ob_1 = , vel_1 =
    // ob_2 = , vel_2 =
    // if (distance(ob_1.pose.position, ob_2.pose.position) > thres && std::abs(vel_1 - vel_2) >
thres
    // ){ output.append(mergeDoppler(ob_1, vel_1, yaw)); output.append(mergeDoppler(ob_2, vel_2,
yaw));
    //   return output;
    // } else if (!radars_within_object.empty()) {
    // }

if (!radars_within_object.empty()) {
  output_objects.feature_objects.emplace_back(mergeDoppler(object, velocity, yaw));
} else {
  // for debug
  auto object_debug = mergeDoppler(object, velocity, yaw);
  object_debug.object.semantic.confidence = 0.1;
  output_objects.feature_objects.emplace_back(object_debug);
}
}
// if (3d bbox is low confidence && not radar point in 3d bbox) then no-detection
// output is empty
return output_objects;
}

}  // namespace radar_fusion_to_3dbbox

*/

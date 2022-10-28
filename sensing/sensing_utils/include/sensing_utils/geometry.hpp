
// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace tier4_autoware_utils
{
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using radar_msgs::msg::RadarReturn;

RadarReturn compensateEgoVehicleTwist(const RadarReturn & radar)
{
  geometry_msgs::msg::PoseStamped radar_pose_stamped{};
  radar_pose_stamped.pose.position = getPoint(radar_track.position);

  geometry_msgs::msg::PoseStamped transformed_pose_stamped{};
  tf2::doTransform(radar_pose_stamped, transformed_pose_stamped, *transform_);
  kinematics.pose_with_covariance.pose = transformed_pose_stamped.pose;
}
}  // namespace tier4_autoware_utils

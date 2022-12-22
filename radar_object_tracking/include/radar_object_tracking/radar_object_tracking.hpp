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

#ifndef RADAR_OBJECT_TRACKING__RADAR_OBJECT_TRACKING_HPP__
#define RADAR_OBJECT_TRACKING__RADAR_OBJECT_TRACKING_HPP__

#include <Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <radar_msgs/msg/radar_scan.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace radar_object_tracking
{
using autoware_auto_perception_msgs::msg::TrackedObjects;
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

class RadarObjectTracking
{
public:
  explicit RadarObjectTracking(const rclcpp::Logger & logger) : logger_(logger) {}

  struct Input
  {
    RadarScan::ConstSharedPtr radar_scan{};
  };

  struct Output
  {
    TrackedObjects output_objects{};
  };

  struct Param
  {
    double update_rate{};
    int num_frame{};
    double clustering_range{};
    double min_object_x{};
    double min_object_y{};
    double min_object_z{};
    double min_sigma_doppler{};
    double min_sigma_range{};
    double max_object_acc{};
    double object_confidence{};
    int noise_thereshold_frame{};
  };

  struct Moment
  {
    double m_00 = 0.0;
    double m_10 = 0.0;
    double m_01 = 0.0;
    double m_11 = 0.0;
    double m_20 = 0.0;
    double m_02 = 0.0;
  };

  void setParam(const Param & param) { param_ = param; }
  void resisterPointcloud(const Input & input);

  Output update(const Input & input);

private:
  rclcpp::Logger logger_;
  Param param_{};

  // Data buffer
  //! cluster_id[cluster_index][frame_index][clustered_point_index] = point_index
  //! Point: buffer_points[frame_index].radar_pointclouds[point_index]
  std::vector<std::vector<std::vector<int>>> cluster_id;
  std::vector<RadarScan> buffer_points;

  // Function
  void updateBufferPoints(const RadarScan::ConstSharedPtr & input);
  void compensateEgoPosition();
  void clusterPointcloud();
  void clusterPointcloudOldFrame();
  void removeNoisePoint();
  bool ReflectFromSameObject(RadarReturn rpc1, RadarReturn rpc2);
  bool ReflectFromSameObjectWithOldFrame(RadarReturn rpc_new, RadarReturn rpc_old, double delta_t);
  TrackedObjects makeBoundingBoxes();
};

}  // namespace radar_object_tracking

#endif  // RADAR_OBJECT_TRACKING__RADAR_OBJECT_TRACKING_HPP__

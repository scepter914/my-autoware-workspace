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

#include "radar_object_tracking/radar_object_tracking.hpp"

namespace radar_object_tracking
{
RadarObjectTracking::Output RadarObjectTracking::update(const RadarObjectTracking::Input & input)
{
  RadarObjectTracking::Output output;

  // Sample
  output.data = input.data + param_.data;
  RCLCPP_INFO(rclcpp::get_logger("radar_object_tracking"), "Debug: %d", 0);
  return output;
}

/**
 * @fn
 * Update buffer radar pointcloud.
 * Add new radar pointcloud to buffer_points, and discard the old radar pointcloud.
 * @param radar_point_now: radar pointclouds getting this frame
 */
void RadarObjectTracking::updateBufferPoints(
  const autoware_radar_msgs::RadarPointcloudArray::ConstPtr & radar_point_now)
{
  cluster_id.clear();
  cluster_id.resize(0);
  /*
  if (param_.num_frame == 1) {
    // single frame object detection
    buffer_points.clear();
  } else if (buffer_points.size() > param_.num_frame - 1) {
    // multi frame object detection
    buffer_points.resize(param_.num_frame - 1);
  }
  */
  buffer_points.insert(buffer_points.begin(), *radar_point_now);
  buffer_points.reserve(param_.num_frame);
}

/**
 * @fn
 * Update old time frame's radar pointcloud position.
 * Modify radar pointclouds position to coordinate of latest using ego vehicle velocity.
 * @param twist:
 */
void RadarObjectTracking::compensateEgoPosition()
{
  // [TODO] implement
}

/**
 * @fn
 * Judge whether two radar points with same time frame reflect from same object.
 * If two radar points reflect from same object, return true.
 * @param rpc1: autoware_radar_msgs::RadarPointcloud
 * @param rpc2: autoware_radar_msgs::RadarPointcloud
 */
bool RadarObjectTracking::ReflectFromSameObject(
  autoware_radar_msgs::RadarPointcloud rpc1, autoware_radar_msgs::RadarPointcloud rpc2)
{
  auto p1 = autoware_utils::fromMsg(rpc1.point_covariance.pose.position).to_2d();
  auto p2 = autoware_utils::fromMsg(rpc2.point_covariance.pose.position).to_2d();
  if (boost::geometry::distance(p1, p2) < param_.clustering_range) {
    double doppler_th =
      std::max(std::min(rpc1.sigma_doppler, rpc2.sigma_doppler), (float)param_.min_sigma_doppler);
    if (std::abs(rpc1.doppler_velocity - rpc2.doppler_velocity) < doppler_th) {
      return true;
    }
  }
  return false;
}

/**
 * @fn
 * Cluster radar pointcloud with now time frame using DBSCAN algorithm
 */
void RadarObjectTracking::clusterPointcloud()
{
  /*
  for (int t = 0; t < 5; t++) {
    for (int j = 0; j < buffer_points.at(t).radar_pointclouds.size(); j++) {
      ROS_INFO(
        "t:%d x:%f, y:%f", t,
        buffer_points.at(t).radar_pointclouds.at(j).point_covariance.pose.position.x,
        buffer_points.at(t).radar_pointclouds.at(j).point_covariance.pose.position.y);
    }
  }
  */
  // init
  int num_points = buffer_points.at(0).radar_pointclouds.size();
  std::vector<bool> flag_visited(num_points, false);
  std::vector<int> stack;
  // std::vector<std::vector<int>> new_cluster(param_.num_frame);

  for (int k = 0; k < num_points; k++) {
    if (flag_visited.at(k)) {
      continue;
    }
    // init
    flag_visited.at(k) = true;
    if (!stack.empty()) {
      ROS_ERROR("stack is not empty!");
    }
    stack = {k};
    std::vector<std::vector<int>> new_cluster(param_.num_frame);
    // new_cluster.clear();
    while (!stack.empty()) {
      int id = stack.back();
      stack.pop_back();
      new_cluster.at(0).push_back(id);
      for (int j = 0; j < num_points; j++) {
        // when a point is not clustered
        if (!flag_visited.at(j)) {
          auto p_id = buffer_points.at(0).radar_pointclouds.at(id);
          auto p_j = buffer_points.at(0).radar_pointclouds.at(j);
          if (ReflectFromSameObject(p_id, p_j)) {
            stack.push_back(j);
            flag_visited.at(j) = true;
          }
        }
      }
    }
    cluster_id.push_back(new_cluster);
  }
}

/**
 * @fn
 * Judge whether two radar points with different time frame reflect from same object.
 * If two radar points reflect from same object, return true.
 * @param rpc_new: new radar point
 * @param rpc_old: old radar point
 * @param delta_t: time delay for two radar points
 */
bool RadarObjectTracking::ReflectFromSameObjectWithOldFrame(
  autoware_radar_msgs::RadarPointcloud rpc_new, autoware_radar_msgs::RadarPointcloud rpc_old,
  double delta_t)
{
  auto p1 = autoware_utils::fromMsg(rpc_new.point_covariance.pose.position).to_2d();
  auto p2 = autoware_utils::fromMsg(rpc_old.point_covariance.pose.position).to_2d();

  double doppler_th = std::max(
    std::min(rpc_new.sigma_doppler, rpc_old.sigma_doppler), (float)param_.min_sigma_doppler);
  double range_th =
    std::max(std::min(rpc_new.sigma_range, rpc_old.sigma_range), (float)param_.min_sigma_range);

  double search_th = range_th + doppler_th * delta_t;
  if (boost::geometry::distance(p1, p2) < search_th) {
    auto doppler_diff = rpc_new.doppler_velocity - rpc_old.doppler_velocity;
    if (std::abs(doppler_diff) < doppler_th + param_.max_object_acc * delta_t) {
      return true;
    }
  }
  return false;
}

/**
 * @fn
 * Cluster old radar pointcloud searching near point from latest clustered pointcloud.
 * Old radar pointcloud can be clustered to multiple cluster.
 */
void RadarObjectTracking::clusterPointcloudOldFrame()
{
  // [TODO] implement for multi frame
  int num_segment = buffer_points.at(0).radar_pointclouds.size();
  if (param_.num_frame == 1) {
    return;
  }
  for (int i = 0; i < cluster_id.size(); i++) {
    for (int t = 1; t < param_.num_frame; t++) {
      double delta_t = t / (double)param_.update_rate;
      for (int j = 0; j < cluster_id.at(i).at(0).size(); j++) {
        int now_point_index = cluster_id.at(i).at(0).at(j);
        auto p_now = buffer_points.at(0).radar_pointclouds.at(now_point_index);
        for (int k = 0; k < buffer_points.at(t).radar_pointclouds.size(); k++) {
          ROS_DEBUG("t: %d, k %d", t, k);
          auto p_old = buffer_points.at(t).radar_pointclouds.at(k);
          if (ReflectFromSameObjectWithOldFrame(p_now, p_old, delta_t)) {
            cluster_id.at(i).at(t).push_back(k);
            ROS_DEBUG("within: t: %d, k %d", t, k);
          }
        }
        ROS_DEBUG("i: %d, t: %d, num: %d", i, t, (int)cluster_id.at(i).at(0).size());
      }
    }
  }
}

/**
 * @fn
 * Remove noise pointcloud like DBSCAN algorithm
 * If cluster do no have old pointcloud, it means noise point.
 */
void RadarObjectTracking::removeNoisePoint()
{
  // [TODO] implement for multi frame
  int i = 0;
  while (i < cluster_id.size()) {
    int old_point_num = 0;
    for (int t = 1; t < cluster_id.at(i).size(); t++) {
      if (cluster_id.at(i).at(t).size() > 0) {
        old_point_num += 1;
      }
    }
    // if old points is too few, regard the core point as noise and remove it
    ROS_DEBUG("%d num: %d", i, old_point_num);
    if (old_point_num < param_.noise_thereshold_frame) {
      cluster_id.erase(cluster_id.begin() + i);
      /*
      ROS_INFO("Delete");
      std::iter_swap(cluster_id.begin() + i, cluster_id.end());
      cluster_id.pop_back();
      */
    } else {
      i++;
    }
  }
}

autoware_perception_msgs::DynamicObjectWithFeatureArray RadarObjectTracking::makeBoundingBoxes(
  std::string frame_id)
{
  autoware_perception_msgs::DynamicObjectWithFeatureArray objects;
  for (int i = 0; i < cluster_id.size(); i++) {
    // caliculate moment
    Moment m{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double z_moment = 0.0;
    autoware_radar_msgs::RadarPointcloud radar_min_range{};
    for (const auto & p_id : cluster_id.at(i).at(0)) {
      // get radar pointcloud
      auto p = buffer_points.at(0).radar_pointclouds.at(p_id).point_covariance.pose.position;
      m.m_00 += 1.0;
      m.m_10 += p.x;
      m.m_01 += p.y;
      m.m_11 += p.x * p.y;
      m.m_20 += p.x * p.x;
      m.m_02 += p.y * p.y;
      // z moment
      z_moment += p.z;
      if (radar_min_range.range < buffer_points.at(0).radar_pointclouds.at(p_id).range) {
        radar_min_range = buffer_points.at(0).radar_pointclouds.at(p_id);
      }
    }

    // caliculate centor of gravity
    if (std::abs(m.m_00) < 0.0001) {
      ROS_ERROR("0 division");
    }
    geometry_msgs::Point pc;
    pc.x = m.m_10 / m.m_00;
    pc.y = m.m_01 / m.m_00;
    pc.z = z_moment / m.m_00;

    // caliculate yaw
    double yaw;
    if (param_.num_frame == 1) {
      if (cluster_id.at(0).at(i).size() == 1) {
        yaw = 0.0;
        // A point come from rear of car, so move to centor of gravity
        pc.x = pc.x + param_.min_object_x * 0.5 - 0.5;
      } else {
        yaw = 0.5 * std::atan2(
                      2.0 * (m.m_11 / m.m_00 - pc.x * pc.y),
                      (m.m_20 / m.m_00 - pc.x * pc.x) - (m.m_02 / m.m_00 - pc.y * pc.y));
      }
    } else {
      // [TODO] implement for multi frame
      yaw = 0.5 * std::atan2(
                    2.0 * (m.m_11 / m.m_00 - pc.x * pc.y),
                    (m.m_20 / m.m_00 - pc.x * pc.x) - (m.m_02 / m.m_00 - pc.y * pc.y));
      if (cluster_id.at(i).at(0).size() == 1) {
        yaw = 0.0;
        // A point come from rear of car, so move to centor of gravity
        pc.x = pc.x + param_.min_object_x * 0.5 - 0.5;
      }
    }

    // fitting 3d bbox
    // modify centor of gravity
    auto pc_ = autoware_utils::fromMsg(pc).to_2d();
    auto pmin_ = autoware_utils::fromMsg(radar_min_range.point_covariance.pose.position)
                   .to_2d();  // minmum range radarpoint
    double ob_shape_x = std::max(param_.min_object_x * 0.5, boost::geometry::distance(pc_, pmin_));
    geometry_msgs::Point new_pc;
    new_pc.x = pc.x + (ob_shape_x - param_.min_object_x * 0.5) * std::cos(yaw);
    new_pc.y = pc.y + (ob_shape_x - param_.min_object_x * 0.5) * std::sin(yaw);
    new_pc.z = pc.z;

    // make 3dbbox
    autoware_perception_msgs::DynamicObjectWithFeature ob{};
    ob.object.state.pose_covariance.pose.position.x = new_pc.x;
    ob.object.state.pose_covariance.pose.position.y = new_pc.y;
    ob.object.state.pose_covariance.pose.position.z = new_pc.z + 0.5 * param_.min_object_z;
    ROS_DEBUG("3d bbox %f %f ", new_pc.x, new_pc.y);

    tf2::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    ob.object.state.pose_covariance.pose.orientation = tf2::toMsg(quat);

    ob.object.state.twist_covariance.twist.linear.x = radar_min_range.doppler_velocity;
    ob.object.state.twist_covariance.twist.linear.y = 0.0;
    ob.object.state.twist_covariance.twist.linear.z = 0.0;

    ob.object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
    ob.object.shape.dimensions.x = 2.0f * ob_shape_x;
    ob.object.shape.dimensions.y = param_.min_object_y;
    ob.object.shape.dimensions.z = param_.min_object_z;

    ob.object.semantic.type = autoware_perception_msgs::Semantic::CAR;
    ob.object.semantic.confidence = param_.object_confidence;
    objects.feature_objects.push_back(ob);
  }
  objects.header.frame_id = frame_id;
  objects.header.stamp = ros::Time::now();

  /* temp until making radar object merger*/
  objects.header.frame_id = "/base_link";
  // geometry_msgs::TransformStamped transformStamped;
  // transformStamped = getTransform(tf_buffer, "map", "base_link", ros::Time(0), ros::Duration(0));

  for (auto & ob : objects.feature_objects) {
    ob.object.state.pose_covariance.pose.position.x += 1.0;
    ob.object.state.pose_covariance.pose.position.z -= 1.5;
    // tf2::doTransform( ob.object.state.pose_covariance.pose, ob.object.state.pose_covariance.pose,
    // transformStamped);
  }
  return objects;
}

void RadarObjectTracking::resisterPointcloud(const RadarObjectTrackingInput & input)
{
  updateBufferPoints(input.input_radar_msg);

  compensateEgoPosition();
}

Output RadarObjectTracking::update(const RadarObjectTrackingInput & input)
{
  Output output{};

  updateBufferPoints(input.input_radar_msg);
  compensateEgoPosition();
  clusterPointcloud();
  clusterPointcloudOldFrame();
  removeNoisePoint();
  output.output_object_msg = makeBoundingBoxes(input.input_radar_msg->header.frame_id);
  return output;
}
}  // namespace radar_object_tracking

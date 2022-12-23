# radar_object_tracking

## radar_object_detection_node

This package cluster radar dynamic pointcloud and publish to 3d bounding box to detection far objects.
In detail, see [this document](/docs/algorithm.md)

- Feature lists
  - [x] Clustering radar dynamic pointcloud
  - [ ] Multi-radar input
  - [ ] Multi-frame detection
  - [ ] Multi object tracking
- Calculation cost O(t n^2)
  - n: the number of radar pointcloud
  - t: the frame number for multi-frame detection

### Input topics

| Name        | Type                        | Description                                    |
| ----------- | --------------------------- | ---------------------------------------------- |
| input/radar | sensor_msgs::msg::RadarScan | Radar dynamic pointcloud after removing noise. |

### Output topics

| Name           | Type                                          | Description     |
| -------------- | --------------------------------------------- | --------------- |
| output/objects | autoware_perception_msgs::msg::TrackedObjects | Tracked objects |

### Parameters

| Name                  | Type   | Description                                                                                                                                                                 |
| --------------------- | ------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| update_rate           | double | Update frame rate [hz]                                                                                                                                                      |
| num_frame             | int    | Number of frame                                                                                                                                                             |
| clustering_range      | double | Searching distance for DBSCAN, set to what want to detect (default set the size of car) [m]                                                                                 |
| min_sigma_doppler     | double | Minimum doppler velocity variance to use clustering. If (difference of doppler velocity between near pointclouds) > 3 \* standard deviation, then not clustering. [(m/s)^2] |
| min_sigma_range       | double | Minimum range variance for old frame clustering [m^2]                                                                                                                       |
| max_object_acc        | double | Maximum object acceleration for old frame clustering [m/s^2]                                                                                                                |
| min_object_x          | double | Minimum x size of object (default set the size of car) [m]                                                                                                                  |
| min_object_y          | double | Minimum y size of object (default set the size of car) [m]                                                                                                                  |
| min_object_z          | double | Minimum z size of object (default set the size of car) [m]                                                                                                                  |
| object_confidence     | double | Output object's confidence                                                                                                                                                  |
| noise_threshold_frame | int    | Sum of old frames which have clustering pointcloud < "noise_threshold_frame", the id is noise point.                                                                        |

### How to launch

```sh
ros2 launch radar_object_detection radar_object_detection.launch.xml
```

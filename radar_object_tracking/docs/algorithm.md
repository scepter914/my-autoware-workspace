
### 1. Ego position modification

- [Under construction]
- Update position of old radar pointclouds by using ego vehicle twist

### 2. Clustering pointcloud

- Clustering for radar pointcloud using something like DBSCAN algorithm

![clustering](radar_object_detection_1.drawio.svg)

- cluster table

```
std::vector<std::vector<std::vector<int>>> cluster_id;
//! cluster_id[cluster_index][frame_index][clustered_point_index] = point_index
```

| id \ frame | 0 (now)    | 1 (1 old frame) | ..  | t (t old frame) |
| ---------- | ---------- | --------------- | --- | --------------- |
| 0          | 0, 1, 4, k |                 |     |                 |
| 1          | 2          |                 |     |                 |
| ..         |            |                 |     |                 |
| i          | 3          |                 |     |                 |
| ..         |            |                 |     |                 |
| n          |            |                 |     |                 |

- Pointclouds array

```
std::vector<autoware_radar_msgs::RadarPointcloudArray> buffer_points;
//! Point: buffer_points[frame_index].radar_pointclouds[point_index]
```

| frame | 0 (now)                                         | ..  | t (t old frame)                                 |
| ----- | ----------------------------------------------- | --- | ----------------------------------------------- |
|       | buffer_points[0].radar_pointclouds[point_index] |     | buffer_points[t].radar_pointclouds[point_index] |

### 3. Clustering oldframe pointcloud

- Clustring for old frame pointcloud using something like DBSCAN algorithm
- For each clustered pointcloud, searching near old pointcloud like "2. clustering pointcloud" using doppler velocity
- When clustering old frame pointcloud, pointcloud can belong to multiple id
    - ex. In below table, point "2" in 1 old frame belong to id 0 and 1

| id \ frame | 0 (now)    | 1 (1 old frame) | ..  | t (t old frame) |
| ---------- | ---------- | --------------- | --- | --------------- |
| 0          | 0, 1, 4, k | 0,2             |     |                 |
| 1          | 2          | 2,3             |     |                 |
| ..         |            |                 |     |                 |
| i          | 3          | 3,5             |     |                 |
| ..         |            |                 |     |                 |
| n          |            |                 |     |                 |

### 4. Remove noise pointcloud

- For each id, counting old frames which have clustering pointcloud.
- If sum of the frames < "noise_thereshold_frame", the id is noise point.

### 5. Make 3d bounding box

1. Yaw estimation
    - simgle frame mode
        - Calculate yaw angle using moment
        - If clustering id has only one point, yaw = 0.
    - Multi-frame mode
        - [Under developmet]
2. Fitting 3d bounding box
    - Move to object's center point to where minimum range point come to close side of the object

![fitting](radar_object_detection_2.drawio.svg)

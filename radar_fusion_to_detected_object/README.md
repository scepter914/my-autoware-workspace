# radar_fusion_to_detected_object

This package is radar-based sensor fusion module to 3d detected object.
Main feature of this package are as following.

- Attach velocity to lidar detection result from radar data to improve for tracking result and planning like adaptive cruise.
- Improve detection result with radar sensor information. If both lidar 3d detected objects with low score and high confidence of radar pointcloud / objects, then improve score of objects.

The document of core algorithm is [here](docs/algorithm.md)

## radar_object_fusion_to_detected_object

Sensor fusion with radar objects and a detected object.

- Calculation cost is O(nm).
  - n: the number of radar objects.
  - m: the number of objects from 3d detection.

### How to launch

```sh
roslaunch radar_fusion_to_detected_object radar_object_to_detected_object.launch
```

### Input / Output

- Input
    - `~/input/objects` (autoware_auto_perception_msgs/msg/DetectedObject.msg): 3d detected object.
    - `~/input/radar_objects` (autoware_auto_perception_msgs/msg/TrackedObjects.msg)
- Output
    - `~/output/objects` (autoware_auto_perception_msgs/msg/DetectedObjects.msg): 3d detected object with twist.

### Parameters

## radar_scan_fusion_to_detected_object (TBD)

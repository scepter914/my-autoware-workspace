# radar_crossing_objects_noise_filter

## Input

| Name              | Type                                                  | Description          |
| ----------------- | ----------------------------------------------------- | -------------------- |
| `~/input/objects` | autoware_auto_perception_msgs/msg/DetectedObjects.msg | 3D detected objects. |

## Output

| Name                        | Type                                                  | Description      |
| --------------------------- | ----------------------------------------------------- | ---------------- |
| `~/output/noise_objects`    | autoware_auto_perception_msgs/msg/DetectedObjects.msg | Noise Objects    |
| `~/output/filtered_objects` | autoware_auto_perception_msgs/msg/DetectedObjects.msg | Filtered Objects |

## Parameters

| Name                 | Type   | Description                        | Default value |
| :------------------- | :----- | :--------------------------------- | :------------ |
| `angle_threshold`    | double | Angle parameter to filter [rad]    | 1.0472        |
| `velocity_threshold` | double | Velocity parameter to filter [m/s] | 2.0           |

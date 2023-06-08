# radar_tracks_crossing_noise_filter

This package contains a radar object filter module for [autoware_auto_perception_msgs/msg/RadarTrack](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/RadarTrack.idl).
This package can split RadarTracks into two messages by object's speed.

## Input

| Name              | Type                                             | Description          |
| ----------------- | ------------------------------------------------ | -------------------- |
| `~/input/objects` | autoware_auto_perception_msgs/msg/RadarTrack.msg | 3D detected objects. |

## Output

| Name                          | Type                                              | Description             |
| ----------------------------- | ------------------------------------------------- | ----------------------- |
| `~/output/low_speed_objects`  | autoware_auto_perception_msgs/msg/RadarTracks.msg | Objects with low speed  |
| `~/output/high_speed_objects` | autoware_auto_perception_msgs/msg/RadarTracks.msg | Objects with high speed |

## Parameters

| Name                 | Type   | Description                               | Default value |
| :------------------- | :----- | :---------------------------------------- | :------------ |
| `velocity_threshold` | double | Velocity parameter to split objects [m/s] | 3.0           |

# radar_tracks_noise_filter

This package contains a radar object filter module for [autoware_auto_perception_msgs/msg/RadarTrack](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/RadarTrack.idl).
This package can split RadarTracks into two messages by object's speed.

## Input

| Name             | Type                                             | Description         |
| ---------------- | ------------------------------------------------ | ------------------- |
| `~/input/tracks` | autoware_auto_perception_msgs/msg/RadarTrack.msg | 3D detected tracks. |

## Output

| Name                         | Type                                              | Description            |
| ---------------------------- | ------------------------------------------------- | ---------------------- |
| `~/output/low_speed_tracks`  | autoware_auto_perception_msgs/msg/RadarTracks.msg | tracks with low speed  |
| `~/output/high_speed_tracks` | autoware_auto_perception_msgs/msg/RadarTracks.msg | tracks with high speed |

## Parameters

| Name                 | Type   | Description                              | Default value |
| :------------------- | :----- | :--------------------------------------- | :------------ |
| `velocity_threshold` | double | Velocity parameter to split tracks [m/s] | 3.0           |

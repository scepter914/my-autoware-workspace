# radar_tracks_noise_filter

This package contains a radar object filter module for `radar_msgs/msg/RadarTrack`.
This package can filter noise objects in RadarTracks.

## Input

| Name             | Type                           | Description         |
| ---------------- | ------------------------------ | ------------------- |
| `~/input/tracks` | radar_msgs/msg/RadarTracks.msg | 3D detected tracks. |

## Output

| Name                       | Type                           | Description           |
| -------------------------- | ------------------------------ | --------------------- |
| `~/output/noise_tracks`    | radar_msgs/msg/RadarTracks.msg | Noise objects         |
| `~/output/filtered_tracks` | radar_msgs/msg/RadarTracks.msg | Objects without noise |

## Parameters

| Name                   | Type   | Description                     | Default value |
| :--------------------- | :----- | :------------------------------ | :------------ |
| `velocity_y_threshold` | double | Y-axis velocity threshold [m/s] | 2.0           |

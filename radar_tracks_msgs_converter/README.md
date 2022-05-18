# radar_tracks_msgs_converter

This package convert from [radar_msgs/msg/RadarTracks](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) into [autoware_auto_perception_msgs/msg/TrackedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/TrackedObject.idl).

## Design
### Input / Output

- Input
  - `~/input/radar_objects` (radar_msgs/msg/RadarTracks.msg): Converted topic
  - `~/input/twist`
- Output
  - `~/output/radar_objects` (autoware_auto_perception_msgs/msg/TrackedObject.msg): Converted topic

### Parameters

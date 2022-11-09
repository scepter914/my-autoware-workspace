# radar_threshold_filter

## radar_threshold_filter_node

- Remove noise from radar pointcloud
    - RCS filter
        - Low RCS consider noise
    - FOV filter
        - Pointcloud from radar's FOV edge occur perturbation
    - Range filter
        - Too near pointcloud often occur noise
- Calculation cost O(n)
    - n: the number of radar pointcloud

### Input topics

| Name        | Type                  | Description |
| ----------- | --------------------- | ----------- |
| input/radar | autoware\_radar\_msgs | Radar data  |


### Output topics

| Name                  | Type                  | Description                                 |
| --------------------- | --------------------- | ------------------------------------------- |
| output/radar          | autoware\_radar\_msgs | filtered radar pointcloud                   |
| debug/pointcloud      | pointcloud2           | Visualize RCS for output/radar              |
| debug/pointcloud\_vel | pointcloud2           | Visualize doppler velocity for output/radar |
| debug/marker          | marker\_array         | Visualize doppler velocity for output/radar |

### Parameters

- For node parameter

| Name                       | Type   | Description                                                                             |
| -------------------------- | ------ | --------------------------------------------------------------------------------------- |
| update\_rate               | double | node Hz                                                                                 |
| is\_rcs\_filter            | bool   | if true, apply RCS filter (publish rcs\_min < rcs < rcs\_max)                           |
| rcs\_min                   | double | [dBm^2]                                                                                 |
| rcs\_max                   | double | [dBm^2]                                                                                 |
| is\_range\_filter          | bool   | if true, apply range filter (publish range\_min < range < range\_max)                   |
| range\_min                 | double | [m]                                                                                     |
| range\_max                 | double | [m]                                                                                     |
| is\_angle\_azimuth\_filter | bool   | if true, apply angle filter (publish angle\_azimuth\_min < range < angle\_azimuth\_max) |
| angle\_azimuth\_min        | double | [rad]                                                                                   |
| angle\_azimuth\_max        | double | [rad]                                                                                   |
| is\_z\_filter              | bool   | if true, apply z position filter (publish z\_min < z < z\_max)                          |
| z\_min                     | double | [m]                                                                                     |
| z\_max                     | double | [m]                                                                                     |

- For debug parameter

| Name            | Type   | Description                          |
| --------------- | ------ | ------------------------------------ |
| is\_rviz\_debug | bool   | publish flag for debug visualization |
| arrow\_length   | double | arrow length for marker\_array [m]   |
| max\_color\_r   | int    | arrow color for marker\_array        |
| max\_color\_g   | int    | arrow color for marker\_array        |
| max\_color\_b   | int    | arrow color for marker\_array        |

### How to launch

```sh
roslaunch radar_threshold_filter radar_threshold_filter.launch
```

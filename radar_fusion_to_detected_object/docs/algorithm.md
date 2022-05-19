
## Common Algorithm
### 1. Link between 3d bounding box and radar data

Choose radar pointcloud/object within 3d bounding box from lidar-base detection with margin space from bird eye view.

![choose_radar](docs/radar_fusion_to_3dbbox_1.drawio.svg)

### 2. Estimate velocity of object.

![estimate_doppler_velocity](docs/radar_fusion_to_3dbbox_2.drawio.svg)

3. Attach object to twist information.

![process_high_confidence](docs/radar_fusion_to_3dbbox_3.drawio.svg)

4. If velocity information has doppler velocity, convert from doppler velocity to twist using yaw angle of 3d detected object.

![process_low_confidence](docs/radar_fusion_to_3dbbox_4.drawio.svg)

5. If object confidence of lidar-based detection is low,
    5.1. and if an object estimate to derive two object, split two object
    5.2. and if there are some radar point/object within 3d bounding box from lidar-base detection, attach object to twist information.
    5.3. and if there are not any radar point/object within 3d bounding box from lidar-base detection, delete the object.


![process_low_confidence](docs/radar_fusion_to_3dbbox_5.drawio.svg)

![process_low_confidence](docs/radar_fusion_to_3dbbox_6.drawio.svg)

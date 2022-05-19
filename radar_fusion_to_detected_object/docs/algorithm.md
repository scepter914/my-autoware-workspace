
## Common Algorithm
### 1. Link between 3d bounding box and radar data

Choose radar pointcloud/objects within 3d bounding box from lidar-base detection with margin space from bird eye view.

![choose_radar](radar_fusion_to_detected_object_1.drawio.svg)

### 2. Estimate twist of object

Estimate twist from choosed radar pointcloud/objects.
Attach object to twist information of estimated twist.

![estimate_doppler_velocity](radar_fusion_to_detected_object_2.drawio.svg)

### 3. [Option] Convert doppler velocity to twist

If the twist information of radars is doppler velocity, convert from doppler velocity to twist using yaw angle of DetectedObject.

![process_high_confidence](radar_fusion_to_detected_object_3.drawio.svg)

### 4. If an object confidence of lidar-based detection is low, fix the object.

- Split two object for the low confidence object that can be estimated to derive two object.

![process_low_confidence](radar_fusion_to_detected_object_4.drawio.svg)

- Delete low confidence objects that do not have some radar points/objects.

![process_low_confidence](radar_fusion_to_detected_object_5.drawio.svg)

![process_low_confidence](radar_fusion_to_detected_object_6.drawio.svg)

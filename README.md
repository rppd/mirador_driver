# Mirador Driver
Mirader ROS driver for bridging ROS robot metadata with Mirador HMI

## mirador_driver

### Parameters

| Name | Default value | Description |
| --- | --- | --- |
| is_orientation_ned | false | Compass convention (NED / ENU): 0 for North with NED and 0 for East with ENU. Basically ROS compass value is ENU. |
| is_zero_altitude | false | Use for ground vehicle. |
| utm_zone | "31n" | Specify UTM zone of robot usage. 31n for Paris per example. |
| utm_frame_id | "utm" |  |
| odom_frame_id | "odom" |  |
| base_link_frame_id | "base_link" |  |
| navsatfix_topic | "/fix" |  |
| use_odometry | false | Specify if you prefer imu or odometry compass for yaw information. |
| imu_topic | "/imu" |  |
| odometry_topic | "/odometry" |  |
| flight_status_topic | "/flight_status" |  |
| camera_elevation_topic | "/camera_elevation" |  |
| camera_zoom_topic:| "/camera_zoom" | 
| e_stop_topic | "/e_stop" |  |
| stream_method | 0 | 0 for URL video stream method, 1 for WebRTC method |
| stream_address | [] | Specify stream URL address in case of stream method 0. Parameter stream_method must be 0 |

### Subscribers

#### Mirador HMI side (Mirador standardized, then computed with JavaScript with roslibjs)

| Name | Message type | Desciption |
| --- | --- | --- |
| /mirador/mission | mirador_driver/Mission |   |
| /mirador/launch | mirador_driver/Launch |   |
| /mirador/abort | mirador_driver/Abort |   |
| /mirador/report | mirador_driver/Report |   |

#### mirador_driver node side

| Default value | Message type | Desciption |
| --- | --- | --- |
| /fix | sensor_msgs/NavSatFix |  |
| /imu | sensor_msgs/Imu |  |
| /odometry | nav_msgs/Odometry|  |
| /flight_status | std_msgs/Float32 |  |
| /camera_elevation | std_msgs/Float32 |  |
| /camera_zoom | std_msgs/Int8 |  |
| /e_stop | std_msgs/Bool | Emergency stop indicator |

### Publishers

#### Mirador HMI side (Mirador standardized, then computed with JavaScript with roslibjs)

| Default value | Message type | Desciption |
| --- | --- | --- |
| /mirador/status | mirador_msgs/Status |  |
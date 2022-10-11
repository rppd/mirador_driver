# Mirador Driver
Mirader ROS driver for bridging ROS robot metadata with **Mirador HMI**

## mirador_driver

This package is a driver that aim to link [**Mirador**](https://github.com/julesberhault/mirador), (Mirador Human-Machine Interface) with ROS. It makes a bridge between ROS topics and JavaScript executing on the HMI web page. The data collected are then displayed on the interface and users can communicate with their robot simply and remotely through a web navigator. Web server is a nodejs server, it rely on socket.io communications to link web pages and share data synchronously.

### Install

Clone this repository:
```bash
clone https://github.com/julesberhault/mirador_driver.git
```

### Compiling

First, you need [**ROS environment**](http://wiki.ros.org/fr/ROS/Installation) distro if not already installed.

Compile package:

```bash
catkin_make mirador_driver
```

### Running

Run node with rosrun:

```bash
rosrun mirador_driver mirador_driver
```

### Parameters

| Name | Default value | Description |
| --- | --- | --- |
| is_orientation_ned | false | Compass convention (NED / ENU): 0 for North with NED and 0 for East with ENU. Basically ROS compass value is ENU. |
| is_zero_altitude | false | Use for ground vehicle. |
| utm_zone | "31n" | Specify UTM zone of robot usage. 31n for Paris per example. |
| utm_frame_id | "utm" |  |
| odom_frame_id | "odom" |  |
| base_link_frame_id | "base_link" |  |
| ping_topic | "/ping" | Ping or delay topic name used to estimate quality of signal |
| state_of_charge_topic | "/state_of_charge" | State of battery charge topic name |
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
| /ping | std_msgs/Float64 | Ping or delay time in ms used to estimate quality of the signal |
| /state_of_charge | std_msgs/Float32 | State of the battery charge of the robot, value must be between 0.0 and 100.0 |
| /fix | sensor_msgs/NavSatFix | GPS or GNSS data, latitude, longitude. Use parameter ***zero_altitude*** setted to **false** to keep altitude value |
| /imu | sensor_msgs/Imu | IMU data used to get orientation. ENU cnvention is used by default. Use parameter ***is_orientation_ned*** to use NED convention |
| /odometry | nav_msgs/Odometry| Odometry data used to get orientation if parameter ***use_odometry*** is setted to **true** |
| /flight_status | std_msgs/Float32 | **LANDED** = 0, **TAKING_OFF** = 1, **FLYING** = 2, **LANDING** = 3 |
| /camera_elevation | std_msgs/Float32 | Value of camera orientation applied, horizontal direction is 0, vertical down direction is -90° |
| /camera_zoom | std_msgs/Int8 | Value of camera zoom applied, default value is 1 |
| /e_stop | std_msgs/Bool | Emergency stop indicator. Setted to **true** mean that emergency stop is currently applied |

### Publishers

#### Mirador HMI side (Mirador standardized, then computed with JavaScript with [**roslibjs**](https://github.com/RobotWebTools/roslibjs))

| Default value | Message type | Desciption |
| --- | --- | --- |
| /mirador/status | mirador_msgs/Status |  |

## rosping

This package is used to get ping (delay in ms) with the robot

You can easily check the package from this [link](https://github.com/julesberhault/rosping).

### Installation

Clone the repository:
```bash
clone https://github.com/julesberhault/rosping.git
```

### Compiling

Compile this package:
```bash
rosping rosping
```

### Running

Run node with rosrun:
```bash
rosrun rosping rosping HOSTNAME
```
with **HOSTNAME** the IP address or hostname you wish to ping

### Parameters

| Name | Default value | Description |
| --- | --- | --- |
| rate | 10 | Frequency for publishing message. 10 is for pûblishing every 10s (0.1 Hz) |

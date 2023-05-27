#pragma once

#include <ros/ros.h>

// Standard
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

// Custom ROS messages
#include "mirador_driver/GeoPose.h"
#include "mirador_driver/Mission.h"
#include "mirador_driver/Report.h"
#include "mirador_driver/Status.h"
#include "mirador_driver/MissionContext.h"
#include "mirador_driver/StrategicPoint.h"

// ROS
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Geographic Lib
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "config.h"
#include "robot.h"
#include "controller.h"


class MiradorDriver {

public:
    Config config;
    Robot robot;
    Controller controller;

    static int sequence;

    MiradorDriver(ros::NodeHandle& n);

    // -------------------- Functions --------------------
    // Reset mission flags to default
    void resetMission();
    // Perform guide mode: Go strait to the point, stop when range is acceptable
    bool setGuide();
    // Build next goal on the list and set it, "first" flag specify the goal setted is the first in the queue
    bool setNextGoal(const bool& _first);
    // Start action move base goal with the specified target pose
    bool startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose);
    // Simply publish Mirador status message
    void publishStatus();

    void publishCmdVel();

    void processMoveBaseGoal();

    bool getTargetPose(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PoseStamped& target_pose);

    void latLongToUtm(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PointStamped& utm_point);

    void utmToLatLong(const geometry_msgs::PointStamped& _utm_point, geographic_msgs::GeoPoint& geo_point);

    bool utmToOdom(const geometry_msgs::PointStamped& _utm_point, geometry_msgs::PointStamped& odom_point);

    bool odomToUtm(const geometry_msgs::PointStamped& _odom_point, geometry_msgs::PointStamped& utm_point);

};
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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MiradorDriver {

    ros::Subscriber missionSubscriber;
    ros::Subscriber reportSubscriber;
    ros::Subscriber abortMissionSubscriber;
    ros::Subscriber launchMissionSubscriber;
    ros::Subscriber pingSubscriber;
    ros::Subscriber stateOfChargeSubscriber;
    ros::Subscriber navsatfixSubscriber;
    ros::Subscriber imuSubscriber;
    ros::Subscriber odometrySubscriber;
    ros::Subscriber flightStatusSubscriber;
    ros::Subscriber cameraElevationSubscriber;
    ros::Subscriber cameraZoomSubscriber;
    ros::Subscriber eStopSubscriber;
    ros::Subscriber mission_contextSubscriber;

    ros::Publisher statusPublisher;
    ros::Publisher cmdVelPublisher;

    MoveBaseClient moveBaseClient;
    Robot robot;
    Config config;

    int sequence;

    void missionCallback(const mirador_driver::Mission& _mission);
    void reportCallback(const mirador_driver::Report& _report);
    void launchMissionCallback(const std_msgs::Empty& _empty);
    void abortMissionCallback(const std_msgs::Empty& _empty);
    void pingCallback(const std_msgs::Float64& _delay);
    void stateOfChargeCallback(const std_msgs::Float32& _soc);
    void navSatFixCallback(const sensor_msgs::NavSatFix& _navsatfix);
    void imuCallback(const sensor_msgs::Imu& _imu);
    void odometryCallback(const nav_msgs::Odometry& _odometry);
    void flightStatusCallback(const std_msgs::Int8& _flight_status);
    void cameraElevationCallback(const std_msgs::Float32& _camera_elevation);
    void cameraZoomCallback(const std_msgs::Int8&  _camera_zoom);
    void eStopCallback(const std_msgs::Bool& _e_stop);
    void missionContextCallback(const mirador_driver::MissionContext& _mission_context);

    void publishCmdVel();
    void publishStatus();

    void processMoveBaseGoal();
    bool startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose);
    bool setNextGoal(bool _first);
    bool setGuide();
    bool getTargetPose(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PoseStamped& target_pose);

public:
    Config config;
    Robot robot;

    MiradorDriver(ros::NodeHandle& n);
};
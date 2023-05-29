#ifndef ROBOT_H
#define ROBOT_H

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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Mission {
    std::vector<geographic_msgs::GeoPoint> points;
    std::string id;
};

class Robot {

public:
    int utzone;
    bool is_north_hemisphere;
    bool is_running = false;
    mirador_driver::Report report;

    geographic_msgs::GeoPoint position;

    ros::Time last_time;
    int signal_quality = 0;
    double heading = .0;
    double yaw = .0;
    bool publish_cmd_vel = false;
    int mode = 0; //0 : no mission, 1 : guide, 2 : route, 3: exploration
    int state_of_charge = 0;
    int flight_status = 0;
    float camera_elevation = .0;
    int camera_zoom = 0;
    bool e_stop = false;
    mirador_driver::MissionContext mission_context;
    int sequence;

    Mission mission;
    Config& config;
    
    Robot(Config& _config): moveBaseClient("move_base", true), config(_config), sequence(0) {};
    //Robot() {};

    MoveBaseClient moveBaseClient;
    
    void launchMission();
    bool setNextGoal(bool _first);
    bool startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose);
    bool makeCmdVel(geometry_msgs::Twist& cmd_vel_twist);
    void resetMission();
    bool setGuide();
    void updateUtmTransform();

    void latLongToUtm(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PointStamped& utm_point);
    void utmToLatLong(const geometry_msgs::PointStamped& _utm_point, geographic_msgs::GeoPoint& geo_point);
    bool utmToOdom(const geometry_msgs::PointStamped& _utm_point, geometry_msgs::PointStamped& odom_point);
    bool odomToUtm(const geometry_msgs::PointStamped& _odom_point, geometry_msgs::PointStamped& utm_point);
    bool getTargetPose(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PoseStamped& target_pose);

};

#endif
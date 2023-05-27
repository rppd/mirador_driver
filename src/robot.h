#ifndef ROBOT_H
#define ROBOT_H

#include "mirador_driver.h"
#include "config.h"

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

    Mission mission;
    Config& config;
    
    Robot(Config& _config): config(_config) {};
    //Robot() {};

    void launchMission() {
        if (mode == 0) ROS_INFO("No route mission to launch");
        if (mode == 2) {
            ROS_INFO("Route mission launched");
            setNextGoal(true);
        }      
    }

    bool setNextGoal(bool _first);
    bool makeCmdVel(geometry_msgs::Twist& cmd_vel_twist);
    void resetMission();

    void latLongToUtm(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PointStamped& utm_point);
    void utmToLatLong(const geometry_msgs::PointStamped& _utm_point, geographic_msgs::GeoPoint& geo_point);
    bool utmToOdom(const geometry_msgs::PointStamped& _utm_point, geometry_msgs::PointStamped& odom_point);

};

#endif
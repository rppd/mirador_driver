#pragma once

#include <ros/ros.h>

// Standard
#include <string>
#include <cmath>
#include <math.h>
#include <eigen3/Eigen/Dense>

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
#include <geographic_msgs/GeoPoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Custom ROS messages
#include "mirador_driver/GeoPose.h"
#include "mirador_driver/Mission.h"
#include "mirador_driver/Report.h"
#include "mirador_driver/Status.h"
#include "mirador_driver/MissionContext.h"
#include "mirador_driver/StrategicPoint.h"
#include "mirador_driver/ConvertGPSToPath.h"
#include "mirador_driver/GoGeoPoseAction.h"



// Geographic Lib
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

class MiradorDriver
{
    public:
        MiradorDriver(ros::NodeHandle& n);

        // -------------------- Callbacks --------------------

        void pingCallback(const std_msgs::Float64& _delay);

        void navSatFixCallback(const sensor_msgs::NavSatFix& _navsatfix);

        void cmdVelCallback(const geometry_msgs::Twist& _cmd_vel);

        // -------------------- Functions --------------------

        // Simply publish Mirador status message
        void publishStatus();
        

        void latLongToUtm(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PointStamped& utm_point);
        
        void utmToLatLong(const geometry_msgs::PointStamped& _utm_point, geographic_msgs::GeoPoint& geo_point);

        void updateYaw(geometry_msgs::PointStamped& m_point_new);

    private:
        // Subscribers
        ros::Subscriber m_pingSubscriber;
        ros::Subscriber m_navsatfixSubscriber;
        ros::Subscriber m_relaltSubscriber;
        ros::Subscriber m_imuSubscriber;
        ros::Subscriber m_odometrySubscriber;
        ros::Subscriber m_cmdVelSubscriber;
        
        // Publishers
        ros::Publisher m_statusPublisher;
        ros::Publisher m_abortPublisher;
        ros::Publisher m_cmdVelPublisher;
        ros::Publisher m_takeOffLandPublisher;

        //Services
        ros::ServiceClient m_convertGPSToPathClient;
        ros::ServiceClient m_wpClearService;
        ros::ServiceClient m_wpPushService;
        ros::ServiceClient m_takeOffLandService;
        ros::ServiceClient m_flightModeService;
        ros::ServiceClient m_armService;
        ros::ServiceClient m_streamRateService;
        ros::ServiceClient m_setRTHService;

        
        // tf Listener
        tf2_ros::Buffer m_tf2_buffer;

        // Parameters
        bool m_is_orientation_ned;
        bool m_is_zero_altitude;
        std::string m_utm_frame_id;
        std::string m_odom_frame_id;
        std::string m_base_link_frame_id;
        std::string m_ping_topic;
        std::string m_state_of_charge_topic;
        std::string m_navsatfix_topic;
        std::string m_relalt_topic;
        bool m_use_odometry;
        std::string m_imu_topic;
        std::string m_odometry_topic;
        std::string m_flight_status_topic;
        std::string m_camera_elevation_topic;
        std::string m_camera_zoom_topic;
        std::string m_e_stop_topic;
        int m_stream_method;
        std::vector<std::string> m_stream_address;
        std::string m_stream_topic;
        std_msgs::Bool m_tol;
        std::string m_mission_context_topic;
        int status_tab [5];

        // Mirador driver data
        int m_utm_zone;
        bool m_is_north_hemisphere;
        bool m_is_running;
        std::vector<geographic_msgs::GeoPoint> m_mission_points;
        geometry_msgs::PointStamped m_gps_points[5];
        std::string m_mission_id;
        std::string m_state_mode;
        mirador_driver::Report m_report;
        int m_sequence;
        int m_signal_quality;
        int m_state_of_charge;
        ros::Time m_last_time;
        geographic_msgs::GeoPoint m_geopoint;
        geographic_msgs::GeoPoint m_geopoint_new;
        geometry_msgs::PointStamped m_point;
        geometry_msgs::PointStamped m_point_new;
        double m_heading;
        double m_yaw;
        bool m_publish_cmd_vel;
        geometry_msgs::Twist m_cmd_vel;
        int m_mode;
        int m_flight_status;
        float m_camera_elevation;
        int m_camera_zoom;
        bool m_e_stop;
        mirador_driver::MissionContext m_mission_context;
        int m_gps_count;
        float m_gps_dist_thresh;
};
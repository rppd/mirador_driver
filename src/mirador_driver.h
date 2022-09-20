#pragma once

#include <ros/ros.h>

// Standard
#include <string>

// ROS
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mirador_driver/GeoPose.h>
#include <mirador_driver/Mission.h>
#include <mirador_driver/Report.h>
#include <mirador_driver/Status.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Geographic Lib
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MiradorDriver
{
    public:
        MiradorDriver(ros::NodeHandle& n);

        // -------------------- Callbacks --------------------

        // Message containing mission sent from Mirador Terminal
        void missionCallback(const mirador_driver::Mission& _mission);
        // Message containing mission report sent from anyone
        void reportCallback(const mirador_driver::Report& _report);
        // Build next goal and init waypoint following
        void launchMissionCallback(const std_msgs::Empty& _empty);
        // Stop mission and reset parameters
        void abortMissionCallback(const std_msgs::Empty& _empty);

        void navSatFixCallback(const sensor_msgs::NavSatFix& _navsatfix);

        void imuCallback(const sensor_msgs::Imu& _imu);

        void odometryCallback(const nav_msgs::Odometry& _odometry);

        void flightStatusCallback(const std_msgs::Int8& _flight_status);

        void cameraElevationCallback(const std_msgs::Float32& _camera_elevation);

        void cameraZoomCallback(const std_msgs::Int8&  _camera_zoom);

        void eStopCallback(const std_msgs::Bool& _e_stop);

        // -------------------- Functions --------------------

        // Build next goal on the list and set it, "first" is a flag to specify the goal setted is the first of the ro.
        bool setNextGoal(const bool& _first);
        // Build next goal
        void buildNextGoal();
        // Publish next goal
        void startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose);

        void publishStatus(const float _delay);
        
        geometry_msgs::PoseStamped getTargetPose(const geographic_msgs::GeoPoint& _geo_point);

        geometry_msgs::PointStamped latLongToUtm(const geographic_msgs::GeoPoint& _geo_point);
        
        geographic_msgs::GeoPoint utmToLatLong(const geometry_msgs::PointStamped& _utm_point);

        geometry_msgs::PointStamped utmToOdom(const geometry_msgs::PointStamped& _utm_point);

        geometry_msgs::PointStamped odomToUtm(const geometry_msgs::PointStamped& _odom_point);

    private:
        // Subscribers
        ros::Subscriber m_missionSubscriber;
        ros::Subscriber m_reportSubscriber;
        ros::Subscriber m_abortMissionSubscriber;
        ros::Subscriber m_launchMissionSubscriber;
        ros::Subscriber m_navsatfixSubscriber;
        ros::Subscriber m_imuSubscriber;
        ros::Subscriber m_odometrySubscriber;
        ros::Subscriber m_flightStatusSubscriber;
        ros::Subscriber m_cameraElevationSubscriber;
        ros::Subscriber m_cameraZoomSubscriber;
        ros::Subscriber m_eStopSubscriber;

        // Action Client
        MoveBaseClient m_moveBaseClient;
        
        // Publishers
        ros::Publisher m_statusPublisher;

        // tf Listener
        tf2_ros::Buffer m_tf2_buffer;

        // Parameters
        bool m_is_orientation_ned;
        bool m_is_zero_altitude;
        std::string m_utm_frame_id;
        std::string m_odom_frame_id;
        std::string m_base_link_frame_id;
        std::string m_navsatfix_topic;
        bool m_use_odometry;
        std::string m_imu_topic;
        std::string m_odometry_topic;
        std::string m_flight_status_topic;
        std::string m_camera_elevation_topic;
        std::string m_camera_zoom_topic;
        std::string m_e_stop_topic;
        int m_stream_method;
        std::vector<std::string> m_stream_address;

        // Mirador driver data
        int m_utm_zone;
        bool m_is_north_hemisphere;
        bool m_is_running;
        std::vector<geographic_msgs::GeoPoint> m_mission_points;
        std::string m_mission_id;
        mirador_driver::Report m_report;
        int m_sequence;
        int m_signal_quality;
        geographic_msgs::GeoPoint m_position;
        double m_orientation;
        int m_mode;
        int m_battery_charge;
        int m_flight_status;
        float m_camera_elevation;
        int m_camera_zoom;
        bool m_e_stop;
};
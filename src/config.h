#ifndef CONFIG_H
#define CONFIG_H

class Config {

public:
    std::string utm_frame_id;
    std::string odom_frame_id;
    std::string base_link_frame_id;
    std::string ping_topic;
    std::string state_of_charge_topic;
    std::string navsatfix_topic;
    std::string imu_topic;
    std::string odometry_topic;
    std::string flight_status_topic;
    std::string camera_elevation_topic;
    std::string camera_zoom_topic;
    std::string e_stop_topic;
    std::string stream_topic;
    std::string mission_context_topic;
    std::string warning_topic;

    int stream_method;
    std::vector<std::string> stream_address;

    int utm_zone;
    bool is_orientation_ned;
    bool is_zero_altitude;
    bool is_north_hemisphere;

    bool use_odometry;

    Config() {
        ros::NodeHandle handle("~");

        //frames
        handle.param<std::string>("utm_frame_id", utm_frame_id, "utm");
        handle.param<std::string>("odom_frame_id", odom_frame_id, "odom");
        handle.param<std::string>("base_link_frame_id", base_link_frame_id, "base_link");

        //topics
        handle.param<std::string>("ping_topic", ping_topic, "/ping");
        handle.param<std::string>("state_of_charge_topic", state_of_charge_topic, "/state_of_charge");
        handle.param<std::string>("navsatfix_topic", navsatfix_topic, "/fix");
        handle.param<bool>("use_odometry", use_odometry, false);
        handle.param<std::string>("flight_status_topic", flight_status_topic, "/flight_status");
        handle.param<std::string>("camera_elevation_topic", camera_elevation_topic, "/camera_elevation");
        handle.param<std::string>("camera_zoom_topic", camera_zoom_topic, "/camera_zoom");
        handle.param<std::string>("e_stop_topic", e_stop_topic, "/e_stop");
        handle.param<std::string>("stream_topic", stream_topic, "/image/compressed");
        handle.param<std::string>("mission_context_topic", mission_context_topic, "mission/mission_context");
        handle.param<std::string>("warning_topic", warning_topic, "/mirador/warning");

        if (use_odometry)
            handle.param<std::string>("odometry_topic", odometry_topic, "/odometry");
        else
            handle.param<std::string>("imu_topic", imu_topic, "/imu");

        //video
        handle.param<std::vector<std::string>>("stream_address", stream_address, std::vector<std::string>());
        handle.param<int>("stream_method", stream_method, int(0));

        //geographic
        handle.param<bool>("is_orientation_ned", is_orientation_ned, false);
        handle.param<bool>("is_zero_altitude", is_zero_altitude, false);
        std::string utgrid_zone;
        handle.param<std::string>("utzone", utgrid_zone, "31n");
        GeographicLib::UTMUPS::DecodeZone(utgrid_zone, utm_zone, is_north_hemisphere);

    }


};

#endif
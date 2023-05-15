#include "mirador_driver.h"

MiradorDriver::MiradorDriver(ros::NodeHandle& n)
{
    ros::NodeHandle private_n("~");
    
    // Parameters
    private_n.param<bool>("is_orientation_ned", m_is_orientation_ned, false);
    private_n.param<bool>("is_zero_altitude", m_is_zero_altitude, false);
    private_n.param<std::string>("base_link_frame_id", m_base_link_frame_id, "base_link");
    std::string utm_grid_zone;
    private_n.param<std::string>("utm_zone", utm_grid_zone, "31n");
    GeographicLib::UTMUPS::DecodeZone(utm_grid_zone, m_utm_zone, m_is_north_hemisphere);
    private_n.param<std::string>("ping_topic", m_ping_topic, "/ping");
    private_n.param<std::string>("navsatfix_topic", m_navsatfix_topic, "/fix");
    private_n.param<std::string>("relalt_topic", m_relalt_topic, "/mavros/global_position/rel_alt");

    // Subscribers
    m_navsatfixSubscriber = n.subscribe(m_navsatfix_topic, 10, &MiradorDriver::navSatFixCallback, this);

    // Publishers
    m_statusPublisher = n.advertise<mirador_driver::Status>("/mirador/status", 10);

    m_sequence = 0;
    m_signal_quality = 0;
    m_is_running = false;
    m_position = geographic_msgs::GeoPoint();
    m_heading = .0;
    m_yaw = .0;
    m_publish_cmd_vel = false;
    m_mode = 0;
    m_mission_id = "";
    m_is_running = false;
    m_state_of_charge = .0;
    m_flight_status = 0;
    m_camera_elevation = .0;
    m_camera_zoom = 0;
    m_e_stop = false;
    m_tol = std_msgs::Bool();
    m_mission_context = mirador_driver::MissionContext();

    ros::Time::init();
};

void MiradorDriver::pingCallback(const std_msgs::Float64& _delay)
{
    if (_delay.data < 0) {
        m_signal_quality = 0;
    }
    else {
        m_signal_quality = round(100 * exp(-_delay.data / 100)); // Convert delay in milliseconds into a percentage
    }
    
}

void MiradorDriver::navSatFixCallback(const sensor_msgs::NavSatFix& _navsatfix)
{ 
    m_position.latitude = _navsatfix.latitude;
    m_position.longitude = _navsatfix.longitude;
    m_position.altitude = _navsatfix.altitude;
    latLongToUtm(m_position, m_point_new);
    updateYaw(m_point_new);
}

// -------------------- Publishers --------------------

void MiradorDriver::publishStatus()
{
    mirador_driver::Status status;

    status.signal_quality = m_signal_quality;
    status.pose.latitude = m_position.latitude;
    status.pose.longitude = m_position.longitude;
    status.pose.altitude = m_position.altitude;
    status.pose.heading = m_heading;
    status.mode = m_mode;
    status.mission.id = m_mission_id;
    status.mission.type = m_mode;
    status.mission.points = m_mission_points;
    status.is_running = m_is_running;
    status.state_of_charge = m_state_of_charge;
    status.flight_status = m_flight_status;
    status.camera_elevation = m_camera_elevation;
    status.camera_zoom = m_camera_zoom;
    status.e_stop = m_e_stop;
    status.stream_method = m_stream_method;
    status.stream_address = m_stream_address;
    status.stream_topic = m_stream_topic;
    status.mission_context = m_mission_context;

    m_statusPublisher.publish(status);
}

// -------------------- Functions --------------------

void MiradorDriver::latLongToUtm(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PointStamped& utm_point)
{
    utm_point.header.frame_id = m_utm_frame_id;
    utm_point.header.stamp = ros::Time(0);
    GeographicLib::UTMUPS::Forward(_geo_point.latitude, _geo_point.longitude, m_utm_zone, m_is_north_hemisphere, utm_point.point.x, utm_point.point.y);
    if (m_is_zero_altitude)
    {
        utm_point.point.z = _geo_point.altitude;
    }
}

void MiradorDriver::utmToLatLong(const geometry_msgs::PointStamped& _utm_point, geographic_msgs::GeoPoint& geo_point)
{
    GeographicLib::UTMUPS::Reverse(m_utm_zone, m_is_north_hemisphere, _utm_point.point.x, _utm_point.point.y, geo_point.latitude, geo_point.longitude);
    if (m_is_zero_altitude)
    {
        geo_point.altitude = _utm_point.point.z;
    }
}

void MiradorDriver::updateYaw(geometry_msgs::PointStamped& m_point_new)
{ 
    //Calcul du cap GPS
    if(m_point != geometry_msgs::PointStamped()){
        double x_delta = m_point_new.point.x - m_point.point.x;
        double y_delta = m_point_new.point.y - m_point.point.y;
        
        m_heading = 90 + atan2(y_delta,x_delta)*180/M_PI;

        ROS_INFO("angle: %f", m_heading);

    }
    //MAJ du nouveau point
    m_point = m_point_new;
}
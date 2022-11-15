#include "mirador_driver.h"

MiradorDriver::MiradorDriver(ros::NodeHandle& n) : m_moveBaseClient("move_base", true)
{
    ros::NodeHandle private_n("~");
    
    // Parameters
    private_n.param<bool>("orientation_ned", m_is_orientation_ned, false);
    private_n.param<bool>("zero_altitude", m_is_zero_altitude, false);
    private_n.param<std::string>("utm_frame_id", m_utm_frame_id, "utm");
    private_n.param<std::string>("odom_frame_id", m_odom_frame_id, "odom");
    private_n.param<std::string>("base_link_frame_id", m_base_link_frame_id, "base_link");
    std::string utm_grid_zone;
    private_n.param<std::string>("utm_zone", utm_grid_zone, "31n");
    GeographicLib::UTMUPS::DecodeZone(utm_grid_zone, m_utm_zone, m_is_north_hemisphere);
    private_n.param<std::string>("ping_topic", m_ping_topic, "/ping");
    private_n.param<std::string>("state_of_charge_topic", m_state_of_charge_topic, "/state_of_charge");
    private_n.param<std::string>("navsatfix_topic", m_navsatfix_topic, "/fix");
    private_n.param<bool>("use_odometry", m_use_odometry, false);
    if (m_use_odometry) {
        private_n.param<std::string>("odometry_topic", m_odometry_topic, "/odometry");
    }
    else {
        private_n.param<std::string>("imu_topic", m_imu_topic, "/imu");
    }
    private_n.param<std::string>("flight_status_topic", m_flight_status_topic, "/flight_status");
    private_n.param<std::string>("camera_elevation_topic", m_camera_elevation_topic, "/camera_elevation");
    private_n.param<std::string>("camera_zoom_topic", m_camera_zoom_topic, "/camera_zoom");
    private_n.param<std::string>("e_stop_topic", m_e_stop_topic, "/e_stop");
    private_n.param<int>("stream_method", m_stream_method, int(0));
    private_n.param<std::vector<std::string>>("stream_address", m_stream_address, std::vector<std::string>());

    // Subscribers
    m_missionSubscriber = n.subscribe("/mirador/mission", 10, &MiradorDriver::missionCallback, this);
    m_launchMissionSubscriber = n.subscribe("/mirador/launch", 10, &MiradorDriver::launchMissionCallback, this);
    m_abortMissionSubscriber = n.subscribe("/mirador/abort", 10, &MiradorDriver::abortMissionCallback, this);
    m_reportSubscriber = n.subscribe("/mirador/report", 10, &MiradorDriver::reportCallback, this);
    m_pingSubscriber = n.subscribe(m_ping_topic, 10, &MiradorDriver::pingCallback, this);
    m_stateOfChargeSubscriber = n.subscribe(m_state_of_charge_topic, 10, &MiradorDriver::stateOfChargeCallback, this);
    m_navsatfixSubscriber = n.subscribe(m_navsatfix_topic, 10, &MiradorDriver::navSatFixCallback, this);
    if (m_use_odometry) {
        m_odometrySubscriber = n.subscribe(m_odometry_topic, 10, &MiradorDriver::odometryCallback, this);
    }
    else {
        m_imuSubscriber = n.subscribe(m_imu_topic, 10, &MiradorDriver::imuCallback, this);
    }
    m_flightStatusSubscriber = n.subscribe(m_flight_status_topic, 10, &MiradorDriver::flightStatusCallback, this);
    m_cameraElevationSubscriber = n.subscribe(m_camera_elevation_topic, 10, &MiradorDriver::cameraElevationCallback, this);
    m_cameraZoomSubscriber = n.subscribe(m_camera_zoom_topic, 10, &MiradorDriver::cameraZoomCallback, this);
    m_eStopSubscriber = n.subscribe(m_e_stop_topic, 10, &MiradorDriver::eStopCallback, this);

    // Publishers
    m_statusPublisher = n.advertise<mirador_driver::Status>("/mirador/status", 10);
    m_cmdVelPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Variables init
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
    m_state_of_charge = 0;
    m_flight_status = 0;
    m_camera_elevation = .0;
    m_camera_zoom = 0;
    m_e_stop = false;

    ros::Time::init();
};

// -------------------- Callbacks --------------------

void MiradorDriver::missionCallback(const mirador_driver::Mission& _mission)
{
    if (m_position.latitude != 0 && m_position.longitude != 0)
    {
        if (_mission.type == 1 && (m_mode == 0 || m_mode == 1))
        {
            ROS_INFO("Guide mission received");
            m_mode = _mission.type;
            m_mission_id = _mission.id;
            m_mission_points = _mission.points;

            m_is_running = true;
            ROS_INFO("Guide mission launched");
            setGuide();
        }
        else {
            if (m_mode == 0)
            {
                switch (_mission.type) {
                    case 2 :
                        ROS_INFO("Route mission received");
                        m_mode = _mission.type;
                        m_mission_id = _mission.id;
                        m_mission_points = _mission.points;
                        break;
                    case 3 :
                        ROS_INFO("Exploration mission received");
                        m_mode = _mission.type;
                        m_mission_id = _mission.id;
                        m_mission_points = _mission.points;
                        break;
                    default :
                        ROS_WARN("Unknown mission type");
                }
            }
        }
    }
    else {
        ROS_WARN("No GPS signal, impossible to perform mission");
    }
    
}

void MiradorDriver::reportCallback(const mirador_driver::Report& _report)
{
    //
}

void MiradorDriver::launchMissionCallback(const std_msgs::Empty& _empty)
{
    if (m_mode == 0)
    {
        ROS_INFO("No route mission to launch");
    }
    if (m_mode == 2)
    {
        ROS_INFO("Route mission launched");
        setNextGoal(true);
    }
}

void MiradorDriver::abortMissionCallback(const std_msgs::Empty& _empty)
{
    switch (m_mode) {
        case 1 :
            m_is_running = false;
            m_mode = 0;
            m_mission_id = "";
            m_mission_points = std::vector<geographic_msgs::GeoPoint>();
            m_publish_cmd_vel = false;
            ROS_INFO("Guide mission aborted");
            break;
        case 2 :
            try
            {
                m_is_running = false;
                m_mode = 0;
                m_mission_id = "";
                m_mission_points = std::vector<geographic_msgs::GeoPoint>();
                m_moveBaseClient.cancelGoal();
                ROS_INFO("Route mission aborted");
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN("%s", ex.what());
            }
            break;
        default :
            ROS_WARN("No mission to abort");
    }
}

void MiradorDriver::pingCallback(const std_msgs::Float64& _delay)
{
    if (_delay.data < 0) {
        m_signal_quality = 0;
    }
    else {
        m_signal_quality = round(100 * exp(-_delay.data / 100)); // Convert delay in milliseconds into a percentage
    }
    
}

void MiradorDriver::stateOfChargeCallback(const std_msgs::Float32& _soc)
{
    m_state_of_charge = std::min(std::max(int(round(100 * _soc.data)), 0), 100);
}

void MiradorDriver::navSatFixCallback(const sensor_msgs::NavSatFix& _navsatfix)
{
    m_position.latitude = _navsatfix.latitude;
    m_position.longitude = _navsatfix.longitude;
    if (!m_is_zero_altitude)
    {
        m_position.altitude = _navsatfix.altitude;
    }
}

void MiradorDriver::imuCallback(const sensor_msgs::Imu& _imu)
{
    double siny_cosp;
    double cosy_cosp;
    siny_cosp = 2 * (_imu.orientation.w * _imu.orientation.z + _imu.orientation.x * _imu.orientation.y);
    cosy_cosp = 1 - 2 * (_imu.orientation.y * _imu.orientation.y + _imu.orientation.z * _imu.orientation.z);
    if (m_is_orientation_ned)
    {
        m_heading = 180.0 * std::atan2(siny_cosp, cosy_cosp) / M_PI;
        m_yaw = M_PI / 2 - std::atan2(siny_cosp, cosy_cosp);
    }
    else
    {
        m_heading = 90.0 - 180.0 * std::atan2(siny_cosp, cosy_cosp) / M_PI;
        m_yaw = std::atan2(siny_cosp, cosy_cosp);
    }
}

void MiradorDriver::odometryCallback(const nav_msgs::Odometry& _odometry)
{
    double siny_cosp;
    double cosy_cosp;
    siny_cosp = 2 * (_odometry.pose.pose.orientation.w * _odometry.pose.pose.orientation.z + _odometry.pose.pose.orientation.x * _odometry.pose.pose.orientation.y);
    cosy_cosp = 1 - 2 * (_odometry.pose.pose.orientation.y * _odometry.pose.pose.orientation.y + _odometry.pose.pose.orientation.z * _odometry.pose.pose.orientation.z);
    if (m_is_orientation_ned)
    {
        m_heading = 180.0 * std::atan2(siny_cosp, cosy_cosp) / M_PI;
        m_yaw = M_PI / 2 - std::atan2(siny_cosp, cosy_cosp);
    }
    else
    {
        m_heading = 90.0 - 180.0 * std::atan2(siny_cosp, cosy_cosp) / M_PI;
        m_yaw = std::atan2(siny_cosp, cosy_cosp);
    }
}

void MiradorDriver::flightStatusCallback(const std_msgs::Int8& _flight_status)
{
    m_flight_status = _flight_status.data;
}
void MiradorDriver::cameraElevationCallback(const std_msgs::Float32& _camera_elevation)
{
    m_camera_elevation =  _camera_elevation.data;
}

void MiradorDriver::cameraZoomCallback(const std_msgs::Int8&  _camera_zoom)
{
    m_camera_zoom = _camera_zoom.data;
}

void MiradorDriver::eStopCallback(const std_msgs::Bool& _e_stop)
{
    m_e_stop = _e_stop.data;
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

    m_statusPublisher.publish(status);
}

void MiradorDriver::publishCmdVel()
{
    if (m_publish_cmd_vel && m_signal_quality > 0) {

        geometry_msgs::PointStamped robot_point;
        geometry_msgs::PointStamped guide_point;

        latLongToUtm(m_position, robot_point);
        latLongToUtm(m_mission_points.front(), guide_point);

        Eigen::Vector3d x_robot(robot_point.point.x, robot_point.point.y, m_yaw);
        Eigen::Vector3d x_guide(guide_point.point.x, guide_point.point.y, 0.0);

        Eigen::Matrix3d R;
        R << cos(x_robot(2)), sin(x_robot(2)), 0,
            -sin(x_robot(2)), cos(x_robot(2)), 0,
            0, 0, 1;

        Eigen::Vector3d x;
        x = R * (x_guide - x_robot);

        if ((pow(x(0), 2) + pow(x(1), 2) > 4.0)) {
            Eigen::Vector2d u;

            if (x(0) > 0.0 || (x(0) > -3.0 && abs(x(1)) < 1.0)) {
                Eigen::Matrix2d A;
                A << -1, x(1),
                    0, -x(0);
                Eigen::Vector2d w(1.0, 0); // Place the the carrot at x = 1 meter in front of the robot.
                u = A.inverse() * (w - x.head(2));
            }
            else {
                u(0) = 0;
                u(1) = (atan2((x_guide - x_robot)(1), (x_guide - x_robot)(0))) / M_PI;
            }

            m_cmd_vel.linear.x = std::max(std::min(u(0), 1.0), -1.0);
            m_cmd_vel.angular.z = std::max(std::min(u(1), 1.0), -1.0);
            m_cmdVelPublisher.publish(m_cmd_vel);
        }
        else {
            ROS_INFO("Guide reached");
            m_is_running = false;
            m_mode = 0;
            m_mission_id = "";
            m_mission_points = std::vector<geographic_msgs::GeoPoint>();
            m_publish_cmd_vel = false;
        }
    }
}

// -------------------- Functions --------------------

bool MiradorDriver::setGuide()
{
    if (m_mission_points.size() > 0)
    {
        ROS_INFO("Setting guide");
        m_publish_cmd_vel = true;
        return true;
    }
    ROS_INFO("Empty mission guide gived");
    m_is_running = false;
    m_mode = 0;
    m_mission_id = "";
    m_mission_points = std::vector<geographic_msgs::GeoPoint>();
    return false;
}

bool MiradorDriver::setNextGoal(const bool& _first)
{
    if (_first) {
        if (m_mission_points.size() > 0)
        {
            ROS_INFO("Setting first goal");
            geometry_msgs::PoseStamped target_pose;
            if (!getTargetPose(m_mission_points.front(), target_pose))
            {
                ROS_WARN("Failed to get target pose");
                return false;
            }
            return startMoveBaseGoal(target_pose);
        }
        else
        {
            ROS_WARN("Empty mission gived");
            m_is_running = false;
            m_mode = 0;
            m_mission_id = "";
            m_mission_points = std::vector<geographic_msgs::GeoPoint>();
            return false;
        }
    }
    else {
        if (m_mission_points.size() > 1)
        {
            ROS_INFO("Setting next goal");
            geometry_msgs::PoseStamped target_pose;
            if (!getTargetPose(*(m_mission_points.erase(m_mission_points.begin())), target_pose))
            {
                ROS_WARN("Failed to get target pose");
                return false;
            }
            return startMoveBaseGoal(target_pose);
        }
        else
        {
            ROS_INFO("Mission finished");
            m_is_running = false;
            m_mode = 0;
            m_mission_id = "";
            m_mission_points = std::vector<geographic_msgs::GeoPoint>();
            return false;
        }
    }
}

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

bool MiradorDriver::utmToOdom(const geometry_msgs::PointStamped& _utm_point, geometry_msgs::PointStamped& odom_point)
{
    bool wait = true;
    int count = 0;
    tf2_ros::TransformListener tfListener(m_tf2_buffer);
    geometry_msgs::TransformStamped transformStamped;
    while (wait)
    {
        if (count >= 5)
        {
            ROS_WARN("Cannot transform point from utm frame to odom frame");
            return false;
        }
        try
        {
            count += 1;
            transformStamped = m_tf2_buffer.lookupTransform(m_odom_frame_id, m_utm_frame_id, ros::Time(0));
            tf2::doTransform(_utm_point, odom_point, transformStamped);
            wait = false;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(.1).sleep();
        }
    }
    return true;
}

bool MiradorDriver::odomToUtm(const geometry_msgs::PointStamped& _odom_point, geometry_msgs::PointStamped& utm_point)
{
    bool wait = true;
    int count = 0;
    tf2_ros::TransformListener tfListener(m_tf2_buffer);
    geometry_msgs::TransformStamped transformStamped;
    while(wait)
    {
        if (count >= 5)
        {
            ROS_WARN("Cannot transform point from odom frame to utm frame");
            return false;
        }
        try
        {
            count += 1;
            transformStamped = m_tf2_buffer.lookupTransform(m_utm_frame_id, m_odom_frame_id, ros::Time(0));
            tf2::doTransform(_odom_point, utm_point, transformStamped);
            wait = false;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(.1).sleep();
            continue;
        }
    }
    return true;
}

bool MiradorDriver::getTargetPose(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PoseStamped& target_pose)
{
    target_pose.header.frame_id = m_odom_frame_id;
    target_pose.header.stamp = ros::Time(0);
    geometry_msgs::PointStamped utm_point;
    latLongToUtm(_geo_point, utm_point);
    geometry_msgs::PointStamped target_point;
    if (!utmToOdom(utm_point, target_point))
    {
        return false;
    }
    target_pose.pose.position.x = target_point.point.x;
    target_pose.pose.position.y = target_point.point.y;
    if (!m_is_zero_altitude)
    {
        target_pose.pose.position.z = target_point.point.z;
    }
    target_pose.pose.orientation.w = 1.0;
    m_sequence++;
    target_pose.header.seq = m_sequence;
    ROS_INFO_STREAM("Goal setted to :"<<"\n"<<"    latitude: "<<_geo_point.latitude<<"\n"<<"   longitude: "<<_geo_point.longitude <<"\n"<<"           x: "<<target_pose.pose.position.x <<"\n"<< "           y: "<<target_pose.pose.position.y);
    return true;
}

bool MiradorDriver::startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose) {
    int count = 0;
    while (!m_moveBaseClient.waitForServer(ros::Duration(1.0)) && count <= 5) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = _target_pose;

    ROS_INFO("Sending goal");
    try
    {
        m_moveBaseClient.sendGoal(goal);
        ROS_INFO("Goal sent");
        m_is_running = true;
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_INFO("Failed to send goal");
        return false;
    }
}

void MiradorDriver::processMoveBaseGoal() {
    if (m_is_running && m_mode == 2)
    {
        if (m_moveBaseClient.waitForResult(ros::Duration(0.5))) {
            actionlib::SimpleClientGoalState state = m_moveBaseClient.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Goal reached");
                setNextGoal(false);
            }
            if (state == actionlib::SimpleClientGoalState::ABORTED && state == actionlib::SimpleClientGoalState::REJECTED && state == actionlib::SimpleClientGoalState::LOST)
            {
                ROS_WARN("Error mission stopped");
                m_is_running = false;
                m_mode = 0;
                m_mission_id = "";
                m_mission_points = std::vector<geographic_msgs::GeoPoint>();
            }
        }
    }
}
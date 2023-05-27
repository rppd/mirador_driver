#include "mirador_driver.h"

#include "config.h"
#include "robot.h"

MiradorDriver::MiradorDriver(ros::NodeHandle& handle): moveBaseClient("move_base", true), robot(config) {
    // Subscribers
    missionSubscriber           = handle.subscribe("/mirador/mission", 10, &MiradorDriver::missionCallback, this);
    launchMissionSubscriber     = handle.subscribe("/mirador/launch", 10, &MiradorDriver::launchMissionCallback, this);
    abortMissionSubscriber      = handle.subscribe("/mirador/abort", 10, &MiradorDriver::abortMissionCallback, this);
    reportSubscriber            = handle.subscribe("/mirador/report", 10, &MiradorDriver::reportCallback, this);
    pingSubscriber              = handle.subscribe(config.ping_topic, 10, &MiradorDriver::pingCallback, this);
    stateOfChargeSubscriber     = handle.subscribe(config.state_of_charge_topic, 10, &MiradorDriver::stateOfChargeCallback, this);
    navsatfixSubscriber         = handle.subscribe(config.navsatfix_topic, 10, &MiradorDriver::navSatFixCallback, this);

    if (config.use_odometry) 
        odometrySubscriber      = handle.subscribe(config.odometry_topic, 10, &MiradorDriver::odometryCallback, this);
    else 
        imuSubscriber           = handle.subscribe(config.imu_topic, 10, &MiradorDriver::imuCallback, this);

    flightStatusSubscriber      = handle.subscribe(config.flight_status_topic, 10, &MiradorDriver::flightStatusCallback, this);
    cameraElevationSubscriber   = handle.subscribe(config.camera_elevation_topic, 10, &MiradorDriver::cameraElevationCallback, this);
    cameraZoomSubscriber        = handle.subscribe(config.camera_zoom_topic, 10, &MiradorDriver::cameraZoomCallback, this);
    eStopSubscriber             = handle.subscribe(config.e_stop_topic, 10, &MiradorDriver::eStopCallback, this);
    mission_contextSubscriber   = handle.subscribe(config.mission_context_topic, 10, &MiradorDriver::missionContextCallback, this);

    // Publishers
    statusPublisher = handle.advertise<mirador_driver::Status>("/mirador/status", 10);
    cmdVelPublisher = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

}

void MiradorDriver::reportCallback(const mirador_driver::Report& _report)
{
    //
}

void MiradorDriver::launchMissionCallback(const std_msgs::Empty& _empty)
{
    robot.launchMission();
}

void MiradorDriver::pingCallback(const std_msgs::Float64& _delay)
{
    if (_delay.data < 0) robot.signal_quality = 0;
    else robot.signal_quality = round(100 * exp(-_delay.data / 100)); // Convert delay in milliseconds into a percentage
}

void MiradorDriver::stateOfChargeCallback(const std_msgs::Float32& _soc)
{
    robot.state_of_charge = std::min(std::max(int(round(100 * _soc.data)), 0), 100);
}

void MiradorDriver::navSatFixCallback(const sensor_msgs::NavSatFix& _navsatfix)
{
    robot.position.latitude = _navsatfix.latitude;
    robot.position.longitude = _navsatfix.longitude;
    if (!config.is_zero_altitude)
    {
        robot.position.altitude = _navsatfix.altitude;
    }
}

void MiradorDriver::imuCallback(const sensor_msgs::Imu& _imu)
{
    double siny_cosp;
    double cosy_cosp;
    siny_cosp = 2 * (_imu.orientation.w * _imu.orientation.z + _imu.orientation.x * _imu.orientation.y);
    cosy_cosp = 1 - 2 * (_imu.orientation.y * _imu.orientation.y + _imu.orientation.z * _imu.orientation.z);
    if (config.is_orientation_ned)
    {
        robot.heading = 180.0 * std::atan2(siny_cosp, cosy_cosp) / M_PI;
        robot.yaw = M_PI / 2 - std::atan2(siny_cosp, cosy_cosp);
    }
    else
    {
        robot.heading = 90.0 - 180.0 * std::atan2(siny_cosp, cosy_cosp) / M_PI;
        robot.yaw = std::atan2(siny_cosp, cosy_cosp);
    }
}

void MiradorDriver::odometryCallback(const nav_msgs::Odometry& _odometry)
{
    double siny_cosp;
    double cosy_cosp;
    siny_cosp = 2 * (_odometry.pose.pose.orientation.w * _odometry.pose.pose.orientation.z + _odometry.pose.pose.orientation.x * _odometry.pose.pose.orientation.y);
    cosy_cosp = 1 - 2 * (_odometry.pose.pose.orientation.y * _odometry.pose.pose.orientation.y + _odometry.pose.pose.orientation.z * _odometry.pose.pose.orientation.z);
    if (config.is_orientation_ned)
    {
        robot.heading = 180.0 * std::atan2(siny_cosp, cosy_cosp) / M_PI;
        robot.yaw = M_PI / 2 - std::atan2(siny_cosp, cosy_cosp);
    }
    else
    {
        robot.heading = 90.0 - 180.0 * std::atan2(siny_cosp, cosy_cosp) / M_PI;
        robot.yaw = std::atan2(siny_cosp, cosy_cosp);
    }
}

void MiradorDriver::flightStatusCallback(const std_msgs::Int8& _flight_status)
{
    robot.flight_status = _flight_status.data;
}

void MiradorDriver::cameraElevationCallback(const std_msgs::Float32& _camera_elevation)
{
    robot.camera_elevation =  _camera_elevation.data;
}

void MiradorDriver::cameraZoomCallback(const std_msgs::Int8&  _camera_zoom)
{
    robot.camera_zoom = _camera_zoom.data;
}

void MiradorDriver::eStopCallback(const std_msgs::Bool& _e_stop)
{
    robot.e_stop = _e_stop.data;
}

void MiradorDriver::missionContextCallback(const mirador_driver::MissionContext& _mission_context)
{
    robot.mission_context = _mission_context;
}


void MiradorDriver::publishStatus()
{
    mirador_driver::Status status;

    status.signal_quality = robot.signal_quality;
    status.pose.latitude = robot.position.latitude;
    status.pose.longitude = robot.position.longitude;
    status.pose.altitude = robot.position.altitude;
    status.pose.heading = robot.heading;

    status.mode = robot.mode;
    status.mission.id = robot.mission.id;
    status.mission.type = robot.mode;
    status.mission.points = robot.mission.points;
    status.is_running = robot.is_running;

    status.state_of_charge = robot.state_of_charge;
    status.flight_status = robot.flight_status;
    status.camera_elevation = robot.camera_elevation;
    status.camera_zoom = robot.camera_zoom;
    status.e_stop = robot.e_stop;

    status.stream_method = config.stream_method;
    status.stream_address = config.stream_address;

    status.stream_topic = config.stream_topic;
    status.mission_context = robot.mission_context;

    statusPublisher.publish(status);
}

void MiradorDriver::publishCmdVel()
{
    if (robot.publish_cmd_vel) {
        if (robot.signal_quality > 0) {
            geometry_msgs::Twist cmd_vel_twist;
            if (robot.makeCmdVel(cmd_vel_twist)) {
                cmdVelPublisher.publish(cmd_vel_twist);
            } else {
                ROS_INFO("Guide reached");
            }
        }
        else {
            ROS_WARN("No signal, can't guide robot safely (Host is unreachable by ping command)");
        }
    }
}

void MiradorDriver::processMoveBaseGoal() {
    if (robot.is_running && robot.mode == 2)
    {
        if (moveBaseClient.waitForResult(ros::Duration(0.5))) {
            actionlib::SimpleClientGoalState state = moveBaseClient.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Goal reached");
                robot.setNextGoal(false);
            }
            if (state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::REJECTED || state == actionlib::SimpleClientGoalState::LOST)
            {
                ROS_WARN("Error mission stopped");
                robot.resetMission();
            }
        }
    }
}

bool MiradorDriver::startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose) {
    int count = 0;
    while (!moveBaseClient.waitForServer(ros::Duration(1.0)) && count <= 5) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = _target_pose;

    ROS_INFO("Sending goal");
    try
    {
        moveBaseClient.sendGoal(goal);
        ROS_INFO("Goal sent");
        robot.is_running = true;
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_INFO("Failed to send goal");
        return false;
    }
}

bool MiradorDriver::setNextGoal(bool _first)
{
    // bool _first = mission;
    if (_first) {
        if (robot.mission.points.size() > 0)
        {
            ROS_INFO("Setting first goal");
            geometry_msgs::PoseStamped target_pose;
            if (!getTargetPose(robot.mission.points.front(), target_pose))
            {
                ROS_WARN("Failed to get target pose");
                return false;
            }
            return startMoveBaseGoal(target_pose);
        }
        else
        {
            ROS_WARN("Empty mission given");
            robot.resetMission();
            return false;
        }
    }
    else {
        if (robot.mission.points.size() > 1)
        {
            ROS_INFO("Setting next goal");
            geometry_msgs::PoseStamped target_pose;
            
            if (!getTargetPose(*(robot.mission.points.erase(robot.mission.points.begin())), target_pose))
            {
                ROS_WARN("Failed to get target pose");
                return false;
            }
            return startMoveBaseGoal(target_pose);
        }
        else
        {
            ROS_INFO("Mission finished");
            robot.resetMission();
            return false;
        }
    }
}
   

bool MiradorDriver::setGuide()
{
    if (robot.mission.points.size() > 0)
    {
        ROS_INFO("Setting guide");
        robot.publish_cmd_vel = true;
        return true;
    }
    ROS_INFO("Empty mission guide gived");
    robot.resetMission();
    return false;
}

bool MiradorDriver::getTargetPose(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PoseStamped& target_pose)
{
    tf2_ros::Buffer tf2_buffer;
    target_pose.header.frame_id = config.odom_frame_id;
    target_pose.header.stamp = ros::Time(0);
    geometry_msgs::PointStamped utm_point;
    robot.latLongToUtm(_geo_point, utm_point);
    geometry_msgs::PointStamped target_point;
    if (!robot.utmToOdom(utm_point, target_point))
    {
        return false;
    }
    target_pose.pose.position.x = target_point.point.x;
    target_pose.pose.position.y = target_point.point.y;
    if (!config.is_zero_altitude)
    {
        target_pose.pose.position.z = target_point.point.z;
    }
    target_pose.pose.orientation.w = 1.0;
    sequence++;
    target_pose.header.seq = sequence;
    ROS_INFO_STREAM("Goal setted to :"<<"\n"<<"    latitude: "<<_geo_point.latitude<<"\n"<<"   longitude: "<<_geo_point.longitude <<"\n"<<"           x: "<<target_pose.pose.position.x <<"\n"<< "           y: "<<target_pose.pose.position.y);
    return true;
}

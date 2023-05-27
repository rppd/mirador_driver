#include "controller.h"
#include "mirador_driver.h"

void Controller::reportCallback(const mirador_driver::Report& _report)
{
    //
}

void Controller::launchMissionCallback(const std_msgs::Empty& _empty)
{
    robot.launchMission();
}

void Controller::pingCallback(const std_msgs::Float64& _delay)
{
    if (_delay.data < 0) robot.signal_quality = 0;
    else robot.signal_quality = round(100 * exp(-_delay.data / 100)); // Convert delay in milliseconds into a percentage
}

void Controller::stateOfChargeCallback(const std_msgs::Float32& _soc)
{
    robot.state_of_charge = std::min(std::max(int(round(100 * _soc.data)), 0), 100);
}

void Controller::navSatFixCallback(const sensor_msgs::NavSatFix& _navsatfix)
{
    robot.position.latitude = _navsatfix.latitude;
    robot.position.longitude = _navsatfix.longitude;
    if (!config.is_zero_altitude)
    {
        robot.position.altitude = _navsatfix.altitude;
    }
}

void Controller::imuCallback(const sensor_msgs::Imu& _imu)
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

void Controller::odometryCallback(const nav_msgs::Odometry& _odometry)
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

void Controller::flightStatusCallback(const std_msgs::Int8& _flight_status)
{
    robot.flight_status = _flight_status.data;
}

void Controller::cameraElevationCallback(const std_msgs::Float32& _camera_elevation)
{
    robot.camera_elevation =  _camera_elevation.data;
}

void Controller::cameraZoomCallback(const std_msgs::Int8&  _camera_zoom)
{
    robot.camera_zoom = _camera_zoom.data;
}

void Controller::eStopCallback(const std_msgs::Bool& _e_stop)
{
    robot.e_stop = _e_stop.data;
}

void Controller::missionContextCallback(const mirador_driver::MissionContext& _mission_context)
{
    robot.mission_context = _mission_context;
}


void Controller::publishStatus()
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

void Controller::publishCmdVel()
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

void Controller::processMoveBaseGoal() {
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

bool Controller::startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose) {
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

bool Controller::setNextGoal(bool _first)
{
    // bool _first = mission;
    if (_first) {
        if (robot.mission.points.size() > 0)
        {
            ROS_INFO("Setting first goal");
            geometry_msgs::PoseStamped target_pose;
            if (!MiradorDriver::getTargetPose(robot.mission.points.front(), target_pose))
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
            
            if (!MiradorDriver::getTargetPose(*(robot.mission.points.erase(robot.mission.points.begin())), target_pose))
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
   
#include "robot.h"
#include "logger.h"
 
bool Robot::makeCmdVel(geometry_msgs::Twist& cmd_vel_twist) {
    geometry_msgs::PointStamped robot_point;
    geometry_msgs::PointStamped guide_point;

    latLongToUtm(position, robot_point);
    latLongToUtm(mission.points.front(), guide_point);

    Eigen::Vector3d x_robot(robot_point.point.x, robot_point.point.y, yaw);
    Eigen::Vector3d x_guide(guide_point.point.x, guide_point.point.y, 0.0);

    Eigen::Matrix3d R;
    R << cos(x_robot(2)), sin(x_robot(2)), 0,
        -sin(x_robot(2)), cos(x_robot(2)), 0,
        0, 0, 1;

    Eigen::Vector3d x;
    x = R * (x_guide - x_robot);

    if ((pow(x(0), 2) + pow(x(1), 2) > 4.0)) {
        Eigen::Vector2d u;

        if ((x(0) - abs(x(1)) + 1) > 0 || (x(0) > -3.0 && abs(x(1)) < 1)) {
            Eigen::Matrix2d A;
            A << -1, x(1),
                0, -x(0);
            Eigen::Vector2d w(1.0, 0); // Place the the carrot at x = 1 meter in front of the robot.
            u = A.inverse() * (w - x.head(2));
        }
        else {
            u(0) = 0;
            if (x(1) > 0) {
                u(1) = 1.0; // Rotate inplace to face the target.
            }
            else {
                u(1) = -1.0;
            }
        }

        cmd_vel_twist.linear.x = std::max(std::min(u(0), 1.0), -1.0);
        cmd_vel_twist.angular.z = std::max(std::min(u(1), 1.0), -1.0);
        return true;
    } else {
        return false; //goal reached
    }
}

void Robot::launchMission() {
        if (mode == 0) Logger::info("No route mission to launch");
        if (mode == 2) {
            Logger::info("Route mission launched");
            setNextGoal(true);
        }      
    }

void Robot::resetMission()
{
    is_running = false;
    mode = 0;
    mission.id = "";
    mission.points = std::vector<geographic_msgs::GeoPoint>();
}

void Robot::latLongToUtm(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PointStamped& utm_point)
{
    utm_point.header.frame_id = config.utm_frame_id;
    utm_point.header.stamp = ros::Time(0);
    GeographicLib::UTMUPS::Forward(_geo_point.latitude, _geo_point.longitude, config.utm_zone, config.is_north_hemisphere, utm_point.point.x, utm_point.point.y);
    if (config.is_zero_altitude)
    {
        utm_point.point.z = _geo_point.altitude;
    }
}

void Robot::utmToLatLong(const geometry_msgs::PointStamped& _utm_point, geographic_msgs::GeoPoint& geo_point)
{
    GeographicLib::UTMUPS::Reverse(config.utm_zone, config.is_north_hemisphere, _utm_point.point.x, _utm_point.point.y, geo_point.latitude, geo_point.longitude);
    if (config.is_zero_altitude)
    {
        geo_point.altitude = _utm_point.point.z;
    }
}

bool Robot::utmToOdom(const geometry_msgs::PointStamped& _utm_point, geometry_msgs::PointStamped& odom_point)
{
    tf2_ros::Buffer tf2_buffer;
    bool wait = true;
    int count = 0;
    tf2_ros::TransformListener tfListener(tf2_buffer);
    geometry_msgs::TransformStamped transformStamped;
    while (wait)
    {
        if (count >= 5)
        {
            Logger::warn("Cannot transform point from utm frame to odom frame");
            return false;
        }
        try
        {
            count += 1;
            transformStamped = tf2_buffer.lookupTransform(config.odom_frame_id, config.utm_frame_id, ros::Time(0));
            tf2::doTransform(_utm_point, odom_point, transformStamped);
            wait = false;
        }
        catch (tf2::TransformException& ex)
        {
            Logger::warn(ex.what());
            ros::Duration(.1).sleep();
        }
    }
    return true;
}

bool Robot::odomToUtm(const geometry_msgs::PointStamped& _odom_point, geometry_msgs::PointStamped& utm_point)
{
    tf2_ros::Buffer tf2_buffer;
    bool wait = true;
    int count = 0;
    tf2_ros::TransformListener tfListener(tf2_buffer);
    geometry_msgs::TransformStamped transformStamped;
    while(wait)
    {
        if (count >= 5)
        {
            Logger::warn("Cannot transform point from odom frame to utm frame");
            return false;
        }
        try
        {
            count += 1;
            transformStamped = tf2_buffer.lookupTransform(config.utm_frame_id, config.odom_frame_id, ros::Time(0));
            tf2::doTransform(_odom_point, utm_point, transformStamped);
            wait = false;
        }
        catch (tf2::TransformException &ex) {
            Logger::warn(ex.what());
            ros::Duration(.1).sleep();
            continue;
        }
    }
    return true;
}

bool Robot::setNextGoal(bool _first)
{
    // bool _first = mission;
    if (_first) {
        if (mission.points.size() > 0)
        {
            ROS_INFO("Setting first goal");
            geometry_msgs::PoseStamped target_pose;
            if (!getTargetPose(mission.points.front(), target_pose))
            {
                Logger::warn("Failed to get target pose");
                return false;
            }
            return startMoveBaseGoal(target_pose);
        }
        else
        {
            Logger::warn("Empty mission given");
            resetMission();
            return false;
        }
    }
    else {
        if (mission.points.size() > 1)
        {
            ROS_INFO("Setting next goal");
            geometry_msgs::PoseStamped target_pose;
            
            if (!getTargetPose(*(mission.points.erase(mission.points.begin())), target_pose))
            {
                Logger::warn("Failed to get target pose");
                return false;
            }
            return startMoveBaseGoal(target_pose);
        }
        else
        {
            ROS_INFO("Mission finished");
            resetMission();
            return false;
        }
    }
}

bool Robot::getTargetPose(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PoseStamped& target_pose)
{
    tf2_ros::Buffer tf2_buffer;
    target_pose.header.frame_id = config.odom_frame_id;
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

bool Robot::startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose) {
    int count = 0;
    while (!moveBaseClient.waitForServer(ros::Duration(1.0)) && count <= 5) {
        Logger::info("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = _target_pose;

    Logger::info("Sending goal");
    try
    {
        moveBaseClient.sendGoal(goal);
        Logger::info("Goal sent");
        is_running = true;
        return true;
    }
    catch(const std::exception& e)
    {
        Logger::info("Failed to send goal");
        return false;
    }
}

bool Robot::setGuide()
{
    if (mission.points.size() > 0)
    {
        Logger::info("Setting guide");
        publish_cmd_vel = true;
        return true;
    }
    Logger::info("Empty mission guide gived");
    resetMission();
    return false;
}

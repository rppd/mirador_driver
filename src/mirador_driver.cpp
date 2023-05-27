#include "mirador_driver.h"

#include "config.h"
#include "robot.h"
#include "controller.h"

MiradorDriver::MiradorDriver(ros::NodeHandle& n): robot(config), controller(robot,config, n) {   };

bool MiradorDriver::setGuide()
{
    if (robot.mission.points.size() > 0)
    {
        ROS_INFO("Setting guide");
        robot.publish_cmd_vel = true;
        return true;
    }
    ROS_INFO("Empty mission guide gived");
    resetMission();
    return false;
}



bool MiradorDriver::odomToUtm(const geometry_msgs::PointStamped& _odom_point, geometry_msgs::PointStamped& utm_point)
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

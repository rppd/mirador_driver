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

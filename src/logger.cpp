#include "logger.h"

ros::Publisher Logger::warningPublisher;

void Logger::setTopic(ros::NodeHandle& handle, std::string topic) {
    warningPublisher = handle.advertise<mirador_driver::Warning>(topic, 10);
}

void Logger::info(std::string msg) {
    ROS_INFO_STREAM(msg);
    publishWarnMessage(msg);
}

void Logger::warn(std::string msg) {
    ROS_WARN_STREAM(msg);
    publishWarnMessage(msg);
}

void Logger::publishWarnMessage(std::string msg) {
    mirador_driver::Warning warning;
    warning.message = msg;
    warningPublisher.publish(warning);
}

#pragma once

#include <ros/ros.h>
#include "mirador_driver/Warning.h"

class Logger {

public:
    static ros::Publisher warningPublisher;
    
    static void setTopic(ros::NodeHandle& handle, std::string message);    
    static void info(std::string message);
    static void warn(std::string message);
    static void publishWarnMessage(std::string message);
    
};
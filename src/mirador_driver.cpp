#include "mirador_driver.h"

#include "config.h"
#include "robot.h"
#include "controller.h"

MiradorDriver::MiradorDriver(ros::NodeHandle& n): robot(config), controller(robot,config, n) {   };
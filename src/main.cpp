#include "mirador_driver.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mirador_driver");

    // Variables init
    ros::Time::init();

    ros::NodeHandle n;
    MiradorDriver MiradorDriver(n);

    int rate;
    ros::NodeHandle private_n("~");
    private_n.param<int>("rate", rate, int(10));
    ros::Rate r(rate);
    
    while (ros::ok())
    {
        MiradorDriver.publishStatus();
        MiradorDriver.publishCmdVel();
        MiradorDriver.processMoveBaseGoal();
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
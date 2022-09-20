#include "mirador_driver.h"
#include "ping.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mirador_driver");

    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <host>" << std::endl;
#if !defined(BOOST_WINDOWS)
        std::cerr << "(You may need to set permission to run this program as root.)\n$ chown root.root mirador_driver; ls -al mirador_driver; chmod 4755 mirador_driver\n" << std::endl;
#endif
        return 1;
    }

    boost::asio::io_service io_service;
    pinger p(io_service, argv[1]);

    ros::NodeHandle n;
    MiradorDriver MiradorDriver(n);
    std::string ip_address;
    int rate;
    ros::NodeHandle private_n("~");
    private_n.param<std::string>("ip_address", ip_address, "0.0.0.0");
    private_n.param<int>("rate", rate, int(10));
    ros::Time last_time = ros::Time::now();
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        io_service.run_one();             
        if ( now - last_time > ros::Duration(rate) ) {
            MiradorDriver.publishStatus(p.delay);
            ros::spinOnce();
            last_time = now;
        }
    }
    return 0;
}
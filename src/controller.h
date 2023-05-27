#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "mirador_driver.h"

#include "robot.h"
#include "config.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Controller {
    
    ros::Subscriber missionSubscriber;
    ros::Subscriber reportSubscriber;
    ros::Subscriber abortMissionSubscriber;
    ros::Subscriber launchMissionSubscriber;
    ros::Subscriber pingSubscriber;
    ros::Subscriber stateOfChargeSubscriber;
    ros::Subscriber navsatfixSubscriber;
    ros::Subscriber imuSubscriber;
    ros::Subscriber odometrySubscriber;
    ros::Subscriber flightStatusSubscriber;
    ros::Subscriber cameraElevationSubscriber;
    ros::Subscriber cameraZoomSubscriber;
    ros::Subscriber eStopSubscriber;
    ros::Subscriber mission_contextSubscriber;

    ros::Publisher statusPublisher;
    ros::Publisher cmdVelPublisher;

    MoveBaseClient moveBaseClient;
    Robot& robot;
    Config& config;

    int sequence;

    void missionCallback(const mirador_driver::Mission& _mission);
    void reportCallback(const mirador_driver::Report& _report);
    void launchMissionCallback(const std_msgs::Empty& _empty);
    void abortMissionCallback(const std_msgs::Empty& _empty);
    void pingCallback(const std_msgs::Float64& _delay);
    void stateOfChargeCallback(const std_msgs::Float32& _soc);
    void navSatFixCallback(const sensor_msgs::NavSatFix& _navsatfix);
    void imuCallback(const sensor_msgs::Imu& _imu);
    void odometryCallback(const nav_msgs::Odometry& _odometry);
    void flightStatusCallback(const std_msgs::Int8& _flight_status);
    void cameraElevationCallback(const std_msgs::Float32& _camera_elevation);
    void cameraZoomCallback(const std_msgs::Int8&  _camera_zoom);
    void eStopCallback(const std_msgs::Bool& _e_stop);
    void missionContextCallback(const mirador_driver::MissionContext& _mission_context);

    void publishCmdVel();
    void publishStatus();

    void processMoveBaseGoal();
    bool startMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose);
    bool setNextGoal(bool _first);
    bool setGuide();
    bool getTargetPose(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::PoseStamped& target_pose);


public:

    //Controller(): moveBaseClient("move_base", true) {}

    Controller(Robot& _robot, Config& _config, ros::NodeHandle& handle): moveBaseClient("move_base", true), robot(_robot), config(_config) {

        // Subscribers
        missionSubscriber           = handle.subscribe("/mirador/mission", 10, &Controller::missionCallback, this);
        launchMissionSubscriber     = handle.subscribe("/mirador/launch", 10, &Controller::launchMissionCallback, this);
        abortMissionSubscriber      = handle.subscribe("/mirador/abort", 10, &Controller::abortMissionCallback, this);
        reportSubscriber            = handle.subscribe("/mirador/report", 10, &Controller::reportCallback, this);
        pingSubscriber              = handle.subscribe(config.ping_topic, 10, &Controller::pingCallback, this);
        stateOfChargeSubscriber     = handle.subscribe(config.state_of_charge_topic, 10, &Controller::stateOfChargeCallback, this);
        navsatfixSubscriber         = handle.subscribe(config.navsatfix_topic, 10, &Controller::navSatFixCallback, this);

        if (config.use_odometry) 
            odometrySubscriber      = handle.subscribe(config.odometry_topic, 10, &Controller::odometryCallback, this);
        else 
            imuSubscriber           = handle.subscribe(config.imu_topic, 10, &Controller::imuCallback, this);

        flightStatusSubscriber      = handle.subscribe(config.flight_status_topic, 10, &Controller::flightStatusCallback, this);
        cameraElevationSubscriber   = handle.subscribe(config.camera_elevation_topic, 10, &Controller::cameraElevationCallback, this);
        cameraZoomSubscriber        = handle.subscribe(config.camera_zoom_topic, 10, &Controller::cameraZoomCallback, this);
        eStopSubscriber             = handle.subscribe(config.e_stop_topic, 10, &Controller::eStopCallback, this);
        mission_contextSubscriber   = handle.subscribe(config.mission_context_topic, 10, &Controller::missionContextCallback, this);

        // Publishers
        statusPublisher = handle.advertise<mirador_driver::Status>("/mirador/status", 10);
        cmdVelPublisher = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    }

};

#endif

        
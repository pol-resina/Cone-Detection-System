/**
 * @file main.cpp
 * @brief Main file
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "Modules/Manager.hpp"


int main(int argc, char **argv){
    // init
    ros::init(argc, argv, "ftfcd");
    ros::NodeHandle *const nh = new ros::NodeHandle();

    Params params(*nh);

    // Debug publishers
    ros::Publisher pubGround = nh->advertise<sensor_msgs::PointCloud2>(params.common.topics.output.ground, 20);
    ros::Publisher pubTime = nh->advertise<std_msgs::Float32>("ftfcd/debug/time", 1);


    // Manager
    Manager &manager = Manager::getInstance();
    manager.init(params, pubGround);

    // velodyne_points -> sensor_msgs/PointCloud2

    // Subscribers
    ros::Subscriber subVelodyne = nh->subscribe(params.common.topics.input.points, 1, &Manager::velodyneCallback, &manager);
    
    ros::spin();

}
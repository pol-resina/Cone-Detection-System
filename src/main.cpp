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

    std_msgs::Float32 time;
    while (ros::ok()) {
        auto beginMain = std::chrono::high_resolution_clock::now();

        // Program execution
        manager.run();
                
        auto endMain = std::chrono::high_resolution_clock::now();
        time.data = std::chrono::duration_cast<std::chrono::milliseconds>(endMain - beginMain).count();
        pubTime.publish(time);
        std::cout << "MAIN LOOP ROSRATE :  " << time.data << "ms" << std::endl;

        ros::spinOnce();
    }
}
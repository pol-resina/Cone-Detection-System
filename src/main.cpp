/**
 * @file main.cpp
 * @brief Main file
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "Modules/Manager.hpp"

#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

Params Config;

void import_params(ros::NodeHandle &nh);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ftfcd");
    ros::NodeHandle nh; 

    import_params(nh);
    Manager manager = Manager(nh);


    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/limovelo/state", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/limovelo/full_pcl", 10);

	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> limoOut;

	message_filters::Synchronizer<limoOut> syncPos(limoOut(100), odom_sub, pcl_sub);
	syncPos.registerCallback(boost::bind(&Manager::limoveloCallback, &manager, _1, _2));

    // ros::Subscriber subVelodyne = nh.subscribe(Config.common.topics.input.points, 1, &Manager::velodyneCallback, &manager);
    ros::Subscriber subVelodyne = nh.subscribe("/AS/P/ftfcd/compensatedVelodyne", 1, &Manager::velodyneCallback, &manager);

    ros::spin(); 

    return 0;
}

void import_params(ros::NodeHandle &nh){
    // COMMON
    nh.param<std::string>("/ftfcd/common/topics/input/points", Config.common.topics.input.points, "/velodyne_points");
    nh.param<std::string>("/ftfcd/common/topics/output/ground", Config.common.topics.output.ground, "/ftfcd/ground");
    nh.param<std::string>("/ftfcd/common/topics/output/clusters", Config.common.topics.output.clusters, "/ftfcd/clusters");
    nh.param<std::string>("/ftfcd/common/topics/output/observations", Config.common.topics.output.observations, "/AS/P/ftfcd/observations");
    // MANAGER
    nh.param<bool>("/ftfcd/manager/publish_debug", Config.manager.publish_debug, true);

    // RANSAC
    nh.param<bool>("ransac/ground_segmentation_mode", Config.ransac.ground_segmentation_mode, true);
    nh.param<bool>("ransac/publish_ground", Config.ransac.publish_ground, true);
    nh.param<bool>("ransac/publish_obstacle", Config.ransac.publish_obstacle, true);
    nh.param<bool>("ransac/vis_outliers", Config.ransac.vis_outliers, false);
    nh.param<int>("ransac/minz", Config.ransac.minz, -6);
    nh.param<int>("ransac/maxz", Config.ransac.maxz, 6);
    nh.param<int>("ransac/max_x", Config.ransac.max_x, 25);
    nh.param<int>("ransac/max_y", Config.ransac.max_y, 5);
    nh.param<int>("ransac/min_y", Config.ransac.min_y, -5);
    nh.param<double>("ransac/x_car_limit_p", Config.ransac.x_car_limit_p, 0.5);
    nh.param<double>("ransac/x_car_limit_n", Config.ransac.x_car_limit_n, -0.5);
    nh.param<double>("ransac/y_car_limit_p", Config.ransac.y_car_limit_p, 0.5);
    nh.param<double>("ransac/y_car_limit_n", Config.ransac.y_car_limit_n, -0.5);
    nh.param<int>("ransac/max_iterations", Config.ransac.max_iterations, 401);
    nh.param<double>("ransac/dist_threshold", Config.ransac.dist_threshold, 0.1);
    nh.param<double>("ransac/plane_angle", Config.ransac.plane_angle, 0.3);

    // DBSCAN
    nh.param<int>("dbscan/eps", Config.dbscan.eps, 40);
    nh.param<int>("dbscan/minPts", Config.dbscan.minPts, 5);
    nh.param<int>("dbscan/maxPts", Config.dbscan.maxPts, 1000);
    nh.param<double>("dbscan/classification/distX", Config.dbscan.classification.distX, 0.5);
    nh.param<double>("dbscan/classification/distY", Config.dbscan.classification.distY, 0.5);
    nh.param<double>("dbscan/classification/distZ", Config.dbscan.classification.distZ, 1);
}



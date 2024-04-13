#include "Modules/Manager.hpp"

extern struct Params Config;

Manager::Manager(ros::NodeHandle &nh): ransac(Config){
    pubGround = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.ground, 20);
    this->publish_debug_ = Config.manager.publish_debug;
}

void Manager::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if (cloud_msg == nullptr) return; // NO data

    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    sensor_msgs::PointCloud2 msg;
    ransac.removeGround(cloud, msg);

    if (publish_debug_) {
        this->publishGround(msg);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() * 1000 << "ms" << std::endl;
}


void Manager::publishGround(sensor_msgs::PointCloud2 msg){
    msg.header.frame_id = "global";
    msg.header.stamp = ros::Time::now();
    this->pubGround.publish(msg);
}

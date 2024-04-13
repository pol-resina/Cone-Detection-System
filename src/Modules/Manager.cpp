#include "Modules/Manager.hpp"

Manager::Manager() {}
Manager::~Manager() {}

void Manager::init(const Params &params, 
                   const ros::Publisher &pubGround) {
    this->params_ = params.manager;
    this->pubGround_ = pubGround;
}


void Manager::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if (cloud_msg == nullptr) return; // NO data

    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);


    std::string filepath = "/home/pol/bcn54/catkin_ws/src/ftfcd/pcds/example.pcd";
    pcl::io::savePCDFileBinary(filepath, *cloud);


    if (this->params_.publish_debug) {
        this->publishGround(cloud);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() * 1000 << "ms" << std::endl;
}


void Manager::publishGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "global";
    msg.header.stamp = ros::Time::now();
    pcl::toROSMsg(*cloud, msg);
    this->pubGround_.publish(msg);
}

#include "Modules/Manager.hpp"

Manager::Manager() {}
Manager::~Manager() {}

void Manager::init(const Params &params, 
                   const ros::Publisher &pubGround) {
    this->params_ = params.manager;
    this->pubGround_ = pubGround;
}

void Manager::run() {
    ROS_INFO("Manager::run()");

    pcl::PointCloud<pcl::PointXYZI>::Ptr dummyCloud(new pcl::PointCloud<pcl::PointXYZI>);
    velodyneCallback(dummyCloud);


}


void Manager::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if (cloud_msg == nullptr) return; // NO data

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (this->params_.publish_debug) {
        this->publishGround(cloud);
    }
}

void Manager::saveRawpoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {

}


void Manager::publishGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "global";
    msg.header.stamp = ros::Time::now();
    pcl::toROSMsg(*cloud, msg);
    this->pubGround_.publish(msg);
}

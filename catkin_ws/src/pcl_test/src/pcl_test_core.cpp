#include "pcl_test_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh){
    sub_point_cloud_ = nh.subscribe("/velodyne_points",10, &PclTestCore::point_cb, this);

    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

    ros::spin();
}

PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){
    
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

    pcl::VoxelGrid<pcl::PointXYZI> vg;

    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*filtered_pc_ptr);

    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);

    pub_pc.header = in_cloud_ptr->header;

    pub_filtered_points_.publish(pub_pc);
}

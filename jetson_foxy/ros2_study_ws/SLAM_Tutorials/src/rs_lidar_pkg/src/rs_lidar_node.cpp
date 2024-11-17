#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"


void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloudIn;
    pcl::fromROSMsg(*cloud_msg,pointcloudIn);

    int Could_size = pointcloudIn.size();
    for (int i=0;i<Could_size;i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rs_lidar_node"),"pointcloud[%d]: %f %f %f,---size%d",i,pointcloudIn.points[i].x,pointcloudIn.points[i].y,pointcloudIn.points[i].z,Could_size);
    }
}

int main(int argc , char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("rs_lidar_node");

    RCLCPP_INFO(node->get_logger(),"rs_lidar_node start");

    auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", 10, pointcloud_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
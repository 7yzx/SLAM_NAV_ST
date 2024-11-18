#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <chrono>
#include <thread>

void moveForward(ros::Publisher &pub, double distance) {
    // 创建速度消息
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.2;  // 前进速度（米/秒）
    move_cmd.angular.z = 0.0;  // 不旋转

    // 计算前进的时间
    double time_to_move = distance / move_cmd.linear.x; // 计算时间
    ros::Rate rate(10);  // 10 Hz

    // 发布速度命令
    double start_time = ros::Time::now().toSec();
    while (ros::ok() && (ros::Time::now().toSec() - start_time) < time_to_move) {
        pub.publish(move_cmd);
        rate.sleep();
    }

    // 停止小车
    move_cmd.linear.x = 0.0;
    pub.publish(move_cmd);
}

int main(int argc, char **argv) {
    // 初始化 ROS
    ros::init(argc, argv, "move_forward_node");
    ros::NodeHandle nh;

    // 创建速度发布者
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 移动小车 50 cm
    moveForward(pub, 0.5);  // 50 cm

    return 0;
}

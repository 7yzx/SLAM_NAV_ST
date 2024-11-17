#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
    // 初始化
    rclcpp::init(argc, argv);
    // 创建一个节点
    auto node = std::make_shared<rclcpp::Node>("node_01");
    RCLCPP_INFO(node->get_logger(), "node_01 node start");
    rclcpp::spin(node);
    return 0;
}
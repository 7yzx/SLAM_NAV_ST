#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class NavigationClient : public rclcpp::Node
{
public:
    NavigationClient() : Node("send_goal")
    {
        // 创建定时器，每200毫秒执行一次回调函数
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&NavigationClient::timerCallback, this));
        // 创建 TF2 监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // 用于发送 navigate_to_pose 请求
    void sendNavigateToPoseRequest()
    {
        // 创建 navigate_to_pose 客户端
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // 等待连接 navigate_to_pose 服务器
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "navigate_to_pose action server is not available.");
            return;
        }

        // 创建请求消息
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map"; 

        goal_msg.pose.pose.position.x = 1.0;
        goal_msg.pose.pose.position.y = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        navigation_goal_ = goal_msg;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto) {
            RCLCPP_INFO(this->get_logger(), "到达");
            navigation_goal_handle_.reset();
            is_running_ = false;
        };

        auto future_goal_handle = action_client_->async_send_goal(navigation_goal_, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
            return;
        }

        navigation_goal_handle_ = future_goal_handle.get();
        if (!navigation_goal_handle_)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }

        is_running_ = true;
    }

    bool isRunning() { return is_running_; }

    void cancelGoal()
    {
        if (navigation_goal_handle_ && (navigation_goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED || navigation_goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING))
        {
            action_client_->async_cancel_goal(navigation_goal_handle_);
            RCLCPP_INFO(this->get_logger(), "导航取消");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            navigation_goal_handle_.reset();
        }
    }

private:
    // 定时器回调函数，用于获取机器人实时坐标
    void timerCallback()
    {
        if (rclcpp::ok() && navigation_goal_handle_)
        {
            try
            {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "机器人实时坐标 (x,y,yaw)=>(%.3f,%.3f,%.3f)", transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, tf2::getYaw(transform_stamped.transform.rotation));
            }
            catch (tf2::TransformException &ex)
            {
                // RCLCPP_ERROR(this->get_logger(), "Tf_listener Exception: %s", ex.what());
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool is_running_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationClient>();
    node->sendNavigateToPoseRequest(); // 调用NavigateToPose请求

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // 主循环，直到目标到达或节点被关闭
    while (rclcpp::ok() && node->isRunning())
    {
        RCLCPP_INFO_ONCE(node->get_logger(), "开始导航");
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 取消
    if (node->isRunning())
    {
        node->cancelGoal();
    }
    rclcpp::shutdown();
    return 0;
}


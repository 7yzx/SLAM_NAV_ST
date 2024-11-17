#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class topic_Subscriber_01:public rclcpp::Node
{
private:
    /* data */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        double speed = 0.0f;
        if(msg->data == "forward")
        {
            speed = 1.0f;
        }
        else if(msg->data == "backward")
        {
            speed = -1.0f;
        }
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
public:
    topic_Subscriber_01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s node start", name.c_str());

        command_subscription_ = this->create_subscription<std_msgs::msg::String>("command",10 ,std::bind(&topic_Subscriber_01::command_callback,this,std::placeholders::_1));
    }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<topic_Subscriber_01>("topic_subscriber_01");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
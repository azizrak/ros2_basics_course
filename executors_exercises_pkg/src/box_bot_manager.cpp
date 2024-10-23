#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistd.h"
#include <memory>

class BotNode : public rclcpp::Node {
public:
    BotNode(float timer) : Node("bot_subscriber"){
    
        odom1_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        this->wait_time = timer;

        options1.callback_group = odom1_callback_group_;

        subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/box_bot_1/odom", 100,
            std::bind(&BotNode::bot1_callback, this, std::placeholders::_1),
            options1);

        timer1 = this->create_wall_timer(
        this->wait_time, std::bind(&BotNode::bot1_callback, this));
    }

private:

    void bot1_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received odometry message for Bot 1");
    }

    rclcpp::SubscriptionOptions options1;

    float wait_time; 

}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv); 

    float timer = 1.0;
    std::shared_ptr<BotNode> bot_node = 
        std::make_shared<BotNode>(timer);

    rclcpp::executors::MultiThreadedExecutor executor; 
    executor.add_node(bot_node); 
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
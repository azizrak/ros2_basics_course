#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/timer.hpp"
#include <chrono>

using namespace std::chrono_literals;

class VelPublisher : public rclcpp::Node { 

    public:
        VelPublisher() : Node("vel_publisher"){
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            timer_ = create_wall_timer(
            500ms, std::bind(&VelPublisher::timer_callback, this));
        };

    private:

    void timer_callback(){
        geometry_msgs::msg::Twist msg; 
        msg.linear.x = 2.0;
        msg.angular.z = 1.57;
        publisher_->publish(msg); 
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelPublisher>());
    rclcpp::shutdown();
    return 0;
}
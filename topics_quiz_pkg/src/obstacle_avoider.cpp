#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ObstacleAvoider : public rclcpp::Node {

public:
    ObstacleAvoider() : Node("obstacle_avoider"){
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); 
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ObstacleAvoider::timer_callback, this, _1); 
        
        timer_ = create_wall_timer(
            500ms, std::bind(&ObstacleAvoider::timer_callback, this))
        

        //sensor_msgs::msg::LaserScan

    };


private:

    void timer_callback(){
        geometry_msgs::msg::Twist msg_pub; 
        sensor_msgs::msg::LaserScan msg_sub; 

        msg_pub.linear.x = 1; 
        msg_pub.angular.z = 0.5; 

        RCLCPP_INFO(this->get_logger(), "Publisher works!")

        //RCLCPP_INFO(this->get_logger(), "rangesize: '%f'; : '%f'", msg_sub.ranges);

        publisher_->publish(msg_pub);
        
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv)
    rclcpp::spin(std::make_shared<ObstacleAvoider>()); 
    rclcpp::shutdown();
    return 0; 
}
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObstacleAvoider : public rclcpp::Node
{
public:
    ObstacleAvoider() : Node("topics_quiz")
    {
        // Subscriber to the laser scan topic
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ObstacleAvoider::scan_callback, this, _1));
        
        // Publisher to control the robot
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Timer to periodically publish a message
        timer_ = create_wall_timer(
            500ms, std::bind(&ObstacleAvoider::timer_callback, this));
    }

private:
    // Timer callback to publish velocity commands periodically
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg_pub;

        // msg_pub.linear.x = 0;
        // msg_pub.angular.z = 0;
        // publisher_->publish(msg_pub);
        


        // Check for obstacles on the left side
        for (int i = 200; i < scan_data_.ranges.size() / 2; i++) {
            if (scan_data_.ranges[i] <= 1) {
                left = true;
                break;  // Exit the loop once an obstacle is found on the left
            }
        }

        // Check for obstacles on the right side
        for (int i = scan_data_.ranges.size() - 200; i > scan_data_.ranges.size() / 2; i--) {
            if (scan_data_.ranges[i] <= 1) {
                right = true;
                break;  // Exit the loop once an obstacle is found on the right
            }
        }

        // Check for obstacles in front
        if (scan_data_.ranges[scan_data_.ranges.size() / 2] <= 1) {
            front = true;
        }

        // Decide action based on priority
        if (front) {
            // If there's an obstacle in front, turn left
            msg_pub.linear.x = 0;
            msg_pub.angular.z = 0.5;
            front = false;
        } else if (right) {
            // If there's an obstacle on the right, keep turning left
            msg_pub.linear.x = 0;
            msg_pub.angular.z = 0.5;
            right = false;
        } else if (left) {
            // If there's an obstacle on the left, turn right
            msg_pub.linear.x = 0;
            msg_pub.angular.z = -0.5;
            left = false;
        } else {
            // If there are no obstacles, move forward
            msg_pub.linear.x = 1;
            msg_pub.angular.z = 0;
        }

    
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        publisher_->publish(msg_pub);
    }

    // Callback for the laser scan data
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received laser scan data. Distance in front: %f", msg->ranges[msg->ranges.size() / 2]);
         scan_data_=*msg; 
        // You can add your logic here to analyze the scan data and adjust velocity as needed.
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::LaserScan scan_data_; 
    bool left = false; 
    bool right = false;
    bool front = false;
    bool turned = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}

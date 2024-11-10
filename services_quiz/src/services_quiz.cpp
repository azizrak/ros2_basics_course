#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>

using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;

class RotateServerNode : public rclcpp::Node
{
public:
  RotateServerNode()
  : Node("rotate_server")
  {

    srv_ = create_service<Spin>("rotate", std::bind(&RotateServerNode::rotating_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void rotating_callback(
      const std::shared_ptr<Spin::Request> request,
      const std::shared_ptr<Spin::Response>
          response) 
    {

        auto message = geometry_msgs::msg::Twist();

        if (request->direction == "clockwise")
        {   
            // Send velocities to move the robot to the right
            message.linear.x = 0;
            message.angular.z = request->angular_velocity;
            publisher_->publish(message);

            std::this_thread::sleep_for(std::chrono::seconds(request->time));

            message.angular.z = 0;
            publisher_->publish(message);

            response->success = true;
        }
        else if (request->direction == "anticlockwise")
        {
            // Send velocities to stop the robot
            message.linear.x = 0;
            message.angular.z = (-1) * request->angular_velocity;
            publisher_->publish(message);

            std::this_thread::sleep_for(std::chrono::seconds(request->time));
            
            message.linear.z = 0;
            publisher_->publish(message);
            
            response->success = true;
        }
        else {
            response->success = false;
        }
                
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotateServerNode>());
  rclcpp::shutdown();
  return 0;
}

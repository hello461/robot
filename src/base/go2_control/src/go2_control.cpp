#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "lf/sportmodestate"

class Go2SportClientNode : public rclcpp::Node{
public:
    explicit Go2SportClientNode() :
        Node("go2_sport_client_mode"), sport_client_(this){
            // Read velocity of robot for keyboard or nav2, send it to the robot
            twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
                                std::bind(&Go2SportClientNode::twist_callback, this, std::placeholders::_1));
        }

        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist)
        {
            x = float(twist->linear.x);
            y = float(twist->linear.y);
            z = float(twist->angular.z);

            if (x != 0 || y != 0 || z != 0)
            {
                sport_client_.Move(req_, x, y, z);
                std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
            }
            else{
                sport_client_.BalanceStand(req_);
            }
        }

private:
    // Node: publish command to control the real robot via "/api/sport/request" topic
    SportClient sport_client_;
    // subcribe to "cmd_vel"
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    // velcity of the robot
    float x{}, y{}, z{};
    // unitree go2 ros2 request message
    unitree_api::msg::Request req_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2SportClientNode>());
    rclcpp::shutdown();
    return 0;
}
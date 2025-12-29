#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "lf/sportmodestate"

class Go2SportClientNode : public rclcpp::Node{
public:
    
private:
    unitree_go::msg::SportModeState state_;
    SportClient sport_client_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    // rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    return 0;
}
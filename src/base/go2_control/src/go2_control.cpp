#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "lf/sportmodestate"

enum robotStates{
    NORMAL_STAND,
    BALANCE_STAND,
    STAND_UP,
    STAND_DOWN,
    RECOVERY_STAND,
    MOVE,
    STOP_MOVE
};

class Go2SportClientNode : public rclcpp::Node{
public:
    explicit Go2SportClientNode(uint8_t robotState) : 
        Node("go2_sport_client_mode"), sport_client_(this), robotState_(robotState){
            // Read velocity of robot for keyboard or nav2, send it to the robot
            twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
                                std::bind(&Go2SportClientNode::twist_callback, this, std::placeholders::_1));
        }
private:
    unitree_go::msg::SportModeState state_;
    // Node: publish command to control the real robot via "/api/sport/request" topic
    SportClient sport_client_;
    // subcribe to "cmd_vel"
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    // subscribe to "\lf\sportmodestates"
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr robot_states_sub_;
    // select robot mode
    uint8_t robotState_;
    // velcity of the robot
    double x{}, y{}, z{};
    // unitree go2 ros2 request message
    unitree_api::msg::Request req_;
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist){
        x = twist->linear.x;
        y = twist->linear.y;
        z = twist->angular.z;

        if(x != 0 || y != 0 || z != 0){
            sport_client_.Move(req_, x, y, z);
        }
        else{
            sport_client_.BalanceStand(req_);
        }
    }

    void robotControl(){

    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2SportClientNode>());
    rclcpp::shutdown;
    return 0;
}
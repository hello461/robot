#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "common/ros2_sport_client.h"

#define HEIGHT_STAND_UP 0.33

class InitStateRobot : public rclcpp::Node{
public:
    InitStateRobot() : Node("Init_state_robot"), sport_client_(this) {
        sub_ = this->create_subscription<unitree_go::msg::SportModeState>("/lf/sportmodestate", 10, 
                    std::bind(&InitStateRobot::callback, this, std::placeholders::_1));
    }

    void callback(const unitree_go::msg::SportModeState::SharedPtr msg){
        robot_height_ = msg->body_height;
        if(robot_height_ < HEIGHT_STAND_UP){
            sport_client_.StandUp(req_);
        }
        else if(robot_height_ >= HEIGHT_STAND_UP){
            std::cout<<"Robot is stand up, node is shutting down!"<<std::endl;
            rclcpp::shutdown();
        }
        // std::cout<<"robot heigt: "<< robot_height_ <<std::endl;
    }

private:
    unitree_api::msg::Request req_;
    float robot_height_;
    SportClient sport_client_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_;
    // Publish robot states to orther tasks
    
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitStateRobot>());
    rclcpp::shutdown();
    return 0;
}
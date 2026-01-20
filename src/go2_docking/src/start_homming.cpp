#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"

class StartHomming : public rclcpp::Node{
public:
    using Docking = opennav_docking_msgs::action::DockRobot;
    using GoalHandleDocking = rclcpp_action::ClientGoalHandle<Docking>;

    StartHomming() : Node("start_docking_client"){
        this->docking_ptr_ = rclcpp_action::create_client<Docking>(this, "dock_robot");
        RCLCPP_INFO(this->get_logger(), "Start docking activated");

        this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
            std::bind(&StartHomming::send_goal, this));

    }

    void send_goal(){
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->docking_ptr_->wait_for_action_server(std::chrono::milliseconds(5000))){
            RCLCPP_ERROR(this->get_logger(), "Docking sever not available");
            rclcpp::shutdown();
        }

        auto goal_msg = Docking::Goal();
        goal_msg.use_dock_id = true;
        goal_msg.dock_id = "home_dock";

        RCLCPP_INFO(this->get_logger(), "Sending goal");
        
        auto send_goal_options = rclcpp_action::Client<Docking>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&StartHomming::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&StartHomming::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&StartHomming::result_callback, this, _1);

        this->docking_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Docking>::SharedPtr docking_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandleDocking::SharedPtr & goal_handle){
        if(!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Homming was rejected by server");
        } else{
            RCLCPP_INFO(this->get_logger(), "Homming accepted, robot is going to home");
        }
    }

    void feedback_callback(GoalHandleDocking::SharedPtr, const std::shared_ptr<const Docking::Feedback> feedback){
        std::stringstream ss;
        ss << "Current docking states: ";
        ss << feedback->state;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleDocking::WrappedResult & result){
        switch (result.code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Homming was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Homming was canceled");
                return;
            default:
                RCLCPP_INFO(this->get_logger(), "Unknown result code");
                return;
        }

        std::stringstream ss;
        if (result.result->success){
            RCLCPP_INFO(this->get_logger(), "Robot returned home successfully");
            ss << "Result received: Number of entries attempted: " << result.result->num_retries;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartHomming>());
    rclcpp::shutdown();
    return 0;
}


#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sport_model.hpp"
#include "nlohmann/json.hpp"

using namespace std::placeholders;

class TwistBridge : public rclcpp::Node
{
public:
    TwistBridge() : Node("twist_bridge_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "TwistBridge creation can convert geometry_msgs/msg/twist messages into unitree_api/msg/request messages!");
        // Create a request posting object
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        // Create a twist subscription object
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&TwistBridge::twist_cb, this, _1));
    }

private:
    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist)
    {
        // 3-3.Implement message transformation and publishing within the callback function.
        unitree_api::msg::Request request;

        // Conversion
        // Get the linear velocity and angular velocity of the twist message.
        double x = twist->linear.x;
        double y = twist->linear.y;
        double z = twist->angular.z;
        // The default api_id is for balanced standing.
        auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
        // auto api_id = ROBOT_SPORT_API_ID_STANDUP;
        
        // Add switch walk mode
        if(defineWalkMode){
            api_id = ROBOT_SPORT_API_ID_STANDUP;
            defineWalkMode = false;
        }

        if(x != 0 || y != 0 || z != 0)
        {
            api_id = ROBOT_SPORT_API_ID_MOVE;

            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;
            js["z"] = z;
            request.parameter = js.dump();
        }
        request.header.identity.api_id = api_id;
        request_pub_->publish(request);
    }
    bool defineWalkMode = true;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistBridge>());
    rclcpp::shutdown();
    return 0;
}
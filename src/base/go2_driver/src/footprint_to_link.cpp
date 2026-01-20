#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

// Custom node class
class TFDynamicBroadcaster : public rclcpp::Node
{
public:
    TFDynamicBroadcaster() : Node("tf_dynamic_broadcaster")
    {   
        sub_ = this->create_subscription<unitree_go::msg::SportModeState>("/lf/sportmodestate", 10, std::bind(&TFDynamicBroadcaster::state_cb, this, _1));
        timer_ = this->create_wall_timer(100ms, std::bind(&TFDynamicBroadcaster::timer_cb, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "Start the dynamic TF transformation node");
    }

private:
    // Create a subscriber to subscribe to the robot dog's high-level motion status.
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_;
    // Create a timer to publish the TensorFlow transformation from base_footprint to base_link at a certain frequency.
    rclcpp::TimerBase::SharedPtr timer_;
    // TF Broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Robot dog body height
    double body_height_;

    // Subscriber callback function
    void state_cb(const unitree_go::msg::SportModeState::SharedPtr state_msg)
    {
        // Create object
        body_height_ = state_msg->body_height + 0.057; // 0.057. From the urdf file
    }

    // Timer callback function
    void timer_cb()
    {
        // Create object
        geometry_msgs::msg::TransformStamped transform_;
        // Add Font_camera -> base_link object
        geometry_msgs::msg::TransformStamped transform2_;

        // Organizational message
        transform_.header.stamp = this->get_clock()->now();
        transform_.header.frame_id = "base_footprint";
        transform_.child_frame_id = "base_link";

        transform2_.header.stamp = transform_.header.stamp;
        transform2_.header.frame_id = "base_link";
        transform2_.child_frame_id = "Front_camera";

        // The translation vector from base_footprint to base_link only has a value on the z-axis.
        transform_.transform.translation.x = 0.0;
        transform_.transform.translation.y = 0.0;
        transform_.transform.translation.z = body_height_;

        transform2_.transform.translation.x = 0.325;
        transform2_.transform.translation.y = 0.0;
        transform2_.transform.translation.z = 0.049;

        // The rotation vector from base_footprint to base_link, a unit vector, has no value.
        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, 0.0);
        transform_.transform.rotation.x = qtn.x();
        transform_.transform.rotation.y = qtn.y();
        transform_.transform.rotation.z = qtn.z();
        transform_.transform.rotation.w = qtn.w();

        transform2_.transform.rotation.x = 0.0;
        transform2_.transform.rotation.y = 0.0;
        transform2_.transform.rotation.z = 0.0;
        transform2_.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transform_);
        tf_broadcaster_->sendTransform(transform2_);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFDynamicBroadcaster>());
    rclcpp::shutdown();
    return 0;
}


#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

using namespace std::chrono_literals;

namespace dockingPoseCompose{
class DetectedDockPose : public rclcpp::Node{
public:
    DetectedDockPose(const rclcpp::NodeOptions& options) : Node("detected_dock_pose", options){
        // declare parameters with defaul values
        this->declare_parameter("parent_frame","Front_camera");
        this->declare_parameter("child_frame", "tag36h11.1");

        // get values of parameters
        parent_frame_ = this->get_parameter("parent_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();

        // create a transform buffer to store and look up transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        // create a transform listener to receive transforms
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // create a publisher for the dock pose
        dock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/detected_dock_pose", 10);

        // create a time wall that will trigger pose update
        timer_  = this->create_wall_timer(100ms, std::bind(&DetectedDockPose::timer_callback, this));

        // log that we've successfully initialized
        RCLCPP_INFO(this->get_logger(),"Detected pose node is initialized successfully!");
    }
    
private:
    void timer_callback(){
        // Create a new pose message
        geometry_msgs::msg::PoseStamped dock_pose;
        // Set the timestamp to now
        dock_pose.header.stamp = this->get_clock()->now();
        // The frame ID should match the frame we want the pose express in
        dock_pose.header.frame_id = parent_frame_;

        try{
            // Look up the transform
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                parent_frame_, child_frame_, tf2::TimePointZero); // Get lastest transform

            // copy the translation from transform to the pose
            dock_pose.pose.position.x = transform.transform.translation.x;
            dock_pose.pose.position.y = transform.transform.translation.y;
            dock_pose.pose.position.z = transform.transform.translation.z;

            // copy the rotation from the transform to the pose
            dock_pose.pose.orientation = transform.transform.rotation;

            // publish dock pose for the navigation system to use
            dock_pose_pub_->publish(dock_pose);
        }
        catch(const tf2::TransformException& ex){
            // if we can get transform, log it at lovel to avoid spamming
            RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }
    }

    std::string parent_frame_;  // Name of the camera frame
    std::string child_frame_;   // Name of the apriltag frame

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // Buffer for storing transforms
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;    // Timer for periodic publishing
 };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dockingPoseCompose::DetectedDockPose);

// int main(int argc, char** argv){
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<DetectedDockPose>());
//     rclcpp::shutdown();
//     return 0;
// }
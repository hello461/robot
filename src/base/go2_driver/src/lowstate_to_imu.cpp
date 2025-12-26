#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "example_interfaces/msg/float32.hpp"

class LowStateToImuNode : public rclcpp::Node
{
public:
    LowStateToImuNode() : Node("lowstate_to_imu_node")
    {
        // 1. Subscribe to LowState messages
        lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lf/lowstate", 10, 
            std::bind(&LowStateToImuNode::lowstate_callback, this, std::placeholders::_1));

        // 2. Publish standard Imu topic
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // 3. Create a TF publisher
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 4. Create battery voltage info
        battery_pub_ = this->create_publisher<example_interfaces::msg::Float32>("/go2_status", 1);
        RCLCPP_INFO(this->get_logger(), "IMU The conversion node has been started.");
    }

private:
    // Low-level state information subscription
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
    // IMU Information Release
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    // TF Broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Publisher to publish robot's battery
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr battery_pub_;

    // Extracting IMU information from low-level state information
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr lowstate_msg)
    {
        // Create IMU message
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        
        imu_msg->header.stamp = this->now();
        imu_msg->header.frame_id = "imu";

        // Quaternion conversion (Unitree format [w,x,y,z] → ROS [x,y,z,w])
        const auto& quat = lowstate_msg->imu_state.quaternion;
        imu_msg->orientation.x = quat[1];  
        imu_msg->orientation.y = quat[2];  
        imu_msg->orientation.z = quat[3];  
        imu_msg->orientation.w = quat[0];

        // Angular velocity (unit: rad/s)
        imu_msg->angular_velocity.x = lowstate_msg->imu_state.gyroscope[0];
        imu_msg->angular_velocity.y = lowstate_msg->imu_state.gyroscope[1];
        imu_msg->angular_velocity.z = lowstate_msg->imu_state.gyroscope[2];

        // acceleration
        imu_msg->linear_acceleration.x = lowstate_msg->imu_state.accelerometer[0];
        imu_msg->linear_acceleration.y = lowstate_msg->imu_state.accelerometer[1];
        imu_msg->linear_acceleration.z = lowstate_msg->imu_state.accelerometer[2];

        // covariance matrix
        // angular velocity covariance
        imu_msg->angular_velocity_covariance[0] = 0.0002;  // xx
        imu_msg->angular_velocity_covariance[4] = 0.0002;  // yy
        imu_msg->angular_velocity_covariance[8] = 0.0002;  // zz

        // Linear acceleration covariance
        imu_msg->linear_acceleration_covariance[0] = 0.02;  // xx
        imu_msg->linear_acceleration_covariance[4] = 0.02;  // yy
        imu_msg->linear_acceleration_covariance[8] = 0.02;  // zz

        // Directional covariance (-1 indicates unknown)
        imu_msg->orientation_covariance[0] = -1.0;

        // IMU News Release
        imu_pub_->publish(std::move(imu_msg));

        // Battery infomation
        example_interfaces::msg::Float32 batteryVoltage;
        batteryVoltage.data = lowstate_msg->power_v;
        battery_pub_->publish(batteryVoltage);

        // Release TF Transformer
        publish_tf_transform();
    }

    // Release TF Transformer
    void publish_tf_transform()
    {
        geometry_msgs::msg::TransformStamped tf;
        
        tf.header.stamp = this->now();
        tf.header.frame_id = "base_link";    
        tf.child_frame_id = "imu";

        // The precise location of the IMU relative to the base link (obtained from the urdf file).
        tf.transform.translation.x = -0.02557; // Forward offset 2.557cm
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.04232; // Offset upwards by 4.232cm

        // Directly set quaternions
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0; // Unit quaternion, no rotation

        // Send TF transformation
        tf_broadcaster_->sendTransform(tf);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LowStateToImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
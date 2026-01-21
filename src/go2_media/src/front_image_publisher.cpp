#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/header.hpp"

// Include libraries for publish camera information
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/header.hpp"
#include "thread"

class UnitreeFrontCamera : public rclcpp::Node{
public:
    UnitreeFrontCamera() : Node("front_camera"), cim_(this){
        // create publisher for publish image
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 1);
        // create publisher for publish camera info
        pub_ci_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 1);

        // Declare parameters
        this->declare_parameter("camera_name", "front");
        this->declare_parameter("camera_info_url", "package://go2_media/config/camera_config_2.yaml");
        this->declare_parameter("frame_id", "Front_camera");

        // Get value of parameters
        name_ = this->get_parameter("camera_name").as_string();
        camera_info_url_ = this->get_parameter("camera_info_url").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Set camera name
        cim_.setCameraName(name_);

        if (!cim_.loadCameraInfo(camera_info_url_)){
            RCLCPP_INFO(this->get_logger(), "Load camera error");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Load camera successfully");
        }

        ci_ = cim_.getCameraInfo();

        header_.frame_id = frame_id_;

        // create time wall
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                    std::bind(&UnitreeFrontCamera::timer_callback, this));
        }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture* cap = new cv::VideoCapture("udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! application/x-rtp, " 
                            "media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw, "
                            "width=1280, height=720, format=BGR ! appsink drop=1", cv::VideoCaptureAPIs::CAP_GSTREAMER);
    sensor_msgs::msg::Image frame_;
    cv::Mat cv_frame_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci_;
    camera_info_manager::CameraInfoManager cim_;

    std::string name_;
    std::string frame_id_;
    std::string camera_info_url_;

    sensor_msgs::msg::CameraInfo ci_;
    std_msgs::msg::Header header_;
    rcl_interfaces::msg::ParameterDescriptor param_descr_camera_info_url;
    std::thread pub_frame_thread_;
    std::thread pub_ci_thread_;

    void timer_callback(){
        std::thread{std::bind(&UnitreeFrontCamera::publish_msg, this)}.detach();
    }

    void publish_msg(){
        // Publish image
        cap->read(cv_frame_);
        frame_ = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_frame_).toImageMsg();

        // assign header to image and camera infor
        header_.stamp = this->get_clock()->now();
        frame_.header = header_;
        ci_.header = header_;
        
        // Publish images
        pub_->publish(std::move(frame_));
        // Publish camera info
        pub_ci_->publish(ci_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeFrontCamera>());
    rclcpp::shutdown();
    return 0;
}
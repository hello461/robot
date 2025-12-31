#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class UnitreeFrontCamera : public rclcpp::Node{
public:
    UnitreeFrontCamera() : Node("front_camera"){
        // create publisher to publish image
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw",10);
        // create time wall
        timer_ = this->create_wall_timer(100ms, 
                    std::bind(&UnitreeFrontCamera::timer_callback, this));
        }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture* cap = new cv::VideoCapture("udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! application/x-rtp," 
                            "media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,"
                            "width=1280,height=720,format=BGR ! appsink drop=1", cv::VideoCaptureAPIs::CAP_GSTREAMER);
    sensor_msgs::msg::Image frame_;
    cv::Mat cv_frame_;
    
    void timer_callback(){
        cap->read(cv_frame_);
        frame_ = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_frame_).toImageMsg();
        pub_->publish(frame_);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeFrontCamera>());
    rclcpp::shutdown();
    return 0;
}
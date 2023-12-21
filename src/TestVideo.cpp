#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node     node("video");
    RCLCPP_INFO(node.get_logger(), "hello world");
    const auto       publisher = node.create_publisher<sensor_msgs::msg::Image>("camera_input", 10);
    cv::VideoCapture capture{"/mnt/c/Projects/C/Algorithm-24/src/power_rune/temp/关灯-红方大能量机关-失败后激活成功的全激活过程.MP4"};
    while (true) {
        cv::Mat frame;
        capture >> frame;


        cv::waitKey(40);

        cv_bridge::CvImage                 cvImage(std_msgs::msg::Header(), "bgr8", frame);
        sensor_msgs::msg::Image::SharedPtr msg = cvImage.toImageMsg();
        RCLCPP_INFO(node.get_logger(), "publishing");
        publisher->publish(*msg);


    }
}

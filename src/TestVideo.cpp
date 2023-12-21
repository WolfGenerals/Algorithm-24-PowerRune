#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node node("video");
    RCLCPP_INFO(node.get_logger(), "hello world");
    const auto       publisher = node.create_publisher<sensor_msgs::msg::Image>("map", 10);
    cv::VideoCapture capture{
                "/mnt/c/Projects/C/Algorithm-24/src/power_rune/temp/关灯-红方大能量机关-失败后激活成功的全激活过程.MP4"
            };
    while (true) {
        cv::Mat frame;
        capture >> frame;

        if (frame.empty()) { break; }
        cv::waitKey(25);

        std_msgs::msg::Header header;
        header.frame_id = "map";
        header.stamp    = node.now();
        cv_bridge::CvImage                 cvImage(header, "bgr8", frame);
        sensor_msgs::msg::Image::SharedPtr msg = cvImage.toImageMsg();
        RCLCPP_INFO(node.get_logger(), "publishing");
        publisher->publish(*msg);
    }
}

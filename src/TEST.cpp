#ifndef TEST_HPP
#define TEST_HPP



#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using namespace tf2_ros;
using ImageMsg = sensor_msgs::msg::Image;


class TestNode final : public Node {
    Time time = now();
    StaticTransformBroadcaster _staticTransformBroadcaster{this};
    Publisher<ImageMsg>::SharedPtr publisher =
        create_publisher<ImageMsg>("/image_raw", 10);
    cv::VideoCapture           capture{
                "/mnt/c/Projects/C/PowerRune/关灯-红方大能量机关-失败后激活成功的全激活过程.MP4"
                // "/mnt/c/Projects/C/PowerRune/关灯-蓝方小能量机关-全激活过程.MP4"
            };
    TimerBase::SharedPtr        timer=
        create_wall_timer(
            20ms,
            [this]() -> void {
                if (!capture.isOpened()) {
                    RCLCPP_ERROR(get_logger(), "video capture is not opened");
                    return;
                }

                cv::Mat frame;
                capture >> frame;
                if (frame.empty()) {
                    RCLCPP_WARN(get_logger(), "frame is empty");
                    return;
                }
                cv::Mat image;
                cv::resize(frame, image, cv::Size(),0.75,0.75);
                std_msgs::msg::Header header{};
                header.frame_id = "camera_link";
                time+=20ms;
                header.stamp    = time;

                ImageMsg::SharedPtr imageMsg = cv_bridge::CvImage{
                    header,
                    "bgr8",
                    image
                }.toImageMsg();
                publisher->publish(*imageMsg);
            }
        );

public:
    TestNode() : Node("test_node") {
        RCLCPP_INFO(get_logger(), "test_node start");

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp            = this->now();
        transformStamped.header.frame_id         = "odom";
        transformStamped.child_frame_id          = "camera_link";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 1);
        transformStamped.transform.rotation = tf2::toMsg(q);
        _staticTransformBroadcaster.sendTransform(transformStamped);
    }
};


int main(int argc, char** argv) {
    init(argc, argv);
    spin(std::make_shared<TestNode>());
    shutdown();
    return 0;
}




#endif //TEST_HPP

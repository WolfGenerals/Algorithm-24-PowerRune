#include <cstdio>
#include "Alias.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


class Node final : public rclcpp::Node {
public:
    explicit Node()
        : rclcpp::Node("power_rune") {
        rclcpp::NodeOptions nodeOptions = get_node_options();
        // nodeOptions.arguments()

        create_subscription<sensor_msgs::msg::Image>(
            "input",
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr imageRos) -> void {
                cv_bridge::CvImageConstPtr image   = cv_bridge::toCvShare(imageRos);
                auto                       time    = image->header.stamp;
                uint32_t                   nanosec = time.nanosec;
            }
        );
    }
};


int main(int argc, char** argv) {
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    spin(std::make_shared<Node>());
    printf("hello world power_rune package\n");
    return 0;
}

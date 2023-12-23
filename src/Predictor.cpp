#include <geometry_msgs/msg/detail/point_stamped__builder.hpp>
#include <rclcpp/node.hpp>
using namespace std;
using namespace rclcpp;

class PredictorNode final : public Node {
    Subscription<geometry_msgs::msg::PointStamped>::SharedPtr center_subscriber;
    Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscriber;

    double omega = 0.0;
public:
    PredictorNode()
        : Node("predictor") {
        RCLCPP_INFO(get_logger(), "predictor start");
    }
};
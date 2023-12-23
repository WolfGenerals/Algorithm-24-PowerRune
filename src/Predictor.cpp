#include <geometry_msgs/msg/detail/point_stamped__builder.hpp>
#include <rclcpp/node.hpp>

#include "Alias.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
// #include "Taylor.hpp"


using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;


class PredictorNode final : public Node {
    tf2_ros::Buffer            buffer{get_clock()};
    tf2_ros::TransformListener listener{buffer};

    deque<geometry_msgs::msg::PointStamped::SharedPtr> history;


    // Configuration
    static auto period() {
        double frequency;
        get_parameter("send_frequency_Hz", frequency);
        return 1s / frequency;
    }
    static auto historySize() {
        int size;
        get_parameter("history_size", size);
        return size;
    }

    // Progress
    void update(const geometry_msgs::msg::PointStamped::SharedPtr& msg) {
        if (!buffer.canTransform("base_link", msg->header.frame_id, tf2::TimePointZero)) {
            RCLCPP_WARN(get_logger(), "can not get transform from base_link to camera_link");
            return;
        }
        const auto pointStamped = buffer.transform(msg, "base_link");
        history.push_back(pointStamped);
        if (history.size() > historySize())
            history.pop_front();
    }

    void predict() {
       //Todo
    }

public:
    PredictorNode()
        : Node("predictor") { RCLCPP_INFO(get_logger(), "predictor start"); }

private:
    auto targetSubscriber = create_subscription<geometry_msgs::msg::PointStamped>(
        "target",
        10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) -> void { update(msg); }
    );
    auto predictionSender = create_wall_timer(period(), [this]() -> void {predict();});
};

#include <geometry_msgs/msg/detail/point_stamped__builder.hpp>
#include <rclcpp/node.hpp>

#include "Alias.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "Function.hpp"


using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;


class PredictorNode final : public Node {
    tf2_ros::Buffer            buffer{get_clock()};
    tf2_ros::TransformListener listener{buffer};

    deque<double> X, Y, Z;
    deque<double> Timestamp;

    Publisher<geometry_msgs::msg::PointStamped> publisher;

    // Configuration
    static auto period() {
        double frequency;
        get_parameter("send_frequency_Hz", frequency);
        return 1s / frequency;
    }
    static  auto delay() {
        double delay;
        get_parameter("delay_ms", delay);
        return delay;
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

        X.push_back(pointStamped->point.x);
        Y.push_back(pointStamped->point.y);
        Z.push_back(pointStamped->point.z);
        Timestamp.push_back(
            pointStamped->header.stamp.sec * 1000 + pointStamped->header.stamp.nanosec / 1000.0
        );
        if (X.size() > historySize()) {
            X.pop_front();
            Y.pop_front();
            Z.pop_front();
            Timestamp.pop_front();
        }
    }

    void predict() {
        QuadraticFunctions x = QuadraticFunctions::fit(X, Timestamp);
        QuadraticFunctions y = QuadraticFunctions::fit(Y, Timestamp);
        QuadraticFunctions z = QuadraticFunctions::fit(Z, Timestamp);

        rclcpp::Time                     now = get_clock()->now();
        double currentTimestamp = now.seconds()*1000 +now.nanoseconds() / 1000.0;

        geometry_msgs::msg::PointStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp    = this->now();
        msg.point.x         = x(currentTimestamp+delay());
        msg.point.y         = y(currentTimestamp+delay());
        msg.point.z         = z(currentTimestamp+delay());
        this->publish(msg);
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
    auto predictionSender = create_wall_timer(period(), [this]() -> void { predict(); });
};

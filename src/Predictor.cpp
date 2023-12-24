#include "Alias.hpp"
#include "Function.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using geometry_msgs::msg::PointStamped;


class PredictorNode final : public Node {
    double _send_frequency_Hz = declare_parameter("send_frequency_Hz", 0.0);
    double _delay_ms          = declare_parameter("delay_ms", 1.0);
    long   _history_size      = declare_parameter("history_size", 100);

    duration<double> period() const {
        double frequency;
        get_parameter("send_frequency_Hz", frequency);
        RCLCPP_INFO(get_logger(), "send_frequency_Hz: %f", frequency);
        return 1000ms / frequency;
    }

    double delay() const {
        double delay;
        get_parameter("delay_ms", delay);
        return delay;
    }

    int historySize() const {
        int size;
        get_parameter("history_size", size);
        return size;
    }


    tf2_ros::Buffer            buffer{get_clock()};
    tf2_ros::TransformListener listener{buffer};

    deque<double> X, Y, Z;
    deque<double> Timestamp;

    Publisher<PointStamped>::SharedPtr publisher =
            create_publisher<PointStamped>("prediction", 10);

    Subscription<PointStamped>::SharedPtr targetSubscriber =
            create_subscription<PointStamped>(
                "target",
                10,
                [this](const PointStamped::SharedPtr msg) -> void {
                    if (!buffer.canTransform(
                        "base_link",
                        msg->header.frame_id,
                        tf2::TimePointZero
                    )) {
                        RCLCPP_WARN(
                            get_logger(),
                            "can not get transform from base_link to camera_link"
                        );
                        return;
                    }

                    const auto pointStamped = buffer.transform(*msg, "base_link");
                    X.push_back(pointStamped.point.x);
                    Y.push_back(pointStamped.point.y);
                    Z.push_back(pointStamped.point.z);
                    Timestamp.push_back(
                        pointStamped.header.stamp.sec * 1000 + pointStamped.header.stamp.nanosec /
                        1000000.0
                    );
                    if (X.size() > historySize()) {
                        X.pop_front();
                        Y.pop_front();
                        Z.pop_front();
                        Timestamp.pop_front();
                    }
                }
            );

    TimerBase::SharedPtr predictionSender =
            create_wall_timer(
                period(),
                [this]() -> void {
                    const auto x = QuadraticFunctions::fit(X, Timestamp);
                    const auto y = QuadraticFunctions::fit(Y, Timestamp);
                    const auto z = QuadraticFunctions::fit(Z, Timestamp);

                    const auto   now              = this->now();
                    const double currentTimestamp =
                            now.seconds() * 1000 +
                            static_cast<int>(now.nanoseconds()) / 1000000.0;

                    PointStamped msg;
                    msg.header.frame_id = "base_link";
                    msg.header.stamp    = now;
                    msg.point.x         = x(currentTimestamp + delay());
                    msg.point.y         = y(currentTimestamp + delay());
                    msg.point.z         = z(currentTimestamp + delay());
                    publisher->publish(msg);
                }
            );

public:
    PredictorNode()
        : Node("predictor") {
        RCLCPP_INFO(get_logger(), "predictor start");
        RCLCPP_INFO(get_logger(), "send_frequency_Hz: %f", period().count());
        RCLCPP_INFO(get_logger(), "delay_ms: %f", delay());
        RCLCPP_INFO(get_logger(), "history_size: %d", historySize());
    }
};


int main(const int argc, char* argv[]) {
    init(argc, argv);
    spin(std::make_shared<PredictorNode>());
    shutdown();
    return 0;
}

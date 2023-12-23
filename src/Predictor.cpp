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


class PredictorNode final : public Node {
    tf2_ros::Buffer            buffer{get_clock()};
    tf2_ros::TransformListener listener{buffer};

    deque<double> X, Y, Z;
    deque<double> Timestamp;

    Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher = create_publisher<
        geometry_msgs::msg::PointStamped>("prediction", 10);

    // Configuration
    duration<double> period() const {
        double frequency;
        get_parameter("send_frequency_Hz", frequency);
        return 1s / frequency;
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

    // Progress
    void update(const geometry_msgs::msg::PointStamped::SharedPtr& msg) {
        if (!buffer.canTransform("base_link", msg->header.frame_id, tf2::TimePointZero)) {
            RCLCPP_WARN(get_logger(), "can not get transform from base_link to camera_link");
            return;
        }
        const auto pointStamped = buffer.transform(*msg, "base_link");

        X.push_back(pointStamped.point.x);
        Y.push_back(pointStamped.point.y);
        Z.push_back(pointStamped.point.z);
        Timestamp.push_back(
            pointStamped.header.stamp.sec * 1000 + pointStamped.header.stamp.nanosec / 1000000.0
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

        const auto   now              = this->now();
        const double currentTimestamp = now.seconds() * 1000 + now.nanoseconds() / 1000000.0;

        geometry_msgs::msg::PointStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp    = now;
        msg.point.x         = x(currentTimestamp + delay());
        msg.point.y         = y(currentTimestamp + delay());
        msg.point.z         = z(currentTimestamp + delay());
        publisher->publish(msg);
    }

public:
    PredictorNode()
        : Node("predictor") {
        declare_parameter("send_frequency_Hz", 0.0);
        declare_parameter("delay_ms", 0.0);
        declare_parameter("history_size", 100);

        RCLCPP_INFO(get_logger(), "predictor start");
        RCLCPP_INFO(get_logger(), "send_frequency_Hz: %f", period().count());
        RCLCPP_INFO(get_logger(), "delay_ms: %f", delay());
        RCLCPP_INFO(get_logger(), "history_size: %d", historySize());
    }

private:
    Subscription<geometry_msgs::msg::PointStamped>::SharedPtr targetSubscriber
            = create_subscription<geometry_msgs::msg::PointStamped>(
                "target",
                10,
                [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) -> void {
                    update(msg);
                }
            );
    TimerBase::SharedPtr predictionSender =
            create_wall_timer(period(), [this]() -> void { predict(); });
};


int main(int argc, char* argv[]) {
    init(argc, argv);
    spin(std::make_shared<PredictorNode>());
    shutdown();
    return 0;
}

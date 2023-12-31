#include "Alias.hpp"
#include "Cache.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/float64_multi_array.hpp"


using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using geometry_msgs::msg::PointStamped;
using std_msgs::msg::Float64MultiArray;
using namespace cv;



/**
 * \brief
 */
class Fit {
    enum class State {
        SMALL,
        LARGE,
        WAIT_MAXIMUM,
        WAIT_MINIMUM,
    };


    static constexpr double MAX = 2.090;

public:
    double A     = 0.0;
    double omega = 0.0;
    double phi   = 0.0;
};


class PredictorNode final : public rclcpp::Node {
    double _send_frequency_Hz = declare_parameter("send_frequency_Hz", 0.0);
    double _delay_ms          = declare_parameter("delay_ms", 1.0);
    long   _history_size      = declare_parameter("history_size", 100);

    double startTimestamp = now().seconds();

    duration<double> period() const {
        double frequency;
        get_parameter("send_frequency_Hz", frequency);
        RCLCPP_INFO(get_logger(), "send_frequency_Hz: %f", frequency);
        return 1000ms / frequency;
    }

    double delay() const {
        double delay;
        get_parameter("delay_s", delay);
        return delay;
    }

    int history_size() const {
        int size;
        get_parameter("history_size", size);
        return size;
    }


    tf2_ros::Buffer            buffer{get_clock()};
    tf2_ros::TransformListener listener{buffer};


    Vec3 lastTarget;
    Cache<Vec3>   centers{history_size()};

    Vec3 axis() const {
        const Vec3 result{centers.avrage()(0), centers.avrage()(1), 0};
        return result / length(result);
    }

    Publisher<PointStamped>::SharedPtr publisher =
            create_publisher<PointStamped>("prediction", 10);


    Subscription<Float64MultiArray>::SharedPtr angularVelocitySubscriber =
                create_subscription<Float64MultiArray>(
                "angular_velocity",
                10,
                [this](const Float64MultiArray::SharedPtr msg) -> void {
                    double time = msg->data[0] - startTimestamp;
                    double anguqlarVelocity = msg->data[1];

                });
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
                        RCLCPP_WARN(get_logger(), "can not get transform from %s to base_link", msg->header.frame_id.c_str());
                        return;
                    }

                    const auto pointStamped = buffer.transform(*msg, "base_link");

                    lastTarget = {
                                static_cast<float>(pointStamped.point.x),
                                static_cast<float>(pointStamped.point.y),
                                static_cast<float>(pointStamped.point.z)
                            };
                }
            );
    Subscription<PointStamped>::SharedPtr centerSubscriber =
            create_subscription<PointStamped>(
                "center",
                10,
                [this](const PointStamped::SharedPtr msg) -> void {
                    if (!buffer.canTransform(
                        "base_link",
                        msg->header.frame_id,
                        tf2::TimePointZero
                    )) {
                        RCLCPP_WARN(get_logger(), "can not get transform from %s to base_link", msg->header.frame_id.c_str());
                        return;
                    }
                    const auto pointStamped = buffer.transform(*msg, "base_link");
                    centers.update(
                        {
                            static_cast<float>(pointStamped.point.x),
                            static_cast<float>(pointStamped.point.y),
                            static_cast<float>(pointStamped.point.z)
                        }
                    );
                }
            );


    TimerBase::SharedPtr predictionSender =
            create_wall_timer(
                period(),
                [this]() -> void {
                    const auto   now              = this->now();
                    const double currentTimestamp = now.seconds() - startTimestamp;


                    PointStamped msg;
                    msg.header.frame_id = "base_link";
                    msg.header.stamp    = now;
                    // msg.point.x         = x(currentTimestamp + delay());
                    // msg.point.y         = y(currentTimestamp + delay());
                    // msg.point.z         = z(currentTimestamp + delay());
                    publisher->publish(msg);
                }
            );

public:
    PredictorNode()
        : Node("predictor") {
        RCLCPP_INFO(get_logger(), "predictor start");
        RCLCPP_INFO(get_logger(), "send_frequency_Hz: %f", period().count());
        RCLCPP_INFO(get_logger(), "delay_s: %f", delay());
        RCLCPP_INFO(get_logger(), "history_size: %d", history_size());
    }
};


int main(const int argc, char* argv[]) {
    init(argc, argv);
    spin(std::make_shared<PredictorNode>());
    shutdown();
    return 0;
}

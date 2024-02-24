#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "Binarizer.hpp"
#include "Configuration.hpp"
#include "FanBladeDetector.hpp"
#include "Masker.hpp"
#include "PowerRune.hpp"
#include "RIconDetector.hpp"
#include "RuneTracker.hpp"
#include "Transformer3D.hpp"

using namespace std;
using namespace cv;
using namespace rclcpp;
using geometry_msgs::msg::PointStamped;
using ImageMsg = sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using std_msgs::msg::Float64;


class PowerRuneNode final : public rclcpp::Node {
    Configuration config{*this};

    tf2_ros::Buffer            buffer{get_clock()};
    tf2_ros::TransformListener listener{buffer};


    Publisher<PointStamped>::SharedPtr prediction_publisher =
            create_publisher<PointStamped>("/prediction", 10);

    Publisher<ImageMsg>::SharedPtr binary_publisher         = create_publisher<ImageMsg>("DEBUG_binary", 10);
    Publisher<ImageMsg>::SharedPtr ricon_publisher          = create_publisher<ImageMsg>("DEBUG_ricon", 10);
    Publisher<ImageMsg>::SharedPtr mask_publisher           = create_publisher<ImageMsg>("DEBUG_mask", 10);
    Publisher<ImageMsg>::SharedPtr powe_runer_2d_publisher  = create_publisher<ImageMsg>("DEBUG_powe_rune_2d", 10);
    Publisher<Float64>::SharedPtr  speed_publisher          = create_publisher<Float64>("DEBUG_speed", 10);
    Publisher<Float64>::SharedPtr  acceleration_publisher   = create_publisher<Float64>("DEBUG_acceleration", 10);
    Publisher<ImageMsg>::SharedPtr preditction_2d_publisher = create_publisher<ImageMsg>("DEBUG_preditction_2d", 10);

    Binarizer        binarizer{config};
    RIconDetector    riconDetector{config};
    Masker           masker{config};
    FanBladeDetector fanBladeDetector{config};
    RuneTracker      runeTracker{config};
    Transformer3D    transformer{config};

    void progress(const Mat& image, const std_msgs::msg::Header& header) {
        const Mat binary = binarizer.binary(image);
        // DEBUG
        if (config.DEBUG())
            binary_publisher->publish(*cv_bridge::CvImage(header, "mono8", binary).toImageMsg());
        const optional<RIcon> icons = riconDetector.detect(binary);
        if (!icons) return;
        // DEBUG
        if (config.DEBUG()) {
            Mat out = image.clone();
            circle(out, icons->position, static_cast<int>(icons->range), Scalar(0, 255, 0), 1);
            circle(out, icons->position, static_cast<int>(config.R标最大半径()), Scalar(255, 255, 255), 1);
            circle(out, icons->position, static_cast<int>(config.R标最小半径()), Scalar(255, 255, 255), 1);
            ricon_publisher->publish(*cv_bridge::CvImage(header, "bgr8", out).toImageMsg());
        }
        const Mat masked = masker.mask(icons->position, binary);
        // DEBUG
        if (config.DEBUG()) {
            Mat out = image.clone();
            circle(out, icons->position, config.外圈半径(), Scalar(0, 255, 255), 3);
            circle(out, icons->position, config.内圈半径(), Scalar(0, 255, 255), 3);
            mask_publisher->publish(*cv_bridge::CvImage(header, "bgr8", out).toImageMsg());
        }
        const auto fanBlades = fanBladeDetector.detect(icons->position, masked);
        runeTracker.track(fanBlades, std::chrono::milliseconds{header.stamp.sec * 1000 + header.stamp.nanosec / 1000000});
        if (runeTracker.state == RuneTracker::State::LOST) return;
        const PowerRune rune = runeTracker.rune;
        // DEBUG
        if (config.DEBUG()) {
            Mat out = image.clone();
            circle(out, icons->position, static_cast<int>(icons->range), Scalar(0, 255, 0), 3);
            for (const auto& [offset, state]: rune.fanBlades) {
                Scalar color{255};
                switch (state) {
                    case RuneState::INACTIVE:
                        color = Scalar(255, 0, 0);
                        break;
                    case RuneState::ACTIVE:
                        color = Scalar(255, 255, 0);
                        break;
                    case RuneState::DISABLED:
                        color = Scalar(0, 255, 255);
                        break;
                    default: ;
                }
                circle(out, icons->position + offset, 5, color, -1);
            }
            powe_runer_2d_publisher->publish(*cv_bridge::CvImage(header, "bgr8", out).toImageMsg());
        }
        // DEBUG
        if (config.DEBUG()) {
            Float64 speedMsg;
            speedMsg.data = rune.speed;
            speed_publisher->publish(speedMsg);
        }
        // DEBUG
        if (config.DEBUG()) {
            Float64 accelerationMsg;
            accelerationMsg.data = rune.acceleration;
            acceleration_publisher->publish(accelerationMsg);
        }
        PowerRune prediction = rune;
        prediction.updateAngle(rune.speed * config.命中延迟_秒());
        // DEBUG
        if (config.DEBUG()) {
            Mat out = image.clone();
            circle(out, icons->position, static_cast<int>(icons->range), Scalar(0, 255, 0), 3);
            for (const auto& [offset, state]: prediction.fanBlades) {
                Scalar color{255};
                switch (state) {
                    case RuneState::INACTIVE:
                        color = Scalar(255, 0, 0);
                        break;
                    case RuneState::ACTIVE:
                        color = Scalar(255, 255, 0);
                        break;
                    case RuneState::DISABLED:
                        color = Scalar(0, 255, 255);
                        break;
                    default: ;
                }
                circle(out, icons->position + offset, 5, color, -1);
            }
            preditction_2d_publisher->publish(*cv_bridge::CvImage(header, "bgr8", out).toImageMsg());
        }

        std::optional<FanBlade> targe = prediction.targe();
        if (!targe) return;
        Point3d      direction = transformer.direction(targe->offset + icons->position);
        double       distance  = transformer.distance(rune.length, 0.7);
        PointStamped predictionMsg;
        predictionMsg.header          = header;
        predictionMsg.header.frame_id = "camera_link";
        predictionMsg.point.x         = direction.x * distance;
        predictionMsg.point.y         = direction.y * distance;
        predictionMsg.point.z         = direction.z * distance;
        if (!buffer.canTransform("camera_link", "odom",header.stamp, tf2::Duration::zero()))return;
        prediction_publisher->publish(buffer.transform(predictionMsg, "odom", tf2::Duration::zero()));
    }

    Subscription<ImageMsg>::SharedPtr image_subscriber =
            create_subscription<ImageMsg>(
                "/image_raw",
                10,
                [this](const ImageMsg::SharedPtr imageRos) -> void {
                    if (!config.enable()) return;
                    const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(imageRos);
                    if (image->image.empty())
                        return;
                    progress(image->image, imageRos->header);
                }
            );

public:
    PowerRuneNode() : Node("node") {}
};


int main(const int argc, char** argv) {
    init(argc, argv);
    spin(make_shared<PowerRuneNode>());
    return 0;
}

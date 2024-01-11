#include <cstdio>
#include "Alias.hpp"
#include "Cache.hpp"
#include "Converter/Transform.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std;
using namespace cv;
using namespace rclcpp;
using geometry_msgs::msg::PointStamped;
using ImageMsg = sensor_msgs::msg::Image;
using std_msgs::msg::Float64MultiArray;

double rotation(const Vec3 axis,const Vec3 from,const Vec3 to) {
    return asin(
        Mat{from}.cross(to).dot(axis) /
        sqrt(from(0) * from(0) + from(1) * from(1) + from(2) * from(2) - from.ddot(axis) * from.ddot(axis)) /
        sqrt(to(0) * to(0) + to(1) * to(1) + to(2) * to(2) - to.ddot(axis) * to.ddot(axis))
    );
};

class PowerRuneNode final : public rclcpp::Node {
    vector<double> _world_target = declare_parameter("world_target", vector<double>());
    vector<double> _world_center = declare_parameter("world_center", vector<double>());
    vector<double> _image_target = declare_parameter("image_target", vector<double>());
    vector<double> _image_center = declare_parameter("image_center", vector<double>());
    vector<double> _world_points = declare_parameter("world_points", vector<double>());
    vector<double> _image_points = declare_parameter("image_points", vector<double>());
    long           _history_size = declare_parameter("history_size", 10);
    string         _source_image = declare_parameter("source_image", "null");

    shared_ptr<Feature> sourceFeature = nullptr;

    std::vector<Vec3> world_points() const {
        vector<Vec3>   points;
        vector<double> rawData;

        get_parameter("world_points", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "world_points is empty");
        if (rawData.size() % 3 != 0)
            RCLCPP_ERROR(get_logger(), "world_points size error");

        for (unsigned long i = 0; i < rawData.size(); i += 3) {
            points.emplace_back(rawData[i], rawData[i + 1], rawData[i + 2]);
        }
        return points;
    };

    std::vector<Vec2> image_points() const {
        vector<Vec2>   points;
        vector<double> rawData;

        get_parameter("image_points", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "image_points is empty");
        if (rawData.size() % 2 != 0)
            RCLCPP_ERROR(get_logger(), "image_points size error");

        for (unsigned long i = 0; i < rawData.size(); i += 2) {
            points.emplace_back(rawData[i], rawData[i + 1]);
        }
        return points;
    };

    Vec3 world_target() const {
        vector<double> rawData;

        get_parameter("world_target", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "world_target is empty");
        if (rawData.size() != 3)
            RCLCPP_ERROR(get_logger(), "world_target size error");

        return {
                    static_cast<float>(rawData[0]),
                    static_cast<float>(rawData[1]),
                    static_cast<float>(rawData[2])
                };
    };

    Vec3 world_center() const {
        vector<double> rawData;

        get_parameter("world_center", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "world_center is empty");
        if (rawData.size() != 3)
            RCLCPP_ERROR(get_logger(), "world_center size error");

        return {
                    static_cast<float>(rawData[0]),
                    static_cast<float>(rawData[1]),
                    static_cast<float>(rawData[2])
                };
    };

    Vec2 image_target() const {
        vector<double> rawData;

        get_parameter("image_target", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "image_target is empty");
        if (rawData.size() != 2)
            RCLCPP_ERROR(get_logger(), "image_target size error");

        return {
                    static_cast<float>(rawData[0]),
                    static_cast<float>(rawData[1])
                };
    };

    Vec2 image_center() const {
        vector<double> rawData;

        get_parameter("image_center", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "image_center is empty");
        if (rawData.size() != 2)
            RCLCPP_ERROR(get_logger(), "image_center size error");

        return {
                    static_cast<float>(rawData[0]),
                    static_cast<float>(rawData[1])
                };
    };

    int history_size() const {
        int size;
        get_parameter("history_size", size);
        return size;
    };

    Feature source_feature() {
        if (sourceFeature != nullptr) { return *sourceFeature; }
        // load
        string source_image;
        get_parameter("source_image", source_image);
        if (source_image == "null")
            RCLCPP_ERROR(get_logger(), "source_image unset");
        RCLCPP_INFO(get_logger(), "source_image: %s", source_image.c_str());
        Image image = cv::imread(source_image);
        if (image.empty())
            RCLCPP_ERROR(get_logger(), "source_image is empty");
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
        sourceFeature = std::make_shared<Feature>(Feature::of(image));
        return *sourceFeature;
    };

   Matx33f           cameraMatrix;
   Matx<float, 1, 5> distCoeffs;

    Cache<double> distances{history_size()};
    Vec2 lastArm{1,0};
    double lastTime{0};

    Publisher<PointStamped>::SharedPtr target_publisher =
            create_publisher<PointStamped>(
                "target",
                10
            );
    Publisher<PointStamped>::SharedPtr center_publisher =
            create_publisher<PointStamped>(
                "center",
                10
            );
    Publisher<Float64MultiArray>::SharedPtr angularVelocity_publisher =
                create_publisher<Float64MultiArray>(
                "angular_velocity",
                10
            );

    Subscription<ImageMsg>::SharedPtr image_subscriber =
            create_subscription<ImageMsg>(
                "/image_raw",
                10,
                [this](const ImageMsg::SharedPtr imageRos) -> void {
                    const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(imageRos);
                    if (image->image.empty()) {
                        RCLCPP_WARN(get_logger(), "image is empty");
                        return;
                    }

                    Image grayImage;
                    cvtColor(image->image, grayImage, cv::COLOR_BGR2GRAY);

                    const Feature feature   = Feature::of(grayImage);
                    const Matches matches   = Matches::between(source_feature(), feature);
                    const auto    transform = Transform2D::fit(matches);
                    if (!transform)
                        return;

                    Vec2 target2d = *transform * image_target();
                    Vec2 center2d = *transform * image_center();

                    const double distance = length(target2d - center2d);
                    distances.update(distance);
                    auto arm =   target2d-center2d;
                    center2d  = target2d - arm / distance * distances.avrage();

                    vector<Vec3> worldPoints = world_points();
                    vector<Vec2> imagePoints = *transform * image_points();

                    worldPoints.push_back(world_target());
                    worldPoints.push_back(world_center());
                    imagePoints.push_back(target2d);
                    imagePoints.push_back(center2d);

                    const Transform3D transform3D = Transform3D::fit(
                        worldPoints,
                        imagePoints,
                        cameraMatrix,
                        distCoeffs
                    );

                    Vec3 target = transform3D * world_target();
                    Vec3 center = transform3D * world_center();

                    if (length(target)<5.5 || length(center)<5.5)
                            return;
                    if (length(target)>7.5 || length(center)>7.5)
                            return;

                    PointStamped taget_msg;
                    taget_msg.point.x = target(0);
                    taget_msg.point.y = target(1);
                    taget_msg.point.z = target(2);
                    taget_msg.header  = imageRos->header;
                    target_publisher->publish(taget_msg);

                    PointStamped center_msg;
                    center_msg.point.x = center(0);
                    center_msg.point.y = center(1);
                    center_msg.point.z = center(2);
                    center_msg.header  = imageRos->header;
                    center_publisher->publish(center_msg);

                    double radian = acos(arm.ddot(lastArm) / length(lastArm)/length(arm));
                    // 判断旋转方向
                    if (arm(0)*lastArm(1)-arm(1)*lastArm(0) < 0)
                    radian = -radian;

                    auto time =imageRos->header.stamp.sec+imageRos->header.stamp.nanosec/1e9;
                    radian    = radian/(time-lastTime);

                    lastArm = arm;
                    lastTime = time;

                    if(fabs(radian) < 0.001)
                        return;
                    if(fabs(radian) > 2.5)
                        return;

                    Float64MultiArray angularVelocity_msg;
                    angularVelocity_msg.data.push_back(time);
                    angularVelocity_msg.data.push_back(radian);
                    angularVelocity_publisher->publish(angularVelocity_msg);

                }
            );

public:
    explicit PowerRuneNode(): Node("power_rune") {
        RCLCPP_INFO(get_logger(), "power_rune main start");
    }

    void init() {
        cameraMatrix = cv::Matx33f{1777, 0, 1920 / 2, 0, 1777, 1071 / 2, 0, 0, 1};
        distCoeffs   = cv::Matx<float, 1, 5>{0, 0, 0, 0, 0};
    }
};


int main(const int argc, char** argv) {
    init(argc, argv);
    const auto powerRuneNode = std::make_shared<PowerRuneNode>();
    powerRuneNode->init();
    spin(powerRuneNode);
    printf("hello world power_rune package\n");
    return 0;
}

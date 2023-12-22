#include <cstdio>
#include "Alias.hpp"
#include "Converter/Transform.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;


class PowerRuneNode final : public Node {
    shared_ptr<Feature> sourceFeature;
    std::vector<Vec3>   sourceWorldPoints;
    std::vector<Vec2>   sourceImagePoints;

    cv::Matx33f           cameraMatrix;
    cv::Matx<float, 1, 5> distCoeffs;


    Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr rune_in_camera_publisher;
    Publisher<geometry_msgs::msg::PointStamped>::SharedPtr   center_in_camera_publisher;
    Publisher<geometry_msgs::msg::PointStamped>::SharedPtr   target_in_camera_publisher;
    Publisher<geometry_msgs::msg::PointStamped>::SharedPtr   prediction_in_camera_publisher;
    Subscription<sensor_msgs::msg::Image>::SharedPtr         image_subscriber;

    void main(const sensor_msgs::msg::Image::SharedPtr& imageRos) {
        const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(imageRos);
        if (image->image.empty()) {
            RCLCPP_WARN(get_logger(), "image is empty");
            return;
        }

        Image grayImage;
        cvtColor(image->image, grayImage, cv::COLOR_BGR2GRAY);

        const Feature feature   = Feature::of(grayImage);
        const Matches matches   = Matches::between(*sourceFeature, feature);
        const auto    transform = Transform2D::fit(matches);
        if (!transform)
            return;

        const vector<Vec2> imagePoints = *transform * sourceImagePoints;

        const Transform3D transform3D = Transform3D::fit(
            sourceWorldPoints,
            imagePoints,
            cameraMatrix,
            distCoeffs
        );

         Vec3 center     = {0.0f, 0.0f, 0.0f};
         Vec3 target     = {0.0f, 0.0f, 0.7f};
         Vec3 prediction = {0.0f, 0.3f, 0.63f};

        center = transform3D*center;
        target = transform3D*target;
        prediction = transform3D*prediction;


        // 测试输出
        Image showImage = image->image.clone();
        drawKeypoints(showImage, feature.keyPoints, showImage);
        for (auto&& imagePoint: imagePoints)
            circle(
                showImage,
                {static_cast<int>(imagePoint(0)), static_cast<int>(imagePoint(1))},
                5,
                cv::Scalar(255, 255, 0),
                -1
            );
        imshow("showImage", showImage);
        cv::waitKey(1);

        geometry_msgs::msg::PolygonStamped polygon_msg;
        for (const Vec3& point: sourceWorldPoints) {
            Vec3                        result = transform3D * point;
            geometry_msgs::msg::Point32 point_msg;
            point_msg.x = result(0);
            point_msg.y = result(1);
            point_msg.z = result(2);
            polygon_msg.polygon.points.push_back(point_msg);
        }
        polygon_msg.header = imageRos->header;
        rune_in_camera_publisher->publish(polygon_msg);

        geometry_msgs::msg::PointStamped center_msg;
        center_msg.point.x =center(0);
        center_msg.point.y =center(1);
        center_msg.point.z =center(2);
        center_msg.header  = imageRos->header;
        center_in_camera_publisher->publish(center_msg);

        geometry_msgs::msg::PointStamped taget_msg;
        taget_msg.point.x = target(0);
        taget_msg.point.y = target(1);
        taget_msg.point.z = target(2);
        taget_msg.header  = imageRos->header;
        target_in_camera_publisher->publish(taget_msg);

        geometry_msgs::msg::PointStamped prediction_msg;
        prediction_msg.point.x =prediction(0);
        prediction_msg.point.y =prediction(1);
        prediction_msg.point.z =prediction(2);
        prediction_msg.header  = imageRos->header;
        prediction_in_camera_publisher->publish(prediction_msg);
    }

public:
    explicit PowerRuneNode(): Node("power_rune") {

        RCLCPP_INFO(get_logger(), "power_rune main start");
        image_subscriber = create_subscription<sensor_msgs::msg::Image>(
            "map",
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr imageRos) -> void { main(imageRos); }
        );

        rune_in_camera_publisher = create_publisher<geometry_msgs::msg::PolygonStamped>(
            "rune_in_camera",
            10
        );
        center_in_camera_publisher = create_publisher<geometry_msgs::msg::PointStamped>(
            "center_in_camera",
            10
        );
        target_in_camera_publisher = create_publisher<geometry_msgs::msg::PointStamped>(
            "target_in_camera",
            10
        );
        prediction_in_camera_publisher = create_publisher<geometry_msgs::msg::PointStamped>(
            "prediction_in_camera",
            10
        );
        declare_parameter()
    }

    void init() {
        cameraMatrix = cv::Matx33f{1777, 0, 1920 / 2, 0, 1777, 1071 / 2, 0, 0, 1};
        distCoeffs   = cv::Matx<float, 1, 5>{0, 0, 0, 0, 0};

        Image image = cv::imread(
            "/mnt/c/Projects/C/Algorithm-24/src/power_rune/temp/屏幕截图 2023-12-19 212720.png"
        );
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
        sourceFeature = std::make_shared<Feature>(Feature::of(image));
        Vec2 imageSize{static_cast<float>(image.cols), static_cast<float>(image.rows)};



        sourceImagePoints = {
                    {imageSize(0) * 0.54f, imageSize(1) * 0.18f},
                    {imageSize(0) * 0.25f, imageSize(1) * 0.6f},
                    {imageSize(0) * 0.25f, imageSize(1) * 0.85f},
                    {imageSize(0) * 0.5f, imageSize(1) * 0.73f},
                    {imageSize(0) * 0.75f, imageSize(1) * 0.85f},
                    {imageSize(0) * 0.75f, imageSize(1) * 0.6f}
                };
        sourceWorldPoints = {
                    {0.0f, 0.0f, 0.0f},
                    {0.0f, 0.16f, 0.54f},
                    {0.0f, 0.16f, 0.86f},
                    {0.0f, 0.0f, 0.7f},
                    {0.0f, -0.16f, 0.86f},
                    {0.0f, -0.16f, 0.54f}
                };
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

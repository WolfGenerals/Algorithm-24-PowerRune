#include "Converter/ConvertTo3D.hpp"
#include "RuneRecognizer/Feature.hpp"
#include "RuneRecognizer/Matches.hpp"
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

int main() {
    auto         rune = imread("屏幕截图 2023-12-19 212720.png");
    Mat          gray;
    Vec2         imageSize{static_cast<float>(rune.cols), static_cast<float>(rune.rows)};
    vector<Vec2> imagePoints{
            {imageSize(0) * 0.54f, imageSize(1) * 0.18f},
            {imageSize(0) * 0.5f, imageSize(1) * 0.73f},
            {imageSize(0) * 0.25f, imageSize(1) * 0.85f},
            {imageSize(0) * 0.75f, imageSize(1) * 0.85f},
            {imageSize(0) * 0.25f, imageSize(1) * 0.6f},
            {imageSize(0) * 0.75f, imageSize(1) * 0.6f}};
    vector<Vec3> worldPoints{
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.7f},
            {0.0f, 0.16f, 0.86f},
            {0.0f, -0.16f, 0.86f},
            {0.0f, 0.16f, 0.54f},
            {0.0f, -0.16f, 0.54f}};

    ConvertTo3D convertTo3D{worldPoints, imagePoints, Matx33f::eye(), Matx<float, 1, 5>::zeros()};


    cvtColor(rune, gray, COLOR_BGR2GRAY);
    Feature featureRune = Feature::of(gray);

    for (auto point: imagePoints) {
        circle(rune, {static_cast<int>(point(0)), static_cast<int>(point(1))}, 5, Scalar(255, 0, 255), 2);
    }

    imshow("ot", rune);
    VideoCapture capture{"关灯-红方大能量机关-失败后激活成功的全激活过程.MP4"};

    while (true) {
        Mat frame;
        capture >> frame;

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        Feature featureFrame = Feature::of(gray);

        Matches                    matches   = Matches::between(featureRune, featureFrame);
        std::optional<Transform2D> transform = matches.transform();
        if (!transform) {
            // cout << "no transform" << endl;
            continue;
        }

        vector untransformedPoints{imagePoints};

        perspectiveTransform(untransformedPoints, untransformedPoints, transform.value());
        for (auto point: untransformedPoints) {
            circle(frame, {static_cast<int>(point(0)), static_cast<int>(point(1))}, 5, Scalar(255, 0, 255), 2);
        }

        auto converter = convertTo3D.converter(transform.value());
        for (auto worldPoint: worldPoints) {
            Vec3 point = converter(worldPoint);
            std::cout << point << std::endl;
        }
        cout << endl;
        imshow("out", frame);
        if (waitKey(100) == 'q')
        break;
    }

}

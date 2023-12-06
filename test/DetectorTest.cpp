#include "Alias.hpp"
#include "Arm.hpp"
#include "FeatureMatch.hpp"
#include "ImageDetail.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main() {
    auto capture = VideoCapture("resources/temp/关灯-红方大能量机关-失败后激活成功的全激活过程.MP4");
    // auto capture = VideoCapture("resources/temp/关灯-蓝方小能量机关-反复失败后激活成功的全激活过程.MP4");

    Image armImage            = imread("resources/Arm.png");
    armImage                  = detailOf(armImage);
    auto          standardArm = Arm(Point(armImage.cols / 2, armImage.rows * 2 / 3),
                                    Point(armImage.cols / 2, armImage.rows / 3));
    const Feature armFeature  = Feature::of(armImage);

    while (capture.isOpened()) {

        Mat image;
        capture >> image;
        clock_t start = clock();

        cvtColor(image, image, COLOR_BGR2GRAY);
        image = image(Range(0, image.rows), Range(image.cols / 5, image.cols / 5 * 4));


        Image   lights  = detailOf(image);
        Feature feature = Feature::of(lights);
        Matches matches = Matches::between(armFeature, feature);
        Arm     arm     = standardArm.transform(matches.transform());

        clock_t end = clock();
        cout << 1 / static_cast<double>(end - start) * CLOCKS_PER_SEC << "帧" << endl;

        // imshow("feature", feature.show());
        // imshow("matches", matches.show());
        imshow("Arm Out", arm.showOn(lights));
        // imshow("Std Arm", standardArm.showOn(armFeature.show()));

        if (waitKey(1) == 'q') return 0;

    }
}

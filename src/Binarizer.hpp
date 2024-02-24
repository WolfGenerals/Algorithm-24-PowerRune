#ifndef BINARIZER_HPP
#define BINARIZER_HPP

#include "opencv2/opencv.hpp"

#include "Configuration.hpp"


class Binarizer {
public:
    const Configuration& config;

    cv::Mat binary(const cv::Mat& image) {
        cv::Mat gray;
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::Mat binary = gray > config.二值化阈值();
        //开闭操作
        morphologyEx(binary, binary, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
        // morphologyEx(binary, binary, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        morphologyEx(binary, binary, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        return binary;
    }
};


#endif //BINARIZER_HPP

// vision/preprocess.cpp
#include "vision/preprocess.h"

#include <vector>

#include <opencv2/imgproc.hpp>

cv::Mat makeColorMask(const cv::Mat& src, TargetColor color,
                      const PreprocessConfig& config) {
    if (src.empty()) {
        return {};
    }

    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    cv::Mat hsv_mask;
    if (color == TargetColor::Red) {
        cv::Mat low;
        cv::Mat high;
        cv::inRange(hsv, cv::Scalar(0, config.saturation_min, config.value_min),
                    cv::Scalar(10, 255, 255), low);
        cv::inRange(hsv, cv::Scalar(170, config.saturation_min, config.value_min),
                    cv::Scalar(180, 255, 255), high);
        cv::bitwise_or(low, high, hsv_mask);
    } else {
        cv::inRange(hsv, cv::Scalar(100, config.saturation_min, config.value_min),
                    cv::Scalar(124, 255, 255), hsv_mask);
    }

    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    cv::Mat color_difference;
    if (color == TargetColor::Red) {
        cv::subtract(channels[2], channels[0], color_difference);
    } else {
        cv::subtract(channels[0], channels[2], color_difference);
    }

    cv::Mat color_mask;
    cv::threshold(color_difference, color_mask,
                  config.color_difference_min, 255, cv::THRESH_BINARY);
    cv::bitwise_and(hsv_mask, color_mask, hsv_mask);

    const cv::Size kernel_size(config.morphology_kernel,
                               config.morphology_kernel);
    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_RECT, kernel_size);
    cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_CLOSE, kernel);

    return hsv_mask;
}

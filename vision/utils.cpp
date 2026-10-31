// vision/utils.cpp
#include "vision/utils.h"

#include <cmath>

#include <opencv2/core.hpp>

namespace utils {

double angleFromVertical(const cv::RotatedRect& rect) {
    // 长边相对竖直方向的夹角，单位度，范围 (-90, 90]。
    //
    // 为什么不直接用 rect.angle：cv::RotatedRect 自带的 angle 语义在
    // OpenCV 版本之间变过（长边/短边、顺时针/逆时针都改过），
    // 这里一律从四个顶点重算，跟版本无关。
    cv::Point2f points[4];
    rect.points(points);
    cv::Point2f long_edge = points[1] - points[0];
    if (cv::norm(points[2] - points[1]) > cv::norm(long_edge)) {
        long_edge = points[2] - points[1];
    }
    double axis_angle = std::atan2(long_edge.y, long_edge.x) * 180.0 / CV_PI;
    while (axis_angle < 0.0) {
        axis_angle += 180.0;
    }
    while (axis_angle >= 180.0) {
        axis_angle -= 180.0;
    }
    return axis_angle - 90.0;
}

double wrapPi(double angle) {
    while (angle > CV_PI) {
        angle -= 2.0 * CV_PI;
    }
    while (angle <= -CV_PI) {
        angle += 2.0 * CV_PI;
    }
    return angle;
}

}  // namespace utils

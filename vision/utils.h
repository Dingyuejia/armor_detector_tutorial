// vision/utils.h
// 两个与检测逻辑无关的小工具。绘制/文件遍历之类的自己按需加。
#ifndef ARMOR_VISION_UTILS_H
#define ARMOR_VISION_UTILS_H

#include <opencv2/core.hpp>

namespace utils {

// 长边相对竖直方向的夹角，单位为度，范围 (-90, 90]。
// 注意 cv::RotatedRect 自带的 angle 语义随 OpenCV 版本变化过，
// 这里一律从四个顶点重算，避免踩这个坑。
double angleFromVertical(const cv::RotatedRect& rect);

// 把角度归一化到 (-pi, pi]。跨 ±180° 的所有比较都要先过它。
double wrapPi(double angle);

}  // namespace utils

#endif  // ARMOR_VISION_UTILS_H

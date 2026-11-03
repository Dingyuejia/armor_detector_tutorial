// vision/preprocess.cpp
#include "vision/preprocess.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

cv::Mat makeColorMask(const cv::Mat &src, TargetColor color,
                      const PreprocessConfig &config) {
// 把 BGR 图变成"灯条是白的、其它全是黑的"单通道 mask。
  cv::Mat hsv;
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  cv::Mat hsv_mask;
  
  // 参数在 vision/config.h 的 PreprocessConfig 里：
  //   saturation_min / value_min       HSV 的 S/V 下限
  //   color_difference_min             对手色通道差的下限（红算 R-B，蓝算 B-R）
  //   morphology_kernel                形态学核边长（必须奇数）
  //
  // 输出要求：CV_8UC1，尺寸和 src 一致。detector.cpp 会直接拿它去
  // findContours，所以不能有噪声点以外的"软边"。
if (color == TargetColor::Red) {
    cv::Mat low, high;
    cv::inRange(hsv, cv::Scalar(0, config.saturation_min, config.value_min),
                      cv::Scalar(10, 255, 255), low);
    cv::inRange(hsv, cv::Scalar(170, config.saturation_min, config.value_min),
                      cv::Scalar(180, 255, 255), high);
    cv::bitwise_or(low, high, hsv_mask);
} else {
    cv::inRange(hsv, cv::Scalar(100, config.saturation_min, config.value_min),
                      cv::Scalar(124, 255, 255), hsv_mask);
}
// 通道差：补过曝白洞
std::vector<cv::Mat> ch;
cv::split(src, ch); // ch[0]=B, ch[1]=G, ch[2]=R
cv::Mat diff;
if (color == TargetColor::Red) {
    cv::absdiff(ch[2], ch[0], diff); // R - B
} else {
    cv::absdiff(ch[0], ch[2], diff); // B - R
}
cv::Mat diff_mask;
cv::threshold(diff, diff_mask, config.color_difference_min, 255, cv::THRESH_BINARY);
// 合并 HSV mask 和通道差 mask
cv::bitwise_or(hsv_mask, diff_mask, hsv_mask);
  
// 形态学
int k = config.morphology_kernel;
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_CLOSE, kernel);  // 补洞
cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_OPEN, kernel);   // 去噪


  return hsv_mask;
}

// vision/preprocess.cpp
#include "vision/preprocess.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

cv::Mat makeColorMask(const cv::Mat &src, TargetColor color,
                      const PreprocessConfig &config) {
  // 把 BGR 图变成"灯条是白的、其它全是黑的"单通道 mask。
  //
  // 参数在 vision/config.h 的 PreprocessConfig 里：
  //   saturation_min / value_min       HSV 的 S/V 下限
  //   color_difference_min             对手色通道差的下限（红算 R-B，蓝算 B-R）
  //   morphology_kernel                形态学核边长（必须奇数）
  //
  // 输出要求：CV_8UC1，尺寸和 src 一致。detector.cpp 会直接拿它去
  // findContours，所以不能有噪声点以外的"软边"。
  (void)src;
  (void)color;
  (void)config;
  return cv::Mat();
}

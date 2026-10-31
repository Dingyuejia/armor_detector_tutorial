// reference/vision_ref.h
//
// 参考实现自带的一份检测链路。
//
// 为什么它不直接调用 vision/ 下的同名函数：那些是**学生的空位**
// （makeColorMask / findLightBars / matchArmors 都还返回空）。
// 如果参考实现也走那一套，`armor_aim_ref` 就什么都检不到，
// 这个"参照点"也就没意义了。
//
// 所以这里放一份自包含的、能跑的检测：形状上和 vision/ 的一致，
// 但完全独立。学生写自己的检测器时可以参考，也可以完全不理它。
#ifndef ARMOR_REFERENCE_VISION_REF_H
#define ARMOR_REFERENCE_VISION_REF_H

#include <vector>

#include <opencv2/core.hpp>

#include "vision/armor.h"
#include "vision/config.h"
#include "vision/detector.h"
#include "vision/light_bar.h"

namespace ref {

cv::Mat makeColorMask(const cv::Mat& src, TargetColor color,
                      const PreprocessConfig& config);

std::vector<LightBar> findLightBars(const cv::Mat& binary,
                                    const LightBarConfig& config);

std::vector<Armor> matchArmors(const std::vector<LightBar>& bars,
                               const ArmorConfig& config);

// 三段串起来 + 计时，和 ArmorDetector::detect 的职责一致。
Detection detect(const cv::Mat& frame, const DetectorConfig& config);

}  // namespace ref

#endif  // ARMOR_REFERENCE_VISION_REF_H

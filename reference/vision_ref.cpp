// reference/vision_ref.cpp
// 参考实现自带的一份检测链路，和 vision/ 下学生的空位完全独立。
#include "reference/vision_ref.h"

#include <algorithm>
#include <cstdint>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace ref {

cv::Mat makeColorMask(const cv::Mat &src, TargetColor color,
                      const PreprocessConfig &config) {
  CV_Assert(!src.empty());
  CV_Assert(src.type() == CV_8UC3);

  cv::Mat hsv;
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

  cv::Mat hue_mask;
  if (color == TargetColor::Red) {
    // 红色在 OpenCV 的 Hue（0..180）里跨了首尾，必须两段取并集。
    cv::Mat low_red;
    cv::Mat high_red;
    cv::inRange(hsv, cv::Scalar(0, config.saturation_min, config.value_min),
                cv::Scalar(10, 255, 255), low_red);
    cv::inRange(hsv, cv::Scalar(170, config.saturation_min, config.value_min),
                cv::Scalar(180, 255, 255), high_red);
    cv::bitwise_or(low_red, high_red, hue_mask);
  } else {
    cv::inRange(hsv, cv::Scalar(90, config.saturation_min, config.value_min),
                cv::Scalar(135, 255, 255), hue_mask);
  }

  // 使用对手色通道差抑制灰白背景：红色计算 R-B，蓝色计算 B-R。
  // 不减去 G 通道，因为过曝的蓝灯常呈青白色，G 通道也会很高。
  cv::Mat target_channel;
  cv::Mat opponent_channel;
  cv::Mat color_difference;
  const int target_index = color == TargetColor::Red ? 2 : 0;
  const int opponent_index = color == TargetColor::Red ? 0 : 2;
  cv::extractChannel(src, target_channel, target_index);
  cv::extractChannel(src, opponent_channel, opponent_index);
  cv::subtract(target_channel, opponent_channel, color_difference);
  cv::threshold(color_difference, color_difference, config.color_difference_min,
                255, cv::THRESH_BINARY);

  cv::Mat mask;
  cv::bitwise_and(hue_mask, color_difference, mask);

  // 形态学核必须是奇数，否则 getStructuringElement 会静默取整。
  int kernel_size = config.morphology_kernel;
  kernel_size = std::max(1, kernel_size);
  if (kernel_size % 2 == 0) {
    ++kernel_size;
  }
  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));

  // 【先闭后开】顺序不能反过来。
  //
  // 灯条在曝光下会过曝成纯白：饱和度和通道差都掉到 0，于是 mask 里只剩
  // 灯条两侧各一两条像素宽的彩色边，中间被“挖空”。这时候如果先做开运算，
  // 那两条细边会被直接抹掉，整根灯条就消失了——而它的“症状”是
  // “画面里明明有灯条，却一根都没检出”。
  //
  // 先闭运算把中间的白洞补上，再用开运算去噪点，顺序才对。
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  return mask;
}

std::vector<LightBar> findLightBars(const cv::Mat& binary,
                                    const LightBarConfig& config) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> bars;
    bars.reserve(contours.size());
    for (const auto& contour : contours) {
        LightBar bar(cv::minAreaRect(contour));
        if (bar.isValid(config)) {
            bars.push_back(bar);
        }
    }

    // 按 x 排序，后面配对时只需要看右侧的灯条，省一半比较。
    std::sort(bars.begin(), bars.end(),
              [](const LightBar& a, const LightBar& b) {
                  return a.center.x < b.center.x;
              });
    return bars;
}

std::vector<Armor> matchArmors(const std::vector<LightBar>& bars,
                               const ArmorConfig& config) {
    std::vector<Armor> armors;
    for (std::size_t i = 0; i < bars.size(); ++i) {
        for (std::size_t j = i + 1; j < bars.size(); ++j) {
            Armor armor(bars[i], bars[j], config);
            if (armor.isValid()) {
                armors.push_back(armor);
            }
        }
    }
    std::sort(armors.begin(), armors.end(),
              [](const Armor& a, const Armor& b) {
                  return a.score() > b.score();
              });
    return armors;
}

Detection detect(const cv::Mat& frame, const DetectorConfig& config) {
    const std::int64_t begin = cv::getTickCount();
    Detection result;
    result.mask = ref::makeColorMask(frame, config.target_color, config.preprocess);
    result.light_bars = ref::findLightBars(result.mask, config.light_bar);
    result.armors = ref::matchArmors(result.light_bars, config.armor);
    result.detect_ms = static_cast<double>(cv::getTickCount() - begin) * 1000.0 /
                       cv::getTickFrequency();
    return result;
}

}  // namespace ref

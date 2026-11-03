// vision/light_bar.cpp
#include "vision/light_bar.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "vision/utils.h"

// LightBar 的构造和取值都保留：它们定义"一个灯条要怎么读"，
// 尤其是 minAreaRect 的角度约定（见 utils::angleFromVertical）。
LightBar::LightBar(const cv::RotatedRect& r) : rect(r) {
    length = std::max(r.size.height, r.size.width);
    width = std::min(r.size.height, r.size.width);
    angle = utils::angleFromVertical(r);
    center = r.center;
}

double LightBar::aspectRatio() const {
    return width > 1e-6 ? length / width : 0.0;
}

bool LightBar::isValid(const LightBarConfig& config) const {
    // 注意边界：这里用严格不等号，是行为约定的一部分。
    // 改成 <= 会让刚好等于阈值的轮廓行为不同。
    //
    // 阈值都在 LightBarConfig 里。这一层是检测的"物理边界"所在：
    // 灯条在远处只有几个像素宽，min_area 定死就会有一个
    // "再远一个都检不到"的悬崖。
    const double area = rect.size.area();
    const double ratio = aspectRatio();
    const bool area_ok = config.min_area < area && area < config.max_area;
    const bool ratio_ok = config.min_aspect_ratio < ratio &&
                          ratio < config.max_aspect_ratio;
    const bool angle_ok = std::abs(angle) < config.max_tilt_deg;
    return area_ok && ratio_ok && angle_ok;
}

std::vector<LightBar> findLightBars(const cv::Mat& binary,
                                    const LightBarConfig& config) {
    // 从 mask 里把每根灯条找出来：轮廓 -> 最小外接旋转矩形 -> isValid 过滤。
    //
    
    std::vector<LightBar> bars;
    if (binary.empty()) {
        return bars;
    }

    // findContours 可能修改输入，所以对 mask 的副本操作。
    cv::Mat work = binary.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(work, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        const cv::RotatedRect rect = cv::minAreaRect(contour);
        LightBar bar(rect);

        if (bar.isValid(config)) {
            bars.push_back(bar);
        }
    }
    // 输出应当按 x 从小到大排好序。为什么要排序：配对的时候只需要看
    // 右边的候选，而且顺序稳定才能让"第 i 个候选"在两帧之间指同一根灯条。
    std::stable_sort(bars.begin(), bars.end(),
                     [](const LightBar& lhs, const LightBar& rhs) {
                         return lhs.center.x < rhs.center.x;
                     });
    return bars;
}

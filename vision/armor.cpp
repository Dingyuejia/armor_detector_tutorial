// vision/armor.cpp
#include "vision/armor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

Armor::Armor(const LightBar& left, const LightBar& right,
             const ArmorConfig& config)
    : left_(left), right_(right) {
    // 两块灯条是否构成一块装甲板，以及给这个候选打多少分。
    //
    // 阈值在 ArmorConfig 里，全部是**归一化**过的量，所以换分辨率、
    // 换距离都不用重调：
    //   angle_diff        两根灯条的倾角差（度）
    //   length_ratio      两根灯条长度的比值
    //   y_offset_ratio    上下端面的高度差 / 平均灯条长度
    //   x_offset_ratio    两根灯条的间距 / 平均灯条长度
    //
    // 不构成一块板时把 score_ 留成 0（isValid() 会判掉，于是它不会
    // 进入 matchArmors 的输出）。
    const double average_length = (left.length + right.length) / 2.0;
    const double angle_diff = std::abs(left.angle - right.angle);
    const double length_ratio =
        std::max(left.length, right.length) /
        std::max(1.0, std::min(left.length, right.length));
    const double y_offset_ratio =
        std::abs(left.center.y - right.center.y) / average_length;
    const double x_offset_ratio =
        std::abs(left.center.x - right.center.x) / average_length;

    const bool geometry_ok =
        angle_diff <= config.max_angle_diff_deg &&
        length_ratio <= config.max_length_ratio &&
        y_offset_ratio <= config.max_y_offset_ratio &&
        x_offset_ratio >= config.min_x_offset_ratio &&
        x_offset_ratio <= config.max_x_offset_ratio;
    if (!geometry_ok) {
        score_ = 0.0;
        return;
    }

    const double angle_score = 1.0 - angle_diff / config.max_angle_diff_deg;
    const double length_score =
        1.0 - (length_ratio - 1.0) / (config.max_length_ratio - 1.0);
    const double y_score = 1.0 - y_offset_ratio / config.max_y_offset_ratio;
    score_ = std::clamp(0.35 * angle_score + 0.35 * length_score +
                            0.30 * y_score,
                        0.0, 1.0);
    if (score_ < config.min_score) {
        score_ = 0.0;
    }
}

cv::Point2f Armor::center() const {
    return (left_.center + right_.center) * 0.5F;
}

cv::Rect Armor::boundingBox() const {
    const cv::Rect l = left_.rect.boundingRect();
    const cv::Rect r = right_.rect.boundingRect();
    return l | r;
}

bool Armor::isValid() const {
    return score_ > 0.0;
}

std::vector<Armor> matchArmors(const std::vector<LightBar>& bars,
                               const ArmorConfig& config) {
    // 把所有灯条两两配对，留下能构成装甲板的（Armor 的构造里已经判好了
    // 几何是否成立，你只需要枚举 + 过滤 + 排序）。
    //
    // 输出按分数从高到低排序。排序顺序是接口的一部分：下游（pipeline）
    // 拿到的 candidate 下标要稳定，否则"第 0 个候选"每帧指不同的板，
    // 你的跟踪器会疯。
        std::vector<Armor> armors;

    // 枚举每一对不同的灯条。i < j 可避免重复配对和灯条与自身配对。
    for (std::size_t i = 0; i < bars.size(); ++i) {
        for (std::size_t j = i + 1; j < bars.size(); ++j) {
            Armor armor(bars[i], bars[j], config);

            // 构造函数会检查几何关系并计算分数。
            if (armor.isValid()) {
                armors.push_back(armor);
            }
        }
    }

    // 分数高的候选排前面；stable_sort 让同分候选维持原有顺序。
    std::stable_sort(
        armors.begin(), armors.end(),
        [](const Armor& lhs, const Armor& rhs) {
            return lhs.score() > rhs.score();
        });

    return armors;

}

// tests/core_tests.cpp
// 两组测试，锁定两类完全不同的东西：
//
//   core          角度符号约定 + 绕回。这是**一直应该绿**的。
//   ideal_samples 4 张实拍正例的检测框 IoU ≥ 0.65。
//                 开局是红的——它就是你检测器的验收标准。
//
// 这两组测试只锁"行为"，不锁"实现"。你可以随便重写内部代码，只要测试还绿。
// 而一旦你把角度符号改反，core 会立刻红——这是它存在的唯一理由。
//
// 想加自己的回归用例（合成图、录下来的帧、装订线边界……）就加在这里。
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "core/types.h"
#include "vision/angles.h"
#include "vision/detector.h"
#include "vision/utils.h"

namespace {

namespace fs = std::filesystem;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool nearly(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}

void testAngleConvention() {
    // 像素向右 -> yaw 为负（云台要向右转 = yaw 减小）
    const RelativeAngle right = relativeAngleFromPixel({420.0f, 240.0f}, 554.256,
                                                       554.256, 320.0, 240.0);
    require(right.yaw < 0.0, "像素在右侧时 yaw 必须为负");
    require(nearly(right.pitch, 0.0), "同一水平线上 pitch 必须为 0");

    // 像素向下 -> pitch 为正（pitch 约定向下为正）
    const RelativeAngle below = relativeAngleFromPixel({320.0f, 340.0f}, 554.256,
                                                       554.256, 320.0, 240.0);
    require(below.pitch > 0.0, "像素在下方时 pitch 必须为正");
    require(nearly(below.yaw, 0.0), "同一竖线上 yaw 必须为 0");

    // 光轴中心 -> 零角
    const RelativeAngle center = relativeAngleFromPixel({320.0f, 240.0f}, 554.256,
                                                        554.256, 320.0, 240.0);
    require(nearly(center.yaw, 0.0) && nearly(center.pitch, 0.0), "光轴中心必须是零角");

    // 100 像素 / fx 的反正切，量级要合理
    const double expected = std::atan2(100.0, 554.256);
    require(std::abs(std::abs(right.yaw) - expected) < 1e-9,
            "yaw 的量级应等于 atan2(dx, fx)");
}

void testWrap() {
    require(std::abs(utils::wrapPi(3.0 * CV_PI) - CV_PI) < 1e-9,
            "3pi 应绕回到 pi");
    require(std::abs(utils::wrapPi(-3.0 * CV_PI) - CV_PI) < 1e-9,
            "-3pi 应绕回到 pi");
    require(nearly(utils::wrapPi(0.3), 0.3), "小角度不应被改动");
}

double intersectionOverUnion(const cv::Rect& lhs, const cv::Rect& rhs) {
    const cv::Rect intersection = lhs & rhs;
    const double intersection_area = intersection.area();
    return intersection_area / (lhs.area() + rhs.area() - intersection_area + 1e-9);
}

void testIdealSamples() {
    // 四张实拍正例 + 人工标注框。要求：标注框对应的候选出现在结果里，
    // 且 IoU ≥ 0.65。
    //
    // 这只检验"检测器能不能在一个正常画面里找到那块板"，不检验
    // "从多个候选里挑对哪一个"——后者是 pipeline 的事。
    struct Sample {
        const char* filename;
        TargetColor color;
        cv::Rect truth;
    };
    const Sample samples[] = {
        {"ideal_blue_near.jpg", TargetColor::Blue, cv::Rect(105, 412, 919, 265)},
        {"ideal_blue_robot.jpg", TargetColor::Blue, cv::Rect(226, 229, 172, 70)},
        {"ideal_red_robot.jpg", TargetColor::Red, cv::Rect(180, 431, 171, 94)},
        {"ideal_red_front.jpg", TargetColor::Red, cv::Rect(419, 357, 494, 167)},
    };

    for (const Sample& sample : samples) {
        const fs::path path =
            fs::path(ARMOR_PROJECT_DIR) / "assets" / "ideal" / sample.filename;
        const cv::Mat image = cv::imread(path.string());
        require(!image.empty(), "读不到正例图片");

        DetectorConfig config;
        config.target_color = sample.color;
        const ArmorDetector detector(config);
        const Detection detection = detector.detect(image);

        require(!detection.armors.empty(),
                "正例应该至少产生一个候选（检测器还没实现？）");
        double best_iou = 0.0;
        for (const Armor& armor : detection.armors) {
            best_iou = std::max(best_iou,
                                intersectionOverUnion(armor.boundingBox(), sample.truth));
        }
        require(best_iou >= 0.65, "没有任何候选框与标注框的 IoU 达到 0.65");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "用法: armor_tests <core|ideal_samples>\n";
        return EXIT_FAILURE;
    }
    const std::string name = argv[1];
    try {
        if (name == "core") {
            testAngleConvention();
            testWrap();
        } else if (name == "ideal_samples") {
            testIdealSamples();
        } else {
            throw std::invalid_argument("未知测试: " + name);
        }
        std::cout << "PASS: " << name << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}

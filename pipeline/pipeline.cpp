// pipeline/pipeline.cpp
//
// 这个文件现在是空的：三个函数各自返回空结构体。你要做的是把那条链路填满。
//
// 建议的推进顺序（每一步都能单独验收，别一次改完）：
//
//   1) 先让它"看得见"：detect() 有输出，estimate() 挑一块板，
//      把像素换成角度填进 yaw/pitch。这时静止靶在 4 m 附近应该能命中。
//   2) 再让它"知道多远"：填 distance，用它算弹道抬升角。
//      这时远处的静止靶应该开始命中，而 4 m 的成绩不能掉。
//   3) 再让它"记得住"：攒历史帧，估速度、做换板判据。
//      这时自转靶的指令应该不再抖。
//   4) 再让它"打得准"：用距离和速度算提前量，把 yaw_rate/pitch_rate
//      填上并接进 Command::yaw_vel/pitch_vel（伺服做前馈用）。
//   5) 最后管"什么时候不该动"：丢帧、暂停、换会话。
//
// 每做一步，用 bench/run.sh 跑对应的场景，记下命中/s。改动前后差多少，
// 你才有资格说"这一步有用"。
#include "pipeline/pipeline.h"
#include <opencv2/calib3d.hpp>
#include <vector>

#include <algorithm>
#include <cmath>

#include "vision/angles.h"
#include "vision/utils.h"
namespace {
constexpr double kFireToleranceRad = 0.6 * CV_PI / 180.0;  // 开火窗口 0.6°
constexpr int    kFireStableFrames  = 3;                   // 连续 3 帧才扣扳机
}  // namespace

Pipeline::Pipeline(PipelineConfig config)
    : config_(config), detector_(config.detector) {}

Detection Pipeline::detect(const cv::Mat& frame) const {
    // 检测器在 vision/detector.cpp，那边也是空的。
    return detector_.detect(frame);
}

TargetEstimate Pipeline::estimate(const Observation& observation,
                                  const Detection& detection) {
    TargetEstimate estimate;
    estimate.stamp = observation.sim_time;

    if (observation.empty() || detection.armors.empty() ||
        observation.fx <= 0.0 || observation.fy <= 0.0) {
        return estimate;
    }

    // 先用评分最高的候选；之后再改进目标选择。
    const int armor_index = 0;
    const Armor& armor = detection.armors[armor_index];

    // armorPoints() 顺序：左上、右上、右下、左下。
    const auto points = armor.armorPoints();
    const std::vector<cv::Point2f> image_points(points.begin(), points.end());

    // 单位：米。先假设这是小装甲板。
    constexpr float width = 0.135F;
    constexpr float height = 0.056F;
    const std::vector<cv::Point3f> object_points{
        {-width / 2, -height / 2, 0.0F},
        { width / 2, -height / 2, 0.0F},
        { width / 2,  height / 2, 0.0F},
        {-width / 2,  height / 2, 0.0F},
    };

    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        observation.fx, 0.0, observation.cx,
        0.0, observation.fy, observation.cy,
        0.0, 0.0, 1.0);

    // 仿真无镜头畸变时可先设为零。
    const cv::Mat distortion = cv::Mat::zeros(5, 1, CV_64F);

    cv::Mat rvec;
    cv::Mat tvec;
    const bool solved = cv::solvePnP(
        object_points, image_points, camera_matrix, distortion,
        rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    if (!solved || tvec.rows != 3 || tvec.cols != 1) {
        return estimate;
    }

    const double x = tvec.at<double>(0);
    const double y = tvec.at<double>(1);
    const double z = tvec.at<double>(2);

    // 先检查目标是否在相机前方。
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(z) || z <= 0.0) {
        return estimate;
    }

    estimate.valid = true;
    estimate.armor_index = armor_index;
    estimate.pixel = armor.center();

    // PnP 的平移向量是相机到装甲板中心的位置。
    // 这里先填相机到目标的直线距离，后续再换算云台坐标并做弹道补偿。
    estimate.distance = std::sqrt(x * x + y * y + z * z);
    estimate.pixel = armor.center();
    const RelativeAngle relative = relativeAngleFromPixel(
    estimate.pixel,
    observation.fx, observation.fy,
    observation.cx, observation.cy);

    estimate.yaw = observation.gimbal_yaw + relative.yaw;
    estimate.pitch = observation.gimbal_pitch + relative.pitch;


    return estimate;
}

Command Pipeline::command(const Observation& observation,
                          const TargetEstimate& estimate) {
    Command command;

    // 没帧 / 没目标：不发指令，并把稳定计数清零
    if (observation.empty() || !estimate.valid) {
        fire_ready_frames_ = 0;
        return command;
    }

    // 绝对角，左正下正；estimate 里已经是绝对角，直接转发
    command.control = true;
    command.yaw = estimate.yaw;
    command.pitch = estimate.pitch;

    // 前馈角速度，伺服做 goal_v 用
    command.yaw_vel = estimate.yaw_rate;
    command.pitch_vel = estimate.pitch_rate;

    // 开火判据：和【云台实际角】比，不是和指令角比
    const double err_yaw = utils::wrapPi(estimate.yaw - observation.gimbal_yaw);
    const double err_pitch = estimate.pitch - observation.gimbal_pitch;
    const double error = std::hypot(err_yaw, err_pitch);

    if (error <= kFireToleranceRad) ++fire_ready_frames_;
    else                            fire_ready_frames_ = 0;

    command.fire = config_.allow_fire && fire_ready_frames_ >= kFireStableFrames;

    return command;
}

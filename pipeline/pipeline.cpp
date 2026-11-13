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
constexpr double kFireToleranceRad = 1.2 * CV_PI / 180.0;  // 开火窗口（圆）
constexpr int    kFireStableFrames = 2;

// 小装甲板尺寸（米）：PnP 靠它把像素换算成距离
constexpr float kArmorWidth  = 0.135F;
constexpr float kArmorHeight = 0.056F;
// 关联门限：角向紧、深度松（PnP 的深度误差就有 0.1~0.3 m）
constexpr double      kGateAngleRad = 1.2 * CV_PI / 180.0;
constexpr double      kGateDepthM   = 0.30;
constexpr std::size_t kTrackLen     = 4;     // 历史长度
constexpr double      kTrackMaxGap  = 0.30;  // 断帧超过 0.3 s 就当目标丢了

// 一块候选板的解算结果：角度 + 距离 + 身份信息
struct ArmorSolution {
    bool ok = false;          // PnP 成功了吗
    int index = -1;           // 在 detection.armors 里排第几
    double score = 0.0;       // 检测器给的几何评分
    cv::Point2f pixel{};      // 板中心像素
    double yaw = 0.0;
    double pitch = 0.0;
    double distance = 0.0;
};

// 对一块候选板做 PnP，返回绝对角（左正下正）和距离（米）
ArmorSolution solveArmor(const Armor& armor, int index, const Observation& obs) {
    ArmorSolution out;
    out.index = index;
    out.score = armor.score();
    out.pixel = armor.center();

    // armorPoints() 顺序：左上、右上、右下、左下
    // object_points 必须用同样的顺序，否则解出来的姿态是翻的
    const auto points = armor.armorPoints();
    const std::vector<cv::Point2f> image_points(points.begin(), points.end());
    const std::vector<cv::Point3f> object_points{
        {-kArmorWidth / 2, -kArmorHeight / 2, 0.0F},
        { kArmorWidth / 2, -kArmorHeight / 2, 0.0F},
        { kArmorWidth / 2,  kArmorHeight / 2, 0.0F},
        {-kArmorWidth / 2,  kArmorHeight / 2, 0.0F},
    };

    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        obs.fx, 0.0, obs.cx,
        0.0, obs.fy, obs.cy,
        0.0, 0.0, 1.0);
    const cv::Mat distortion = cv::Mat::zeros(5, 1, CV_64F);

    cv::Mat rvec, tvec;
    if (!cv::solvePnP(object_points, image_points, camera_matrix, distortion,
                      rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
        return out;
    }

    const double x = tvec.at<double>(0);
    const double y = tvec.at<double>(1);
    const double z = tvec.at<double>(2);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.0) {
        return out;
    }

    // 角度用像素中心算，比 PnP 的 rvec 稳
    const RelativeAngle rel =
        relativeAngleFromPixel(out.pixel, obs.fx, obs.fy, obs.cx, obs.cy);
    out.yaw = obs.gimbal_yaw + rel.yaw;
    out.pitch = obs.gimbal_pitch + rel.pitch;
    out.distance = std::sqrt(x * x + y * y + z * z);
    out.ok = true;
    return out;
}
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
        track_.clear();
        return estimate;
    }

    if (observation.session_id != last_session_) {
        last_session_ = observation.session_id;
        track_.clear();
        yaw_rate_ = 0.0;
        pitch_rate_ = 0.0;
    }

    // ① 每块候选都解一次
    std::vector<ArmorSolution> cands;
    for (int i = 0; i < static_cast<int>(detection.armors.size()); ++i) {
        ArmorSolution s = solveArmor(detection.armors[i], i, observation);
        if (s.ok) {
            cands.push_back(s);
        }
    }
    if (cands.empty()) {
        track_.clear();
        return estimate;
    }

    // ② 预测
    const double now = observation.sim_time;
    bool have_pred = false;
    double pred_yaw = 0.0;
    double pred_pitch = 0.0;
    if (!track_.empty()) {
        const TrackSample& last = track_.back();
        const double dt = now - last.t;
        if (dt >= 0.0 && dt <= kTrackMaxGap) {
            pred_yaw = last.yaw + yaw_rate_ * dt;
            pred_pitch = last.pitch + pitch_rate_ * dt;
            have_pred = true;
        }
    }

    // ③ 关联
    int best = -1;
    double best_cost = 1e9;
    if (have_pred) {
        for (int i = 0; i < static_cast<int>(cands.size()); ++i) {
            const ArmorSolution& c = cands[i];
            const double dy = std::abs(utils::wrapPi(c.yaw - pred_yaw));
            const double dp = std::abs(c.pitch - pred_pitch);
            const double dd = std::abs(c.distance - track_.back().distance);
            if (dy > kGateAngleRad || dp > kGateAngleRad || dd > kGateDepthM) {
                continue;
            }
            const double cost = dy + dp + 0.5 * dd;
            if (cost < best_cost) {
                best_cost = cost;
                best = i;
            }
        }
    }

    // ④ 没接上：挑分数最高的重开，速度保留
    if (best < 0) {
        int top = 0;
        for (int i = 1; i < static_cast<int>(cands.size()); ++i) {
            if (cands[i].score > cands[top].score) {
                top = i;
            }
        }
        best = top;
        track_.clear();
    }

    const ArmorSolution& sel = cands[best];

    estimate.valid = true;
    estimate.armor_index = sel.index;
    estimate.pixel = sel.pixel;
    estimate.distance = sel.distance;

    // ⑤ yaw 连续化
    double yaw = sel.yaw;
    if (!track_.empty()) {
        yaw = track_.back().yaw + utils::wrapPi(sel.yaw - track_.back().yaw);
    }
    estimate.yaw = yaw;
    estimate.pitch = sel.pitch;

        // ⑥ 记历史（顺便量一下帧间隔，提前量里要用）
    if (!track_.empty()) {
        frame_dt_ = now - track_.back().t;
    }
    TrackSample sample;
    sample.t = now;
    sample.yaw = yaw;
    sample.pitch = sel.pitch;
    sample.distance = sel.distance;
    track_.push_back(sample);
    while (track_.size() > kTrackLen) {
        track_.pop_front();
    }

    // ⑦ 拟合速度
    if (track_.size() >= 4) {
        const double t0 = track_.front().t;
        double n = 0.0, st = 0.0, stt = 0.0;
        double sy = 0.0, sty = 0.0, sp = 0.0, stp = 0.0;
        for (const TrackSample& s : track_) {
            const double dt = s.t - t0;
            n += 1.0;
            st += dt;
            stt += dt * dt;
            sy += s.yaw;
            sty += dt * s.yaw;
            sp += s.pitch;
            stp += dt * s.pitch;
        }
        const double den = n * stt - st * st;
        if (std::abs(den) > 1e-9) {
            yaw_rate_ = (n * sty - st * sy) / den;
            pitch_rate_ = (n * stp - st * sp) / den;
        }
    }

    estimate.yaw_rate = yaw_rate_;
    estimate.pitch_rate = pitch_rate_;

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

    command.control = true;

    // ① 飞行时间：扣扳机到命中有多久
    double flight = 0.0;
    if (estimate.distance > 0.0 && observation.bullet_speed > 0.0) {
        flight = (estimate.distance - observation.barrel_length) /
                     observation.bullet_speed +
                 observation.launch_delay;
        if (flight < 0.0) {
            flight = 0.0;
        }
    }

    // ② 提前量：观测是"过去"的，命中是"将来"的
    const double lead_time = flight + frame_dt_;

    const double aim_yaw = estimate.yaw + estimate.yaw_rate * lead_time;
    double aim_pitch = estimate.pitch + estimate.pitch_rate * lead_time;

    // ③ 弹道抬升：pitch 向下为正，补偿是减
    if (flight > 0.0 && estimate.distance > 0.0) {
        const double drop = 0.5 * observation.gravity * flight * flight;
        aim_pitch -= std::atan2(drop, estimate.distance);
    }

    command.yaw = aim_yaw;
    command.pitch = aim_pitch;

    // 前馈角速度，伺服做 goal_v 用
    command.yaw_vel = estimate.yaw_rate;
    command.pitch_vel = estimate.pitch_rate;

    // ④ 开火判据：炮口是否已经到达"提前量之后的瞄准点"
    const double err_yaw = utils::wrapPi(aim_yaw - observation.gimbal_yaw);
    const double err_pitch = aim_pitch - observation.gimbal_pitch;
    const double error = std::hypot(err_yaw, err_pitch);

    if (error <= kFireToleranceRad) {
        ++fire_ready_frames_;
    } else {
        fire_ready_frames_ = 0;
    }

    command.fire = config_.allow_fire &&
                   fire_ready_frames_ >= kFireStableFrames;

    return command;
}

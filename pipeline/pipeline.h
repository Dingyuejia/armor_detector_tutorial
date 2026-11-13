// pipeline/pipeline.h
// 把检测结果变成一条云台指令。
//
//   所有候选 --(挑一个)--> 选中板 --(解算)--> 目标角 --(开火判据)--> Command
//
// 这条链路上"挑哪一个""怎么算角度""什么时候开火"全部是空的，要你填。
// 先读 core/types.h：Observation 里有标定、云台姿态、弹道参数；
// Command 里有绝对角，也有给伺服做前馈用的 yaw_vel / pitch_vel。
#ifndef ARMOR_PIPELINE_PIPELINE_H
#define ARMOR_PIPELINE_PIPELINE_H

#include <cstddef>
#include <deque>  

#include "core/types.h"
#include "vision/detector.h"

// 当前对目标的估计。下游（控制器/开火判据）只认这个结构体，
// 所以你可以随便换内部算法，只要把字段填对。
//
// 注意 pitch 的约定是【向下为正】、yaw 是【向左为正】，和 Observation 一致。
// 两个角都是**绝对角**：estimate.yaw = observation.gimbal_yaw + 相对角。
struct TargetEstimate {
    bool valid = false;
    int armor_index = -1;      // 选中了 detection.armors 里的哪一个
    cv::Point2f pixel{0.0f, 0.0f};  // 瞄准点（图像坐标）

    double distance = 0.0;     // 到云台旋转中心的距离，单位 m
    double yaw = 0.0;          // 绝对目标角（含云台当前角），向左为正
    double pitch = 0.0;        // 绝对目标角（含云台当前角），向下为正
    double yaw_rate = 0.0;     // 目标角速度，rad/s
    double pitch_rate = 0.0;   // 同上
    double stamp = 0.0;        // 这份估计对应的时间（仿真秒）
};

struct PipelineConfig {
    DetectorConfig detector;
    bool allow_fire = false;
};

class Pipeline {
public:
    explicit Pipeline(PipelineConfig config);

    const ArmorDetector& detector() const { return detector_; }

    // detect 是纯函数；estimate / command 不是——它们要攒历史帧才能估速度，
    // 所以这两个特意不是 const。状态放在成员里就行，但记住“什么时候该清掉”
    // （换会话、目标丢了、仿真重启）也是你要想清楚的一部分。
    Detection detect(const cv::Mat& frame) const;
    TargetEstimate estimate(const Observation& observation,
                            const Detection& detection);
    Command command(const Observation& observation,
                    const TargetEstimate& estimate);

private:
    PipelineConfig config_;
    ArmorDetector detector_;
    int fire_ready_frames_ = 0;
    struct TrackSample {
        double t = 0.0;
        double yaw = 0.0;
        double pitch = 0.0;
        double distance = 0.0;
    };
    std::deque<TrackSample> track_;
    double yaw_rate_ = 0.0;
    double pitch_rate_ = 0.0;
    double frame_dt_ = 0.017;   // 上一帧到这一帧的间隔
    std::string last_session_;

};

#endif  // ARMOR_PIPELINE_PIPELINE_H

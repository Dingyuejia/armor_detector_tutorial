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

#include <algorithm>
#include <cmath>

#include "vision/angles.h"
#include "vision/utils.h"

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

    // 你要填的东西都在 TargetEstimate 里：
    //   valid / armor_index / pixel  —— 挑了哪一块板
    //   distance                     —— 弹道补偿和提前量都要它
    //   yaw / pitch                  —— 绝对角，向下为正、向左为正
    //   yaw_rate / pitch_rate        —— 目标角速度，开火判据和前馈要用
    //
    // 手头有的信息都在 observation 里：像素、标定（fx/fy/cx/cy）、
    // 拍摄这一帧时的云台姿态、云台旋转中心、相机安装偏移、弹道参数。
    (void)observation;
    (void)detection;
    return estimate;
}

Command Pipeline::command(const Observation& observation,
                          const TargetEstimate& estimate) {
    Command command;

    // command.control = false 时仿真会让云台按限加速度停下；
    // fire 会被就地降到 false，所以"不开火"最省事的写法就是返回空 Command。
    //
    // 开火别只看当前帧的角度差：目标的观测可能是几帧之前的，
    // 云台也可能还在追。想清楚"凭什么认为这一发会中"。
    (void)observation;
    (void)estimate;
    return command;
}

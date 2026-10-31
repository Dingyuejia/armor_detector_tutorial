// app/main.cpp
// 主循环。它的职责只有三件事：
//   1) 从采集层拿 Observation
//   2) 交给 pipeline 走一遍（检测 -> 估计 -> 指令）
//   3) 把指令交给采集层，顺便记统计
//
// 主循环里不应该出现任何算法细节。这样换采集源、换算法都不用动它。
//
// 这个文件现在是"接通了但很幼稚"的版本。举两个例子，都属于你要自己判断的事：
//   * 没有新帧时它会一直重发上一条指令（包括上一条里的 fire）—— 真机上这
//     就是"丢了目标还在开火"；
//   * 它不知道仿真重启过，会接着用旧的会话往下发。
// 先把算法做对，再回来处理这些。
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/highgui.hpp>

#include "app/options.h"
#include "io/capture.h"
#include "io/sim_capture.h"
#include "pipeline/pipeline.h"
#include "vision/utils.h"

namespace {

using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_exit = 0;

void onSignal(int) { g_exit = 1; }

constexpr double kRadToDeg = 180.0 / CV_PI;

std::string jsonNumberOrDash(const nlohmann::json& object, const char* key) {
    if (!object.is_object() || !object.contains(key) || !object[key].is_number()) {
        return "-";
    }
    return std::to_string(object[key].get<double>());
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);

        PipelineConfig pipeline_config;
        pipeline_config.detector.target_color = options.color;
        pipeline_config.allow_fire = options.allow_fire;
        Pipeline pipeline(pipeline_config);

        std::unique_ptr<ICapture> capture = std::make_unique<SimCapture>(options.channel);

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        std::cout << "[input] " << capture->describe() << '\n';
        if (capture->controllable() && !options.allow_fire) {
            std::cout << "[note] 未加 --fire，只瞄准不开火（先看角度，再看命中）\n";
        }

        const auto begin = Clock::now();
        auto next_send = begin;

        Command last_command;
        bool have_command = false;

        int frames = 0;
        int frames_with_target = 0;
        double detect_ms_sum = 0.0;
        double gap_sum = 0.0;
        int gap_count = 0;

        while (g_exit == 0) {
            const auto now = Clock::now();
            if (options.duration > 0.0 &&
                std::chrono::duration<double>(now - begin).count() >= options.duration) {
                break;
            }
            if (options.max_frames > 0 && frames >= options.max_frames) {
                break;
            }

            capture->pump();

            Observation observation;
            if (capture->read(observation)) {
                ++frames;
                const Detection detection = pipeline.detect(observation.bgr);
                const TargetEstimate estimate = pipeline.estimate(observation, detection);
                const Command command = pipeline.command(observation, estimate);

                last_command = command;
                have_command = true;

                detect_ms_sum += detection.detect_ms;
                if (estimate.valid) {
                    ++frames_with_target;
                    // 这一项是"指令角 - 云台实际角"，量的是伺服跟不跟得上，
                    // **不是**瞄得准不准。瞄得准不准只有仿真统计（命中）能说。
                    gap_sum += std::hypot(
                        utils::wrapPi(estimate.yaw - observation.gimbal_yaw),
                        estimate.pitch - observation.gimbal_pitch);
                    ++gap_count;
                }

                if (options.show_gui) {
                    cv::imshow("camera", observation.bgr);
                    const int key = cv::waitKey(1);
                    if (key == 'q' || key == 27) {
                        break;
                    }
                }
            }

            // 控制频率和取图频率解耦：50 Hz 下发，中间帧复用上一条指令。
            if (capture->controllable() && now >= next_send) {
                capture->write(have_command ? last_command : Command{});
                next_send += std::chrono::milliseconds(20);
                if (next_send < now) {
                    next_send = now + std::chrono::milliseconds(20);
                }
            }

            if (capture->finished()) {
                break;
            }
            if (capture->controllable()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }

        if (options.show_gui) {
            cv::destroyAllWindows();
        }

        // 这两行是自评接口。bench/report.py 就是解析它们出表的，
        // 改名字/改格式会让评分表空掉。
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "[self] frames=" << frames
                  << " with_target=" << frames_with_target
                  << " target_rate="
                  << (frames > 0 ? static_cast<double>(frames_with_target) / frames : 0.0)
                  << " mean_detect_ms="
                  << (frames > 0 ? detect_ms_sum / frames : 0.0)
                  << " mean_cmd_gap_deg="
                  << (gap_count > 0 ? gap_sum / gap_count * kRadToDeg : 0.0)
                  << '\n';

        if (auto* sim = dynamic_cast<SimCapture*>(capture.get())) {
            const auto& stats = sim->stats();
            std::cout << "[sim] fired=" << jsonNumberOrDash(stats, "fired")
                      << " resolved=" << jsonNumberOrDash(stats, "resolved")
                      << " hit=" << jsonNumberOrDash(stats, "hit")
                      << " hit_rate=" << jsonNumberOrDash(stats, "hit_rate")
                      << " kills=" << jsonNumberOrDash(stats, "kills") << '\n';
            if (!sim->enabled()) {
                std::cout << "[warn] 仿真端没有打开\"本机自瞄控制\"，"
                             "指令会被丢弃。请用 ./run.sh -- --remote 启动。\n";
            }
        }

        return frames > 0 ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}

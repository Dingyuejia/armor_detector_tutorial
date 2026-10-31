// app/options.h
// 命令行选项。校验放在这里，主循环里不要出现 std::stod。
#ifndef ARMOR_APP_OPTIONS_H
#define ARMOR_APP_OPTIONS_H

#include <string>

#include "vision/config.h"

struct Options {
    std::string channel = "default";

    double duration = 0.0; // 秒；0 表示不限
    int max_frames = 0;    // 0 表示不限

    bool show_gui = true;
    bool allow_fire = false;
    TargetColor color = TargetColor::Blue;
};

// 解析失败时抛 std::invalid_argument。
Options parseOptions(int argc, char** argv);

void printUsage(const char* program);

#endif  // ARMOR_APP_OPTIONS_H

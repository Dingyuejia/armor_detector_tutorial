cmake_minimum_required(VERSION 3.16)
project(armor_aim VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- 依赖 ---------------------------------------------------------------
#   sudo apt install build-essential cmake libopencv-dev nlohmann-json3-dev
#
# OpenCV 提供图像处理，nlohmann-json 提供共享内存协议里的 JSON 读写。
# calib3d 已经链上了：做距离（PnP）会用到它。
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d)
find_package(nlohmann_json 3.10 REQUIRED)

# --- 算法核心（不含 IO，不依赖 nlohmann）--------------------------------
# 新加文件记得加在这里，否则链接会报 undefined reference。
add_library(armor_core
    vision/utils.cpp
    vision/preprocess.cpp
    vision/light_bar.cpp
    vision/armor.cpp
    vision/detector.cpp
    vision/angles.cpp
    pipeline/pipeline.cpp
)
target_include_directories(armor_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(armor_core PUBLIC ${OpenCV_LIBS})
target_compile_options(armor_core PRIVATE -Wall -Wextra -Wpedantic)

# --- 采集层（共享内存协议 + 仿真适配）-----------------------------------
add_library(armor_io
    io/sim_client.cpp
    io/sim_capture.cpp
)
target_link_libraries(armor_io PUBLIC armor_core nlohmann_json::nlohmann_json)
target_compile_options(armor_io PRIVATE -Wall -Wextra -Wpedantic)

# --- 可执行文件 ---------------------------------------------------------
add_executable(armor_aim
    app/main.cpp
    app/options.cpp
)
target_link_libraries(armor_aim PRIVATE armor_io)
target_compile_options(armor_aim PRIVATE -Wall -Wextra -Wpedantic)

# --- 参考实现（默认不构建）----------------------------------------------
# 同一道题的另一份答案，用来给"这件事到底能做到多好"定一个参照点。
#   cmake -S . -B build-ref -DBUILD_REFERENCE=ON
#   build-ref/armor_aim_ref ...
# 先自己做，卡住了再去读 reference/pipeline_ref.cpp —— 直接抄会跳掉
# 全部学习过程，而且它的 combined 场景成绩也很差（见 docs/stages.md）。
option(BUILD_REFERENCE "构建参考实现 armor_aim_ref" OFF)
if(BUILD_REFERENCE)
    add_executable(armor_aim_ref
        reference/main_ref.cpp
        reference/pipeline_ref.cpp
        reference/solver_ref.cpp
        reference/vision_ref.cpp
        app/options.cpp
    )
    target_link_libraries(armor_aim_ref PRIVATE armor_io)
    target_compile_options(armor_aim_ref PRIVATE -Wall -Wextra -Wpedantic)
endif()

# --- 测试 ---------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
    add_executable(armor_tests tests/core_tests.cpp)
    target_link_libraries(armor_tests PRIVATE armor_core)
    target_compile_definitions(armor_tests PRIVATE
        ARMOR_PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_compile_options(armor_tests PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME core COMMAND armor_tests core)
    add_test(NAME ideal_samples COMMAND armor_tests ideal_samples)
endif()

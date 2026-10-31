# armor_aim — 从零写一条自瞄链路

一台**接好了仿真、但算法全是空的**机器：

```text
仿真相机 ──▶ mask ──▶ 灯条 ──▶ 装甲板候选 ──▶ 像素→角度 ──▶ 云台指令
              ↑         ↑          ↑              ↑
             空        空         空             空            ← 全是你要写的
```

连上仿真、取帧、下发指令、打印统计这几件事是通的。中间的四段没有实现。

> 同一道题有一份完整实现在 **`solution` 分支**上，怎么读它见
> **[`docs/reading-guide.md`](docs/reading-guide.md)**。先自己写、卡住了再去读。

## 里面有什么

| | 状态 |
|---|---|
| 共享内存协议、取帧握手、看门狗、会话/序号管理 | ✅ 通 |
| 主循环（取帧 → 检测 → 估计 → 指令，50 Hz 下发） | ✅ 通 |
| 指令安全降级（仿真暂停 / 未开外部控制 / 状态不新鲜） | ✅ 通 |
| `Observation` 里的标定、云台姿态、相机安装偏移、弹道参数 | ✅ 有 |
| 灯条/装甲板的几何判定（`LightBar::isValid`、`Armor` 的评分公式） | ✅ 有 |
| `makeColorMask` / `findLightBars` / `matchArmors` | ❌ 返回空 |
| `Pipeline::estimate` / `Pipeline::command` | ❌ 返回空 |
| 逐帧遥测、离屏回放、绘图工具、参数扫描 | ❌ 没有，自己造 |

`ctest` 是两组：`core` 绿（锁角度符号约定），`ideal_samples` 红（你检测器的验收标准）。

## 1. 装依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev nlohmann-json3-dev
```

验证：

```bash
pkg-config --modversion opencv4          # 期望 4.x
ls /usr/include/nlohmann/json.hpp        # 有就是装好了
```

## 2. 构建与测试

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

## 3. 跑起来

仿真在本工程旁边（默认 `../rm_aim_sim`）。

```bash
# 终端一：起仿真。必须图形模式，--headless 没有像素输出。
cd ../rm_aim_sim
./run.sh -- --remote                       # 默认场景：4 m 静止靶
./run.sh -- --remote --config=/绝对路径/armor_detector_tutorial/scenarios/spin.cfg

# 终端二：跑自瞄
cd armor_detector_tutorial
./build/armor_aim --color=blue --no-gui            # 只瞄准，不开火
./build/armor_aim --color=blue --fire --no-gui     # 瞄准 + 开火
./build/armor_aim --color=blue --duration=8        # 带窗口，按 q 退出
```

结束时打印两行：

```text
[self] frames=438 with_target=0 target_rate=0.000 mean_detect_ms=0.000 mean_cmd_gap_deg=0.000
[sim]  fired=0 resolved=0 hit=0 hit_rate=0.000 kills=0
```

- `[self]` 是这条链路自己的体检：`target_rate` 有目标的帧占比，`mean_detect_ms`
  单帧检测耗时，`mean_cmd_gap_deg` 指令角与云台实际角之差（伺服跟不跟得上，
  **不是**瞄得准不准）。
- `[sim]` 是仿真给出的命中统计，**这个才算数**。

> `target_rate` 在静止靶上最高只有 0.7 左右不是 bug：**命中后灯条会熄灭 50 ms**。

批量跑分用 `bench/run.sh <场景> <秒>` 和 `bench/run_all.sh`，
一次跑完的场景用 `bench/report.py` 汇总成表。**主指标是命中/秒**
（`hit / 运行秒数`），口径、噪声、健康指标见 `docs/stages.md` 开头那一节。

## 4. 场景

`scenarios/*.cfg` 直接喂给仿真（`--config=`）。每个场景专门逼出一样能力：

| 场景 | 条件 | 逼你做什么 |
|---|---|---|
| `static` | 4 m 静止 | 先把闭环打通 |
| `lowlight` | 4 m，0.15× 曝光 | 看看你觉得"肯定会崩"的猜测对不对 |
| `far` | 5.5 m 静止 | 拿不到距离就没法做弹道补偿 |
| `translate` | 5.5 m，1.5 m/s 横移 | 提前量 |
| `spin` | 4 m，4 rad/s 自转 | 目标关联 + 速度估计 |
| `combined` | 4.5 m，1.2 m/s + 2.5 rad/s | 附加题：换板时的关联策略是关键 |
| `toofar` | 7 m 静止 | 检测的物理极限，不是算法问题 |

每个场景的现象、需要什么信息、改哪里、验收命令、陷阱，都在 `docs/stages.md`。

## 5. 你要自己造的工具

遥测和调试工具没有，缺什么造什么，通常按这个顺序：

| 症状 | 你缺的工具 |
|---|---|
| "到底检出来没有？" | 把 mask、灯条、候选、选中的板画在图上 |
| "云台怎么越推越远？" | 逐帧落盘（时间/真值角/指令角/云台角），事后画曲线 |
| "这个改动是真的还是运气？" | 跑 N 次取均值方差 |
| "检测的边界在哪？" | 扫距离 / 扫曝光的批量脚本 |
| "它收敛快不快、超调没有？" | 单次阶跃输入的响应曲线 + 收敛判据 |

改检测器最需要的是**离线复跑**：录一段帧，之后不开仿真也能跑。那个 recorder
是 40 行。

`app/options.cpp` 里加自己的命令行开关是允许的、也是预期的。但 `[self]` /
`[sim]` 这两行的**字段名别改**——评分脚本按名字取值。

## 6. 一条硬规矩

仿真的 `state.json` 里带 `truth`（每块装甲的真实位姿）。**不许用它瞄准。**

真机上没有这个信号，用了它你写的就不是自瞄。判断标准很简单：把代码里所有
`truth` 相关的读法删掉，成绩不能变。

## 7. 代码地图

| 文件 | 教什么 | 你大概会改它 |
|---|---|---|
| `core/types.h` | 一帧的输入长什么样；为什么姿态要跟着帧走 | 少 |
| `io/capture.h` | 采集层抽象 | 少 |
| `io/sim_client.cpp` | 共享内存协议、原子发布、看门狗 | 别动 |
| `io/sim_capture.cpp` | 协议 → `Observation` 的适配 | 少 |
| `vision/preprocess.cpp` | HSV、通道差、形态学 | **是** |
| `vision/light_bar.cpp` | 轮廓、`minAreaRect` 角度约定、几何过滤 | **是** |
| `vision/armor.cpp` | 两灯条配对与归一化几何量 | **是** |
| `vision/angles.cpp` | 像素 → 相对角，符号约定（被 `core` 测试锁着） | 少 |
| `pipeline/pipeline.cpp` | 目标选择、距离、提前量、开火门控 | **主要战场** |
| `app/main.cpp` | 主循环：取帧→检测→估计→指令 | 中 |

`detect()` 是纯函数，可以离线单测；`estimate()` / `command()` 带状态
（要攒历史帧才能估速度）。

## 8. 答案在哪

```bash
git fetch origin
git checkout solution      # 看完用 git checkout main 回来
```

读法在 [`docs/reading-guide.md`](docs/reading-guide.md)：按 S0→S4 分轮，
每轮只读一小段，读完立刻用对应场景验收。`solution` 的成绩和已知弱点也在那里。

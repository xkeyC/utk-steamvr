# UTK for Linux SteamVR

[English](README.md)

让 VIVE Ultimate Tracker（UTK）在 Linux 的 SteamVR 里作为追踪器使用，头显是 Quest 3 这类不带 Lighthouse 的一体机（通过 Steam Link 串流）。

Windows 上 HTC 的方案是 VIVE Hub + SteamVR 驱动 + 「VIVE Space Calibrator」。Linux 上 HTC 发布了 VIVE Hub for Linux（配对、建图）和 VIVE Ultimate Tracker Linux SDK，但没有 SteamVR 驱动和校准。本项目补上这两部分：

- **SteamVR 驱动**：在 SteamVR 里直接运行 HTC 的追踪器 SDK，每个追踪器约 125 Hz，数据延迟 2～4 ms，注册成 SteamVR 通用追踪器（可在「管理追踪器」里分配身体部位）。
- **VR 内校准**：SteamVR 菜单里的「UTK Calibration」标签页。手柄和任意一个追踪器握在一起画 8 字，自动识别是哪个追踪器，不用按键。

> 这是非官方的社区项目，与 HTC、Valve 无关，也未经其认可。VIVE、HTC 是 HTC Corporation 的商标。

## 安装

要求：Linux x86_64，glibc 2.35 及以上（Ubuntu 22.04、Debian 12、SteamOS 3、Arch、Fedora 等），使用原生 Steam 安装的 SteamVR，且至少启动过一次。暂不支持 Flatpak 版 Steam。

先退出 SteamVR，然后运行：

```sh
curl -fsSL https://github.com/xkeyC/utk-steamvr/releases/latest/download/install.sh | bash
```

安装脚本会：

1. 下载最新 Release 并校验 SHA-256；
2. 显示 VIVE Product EULA，你同意后从 [HTC 的 SDK 仓库](https://github.com/ViveRoboticsSoftware/vive-ultimate-tracker-sdk)下载 `libultimate-tracker.so`（固定提交，校验哈希）；
3. 装到 `~/.local/share/utk-steamvr`，不需要 root；
4. 把驱动注册进 `~/.config/openvr/openvrpaths.vrpath`，打开 `steamvr/activateMultipleDrivers`，并把校准页加进 SteamVR 菜单。改动的文件第一次修改前会备份为 `*.utk-backup`。

更新时再运行一次同样的命令。参数写在 `bash -s --` 后面：`--version v0.1.0` 安装指定版本，`--accept-eula` 非交互同意 EULA，`--uninstall` 卸载（保留 `~/.config/utk-steamvr` 里的校准）。

`activateMultipleDrivers` 必须打开：不开的话，SteamVR 只加载提供头显的驱动（Steam Link 的 vrlink、ALVR 等），不会加载本驱动。

## 使用

### 第一次：在 VIVE Hub 里配对、建图

用 VIVE Hub 配对追踪器并建好地图（「Start setup」）。地图存在追踪器里，环境不大变就只需要做一次。

**做完后关闭 VIVE Hub。** VIVE Hub 和 SteamVR 驱动都要独占追踪器的 dongle，不能同时运行。VIVE Hub 开着时，驱动不会启动追踪器服务，菜单里的校准面板会提示关闭它；关掉后驱动自动接管，不用重启 SteamVR。

### 在 VR 里校准

打开 SteamVR 菜单，点底栏的「UTK」图标：

1. 面板显示追踪器服务状态和每个追踪器的同步状态（就绪、同步地图、需要建图等）、电量、频率、延迟。
2. 点「Start calibration」，或在菜单关闭时按 A。
3. 把任意一个手柄和任意一个追踪器紧握在一起，画大的 8 字，同时转动手腕。程序自动识别是哪一对；需要约 25 秒的有效运动和足够的旋转。
4. 误差在 20 mm、3° 以内就保存到 `~/.config/utk-steamvr/calibration.json`，驱动 1 秒内生效；否则提示重画。

菜单关闭时：A 开始，Y 查看当前校准，B 取消。菜单打开时 SteamVR 不把手柄按键交给其他程序，只能用激光笔点。

重新建图、头显重新设定边界或重新定位后，需要重新校准。

## 从源码构建

用 Nix：

```sh
nix develop
cargo build && cargo test
./scripts/install-driver.sh        # 编译并装到 ~/.local/share/utk-steamvr
```

不用 Nix 时需要 Rust（stable）、clang/libclang、pkg-config 和 EGL/GL 开发头文件；`scripts/ci-deps.sh` 下载 `openvr_driver.h` 和 HTC 的 SDK 库，并打印构建需要的环境变量。完整步骤见 `.github/workflows/ci.yml`，`scripts/package.sh` 生成 Release 压缩包。

工具：

```sh
./target/release/utk-calibrate --snapshot panel.png   # 不连 SteamVR，把面板画成 PNG
./target/release/utk status --watch                    # 在终端里看驱动状态（读 status.json）
./target/release/utk record -d 30                      # 录 SteamVR 里手柄、头显和 utk 追踪器的姿态
```

NixOS 上用 Release 安装时，安装脚本最后一步需要 `programs.nix-ld.enable`；上面的 Nix 构建不需要。

## 目录

```
crates/utk-driver/     SteamVR 驱动：C++ 层实现 OpenVR 驱动接口并接入 HTC SDK，Rust 层处理姿态、校准、状态
crates/utk-driver/sdk/ HTC SDK 头文件（Apache-2.0）
crates/utk-calib/      校准算法（自动识别、只解竖直轴旋转的 Kabsch、Gauss–Newton 联合精修）
crates/utk-calibrate/  VR 内校准：SteamVR 菜单面板 + 校准时的悬浮面板（egui，headless EGL），以及安装注册
crates/utk-cli/        utk 命令行：status / record
scripts/               安装脚本、CI 依赖下载、打包
flake.nix              开发环境；HTC SDK 库从 HTC 的 GitHub 下载
```

## 运行时文件

| 文件 | 内容 |
|---|---|
| `~/.config/utk-steamvr/calibration.json` | 校准结果：追踪器地图 → SteamVR 原始坐标系 |
| `~/.local/state/utk-steamvr/status.json` | 驱动每半秒写一次：服务状态、每个追踪器的状态 |
| `~/.local/state/utk-steamvr/calibrate.log` | 校准程序日志 |
| `vrserver.txt`（Steam 的 logs 目录）| 驱动日志，以 `utk:` 开头 |

## 已确认的事实

- HTC SDK 回调：每个追踪器约 125 Hz，逐个样本到达（间隔 8 ms），`GetTimestamp() - Timestamp_us` 为 2～4 ms。角速度有值，线速度为 0。坐标系为 Isaac Sim（X 前、Y 左、Z 上），驱动换成 OpenVR（X 右、Y 上、Z 后）。有 `PoseStable`（静止）标志，驱动在静止时固定姿态。
- VIVE Hub 的 `tracker_server` 每 50 ms 打包发一批，同批时间戳几乎相同；dongle 本身每约 3 ms 一份 HID 报告。经它读姿态平均晚 25 ms 以上并且每批跳一次，直接用 SDK 没有这个问题。
- 追踪器地图是重力对齐的；原点在建图时手持主追踪器的高度。
- SteamVR 的进程间通信用抽象 socket，按网络命名空间隔离；Steam 和 pressure-vessel 有私有 `/tmp`。
- 校准算法（实录数据回测）：识别相关 0.99 vs 其他组合 ≤ 0.24；位置残差 RMS 8.8 mm，姿态残差中位数 1.1°。

## 协议

MIT 或 Apache-2.0 双协议，任选其一（[LICENSE-MIT](LICENSE-MIT)、[LICENSE-APACHE](LICENSE-APACHE)）。

`crates/utk-driver/sdk/ultimate_tracker.h` 版权属于 HTC，Apache-2.0。HTC 的 SDK 库（`libultimate-tracker.so`）受 [VIVE Product EULA](https://dl.vive.com/EULA/unified_eula_20230721_eng.pdf) 约束，不允许再分发，所以本仓库和 Release 压缩包都不包含它，由安装脚本在你的机器上从 HTC 下载。Release 压缩包里还有 `libopenvr_api.so`（Valve，BSD-3-Clause）和 egui 内嵌的字体，协议见 `licenses/`。

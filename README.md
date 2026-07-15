# Digua Robot Competition Workspace

第 21 届全国大学生智能汽车竞赛「地瓜机器人」赛项开发工作空间。项目基于 OriginCar 阿克曼底盘、RDK X5、TogetheROS.Bot / ROS 2 Humble，围绕 SLAM 建图、Nav2 自主导航、二维码识别、车载屏幕显示和任务自动巡航进行开发。

> 说明：本仓库已按公开仓库处理，不包含本地 SSH 密码、私有调试日志、会话记忆和个人机器路径。实际小车 IP、账号、地图文件和现场点位请以比赛现场配置为准。

## 当前能力

- ROS 2 工作空间结构，支持 `colcon build`
- OriginCar 底盘源码与 N10 激光雷达驱动源码
- `dg_pkg` 竞赛业务包：
  - Nav2 启动封装
  - 阿克曼安全行为树
  - Smac Hybrid-A* 全局规划配置
  - Regulated Pure Pursuit 局部控制配置
  - 二维码识别节点
  - 7 寸屏二维码结果显示节点
  - 最小自动巡航节点：P 点出发、到 QR 点、等待二维码方向、按预设航点返回 P 点停车
- SLAM、Nav2、二维码识别、屏幕显示、自动巡航等操作指南

## 仓库结构

```text
dev_ws/
├── README.md
├── FRAMEWORK.md
├── src/
│   ├── dg_pkg/                 # 竞赛业务代码
│   │   ├── dg_pkg/
│   │   │   ├── nav_cruise.py
│   │   │   ├── qrcode_detector.py
│   │   │   └── qr_result_display.py
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   ├── competition_params.yaml
│   │   │   └── display_text/
│   │   ├── behavior_trees/
│   │   └── launch/
│   ├── origincar/              # OriginCar 底盘相关源码
│   ├── lslidar_ros2/           # 轮趣雷达驱动
│   ├── ekf_no_imu_yaw.yaml
│   ├── ekf_calibrated.yaml
│   ├── slam_toolbox_params.yaml
│   ├── SLAM建图操作指南.md
│   ├── Nav2启动与进程清理操作指南.md
│   ├── 二维码识别与屏幕显示操作指南.md
│   └── 自动巡航最小闭环操作指南.md
└── .gitignore
```

## 环境

目标平台：

- RDK X5
- TogetheROS.Bot 3.2.3 / ROS 2 Humble
- OriginCar 阿克曼底盘
- 轮趣 N10 激光雷达
- USB 摄像头
- 7 寸 HDMI 屏幕

常用 ROS 环境变量：

```bash
source /opt/tros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 构建

在 RDK X5 小车端：

```bash
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只构建竞赛业务包：

```bash
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select dg_pkg --symlink-install
source install/setup.bash
```

## 关键启动入口

Nav2：

```bash
ros2 launch dg_pkg dg_nav2.launch.py map:=/root/maps/competition_map.yaml
```

二维码识别：

```bash
ros2 run dg_pkg qrcode_detector
```

二维码结果屏幕显示：

```bash
export DISPLAY=:0
ros2 run dg_pkg qr_result_display
```

最小自动巡航：

```bash
ros2 launch dg_pkg dg_competition.launch.py start_qrcode:=true start_display:=false
```

更完整的启动顺序见：

- [Nav2启动与进程清理操作指南](src/Nav2启动与进程清理操作指南.md)
- [二维码识别与屏幕显示操作指南](src/二维码识别与屏幕显示操作指南.md)
- [自动巡航最小闭环操作指南](src/自动巡航最小闭环操作指南.md)

## 二维码规则

当前比赛规则中二维码为 `1~9999` 的数字：

- 奇数：顺时针，发布 `/sign_foxglove = 3`
- 偶数：逆时针，发布 `/sign_foxglove = 4`
- 非法内容：发布 `/sign_foxglove = 0`

识别节点同时发布：

- `/qr_code/data`：二维码原始内容
- `/qr_code/direction`：方向字符串

屏幕显示节点会显示方向和二维码原始数据。

## 自动巡航概要

`nav_cruise.py` 从 `competition_params.yaml` 读取初始位姿和航点，当前最小闭环为：

```text
P 点启动
  ↓ 自动发布 /initialpose
  ↓ 导航到 QR 点
  ↓ 等待 /sign_foxglove = 3 或 4
  ↓ 按 route_minimal 后续航点行驶
  ↓ 返回 P 点
  ↓ 停车
```

新增航点通常只需要修改：

```text
src/dg_pkg/config/competition_params.yaml
```

## 公开仓库脱敏说明

本仓库不跟踪以下本地文件或目录：

- `.agents/`
- `.claude/`
- `AGENTS.md`
- `CLAUDE.md`
- `PROJECT_MEMORY.md`
- `CONVERSATION_ARCHIVE.md`
- `PROJECT_INDEX.md`
- `nav2日志.md`
- `src/当前调试交接清单.md`
- `build/`
- `install/`
- `log/`

这些文件包含本地协作记忆、调试日志、机器路径或现场连接信息，不适合公开发布。

## 注意

- `origincar/` 和 `lslidar_ros2/` 属于底盘与雷达基础包，修改前应先确认影响范围。
- 地图文件、现场小车 IP、账号密码、比赛当天路障布置不应硬编码到公开仓库。
- 若重新建图或重新采点，需要同步更新 `competition_params.yaml` 中的点位。

# OriginCar Nav2 启动与进程清理操作指南

> 适用环境：OriginCar + RDK X5 + N10 激光雷达  
> ROS2：TogetheROS.Bot 3.2.3（Humble）  
> 小车工作空间：`/userdata/dev_ws`  
> 默认通信环境：Domain 30 + CycloneDDS

## 1. 重要原则

1. 底盘、EKF、雷达和 Nav2 各启动一次。
   `base_footprint -> base_link` 与 `base_link -> laser` 静态 TF 由修正后的
   `origincar_bringup.launch.py` 自动发布，不再另开终端手动补 TF。
2. 所有终端必须使用相同的 `ROS_DOMAIN_ID` 和 RMW。
3. 不要同时运行官方 Nav2 和 `dg_pkg` Nav2。
4. 不要同时运行两个 EKF 或两套 lifecycle manager。
5. `2D Pose Estimate` 必须设置在地图中的开阔区域，机器人 footprint
   不能压住墙壁或膨胀区。
6. OriginCar 是阿克曼底盘，不能执行零线速度原地旋转。

## 2. 配置终端默认环境

只需配置一次。以后新打开的 MobaXterm SSH 终端会自动加载。

> 贴到 MobaXterm 执行：

```bash
grep -qxF 'source /opt/tros/humble/setup.bash' ~/.bashrc || \
  echo 'source /opt/tros/humble/setup.bash' >> ~/.bashrc

grep -qxF 'source /userdata/dev_ws/install/setup.bash' ~/.bashrc || \
  echo 'source /userdata/dev_ws/install/setup.bash' >> ~/.bashrc

grep -qxF 'export ROS_DOMAIN_ID=30' ~/.bashrc || \
  echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc

grep -qxF 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' ~/.bashrc || \
  echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

source ~/.bashrc

echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
```

预期：

```text
ROS_DISTRO=humble
ROS_DOMAIN_ID=30
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 3. 停止和清理全部 ROS2 进程

### 3.1 正常停止

先在所有正在运行 ROS2 命令的终端按 `Ctrl+C`。

确认底盘已经停止后，再进行下一步。

### 3.2 彻底清理残留进程

以下命令会停止小车上的全部 ROS2、底盘、雷达、EKF 和 Nav2 进程。
执行前确保小车处于安全位置。

> 贴到 MobaXterm 执行：

```bash
sudo pkill -9 -f ros2
sudo pkill -9 -f nav2
sudo pkill -9 -f amcl
sudo pkill -9 -f map_server
sudo pkill -9 -f planner_server
sudo pkill -9 -f controller_server
sudo pkill -9 -f behavior_server
sudo pkill -9 -f bt_navigator
sudo pkill -9 -f velocity_smoother
sudo pkill -9 -f lifecycle_manager
sudo pkill -9 -f ekf_node
sudo pkill -9 -f origincar
sudo pkill -9 -f lslidar
sudo pkill -9 -f static_transform_publisher
sudo pkill -9 -f rviz2
```

### 3.3 重启 ROS2 daemon

```bash
source ~/.bashrc
ros2 daemon stop
ros2 daemon start
```

### 3.4 确认清理完成

```bash
ps -ef | grep -E \
"ros2|nav2|amcl|map_server|planner_server|controller_server|behavior_server|bt_navigator|velocity_smoother|lifecycle_manager|ekf|origincar|lslidar|static_transform" \
| grep -v grep
```

清理完成时不应看到相关节点进程。`ros2 daemon` 进程可以存在。

## 4. 启动前检查

```bash
source ~/.bashrc

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

ls -l /dev/ttyACM0 /dev/ttyACM1
ls -l /root/maps/competition_map.yaml
ls -l /root/maps/competition_map.pgm
```

设备约定：

| 设备 | 用途 |
|------|------|
| `/dev/ttyACM0` | STM32 底盘控制器 |
| `/dev/ttyACM1` | N10 激光雷达 |

如果串口权限不足：

```bash
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1
```

## 5. 按顺序启动系统

每个命令使用一个独立终端，并保持终端运行。

### 5.1 终端 1：启动底盘

```bash
source ~/.bashrc
ros2 launch origincar_base origincar_bringup.launch.py carto_slam:=true
```

`carto_slam:=true` 用于禁止底盘 launch 内的默认 EKF，避免重复启动 EKF。

### 5.2 终端 2：启动自定义 EKF

```bash
source ~/.bashrc
ros2 run robot_localization ekf_node --ros-args \
  --params-file /userdata/dev_ws/src/ekf_no_imu_yaw.yaml \
  -r odometry/filtered:=odom_combined
```

TF 帧为 `odom`，EKF 输出话题为 `/odom_combined`。二者名称不同是正常的。

### 5.3 终端 3：启动 N10 雷达

```bash
source ~/.bashrc
ros2 launch lslidar_driver lsn10_launch.py
```

### 5.4 终端 4：启动自定义 Nav2

```bash
source ~/.bashrc
ros2 launch dg_pkg dg_nav2.launch.py \
  map:=/root/maps/competition_map.yaml
```

禁止同时执行：

```bash
ros2 launch nav2_bringup bringup_launch.py ...
```

否则会产生重复 Nav2 节点和 lifecycle manager。

## 6. 启动后逐层验收

另开一个终端执行。

### 6.1 检查节点

```bash
source ~/.bashrc

ros2 node list | sort
```

至少应看到：

```text
/origincar_base
/ekf_filter_node
/x10/lslidar_driver_node
/map_server
/amcl
/planner_server
/controller_server
/smoother_server
/behavior_server
/bt_navigator
/velocity_smoother
```

### 6.2 检查生命周期

```bash
for node in map_server amcl planner_server controller_server \
smoother_server behavior_server bt_navigator velocity_smoother; do
  echo -n "$node: "
  ros2 lifecycle get /$node
done
```

所有节点都应显示：

```text
active [3]
```

### 6.3 检查基础话题

```bash
timeout 5 ros2 topic hz /odom
timeout 5 ros2 topic hz /odom_combined
timeout 5 ros2 topic hz /scan
ros2 topic info /map
```

预期：

- `/odom` 约 20Hz
- `/odom_combined` 约 20Hz
- `/scan` 约 10Hz
- `/map` 有一个发布者

### 6.4 检查 TF

```bash
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint
timeout 5 ros2 run tf2_ros tf2_echo base_footprint base_link
timeout 5 ros2 run tf2_ros tf2_echo base_link laser
```

静态 TF 预期值：

```text
base_footprint -> base_link = [0.000, 0.000, 0.000]
base_link -> laser = [0.036, 0.000, 0.155]
```

设置初始位姿后再检查：

```bash
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
```

完整 TF 应为：

```text
map -> odom -> base_footprint -> base_link -> laser
```

## 7. 启动 RViz2

### 7.1 终端 5：启动 RViz2

```bash
source ~/.bashrc
rviz2 -d /root/slam.rviz
```

确认配置文件存在：

```bash
ls -l /root/slam.rviz
```

`/root/slam.rviz` 是当前小车保存的 RViz2 配置文件。

### 7.2 RViz2 显示配置

1. `Global Options -> Fixed Frame` 设置为 `map`。
2. 添加 `Map`，Topic 设置为 `/map`。
3. Map 的 Durability 设置为 `Transient Local`。
4. 添加 `LaserScan`，Topic 设置为 `/scan`。
5. 添加 `TF`、`RobotModel`、`Path`。
6. Path 的 Topic 设置为 `/plan`。

如果 `/root/slam.rviz` 已经保存上述配置，只需检查 Fixed Frame、Map 和
Path 的 Topic 是否仍正确。

### 7.3 设置初始位姿

点击 `2D Pose Estimate`，在地图上小车的真实位置点击并拖动朝向。

注意：

- 小车中心必须位于开阔白色区域。
- 机器人 footprint 不能压住黑色墙壁或红色膨胀区域。
- 不要因为发车点叫 P 点就直接把坐标设置为 `(0, 0)`。
- 当前地图分析曾发现 `(0.00, 0.03)` 距障碍约 5cm，无法作为规划起点。

设置后检查：

```bash
ros2 topic echo /amcl_pose --once
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
```

同时观察 `/scan` 与地图墙面是否大体重合：

- 基本贴墙：TF 和初始位姿可继续用于 Nav2 静态检查。
- 整体平移：优先复核 `base_link -> laser` 是否仍为 `[0.036, 0, 0.155]`。
- 明显旋转或镜像：不要发导航目标，先排查雷达 frame、TF 和地图来源。

## 8. 设置导航目标

1. 确认初始位姿正确。
2. 使用 `Navigation2 Goal` 在同一片可通行白色区域设置目标。
3. 首次测试目标距离小车约 0.5 至 1m，方向尽量朝前。
4. 观察是否出现绿色 `/plan` 路径。

检查导航状态：

```bash
timeout 5 ros2 topic echo /navigate_to_pose/_action/status
```

状态含义：

| 状态值 | 含义 |
|--------|------|
| `2` | 正在执行 |
| `4` | 成功 |
| `5` | 已取消 |
| `6` | 中止 |

检查速度：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
```

正常导航时应看到：

```text
linear.x > 0
```

## 9. 常见问题

### 9.1 `frame 'map' does not exist`

检查：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /map
```

`map_server` 和 `amcl` 必须为 `active`。

### 9.2 进程存在，但 `ros2 node list` 看不到

检查进程实际环境：

```bash
pid=$(pgrep -n ekf_node)
tr '\0' '\n' < /proc/$pid/environ | \
  grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_DISTRO'
```

如果缺少 Domain 30 或 CycloneDDS，停止节点并从加载了 `~/.bashrc`
的新终端重新启动。

### 9.3 有地图但没有绿色 Path

查看 Nav2 终端第一条规划错误。

常见日志：

```text
GridBased: failed to create plan
```

优先检查：

1. 起点或目标是否压在障碍物和膨胀区。
2. 初始位姿是否与小车真实位置一致。
3. 起点和目标是否位于同一片自由空间。
4. 地图是否为小车上的最新版本。

### 9.4 有 Path 但小车不动

检查：

```bash
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel --once
```

- `/cmd_vel_nav` 无数据：控制器未执行。
- `/cmd_vel_nav` 有数据、`/cmd_vel` 无数据：velocity smoother 链路异常。
- `/cmd_vel` 非零但车不动：检查底盘、串口或急停。
- `linear.x=0`、仅 `angular.z` 非零：控制器要求原地旋转，阿克曼底盘无法执行。

当前自定义 Nav2 使用 Regulated Pure Pursuit，并设置：

```yaml
use_rotate_to_heading: false
```

### 9.5 目标立即中止

```bash
timeout 5 ros2 topic echo /navigate_to_pose/_action/status
```

如果状态为 `6`，查看 Nav2 终端中目标开始后的第一条 `WARN` 或 `ERROR`。
恢复行为失败通常是前面规划或碰撞问题的结果，不一定是最初原因。

## 10. 推荐的每次启动顺序

```text
1. 所有旧终端 Ctrl+C
2. 清理全部残留 ROS2 进程
3. 重启 ROS2 daemon
4. 启动底盘
5. 启动自定义 EKF
6. 启动 N10 雷达
7. 启动 dg_pkg Nav2
8. 检查 lifecycle、话题和 TF
9. 启动 RViz2
10. 设置准确的 2D Pose Estimate，并确认 /scan 与墙面重合
11. 设置近距离 Navigation2 Goal
12. 验证 /plan、/cmd_vel_nav 和实车运动
```

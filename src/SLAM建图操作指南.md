# OriginCar SLAM 建图操作指南

> **适用环境**: TogetheROS.Bot 3.2.3 (Humble) + RDK X5 + OriginCar + 轮趣N10 激光雷达  
> **已有功能包**: `lslidar_ros2`（雷达驱动）、`origincar`（底盘驱动）  
> **原理**: 利用现有功能包启动底盘和雷达，配合 ROS2 标准 `slam_toolbox` 实现在线建图

---

## 1. 前置检查

### 1.1 确认 TogetheROS 环境

```bash
# SSH 登录 RDK
ssh root@<RDK_IP>

# 确认 tros.b 版本
apt show tros | grep Version

# 确认 ROS2 发行版
echo $ROS_DISTRO
# 应输出: humble

# 确认环境变量
source /opt/tros/humble/setup.bash
```

### 1.2 安装 slam_toolbox（如未安装）

`slam_toolbox` 不随 tros.b 基础包安装，需单独安装：

```bash
source /opt/tros/humble/setup.bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-slam-toolbox -y
```

验证安装：

```bash
ros2 pkg list | grep slam_toolbox
# 应输出: slam_toolbox
```



### 1.3 安装键盘遥控（RDK 端，用于手动控制建图）

```bash
source /opt/tros/humble/setup.bash
sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard -y
```



### 1.4 确认串口设备

```bash
# 在 RDK 上查看串口
ls /dev/ | grep -E "ttyUSB|ttyACM"
```


| 设备             | 用途          | 对应配置                    |
| -------------- | ----------- | ----------------------- |
| `/dev/ttyACM0` | STM32 底盘控制器 | `origincar_base` 默认     |
| `/dev/ttyACM1` | N10 激光雷达    | `lslidar_n10_uart.yaml` |


> **注意**: 如果 N10 雷达串口不是 `/dev/ttyACM1`，需要修改 `lslidar_driver/config/lslidar_n10_uart.yaml` 中的 `serial_port` 参数。

```bash
# 给串口赋权（每次重新插拔后需要执行）
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1
```

---



## 2. 编译工作空间

```bash
cd /userdata/dev_ws

# 配置环境
source /opt/tros/humble/setup.bash

# 编译（首次全量，后续可按需增量）
colcon build --symlink-install

# 加载工作空间环境
source install/setup.bash

# 建议将环境变量写入 ~/.bashrc（如未写入）
echo "source /opt/tros/humble/setup.bash" >> ~/.bashrc
echo "source /userdata/dev_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

---



## 3. 创建 SLAM 参数文件

`slam_toolbox` 的默认参数中 `odom_frame` 为 `odom`，自定义 EKF 也发布标准 `odom` 帧，与 Nav2 完全兼容。

创建 `/userdata/dev_ws/src/slam_toolbox_params.yaml`：

```bash
cat > /userdata/dev_ws/src/slam_toolbox_params.yaml << 'EOF'
slam_toolbox:
  ros__parameters:

    # ====== ROS 参数（重要：匹配 OriginCar 坐标系）======
    odom_frame: odom                # EKF 发布标准 odom 帧（兼容 Nav2）
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping                   # 建图模式
    use_map_saver: true

    # ====== 地图参数 ======
    map_file_name: /root/maps/competition_map
    map_start_pose: [0.0, 0.0, 0.0]
    map_update_interval: 5.0        # 地图更新间隔 (秒)
    resolution: 0.05                # 地图分辨率 (m/pixel)，5cm
    max_laser_range: 20.0           # 激光最大使用距离 (m)
    minimum_time_interval: 0.5      # 最小处理间隔 (秒)
    transform_timeout: 0.2          # TF 变换超时 (秒)
    tf_buffer_duration: 30.0        # TF 缓冲区 (秒)
    stack_size_to_use: 40000000     # 堆栈大小 (40MB)

    # ====== 求解器 ======
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_diagonal_type: CLUSTERED
    ceres_loss_function: None

    # ====== 回环检测 ======
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # ====== 扫描匹配 ======
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # ====== 动态处理 ======
    enable_dynamic_processing: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # ====== 搜索参数 ======
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
EOF
```

> **参数说明**:
>
> - `odom_frame: odom` — 自定义 EKF 发布标准 odom 帧，兼容 Nav2 默认参数
> - `base_frame: base_footprint` — 与 OriginCar TF 树的基座标系一致
> - `resolution: 0.05` — 5cm 分辨率，适合竞赛场地（5m×5m）
> - `max_laser_range: 20.0` — N10 雷达最大测距 25m，取 20m 保证数据质量

---



## 4. 确认 TF 坐标变换

建图前需确认完整的 TF 树。2026-06-26 已根据实车三视图和 RViz
`/scan` 对墙结果修正 `origincar_bringup.launch.py`，当前项目基准 TF 链路为：

```
map → odom → base_footprint → base_link → laser
  ↑     ↑          ↑              ↑           ↑
slam   EKF       静态TF         静态TF      静态TF
发布  发布       (0,0,0)        URDF       (0.036,0,0.155)
```



### 4.1 雷达 TF 要求（N10 文档原文）

> Nav2 或 SLAM 配置中常见设置如下：
>
> ```
> scan_topic: /scan
> ```
>
> 同时需要保证 TF 中存在从机器人底盘坐标系到雷达坐标系的变换：
>
> ```
> base_link -> laser
> ```
>
> 如果没有雷达 TF，可以临时发布静态 TF：
>
> ```bash
> ros2 run tf2_ros static_transform_publisher 0 0 0.18 0 0 0 base_link laser
> ```
>
> 其中 `0 0 0.15` 需要根据雷达实际安装高度和位置修改。



### 4.2 本项目已修正的 OriginCar TF

在 `origincar_base/launch/origincar_bringup.launch.py` 中，当前项目版本已修正为：

```python
# base_footprint → base_link
arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']

# base_link → laser
arguments=['0.036', '0', '0.155', '0', '0', '0', 'base_link', 'laser']
```

同时 `link_to_laser` 已加入 `LaunchDescription`，因此正常启动底盘后会自动发布
`base_link → laser`，不再需要另开终端手动执行 `static_transform_publisher`。

板端验收命令：

```bash
timeout 5 ros2 run tf2_ros tf2_echo base_footprint base_link
timeout 5 ros2 run tf2_ros tf2_echo base_link laser
```

预期：

```text
base_footprint -> base_link: [0.000, 0.000, 0.000]
base_link -> laser: [0.036, 0.000, 0.155]
```

> **重要**: 如果 `/scan` 与墙面整体错位，应优先复核上述 TF 是否是新值。
> 修改依据、影响和回滚方法见 `src/Origincar_TF修改说明.md`。

---



## 5. N10 雷达参数参考

> 以下参数说明来自 [N10 官方文档](https://origincar.guyuehome.com/application/n10_lidar_ros2_package_usage/)，配置文件位于 `lslidar_driver/config/lslidar_n10_uart.yaml`。



### 5.1 关键参数确认

打开配置文件确认以下参数：

```bash
cat /userdata/dev_ws/src/lslidar_ros2/lslidar_driver/config/lslidar_n10_uart.yaml
```

重点关注（YAML 结构为 `x10 → lslidar_driver_node → ros__parameters`）：

```yaml
x10:
  lslidar_driver_node:
    ros__parameters:
      lidar_type: "X10"                  # N10 所属驱动类型，不建议修改
      lidar_model: "N10"                 # 雷达型号
      serial_port: "/dev/ttyACM1"        # 串口名称
      frame_id: "laser"                  # 雷达坐标系（与 TF 中 laser frame 一致）
      pointcloud_topic: "lslidar_point_cloud"  # 点云话题（相对路径）
      laserscan_topic: "/scan"           # LaserScan 话题（绝对路径，无视namespace）
      publish_scan: true                 # 必须为 true，SLAM 依赖 /scan
      use_high_precision: false          # 高精度模式（消耗算力，建议关闭）
      min_range: 0.15                    # 最小有效距离 (m)
      max_range: 50.0                    # 最大有效距离 (m)，slam_toolbox 取 20m 即可
```



### 5.2 角度屏蔽配置

如果车身结构遮挡雷达部分角度，可屏蔽：

```yaml
# 默认不屏蔽（全 360°）
angle_disable_min: [0]
angle_disable_max: [0]

# 示例：屏蔽 0°~90°（角度单位为 0.01°，9000 = 90°）
angle_disable_min: [0]
angle_disable_max: [9000]

# 示例：屏蔽两个区间 0°~90° 和 180°~270°
angle_disable_min: [0, 18000]
angle_disable_max: [9000, 27000]
```



### 5.3 参数修改后注意事项

> **Attention**: 修改 `config/*.yaml` 参数后，需重新编译并 source 环境：
>
> ```bash
> cd /userdata/dev_ws
> colcon build --packages-select lslidar_driver
> source install/setup.bash
> ```

---



## 6. SLAM 建图操作



### 6.1 创建地图保存目录

```bash
mkdir -p ~/maps
```



### 6.2 启动建图（6 个 RDK 终端）

**启动前清环境**（每次重新启动必做）：

```bash
sudo pkill -9 -f ros2
ros2 daemon stop && ros2 daemon start
```



#### 终端 1（RDK）：启动底盘 + 传感器（禁用默认 EKF）

```bash
source /opt/tros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash

# carto_slam:=true 禁用默认 EKF，改为用终端2的自定义 EKF
ros2 launch origincar_base origincar_bringup.launch.py carto_slam:=true
```

预期输出：底盘串口连接成功，IMU 启动，TF 发布正常（EKF 不启动）。

#### 终端 2（RDK）：启动自定义 EKF（禁用 IMU yaw）

> **关键**：默认 EKF 融合了 IMU 陀螺仪的 yaw，陀螺零偏会导致 `odom` 坐标持续旋转 → 地图跟着旋转。自定义配置 `ekf_no_imu_yaw.yaml` 禁用了 IMU yaw/vya，yaw 仅由轮式里程计推定，避免漂移。

```bash
source /opt/tros/humble/setup.bash

# 使用自定义 EKF 参数（禁用 IMU yaw 融合）
ros2 run robot_localization ekf_node --ros-args \
  --params-file /userdata/dev_ws/src/ekf_no_imu_yaw.yaml \
  -r odometry/filtered:=odom_combined
```

验证 EKF：

```bash
# 新终端检查
source /opt/tros/humble/setup.bash
ros2 topic echo /odom_combined --once  # 应有数据
```



#### 终端 3（RDK）：启动 N10 激光雷达

```bash
source /opt/tros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash

# 启动 N10 雷达（串口版）
ros2 launch lslidar_driver lsn10_launch.py
```

验证雷达数据：

```bash
# 新终端检查
source /opt/tros/humble/setup.bash
ros2 topic hz /scan          # 应稳定输出 ~10Hz
ros2 topic echo /scan --once --field range_min  # 应输出 0.15
```

> **如果 /scan 无数据**：检查串口权限 (`sudo chmod 777 /dev/ttyACM1`)，确认 `lslidar_n10_uart.yaml` 中 `serial_port` 正确。



#### 终端 4（RDK）：启动 SLAM Toolbox

```bash
source /opt/tros/humble/setup.bash

# 使用自定义参数启动在线建图
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=/userdata/dev_ws/src/slam_toolbox_params.yaml
```

预期输出：

```
[INFO] [slam_toolbox]: Node using stack size 40000000
[INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
Registering sensor: [Custom Described Lidar]
```



#### 终端 5（RDK）：键盘遥控建图

```bash
source /opt/tros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> **OriginCar 是阿克曼底盘**，键盘控制方式与差速底盘略有不同。建议使用低速度（~0.15 m/s）缓慢行驶。



#### 终端 6（RDK）：Rviz2 可视化

**方式A：使用 lslidar_driver 自带 RViz 配置（推荐）**

```bash
source /opt/tros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
rviz2 -d /userdata/dev_ws/install/lslidar_driver/share/lslidar_driver/rviz/lslidar_x10.rviz
```

> 自带配置文件已预配置 `/scan` LaserScan 和 `/lslidar_point_cloud` PointCloud2 显示，Fixed Frame 预设为 `laser`。**注意**：建图时需将 Fixed Frame 改为 `map`。

**方式B：手动配置（SLAM 建图完整视图）**

```bash
source /opt/tros/humble/setup.bash
rviz2
```

在 RViz2 中手动添加以下 Display：


| 步骤  | Display 类型      | 操作                                                    | 说明                              |
| --- | --------------- | ----------------------------------------------------- | ------------------------------- |
| 1   | **Fixed Frame** | Global Options → Fixed Frame 设为 `map`                 | SLAM 建图时用 `map`；仅测试雷达时用 `laser` |
| 2   | **Map**         | Add → By topic → `/map` → Map                         | 实时显示 slam_toolbox 构建的地图         |
| 3   | **LaserScan**   | Add → By topic → `/scan` → LaserScan                  | N10 雷达二维扫描数据                    |
| 4   | **PointCloud2** | Add → By topic → `/lslidar_point_cloud` → PointCloud2 | N10 雷达点云数据                      |
| 5   | **TF**          | Add → TF                                              | 显示坐标系变换关系（map→odom→base→laser）  |
| 6   | **RobotModel**  | Add → RobotModel                                      | 显示 OriginCar 3D 模型              |
| 7   | **Path** (可选)   | Add → By topic → `/plan` → Path                       | 显示 Nav2 规划路径（导航时用）              |


**仅测试雷达时的 RViz 配置**（不需要 SLAM）：


| 步骤          | Display                       | 设置                                                                   |
| ----------- | ----------------------------- | -------------------------------------------------------------------- |
| Fixed Frame | `laser`                       | 与 N10 参数 `frame_id: "laser"` 一致                                      |
| LaserScan   | topic: `/scan`                | N10 发布在绝对路径 `/scan`                                                  |
| PointCloud2 | topic: `/lslidar_point_cloud` | N10 发布在 namespace `x10` 下，完整路径为 `/lslidar_point_cloud`（参数中不带 `/` 前缀） |


> **说明**: N10 雷达的 `laserscan_topic` 参数配置为绝对路径 `"/scan"`，因此无论节点 namespace 是否为 `x10`，`/scan` 始终在根命名空间。而 `pointcloud_topic` 配置为 `"lslidar_point_cloud"`（相对路径），实际话题为 `/x10/lslidar_point_cloud`。

---



### 6.3 建图步骤

1. 将车模放置在 **P 点**（发车点）
2. 在 RViz2 中点击 **"2D Pose Estimate"**，在地图上 P 点位置点击并拖动设置初始朝向
3. 用键盘/手柄遥控车模在场地内 **缓慢匀速** 行驶
4. 确保覆盖所有区域：
  - ✅ 区域A（大厅）
  - ✅ 区域B（通道区）
  - ✅ 区域C（诊疗区）
  - ✅ 黄色通道全覆盖
  - ✅ 通道两侧边界清晰
5. 观察 RViz2 中地图逐渐完整，无明显重影或错位
6. **回环检测**：让车模回到起点附近，帮助 slam_toolbox 优化地图

---



## 7. 保存地图

建图满意后，在 RDK 上执行：

```bash
# 保存地图（RDK 端，推荐用 slam_toolbox 内部服务，避免 map_saver_cli 订阅超时）
source /opt/tros/humble/setup.bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/root/maps/competition_map'}}"

# 备选：如果上面失败，用 map_saver（加长超时）
ros2 run nav2_map_server map_saver_cli --ros-args -p save_map_timeout:10000.0 -- -f ~/maps/competition_map
```

保存成功后会生成两个文件：

```
~/maps/
├── competition_map.pgm       # 地图图像（灰度图）
└── competition_map.yaml      # 地图描述文件（分辨率、原点等）
```

查看地图文件：

```bash
cat ~/maps/competition_map.yaml
```

示例内容：

```yaml
image: competition_map.pgm
mode: trinary
resolution: 0.05
origin: [-2.5, -2.5, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

---



## 8. 地图验证（可选）

使用项目自定义 Navigation2 加载地图进行导航测试。

> EKF 已使用标准 `odom` 帧（`ekf_no_imu_yaw.yaml`）。
> 车体与雷达静态 TF 已由修正后的 `origincar_bringup.launch.py` 自动发布。
> 地图验证统一使用 `dg_pkg` 的自定义 Nav2，不再直接启动官方默认
> `nav2_bringup bringup_launch.py`。

**启动前清环境**（每次重新启动必做）：

```bash
sudo pkill -9 -f ros2
ros2 daemon stop && ros2 daemon start
```

```bash
# === RDK 终端1: 底盘（沿用 SLAM 建图配置）===
source /opt/tros/humble/setup.bash && source /userdata/dev_ws/install/setup.bash
ros2 launch origincar_base origincar_bringup.launch.py carto_slam:=true

# === RDK 终端2: 自定义 EKF（发布 odom → base_footprint）===
source /opt/tros/humble/setup.bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file /userdata/dev_ws/src/ekf_no_imu_yaw.yaml \
  -r odometry/filtered:=odom_combined

# === RDK 终端3: N10 雷达 ===
source /opt/tros/humble/setup.bash && source /userdata/dev_ws/install/setup.bash
ros2 launch lslidar_driver lsn10_launch.py

# === RDK 终端4: 项目自定义 Nav2 ===
source /opt/tros/humble/setup.bash && source /userdata/dev_ws/install/setup.bash
ros2 launch dg_pkg dg_nav2.launch.py map:=/root/maps/competition_map.yaml

# === RDK 终端5: RViz2 可视化 ===
source /opt/tros/humble/setup.bash && source /userdata/dev_ws/install/setup.bash
rviz2 -d /root/slam.rviz
```

**TF 树**（标准 Nav2 结构）：

```
map → odom → base_footprint → base_link → laser
  ↑     ↑          ↑              ↑          ↑
AMCL   EKF      静态TF          静态TF     静态TF
```



### 启动后操作

在 RViz2 中：

1. Fixed Frame 设为 `map`，添加 Map、LaserScan、TF、RobotModel、Path
2. 先用 **"2D Pose Estimate"** 点击小车在地图上的实际位置和朝向（激活 AMCL）
3. 再用 **"Navigation2 Goal"** 设目标点，验证路径规划



### 8.1 常见问题


| 现象                                                      | 原因                              | 解决                                                                                                                                          |
| ------------------------------------------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| `frame 'odom' does not exist`                           | EKF 未启动或 `odom_frame` 不是 `odom` | 确认 EKF 终端带 `-r odometry/filtered:=odom_combined`；确认 `grep odom_frame /userdata/dev_ws/src/ekf_no_imu_yaw.yaml` 输出 `odom` 而非 `odom_combined` |
| `frame 'map' does not exist`                            | AMCL 未收到初始位姿                    | 在 RViz2 用 **"2D Pose Estimate"** 点击小车位置和朝向                                                                                                  |
| `Timed out waiting for transform from base_link to map` | 同上一项，AMCL 还没发布 `map→odom`       | 同上，设置初始位姿后自动消失                                                                                                                              |
| RViz2 只有雷达图没有地图                                         | Fixed Frame 是 `laser`           | Global Options → Fixed Frame 改为 `map`，再 Add → Map（topic: `/map`）                                                                            |
| Nav2 一切正常但路径规划偶尔失败                                      | AMCL 粒子滤波收敛需要好的初始位姿             | 确保 "2D Pose Estimate" 位置和朝向尽量准确；或让小车稍移动几厘米辅助收敛                                                                                              |
| **PC 改了 YAML 但 RDK 上不生效**                               | RDK 和 PC 文件不同步，RDK 上跑的是旧文件      | **每次改参数后先在 RDK 上** `cat` **确认内容**，不要假设文件已同步                                                                                                 |




### 8.2 调试方法论（逐层验证）

> **核心原则**：导航失败时不要猜测，逐层验证节点→话题→TF 每一步是否正常。
> 不要跳过任何一层，每一层通过后才进入下一层。


| 层级          | 新增内容                                                           | 关键验证命令                                                                          |
| ----------- | -------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| L1 底盘       | 基础节点 + 静态 TF                                                   | `ros2 node list`, `ros2 topic list`, `ros2 run tf2_tools view_frames`           |
| L2 EKF      | `odom→base_footprint` TF + `/odom_combined` 话题                 | `ros2 topic echo /odom_combined --once | grep frame_id`（必须是 `odom`）             |
| L3 雷达       | `/scan` 话题                                                     | `ros2 topic hz /scan`（应 ~10Hz）                                                  |
| L4 laser TF | `base_link→laser` TF（由修正后的 `origincar_bringup.launch.py` 自动发布） | `timeout 5 ros2 run tf2_ros tf2_echo base_link laser`（确认约为 `[0.036, 0, 0.155]`） |
| L5 Nav2     | map_server + AMCL + planner/controller                         | `ros2 node list | grep -E "map|amcl"`, `ros2 topic echo /map --once`            |
| L6 初始位姿     | `map→odom` TF（AMCL 发布）                                         | RViz2 "2D Pose Estimate" 后 `ros2 run tf2_tools view_frames`                     |
| L7 导航目标     | 路径规划 + 运动控制                                                    | RViz2 "Navigation2 Goal"                                                        |


**每一层的预期 TF 树**：

```
L1: (无 odom 帧)
L2: odom → base_footprint → base_link → (无 laser)
L3: odom → base_footprint → base_link → (无 laser，但 /scan 有数据)
L4: odom → base_footprint → base_link → laser
L6: map → odom → base_footprint → base_link → laser   ← 完整链路
```

---



## 9. 一键启动脚本（可选）

将以下内容保存为 `~/slam_mapping.sh`，实现一键启动：

```bash
#!/bin/bash
# SLAM 建图一键启动脚本（使用自定义 EKF 避免 IMU 陀螺零偏导致地图旋转）

echo "===== OriginCar SLAM 建图 ====="

# 环境配置
source /opt/tros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash

# 创建地图目录
mkdir -p ~/maps

# 串口赋权
sudo chmod 777 /dev/ttyACM0 2>/dev/null
sudo chmod 777 /dev/ttyACM1 2>/dev/null

# 使用 tmux 分屏启动（需先安装 tmux: sudo apt install tmux -y）
SESSION="slam_mapping"

# 窗口1: 底盘（carto_slam:=true 禁用默认EKF）
tmux new-session -d -s $SESSION -n "chassis"
tmux send-keys -t $SESSION:chassis "source /opt/tros/humble/setup.bash && source /userdata/dev_ws/install/setup.bash && ros2 launch origincar_base origincar_bringup.launch.py carto_slam:=true" C-m

# 窗口2: 自定义EKF（禁用IMU yaw，避免陀螺零偏导致地图旋转）
tmux new-window -t $SESSION -n "ekf"
tmux send-keys -t $SESSION:ekf "sleep 2 && source /opt/tros/humble/setup.bash && ros2 run robot_localization ekf_node --ros-args --params-file /userdata/dev_ws/src/ekf_no_imu_yaw.yaml -r odometry/filtered:=odom_combined" C-m

# 窗口3: N10 激光雷达
tmux new-window -t $SESSION -n "lidar"
tmux send-keys -t $SESSION:lidar "sleep 4 && source /opt/tros/humble/setup.bash && source /userdata/dev_ws/install/setup.bash && ros2 launch lslidar_driver lsn10_launch.py" C-m

# 窗口4: SLAM Toolbox
tmux new-window -t $SESSION -n "slam"
tmux send-keys -t $SESSION:slam "sleep 6 && source /opt/tros/humble/setup.bash && ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/userdata/dev_ws/src/slam_toolbox_params.yaml" C-m

# 窗口5: 键盘遥控
tmux new-window -t $SESSION -n "keyboard"
tmux send-keys -t $SESSION:keyboard "sleep 8 && source /opt/tros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard" C-m

# 窗口6: RViz2（RDK需接显示器）
tmux new-window -t $SESSION -n "rviz"
tmux send-keys -t $SESSION:rviz "sleep 8 && source /opt/tros/humble/setup.bash && source /userdata/dev_ws/install/setup.bash && rviz2 -d /userdata/dev_ws/install/lslidar_driver/share/lslidar_driver/rviz/lslidar_x10.rviz" C-m

echo "SLAM 建图已启动（tmux session: $SESSION）"
echo "  连接: tmux attach -t $SESSION"
echo "  保存地图: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '/root/maps/competition_map'}}\""
echo "  退出: Ctrl+B 然后 D（detach）"

tmux attach -t $SESSION
```

```bash
chmod +x ~/slam_mapping.sh
./slam_mapping.sh
```

---



## 10. 常见问题排查


| 问题                        | 原因                                                            | 解决方法                                                                                                                                        |
| ------------------------- | ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| N10 雷达无 `/scan` 数据        | 串口权限不足或端口不对                                                   | `sudo chmod 777 /dev/ttyACM1`，检查 `lslidar_n10_uart.yaml` 中 `serial_port` 和 `publish_scan: true`                                             |
| `open_port ERROR`         | 串口被占用或权限不足                                                    | `sudo chmod 777 /dev/ttyACM1`；确认无其他进程占用串口                                                                                                   |
| 雷达数据断断续续                  | USB 供电不足 / 波特率不对                                              | N10 波特率=230400；使用独立供电 USB Hub                                                                                                               |
| `slam_toolbox` 报 TF 超时    | EKF 未启动，或 `slam_toolbox_params.yaml` 中 `odom_frame` 与 EKF 不一致 | **第一步**：`grep odom_frame /userdata/dev_ws/src/slam_toolbox_params.yaml`（必须是 `odom`，不能是 `odom_combined`）；**第二步**：确认 EKF 运行且 `frame_id: odom` |
| 地图重影/错位                   | 雷达安装位置与 TF 不一致                                                | 先确认 `base_link → laser` 为 `(0.036,0,0.155)`；如仍错位，再按实车微调 `origincar_bringup.launch.py` 并重建 `origincar_base`                                  |
| 建图时机器人定位漂移                | 里程计累积误差                                                       | 确保 EKF 正常运行，检查 `/imu/data_raw` 和 `/odom` 话题                                                                                                 |
| **地图持续旋转/雷达图旋转**          | IMU 陀螺零偏导致 EKF yaw 漂移                                         | 使用 `ekf_no_imu_yaw.yaml`（禁用 IMU yaw）+ `carto_slam:=true` 启动底盘，详见 6.2 节                                                                      |
| 键盘不能控制小车                  | `cmd_vel` 话题不通                                                | 检查底盘串口 `/dev/ttyACM0`，确认 `base_serial` 节点运行                                                                                                 |
| 编译报错 `diagnostic_updater` | 缺少依赖                                                          | `sudo apt install ros-$ROS_DISTRO-diagnostic-updater -y`                                                                                    |
| 修改参数后不生效                  | 未重新编译                                                         | 修改 YAML 后执行 `colcon build --packages-select lslidar_driver && source install/setup.bash`                                                    |
| 地图保存失败                    | `nav2_map_server` 未安装                                         | `sudo apt install ros-${ROS_DISTRO}-nav2-map-server -y`                                                                                     |
| N10 不旋转                   | 供电问题                                                          | 检查 USB 供电是否足够（≥5V 1A），尝试换 USB 口                                                                                                             |


---



## 11. 总结

**无需新建 ROS2 功能包**，利用已有的 `lslidar_ros2`（雷达）和 `origincar`（底盘）两个功能包，配合 TogetheROS.Bot 自带的 `slam_toolbox`，即可完成 SLAM 建图。

**核心启动命令**：

```bash
# 终端1: 底盘（禁用默认EKF）
ros2 launch origincar_base origincar_bringup.launch.py carto_slam:=true

# 终端2: 自定义EKF（禁用IMU yaw，避免陀螺零偏导致地图旋转）
ros2 run robot_localization ekf_node --ros-args \
  --params-file /userdata/dev_ws/src/ekf_no_imu_yaw.yaml \
  -r odometry/filtered:=odom_combined

# 终端3: 雷达
ros2 launch lslidar_driver lsn10_launch.py

# 终端4: SLAM
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/userdata/dev_ws/src/slam_toolbox_params.yaml

# 终端5: 键盘遥控（RDK端）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 终端6: RViz2 可视化（RDK需接显示器）
rviz2 -d /userdata/dev_ws/install/lslidar_driver/share/lslidar_driver/rviz/lslidar_x10.rviz

# 保存地图
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/root/maps/competition_map'}}"
```

> **参考资料**:
>
> - [TogetheROS.Bot SLAM 建图](https://developer.d-robotics.cc/tros_doc/apps/slam)
> - [TogetheROS.Bot Navigation2](https://developer.d-robotics.cc/tros_doc/apps/navigation2)
> - [OriginCar N10 雷达文档](https://origincar.guyuehome.com/application/n10_lidar_ros2_package_usage/)
> - [Slam Toolbox 官方文档](https://github.com/SteveMacenski/slam_toolbox)

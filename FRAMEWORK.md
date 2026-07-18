 # 地瓜小车竞赛项目 — 总框架文档
 
 > **维护者**: Codex CLI（框架总控）
 > **开发者**: Claude Code（代码实现）
 > **状态**: 框架 v1.1（已修正竞赛流程、话题标准、技术方案）
 
 ---
 
## 一、项目概览

### 1.1 竞赛信息
 
 | 项目 | 内容 |
 |------|------|
 | 竞赛 | 第21届全国大学生智能汽车竞赛 — 地瓜机器人赛项 |
 | 场景 | 智慧医疗场景 |
| 平台 | OriginCar（阿克曼底盘） |
| 主控 | RDK X5 (aarch64, Linux 6.1.83) |
| ROS 发行版 | TogetheROS.Bot 3.2.3 (Humble) |
| 雷达 | 轮趣 N10 激光雷达（串口版） |
| 工作空间 | /userdata/dev_ws/（小车端） / src/（开发端） |
| 轴距 | 实测约 0.14m |
| 车体尺寸 | 长 0.28m，宽 0.185m |
| 雷达安装 | 距车头 0.12m，距车尾 0.16m，左右居中；相对车体中心 x 约 +0.020m |

### 1.1.1 当前规则与罚时依据

当前项目按用户在 2026-07-14 提供的规则细则执行，完整记录见
`src/比赛规则与罚时记录.md`。关键约束如下：

- 比赛总时限 180 秒；裁判发出开始指令后计时，车模满足完赛条件后结束计时。
- 场内选手不得交流二维码内容、行驶方向或任务提示；违反每次罚时 20 秒。
- 子任务一：车模必须全程自主运行，自主识别二维码，并展示原始数字和顺/逆时针方向；失败罚 50 秒。
- 子任务二：二维码奇数顺时针、偶数逆时针；必须经指定通道进入 C 区，只能在黄色环形通道内行驶，不能驶入浅绿色诊疗室区域；必须识别人形立牌、生成描述并通过屏幕或语音展示；失败罚 50 秒。
- 子任务三：车模必须自主返回 P 区，自动停止，停止后车身至少二分之一位于 P 区域内并保持静止；失败罚 50 秒。
- 障碍物罚时：碰撞一次通常罚 10 秒；推动障碍物大于等于 1m 罚 30 秒；子任务一障碍罚时累计最高 50 秒，子任务二/三各最高 30 秒。
 
 ### 1.2 ROS2 包结构
 
 ```
 src/
 |-- dg_pkg/                  # [核心] 竞赛业务逻辑包 (Python)
 |   |-- dg_pkg/
 |   |   |-- __init__.py
 |   |   |-- qrcode_detector.py    # QR码检测节点 (已实现)
 |   |   |-- qr_move.py            # QR前进控制节点 (已实现，测试用)
 |   |   |-- state_machine.py      # [待开发] 任务状态机
 |   |   |-- nav_cruise.py         # [已实现最小闭环] P->QR->P 自主巡航节点
 |   |   |-- vlm_trigger.py        # [待开发] hobot_llamacpp 触发器
 |   |   |-- lane_patrol.py        # [待开发] 黄色通道巡线
 |   |   |-- screen_display.py     # [待开发] HDMI屏幕显示节点
 |   |-- launch/
 |   |   |-- dg_competition.launch.py  # [已实现最小闭环] P->QR->P 启动文件
 |   |   |-- dg_debug.launch.py        # [待开发] 调试启动
 |   |-- config/
 |   |   |-- competition_params.yaml   # [待开发] 竞赛参数配置
 |   |-- package.xml
 |   |-- setup.py
 |
 |-- lslidar_ros2/            # 激光雷达驱动 (C++) -- 已有，无需修改
 |-- origincar/               # 底盘驱动 + 描述 -- 已有，无需修改
 |
 |-- ekf_no_imu_yaw.yaml      # EKF 配置（禁用IMU yaw，SLAM建图用）
 |-- ekf_calibrated.yaml      # EKF 配置（校准IMU后启用yaw）
 |-- imu_calibrator.py        # IMU零偏校准节点
 |-- SLAM建图操作指南.md      # SLAM建图完整操作文档
 |-- slam_toolbox_params.yaml # [待创建] slam_toolbox 参数
 ```
 
 ### 1.3 现有代码评估
 
 | 文件 | 状态 | 说明 |
 |------|------|------|
 | qrcode_detector.py | OK 可用 | QR码检测 + 方向判定 + 防抖，功能完整 |
 | qr_move.py | OK 可用 | 收到信号后前进指定距离，测试用 |
 | imu_calibrator.py | OK 可用 | IMU零偏校准，发布 /imu/data_calibrated |
 | ekf_no_imu_yaw.yaml | OK 可用 | SLAM建图模式EKF |
 | ekf_calibrated.yaml | OK 可用 | 校准后IMU的EKF |
 | slam_toolbox_params.yaml | 失败 未创建 | 文档中有完整内容，需创建文件 |
 
 评估结论: 基础驱动层完整，业务层只有QR检测两个节点，竞赛全流程框架完全缺失。
 
 ---
 
 ## 二、竞赛流程与状态机设计
 
 ### 2.1 竞赛流程（智慧医疗场景）
 
 子任务1: P点 -> QR任务发布点
   P点(发车点) --- Nav2导航 ---> QR点（A区右上角）
     到达后停靠 -> 二维码扫描 -> 获取方向（1~9999 奇数=CW，偶数=CCW）
     方向决定了黄色通道巡行方向（顺时针/逆时针）
 
 子任务2: 黄色通道巡行 + VLM图生文
   进入通道入口 -> 沿黄色通道巡行一周
     遇到人形立牌
     -> YOLO检测到人形立牌, 截取图像
     -> hobot_llamacpp（InternVL3 1B）端侧推理
     -> /vlm/person_description 得到文字描述
     -> HDMI屏幕显示播报内容
 
 子任务3: 返回P点
   离开通道 --- Nav2导航 ---> 回到P点 -> 精确停车
   竞赛完成
 
 ### 2.2 状态机定义
 
 状态机是竞赛流程的核心，定义了完整的生命周期：
 
 | 状态 | 名称 | 入口操作 | 出口条件 |
 |------|------|----------|----------|
 | INIT | 初始化 | 节点启动 | 收到 start 信号 |
 | TASK1_NAV_TO_QR | 导航到QR点 | 发布 Nav2 goal 到 QR | 到达QR点范围内 |
 | TASK1_SCAN_QR | 扫描二维码 | 订阅 /qr_code/data | 收到方向 (CW/CCW) |
 | TASK2_ENTER_CHANNEL | 进入通道 | 导航至通道入口 | 到达入口 |
 | TASK2_PATROL | 通道巡行 | 巡线/路径点跟随 | 检测到人形立牌 |
 | TASK2_VLM | VLM图生文 | hobot_llamacpp 推理 | 收到描述结果 |
 | TASK2_DISPLAY | 屏幕显示 | HDMI显示描述结果 | 显示完成 |
 | TASK3_RETURN | 返回P点 | Nav2导航回P | 到达P点范围内 |
 | TASK3_PARK | 精确停车 | 调整位姿+停车 | 停车到位 |
 | FINISH | 完成 | 日志+等待复位 | -- |
 | ERROR | 错误处理 | 停车+日志 | 手动恢复 |
 
 ### 2.3 话题接口（使用教程标准命名）
 
 ```
 dg_pkg 内部通信:
  /competition/state      (std_msgs/String)     状态机当前状态
  /qr_code/data           (std_msgs/String)     二维码解码原始文本
  /vlm/person_description (std_msgs/String)     VLM图生文描述结果
  /display/text           (std_msgs/String)     HDMI屏幕显示文字
 
 dg_pkg <-> 底盘层:
  /cmd_vel            (geometry_msgs/Twist)  速度指令（底盘订阅）
  /odom_combined      (nav_msgs/Odometry)    融合里程计（EKF输出）
 
 dg_pkg <-> 感知层:
  /scan               (sensor_msgs/LaserScan)        雷达扫描（SLAM输入）
  /image              (sensor_msgs/CompressedImage)  相机图像（QR + VLM输入）
  /hobot_dnn_detection (DnnDetectResult)             YOLO目标检测结果
 
 dg_pkg -> Nav2:
  /initialpose        (geometry_msgs/PoseWithCovarianceStamped)  初始位姿
  /goal_pose          (geometry_msgs/PoseStamped)                 导航目标
 
 Nav2 -> dg_pkg:
  /amcl_pose          (geometry_msgs/PoseWithCovarianceStamped)  定位结果
 
 兼容话题（保留现有代码）:
  /sign_foxglove      (std_msgs/Int32)     QR检测信号：1~9999 奇数发布 3=CW，偶数发布 4=CCW
 ```
 
 ---
 
 ## 三、模块详细设计

### 3.1 [M1] 任务状态机 -- state_machine.py
 
 ### 3.10 [M10] Navigation2 导航栈搭建 -- dg_nav2.launch.py + nav2_params.yaml
 
 **开发者**: Claude Code
 **优先级**: P0（Nav2 导航基础设施，阻塞 M2 自主巡航）

 **用户决策（2026-06-24）**:
 - M10 的自定义 Nav2 文件统一放入 `dg_pkg`
 - 不修改 `origincar` 和 `lslidar_ros2`
- 底盘、EKF、雷达和 `base_link -> laser` 静态 TF 由外部终端先行启动
 - M10 不得重复启动硬件节点或创建第二套 Nav2 生命周期管理器
 
 #### 接口规格
 
 新建文件:
 
 | 文件 | 说明 |
 |------|------|
 | `dg_pkg/launch/dg_nav2.launch.py` | 包装官方 `nav2_bringup` 的导航启动文件 |
 | `dg_pkg/config/nav2_params.yaml` | OriginCar 阿克曼底盘 Nav2 参数 |
 
 #### 启动边界
 
`dg_nav2.launch.py` 只负责：
 
1. 声明 `map`、`params_file`、`use_sim_time` 和 `autostart` 参数。
2. Include 官方 `nav2_bringup/bringup_launch.py`。
3. 将地图路径和 `dg_pkg/config/nav2_params.yaml` 传入官方 launch。
4. 由官方 bringup 启动唯一一套 Nav2 节点和生命周期管理器。

明确禁止在 M10 中 Include 或启动：

- `origincar_base`
- `robot_localization/ekf_node`
- `lslidar_driver`
- `static_transform_publisher`
- 额外的 `lifecycle_manager_localization` 或 `lifecycle_manager_navigation`

2026-06-24 实测中，两套 lifecycle manager 曾导致 `map_server` 状态冲突、
AMCL 不激活和 `map` 帧缺失。

#### TF 与话题约定

- 当前 `ekf_no_imu_yaw.yaml` 的 TF 帧是标准 `odom`：
  `map -> odom -> base_footprint -> base_link -> laser`
- 2026-06-26 经用户授权修正 `origincar_bringup.launch.py`：
  - `base_footprint -> base_link = (0, 0, 0)`
  - `base_link -> laser = (0.036, 0, 0.155)`
  - `link_to_laser` 已加入 LaunchDescription
- EKF 输出话题仍为 `/odom_combined`，话题名不等于 TF 帧名。
- Nav2 的 `odom_frame_id`、局部 costmap `global_frame` 使用 `odom`。
- Nav2 的 `odom_topic` 使用 `/odom_combined`。
- 后续如需修改 EKF 的 TF 帧名，必须先由用户确认，并同步全部 Nav2 参数。
 
 #### 关键参数设计
 
 **AMCL**（`nav2_params.yaml` → `amcl`）:
 - 定位模型: likelihood_field（室内环境优于 beam 模型）
 - 粒子数: 500~2000（RDK X5 性能上限）
 - 激光范围: min 0.12m / max 10.0m（适配 N10）
 - 坐标系: global_frame=map, base_frame=base_footprint
 
**Controller**（`nav2_params.yaml` → `controller_server.FollowPath`）:
 - 控制器类型: RegulatedPurePursuitController（非完整约束 / 阿克曼适配）
 - 调试巡航速度: `desired_linear_vel=0.10 m/s`，速度平滑上限 `0.12 m/s`
 - 稳定后比赛巡航候选: `0.18~0.20 m/s`
 - 预瞄距离: `lookahead_dist=0.35m`，动态范围 `0.35~0.70m`，`lookahead_time=0.4`
 - 碰撞检测: 开启
 - 当前 RPP 稳定跟踪档：`regulated_linear_scaling_min_radius=0.4`、`regulated_linear_scaling_min_speed=0.08`、`max_angular_accel=1.5`
 - R11 起主路径允许低速规划倒车：`allow_reversing: true`
 - `use_rotate_to_heading: false`，禁止零线速度原地旋转
 - 倒车策略不是“绝对禁止”，而是“禁止 Nav2 无安全判断地盲倒车”；
   后续可由 `dg_pkg` 任务逻辑在后方安全检测通过时执行受控倒车。

**Planner**（`nav2_params.yaml` → `planner_server.GridBased`）:
- 使用 `nav2_smac_planner/SmacPlannerHybrid`
- R11 起主巡航使用 `motion_model_for_search: REEDS_SHEPP`，允许 Smac Hybrid-A*
  生成前进/倒车混合路径；倒车由规划路径产生，不恢复默认 recovery 盲倒车
- `minimum_turning_radius: 0.35` m
- `angle_quantization_bins: 72`
- `smooth_path: true`
- 实测依据：轴距约 0.14m、最大前轮转角约 30°，
  自行车模型最小半径约 `0.14 / tan(30°) = 0.242m`，工程取 0.25m
- R13 当前值：`reverse_penalty: 8.0`、`change_penalty: 2.0`、`cost_penalty: 3.0`。
  倒车允许但代价更高，优先前进并减少频繁前后换向。
 
**Costmap**（`nav2_params.yaml` → local_costmap / global_costmap）:
 - 局部 costmap: 3m × 3m 滚动窗口，跟随 odom_combined
 - 全局 costmap: 30m × 30m 静态 + 动态障碍
 - footprint: [[-0.15,-0.105],[-0.15,0.105],[0.15,0.105],[0.15,-0.105]]
   （实车 0.28m × 0.185m 外廓加约 1cm 余量）
 - R28 后膨胀半径当前值: 局部 0.25m / 全局 0.30m，
   `cost_scaling_factor: 2.5`（该值是代价衰减系数，不是扩张距离；当前按用户要求提高全局规划层留边，若路径明显绕远或无法穿过足够宽空隙，优先把全局回退到 0.24 或 0.20）
 - R29 障碍观测范围：全局 `obstacle_max_range=4.5m`、`raytrace_max_range=5.0m`；局部保持 `2.5m/3.0m`，只扩大 Smac 可用的远场动态障碍信息，不扩大局部滚动窗口。

**阿克曼安全恢复**:
- 使用 `dg_pkg/behavior_trees/navigate_to_pose_ackermann_safe.xml`
- 使用 `dg_pkg/behavior_trees/navigate_through_poses_ackermann_safe.xml`
- 仅允许清除全局/局部 costmap 和 `wait`
- 禁止默认 `spin` 和默认 `backup`，规划失败后停车并返回失败
- 默认 recovery 仍不允许 `BackUp`；R11 只允许按 Smac/RPP 规划路径低速倒车
- 后续仍可新增更细的“安全倒车”业务动作：检查后方雷达/代价地图净空，
  后方有墙、障碍或未知空间时禁止执行该业务倒车。
 
 **地图路径**: 通过 `map` launch arg 传入，默认 `/root/maps/competition_map.yaml`
 
 #### Python 包安装配置
 
修改 `dg_pkg/setup.py` 的 `data_files`：

- 安装 `launch/*.launch.py` 到 `share/dg_pkg/launch`
- 安装 `config/*.yaml` 到 `share/dg_pkg/config`
- 安装 `behavior_trees/*.xml` 到 `share/dg_pkg/behavior_trees`
 
 #### package.xml 新增依赖
 
`nav2_bringup`、`nav2_regulated_pure_pursuit_controller`、
`nav2_smac_planner`、`launch`、`launch_ros`、`ament_index_python`、
`python3-yaml`。

#### 启动命令

```bash
ros2 launch dg_pkg dg_nav2.launch.py \
  map:=/root/maps/competition_map.yaml
```

#### 验收标准

1. `/map_server` 和 `/amcl` 均为 `active`。
2. `/map` 正常发布，且不存在重复生命周期管理器。
3. `tf2_echo map base_link` 持续成功。
4. 设置前方约 1m 的目标后，`/cmd_vel_nav` 出现 `linear.x > 0`，
   而不是仅有 `angular.z` 的原地旋转。
5. `/cmd_vel_nav -> velocity_smoother -> /cmd_vel -> origincar_base`
   发布订阅链完整。
6. 小车能沿 Path 前进，取消目标后速度归零。
 
 ---
 
 ## 四、启动流程
 
 ### 4.1 竞赛完整启动（多终端）
 
 **开发者**: Claude Code
 **优先级**: P0（阻塞所有后续模块）
 
 - 实现 StateMachine(Node) 类，管理 2.2 节定义的 11 个状态
 - 状态转移：INIT -> TASK1_NAV_TO_QR -> TASK1_SCAN_QR -> TASK2_ENTER_CHANNEL -> TASK2_PATROL -> TASK2_VLM -> TASK2_DISPLAY -> TASK3_RETURN -> TASK3_PARK -> FINISH
 - 每个状态设定 timeout，超时进入 ERROR
 - 错误恢复：ERROR 状态下尝试重试（VLM失败重试3次）
 - 发布当前状态到 /competition/state (std_msgs/String)
 
 ### 3.2 [M2] 自主巡航 -- nav_cruise.py
 
 **开发者**: Claude Code
 **优先级**: P0（自动导航核心）
 
 - 封装 Nav2 导航接口（action: navigate_to_pose）
 - 航点和执行顺序由 `competition_params.yaml` 的 `waypoints` 与
   `routes.route_minimal` 配置；增加普通 Nav2 中间点不改代码
 - waypoint 可选到点动作接口同时兼容直线与定半径弧线：
   `post_arrival_settle_sec`、`post_arrival_forward_distance_m`、
   `post_arrival_forward_speed_mps`、`post_arrival_reverse_distance_m`、
   `post_arrival_reverse_speed_mps`、`post_arrival_turn_radius_m`、
   `post_arrival_turn_direction`、`post_arrival_forward_duration_sec`、
   `post_arrival_reverse_duration_sec`；还可用
   `post_arrival_pre_turn_radius_m`、`post_arrival_pre_turn_angle_deg`、
   `post_arrival_pre_turn_speed_mps`、`post_arrival_pre_turn_direction`
   在直线动作前增加一次定角度弧线转向；设置
   `post_arrival_return_to_pre_turn_pose=true` 后，会在直线后退完成后沿
   相同弧线倒车，恢复转向前的位置与朝向。
 - 当前任务 1+3 路线：P 点 -> QR 点 -> 进通道前 -> P 点。
 - `qr.skip_qr_goal_on_signal=false`：必须先到固定 QR 点并识别有效二维码，再按
   `qr.post_detection_motion` 执行半径 0.35m、速度 0.20m/s 的右后弧倒车
   100°，替代原 `return_mid_after_qr` Nav2 过渡点。
 - 到达通道前点后，先以半径 0.35m、速度 0.20m/s 右转 30°（约0.92s），
   再直行前进 1.00m@0.25m/s、保持零速 1.0s、直行后退
   1.00m@0.25m/s，最后沿同一右转弧线倒车30°，理论上回到转向前。
 - 航点坐标实测后填入 `competition_params.yaml`
 - 导航参数（教程推荐）：
   - 最大线速度: 0.3 m/s，最大角速度: 0.5 rad/s
   - 阿克曼转弯半径: 0.25m，轴距: 0.25m
   - 目标精度: xy 5cm, yaw 5.7度，障碍物膨胀半径: 15cm
   - 到达判定: < arrival_distance（默认 0.3m）
 - 导航超时处理：超过 nav_timeout（默认 120s）触发重试
 
 ### 3.3 [M3] 黄色通道巡线 -- lane_patrol.py
 
 **开发者**: Claude Code
 **优先级**: P0（Task2 核心）
 
 提供两种方案，由 patrol.method 参数控制选择：
 
 方案A（Nav2路径点）:
 - 在 C 区定义 7 个路径点构成环形轨迹
 - Nav2 顺序导航至每个路径点
 - 优点：与导航框架一致，稳定可靠
 
 方案B（视觉巡线）:
 - 使用 YOLO 检测黄色通道边界
 - PID 控制跟随通道中心线
 - 优点：适应性强，不受全局定位漂移影响
 
 YOLO 检测：
 - 使用 hobot_dnn + YOLOv8n（140fps on BPU）
 - 检测类别：cone（锥桶障碍物），person_standee（人形立牌）
 - 检测到人形立牌时，通知状态机进入 VLM 阶段
 
 ### 3.4 [M4] VLM 图生文触发器 -- vlm_trigger.py
 
 **开发者**: Claude Code
 **优先级**: P1（Task2 依赖）
 
 方案: hobot_llamacpp 端侧推理
 - 安装：sudo apt install tros-humble-hobot-llamacpp
 - 模型：InternVL3 1B（推荐，中文能力强）
 - 前提条件：ION 内存需设为 1.6GB（sudo srpi-config）
 - 启动模式：feed_type:=1 订阅摄像头话题 /image
 
 触发器职责：
 - 当检测到人形立牌时，截取当前帧图像
 - 调用 hobot_llamacpp 推理
 - 监听 /vlm/person_description 获取结果
 - 转发结果给屏幕显示模块
 
 提示词：
   系统提示：你是一个医疗巡检机器人。请描述图像中看到的人形立牌内容。
   描述应简洁准确，包含：人物身份（医生/护士/病人/访客）、
   姿势（站立/行走/坐着）、可能的行为。用中文回答，不超过50字。
 
 ### 3.5 [M5] HDMI 屏幕显示 -- screen_display.py
 
 **开发者**: Claude Code
 **优先级**: P1
 
 方案: pygame / tkinter 在 HDMI 显示器上渲染大字号文字
 - 通过 HDMI 接口连接 7寸显示屏
 - 订阅 /display/text 话题接收显示内容
 - 显示内容：竞赛当前状态、VLM 描述结果、倒计时、进度
 - 大字号中文字体渲染，确保可视化效果
 - 显示完成后通过话题通知状态机
 
 ### 3.6 [M6] QR 检测优化 -- qrcode_detector.py（现有代码强化）
 
 **开发者**: Codex CLI 评审 -> Claude Code 修改
 **优先级**: P2
 
评审问题:
 | 问题 | 严重程度 | 说明 |
 |------|----------|------|
 | 话题名不统一 | 中 | 订阅 /image 但旧文档写 /image_raw，统一为 /image；比赛二维码规则为 1~9999 奇数=CW、偶数=CCW |
 | same_count 归零过早 | 低 | publish 后立即归零可能重复触发 |

当前比赛限制：`qrcode_detector.scan_start_delay_sec=15.0`，节点启动后的前
15 秒忽略图像；`dg_competition.launch.py` 可用
`qr_scan_start_delay_sec` 覆盖。
 | 缺少状态检测抑制 | 中 | 检测到后应抑制直到状态机进入 TASK1_SCAN_QR |
 
 ### 3.7 [M7] 一键启动 Launch 文件
 
 **开发者**: Claude Code
 **优先级**: P0
 
 dg_competition.launch.py - 竞赛模式：
 - 启动 QR检测节点
 - 启动状态机节点
 - 启动巡航节点
 - 启动 VLM 触发器
 - 启动屏幕显示节点
 - 依赖底盘、雷达、EKF（通过 IncludeLaunchDescription）
 
 dg_debug.launch.py - 调试模式：
 - 可选启动单个节点
 - 发布调试话题
 
 ### 3.8 [M8] 竞赛参数配置 -- config/competition_params.yaml
 
 ```yaml
 dg_robot:
   waypoints:
     p_point: {x: 0.0, y: 0.0, yaw: 0.0}        # 发车点（待实测）
     qr_point: {x: 0.0, y: 0.0, yaw: 0.0}       # QR任务发布点（待实测）
     channel_entry: {x: 0.0, y: 0.0, yaw: 0.0}  # 通道入口（待实测）
     c_zone_waypoints: []                         # C区路径点（待实测）
   navigation:
     max_linear_speed: 0.3          # 最大线速度 (m/s)
     max_angular_speed: 0.5         # 最大角速度 (rad/s)
     wheelbase: 0.14                # 实测轴距 (m)
     turning_radius: 0.25           # 转弯半径 (m)
     arrival_distance: 0.3          # 到达判定距离 (m)
     goal_accuracy_xy: 0.05         # 目标精度 xy (m)
     goal_accuracy_yaw: 0.1         # 目标精度 yaw (rad)
     local_inflation_radius: 0.25   # 局部代价地图膨胀半径 (m)
     global_inflation_radius: 0.30  # 全局代价地图膨胀半径 (m)
     nav_timeout: 120.0             # 导航超时 (s)
   qr:
     debounce_frames: 5             # QR去抖帧数
   vlm:
     model_path: /root/vlm_models/
     vit_model: vit_model_int16_VL3_1B_Instruct_X5.bin
     llm_model: qwen2_5_q8_0_InternVL3_1B_Instruct.gguf
     user_prompt: 描述这张图片中的人形立牌
     timeout: 30.0
   patrol:
     method: nav_waypoints          # nav_waypoints | visual_line
     detect_person_standee: true
 ```
 
 ---
 
 ## 四、启动流程
 
 ### 4.1 竞赛完整启动（多终端）
 
 按顺序启动：
 
 ```
 赛前: 设置 CPU 性能模式 + ION 内存 1.6GB
 
 终端1: rosbridge WebSocket 服务
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
 
 终端2: 底盘驱动
   ros2 launch origincar_base origincar_bringup.launch.py
 
 终端3: USB 摄像头
   ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_video_device:=/dev/video0
 
 终端4: N10 激光雷达
   ros2 launch lslidar_driver lsn10_launch.py
 
 终端5: IMU 校准 + EKF（或 ekf_no_imu_yaw.yaml）
   python3 /userdata/dev_ws/src/imu_calibrator.py
   ros2 run robot_localization ekf_node --ros-args
     --params-file /userdata/dev_ws/src/ekf_calibrated.yaml
     -r odometry/filtered:=odom_combined
 
 终端6: Navigation2 导航栈（底盘、EKF、雷达、静态TF已由前置终端启动）
   ros2 launch dg_pkg dg_nav2.launch.py map:=/root/maps/competition_map.yaml
 
 终端7: dg_pkg 竞赛节点
   ros2 launch dg_pkg dg_competition.launch.py
 ```
 
 ### 4.2 调试单个节点
 
 ```bash
 ros2 run dg_pkg state_machine --ros-args -p debug:=true
 ```
 
 ---
 
 ## 五、依赖安装清单
 
 在 RDK X5 上执行：
 
 ```bash
 source /opt/tros/humble/setup.bash
 
 # SLAM / 导航
 sudo apt install ros-${ROS_DISTRO}-slam-toolbox -y
 sudo apt install ros-${ROS_DISTRO}-nav2-* -y
 
 # VLM 图生文（hobot_llamacpp）
 sudo apt install tros-humble-hobot-llamacpp -y
 
 # USB 摄像头驱动
 sudo apt install tros-humble-hobot-usb-cam -y
 
 # rosbridge
 sudo apt install ros-${ROS_DISTRO}-rosbridge-suite -y
 
 # 调试工具
 sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard -y
 pip install pyzbar pygame
 ```
 
 ---
 
 ## 六、开发进度跟踪
 
 | 模块 | 状态 | 开发者 | 优先级 | 依赖 |
 |------|------|--------|--------|------|
 | M1 状态机 | 待开发 | Claude Code | P0 | -- |
| M2 自主巡航 | 最小闭环已实现，待板端验收 | Codex CLI | P0 | Nav2 |
 | M3 通道巡线 | 待开发 | Claude Code | P0 | M1 |
 | M4 VLM触发器 | 待开发 | Claude Code | P1 | M1 |
 | M5 屏幕显示 | 待开发 | Claude Code | P1 | M1 |
 | M6 QR优化 | 待评审 | Codex->Claude | P2 | -- |
 | M7 Launch文件 | 待开发 | Claude Code | P0 | M1-M5 |
 | M8 参数配置 | 待开发 | Claude Code | P0 | M1-M5 |
 | M9 SLAM参数文件 | 待创建 | Claude Code | P0 | -- |
 
 ---
 
 *本框架由 Codex CLI 维护 v1.1。技术修改经评审后更新。*
 | M10 Nav2导航栈 | 接口已确认，待Claude在 dg_pkg 实现 | Claude Code | P0 | -- |

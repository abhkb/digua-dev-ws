# Nav2 参数逐项说明

> 对应配置：`src/dg_pkg/config/nav2_params.yaml`
> 适用环境：OriginCar 阿克曼底盘、RDK X5、ROS 2 Humble、N10 激光雷达
> 当前配置快照：2026-07-18
> 本文的“当前值”以源码目录中的 YAML 为准；板端启动时实际读取的是编译安装后的副本。

## 1. 先看懂这份配置的调用关系

当前导航链路是：

```text
地图 + /scan + /odom_combined
        ↓
AMCL 定位（map → odom）
        ↓
SmacPlannerHybrid 全局规划（可前进、可倒车）
        ↓
RegulatedPurePursuitController 跟踪路径
        ↓
velocity_smoother 限速和限制加减速度
        ↓
/cmd_vel → origincar_base
```

必须同时注意以下三点：

1. `dg_nav2.launch.py` 会把地图路径和两棵阿克曼安全行为树动态注入临时参数文件，所以 YAML 中部分空字符串不是漏填。
2. `nav_cruise.py` 的定时开环动作直接发布 `/cmd_vel`，不经过 Nav2 控制器和 `velocity_smoother`；这些动作的速度应在 `competition_params.yaml` 中调整。
3. 当前安全行为树只执行规划和路径跟踪，没有调用 `SmoothPath`，因此 `smoother_server` 虽然处于 active，当前最小巡航流程中并未实际平滑路径。

## 2. 修改效果速查

| 想达到的效果 | 优先查看 | 主要代价或风险 |
|---|---|---|
| 提高直线巡航速度 | `desired_linear_vel`、`max_velocity[0]` | 制动距离增大，局部避障余量减少 |
| 提高倒车速度 | `min_velocity[0]` 的绝对值 | 倒车碰撞风险和定位误差增大 |
| 提高转弯速度 | `regulated_linear_scaling_min_radius`、`regulated_linear_scaling_min_speed` | 更容易甩尾、贴障碍或偏离路径 |
| 减少急转和频繁改向 | 增大前视距离、提高 `change_penalty`、降低重规划频率 | 绕障反应可能变慢，路线可能稍长 |
| 更早看见远处随机障碍 | 全局 `obstacle_max_range`、`raytrace_max_range` | CPU、代价地图更新量和远场噪点增加 |
| 离障碍更远 | `inflation_radius` 增大或 `cost_scaling_factor` 减小 | 可通行空间变窄，窄通道可能无解 |
| 到点更精确 | 减小 `xy_goal_tolerance`、`yaw_goal_tolerance` | 阿克曼车更容易在终点前后来回修正或超时 |
| 定位更及时贴墙 | 减小 AMCL `update_min_d/a`、增加 `max_beams`/粒子数 | CPU 占用增加，噪声也可能更快进入定位结果 |

## 3. 通用参数

### `use_sim_time: false`

该参数在 AMCL、BT Navigator、控制器、代价地图、地图服务、规划器、平滑器、行为服务器、机器人状态发布器、航点跟随器和速度平滑器中重复出现。

- 用途：决定节点使用真实系统时间还是 `/clock` 仿真时间。
- 当前值：`false`，实车必须使用系统时间。
- 改为 `true`：只有在 Gazebo、rosbag 回放等持续发布 `/clock` 的环境中才应使用；实车没有 `/clock` 时，TF、传感器消息和导航状态可能全部停住或超时。
- 项目变化：实车配置始终保留 `false`，没有调参需求。

## 4. AMCL 定位参数

### 4.1 坐标系、模型和发布

| 参数 | 当前值 | 用途 | 修改后的变化 / 本项目说明 |
|---|---:|---|---|
| `base_frame_id` | `base_footprint` | AMCL 认为的机器人二维基座坐标系 | 必须与 TF、Nav2 footprint 和里程计 child frame 一致。错用 `base_link` 会造成 TF 树不一致或位姿偏移。本项目已统一为 `base_footprint`。 |
| `global_frame_id` | `map` | AMCL 定位结果所在的全局坐标系 | 改动会破坏标准 `map → odom → base_footprint` 链路，当前不要改。 |
| `odom_frame_id` | `odom` | AMCL 使用的里程计坐标系 | `/odom_combined.header.frame_id` 已统一为 `odom`。改成 `odom_combined` 会再次造成 TF 断树。 |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | 根据里程计增量预测粒子运动 | 当前底盘驱动发布的是二维里程计，项目实测该模型稳定。切换 Omni 模型会引入横移假设，不适合当前无横移底盘。 |
| `tf_broadcast` | `true` | 允许 AMCL 发布 `map → odom` | 关闭后必须有别的唯一节点发布该 TF，否则 RViz 中车体和地图断开。 |
| `transform_tolerance` | `1.0 s` | 将 AMCL 发布的 TF 向未来容忍一段时间 | 增大可缓解短暂延迟，但掩盖时间戳问题；减小可更严格，但计算抖动时更容易报 TF extrapolation。 |
| `set_initial_pose` | `true` | 节点启动时使用下方 `initial_pose` | 当前固定从 P 点 `(0,0,0)` 启动。关闭后通常要人工或程序发布 `/initialpose`。 |
| `initial_pose.x/y/z/yaw` | `0/0/0/0` | AMCL 启动初始位姿，单位为 m、m、m、rad | `x/y/yaw` 必须和建图时固定 P 点坐标一致；`z` 在二维定位中通常保持 0。改错会使 `/scan` 初始不贴墙。 |
| `save_pose_rate` | `0.5 Hz` | 最多每秒保存 AMCL 最佳位姿的频率 | `0.5` 约为每 2 秒一次。调高增加参数写入频率；当前任务主要依赖固定 initial pose，不需要提高。 |

### 4.2 里程计运动噪声

| 参数 | 当前值 | 用途 | 调大 / 调小的效果 |
|---|---:|---|---|
| `alpha1` | `0.2` | 旋转运动引起的旋转估计噪声 | 调大：粒子角度扩散更宽，更不信任转向里程计；调小：更信任里程计 yaw。 |
| `alpha2` | `0.2` | 平移运动引起的旋转估计噪声 | 调大：直行时也允许更多角度漂移；调小：假定直行 yaw 更可靠。 |
| `alpha3` | `0.2` | 平移运动引起的平移估计噪声 | 调大：位置粒子扩散更宽；调小：更信任行驶距离。 |
| `alpha4` | `0.2` | 旋转运动引起的平移估计噪声 | 调大：转弯时位置不确定性更高；调小：更信任转弯里程计轨迹。 |
| `alpha5` | `0.2` | 全向模型中的横移噪声项 | 当前 Differential 模型基本不依赖横移能力，保留值即可。 |

这些值目前保持统一的 `0.2`。若以后重新融合 IMU 或校准底盘里程计，必须用实测数据整体调整，不能只凭 RViz 画面随意减小。

### 4.3 激光模型和量测权重

| 参数 | 当前值 | 用途 | 修改后的变化 / 本项目说明 |
|---|---:|---|---|
| `laser_model_type` | `likelihood_field` | 使用似然场比较激光点与地图障碍距离 | 室内墙面定位较平滑；切换 `beam` 会更精细地模拟单束激光，但通常更耗算力且更敏感。 |
| `laser_min_range` | `0.12 m` | 小于该距离的激光量测不用于定位 | 应与 N10 近距离可靠范围一致。调大可过滤车体/近场噪声，但会丢失贴墙信息。 |
| `laser_max_range` | `10.0 m` | 大于该距离的量测按最大量程处理或忽略 | 调小降低远场噪声和计算量；调大只有雷达确实可靠时才有意义。 |
| `laser_likelihood_max_dist` | `1.0 m` | 似然场中障碍距离影响的最大范围 | 已从较小范围提高到 `1.0`，让稍有偏移的 scan 仍能给出渐进定位概率；过大可能降低墙体区分度。 |
| `max_beams` | `120` | 每帧最多抽样多少束激光参与 AMCL | 项目从约 `60` 提高到 `120`，提高墙面约束和转弯定位稳定性；代价是 AMCL CPU 增加。 |
| `sigma_hit` | `0.12 m` | 命中障碍模型的高斯标准差 | 项目调到 `0.12`，较小值更严格要求 scan 贴墙，较大值更容忍误差但定位会变钝。 |
| `lambda_short` | `0.1` | 意外短量程模型的指数衰减率 | 增大使短量程概率衰减更快；主要影响 `beam`/短量程模型，当前一般保留。 |
| `z_hit` | `0.7` | 正常命中地图障碍的权重 | 已调整为主要权重。增大更依赖地图墙面，地图准确时定位更紧；地图变化大时可能更易失配。 |
| `z_rand` | `0.25` | 随机量测权重 | 已提高抗路障、人员和噪点能力；过大则 scan 与地图不贴也可能被接受。 |
| `z_max` | `0.05` | 最大量程读数权重 | 增大表示更相信“未测到障碍”的最大量程读数。 |
| `z_short` | `0.05` | 意外短量程读数权重 | 增大可容忍临时近障碍，但也可能削弱地图墙面对定位的约束。 |

### 4.4 Beam Skip（当前关闭）

| 参数 | 当前值 | 用途 | 生效条件和变化 |
|---|---:|---|---|
| `do_beamskip` | `false` | 是否跳过与大多数粒子都不一致的激光束 | 当前关闭，因此下面三个 beam skip 阈值不参与计算。开启可抗动态障碍，但设置不当会丢失关键墙面。 |
| `beam_skip_distance` | `0.5 m` | 激光端点距地图障碍超过该值时视为可能不匹配 | 仅 `do_beamskip=true` 生效；调小更容易跳束。 |
| `beam_skip_threshold` | `0.3` | 当不匹配粒子比例达到阈值时决定跳过该束 | 仅 beam skip 开启时生效；调小会跳过更多束。 |
| `beam_skip_error_threshold` | `0.9` | 跳束过多时触发错误保护、恢复使用全部束的阈值 | 调小更早停止 beam skip，调大更容忍大量不匹配。 |

### 4.5 粒子滤波更新

| 参数 | 当前值 | 用途 | 修改后的变化 / 本项目说明 |
|---|---:|---|---|
| `min_particles` | `800` | 自适应采样的最少粒子数 | 项目由约 `500` 提高到 `800`，减少走动时姿态漂移；CPU 占用提高。 |
| `max_particles` | `3000` | 自适应采样的最多粒子数 | 项目由约 `2000` 提高到 `3000`，定位不确定时允许更广搜索；峰值计算量提高。 |
| `pf_err` | `0.05` | KLD 自适应采样允许的粒子分布误差 | 调小通常需要更多粒子、精度更高；调大可降算力但粒子近似更粗。 |
| `pf_z` | `0.99` | KLD 采样置信度 | 调高通常需要更多粒子；调低减少计算但置信保证降低。 |
| `resample_interval` | `1` | 每多少次滤波更新重采样一次 | `1` 表示每次更新都重采样。增大可省算力，但粒子权重退化保持更久。 |
| `update_min_d` | `0.05 m` | 平移至少多少才进行一次滤波更新 | 已减小到 5 cm，使连续短距离运动也及时用 scan 修正；调得更小会增加 CPU 和对噪声的敏感度。 |
| `update_min_a` | `0.05 rad` | 旋转至少多少才更新，约 `2.86°` | 已减小以提高转弯定位刷新速度；太小会让静止抖动也频繁更新。 |
| `recovery_alpha_fast` | `0.0` | 快速平均权重衰减率，用于随机粒子恢复 | `0` 表示关闭该恢复机制。增大可在绑架/严重失定位时注入随机粒子，但正常运行可能抖动。 |
| `recovery_alpha_slow` | `0.0` | 慢速平均权重衰减率 | 同上，当前关闭。若启用必须与 fast 配对调试。 |

`amcl_map_client.use_sim_time` 和 `amcl_rclcpp_node.use_sim_time` 只是 AMCL 相关辅助节点的时钟选择，当前同样为 `false`。

## 5. BT Navigator 行为树参数

| 参数 | 当前值 | 用途 | 修改后的变化 / 本项目说明 |
|---|---:|---|---|
| `global_frame` | `map` | 导航目标和行为树使用的全局坐标系 | 必须与 AMCL/global costmap 一致。 |
| `robot_base_frame` | `base_footprint` | 行为树查询机器人当前位姿的基座帧 | 必须与 TF 主链一致。 |
| `odom_topic` | `/odom_combined` | BT 中需要里程计信息的节点所订阅的话题 | 当前为禁用 IMU yaw 后的 EKF 输出。话题中断会影响导航行为和速度判断。 |
| `default_nav_to_pose_bt_xml` | `""` | NavigateToPose 默认 BT 文件路径 | 空值由 `dg_nav2.launch.py` 动态替换为 `navigate_to_pose_ackermann_safe.xml`。直接绕过本项目 launch 启动可能加载默认 BT，不安全。 |
| `default_nav_through_poses_bt_xml` | `""` | NavigateThroughPoses 默认 BT 路径 | 同样由 launch 注入；当前 `navigators` 仅启用 NavigateToPose，这一项是兼容保留。 |
| `bt_loop_duration` | `10 ms` | 行为树 tick 周期 | 减小响应更快但 CPU 增加；增大可省算力但取消、恢复和状态响应变慢。 |
| `default_server_timeout` | `20 ms` | BT ROS action/service 节点的默认等待超时 | 过小可能在板端负载高时误超时；过大则故障响应变慢。 |
| `wait_for_service_timeout` | `1000 ms` | 启动时等待所需服务的超时 | 当前 1 秒。若 Nav2 节点启动慢可适当提高，但应先排查 lifecycle/TF。 |
| `action_server_result_timeout` | `900.0 s` | action 结果在内部保留/等待的时间限制 | 为长任务保留较大余量。它不是 `nav_cruise` 每个目标的任务超时；后者由竞赛参数控制。 |
| `navigators` | `[navigate_to_pose]` | 注册哪些 BT Navigator action | 当前最小巡航只需要单目标导航，减少不需要的 navigator。 |
| `navigate_to_pose.plugin` | `nav2_bt_navigator::NavigateToPoseNavigator` | 单目标导航插件类型 | 接口绑定项，不应随意修改。 |
| `error_code_names` | 4 个错误名 | 行为树共享黑板中可记录/传播的错误码名称 | 增删必须与 BT 节点实际使用的端口一致；当前保留 `unknown`、`patience_exceeded`、`tf_error`、`goal_tolerance_violated`。 |

当前安全 BT 约每 3 秒重新调用一次全局规划器。频率不在本 YAML，而在：

- `src/dg_pkg/behavior_trees/navigate_to_pose_ackermann_safe.xml`
- `src/dg_pkg/behavior_trees/navigate_through_poses_ackermann_safe.xml`

其中 `RateController hz="0.333"`：增大时更频繁适应新障碍，但路径跳变和 CPU 占用增加；减小时路径更稳定，但绕开新出现障碍更慢。

## 6. Controller Server 与目标判定

### 6.1 服务器通用参数

| 参数 | 当前值 | 用途 | 调整效果 |
|---|---:|---|---|
| `controller_frequency` | `20.0 Hz` | 计算速度指令的频率 | 调高控制更细但 CPU 增加；调低会使转弯和避障反应变粗。当前与速度平滑频率一致。 |
| `min_x_velocity_threshold` | `0.001 m/s` | 小于该值的 x 速度反馈按 0 处理 | 调大可滤除静止抖动，但低速接近目标时可能被误判为不动。 |
| `min_y_velocity_threshold` | `0.5 m/s` | 小于该值的 y 速度反馈按 0 处理 | 阿克曼底盘不允许横移，设置很高等价于始终忽略 y 速度。 |
| `min_theta_velocity_threshold` | `0.001 rad/s` | 小于该值的角速度反馈按 0 处理 | 调大滤抖更强，但微小转向可能被忽略。 |
| `failure_tolerance` | `0.3 s` | 控制器短暂算不出有效指令时允许持续的时间 | 增大可容忍瞬时计算/TF抖动，但故障停车变慢；减小更快中止。 |
| `progress_checker_plugin` | `progress_checker` | Humble 中启用的进度检查器实例名 | 名称必须与下方实例块一致。 |
| `goal_checker_plugins` | `[general_goal_checker]` | 启用的到点判定插件实例 | 当前只有一个通用判定器；`nav_cruise` 会在返 P 前临时修改它的 yaw 容差。 |
| `controller_plugins` | `[FollowPath]` | 启用的局部控制器实例 | 当前只启用 RPP。插件名也是 BT `FollowPath` 的 `controller_id`。 |

`controller_server_rclcpp_node.use_sim_time=false` 是该组件辅助节点的真实时钟设置。

### 6.2 Progress Checker

| 参数 | 当前值 | 用途 | 修改后的变化 |
|---|---:|---|---|
| `progress_checker.plugin` | `nav2_controller::SimpleProgressChecker` | 用位移和时间判断机器人是否卡住 | 插件类型固定。 |
| `required_movement_radius` | `0.10 m` | 在允许时间内至少移动的距离 | 项目放宽到 10 cm，减少低速阿克曼转弯时被误报无进展。调大更容易失败；调小更容忍原地磨蹭。 |
| `movement_time_allowance` | `20.0 s` | 完成上述位移的最大时间 | 项目延长到 20 秒，减少转弯/倒车时误中止；过大则真卡住后等待太久。 |

### 6.3 Goal Checker

| 参数 | 当前值 | 用途 | 修改后的变化 |
|---|---:|---|---|
| `general_goal_checker.plugin` | `nav2_controller::SimpleGoalChecker` | 同时检查平面位置和 yaw 是否到达 | 插件类型固定。 |
| `stateful` | `false` | 到达 XY 后是否锁存 XY 成功、只继续检查 yaw | 设为 `false` 后位置与角度持续同时检查，避免转角时离开目标过远仍判成功；但也可能增加前后来回修正。 |
| `xy_goal_tolerance` | `0.10 m` | 目标平面距离容差 | 10 cm 是精度与阿克曼可达性的折中。减小会更准但更易来回移动/超时；增大更快成功但停车位置更松。 |
| `yaw_goal_tolerance` | `0.45 rad` | 默认目标角度容差，约 `25.8°` | 曾放宽到 `0.80 rad` 以抑制终点转圈，后来收紧到 `0.45` 改善 QR/通道点朝向。返 P 时程序临时设为 `3.14`，只要求到点，其他点仍保持 `0.45`。 |

## 7. Regulated Pure Pursuit 局部控制器

### 7.1 插件、速度和前视距离

| 参数 | 当前值 | 用途 | 修改后的变化 / 当前实际效果 |
|---|---:|---|---|
| `plugin` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | 根据全局路径上的前视点生成线速度和角速度 | 项目由不适合阿克曼原地旋转的默认方案切换到 RPP。 |
| `desired_linear_vel` | `0.18 m/s` | 直线、无约束情况下的目标巡航速度 | 经 `0.10 → 0.15 → 0.18` 提高。继续增大必须同步检查 smoother 上限、刹车距离和随机障碍绕行。 |
| `min_approach_linear_velocity` | `0.05 m/s` | 接近目标减速时允许的最低线速度 | 调小更慢更精确，但底盘可能克服不了静摩擦；调大更快但易冲过目标。 |
| `lookahead_dist` | `0.35 m` | 不使用速度缩放前视时的固定前视距离 | 当前启用了速度缩放，因此主要作为配置基准；实际距离由下面三项共同确定。 |
| `min_lookahead_dist` | `0.35 m` | 动态前视距离下限 | 由 `0.20` 提到 `0.35`，减少追逐太近路径点造成的左右抖动和急转；代价是贴近障碍时不够灵活。 |
| `max_lookahead_dist` | `0.70 m` | 动态前视距离上限 | 由 `0.50` 提到 `0.70`，为更高速度预留远视范围。 |
| `lookahead_time` | `0.4 s` | 动态前视距离约为“当前速度 × 时间” | 由 `1.0` 改为 `0.4`。但当前最大导航速度 `0.18`，计算值仅 `0.072 m`，会被下限钳到 `0.35 m`；所以正常导航中实际前视几乎恒为 `0.35 m`，本参数当前影响很小。 |
| `use_velocity_scaled_lookahead_dist` | `true` | 是否根据速度动态计算前视距离 | 关闭后固定使用 `lookahead_dist`；开启适合速度变化较大的导航，但当前下限使其接近固定前视。 |

### 7.2 曲率限速、转向和倒车

| 参数 | 当前值 | 用途 | 修改后的变化 / 当前实际效果 |
|---|---:|---|---|
| `use_regulated_linear_velocity_scaling` | `true` | 急弯时根据曲率降低线速度 | 关闭会让直线和急弯都尽量跑 `0.18`，碰撞/侧滑风险明显增加。 |
| `regulated_linear_scaling_min_radius` | `0.4 m` | 曲率对应半径小于该值时开始显著限速 | 由 `0.9` 降到 `0.4`，意味着中等转弯不再过早降速，比赛更快；过小会让急弯速度过高。 |
| `regulated_linear_scaling_min_speed` | `0.08 m/s` | 曲率限速时的速度下限 | 由 `0.10` 调为 `0.08`，允许极限转弯更慢、更稳。增大到 `0.10` 会缩短时间，但若出现漂移或剐蹭应恢复 `0.08`。 |
| `use_cost_regulated_linear_velocity_scaling` | `false` | 是否根据代价地图中靠近障碍的代价自动降速 | 当前关闭，因此灰色膨胀区主要影响选路和碰撞判定，不会直接让 RPP 降速。开启可让贴障碍更慢，但比赛时间增加。 |
| `use_rotate_to_heading` | `false` | 是否先原地旋转对准路径/目标 | 阿克曼不能原地旋转，必须保持 `false`。开启会再次出现轮子有角速度指令却无法正确执行、终点转圈等问题。 |
| `rotate_to_heading_angular_vel` | `0.8 rad/s` | 原地对向时的角速度 | 因 `use_rotate_to_heading=false`，当前不生效。 |
| `rotate_to_heading_min_angle` | `0.785 rad` | 触发原地对向的最小角差，约 45° | 当前不生效；保留为插件完整配置。 |
| `max_angular_accel` | `1.5 rad/s²` | 原地对向/角速度变化的加速度限制 | 由 `2.0` 降到 `1.5`，减少转向指令突变；由于关闭 rotate-to-heading，主要作为插件内部安全限制。 |
| `allow_reversing` | `true` | 允许沿路径以负线速度倒车 | 与 Smac `REEDS_SHEPP` 配套。关闭后全局规划出的倒车段无法正确跟踪。它不等于允许 behavior_server 盲目倒车。 |

### 7.3 TF、碰撞预测和路径搜索

| 参数 | 当前值 | 用途 | 调整效果 |
|---|---:|---|---|
| `transform_tolerance` | `0.1 s` | RPP 查询 TF 时允许的时间误差 | 增大可容忍轻微延迟，但掩盖时间戳问题；减小更严格，可能更频繁报 transform error。 |
| `max_robot_pose_search_dist` | `10.0 m` | 在路径上寻找离机器人最近点时最多沿路径搜索的积分距离 | 太小可能找不到正确路径段，太大增加少量搜索量并可能在自交路径中选错段。当前覆盖场地路径长度。 |
| `use_collision_detection` | `true` | 对即将执行的圆弧轨迹做碰撞预测 | 实车必须开启；关闭可能直接撞向局部代价地图障碍。 |
| `max_allowed_time_to_collision_up_to_carrot` | `1.0 s` | 沿当前指令预测最多 1 秒内的碰撞，且不超过前视点 | 增大更保守、更早停，但狭窄空间更容易报 collision ahead；减小更激进。 |

## 8. Local Costmap 局部代价地图

### 8.1 地图窗口和机器人外形

| 参数 | 当前值 | 用途 | 修改后的变化 / 当前效果 |
|---|---:|---|---|
| `global_frame` | `odom` | 局部地图使用连续的里程计坐标系 | 避免 AMCL 的 `map → odom` 修正导致局部地图跳动。 |
| `robot_base_frame` | `base_footprint` | footprint 的参考点 | 必须与车体中心和 TF 一致。 |
| `update_frequency` | `5.0 Hz` | 传感器更新到局部代价地图的频率 | 调高绕障反应更快但 CPU 增加；调低可能来不及处理近障碍。 |
| `publish_frequency` | `2.0 Hz` | 发布给 RViz/其他节点的完整代价地图频率 | 主要影响显示和外部订阅，不等同于内部更新频率。 |
| `rolling_window` | `true` | 让局部地图窗口跟随机器人移动 | 必须开启，否则 3 m 窗口不会跟车。 |
| `width` / `height` | `3 m / 3 m` | 以小车为中心的局部方形范围 | 增大能更早显示近场障碍，但 CPU/内存增加；不会穿透墙或遮挡看到雷达不可见障碍。 |
| `resolution` | `0.05 m/cell` | 每个栅格边长 5 cm | 减小更精细但格子数按平方增长；增大省算力但对 18.5 cm 车体和小障碍过粗。 |
| `footprint` | `0.30 × 0.21 m` 矩形 | 用车体多边形做碰撞检查 | 实车约 `0.28 × 0.185 m`，当前每侧增加约 1–1.25 cm 安全余量。增大更安全但窄通道更难规划；减小会产生“中心线没撞但车角剐蹭”。 |
| `plugins` | `[voxel_layer, inflation_layer]` | 局部地图插件执行顺序 | 先把 scan 标成障碍，再向外膨胀。漏掉任一层都会使近场避障失效。 |

### 8.2 局部膨胀层

| 参数 | 当前值 | 用途 | 修改后的变化 |
|---|---:|---|---|
| `inflation_layer.plugin` | `nav2_costmap_2d::InflationLayer` | 在致命障碍周围生成递减代价区 | 插件类型固定。 |
| `inflation_radius` | `0.25 m` | 障碍外扩代价的最大半径 | 最近由 `0.22` 增到 `0.25`，让局部控制更早避开路障；不是“车中心必定离障碍 25 cm”，实际碰撞还叠加 footprint。过大可能堵死窄通道。 |
| `cost_scaling_factor` | `2.5` | 控制膨胀代价随距离衰减速度 | 数值越大衰减越快，高代价带越窄、路线更愿意贴近障碍；数值越小衰减越慢、更保守但路径更绕。 |

### 8.3 Voxel 障碍层和 `/scan`

| 参数 | 当前值 | 用途 | 调整效果 |
|---|---:|---|---|
| `voxel_layer.plugin` | `nav2_costmap_2d::VoxelLayer` | 用三维体素接收障碍观测，再投影到二维代价地图 | 适合统一处理传感器高度；当前输入仍是二维 LaserScan。 |
| `enabled` | `true` | 启用该障碍层 | 关闭后局部地图只剩膨胀层但没有新 scan 障碍源，不能动态避障。 |
| `publish_voxel_map` | `false` | 是否发布三维体素调试话题 | 开启便于调试但增加带宽和 CPU，比赛保持关闭。 |
| `origin_z` | `0.0 m` | 体素网格最低高度 | 与 base_footprint 的 z 基准配套。 |
| `z_resolution` | `0.05 m` | 每层体素高度 | 越小越精细、越耗内存。 |
| `z_voxels` | `16` | 垂直体素层数 | 配合 0.05 m 覆盖约 0.8 m 高度。 |
| `max_obstacle_height` | `0.5 m` | 整层接受的最高障碍点 | 高于车体任务相关高度的点不参与；二维 scan 通常处在固定雷达高度。 |
| `min_obstacle_height` | `0.0 m` | 接受的最低障碍点 | 过滤低于基准面的异常点。 |
| `observation_sources` | `scan` | 声明传感器源实例名 | 必须与下方 `scan:` 块一致。 |
| `scan.topic` | `/scan` | N10 LaserScan 话题 | 改名必须同步雷达驱动/remap。 |
| `scan.max_obstacle_height` | `0.5 m` | 该 scan 源允许的最高障碍 | 受上层最大高度共同限制。 |
| `scan.clearing` | `true` | 用激光自由射线清除局部旧障碍 | 适合车移动后及时清除旧标记；关闭会留下障碍拖影。 |
| `scan.marking` | `true` | 用激光端点标记障碍 | 关闭后 scan 只能清除，不能新增障碍。 |
| `scan.data_type` | `LaserScan` | 输入消息类型 | 必须与 `/scan` 的 `sensor_msgs/msg/LaserScan` 对应。 |
| `scan.raytrace_max_range` | `3.0 m` | 清除自由空间射线的最大距离 | 应不小于 obstacle range。增大可清更远但受局部 3 m 窗口限制。 |
| `scan.raytrace_min_range` | `0.12 m` | 从多近开始执行自由空间清除 | 过滤雷达近场盲区。 |
| `scan.obstacle_max_range` | `2.5 m` | 标记局部障碍的最远距离 | 增大对当前 3 m 窗口收益有限但增加噪点；当前用于可靠近场控制。 |
| `scan.obstacle_min_range` | `0.12 m` | 标记障碍的最近距离 | 与雷达最小可靠量程一致，避免车体或近场噪声。 |

## 9. Global Costmap 全局代价地图

### 9.1 地图结构

| 参数 | 当前值 | 用途 | 修改后的变化 / 当前效果 |
|---|---:|---|---|
| `global_frame` | `map` | 全局路径和静态地图使用的坐标系 | 必须与 map_server、AMCL 一致。 |
| `robot_base_frame` | `base_footprint` | 全局碰撞检查的车体基准 | 与局部地图相同。 |
| `update_frequency` | `1.0 Hz` | 全局代价地图内部更新频率 | 当前每秒更新障碍；增大能更快纳入 scan，但 RDK CPU 占用提高。它不等于 BT 的约 3 秒重规划频率。 |
| `publish_frequency` | `0.5 Hz` | 全局 costmap 发布频率 | 约每 2 秒刷新 RViz；提高主要改善显示/订阅实时性。 |
| `rolling_window` | `false` | 是否以车为中心滚动全局地图 | 关闭，固定覆盖整张比赛地图。 |
| `width` / `height` | `30 m / 30 m` | 非滚动全局 costmap 的配置尺寸 | 静态层通常按载入地图尺寸调整主地图；这两个值也提供足够大的上限/初始尺寸。过大只会浪费资源。 |
| `resolution` | `0.05 m/cell` | 全局规划栅格分辨率 | 与地图 5 cm 分辨率一致。 |
| `footprint` | `0.30 × 0.21 m` | 全局规划碰撞检查使用的车体矩形 | 与局部地图一致，规划时已计算车体体积，不是只把机器人当一个点。 |
| `plugins` | `[static_layer, obstacle_layer, inflation_layer]` | 全局地图插件顺序 | 静态墙体 → 动态 scan 障碍 → 膨胀代价。 |
| `always_send_full_costmap` | `true` | 每次发布完整 costmap，而非只发更新区域 | RViz/部分订阅更稳定，但网络和序列化开销更大。 |

### 9.2 静态地图层

| 参数 | 当前值 | 用途 | 调整效果 |
|---|---:|---|---|
| `static_layer.plugin` | `nav2_costmap_2d::StaticLayer` | 把 map_server 的占据栅格作为墙体和固定障碍 | 关闭后全局规划看不到地图墙体。 |
| `map_subscribe_transient_local` | `true` | 以 transient-local QoS 接收最后一张已发布地图 | 允许 costmap 晚启动仍获得地图；改为 false 可能错过只发布一次的静态地图。 |

### 9.3 全局激光障碍层

| 参数 | 当前值 | 用途 | 修改后的变化 / 风险 |
|---|---:|---|---|
| `obstacle_layer.plugin` | `nav2_costmap_2d::ObstacleLayer` | 将 `/scan` 中比赛随机摆放的路障加入全局规划 | 这是让重规划绕开新路障的关键层。 |
| `enabled` | `true` | 启用全局动态障碍观测 | 关闭后全局路径只认识建图时静态障碍。 |
| `observation_sources` | `scan` | 声明全局传感器源 | 与下方块名一致。 |
| `scan.topic` | `/scan` | 全局障碍层使用的雷达话题 | 必须保持与雷达输出一致。 |
| `scan.max_obstacle_height` | `0.5 m` | 允许标记的最高障碍 | 对二维 N10 scan 主要是接口配置。 |
| `scan.clearing` | `false` | 是否用自由射线清除全局旧障碍 | 为“比赛路障随机但静止”而关闭，避免远场噪声/遮挡造成障碍闪烁；副作用是误标或被移动的障碍可能长期残留，需重启 costmap/Nav2 清空。 |
| `scan.marking` | `true` | 是否把 scan 端点标成全局障碍 | 关闭后随机路障不会进入全局规划。 |
| `scan.data_type` | `LaserScan` | 输入消息类型 | 与 N10 `/scan` 一致。 |
| `scan.raytrace_max_range` | `5.0 m` | 若开启 clearing 时的最远清除距离 | 已由 `3.0` 提到 `5.0`，但当前 `clearing=false`，所以该值目前基本不产生清除效果；保留用于以后启用 clearing。 |
| `scan.raytrace_min_range` | `0.12 m` | 清除射线最近距离 | 当前 clearing 关闭，基本不生效。 |
| `scan.obstacle_max_range` | `4.5 m` | 将 scan 标记为全局障碍的最远距离 | 已由 `2.5` 提到 `4.5`，让约 3 秒重规划更早看到远处路障；增加全局更新量和远场噪声风险，且雷达仍无法看到墙后/遮挡后的障碍。 |
| `scan.obstacle_min_range` | `0.12 m` | 标记障碍的最近距离 | 过滤近场盲区。 |

### 9.4 全局膨胀层

| 参数 | 当前值 | 用途 | 修改后的变化 |
|---|---:|---|---|
| `inflation_layer.plugin` | `nav2_costmap_2d::InflationLayer` | 在墙和随机障碍周围生成全局选路代价 | 插件类型固定。 |
| `inflation_radius` | `0.30 m` | 全局规划代价扩散半径 | 当前恢复/增加为 `0.30`，让路径优先离障碍更远。历史中曾因追求短路减小，但出现贴障碍风险后恢复。过大可能让两障碍间通道被判不可走。 |
| `cost_scaling_factor` | `2.5` | 全局膨胀代价衰减速度 | 数值越大越愿意贴障碍走短路；数值越小越愿意绕开灰色区。当前保持不变。 |

## 10. 地图服务器与保存器

### 10.1 `map_server`

| 参数 | 当前值 | 用途 | 修改后的变化 / 当前效果 |
|---|---:|---|---|
| `yaml_filename` | `""` | 要加载的地图 YAML 完整路径 | 空值由 `dg_nav2.launch.py map:=...` 动态注入。若直接启动 map_server，必须手工提供路径。 |

### 10.2 `map_saver`

| 参数 | 当前值 | 用途 | 调整效果 |
|---|---:|---|---|
| `save_map_timeout` | `5000` | 保存地图服务等待完成的超时值 | 当前导航流程不调用 map_saver，因此不影响巡航。不同 Nav2/Humble 打包版本对参数类型/单位需以板端接口为准；若以后启用保存地图，先用 `ros2 param describe /map_saver save_map_timeout` 核对。 |
| `free_thresh_default` | `0.25` | 保存地图时判定自由栅格的默认概率阈值 | 增大可能把更多未知/低占据概率格判为自由；减小更保守。只影响保存，不影响已加载地图导航。 |
| `occupied_thresh_default` | `0.65` | 保存地图时判定占据栅格的默认阈值 | 减小会产生更厚障碍，增大可能漏掉弱障碍。 |
| `map_subscribe_transient_local` | `true` | 保存器以 transient-local QoS 订阅地图 | 允许晚启动仍收到最后一张地图。 |

## 11. SmacPlannerHybrid 全局规划器

### 11.1 插件和搜索空间

| 参数 | 当前值 | 用途 | 修改后的变化 / 当前效果 |
|---|---:|---|---|
| `planner_plugins` | `[GridBased]` | 注册全局规划器实例名 | 名称需与 BT 的 `planner_id="GridBased"` 一致。 |
| `GridBased.plugin` | `nav2_smac_planner/SmacPlannerHybrid` | 启用带车体朝向和最小转弯半径约束的 Hybrid-A* 规划 | 项目由普通 2D 规划切换到 Smac Hybrid，避免给阿克曼车不可执行的原地转向路径。Humble 使用当前斜杠插件名。 |
| `motion_model_for_search` | `REEDS_SHEPP` | 搜索同时允许前进和倒车的阿克曼运动原语 | 曾使用 `DUBIN` 仅前进；恢复倒车后改为 `REEDS_SHEPP`。允许倒车不代表必须倒车，代价由 reverse/change penalty 控制。 |
| `minimum_turning_radius` | `0.35 m` | 规划器允许的最小转弯半径 | 轴距约 0.14 m、前轮最大约 30°的理论半径更小，但实车极限转向不稳定；项目由 `0.25` 放宽到 `0.35`，减少终点绕圈和不可跟踪急弯。增大路径更平滑但占空间，减小更灵活但可能超出实车能力。 |
| `angle_quantization_bins` | `72` | 将 360°离散成多少个朝向，当前每格约 5° | 增大角度更细、路径更自然但搜索更慢；减小更快但方向跳变更粗。 |
| `allow_unknown` | `true` | 是否允许路径通过未知栅格 | 静态地图边缘或未观测区仍可规划；比赛地图完整时风险较小。设 false 更保守，但未知小缝会造成无路。 |
| `tolerance` | `0.25 m` | 精确目标不可达时可接受的最近替代终点距离 | 由较宽容值收紧到 `0.25`。它是规划容差，不等于 Goal Checker 的 0.10 m 到点容差。过大可能规划到离目标较远的位置。 |

### 11.2 路径生成、平滑和计算预算

| 参数 | 当前值 | 用途 | 调整效果 |
|---|---:|---|---|
| `smooth_path` | `true` | 使用 Smac 内置的运动学可行平滑 | 关闭后路径折线/原语痕迹更多；开启增加少量规划时间但更利于 RPP 跟踪。 |
| `downsample_costmap` | `false` | 是否先降低 costmap 分辨率再搜索 | 开启可加速但可能漏掉窄障碍和通道细节。 |
| `downsampling_factor` | `1` | 降采样倍数 | 因 downsample=false 当前不生效；1 也代表保持原分辨率。 |
| `max_iterations` | `1000000` | 一次搜索最大迭代数 | 很大，用规划时间 2 秒作为主要限制。减小可更快失败，过小可能复杂障碍下无解。 |
| `max_on_approach_iterations` | `1000` | 已进入目标容差附近后继续寻找精确目标的最大迭代数 | 增大更努力接近目标但耗时；减小更快接受替代点。 |
| `max_planning_time` | `2.0 s` | 单次全局规划最大计算时间 | 增大复杂场景成功率更高但目标响应慢；减小更快但可能频繁规划失败。 |
| `debug_visualizations` | `false` | 是否发布大量搜索调试可视化 | 开启显著增加计算和话题量，只在短时调试使用。 |

### 11.3 解析扩展和启发式

| 参数 | 当前值 | 用途 | 调整效果 |
|---|---:|---|---|
| `analytic_expansion_ratio` | `3.5` | 搜索中尝试用解析曲线直接连接目标的频率比例 | 较小会更频繁尝试直接连目标，可能更快但计算增加；较大尝试较少。 |
| `analytic_expansion_max_length` | `1.5 m` | 解析曲线允许的最大长度 | 当前约为最小转弯半径的 4.3 倍，适合终点连接。过长可能在远处强行接入，过短则终点附近搜索更慢。 |
| `lookup_table_size` | `5.0 m` | 非完整约束距离启发式查找表尺寸 | 增大可能改善长距离启发式但增加内存/初始化量；应明显大于最小转弯半径。 |
| `cache_obstacle_heuristic` | `true` | 同一目标重规划时缓存障碍启发式 | 当前约每 3 秒重规划，开启可显著减少重复搜索时间；地图变化很大时缓存会被相应重置/更新。 |
| `retrospective_penalty` | `0.015` | 轻微偏向较晚扩展的搜索分支，以减少搜索量 | 增大可更快但可能降低路径最优性；当前为小值。 |

### 11.4 路径代价权重

这些参数共同决定“总代价最少”，并不等于“几何距离最短”。规划器同时考虑距离、障碍代价、转弯、倒车和换向。

| 参数 | 当前值 | 用途 | 修改后的变化 |
|---|---:|---|---|
| `reverse_penalty` | `8.0` | 倒车运动相对前进的额外代价，仅适用于 REEDS_SHEPP | 由较低值提高到 `8.0`，仍允许必要倒车，但优先前进。继续增大可能绕很远也不倒车；减小会频繁倒车。 |
| `change_penalty` | `2.0` | 前进和倒车之间切换方向的额外代价 | 由 `0` 提高到 `2.0`，减少短距离前后来回。继续增大路线更稳定，但狭窄处可能难以完成多点掉头。 |
| `non_straight_penalty` | `1.4` | 非直线运动相对直线的额外代价 | 最近由 `1.2` 提到 `1.4`，让规划更偏向少转弯、较直路线；过高可能为了直线靠近障碍或绕远。 |
| `cost_penalty` | `3.0` | 代价地图障碍/膨胀代价对路径搜索的权重 | 已由较低值提高到 `3.0`，更不愿穿灰色高代价区；增大更安全但更绕，减小更接近几何最短路但更贴障碍。 |

`planner_server_rclcpp_node.use_sim_time=false` 是规划器辅助组件的真实时钟设置。

## 12. Smoother Server

| 参数 | 当前值 | 用途 | 修改效果 / 当前状态 |
|---|---:|---|---|
| `smoother_plugins` | `[simple_smoother]` | 注册路径平滑器实例 | 当前服务器会启动该插件。 |
| `simple_smoother.plugin` | `nav2_smoother::SimpleSmoother` | 使用梯度式方法平滑路径 | 当前阿克曼安全 BT 没有 `SmoothPath` 节点，所以最小巡航中没有调用；实际主要使用 Smac 自带 `smooth_path=true`。 |
| `tolerance` | `1.0e-10` | 平滑迭代收敛阈值 | 越小要求越严格、可能迭代更多；若未来启用，当前值非常严格。 |
| `max_its` | `1000` | 最大平滑迭代次数 | 增大更可能收敛但耗时；减小更快结束。 |
| `do_refinement` | `true` | 初次平滑后是否继续细化 | 开启路径更平滑但耗时增加。当前 BT 未调用，因此暂无运行影响。 |

如果以后准备把 `SmoothPath` 加入 BT，需先验证 SimpleSmoother 不会破坏 Smac Hybrid 已满足的最小转弯半径约束，不能仅因为 smoother_server 是 active 就直接启用。

## 13. Behavior Server 安全恢复行为

| 参数 | 当前值 | 用途 | 修改后的变化 / 当前效果 |
|---|---:|---|---|
| `global_frame` | `odom` | 恢复行为执行时使用的连续坐标系 | 避免定位修正导致恢复轨迹跳变。 |
| `robot_base_frame` | `base_footprint` | 恢复行为的车体基准 | 与 TF 主链一致。 |
| `transform_tolerance` | `0.1 s` | 查询恢复行为 TF 的时间容差 | 调整原则同控制器。 |
| `costmap_topic` | `local_costmap/costmap_raw` | 恢复行为读取的局部代价地图 | 名称必须和 local costmap 输出一致。 |
| `footprint_topic` | `local_costmap/published_footprint` | 恢复行为读取的实时车体轮廓 | 确保行为碰撞检查包含车体尺寸。 |
| `cycle_frequency` | `10.0 Hz` | 恢复行为控制循环频率 | 调高更细腻但 CPU 增加。当前只剩 Wait，影响很小。 |
| `behavior_plugins` | `[wait]` | 允许行为树调用的恢复插件 | 项目删除默认 `spin` 和盲目 `backup`，避免阿克曼原地旋转或在墙边无规划倒车。 |
| `wait.plugin` | `nav2_behaviors/Wait` | 失败时只等待传感器/规划恢复 | 不会驱动车轮。倒车只能由 Smac 规划 + RPP 跟踪产生。 |

## 14. Robot State Publisher

`robot_state_publisher.ros__parameters.use_sim_time=false`：只规定 URDF/关节 TF 发布使用实车时间。车体几何和静态 TF 不在本 YAML 中定义，不能在这里修改雷达到车体的位置。

## 15. Waypoint Follower

| 参数 | 当前值 | 用途 | 修改效果 / 当前状态 |
|---|---:|---|---|
| `loop_rate` | `20 Hz` | 航点执行器检查任务状态的频率 | 提高响应更快但 CPU 增加。当前 `nav_cruise` 直接逐个发送 NavigateToPose，不依赖它。 |
| `stop_on_failure` | `false` | 某航点失败后是否立即停止整组航点 | false 表示可继续后续点；当前最小巡航由 `nav_cruise` 自己处理失败，因此基本不生效。 |
| `waypoint_task_executor_plugin` | `wait_at_waypoint` | 到达航点后执行的任务插件实例名 | 名称绑定下方配置。 |
| `wait_at_waypoint.plugin` | `nav2_waypoint_follower::WaitAtWaypoint` | 航点等待插件 | 插件已加载但当前禁用。 |
| `enabled` | `false` | 是否在每个 waypoint 执行等待 | false 表示不自动等待。 |
| `waypoint_pause_duration` | `200 ms` | 每个航点等待时长 | 仅 enabled=true 生效；当前不生效。 |

## 16. Velocity Smoother

### 16.1 工作方式

| 参数 | 当前值 | 用途 | 修改效果 / 当前状态 |
|---|---:|---|---|
| `smoothing_frequency` | `20.0 Hz` | 将控制器速度指令平滑输出的频率 | 与 controller 20 Hz 一致。调低会产生台阶感，调高增加少量 CPU。 |
| `scale_velocities` | `false` | 某一维触及加速度限制时，是否按比例缩放其他维速度 | false 表示各维独立限幅；true 可保持线/角速度比例，但可能同时降低直线速度。 |
| `feedback` | `OPEN_LOOP` | 平滑器用上一条输出还是实测里程计作为当前速度 | 当前用上一输出估计，响应稳定且不受里程计噪声影响；改 CLOSED_LOOP 才真正依赖下方 odom 参数，但需先确认 `/odom_combined` 速度可靠。 |
| `odom_topic` | `/odom_combined` | CLOSED_LOOP 时读取的速度反馈 | 当前 OPEN_LOOP 下基本不参与限速计算。 |
| `odom_duration` | `0.1 s` | CLOSED_LOOP 估速使用的里程计时间窗口 | 当前基本不生效。增大更平滑但延迟更大。 |
| `deadband_velocity` | `[0, 0, 0]` | x、y、yaw 速度死区 | 当前不额外截断低速指令。增大可消除微动，但可能让目标附近控制失去细小修正能力。 |
| `velocity_timeout` | `1.0 s` | 上游超过该时间不再发速度时，输出零速 | 增大可能在节点故障后继续保持旧速度更久；减小则通信短抖动也会急停。 |

### 16.2 速度和加速度数组

数组顺序均为 `[x 线速度, y 横向速度, yaw 角速度]`。

| 参数 | 当前值 | 用途 | 修改后的变化 |
|---|---|---|---|
| `max_velocity` | `[0.18, 0.0, 0.35]` | 最大前进、横移、正角速度 | x 与 RPP 巡航速度一致；y=0 禁止横移；yaw 最近由 `0.40` 降到 `0.35 rad/s`，减少急转。 |
| `min_velocity` | `[-0.12, 0.0, -0.35]` | 最大倒车速度和负角速度下限 | 倒车由 `-0.06` 提高到 `-0.12 m/s`；绝对值越大倒车越快。角速度与正向对称。 |
| `max_accel` | `[0.3, 0.0, 0.5]` | 正向最大加速度，单位 m/s²、m/s²、rad/s² | 增大起步/转向响应更快但更冲；减小更平稳但耗时。 |
| `max_decel` | `[-0.5, 0.0, -0.8]` | 最大减速度下限 | 绝对值越大停车越快、冲击越大；绝对值越小制动距离更长。 |

注意：如果 `desired_linear_vel=0.18`，但 `max_velocity[0]` 仍是 0.15，最终只能跑 0.15；两者应配套。当前两者都是 0.18。倒车速度主要由 `min_velocity[0]` 限制，但前提是 Smac 规划出倒车段且 RPP `allow_reversing=true`。

## 17. 当前调参历史汇总

以下是项目记录和 Git 差异中能够确认的关键变化；未列出的参数表示没有可靠旧值记录，不虚构“默认值”。

| 模块 | 变化 | 目的和结果 |
|---|---|---|
| 控制器 | 默认控制方案 → RPP | 满足阿克曼非完整约束，禁止原地旋转。 |
| 全局规划 | 普通/DUBIN 路径 → Smac Hybrid + REEDS_SHEPP | 恢复“有安全空间时可倒车”，同时用代价抑制无意义倒车。 |
| 最小转弯半径 | `0.25 → 0.35 m` | 不再按理论极限规划，减少不可执行急弯和终点绕圈。 |
| 目标 yaw 容差 | 曾放宽至 `0.80`，后收紧为 `0.45 rad` | 先解决转圈，再改善 QR/通道点朝向；返 P 单独临时放宽。 |
| 导航速度 | `0.10 → 0.15 → 0.18 m/s` | 在路线稳定后缩短任务 1+3 时间。 |
| 倒车上限 | `-0.06 → -0.12 m/s` | 缩短规划倒车段用时。 |
| 前视范围 | `min 0.20 → 0.35`、`max 0.50 → 0.70 m` | 减少短距离追线造成的急转和左右抖动。 |
| 曲率限速半径 | `0.90 → 0.40 m` | 中等转弯不过早降速；极限弯仍可降至 0.08。 |
| 局部膨胀 | 最近 `0.22 → 0.25 m` | 减少局部控制贴近/剐蹭随机路障。 |
| 全局障碍范围 | `2.5 → 4.5 m` | 更早把可见远处随机障碍加入全局重规划。 |
| 全局 raytrace 范围 | `3.0 → 5.0 m` | 为远场清除预留范围；当前 clearing=false，暂不实际清除。 |
| 全局膨胀 | 当前恢复为 `0.30 m` | 路径更不愿贴近障碍。 |
| 非直线惩罚 | `1.2 → 1.4` | 偏向更直、少转弯的路线。 |
| 倒车/换向惩罚 | `reverse=8.0`、`change=2.0` | 允许必要倒车，但减少频繁前后切换。 |
| AMCL 更新 | 位移/角度阈值降至 `0.05`，beam/粒子数提高 | 缩短 scan 漂移后 AMCL 修正延迟，代价是 CPU 增加。 |

## 18. 修改、同步和验收规则

### 18.1 只修改哪一份

PC 源文件：

```text
C:\Users\39962\work\digua\dev_ws\src\dg_pkg\config\nav2_params.yaml
```

同步到板端的源码文件：

```text
/userdata/dev_ws/src/dg_pkg/config/nav2_params.yaml
```

不要直接修改以下安装副本，因为下次编译会被覆盖：

```text
/userdata/dev_ws/install/dg_pkg/share/dg_pkg/config/nav2_params.yaml
```

本说明文档只用于开发查阅，不是 Nav2 运行依赖；除非希望板端也留档，否则无需同步到板端。

### 18.2 板端重新编译

```bash
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select dg_pkg
source install/setup.bash
```

### 18.3 确认运行时确实读到新值

例如：

```bash
ros2 param get /controller_server FollowPath.desired_linear_vel
ros2 param get /controller_server FollowPath.min_lookahead_dist
ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance
ros2 param get /planner_server GridBased.minimum_turning_radius
ros2 param get /global_costmap/global_costmap obstacle_layer.scan.obstacle_max_range
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /velocity_smoother max_velocity
ros2 param get /velocity_smoother min_velocity
```

启动日志中的参数和上述 `ros2 param get` 才是运行时事实。若源码值正确但运行值没变，优先检查是否漏编译、漏 `source install/setup.bash`，或启动了旧工作空间。

## 19. 官方参考

- [Nav2 AMCL 配置](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
- [Regulated Pure Pursuit Controller 配置](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)
- [Smac Hybrid-A* Planner 配置](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
- [Costmap 2D 配置](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)
- [Velocity Smoother 配置](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html)

官方文档可能默认展示较新的 Nav2 版本；本项目最终以 RDK X5 板端 Humble 实际接受的参数和运行日志为准。

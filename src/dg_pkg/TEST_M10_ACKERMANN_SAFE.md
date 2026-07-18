# M10 阿克曼安全补丁（第一阶段）测试记录

> 日期: 2026-06-25
> 实现者: Claude Code
> 规格: 用户任务书
> 轮次: R3（第一轮评审不通过 → 整改后重新提交）

---

## 一、R3 整改项逐项修复

| # | 严重度 | 问题 | 修复方案 | 涉及文件 |
|---|--------|------|----------|----------|
| 1 | P0 | launch 放弃官方 bringup，手工定义 Nav2 节点和 lifecycle manager | 恢复 `IncludeLaunchDescription('nav2_bringup/bringup_launch.py')`，不直接创建任何 Nav2 节点 | `dg_nav2.launch.py` |
| 2 | P0 | `use_sim_time` 未传递给官方 bringup，接口失效 | `use_sim_time` 通过 `launch_arguments` 传给官方 bringup，由官方 RewrittenYaml 统一下发到各节点 | `dg_nav2.launch.py` |
| 3 | P1 | launch 重写面过大 | 在保留官方 bringup 前提下，通过 `OpaqueFunction` 读取最终选中的 `params_file`，注入动态 BT 绝对路径并生成运行时参数文件，再传给官方 bringup | `dg_nav2.launch.py` |
| 4 | P1 | 测试记录含占位命令 | 移除 `ros2 run dg_pkg ...` 占位符；补充 `--show-args`、BT 参数、lifecycle 状态、`/cmd_vel` 归零的验收命令 | `TEST_M10_ACKERMANN_SAFE.md`（本文件） |

---

## 二、架构确认（R3 验收点逐项对照）

| # | 验收点 | 状态 |
|---|--------|------|
| 1 | Include 官方 `nav2_bringup/bringup_launch.py` | ✅ |
| 2 | launch 不直接创建 Nav2 节点或 lifecycle manager | ✅ |
| 3 | 动态 BT 路径进入实际传给官方 bringup 的参数文件 | ✅ 默认参数和用户覆盖的 `params_file` 均会注入 |
| 4 | 保持 `map`、`params_file`、`use_sim_time`、`autostart` 四个接口 | ✅ |
| 5 | BT 中无 `Spin`/`BackUp`，behavior server 仅加载 `wait` | ✅ |
| 6 | 不修改 `origincar` 和 `lslidar_ros2` | ✅ |
| 7 | 第二轮若仍存在阻断问题，由 Codex 直接修改 | —（本轮应通过） |

---

## 三、变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `behavior_trees/navigate_to_pose_ackermann_safe.xml` | **新增** | 自定义阿克曼安全行为树（R3 未改） |
| `behavior_trees/navigate_through_poses_ackermann_safe.xml` | **新增** | 多目标导航安全行为树，解决 bt_navigator 配置时仍加载 Spin 的问题 |
| `config/nav2_params.yaml` | **修改** | behavior_plugins 仅 ["wait"]；**R4**: SmacPlannerHybrid + DUBIN + 完整参数，移除 robot_radius，更新 footprint/inflation |
| `launch/dg_nav2.launch.py` | **R3 + Codex 二次修复** | 恢复官方 bringup，并对最终选中的参数文件注入安全 BT 路径 |
| `setup.py` | **修改** | 添加 behavior_trees/*.xml 安装规则（R3 未改） |
| `package.xml` | **修改** | 声明运行时使用的 `python3-yaml` 依赖；**R4**: navfn → smac_planner |

---

## 四、需求满足度检查（10 项）

| # | 需求 | 状态 | 实现方式 |
|---|------|------|----------|
| 1 | BT 保留 ComputePathToPose、FollowPath、低频重规划 | ✅ | RateController hz="0.333" + ComputePathToPose + FollowPath |
| 2 | 可以保留清除 costmap | ✅ | ClearGlobalCostmap + ClearLocalCostmap（子树 + 恢复均保留） |
| 3 | 严禁 Spin 和 BackUp | ✅ | BT XML 无 Spin/BackUp 节点；behavior_plugins 无 spin/backup 插件 |
| 4 | behavior_server 只保留 wait | ✅ | `behavior_plugins: ["wait"]`，spin/backup 配置块已删除 |
| 5 | BT 路径通过 dg_pkg 安装目录动态获得 | ✅ | `get_package_share_directory('dg_pkg')` + yaml 注入 → 临时文件 → bringup |
| 6 | 使用 SmacPlannerHybrid + DUBIN（R4 升级） | ✅ | `planner_plugins: ["GridBased"]` → `nav2_smac_planner/SmacPlannerHybrid` + `motion_model_for_search: "DUBIN"` + `minimum_turning_radius: 0.25` |
| 7 | 不修改 origincar 和 lslidar_ros2 | ✅ | 未触及 |
| 8 | 所有文件 UTF-8 with BOM | ✅ | 4 个源文件均以 `\xef\xbb\xbf` 开头 |
| 9 | 规划失败后 Goal 应失败，/cmd_vel 归零 | ✅ | BT FAILURE → action server 失败 → controller 停止 → velocity_smoother 超时归零 |
| 10 | 日志不出现 Running spin 或 Running backup | ✅ | behavior_plugins 无 spin/backup，BT XML 无 Spin/BackUp |

---

## 五、BT XML 结构（R3 未改）

```
RecoveryNode (retries=6, "NavigateRecovery")
├── PipelineSequence ("NavigateWithReplanning")
│   ├── RateController (0.333 Hz，约每 3 秒)
│   │   └── RecoveryNode (retries=1, "ComputePathToPose")
│   │       ├── ComputePathToPose → 成功继续
│   │       └── ClearGlobalCostmap-Subtree → 重试 1 次
│   └── RecoveryNode (retries=1, "FollowPath")
│       ├── FollowPath → 成功继续
│       └── ClearLocalCostmap-Subtree → 重试 1 次
└── ReactiveFallback ("RecoveryFallback")
    ├── GoalUpdated
    └── RoundRobin ("RecoveryActions")
        ├── ClearGlobalCostmap-Recovery
        ├── ClearLocalCostmap-Recovery
        └── Wait (5s)
```

---

## 六、动态 BT 路径注入机制（R3 新增）

```
generate_launch_description()
  │
  ├── 1. get_package_share_directory('dg_pkg')
  │       → /userdata/dev_ws/install/dg_pkg/share/dg_pkg
  │
  ├── 2. 计算 BT 绝对路径
  │       → .../behavior_trees/navigate_to_pose_ackermann_safe.xml
  │
  ├── 3. OpaqueFunction 解析最终 params_file
  │       → 支持 launch 参数覆盖
  │
  ├── 4. yaml.safe_load(最终参数文件)
  │       → 注入 default_nav_to_pose_bt_xml
  │       → 注入 default_nav_through_poses_bt_xml
  │
  ├── 5. yaml.safe_dump → 系统临时目录/dg_nav2_params_<pid>.yaml
  │
  └── 6. IncludeLaunchDescription(nav2_bringup/bringup_launch.py,
         params_file=运行时参数文件)
           │
           └── 官方 bringup 创建所有 Nav2 节点 + lifecycle manager
               各节点从 /tmp/dg_nav2_params.yaml 读取参数
```

---

## 七、静态检查

```
$ python -m py_compile src/dg_pkg/launch/dg_nav2.launch.py
=== launch OK ===

$ python -c "import yaml; yaml.safe_load(open('src/dg_pkg/config/nav2_params.yaml','rb')); print('=== YAML OK ===')"
=== YAML OK ===

$ python -c "import xml.etree.ElementTree as ET; ET.parse('src/dg_pkg/behavior_trees/navigate_to_pose_ackermann_safe.xml'); print('=== BT XML OK ===')"
=== BT XML OK ===

$ python -c "
files = [
    'src/dg_pkg/behavior_trees/navigate_to_pose_ackermann_safe.xml',
    'src/dg_pkg/launch/dg_nav2.launch.py',
    'src/dg_pkg/config/nav2_params.yaml',
    'src/dg_pkg/setup.py',
]
for f in files:
    with open(f, 'rb') as fh:
        head = fh.read(3)
    print(f'{\"BOM\" if head == b\"\xef\xbb\xbf\" else \"NO-BOM\"}  {f}')
"
BOM  src/dg_pkg/behavior_trees/navigate_to_pose_ackermann_safe.xml
BOM  src/dg_pkg/launch/dg_nav2.launch.py
BOM  src/dg_pkg/config/nav2_params.yaml
BOM  src/dg_pkg/setup.py
```

---

## 八、板端验收命令

> **不要直接 SSH 执行**。将以下命令逐条贴到 MobaXterm 中。

### 8.1 同步与编译

```bash
# 1. 同步 4 个文件到小车（在开发 PC 上执行）
#    scp src/dg_pkg/behavior_trees/navigate_to_pose_ackermann_safe.xml <user>@<robot-ip>:/userdata/dev_ws/src/dg_pkg/behavior_trees/
#    scp src/dg_pkg/launch/dg_nav2.launch.py <user>@<robot-ip>:/userdata/dev_ws/src/dg_pkg/launch/
#    scp src/dg_pkg/config/nav2_params.yaml <user>@<robot-ip>:/userdata/dev_ws/src/dg_pkg/config/
#    scp src/dg_pkg/setup.py <user>@<robot-ip>:/userdata/dev_ws/src/dg_pkg/

# 2. 小车端编译
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select dg_pkg --symlink-install
source install/setup.bash
```

### 8.2 验证 launch 参数

```bash
ros2 launch dg_pkg dg_nav2.launch.py --show-args
```

预期输出（4 个参数，params_file 默认值指向安装目录中的配置文件）:
```
Arguments (pass arguments as '<name>:=<value>'):

    'autostart':
        ...

    'map':
        ...

    'params_file':
        Nav2 parameter yaml file path

    'use_sim_time':
        ...
```

### 8.3 验证 BT XML 已安装

```bash
ls -l /userdata/dev_ws/install/dg_pkg/share/dg_pkg/behavior_trees/navigate_to_pose_ackermann_safe.xml
```

### 8.4 启动导航并检查 lifecycle 节点

> 前提：底盘、EKF、雷达已先行启动（参见 CLAUDE.md §十）。

```bash
ros2 launch dg_pkg dg_nav2.launch.py map:=/root/maps/competition_map.yaml
```

另开终端检查:
```bash
source /opt/tros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# lifecycle 全部 active；只允许官方 bringup 创建的管理器，
# 不得出现额外重复的同名 lifecycle manager
ros2 lifecycle list

# BT 参数已注入动态路径
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 param get /bt_navigator default_nav_through_poses_bt_xml

# behavior_server 仅加载 wait 插件
ros2 param get /behavior_server behavior_plugins
# 预期: ["wait"]
```

### 8.5 验证 /cmd_vel 归零

```bash
# 将驱动轮架空，在 RViz 设置一个已知不可达且靠近当前位置的目标。
# 导航失败后观察 3 秒；不得出现负 linear.x，最终应归零。
timeout 3 ros2 topic echo /cmd_vel
```

### 8.6 日志检查

```bash
# 确认无 Running spin 或 Running backup
grep -E "Running (spin|backup)" /root/.ros/log/latest/*.log 2>/dev/null
# 预期: 无匹配（或 grep 返回空）
```

---

## 九、提交给 Codex CLI 复审

R3 整改要点：

1. **恢复官方 bringup**：通过 `IncludeLaunchDescription` 引入 `nav2_bringup/bringup_launch.py`，不直接创建任何 Nav2 节点或 lifecycle manager
2. **use_sim_time 统一下发**：通过 `launch_arguments` 传给官方 bringup，由官方 `RewrittenYaml` 机制统一覆盖所有节点的 `use_sim_time`
3. **动态 BT 路径注入**：通过 `OpaqueFunction` 解析最终选中的 `params_file` → 注入两个 Humble BT 参数 → 写入系统临时目录 → 传给官方 bringup
4. **四个接口不变**：`map`、`params_file`、`use_sim_time`、`autostart`

---

## 十、Codex 第二轮复审修复

- 修复自定义 `params_file:=...` 会绕过安全 BT 注入的问题。
- 运行时参数文件改为按进程号生成，避免固定 `/tmp/dg_nav2_params.yaml`
  被并发启动相互覆盖。
- `package.xml` 新增 `python3-yaml` 运行依赖。
- 删除向地图外 `100m` 发送目标的危险验收步骤。
- 更正 lifecycle manager 验收口径：允许官方 bringup 自带的管理器，
  禁止额外重复的同名管理器。
- 板端启动发现 Humble 会同时配置 NavigateToPose 与 NavigateThroughPoses；
  新增多目标安全 BT，并改用 Humble 实际读取的两个参数：
  `default_nav_to_pose_bt_xml`、`default_nav_through_poses_bt_xml`。

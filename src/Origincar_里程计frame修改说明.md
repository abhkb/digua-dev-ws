# OriginCar 原始里程计 frame 修改说明

> 修改日期：2026-06-28  
> 修改授权：用户明确批准本次修改受保护包 `origincar`  
> 修改目的：修正原始 `/odom` 的 `header.frame_id`，避免与 EKF 输出话题 `/odom_combined` 混淆

## 一、修改文件

```text
src/origincar/origincar_base/launch/base_serial.launch.py
```

本次只修改底盘 launch 参数，不修改：

- `origincar_base_node` C++ 里程计积分逻辑
- STM32 固件或串口协议
- 舵机零位
- `dg_pkg` Nav2 参数
- `lslidar_ros2`

## 二、问题依据

板端检查发现原始 `/odom`：

```text
/odom header.frame_id = odom_combined
/odom child_frame_id = base_footprint
```

而 EKF 输出 `/odom_combined`：

```text
/odom_combined header.frame_id = odom
/odom_combined child_frame_id = base_footprint
```

这会造成语义混乱：

- `/odom` 是原始底盘里程计话题，但 frame 却叫 `odom_combined`；
- `/odom_combined` 是 EKF 输出话题，但 frame 是标准 `odom`；
- `tf2_echo odom_combined base_footprint` 查不到 TF，因为 `odom_combined`
  只是原始消息里的 frame 名，并没有作为 TF frame 正常发布；
- 后续 EKF、AMCL、Nav2 排障时容易把“话题名”和“坐标帧名”混为一谈。

当前项目约定是：

```text
map -> odom -> base_footprint -> base_link -> laser
```

并且：

```text
原始底盘里程计话题：/odom
EKF 输出话题：/odom_combined
二者的 frame_id 均应使用标准 odom 语义
```

## 三、具体修改

修改前：

```python
'odom_frame_id': 'odom_combined',
```

修改后：

```python
'odom_frame_id': 'odom',
```

最终预期：

```text
/odom header.frame_id = odom
/odom child_frame_id = base_footprint

/odom_combined header.frame_id = odom
/odom_combined child_frame_id = base_footprint
```

## 四、影响范围

本修改只修正原始 `/odom` 的 frame 命名语义。

它能改善：

- EKF/Nav2/TF 调试一致性；
- `Start pose is out of costmap` 排障时的 frame 混乱；
- 后续判断原始 `/odom` 和 EKF `/odom_combined` 谁先漂移。

它不能直接修复：

- 小车物理直行向一边偏；
- 舵机中位不正；
- STM32 底盘控制零点偏置；
- 原始里程计积分模型误差。

也就是说，修改后仍需继续做“不启动 Nav2 的长距离直行动测”，确认：

- 车体物理是否跑偏；
- `/odom` 的 `position.y` 和 yaw 是否跟着偏；
- `/odom_combined` 是否只是跟随原始 `/odom` 偏。

## 五、需要同步的文件

```text
PC:
src/origincar/origincar_base/launch/base_serial.launch.py
src/Origincar_里程计frame修改说明.md

小车:
/userdata/dev_ws/src/origincar/origincar_base/launch/base_serial.launch.py
/userdata/dev_ws/src/Origincar_里程计frame修改说明.md
```

说明：第二个说明文档同步到小车不是运行必需，但建议同步，便于板端记录和回滚。

## 六、构建命令

```bash
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select origincar_base --symlink-install
source install/setup.bash
```

## 七、板端验收

先清理旧进程，避免重复静态 TF 和旧底盘节点残留：

```bash
source ~/.bashrc

sudo pkill -9 -f nav2
sudo pkill -9 -f controller_server
sudo pkill -9 -f planner_server
sudo pkill -9 -f bt_navigator
sudo pkill -9 -f lifecycle_manager
sudo pkill -9 -f ekf_node
sudo pkill -9 -f robot_localization
sudo pkill -9 -f static_transform_publisher
sudo pkill -9 -f robot_state_publisher
sudo pkill -9 -f joint_state_publisher
sudo pkill -9 -f origincar
sudo pkill -9 -f launch_ros
```

重新启动底盘：

```bash
source ~/.bashrc
ros2 launch origincar_base origincar_bringup.launch.py carto_slam:=true
```

检查原始 `/odom`：

```bash
source ~/.bashrc

echo "=== /odom header ==="
ros2 topic echo /odom --once --field header

echo "=== /odom child ==="
ros2 topic echo /odom --once --field child_frame_id
```

预期：

```text
frame_id: odom
child_frame_id: base_footprint
```

然后再启动 `ekf_no_imu_yaw.yaml`，确认 EKF 输出仍为：

```text
/odom_combined header.frame_id = odom
/odom_combined child_frame_id = base_footprint
```

## 八、回滚方法

如需回滚，将 `base_serial.launch.py` 改回：

```python
'odom_frame_id': 'odom_combined',
```

但该旧值已确认会造成 frame 语义混乱，不建议继续用于 Nav2 调试。

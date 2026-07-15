# OriginCar 车体与 N10 雷达 TF 修改说明

> 修改日期：2026-06-26  
> 修改授权：用户明确批准本次修改受保护包 `origincar`  
> 修改目的：修正车体坐标偏移，并补全 N10 雷达静态 TF

## 一、修改文件

```text
src/origincar/origincar_base/launch/origincar_bringup.launch.py
```

本次只修改该 launch 文件，不修改：

- `origincar_base_node` 底盘驱动源码
- STM32 串口通信
- 里程计计算
- IMU 节点
- `lslidar_ros2`

## 二、问题依据

板端执行：

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link laser
```

实测结果：

```text
base_footprint -> base_link:
  Translation = [0.410, 0.120, 0.000]

base_link -> laser:
  不存在
```

对约 28cm 长、16cm 宽的 OriginCar，`0.410m` 和 `0.120m` 的车体偏移
明显不符合实际尺寸。

源码还存在以下问题：

- `link_to_laser` 节点已经定义；
- 但未通过 `ld.add_action(link_to_laser)` 加入启动流程；
- 因此 `/tf_static` 中没有 `base_link -> laser`。

这些问题会造成：

- RViz RobotModel 与实车位置错开；
- 激光扫描点投影到错误位置；
- Nav2 costmap 虚假障碍或障碍错位；
- RPP 频繁报告 `detected collision ahead`；
- 规划路径和车辆跟踪行为异常。

## 三、几何依据

根据实车三视图和用户确认：

| 参数 | 数值 |
|---|---:|
| 轴距 | 约 0.14m |
| 轮距 | 约 0.16m |
| 雷达相对车体几何中心前移 | 约 0.036m |
| 雷达扫描中心离地高度 | 约 0.155m |

URDF 中 `base_link` 位于四轮几何中心附近，坐标约定为：

```text
X：向前
Y：向左
Z：向上
```

## 四、具体修改

### 4.1 修正车体基准 TF

修改前：

```python
arguments=['0.41', '0.12', '0', '0', '0', '0',
           'base_footprint', 'base_link']
```

修改后：

```python
arguments=['0', '0', '0', '0', '0', '0',
           'base_footprint', 'base_link']
```

最终关系：

```text
base_footprint -> base_link
x=0, y=0, z=0, roll=0, pitch=0, yaw=0
```

### 4.2 设置雷达安装 TF

修改前：

```python
arguments=['0', '0', '0', '0', '0', '0',
           'base_link', 'laser']
```

修改后：

```python
arguments=['0.036', '0', '0.155', '0', '0', '0',
           'base_link', 'laser']
```

最终关系：

```text
base_link -> laser
x=0.036m, y=0, z=0.155m, roll=0, pitch=0, yaw=0
```

### 4.3 将雷达 TF 节点加入启动流程

新增：

```python
ld.add_action(link_to_laser)
```

## 五、同步与构建

将修改后的文件同步到小车：

```text
/userdata/dev_ws/src/origincar/origincar_base/launch/origincar_bringup.launch.py
```

然后重新构建：

```bash
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select origincar_base --symlink-install
source install/setup.bash
```

## 六、板端验收

停止旧的底盘、Nav2 和静态 TF 进程后，重新启动底盘：

```bash
ros2 launch origincar_base origincar_bringup.launch.py carto_slam:=true
```

另开终端检查：

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

预期：

```text
Translation: [0.000, 0.000, 0.000]
Rotation RPY: [0.000, 0.000, 0.000]
```

再检查：

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

预期：

```text
Translation: [0.036, 0.000, 0.155]
Rotation RPY: [0.000, 0.000, 0.000]
```

还应确认：

```bash
ros2 node list | grep -E "base_to_link|link_to_laser"
```

预期同时存在：

```text
/base_to_link
/link_to_laser
```

## 七、RViz 复核方法

启动雷达后，在 RViz 中观察 `/scan`：

- 墙面方向正确但整体前后平移：以 0.005m 为步长调整雷达 `x`；
- 左右整体平移：调整雷达 `y`；
- 扫描整体旋转：调整 `yaw`，不要用 XYZ 补偿；
- 高度通常不影响二维 LaserScan 地图，但应保持与实际扫描平面一致。

当前 `(0.036, 0, 0.155)` 来自三视图比例估算，最终仍应以板端雷达与墙面
重合效果为准。

## 八、影响与回滚

### 影响

- 修正 TF 后，已有地图与当前雷达点可能出现位置差异；
- 如果旧地图是在错误 TF 下建立的，建议先验证墙面重合度；
- 差异明显时应使用正确 TF 重新建图。

### 回滚

如需临时恢复旧值：

```python
base_footprint -> base_link = (0.41, 0.12, 0)
base_link -> laser = (0, 0, 0)
```

并删除或注释：

```python
ld.add_action(link_to_laser)
```

旧值已被确认与实车尺寸不符，因此只建议用于问题对照，不建议继续导航。

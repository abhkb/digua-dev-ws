#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Minimal autonomous cruise node for the DG competition.

Flow:
  1. Publish the fixed initial pose at P.
  2. Navigate to the QR waypoint with Nav2 NavigateToPose.
  3. Wait for `/sign_foxglove` to become 3 or 4.
  4. Execute the configured QR post-detection maneuver and waypoint route,
     including optional post-arrival straight or arc motions.
  5. Return to P, publish zero `/cmd_vel`, and finish.

This node intentionally implements only the first closed loop
`P -> QR -> P`. Channel patrol and VLM are added later after this
navigation chain is stable on the real car.
"""

import math
import time
from pathlib import Path

import yaml

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Int32
from std_msgs.msg import String


class NavCruise(Node):
    """Run the minimal P -> QR -> P autonomous navigation flow."""

    def __init__(self):
        super().__init__('nav_cruise')

        package_share = Path(get_package_share_directory('dg_pkg'))
        default_params_file = (
            package_share / 'config' / 'competition_params.yaml'
        )

        self.declare_parameter('params_file', str(default_params_file))
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('autostart', True)
        self.declare_parameter('nav_action_name', '/navigate_to_pose')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('qr_signal_topic', '/sign_foxglove')
        self.declare_parameter('state_topic', '/competition/state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('start_delay_sec', -1.0)
        self.declare_parameter('qr_wait_timeout_sec', 60.0)
        self.declare_parameter('action_server_timeout_sec', 20.0)
        self.declare_parameter('stop_cmd_count', 10)
        self.declare_parameter('stop_cmd_interval_sec', 0.1)

        self.params_file = Path(self.get_parameter('params_file').value)
        self.publish_initial_pose_enabled = bool(
            self.get_parameter('publish_initial_pose').value
        )
        self.autostart = bool(self.get_parameter('autostart').value)
        self.nav_action_name = self.get_parameter('nav_action_name').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.qr_signal_topic = self.get_parameter('qr_signal_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.start_delay_sec = float(
            self.get_parameter('start_delay_sec').value
        )
        self.qr_wait_timeout_sec = float(
            self.get_parameter('qr_wait_timeout_sec').value
        )
        self.action_server_timeout_sec = float(
            self.get_parameter('action_server_timeout_sec').value
        )
        self.stop_cmd_count = int(self.get_parameter('stop_cmd_count').value)
        self.stop_cmd_interval_sec = float(
            self.get_parameter('stop_cmd_interval_sec').value
        )

        self.config = self._load_config(self.params_file)
        self.competition_config = self.config['dg_competition']
        self.map_frame = self.competition_config.get('map_frame', 'map')
        self.initial_pose = self.competition_config['initial_pose']
        self.waypoints = self.competition_config['waypoints']
        self.routes = self.competition_config.get('routes', {})
        self.navigation = self.competition_config.get('navigation', {})
        self.qr_config = self.competition_config.get('qr', {})

        self.initialpose_publish_count = int(
            self.navigation.get('initialpose_publish_count', 5)
        )
        self.initialpose_interval_sec = float(
            self.navigation.get('initialpose_interval_sec', 0.3)
        )
        if self.start_delay_sec < 0.0:
            self.start_delay_sec = float(
                self.navigation.get('start_delay_sec', 4.0)
            )
        self.nav_timeout_sec = float(
            self.navigation.get('nav_timeout_sec', 90.0)
        )
        self.auto_compute_goal_yaw = bool(
            self.navigation.get('auto_compute_goal_yaw', False)
        )
        self.default_yaw_goal_tolerance = float(
            self.navigation.get('default_yaw_goal_tolerance', 0.45)
        )
        self.ignore_yaw_goal_tolerance = float(
            self.navigation.get('ignore_yaw_goal_tolerance', 3.14)
        )
        self.controller_server_node = str(
            self.navigation.get('controller_server_node', '/controller_server')
        )
        self.clockwise_signal = int(self.qr_config.get('clockwise_signal', 3))
        self.anticlockwise_signal = int(
            self.qr_config.get('anticlockwise_signal', 4)
        )
        self.skip_qr_goal_on_signal = bool(
            self.qr_config.get('skip_qr_goal_on_signal', True)
        )

        self.latest_qr_signal = None

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            10,
        )
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.qr_signal_sub = self.create_subscription(
            Int32,
            self.qr_signal_topic,
            self._qr_signal_callback,
            10,
        )
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav_action_name,
        )
        self.controller_params_client = self.create_client(
            SetParameters,
            f'{self.controller_server_node}/set_parameters',
        )

        self.get_logger().info(
            '最小自动巡航节点已启动: '
            f'params_file={self.params_file}, '
            f'nav_action={self.nav_action_name}, '
            f'qr_signal={self.qr_signal_topic}'
        )

    def _route_waypoint_names(self):
        """读取 route_minimal 航点列表。

        后续增加中间点时，只需要修改 competition_params.yaml：

        routes:
          route_minimal:
            - "qr_stop"
            - "task13_channel_near"
            - "p_return"
        """

        route = self.routes.get('route_minimal', ['qr_stop', 'p_return'])

        if not isinstance(route, list):
            self.get_logger().warn(
                'routes.route_minimal 不是列表，回退为 qr_stop -> p_return'
            )
            return ['qr_stop', 'p_return']

        waypoint_names = [str(name) for name in route]

        if len(waypoint_names) < 2:
            self.get_logger().warn(
                'routes.route_minimal 航点少于 2 个，回退为 qr_stop -> p_return'
            )
            return ['qr_stop', 'p_return']

        missing = [
            name for name in waypoint_names
            if name not in self.waypoints
        ]
        if missing:
            raise RuntimeError(
                f'routes.route_minimal 中存在未定义航点: {missing}'
            )

        return waypoint_names

    def _set_controller_yaw_tolerance(self, tolerance):
        """Set Nav2 controller goal checker yaw tolerance if available."""

        if not self.controller_params_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '无法连接 controller_server 参数服务，'
                f'跳过 yaw_goal_tolerance={tolerance:.3f} 设置'
            )
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'general_goal_checker.yaw_goal_tolerance',
                Parameter.Type.DOUBLE,
                float(tolerance),
            ).to_parameter_msg()
        ]
        future = self.controller_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done() or future.result() is None:
            self.get_logger().warn(
                f'设置 yaw_goal_tolerance={tolerance:.3f} 超时或无响应'
            )
            return False

        results = future.result().results
        if not results or not results[0].successful:
            reason = results[0].reason if results else 'no result'
            self.get_logger().warn(
                f'设置 yaw_goal_tolerance={tolerance:.3f} 失败: {reason}'
            )
            return False

        self.get_logger().info(
            f'已设置 yaw_goal_tolerance={tolerance:.3f}'
        )
        return True

    def _apply_waypoint_goal_tolerance(self, waypoint_name):
        """Apply normal or yaw-ignored goal tolerance for a waypoint."""

        waypoint = self.waypoints[waypoint_name]
        ignore_yaw = bool(waypoint.get('ignore_yaw', False))

        if ignore_yaw:
            self.get_logger().info(
                f'航点 {waypoint_name} 设置为 ignore_yaw=true，'
                '只要求到点，不强求最终朝向'
            )
            self._set_controller_yaw_tolerance(
                self.ignore_yaw_goal_tolerance
            )
        else:
            self._set_controller_yaw_tolerance(
                self.default_yaw_goal_tolerance
            )

    def _load_config(self, path):
        if not path.is_file():
            raise FileNotFoundError(f'competition params not found: {path}')

        with path.open('r', encoding='utf-8-sig') as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict) or 'dg_competition' not in data:
            raise ValueError(
                f'Invalid competition params file: {path}'
            )

        return data

    def _qr_signal_callback(self, msg):
        if msg.data in (self.clockwise_signal, self.anticlockwise_signal):
            self.latest_qr_signal = msg.data
            self.get_logger().info(
                f'收到二维码方向信号: {msg.data} '
                f'({self._signal_to_direction(msg.data)})'
            )
        elif msg.data == 0:
            self.get_logger().warn('收到未知二维码信号: 0，继续等待 3/4')
        else:
            self.get_logger().warn(
                f'收到非预期二维码信号: {msg.data}，继续等待'
            )

    def _publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f'状态切换: {state}')

    def _signal_to_direction(self, signal):
        if signal == self.clockwise_signal:
            return 'ClockWise'
        if signal == self.anticlockwise_signal:
            return 'AntiClockWise'
        return 'unknown'

    def _has_valid_qr_signal(self):
        """Return whether a valid competition QR direction is available."""

        return self.latest_qr_signal in (
            self.clockwise_signal,
            self.anticlockwise_signal,
        )

    def _quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _pose_to_xy(self, pose_dict):
        return float(pose_dict['x']), float(pose_dict['y'])

    def _goal_yaw(self, previous_pose, target_pose):
        if self.auto_compute_goal_yaw:
            prev_x, prev_y = self._pose_to_xy(previous_pose)
            target_x, target_y = self._pose_to_xy(target_pose)
            dx = target_x - prev_x
            dy = target_y - prev_y
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                return math.atan2(dy, dx)

        if 'yaw' in target_pose:
            return float(target_pose['yaw'])

        orientation = target_pose.get('orientation', {})
        z = float(orientation.get('z', 0.0))
        w = float(orientation.get('w', 1.0))
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _make_pose_stamped(self, waypoint_name, previous_pose):
        waypoint = self.waypoints[waypoint_name]
        yaw = self._goal_yaw(previous_pose, waypoint)

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(waypoint['x'])
        pose.pose.position.y = float(waypoint['y'])
        pose.pose.position.z = float(waypoint.get('z', 0.0))
        pose.pose.orientation = self._quaternion_from_yaw(yaw)

        self.get_logger().info(
            f'目标点 {waypoint_name}: '
            f'x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}, '
            f'yaw={math.degrees(yaw):.1f} deg'
        )

        return pose

    def _make_initial_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = float(self.initial_pose['x'])
        pose.pose.pose.position.y = float(self.initial_pose['y'])
        pose.pose.pose.position.z = float(self.initial_pose.get('z', 0.0))

        if 'yaw' in self.initial_pose:
            pose.pose.pose.orientation = self._quaternion_from_yaw(
                float(self.initial_pose['yaw'])
            )
        else:
            orientation = self.initial_pose.get('orientation', {})
            pose.pose.pose.orientation.x = float(orientation.get('x', 0.0))
            pose.pose.pose.orientation.y = float(orientation.get('y', 0.0))
            pose.pose.pose.orientation.z = float(orientation.get('z', 0.0))
            pose.pose.pose.orientation.w = float(orientation.get('w', 1.0))

        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685

        return pose

    def publish_initial_pose(self):
        self._publish_state('INIT_PUBLISH_INITIALPOSE')
        pose = self._make_initial_pose()

        for index in range(self.initialpose_publish_count):
            pose.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(pose)
            self.get_logger().info(
                f'发布 /initialpose {index + 1}/'
                f'{self.initialpose_publish_count}'
            )
            end_time = time.monotonic() + self.initialpose_interval_sec
            while time.monotonic() < end_time and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_nav_server(self):
        self._publish_state('WAIT_NAV2_ACTION_SERVER')
        ok = self.nav_client.wait_for_server(
            timeout_sec=self.action_server_timeout_sec
        )
        if not ok:
            raise RuntimeError(
                f'Nav2 action server not available: {self.nav_action_name}'
            )

    def wait_start_delay(self):
        """Wait before sending the first Nav2 goal."""

        if self.start_delay_sec <= 0.0:
            return

        self._publish_state('START_DELAY')
        self.get_logger().info(
            f'发车倒计时 {self.start_delay_sec:.1f}s：'
            '倒计时结束后发送第一个导航目标'
        )

        start_time = time.monotonic()
        last_remaining = None
        while rclpy.ok():
            elapsed = time.monotonic() - start_time
            remaining = self.start_delay_sec - elapsed
            if remaining <= 0.0:
                break

            remaining_int = int(math.ceil(remaining))
            if remaining_int != last_remaining:
                self.get_logger().info(f'发车倒计时: {remaining_int}s')
                last_remaining = remaining_int

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('发车倒计时结束，开始自动巡航')

    def navigate_to(
            self, waypoint_name, previous_pose, stop_on_qr_signal=False):
        self._apply_waypoint_goal_tolerance(waypoint_name)
        waypoint = self.waypoints[waypoint_name]
        pose = self._make_pose_stamped(waypoint_name, previous_pose)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._publish_state(f'NAV_TO_{waypoint_name.upper()}')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self,
            send_goal_future,
            timeout_sec=10.0,
        )

        if not send_goal_future.done():
            raise RuntimeError(f'发送目标超时: {waypoint_name}')

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f'Nav2 拒绝目标: {waypoint_name}')

        self.get_logger().info(f'Nav2 已接受目标: {waypoint_name}')

        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        last_log_time = 0.0

        while rclpy.ok() and not result_future.done():
            if stop_on_qr_signal and self._has_valid_qr_signal():
                self._cancel_goal_after_early_qr(
                    waypoint_name,
                    goal_handle,
                    result_future,
                )
                return self.waypoints[waypoint_name]

            elapsed = time.monotonic() - start_time
            if elapsed > self.nav_timeout_sec:
                self.get_logger().error(
                    f'导航到 {waypoint_name} 超时，取消目标'
                )
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self,
                    cancel_future,
                    timeout_sec=5.0,
                )
                raise RuntimeError(f'导航超时: {waypoint_name}')

            if elapsed - last_log_time >= 5.0:
                self.get_logger().info(
                    f'正在导航到 {waypoint_name}，已用时 {elapsed:.1f}s'
                )
                last_log_time = elapsed

            rclpy.spin_once(self, timeout_sec=0.1)

        if not result_future.done():
            raise RuntimeError(f'导航结果未知: {waypoint_name}')

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f'导航失败: {waypoint_name}, status={result.status}'
            )

        self.get_logger().info(f'已到达目标: {waypoint_name}')
        self.execute_post_arrival_motion(waypoint_name)
        return self.waypoints[waypoint_name]

    def _cancel_goal_after_early_qr(
            self, waypoint_name, goal_handle, result_future):
        """Cancel the QR navigation goal after a valid early detection."""

        direction = self._signal_to_direction(self.latest_qr_signal)
        self._publish_state('QR_DETECTED_EARLY')
        self.get_logger().info(
            f'前往 {waypoint_name} 途中已识别二维码 '
            f'{self.latest_qr_signal} ({direction})，取消当前目标并继续下一点'
        )

        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self,
            cancel_future,
            timeout_sec=5.0,
        )
        if not cancel_future.done():
            raise RuntimeError(
                f'提前识别二维码后取消目标超时: {waypoint_name}'
            )

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=5.0,
        )
        if not result_future.done():
            raise RuntimeError(
                f'提前识别二维码后等待停车超时: {waypoint_name}'
            )

        status = result_future.result().status
        if status not in (
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_SUCCEEDED):
            raise RuntimeError(
                f'提前识别二维码后目标未安全取消: '
                f'{waypoint_name}, status={status}'
            )

        self.get_logger().info(
            f'已停止前往 {waypoint_name}，开始执行下一航点'
        )

    def execute_post_arrival_motion(self, waypoint_name):
        """Execute optional open-loop straight or arc waypoint motions."""

        waypoint = self.waypoints[waypoint_name]
        forward_distance_m = float(
            waypoint.get('post_arrival_forward_distance_m', 0.0)
        )
        forward_speed_mps = float(
            waypoint.get('post_arrival_forward_speed_mps', 0.0)
        )
        reverse_distance_m = float(
            waypoint.get('post_arrival_reverse_distance_m', 0.0)
        )
        reverse_speed_mps = float(
            waypoint.get('post_arrival_reverse_speed_mps', 0.0)
        )
        pre_turn_radius_m = max(
            0.0,
            float(waypoint.get('post_arrival_pre_turn_radius_m', 0.0)),
        )
        pre_turn_angle_deg = max(
            0.0,
            float(waypoint.get('post_arrival_pre_turn_angle_deg', 0.0)),
        )
        pre_turn_speed_mps = abs(float(
            waypoint.get('post_arrival_pre_turn_speed_mps', 0.0)
        ))
        pre_turn_direction = str(
            waypoint.get('post_arrival_pre_turn_direction', 'right')
        ).lower()
        return_to_pre_turn_pose = bool(
            waypoint.get('post_arrival_return_to_pre_turn_pose', False)
        )
        turn_radius_m = max(
            0.0,
            float(waypoint.get('post_arrival_turn_radius_m', 0.0)),
        )
        turn_direction = str(
            waypoint.get('post_arrival_turn_direction', 'right')
        ).lower()
        forward_duration_sec = max(
            0.0,
            float(waypoint.get(
                'post_arrival_forward_duration_sec',
                0.0,
            )),
        )
        reverse_duration_sec = max(
            0.0,
            float(waypoint.get(
                'post_arrival_reverse_duration_sec',
                0.0,
            )),
        )
        direction_change_pause_sec = max(
            0.0,
            float(waypoint.get(
                'post_arrival_direction_change_pause_sec',
                1.0,
            )),
        )

        has_pre_turn_arc = (
            pre_turn_radius_m > 0.0
            and pre_turn_angle_deg > 0.0
            and pre_turn_speed_mps > 0.0
        )
        has_forward_arc = (
            turn_radius_m > 0.0
            and forward_duration_sec > 0.0
            and abs(forward_speed_mps) > 0.0
        )
        has_reverse_arc = (
            turn_radius_m > 0.0
            and reverse_duration_sec > 0.0
            and abs(reverse_speed_mps) > 0.0
        )
        has_forward_motion = has_forward_arc or (
            forward_distance_m > 0.0 and abs(forward_speed_mps) > 0.0
        )
        has_reverse_motion = has_reverse_arc or (
            reverse_distance_m > 0.0 and abs(reverse_speed_mps) > 0.0
        )
        if has_pre_turn_arc or has_forward_motion or has_reverse_motion:
            settle_sec = max(
                0.0,
                float(waypoint.get('post_arrival_settle_sec', 1.2)),
            )
            self._wait_for_motion_channel_settle(waypoint_name, settle_sec)

        if has_pre_turn_arc:
            pre_turn_duration_sec = (
                math.radians(pre_turn_angle_deg)
                * pre_turn_radius_m
                / pre_turn_speed_mps
            )
            self._execute_open_loop_arc_motion(
                waypoint_name,
                '右前转弯' if pre_turn_direction == 'right' else '左前转弯',
                pre_turn_speed_mps,
                pre_turn_radius_m,
                pre_turn_duration_sec,
                pre_turn_direction,
                'POST_MOTION_PRE_TURN_ARC',
            )

        if has_forward_arc:
            self._execute_open_loop_arc_motion(
                waypoint_name,
                '右前弧' if turn_direction == 'right' else '左前弧',
                abs(forward_speed_mps),
                turn_radius_m,
                forward_duration_sec,
                turn_direction,
                'POST_MOTION_FORWARD_ARC',
            )
        else:
            self._execute_open_loop_linear_motion(
                waypoint_name,
                '前进',
                forward_distance_m,
                forward_speed_mps,
            )
        if has_forward_motion and has_reverse_motion:
            self._hold_motion_stop(
                waypoint_name,
                direction_change_pause_sec,
            )
        if has_reverse_arc:
            self._execute_open_loop_arc_motion(
                waypoint_name,
                '右后弧' if turn_direction == 'right' else '左后弧',
                -abs(reverse_speed_mps),
                turn_radius_m,
                reverse_duration_sec,
                turn_direction,
                'POST_MOTION_REVERSE_ARC',
            )
        else:
            self._execute_open_loop_linear_motion(
                waypoint_name,
                '后退',
                reverse_distance_m,
                -abs(reverse_speed_mps),
            )

        if has_pre_turn_arc and return_to_pre_turn_pose:
            pre_turn_duration_sec = (
                math.radians(pre_turn_angle_deg)
                * pre_turn_radius_m
                / pre_turn_speed_mps
            )
            self._execute_open_loop_arc_motion(
                waypoint_name,
                '右后弧返回转向前' if pre_turn_direction == 'right'
                else '左后弧返回转向前',
                -pre_turn_speed_mps,
                pre_turn_radius_m,
                pre_turn_duration_sec,
                pre_turn_direction,
                'POST_MOTION_RETURN_TURN_ARC',
            )

    def execute_qr_post_detection_motion(self):
        """Execute the configured motion after a valid QR signal."""

        motion = self.qr_config.get('post_detection_motion', {})
        if not isinstance(motion, dict) or not bool(
                motion.get('enabled', False)):
            return

        motion_type = str(motion.get('type', 'arc')).lower()
        if motion_type != 'arc':
            raise RuntimeError(
                f'不支持的 QR 后动作类型: {motion_type}'
            )

        radius_m = float(motion.get('radius_m', 0.0))
        angle_deg = float(motion.get('angle_deg', 0.0))
        speed_mps = abs(float(motion.get('speed_mps', 0.0)))
        travel_direction = str(
            motion.get('travel_direction', 'reverse')
        ).lower()
        turn_direction = str(
            motion.get('turn_direction', 'right')
        ).lower()

        if radius_m <= 0.0 or angle_deg <= 0.0 or speed_mps <= 0.0:
            raise RuntimeError(
                'QR 后弧线动作的 radius_m、angle_deg、speed_mps '
                '必须大于 0'
            )
        if travel_direction not in ('forward', 'reverse'):
            raise RuntimeError(
                f'不支持的 QR 后行驶方向: {travel_direction}'
            )

        settle_sec = max(0.0, float(motion.get('settle_sec', 1.2)))
        self._wait_for_motion_channel_settle('qr_stop', settle_sec)

        signed_speed_mps = (
            speed_mps if travel_direction == 'forward' else -speed_mps
        )
        duration_sec = math.radians(angle_deg) * radius_m / speed_mps
        motion_name = (
            ('右' if turn_direction == 'right' else '左')
            + ('前弧' if travel_direction == 'forward' else '后弧')
        )
        self._execute_open_loop_arc_motion(
            'qr_stop',
            motion_name,
            signed_speed_mps,
            radius_m,
            duration_sec,
            turn_direction,
            'POST_QR_REVERSE_ARC',
        )

    def _wait_for_motion_channel_settle(self, waypoint_name, settle_sec):
        """Wait for Nav2 velocity smoother output to become idle."""

        if settle_sec <= 0.0:
            return

        self.get_logger().info(
            f'航点 {waypoint_name} 到达后等待速度链静置 '
            f'{settle_sec:.2f}s'
        )
        end_time = time.monotonic() + settle_sec
        while time.monotonic() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _execute_open_loop_linear_motion(
            self, waypoint_name, motion_name, distance_m, speed_mps):
        """Publish one timed, straight open-loop linear motion."""

        distance_m = abs(float(distance_m))
        speed_mps = float(speed_mps)

        if distance_m <= 0.0 or abs(speed_mps) <= 0.0:
            return

        duration_sec = distance_m / abs(speed_mps)
        twist = Twist()
        twist.linear.x = speed_mps
        twist.angular.z = 0.0

        self.get_logger().info(
            f'航点 {waypoint_name} 到达后执行{motion_name}: '
            f'distance={distance_m:.3f}m, speed={speed_mps:.3f}m/s, '
            f'duration={duration_sec:.2f}s'
        )
        self._publish_state(
            f'POST_MOTION_{"FORWARD" if speed_mps > 0.0 else "REVERSE"}'
        )

        start_time = time.monotonic()
        publish_interval_sec = 0.1
        while rclpy.ok() and (time.monotonic() - start_time) < duration_sec:
            self.cmd_vel_pub.publish(twist)
            end_time = time.monotonic() + publish_interval_sec
            while time.monotonic() < end_time and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)

        self._publish_motion_stop()

    def _execute_open_loop_arc_motion(
            self, waypoint_name, motion_name, speed_mps, radius_m,
            duration_sec, turn_direction, state_name):
        """Publish one timed constant-radius open-loop arc motion."""

        speed_mps = float(speed_mps)
        radius_m = abs(float(radius_m))
        duration_sec = max(0.0, float(duration_sec))
        turn_direction = str(turn_direction).lower()

        if abs(speed_mps) <= 0.0 or radius_m <= 0.0 or duration_sec <= 0.0:
            return
        if turn_direction not in ('left', 'right'):
            raise RuntimeError(
                f'不支持的转弯方向: {turn_direction}'
            )

        curvature_sign = -1.0 if turn_direction == 'right' else 1.0
        angular_z = speed_mps * curvature_sign / radius_m
        angle_deg = math.degrees(
            abs(angular_z) * duration_sec
        )

        twist = Twist()
        twist.linear.x = speed_mps
        twist.angular.z = angular_z

        self.get_logger().info(
            f'航点 {waypoint_name} 执行{motion_name}: '
            f'speed={speed_mps:.3f}m/s, radius={radius_m:.3f}m, '
            f'angular_z={angular_z:.3f}rad/s, '
            f'angle={angle_deg:.1f}deg, duration={duration_sec:.2f}s'
        )
        self._publish_state(state_name)

        start_time = time.monotonic()
        publish_interval_sec = 0.1
        while rclpy.ok() and (time.monotonic() - start_time) < duration_sec:
            self.cmd_vel_pub.publish(twist)
            end_time = time.monotonic() + publish_interval_sec
            while time.monotonic() < end_time and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)

        self._publish_motion_stop()

    def _hold_motion_stop(self, waypoint_name, pause_sec):
        """Hold zero velocity before changing travel direction."""

        if pause_sec <= 0.0:
            return

        self._publish_state('POST_MOTION_DIRECTION_CHANGE_PAUSE')
        self.get_logger().info(
            f'航点 {waypoint_name} 前进结束，保持零速 '
            f'{pause_sec:.2f}s 后再后退'
        )
        stop_msg = Twist()
        end_time = time.monotonic() + pause_sec
        while time.monotonic() < end_time and rclpy.ok():
            self.cmd_vel_pub.publish(stop_msg)
            interval_end = time.monotonic() + 0.1
            while time.monotonic() < interval_end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)

    def _publish_motion_stop(self):
        """Publish a short zero-velocity burst between open-loop motions."""

        stop_msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            end_time = time.monotonic() + 0.05
            while time.monotonic() < end_time and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_qr_signal(self):
        if self._has_valid_qr_signal():
            direction = self._signal_to_direction(self.latest_qr_signal)
            self.get_logger().info(
                f'复用途中已收到的二维码方向信号: '
                f'{self.latest_qr_signal} ({direction})'
            )
            self._publish_state(f'QR_{direction}')
            return self.latest_qr_signal

        self._publish_state('WAIT_QR_SIGNAL')
        self.get_logger().info(
            f'等待二维码方向信号 {self.clockwise_signal}/'
            f'{self.anticlockwise_signal}，超时 '
            f'{self.qr_wait_timeout_sec:.1f}s'
        )

        start_time = time.monotonic()
        while rclpy.ok():
            if self._has_valid_qr_signal():
                direction = self._signal_to_direction(self.latest_qr_signal)
                self._publish_state(f'QR_{direction}')
                return self.latest_qr_signal

            elapsed = time.monotonic() - start_time
            if self.qr_wait_timeout_sec > 0.0:
                if elapsed > self.qr_wait_timeout_sec:
                    raise RuntimeError('等待二维码方向信号超时')

            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_stop(self):
        self._publish_state('STOP')
        stop_msg = Twist()
        for index in range(self.stop_cmd_count):
            self.cmd_vel_pub.publish(stop_msg)
            self.get_logger().info(
                f'发布停车 /cmd_vel 0 {index + 1}/{self.stop_cmd_count}'
            )
            end_time = time.monotonic() + self.stop_cmd_interval_sec
            while time.monotonic() < end_time and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)

    def run_mission(self):
        if not self.autostart:
            self.get_logger().info('autostart=false，节点保持等待')
            self._publish_state('IDLE')
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2)
            return

        try:
            self._publish_state('INIT')
            if self.publish_initial_pose_enabled:
                self.publish_initial_pose()

            self._wait_for_nav_server()
            self.wait_start_delay()

            route = self._route_waypoint_names()
            self.get_logger().info(
                '最小自动巡航路线: P -> ' + ' -> '.join(route)
            )

            current_pose = self.initial_pose
            current_pose = self.navigate_to(
                route[0],
                current_pose,
                stop_on_qr_signal=(
                    self.skip_qr_goal_on_signal
                    and route[0] == 'qr_stop'
                ),
            )
            self.wait_for_qr_signal()
            self.execute_qr_post_detection_motion()

            for waypoint_name in route[1:]:
                current_pose = self.navigate_to(waypoint_name, current_pose)

            self.publish_stop()
            self._publish_state('FINISH')
            self.get_logger().info(
                '最小自动巡航完成: P -> ' + ' -> '.join(route)
            )

        except Exception as exc:
            self.get_logger().error(f'最小自动巡航失败: {exc}')
            self.publish_stop()
            self._publish_state('ERROR')
            raise


def main(args=None):
    rclpy.init(args=args)
    node = NavCruise()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.publish_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

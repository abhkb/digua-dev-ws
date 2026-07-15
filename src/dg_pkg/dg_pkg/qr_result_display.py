#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Display QR direction results on the HDMI screen using Tkinter.

The node subscribes to `/sign_foxglove`:

- 3: load and display clockwise text.
- 4: load and display anticlockwise text.

It also subscribes to `/qr_code/data` and displays the raw QR content.

Text content is kept in `.txt` files so competition wording can be changed
without modifying code.
"""

from pathlib import Path
import tkinter as tk
from tkinter import font as tkfont

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Int32
from std_msgs.msg import String


class QRResultDisplay(Node):
    """Fullscreen Tkinter display for QR direction results."""

    def __init__(self):
        super().__init__('qr_result_display')

        package_share = Path(get_package_share_directory('dg_pkg'))
        default_text_dir = package_share / 'config' / 'display_text'

        self.declare_parameter('signal_topic', '/sign_foxglove')
        self.declare_parameter('raw_data_topic', '/qr_code/data')
        self.declare_parameter(
            'clockwise_text_file',
            str(default_text_dir / 'clockwise.txt'),
        )
        self.declare_parameter(
            'anticlockwise_text_file',
            str(default_text_dir / 'anticlockwise.txt'),
        )
        self.declare_parameter(
            'waiting_text_file',
            str(default_text_dir / 'waiting.txt'),
        )
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('window_width', 800)
        self.declare_parameter('window_height', 480)
        self.declare_parameter('title_font_size', 64)
        self.declare_parameter('body_font_size', 44)

        self.signal_topic = self.get_parameter('signal_topic').value
        self.raw_data_topic = self.get_parameter('raw_data_topic').value
        self.clockwise_text_file = Path(
            self.get_parameter('clockwise_text_file').value
        )
        self.anticlockwise_text_file = Path(
            self.get_parameter('anticlockwise_text_file').value
        )
        self.waiting_text_file = Path(
            self.get_parameter('waiting_text_file').value
        )
        self.fullscreen = bool(self.get_parameter('fullscreen').value)
        self.window_width = int(self.get_parameter('window_width').value)
        self.window_height = int(self.get_parameter('window_height').value)
        self.title_font_size = int(
            self.get_parameter('title_font_size').value
        )
        self.body_font_size = int(self.get_parameter('body_font_size').value)

        self.current_signal = 0
        self.current_raw_data = ''
        self.current_title = '等待二维码'
        self.current_body = self._read_text_file(self.waiting_text_file)
        self._closing = False

        self._init_tk()
        self._render()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            Int32,
            self.signal_topic,
            self._signal_callback,
            qos,
        )
        self.raw_data_subscription = self.create_subscription(
            String,
            self.raw_data_topic,
            self._raw_data_callback,
            qos,
        )

        self.get_logger().info(
            '二维码结果屏幕节点启动 — '
            f'订阅: {self.signal_topic}, '
            f'原始数据订阅: {self.raw_data_topic}, '
            f'顺时针文本: {self.clockwise_text_file}, '
            f'逆时针文本: {self.anticlockwise_text_file}'
        )

    def _init_tk(self):
        self.root = tk.Tk()
        self.root.title('QR Direction Result')
        self.root.configure(bg='#080c12')

        if self.fullscreen:
            self.root.attributes('-fullscreen', True)
            self.root.update_idletasks()
            self.window_width = self.root.winfo_screenwidth()
            self.window_height = self.root.winfo_screenheight()
        else:
            self.root.geometry(f'{self.window_width}x{self.window_height}')

        self.root.bind('<Escape>', self._on_close)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        font_family = self._choose_font_family()
        self.title_font = tkfont.Font(
            family=font_family,
            size=self.title_font_size,
            weight='bold',
        )
        self.body_font = tkfont.Font(
            family=font_family,
            size=self.body_font_size,
            weight='bold',
        )

        self.title_label = tk.Label(
            self.root,
            text='',
            bg='#080c12',
            fg='#ffd250',
            font=self.title_font,
        )
        self.title_label.pack(pady=(45, 25))

        self.body_label = tk.Label(
            self.root,
            text='',
            bg='#080c12',
            fg='#f5f5f5',
            font=self.body_font,
            justify='center',
            wraplength=max(300, self.window_width - 80),
        )
        self.body_label.pack(expand=True, fill='both', padx=40, pady=20)

    def _choose_font_family(self):
        candidates = [
            'Noto Sans CJK SC',
            'Noto Sans CJK',
            'WenQuanYi Zen Hei',
            'WenQuanYi Micro Hei',
            'Microsoft YaHei',
            'SimHei',
            'DejaVu Sans',
        ]
        available = set(tkfont.families())
        for name in candidates:
            if name in available:
                return name
        return 'TkDefaultFont'

    def _read_text_file(self, path):
        try:
            return path.read_text(encoding='utf-8-sig').strip()
        except OSError as exc:
            self.get_logger().error(f'读取显示文本失败: {path}: {exc}')
            return f'无法读取文本文件:\n{path}'

    def _signal_callback(self, msg):
        if msg.data == self.current_signal and msg.data != 0:
            return

        self.current_signal = msg.data

        if msg.data == 3:
            self.current_title = '顺时针'
            self.current_body = self._read_text_file(
                self.clockwise_text_file
            )
        elif msg.data == 4:
            self.current_title = '逆时针'
            self.current_body = self._read_text_file(
                self.anticlockwise_text_file
            )
        else:
            self.current_title = '未知二维码'
            self.current_body = self._read_text_file(self.waiting_text_file)

        self._render()
        self.get_logger().info(
            f'收到二维码方向信号: {msg.data}，显示: {self.current_title}'
        )

    def _raw_data_callback(self, msg):
        self.current_raw_data = msg.data
        self._render()
        self.get_logger().info(
            f'收到二维码原始数据: "{self.current_raw_data}"'
        )

    def _render(self):
        self.title_label.configure(
            text=self.current_title,
            fg=self._title_color(),
        )
        self.body_label.configure(text=self._compose_body_text())

    def _compose_body_text(self):
        if not self.current_raw_data:
            return self.current_body

        return (
            f'{self.current_body}\n\n'
            f'二维码原始数据：{self.current_raw_data}'
        )

    def _title_color(self):
        if self.current_signal == 3:
            return '#00dc78'
        if self.current_signal == 4:
            return '#50aaff'
        return '#ffd250'

    def _spin_once(self):
        if self._closing or not rclpy.ok():
            self._on_close()
            return

        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(50, self._spin_once)

    def _on_close(self, event=None):
        self._closing = True
        try:
            self.root.quit()
        except tk.TclError:
            pass

    def run(self):
        self.root.after(50, self._spin_once)
        self.root.mainloop()

    def destroy_node(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRResultDisplay()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

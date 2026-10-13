import numpy as np

# 强制用 TkAgg 后端。
# 不写这行的话，如果 DISPLAY 有问题，matplotlib 会"静默"退回 Agg（无窗口）后端，
# plt.show() 立刻返回、什么都不显示 —— 很难查。写死后至少会报错。
import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.x = 0.0          # 小车位置
        self.th = 0.05        # 摆杆角度
        self.l = 0.5          # 摆杆半长（和 plant 里定义一致）
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.on_state, 10)
        self.get_logger().info(f'可视化节点启动，后端 = {matplotlib.get_backend()}')

    def on_state(self, msg):
        self.x = msg.position[0]
        self.th = msg.position[1]


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.set_xlim(-3, 3)
    ax.set_ylim(-0.9, 0.9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='k', lw=1)
    ax.set_title('倒立摆   (Ctrl+C 退出)')

    cart, = ax.plot([], [], 's', color='tab:blue', markersize=26)
    pole, = ax.plot([], [], '-', color='tab:red', lw=4)
    label = ax.text(-2.9, 0.72, '', family='monospace', fontsize=11)

    def update(_frame):
        # ★ 关键：主循环被 matplotlib 抢走了，所以每帧手动处理一次 ROS2 消息
        rclpy.spin_once(node, timeout_sec=0.01)

        w = 0.3
        cart.set_data([node.x - w / 2, node.x + w / 2], [0, 0])

        tip_x = node.x + node.l * np.sin(node.th)    # 杆端坐标
        tip_y = node.l * np.cos(node.th)
        pole.set_data([node.x, tip_x], [0, tip_y])

        label.set_text(f'x     = {node.x:+.3f} m\ntheta = {node.th:+.4f} rad')
        return cart, pole, label

    ani = animation.FuncAnimation(
        fig, update, interval=30, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

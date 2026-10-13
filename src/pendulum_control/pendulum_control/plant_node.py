import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class PlantNode(Node):
    def __init__(self):
        super().__init__('plant')

        # 物理参数
        self.M = 1.0        # 小车质量 kg
        self.m = 0.1        # 摆杆质量 kg
        self.l = 0.5        # 摆杆半长 m
        self.g = 9.81

        # 状态向量 [x, x_dot, theta, theta_dot]
        self.state = np.array([0.0, 0.0, 0.05, 0.0])
        self.force = 0.0
        self.dt = 0.01

        # 出：状态
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        # 入：控制力
        self.sub = self.create_subscription(
            Float64, '/pendulum/cmd_force', self.on_force, 10)

        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info('plant 启动，等待控制力...')

    def on_force(self, msg):
        self.force = msg.data

    def dynamics(self, s, F):
        x, xd, th, thd = s
        s_th, c_th = np.sin(th), np.cos(th)

        th_dd = ((self.M + self.m) * self.g * s_th - F * c_th
                 - self.m * self.l * thd**2 * s_th * c_th) \
                / (self.l * (self.M + self.m * s_th**2))
        x_dd = (F - self.m * self.l * (th_dd * c_th - thd**2 * s_th)) / (self.M + self.m)

        return np.array([xd, x_dd, thd, th_dd])

    def rk4(self, s, F, dt):
        k1 = self.dynamics(s, F)
        k2 = self.dynamics(s + dt / 2 * k1, F)
        k3 = self.dynamics(s + dt / 2 * k2, F)
        k4 = self.dynamics(s + dt * k3, F)
        return s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def on_timer(self):
        self.state = self.rk4(self.state, self.force, self.dt)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['cart_joint', 'pole_joint']
        msg.position = [float(self.state[0]), float(self.state[2])]
        msg.velocity = [float(self.state[1]), float(self.state[3])]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlantNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

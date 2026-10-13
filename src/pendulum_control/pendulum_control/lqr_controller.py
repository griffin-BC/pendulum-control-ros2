import numpy as np
import rclpy
from scipy.linalg import solve_continuous_are
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class LQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')

        # 模型参数 —— 必须和 plant_node.py 一致
        M, m, l, g = 1.0, 0.1, 0.5, 9.81

        # 直立点线性化:  X_dot = A X + B u
        A = np.array([
            [0, 1, 0, 0],
            [0, 0, -m * g / M, 0],
            [0, 0, 0, 1],
            [0, 0, (M + m) * g / (l * M), 0],
        ])
        B = np.array([[0.0], [1.0 / M], [0.0], [-1.0 / (l * M)]])

        # 代价函数权重
        Q = np.diag([10.0, 1.0, 100.0, 1.0])   # [x, x_dot, theta, theta_dot]
        R = np.array([[1.0]])

        # 解代数黎卡提方程
        P = solve_continuous_are(A, B, Q, R)
        self.K = (np.linalg.inv(R) @ B.T @ P).flatten()

        self.get_logger().info(f'LQR 增益 K = {np.round(self.K, 3)}')

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.on_state, 10)
        self.pub = self.create_publisher(Float64, '/pendulum/cmd_force', 10)

    def on_state(self, msg):
        X = np.array([
            msg.position[0],   # x
            msg.velocity[0],   # x_dot
            msg.position[1],   # theta
            msg.velocity[1],   # theta_dot
        ])

        u = -self.K @ X

        out = Float64()
        out.data = float(u)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')

        self.Kp = 40.0
        self.Kd = 8.0

        # 入：状态
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.on_state, 10)
        # 出：控制力
        self.pub = self.create_publisher(Float64, '/pendulum/cmd_force', 10)

        self.get_logger().info(f'PD 控制器启动 Kp={self.Kp} Kd={self.Kd}')

    def on_state(self, msg):
        x   = msg.position[0]
        th  = msg.position[1]
        xd  = msg.velocity[0]
        thd = msg.velocity[1]

        # u = Kp*theta + Kd*theta_dot   (正反馈方向: 摆往右倒 -> 小车往右推)
        F = self.Kp * th + self.Kd * thd

        out = Float64()
        out.data = F
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

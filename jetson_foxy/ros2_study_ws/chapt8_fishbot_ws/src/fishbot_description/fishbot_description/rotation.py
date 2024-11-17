
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import threading
import time

class RotationWheelNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"node {name} started")

        self.joint_state_publisher_ = self.create_publisher(JointState, 'joint_states', 10) 

        # init

        self._init_joint_state()
        self.pub_rate = self.create_rate(30)
        self.thread_ = threading.Thread(target=self._thread_pub)

        self.thread_.start()

    def _init_joint_state(self):
        self.joint_speeds = [0.0, 0.0]
        self.joint_state_ = JointState()
        self.joint_state_.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_.header.frame_id = ""
        self.joint_state_.name = ["left_wheel_joint", "right_wheel_joint"]
        self.joint_state_.position = [0.0, 0.0]
        self.joint_state_.velocity = self.joint_speeds
        self.joint_state_.effort = []

    def update_speed(self,speeds):
        self.joint_speeds = speeds

    def _thread_pub(self):
        last_update_time = time.time()
        while rclpy.ok():
            delta_time =  time.time()-last_update_time
            last_update_time = time.time()
            # 更新位置
            self.joint_state_.position[0]  += delta_time*self.joint_state_.velocity[0]
            self.joint_state_.position[1]  += delta_time*self.joint_state_.velocity[1]
            # 更新速度
            self.joint_state_.velocity = self.joint_speeds
            # 更新 header
            self.joint_state_.header.stamp = self.get_clock().now().to_msg()
            # 发布关节数据
            self.joint_state_publisher_.publish(self.joint_state_)
            self.pub_rate.sleep()

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = RotationWheelNode("rotate_fishbot_wheel")  # 新建一个节点
    node.update_speed([15.0,-15.0])
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
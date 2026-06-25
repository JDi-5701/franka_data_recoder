"""Publish continuous fake data on every topic type the recorder/GUI uses, so the GUI
(and the recorder) can be exercised WITHOUT the real robot.

Topics (match config/recorder.yaml + the streams we'll add later):
  /cartesian_impedance_node/current_pose   geometry_msgs/PoseStamped
  /cartesian_impedance_node/target_pose    geometry_msgs/PoseStamped
  /cartesian_impedance_node/ext_wrench     geometry_msgs/WrenchStamped
  /franka_gripper/joint_states             sensor_msgs/JointState   (gripper width)
  /franka/joint_states                     sensor_msgs/JointState   (7 arm joints)
  /camera/wrist/color/image_raw            sensor_msgs/Image        (moving gradient)

Run (conda ros_ml):  ros2 run franka_data_recorder fake
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState, Image


class FakePublisher(Node):
    def __init__(self):
        super().__init__('fake_data_publisher')
        self.rate = float(self.declare_parameter('rate', 30.0).value)
        self.cur = self.create_publisher(PoseStamped, '/cartesian_impedance_node/current_pose', 10)
        self.tgt = self.create_publisher(PoseStamped, '/cartesian_impedance_node/target_pose', 10)
        self.wr = self.create_publisher(WrenchStamped, '/cartesian_impedance_node/ext_wrench', 10)
        self.grip = self.create_publisher(JointState, '/franka_gripper/joint_states', 10)
        self.joints = self.create_publisher(JointState, '/franka/joint_states', 10)
        self.img = self.create_publisher(Image, '/camera/wrist/color/image_raw', 10)
        self.t = 0.0
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'publishing fake data @ {self.rate} Hz '
                               '(pose/target/wrench/gripper/joints/image)')

    def _pose(self, stamp, phase):
        m = PoseStamped()
        m.header.stamp = stamp
        m.header.frame_id = 'base'
        m.pose.position.x = 0.40 + 0.05 * math.sin(self.t + phase)
        m.pose.position.y = 0.00 + 0.05 * math.cos(self.t + phase)
        m.pose.position.z = 0.40 + 0.02 * math.sin(0.5 * self.t + phase)
        m.pose.orientation.x = 1.0
        m.pose.orientation.w = 0.0
        return m

    def tick(self):
        self.t += 1.0 / self.rate
        t = self.t
        now = self.get_clock().now().to_msg()

        self.cur.publish(self._pose(now, 0.0))
        self.tgt.publish(self._pose(now, 0.25))

        wr = WrenchStamped()
        wr.header.stamp = now
        wr.header.frame_id = 'base'
        wr.wrench.force.x = 2.0 * math.sin(t)
        wr.wrench.force.y = 1.5 * math.cos(0.7 * t)
        wr.wrench.force.z = 1.0 * math.sin(0.3 * t)
        wr.wrench.torque.z = 0.2 * math.sin(2.0 * t)
        self.wr.publish(wr)

        gj = JointState()
        gj.header.stamp = now
        w = 0.02 + 0.02 * abs(math.sin(0.4 * t))
        gj.name = ['panda_finger_joint1', 'panda_finger_joint2']
        gj.position = [w, w]
        self.grip.publish(gj)

        aj = JointState()
        aj.header.stamp = now
        aj.name = [f'panda_joint{i + 1}' for i in range(7)]
        aj.position = [0.3 * math.sin(t + i) for i in range(7)]
        aj.velocity = [0.3 * math.cos(t + i) for i in range(7)]
        aj.effort = [0.5 * math.sin(0.5 * t + i) for i in range(7)]
        self.joints.publish(aj)

        h, w_ = 240, 320
        xx = (np.arange(w_, dtype=np.uint16)[None, :] + int(t * 40)) % 256
        yy = (np.arange(h, dtype=np.uint16)[:, None]) % 256
        rgb = np.zeros((h, w_, 3), np.uint8)
        rgb[..., 0] = xx
        rgb[..., 1] = yy
        rgb[..., 2] = (xx + yy) % 256
        im = Image()
        im.header.stamp = now
        im.header.frame_id = 'camera'
        im.height, im.width = h, w_
        im.encoding = 'rgb8'
        im.is_bigendian = 0
        im.step = w_ * 3
        im.data = rgb.tobytes()
        self.img.publish(im)


def main():
    rclpy.init()
    node = FakePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

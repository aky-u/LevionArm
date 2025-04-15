import sys
import math
from PyQt6.QtCore import Qt, QPointF, QTimer
from PyQt6.QtGui import QPainter, QColor, QBrush
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# from urdfpy import URDF


class JointVelocityGUI(Node):
    def __init__(self):
        super().__init__("joint_velocity_gui")
        self.publisher = self.create_publisher(
            JointState, "/joint_velocity_command", 10
        )
        self.robot_description_received = False
        self.joint_names = []
        self.joint_sliders = {}
        self.joint_velocities = {}

        self.subscription = self.create_subscription(
            String, "/robot_description", self.robot_description_callback, 10
        )

        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.layout = QVBoxLayout()
        self.window.setWindowTitle("Joint Velocity GUI")
        self.window.setLayout(self.layout)

        # Setup timer to periodically publish velocities
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_velocities)
        self.timer.start(100)  # 10 Hz

    def robot_description_callback(self, msg):
        if self.robot_description_received:
            return

        try:
            robot = URDF.load_from_string(msg.data)
            self.joint_names = [
                joint.name for joint in robot.joints if joint.joint_type != "fixed"
            ]
            self.robot_description_received = True
            self.get_logger().info(f"Found joints: {self.joint_names}")
            self.setup_gui()
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")

    def setup_gui(self):
        for name in self.joint_names:
            label = QLabel(name)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-100)
            slider.setMaximum(100)
            slider.setValue(0)
            slider.valueChanged.connect(self.slider_changed(name))
            self.joint_sliders[name] = slider
            self.joint_velocities[name] = 0.0

            self.layout.addWidget(label)
            self.layout.addWidget(slider)

        self.window.show()

    def slider_changed(self, joint_name):
        def callback(value):
            velocity = value / 10.0  # scale for finer control
            self.joint_velocities[joint_name] = velocity

        return callback

    def publish_velocities(self):
        if not self.joint_names:
            return

        msg = JointState()
        msg.name = self.joint_names
        msg.velocity = [self.joint_velocities[name] for name in self.joint_names]
        self.publisher.publish(msg)

    def run(self):
        sys.exit(self.app.exec())


def main(args=None):
    rclpy.init(args=args)
    node = JointVelocityGUI()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

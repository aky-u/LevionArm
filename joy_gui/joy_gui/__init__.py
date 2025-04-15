# joystick_gui.py

import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt6.QtCore import Qt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class JoystickNode(Node):
    def __init__(self):
        super().__init__("joystick_gui")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

    def publish_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)


class JoystickGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("ROS2 Joystick GUI")

        layout = QVBoxLayout()

        self.label_linear = QLabel("Linear Velocity: 0.0")
        self.slider_linear = QSlider(Qt.Orientation.Horizontal)
        self.slider_linear.setRange(-100, 100)
        self.slider_linear.setValue(0)
        self.slider_linear.valueChanged.connect(self.update_twist)

        self.label_angular = QLabel("Angular Velocity: 0.0")
        self.slider_angular = QSlider(Qt.Orientation.Horizontal)
        self.slider_angular.setRange(-100, 100)
        self.slider_angular.setValue(0)
        self.slider_angular.valueChanged.connect(self.update_twist)

        layout.addWidget(self.label_linear)
        layout.addWidget(self.slider_linear)
        layout.addWidget(self.label_angular)
        layout.addWidget(self.slider_angular)

        self.setLayout(layout)

    def update_twist(self):
        linear = self.slider_linear.value() / 100.0  # scale to [-1.0, 1.0]
        angular = self.slider_angular.value() / 100.0
        self.label_linear.setText(f"Linear Velocity: {linear:.2f}")
        self.label_angular.setText(f"Angular Velocity: {angular:.2f}")
        self.ros_node.publish_twist(linear, angular)


def main():
    rclpy.init()
    ros_node = JoystickNode()

    app = QApplication(sys.argv)
    gui = JoystickGUI(ros_node)
    gui.show()

    timer = ros_node.create_timer(
        0.1, lambda: None
    )  # Create a timer to keep the node alive

    def qt_spin():
        rclpy.spin_once(ros_node, timeout_sec=0.01)

    # QtのタイマーでROSノードのスピンを継続
    from PyQt6.QtCore import QTimer

    ros_timer = QTimer()
    ros_timer.timeout.connect(qt_spin)
    ros_timer.start(10)

    sys.exit(app.exec())

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

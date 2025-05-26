#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QSlider,
    QLabel,
    QHBoxLayout,
)
from PyQt5.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class SmartSlider(QSlider):
    """Custom QSlider that resets value to zero on mouse release."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def mouseReleaseEvent(self, event):
        self.setValue(0)  # Reset to zero
        super().mouseReleaseEvent(event)


class JointVelocityGui(Node):
    def __init__(self):
        super().__init__("joint_velocity_slider_gui")

        # Declare and read parameter
        self.declare_parameter("controller_name", "forward_velocity_controller")
        controller_name = self.get_parameter("controller_name").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(
            Float64MultiArray, f"{controller_name}/commands", 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.joint_names = []
        self.sliders = []
        self.joint_name_to_slider = {}

        self.gui_ready = False

        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Joint Velocity GUI")
        self.layout = QVBoxLayout()
        self.window.setLayout(self.layout)
        self.window.show()

        # Timer for publishing
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_velocities)
        self.timer.start(20)  # 50Hz

    def joint_state_callback(self, msg: JointState):
        if not self.gui_ready:
            self.joint_names = msg.name
            self.create_sliders()
            self.gui_ready = True

    def create_sliders(self):
        for name in self.joint_names:
            row = QHBoxLayout()
            label = QLabel(name)
            slider = SmartSlider(Qt.Horizontal)
            slider.setMinimum(-100)
            slider.setMaximum(100)
            slider.setValue(0)
            slider.setTickInterval(1)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.valueChanged.connect(self.publish_velocities)
            self.sliders.append(slider)
            self.joint_name_to_slider[name] = slider
            row.addWidget(label)
            row.addWidget(slider)
            self.layout.addLayout(row)

    def publish_velocities(self):
        if not self.gui_ready:
            return
        msg = Float64MultiArray()
        msg.data = [self.joint_name_to_slider[name].value() * 0.01 for name in self.joint_names]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gui_node = JointVelocityGui()

    try:
        while rclpy.ok():
            rclpy.spin_once(gui_node, timeout_sec=0.1)
            gui_node.app.processEvents()
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

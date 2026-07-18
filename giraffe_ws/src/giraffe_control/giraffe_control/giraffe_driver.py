#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from giraffe_control.feetech import FeetechMotorsBus
from giraffe_control.follower_config import JOINT_NAMES, load_follower_config


class GiraffeDriver(Node):

    def __init__(self):
        super().__init__("giraffe_driver")

        self.declare_parameter("follower_config", "")
        config_param = self.get_parameter("follower_config").get_parameter_value().string_value
        config_path = config_param.strip() or None

        try:
            follower, resolved_path = load_follower_config(config_path)
        except FileNotFoundError as exc:
            self.get_logger().fatal(str(exc))
            raise

        self.motor_order = list(JOINT_NAMES)
        port = follower.port
        if port == "auto":
            port = self._autodetect_port()
            self.get_logger().info(f"Auto-selected serial port: {port}")

        self.motors_bus = FeetechMotorsBus(
            port=port,
            motors=follower.motors_for_bus(),
        )
        self.motors_bus.connect()

        self.joint_reverse = {}
        self.offsets = []
        self.range_min_steps = {}
        self.range_max_steps = {}
        for motor_name in self.motor_order:
            motor = follower.motors[motor_name]
            self.declare_parameter(f"joint_reverse.{motor_name}", motor.reverse)
            self.declare_parameter(f"joint_offset.{motor_name}", motor.offset)
            self.joint_reverse[motor_name] = (
                self.get_parameter(f"joint_reverse.{motor_name}")
                .get_parameter_value()
                .bool_value
            )
            self.offsets.append(
                self.get_parameter(f"joint_offset.{motor_name}")
                .get_parameter_value()
                .double_value
            )
            self.range_min_steps[motor_name] = motor.range_min_steps
            self.range_max_steps[motor_name] = motor.range_max_steps

        id_summary = {
            name: follower.motors[name].id for name in self.motor_order
        }
        self.get_logger().info(f"Loaded follower config: {resolved_path}")
        self.get_logger().info(f"Port: {port}; motor IDs: {id_summary}")
        self.get_logger().info(f"Loaded joint_reverse: {self.joint_reverse}")
        self.get_logger().info(f"Loaded joint offsets: {dict(zip(self.motor_order, self.offsets))}")
        self.get_logger().info(
            f"Software ranges: "
            f"{ {n: (self.range_min_steps[n], self.range_max_steps[n]) for n in self.motor_order} }"
        )

        self.joint_state_pub = self.create_publisher(JointState, "/feedback", 10)
        self.joint_command_sub = self.create_subscription(
            JointState, "/command", self.joint_state_callback, 10
        )

        self.timer = self.create_timer(0.01, self.publish_joint_states)

        self.set_motor_acceleration(20, 50)

    @staticmethod
    def _autodetect_port() -> str:
        import serial.tools.list_ports

        ports = [p.device for p in serial.tools.list_ports.comports()]
        preferred = [p for p in ports if "ACM" in p or "USB" in p or "usbmodem" in p]
        if preferred:
            return preferred[0]
        if ports:
            return ports[0]
        raise RuntimeError(
            "port: auto but no serial devices found. Plug in the Waveshare driver "
            "or set an explicit port in config/follower.yaml"
        )

    def joint_state_callback(self, msg: JointState):
        positions = []

        for motor_name, offset in zip(self.motor_order, self.offsets):
            if motor_name in msg.name:
                idx = msg.name.index(motor_name)
                radians = msg.position[idx]
                if self.joint_reverse[motor_name]:
                    radians = -radians
                model = self.motors_bus.motors[motor_name][1]
                step_value = self.radians_to_steps(-radians, model) + self.radians_to_steps(offset, model)
                lo = self.range_min_steps.get(motor_name)
                hi = self.range_max_steps.get(motor_name)
                if lo is not None:
                    step_value = max(int(lo), step_value)
                if hi is not None:
                    step_value = min(int(hi), step_value)
                positions.append(step_value)
            else:
                positions.append(0)

        self.motors_bus.write("Goal_Position", np.array(positions), self.motor_order)

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.motor_order
        positions = self.motors_bus.read("Present_Position", self.motor_order)
        position_radians = self.motors_bus.steps_to_radians(
            positions, self.motors_bus.motors[self.motor_order[0]][1]
        )

        for motor_name, position, offset in zip(self.motor_order, position_radians, self.offsets):
            radians = -position + offset
            if self.joint_reverse[motor_name]:
                radians = -radians
            joint_state.position.append(radians)

        self.joint_state_pub.publish(joint_state)

    def set_motor_acceleration(self, acceleration: int, gripper_acceleration: int):
        try:
            motor_names = self.motors_bus.motor_names
            non_gripper_motors = motor_names[:-1]
            accelerations = [acceleration] * len(non_gripper_motors)
            self.motors_bus.write("Acceleration", accelerations, non_gripper_motors)
            gripper = motor_names[-1]
            self.motors_bus.write("Acceleration", gripper_acceleration, gripper)
        except Exception as e:
            self.get_logger().warn(f"Failed to set acceleration: {e}")

    def radians_to_steps(self, radians: float, model: str) -> int:
        resolution = 4096
        degrees = np.degrees(radians)
        steps = int(degrees / 360.0 * resolution)
        return steps


def main(args=None):
    rclpy.init(args=args)
    giraffe_driver = GiraffeDriver()

    try:
        rclpy.spin(giraffe_driver)
    except KeyboardInterrupt:
        pass

    giraffe_driver.destroy_node()
    giraffe_driver.motors_bus.disconnect()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

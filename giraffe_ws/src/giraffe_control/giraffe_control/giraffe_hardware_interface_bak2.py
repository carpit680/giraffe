#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from giraffe_control.feetech import FeetechMotorsBus
import numpy as np

class GiraffeHardwareInterface(Node):

    def __init__(self):
        super().__init__("giraffe_hardware_interface")
        example_param = self.declare_parameter("example_param", "default_value").value
        self.get_logger().info(f"Declared parameter 'example_param'. Value: {example_param}")

        self.motors_bus = FeetechMotorsBus(
            port="/dev/ttyACM0",
            motors={
                "base_link_shoulder_pan_joint": (1, "sts3215"),
                "shoulder_pan_shoulder_lift_joint": (2, "sts3215"),
                "shoulder_lift_elbow_joint": (3, "sts3215"),
                "elbow_wrist_1_joint": (4, "sts3215"),
                "wrist_1_wrist_2_joint": (5, "sts3215"),
                "wrist_2_gripper_joint": (6, "sts3215"),
            },
        )
        self.motors_bus.connect()

        # ROS topics
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_command_sub = self.create_subscription(
            JointTrajectory, "/arm_controller/joint_trajectory", self.joint_trajectory_callback, 10
        )

        # Timer for publishing joint states
        self.create_timer(0.02, self.publish_joint_states)
        self.offsets = [3.223, 3.043, 2.979, 3.152, 1.577, 4.9547]
        self.set_motor_acceleration(10)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.motors_bus.motor_names)

        # Read joint positions and convert to radians, with direction inversion
        try:
            steps = self.motors_bus.read("Present_Position")  # Read all motors at once
            positions = []
            self.get_logger().info(f"Published positions: {steps}")

            for motor_name, step, offset in zip(msg.name, steps, self.offsets):
                model = self.motors_bus.motors[motor_name][1]
                radians = self.motors_bus.steps_to_radians(step, model)
                radians = -radians + offset  # Invert direction if needed
                positions.append(float(radians))  # Ensure values are floats
                # self.get_logger().info(f"Read position for {motor_name}: {step}")

        except Exception as e:
            self.get_logger().warn(f"Failed to read positions for motors: {e}")
            positions = [0.0] * len(msg.name)  # Default to 0.0 for all motors if there's an error
        msg.position = positions  # Assign a flat list of floats
        self.joint_state_pub.publish(msg)

    def set_motor_acceleration(self, acceleration: int):
        """Set acceleration for all motors."""
        try:
            motor_names = self.motors_bus.motor_names
            accelerations = [acceleration] * len(motor_names)
            self.motors_bus.write("Acceleration", accelerations, motor_names)
            self.get_logger().info(f"Set acceleration to {acceleration} for all motors.")
        except Exception as e:
            self.get_logger().warn(f"Failed to set acceleration for motors: {e}")

    def radians_to_steps(self, radians: float, model: str) -> int:
        """
        Converts radians to motor steps based on the model resolution.
        Assumes a full rotation (-pi to +pi radians) maps to the full step range.
        """
        resolution = 4096  # Get resolution from model
        degrees = np.degrees(radians)  # Convert radians to degrees
        steps = int(degrees / 360.0 * resolution)  # Map degrees to steps
        return steps

    def joint_trajectory_callback(self, msg: JointTrajectory):
        for point in msg.points:
            positions = point.positions
            motor_positions = []
            homing_offsets = [-2082, -999, -936, -2023, -2046, -2410]
            for motor_name, offset in zip(self.motors_bus.motor_names[:-1], homing_offsets[:-1]):
                if motor_name in msg.joint_names:
                    idx = msg.joint_names.index(motor_name)
                    model = self.motors_bus.motors[motor_name][1]
                    # Convert radians to steps
                    step_value = self.radians_to_steps(positions[idx], model) - offset
                    motor_positions.append(step_value)
                else:
                    motor_positions.append(0)  # Default to 0 if no position provided
            motor_positions.append(2410)
            try:
                self.get_logger().info(f"Sent positions in steps: {motor_positions}")
                self.motors_bus.write("Goal_Position", motor_positions, self.motors_bus.motor_names)  # Write all motors at once
            except Exception as e:
                self.get_logger().warn(f"Failed to send positions to motors: {e}")


def main(args=None):
    rclpy.init(args=args)

    giraffe_hardware_interface = GiraffeHardwareInterface()

    try:
        rclpy.spin(giraffe_hardware_interface)
    except KeyboardInterrupt:
        pass

    giraffe_hardware_interface.destroy_node()
    giraffe_hardware_interface.motors_bus.disconnect()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
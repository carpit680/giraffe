#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from giraffe_control.feetech import FeetechMotorsBus
import numpy as np

class GiraffeDriver(Node):

    def __init__(self):
        super().__init__("giraffe_driver")
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
        self.motor_order = [
            "base_link_shoulder_pan_joint",
            "shoulder_pan_shoulder_lift_joint",
            "shoulder_lift_elbow_joint",
            "elbow_wrist_1_joint",
            "wrist_1_wrist_2_joint",
            "wrist_2_gripper_joint",
        ]
        self.motors_bus.connect()

        # ROS topics
        self.joint_state_pub = self.create_publisher(JointState, "/feedback", 10)
        self.joint_command_sub = self.create_subscription(
            JointState, "/command", self.joint_state_callback, 10
        )


        # Timer for publishing joint states
        self.timer = self.create_timer(0.01, self.publish_joint_states)

        # Homing offsets
        self.offsets = [3.223, 3.043, 2.979, 3.152, 3.1415, 4.9532]
        self.set_motor_acceleration(10, 50)
    
    def joint_state_callback(self, msg: JointState):
        """
        Callback for processing incoming JointState messages and writing positions and velocities to servos.
        """
        positions = []

        homing_offsets = [-2082, -1992, -1949, -2023, -2046, -3225]
        for motor_name, offset in zip(self.motors_bus.motor_names, homing_offsets):
            if motor_name in msg.name:
                idx = msg.name.index(motor_name)
                try:
                    # Convert position to steps
                    radians = msg.position[idx]
                    model = self.motors_bus.motors[motor_name][1]
                    step_value = self.radians_to_steps(-radians, model) - offset
                    positions.append(step_value)


                except Exception as e:
                    self.get_logger().warn(f"Failed to process joint {motor_name}: {e}")
                    positions.append(0)
            else:
                positions.append(0)

        # Write all positions and velocities
        self.motors_bus.write("Goal_Position", np.array(positions), self.motor_order)

        # self.get_logger().info(f"Set positions: {positions}")

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.motor_order
        positions = self.motors_bus.read("Present_Position", self.motor_order)
        position_radians = self.motors_bus.steps_to_radians(positions, self.motors_bus.motors[self.motor_order[0]][1])
        for position, offset in zip(position_radians, self.offsets):
            joint_state.position.append(-position + offset)
        # TODO: Test velocity and effort before using them in state interface. Also add velocity and effort state interfaces to the controller.
        # joint_state.velocity = self.motors_bus.read("Present_Speed", self.motor_order)
        # joint_state.effort = self.motors_bus.read("Present_Load", self.motor_order)
        self.joint_state_pub.publish(joint_state)

    def set_motor_acceleration(self, acceleration: int, gripper_acceleration: int):
        """Set acceleration for all motors."""
        try:
            motor_names = self.motors_bus.motor_names
            non_gripper_motors = motor_names[:-1]
            accelerations = [acceleration] * len(non_gripper_motors)
            self.motors_bus.write("Acceleration", accelerations, non_gripper_motors)
            gripper = motor_names[-1]
            self.motors_bus.write("Acceleration", gripper_acceleration, gripper)
            self.get_logger().info(f"Set acceleration to {acceleration} for {motor_names[:-1]}.")
            self.get_logger().info(f"Set acceleration to {gripper_acceleration} for {gripper}.")
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


if __name__ == '__main__':
    main()

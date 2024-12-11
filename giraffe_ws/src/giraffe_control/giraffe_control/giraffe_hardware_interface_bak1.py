#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from giraffe_control.feetech import FeetechMotorsBus
import numpy as np

# Define model resolution mapping (example values)
MODEL_RESOLUTION = {
    "sts3215": 4096,
    # Add other models here if needed
}

class GiraffeHardwareInterface(Node):

    def __init__(self):
        super().__init__("giraffe_hardware_interface")
        example_param = self.declare_parameter("example_param", "default_value").value
        self.get_logger().info(f"Declared parameter 'example_param'. Value: {example_param}")

        # self.motors_bus = FeetechMotorsBus(
        #     port="/dev/ttyACM0",
        #     motors={
        #         "base_link_shoulder_pan_joint": (1, "sts3215"),
        #         "shoulder_pan_shoulder_lift_joint": (2, "sts3215"),
        #         "shoulder_lift_elbow_joint": (3, "sts3215"),
        #         "elbow_wrist_1_joint": (4, "sts3215"),
        #         "wrist_1_wrist_2_joint": (5, "sts3215"),
        #         "wrist_2_gripper_joint": (6, "sts3215"),
        #     },
        # )
        # self.motors_bus.connect()

        # ROS topic subscription
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

    def joint_state_callback(self, msg: JointState):
        """
        Callback for processing incoming JointState messages and writing positions and velocities to servos.
        """
        motor_order = [
            "base_link_shoulder_pan_joint",
            "shoulder_pan_shoulder_lift_joint",
            "shoulder_lift_elbow_joint",
            "elbow_wrist_1_joint",
            "wrist_1_wrist_2_joint",
            "wrist_2_gripper_joint",
        ]

        positions = []
        velocities = []

        for motor_name in motor_order:
            if motor_name in msg.name:
                idx = msg.name.index(motor_name)
                try:
                    # Convert position to steps
                    radians = msg.position[idx]
                    model = self.motors_bus.motors[motor_name][1]
                    step_value = self.radians_to_steps(radians, model)
                    positions.append(step_value)

                    # Add velocity if available
                    velocity = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
                    velocities.append(velocity)

                except Exception as e:
                    self.get_logger().warn(f"Failed to process joint {motor_name}: {e}")
                    positions.append(0)
                    velocities.append(0.0)
            else:
                positions.append(0)
                velocities.append(0.0)

        # Write all positions and velocities
        # self.motors_bus.write("Goal_Position", np.array(positions), motor_order)
        # self.motors_bus.write("Goal_Velocity", np.array(velocities), motor_order)

        self.get_logger().info(f"Set positions: {positions}")

    def radians_to_steps(self, radians: float, model: str) -> int:
        """
        Converts radians to motor steps based on the model resolution.
        Assumes a full rotation (-pi to +pi radians) maps to the full step range.
        """
        resolution = MODEL_RESOLUTION[model]
        degrees = np.degrees(radians)
        steps = degrees / 180 * resolution / 2
        return int(steps)

    def destroy_node(self):
        self.get_logger().info("Shutting down GiraffeHardwareInterface")
        self.motors_bus.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    giraffe_hardware_interface = GiraffeHardwareInterface()

    try:
        rclpy.spin(giraffe_hardware_interface)
    except KeyboardInterrupt:
        pass

    giraffe_hardware_interface.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
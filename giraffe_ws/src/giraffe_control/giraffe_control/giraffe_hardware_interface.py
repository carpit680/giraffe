#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from giraffe_control.feetech import FeetechMotorsBus




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
        self.offsets = [3.223, 3.043, 2.979, 3.152, 1.577, 0.0]

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.motors_bus.motor_names)

        # Read joint positions and convert to radians, with direction inversion
        positions = []
        for motor_name, offset in zip(msg.name, self.offsets):
            try:
                steps = self.motors_bus.read("Present_Position", motor_name)
                model = self.motors_bus.motors[motor_name][1]
                radians = self.motors_bus.steps_to_radians(steps, model)

                # Invert direction if needed
                # if motor_name in ["base_link_shoulder_pan_joint", "elbow_wrist_1_joint"]:
                radians = -radians + offset

                positions.append(float(radians))  # Ensure values are floats
                # self.get_logger().info(f"Read position for {motor_name}: {radians}")

            except Exception as e:
                self.get_logger().warn(f"Failed to read position for {motor_name}: {e}")
                positions.append(0.0)  # Default to 0.0 if there's an error

        msg.position = positions  # Assign a flat list of floats
        self.joint_state_pub.publish(msg)



    def joint_trajectory_callback(self, msg: JointTrajectory):
        for point in msg.points:
            positions = point.positions
            for idx, joint_name in enumerate(msg.joint_names):
                self.motors_bus.write("Goal_Position", positions[idx], motor_name=joint_name)



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

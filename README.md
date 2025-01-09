<!-- @format -->

# _giraffe: a low-cost robotic manipulator_ 🦒

```text
                                   __             ___  ___
                            .-----|__.----.---.-.'  _.'  _.-----.
                            |  _  |  |   _|  _  |   _|   _|  -__|
                            |___  |__|__| |___._|__| |__| |_____|
                            |_____|

                               Why should fun be out of reach?
```

A [Koch v1.1](https://github.com/jess-moss/koch-v1-1) inspired even more cost-effective, ROS2-compatible, Open-Source robotic manipulator designed to lower the barriers of entry for Embodied AI and whatever else your robotic dreams may be.

To achieve these outcomes, we implemented the following significant changes:

## **Servo Selection**

We redesigned the arm around cost-efficient Waveshare servos replacing Dynamixel servos, effectively doubling the torque while reducing costs.

## **Design Enhancements**

- Adjusted the base design to transfer the radial load from the base motor to the supporting structure, reducing motor stress.
- Relocated the servo driver closer to the base for cleaner design.
- The servo mounts were redesigned to utilize the fasteners provided with the servos, minimizing the required assembly components to just the servos, 3D-printed parts, and a screwdriver.

- **[Teleop Tongs](https://github.com/carpit680/teleop_tongs) Integration**

  We integrated support for Teleop Tongs, another open-source project we built on top of [Dex Teleop](https://github.com/hello-robot/stretch_dex_teleop) (for the [Stretch 3](https://hello-robot.com/stretch-3-product) mobile manipulator by [Hello Robot](https://hello-robot.com/)), designed for teleoperating general-purpose robotic manipulators. This system features a 3D-printed tongs assembly equipped with multiple fiducial markers, serving as a stand-in for the end effector. This design enables intuitive and accessible control of the robotic arm.

  The motivation behind this integration was to offer a cost-effective, and user-friendly alternative to a leader & follower arm setup. By using Teleop Tongs, operators can manipulate the robotic arm naturally, simplifying teleoperation for applications in education, research, or DIY robotics projects.

---

## Assembly Instructions

![Image of Giraffe Rendered](assets/render.png)

### Sourcing Parts

Order the off the shelf parts for the arm using the links below.

| Part                                    | Amount | Unit Cost (India)| Total Cost (India)| Buying Link (India)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| --------------------------------------- | ------ | --------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| WaveShare ST3215 30Kg                   | 6      | ₹1,949.00 | ₹11,694.00 | [Robu.in](https://robu.in/product/waveshare-30kg-serial-bus-servo/)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Waveshare Serial Bus Servo Driver Board | 1      | ₹499.00   | ₹499.00    | [Robu.in](https://robu.in/product/waveshare-serial-bus-servo-driver-board-integrates-servo-power-supply-and-control-circuit-applicable-for-st-sc-series-serial-bus-servos/)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 12V 20A Power Supply                    | 1      | ₹895.00   | ₹895.00    | [Sharvi Electronics](https://sharvielectronics.com/product/12v-20a-smps-240w-dc-metal-power-supply-non-water-proof/)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| Barrel Jack                             | 1      | ₹48.00    | ₹48.00     | [Robu.in](https://robu.in/product/90-degree-dc-5-52-1-wire/?gad_source=1&gclid=CjwKCAiA6t-6BhA3EiwAltRFGNNYxj1vEJeKhvtbnl8pWDtIrHhBP-588FWfVt-4cpQKjADzB8ReXRoC6OgQAvD_BwE)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| Power cord                              | 1      | ₹95.00    | ₹95.00     | [Sharvi Electronics](https://sharvielectronics.com/product/3pin-250vac-6a-power-cord-with-open-ended-cable-1-8-meter/)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| USB-C cable                             | 1      | ₹60.85    | ₹60.85     | [Sharvi Electronics](https://sharvielectronics.com/product/usb-2-0-a-type-male-usb-to-c-type-male-usb-cable-white-1-meter/)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| Table clamp                             | 1      | ₹349.00   | ₹349.00    | [Amazon.in](https://www.amazon.in/Homdum%C2%AE-Heavy-Clamp-Clamping-Pieces/dp/B081JYTMMG/ref=sr_1_32?crid=QS1GUQTHCIA4&dib=eyJ2IjoiMSJ9.Y6mMQKO3pYbkI5fuZZzRhmnaPEBkYUkfOdl_Uj2xmTahB1NzMLIqDi11tQEZsaF1AxDV1ndeI3h8bgnuV-SC9BiiFRj-ue_9jcyv4AsPg8YFZYe88-nm9JJ-UuEi7mFuk_8BUDldMJHKtgjKadYxvK3mqiltsGnM-1lkpJP6EmLklcT_r5J6PWWOvkh3a61a820TLtVkROcI2NKFt01PPFNt-EFB345zzs7uvYM434AFK9pRAve6-BtV_NEjXxhXwu4jVUDtNKTafPm8gwMow4hQDV2vYJ3KfqIFEPE8McGscfs-zgWCnpzyl6Dw0D1JuSiDTOfO9F1zKRaEgtLh-O48MckMmsgBaoCpQPOQqy0NKi6T0F4Wchb-x0TGvVZlh8rBH70Wz2G03owy2XS0XfroLHMvSb0RIvstaE2XQ8ID1pp8pUB0JZzPzPM_asOy.AOdTW8GzBEwdDFN3hAbqILHzc8RUdrFOTYUdAGj1WnU&dib_tag=se&keywords=table%2Bclamp&qid=1734041544&refinements=p_72%3A1318476031&rnid=1318475031&sprefix=table%2Bclamp%2Caps%2C248&sr=8-32&th=1) |
| Total                                   |        |           | ₹14,490    |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

### Printing The Parts

A variety of 3D printers can be used to print the necessary parts for the arm. Follow these steps for optimal printing results.

#### 1. Select A Printer

When choosing a printer, keep the following recommended specifications in mind. While other printers may work, these spececifications are a good starting point:

- **Layer Height:** Minimum 0.2mm
- **Material:** PLA+, ABS, PETG, or other durable plastics
- **Nozzle Diameter:** Maximum 0.4mm
- **Infill Density:** Approximately 30%
- **Suggested Printers:** Prusa Mini+, Bambu P1, Ender3, and similar models

#### 2. Prepare The Printer

- **Materials Needed:**

  - Standard Glue Stick
  - Putty Knife

- **Setup and Printing Process:**
  1. Calibrate the printer and level the print bed following your printer’s specific instructions.
  2. Clean the print bed, removing any dust or grease. If you use water or other cleaning agents, ensure the bed is fully dry.
  3. Apply a thin, even layer of glue to the print area. Avoid uneven application or clumps.
  4. Load the printer filament according to the printer's guidelines.
  5. Adjust the printer settings to match the recommended specifications listed above.
  6. Verify the file format, select files from the hardware folder, and begin printing.

#### 3. Print The Parts

Print one of each part found in `CAD/STL`

> List of Parts:
>
> - base_retainer
> - base
> - driver_mount
> - elbow
> - gripper
> - shoulder_lift
> - shoulder_pan
> - wrist_1
> - wrist_2

#### 4. Take Down

- After the print is done, use the putty knife to scrape the the parts off the print bed.
- Remove any support material from parts.

### Assembling The Parts

Construct the arms using this Assembly [Video](https://www.youtube.com/watch?v=8nQIg9BwwTk&t=8m20s) (Note: Follow the assembly instructions provided for Follower Arm starting at 08:20 of the video). After you assemble the arms from the video, power the arm using the 12V power supply. In addition, plug the arm into your computer using a USB-C cable.

The Arm after assembly should look like this:

![Image of Giraffe](assets/giraffe.png)

---

## Hardware Setup Instructions

> NOTE: Configurator and the rest of the high-level software stack is presently only compatible with Python.

### Clone The [giraffe](https://github.com/carpit680/giraffe) Repository

```bash
git clone https://github.com/carpit680/giraffe.git
cd giraffe
```

### Install Dependencies

```bash
pip install -r requirements.txt
pip install .
```

### Setup Permissions

```bash
sudo usermod -a -G dialout $USER
sudo newgrp dialout
```

### Setup Servo IDs

Use the configurator script in `scripts/` directory

```bash
python3 scripts/st_configurator.py
```
## Optional ROS2 Docker Development Environment Setup

Follow the instructions given here to set up a ROS2 Docker development environment: [ros2_docker_env](https://github.com/carpit680/ros2_docker_env)

## ROS2 Worksapce Setup

1. Install ROS2 Humble following these [installation instructions](https://docs.ros.org/en/humble/Installation.html).
2. Install Gazebo Ignition Fortress(LTS) following these [instructions](https://gazebosim.org/docs/fortress/install_ubuntu/).
3. Install Moveit 2

   ```bash
   # Install MoveIt 2 for ROS 2 Humble
   sudo apt update
   sudo apt install -y ros-humble-moveit
   ```

4. Install other dependencies

   ```bash
   sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
   sudo apt install -y python3-colcon-common-extensions python3-rosdep
   ```

5. Set Up giraffe_ws

   ```bash
   # Clone giraffe repository if you have not done so already

   # Update dependencies using rosdep
   cd <path-to-giraffe-repo>/giraffe_ws
   sudo rosdep init  # Only if not already initialized
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

6. Build and source the workspace

   ```bash
   cd <path-to-giraffe-repo>/giraffe_ws
   colcon build --symlink-install

   source install/local_setup.zsh
   # OR
   source install/local_setup.bash
   ```

## ROS2 Workspace Description
[giraffe_moveit_sim.webm](https://github.com/user-attachments/assets/942947e6-a5ca-4b55-a39b-d64a580de182)

### giraffe_description

This package contains URDF for _giraffe_ robotic manipulator along with ros2 control xacro, ros2 controller config files, and the launch files for the entire workspace.

- _display.launch.py_: This launch file visualizes the giraffe robot model in ROS 2. It includes:

  1. Robot State Publisher: Publishes the robot's state using the URDF.
  2. Joint State Publisher GUI: Enables interactive joint control.
  3. RViz Visualization: Displays the robot in a pre-configured RViz environment.

  _Usage_:

  ```bash
  ros2 launch giraffe_description display.launch.py
  ```

- _simulation.launch.py_: This launch file sets up the simulation environment for the giraffe robot in Gazebo and RViz. It includes:

  1. Gazebo Simulation: Starts Gazebo server and client with the giraffe robot model.
  2. Robot Description and State Publisher: Publishes the robot's URDF and joint states.
  3. Controllers: Spawns and activates controllers for joint trajectory and gripper control.
  4. RViz Visualization: Displays the robot model and state in RViz.

  _Usage_:

  ```bash
  ros2 launch giraffe_description simulation.launch.py
  ```

- _moveit_sim.launch.py_: This launch file integrates the giraffe robot with Gazebo, MoveIt! 2, and RViz for advanced motion planning and control. Key features:

  1. Gazebo Integration:
      - Spawns the giraffe robot in Gazebo.
      - Configures ros2_control and joint controllers.
  2. MoveIt! 2 Motion Planning:
      - Loads MoveIt! 2 configurations (SRDF, kinematics, OMPL planning).
      - Starts the move_group node for motion planning and execution.
  3. RViz Visualization:
      - Launches RViz preconfigured for MoveIt! 2 to visualize and interact with the robot.

  _Usage_:

  ```bash
  ros2 launch giraffe_description moveit_sim.launch.py
  ```

- _moveit_hardware.launch.py_: This launch file configures and launches the giraffe robot simulation, integrating Gazebo, MoveIt! 2, and a hardware interface for motion control. Key features:

  1. Gazebo Simulation:
      - Spawns the giraffe robot in Gazebo with URDF and robot controllers.
      - Initializes ros2_control and joint broadcasters for simulation.
  2. MoveIt! 2 Motion Planning:
      - Loads MoveIt! 2 configurations (SRDF, kinematics, and OMPL planning).
      - Starts the move_group node for motion planning and trajectory execution.
  3. RViz Visualization:
      - Displays the robot's state and motion planning visualization using a preconfigured RViz setup.
  4. Hardware Interface:
      - Includes a node for the giraffe robot's hardware interface for integration with controllers.

  _Usage_:

  ```bash
  ros2 launch giraffe_description moveit_hardware.launch.py
  ```

### giraffe_moveit_config

The giraffe_moveit_config package provides the MoveIt! 2 configuration for the 5-DoF robotic arm named "Giraffe," designed for use with ROS 2 Humble. It includes essential files for motion planning and execution, such as:

- **URDF and SRDF**: Defines the robot's kinematic structure and semantic description.
- **Kinematics Configuration**: Specifies IK solvers for planning.
- **OMPL Planning Configuration**: Configures planning pipelines for trajectory generation.
- **Controller Configuration**: Integrates with ros2_control for real-time trajectory execution.
- **RViz Configuration**: Pre-configured visualization setup for MoveIt! 2.

This package is utilized by the **giraffe_description** package's launch file to enable simulation and motion planning for the Giraffe arm in Gazebo and MoveIt! 2 environments.

### giraffe_control
The giraffe_control package provides hardware-level control for the 5-DoF Giraffe robotic arm. It includes a ROS 2 node, giraffe_driver, and a corresponding launch file to facilitate communication between ROS 2 and the physical hardware.

#### Features

1. Giraffe Servo Driver (giraffe_driver):

   - Implements direct communication with the Giraffe arm's servos using the Feetech motor bus.
   - Processes incoming command messages to set motor positions.
   - Read motor position feedback from the servos to publish feedback.
   - Supports homing offsets, acceleration settings, and position conversion from radians to motor steps.
   - Subscribes to /command for joint commands and publishes feedback to /feedback topic.
   - Interfaces with six motors:
     - base_link_shoulder_pan_joint
     - shoulder_pan_shoulder_lift_joint
     - shoulder_lift_elbow_joint
     - elbow_wrist_1_joint
     - wrist_1_wrist_2_joint
     - wrist_2_gripper_joint

2. Launch File:
   - Starts the giraffe_driver node.
   - Configures parameters for easy integration with other ROS 2 packages.

_Usage_:

The giraffe_control package is used by the giraffe_description package's launch file to provide hardware control during simulations and real-world operation. It ensures seamless integration of the Giraffe robotic arm into ROS 2 for both motion execution and feedback.

### giraffe_hardware
The giraffe_hardware package provides a ros2_control hardware interface for the Giraffe 5-DoF robotic arm plus a gripper joint. This interface lets you control and monitor the arm through standard ROS 2 controllers and topics, simplifying integration with motion planning frameworks like MoveIt.

#### Features:

1. Giraffe Hardware Interface (GiraffeInterface):
   - Implements a hardware_interface::SystemInterface plugin.
   - Subscribes to feedback (sensor_msgs/msg/JointState) for joint position updates.
   - Publishes to command (sensor_msgs/msg/JointState) to send joint commands.
   - Handles all six joints of the arm.

_Usage_:

- Integrate with controller_manager and standard controllers (e.g., joint_trajectory_controller).
- Place giraffe_interface in your ros2_control configuration.
- Commands and feedback are exchanged via standard ROS topics, enabling easy simulation or real hardware operation.


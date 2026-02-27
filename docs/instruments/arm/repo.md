
# Arm code repository

We can summarize the operation as follows:

1. Launch: `ros2 launch bracc8_control maintenance_task.launch.py` starts everything.  
2. Initialization: `interactive_control_node` loads the kinematics and starts the controller. `realsense_node` begins streaming data.  
3. Perception: `maintenance_task_node` uses ArUco to define the work plane (`panel_frame`) and then activates YOLO to find the switches.  
4. Planning: The 3D coordinates of the objects are passed to the `trajectory_generator`, which calculates the IK and generates smooth paths.  
5. Control: `bracc8_controller` executes the trajectory, sending commands to the robot and monitoring safety.  
6. Visualization: The operator supervises everything in RViz, seeing both the real robot and the preview of the robot's intentions (ghost robot and path lines).

## bracc8_interfaces
1. Role: Defines all communication interfaces (messages and services) for the entire robotic arm workspace.
2. Main Messages:  
   1. ControlCommandMsg: Comprehensive message to send teleoperation commands (movement, mode change, trajectory recording) from an input device to the robot.  
   2. TeleopStateMsg: Status message describing the currently active control mode.  
   3. DetectionArray: Message for publishing results from a computer vision system.  
3. Main Services:
    1. MaintenanceDetectTag and MaintenanceNextPose:
    Services to orchestrate a step-by-step guided maintenance task, combining perception and movement.  

## bracc8_description
1. Purpose: Provides the URDF description of the robot, including visual geometry, collision volumes, and inertial properties for simulation and control.
2. Kinematics:
a. 6-degree-of-freedom (6DOF) anthropomorphic robot.
b. Joints defined as continuous (continuous rotation), with software limits set to ±π radians.
3. End-Effector and Sensors:
a. The `ee` link represents the Tool Center Point (TCP).
b. Eye-in-Hand configuration: The Intel RealSense camera is rigidly mounted to the end-effector (offset: x=3cm, z=3cm).
4. Mesh Files: The real 3D geometries are loaded from the `meshes/` folder for realistic visualization in RViz.

## bracc8_control

### bracc8_control/bracc8_control
This folder contains the main system logic. It is organized in modules to separate responsibilities. 

Folder layout

    common/                         # Global definitions to avoid redundancy
        constants.py                # 
    communication/                  # Handles ROS2 interface, isolating control logic from middleware
        ros_communicator.py         #
    kinematics/                     # Kinematic calculations for forward and inverse kinematics
        kinematics.py               #
    trajectory_generation/          # Path planning logic
        trajectory_generator.py     #
    control/                        # Business logic and state handling
        bracc8_controller.py        #
    input_handling/                 # Input hardware abstraction
        getch.py                    # Utility to read a single char from keyboard on Linux without pressing Enter (raw input).
        input_mappings.py           #
        input_handler.py            # Handles joystick and keyboard logic
    nodes/                          # Executable ROS2 nodes
        camera_tf_publisher.py
        teleop_node.py
        interactive_control_node.py
        maintenance_task_node.py
        semaforo_node.py    

=== "📂 common"
    #### constants.py
    * Defines `BRACC8_JOINT_NAMES`, a numpy array with the strings 'joint1'...'joint6'.
    * Ensures that all nodes (ROS communicator, TF publisher, etc.) use the exact same names defined in the URDF.  
=== "📂 communication"
    Manages the interface with ROS2, isolating control logic from the middleware.
    #### ros_communicator.py
      1. QoS (Quality of Service):
        * Configures different profiles: Reliable for critical commands (e.g., trajectories, emergency stop) and Best Effort for high-frequency data (e.g., joint states).  
      2. Publisher:  
        1. /command/joint_trajectory_point:
           * Sends target positions/velocities to the arm driver.
        2. /rviz/commanded/trajectory_preview:
           * Sends a Path message to visualize the green line in RViz before movement.
      3. Subscriber:
         1. /state/joint_states:
            1. Receives real feedback from the robot.
      4. Services:
         1. Handles asynchronous calls (`call_async`) for external sensors (load cell, pH) to avoid blocking the control loop. 
=== "📂 kinematics"
    The mathematical engine of the robot. Implements geometry and differential calculations.
    #### kinematics.py:
    Kinematics class: Defines the DH (Denavit-Hartenberg) table of the arm.
      1. forward_kinematics(q):
         1. Calculates the Cartesian pose (4x4 SE3 matrix) given the joint configuration.
      2. analytical_inverse_kinematics(T):
         1. Implements a closed-form geometric solution (spherical wrist decoupling). Preferred method because it is fast and exact. Returns all possible solutions (e.g., elbow up/down).
      3. numeric_inverse_kinematics(T):
         1. Iterative solver (Levenberg-Marquardt) used as a fallback if the analytical solution fails.
      4. differential_inverse_kinematics(q, v):
         1. Used for joystick control. Calculates the joint velocities (q˙) needed to achieve a Cartesian velocity (v) using the damped pseudo-inverse Jacobian (DLS) to handle singularities. Includes a Velocity Scaling algorithm to smoothly slow down near endstops.
      5. get_camera_pose(q):
         1. Calculates the camera position (Eye-in-Hand) by composing the kinematic transforms.  
=== "📂 trajectory_generation"
    Path planner.
    #### trajectory_generator.py:
    TrajectoryGenerator class:
      1. generate_joint_trajectory:
         1. Generates smooth trajectories in joint space using 5th-degree polynomials (`jtraj` or `mstraj` from robotics toolbox). Ensures continuity in velocity and acceleration.
      2. generate_cartesian_trajectory:
         1. Takes a list of Cartesian Waypoints. For each point, calculates the IK choosing the solution closest to the previous configuration (to avoid abrupt jumps). Then interpolates in joint space.
      3. generate_incremental_rotation:
         1. Specific functions for teleoperation. Calculate micro-trajectories for small linear displacements or rotations with respect to the Base or Tool frame.
      1. get_predefined_trajectory:
        1. Contains hardcoded movements (e.g., "home", "paletta") useful for tests and demos.  
=== "📂 control"
    Business logic and state management.
    #### bracc8_controller.py:
    Class: Maintains the robot state (`current_q`, `control_mode`, `emergency_stop`).
      1. process_command:
        1. The reactive "brain". Receives a ControlCommand and decides what action to take (e.g., calculate inverse kinematics, start a trajectory, change frame).
      2. Thread Management:
        1. Method `execute_trajectory_non_blocking`. Launches physical execution on a separate thread to allow the user to interrupt movement (Emergency Stop) at any time.
      3. Recorder:
        1. Methods `_save_all_trajectories` and `_load_...` to manage persistence of waypoints on JSON files.  
=== "📂 input_handling"
    Input hardware abstraction.
    #### getch.py
    Utility to read a single character from the keyboard on Linux without pressing Enter (raw input).
    #### input_mappings.py
      1. Defines Enum: ControlCommand (logical commands), ControlMode (operating states), ReferenceFrame.
      2. JoystickMappings: Dictionary mapping physical indices (e.g., Axis 1) to logical names (e.g., 'LS_Y'). Allows support for different controllers (Xbox, PS4) by changing only this file.
    #### input_handler.py
      1. InputHandler class: Manages pygame (for joystick) and keyboard.
         1. Joystick logic: Applies deadzone and converts analog axis readings into a normalized 6D velocity vector.  
=== "📂 nodes"
    Executable ROS2 nodes.
    #### camera_tf_publisher.py
      1. Subscribes to `/joint_states`.
      2. Uses `kinematics.get_camera_pose` to calculate where the camera is relative to the base.
      3. Publishes the dynamic transform on `/tf`. Fundamental for Eye-in-Hand configuration.
    #### teleop_node.py
      1. "Lightweight" node running at 20Hz.
      2. Queries InputHandler and publishes ControlCommandMsg.
      3. Listens to TeleopStateMsg to adapt commands to the robot's current state.
    #### interactive_control_node.py
      The central node. Instantiates Bracc8Controller and connects ROS callbacks. It is the bridge between the ROS world (messages) and pure Python logic (Controller).
    #### maintenance_task_node.py
      1. Complex autonomous node for the maintenance task.
      2. Implements a Finite State Machine (FSM):
        1. DISCOVERING: Finds ArUco.
        2. LOCALIZING: Builds `panel_frame`.
        3. SCENE_UNDERSTANDING: Uses YOLO + Depth to map 3D objects.
        4. EXECUTE: Plans and executes the action (e.g., flip switch).
      3. 3D Vision: Method `get_object_pose` transforms 2D pixel -> 3D Camera point -> 3D Panel Frame point using TF buffer.
    #### semaforo_node.py
      1. A simple subscriber to control external LEDs (probably for system state debugging).

### bracc8_control/launch - Startup Configuration
Python scripts for node orchestration.
#### bracc8_visualization.launch.py
   1. Launches the basic infrastructure.
   2. Starts two `robot_state_publisher`.
   3. One in the `rviz/commanded` namespace (ghost/target robot) and one in `rviz/real` (physical/feedback robot). This allows visualization of tracking error. Loads the URDF via xacro.
   4. Launches RViz with the saved configuration.
#### realsense.launch.py
   1. Starts the `realsense2_camera_node` driver.
   2.  Enables `align_depth` (color-depth pixel alignment) and pointcloud. Also launches `aruco_detector` for panel localization.
#### maintenance_task.launch.py
   1. "Master" launch file for the autonomous demo.
   2. Includes `bracc8_visualization` and `realsense`.
   3. Starts `yolo_detector_lifecycle` (the AI node) passing the path to the `.pt` model.
   4. Starts `maintenance_task_node` with operational parameters (approach offset, flip distances).
#### debug_aruco/yolo_realsense.launch.py
   * Lightweight launch files to test subsystems (only ArUco or only YOLO) without starting the entire robot stack.
#### jetson.launch.py
   * Likely an optimized variant to run on NVIDIA Jetson hardware (may have specific configurations for limited resources).

### bracc8_control/models - AI Resources
Contains the neural network binary files.
#### best.pt, best_11n_newseg.pt
1. Pre-trained weights for the YOLOv8 (You Only Look Once) model.
2. Contain the "knowledge" needed to identify switches, sockets, and levers in RGB images. Loaded by the `yolo_detector` node.

### bracc8_control/rviz - Graphic Configuration
#### bracc8_view.rviz
1. YAML file saved from RViz
2. Configures the active Displays:
   1. RobotModel (Commanded): Alpha 0.3 (transparent).
   2. RobotModel (Real): Alpha 1.0 (solid).
   3. PointCloud2: Displays RGBD data from RealSense.
   4. Path: Displays the `/rviz/commanded/trajectory_preview` topic (green line).
3. Saves the virtual camera pose to have the correct view immediately at startup.

### bracc8_control/data - Data Persistence
#### recorded_trajectories.json
   1. JSON database where user-recorded trajectories are saved.
   2. Structure: Trajectory Name -> List of Waypoints -> (Pose, Gripper State, Joint Configuration).
   3. Allows replay ("Play") of complex movements manually taught.

### bracc8_control/test/templates - Mathematical Validation
Contains MATLAB scripts used to prototype and validate algorithms.
#### compute_A_matrices.m
   Symbolic calculation of homogeneous transformation matrices from the table
#### DH.DH_to_JA.m /DH_to_JL.m
   Symbolic calculation of the Analytical and Linear Jacobian. Used to verify that the Jacobian calculated in Python is correct.
#### cubic_poly_coeff.m
   Mathematical validation of trajectory generation
#### cubiche.newton_method.m
   Test implementation of the Newton algorithm for numerical inverse kinematics.
#### find_singularity.m
   Symbolic analysis of the Jacobian determinant to find singular configurations (where the robot loses degrees of freedom).
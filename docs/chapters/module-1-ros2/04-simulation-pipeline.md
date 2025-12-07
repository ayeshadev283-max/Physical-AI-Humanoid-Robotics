---
sidebar_position: 4
title: "Chapter 4: Simulation Pipeline"
description: Gazebo integration, launch files, and development workflow
tags: [ros2, gazebo, launch, colcon, development]
---

# Chapter 4: Simulation Pipeline

## Learning Objectives

1. Integrate ROS 2 with Gazebo simulation
2. Write Python launch files for complex system bringup
3. Use colcon for building, testing, and debugging ROS 2 packages

## 4.1 Gazebo Integration

### Gazebo + ROS 2

**Gazebo** (formerly Ignition, now Gazebo Sim):
- Open-source 3D robot simulator
- Physics engines: ODE, Bullet, DART
- Sensor simulation: cameras, LiDAR, IMU, contact
- Plugin system for custom behavior

**ROS 2 Integration** (`ros_gz`):
- Bridge: Translates Gazebo topics ↔ ROS 2 topics
- Plugins: Sensors, actuators publish to ROS 2
- URDF/SDF: Same robot description in simulation and hardware

### ros2_control

**Abstraction layer** for hardware interfaces:
- Same controller code runs on simulation and real robot
- Hardware interface plugins (Gazebo, real motors)
- Standard controllers: `joint_trajectory_controller`, `diff_drive_controller`

**Architecture**:
```
┌─────────────────────────────────────┐
│     Controller Manager              │
├─────────────────────────────────────┤
│  joint_trajectory_controller        │
│  diff_drive_controller              │
└─────────────┬───────────────────────┘
              │ ros2_control interface
┌─────────────┴───────────────────────┐
│  Hardware Interface                 │
│  - GazeboSystem (sim)               │
│  - RobotHardware (real)             │
└─────────────────────────────────────┘
```

**URDF Configuration**:
```xml
<ros2_control name="robot_hardware" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

**Controller Config** (`controllers.yaml`):
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.4
    wheel_radius: 0.1
    publish_rate: 50.0
    base_frame_id: base_link
```

### Sensor Plugins

**Camera** (URDF Gazebo plugin):
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_driver" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image</remapping>
        <remapping>camera_info:=camera/info</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**LiDAR**:
```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_ray">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14</min_angle>
          <max_angle>3.14</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
      </range>
    </ray>
    <plugin name="lidar_driver" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## 4.2 Launch Files (Python Launch System)

### Why Python Launch?

**ROS 1 XML Launch** limitations:
- No conditionals (if/else)
- No loops
- No programmatic configuration

**ROS 2 Python Launch**:
- Full Python: conditionals, functions, external configs
- Type checking
- Composable nodes (multiple nodes in one process)

### Basic Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener',
            output='screen'
        )
    ])
```

**Run**:
```bash
ros2 launch my_package demo.launch.py
```

### Advanced Features

**Parameters from YAML**:
```python
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

config = PathJoinSubstitution([
    FindPackageShare('my_robot'),
    'config',
    'params.yaml'
])

Node(
    package='my_robot',
    executable='controller',
    parameters=[config]
)
```

**Conditionals**:
```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(LaunchConfiguration('use_rviz'))
)
```

**Remapping**:
```python
Node(
    package='image_tools',
    executable='cam2image',
    remappings=[
        ('/image', '/camera/image_raw'),
        ('/camera_info', '/camera/info')
    ]
)
```

**Composable Nodes** (lower latency):
```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ComposableNodeContainer(
    name='camera_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify'
        ),
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzrgbNode',
            name='pointcloud'
        )
    ]
)
```

### Gazebo Launch Example

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Robot description
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf', 'robot.urdf.xacro'
    ])

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': Command(['xacro ', urdf_path])},
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'config', 'controllers.yaml'
            ])
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        controller_manager
    ])
```

## 4.3 Development Workflow

### Colcon Build System

**Colcon** (collective construction):
- Successor to `catkin_make`, `catkin build`
- Language-agnostic (Python, C++, Rust)
- Parallel builds

**Workspace Structure**:
```
ros2_ws/
├── src/                    # Source packages
│   ├── my_robot_description/
│   ├── my_robot_bringup/
│   └── my_robot_control/
├── build/                  # Compiled artifacts
├── install/                # Install space (setup.bash here)
└── log/                    # Build logs
```

**Build Commands**:
```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_robot_description

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build in parallel (4 jobs)
colcon build --parallel-workers 4

# Symlink Python files (no rebuild needed for changes)
colcon build --symlink-install
```

**Source Workspace**:
```bash
source install/setup.bash  # Bash
source install/setup.zsh   # Zsh
```

### Testing

**Unit Tests** (Python):
```python
import unittest
from my_package.my_module import add

class TestAdd(unittest.TestCase):
    def test_add_positive(self):
        self.assertEqual(add(2, 3), 5)

    def test_add_negative(self):
        self.assertEqual(add(-1, -1), -2)
```

**Integration Tests** (`launch_testing`):
```python
import launch_testing
import pytest

@pytest.mark.launch_test
def test_talker_listener():
    # Launch nodes, check for expected output
    pass
```

**Run Tests**:
```bash
# All tests
colcon test

# Specific package
colcon test --packages-select my_package

# View results
colcon test-result --all
```

### Debugging

**GDB (C++)**:
```bash
# Launch with gdb
ros2 run --prefix 'gdb -ex run --args' my_package my_node
```

**Python Debugger**:
```python
import pdb; pdb.set_trace()  # Breakpoint
```

**Logging Levels**:
```bash
ros2 run my_package my_node --ros-args --log-level DEBUG
```

**Topic Introspection**:
```bash
ros2 topic echo /scan
ros2 topic hz /camera/image_raw  # Check frequency
ros2 topic bw /camera/image_raw  # Check bandwidth
```

**rqt Tools**:
```bash
rqt_graph     # Visualize node graph
rqt_console   # Log messages
rqt_plot      # Plot numeric topics
rqt_bag       # Record/playback data
```

### Best Practices

1. **Version Control**: Use git, `.gitignore` build artifacts
2. **Dependencies**: List in `package.xml`, use `rosdep install`
3. **Documentation**: README per package, code comments
4. **CI/CD**: GitHub Actions for automated builds/tests
5. **Code Style**: `ament_lint` for C++, `flake8` for Python

## Summary

**Gazebo**: Physics simulation with `ros2_control` abstraction
**Launch Files**: Python-based, composable, conditional logic
**Colcon**: Build system with parallel compilation, testing
**Development**: Debugging tools, CI/CD, code quality checks



## Exercises

**Exercise 4.1**: Configure `ros2_control` for a differential drive robot in Gazebo. Define hardware interfaces for left and right wheel velocity commands. Load the `diff_drive_controller` and test with `ros2 topic pub /cmd_vel`.

**Exercise 4.2**: Write a Python launch file that:
1. Launches Gazebo with a custom world file
2. Spawns a robot from URDF
3. Starts the robot_state_publisher
4. Conditionally launches RViz if `use_rviz:=true`
5. Accepts a `robot_name` argument

**Exercise 4.3**: Add a camera sensor plugin to your robot's URDF. Configure it to publish to `/robot/camera/image_raw` at 30 Hz with 640x480 resolution. Verify the image stream in RViz.

**Exercise 4.4**: Set up continuous integration (CI) for a ROS 2 workspace using GitHub Actions. The CI should:
1. Build all packages with `colcon build`
2. Run unit tests with `colcon test`
3. Run static analysis (ament_lint)
4. Fail if any step produces errors


**Module 1 Complete!** Next modules cover Gazebo/Unity (Module 2), NVIDIA Isaac (Module 3), VLA models (Module 4), and capstone project (Module 5).

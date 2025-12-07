# Python Launch File Example

This example demonstrates advanced ROS 2 launch features:
- Launch arguments with defaults
- Conditional node launching (RViz only if `use_rviz:=true`)
- URDF/Xacro processing with `Command` substitution
- Including other launch files
- Path resolution with `FindPackageShare`

## Usage

```bash
# Launch with defaults
ros2 launch launch_demo robot_launch.py

# Override arguments
ros2 launch launch_demo robot_launch.py use_rviz:=false robot_name:=robot_01

# Without simulation time
ros2 launch launch_demo robot_launch.py use_sim_time:=false
```

## Key Concepts

- **DeclareLaunchArgument**: Defines configurable parameters
- **LaunchConfiguration**: Retrieves argument values
- **IfCondition**: Conditional node launching
- **Command**: Executes shell commands (e.g., `xacro`)
- **PathJoinSubstitution**: Cross-platform path joining
- **FindPackageShare**: Locates installed package directories
- **IncludeLaunchDescription**: Modular launch file composition

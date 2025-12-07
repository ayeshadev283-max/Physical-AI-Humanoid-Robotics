# ROS 2 VLA Policy Integration

This example demonstrates integrating OpenVLA with ROS 2 for real-time robot control.

## Prerequisites

- ROS 2 Humble or newer
- Python 3.8+
- OpenVLA installed (`pip install openvla`)
- Robot with ROS 2 interface (or Gazebo simulation)

## Installation

```bash
# Install ROS 2 dependencies
sudo apt-get install ros-humble-cv-bridge ros-humble-sensor-msgs

# Install Python packages
pip install rclpy opencv-python torch openvla

# Build workspace
cd ~/ros2_ws
colcon build --packages-select vla_policy
source install/setup.bash
```

## Example 1: VLA Policy Node

**File**: `vla_policy_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import torch
from openvla import OpenVLA
import numpy as np

class VLAPolicyNode(Node):
    def __init__(self):
        super().__init__('vla_policy_node')

        # Parameters
        self.declare_parameter('model_name', 'openvla-7b')
        self.declare_parameter('policy_rate', 10.0)  # Hz
        self.declare_parameter('action_dim', 7)

        model_name = self.get_parameter('model_name').value
        policy_rate = self.get_parameter('policy_rate').value

        # Load VLA model
        self.get_logger().info(f"Loading {model_name}...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = OpenVLA.from_pretrained(model_name).to(self.device)
        self.model.eval()
        self.get_logger().info(f"Model loaded on {self.device}")

        # ROS utilities
        self.bridge = CvBridge()
        self.current_image = None
        self.current_instruction = "pick up the cup"

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.instruction_sub = self.create_subscription(
            String,
            '/task/instruction',
            self.instruction_callback,
            10
        )

        # Publishers
        self.action_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Policy timer
        period = 1.0 / policy_rate
        self.timer = self.create_timer(period, self.policy_loop)

        self.get_logger().info("VLA Policy Node ready")

    def image_callback(self, msg):
        """Convert ROS Image to numpy array"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def instruction_callback(self, msg):
        """Update task instruction"""
        self.current_instruction = msg.data
        self.get_logger().info(f"New instruction: {self.current_instruction}")

    def policy_loop(self):
        """Main policy loop (runs at policy_rate Hz)"""
        if self.current_image is None:
            self.get_logger().warn("No image received yet")
            return

        # Predict action with VLA
        with torch.no_grad():
            action = self.model.predict_action(
                image=self.current_image,
                instruction=self.current_instruction,
                unnormalize=True
            )

        # Publish as JointTrajectory
        self.publish_action(action)

    def publish_action(self, action):
        """Publish action as joint trajectory command"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [f'joint_{i}' for i in range(1, 8)]

        point = JointTrajectoryPoint()
        point.positions = action[:7].tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100_000_000  # 100ms

        msg.points = [point]
        self.action_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VLAPolicyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 2: Launch File

**File**: `vla_policy_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'model_name',
            default_value='openvla-7b',
            description='VLA model to use'
        ),
        DeclareLaunchArgument(
            'policy_rate',
            default_value='10.0',
            description='Policy execution frequency (Hz)'
        ),

        # VLA Policy Node
        Node(
            package='vla_policy',
            executable='vla_policy_node.py',
            name='vla_policy',
            parameters=[{
                'model_name': LaunchConfiguration('model_name'),
                'policy_rate': LaunchConfiguration('policy_rate')
            }],
            output='screen'
        ),

        # Camera Node (example: USB camera)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw')
            ]
        ),

        # Robot Controller (example: ros2_control)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen'
        )
    ])
```

## Example 3: Safety Wrapper

Add safety checks before executing VLA actions.

```python
class SafeVLAPolicyNode(VLAPolicyNode):
    def __init__(self):
        super().__init__()

        # Safety parameters
        self.workspace_min = np.array([-0.5, -0.5, 0.0, -3.14, -1.57, -3.14])
        self.workspace_max = np.array([0.5, 0.5, 0.6, 3.14, 1.57, 3.14])
        self.max_velocity = 0.5  # m/s

    def policy_loop(self):
        """Policy loop with safety checks"""
        if self.current_image is None:
            return

        # Predict action
        with torch.no_grad():
            raw_action = self.model.predict_action(
                image=self.current_image,
                instruction=self.current_instruction,
                unnormalize=True
            )

        # Safety checks
        safe_action = self.apply_safety_checks(raw_action)

        # Publish
        self.publish_action(safe_action)

    def apply_safety_checks(self, action):
        """Clip action to safe bounds"""
        safe_action = action.copy()

        # Workspace limits (xyz + rpy)
        safe_action[:6] = np.clip(
            action[:6],
            self.workspace_min,
            self.workspace_max
        )

        # Velocity limit (simple check)
        if hasattr(self, 'previous_action'):
            delta = safe_action[:3] - self.previous_action[:3]
            speed = np.linalg.norm(delta) * self.get_parameter('policy_rate').value

            if speed > self.max_velocity:
                # Scale down to max velocity
                safe_action[:3] = self.previous_action[:3] + \
                    delta * (self.max_velocity / speed)

        self.previous_action = safe_action
        return safe_action
```

## Running the Examples

### Launch VLA Policy with Gazebo Simulation

```bash
# Terminal 1: Start Gazebo simulation
ros2 launch robot_gazebo robot.launch.py

# Terminal 2: Launch VLA policy node
ros2 launch vla_policy vla_policy_launch.py

# Terminal 3: Send task instruction
ros2 topic pub /task/instruction std_msgs/String "data: 'pick up the red cup'"
```

### Monitor Performance

```bash
# Check policy rate
ros2 topic hz /arm_controller/joint_trajectory

# Monitor latency
ros2 topic echo /vla_policy/latency

# Visualize predictions
rviz2 -d config/vla_visualization.rviz
```

## Performance Tips

1. **Use GPU**: Ensure CUDA is available (`nvidia-smi`)
2. **Optimize Image Transport**: Use `image_transport_plugins` for compression
3. **Reduce Image Size**: Resize to 224x224 before inference
4. **Batch Processing**: Process multiple camera views in one forward pass
5. **TensorRT**: Compile model with TensorRT for 3-5x speedup

## Example: TensorRT Optimization

```python
import torch_tensorrt

class OptimizedVLAPolicyNode(VLAPolicyNode):
    def __init__(self):
        super().__init__()

        # Compile with TensorRT
        self.get_logger().info("Compiling model with TensorRT...")
        self.model = torch_tensorrt.compile(
            self.model,
            inputs=[
                torch_tensorrt.Input((1, 3, 224, 224)),  # Image
                torch_tensorrt.Input((1, 512))  # Language embedding
            ],
            enabled_precisions={torch.float16}
        )
        self.get_logger().info("TensorRT compilation complete")
```

## Troubleshooting

**Issue**: High latency (>100ms)
- Solution: Use TensorRT, reduce image resolution, check GPU usage

**Issue**: Actions too fast/jerky
- Solution: Reduce policy rate (5 Hz), add action smoothing

**Issue**: Robot safety violations
- Solution: Enable safety wrapper, tune workspace bounds

## Next Steps

- Integrate with MoveIt for collision-free motion planning
- Add force/torque feedback for contact-rich manipulation
- Deploy on real robot hardware (Franka Emika, UR5, etc.)
- Fine-tune VLA on robot-specific demonstrations

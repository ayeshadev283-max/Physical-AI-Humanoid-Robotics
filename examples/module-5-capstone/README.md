# Module 5: Autonomous Humanoid Capstone Project

This directory contains complete examples for building an autonomous humanoid robot system.

## Project Structure

```
module-5-capstone/
├── perception_pipeline/    # Sensor integration, SLAM, object detection
├── control_system/         # Whole-body control, balance, joint control
├── vla_integration/        # VLA policy integration and task planning
└── full_system/            # Complete integrated system
```

## Quick Start

### 1. Perception Pipeline

```bash
cd perception_pipeline/
ros2 launch perception_stack perception.launch.py
```

**Components**:
- RGB-D camera integration (RealSense D435i)
- Isaac ROS Visual SLAM
- YOLOv8 object detection
- EKF sensor fusion

### 2. Control System

```bash
cd control_system/
ros2 launch control_stack whole_body_control.launch.py
```

**Components**:
- Whole-body QP controller
- ZMP-based balance controller
- Joint PID controllers (30x motors)

### 3. VLA Integration

```bash
cd vla_integration/
ros2 launch vla_autonomy vla_policy.launch.py
```

**Components**:
- Fine-tuned OpenVLA for humanoid
- Behavior tree task planner
- Skill library (navigate, grasp, place)

### 4. Full System (Simulation)

```bash
# Terminal 1: Start Isaac Sim
~/isaac-sim/python.sh scripts/start_isaac_sim.py

# Terminal 2: Launch full stack
ros2 launch full_system humanoid_autonomy.launch.py

# Terminal 3: Send task
ros2 topic pub /task/instruction std_msgs/String "data: 'fetch me the cup from the kitchen'"
```

## System Requirements

### Hardware
- **Compute**: NVIDIA Jetson Orin (edge deployment) or RTX 4090 (development)
- **RAM**: 32GB minimum, 64GB recommended
- **Storage**: 100GB+ (models, datasets, logs)

### Software
- Ubuntu 22.04
- ROS 2 Humble
- NVIDIA Isaac Sim 2023.1.0+
- Python 3.8+
- CUDA 12.0+

## Installation

```bash
# 1. Install ROS 2 Humble
sudo apt-get install ros-humble-desktop

# 2. Install Isaac ROS packages
sudo apt-get install ros-humble-isaac-ros-visual-slam \
                     ros-humble-isaac-ros-dnn-inference

# 3. Install Python dependencies
pip install torch torchvision opencv-python \
            openvla ultralytics py-trees \
            qpsolvers cvxpy

# 4. Build workspace
cd ~/humanoid_ws
colcon build
source install/setup.bash
```

## Development Roadmap

Based on Chapter 1 (12-week plan):

**Weeks 1-2**: Simulation setup (Isaac Sim, URDF, ROS 2 bridge)
**Weeks 3-4**: Perception stack (SLAM, object detection, sensor fusion)
**Weeks 5-6**: Control stack (whole-body, balance, joint control)
**Weeks 7-8**: VLA integration (fine-tuning, task planning, ROS 2 deployment)
**Weeks 9-10**: Sim-to-real transfer (system ID, domain randomization, real robot)
**Weeks 11-12**: Iteration and scaling (failure analysis, DAgger, multi-task)

## Example Tasks

1. **Fetch Task**: "Fetch me the cup from the kitchen"
   - Navigate to kitchen
   - Detect and localize cup
   - Plan and execute grasp
   - Navigate back to user
   - Hand over cup

2. **Cleaning Task**: "Clean up the toys from the floor"
   - Detect toys on floor
   - Plan pickup sequence
   - Grasp each toy
   - Place in container

3. **Delivery Task**: "Bring me the book from the shelf"
   - Navigate to bookshelf
   - Identify target book (visual + semantic)
   - Grasp and retrieve
   - Navigate and deliver

## Performance Targets

| Metric | Target | Current |
|--------|--------|---------|
| Task Success Rate | `>80%` | TBD |
| End-to-End Latency | `<10s` | TBD |
| VLA Inference Time | `<50ms` | TBD |
| SLAM Accuracy | `<5cm` error | TBD |
| Battery Life | `>2 hours` | TBD |

## Troubleshooting

**Issue**: SLAM drift too high
- **Solution**: Calibrate cameras, enable IMU fusion, add loop closure detection

**Issue**: VLA latency `>100ms`
- **Solution**: Use TensorRT compilation, INT8 quantization, reduce image size

**Issue**: Balance instability
- **Solution**: Tune ZMP controller gains, lower CoM height, increase foot contact area

**Issue**: Grasp failures
- **Solution**: Collect more grasp demos, improve depth estimation, add force feedback

## Contributing

To add new tasks or improve existing components:

1. Create branch: `git checkout -b feature/new-task`
2. Implement and test in simulation (100+ trials)
3. Document performance metrics
4. Submit PR with evaluation results

## References

- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
- [Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
- [OpenVLA](https://github.com/openvla/openvla)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)

## License

See project root LICENSE files for code and content licensing.

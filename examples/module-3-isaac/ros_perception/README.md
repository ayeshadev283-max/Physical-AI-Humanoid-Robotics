# Isaac ROS Perception Example

This example demonstrates GPU-accelerated perception with Isaac ROS for object detection and SLAM.

## Prerequisites

- ROS 2 Humble
- NVIDIA Jetson Orin or x86 system with NVIDIA GPU
- Isaac ROS packages installed

## Installation

```bash
# Install Isaac ROS (Jetson/x86 with GPU)
sudo apt-get install ros-humble-isaac-ros-dnn-inference
sudo apt-get install ros-humble-isaac-ros-visual-slam
sudo apt-get install ros-humble-isaac-ros-nvblox

# Install dependencies
sudo apt-get install ros-humble-vision-msgs
sudo apt-get install ros-humble-nav-msgs
```

## Example 1: Object Detection with TensorRT

```bash
# Terminal 1: Start camera
ros2 run v4l2_camera v4l2_camera_node

# Terminal 2: Run TensorRT inference
ros2 launch isaac_ros_dnn_inference isaac_ros_dnn_inference.launch.py model_file_path:=yolov8.engine

# Terminal 3: Visualize
ros2 run rqt_image_view rqt_image_view /detections/image
```

## Example 2: Visual SLAM

```bash
# Launch visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Publish camera
ros2 run usb_cam usb_cam_node --ros-args -p video_device:=/dev/video0

# Visualize in RViz
rviz2 -d isaac_slam.rviz
```

## Example 3: 3D Mapping with Nvblox

```bash
# Launch nvblox
ros2 launch isaac_ros_nvblox isaac_ros_nvblox.launch.py

# Publish depth camera
ros2 launch realsense2_camera rs_launch.py enable_depth:=true

# View mesh
rviz2 -d nvblox.rviz
```

## Performance Comparison

| Pipeline | CPU (FPS) | GPU/TensorRT (FPS) |
|----------|-----------|-------------------|
| YOLOv8 (640x640) | 5 | 60 |
| SegFormer | 2 | 30 |
| Visual SLAM | 5 | 30 |
| Nvblox Mapping | N/A | 30 |

## Key Advantages

- **NITROS**: Zero-copy GPU-to-GPU messaging (10ms latency)
- **TensorRT**: FP16/INT8 quantization for speed
- **Jetson Optimized**: Runs on edge devices (Orin, Xavier)

## Troubleshooting

**No GPU detected**: Check `nvidia-smi`, install CUDA drivers
**NITROS errors**: Ensure all nodes support NITROS (check package versions)
**Low FPS**: Reduce image resolution, enable FP16 quantization

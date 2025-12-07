---
sidebar_position: 3
title: "Chapter 3: Unity Animation and Rendering"
description: Unity Robotics Hub, articulation bodies, and ML-Agents
tags: [unity, robotics-hub, articulation-bodies, ml-agents, rendering]
---

# Chapter 3: Unity Animation and Rendering

## Learning Objectives

1. Use Unity Robotics Hub to import URDF and connect to ROS 2
2. Configure articulation bodies for realistic robot physics
3. Compare Unity and Gazebo to select the appropriate tool

## 3.1 Unity Robotics Hub

### Overview

**Unity Robotics Hub**: Open-source package for integrating Unity with ROS/ROS 2

**Components**:
1. **URDF Importer**: Convert URDF → Unity GameObjects
2. **ROS-TCP-Connector**: Bidirectional message passing (Unity ↔ ROS 2)
3. **Visualizations**: Sensor data rendering (point clouds, markers)

**Why Unity for Robotics?**
- **Photorealistic rendering**: Real-time ray tracing, global illumination
- **High-performance**: Optimized for games (60+ FPS with complex scenes)
- **Cross-platform**: Desktop, mobile, VR/AR, cloud
- **Asset ecosystem**: 3D models, environments (Unity Asset Store)
- **ML-Agents**: Reinforcement learning framework (Unity ML-Agents)

### Installation

```bash
# Unity Hub + Unity Editor (2021.3 LTS recommended)
# Install via Unity Hub: https://unity.com/download

# Add packages via Package Manager
# 1. URDF Importer: com.unity.robotics.urdf-importer
# 2. ROS-TCP-Connector: com.unity.robotics.ros-tcp-connector
```

**ROS 2 Side**:
```bash
sudo apt install ros-humble-ros-tcp-endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### URDF Import Workflow

**Step 1: Prepare URDF**
- Ensure mesh files (STL, OBJ, DAE) are referenced correctly
- Use relative paths: `package://robot_description/meshes/base.stl`

**Step 2: Import in Unity**
```
Assets → Import Robot from URDF
Select URDF file
Configure import settings:
  - Axis Convention: Y-up (Unity) or Z-up (ROS)
  - Mesh Decomposition: Convex (for collision)
  - Material: Standard (PBR)
```

**Step 3: Articulation Body Conversion**
- Unity auto-creates ArticulationBody components for joints
- Hierarchy: Root (immovable) → links connected by articulations

**Example Hierarchy**:
```
Robot_Root (ArticulationBody: Fixed)
├── base_link
│   ├── left_wheel (ArticulationBody: Continuous)
│   ├── right_wheel (ArticulationBody: Continuous)
│   └── camera_link (ArticulationBody: Fixed)
```

### ROS-TCP-Connector Setup

**Unity Side** (C# script):
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>("cmd_vel");
        ros.Subscribe<TwistMsg>("unity/cmd_vel", OnCmdVel);
    }

    void OnCmdVel(TwistMsg msg)
    {
        // Apply velocity to robot
        float linear = (float)msg.linear.x;
        float angular = (float)msg.angular.z;
        // ...control logic
    }

    void PublishOdometry()
    {
        var odom = new TwistMsg();
        // ...populate odom
        ros.Publish("odom", odom);
    }
}
```

**ROS 2 Side**:
```bash
# Start TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Publish to Unity
ros2 topic pub /unity/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"

# Subscribe from Unity
ros2 topic echo /odom
```

**Message Types**: Auto-generated from ROS .msg files

## 3.2 Animation and Rendering

### Articulation Bodies

**Unity Physics**: PhysX (NVIDIA's physics engine)

**ArticulationBody**: Unity's component for robotic joints (replaces older Rigidbody chains)

**Advantages**:
- Reduced jitter (stable multi-body dynamics)
- Direct joint control (position, velocity, force)
- Better for high-DOF robots (humanoids, manipulators)

**Joint Types**:
- **Fixed**: Welded (sensor mounts)
- **Revolute**: Hinge with limits
- **Prismatic**: Linear slide
- **Spherical**: Ball joint (3-DOF)

**Configuration** (via Unity Inspector):
```
ArticulationBody Component:
  - Anchor Position/Rotation: Joint origin
  - Joint Type: Revolute/Prismatic/Fixed
  - X/Y/Z Motion: Locked/Limited/Free
  - Limits: Min/Max angle (revolute), Min/Max distance (prismatic)
  - Stiffness: PD controller gains
  - Damping: Velocity damping
```

**Control Modes**:
1. **Force**: Apply torque, physics computes motion
2. **Acceleration**: Set target acceleration
3. **Velocity**: PD controller to target velocity
4. **Position**: PD controller to target angle

**Example** (setting joint velocity):
```csharp
ArticulationBody joint = GetComponent<ArticulationBody>();
var drive = joint.xDrive;
drive.target = 2.0f;  // rad/s (for revolute)
drive.targetVelocity = 2.0f;
joint.xDrive = drive;
```

### High-Fidelity Rendering

**Physically Based Rendering (PBR)**:
- Metallic/Roughness workflow
- Realistic materials (metal, plastic, rubber)
- HDRP (High Definition Render Pipeline) for ray tracing

**Lighting**:
- **Directional Light**: Sunlight (parallel rays)
- **Point Light**: Omnidirectional (lightbulb)
- **Spot Light**: Cone-shaped (flashlight)
- **Area Light**: Soft shadows (HDRP only)
- **Global Illumination**: Indirect lighting (baked lightmaps)

**Post-Processing**:
- Bloom, lens flare, motion blur
- Color grading, vignette
- Depth of field (focus on robot, blur background)

**Camera Simulation**:
```csharp
// Capture camera feed
Camera cam = GetComponent<Camera>();
RenderTexture rt = new RenderTexture(640, 480, 24);
cam.targetTexture = rt;
cam.Render();

// Read pixels → publish to ROS
Texture2D tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
RenderTexture.active = rt;
tex.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
tex.Apply();

// Convert to sensor_msgs/Image
byte[] bytes = tex.GetRawTextureData();
// ...publish via ROS-TCP-Connector
```

### Unity ML-Agents

**ML-Agents**: Reinforcement learning framework for Unity

**Architecture**:
```
┌─────────────────┐
│ Python Trainer  │ (PyTorch/TensorFlow)
└────────┬────────┘
         │ RPC (grpc)
┌────────┴────────┐
│ Unity C# Agent  │ (Environment)
└─────────────────┘
```

**Agent Example** (robot navigation):
```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RobotAgent : Agent
{
    public Transform target;

    public override void OnEpisodeBegin()
    {
        // Reset robot position, randomize target
        transform.position = new Vector3(0, 0, 0);
        target.position = Random.insideUnitSphere * 5f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // State: robot pos, target pos, robot velocity
        sensor.AddObservation(transform.position);
        sensor.AddObservation(target.position);
        sensor.AddObservation(GetComponent<Rigidbody>().velocity);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Actions: linear/angular velocity
        float linear = actions.ContinuousActions[0];
        float angular = actions.ContinuousActions[1];

        // Apply to robot
        rb.AddForce(transform.forward * linear * speed);
        rb.AddTorque(Vector3.up * angular * torque);

        // Reward: distance to target
        float dist = Vector3.Distance(transform.position, target.position);
        if (dist < 0.5f) {
            SetReward(1.0f);
            EndEpisode();
        } else {
            SetReward(-0.01f * dist);
        }
    }
}
```

**Training** (Python):
```bash
mlagents-learn config.yaml --run-id=robot_nav_01
# Play in Unity Editor → agent learns
```

**Benefits**:
- Parallel environments (1000s of robots in one scene)
- Curriculum learning (progressively harder tasks)
- Imitation learning (demonstrations → policy)

## 3.3 Unity vs Gazebo

### When to Use Unity

**Advantages**:
- **Visuals**: Photorealistic (ray tracing, PBR)
- **Performance**: Faster rendering (optimized for games)
- **Cross-platform**: VR/AR support, mobile deployment
- **Asset ecosystem**: Pre-built environments, objects
- **ML-Agents**: Integrated RL framework

**Use Cases**:
1. **Synthetic data generation**: Train computer vision models
2. **Human-robot interaction**: VR teleoperation, AR overlays
3. **Marketing/demos**: Photorealistic videos for presentations
4. **Large-scale RL**: 1000s of parallel agents
5. **Mobile/AR robotics**: Visualize robot in real environment (ARCore/ARKit)

### When to Use Gazebo

**Advantages**:
- **Physics accuracy**: DART engine for manipulation
- **ROS 2 integration**: Native, no TCP bridge
- **Sensor fidelity**: Realistic LiDAR, radar models
- **Open-source**: Free, community-maintained
- **Determinism**: Reproducible results (critical for research)

**Use Cases**:
1. **Contact-rich tasks**: Manipulation, grasping, assembly
2. **Legged locomotion**: Accurate foot contacts
3. **ROS 2 ecosystem**: Seamless integration with Nav2, MoveIt
4. **Research**: Deterministic experiments, open-source transparency
5. **Deployment testing**: Closest to hardware behavior

### Comparison Table

| Feature | Unity | Gazebo |
|---------|-------|--------|
| **Rendering** | Photorealistic | Basic |
| **Physics Accuracy** | Medium | High (DART) |
| **ROS 2 Integration** | TCP bridge | Native |
| **Performance** | Fast (60+ FPS) | Medium |
| **RL Support** | ML-Agents | External (Isaac, etc) |
| **License** | Free (Personal) | Open-source |
| **Best For** | Vision, VR/AR, ML | Manipulation, locomotion |

### Hybrid Approach

**Scenario**: Train vision policy in Unity, validate control in Gazebo

**Workflow**:
1. **Unity**: Generate synthetic images (domain randomization)
2. **Train**: Object detection model (YOLOv8, Mask R-CNN)
3. **Gazebo**: Deploy detector + motion planner
4. **Validate**: Pick-and-place task with accurate physics
5. **Transfer**: Real robot with fine-tuning

**Example**: Amazon Robotics Challenge
- Unity for vision training (synthetic shelves with varied products)
- Gazebo for grasp planning (DART physics for contact)
- Real robot deployment

## Summary

**Unity Robotics Hub**: URDF import, ROS-TCP-Connector for ROS 2 integration
**Articulation Bodies**: PhysX-based robot physics with joint control
**Unity vs Gazebo**: Unity (visuals, RL, VR) vs Gazebo (accuracy, ROS 2 native)



## Exercises

**Exercise 3.1**: Import a robot URDF into Unity and configure ArticulationBody components for a 3-DOF arm (base rotation, shoulder, elbow). Set joint limits, stiffness, and damping. Verify the robot can reach a target position using position control.

**Exercise 3.2**: Implement a Unity ML-Agents environment for robot navigation. The agent should:
- Observe: Robot position, target position, obstacle positions
- Act: Linear and angular velocity commands
- Reward: +1 for reaching target, -0.01 per timestep, -1 for collision
Train with PPO for 1M steps and measure success rate.

**Exercise 3.3**: Generate synthetic data for object detection. Create a Unity scene with 100 random objects (varying poses, lighting, backgrounds). Capture 10k images with bounding box annotations. Train YOLOv8 and measure mAP@0.5.

**Exercise 3.4**: Compare Unity and Gazebo for a specific robot task (your choice). Measure:
- Rendering performance (FPS)
- Physics accuracy (e.g., contact force errors)
- ROS 2 integration latency (message round-trip time)
- Ease of use (subjective, but document setup time)
Present findings in a table with recommendations.


**Next**: Chapter 4 integrates Gazebo and Unity with ROS 2 for multi-simulator workflows.

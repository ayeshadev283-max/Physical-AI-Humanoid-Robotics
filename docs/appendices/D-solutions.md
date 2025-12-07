---
sidebar_position: 4
title: "Appendix D: Exercise Solutions"
description: Solutions to chapter exercises
---

# Appendix D: Exercise Solutions

Solutions to exercises from each module. Try solving problems independently before consulting solutions.

## Module 0: Physical AI Foundations

### Chapter 1: Introduction

**Exercise 1.1**: Define Physical AI and explain how it differs from traditional AI systems.

**Solution**:
Physical AI refers to artificial intelligence systems that possess physical embodiment and interact with the real world through sensors and actuators. Key differences from traditional AI:

1. **Real-time constraints**: Must process sensor data and generate control commands within strict time limits (e.g., 100Hz for motor control)
2. **Physical dynamics**: Must account for inertia, friction, and other physical phenomena
3. **Uncertainty**: Sensor noise, actuation errors, and environmental unpredictability
4. **Safety-critical**: Physical actions can cause damage or harm
5. **Embodiment**: The physical body itself contributes to computation (morphological computation)

Traditional AI systems (e.g., language models, recommendation systems) operate in purely digital domains without these constraints.

---

**Exercise 1.2**: List three real-world applications of humanoid robots and their primary challenges.

**Solution**:

1. **Healthcare assistance**
   - Challenge: Safe physical interaction with vulnerable patients, unpredictable human behavior

2. **Warehouse automation**
   - Challenge: Dexterous manipulation of varied objects, navigation in dynamic environments

3. **Disaster response**
   - Challenge: Operating in unknown, hazardous environments with degraded communication

---

### Chapter 2: Embodied Intelligence

**Exercise 2.1**: Explain the embodiment hypothesis with a concrete example.

**Solution**:
The embodiment hypothesis states that intelligence emerges from the interaction between brain, body, and environment—not from abstract reasoning alone.

**Example**: Human infants learn object permanence through physical manipulation:
- Reaching for objects → hand-eye coordination
- Dropping objects → understanding gravity
- Stacking blocks → learning balance and stability

A purely digital AI could learn these concepts from data, but an embodied robot learns them through sensorimotor experience, which may lead to more robust and generalizable understanding.

---

**Exercise 2.2**: Describe morphological computation and provide a robotics example.

**Solution**:
Morphological computation: Using the physical structure of a robot to simplify control problems.

**Example**: Passive dynamic walkers
- Simple bipedal robots with no motors or sensors
- Walk down slopes purely due to mechanical design (leg length, mass distribution, joint compliance)
- Demonstrates that control complexity can be offloaded to body morphology
- Modern application: Boston Dynamics uses leg compliance to absorb landing impacts without complex control

---

### Chapter 3: Sensing and Perception

**Exercise 3.1**: Design a sensor fusion system combining camera and LiDAR for obstacle detection.

**Solution**:

```python
class ObstacleDetector:
    def __init__(self):
        self.camera_sub = rospy.Subscriber('/camera/image', Image, self.camera_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        
    def fuse_sensors(self, camera_obstacles, lidar_obstacles):
        """
        Sensor fusion logic:
        - Camera: Good for classification (car, person, box)
        - LiDAR: Accurate distance measurement
        - Combine: Classified obstacles with precise 3D location
        """
        fused_obstacles = []
        for lidar_obs in lidar_obstacles:
            # Find matching camera detection
            for cam_obs in camera_obstacles:
                if self.spatial_match(lidar_obs, cam_obs):
                    fused_obstacles.append({
                        'position': lidar_obs.position,  # Accurate from LiDAR
                        'class': cam_obs.class_label,    # From camera
                        'confidence': min(lidar_obs.conf, cam_obs.conf)
                    })
        return fused_obstacles
```

**Advantages**:
- LiDAR provides accurate range (immune to lighting)
- Camera provides semantic classification
- Fusion improves robustness (redundancy if one sensor fails)

---

### Chapter 4: Locomotion and Motor Control

**Exercise 4.1**: Calculate inverse kinematics for a 2-DOF planar arm.

**Solution**:

Given:
- Link lengths: L₁ = 0.5m, L₂ = 0.3m
- Target end effector position: (x, y) = (0.6, 0.4)

**Inverse Kinematics (geometric approach)**:

```python
import numpy as np

def inverse_kinematics_2dof(x, y, L1, L2):
    # Distance from origin to target
    D = np.sqrt(x**2 + y**2)
    
    # Check reachability
    if D > L1 + L2 or D < abs(L1 - L2):
        raise ValueError("Target unreachable")
    
    # Angle θ2 (elbow angle)
    cos_theta2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(cos_theta2)  # Elbow-up solution
    
    # Angle θ1 (shoulder angle)
    alpha = np.arctan2(y, x)
    beta = np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    theta1 = alpha - beta
    
    return theta1, theta2

# Example
theta1, theta2 = inverse_kinematics_2dof(0.6, 0.4, 0.5, 0.3)
print(f"θ1 = {np.degrees(theta1):.2f}°")
print(f"θ2 = {np.degrees(theta2):.2f}°")
```

**Note**: Multiple solutions exist (elbow-up vs elbow-down). Choose based on singularity avoidance and joint limits.

---

## Module 1: ROS 2 Ecosystem

*Solutions will be added as Module 1 exercises are created*

---

## Module 2: Digital Twin

*Solutions will be added as Module 2 exercises are created*

---

## Module 3: NVIDIA Isaac

*Solutions will be added as Module 3 exercises are created*

---

## Module 4: VLA Models

*Solutions will be added as Module 4 exercises are created*

---

## Module 5: Capstone Project

*Solutions will be added as Module 5 exercises are created*

---

## Additional Practice Problems

For more practice problems and coding challenges, see:
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **Isaac Sim Tutorials**: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials/index.html
- **Robotics Stack Exchange**: https://robotics.stackexchange.com/

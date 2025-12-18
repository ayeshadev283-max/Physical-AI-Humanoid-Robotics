---
sidebar_position: 4
title: "Chapter 4: Locomotion and Motor Control"
description: Locomotion strategies, kinematics, and balance
tags: [locomotion, motor-control, kinematics, balance]
---
<!-- import { ChatbotWidget } --> from '@site/src/components/ChatbotWidget';

# Chapter 4: Locomotion and Motor Control

## Learning Objectives

1. Compare locomotion strategies (wheeled, legged, hybrid)
2. Understand forward/inverse kinematics
3. Explain balance and stability for legged robots

## 4.1 Locomotion Strategies

### Wheeled

**Advantages**:
- Energy efficient on flat ground
- Simple control (differential drive, Ackermann steering)
- High speed, stable

**Disadvantages**:
- Cannot climb stairs
- Poor on rough terrain
- Limited maneuverability in tight spaces

**Use Cases**: Warehouses, roads, indoor navigation

### Legged (Bipedal, Quadrupedal, Hexapod)

**Bipedal** (humanoids):
- Human environments (stairs, narrow paths)
- **Challenge**: Unstable (small support polygon)

**Quadrupedal** (Spot, ANYmal):
- Stable (always 3+ feet on ground)
- Good rough terrain performance
- **Trade-off**: Larger footprint than bipeds

**Hexapod**:
- Maximum stability (can lift 3 legs, still stable)
- Slow, complex coordination

**Advantages**: Stairs, rocks, rubble, disaster sites

**Disadvantages**: High energy cost, complex control

### Hybrid

**Wheeled-Legged** (Handle by Boston Dynamics):
- Wheels on legs
- Efficiency of wheels + mobility of legs

**Transforming**: Switch modes based on terrain

## 4.2 Motor Control Fundamentals

### Forward Kinematics

**Problem**: Given joint angles θ, find end effector pose (x, y, z, orientation)

**Example** (2D, 2-link arm):
```
x = L1*cos(θ1) + L2*cos(θ1+θ2)
y = L1*sin(θ1) + L2*sin(θ1+θ2)
```

**Use**: Visualization, collision checking

### Inverse Kinematics (IK)

**Problem**: Given desired pose (x, y, z), find joint angles θ

**Challenges**:
- Multiple solutions (elbow up/down)
- No solution (out of reach)
- Singularities (loss of DOF)

**Methods**:
- **Analytical**: Closed-form (fast, limited to simple chains)
- **Numerical**: Jacobian-based optimization (general, slower)

**Use**: Reaching, manipulation, walking (foot placement)

### Dynamics

**Forward Dynamics**: Forces → accelerations
**Inverse Dynamics**: Accelerations → required forces/torques

**Application**: Gravity compensation (hold arm against gravity)

## 4.3 Balance and Stability

### Zero Moment Point (ZMP)

**Definition**: Point on ground where net moment = 0

**Stable Walking**: ZMP inside support polygon (foot/feet area)

**Control Strategy**:
- Plan footsteps to keep ZMP inside
- Adjust center of mass (CoM) trajectory
- **Limitation**: Assumes flat ground, no slipping

### Central Pattern Generators (CPG)

**Biological Inspiration**: Spinal cord rhythmic patterns (no brain needed)

**Robot Implementation**:
- Coupled oscillators generate leg motions
- **Advantage**: Robust, natural gait transitions

**Example**: Quadruped trot (diagonal legs in phase)

### Learning-Based Control

**Reinforcement Learning**:
- Train in Isaac Gym (1000s of parallel robots)
- Reward: Forward velocity, penalty for falling
- **Result**: Emergent gaits (walk, trot, bound)

**Advantages**:
- Handles complex dynamics (humanoid = 20+ DOF)
- Adapts to terrain changes

**Challenges**:
- Sim-to-real gap
- Safety (no guarantees)

## Summary

**Locomotion**: Wheeled (efficient, limited terrain) vs Legged (versatile, complex)
**Control**: IK for reaching, dynamics for torque, ZMP for balance
**Modern**: Learning-based methods dominate for complex robots

**Module 0 Complete!** Next modules cover ROS 2, simulation, Isaac, and VLAs.

## Exercises

**Exercise 4.1**: For a 2-DOF robot arm with link lengths L1=1m and L2=0.5m, compute the forward kinematics for joint angles θ1=45° and θ2=30°. Then solve the inverse kinematics problem to find joint angles that reach the point (x=1.0, y=0.5).

**Exercise 4.2**: Compare the Zero Moment Point (ZMP) criterion with a learning-based balance controller. What are the advantages and disadvantages of each approach for humanoid walking?

**Exercise 4.3**: Design a hybrid locomotion system for a search-and-rescue robot that must navigate both urban environments (flat) and disaster sites (rubble). Specify the locomotion modes, when to switch between them, and how the switching decision would be made.

**Exercise 4.4**: Research one of the locomotion robots mentioned (Spot, ANYmal, Handle, or Atlas) and write a technical summary including: degrees of freedom, actuation type, key sensors, and one example application where its locomotion strategy provides a unique advantage.


---

## Ask Questions About This Chapter

Have questions about the content? Use our AI-powered chatbot to get instant answers based on the book material:

<!-- <ChatbotWidget 
  bookId="physical-ai-robotics" 
  chapterNumber={0} 
 /> -->

:::tip
The chatbot provides answers grounded in the book content with source references. Try asking questions about the concepts covered in this chapter.
:::

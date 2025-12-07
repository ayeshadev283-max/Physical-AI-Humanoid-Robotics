---
sidebar_position: 1
slug: /
title: Introduction
---

# Physical AI & Humanoid Robotics

**A Graduate-Level Technical Textbook**

Welcome to the Physical AI & Humanoid Robotics textbook—an open-source, comprehensive guide to building intelligent, embodied systems. This book bridges the gap between theoretical AI and physical robot implementation, focusing on modern tools and frameworks used in industry and research.

## What is Physical AI?

**Physical AI** refers to artificial intelligence systems that interact with the physical world through sensors, actuators, and robot bodies. Unlike purely digital AI (e.g., large language models running in data centers), Physical AI systems must:

- **Perceive** the world through noisy, real-time sensor data (cameras, LiDAR, IMUs)
- **Reason** about uncertainty, dynamics, and partial observability
- **Act** through continuous control of motors, grippers, and locomotion systems
- **Learn** from physical interaction and simulation

This textbook focuses on **humanoid robots**—systems designed to operate in human environments, with bipedal locomotion, dexterous manipulation, and multimodal perception.

## Who is This Book For?

This textbook is designed for:

- **Graduate students** in robotics, computer science, or electrical engineering
- **Researchers** exploring embodied AI and vision-language-action models
- **Engineers** building real-world robotic systems with ROS 2, Gazebo, Isaac, or Unity
- **Self-learners** with programming experience and interest in robotics

### Prerequisites

- **Programming**: Python 3.x, basic C++ (for ROS 2 and performance-critical code)
- **Mathematics**: Linear algebra, probability, basic calculus (derivatives, gradients)
- **AI/ML**: Familiarity with neural networks, reinforcement learning (helpful but not required)

## Book Structure

This book is organized into **six modules**, each building on the previous:

### Module 0: Physical AI Foundations

Foundational concepts:
- **Embodied AI**: Why physical bodies matter for intelligence
- **Subsumption Architecture**: Behavior-based robotics (Brooks, 1991)
- **Probabilistic Robotics**: Handling sensor noise and uncertainty (Bayes filters, Kalman filters)

### Module 1: ROS 2 (Robotic Nervous System)

The computational graph for robot software:
- **Nodes, Topics, Services, Actions**: Core ROS 2 communication patterns
- **Launch Files**: Orchestrating multi-node systems
- **URDF & Transforms**: Robot kinematics and coordinate frames

### Module 2: Gazebo & Unity (Digital Twin)

Simulation for rapid iteration and zero-cost failure:
- **Gazebo Harmonic**: Physics-based simulation with SDF models
- **Unity Robotics Hub**: Photorealistic rendering and imitation learning data collection
- **Sim-to-Real Transfer**: Bridging the reality gap

### Module 3: NVIDIA Isaac (AI-Robot Brain)

GPU-accelerated robotics for AI-native workflows:
- **Isaac Sim**: High-fidelity simulation with RTX ray tracing and physics
- **Isaac Gym**: Massively parallel RL training (thousands of environments)
- **Isaac ROS**: Real-time perception with DNN inference on Jetson/Orin

### Module 4: Vision-Language-Action (VLA)

Foundation models for generalizable robot control:
- **OpenVLA**: 7B parameter model for vision-language-action tasks
- **SmolVLA**: 450M parameter efficient model for edge deployment
- **Fine-tuning and Deployment**: Adapting VLAs to custom tasks

### Module 5: Capstone Project

End-to-end integration:
- **Humanoid Robot Simulation**: Full-body control with Unitree H1 or iCub
- **Task Planning**: High-level reasoning with VLA models
- **Deployment**: Running on real hardware or cloud infrastructure

## How to Use This Book

### Linear vs. Modular Approach

- **Linear Path (Recommended for Beginners)**: Start with Module 0 and progress sequentially through Module 5
- **Modular Path (For Experienced Learners)**: Skip to specific modules based on your interests:
  - Already familiar with ROS 2? Jump to Module 2 or 3
  - Want to explore VLA models immediately? Read Module 0 for context, then skip to Module 4

### Code Examples

All code examples are:
- **Tested and Runnable**: Dockerized environments ensure reproducibility
- **Annotated**: Inline comments explain design decisions
- **Open-Source**: Licensed under Apache 2.0, freely modifiable

Find them in the [`examples/` directory](https://github.com/your-organization/physical-ai-book/tree/main/examples).

### Interactive Elements

- **Diagrams**: Mermaid flowcharts and architecture diagrams embedded directly
- **Math**: Rendered with KaTeX for clarity (e.g., Kalman filter equations)
- **Videos**: Links to demonstrations and tutorials

## Learning Outcomes

By the end of this textbook, you will be able to:

1. **Explain** core principles of embodied AI and probabilistic robotics
2. **Implement** robot software using ROS 2 (nodes, launch files, custom messages)
3. **Simulate** robots in Gazebo, Unity, and Isaac Sim
4. **Train** RL policies in Isaac Gym and deploy them to simulated/real robots
5. **Fine-tune** vision-language-action models (OpenVLA/SmolVLA) for custom tasks
6. **Integrate** all components into a working humanoid robot system

## Contributing

This is an **open-source textbook**. We welcome contributions:

- **Report Errors**: Found a typo, broken link, or incorrect formula? [Open an issue](https://github.com/your-organization/physical-ai-book/issues)
- **Suggest Improvements**: Have ideas for new sections or better explanations? [Start a discussion](https://github.com/your-organization/physical-ai-book/discussions)
- **Submit Content**: Write new chapters, add code examples, or improve diagrams. See [CONTRIBUTING.md](https://github.com/your-organization/physical-ai-book/blob/main/CONTRIBUTING.md)

## License

- **Content** (all chapters, appendices): [CC BY-SA 4.0](https://github.com/your-organization/physical-ai-book/blob/main/LICENSE.content.md)
- **Code** (examples, Dockerfiles): [Apache 2.0](https://github.com/your-organization/physical-ai-book/blob/main/LICENSE.code.md)

## Acknowledgments

This textbook builds on decades of robotics research. Key influences include:

- Brooks (1991): Subsumption architecture and embodied AI
- Thrun et al. (2005): Probabilistic robotics and SLAM
- Siciliano & Khatib (2016): Springer Handbook of Robotics
- Open Robotics: ROS 2 framework
- NVIDIA: Isaac platform
- Researchers at UC Berkeley, Stanford, CMU: OpenVLA and modern VLA models

Full bibliography: [Bibliography](references/bibliography.md)

## Let's Begin

Ready to dive into Physical AI? Use the sidebar to navigate to Module 0: Physical AI Foundations and begin your journey.

---

**Questions or Feedback?** Open an issue or discussion on [GitHub](https://github.com/your-organization/physical-ai-book).

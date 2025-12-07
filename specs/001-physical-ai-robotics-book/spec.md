# Feature Specification: Physical AI & Humanoid Robotics Book

**Feature Branch**: `001-physical-ai-robotics-book`
**Created**: 2025-12-05
**Updated**: 2025-12-07
**Status**: In Progress
**Input**: "Create an AI/Spec-driven Docusaurus book on Physical AI and Humanoid Robotics. Target: Robotics learners, engineering students, developers. Focus: High-level concepts, diagrams, workflows, conceptual code - NO installation tutorials. Covers: Physical AI foundations, ROS2, Gazebo/Unity, NVIDIA Isaac, VLA (OpenVLA/RT-2 reference), Autonomous Humanoid capstone."

## Known Issues

### Docusaurus Homepage Not Loading (Priority: P1)

**Issue**: First page of Docusaurus (/docs/) is not loading correctly.

**Suspected Causes**:
- Wrong baseUrl configuration in docusaurus.config.js
- Incorrect docs route or sidebar ID mismatch
- Navbar logo href pointing to /docs/ but entry file missing
- Missing or misconfigured index.md in /docs/

**Impact**: Users cannot access the book homepage, blocking all content navigation.

**Current Configuration**:
- baseUrl: `/` (docusaurus.config.js:19)
- Navbar logo href: `/docs/` (docusaurus.config.js:78)
- Sidebar ID: `bookSidebar` (docusaurus.config.js:83, sidebars.js:17)
- Index file exists: `docs/index.md` with `slug: /` and `sidebar_position: 1`

**Next Steps**: Investigate routing configuration, verify slug conflicts, ensure docs preset routing is correct.

## Module Structure & Flow

This section defines the detailed chapter-level organization within each module, establishing the learning progression and conceptual flow.

### Module 0: Physical AI Foundations

**Flow**:
1. Introduction to Physical AI
2. Embodied Intelligence concepts
3. Sensing & Perception systems
4. Locomotion & Motor Control fundamentals

**Purpose**: Establish foundational understanding of what Physical AI is, how robots perceive and move in the physical world, and why embodiment matters for intelligence.

### Module 1: ROS 2 (Robotic Nervous System)

**Flow**:
1. Core Concepts (computational graph, DDS, QoS)
2. Nodes, Topics, Services, Actions (communication patterns)
3. TF (Transform) system & URDF (robot modeling)
4. Simulation Pipeline integration with ROS 2

**Purpose**: Master the middleware that connects all robot components, enabling distributed computation and communication.

### Module 2: Digital Twin (Gazebo + Unity)

**Flow**:
1. Digital Twin Basics (simulation principles, why simulate)
2. Gazebo Physics (SDF models, physics engines, sensors)
3. Unity Animation & Rendering (photorealism, imitation learning data)
4. ROS 2 Integration patterns for both simulators

**Purpose**: Learn to create virtual representations of robots for safe testing, rapid iteration, and AI training.

### Module 3: NVIDIA Isaac (AI-Robot Brain)

**Flow**:
1. Isaac Overview (Isaac Sim, Isaac Gym, Isaac ROS ecosystem)
2. Perception & Planning (DNN inference, object detection, pose estimation)
3. Sim-to-Real transfer (domain randomization, synthetic data generation)
4. Control Loops (RL policies, navigation, manipulation primitives)

**Purpose**: Integrate AI capabilities into robotic systems using GPU-accelerated simulation and perception.

### Module 4: Vision-Language-Action (VLA) Models

**Flow**:
1. VLA Fundamentals (architecture, vision-language grounding)
2. RT-2 and Modern VLA Models (OpenVLA, SmolVLA, model comparison)
3. Policy Integration (deploying VLA models with ROS 2)
4. Humanoid Skills (manipulation, navigation with natural language)

**Purpose**: Enable robots to understand natural language commands and execute complex tasks through learned policies.

### Module 5: Capstone Project — Autonomous Humanoid

**Flow**:
1. System Overview & Architecture Design
2. Perception Stack Integration (vision, language understanding)
3. Control Stack Integration (planning, execution, low-level control)
4. VLA-Based Autonomy (end-to-end task execution)

**Purpose**: Synthesize all concepts into a working autonomous humanoid system demonstrating mastery of Physical AI principles.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Physical AI Foundations (Priority: P1)

A robotics learner needs to understand what Physical AI is, how it differs from digital AI, and the architectural components that enable robots to perceive, think, and act in the physical world.

**Why this priority**: Foundation module establishes the conceptual framework for understanding the subsequent technical modules (ROS 2, Gazebo/Unity, Isaac, VLA). Without this grounding, learners cannot contextualize the purpose and integration of each technology stack.

**Independent Test**: Can be tested by having a learner complete the foundations module and then correctly identify and explain the role of each component (nervous system, digital twin, AI brain, VLA) in a simple autonomous robot scenario.

**Acceptance Scenarios**:

1. **Given** a learner with AI/ML background, **When** they complete the Physical AI Foundations module, **Then** they can explain what Physical AI is, differentiate it from digital AI, and describe the four key subsystems (perception, planning, control, learning)
2. **Given** a learner encounters the conceptual architecture, **When** they study the system diagrams, **Then** they can map real-world robot components to the Robotic Nervous System, Digital Twin, AI-Robot Brain, and VLA frameworks
3. **Given** a learner completes foundation chapters, **When** they move to technical modules, **Then** they understand why ROS 2 is needed (nervous system), why simulation matters (digital twin), what Isaac provides (AI brain), and how VLA enables task learning

---

### User Story 2 - ROS 2: The Robotic Nervous System (Priority: P1)

A learner needs to understand and use ROS 2 as the communication backbone for robot systems, including nodes, topics, services, actions, parameters, and how to build distributed robotic applications.

**Why this priority**: ROS 2 is the industry-standard middleware for robotics. Understanding ROS 2 is essential for all subsequent modules (simulation, AI integration, VLA deployment) and the capstone project.

**Independent Test**: Can be tested by having a learner create a simple ROS 2 workspace with multiple nodes that communicate via topics and services, demonstrating understanding of the publish-subscribe pattern and ROS 2 architecture.

**Acceptance Scenarios**:

1. **Given** a learner completes the ROS 2 module, **When** they set up a ROS 2 workspace, **Then** they can create nodes, define custom messages, publish/subscribe to topics, and use services for synchronous communication
2. **Given** a learner studies ROS 2 architectural concepts, **When** they design a simple robot system, **Then** they can decompose functionality into appropriate nodes, select communication patterns (topics vs services vs actions), and justify their design decisions
3. **Given** a learner encounters ROS 2 code examples, **When** they run and modify them, **Then** they can trace message flow, debug communication issues using ROS 2 CLI tools, and extend examples with new functionality

---

### User Story 3 - Gazebo & Unity: The Digital Twin (Priority: P1)

A learner needs to understand robot simulation environments, how to create digital twins of physical robots, and how to use simulation for testing, training, and validation before deploying to real hardware.

**Why this priority**: Simulation is essential for safe, cost-effective robot development. Digital twins enable rapid prototyping, reinforcement learning, and testing scenarios too dangerous or expensive for real hardware. This knowledge is foundational for the Isaac and VLA modules.

**Independent Test**: Can be tested by having a learner create a simulated robot environment in Gazebo, spawn a robot model (URDF/SDF), configure sensors, and run basic control loops while visualizing sensor data.

**Acceptance Scenarios**:

1. **Given** a learner completes the Digital Twin module, **When** they work with Gazebo, **Then** they can create world files, spawn robots from URDF/SDF descriptions, attach sensors (cameras, LiDAR, IMU), and collect simulated sensor data
2. **Given** a learner studies Unity for robotics simulation, **When** they compare it with Gazebo, **Then** they can explain the tradeoffs (physics fidelity vs rendering quality, ROS integration vs visual realism), and select appropriate tools for different use cases
3. **Given** a learner uses simulation for robot development, **When** they implement a control algorithm, **Then** they can test it in simulation first, validate behavior, and understand the sim-to-real gap and transfer learning challenges

---

### User Story 4 - NVIDIA Isaac: The AI-Robot Brain (Priority: P1)

A learner needs to understand how to integrate AI capabilities into robotic systems using NVIDIA Isaac platform, including perception models, motion planning, reinforcement learning, and sim-to-real transfer.

**Why this priority**: Isaac represents the state-of-the-art in AI-enabled robotics, bridging the gap between AI research and physical deployment. Understanding Isaac is critical for implementing the VLA module and capstone project.

**Independent Test**: Can be tested by having a learner set up Isaac Sim, load a robot model, deploy a pre-trained perception model, and run a simple manipulation or navigation task using Isaac's AI capabilities.

**Acceptance Scenarios**:

1. **Given** a learner completes the Isaac module, **When** they work with Isaac Sim, **Then** they can load environments, configure robot assets, attach AI-powered sensors (depth cameras, semantic segmentation), and run perception pipelines
2. **Given** a learner studies Isaac's AI capabilities, **When** they explore pre-trained models, **Then** they can deploy object detection, pose estimation, or navigation models, evaluate their performance, and fine-tune for custom scenarios
3. **Given** a learner uses Isaac for robot learning, **When** they implement a reinforcement learning task, **Then** they can define reward functions, train policies in simulation, and understand domain randomization techniques for sim-to-real transfer

---

### User Story 5 - Vision-Language-Action (VLA) Models (Priority: P1)

A learner needs to understand how modern vision-language-action models enable robots to understand natural language commands, perceive their environment, and execute complex manipulation tasks through learned policies.

**Why this priority**: VLA represents the cutting edge of Physical AI, combining computer vision, natural language processing, and action learning. This module ties together all previous concepts (ROS 2 for deployment, simulation for training, Isaac for AI integration) into a unified end-to-end learning framework.

**Independent Test**: Can be tested by having a learner deploy a pre-trained VLA model, provide natural language commands, observe the model's vision-based understanding, and analyze the generated action sequences for a manipulation task.

**Acceptance Scenarios**:

1. **Given** a learner completes the VLA module, **When** they study VLA architectures, **Then** they can explain how vision encoders, language models, and action decoders work together, describe training approaches (imitation learning, RLHF), and understand the role of large-scale datasets
2. **Given** a learner works with a VLA model, **When** they provide natural language instructions, **Then** they can trace how language is grounded in visual perception, how the model generates action sequences, and understand failure modes and limitations
3. **Given** a learner explores VLA deployment, **When** they integrate a VLA model with ROS 2 and simulation, **Then** they can set up the vision pipeline, connect language input interfaces, and execute VLA-generated actions in simulated or real environments

---

### User Story 6 - Capstone: Autonomous Humanoid Robot Project (Priority: P2)

A learner needs to apply all learned concepts (Physical AI foundations, ROS 2, simulation, Isaac, VLA) by implementing an end-to-end autonomous humanoid robot system capable of perceiving, understanding commands, planning actions, and executing tasks.

**Why this priority**: The capstone project integrates all modules into a coherent system, demonstrating practical mastery. This is P2 because it requires completion of all P1 modules first, but it's the ultimate demonstration of learning outcomes.

**Independent Test**: Can be tested by evaluating a learner's capstone project against defined requirements: Does it use ROS 2? Does it include simulation? Does it integrate AI perception/planning? Does it respond to natural language? Does it execute manipulation or navigation tasks?

**Acceptance Scenarios**:

1. **Given** a learner has completed all prior modules, **When** they design their capstone project, **Then** they can define project scope, select appropriate technologies from the stack, create system architecture diagrams, and justify design decisions based on learned principles
2. **Given** a learner implements the capstone project, **When** they integrate ROS 2, simulation, and AI models, **Then** they can demonstrate a working system with perception, planning, and control components communicating via ROS 2, validated in simulation, and optionally deployed to real hardware
3. **Given** a learner completes the capstone, **When** they document and present their work, **Then** they can explain technical challenges encountered, solutions implemented, lessons learned, and how the project demonstrates mastery of Physical AI and humanoid robotics concepts

---

### Edge Cases

- What happens when a learner lacks ROS 2 or simulation experience? (Foundations module provides conceptual overview; each technical module explains concepts before workflows; detailed installation/setup in appendices only, not main content)
- What happens when hardware requirements (GPU for Isaac) cannot be met? (Document cloud-based alternatives and minimum specs in appendices; main content focuses on concepts applicable regardless of hardware)
- What happens when NVIDIA Isaac or VLA model versions change? (Document concepts and architectures that transcend specific versions; reference official docs for latest API details)
- What happens when learners have different goals (research vs industry vs hobbyist)? (Modular structure allows selective reading; focus on transferable concepts rather than tool-specific tutorials)
- What happens when learners want more depth in a specific area? (Provide extensive references to peer-reviewed papers, official documentation, and advanced resources)
- What happens when simulations don't transfer well to real robots? (Dedicated conceptual section on sim-to-real gap, domain randomization principles, when to prioritize real hardware)
- What happens when learners have different OS environments (Linux/Windows/macOS)? (Focus on platform-agnostic concepts; note that Docker/Linux examples in appendices apply cross-platform)

## Requirements *(mandatory)*

### Functional Requirements

**Module 0: Foundations**
- **FR-001**: Book MUST include a Foundations module explaining Physical AI, distinguishing it from digital AI, and introducing the four pillars: Robotic Nervous System (ROS 2), Digital Twin (Simulation), AI-Robot Brain (Isaac), and Vision-Language-Action (VLA)
- **FR-002**: Foundations module MUST include conceptual diagrams showing how perception, planning, control, and learning interact in autonomous robots
- **FR-003**: Foundations module MUST establish terminology, provide historical context, and explain why each subsequent technical module is necessary
- **FR-003a**: Module 0 MUST follow this chapter flow: (1) Introduction to Physical AI, (2) Embodied Intelligence concepts, (3) Sensing & Perception systems, (4) Locomotion & Motor Control fundamentals

**Module 1: ROS 2 - The Robotic Nervous System**
- **FR-004**: ROS 2 module MUST cover nodes, topics, services, actions, parameters, launch files, and the publish-subscribe communication pattern with conceptual explanations and architectural diagrams
- **FR-005**: ROS 2 module MUST include conceptual code examples showing message flow and communication patterns (NOT step-by-step installation tutorials)
- **FR-006**: ROS 2 module MUST explain DDS (Data Distribution Service), QoS policies, lifecycle nodes, and architectural differences from ROS 1
- **FR-007**: ROS 2 module MUST provide workflow diagrams showing how components interact; detailed setup instructions relegated to appendices
- **FR-007a**: Module 1 MUST follow this chapter flow: (1) Core Concepts (computational graph, DDS, QoS), (2) Nodes, Topics, Services, Actions, (3) TF & URDF, (4) Simulation Pipeline integration

**Module 2: Gazebo & Unity - The Digital Twin**
- **FR-008**: Digital Twin module MUST explain digital twin concepts, physics simulation principles, URDF/SDF modeling concepts, and sensor simulation architectures
- **FR-009**: Digital Twin module MUST compare Gazebo vs Unity tradeoffs (physics fidelity vs rendering, ROS integration vs visual quality) with decision frameworks
- **FR-010**: Digital Twin module MUST include high-level workflows and conceptual code showing simulation pipelines (NOT detailed installation steps)
- **FR-011**: Digital Twin module MUST explain sim-to-real transfer principles, domain randomization techniques, and validation strategies conceptually
- **FR-011a**: Module 2 MUST follow this chapter flow: (1) Digital Twin Basics, (2) Gazebo Physics, (3) Unity Animation & Rendering, (4) ROS 2 Integration patterns

**Module 3: NVIDIA Isaac - The AI-Robot Brain**
- **FR-012**: Isaac module MUST explain Isaac platform architecture, Omniverse concepts, and Isaac ROS integration patterns (high-level, not installation)
- **FR-013**: Isaac module MUST cover perception capabilities conceptually: object detection, pose estimation, depth estimation, semantic segmentation architectures
- **FR-014**: Isaac module MUST explain synthetic data generation (SDG) principles, reinforcement learning workflows, and training pipelines for manipulation/navigation
- **FR-015**: Isaac module MUST provide workflow diagrams and conceptual code showing AI model deployment patterns (setup details in appendices)
- **FR-015a**: Module 3 MUST follow this chapter flow: (1) Isaac Overview, (2) Perception & Planning, (3) Sim-to-Real transfer, (4) Control Loops

**Module 4: Vision-Language-Action (VLA)**
- **FR-016**: VLA module MUST explain VLA architectures, how vision encoders + language models + action decoders work together, with architectural diagrams and conceptual explanations
- **FR-017**: VLA module MUST cover training approaches conceptually: imitation learning from demonstrations, reinforcement learning with human feedback (RLHF), and large-scale robotics dataset requirements
- **FR-018**: VLA module MUST reference state-of-the-art models (RT-2, OpenVLA, SmolVLA) and explain integration patterns with ROS 2 at conceptual level
- **FR-019**: VLA module MUST explain failure modes, safety considerations, current research limitations, and ethical implications
- **FR-019a**: Module 4 MUST follow this chapter flow: (1) VLA Fundamentals, (2) RT-2 and Modern VLA Models, (3) Policy Integration with ROS 2, (4) Humanoid Skills

**Module 5: Capstone Project**
- **FR-020**: Capstone module MUST provide a complete project template: autonomous humanoid robot performing manipulation or navigation tasks using natural language commands
- **FR-021**: Capstone module MUST guide integration of all components: ROS 2 architecture, simulation for testing, Isaac for perception/planning, VLA for language grounding
- **FR-022**: Capstone module MUST include evaluation criteria, testing protocols, and documentation requirements
- **FR-023**: Capstone module MUST provide alternative project ideas for different skill levels and interests
- **FR-023a**: Module 5 MUST follow this chapter flow: (1) System Overview & Architecture Design, (2) Perception Stack Integration, (3) Control Stack Integration, (4) VLA-Based Autonomy

**Cross-Module Requirements**
- **FR-024**: Each module MUST include clear learning objectives, prerequisite knowledge, estimated completion time, and self-assessment questions
- **FR-025**: Each module MUST provide extensive diagrams explaining concepts, system architectures, data flows, and workflows (visual-first approach)
- **FR-026**: All code examples MUST be conceptual/illustrative (showing logic and patterns) - NOT step-by-step installation tutorials; setup details in appendices only
- **FR-027**: Book MUST follow APA citation format with at least 50% peer-reviewed sources (constitution requirement)
- **FR-028**: Book MUST include comprehensive glossary, bibliography, and appendices (setup guides, prerequisites, troubleshooting relegated to appendix sections)
- **FR-029**: Book MUST be deployed via Docusaurus to GitHub Pages with full-text search, responsive design, and dark mode support
- **FR-030**: Docusaurus homepage route (/docs/) MUST load correctly without routing errors, with proper baseUrl configuration, correct sidebar ID mapping, and functional navbar navigation

### Key Entities

- **Module**: Represents one of six major learning units (Foundations, ROS 2, Digital Twin, Isaac, VLA, Capstone). Each module contains multiple chapters, learning objectives, hands-on exercises, code examples, and assessments. Modules are designed to be completed sequentially.

- **Chapter**: Represents a coherent subtopic within a module (e.g., "ROS 2 Topics and Publishers" within the ROS 2 module). Contains introduction, conceptual explanations, diagrams, code walkthroughs, exercises, and summary/key takeaways.

- **Code Example**: Represents executable, reproducible code demonstrating a concept or workflow. Contains:
  - Source code (Python/C++/YAML)
  - Dockerfile or requirements.txt for dependencies
  - Setup instructions (environment, installation)
  - Usage instructions (how to run)
  - Expected outputs (terminal output, visualizations)
  - Troubleshooting common issues

- **Diagram**: Represents visual content explaining concepts, architectures, or workflows. Types include:
  - Conceptual diagrams (e.g., "Physical AI vs Digital AI")
  - System architecture diagrams (e.g., "ROS 2 node communication graph")
  - Workflow diagrams (e.g., "VLA training pipeline")
  - All diagrams include alt text for accessibility and source files for editing

- **Exercise**: Represents a hands-on learning activity. Contains:
  - Problem statement
  - Difficulty level (Beginner/Intermediate/Advanced)
  - Learning objectives addressed
  - Prerequisites (knowledge, software setup)
  - Step-by-step guidance or hints
  - Solution approach (not full solution, encourages exploration)

- **Assessment Question**: Represents self-check comprehension questions. Types include:
  - Multiple choice (conceptual understanding)
  - Short answer (explain in your own words)
  - Code analysis (what does this code do?)
  - Design question (propose a solution)

- **Reference/Citation**: Represents a cited source in APA format. Categorized as:
  - Peer-reviewed (journal articles, conference papers)
  - Official documentation (ROS 2 docs, Isaac docs)
  - Books (robotics textbooks, AI references)
  - Preprints/arXiv (cutting-edge research)
  - All references linked in bibliography with DOI/URL where available

- **Glossary Entry**: Represents a technical term definition. Contains:
  - Term name
  - Concise definition
  - Example usage in context
  - Module/chapter where introduced
  - Related terms (cross-references)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Learners can complete the Foundations module and correctly identify the purpose and relationship between ROS 2, Simulation, Isaac, and VLA in a system architecture diagram
- **SC-002**: Learners can set up ROS 2 development environment (Docker or native) and successfully run all Module 1 code examples within 2 hours using provided documentation
- **SC-003**: Learners can create a simulated robot in Gazebo or Unity, attach sensors, and collect sensor data after completing Module 2
- **SC-004**: Learners can deploy at least one pre-trained perception model using Isaac Sim and visualize results after completing Module 3
- **SC-005**: Learners can demonstrate a working capstone project integrating ROS 2, simulation, and AI components, validated through automated tests or video demonstration
- **SC-006**: At least 50% of cited sources are peer-reviewed (constitution requirement), verified by bibliography metadata
- **SC-007**: All code examples execute successfully in documented environments (Docker containers with pinned versions), verified by CI/CD pipeline
- **SC-008**: Docusaurus site builds without errors, renders correctly on mobile/tablet/desktop, and provides full-text search functionality
- **SC-009**: Each module includes at least 3 diagrams explaining key concepts, with all diagrams having descriptive alt text for accessibility
- **SC-010**: Book receives validation from at least two robotics researchers or educators confirming technical accuracy and pedagogical soundness

### Assumptions

- Learners have basic programming knowledge (Python preferred, some exposure to C++ helpful but not required)
- Learners have foundational AI/ML knowledge (neural networks, supervised learning concepts)
- Learners have access to a computer with minimum 16GB RAM (32GB recommended for Isaac module) or cloud compute resources
- Learners have access to NVIDIA GPU for Module 3 (Isaac) and Module 4 (VLA), or can use cloud-based alternatives (documented in appendix)
- Code examples use pinned versions: ROS 2 Humble/Iron, Gazebo Harmonic, Isaac Sim 2023.1+, specific VLA model checkpoints
- Book is created using Docusaurus and deployed to GitHub Pages for free, searchable online access
- Learners use Linux (Ubuntu 22.04 recommended) natively, via WSL2, or via Docker containers (cross-platform support documented)
- Book will be maintained with version updates as major dependencies evolve (migration guides provided)
- Isaac and VLA modules acknowledge rapid evolution and link to latest official documentation
- Capstone projects are evaluated through simulation demonstrations or optional real hardware deployment (real hardware NOT required)

### Out of Scope

- **Deep Academic Research**: Novel algorithm development, mathematical proofs, cutting-edge research contributions (focus is on learning existing techniques)
- **Hardware Building Guides**: Mechanical design, electrical circuits, motor driver selection, PCB design, assembling physical robots from parts
- **Installation Steps in Main Content**: Detailed installation tutorials, environment setup, dependency management (relegated to appendices; main content is concept-focused)
- **Vendor Comparisons**: Detailed product comparisons, pricing analysis, vendor selection criteria (focus on concepts applicable across platforms)
- **AI Ethics and Philosophy**: Comprehensive treatment of AI ethics, robot rights, societal implications (briefly acknowledged; not primary focus)
- **Non-Humanoid Robotics**: Drones, autonomous vehicles, industrial manipulators, quadrupeds (except brief conceptual examples)
- **Low-Level Firmware**: Motor controller firmware, embedded systems programming, RTOS development
- **Manufacturing & Production**: Supply chain, cost optimization, mass production, commercial deployment strategies
- **Non-ROS Middleware**: YARP, LCM, other robot middleware (ROS 2 is the standard; others mentioned for context only)
- **Comprehensive Math Derivations**: Full proofs of all algorithms (appendix provides math review; focus is conceptual understanding)
- **Custom VLA Training from Scratch**: Training VLA models with massive datasets and compute (focus on using/fine-tuning pre-trained models)
- **Real Hardware Deployment Details**: Physical robot purchase, calibration procedures, hardware maintenance (capstone completable in simulation)
- **Alternative Simulation Tools in Depth**: MuJoCo, PyBullet, Webots (Gazebo and Unity are primary; others mentioned briefly)
- **Multi-Robot Systems**: Swarm robotics, multi-agent coordination (focus is single autonomous humanoid)

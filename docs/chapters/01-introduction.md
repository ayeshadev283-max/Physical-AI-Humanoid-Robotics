---
id: chapter-1-introduction
title: "Chapter 1: Introduction to Physical AI"
sidebar_label: "1. Introduction"
sidebar_position: 1
description: "Comprehensive introduction to physical AI, embodied cognition, and the fundamental challenges of creating intelligent systems that interact with the physical world"
difficulty: beginner
estimated_time: 4 hours
prerequisites:
  - Basic artificial intelligence concepts
  - Fundamental machine learning knowledge
  - Introduction to robotics (helpful but not required)
learning_objectives:
  - Define physical AI and distinguish it from disembodied AI systems
  - Understand the principles of embodied cognition and their implications for AI
  - Trace the historical development of physical AI from early robots to modern humanoids
  - Explain Moravec's paradox and its significance for robot design
  - Identify and analyze the key challenges of embodiment in AI systems
tags:
  - physical-ai
  - embodied-cognition
  - robotics-history
  - moravec-paradox
  - humanoid-robotics
code_examples: []
---

# Chapter 1: Introduction to Physical AI

**Learning Objectives:**
- Define physical AI and distinguish it from disembodied AI systems
- Understand the principles of embodied cognition and their implications for artificial intelligence
- Trace the historical development of physical AI from early robots to modern humanoids
- Explain Moravec's paradox and its significance for contemporary robot design
- Identify and analyze the key challenges of embodiment in AI systems

**Prerequisites:** Basic AI/ML knowledge, familiarity with fundamental computer science concepts
**Estimated Reading Time:** 4 hours
**Difficulty:** Beginner

## Introduction

The field of artificial intelligence has traditionally focused on disembodied cognitive tasks—playing chess, proving theorems, recognizing patterns in data, or generating human-like text. These achievements, while remarkable, represent only one dimension of intelligence. A chess-playing algorithm that can defeat world champions struggles to pick up a single chess piece without dropping it. A language model that can discuss quantum mechanics in eloquent prose cannot navigate a cluttered room or grasp a door handle. This disparity reveals a fundamental truth: intelligence, as it evolved in biological systems, is intrinsically tied to physical interaction with the world (Brooks, 1991; Pfeifer & Bongard, 2006).

Physical AI—artificial intelligence systems embodied in physical agents that sense and act upon their environment—represents a paradigm shift from purely computational intelligence to intelligence grounded in sensorimotor experience. These systems must not only process information but also contend with the complexities of the physical world: uncertain sensor data, dynamic environments, real-time constraints, and the consequences of physical actions. As robotics and AI converge, we are witnessing the emergence of increasingly sophisticated physical AI systems, from warehouse robots that navigate complex logistics centers to humanoid robots capable of parkour and manipulation tasks that rival human performance (Kuindersma et al., 2016).

This chapter introduces the foundational concepts of physical AI with a specific focus on humanoid robotics—systems designed to approximate human morphology and capabilities. We begin by exploring embodied cognition and how it reframes our understanding of intelligence. We then distinguish physical AI from disembodied AI, examining what changes when intelligence must operate through physical embodiment. A historical survey traces the evolution from early robotic systems like SHAKEY to modern humanoids such as Boston Dynamics' Atlas and Tesla's Optimus. We examine Moravec's paradox—the observation that high-level reasoning is computationally easier than low-level sensorimotor skills—and its profound implications for robot design. Finally, we identify the key technical challenges that arise from embodiment: perception under uncertainty, real-time control, physical safety, and the integration of learning with physical interaction.

## 1.1 Embodied Cognition and Physical AI

### 1.1.1 The Embodied Cognition Hypothesis

Traditional cognitive science, influenced heavily by the computational theory of mind, viewed intelligence as symbol manipulation—abstract, disembodied computation that could in principle occur independently of any physical substrate (Brooks, 1991). Under this view, a mind is fundamentally a program, and bodies are merely input-output devices for interacting with the environment. This perspective led to early AI research focusing almost exclusively on symbolic reasoning, logical inference, and knowledge representation.

The embodied cognition hypothesis challenges this view fundamentally. It posits that cognitive processes are deeply rooted in the body's interactions with the world—that perception, action, and cognition are inseparably intertwined (Pfeifer & Bongard, 2006). According to this perspective, intelligence cannot be fully understood or replicated without considering the physical body, its sensorimotor capabilities, and its dynamic interaction with the environment. The body is not merely a vessel for the mind but an integral component of cognitive processing itself.

Several key principles characterize embodied cognition:

**Morphological Computation**: The physical structure and dynamics of a body can perform computational functions, offloading processing from the brain or central controller. For example, the passive dynamics of a robot's legs during walking can provide stability without explicit computational control, reducing the complexity of gait control algorithms (Pfeifer & Bongard, 2006). The mechanical properties of tendons, joints, and limbs contribute to control in ways that would require extensive computation if implemented purely algorithmically.

**Sensorimotor Coupling**: Cognition emerges from tight, real-time coupling between sensory perception and motor action. An agent does not first perceive the world completely, then plan, then act; instead, perception and action occur in continuous, coupled loops. A robot navigating a corridor, for instance, continuously adjusts its trajectory based on visual feedback rather than pre-computing a complete path (Brooks, 1991).

**Situatedness**: Intelligent behavior is fundamentally situated in specific environments and contexts. Rather than abstract reasoning over world models, embodied intelligence involves direct, context-specific responses to environmental stimuli. A legged robot traversing rough terrain must respond to local terrain features in real-time rather than planning every footstep from a global map.

**Environmental Scaffolding**: The environment itself can serve as an extension of cognitive processing. Animals and robots alike use environmental structure to reduce computational demands—using landmarks for navigation, using surfaces for support, exploiting gravity for manipulation. The environment is not merely a backdrop for cognition but an active participant in it.

These principles have profound implications for how we design and understand physical AI systems. Rather than attempting to replicate human-level abstract reasoning first and then "add" a body, embodied AI approaches suggest that intelligence must be built from the ground up through sensorimotor interaction.

### 1.1.2 Defining Physical AI

Physical AI, also referred to as embodied AI or robotic AI, encompasses artificial intelligence systems that possess physical bodies enabling them to sense and act in the physical world. More formally, we can define physical AI as:

> **Physical AI**: An autonomous or semi-autonomous agent comprising (1) a physical body with sensors for perceiving environmental states, (2) actuators for executing physical actions, (3) computational systems for processing sensory information and generating control commands, and (4) algorithms enabling the agent to achieve goals through physical interaction with its environment.

This definition encompasses several key elements:

**Physical Embodiment**: The system has a tangible body subject to physical laws—gravity, friction, inertia, and material constraints. This embodiment introduces challenges absent in purely computational systems: mechanical wear, energy limitations, mass and inertia effects, and physical safety constraints.

**Perception**: The system acquires information about its environment and internal state through sensors—cameras, force sensors, proprioceptive sensors, inertial measurement units (IMUs), and others. Unlike simulated agents with perfect access to world state, physical AI systems must deal with noisy, incomplete, and sometimes contradictory sensor data.

**Action**: The system affects its environment through actuators—motors, hydraulic systems, pneumatic actuators, or other mechanisms that convert control signals into physical motion. Actions have consequences that cannot always be predicted perfectly: objects slip, surfaces are uneven, and disturbances occur.

**Autonomy**: The system makes decisions and executes actions without continuous human intervention, though the degree of autonomy varies widely. Full autonomy represents a long-term goal; many current physical AI systems operate with varying levels of human supervision or teleoperation.

**Goal-Directedness**: The system pursues objectives, whether explicitly programmed, learned through experience, or specified through natural language commands. Goals may range from simple (navigate to a location) to complex (prepare a meal in an unfamiliar kitchen).

Physical AI systems span a wide spectrum of embodiments and capabilities. Industrial robot arms performing repetitive assembly tasks represent one end of this spectrum—highly specialized systems operating in structured environments. Autonomous vehicles navigating public roads represent intermediate complexity, dealing with dynamic environments but constrained to two-dimensional motion on relatively predictable surfaces. Humanoid robots represent the high-complexity end: general-purpose systems designed to operate in human environments, manipulate human tools, and potentially collaborate with humans across diverse tasks.

### 1.1.3 The Importance of Physical Interaction

Why build physical AI systems when simulation environments can train agents far more quickly and safely? The answer lies in several irreducible aspects of physical interaction:

**Reality Gap**: Simulations, no matter how sophisticated, cannot perfectly model physical reality. Subtle effects—friction variations, material deformation, sensor noise characteristics, actuator dynamics—differ between simulation and reality. Agents trained purely in simulation often fail when deployed on real robots, a challenge known as the "sim-to-real" gap (Tobin et al., 2017). While techniques like domain randomization can narrow this gap, complete closure remains elusive for complex tasks.

**Physical Common Sense**: Humans develop intuitions about physical causality through embodied experience—understanding that unsupported objects fall, that liquids spill, that fragile objects break when dropped. These intuitions, trivial for humans, are extraordinarily difficult to encode explicitly or learn from static data. Physical interaction provides the grounding necessary for developing robust common-sense physical reasoning.

**Social and Economic Value**: Many valuable tasks require physical interaction with the world—manufacturing, logistics, construction, healthcare, domestic assistance, and emergency response. The economic incentive to automate physical labor drives substantial investment in physical AI. Beyond economic value, physical AI systems can perform tasks too dangerous for humans (disaster response, hazardous material handling) or extend human capabilities (surgery assistance, elder care).

**Scientific Understanding**: Building physical AI systems advances our understanding of intelligence itself. Neuroscience and cognitive science benefit from computational models that must contend with real-world physical constraints. Conversely, understanding biological intelligence—how animals solve locomotion, manipulation, and navigation problems—informs robotics (Pfeifer & Bongard, 2006).

The progression from disembodied AI to physical AI thus represents not merely an engineering challenge but a fundamental expansion of what we mean by artificial intelligence.

## 1.2 Physical AI versus Disembodied AI

### 1.2.1 Contrasting Paradigms

Disembodied AI systems—those operating purely in computational or virtual environments—and physical AI systems differ along several critical dimensions. Understanding these differences illuminates the unique challenges of physical embodiment.

**State Representation and Uncertainty**: Disembodied AI systems often operate with complete or near-complete state information. A chess engine has perfect knowledge of the board state; a theorem prover has complete access to axioms and current proof steps. Physical AI systems, conversely, must infer environmental state from noisy, partial sensor observations. A robot navigating a building doesn't know precisely where it is or what objects surround it—it must estimate these from camera images, lidar scans, or other sensors, each with characteristic noise and failure modes (Thrun et al., 2005).

**Action Consequences and Reversibility**: In many disembodied domains, actions are easily reversible or consequence-free. A chess player can analyze millions of hypothetical moves without moving a physical piece. An AI exploring a decision tree can backtrack costlessly. Physical actions, however, have irreversible consequences. A robot that drops a glass cannot undo that action. Energy expended in motion cannot be recovered fully. This irreversibility demands more cautious, robust decision-making (Deisenroth et al., 2013).

**Real-Time Constraints**: Disembodied systems often operate without hard real-time constraints. A language model can take minutes to generate a response; a game-playing AI can deliberate for extended periods. Physical AI systems frequently face hard real-time deadlines. A humanoid robot maintaining balance must compute motor commands at hundreds of Hertz; delays of even milliseconds can result in falls. Perception, planning, and control must all execute within stringent time budgets (Siciliano & Khatib, 2016).

**Safety and Risk**: Errors in disembodied AI systems typically have limited consequences—a wrong answer, a poor game move, or an inappropriate text generation. Physical AI system failures can cause physical harm—colliding with humans, damaging property, or harming the robot itself. Safety becomes a first-order design constraint, requiring redundant systems, conservative behavior, and extensive validation (Haddadin et al., 2017).

**Dimensionality and Continuous State-Action Spaces**: Many disembodied AI problems have discrete state and action spaces (chess has ~10^43 possible board states, large but discrete). Physical AI systems inhabit continuous, high-dimensional state and action spaces. A humanoid robot might have 30+ degrees of freedom, each represented by continuous angles and velocities, yielding state spaces with 60+ dimensions. Action spaces are similarly continuous and high-dimensional. This continuity complicates planning, learning, and control (LaValle, 2006).

### 1.2.2 Task Complexity Inversions: Moravec's Paradox Preview

One of the most striking contrasts between disembodied and physical AI lies in what tasks prove difficult. Disembodied AI has achieved superhuman performance in chess, Go, mathematical reasoning, and language generation—tasks that represent the pinnacle of human cognitive achievement. Yet even the most advanced robots struggle with tasks that human toddlers perform effortlessly: walking across uneven ground, picking up unfamiliar objects, or recognizing a face in varied lighting conditions.

This inversion—that what's computationally hard for disembodied AI is often easy for physical AI, and vice versa—is known as Moravec's paradox, which we will explore in detail in Section 1.4. For now, we note that this paradox highlights a fundamental difference in problem character: Abstract reasoning often reduces to search and pattern-matching in well-defined spaces, amenable to computational brute force. Physical skills require navigating continuous, high-dimensional spaces under uncertainty and real-time constraints—a very different challenge.

### 1.2.3 Hybrid Approaches: Combining Physical and Disembodied AI

Contemporary AI systems increasingly blur the boundary between physical and disembodied AI through hybrid architectures. Consider a household service robot tasked with preparing a meal:

- **High-level planning** (disembodied): The robot uses language understanding to interpret the recipe and abstract reasoning to determine the sequence of steps—tasks better suited to disembodied AI approaches.
- **Perception and manipulation** (physical): The robot uses computer vision to locate ingredients, force sensing to grasp objects appropriately, and motor control to manipulate utensils—intrinsically physical tasks.
- **Learning from simulation and reality** (hybrid): The robot may train manipulation policies initially in simulation (disembodied), then fine-tune them through physical interaction (embodied), bridging the sim-to-real gap (Tobin et al., 2017).

These hybrid systems leverage the strengths of both paradigms: disembodied AI for abstract reasoning, planning, and rapid simulated learning; physical AI for sensorimotor skills, real-world validation, and common-sense physical understanding. Understanding both paradigms and their integration is essential for building capable, general-purpose robots.

## 1.3 Historical Development of Physical AI

### 1.3.1 Early Mobile Robots: SHAKEY and Its Contemporaries (1960s-1980s)

The history of physical AI begins in earnest in the 1960s with the development of mobile robots capable of autonomous reasoning and action. The most iconic of these early systems was SHAKEY, developed at the Stanford Research Institute (now SRI International) from 1966 to 1972 (Siciliano & Khatib, 2016).

**SHAKEY the Robot**: SHAKEY was a mobile robot equipped with a television camera, range finder, and bump sensors, connected via radio to a large mainframe computer (a DEC PDP-10 and PDP-15). Despite its mobility, SHAKEY's "brain" remained off-board due to the computational demands of its AI systems. SHAKEY operated in carefully constructed environments—rooms with geometric blocks, ramps, and clearly marked boundaries.

SHAKEY's significance lay not in its physical capabilities, which were modest, but in its integration of multiple AI subsystems:

- **Vision**: Image processing to identify objects and spatial relationships
- **Planning**: The STRIPS planner, which could decompose high-level goals into sequences of primitive actions
- **Navigation**: Path planning algorithms for obstacle avoidance
- **Reasoning**: Logical inference over a symbolic world model

SHAKEY demonstrated that an artificial agent could perceive its environment, reason about actions, plan sequences of operations, and execute them to achieve goals—the fundamental loop of autonomous robotics. However, SHAKEY also revealed profound limitations:

- **Computational Requirements**: Even simple tasks required minutes of computation on powerful mainframes.
- **Fragile Perception**: Vision systems worked only in controlled environments with good lighting and simple objects.
- **Brittle Planning**: The symbolic planner assumed perfect world knowledge and could not handle uncertainty or unexpected disturbances.
- **Limited Robustness**: Small deviations from expected conditions could cause complete failure.

**Contemporary Systems**: Other notable early mobile robots included the Stanford Cart (1960s-1970s), which pioneered stereo vision for obstacle avoidance, and mobile robots developed at MIT's AI Lab, which explored reactive control architectures as alternatives to the sense-plan-act paradigm exemplified by SHAKEY.

### 1.3.2 The Reactive Revolution: Brooks and Behavior-Based Robotics (1980s-1990s)

By the mid-1980s, the limitations of SHAKEY-style "sense-plan-act" robotics were evident. Systems were slow, fragile, and worked only in highly controlled environments. Rodney Brooks, then at MIT, proposed a radical alternative: reactive, behavior-based robotics (Brooks, 1991).

**Subsumption Architecture**: Brooks introduced the subsumption architecture, which decomposed robot control into layers of simple behaviors that operated in parallel, directly connecting sensors to actuators without centralized planning or world models. Higher layers could "subsume" (override) lower layers when appropriate. For example, a mobile robot might have:

- **Layer 0**: Wander randomly, avoiding immediate obstacles
- **Layer 1**: Head toward distant goals
- **Layer 2**: Recognize and approach specific objects

Each layer operated independently with its own sensorimotor loop. There was no central world model, no explicit planning—just collections of simple behaviors that, in aggregate, produced intelligent-seeming behavior.

**"Intelligence Without Representation"**: Brooks famously argued that intelligence doesn't require explicit internal representations of the world. Instead, "the world is its own best model"—robots should react directly to sensory stimuli rather than building abstract models (Brooks, 1991). This perspective, heavily influenced by embodied cognition, suggested that sensorimotor coupling and reactive behaviors were more fundamental to intelligence than abstract reasoning.

Brooks' robots, including Genghis (a hexapod robot) and later planetary rovers, demonstrated impressive robustness compared to earlier systems. They navigated cluttered, unstructured environments and gracefully handled sensor noise and unexpected obstacles. The subsumption architecture influenced a generation of roboticists and contributed to the embodied AI perspective discussed in Section 1.1.

**Limitations of Pure Reactivity**: However, purely reactive approaches also had limitations. They struggled with tasks requiring long-term planning, abstract reasoning, or learning complex skills. A purely reactive robot could navigate a cluttered room but couldn't assemble furniture or engage in multi-step manipulation. The field eventually recognized that both reactive and deliberative capabilities were necessary, leading to hybrid architectures combining fast reactive behaviors with slower deliberative planning.

### 1.3.3 Humanoid Robotics Emerges (1990s-2000s)

While early robotics focused primarily on mobile platforms or industrial manipulators, the 1990s saw growing interest in humanoid robots—systems designed to approximate human morphology and capabilities.

**Why Humanoid Robots?** Several motivations drove humanoid robotics research:

1. **Human-Centered Environments**: Human buildings, tools, and infrastructure are designed for human body proportions and capabilities. A humanoid robot could potentially use existing tools and navigate existing spaces without environmental modification.

2. **Social Interaction**: Human-like form facilitates natural human-robot interaction. Humans intuitively understand humanoid gestures, expressions, and movements.

3. **Scientific Understanding**: Building humanoid robots serves as a test of our understanding of human biomechanics, cognition, and sensorimotor control.

4. **Versatility**: The human body is remarkably versatile, capable of locomotion across diverse terrains, manipulation of countless objects, and a vast repertoire of motor skills. A robot approaching human capabilities could be similarly versatile.

**Early Humanoids**: Notable early humanoid robots included:

- **Honda P-Series and ASIMO** (1993-2000): Honda's humanoid program, beginning with the P1 prototype in 1993, culminated in ASIMO (Advanced Step in Innovative Mobility), unveiled in 2000. ASIMO demonstrated stable biped walking, stair climbing, and basic manipulation, representing a major engineering achievement (Siciliano & Khatib, 2016).

- **Sony QRIO** (2003): Sony developed QRIO, a small humanoid capable of running, dancing, and social interaction. QRIO emphasized fluid, dynamic motion and human-robot interaction rather than task performance.

- **Waseda University WL-Series** (1973-present): Waseda University in Japan pioneered humanoid research with the WABOT series starting in 1973, developing successive generations exploring biped locomotion, manipulation, and human interaction.

These early humanoids demonstrated the feasibility of stable biped locomotion and basic manipulation but were far from general-purpose capabilities. Walking was slow and fragile; manipulation was limited to carefully staged demonstrations. Energy efficiency was poor, and robustness to disturbances was minimal.

**Perception and Control Challenges**: Early humanoids revealed profound challenges in perception and control. Biped balance requires real-time feedback and rapid control adjustments. Manipulation requires coordinating many degrees of freedom while processing uncertain visual and tactile feedback. These challenges drove advances in control theory, sensor fusion, and real-time computing (Siciliano & Khatib, 2016).

### 1.3.4 Modern Humanoids and the Age of Learning (2010s-Present)

The 2010s and 2020s have witnessed dramatic advances in humanoid robotics driven by several converging trends: advances in machine learning (especially deep learning and reinforcement learning), improved sensors and actuators, increased computational power, and substantial industrial investment.

**Boston Dynamics Atlas**: Perhaps the most iconic modern humanoid, Boston Dynamics' Atlas robot (introduced in 2013 for the DARPA Robotics Challenge) represents the state of the art in dynamic locomotion and whole-body control. Atlas has demonstrated capabilities that seemed futuristic only years ago:

- **Dynamic Locomotion**: Walking, running, and jumping across rough terrain, including backflips and parkour sequences
- **Robust Balance**: Recovering from pushes, slips, and other disturbances
- **Whole-Body Manipulation**: Lifting and moving heavy objects, using tools
- **Perception-Based Navigation**: Navigating complex, unstructured environments using vision and lidar

Atlas's capabilities rest on sophisticated optimization-based control algorithms that compute motor commands at high rates (100+ Hz) by solving constrained optimization problems in real-time (Kuindersma et al., 2016). These controllers plan footsteps, body trajectories, and joint motions simultaneously while respecting physical constraints (friction limits, joint limits, balance constraints). We will explore Atlas in detail in Chapter 9.

**Tesla Optimus**: Tesla's entry into humanoid robotics, Optimus (unveiled in 2022), represents a different approach: end-to-end learning from human demonstrations. Rather than hand-engineering control algorithms, Tesla aims to learn manipulation and locomotion skills from large datasets of human teleoperation, similar to how Tesla trains autonomous driving systems from fleet data. Optimus emphasizes manufacturability and cost reduction, targeting eventual mass production for industrial and domestic applications. We will examine Optimus in detail in Chapter 10.

**Research Platforms**: Academic and industry research labs use sophisticated humanoid platforms including:

- **PAL Robotics TALOS**: A torque-controlled humanoid designed for research in locomotion, manipulation, and human-robot collaboration (Stasse et al., 2017)
- **Italian Institute of Technology iCub**: A child-sized humanoid emphasizing cognitive development, learning, and human interaction (Metta et al., 2010)
- **NASA Valkyrie**: A humanoid designed for operations in hazardous environments

These platforms enable researchers worldwide to explore algorithms for perception, control, learning, and human-robot interaction without building custom hardware. We will discuss these platforms in Chapter 11.

**Machine Learning Integration**: Modern humanoids increasingly leverage machine learning, particularly deep reinforcement learning, to acquire complex skills:

- **End-to-End Visuomotor Policies**: Training policies that map directly from camera images to motor commands, bypassing hand-engineered perception and planning modules (Levine et al., 2016)
- **Imitation Learning**: Learning from human demonstrations via teleoperation or motion capture
- **Sim-to-Real Transfer**: Training policies in simulation with domain randomization, then deploying them on real robots (Tobin et al., 2017)

These learning approaches complement traditional control methods, with hybrid systems using learning for perception and high-level skills while retaining model-based control for critical low-level functions like balance.

### 1.3.5 The Current State and Future Trajectory

As of 2025, physical AI—and humanoid robotics specifically—stands at an inflection point. We have robots capable of impressive demonstrations: dynamic parkour, robust locomotion, dexterous manipulation in structured settings. Yet we lack robots capable of open-ended, general-purpose operation in human environments. Several factors will shape the next decade:

**Scaling Laws**: Will data-driven learning scale to general-purpose physical AI as it has for language models? Can we collect and leverage massive datasets of physical interaction?

**Hardware Advances**: Improved actuators (compact, powerful, energy-efficient), better sensors (robust vision, tactile sensing), and edge computing may remove current hardware bottlenecks.

**Integration Challenges**: Combining perception, planning, control, and learning into robust, safe, general-purpose systems remains a profound systems engineering challenge.

**Economic Drivers**: Labor shortages, dangerous work environments, and aging populations create strong economic incentives for capable physical AI systems, driving investment and research.

Understanding this historical progression—from the symbolic reasoning of SHAKEY through the reactive revolution to modern learning-based systems—provides essential context for the technical material in subsequent chapters.

## 1.4 Moravec's Paradox and Implications for Robot Design

### 1.4.1 Formulating the Paradox

In the 1980s, Hans Moravec, a roboticist at Carnegie Mellon University, observed a striking pattern in AI and robotics research:

> "It is comparatively easy to make computers exhibit adult-level performance on intelligence tests or playing checkers, and difficult or impossible to give them the skills of a one-year-old when it comes to perception and mobility." (Moravec, 1988, cited in Siciliano & Khatib, 2016)

This observation, now known as Moravec's paradox, captures a fundamental inversion in computational difficulty: Tasks that humans find cognitively demanding—abstract reasoning, complex calculations, logical deduction, game-playing—prove relatively tractable for AI systems. Conversely, tasks that humans (and animals) perform effortlessly and unconsciously—walking, grasping objects, recognizing faces, navigating cluttered environments—prove extraordinarily difficult for AI and robotics.

**Counterintuitive Difficulty**: Consider these contrasts:

- Deep Blue defeated world chess champion Garry Kasparov in 1997, yet no robot in 1997 could reliably set up a chessboard.
- AlphaGo defeated world Go champion Lee Sedol in 2016, yet no robot could match a five-year-old child in stacking blocks.
- GPT-4 can write coherent essays on quantum mechanics, yet struggles to reason about physical causality (e.g., what happens if you stack a book on top of an egg).

### 1.4.2 Evolutionary Explanations

Why are sensorimotor skills so much harder than abstract reasoning? Moravec's explanation appeals to evolutionary timescales and computational complexity.

**Evolutionary Perspective**: Abstract reasoning—the kind tested by IQ tests and exemplified by mathematics, logic, and language—is evolutionarily recent. Written language emerged only ~5,000 years ago; formal mathematics even more recently. Natural selection has had minimal time to optimize human brains for these tasks.

In contrast, sensorimotor skills—vision, locomotion, manipulation, spatial navigation—have been under intense evolutionary pressure for hundreds of millions of years. Every mobile animal must solve these problems to survive. Consequently, biological brains have evolved extraordinarily sophisticated, specialized circuits for perception and motor control (Pfeifer & Bongard, 2006).

**Computational Complexity**: Moravec argued that sensorimotor skills appear easy to us precisely because they rely on massively parallel, unconscious processing by specialized neural circuits refined over evolutionary time. Abstract reasoning, being more recent, relies on slow, serial, conscious processing. The computational work required for sensorimotor skills is enormous—far exceeding that required for chess or theorem-proving—but it's performed unconsciously by dedicated neural hardware, making it feel effortless.

Consider visual object recognition: The human visual cortex contains billions of neurons forming hierarchical processing stages that extract edges, textures, shapes, and object categories from raw retinal input at millisecond timescales. Replicating this with conventional algorithms stumped AI researchers for decades. Only with deep convolutional neural networks, which mimic the hierarchical structure of the visual cortex, did performance approach human levels—and these networks require millions of training examples and substantial computation (Levine et al., 2016).

### 1.4.3 Implications for Robot Design and Development Priorities

Moravec's paradox has profound implications for how we approach physical AI and humanoid robotics:

**Development Priorities**: The paradox suggests that enabling robots to perform "simple" physical tasks—walking reliably, manipulating diverse objects, perceiving cluttered environments—should be a primary focus, potentially more so than high-level reasoning. A robot capable of robust, general-purpose physical interaction can be given reasoning capabilities; a robot that cannot navigate or manipulate is useless regardless of its reasoning ability.

**Respect for "Low-Level" Skills**: The paradox elevates the status of perception, motor control, and sensorimotor integration from mere implementation details to central scientific and engineering challenges. These are not simple input-output functions to be dispatched quickly; they are core problems deserving sustained research attention.

**Learning and Data Requirements**: If sensorimotor skills are computationally intensive, we should expect that learning these skills requires substantial data and computation—much as visual recognition required massive labeled image datasets (ImageNet) and deep networks. This insight motivates efforts to collect large-scale robot interaction datasets and train models via simulation (Levine et al., 2016; Tobin et al., 2017).

**Specialization and Morphological Computation**: The paradox reinforces the importance of specialized hardware and morphological computation. Just as biological systems use dedicated neural circuits and exploit biomechanics, robots can benefit from specialized actuators, sensors, and mechanical designs that simplify control problems. For instance, compliant actuators (mimicking muscle elasticity) can provide passive stability, reducing computational control demands (Pfeifer & Bongard, 2006).

**Hybrid Architectures**: The paradox supports hybrid architectures combining learned sensorimotor skills with symbolic reasoning. Reactive, learned controllers handle perception and low-level motor control—the "hard" problems. Higher-level planning and reasoning, being "easier" computationally, can use more conventional AI methods.

**Humility and Long-Term Perspective**: Finally, Moravec's paradox counsels humility. Tasks that appear simple often hide profound complexity. General-purpose humanoid robotics remains a long-term challenge precisely because it requires solving many Moravec-hard problems simultaneously: robust vision, dexterous manipulation, dynamic locomotion, and their integration. Achieving human-level performance in these domains may require decades of research, specialized hardware, and massive datasets.

## 1.5 Key Challenges of Embodiment

Having established what physical AI is, how it differs from disembodied AI, its historical development, and the paradoxical difficulty of sensorimotor skills, we now survey the key technical challenges that embodiment introduces. These challenges will be explored in depth in subsequent chapters; here we provide an overview.

### 1.5.1 Perception Under Uncertainty

Physical AI systems must infer environmental state from sensor measurements that are inevitably noisy, incomplete, and sometimes contradictory.

**Sensor Noise and Failure**: All physical sensors introduce noise. Cameras capture blur, motion artifacts, lighting variations, and occlusions. Lidar measurements are corrupted by reflections and material properties. Tactile sensors drift and saturate. Perception algorithms must be robust to these imperfections (Cadena et al., 2016).

**Partial Observability**: Unlike game-playing AIs with complete state knowledge, robots observe only a tiny fraction of the world at any moment. A robot in a building cannot see through walls, doesn't know the location of objects in closed drawers, and has limited fields of view. Perception requires integrating observations over time to build coherent estimates of world state—a process fraught with uncertainty (Thrun et al., 2005).

**Dynamic Environments**: Environments change while robots act. People move, doors open and close, lighting varies, objects are displaced. Perception systems must track these changes, distinguish stable from transient features, and update beliefs accordingly.

**Multi-Modal Integration**: Robots typically have many sensors—cameras, depth sensors, force sensors, proprioceptive sensors (joint encoders, IMUs). Integrating these heterogeneous data sources into coherent state estimates requires sophisticated sensor fusion techniques, such as Kalman filtering or particle filters (Thrun et al., 2005). We will explore perception and sensor fusion in Chapter 4.

### 1.5.2 Real-Time Control and Stability

Physical systems have continuous dynamics governed by physics. Controlling these systems requires generating appropriate motor commands at high rates under hard real-time constraints.

**Feedback Control**: Unlike open-loop systems that execute pre-planned commands, physical robots require continuous feedback control—measuring state, comparing to desired state, computing corrective actions. For humanoid robots, balance control operates at hundreds of Hertz; slower control rates lead to instability and falls (Siciliano & Khatib, 2016).

**High-Dimensional Control**: A humanoid robot may have 30+ degrees of freedom (joints). Controlling all these joints simultaneously to achieve whole-body motions—walking, reaching, manipulating—requires solving high-dimensional optimization problems in real-time. This is computationally intensive and requires efficient algorithms (Kuindersma et al., 2016).

**Stability and Safety**: Physical systems can become unstable—a robot can fall, collide, or damage itself or its environment. Control algorithms must guarantee stability under disturbances and ensure safe operation even when perception or planning fails. Safety constraints (avoiding collisions, respecting joint limits, maintaining balance) must be enforced continuously (Haddadin et al., 2017).

**Model Uncertainty**: Control often relies on models of robot dynamics—how forces and torques translate to motion. But models are never perfect; parameters like mass, friction, and actuator response are uncertain and vary. Robust control must account for model uncertainty. We will explore control architectures in Chapter 6.

### 1.5.3 Planning in Continuous, High-Dimensional Spaces

Physical robots inhabit continuous configuration spaces. Planning motions or action sequences in these spaces is computationally challenging.

**Configuration Space Complexity**: A humanoid robot's configuration space has 30+ dimensions (one per joint). A 6-DOF manipulator has a 6D configuration space. Planning paths in such high-dimensional spaces using naive grid-based methods is intractable due to the curse of dimensionality (LaValle, 2006).

**Kinodynamic Constraints**: Plans must respect not only geometric constraints (collision avoidance) but also kinodynamic constraints—velocity limits, acceleration limits, balance constraints. A humanoid cannot teleport between configurations; it must maintain balance throughout a motion. These constraints couple spatial and temporal planning (LaValle & Kuffner, 2001).

**Replanning Under Uncertainty**: Environments are dynamic and partially observable. Initial plans often become invalid as new information arrives or disturbances occur. Robots need replanning capabilities—updating plans efficiently in real-time as conditions change.

**Sampling-Based Planning**: Modern motion planning algorithms use sampling-based methods (Rapidly-exploring Random Trees, Probabilistic Roadmaps) that efficiently explore high-dimensional spaces by randomly sampling configurations and building graphs or trees. These methods trade completeness for computational tractability (Karaman & Frazzoli, 2011). We will explore motion planning in Chapter 5.

### 1.5.4 Learning from Physical Interaction

Machine learning has revolutionized many AI domains, but learning in physical systems introduces unique challenges.

**Sample Efficiency**: Physical robots cannot train for millions of episodes as simulated agents can. Collecting data on real robots is slow, expensive, and risky. Learning algorithms for robotics must be far more sample-efficient than those for game-playing or language modeling (Deisenroth et al., 2013).

**Safety During Learning**: Exploration is essential for learning, but robots exploring freely can damage themselves, their environment, or harm humans. Safe exploration—learning while respecting safety constraints—is a critical challenge (Haddadin et al., 2017).

**Credit Assignment**: When a robot executes a long sequence of actions and eventually succeeds or fails at a task, determining which actions were responsible (the credit assignment problem) is difficult. Sparse rewards—feedback only at task completion—make credit assignment particularly challenging in robotics.

**Sim-to-Real Transfer**: To overcome sample efficiency limitations, robots often train in simulation. But simulation differs from reality (the reality gap). Transferring policies learned in simulation to real robots requires techniques like domain randomization, which trains policies robust to simulation-reality discrepancies (Tobin et al., 2017).

**Integration with Control**: Learned policies must integrate with low-level controllers. Often, learning occurs at high levels (what to do) while control handles low levels (how to do it). Designing interfaces between learned and engineered components is a systems challenge. We will explore machine learning for robotics in Chapter 7.

### 1.5.5 Human-Robot Interaction and Safety

Humanoid robots designed to operate in human environments must interact safely and intuitively with people.

**Physical Safety**: Robots are heavy, powerful machines. Collisions can cause injury. Robots must detect imminent collisions, reduce impact forces, and stop safely when contact occurs. This requires advanced sensors (e.g., whole-body tactile sensing) and collision-aware control (Haddadin et al., 2017).

**Intent Recognition and Prediction**: To collaborate effectively, robots must infer human intent from observations—predicting what a person will do next to plan complementary actions. This requires modeling human behavior, which is complex and variable.

**Natural Interfaces**: Humans should be able to instruct and correct robots using natural communication—language, gestures, demonstrations. Developing robust natural language understanding and gesture recognition for human-robot interaction is an active research area.

**Social Acceptability**: Beyond functional interaction, robots in human spaces must behave in socially acceptable ways—respecting personal space, moving predictably, exhibiting appropriate nonverbal cues. Social robotics studies these factors (Hoffman & Breazeal, 2007). We will explore human-robot interaction in Chapter 8.

### 1.5.6 Energy and Autonomy

Unlike simulated agents or tethered systems, autonomous robots have limited on-board energy.

**Power Constraints**: Humanoid robots consume substantial power—hundreds of watts or more during dynamic motion. Battery capacity limits operation time, often to 1-2 hours for current systems. Energy-efficient actuation, computation, and operation strategies are essential for practical autonomy.

**Recharging and Self-Maintenance**: For long-term autonomy, robots must locate and use charging stations autonomously. More ambitiously, they might eventually perform self-maintenance—detecting worn components and seeking repairs.

**Computational Resources**: On-board computing is limited by power, weight, and heat dissipation. Complex perception and control algorithms must run on embedded computers with limited CPU, GPU, and memory. Offloading computation to remote servers introduces latency, which is problematic for real-time control. Balancing computational capability with energy constraints is a persistent challenge (Siciliano & Khatib, 2016).

### 1.5.7 Integration and Systems Engineering

Perhaps the most profound challenge is integration—combining perception, planning, control, learning, and human interaction into a cohesive, reliable system.

**Modularity vs. End-to-End Learning**: Traditional robotics uses modular architectures—separate perception, planning, and control modules. This modularity simplifies development and debugging but can introduce information bottlenecks and compounding errors. End-to-end learning, conversely, trains unified policies but sacrifices interpretability and is data-intensive (Levine et al., 2016).

**Fault Tolerance and Graceful Degradation**: Physical robots must handle component failures gracefully. If a sensor fails, can the robot continue with reduced capabilities? If a joint seizes, can it adapt? Building fault-tolerant systems requires redundancy, monitoring, and adaptive strategies.

**Real-World Deployment**: Transitioning from laboratory demonstrations to real-world deployment introduces unforeseen challenges—lighting variation, unexpected obstacles, human behavior, communication failures. Robust systems engineering, extensive testing, and iterative refinement are essential.

These challenges are deeply interrelated. Progress in perception enables better planning; advances in learning improve control; safe human interaction requires robust perception and predictive control. The following chapters explore each of these areas in depth, providing the theoretical foundations, algorithmic techniques, and practical considerations necessary to address these challenges.

## 1.6 Case Study: From SHAKEY to Optimus—A Comparative Perspective

To ground the concepts introduced in this chapter, we briefly compare SHAKEY (1966-1972) with Tesla Optimus (2022-present), illustrating how the field has evolved over five decades.

**SHAKEY (1966-1972)**:
- **Computation**: Off-board mainframe computers (DEC PDP-10, PDP-15); tasks required minutes of computation
- **Perception**: Monochrome camera, range finder; simple geometric environments only
- **Mobility**: Wheeled platform, slow and deliberate movement
- **Control Paradigm**: Sense-plan-act; symbolic reasoning over explicit world models
- **Capabilities**: Navigate simple rooms, push blocks, plan action sequences
- **Limitations**: Slow, brittle, required carefully engineered environments, no learning

**Tesla Optimus (2022-present)**:
- **Computation**: On-board embedded computers with GPU acceleration; real-time operation
- **Perception**: Multiple cameras, depth sensors, possibly lidar; operates in unstructured environments
- **Mobility**: Biped locomotion with dynamic balance; human-like form factor
- **Control Paradigm**: Hybrid—end-to-end learned visuomotor policies for high-level tasks, model-based control for low-level balance and actuation
- **Capabilities**: Biped walking, manipulation of diverse objects, learning from human demonstrations
- **Limitations**: Still in development; robustness, generalization, and long-term autonomy remain challenges

**Key Differences**: The transformation from SHAKEY to Optimus reflects several fundamental shifts:

1. **From Symbolic to Subsymbolic**: SHAKEY relied on symbolic AI—explicit representations, logical reasoning, search-based planning. Optimus relies heavily on neural networks and learned representations—subsymbolic, data-driven approaches.

2. **From Engineered to Learned**: SHAKEY's behaviors were hand-engineered by expert programmers. Optimus learns many behaviors from data—human demonstrations, simulated experience, trial and error.

3. **From Static to Dynamic**: SHAKEY operated in static, carefully controlled environments. Optimus targets dynamic, unstructured human environments.

4. **From Computation-Limited to Data-Limited**: SHAKEY was limited by computational power; even simple tasks required extensive computation. Optimus has ample computation but is limited by data—collecting diverse, high-quality robot interaction data remains a bottleneck.

5. **From Single-Task to Multi-Task**: SHAKEY performed narrow, predefined tasks. Optimus aims for general-purpose capabilities, learning a repertoire of skills applicable to diverse tasks.

This comparison illustrates the extraordinary progress in physical AI over five decades while also highlighting persistent challenges. Despite advances, truly general-purpose, robust, long-term autonomous humanoid robots remain aspirational.

## Exercises

1. **Embodied Cognition Analysis** (Difficulty: Beginner, Type: Conceptual):

   Consider a household service robot tasked with setting a table for dinner. Describe three specific ways in which embodied cognition principles (morphological computation, sensorimotor coupling, situatedness, or environmental scaffolding) could be applied to this task. For each principle, explain how it would reduce computational demands compared to a purely disembodied planning approach.

2. **Physical AI vs. Disembodied AI** (Difficulty: Intermediate, Type: Conceptual):

   Select a task domain where both physical AI and disembodied AI have achieved success (e.g., game-playing for disembodied AI and locomotion for physical AI). Compare and contrast the two domains along the following dimensions:
   - State representation and observability
   - Action consequences and reversibility
   - Evaluation and success metrics
   - Safety considerations
   - Sample efficiency for learning

   Discuss why methods successful in one domain may not transfer directly to the other.

3. **Moravec's Paradox in Modern AI** (Difficulty: Intermediate, Type: Conceptual):

   Large language models (LLMs) such as GPT-4 can perform impressive abstract reasoning tasks—writing code, explaining scientific concepts, solving logic puzzles. Yet they struggle with physical common sense—reasoning about what happens if you stack a book on an egg, or predicting the trajectory of a thrown ball.

   a) Explain this phenomenon in terms of Moravec's paradox.

   b) Propose and justify an approach to give LLMs better physical common sense. Should it involve embodied interaction, physics simulation, structured knowledge, or some combination?

4. **Historical Development Timeline** (Difficulty: Beginner, Type: Research):

   Create a timeline of major milestones in physical AI and humanoid robotics from 1960 to 2025. For each milestone, briefly note:
   - The system or achievement
   - Key technical innovations
   - Limitations at the time

   Include at least 8-10 milestones spanning the full period. Cite sources for each milestone using APA format.

5. **Challenge Analysis and Prioritization** (Difficulty: Advanced, Type: Conceptual):

   Section 1.5 identified seven key challenges of embodiment: perception under uncertainty, real-time control, planning in high-dimensional spaces, learning from physical interaction, human-robot interaction, energy and autonomy, and integration/systems engineering.

   Consider a specific application domain (e.g., warehouse logistics, elder care, disaster response, or agricultural automation). For your chosen domain:

   a) Rank these seven challenges from most critical to least critical for success in that domain. Justify your ranking.

   b) For the top three challenges in your ranking, propose a concrete technical approach to address each challenge, citing relevant work from the chapter's references.

   c) Discuss potential tradeoffs between your proposed approaches (e.g., between safety and performance, or between autonomy and complexity).

## Summary

This chapter introduced the foundational concepts of physical AI and humanoid robotics, establishing the context and key challenges for the remainder of the book. We explored:

- **Embodied Cognition**: Intelligence is not abstract symbol manipulation but deeply rooted in sensorimotor interaction with the physical world. Principles such as morphological computation, sensorimotor coupling, and situatedness inform how we design physical AI systems (Brooks, 1991; Pfeifer & Bongard, 2006).

- **Physical AI Defined**: Physical AI systems have bodies, sensors, actuators, and must achieve goals through physical interaction. This embodiment introduces challenges absent in disembodied AI—uncertain perception, real-time constraints, irreversible actions, safety considerations, and continuous high-dimensional state-action spaces.

- **Physical vs. Disembodied AI**: The two paradigms differ fundamentally in state observability, action consequences, temporal constraints, safety requirements, and problem dimensionality. Understanding these differences is essential for designing physical AI systems and for appreciating why techniques from disembodied AI cannot be applied naively to robotics.

- **Historical Development**: The field evolved from early symbolic systems like SHAKEY through the reactive revolution led by Brooks to modern learning-based humanoids like Atlas and Optimus. This progression reflects shifts from symbolic to subsymbolic processing, from engineered to learned behaviors, and from narrow to increasingly general-purpose capabilities (Siciliano & Khatib, 2016).

- **Moravec's Paradox**: High-level reasoning is computationally easier than low-level sensorimotor skills—an inversion of intuitive difficulty. This paradox reflects evolutionary timescales and the enormous computational complexity hidden in "simple" perceptual and motor tasks. It has profound implications for development priorities and architectural choices in robotics.

- **Key Challenges**: Embodiment introduces multifaceted challenges—perception under uncertainty, real-time control, high-dimensional planning, sample-efficient learning, safe human interaction, energy constraints, and systems integration. Subsequent chapters address each challenge in depth, providing theoretical foundations and practical techniques.

As we proceed through the book, remember that physical AI is not merely AI "with a body added." Embodiment fundamentally transforms the nature of intelligence, introducing constraints, opportunities, and complexities that shape every aspect of system design. The goal of this book is to equip you with the conceptual foundations, algorithmic techniques, and practical insights needed to build physical AI systems—particularly humanoid robots—capable of robust, safe, and versatile operation in human environments.

## Further Reading

- **Brooks, R. A. (1991)**. Intelligence without representation. *Artificial Intelligence*, 47(1-3), 139-159. — Seminal paper arguing for reactive, behavior-based robotics over symbolic planning.

- **Pfeifer, R., & Bongard, J. C. (2006)**. *How the body shapes the way we think: A new view of intelligence*. MIT Press. — Comprehensive exploration of embodied cognition and its implications for AI and robotics.

- **Siciliano, B., & Khatib, O. (Eds.). (2016)**. *Springer handbook of robotics* (2nd ed.). Springer. — Authoritative, comprehensive reference covering all aspects of robotics; excellent for deep dives into specific topics.

- **Deisenroth, M. P., Neumann, G., & Peters, J. (2013)**. A survey on policy search for robotics. *Foundations and Trends in Robotics*, 2(1-2), 1-142. — Thorough survey of learning approaches for robot control, essential for understanding Chapter 7 material.

- **Kuindersma, S., et al. (2016)**. Optimization-based locomotion planning, estimation, and control design for the Atlas humanoid robot. *Autonomous Robots*, 40(3), 429-455. — Detailed technical description of Atlas's control architecture, preview of Chapter 9 case study.

## References

Brooks, R. A. (1991). Intelligence without representation. *Artificial Intelligence*, 47(1-3), 139-159. https://doi.org/10.1016/0004-3702(91)90053-M

Cadena, C., Carlone, L., Carrillo, H., Latif, Y., Scaramuzza, D., Neira, J., Reid, I., & Leonard, J. J. (2016). Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age. *IEEE Transactions on Robotics*, 32(6), 1309-1332. https://doi.org/10.1109/TRO.2016.2624754

Dai, H., Valenzuela, A., & Tedrake, R. (2014). Whole-body motion planning with centroidal dynamics and full kinematics. *IEEE-RAS International Conference on Humanoid Robots*, 295-302. https://doi.org/10.1109/HUMANOIDS.2014.7041375

Deisenroth, M. P., Neumann, G., & Peters, J. (2013). A survey on policy search for robotics. *Foundations and Trends in Robotics*, 2(1-2), 1-142. https://doi.org/10.1561/2300000021

Haddadin, S., De Luca, A., & Albu-Schäffer, A. (2017). Robot collisions: A survey on detection, isolation, and identification. *IEEE Transactions on Robotics*, 33(6), 1292-1312. https://doi.org/10.1109/TRO.2017.2723903

Hoffman, G., & Breazeal, C. (2007). Cost-based anticipatory action selection for human-robot fluency. *IEEE Transactions on Robotics*, 23(5), 952-961. https://doi.org/10.1109/TRO.2007.907483

Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. *The International Journal of Robotics Research*, 30(7), 846-894. https://doi.org/10.1177/0278364911406761

Kuindersma, S., Deits, R., Fallon, M., Valenzuela, A., Dai, H., Permenter, F., Koolen, T., Marion, P., & Tedrake, R. (2016). Optimization-based locomotion planning, estimation, and control design for the Atlas humanoid robot. *Autonomous Robots*, 40(3), 429-455. https://doi.org/10.1007/s10514-015-9479-3

LaValle, S. M. (2006). *Planning algorithms*. Cambridge University Press. https://doi.org/10.1017/CBO9780511546877

LaValle, S. M., & Kuffner, J. J. (2001). Randomized kinodynamic planning. *The International Journal of Robotics Research*, 20(5), 378-400. https://doi.org/10.1177/02783640122067453

Levine, S., Finn, C., Darrell, T., & Abbeel, P. (2016). End-to-end training of deep visuomotor policies. *Journal of Machine Learning Research*, 17(1), 1334-1373.

Metta, G., Natale, L., Nori, F., Sandini, G., Vernon, D., Fadiga, L., Von Hofsten, C., Rosander, K., Lopes, M., Santos-Victor, J., Bernardino, A., & Montesano, L. (2010). The iCub humanoid robot: An open-systems platform for research in cognitive development. *Neural Networks*, 23(8-9), 1125-1134. https://doi.org/10.1016/j.neunet.2010.08.010

Pfeifer, R., & Bongard, J. C. (2006). *How the body shapes the way we think: A new view of intelligence*. MIT Press.

Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer handbook of robotics* (2nd ed.). Springer. https://doi.org/10.1007/978-3-319-32552-1

Stasse, O., Flayols, T., Budhiraja, R., Giraud-Esclasse, K., Carpentier, J., Del Prete, A., Saurel, G., Mansard, N., Lamiraux, F., Laumond, J.-P., Marchionni, L., Tome, H., & Ferro, F. (2017). TALOS: A new humanoid research platform targeted for industrial applications. *IEEE-RAS International Conference on Humanoid Robots*, 689-695. https://doi.org/10.1109/HUMANOIDS.2017.8246947

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT Press.

Tobin, J., Fong, R., Ray, A., Schneider, J., Zaremba, W., & Abbeel, P. (2017). Domain randomization for transferring deep neural networks from simulation to the real world. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 23-30. https://doi.org/10.1109/IROS.2017.8202133

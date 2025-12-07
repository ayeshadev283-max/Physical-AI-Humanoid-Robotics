---
id: 0013
title: Update spec with module flow and Docusaurus issue
stage: spec
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: CW
command: /sp.specify
labels: ["specification", "module-structure", "docusaurus", "issue-tracking"]
links:
  spec: specs/001-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-robotics-book/spec.md
tests:
  - None (specification document update)
---

## Prompt

Book Structure — Module Flow Specification + Known Issue

--- Known Issue ---
First page of Docusaurus ( /docs/ ) is not loading correctly.
Suspected causes:
- Wrong baseUrl configuration
- Incorrect docs route or sidebar ID mismatch
- Navbar logo href pointing to /docs/ but entry file missing
- Missing index.md in /docs/

--- Module Flow ---

Module 1 — Physical AI Foundations
Flow:
1. Introduction
2. Embodied Intelligence
3. Sensing & Perception
4. Locomotion & Motor Control

Module 2 — ROS 2
Flow:
1. Core Concepts
2. Nodes, Topics, Services, Actions
3. TF & URDF
4. Simulation Pipeline

Module 3 — Digital Twin (Gazebo + Unity)
Flow:
1. Digital Twin Basics
2. Gazebo Physics
3. Unity Animation
4. ROS 2 Integration

Module 4 — NVIDIA Isaac
Flow:
1. Isaac Overview
2. Perception & Planning
3. Sim-to-Real
4. Control Loops

Module 5 — VLA (RT-2)
Flow:
1. VLA Fundamentals
2. RT-2
3. Policy Integration
4. Humanoid Skills

Capstone Project — Autonomous Humanoid
Flow:
1. System Overview
2. Perception Stack
3. Control Stack
4. VLA-Based Autonomy

## Response snapshot

Updated the existing specification (specs/001-physical-ai-robotics-book/spec.md) with:

1. **Known Issues Section**: Documented the Docusaurus homepage routing issue with suspected causes, current configuration details, and impact analysis.

2. **Module Structure & Flow Section**: Added detailed chapter-level organization for all 6 modules (Module 0-5), including:
   - Specific chapter flow for each module
   - Purpose statement explaining learning objectives
   - Conceptual progression within each module

3. **Updated Functional Requirements**: Added FR-003a, FR-007a, FR-011a, FR-015a, FR-019a, FR-023a to codify the required chapter flow for each module, and FR-030 for the Docusaurus routing fix.

4. **Updated Metadata**: Changed "Updated" date to 2025-12-07.

The specification now provides clear guidance for:
- Content creators: exact chapter structure and flow
- Developers: specific Docusaurus issue to resolve
- Learners: understanding the learning progression

## Outcome

- ✅ Impact: Specification now includes detailed module flow structure and documents critical Docusaurus routing issue
- 🧪 Tests: None (specification document)
- 📁 Files: Modified specs/001-physical-ai-robotics-book/spec.md
- 🔁 Next prompts: Fix Docusaurus routing issue (investigate baseUrl, slug conflicts, sidebar configuration), or proceed to /sp.plan for implementation planning
- 🧠 Reflection: The user provided both structural requirements (module flow) and a known issue in one message. Successfully captured both concerns in the specification without creating a new feature branch, as this was clearly refining the existing Physical AI Robotics Book feature.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable (specification update)
- Prompt variant (if applicable): Standard /sp.specify invocation with combined requirements and issue documentation
- Next experiment (smallest change to try): Consider creating a separate Known Issues section template for tracking implementation blockers in specifications

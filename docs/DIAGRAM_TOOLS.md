# Diagram Tools Setup

This document describes the diagram tools used in the Physical AI & Humanoid Robotics book.

## Mermaid.js (Primary Tool) ✅

**Status**: Configured and ready to use

Mermaid is embedded directly in Docusaurus and renders diagrams from markdown code blocks.

### Installation
- Already installed via `@docusaurus/theme-mermaid@3.9.2`
- Configured in `docusaurus.config.js`

### Usage in Markdown
```markdown
​```mermaid
graph TD
    A[Sensor Input] --> B[Perception]
    B --> C[Decision Making]
    C --> D[Motor Control]
    D --> E[Actuator Output]
​```
```

### Supported Diagram Types
- Flowcharts (`graph TD`, `graph LR`)
- Sequence diagrams (`sequenceDiagram`)
- State diagrams (`stateDiagram-v2`)
- Class diagrams (`classDiagram`)
- Entity-Relationship diagrams (`erDiagram`)
- Gantt charts (`gantt`)

### Best Practices
- Keep diagrams simple and focused
- Use consistent color schemes
- Add descriptive labels
- Test rendering in dev server before committing

## Excalidraw (Secondary Tool) 📝

**Status**: Manual installation required

Excalidraw is recommended for freehand diagrams, architecture sketches, and complex visualizations that don't fit Mermaid's structured format.

### Installation
1. Download Excalidraw Desktop: https://github.com/excalidraw/excalidraw/releases
2. Or use web version: https://excalidraw.com/

### Workflow
1. Create diagram in Excalidraw
2. Export as SVG or PNG
3. Save to `static/img/diagrams/{module-name}/`
4. Reference in markdown:
   ```markdown
   ![System Architecture](../../static/img/diagrams/module-3-isaac/isaac-ecosystem.svg)
   ```

### Use Cases
- Complex system architectures
- Hardware diagrams
- Robotic kinematic chains
- Network topologies
- Hand-drawn conceptual diagrams

## Diagram Guidelines

### Module 0: Physical AI Foundations
- Use Mermaid for: perception-action loops, subsumption architecture layers
- Use Excalidraw for: embodiment concept illustrations, sensor fusion pipelines

### Module 1: ROS 2
- Use Mermaid for: node graphs, topic flows, TF trees
- Use Excalidraw for: URDF visual representations

### Module 2: Digital Twin
- Use Mermaid for: simulation pipelines, data flow
- Use Excalidraw for: Gazebo/Unity integration architectures

### Module 3: Isaac
- Use Mermaid for: Isaac ROS perception pipelines
- Use Excalidraw for: GPU acceleration workflows

### Module 4: VLA
- Use Mermaid for: model architectures, training pipelines
- Use Excalidraw for: attention mechanisms, multimodal fusion

### Module 5: Capstone
- Use both as needed for complete system design

# Data Model: Physical AI & Humanoid Robotics Book

**Date**: 2025-12-05
**Purpose**: Phase 1 design - Define content entity model, metadata schema, and relationships

## Content Entity Model

### Entity 1: Chapter

**Purpose**: Primary content unit covering a coherent topic

**Metadata**:
- `title`: String (e.g., "Perception and Sensor Fusion")
- `number`: Integer (1-12)
- `learning_objectives`: List of strings (3-5 objectives)
- `prerequisites`: List of chapter numbers or background topics
- `estimated_time`: Integer (hours)
- `difficulty`: Enum (Beginner | Intermediate | Advanced)
- `tags`: List of strings (e.g., ["perception", "sensors", "fusion"])

**Content Structure**:
- Introduction (2-3 paragraphs)
- Main sections (2-5 sections with subsections)
- Exercises (3-10 problems)
- Summary (3-5 key takeaways)
- References (10-30 APA citations)

**Relationships**:
- `previous_chapter`: Link to prerequisite chapter
- `next_chapter`: Link to subsequent chapter
- `code_examples`: List of associated code examples
- `glossary_terms`: List of terms introduced/used
- `case_study`: Optional link to case study chapter

### Entity 2: Code Example

**Purpose**: Executable demonstration of algorithms/concepts

**Metadata**:
- `title`: String
- `language`: "python" (all examples)
- `difficulty`: Enum (Beginner | Intermediate | Advanced)
- `dependencies`: Dict mapping package names to pinned versions
- `estimated_runtime`: Integer (minutes)
- `parent_chapter`: Integer (chapter number)

**Content Structure**:
- Purpose statement
- Dependencies and setup
- Code with inline comments
- Expected output
- Variations/exercises
- Troubleshooting
- References

**Relationships**:
- `chapter`: Parent chapter
- `related_examples`: Prerequisites (other examples)
- `concepts`: List of concepts demonstrated

### Entity 3: Case Study

**Purpose**: In-depth analysis of real-world humanoid system

**Metadata**:
- `robot_name`: String (e.g., "Atlas", "Optimus", "TALOS")
- `organization`: String
- `year_introduced`: Integer
- `architecture_type`: String (e.g., "hydraulic", "electric")
- `primary_sources`: List of key paper citations

**Content Structure**:
- System overview
- Hardware architecture
- Perception system
- Motion planning and control
- Learning/adaptation (if applicable)
- Performance analysis
- Design trade-offs
- Lessons learned
- References

**Relationships**:
- `related_chapters`: Chapters demonstrating relevant concepts
- `code_examples`: Related implementations
- `references`: Peer-reviewed papers analyzing system

### Entity 4: Exercise/Problem

**Purpose**: Assessment and learning activities

**Metadata**:
- `id`: String (e.g., "CH4-E3")
- `difficulty`: Enum (Beginner | Intermediate | Advanced)
- `type`: Enum (Conceptual | Mathematical | Implementation)
- `estimated_time`: Integer (minutes)
- `chapter`: Integer

**Content Structure**:
- Problem statement
- Hints (optional)
- Solution outline (in appendix)

**Relationships**:
- `chapter`: Parent chapter
- `learning_objectives`: Objectives addressed
- `solution`: Link to appendix solution (if provided)

### Entity 5: Reference/Citation

**Purpose**: Bibliographic entry for sourced content

**Metadata**:
- `id`: String (unique identifier)
- `type`: Enum (Peer-Reviewed | Textbook | Documentation | Web)
- `authors`: List of strings
- `year`: Integer
- `title`: String
- `publication`: String (journal, conference, publisher)
- `doi_url`: String (optional)
- `is_peer_reviewed`: Boolean

**Content Structure**:
- APA-formatted citation
- Summary/notes (internal, not published)
- Tags/categories

**Relationships**:
- `citing_chapters`: Chapters that reference this source
- `claims_supported`: Specific claims this source supports

### Entity 6: Glossary Entry

**Purpose**: Definition of technical terminology

**Metadata**:
- `term`: String
- `aliases`: List of strings (alternative terms)
- `introduced_in_chapter`: Integer
- `category`: String (e.g., "perception", "planning", "hardware")

**Content Structure**:
- Definition (1-3 sentences)
- Context of usage
- Related terms
- Example usage (optional)

**Relationships**:
- `chapters`: Chapters using this term
- `related_terms`: Other glossary entries

## Navigation Structure (Docusaurus Sidebar)

```javascript
// sidebars.js structure
module.exports = {
  bookSidebar: [
    'index',  // Landing page
    {
      type: 'category',
      label: 'Part I: Foundations',
      items: [
        'chapters/01-introduction',
        'chapters/02-foundations-physical-ai',
        'chapters/03-humanoid-hardware',
      ],
    },
    {
      type: 'category',
      label: 'Part II: Core Algorithmic Domains',
      items: [
        'chapters/04-perception-sensors',
        'chapters/05-motion-planning',
        'chapters/06-control-architectures',
        'chapters/07-learning-algorithms',
        'chapters/08-human-robot-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Part III: Case Studies',
      items: [
        'chapters/09-case-study-atlas',
        'chapters/10-case-study-optimus',
        'chapters/11-case-study-research-platforms',
      ],
    },
    'chapters/12-future-directions',
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/A-prerequisites',
        'appendices/B-mathematics-review',
        'appendices/C-setup-instructions',
        'appendices/D-solutions',
      ],
    },
    'references/bibliography',
  ],
};
```

## Metadata Schema (Front Matter for Markdown Files)

### Chapter Front Matter

```yaml
---
id: chapter-04-perception
title: "Chapter 4: Perception and Sensor Fusion"
sidebar_label: "4. Perception"
sidebar_position: 4
description: "Learn vision pipelines, sensor fusion, and SLAM for humanoid robots"
difficulty: intermediate
estimated_time: 3
prerequisites:
  - chapter-03-hardware
  - linear-algebra
  - probability
learning_objectives:
  - Implement camera calibration and vision processing pipelines
  - Apply Kalman filtering for sensor fusion
  - Understand SLAM basics for robotic localization
tags:
  - perception
  - computer-vision
  - sensor-fusion
  - slam
code_examples:
  - vision-pipeline
  - sensor-fusion-ekf
  - slam-basic
---
```

### Code Example Front Matter

```yaml
---
id: vision-pipeline
title: "Vision Processing Pipeline"
language: python
difficulty: intermediate
estimated_runtime: 5
chapter: 4
dependencies:
  numpy: "1.24.3"
  opencv-python: "4.8.0"
  matplotlib: "3.7.1"
related_concepts:
  - camera-calibration
  - object-detection
  - image-preprocessing
---
```

## Design Complete

All entity models defined. Navigation structure specified. Metadata schemas documented. Ready for template creation (contracts/) and quickstart guide.

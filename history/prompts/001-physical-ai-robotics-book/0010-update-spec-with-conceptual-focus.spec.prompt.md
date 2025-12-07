---
id: 0010
title: Update Spec with Conceptual Focus
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: CW
command: /sp.specify
labels: ["spec-update", "conceptual-focus", "documentation", "book-structure"]
links:
  spec: specs/001-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-robotics-book/spec.md
  - specs/001-physical-ai-robotics-book/checklists/requirements.md
tests:
  - Validation: All checklist items PASS
  - No [NEEDS CLARIFICATION] markers
  - Out-of-scope section expanded
---

## Prompt

User ran `/sp.specify` with updated feature description emphasizing:
- **Focus**: High-level concepts, diagrams, workflows, conceptual code
- **Explicitly NO**: Installation tutorials in main content
- **Out of scope**: Deep academic research, hardware building guides, vendor comparisons, AI ethics
- **VLA**: Mentioned "VLA (RT-2)" but existing spec uses OpenVLA/SmolVLA
- **Goal**: Structured Docusaurus book on Physical AI and Humanoid Robotics

User chose option "1" to update the existing spec (001-physical-ai-robotics-book) rather than create a new feature branch.

## Response snapshot

Successfully updated the existing specification with the following changes:

**1. Focus Shift to Conceptual Learning**:
- Updated edge cases to emphasize conceptual overview over hands-on installation
- Changed functional requirements to focus on diagrams, workflows, and architectural explanations
- Made clear that installation/setup details belong in appendices, not main modules

**2. Expanded Out-of-Scope Section**:
Added explicit exclusions for:
- Deep academic research (novel algorithms, proofs)
- Hardware building guides (mechanical/electrical design)
- Installation steps in main content (relegated to appendices)
- Vendor comparisons (focus on concepts, not products)
- AI ethics and philosophy (briefly acknowledged, not primary focus)

**3. Updated Functional Requirements** (FR-004 through FR-026):
- **ROS 2 module**: "conceptual code examples" and "workflow diagrams" instead of "hands-on examples" and "complete workspace setup"
- **Digital Twin**: "explain concepts" and "high-level workflows" instead of "spawning robots" and "configuring physics"
- **Isaac module**: "explain architecture" and "workflow diagrams" instead of "setup" and "loading scenes"
- **VLA module**: References both RT-2 (research reference) and OpenVLA/SmolVLA (open-source), updated to include "ethical implications"
- **Cross-module**: "conceptual/illustrative code" and "visual-first approach" instead of "reproducible with pinned versions"

**4. Updated Edge Cases**:
Changed all edge case responses to de-emphasize installation/setup:
- "conceptual overview" instead of "setup guides and introductory tutorials"
- "detailed installation/setup in appendices only, not main content"
- "Document concepts and architectures that transcend specific versions"
- "Focus on platform-agnostic concepts"

**5. VLA Model Handling**:
Kept OpenVLA/SmolVLA as primary (based on research decision that RT-2 is not open-sourced), but added RT-2 as a reference model in FR-018

**6. Updated Checklist**:
Updated requirements.md with new validation section documenting the 2025-12-06 changes and confirming all checks still pass.

## Outcome

- ✅ Impact: Spec updated to emphasize conceptual learning over hands-on tutorials, aligning with user's updated vision
- 🧪 Tests: All checklist items PASS; no [NEEDS CLARIFICATION] markers
- 📁 Files: 2 files updated (spec.md, checklists/requirements.md)
- 🔁 Next prompts: Spec is validated and ready for `/sp.plan` (update existing plan) or continue content creation
- 🧠 Reflection: Successfully balanced the shift to conceptual focus while maintaining the existing spec structure and quality

## Evaluation notes (flywheel)

- Failure modes observed: None; update was straightforward
- Graders run and results (PASS/FAIL): Checklist validation PASS (all 16 items)
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Consider updating plan.md to reflect the conceptual-first approach in implementation tasks

---
id: 0011
title: Update Plan for Conceptual Focus
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: CW
command: /sp.plan
labels: ["plan-update", "conceptual-focus", "6-modules", "documentation"]
links:
  spec: specs/001-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-robotics-book/plan.md
tests:
  - Constitution Check: All gates PASS
  - Phase 0 Research: COMPLETE (research.md exists)
  - Phase 1 Design: COMPLETE (data-model.md, quickstart.md, contracts/ exist)
  - Phase 1 Setup: COMPLETE (Docusaurus builds successfully)
---

## Prompt

User ran `/sp.plan` after updating the specification to emphasize conceptual-first approach (no installation tutorials in main content). The plan needed to align with:
- 6-module structure (Foundations, ROS 2, Gazebo/Unity, Isaac, VLA, Capstone)
- Conceptual focus: diagrams, workflows, architectural explanations
- Installation tutorials relegated to appendices
- Visual-first approach with extensive diagrams

## Response snapshot

Successfully updated plan.md to align with conceptual-first specification:

**1. Technical Context Updated**:
- **Content Focus** section added documenting conceptual-first approach
- Main modules focus on "what" and "why" (concepts, architectures, tradeoffs)
- Code examples are conceptual/illustrative (patterns and logic, NOT installation)
- Appendices contain setup guides, installation steps, troubleshooting

**2. Constitution Check Enhanced**:
- Added post-design re-check section documenting 2025-12-06 updates
- Verified spec FR-026 ("conceptual/illustrative code"), FR-025 ("extensive diagrams")
- Confirmed out-of-scope clarifications (installation tutorials excluded from main content)

**3. Project Structure Documented**:
- Clear separation: docs/ (main content), examples/ (conceptual code), references/ (bibliography)
- Docker infrastructure for reproducible examples (Dockerfile.ros2, Dockerfile.isaac, Dockerfile.vla)
- CI/CD workflows (build-test.yml, link-check.yml, validate_content.sh)

**4. Phase 0 Research Status**: ✅ COMPLETE
- 7 research decisions documented in research.md:
  1. Licensing: CC BY-SA 4.0 (content) + Apache 2.0 (code)
  2. VLA Models: OpenVLA/SmolVLA (open-source); RT-2 referenced
  3. Cloud: AWS RoboMaker + AWS Educate
  4. Robot Models: Unitree H1, iCub
  5. Citations: Zotero + Better BibTeX + Pandoc (100-200 references)
  6. Docusaurus: Git-branch versioning, Algolia v4, KaTeX, Mermaid
  7. CI/CD: GitHub Actions + Docker matrix builds

**5. Phase 1 Design Status**: ✅ COMPLETE
- data-model.md: Content entities (Module, Chapter, Code Example, Diagram, etc.)
- quickstart.md: Getting started guide for contributors
- contracts/: Chapter templates, citation formats

**6. Phase 2 Implementation Strategy**: ✅ COMPLETE
- tasks.md exists with 160 tasks across 9 phases
- Phase 1 (Setup) marked COMPLETE: Infrastructure ready, build succeeds
- Phases 2-9 marked NOT STARTED: Content creation pending

**7. Conceptual-First Implementation Notes**:
- **Main Modules**: Concepts, diagrams, workflows (NO installation tutorials)
- **Appendices**: Setup instructions, prerequisites, troubleshooting
- **Code Examples**: Dockerized, conceptual focus, SPDX headers

**8. Risks & Mitigations Added**:
- Technical: Rapid evolution, scope creep, citation ratio
- Content: Consistency, diagram accessibility, broken links
- Mitigations: Version-agnostic concepts, strict spec adherence, CI/CD validation

**9. Success Metrics Defined**:
- Phase 1 complete ✓
- 6 modules with 3+ chapters each (pending)
- 3+ diagrams per module with alt text (pending)
- 50%+ citation ratio maintained (67% currently)
- Docusaurus build <3 minutes (currently ~7 seconds)
- All CI/CD checks passing ✓

## Outcome

- ✅ Impact: Plan updated to reflect conceptual-first approach; aligns with updated spec and Phase 1 implementation
- 🧪 Tests: Constitution Check PASS; research complete; design complete; Phase 1 setup complete
- 📁 Files: 1 file updated (plan.md, 326 lines)
- 🔁 Next prompts: Begin Phase 2 (Foundational appendices) OR proceed to module content (Phases 3-8)
- 🧠 Reflection: Successfully aligned plan with conceptual-first spec while preserving completed work (research, design, Phase 1 setup)

## Evaluation notes (flywheel)

- Failure modes observed: setup-plan.sh overwrote existing plan.md; recovered by recreating from research.md + spec.md context
- Graders run and results (PASS/FAIL): Constitution Check PASS, all Phase 0-1 artifacts exist and valid
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Consider protecting completed plans from setup script overwrites

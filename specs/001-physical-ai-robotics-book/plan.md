# Implementation Plan: Physical AI & Humanoid Robotics Book

**Branch**: `001-physical-ai-robotics-book` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-robotics-book/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive, AI/Spec-driven graduate-level textbook on Physical AI and Humanoid Robotics using Docusaurus. The book targets robotics learners, engineering students, and developers, focusing on high-level concepts, diagrams, workflows, and conceptual code (NOT installation tutorials). Content covers six modules: Physical AI Foundations, ROS 2 (Robotic Nervous System), Gazebo & Unity (Digital Twin), NVIDIA Isaac (AI-Robot Brain), Vision-Language-Action (VLA) models, and an Autonomous Humanoid capstone project. The implementation will follow academic standards with at least 50% peer-reviewed sources, APA citations, and complete reproducibility documentation for all code examples.

## Technical Context

**Language/Version**: Markdown (CommonMark), JavaScript (Node.js 18+), React 18+ (Docusaurus framework)
**Primary Dependencies**: Docusaurus 3.x, React 18+, @docusaurus/preset-classic, @docusaurus/theme-mermaid, remark-math, rehype-katex
**Storage**: Static files (markdown content in docs/, examples in examples/, references in docs/references/)
**Testing**: Docusaurus build validation, markdown linting, link checking, code syntax validation (language-specific linters)
**Target Platform**: Web (GitHub Pages), responsive design for desktop/tablet/mobile
**Project Type**: Static documentation site (Docusaurus-based educational content)
**Performance Goals**: <3s initial page load, <1s navigation between pages, full-text search response <500ms
**Constraints**: GitHub Pages hosting limits (1GB recommended, 100GB bandwidth/month), accessible on mobile devices, supports offline reading (PWA optional)
**Scale/Scope**: 6 major modules, ~30-40 chapters total, 50+ diagrams, 100+ code examples, 200+ citations, comprehensive glossary and bibliography

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Academic Rigor Compliance:**
- [x] All technical claims are verifiable against primary or authoritative sources
- [x] Code examples include complete dependency specifications and reproducibility details
- [x] Content maintains clarity for academic audience with CS background
- [x] Citations follow APA format (when applicable)
- [x] At least 50% of sources are peer-reviewed (when applicable)
- [x] Speculative content is clearly marked as such

**Technical Standards:**
- [x] Markdown follows semantic heading structure
- [x] Code blocks specify language for syntax highlighting
- [x] Links use descriptive text
- [x] File organization follows project structure in constitution

**Quality Gates:**
- [x] Code examples will be syntax-validated before publication
- [x] Links will be verified (internal and external)
- [x] Reproducibility documented for all methodologies

**Status**: ✅ All constitution checks pass. This project aligns with academic rigor, technical standards, and quality gates defined in the constitution.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-robotics-book/
├── spec.md              # Feature specification
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (research decisions, best practices)
├── data-model.md        # Phase 1 output (content entities, metadata structure)
├── quickstart.md        # Phase 1 output (getting started guide for contributors)
├── contracts/           # Phase 1 output (content schemas, validation rules)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

This is a **Docusaurus-based documentation project**, not a traditional application. The structure follows Docusaurus conventions:

```text
docs/                    # All book content (Markdown)
├── index.md             # Homepage/Introduction
├── chapters/            # Main book chapters organized by module
│   ├── module-0-foundations/
│   │   ├── 01-introduction.md
│   │   ├── 02-embodied-intelligence.md
│   │   ├── 03-sensing-perception.md
│   │   └── 04-locomotion-motor-control.md
│   ├── module-1-ros2/
│   │   ├── 01-core-concepts.md
│   │   ├── 02-nodes-topics-services-actions.md
│   │   ├── 03-tf-urdf.md
│   │   └── 04-simulation-pipeline.md
│   ├── module-2-digital-twin/
│   │   ├── 01-basics.md
│   │   ├── 02-gazebo-physics.md
│   │   ├── 03-unity-animation.md
│   │   └── 04-ros2-integration.md
│   ├── module-3-isaac/
│   │   ├── 01-overview.md
│   │   ├── 02-perception-planning.md
│   │   ├── 03-sim-to-real.md
│   │   └── 04-control-loops.md
│   ├── module-4-vla/
│   │   ├── 01-fundamentals.md
│   │   ├── 02-rt2-models.md
│   │   ├── 03-policy-integration.md
│   │   └── 04-humanoid-skills.md
│   └── module-5-capstone/
│       ├── 01-system-overview.md
│       ├── 02-perception-stack.md
│       ├── 03-control-stack.md
│       └── 04-vla-autonomy.md
├── appendices/          # Supplementary materials
│   ├── A-prerequisites.md
│   ├── B-mathematics-review.md
│   ├── C-setup-instructions.md
│   └── D-solutions.md
├── references/          # Bibliography and citations
│   └── bibliography.md
├── glossary.md          # Technical terms and definitions
└── assets/              # Images, diagrams, videos
    ├── diagrams/
    ├── screenshots/
    └── videos/

examples/                # Runnable code examples
├── module-1-ros2/
├── module-2-simulation/
├── module-3-isaac/
├── module-4-vla/
└── module-5-capstone/

src/                     # Docusaurus React components (custom if needed)
├── components/          # Custom React components for book
├── css/                 # Custom styles
└── pages/               # Custom pages (if needed beyond docs)

static/                  # Static assets served directly
├── img/
└── files/

sidebars.js              # Docusaurus sidebar configuration
docusaurus.config.js     # Main Docusaurus configuration
package.json             # Node.js dependencies
```

**Structure Decision**: Docusaurus static site structure chosen because:
1. Built-in markdown support with MDX for interactive components
2. Versioning support for future editions
3. Full-text search via Algolia integration
4. GitHub Pages deployment integration
5. Dark mode, responsive design, and accessibility built-in
6. Mermaid diagram support via plugin
7. KaTeX math rendering via remark/rehype plugins

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected.** All constitution checks pass. No complexity justification required.



## Known Issues

### Issue 1: Docusaurus Homepage Not Loading (Priority: P1)

**Description**: The first page of Docusaurus (/docs/) is not loading correctly.

**Suspected Root Causes**:
1. baseUrl configuration mismatch (currently `/` but logo href is `/docs/`)
2. Sidebar ID mismatch between docusaurus.config.js and sidebars.js
3. docs/index.md has `slug: /` which may conflict with baseUrl routing
4. Missing or incorrect routing preset configuration

**Current Configuration Analysis**:
- `docusaurus.config.js:19`: `baseUrl: '/'`
- `docusaurus.config.js:78`: `logo.href: '/docs/'`
- `docusaurus.config.js:83`: `sidebarId: 'bookSidebar'`
- `sidebars.js:17`: `bookSidebar: [...]`
- `docs/index.md`: `slug: /` and `sidebar_position: 1`

**Hypothesis**: The slug `/` in docs/index.md conflicts with the baseUrl `/`. When baseUrl is `/`, Docusaurus expects docs to be at `/docs/` by default, but the slug `/` tries to make index.md the root route, causing a conflict.

**Resolution Plan** (to be researched in Phase 0):
1. Research Docusaurus routing best practices for docs-only sites
2. Test three approaches:
   - **Option A**: Keep `baseUrl: '/'`, change `docs/index.md` slug to `intro` or remove slug
   - **Option B**: Keep `baseUrl: '/'`, configure docs plugin with `routeBasePath: '/'` to make docs the root
   - **Option C**: Change `baseUrl: '/Book/'` for GitHub Pages subpath, adjust all hrefs accordingly
3. Validate routing with local build and navigation testing
4. Document chosen solution in research.md with rationale

## Phase 0: Research Plan

### Research Tasks

The following unknowns from Technical Context and Known Issues require research before implementation:

1. **Docusaurus Routing Best Practices**
   - **Question**: What is the correct way to configure Docusaurus for a docs-only site where the homepage is the first doc?
   - **Research Approach**: Official Docusaurus documentation (docs.docusaurus.io), GitHub issues, successful Docusaurus projects (React, Jest, Babel docs)
   - **Deliverable**: Routing configuration decision in research.md

2. **Academic Citation Management**
   - **Question**: What tools/workflows best support APA citations in Markdown with Docusaurus?
   - **Research Approach**: Evaluate Zotero + Better BibTeX, pandoc-citeproc, remark-citation plugins
   - **Deliverable**: Citation workflow recommendation in research.md

3. **Code Example Validation Strategy**
   - **Question**: How to automate syntax validation and testing for 100+ code examples across Python, C++, YAML, etc.?
   - **Research Approach**: CI/CD integration (GitHub Actions), language-specific linters, Dockerized testing environments
   - **Deliverable**: Testing strategy and CI/CD pipeline design in research.md

4. **Diagram Generation and Management**
   - **Question**: What tools should be used for creating 50+ diagrams (architecture, workflows, concepts)?
   - **Research Approach**: Mermaid.js, Excalidraw, draw.io - evaluate tradeoffs (editability, version control, rendering quality)
   - **Deliverable**: Diagram tool stack recommendation in research.md

5. **VLA Model Integration Examples**
   - **Question**: What are reproducible, well-documented examples of OpenVLA/SmolVLA deployment that can be adapted for the book?
   - **Research Approach**: OpenVLA official repository, SmolVLA papers, RT-2 implementations, community tutorials
   - **Deliverable**: Curated list of example sources in research.md

6. **Isaac Sim + ROS 2 Integration Patterns**
   - **Question**: What are the current best practices for integrating Isaac Sim with ROS 2 for perception and control?
   - **Research Approach**: NVIDIA Isaac Sim documentation, Isaac ROS docs, peer-reviewed papers on sim-to-real transfer
   - **Deliverable**: Integration pattern recommendations in research.md

### Research Output Format

All research findings will be consolidated in `specs/001-physical-ai-robotics-book/research.md` with this structure:

```markdown
# Research Findings: Physical AI & Humanoid Robotics Book

## 1. Docusaurus Routing Configuration

**Decision**: [Chosen approach]
**Rationale**: [Why this approach was selected]
**Alternatives Considered**: [Other options evaluated]
**References**: [Links to official docs, GitHub issues, examples]

## 2. Citation Management Workflow

[Same structure]

## 3. Code Validation Strategy

[Same structure]

... (for each research task)
```

## Phase 1: Design & Contracts Plan

**Prerequisites**: `research.md` complete with all NEEDS CLARIFICATION resolved

### Data Model (data-model.md)

The "data model" for a documentation project represents content entities and their metadata structure:

**Content Entities**:
1. **Module** (top-level organizational unit)
   - Fields: module_number, title, description, learning_objectives[], prerequisites[]
   - Relationships: contains multiple Chapters

2. **Chapter** (individual markdown document)
   - Fields: chapter_number, title, module_ref, sidebar_position, slug, authors[], last_updated
   - Relationships: belongs to Module, contains multiple Diagrams, Code Examples, Citations

3. **Code Example**
   - Fields: example_id, title, language, file_path, docker_file, dependencies[], tested_versions[]
   - Validation: Must have README.md, must build/run without errors, must document expected output

4. **Diagram**
   - Fields: diagram_id, title, type (mermaid/image), file_path, alt_text, source_file
   - Validation: Must have alt text, source must be in version control, must render correctly

5. **Citation**
   - Fields: citation_key, authors[], year, title, publication, doi, url, source_type (peer-reviewed/documentation/book/preprint)
   - Validation: Must follow APA format, at least 50% must be peer-reviewed

6. **Glossary Entry**
   - Fields: term, definition, module_introduced, related_terms[]

### Contracts (contracts/)

For a documentation project, "contracts" define validation rules and content schemas:

1. **content-schema.yaml** - JSON Schema for frontmatter metadata in all markdown files
2. **citation-rules.yaml** - Rules for APA format validation and peer-review percentage requirement
3. **code-example-schema.yaml** - Required structure for code example directories
4. **quality-gates.yaml** - Checklist validation rules from constitution

### Quickstart Guide (quickstart.md)

A contributor onboarding guide covering:
1. Prerequisites (Node.js, Git, markdown editor)
2. Local setup (clone repo, npm install, npm start)
3. Content creation workflow (create branch, write chapter, add examples, cite sources)
4. Quality checks (run linters, build locally, verify citations)
5. Contribution guidelines (PR process, review checklist)

### Agent Context Update

After Phase 1 completion, run:
```bash
.specify/scripts/bash/update-agent-context.sh claude
```

This will add Docusaurus, React, and documentation-specific technologies to Claude Code's context without overwriting manual additions.

## Next Steps

1. **Complete Phase 0**: Generate research.md by dispatching research agents for each unknown
2. **Complete Phase 1**: Create data-model.md, contracts/, quickstart.md based on research findings
3. **Run Phase 2 separately**: Execute `/sp.tasks` to generate tasks.md (NOT part of this command)

## Architectural Decision Candidates

The following decisions made during planning may warrant ADRs:

1. **Docusaurus Routing Configuration** - If significant tradeoffs exist between routing approaches
2. **Citation Management Toolchain** - Choice of Zotero vs. manual vs. pandoc affects long-term maintainability
3. **Code Example Testing Strategy** - CI/CD setup and Docker vs. native testing has complexity implications
4. **Diagram Tool Selection** - Mermaid vs. external tools affects version control and collaboration

**ADR Suggestion**: After Phase 1 completion, evaluate which decisions meet the three-part test (Impact + Alternatives + Scope) and suggest:

```
📋 Architectural decision detected: [specific decision]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`
```

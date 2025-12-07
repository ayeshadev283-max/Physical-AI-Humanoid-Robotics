<!--
SYNC IMPACT REPORT
==================
Version change: 0.0.0 → 1.0.0
Principles defined: 4 core academic principles
Sections added: Core Principles, Academic Standards, Technical Standards, Development Workflow, Governance
Templates requiring updates:
  ✅ plan-template.md (Constitution Check section will reference these principles)
  ✅ spec-template.md (Requirements align with academic rigor)
  ✅ tasks-template.md (Quality gates align with verification standards)
Follow-up TODOs: None
-->

# AI-Native Book Creation Constitution
<!-- Technical Book on AI-Driven Software Development using Docusaurus and SpecKit Plus -->

## Core Principles

### I. Accuracy (NON-NEGOTIABLE)

All claims, examples, code snippets, and technical assertions MUST be verified against primary or authoritative sources before publication. No unverified statements are permitted.

**Requirements:**
- Every factual claim MUST be traceable to a reliable source
- Code examples MUST be tested and verified to work as documented
- Technical specifications MUST reference official documentation or peer-reviewed literature
- AI-generated content MUST be validated by human review against authoritative sources
- When sources conflict, multiple sources MUST be consulted and conflicts documented

**Rationale:** Academic and professional credibility depends on factual accuracy. Readers rely on this book for learning; inaccuracies undermine trust and pedagogical value.

### II. Clarity (NON-NEGOTIABLE)

Writing MUST be accessible to an academic audience with a computer science background, maintaining clear explanations, structured arguments, and well-labeled examples throughout.

**Requirements:**
- Technical concepts MUST be defined before use
- Arguments MUST follow logical structure with clear premises and conclusions
- Examples MUST be labeled with purpose, context, and expected outcomes
- Jargon MUST be explained or linked to glossary definitions
- Complex topics MUST be scaffolded from foundational concepts to advanced applications
- Visual aids (diagrams, flowcharts, code listings) MUST supplement textual explanations where appropriate

**Rationale:** Clarity ensures knowledge transfer. Even expert audiences benefit from well-structured, unambiguous technical writing that reduces cognitive load and maximizes comprehension.

### III. Reproducibility (NON-NEGOTIABLE)

Any methodology, code snippet, AI experiment, or technical process described MUST be traceable and repeatable by readers with documented prerequisites and expected outcomes.

**Requirements:**
- All code examples MUST include:
  - Complete dependency specifications (versions, package managers)
  - Environment setup instructions
  - Expected input/output with sample data
  - Known limitations or edge cases
- AI experiments MUST document:
  - Model versions and parameters
  - Prompt templates with placeholders clearly marked
  - Evaluation criteria and metrics
  - Expected variance in results
- Methodologies MUST specify:
  - Step-by-step procedures
  - Decision points and alternatives
  - Success criteria for each step
  - Troubleshooting common issues

**Rationale:** Reproducibility is fundamental to scientific and engineering practice. Readers must be able to validate claims and build upon documented techniques.

### IV. Rigor

Sources MUST be peer-reviewed where possible, and speculative claims MUST be clearly identified as such with supporting rationale.

**Requirements:**
- At least 50% of cited sources MUST be peer-reviewed academic publications
- Industry sources (documentation, blog posts, technical articles) are acceptable when:
  - No peer-reviewed alternative exists for cutting-edge topics
  - Source is from a recognized authority (e.g., official framework docs, reputable tech companies)
  - Multiple corroborating sources are provided
- Speculative content MUST be explicitly labeled with phrases like:
  - "Current evidence suggests..."
  - "This approach may..."
  - "Preliminary results indicate..."
- Opinion-based content MUST be clearly distinguished from factual assertions
- Emerging technologies MUST acknowledge uncertainty and rapid evolution

**Rationale:** Academic rigor ensures intellectual honesty and helps readers distinguish established knowledge from emerging practices and speculation.

## Academic Standards

### Source Verification

**Primary Source Mandate:**
- Prefer primary sources (original research papers, official specifications) over secondary interpretations
- When using secondary sources, verify claims against primary sources when feasible
- Document the verification process for key technical assertions

**Source Quality Hierarchy:**
1. Peer-reviewed journal articles and conference proceedings
2. Official technical documentation and specifications
3. Books from academic or reputable technical publishers
4. Technical blog posts from recognized experts or organizations
5. General web sources (require corroboration)

### Citation Requirements

**Format:** APA (American Psychological Association) style for all citations

**Inline Citations:**
- Direct quotes: (Author, Year, p. XX)
- Paraphrased ideas: (Author, Year)
- Multiple sources: (Author1, Year; Author2, Year)

**Bibliography:**
- Comprehensive reference list at chapter or book end
- Minimum required fields: Author, Year, Title, Publication/Source, DOI/URL where applicable
- All inline citations MUST have corresponding bibliography entries

**Source Diversity:**
- At least 50% peer-reviewed academic sources
- Maximum 25% from any single source type
- No more than 20% of citations from the same author/organization

### Content Validation

**Code Validation:**
- All code snippets MUST be syntax-checked before publication
- Functional code MUST be tested in documented environments
- Code examples MUST include comments explaining key concepts
- Deprecated or version-specific code MUST be clearly marked

**AI Experiment Validation:**
- AI-generated examples MUST be reproduced at least twice with documented variance
- Model outputs MUST be representative (not cherry-picked best results)
- Failures and limitations MUST be documented alongside successes
- Prompt engineering techniques MUST be validated across multiple use cases

## Technical Standards

### Technology Stack

**Core Technologies:**
- **Documentation Platform:** Docusaurus (version MUST be specified in package.json)
- **Development Methodology:** SpecKit Plus with Spec-Driven Development (SDD)
- **Version Control:** Git with GitHub for source code and deployment
- **Deployment:** GitHub Pages for public hosting

**Quality Tools:**
- Markdown linting for content consistency
- Link checking for citation and reference validity
- Code syntax validation for all embedded code
- Spell checking and grammar validation

### File Organization

**Content Structure:**
```
docs/
├── chapters/           # Book chapters in logical order
├── appendices/         # Supplementary materials
├── references/         # Bibliography and citation database
└── assets/            # Images, diagrams, code samples

specs/                 # Feature specifications (SpecKit Plus)
├── <feature>/
│   ├── spec.md
│   ├── plan.md
│   └── tasks.md

history/               # Development history (PHRs, ADRs)
├── prompts/
│   ├── constitution/
│   ├── <feature>/
│   └── general/
└── adr/
```

**Naming Conventions:**
- Chapter files: `XX-chapter-title.md` (numbered for ordering)
- Asset files: `descriptive-name.ext` (lowercase, hyphen-separated)
- Code samples: `example-name.lang` (with file extension)

### Documentation Standards

**Markdown Requirements:**
- Use semantic headings (H1 for chapter titles, H2 for sections, etc.)
- Code blocks MUST specify language for syntax highlighting
- Links MUST use descriptive text (not "click here")
- Images MUST have alt text for accessibility

**Code Sample Standards:**
- Maximum 50 lines per code block (longer examples in appendices or external files)
- Include language tag: ```python, ```javascript, etc.
- Add comments for complex logic
- Follow language-specific style guides (PEP 8 for Python, etc.)

## Development Workflow

### Spec-Driven Development Process

**Feature Development:**
1. Create feature specification (`specs/<feature>/spec.md`)
2. Generate implementation plan (`specs/<feature>/plan.md`)
3. Create task breakdown (`specs/<feature>/tasks.md`)
4. Implement with AI assistance (Claude Code + SpecKit Plus)
5. Review and validate against specification
6. Create Prompt History Records (PHRs) for significant decisions
7. Document Architectural Decision Records (ADRs) when applicable

**Content Creation Workflow:**
1. **Research Phase:**
   - Gather peer-reviewed sources
   - Validate technical claims
   - Test code examples
   - Document reproducibility steps

2. **Writing Phase:**
   - Draft content following constitution principles
   - Include inline citations in APA format
   - Label all examples and diagrams
   - Mark speculative content explicitly

3. **Review Phase:**
   - Verify all sources are accessible and correctly cited
   - Test all code examples
   - Check reproducibility of methodologies
   - Validate 50% peer-reviewed source requirement

4. **Publication Phase:**
   - Run automated checks (linting, link validation, spell check)
   - Build Docusaurus site locally
   - Review rendered output
   - Deploy to GitHub Pages

### Quality Gates

**Pre-Commit Checks:**
- [ ] All code examples syntax-validated
- [ ] All citations formatted in APA style
- [ ] All links verified (internal and external)
- [ ] Markdown linting passes
- [ ] Spell check complete

**Pre-Publication Checks:**
- [ ] At least 50% of sources are peer-reviewed
- [ ] All claims verified against sources
- [ ] All code examples tested
- [ ] All AI experiments documented with reproducibility details
- [ ] Speculative content clearly marked
- [ ] Docusaurus build succeeds without errors

### Prompt History Records (PHRs)

**When to Create PHRs:**
- After significant content creation sessions
- When making architectural decisions about book structure
- When resolving technical challenges in examples
- When clarifying complex topics or methodologies

**PHR Routing:**
- Constitution-related: `history/prompts/constitution/`
- Feature-specific (chapters, appendices): `history/prompts/<feature-name>/`
- General development: `history/prompts/general/`

**PHR Content Requirements:**
- Full user prompt (verbatim, not truncated)
- Key assistant response or summary
- Stage identifier (spec, plan, tasks, red, green, refactor, explainer, misc, general)
- Links to related artifacts (specs, ADRs, PRs)

### Architectural Decision Records (ADRs)

**Significance Test for ADR Creation:**

An ADR MUST be created when ALL three conditions are met:
1. **Impact:** The decision has long-term consequences for book structure, technical approach, or content organization
2. **Alternatives:** Multiple viable options were considered with distinct tradeoffs
3. **Scope:** The decision influences cross-cutting concerns or future development

**Examples of ADR-Worthy Decisions:**
- Choice of Docusaurus over other static site generators
- Content organization strategy (chronological vs. topical)
- Citation management approach
- Code example validation methodology
- AI experiment documentation format

**ADR Suggestion Format:**
```
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`
```

**ADR Storage:** `history/adr/XXX-decision-title.md`

## Governance

### Constitution Authority

This constitution supersedes all other development practices and documentation standards for this project. All content creation, technical decisions, and development workflows MUST comply with these principles.

### Amendment Process

**Requirements for Amendments:**
1. Proposed changes MUST be documented with rationale
2. Impact analysis MUST identify affected workflows and content
3. Amendment MUST follow semantic versioning:
   - **MAJOR:** Backward-incompatible changes (principle removals, redefinitions)
   - **MINOR:** New principles or materially expanded guidance
   - **PATCH:** Clarifications, wording improvements, typo fixes
4. All dependent templates MUST be updated to reflect amendments
5. Amendment MUST be ratified before taking effect

### Compliance Review

**Review Triggers:**
- Every pull request MUST verify compliance with core principles
- Monthly audits of source distribution (50% peer-reviewed requirement)
- Quarterly reviews of reproducibility (test code examples)
- Annual full constitution compliance audit

**Violation Handling:**
- Critical violations (accuracy, reproducibility) MUST be fixed before publication
- Minor violations (formatting, citation style) SHOULD be fixed in next revision
- Complexity violations MUST be justified in writing

### Complexity Justification

When deviating from simplicity principles (e.g., introducing complex technical explanations), justification MUST include:
- Why the complexity is necessary
- What simpler alternatives were considered
- Why simpler alternatives were insufficient
- How the complexity serves the reader's learning goals

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05

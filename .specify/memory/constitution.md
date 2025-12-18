<!--
SYNC IMPACT REPORT
==================
Version change: 1.0.0 → 1.1.0
Principles modified: None (core principles retained)
Sections added:
  - RAG Chatbot Integration Standards (new major section)
  - Interactive Learning Standards (new subsection in Academic Standards)
  - AI-Native Architecture Standards (new subsection in Technical Standards)
Templates requiring updates:
  ✅ plan-template.md (Constitution Check section references updated principles - already aligned)
  ✅ spec-template.md (Requirements align with academic rigor - already aligned)
  ✅ tasks-template.md (Quality gates align with verification standards - already aligned)
  ⚠ New templates may be needed for RAG chatbot feature specifications
Follow-up TODOs:
  - Consider creating RAG-specific testing checklist template
  - Consider ADR for RAG architecture choices (Qdrant, OpenAI, FastAPI stack)
Version bump rationale: MINOR version bump - New section added (RAG Chatbot Integration Standards) with materially expanded guidance for AI-native interactive features, but no backward-incompatible changes to existing principles.
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
- RAG chatbot responses MUST be grounded in book content and traceable to source text

**Rationale:** Academic and professional credibility depends on factual accuracy. Readers rely on this book for learning; inaccuracies undermine trust and pedagogical value. RAG integration ensures all AI-generated responses are verifiable against source material.

### II. Clarity (NON-NEGOTIABLE)

Writing MUST be accessible to an academic audience with a computer science background, maintaining clear explanations, structured arguments, and well-labeled examples throughout.

**Requirements:**
- Technical concepts MUST be defined before use
- Arguments MUST follow logical structure with clear premises and conclusions
- Examples MUST be labeled with purpose, context, and expected outcomes
- Jargon MUST be explained or linked to glossary definitions
- Complex topics MUST be scaffolded from foundational concepts to advanced applications
- Visual aids (diagrams, flowcharts, code listings) MUST supplement textual explanations where appropriate
- Interactive chatbot responses MUST maintain the same clarity standards as static content

**Rationale:** Clarity ensures knowledge transfer. Even expert audiences benefit from well-structured, unambiguous technical writing that reduces cognitive load and maximizes comprehension. Interactive AI assistance extends this clarity to dynamic user queries.

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
- RAG system implementations MUST document:
  - Embedding model versions and parameters
  - Chunking strategies and chunk sizes
  - Retrieval parameters (top-k, similarity thresholds)
  - Query logging and reproducibility mechanisms

**Rationale:** Reproducibility is fundamental to scientific and engineering practice. Readers must be able to validate claims and build upon documented techniques. RAG systems require reproducibility to enable verification and debugging.

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
- RAG chatbot responses MUST NOT introduce speculation beyond source material

**Rationale:** Academic rigor ensures intellectual honesty and helps readers distinguish established knowledge from emerging practices and speculation. RAG systems preserve rigor by grounding all responses in verified source content.

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

### Interactive Learning Standards

**RAG Chatbot Quality Assurance:**
- All chatbot responses MUST be grounded in book content with source references
- Retrieved context MUST be semantically relevant to user queries
- Responses MUST maintain academic tone and accuracy standards
- Hallucination detection mechanisms MUST be in place and monitored
- User queries and responses MUST be logged for quality review

**Evaluation Metrics:**
- **Accuracy**: Percentage of answers correctly grounded in book content (target: >95%)
- **Latency**: Response time from query submission to generation (target: <3 seconds p95)
- **User Satisfaction**: Qualitative feedback on answer clarity and relevance (tracked via optional feedback)
- **Traceability**: All responses MUST include source references to verify against original text

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

### AI-Native Architecture Standards

**RAG Chatbot Technology Stack:**
- **Frontend Integration:** Embedded within Docusaurus interface
  - User interaction layer for highlighting text and submitting queries
  - Real-time response display with source references
- **Vector Database:** Qdrant Cloud Free Tier
  - Stores and indexes book content as embeddings
  - Semantic similarity search for context retrieval
- **Embedding Model:** OpenAI embedding models
  - Text chunking strategy: ~500 words per chunk with overlap
  - Embedding dimension and model version MUST be documented
- **Generation Layer:** OpenAI Agents / ChatKit SDKs
  - Context-aware response generation
  - Grounding mechanism to prevent hallucination
- **Backend API:** FastAPI
  - Query handling and orchestration
  - Retrieval call management to Qdrant
  - Context forwarding to AI agent
  - Interaction logging for reproducibility
- **Database Layer:** Neon Serverless Postgres
  - Query storage and metadata
  - User interaction logs
  - Analytics on usage patterns and performance
  - Answer verification data

**Implementation Workflow:**
1. **Preprocessing and Embedding Generation:**
   - Segment book text into manageable chunks (~500 words each)
   - Convert chunks to embeddings using OpenAI embedding model
   - Upload chunks and embeddings to Qdrant with metadata
2. **Query Handling:**
   - Receive user queries via frontend
   - Convert query to embedding (same model as content)
   - Retrieve top-N relevant chunks from Qdrant
3. **Answer Generation:**
   - Pass retrieved chunks to OpenAI Agent
   - Generate response strictly based on retrieved content
   - Include source references for traceability
4. **Logging and Reproducibility:**
   - Store query, retrieved context, and generated response in Neon Postgres
   - Enable debugging and verification against book text

**RAG Quality Gates:**
- [ ] All book content properly chunked and embedded
- [ ] Embedding model version documented and locked
- [ ] Retrieval returns semantically relevant chunks (verified through test queries)
- [ ] Generated responses reference source chunks accurately
- [ ] No hallucination detected in sample testing (100+ test queries)
- [ ] All queries and responses logged with timestamps
- [ ] Source traceability functional (can map response to exact book location)
- [ ] Latency targets met (<3s p95)
- [ ] Error handling in place for retrieval failures and generation errors

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

backend/               # RAG chatbot backend (if applicable)
├── src/
│   ├── api/          # FastAPI endpoints
│   ├── services/     # Business logic (embedding, retrieval, generation)
│   ├── models/       # Data models
│   └── config/       # Configuration management
└── tests/
    ├── contract/     # API contract tests
    ├── integration/  # End-to-end RAG workflow tests
    └── unit/         # Unit tests for services
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

## RAG Chatbot Integration Standards

### Retrieval Quality

**Chunking Strategy:**
- Chunk size: ~500 words with 50-word overlap to preserve context boundaries
- Chunks MUST respect semantic boundaries (paragraphs, sections)
- Metadata MUST include: chapter, section, subsection, page number (if applicable)
- Code blocks MUST be kept intact within chunks where possible

**Embedding Quality:**
- Use consistent embedding model across all content
- Document model version and parameters
- Validate embedding quality with test queries (semantic similarity checks)
- Re-embed content if model is upgraded

**Retrieval Parameters:**
- Top-k retrieval: Start with k=5, tune based on response quality
- Similarity threshold: Document minimum threshold for relevance
- Contextual re-ranking: Consider implementing if initial retrieval quality insufficient

### Generation Quality

**Grounding Mechanism:**
- Agent MUST be instructed to answer ONLY from retrieved context
- Responses MUST include source references (chapter, section, or chunk ID)
- If context insufficient, agent MUST respond with: "I don't have enough information in the retrieved sections to answer this question accurately."
- NO speculation or general knowledge beyond source material

**Response Format:**
- Clear, concise answer based on retrieved text
- APA-style citations or internal references to source location
- Optional: Direct quotes from source material
- Explicit statement if question is partially answered due to insufficient context

**Safety and Accuracy:**
- Implement content filtering to prevent inappropriate queries
- Log all generation failures for review
- Monitor for hallucination patterns
- Regular audits of randomly sampled query-response pairs

### Logging and Reproducibility

**Query Logging Schema:**
- Timestamp (ISO 8601 format)
- User query (full text)
- Query embedding vector
- Retrieved chunks (IDs and similarity scores)
- Generated response (full text)
- Source references included in response
- User feedback (if provided)
- Response latency (milliseconds)

**Reproducibility Requirements:**
- All queries MUST be reproducible from logs
- Embedding model version MUST be logged
- Retrieval parameters MUST be logged
- Generation model and parameters MUST be logged
- Database schema MUST support historical query replay

**Analytics and Monitoring:**
- Track query volume over time
- Identify most common query topics
- Monitor retrieval quality (average similarity scores)
- Monitor generation latency (p50, p95, p99)
- Track user satisfaction (if feedback mechanism present)

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

**RAG Integration Workflow:**
1. **Preprocessing:**
   - Segment new/updated content into chunks
   - Generate embeddings
   - Upload to Qdrant with metadata
   - Verify retrieval quality with test queries

2. **Testing:**
   - Create test query set covering key topics
   - Validate retrieval returns relevant chunks
   - Validate generated responses are accurate and grounded
   - Check source references are correct

3. **Deployment:**
   - Deploy backend API updates
   - Update frontend integration if needed
   - Monitor initial query performance
   - Review logs for issues

4. **Monitoring:**
   - Regular review of query logs
   - Identify and address failure cases
   - Update retrieval parameters if needed
   - Re-embed content if embedding model upgraded

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

**RAG Integration Checks:**
- [ ] New content properly chunked and embedded
- [ ] Test queries return semantically relevant chunks
- [ ] Generated responses are accurate and grounded
- [ ] Source references work correctly
- [ ] No hallucination in test query set
- [ ] Latency targets met
- [ ] Logging functional and complete

### Prompt History Records (PHRs)

**When to Create PHRs:**
- After significant content creation sessions
- When making architectural decisions about book structure
- When resolving technical challenges in examples
- When clarifying complex topics or methodologies
- After implementing RAG chatbot features or updates

**PHR Routing:**
- Constitution-related: `history/prompts/constitution/`
- Feature-specific (chapters, appendices, RAG chatbot): `history/prompts/<feature-name>/`
- General development: `history/prompts/general/`

**PHR Content Requirements:**
- Full user prompt (verbatim, not truncated)
- Key assistant response or summary
- Stage identifier (spec, plan, tasks, red, green, refactor, explainer, misc, general, constitution)
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
- RAG architecture choices (Qdrant vs. alternatives, OpenAI vs. open-source models)
- Embedding model selection
- Chunking strategy for book content
- Backend framework selection (FastAPI vs. alternatives)

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
- RAG chatbot quality audits (monthly review of query logs and response accuracy)

**Violation Handling:**
- Critical violations (accuracy, reproducibility, RAG grounding failures) MUST be fixed before publication
- Minor violations (formatting, citation style) SHOULD be fixed in next revision
- Complexity violations MUST be justified in writing

### Complexity Justification

When deviating from simplicity principles (e.g., introducing complex technical explanations or architectures), justification MUST include:
- Why the complexity is necessary
- What simpler alternatives were considered
- Why simpler alternatives were insufficient
- How the complexity serves the reader's learning goals

**Version**: 1.1.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-11

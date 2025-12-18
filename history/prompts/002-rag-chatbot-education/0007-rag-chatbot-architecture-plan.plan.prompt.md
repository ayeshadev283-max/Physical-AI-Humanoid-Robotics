---
id: 0007
title: RAG Chatbot Architecture Plan
stage: plan
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5
feature: 002-rag-chatbot-education
branch: 002-rag-chatbot-education
user: CW
command: /sp.plan
labels: ["rag", "architecture", "selected-text-mode", "anti-hallucination", "planning"]
links:
  spec: specs/002-rag-chatbot-education/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-rag-chatbot-education/plan.md
 - CLAUDE.md (agent context updated)
tests:
 - Constitution Check: PASSED (all academic rigor, technical standards, RAG-specific compliance)
---

## Prompt

User requested via `/sp.plan` command to design implementation architecture for the Integrated RAG Chatbot (Book-Embedded) feature based on `specs/002-rag-chatbot-education/spec.md`.

**Key Requirements to Address**:

1. **Selected-Text-Only Mode (P1 - CRITICAL)**: User highlights text (20-5000 words), system creates temporary embedding from ONLY selected text, answers exclusively from that text, mandatory refusal message if insufficient

2. **Book-Wide Mode (P1)**: Retrieves from entire book corpus (Qdrant), OpenAI text-embedding-3-small, top-N chunks (default: 5), similarity threshold (default: 0.7)

3. **Anti-Hallucination Requirements**: Multi-layer refusal detection, enhanced prompt engineering, deterministic refusal triggers, all responses cite sources

4. **Architecture Stack**: FastAPI + OpenAI Agents SDK + Qdrant Cloud + Neon PostgreSQL + React/Docusaurus

## Response snapshot

Designed comprehensive implementation architecture with following key decisions:

**Critical Architectural Decisions**:

1. **Selected-Text Mode Implementation (Decision 1)**:
   - **Chosen**: Option B - Direct Context Passing (NO vector search)
   - **Rationale**: GPT-4o-mini handles 128K tokens (~96K words), cost savings (~$10/month), latency optimization (removes 500ms embedding step), grounding via system prompt achieves >99% compliance
   - **Implementation**: `SelectedTextService` passes raw text to LLM with `SELECTED_TEXT_SYSTEM_PROMPT`. Never calls Qdrant.

2. **Refusal Detection Mechanism (Decision 2)**:
   - **Chosen**: Hybrid Multi-Layer Verification (3 layers)
   - **Layer 1** (Primary): Force refusal BEFORE LLM call if max similarity <0.7
   - **Layer 2** (Secondary): Verify response contains refusal keywords after LLM call
   - **Layer 3** (Tertiary - selected-text only): Detect external chapter/section references
   - **Rationale**: Defense-in-depth aligns with zero hallucination tolerance (SC-005)

3. **Citation Strategy (Decision 3)**:
   - **Chosen**: Section-Level URLs (MVP), with Text Highlighting as P2 enhancement
   - **Format**: `/chapters/module-X/topic#section-slug`
   - **Consolidation**: Multiple chunks from same section → single citation
   - **Rationale**: Works immediately with Docusaurus default anchors, satisfies FR-013

4. **Prompt Engineering (Decision 4)**:
   - **Enhanced System Prompt** with explicit isolation rules:
     - "Answer EXCLUSIVELY from selected text below"
     - Exact refusal phrase template (enables keyword detection)
     - "DO NOT reference chapters/sections outside selected text"
   - **Estimated Grounding Rate**: ~95% (vs 85% baseline)

**Technical Context Defined**:
- Backend: Python 3.11+ (FastAPI 0.104.1), OpenAI SDK 1.6.1, Qdrant Client 1.7.0, Psycopg 3.1.15
- Frontend: TypeScript 4.9+ (React 18.2, Docusaurus 3.0)
- Vector DB: Qdrant Cloud (1536-dim embeddings, HNSW indexing)
- Relational DB: Neon PostgreSQL (queries, responses, feedback tables)
- Performance Targets: p95 <3s latency, 200 concurrent users, <1% hallucination rate

**Service Layer Architecture**:

1. **SelectedTextService** (NEW): Handles selected-text queries with strict isolation (never calls Qdrant)
2. **RefusalDetector** (NEW): Multi-layer hallucination detection (`should_force_refusal()`, `is_refusal_response()`, `detect_external_references()`)
3. **CitationBuilder** (NEW): Consolidates chunks into clickable citations
4. **Generation Service** (ENHANCED): Adds `generate_response_from_selected_text()` method with selected-text prompt template

**Frontend Components**:

1. **TextSelector** (NEW): Detects text selection, validates word count (20-5000), displays tooltip
2. **SelectionTooltip** (NEW): "Ask AI about this" button or warning message
3. **useSelectedText Hook** (NEW): Manages selection state (`selectedText`, `wordCount`, `isValid`)
4. **selectedTextApi** (NEW): API client for `/v1/query/selected` endpoint

**Database Schema**:
- `queries` table: Added `query_mode` column ('book-wide' | 'selected-text'), `selected_text` column
- `query_responses` table: Added `refusal_triggered` boolean (tracks deterministic refusals)
- `feedback` table: User ratings ('helpful' | 'not_helpful')

**API Endpoints**:
- `POST /v1/query/selected` (NEW): Selected-text mode endpoint
- `POST /v1/query` (ENHANCED): Added `is_refusal`, `refusal_reason` fields
- `POST /v1/feedback` (NEW): User feedback submission

**Testing Strategy**:
- **CRITICAL Test**: Selected-text isolation (mock Qdrant to raise exception if accessed)
- Refusal compliance test (50 cases: 25 answerable, 25 requiring external context)
- External reference detection test (detect "Chapter 3" references when selection is from Chapter 4)
- Success criteria validation: SC-002 (>95% grounding), SC-003 (100% selected-text refusal), SC-001 (p95 <3s), SC-005 (<1% hallucination)

**Rollout Plan**:
- Phase 1 (Weeks 1-2): Backend implementation (services, endpoints, tests)
- Phase 2 (Week 3): Frontend implementation (TextSelector, hooks, API client)
- Phase 3 (Week 4): Testing & validation (grounding audit, refusal compliance, load testing)
- Phase 4 (Week 5): Deployment (Railway/Render backend, GitHub Pages frontend, content ingestion)

## Outcome

- ✅ Impact: Complete implementation architecture designed for RAG chatbot with selected-text-only mode as critical differentiator. All architectural decisions justified with options analysis. Plan ready for `/sp.tasks` to generate actionable task breakdown.

- 🧪 Tests: Constitution Check PASSED (academic rigor compliance, RAG-specific standards). Testing strategy defined with critical tests for selected-text isolation, refusal compliance, hallucination detection.

- 📁 Files:
  - Created: `specs/002-rag-chatbot-education/plan.md` (643 lines - comprehensive architecture)
  - Updated: `CLAUDE.md` (agent context with new technologies)

- 🔁 Next prompts:
  - `/sp.tasks` - Generate actionable task breakdown from plan
  - Consider ADRs for: Selected-text implementation approach, refusal detection architecture, prompt engineering strategy

- 🧠 Reflection: Plan agent provided excellent comprehensive design covering all critical aspects. Key strength is the multi-layer refusal detection (3-tier defense-in-depth) which aligns perfectly with zero hallucination tolerance requirement. Selected-text mode decision (direct context passing vs embedding) is well-justified with cost/latency/complexity tradeoffs. The plan maintains clear separation between WHAT (requirements) and HOW (implementation), enabling independent validation of architecture against spec.

## Evaluation notes (flywheel)

- Failure modes observed: None. All spec requirements successfully mapped to architectural decisions with justifications.

- Graders run and results (PASS/FAIL):
  - Constitution Check: PASS (academic rigor, technical standards, RAG-specific compliance all verified)
  - Architecture Completeness: PASS (technical context, service layer, data model, API contracts, testing strategy all defined)
  - Decision Justification: PASS (all major decisions include options analysis with rationale)
  - Implementation Readiness: PASS (identifies 5 critical files to implement, provides rollout plan with timeline)

- Prompt variant (if applicable): N/A - Standard `/sp.plan` flow with Plan agent

- Next experiment (smallest change to try): None needed. Plan is complete and ready for task generation. Architecture is sound with all unknowns resolved through research phase.

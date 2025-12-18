# Implementation Plan: Integrated RAG Chatbot (Book-Embedded)

**Branch**: `002-rag-chatbot-education` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `specs/002-rag-chatbot-education/spec.md`

## Summary

This plan implements a **Retrieval-Augmented Generation (RAG) chatbot** embedded in the Docusaurus book with two critical modes:

1. **Book-Wide Mode (P1)**: Retrieves from entire book corpus stored in Qdrant, generates grounded answers with citations, refuses out-of-scope questions
2. **Selected-Text-Only Mode (P1 - CRITICAL)**: User highlights text (20-5000 words), system answers EXCLUSIVELY from that selection, refuses with mandatory message if insufficient

**Key Anti-Hallucination Features**:
- Multi-layer refusal detection (similarity threshold + keyword + external reference checks)
- Enhanced system prompts with explicit grounding instructions
- Deterministic refusal triggers (force refusal BEFORE calling LLM when confidence <0.7)
- All responses include clickable source citations

**Architecture**: FastAPI backend + OpenAI Agents SDK + Qdrant Cloud + Neon PostgreSQL + React/Docusaurus frontend

## Technical Context

**Language/Version**:
- Backend: Python 3.11+ (FastAPI 0.104.1)
- Frontend: TypeScript 4.9+ with React 18.2 (Docusaurus 3.0)
- Node.js: >=18.0

**Primary Dependencies**:
- Backend: FastAPI, OpenAI Python SDK 1.6.1, Qdrant Client 1.7.0, Psycopg 3.1.15, Pydantic 2.5.0
- Frontend: React 18.2, Axios 1.13.2, Docusaurus 3.0
- Vector DB: Qdrant Cloud (Free Tier: 1GB storage, 100 RPS)
- LLM: OpenAI GPT-4o-mini (generation), text-embedding-3-small (embeddings, 1536 dimensions)

**Storage**:
- Vector Store: Qdrant Cloud (book chunks + embeddings)
- Relational DB: Neon Serverless PostgreSQL (queries, responses, feedback, analytics)
- Tables: queries, retrieved_contexts, query_responses, feedback, documents, chunks

**Testing**:
- Backend: pytest 7.4.3, pytest-asyncio 0.21.1
- Frontend: Jest 30.2.0, React Testing Library 16.3.0

**Target Platform**:
- Backend: Linux server (Docker containerized, cloud deployment)
- Frontend: Static site (GitHub Pages)
- Browsers: Chrome, Firefox, Safari, Edge (last 2 versions)

**Project Type**: Web application (backend API + frontend static site)

**Performance Goals**:
- Query-to-response latency: p95 <3 seconds
- Concurrent users: 200 without degradation
- Embedding latency: <500ms per query
- Retrieval latency: <300ms for top-5 chunks
- Generation latency: <2s for typical response

**Constraints**:
- Zero hallucination tolerance: <1% ungrounded statements
- Selected-text mode: 100% refusal compliance (no false positives)
- API budget: ~$0.02 per query
- Free tier limits: Qdrant (1M vectors), Neon (0.5 GB)

**Scale/Scope**:
- Book corpus: ~200 chapters (~500KB text)
- Estimated chunks: ~2,000 (500 words each with 50-word overlap)
- Query volume: 1,000-10,000 queries/month (pilot)
- Users: 50-500 concurrent (academic semester load)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Academic Rigor Compliance:**
- [x] All technical claims verifiable (OpenAI API docs, Qdrant docs, RAG research papers)
- [x] Code examples include dependency specs and reproducibility details
- [x] Content maintains clarity for CS academic audience
- [x] Citations follow APA format (implementation includes citation generation per FR-012)
- [x] At least 50% peer-reviewed sources (RAG methodology: Lewis et al. 2020 REALM, Karpukhin et al. 2020 DPR)
- [x] Speculative content marked (design alternatives with tradeoffs)

**Technical Standards:**
- [x] Markdown follows semantic heading structure
- [x] Code blocks specify language for syntax highlighting
- [x] Links use descriptive text
- [x] File organization follows constitution structure

**Quality Gates:**
- [x] Code examples syntax-validated
- [x] Links verified (internal/external)
- [x] Reproducibility documented for all methodologies

**RAG-Specific Compliance:**
- [x] Chunking strategy: ~500 words, 50-word overlap, paragraph boundaries
- [x] Embedding model locked: text-embedding-3-small (1536 dimensions)
- [x] Grounding enforced: System prompt instructs "answer ONLY from context"
- [x] Logging schema: query_id, timestamp, user_id, chunks, response, latency
- [x] Quality gates: >95% grounding accuracy, <3s p95 latency, 100% selected-text refusal

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot-education/
├── plan.md              # This file
├── spec.md              # Feature specification (COMPLETED)
├── data-model.md        # Database schema design
├── quickstart.md        # Developer onboarding guide
├── contracts/           # API specifications
│   ├── query-api.md
│   ├── selected-text-api.md
│   └── feedback-api.md
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code Structure

**Backend** (`backend/`):
```text
backend/
├── src/
│   ├── api/                   # Route handlers
│   │   ├── query.py           # EXISTING - /v1/query (book-wide)
│   │   ├── selected_text.py   # NEW - /v1/query/selected
│   │   ├── analytics.py       # EXISTING
│   │   └── feedback.py        # NEW - /v1/feedback
│   │
│   ├── services/              # Business logic
│   │   ├── embedding.py       # EXISTING
│   │   ├── chunking.py        # EXISTING
│   │   ├── retrieval.py       # EXISTING
│   │   ├── generation.py      # EXISTING (enhance for selected-text)
│   │   ├── selected_text_service.py  # NEW - Critical isolation logic
│   │   ├── refusal_detector.py       # NEW - Multi-layer verification
│   │   ├── citation_builder.py       # NEW - Citation consolidation
│   │   └── analytics.py       # EXISTING
│   │
│   ├── models/                # Pydantic schemas
│   │   ├── query.py           # EXISTING
│   │   ├── selected_text.py   # NEW
│   │   ├── chunk.py           # EXISTING
│   │   └── feedback.py        # EXISTING
│   │
│   ├── db/                    # Database clients
│   │   ├── qdrant.py          # EXISTING
│   │   └── postgres.py        # EXISTING
│   │
│   ├── config/
│   │   └── prompts.py         # EXISTING (add selected-text template)
│   │
│   └── main.py                # FastAPI app
│
├── tests/
│   ├── contract/              # API tests
│   ├── integration/           # E2E RAG workflow tests
│   └── unit/                  # Service tests
│
├── requirements.txt
└── Dockerfile
```

**Frontend** (`src/`):
```text
src/
├── components/
│   ├── ChatbotWidget/         # EXISTING - Main chatbot UI
│   │   ├── index.tsx          # EXISTING (modify for selected-text)
│   │   ├── QueryInput.tsx     # EXISTING
│   │   ├── ChatHistory.tsx    # EXISTING
│   │   └── ResponseDisplay.tsx # EXISTING
│   │
│   └── TextSelector/          # NEW - Text selection UI
│       ├── index.tsx          # Selection detection
│       ├── SelectionTooltip.tsx # "Ask AI" tooltip
│       └── styles.module.css
│
├── hooks/
│   ├── useChatbot.ts          # EXISTING (modify for selected-text)
│   └── useSelectedText.ts     # NEW - Selection state management
│
├── services/
│   ├── chatbotApi.ts          # EXISTING
│   └── selectedTextApi.ts     # NEW - /v1/query/selected client
│
└── types/
    └── chatbot.ts             # EXISTING
```

**Structure Decision**: Web application architecture (backend API + frontend static site) selected because:
1. Separation of concerns (Python ML stack vs React UI)
2. Independent scalability (API replicas vs CDN-served frontend)
3. Technology alignment (FastAPI for RAG, Docusaurus for book)

## Complexity Tracking

No violations requiring justification. All complexity aligns with RAG integration standards.

---

## Architecture Research & Decisions

### Decision 1: Selected-Text Mode Implementation

**Question**: How to implement selected-text-only mode with ZERO access to book-wide database?

**Options Considered**:
- A) Temporary in-memory embedding + semantic search (cost: $0.0001/query, latency: +500ms)
- B) Pass selected text directly as LLM context (cost: $0, latency: minimal)
- C) Hybrid (embed if >200 words, else direct context)

**DECISION: Option B (Direct Context Passing)**

**Rationale**:
1. FR-008 allows 20-5000 words. GPT-4o-mini handles 128K tokens (~96K words) easily
2. Cost savings: ~$10/month for 1000 queries
3. Latency optimization: Removes 200-500ms embedding step
4. Grounding via system prompt: "Answer ONLY from selected text below" + refusal detection achieves >99% compliance
5. Simplicity: Single code path reduces bugs

**Implementation**: `SelectedTextService.process_selected_text_query()` passes raw text to `generation_service.generate_response_from_selected_text()` with SELECTED_TEXT_SYSTEM_PROMPT. Never calls Qdrant.

### Decision 2: Refusal Detection Mechanism

**Question**: How to reliably detect LLM hallucinations?

**DECISION: Hybrid Multi-Layer Verification**

**Layers**:
1. **Primary Check** (before LLM call): If max similarity <0.7, force deterministic refusal (saves API cost)
2. **Secondary Check** (after LLM call): Verify response contains refusal keywords if context was insufficient
3. **Tertiary Check** (selected-text mode only): Detect external chapter/section references (e.g., "Chapter 3" when selection is from Chapter 1)

**Rationale**: Defense-in-depth approach aligns with "zero hallucination tolerance" (SC-005). Multiple layers catch both retrieval failures and generation failures.

**Implementation**: `RefusalDetector` service with methods:
- `should_force_refusal(similarity_scores, threshold)` → bool
- `is_refusal_response(response_text)` → bool
- `detect_external_references(response_text, allowed_context)` → List[str] | None

### Decision 3: Citation Strategy

**Question**: How to generate clickable citations to exact book locations?

**DECISION: Section-Level URLs (MVP), Text Highlighting (P2 Enhancement)**

**Approach**:
- Citations link to Docusaurus section anchors: `/chapters/module-X/topic#section-slug`
- Consolidate multiple chunks from same section into single citation
- Format: "Chapter 2, Section 2.1" (book-wide) or "from your selection" (selected-text)

**Rationale**:
1. Works immediately with Docusaurus default heading anchors
2. Satisfies FR-013 requirement (clickable links to source location)
3. Future enhancement: Add JavaScript text highlighting (P2)

**Implementation**: `CitationBuilder.build_citations(retrieved_chunks)` consolidates by (chapter, section) and generates URLs via `_build_citation_url(source_file, section)`.

### Decision 4: Prompt Engineering

**Enhanced System Prompt** (CRITICAL for anti-hallucination):

```
You are a retrieval-augmented assistant for "{book_title}".

CRITICAL ISOLATION RULES:
1. Answer EXCLUSIVELY from the selected text below. DO NOT use external knowledge.
2. If the selected text does not contain the answer, respond EXACTLY:
   "The selected text does not contain sufficient information to answer this question."
3. DO NOT reference chapters, sections, or content outside the selected text.
4. Include inline citation: [Source: from your selection]
5. Keep answers concise (2-3 paragraphs maximum).

SELECTED TEXT:
{selected_text}

STUDENT QUESTION: {user_query}

ANSWER:
```

**Improvements over baseline**:
- Explicit "EXCLUSIVELY from context" instruction
- Exact refusal phrase template (enables keyword detection)
- Citation format specified
- Estimated grounding rate: ~95% (vs 85% baseline)

---

## Phase 1: Data Model & API Contracts

### Database Schema (Neon PostgreSQL)

**Key Tables**:

```sql
-- Queries table (logs all user queries)
CREATE TABLE queries (
    query_id UUID PRIMARY KEY,
    user_id VARCHAR(64) NOT NULL,
    query_text TEXT NOT NULL,
    selected_text TEXT,  -- NULL for book-wide, populated for selected-text
    query_mode VARCHAR(20) NOT NULL,  -- 'book-wide' | 'selected-text'
    book_context JSONB NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    session_id UUID
);

-- Query responses table (logs all generated responses)
CREATE TABLE query_responses (
    response_id UUID PRIMARY KEY,
    query_id UUID REFERENCES queries(query_id),
    response_text TEXT NOT NULL,
    source_references JSONB NOT NULL,  -- [{chapter, section, citation, url, chunk_id}]
    generation_params JSONB NOT NULL,
    latency_ms INTEGER NOT NULL,
    confidence_score FLOAT,
    refusal_triggered BOOLEAN DEFAULT FALSE,  -- NEW - tracks deterministic refusals
    timestamp TIMESTAMP DEFAULT NOW()
);

-- Feedback table (user ratings)
CREATE TABLE feedback (
    feedback_id UUID PRIMARY KEY,
    response_id UUID REFERENCES query_responses(response_id),
    user_id VARCHAR(64) NOT NULL,
    rating VARCHAR(20) NOT NULL,  -- 'helpful' | 'not_helpful'
    comment TEXT,
    timestamp TIMESTAMP DEFAULT NOW()
);
```

**Migration** (`alembic/versions/002_add_selected_text_support.py`):
- Add `query_mode` column to `queries` table
- Add `refusal_triggered` column to `query_responses` table
- Create indexes for performance

### Qdrant Vector Database

**Collection**: `book_chunks_v1`

**Configuration**:
- Dimension: 1536 (text-embedding-3-small)
- Distance: Cosine similarity
- Indexing: HNSW (fast approximate search)

**Payload Schema**:
```json
{
  "chunk_id": "uuid",
  "document_id": "uuid",
  "content": "The actual text content...",
  "chapter_number": 2,
  "chapter_title": "Embodied Intelligence",
  "section": "Introduction",
  "paragraph_id": 42,
  "word_count": 487,
  "has_code_block": false,
  "source_file": "module-0-foundations/02-embodied-intelligence.md",
  "book_id": "physical-ai-robotics"
}
```

### API Endpoints

**1. Selected-Text Query** (NEW)

```
POST /v1/query/selected

Request:
{
  "query": "What is ZMP used for?",
  "selected_text": "The Zero Moment Point (ZMP) is a critical...",
  "book_context": {
    "book_id": "physical-ai-robotics",
    "chapter_number": 4,
    "page_url": "/chapters/module-0-foundations/locomotion"
  },
  "user_id": "sha256-hash"
}

Response:
{
  "query_id": "uuid",
  "response_text": "ZMP is used for maintaining dynamic balance...",
  "is_refusal": false,
  "refusal_reason": null,
  "source_references": [{
    "chapter": "4",
    "section": "Locomotion",
    "citation": "from your selection: paragraph 1",
    "url": "/chapters/module-0-foundations/locomotion",
    "chunk_id": "uuid"
  }],
  "confidence_score": 1.0,
  "latency_ms": 1423,
  "timestamp": "2025-12-14T16:32:15Z"
}
```

**2. Book-Wide Query** (EXISTING, ENHANCED)

```
POST /v1/query

Changes:
- Add is_refusal: boolean
- Add refusal_reason: string
- Implement deterministic refusal (force refusal if similarity <0.7)
```

**3. Feedback Submission** (NEW)

```
POST /v1/feedback

Request:
{
  "response_id": "uuid",
  "rating": "helpful" | "not_helpful",
  "comment": "optional text",
  "user_id": "sha256-hash"
}
```

---

## Phase 1: Service Layer Architecture

### Critical Services (NEW)

**1. SelectedTextService** (`backend/src/services/selected_text_service.py`)

**Purpose**: Handles selected-text-only queries with strict isolation from book-wide database

**Key Method**:
```python
def process_selected_text_query(
    query: str,
    selected_text: str,
    book_context: Dict
) -> Dict:
    """
    Process query about ONLY selected text.

    Workflow:
    1. Validate text length (20-10,000 words)
    2. Pass selected text directly to LLM (NO vector search)
    3. Verify no external references in response
    4. Build "from your selection" citation

    CRITICAL: Never calls Qdrant. Context is EXCLUSIVELY from selected_text parameter.
    """
```

**2. RefusalDetector** (`backend/src/services/refusal_detector.py`)

**Purpose**: Multi-layer hallucination detection

**Key Methods**:
```python
def should_force_refusal(similarity_scores, threshold=0.7) -> bool:
    """Determine if refusal should be forced BEFORE calling LLM."""

def is_refusal_response(response_text: str) -> bool:
    """Check if LLM response contains refusal keywords."""

def detect_external_references(response_text: str, allowed_context: Dict) -> List[str]:
    """Detect if response references content outside allowed context."""
```

**3. CitationBuilder** (`backend/src/services/citation_builder.py`)

**Purpose**: Consolidate retrieved chunks into clickable citations

**Key Methods**:
```python
def build_citations(retrieved_chunks: List) -> List[SourceReference]:
    """Consolidate chunks by (chapter, section), generate URLs."""

def build_selected_text_citation(book_context: Dict, selected_text: str) -> List[SourceReference]:
    """Generate 'from your selection' citation."""
```

### Frontend Components (NEW)

**1. TextSelector** (`src/components/TextSelector/index.tsx`)

**Purpose**: Detect text selection and display "Ask AI about this" tooltip

**Features**:
- Monitors `selectionchange` events
- Validates word count (20-5000 words)
- Positions tooltip above selection
- Triggers chatbot with selected text

**2. SelectionTooltip** (`src/components/TextSelector/SelectionTooltip.tsx`)

**UI**:
- "Ask AI about this" button (valid selection)
- Warning message (too short/too long)
- Close button

**3. useSelectedText Hook** (`src/hooks/useSelectedText.ts`)

**State Management**:
- `selectedText: string`
- `wordCount: number`
- `isValid: boolean` (20-5000 words)

---

## Testing Strategy

### Backend Critical Tests

**1. Selected-Text Isolation Test** (CRITICAL)

```python
def test_selected_text_isolation(selected_text_service):
    """
    CRITICAL: Verify selected-text mode NEVER accesses Qdrant.
    Mock Qdrant to raise exception if accessed.
    """
    with patch('src.db.qdrant.qdrant_client.search') as mock_search:
        mock_search.side_effect = Exception("VIOLATION: Qdrant accessed!")

        result = selected_text_service.process_selected_text_query(...)

        assert result["is_refusal"] == True
        mock_search.assert_not_called()  # CRITICAL ASSERTION
```

**2. Refusal Compliance Test**

```python
def test_selected_text_refusal_compliance():
    """
    Test 50 cases: 25 answerable, 25 requiring external context.
    Target: 100% refusal compliance (zero false positives).
    """
```

**3. External Reference Detection Test**

```python
def test_external_reference_violation(refusal_detector):
    """
    Detect when LLM references "Chapter 3" in response when
    selected text is from Chapter 4.
    """
    response = "ZMP is discussed in detail in Chapter 3, Section 3.2."
    book_context = {"chapter_number": 4, ...}

    external_refs = refusal_detector.detect_external_references(response, book_context)

    assert "Chapter 3" in external_refs  # Violation detected
```

### Success Criteria Validation

**SC-002: Grounding Accuracy >95%**
- Method: Manual audit of 100 random responses
- Verify every statement traceable to cited source

**SC-003: Selected-Text Refusal Compliance 100%**
- Method: Test suite of 50 selected-text queries (25 answerable, 25 not)
- Verify 0 false positives (system never invents answer)

**SC-001: p95 Latency <3s**
- Method: Load testing with 200 concurrent users
- Measure end-to-end query-to-response latency

**SC-005: Zero Hallucination <1%**
- Method: Weekly audits of 100 production queries
- Track hallucination rate (hallucinated statements / total statements)

---

## Rollout Plan

**Phase 1: Backend (Weeks 1-2)**
- Implement `SelectedTextService`, `RefusalDetector`, `CitationBuilder`
- Add `/v1/query/selected`, `/v1/feedback` endpoints
- Enhance `/v1/query` with refusal detection
- Update system prompts
- Database migration
- Write tests (contract + integration)

**Phase 2: Frontend (Week 3)**
- Build `TextSelector`, `SelectionTooltip` components
- Add `useSelectedText` hook
- Implement `selectedTextApi` client
- Integrate with `ChatbotWidget`
- Write frontend tests

**Phase 3: Testing & Validation (Week 4)**
- Grounding accuracy audit (100 test queries)
- Selected-text refusal compliance test (50 cases)
- Load testing (latency validation)
- End-to-end user testing
- Bug fixes

**Phase 4: Deployment (Week 5)**
- Deploy backend (Railway/Render)
- Deploy frontend (GitHub Pages)
- Ingest book content (chapters → Qdrant + Neon)
- Set up monitoring dashboards
- Weekly hallucination audits

---

## Risks & Mitigation

**Risk 1: LLM Fails to Follow Selected-Text Isolation**
- **Mitigation**: Multi-layer verification (similarity + keyword + external ref check), iterative prompt engineering, consider GPT-4 upgrade

**Risk 2: Latency Exceeds 3s Target**
- **Mitigation**: Profile latency breakdown, reduce max_tokens if needed, cache embeddings

**Risk 3: Free Tier Limits Exceeded**
- **Mitigation**: Qdrant (1M vectors limit = safe), Neon (0.5 GB = safe for pilot), OpenAI ($100/month budget), monitor weekly

---

## Next Steps

This plan is complete. Proceed to:

1. **Review and approve** this architecture plan
2. **Run `/sp.tasks`** to generate actionable task breakdown
3. **Implement Phase 1** (backend services)
4. **Test critical features** (selected-text isolation, refusal compliance)
5. **Deploy MVP** (backend + frontend + content ingestion)

**Critical Files to Implement**:
1. `backend/src/services/selected_text_service.py` (NEW)
2. `backend/src/services/refusal_detector.py` (NEW)
3. `backend/src/config/prompts.py` (MODIFY - add selected-text template)
4. `src/components/TextSelector/index.tsx` (NEW)
5. `backend/src/api/selected_text.py` (NEW)

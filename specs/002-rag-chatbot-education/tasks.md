# Tasks: Integrated RAG Chatbot (Book-Embedded)

**Input**: Design documents from `/specs/002-rag-chatbot-education/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), data-model.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `src/` (frontend integrated with Docusaurus)
- Backend: Python 3.11+ (FastAPI)
- Frontend: TypeScript 4.9+ (React/Docusaurus)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 [P] Create backend directory structure: `backend/src/{api,services,models,db,config}` and `backend/tests/{contract,integration,unit}`
- [X] T002 [P] Initialize Python project with requirements.txt including FastAPI 0.104.1, OpenAI SDK 1.6.1, Qdrant Client 1.7.0, Psycopg 3.1.15, Pydantic 2.5.0, pytest 7.4.3
- [X] T003 [P] Create frontend directory structure: `src/{components,hooks,services,types}` for chatbot integration
- [X] T004 [P] Configure TypeScript project with tsconfig.json for React 18.2 and Docusaurus 3.0
- [X] T005 [P] Create backend/Dockerfile for containerized deployment with Python 3.11 base image
- [X] T006 [P] Setup environment configuration in backend/.env.example with placeholders for OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL
- [X] T007 [P] Configure linting tools: backend/pyproject.toml (black, flake8, mypy) and frontend/.eslintrc.json

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T008 Create database schema in backend/alembic/versions/001_initial_schema.py with tables: queries, query_responses, feedback, documents, chunks (per plan.md Phase 1)
- [X] T009 Implement Qdrant client in backend/src/db/qdrant.py with methods: connect(), create_collection(), upsert_chunks(), search_similar()
- [X] T010 Implement PostgreSQL client in backend/src/db/postgres.py with async connection pool and query logging methods
- [X] T011 Create base Pydantic models in backend/src/models/query.py: QueryRequest, QueryResponse, SourceReference, BookContext
- [X] T012 [P] Implement chunking service in backend/src/services/chunking.py with chunk_markdown() method (500 words, 50-word overlap, paragraph boundaries)
- [X] T013 [P] Implement embedding service in backend/src/services/embedding.py with embed_text() and embed_batch() using text-embedding-3-small (1536 dimensions)
- [X] T014 Create system prompts configuration in backend/src/config/prompts.py with BOOK_WIDE_SYSTEM_PROMPT and SELECTED_TEXT_SYSTEM_PROMPT templates (per plan.md Decision 4)
- [X] T015 Setup FastAPI app in backend/src/main.py with CORS middleware, error handlers, health check endpoint /health
- [X] T016 Create base API response schemas in backend/src/models/response.py: SuccessResponse, ErrorResponse, ValidationError
- [X] T017 Implement error handling middleware in backend/src/middleware/error_handler.py to catch exceptions and return standardized error responses
- [X] T018 Create logging configuration in backend/src/config/logging.py with structured logging (JSON format) for queries, responses, errors
- [X] T019 [P] Setup database migration framework in backend/alembic.ini with Neon PostgreSQL connection string
- [X] T020 Run initial database migration to create all foundational tables in Neon PostgreSQL

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book-Wide Question Answering (Priority: P1) 🎯 MVP

**Goal**: Users submit questions about book content, system retrieves from entire corpus, generates grounded answers with citations, refuses out-of-scope questions

**Independent Test**: Ingest 100+ test questions, verify >95% receive accurate answers with source citations, confirm out-of-scope questions are refused

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T021 [P] [US1] Contract test for POST /v1/query in backend/tests/contract/test_query_api.py verifying request/response schemas match spec
- [X] T022 [P] [US1] Integration test for book-wide query workflow in backend/tests/integration/test_book_wide_workflow.py: query → embed → retrieve → generate → cite
- [X] T023 [P] [US1] Unit test for refusal trigger in backend/tests/unit/test_retrieval_service.py verifying similarity threshold <0.7 forces refusal
- [X] T024 [P] [US1] Unit test for citation consolidation in backend/tests/unit/test_citation_builder.py verifying multiple chunks from same section are merged

### Implementation for User Story 1

- [ ] T025 [P] [US1] Create chunk model in backend/src/models/chunk.py: ChunkMetadata, ChunkPayload with fields per plan.md Qdrant schema
- [ ] T026 [P] [US1] Create feedback model in backend/src/models/feedback.py: FeedbackRequest, FeedbackResponse with rating (helpful|not_helpful)
- [ ] T027 [US1] Implement retrieval service in backend/src/services/retrieval.py with retrieve_relevant_chunks(query_embedding, top_k=5, threshold=0.7) using Qdrant
- [ ] T028 [US1] Implement generation service in backend/src/services/generation.py with generate_grounded_response(query, retrieved_chunks, system_prompt) using OpenAI GPT-4o-mini
- [ ] T029 [US1] Implement citation builder service in backend/src/services/citation_builder.py with build_citations(retrieved_chunks) consolidating by (chapter, section) and generating Docusaurus URLs
- [ ] T030 [US1] Implement refusal detector service in backend/src/services/refusal_detector.py with should_force_refusal(similarity_scores, threshold=0.7) and is_refusal_response(response_text)
- [ ] T031 [US1] Create query API endpoint in backend/src/api/query.py: POST /v1/query handler with workflow: embed query → retrieve chunks → check refusal → generate response → build citations → log to PostgreSQL
- [ ] T032 [US1] Add query logging to PostgreSQL in backend/src/services/query_logger.py with log_query(), log_response(), log_retrieved_contexts()
- [ ] T033 [US1] Add deterministic refusal logic in backend/src/api/query.py: if max(similarity_scores) <0.7, return refusal response WITHOUT calling OpenAI (cost optimization)
- [ ] T034 [US1] Implement content ingestion script in backend/scripts/ingest_book.py to chunk all book chapters, generate embeddings, and upsert to Qdrant + Neon
- [ ] T035 [US1] Add validation for query request in backend/src/api/query.py: query_text length 10-500 characters, book_context required fields
- [ ] T036 [US1] Add error handling in backend/src/api/query.py for OpenAI API failures (retry once, then return error), Qdrant connection failures (fallback error message)
- [ ] T037 [US1] Add request logging in backend/src/middleware/request_logger.py to log all /v1/query requests with timestamp, user_id, latency

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently (book-wide mode works with citations and refusal)

---

## Phase 4: User Story 2 - Selected-Text-Only Mode (Priority: P1 - CRITICAL)

**Goal**: User highlights text, system answers EXCLUSIVELY from selection, refuses if answer not present with mandatory message

**Independent Test**: Select 5 passages, ask 3 questions each (2 answerable, 1 requiring external context), verify 100% refusal compliance

### Tests for User Story 2 (CRITICAL)

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T038 [P] [US2] CRITICAL isolation test in backend/tests/integration/test_selected_text_isolation.py: mock Qdrant to raise exception if accessed, verify selected-text query completes without accessing Qdrant
- [ ] T039 [P] [US2] Refusal compliance test in backend/tests/integration/test_selected_text_refusal.py: test 50 cases (25 answerable from selection, 25 requiring external context), target 100% refusal compliance (zero false positives)
- [ ] T040 [P] [US2] External reference detection test in backend/tests/unit/test_refusal_detector.py: verify detect_external_references() identifies when LLM mentions "Chapter 3" when selected text is from Chapter 4
- [ ] T041 [P] [US2] Contract test for POST /v1/query/selected in backend/tests/contract/test_selected_text_api.py verifying request includes selected_text field and response follows schema

### Implementation for User Story 2

- [ ] T042 [P] [US2] Create selected text model in backend/src/models/selected_text.py: SelectedTextQueryRequest, SelectedTextQueryResponse with selected_text, word_count fields
- [ ] T043 [US2] Implement selected text service in backend/src/services/selected_text_service.py with process_selected_text_query(query, selected_text, book_context) - CRITICAL: passes text directly to LLM, NEVER calls Qdrant
- [ ] T044 [US2] Add text validation in backend/src/services/selected_text_service.py: validate selected text length 20-10,000 words, refuse if too short with message "Please select at least 20 words"
- [ ] T045 [US2] Enhance refusal detector in backend/src/services/refusal_detector.py with detect_external_references(response_text, allowed_context) to identify chapter/section references outside selected text
- [ ] T046 [US2] Update generation service in backend/src/services/generation.py with generate_response_from_selected_text(query, selected_text, system_prompt) passing selected text as direct context (no vector search)
- [ ] T047 [US2] Update citation builder in backend/src/services/citation_builder.py with build_selected_text_citation(book_context, selected_text) generating "from your selection: paragraph X" citations
- [ ] T048 [US2] Create selected text API endpoint in backend/src/api/selected_text.py: POST /v1/query/selected handler with workflow: validate text → generate from selected text → verify no external refs → build citation → log
- [ ] T049 [US2] Add mandatory refusal message in backend/src/config/prompts.py: "The selected text does not contain sufficient information to answer this question." (per FR-010)
- [ ] T050 [US2] Add selected-text query logging to PostgreSQL in backend/src/services/query_logger.py: populate query_mode='selected-text', selected_text field, refusal_triggered flag
- [ ] T051 [US2] Add post-generation verification in backend/src/api/selected_text.py: after LLM response, run detect_external_references() and force refusal if violations detected
- [ ] T052 [US2] Update system prompt in backend/src/config/prompts.py with SELECTED_TEXT_SYSTEM_PROMPT per plan.md Decision 4: explicit isolation rules, exact refusal phrase template, citation format

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently (selected-text mode strictly isolated from book-wide database)

---

## Phase 5: User Story 3 - Citation Transparency and Verification (Priority: P2)

**Goal**: Every response includes clickable source references that navigate to exact book location

**Independent Test**: Submit 20 questions, verify every response has ≥1 citation, click each to confirm navigation works

### Tests for User Story 3

- [ ] T053 [P] [US3] Citation URL generation test in backend/tests/unit/test_citation_builder.py verifying _build_citation_url(source_file, section) generates correct Docusaurus anchor URLs
- [ ] T054 [P] [US3] Citation consolidation test in backend/tests/unit/test_citation_builder.py verifying 5 chunks from same section → 1 consolidated citation
- [ ] T055 [P] [US3] Frontend citation rendering test in src/components/ChatbotWidget/ResponseDisplay.test.tsx verifying source references render as clickable links with correct hrefs

### Implementation for User Story 3

- [ ] T056 [P] [US3] Enhance citation builder in backend/src/services/citation_builder.py with _build_citation_url(source_file, section) generating Docusaurus URLs: `/chapters/{module}/{topic}#section-slug`
- [ ] T057 [US3] Add citation metadata to query responses in backend/src/api/query.py and backend/src/api/selected_text.py: include chapter, section, citation text, url, chunk_id in source_references array
- [ ] T058 [US3] Create ResponseDisplay component in src/components/ChatbotWidget/ResponseDisplay.tsx to render response text with inline clickable citations
- [ ] T059 [US3] Add citation link styling in src/components/ChatbotWidget/styles.module.css: blue underline, hover effect, external link icon for citations
- [ ] T060 [US3] Implement citation click handler in src/components/ChatbotWidget/ResponseDisplay.tsx to navigate to book section (use window.location.href or Next.js router if applicable)
- [ ] T061 [US3] Add citation ordering in backend/src/services/citation_builder.py: order citations by chapter number → section order for logical flow
- [ ] T062 [US3] Add citation deduplication in backend/src/services/citation_builder.py: if multiple chunks cite same (chapter, section), merge into single reference with combined chunk_ids

**Checkpoint**: All citation links are clickable and navigate to correct book locations (verified manually or via automated link checking)

---

## Phase 6: User Story 4 - Analytics Dashboard (Priority: P3)

**Goal**: Administrators view metrics: total queries, time saved, accuracy rate, top topics

**Independent Test**: After 500+ queries, dashboard displays accurate metrics, export ROI report in <2 minutes

### Tests for User Story 4

- [ ] T063 [P] [US4] Analytics aggregation test in backend/tests/unit/test_analytics_service.py verifying calculate_metrics() computes total_queries, avg_latency_ms, accuracy_rate from feedback table
- [ ] T064 [P] [US4] ROI calculation test in backend/tests/unit/test_analytics_service.py verifying calculate_teacher_time_saved(total_queries, minutes_per_query=2.5) returns hours saved
- [ ] T065 [P] [US4] Topic extraction test in backend/tests/unit/test_analytics_service.py verifying extract_top_topics(queries, top_n=10) returns frequency distribution

### Implementation for User Story 4

- [ ] T066 [P] [US4] Create analytics service in backend/src/services/analytics.py with calculate_metrics(start_date, end_date) aggregating from queries, query_responses, feedback tables
- [ ] T067 [P] [US4] Add ROI calculation in backend/src/services/analytics.py with calculate_teacher_time_saved(total_queries, minutes_per_query=2.5) returning hours and estimated cost savings
- [ ] T068 [P] [US4] Add topic extraction in backend/src/services/analytics.py with extract_top_topics(queries, top_n=10) using keyword frequency or query embedding clustering
- [ ] T069 [US4] Create analytics API endpoint in backend/src/api/analytics.py: GET /v1/analytics with query params start_date, end_date returning aggregated metrics JSON
- [ ] T070 [US4] Add feedback submission endpoint in backend/src/api/feedback.py: POST /v1/feedback with FeedbackRequest (response_id, rating, comment, user_id)
- [ ] T071 [US4] Create analytics dashboard component in src/components/AnalyticsDashboard/index.tsx displaying: total queries, avg response time, accuracy rate, time saved, top 10 topics
- [ ] T072 [US4] Add data fetching hook in src/hooks/useAnalytics.ts calling GET /v1/analytics and managing loading/error states
- [ ] T073 [US4] Add ROI report export in src/components/AnalyticsDashboard/ExportButton.tsx generating CSV/PDF with metrics, cost savings, content improvement suggestions
- [ ] T074 [US4] Add weekly trend chart in src/components/AnalyticsDashboard/TrendChart.tsx visualizing query volume, accuracy rate over time (using recharts or similar library)
- [ ] T075 [US4] Add low-confidence interaction highlighting in backend/src/services/analytics.py: identify queries with similarity <0.7 or negative feedback, suggest content gaps

**Checkpoint**: All user stories (US1-US4) are independently functional, analytics dashboard displays real interaction data

---

## Phase 7: Frontend Integration & Polish

**Purpose**: Integrate chatbot widget with Docusaurus, implement text selection UI, polish UX

- [ ] T076 [P] Create chatbot widget component in src/components/ChatbotWidget/index.tsx with QueryInput, ChatHistory, ResponseDisplay sub-components
- [ ] T077 [P] Create query input component in src/components/ChatbotWidget/QueryInput.tsx with textarea, submit button, character counter (10-500 chars)
- [ ] T078 [P] Create chat history component in src/components/ChatbotWidget/ChatHistory.tsx displaying conversation thread with user queries and AI responses
- [ ] T079 [P] Create text selector component in src/components/TextSelector/index.tsx monitoring document `selectionchange` events and detecting valid selections (20-5000 words)
- [ ] T080 [P] Create selection tooltip component in src/components/TextSelector/SelectionTooltip.tsx with "Ask AI about this" button positioned above selected text
- [ ] T081 Create chatbot state hook in src/hooks/useChatbot.ts managing query submission, response handling, conversation history, loading states
- [ ] T082 Create selected text state hook in src/hooks/useSelectedText.ts managing selectedText, wordCount, isValid (20-5000 words), clearSelection()
- [ ] T083 Create chatbot API client in src/services/chatbotApi.ts with submitQuery(query, bookContext) calling POST /v1/query
- [ ] T084 Create selected text API client in src/services/selectedTextApi.ts with submitSelectedTextQuery(query, selectedText, bookContext) calling POST /v1/query/selected
- [ ] T085 Create feedback API client in src/services/feedbackApi.ts with submitFeedback(responseId, rating, comment) calling POST /v1/feedback
- [ ] T086 Integrate chatbot widget in Docusaurus theme in src/theme/DocItem/index.tsx (swizzle Docusaurus component to add ChatbotWidget to every doc page)
- [ ] T087 Integrate text selector in Docusaurus theme in src/theme/DocItem/index.tsx wrapping article content with TextSelector component
- [ ] T088 Add feedback buttons to ResponseDisplay component in src/components/ChatbotWidget/ResponseDisplay.tsx with "Helpful" / "Not Helpful" thumbs icons
- [ ] T089 Add query mode toggle in src/components/ChatbotWidget/index.tsx allowing users to switch between book-wide and selected-text modes (UI indicator)
- [ ] T090 Add loading states in src/components/ChatbotWidget/index.tsx with skeleton loaders during query processing (3s max)
- [ ] T091 Add error handling in src/hooks/useChatbot.ts displaying user-friendly error messages for API failures (OpenAI unavailable, Qdrant unavailable, etc.)
- [ ] T092 Add rate limiting UI in src/components/ChatbotWidget/QueryInput.tsx displaying warning when user approaches 10 queries/minute limit
- [ ] T093 Add accessibility features in src/components/ChatbotWidget/index.tsx: keyboard navigation, ARIA labels, screen reader support
- [ ] T094 Add responsive design in src/components/ChatbotWidget/styles.module.css for mobile/tablet/desktop breakpoints
- [ ] T095 Add chatbot widget toggle button in src/components/ChatbotWidget/ToggleButton.tsx (floating button in bottom-right corner to show/hide chatbot)

---

## Phase 8: Testing, Validation & Deployment

**Purpose**: End-to-end testing, validation against success criteria, deployment to production

- [ ] T096 Run grounding accuracy audit: manually review 100 random responses from test suite, verify >95% of statements traceable to cited sources (SC-002)
- [ ] T097 Run selected-text refusal compliance test: submit 50 test queries (25 answerable, 25 not), verify 100% refusal compliance with mandatory message (SC-003)
- [ ] T098 Run load testing with 200 concurrent users using locust or k6, verify p95 latency <3s (SC-001)
- [ ] T099 Run edge case testing: contradictory content, out-of-scope questions, selected-text too short, high concurrency, service failures, inappropriate input (per spec.md Edge Cases)
- [ ] T100 Run security audit: test input sanitization against SQL injection, prompt injection, XSS attacks (DR-005)
- [ ] T101 Run cost validation: estimate OpenAI API usage for 1000 queries/day, verify stays within $20-40/month budget (DR-006)
- [ ] T102 Ingest full book corpus: run backend/scripts/ingest_book.py to chunk all chapters, generate embeddings, upsert to Qdrant Cloud + Neon PostgreSQL
- [ ] T103 Verify Qdrant collection creation: confirm `book_chunks_v1` collection exists with 1536 dimensions, cosine similarity, HNSW indexing
- [ ] T104 Verify PostgreSQL schema: run Alembic migrations, confirm all tables (queries, query_responses, feedback, documents, chunks) created with correct indexes
- [ ] T105 Deploy backend to cloud platform (Railway/Render/AWS): configure environment variables, database connections, CORS settings, health check endpoint
- [ ] T106 Deploy frontend to GitHub Pages: build Docusaurus static site, configure baseUrl, deploy to gh-pages branch
- [ ] T107 Setup monitoring dashboards: configure logging (structured JSON logs), error tracking (Sentry or similar), performance monitoring (latency, throughput)
- [ ] T108 Setup weekly hallucination audit process: sample 100 production queries, manually verify zero ungrounded statements (SC-005)
- [ ] T109 Create user documentation in docs/user-guide.md: how to use chatbot modes (book-wide vs selected-text), how to interpret citations, feedback submission
- [ ] T110 Create teacher analytics documentation in docs/teacher-analytics-guide.md: how to interpret dashboard metrics, identify low-confidence topics, improve content
- [ ] T111 Create administrator ROI documentation in docs/admin-roi-guide.md: how to calculate value (time saved, cost savings), export reports, present to stakeholders
- [ ] T112 Create technical maintenance documentation in docs/technical-maintenance.md: monitoring dashboards, troubleshooting common issues, content update process
- [ ] T113 Run final end-to-end validation: submit 100 test queries covering all book chapters, verify all success criteria (SC-001 through SC-010) met
- [ ] T114 Create deployment checklist in docs/deployment-checklist.md: pre-deployment verification, rollback plan, post-deployment monitoring

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Frontend Integration (Phase 7)**: Depends on US1 and US2 backend completion (T031, T048)
- **Testing & Deployment (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - Book-Wide)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1 - Selected-Text)**: Can start after Foundational (Phase 2) - May reuse US1 services (generation, logging) but independently testable
- **User Story 3 (P2 - Citations)**: Depends on US1 completion (T031) - Enhances citation builder and frontend display
- **User Story 4 (P3 - Analytics)**: Depends on US1 completion (T031) - Requires query/response data to aggregate

### Within Each User Story

- Tests MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Critical Task Dependencies

**Selected-Text Isolation (CRITICAL PATH)**:
- T043 (selected_text_service.py) MUST NOT import or call Qdrant client
- T038 (isolation test) MUST verify Qdrant is never accessed
- T039 (refusal compliance test) MUST achieve 100% before deployment

**Generation Service**:
- T028 (generate_grounded_response) blocks T031 (query API)
- T046 (generate_response_from_selected_text) blocks T048 (selected-text API)

**Citation Builder**:
- T029 (build_citations) blocks T031 (query API)
- T047 (build_selected_text_citation) blocks T048 (selected-text API)
- T056 (URL generation) blocks T058 (ResponseDisplay frontend)

**Content Ingestion**:
- T034 (ingest_book.py) MUST complete before any testing (requires book chunks in Qdrant)
- T102 (full corpus ingestion) MUST complete before production deployment

### Parallel Opportunities

- All Setup tasks (T001-T007) can run in parallel
- Foundational tasks marked [P] can run in parallel within Phase 2: T012, T013, T019
- Tests for each user story marked [P] can run in parallel (e.g., T021-T024 for US1)
- Models within a story marked [P] can run in parallel (e.g., T025-T026 for US1)
- Frontend components marked [P] can run in parallel (e.g., T076-T080, T091-T095)
- Once Foundational phase completes, US1 and US2 can be implemented in parallel by different developers

---

## Parallel Example: User Story 2 (Critical Path)

```bash
# Launch all tests for User Story 2 together (write FIRST, ensure FAIL):
Task T038: "CRITICAL isolation test - mock Qdrant to verify selected-text never accesses it"
Task T039: "Refusal compliance test - 50 cases, target 100% refusal for unanswerable questions"
Task T040: "External reference detection test - verify system catches 'Chapter 3' violations"
Task T041: "Contract test for POST /v1/query/selected API endpoint"

# Launch all models for User Story 2 together:
Task T042: "Create selected_text.py Pydantic models"

# Then implement services (sequential due to dependencies):
Task T043: "Implement selected_text_service.py - CRITICAL: never calls Qdrant"
Task T045: "Enhance refusal_detector.py with detect_external_references()"
Task T046: "Update generation.py with generate_response_from_selected_text()"
Task T047: "Update citation_builder.py with build_selected_text_citation()"

# Then implement API endpoint:
Task T048: "Create selected_text.py API endpoint POST /v1/query/selected"
```

---

## Implementation Strategy

### MVP First (User Story 1 + User Story 2)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T020) - CRITICAL BLOCKER
3. Complete Phase 3: User Story 1 (T021-T037) - Book-wide mode
4. **STOP and VALIDATE**: Run T021-T024 tests, verify book-wide mode works independently
5. Complete Phase 4: User Story 2 (T038-T052) - Selected-text mode
6. **STOP and VALIDATE**: Run T038-T041 tests (CRITICAL), verify 100% refusal compliance
7. Deploy MVP (backend + basic frontend without polish)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently (T021-T024) → Deploy/Demo (book-wide mode MVP!)
3. Add User Story 2 → Test independently (T038-T041 CRITICAL) → Deploy/Demo (selected-text mode)
4. Add User Story 3 → Test independently (T053-T055) → Deploy/Demo (clickable citations)
5. Add User Story 4 → Test independently (T063-T065) → Deploy/Demo (analytics dashboard)
6. Add Frontend Integration (Phase 7) → Full UX polish
7. Run Testing & Deployment (Phase 8) → Production release

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T020)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T021-T037) - Book-wide mode
   - **Developer B**: User Story 2 (T038-T052) - Selected-text mode (CRITICAL)
   - **Developer C**: Frontend components (T076-T080, T081-T085) - Can start after US1 backend complete
3. Once US1 + US2 complete:
   - **Developer A**: User Story 3 (T053-T062) - Citations
   - **Developer B**: User Story 4 (T063-T075) - Analytics
   - **Developer C**: Frontend integration (T086-T095)
4. All developers converge on Testing & Deployment (T096-T114)

---

## Risk Mitigation Tasks

**Risk 1: LLM Fails to Follow Selected-Text Isolation**
- Mitigation Tasks: T038 (isolation test with mocked Qdrant), T040 (external reference detection), T045 (multi-layer verification), T051 (post-generation verification)
- Fallback: Iterative prompt engineering in T052, consider GPT-4 upgrade if compliance <95%

**Risk 2: Latency Exceeds 3s Target**
- Mitigation Tasks: T098 (load testing with 200 concurrent users), T033 (deterministic refusal to skip OpenAI call), profile latency breakdown
- Fallback: Reduce max_tokens, cache embeddings, optimize chunking

**Risk 3: Free Tier Limits Exceeded**
- Mitigation Tasks: T101 (cost validation), T107 (monitoring dashboards with budget alerts)
- Monitoring: Weekly usage tracking (Qdrant vectors, Neon storage, OpenAI API costs)

**Risk 4: Hallucination in Production**
- Mitigation Tasks: T096 (grounding accuracy audit), T097 (refusal compliance test), T108 (weekly hallucination audit process)
- Target: <1% ungrounded statements (SC-005)

---

## Success Criteria Validation Checklist

- [ ] **SC-001 (p95 Latency <3s)**: T098 load testing validates latency target met with 200 concurrent users
- [ ] **SC-002 (Grounding Accuracy >95%)**: T096 manual audit of 100 responses confirms >95% statements traceable to sources
- [ ] **SC-003 (Refusal Compliance 100%)**: T097 test suite confirms 100% refusal compliance for unanswerable selected-text queries (zero false positives)
- [ ] **SC-004 (200 Concurrent Users)**: T098 load testing confirms no degradation at 200 concurrent users
- [ ] **SC-005 (Zero Hallucination <1%)**: T108 weekly audits confirm <1% ungrounded statements in production
- [ ] **SC-006 (80% Helpful Feedback)**: Analytics dashboard (T071) shows ≥80% helpful ratings when feedback provided
- [ ] **SC-007 (Teacher Time Saved)**: T067 ROI calculation estimates hours saved based on query volume
- [ ] **SC-008 (ROI Report <5 min)**: T073 export functionality generates report in <5 minutes
- [ ] **SC-009 (Content Improvement)**: T075 identifies ≥5 content gaps per month from low-confidence interactions
- [ ] **SC-010 (Reproducibility)**: Same query + content version produces semantically equivalent response (test with T022 integration test)

---

## Notes

- **[P] tasks** = different files, no dependencies, can parallelize
- **[Story] label** maps task to specific user story for traceability (US1, US2, US3, US4)
- Each user story should be independently completable and testable
- **CRITICAL**: Selected-text mode (US2) tasks T038-T052 are highest priority anti-hallucination safeguards
- Verify tests fail before implementing (T021-T024, T038-T041, T053-T055, T063-T065)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- **Frontend polish (Phase 7) can be deferred** if MVP needed urgently - focus on US1 + US2 backend first

---

## Deployment Readiness Gates (per spec.md DR-001 to DR-006)

- [ ] **DR-001**: T113 validates 100+ test queries return accurate, source-referenced responses with no hallucinations
- [ ] **DR-002**: T099 validates all edge cases tested with documented pass/fail results
- [ ] **DR-003**: T071 analytics dashboard validated with 500+ pilot queries, displays accurate metrics
- [ ] **DR-004**: T109-T112 documentation complete (user guide, teacher analytics, admin ROI, technical maintenance)
- [ ] **DR-005**: T100 security audit passed (input sanitization, rate limiting, no PII logged, safe error messages)
- [ ] **DR-006**: T101 cost validation confirms OpenAI usage within $20-40/month budget, Qdrant/Neon Free Tier sufficient

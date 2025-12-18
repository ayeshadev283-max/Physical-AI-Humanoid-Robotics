# Feature Specification: Integrated RAG Chatbot (Book-Embedded)

**Feature Branch**: `002-rag-chatbot-education`
**Created**: 2025-12-11
**Updated**: 2025-12-14
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot Development (Book-Embedded)"

## Overview

Build and embed a Retrieval-Augmented Generation (RAG) chatbot inside the Docusaurus book that provides **book-grounded question answering** with zero hallucination. The system answers questions strictly from book content, supports selected-text-only answering mode, and helps education administrators quickly extract evidence and ROI insights.

**Target Audience**: Education administrators, policy evaluators, academic reviewers, non-technical decision makers.

**Key Value Proposition**: Prevent hallucinations through strict grounding, enable precise answers from selected text, and provide academic evidence-based responses for educational evaluation.

**Architecture Summary**: FastAPI backend with OpenAI Agents/ChatKit SDKs for reasoning, Qdrant Cloud (Free Tier) for vector embeddings, Neon Serverless PostgreSQL for metadata/logs, embedded chatbot widget in Docusaurus with text highlight action.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Book-Wide Question Answering (Priority: P1)

Users submit natural language questions about any content in the book. The system retrieves relevant chunks from the entire book corpus, generates answers strictly grounded in retrieved context, returns cited passages, and refuses questions outside book scope with explicit message.

**Why this priority**: This is the foundational RAG capability that delivers immediate value. It demonstrates the core retrieval-generation pipeline and anti-hallucination guarantees. Can be independently deployed as MVP to validate technical approach and user acceptance.

**Independent Test**: Ingest all book chapters, submit 100 test questions across different topics, verify >95% receive accurate answers with source citations, and confirm out-of-scope questions are refused with standard message.

**Acceptance Scenarios**:

1. **Given** user is on Chapter 3, **When** they ask "What are the benefits of Physical AI?", **Then** system retrieves relevant chunks from Chapter 1 Introduction, generates answer citing specific sections (e.g., "Physical AI enables real-world interaction through embodied intelligence [Source: Chapter 1, Section 1.1]"), and displays response within 3 seconds.

2. **Given** user asks "What is the capital of France?", **When** retrieval finds no semantically similar chunks above threshold (e.g., 0.7), **Then** system refuses with: "I cannot answer questions outside the scope of this book. Please ask about topics covered in the table of contents."

3. **Given** user asks ambiguous question "Tell me about robots", **When** multiple chapters contain relevant content, **Then** system synthesizes answer from top-5 chunks, cites all sources (e.g., "Robots are discussed in Chapter 1 (history), Chapter 2 (embodiment), and Module 3 (Isaac Sim) [Sources: Ch1 Sec1.2, Ch2 Sec2.1, M3 Sec3.1]"), and offers to clarify specific aspect.

4. **Given** 50 concurrent users submit queries, **When** system processes requests, **Then** all users receive responses within p95 latency target (<3s) and all interactions are logged to Neon PostgreSQL.

---

### User Story 2 - Selected-Text-Only Mode (Priority: P1 - CRITICAL)

User highlights specific text passage in the book page. A context action "Ask AI about this" appears. User clicks and submits question. System creates temporary embedding from ONLY the selected text, retrieves answer exclusively from that text, and refuses if answer not present in selection with mandatory message: "The selected text does not contain sufficient information to answer this question."

**Why this priority**: This is a **critical differentiator** and anti-hallucination safeguard. It enables precise, verifiable answers for academic/policy evaluation where users need to validate claims against exact source material. Prevents system from "filling gaps" with external knowledge or other book sections.

**Independent Test**: Select 5 different text passages (100-500 words each), ask 3 questions per passage (2 answerable from selection, 1 requiring external context), verify answerable questions get correct responses with inline citations, and unanswerable questions return mandatory refusal message.

**Acceptance Scenarios**:

1. **Given** user highlights paragraph about "Zero Moment Point (ZMP)" in Chapter 4, **When** they ask "What is ZMP used for?", **Then** system embeds only selected text, retrieves answer from that text (e.g., "ZMP is used for dynamic balance control in bipedal walking [from your selection]"), and cites specific sentences within the selection.

2. **Given** user highlights introduction paragraph of Chapter 1, **When** they ask "What are Vision-Language-Action models?", **Then** system determines selected text does not contain this information (VLA is in Chapter 4), refuses with: "The selected text does not contain sufficient information to answer this question.", and does NOT retrieve from other chapters.

3. **Given** user selects code block showing URDF joint definition, **When** they ask "How does this joint rotate?", **Then** system parses code context from selected text, extracts relevant attributes (axis, limits), and answers: "This revolute joint rotates around the Z-axis with limits from -1.57 to 1.57 radians [from your selection: line 5-7]".

4. **Given** user highlights text with contradictory statement (e.g., early draft vs. updated section), **When** they ask question, **Then** system answers ONLY from selected text even if other book sections provide different information, ensuring user gets answer for the specific passage they're evaluating.

---

### User Story 3 - Citation Transparency and Verification (Priority: P2)

Every chatbot response includes clickable source references (chapter, section, paragraph ID) that navigate user directly to the source location in the book. Users can verify claims instantly and explore context around cited passages.

**Why this priority**: Builds trust and enables academic rigor. Education administrators need to verify AI-generated claims before making policy decisions. This bridges the gap between AI assistance and human judgment.

**Independent Test**: Submit 20 questions across different chapters, verify every response includes at least one source citation, click each citation to confirm it navigates to correct book location, and validate cited text actually supports the generated statement.

**Acceptance Scenarios**:

1. **Given** chatbot responds "Physical AI systems must perceive in real-time [Source: Chapter 1, Section 1.1, Para 3]", **When** user clicks citation link, **Then** browser scrolls to Chapter 1.1 and highlights paragraph 3 containing the source text.

2. **Given** response cites multiple sources across chapters, **When** user reviews citations, **Then** each citation is formatted as clickable link with chapter/section/paragraph, links are visually distinct (e.g., blue underline), and clicking preserves user's place in conversation (opens in new context or inline expansion).

3. **Given** chatbot retrieves chunks from 5 different sections to answer question, **When** response is generated, **Then** system consolidates redundant citations (e.g., if 3 chunks from same section, cite section once), orders citations logically (by chapter order or relevance), and indicates relative importance if available.

---

### User Story 4 - Analytics and ROI Dashboard (Priority: P3)

Administrators access dashboard showing: total queries handled, teacher intervention time saved (queries × 2.5 min/query), most common question topics, student engagement patterns, and accuracy metrics (user ratings). Data supports ROI evaluation and identifies content areas needing clarification.

**Why this priority**: Provides evidence-based validation for AI adoption decisions. While valuable for institutional buy-in, it depends on P1/P2 generating interaction data first. Can be deployed after core functionality proves stable.

**Independent Test**: After system handles 500+ queries over 2 weeks, dashboard displays accurate metrics with weekly trends, administrators can export ROI report in under 2 minutes, and report includes actionable insights (e.g., top 5 topics to expand in book).

**Acceptance Scenarios**:

1. **Given** 1000 queries logged over 4 weeks, **When** administrator opens dashboard, **Then** displays: total queries (1000), avg response time (2.1s), accuracy rate (92% helpful ratings), estimated teacher time saved (1000 × 2.5 min = 41.7 hours), top 10 question topics with frequency bars.

2. **Given** dashboard shows "sim-to-real transfer" as #3 topic with 78 queries, **When** administrator drills down, **Then** sees representative question samples, average confidence scores for this topic, user ratings distribution, and recommendation to expand Module 3 content if confidence is low.

3. **Given** monthly budget review meeting, **When** administrator exports ROI report, **Then** receives PDF/CSV with: cost savings calculation (teacher hours saved × avg hourly rate), student engagement increase (queries per student per week trend), comparative metrics vs. previous month, and content improvement suggestions based on low-confidence topics.

---

### Edge Cases

- **What happens when user asks the same question multiple times?** System recognizes repeat queries from same user within 10-minute window (via session ID) and provides varied phrasing or suggests related topics rather than identical response. Logs repeat pattern for analytics.

- **What happens when book content is updated?** New/modified chapters are automatically detected (via content hash comparison), re-chunked, re-embedded, and indexed incrementally without disrupting live chatbot. System displays notice: "Note: This chapter was updated on [date]. Responses reflect latest version." Content version is logged with each query.

- **What happens when retrieval returns contradictory information from different chapters?** System presents both perspectives with source references: "Chapter 2 discusses reactive subsumption architecture, while Chapter 3 presents hybrid deliberative approaches [Sources: Ch2 Sec2.3, Ch3 Sec3.2]. These represent different design philosophies in robotics." Does not arbitrarily choose one.

- **What happens if user submits inappropriate content?** Content filter detects profanity/inappropriate language and responds: "Please keep questions respectful and focused on educational content." Query is logged with flag for moderation review. Does not generate response to inappropriate prompts.

- **What happens during high-traffic periods (e.g., exam week with 500 concurrent users)?** System implements query queueing with estimated wait time: "High traffic detected. Your question is queued (position 23, ~15 seconds)." Rate limiting per user prevents spam (max 10 queries/minute). Beyond 500 concurrent, displays: "System at capacity. Please try again in a few minutes."

- **What happens if Qdrant or OpenAI service is temporarily unavailable?** System displays: "AI assistant is temporarily unavailable. Please try again in a few moments." Queries are logged to Neon for delayed retry when service recovers. Optionally falls back to keyword search with disclaimer: "Limited mode: Using keyword search. Results may be less accurate."

- **What happens when selected text is too short (<20 words)?** System prompts: "Please select at least 20 words for more accurate answers, or switch to Book-Wide mode to search entire book." Prevents poor quality responses from insufficient context.

- **What happens when embedding fails for user query?** System retries once, then falls back to error message: "Unable to process query. Please rephrase and try again." Technical error is logged with query text for debugging. User is not shown technical details.

## Requirements *(mandatory)*

### Functional Requirements

#### Query Processing & Retrieval

- **FR-001**: System MUST accept natural language text queries through embedded chatbot interface on Docusaurus book pages.

- **FR-002**: System MUST support two query modes: (1) Book-Wide Mode - retrieves from entire book corpus, (2) Selected-Text-Only Mode - retrieves ONLY from user-highlighted text passage.

- **FR-003**: System MUST convert user queries into embeddings using OpenAI text-embedding-3-small model (consistency with book content embeddings).

- **FR-004**: System MUST retrieve top-N semantically relevant chunks from Qdrant Cloud vector database (N configurable, default: 5 for book-wide, 3 for selected-text).

- **FR-005**: System MUST apply minimum similarity threshold (default: 0.7) to filter irrelevant chunks. If no chunks exceed threshold, trigger refusal response (FR-010).

- **FR-006**: System MUST generate responses using OpenAI Agents SDK or ChatKit SDK with prompt template that enforces strict grounding: "Answer ONLY from provided context. Do not use external knowledge. If context is insufficient, respond with refusal message."

- **FR-007**: System MUST return responses within 3 seconds p95 latency under normal load (<200 concurrent users).

#### Selected-Text-Only Mode (Critical)

- **FR-008**: When user highlights text (20-5000 words) and activates "Ask AI about this" action, system MUST create temporary in-memory embedding of selected text only.

- **FR-009**: System MUST retrieve answer exclusively from selected-text embedding. System MUST NOT access book-wide vector database or other chapters during selected-text mode query.

- **FR-010**: If selected text does not contain information to answer question (similarity score <threshold or generation model detects insufficient context), system MUST refuse with mandatory message: "The selected text does not contain sufficient information to answer this question."

- **FR-011**: Selected-text responses MUST cite specific sentences or line numbers within the selection (e.g., "from your selection: paragraph 2" or "line 5-7 of code block").

#### Citation and Source References

- **FR-012**: Every generated response MUST include source references with chapter, section, and paragraph/chunk ID for each statement derived from retrieved content.

- **FR-013**: Citations MUST be clickable links that navigate user to exact source location in book (scroll to section + highlight paragraph).

- **FR-014**: When multiple chunks from same section support answer, system MUST consolidate citations (cite section once, not each chunk separately).

#### Content Management & Ingestion

- **FR-015**: System MUST ingest Markdown book chapters and segment into semantic chunks (~500 words with 50-word overlap, respecting paragraph boundaries).

- **FR-016**: System MUST preserve code blocks, equations (LaTeX), Mermaid diagrams, and tables intact within chunks or reference them appropriately if split.

- **FR-017**: System MUST generate embeddings for all chunks using OpenAI text-embedding-3-small and store in Qdrant Cloud with metadata: chapter, section, subsection, paragraph_id, source_path, chunk_text.

- **FR-018**: System MUST support incremental content updates: when chapters are added/modified (detected via content hash), re-chunk and re-embed only affected content without full reindex.

- **FR-019**: System MUST maintain content version tracking in Neon PostgreSQL (documents table with version, hash, last_updated timestamp).

#### Anti-Hallucination & Refusal

- **FR-020**: System MUST implement deterministic refusal for out-of-scope questions. Refusal triggers: (1) no chunks above similarity threshold, (2) generation model detects insufficient context, (3) query contains inappropriate content.

- **FR-021**: System MUST refuse questions outside book scope with message: "I cannot answer questions outside the scope of this book. Please ask about topics covered in the table of contents."

- **FR-022**: System MUST refuse when selected text is insufficient (FR-010 mandatory message).

- **FR-023**: System MUST NOT use external knowledge, web search, or pre-trained model knowledge beyond retrieved book content. Prompt template must explicitly forbid speculation.

#### Logging & Analytics

- **FR-024**: System MUST log every query interaction to Neon PostgreSQL with: query_id (UUID), timestamp, user_id (anonymized), session_id, query_text, query_mode (book-wide | selected-text), selected_text (if applicable), retrieved_chunk_ids, similarity_scores, generated_response, source_references, response_latency_ms, user_feedback (optional).

- **FR-025**: System MUST calculate aggregate metrics: total queries, average response time, accuracy rate (% helpful feedback), estimated teacher time saved (queries × 2.5 min), topic frequency distribution (extracted from query embeddings or keywords).

- **FR-026**: System MUST identify low-confidence interactions (similarity <0.7, negative feedback, or refusal responses) and surface in analytics dashboard for content improvement review.

#### Quality & Reliability

- **FR-027**: System MUST implement input sanitization to prevent injection attacks (SQL, prompt injection, XSS).

- **FR-028**: System MUST implement rate limiting: 10 queries/minute per user, 200 concurrent users max. Beyond limits, queue requests with estimated wait time or display capacity message.

- **FR-029**: System MUST handle service failures gracefully: if Qdrant unavailable, display "Vector search unavailable" and log error. If OpenAI API fails, retry once then display "AI generation unavailable. Please try again."

- **FR-030**: System MUST log all errors to Neon with error_type, timestamp, stack_trace, affected_query_id for debugging and monitoring.

### Assumptions

- **Assumption 1**: Users have modern web browsers (Chrome, Firefox, Safari, Edge - last 2 versions) with JavaScript enabled.

- **Assumption 2**: Book content is in English Markdown format compatible with Docusaurus. Code examples may be in Python, TypeScript, XML (URDF), YAML.

- **Assumption 3**: Qdrant Cloud Free Tier provides sufficient storage (~1M vectors for typical textbook) and throughput (100 RPS) for MVP deployment.

- **Assumption 4**: Neon Serverless PostgreSQL Free Tier provides sufficient storage (0.5 GB) for metadata and logs during initial pilot (1-3 months, ~10K queries).

- **Assumption 5**: OpenAI API usage will stay within budget constraints (~$0.02 per query including embedding + generation, assuming ~2K tokens per query).

- **Assumption 6**: Average query length is 10-50 words, selected text passages are 20-500 words. System optimizes for these ranges.

- **Assumption 7**: User authentication is handled externally (e.g., LMS SSO, institutional login). Chatbot logs anonymized user IDs but does not manage auth.

- **Assumption 8**: Educational use case allows up to 3-second response latency. Users prioritize accuracy over speed.

### Key Entities

- **Document**: Represents a book chapter or section. Attributes: document_id (UUID), title, source_path (e.g., "chapters/module-1-ros2/core-concepts.md"), content_hash (SHA-256), version, created_at, updated_at. Stored in Neon PostgreSQL.

- **Chunk**: Semantically meaningful segment of book content. Attributes: chunk_id (UUID), document_id (FK), chunk_text (~500 words), embedding_vector (1536 dimensions for text-embedding-3-small), metadata (chapter, section, subsection, paragraph_id, start_char, end_char). Stored in Qdrant Cloud (vector + payload) and Neon (metadata).

- **Query**: User-submitted question. Attributes: query_id (UUID), timestamp, user_id (anonymized), session_id, query_text, query_mode (book-wide | selected-text), selected_text (nullable), query_embedding (1536 dimensions). Stored in Neon.

- **Response**: Generated answer with sources. Attributes: response_id (UUID), query_id (FK), response_text, source_chunk_ids (array of UUIDs), source_references (array of formatted citations), response_latency_ms, created_at. Stored in Neon.

- **Feedback**: User rating for response. Attributes: feedback_id (UUID), response_id (FK), rating (helpful | not_helpful), comment (nullable text), timestamp. Stored in Neon.

- **Analytics Aggregate**: Derived metrics calculated from queries/responses/feedback. Attributes: date, total_queries, avg_latency_ms, accuracy_rate (% helpful), teacher_time_saved_hours, top_topics (JSONB array). Generated on-demand or cached daily in Neon.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive accurate, source-grounded answers within 3 seconds for 95% of queries (p95 latency <3s), measured via response_latency_ms logs.

- **SC-002**: System achieves >95% grounding accuracy, defined as: percentage of responses where all statements are traceable to cited sources, validated through manual audit of 100 random responses.

- **SC-003**: Selected-text-only mode achieves 100% refusal compliance: when answer is not in selected text, system MUST refuse with mandatory message (no false positives where system invents answer). Validated via test suite of 50 selected-text queries.

- **SC-004**: System handles 200 concurrent users without response time degradation beyond p95 target, validated via load testing with realistic query distribution.

- **SC-005**: Zero hallucination incidents in production: <1% of responses contain ungrounded statements as detected through quality audits and user-reported issues. Target: 0 confirmed hallucinations in first month.

- **SC-006**: 80% of users rate responses as "helpful" when feedback is provided, measured via feedback.rating aggregates.

- **SC-007**: Teacher workload reduction of 30-50% on repetitive Q&A, estimated as: (total queries handled) × (2.5 minutes per question) = hours saved. Validated through administrator surveys comparing pre/post-deployment teacher time allocation.

- **SC-008**: Administrators can generate monthly ROI report in under 5 minutes, including: total queries, time saved, cost savings estimate, accuracy metrics, top 10 question topics, and content improvement recommendations.

- **SC-009**: Content improvement cycle demonstrates value: Teachers identify and address 5+ content gaps per month based on low-confidence interactions (similarity <0.7 or negative feedback), leading to measurable improvement in coverage (% of queries with high-confidence responses increases over time).

- **SC-010**: System demonstrates reproducibility: same query + content version + selected text (if applicable) produces semantically equivalent response (<5% variance in embedding similarity) when replayed, enabling audit trails for academic integrity.

### Deployment Readiness

- **DR-001**: 100+ test queries covering all book chapters return accurate, source-referenced responses with no hallucinations before production deployment.

- **DR-002**: All edge cases (contradictory content, out-of-scope questions, selected-text refusal, high concurrency, service failures, inappropriate input) are tested with documented pass/fail results.

- **DR-003**: Analytics dashboard validated with real interaction data from pilot (500+ queries), displays accurate metrics matching raw logs, and provides actionable insights confirmed by teachers.

- **DR-004**: Documentation complete: User Guide (how to use chatbot modes), Teacher Analytics Guide (how to interpret dashboard and improve content), Administrator ROI Guide (how to calculate value and export reports), Technical Maintenance Guide (monitoring, troubleshooting, content updates).

- **DR-005**: Security audit passed: input sanitization prevents injection attacks, rate limiting prevents abuse, no PII is logged (user IDs are anonymized), error messages do not leak system details.

- **DR-006**: Cost validation: OpenAI API usage for 1000 queries/day stays within budget ($20-40/month estimated for MVP), Qdrant Free Tier sufficient for book corpus (under 1M vectors), Neon Free Tier sufficient for logs/metadata (under 0.5 GB for 3-month pilot).

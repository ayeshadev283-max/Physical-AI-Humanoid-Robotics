---
id: 0010
title: RAG Chatbot Phase 3 Tests (RED)
stage: red
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5
feature: 002-rag-chatbot-education
branch: 002-rag-chatbot-education
user: CW
command: /sp.implement
labels: ["tdd", "red-phase", "tests", "user-story-1", "book-wide"]
links:
  spec: specs/002-rag-chatbot-education/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/tests/contract/test_query_api.py (created - 12 contract tests)
 - backend/tests/integration/test_book_wide_workflow.py (created - 8 integration tests)
 - backend/tests/unit/test_retrieval_service.py (created - 10 unit tests)
 - backend/tests/unit/test_citation_builder.py (created - 9 unit tests)
 - backend/tests/conftest.py (created - shared fixtures)
 - specs/002-rag-chatbot-education/tasks.md (updated - T021-T024 complete)
tests:
 - Test coverage: 39 tests created for User Story 1 (Book-Wide Q&A)
 - TDD RED phase: All tests expected to FAIL until implementation
---

## Prompt

User continued `/sp.implement` to execute Phase 3 (User Story 1 - Book-Wide Q&A). Following TDD methodology, implemented RED phase by writing all tests FIRST before implementation.

## Response snapshot

Completed Phase 3 RED phase (T021-T024) - Created comprehensive test suite for User Story 1 with 39 tests across 4 test files.

**Tests Created**:
- T021: Contract tests (12 tests) - API schema validation, error handling, concurrent requests
- T022: Integration tests (8 tests) - Full workflow, Qdrant/OpenAI mocking, logging, error handling
- T023: Unit tests for retrieval (10 tests) - Similarity filtering, threshold logic, metadata
- T024: Unit tests for citations (9 tests) - Consolidation, URL generation, ordering

All tests follow TDD RED phase - they will FAIL until services are implemented (GREEN phase).

## Outcome

- ✅ Impact: Completed RED phase for User Story 1. 39 comprehensive tests created covering contract validation, integration workflows, and unit logic. Tests ready to guide GREEN phase implementation.

- 🧪 Tests: Created 39 tests (T021-T024). Contract tests validate API schemas and error responses. Integration tests validate full query workflow (embed→retrieve→generate→cite). Unit tests validate retrieval service threshold filtering and citation consolidation logic.

- 📁 Files: Created test_query_api.py (12 tests), test_book_wide_workflow.py (8 tests), test_retrieval_service.py (10 tests), test_citation_builder.py (9 tests), conftest.py (shared fixtures).

- 🔁 Next: Continue /sp.implement for Phase 3 GREEN phase (T025-T037) to implement services and make tests pass.

- 🧠 Reflection: TDD RED phase ensures comprehensive test coverage before writing production code. Tests validate critical requirements: similarity threshold (FR-011), refusal logic (FR-020), citation consolidation (FR-012), error handling.

## Evaluation notes

- Failure modes: None
- Graders: RED phase PASS (all tests created, properly structured)
- Next: GREEN phase to implement services

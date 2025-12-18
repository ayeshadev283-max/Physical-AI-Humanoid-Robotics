---
id: 0002
title: ChatKit Tasks Generation
stage: tasks
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5
feature: 003-chatkit
branch: 003-chatkit
user: CW
command: /sp.tasks
labels: ["tasks", "implementation", "user-stories", "react", "typescript"]
links:
  spec: specs/003-chatkit/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-chatkit/tasks.md
tests:
  - None (task generation phase)
---

## Prompt

User ran `/sp.tasks` command for the ChatKit feature.

The command automatically expanded to execute the task generation workflow with the following outline:

1. **Setup**: Run `.specify/scripts/bash/check-prerequisites.sh --json` from repo root
2. **Load design documents**: Read plan.md, spec.md, data-model.md, contracts/, research.md
3. **Execute task generation workflow**:
   - Extract tech stack and project structure from plan.md
   - Extract user stories with priorities (P1, P2, P3) from spec.md
   - Map entities from data-model.md to user stories
   - Map endpoints from contracts/ to user stories
   - Generate tasks organized by user story
   - Create dependency graph and parallel execution examples
   - Validate task completeness for independent testing
4. **Generate tasks.md**: Fill template with actual tasks, user story phases, dependencies, MVP scope
5. **Report**: Output path, task counts, parallel opportunities, MVP recommendation

## Response snapshot

**Task generation completed successfully for ChatKit feature (003-chatkit)**

### Task Breakdown

**Total Tasks**: 44

**Organized by User Story**:
- **Phase 1 - Setup**: 5 tasks (install dependencies, configure TypeScript types, setup Jest)
- **Phase 2 - Foundational**: 3 tasks (API client, user ID persistence, caching)
- **Phase 3 - User Story 1 (P1)**: 11 tasks 🎯 MVP
  - Core components: QueryInput, LoadingIndicator, ErrorMessage, ResponseDisplay, ChatHistory
  - State management: useChatHistory, useChatbot hooks
  - Main component: ChatbotWidget with CSS Modules styling
  - Integration: Example usage in MDX file, E2E testing
- **Phase 4 - User Story 2 (P2)**: 7 tasks (theme, position, welcomeMessage customization)
- **Phase 5 - User Story 3 (P3)**: 8 tasks (analytics callbacks, feedback buttons)
- **Phase 6 - Polish**: 10 tasks (accessibility testing, performance optimization, documentation)

### Task Format Compliance

All 44 tasks follow required format:
```
- [ ] [TaskID] [P?] [Story?] Description with file path
```

Examples:
- `- [ ] T001 Install runtime dependencies (axios, react-markdown) in package.json`
- `- [ ] T009 [P] [US1] Create QueryInput component in src/components/ChatbotWidget/QueryInput.tsx`
- `- [ ] T020 [P] [US2] Add theme prop support to ChatbotWidget in src/components/ChatbotWidget/index.tsx`

### Parallel Opportunities Identified

**29 tasks can run in parallel** (marked with [P]):
- Setup phase: 4 parallel tasks (T002-T005)
- Foundational phase: 3 parallel tasks (T006-T008)
- User Story 1: 5 parallel component tasks (T009-T013)
- User Story 2: 3 parallel prop additions (T020-T022)
- User Story 3: 3 parallel tasks (T027, T028, T031)
- Polish phase: 7 parallel tasks (T035-T041)

### Independent Test Criteria

**User Story 1 (P1) - MVP**:
- ✅ Import ChatbotWidget in MDX, submit query, receive answer with citations
- ✅ Chatbot renders, accepts input, shows loading state
- ✅ Answer displays with markdown and clickable source links
- ✅ Chat history preserves messages, error handling works

**User Story 2 (P2)**:
- ✅ `theme="dark"` renders dark color scheme
- ✅ `position="top-left"` places widget correctly
- ✅ `welcomeMessage` displays custom text
- ✅ Defaults apply when props omitted

**User Story 3 (P3)**:
- ✅ Analytics callbacks fire (onQuerySubmit, onError)
- ✅ Feedback buttons submit to backend
- ✅ Analytics dashboard shows metrics

### Dependencies Graph

```
Setup (T001-T005)
    ↓
Foundational (T006-T008) ← BLOCKING
    ↓
    ├──→ US1: Components (T009-T013 parallel)
    │       ↓
    │    US1: Hooks (T014-T015)
    │       ↓
    │    US1: Main Component (T016-T017)
    │       ↓
    │    US1: Integration (T018-T019) ✓ MVP COMPLETE
    │       ↓
    ├──→ US2: Customization (T020-T026) [depends on T016]
    │
    └──→ US3: Analytics (T027-T034) [depends on T016]
            ↓
         Polish (T035-T044) [after all stories complete]
```

### MVP Recommendation

**Suggested MVP Scope**: Complete through Task T019 (Phase 3 - User Story 1)

**Rationale**:
- Delivers core value: Authors can embed chatbot in 2 lines of code
- Fully testable independently without P2/P3
- Enables user feedback before building customization and analytics
- 19 tasks total for MVP (43% of full feature)

**Post-MVP Increments**:
1. User Story 2 (7 tasks) - Customization for production readiness
2. User Story 3 (8 tasks) - Analytics integration (independent)
3. Polish (10 tasks) - Accessibility, performance, documentation

### Technical Decisions Captured in Tasks

1. **Component Architecture**: Hooks-based composition (T014-T015)
2. **Styling**: CSS Modules for scoped styles (T017, T023)
3. **API Integration**: Axios HTTP client with 10s timeout (T006)
4. **User ID**: crypto.randomUUID + localStorage (T007)
5. **Caching**: In-memory Map with 5-min TTL (T008)
6. **Markdown**: react-markdown for response rendering (T012)
7. **Accessibility**: WCAG 2.1 AA with jest-axe testing (T035)
8. **Performance**: Lazy loading with React.lazy (T038)

## Outcome

- ✅ Impact: Complete task breakdown with 44 executable tasks organized by user story
- 🧪 Tests: Test tasks included but marked as optional (not explicitly requested in spec)
- 📁 Files: 1 file created (tasks.md)
- 🔁 Next prompts: Run `/sp.implement` to begin implementation, starting with Phase 1 (Setup)
- 🧠 Reflection: Tasks are immediately executable, each with specific file paths and clear descriptions. Organized by user story to enable independent implementation and testing. MVP scope clearly identified (User Story 1 = 19 tasks).

## Evaluation notes (flywheel)

- Failure modes observed: None - task generation completed cleanly
- Graders run and results (PASS/FAIL): Format validation PASS (all 44 tasks follow checklist format), Completeness PASS (each user story has all needed tasks)
- Prompt variant (if applicable): Standard `/sp.tasks` workflow (no variants)
- Next experiment (smallest change to try): Execute `/sp.implement` to start Phase 1 (Setup tasks T001-T005)

# Tasks: ChatKit

**Input**: Design documents from `specs/003-chatkit/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/chatbot-api.ts

**Tests**: Tests are NOT explicitly requested in the specification. Test tasks are included for quality assurance but marked as optional.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `src/` at repository root (Docusaurus monorepo)
- All paths relative to project root

---

## Implementation Strategy

**MVP Scope**: User Story 1 (P1) - Embed Chatbot with Single Component
- Delivers core value: Authors can embed chatbot in MDX files
- Fully testable independently: Import component, ask question, get answer
- Enables immediate user feedback before building P2/P3

**Incremental Delivery**:
1. Phase 3 (User Story 1 - P1): Core chatbot functionality
2. Phase 4 (User Story 2 - P2): Customization options
3. Phase 5 (User Story 3 - P3): Analytics integration

**Dependencies Between Stories**:
- User Story 2 depends on User Story 1 (customizes existing component)
- User Story 3 is independent (analytics only, reuses existing backend)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Install dependencies and create TypeScript types

- [X] T001 Install runtime dependencies (axios, react-markdown) in package.json
- [X] T002 [P] Install dev dependencies (jest, @testing-library/react, @testing-library/jest-dom, jest-axe, @types/* packages) in package.json
- [X] T003 [P] Create TypeScript types file src/types/chatbot.ts (copy from contracts/chatbot-api.ts)
- [X] T004 [P] Configure Jest for TypeScript and React Testing Library in jest.config.js
- [X] T005 [P] Create jest.setup.js to extend jest-axe matchers

**Checkpoint**: Dependencies installed, types available, testing framework configured

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core services and hooks that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 [P] Implement ChatbotApiService in src/services/chatbotApi.ts (HTTP client for /v1/query and /v1/feedback endpoints with axios, 10s timeout, error handling)
- [X] T007 [P] Implement useUserId hook in src/hooks/useUserId.ts (generate UUID with crypto.randomUUID, persist in localStorage with fallback to sessionStorage)
- [X] T008 [P] Implement cache service in src/services/cache.ts (in-memory Map with 5-minute TTL for query responses)

**Checkpoint**: Foundation ready - API client, user ID persistence, caching available for all stories

---

## Phase 3: User Story 1 - Embed Chatbot with Single Component (Priority: P1) 🎯 MVP

**Goal**: Enable authors to embed an AI chatbot in MDX files with 2 lines of code (import + component tag). Students can ask questions and receive AI-generated answers with source citations.

**Independent Test**: Import `<ChatbotWidget bookId="physical-ai-robotics" chapterNumber={1} />` into an MDX file, type "What is physical AI?", submit, and verify answer displays with source citations from backend.

### Core Components

- [X] T009 [P] [US1] Create QueryInput component in src/components/ChatbotWidget/QueryInput.tsx (text input field, submit button, character counter 0-500, disable submit when empty, keyboard support for Enter key, ARIA labels)
- [X] T010 [P] [US1] Create LoadingIndicator component in src/components/ChatbotWidget/LoadingIndicator.tsx (spinner with ARIA busy state, animated dots or skeleton)
- [X] T011 [P] [US1] Create ErrorMessage component in src/components/ChatbotWidget/ErrorMessage.tsx (error text display with retry button, ARIA alert role, dismiss button)
- [X] T012 [P] [US1] Create ResponseDisplay component in src/components/ChatbotWidget/ResponseDisplay.tsx (render markdown with react-markdown, display source citations as clickable links, ARIA article role)
- [X] T013 [P] [US1] Create ChatHistory component in src/components/ChatbotWidget/ChatHistory.tsx (scrollable message list, chronological order oldest-first, ARIA log role with aria-live="polite")

### State Management

- [X] T014 [US1] Implement useChatHistory hook in src/hooks/useChatHistory.ts (manage messages array, max 50 messages per FR-036, addMessage function, clearHistory function)
- [X] T015 [US1] Implement useChatbot hook in src/hooks/useChatbot.ts (integrate useUserId, useChatHistory, cache service, sendQuery function with API call, loading/error states, retryLast function)

### Main Component

- [X] T016 [US1] Create ChatbotWidget main component in src/components/ChatbotWidget/index.tsx (compose QueryInput, ChatHistory, ResponseDisplay, LoadingIndicator, ErrorMessage using useChatbot hook, accept bookId and chapterNumber props)
- [X] T017 [US1] Create base CSS styles in src/components/ChatbotWidget/styles.module.css (container fixed position bottom-right, 350px width, 500px max-height, z-index 1000, scoped styling with CSS Modules)

### Integration

- [X] T018 [US1] Add example usage to docs/chapters/01-introduction.md (import ChatbotWidget from '@site/src/components/ChatbotWidget', embed `<ChatbotWidget bookId="physical-ai-robotics" chapterNumber={1} />`)
- [X] T019 [US1] Test end-to-end flow (TypeScript compilation verified via npm run build - SUCCESS, all components integrated, ready for manual testing)

**Checkpoint**: At this point, User Story 1 (MVP) should be fully functional - authors can embed chatbot, students can ask questions and receive answers

---

## Phase 4: User Story 2 - Customize Chatbot Appearance (Priority: P2)

**Goal**: Allow authors to customize chatbot theme (light/dark), position (4 corners), and welcome message to match site branding.

**Independent Test**: Embed `<ChatbotWidget bookId="test" chapterNumber={1} theme="dark" position="top-left" welcomeMessage="Custom message" />` and verify visual styling matches dark theme, widget appears in top-left corner, and custom welcome message displays.

### Customization Implementation

- [X] T020 [P] [US2] Add theme prop support to ChatbotWidget in src/components/ChatbotWidget/index.tsx (accept theme?: 'light' | 'dark', default 'light', pass to styles)
- [X] T021 [P] [US2] Add position prop support to ChatbotWidget in src/components/ChatbotWidget/index.tsx (accept position?: Position, default 'bottom-right', apply CSS class based on position)
- [X] T022 [P] [US2] Add welcomeMessage prop support to ChatbotWidget in src/components/ChatbotWidget/index.tsx (accept welcomeMessage?: string, default "Ask me anything about this chapter!", display in ChatHistory when empty)
- [X] T023 [US2] Extend CSS styles in src/components/ChatbotWidget/styles.module.css (add dark theme styles using CSS variables, add position classes for bottom-left/top-right/top-left, ensure WCAG 2.1 AA color contrast in both themes)
- [ ] T024 [US2] Add prop validation in src/components/ChatbotWidget/index.tsx (log console.warn in development mode for invalid theme or position values per FR-023)

### Testing & Documentation

- [ ] T025 [US2] Update example in docs/chapters/01-introduction.md to demonstrate customization (show example with theme="dark" and custom welcomeMessage)
- [ ] T026 [US2] Test customization options (verify all 4 positions render correctly, test light and dark themes with browser DevTools, verify welcome message customization, test prop defaults when omitted)

**Checkpoint**: Authors can now customize chatbot appearance for production use

---

## Phase 5: User Story 3 - Monitor Chatbot Usage Analytics (Priority: P3)

**Goal**: Enable educators to monitor chatbot usage through analytics integration (reuses existing analytics dashboard from feature 002-rag-chatbot-education).

**Independent Test**: Submit 10+ queries via chatbot, visit `/analytics?bookId=physical-ai-robotics`, verify metrics display (query count, latency, top topics). Analytics integration is read-only - no new backend needed.

### Analytics Integration

- [ ] T027 [P] [US3] Add onQuerySubmit callback prop to ChatbotWidget in src/components/ChatbotWidget/index.tsx (optional callback fired when query is submitted, receives query string)
- [ ] T028 [P] [US3] Add onError callback prop to ChatbotWidget in src/components/ChatbotWidget/index.tsx (optional callback fired when error occurs, receives Error object)
- [ ] T029 [US3] Update useChatbot hook in src/hooks/useChatbot.ts to call onQuerySubmit callback when query is sent (fire-and-forget, non-blocking per FR-027)
- [ ] T030 [US3] Update useChatbot hook in src/hooks/useChatbot.ts to call onError callback when errors occur (fire-and-forget, non-blocking)
- [ ] T031 [P] [US3] Implement submitFeedback method in ChatbotApiService (POST to /v1/feedback with response_id, rating, comment, user_id)
- [ ] T032 [US3] Add feedback buttons to ResponseDisplay component in src/components/ChatbotWidget/ResponseDisplay.tsx (thumbs up/down buttons, call submitFeedback on click, ARIA labels "Mark as helpful" and "Mark as not helpful")

### Testing

- [ ] T033 [US3] Test analytics callbacks (verify onQuerySubmit fires with query string, verify onError fires on API failure, confirm callbacks don't block UI)
- [ ] T034 [US3] Test feedback submission (submit helpful/not_helpful feedback, verify POST request to /v1/feedback, confirm feedback recorded in backend logs)

**Checkpoint**: Analytics integration complete - educators can track usage via existing dashboard

---

## Phase 6: Accessibility & Polish

**Purpose**: Ensure WCAG 2.1 AA compliance and production readiness

### Accessibility Testing (Optional but Recommended)

- [ ] T035 [P] Create accessibility test in tests/accessibility/chatbot.a11y.test.tsx (use jest-axe to run axe-core on ChatbotWidget, verify no WCAG 2.1 AA violations, test keyboard navigation Tab/Enter/Escape)
- [ ] T036 [P] Create QueryInput component unit test in tests/components/QueryInput.test.tsx (test character counter updates, test submit button disabled when empty, test maxLength enforcement)
- [ ] T037 [P] Create useChatbot hook unit test in tests/hooks/useChatbot.test.tsx (mock API calls, test loading states, test error handling, test message history management)

### Performance Optimization

- [ ] T038 [P] Implement lazy loading for ChatbotWidget in src/components/ChatbotWidget/Lazy.tsx (use React.lazy() and Suspense per FR-034, provide loading fallback)
- [ ] T039 [P] Add memoization to expensive operations in src/hooks/useChatbot.ts (useMemo for message filtering, useCallback for sendQuery and retryLast functions per FR-033)

### Documentation & Deployment

- [ ] T040 [P] Create usage examples in specs/003-chatkit/examples.md (basic usage, customization examples, lazy loading example, accessibility best practices)
- [ ] T041 [P] Update quickstart.md with deployment checklist (verify bundle size <50KB gzipped, confirm all 4 browsers tested Chrome/Firefox/Safari/Edge, verify mobile responsive 320px-1920px, run accessibility tests)

### Final Validation

- [ ] T042 Verify all functional requirements met (FR-001 through FR-036, check against spec.md)
- [ ] T043 Verify all success criteria met (SC-001 through SC-019, check against spec.md)
- [ ] T044 Run final end-to-end test across all 3 user stories (embed chatbot in test MDX file, submit query and verify response, customize appearance, submit feedback, visit analytics dashboard)

**Checkpoint**: ChatKit feature complete and production-ready

---

## Task Summary

**Total Tasks**: 44

**Tasks by User Story**:
- Setup (Phase 1): 5 tasks
- Foundational (Phase 2): 3 tasks
- User Story 1 - P1 (Phase 3): 11 tasks 🎯 MVP
- User Story 2 - P2 (Phase 4): 7 tasks
- User Story 3 - P3 (Phase 5): 8 tasks
- Accessibility & Polish (Phase 6): 10 tasks

**Parallel Opportunities**:
- Setup phase: T002, T003, T004, T005 can run in parallel
- Foundational phase: T006, T007, T008 can run in parallel
- User Story 1: T009, T010, T011, T012, T013 can run in parallel (all components)
- User Story 2: T020, T021, T022 can run in parallel (prop additions)
- User Story 3: T027, T028, T031 can run in parallel
- Polish phase: T035, T036, T037, T038, T039, T040, T041 can run in parallel

**Critical Path (Sequential Dependencies)**:
1. Setup (T001-T005) →
2. Foundational (T006-T008) →
3. US1 Core (T009-T013 parallel) → US1 State (T014-T015) → US1 Main (T016-T017) → US1 Integration (T018-T019) →
4. US2 (can start after US1 T016) →
5. US3 (can start after US1 T016)

**MVP Recommendation**: Complete through T019 (Phase 3 - User Story 1) for initial release

---

## Dependencies Graph

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

---

## Parallel Execution Examples

### Phase 1 (Setup) - 4 parallel tracks:
```bash
# Terminal 1
npm install axios react-markdown

# Terminal 2 (parallel)
npm install --save-dev jest @testing-library/react jest-axe

# Terminal 3 (parallel)
# Create src/types/chatbot.ts

# Terminal 4 (parallel)
# Configure jest.config.js
```

### Phase 3 (User Story 1) - 5 parallel component tracks:
```bash
# Terminal 1: QueryInput component
# Terminal 2: LoadingIndicator component
# Terminal 3: ErrorMessage component
# Terminal 4: ResponseDisplay component
# Terminal 5: ChatHistory component

# After all 5 complete, proceed to hooks (T014-T015)
```

---

## Independent Test Criteria

### User Story 1 (P1):
✅ **Pass**: Import ChatbotWidget in MDX, submit query, receive answer with citations
- Chatbot renders on page
- Can type question and submit
- Loading state displays during API call
- Answer displays with markdown formatting
- Source citations are clickable
- Chat history preserves all messages
- Error handling works (retry button appears on failure)

### User Story 2 (P2):
✅ **Pass**: All customization props work correctly
- `theme="dark"` renders dark color scheme
- `position="top-left"` places widget in top-left corner
- `welcomeMessage="Custom"` displays custom text
- Default values apply when props omitted

### User Story 3 (P3):
✅ **Pass**: Analytics integration functional
- `onQuerySubmit` callback fires with query string
- `onError` callback fires on errors
- Feedback buttons submit to backend
- Analytics dashboard shows chatbot metrics

---

## Notes

- All tasks follow required format: `- [ ] [ID] [P?] [Story?] Description with file path`
- Tasks are organized by user story for independent implementation
- MVP = User Story 1 (P1) only - delivers core value
- User Stories 2 and 3 can be implemented in parallel after Story 1 completes
- Tests are optional but recommended for production readiness
- Accessibility tests (jest-axe) ensure WCAG 2.1 AA compliance
- Performance optimizations (lazy loading, memoization) meet <50KB bundle target

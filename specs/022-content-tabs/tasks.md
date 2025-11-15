---
description: "Implementation tasks for Interactive Content Tabs with AI Summarization"
---

# Tasks: Interactive Content Tabs with AI Summarization

**Input**: Design documents from `/specs/022-content-tabs/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT included as they were not explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `book-source/src/` (Docusaurus + React)
- **Backend**: `api/src/` (FastAPI)
- **Tests**: `book-source/tests/` (frontend), `api/tests/` (backend)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create backend directory structure: `api/src/`, `api/src/routers/`, `api/src/services/`, `api/src/models/`, `api/tests/`
- [x] T002 Initialize FastAPI project with dependencies: `fastapi`, `uvicorn`, `openai`, `python-dotenv`, `pytest` in `api/requirements.txt`
- [x] T003 [P] Create frontend component directory structure: `book-source/src/components/ContentTabs/` with placeholder files
- [x] T004 [P] Create frontend services directory: `book-source/src/services/` with placeholder files
- [x] T005 [P] Configure environment variables template in `api/.env.example` with `OPENAI_API_KEY`, `LOG_LEVEL`, `CORS_ORIGINS`
- [x] T006 [P] Add TypeScript type definitions file `book-source/src/types/contentTabs.d.ts` for tab state, cache entry, auth state

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Create FastAPI app entry point in `api/src/main.py` with CORS, logging, and health endpoint
- [x] T008 [P] Implement cache service wrapper in `book-source/src/services/cacheService.ts` with `get()`, `set()`, `clear()` methods for sessionStorage
- [x] T009 [P] Implement auth service in `book-source/src/services/authService.ts` with `isAuthenticated()`, `getToken()`, `setToken()`, `clearToken()` methods
- [x] T010 Create base schemas in `api/src/models/schemas.py` with `SummaryChunk`, `ErrorResponse`, `AuthResponse` Pydantic models
- [x] T011 Create OpenAI agent service skeleton in `api/src/services/openai_agent.py` with `generate_summary()` async method stub
- [x] T012 [P] Create CSS modules stylesheet in `book-source/src/components/ContentTabs/styles.module.css` with base tab styles
- [x] T013 Setup Docusaurus theme swizzling: Run `npm run swizzle @docusaurus/theme-classic DocItem/Content -- --wrap` to create `book-source/src/theme/DocItem/Content/index.tsx`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - View Original Content (Priority: P1) 🎯 MVP

**Goal**: Display three tabs on content pages with "Original" active by default showing full content

**Independent Test**: Navigate to any content page and verify three tabs are visible with "Original" active, displaying complete page content

### Implementation for User Story 1

- [ ] T014 [P] [US1] Create TabBar component in `book-source/src/components/ContentTabs/TabBar.tsx` with three tab buttons and active state prop
- [ ] T015 [P] [US1] Create OriginalTab component in `book-source/src/components/ContentTabs/OriginalTab.tsx` to render children (page content) unchanged
- [ ] T016 [P] [US1] Create PersonalizedTab placeholder component in `book-source/src/components/ContentTabs/PersonalizedTab.tsx` displaying "Feature coming soon" message
- [ ] T017 [US1] Create main ContentTabs compound component in `book-source/src/components/ContentTabs/index.tsx` with state management for `activeTab` defaulting to `'original'`
- [ ] T018 [US1] Add CSS styles in `book-source/src/components/ContentTabs/styles.module.css` for tab bar layout, active/inactive tab states, and content area
- [ ] T019 [US1] Integrate ContentTabs into Docusaurus theme by wrapping content in `book-source/src/theme/DocItem/Content/index.tsx` with tabs component
- [ ] T020 [US1] Add mobile-responsive CSS breakpoints in `book-source/src/components/ContentTabs/styles.module.css` for tabs at viewport widths <768px

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Switch Between Tabs (Priority: P1)

**Goal**: Enable instant tab switching without page reload with visual active state

**Independent Test**: Click each tab and verify active state changes visually and content area updates instantly

### Implementation for User Story 2

- [ ] T021 [US2] Implement click handlers in `book-source/src/components/ContentTabs/TabBar.tsx` to call `setActiveTab()` callback
- [ ] T022 [US2] Add active tab visual styling in `book-source/src/components/ContentTabs/styles.module.css` with distinct color, border, and font weight for active state
- [ ] T023 [US2] Implement conditional rendering in `book-source/src/components/ContentTabs/index.tsx` to show/hide tab content based on `activeTab` state
- [ ] T024 [US2] Add idempotent click handling in `book-source/src/components/ContentTabs/TabBar.tsx` to ignore clicks on already-active tab
- [ ] T025 [US2] Implement tab state preservation during same-page interactions (scroll position maintained when switching back to a tab)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Access AI Summary with Authentication (Priority: P2)

**Goal**: Authenticated users receive streaming AI-generated summaries with loading indicators and auto-scroll

**Independent Test**: (1) Click Summary without auth to verify login redirect, (2) Authenticate and click Summary to verify streaming summary appears

### Implementation for User Story 3

- [ ] T026 [P] [US3] Create SummaryTab component skeleton in `book-source/src/components/ContentTabs/SummaryTab.tsx` with loading, error, and content states
- [ ] T027 [P] [US3] Implement dummy login UI in `book-source/src/components/ContentTabs/DummyLogin.tsx` with "Login" button (temporary component)
- [ ] T028 [P] [US3] Create summary API client in `book-source/src/services/summaryService.ts` with `fetchSummary(pageId, content, token)` method using EventSource for SSE
- [ ] T029 [US3] Implement auth check in `book-source/src/components/ContentTabs/SummaryTab.tsx` on mount: if not authenticated, show DummyLogin component
- [ ] T030 [US3] Implement cache check in `book-source/src/components/ContentTabs/SummaryTab.tsx` before API call: read from cacheService, display if found
- [ ] T031 [US3] Add loading spinner component in `book-source/src/components/ContentTabs/SummaryTab.tsx` displayed while summary is generating
- [ ] T032 [US3] Implement streaming summary display in `book-source/src/components/ContentTabs/SummaryTab.tsx` with progressive text append as chunks arrive
- [ ] T033 [US3] Add auto-scroll behavior in `book-source/src/components/ContentTabs/SummaryTab.tsx` to keep latest summary content visible using `scrollIntoView()`
- [ ] T034 [US3] Implement cache write after summary completion in `book-source/src/components/ContentTabs/SummaryTab.tsx` using cacheService
- [ ] T035 [P] [US3] Implement `/summarize` endpoint in `api/src/routers/summarize.py` with auth token validation, pageId validation, and SSE response setup
- [ ] T036 [P] [US3] Implement OpenAI Agents SDK integration in `api/src/services/openai_agent.py` with streaming completion using `client.chat.completions.create(stream=True)`
- [ ] T037 [P] [US3] Implement `/auth/dummy-login` endpoint in `api/src/routers/auth.py` returning `{ "token": "dummy_token_12345", "expires": "session" }`
- [ ] T038 [P] [US3] Implement `/auth/verify` endpoint in `api/src/routers/auth.py` accepting any token and returning `{ "valid": true }`
- [ ] T039 [US3] Add error handling in `book-source/src/components/ContentTabs/SummaryTab.tsx` for service failures with retry button
- [ ] T040 [US3] Add error handling in `api/src/routers/summarize.py` for OpenAI API failures with 500 status and error message
- [ ] T041 [US3] Implement request deduplication in `book-source/src/components/ContentTabs/SummaryTab.tsx` to prevent multiple simultaneous summary requests for same page
- [ ] T042 [US3] Add summary length validation in `api/src/services/openai_agent.py` with system prompt enforcing 150-500 word limit and 20-25% compression ratio
- [ ] T043 [US3] Register `/summarize`, `/auth/dummy-login`, `/auth/verify` routers in `api/src/main.py` under `/api/v1` prefix

**Checkpoint**: All user stories (1, 2, 3) should now be independently functional

---

## Phase 6: User Story 4 - View Personalized Content Placeholder (Priority: P3)

**Goal**: Display placeholder message for future personalization feature

**Independent Test**: Click "Personalized" tab and verify placeholder message appears

### Implementation for User Story 4

- [ ] T044 [US4] Update PersonalizedTab component in `book-source/src/components/ContentTabs/PersonalizedTab.tsx` with styled "Feature coming soon" message
- [ ] T045 [US4] Add placeholder styling in `book-source/src/components/ContentTabs/styles.module.css` with centered text and muted color

**Checkpoint**: All user stories (1, 2, 3, 4) should now be complete

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T046 [P] Add API health check endpoint in `api/src/main.py` returning `{ "status": "healthy" }`
- [ ] T047 [P] Add logging to all API endpoints in `api/src/routers/` using Python `logging` module
- [ ] T048 [P] Add error boundary component in `book-source/src/components/ContentTabs/ErrorBoundary.tsx` to catch React errors
- [ ] T049 [P] Add TypeScript strict mode validation: ensure no `any` types in `book-source/src/components/ContentTabs/` and `book-source/src/services/`
- [ ] T050 [P] Optimize CSS in `book-source/src/components/ContentTabs/styles.module.css` for performance (minimize reflows, use GPU-accelerated properties)
- [ ] T051 Add network interruption handling in `book-source/src/services/summaryService.ts` with EventSource error listener and retry logic
- [ ] T052 [P] Add API request timeout in `book-source/src/services/summaryService.ts` with 30-second timeout for SSE connection
- [ ] T053 [P] Document environment variables in `api/README.md` with setup instructions
- [ ] T054 [P] Document frontend component API in `book-source/src/components/ContentTabs/README.md` with usage examples
- [ ] T055 Run quickstart.md validation: verify all steps work for fresh developer setup

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Extends US1 tab switching but independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Requires auth and cache services from Foundational phase but independent of US1/US2
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories (just placeholder text)

### Within Each User Story

- **User Story 1**: T014-T016 can run in parallel (separate components), then T017 (integrates them), then T018-T020 (styling and integration)
- **User Story 2**: Sequential (extends existing components from US1)
- **User Story 3**: 
  - T026-T028 can run in parallel (frontend components and services)
  - T035-T038 can run in parallel (backend endpoints)
  - T029-T034 sequential (frontend flow implementation)
  - T039-T043 sequential (error handling and integration)
- **User Story 4**: T044-T045 can run in parallel or sequential (simple placeholder)

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Within US3: Frontend components (T026-T028) can be built in parallel with backend endpoints (T035-T038)
- Polish tasks marked [P] can run in parallel

---

## Parallel Example: User Story 3

```bash
# Launch frontend components for US3 together:
Task: "Create SummaryTab component skeleton in book-source/src/components/ContentTabs/SummaryTab.tsx"
Task: "Implement dummy login UI in book-source/src/components/ContentTabs/DummyLogin.tsx"
Task: "Create summary API client in book-source/src/services/summaryService.ts"

# Launch backend endpoints for US3 together:
Task: "Implement /summarize endpoint in api/src/routers/summarize.py"
Task: "Implement OpenAI Agents SDK integration in api/src/services/openai_agent.py"
Task: "Implement /auth/dummy-login endpoint in api/src/routers/auth.py"
Task: "Implement /auth/verify endpoint in api/src/routers/auth.py"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (tab UI with original content)
4. Complete Phase 4: User Story 2 (tab switching)
5. **STOP and VALIDATE**: Test tab navigation independently
6. Deploy/demo if ready

### Full Feature Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Tab UI works
3. Add User Story 2 → Test independently → Tab switching works
4. Add User Story 3 → Test independently → AI summarization works (MVP complete!)
5. Add User Story 4 → Test independently → Placeholder works
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Stories 1 & 2 (tab UI and switching)
   - Developer B: User Story 3 Frontend (SummaryTab, auth check, cache, streaming display)
   - Developer C: User Story 3 Backend (FastAPI endpoints, OpenAI integration)
   - Developer D: User Story 4 (placeholder)
3. Stories complete and integrate independently

---

## Task Summary

- **Total Tasks**: 55
- **Setup Phase**: 6 tasks
- **Foundational Phase**: 7 tasks (CRITICAL BLOCKING)
- **User Story 1 (P1)**: 7 tasks
- **User Story 2 (P1)**: 5 tasks
- **User Story 3 (P2)**: 18 tasks (9 frontend, 9 backend)
- **User Story 4 (P3)**: 2 tasks
- **Polish Phase**: 10 tasks
- **Parallelizable Tasks**: 28 tasks marked [P]
- **Suggested MVP**: User Stories 1 & 2 (basic tab UI and switching)
- **Full Feature**: User Stories 1-4 (includes AI summarization)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Tests are NOT included (not requested in spec) - add if needed later
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

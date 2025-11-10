# Tasks: Highlight Selection AI Dialog

**Input**: Design documents from `/specs/001-highlight-ai-dialog/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification does not explicitly request test tasks, so they are omitted from this list.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
-   Include exact file paths in descriptions

## Path Conventions

-   **Web app**: `book-source/` for frontend (Docusaurus), `backend/` for AI service (FastAPI)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both frontend and backend.

- [x] T001 Create `backend/` directory for FastAPI AI service

- [x] T002 Initialize Python project in `backend/` with FastAPI and Uvicorn dependencies

- [x] T003 [P] Configure Python linting (e.g., Black, Flake8) in `backend/`

- [x] T004 [P] Create `book-source/src/components/` directory for new React components

- [x] T005 [P] Configure TypeScript for frontend components in `book-source/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [x] T006 Implement basic FastAPI application structure in `backend/main.py`

- [x] T007 Implement CORS middleware for FastAPI in `backend/main.py`

- [x] T008 Implement AI generation endpoint (`POST /generate-ai-content`) in `backend/api/ai_generation.py` based on `contracts/ai-generation-service.yaml`

- [x] T009 Implement AI service client (for Gemini API calls) in `backend/services/gemini_service.py`

- [x] T010 Configure environment variables for Gemini API key in `backend/.env`

- [x] T011 Research Task 1: Investigate Docusaurus highlighting capabilities and update `research.md`

- [x] T012 Research Task 2: Explore Gemini prompt engineering best practices and update `research.md`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel.

---

## Phase 3: User Story 1 - View AI-generated context for highlighted text (Priority: P1) 🎯 MVP

**Goal**: Enable users to highlight text and see an AI dialog with relevant context, theory, instructions, and an example.

**Independent Test**: A user can highlight a text snippet, and the AI dialog will appear, populated with the expected sections and relevant content.

### Implementation for User Story 1

-   [ ] T013 [P] [US1] Create `HighlightDetector` React component in `book-source/src/components/HighlightDetector.tsx` to detect text selections.
-   [ ] T014 [P] [US1] Create `AIDialog` React component in `book-source/src/components/AIDialog.tsx` for displaying AI content.
-   [ ] T015 [US1] Implement logic in `HighlightDetector` to send highlighted text to the FastAPI backend.
-   [ ] T016 [US1] Implement logic in `AIDialog` to receive and display AI-generated content from the backend.
-   [ ] T017 [US1] Integrate `HighlightDetector` into a Docusaurus page/layout (e.g., `book-source/src/theme/Layout/index.tsx` or a specific MDX page).
-   [ ] T018 [US1] Implement basic error handling and loading states for AI content generation in `AIDialog.tsx`.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Edit AI-generated content in the dialog (Priority: P2)

**Goal**: Allow users to edit AI-generated content and manage custom sections within the dialog.

**Independent Test**: A user can modify the content of a section, add a new section, and remove a custom section, verifying that the changes are reflected in the dialog.

### Implementation for User Story 2

-   [ ] T019 [P] [US2] Enhance `AIDialog` component in `book-source/src/components/AIDialog.tsx` to make content sections editable.
-   [ ] T020 [P] [US2] Add "Add Section" button and functionality to `AIDialog.tsx`.
-   [ ] T021 [P] [US2] Add "Remove Section" button and functionality for custom sections to `AIDialog.tsx`.
-   [ ] T022 [US2] Implement local state management within `AIDialog` for edited and custom sections.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - View AI-generated context for highlighted diagrams or code snippets (Priority: P3)

**Goal**: Extend AI dialog functionality to highlighted diagrams and code snippets.

**Independent Test**: A user can highlight a diagram or a code snippet, and the AI dialog will appear, populated with the expected sections and relevant content for that specific type of selection.

### Implementation for User Story 3

-   [ ] T023 [P] [US3] Enhance `HighlightDetector` in `book-source/src/components/HighlightDetector.tsx` to detect diagram and code snippet selections.
-   [ ] T024 [US3] Modify backend AI generation logic in `backend/api/ai_generation.py` and `backend/services/gemini_service.py` to process diagram/code context effectively.
-   [ ] T025 [US3] Update `AIDialog` to display AI-generated content specific to diagrams/code snippets.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

-   [ ] T026 Code cleanup and refactoring for both frontend and backend.
-   [ ] T027 Performance optimization for highlight detection and AI dialog rendering.
-   [ ] T028 Update documentation (e.g., `README.md` for the backend service, Docusaurus usage guide).
-   [ ] T029 Implement Dockerfile for the FastAPI backend in `backend/Dockerfile`.
-   [ ] T030 Run quickstart.md validation (once created).

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
-   **User Stories (Phase 3+)**: All depend on Foundational phase completion.
    -   User stories can then proceed in parallel (if staffed).
    -   Or sequentially in priority order (P1 → P2 → P3).
-   **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
-   **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable.
-   **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable.

### Within Each User Story

-   Models before services.
-   Services before endpoints.
-   Core implementation before integration.
-   Story complete before moving to next priority.

### Parallel Opportunities

-   All Setup tasks marked [P] can run in parallel.
-   All Foundational tasks marked [P] can run in parallel (within Phase 2).
-   Once Foundational phase completes, all user stories can start in parallel (if team capacity allows).
-   Tasks within a story marked [P] can run in parallel.
-   Different user stories can be worked on in parallel by different team members.

---

## Parallel Example: User Story 1

```bash
# Launch all parallel tasks for User Story 1 together:
- [ ] T013 [P] [US1] Create `HighlightDetector` React component in `book-source/src/components/HighlightDetector.tsx` to detect text selections.
- [ ] T014 [P] [US1] Create `AIDialog` React component in `book-source/src/components/AIDialog.tsx` for displaying AI content.
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test User Story 1 independently
5.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Add User Story 2 → Test independently → Deploy/Demo
4.  Add User Story 3 → Test independently → Deploy/Demo
5.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together
2.  Once Foundational is done:
    -   Developer A: User Story 1
    -   Developer B: User Story 2
    -   Developer C: User Story 3
3.  Stories complete and integrate independently.

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

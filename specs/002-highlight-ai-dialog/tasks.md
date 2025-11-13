# Tasks: Highlight Selection AI Dialog

**Feature Branch**: `002-highlight-ai-dialog` | **Date**: Tuesday, November 11, 2025 | **Spec**: /home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/spec.md

## Phase 1: Setup

- [x] T001 Create backend project structure in backend/
- [x] T002 Create frontend project structure in frontend/
- [x] T003 Initialize Python virtual environment and install dependencies for backend in backend/
- [x] T004 Initialize Node.js project and install dependencies for frontend in frontend/

## Phase 2: Foundational

- [x] T005 Configure `GEMINI_API_KEY` environment variable for backend in backend/.env
- [x] T006 Implement basic FastAPI application in backend/main.py
- [x] T007 Implement basic health check endpoint in backend/src/api/routes.py

## Phase 3: User Story 1 (P1) - Invoke AI Dialog with Highlighted Text

**Goal**: A user can highlight text, open an AI dialog, submit a query, and see an AI response.
**Independent Test**: Highlight text, invoke dialog, type "summarize this", submit, verify response appears.

- [X] T008 [P] [US1] Create Gemini API service in backend/src/services/gemini_service.py
- [X] T009 [P] [US1] Implement `/api/ai/query` endpoint in backend/src/api/routes.py
- [X] T010 [P] [US1] Add unit tests for Gemini API service in backend/tests/unit/test_gemini_service.py
- [X] T011 [P] [US1] Add integration tests for `/api/ai/query` endpoint in backend/tests/integration/test_ai_query.py
- [X] T012 [P] [US1] Implement text highlighting functionality in frontend/src/services/text_selection_service.ts
- [X] T013 [P] [US1] Create AI Dialog UI component in frontend/src/components/AIDialog.tsx
- [X] T014 [P] [US1] Implement logic to populate AI Dialog with highlighted text in frontend/src/components/AIDialog.tsx
- [X] T015 [P] [US1] Implement logic to send AI queries to backend in frontend/src/services/ai_service.ts
- [X] T016 [P] [US1] Implement logic to display AI responses in frontend/src/components/AIDialog.tsx
- [X] T017 [P] [US1] Add unit tests for AI Dialog component in frontend/tests/unit/AIDialog.test.tsx
- [X] T018 [P] [US1] Add e2e tests for invoking AI Dialog and submitting query in frontend/tests/e2e/ai_dialog.spec.ts

## Phase 4: User Story 2 (P2) - Configure Gemini API Key and Model

**Goal**: A user can configure their Gemini API key and the system uses it for AI interactions.
**Independent Test**: Access config UI, enter valid key, save, verify AI dialog works. Enter invalid key, verify error feedback.

- [X] T019 [P] [US2] Implement `/api/config/gemini-key` endpoint in backend/src/api/routes.py
- [X] T020 [P] [US2] Implement `/api/config/status` endpoint in backend/src/api/routes.py
- [X] T021 [P] [US2] Add validation logic for Gemini API key in backend/src/services/config_service.py
- [X] T022 [P] [US2] Add unit tests for config service in backend/tests/unit/test_config_service.py
- [X] T023 [P] [US2] Add integration tests for config endpoints in backend/tests/integration/test_config_api.py
- [X] T024 [P] [US2] Create Configuration UI component in frontend/src/components/ConfigUI.tsx
- [X] T025 [P] [US2] Implement logic to send Gemini API key to backend in frontend/src/services/config_service.ts
- [X] T026 [P] [US2] Implement logic to display configuration status and validation feedback in frontend/src/components/ConfigUI.tsx
- [X] T027 [P] [US2] Add unit tests for ConfigUI component in frontend/tests/unit/ConfigUI.test.tsx
- [X] T028 [P] [US2] Add e2e tests for configuring API key and checking status in frontend/tests/e2e/config_flow.spec.ts

## Phase 5: Polish & Cross-Cutting Concerns

- [x] T029 Implement robust error handling for backend in backend/src/api/routes.py and backend/src/services/
- [x] T030 Implement robust error handling for frontend in frontend/src/services/ and frontend/src/components/
- [x] T031 Add structured logging for backend operations in backend/main.py and backend/src/services/
- [x] T032 Update `quickstart.md` with detailed instructions for frontend configuration in specs/002-highlight-ai-dialog/quickstart.md
- [x] T033 Review and refine all documentation in specs/002-highlight-ai-dialog/

## Dependencies

- Phase 1 must be completed before Phase 2.
- Phase 2 must be completed before Phase 3 and Phase 4.
- Phase 3 and Phase 4 can be developed in parallel.
- Phase 5 depends on the completion of all prior phases.

## Parallel Execution Examples

**Example 1 (Backend Focus)**:
- T008 [P] [US1] Create Gemini API service in backend/src/services/gemini_service.py
- T010 [P] [US1] Add unit tests for Gemini API service in backend/tests/unit/test_gemini_service.py
- T019 [P] [US2] Implement `/api/config/gemini-key` endpoint in backend/src/api/routes.py

**Example 2 (Frontend Focus)**:
- T012 [P] [US1] Implement text highlighting functionality in frontend/src/services/text_selection_service.ts
- T013 [P] [US1] Create AI Dialog UI component in frontend/src/components/AIDialog.tsx
- T024 [P] [US2] Create Configuration UI component in frontend/src/components/ConfigUI.tsx

## Implementation Strategy

This feature will be implemented using an MVP-first approach, focusing on delivering User Story 1 (Invoke AI Dialog with Highlighted Text) as the initial functional increment. User Story 2 (Configure Gemini API Key and Model) will follow, building upon the foundational elements. Tasks within each user story are designed to be as independent as possible to facilitate parallel development where feasible. Testing will be integrated throughout the development process, with unit tests for individual components and integration/e2e tests for user flows.

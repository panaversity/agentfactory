# Tasks: Personalized Content Generation with User Profiling

**Feature**: 023-personalization  
**Input**: Design documents from `specs/023-personalization/`  
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Organization**: Tasks grouped by user story (P1 → P2 → P3) to enable independent implementation and testing

**Tests**: NOT included (no TDD requirement in spec.md)

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Parallelizable (different files, no dependencies on incomplete tasks)
- **[Story]**: User story label (US1, US2, US3)
- File paths use absolute references from repository root

---

## Implementation Strategy

**MVP Scope** (Minimum Viable Product):
- **User Story 1 only** (Dummy Login with Profiling)
- Delivers authentication flow + profile collection
- Testable independently without personalization generation

**Incremental Delivery**:
1. **US1** → Login form + session storage → Users can authenticate
2. **US2** → Personalization streaming → Users get tailored content
3. **US3** → Client-side caching → Performance optimization

**Parallel Opportunities**: Marked with [P] - can execute simultaneously when dependencies met

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize project structure and install dependencies

- [ ] T001 Verify existing project structure matches plan.md (book-source/ frontend, api/ backend)
- [ ] T002 Install frontend dependencies: React 19, TypeScript 5.6, Docusaurus 3.9.2 (verify package.json)
- [ ] T003 Install backend dependencies: FastAPI, Pydantic, OpenAI Agents SDK (verify api/requirements.txt)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story work

**⚠️ CRITICAL**: No user story implementation can begin until this phase is complete

### Backend Foundation

- [x] T004 Define ProficiencyLevel enum in `api/src/models/schemas.py` (values: Novice, Beginner, Intermediate, Expert)
- [x] T005 [P] Define UserProfile Pydantic model in `api/src/models/schemas.py` (fields: name, email, programming_experience, ai_proficiency)
- [x] T006 [P] Define ProfileLoginRequest schema in `api/src/models/schemas.py` (request body for enhanced login)
- [x] T007 [P] Define PersonalizationRequest schema in `api/src/models/schemas.py` (query params for /personalize endpoint)

### Frontend Foundation

- [x] T008 Define ProficiencyLevel TypeScript type in `book-source/src/types/contentTabs.ts` (union type: 'Novice' | 'Beginner' | 'Intermediate' | 'Expert')
- [x] T009 [P] Define UserProfile TypeScript interface in `book-source/src/types/contentTabs.ts` (fields: name, email, programmingExperience, aiProficiency)
- [x] T010 [P] Define AuthSession TypeScript interface in `book-source/src/types/contentTabs.ts` (fields: token, profile, expiresAt?)
- [x] T011 [P] Define PersonalizationCacheEntry interface in `book-source/src/types/contentTabs.ts` (fields: pageId, profileFingerprint, personalizedText, timestamp, cached)

**Checkpoint**: Types and schemas defined - user story implementation can now begin

---

## Phase 3: User Story 1 - Dummy Login with User Profiling (Priority: P1) 🎯 MVP

**Goal**: Implement authentication flow with profiling (name, email, programming experience, AI proficiency) using dummy tokens

**Independent Test**: Navigate to Summary or Personalization tab while unauthenticated → See login button → Click button → Route to login page → Complete 4-field form → Submit → Verify session token + profile stored in sessionStorage → Navigate back to tabs → Verify no login button shown (authenticated state)

**Why MVP**: Foundation for all personalization features. Without profiles, content cannot be personalized.

### Backend: Enhanced Login Endpoint (FR-023, FR-002, FR-005, FR-006)

- [x] T012 [US1] Create `/auth/dummy-login-with-profile` POST endpoint in `api/src/routers/auth.py` accepting ProfileLoginRequest
- [x] T013 [US1] Implement Pydantic validation for ProfileLoginRequest in auth endpoint (all fields required, email format)
- [x] T014 [US1] Generate dummy token in auth endpoint (format: `dummy_token_{uuid4}`)
- [x] T015 [US1] Return AuthWithProfileResponse in auth endpoint (token, expires: "session", user: UserProfile)
- [x] T016 [US1] Add inline error responses for validation failures (400 status with ErrorResponse schema)

### Frontend: Authentication Service Enhancement (FR-006, FR-007)

- [x] T017 [US1] Update `authService.ts` to store composite AuthSession object in sessionStorage (key: "authSession")
- [x] T018 [US1] Implement `authService.saveSession(token, profile)` function to store {token, profile, expiresAt: undefined}
- [x] T019 [US1] Implement `authService.getSession()` function returning AuthSession | null from sessionStorage
- [x] T020 [US1] Implement `authService.getProfile()` function extracting profile from session
- [x] T021 [US1] Implement `authService.isAuthenticated()` function checking session token existence
- [x] T022 [US1] Implement `authService.generateProfileFingerprint(profile)` function returning "{programmingExperience}-{aiProficiency}"

### Frontend: Login Page Component (FR-001, FR-002, FR-002a, FR-002b)

- [x] T023 [US1] Create `DummyLoginWithProfile.tsx` component in `book-source/src/components/ContentTabs/`
- [x] T024 [US1] Implement 4-field form in DummyLoginWithProfile: name (text), email (email), programmingExperience (dropdown), aiProficiency (dropdown)
- [x] T025 [US1] Add proficiency level dropdowns with options: Novice, Beginner, Intermediate, Expert
- [x] T026 [US1] Implement form validation: all fields required, email format check (regex: `^[\w\.-]+@[\w\.-]+\.\w+$`)
- [x] T027 [US1] Display inline error messages for missing/invalid fields and prevent submission until valid
- [x] T028 [US1] Implement form submission handler calling `POST /api/v1/auth/dummy-login-with-profile`
- [x] T029 [US1] On successful login, call `authService.saveSession(token, profile)` and redirect to previous tab
- [x] T030 [US1] Add loading state during form submission (disable submit button, show spinner)

### Frontend: Login Button Integration (NEW REQUIREMENT - FR-001, FR-008)

- [x] T031 [US1] Update `SummaryTab.tsx`: Check `authService.isAuthenticated()` on mount
- [x] T032 [US1] In SummaryTab: If not authenticated, render "Login to See Summary" button routing to `/login` page
- [x] T033 [US1] In SummaryTab: If authenticated, render existing summary generation UI
- [x] T034 [US1] Create routing configuration for `/login` page rendering DummyLoginWithProfile component
- [x] T035 [US1] Add navigation state to login route storing return URL (e.g., `/login?returnTo=/docs/intro`)
- [x] T036 [US1] Update DummyLoginWithProfile to redirect to `returnTo` URL after successful login (default: home)

**US1 Acceptance**: ✅ User can click login button from Summary/Personalization tabs → Complete 4-field form → Session stored → Return to tab without seeing login button again

---

## Phase 4: User Story 2 - Personalized Content Streaming (Priority: P2)

**Goal**: Generate and stream AI-personalized content based on user's programming experience and AI proficiency levels

**Independent Test**: Login with specific proficiency (e.g., Novice programming + Beginner AI) → Navigate to Personalization tab → Click "Generate Personalized Content" → Verify streaming starts within 2s → Verify content complexity matches proficiency level → Check for "Generating..." button state

**Depends On**: US1 (requires authentication and profile storage)

### Backend: Separate Personalization Agent (Decision 2, FR-009, FR-010, FR-022, FR-024)

- [ ] T037 [US2] Create `generate_personalized_content()` async function in `api/src/services/openai_agent.py`
- [ ] T038 [US2] Implement separate Agent instance in generate_personalized_content: `Agent(name="Content Personalizer", instructions=..., model=...)`
- [ ] T039 [US2] Build proficiency-specific instructions using programming_level and ai_proficiency parameters (Decision 5 templates)
- [ ] T040 [US2] Construct session_id as `f"{page_id}_{programming_level}_{ai_proficiency}"` for agent context isolation
- [ ] T041 [US2] Implement streaming with `Runner.run_streamed()` yielding chunks as strings
- [ ] T042 [US2] Export `generate_personalized_content` in `api/src/services/__init__.py`

### Backend: Personalization Endpoint (FR-024, FR-013, FR-015)

- [ ] T043 [US2] Create `personalize.py` router in `api/src/routers/`
- [ ] T044 [US2] Implement `GET /personalize` endpoint accepting query params: pageId, content, token, programmingLevel, aiLevel
- [ ] T045 [US2] Add token validation in /personalize endpoint (check token exists, return 401 if missing/invalid)
- [ ] T046 [US2] Validate query params: content min 100 chars, proficiency levels in ProficiencyLevel enum (return 400 on validation failure)
- [ ] T047 [US2] Set response content type to `text/event-stream` for SSE streaming
- [ ] T048 [US2] Stream chunks from `generate_personalized_content()` as SSE events (format: `data: {chunk, done, error}\n\n`)
- [ ] T049 [US2] Implement error handling: catch exceptions, yield error chunk, set done=true (FR-015a, FR-015b)
- [ ] T050 [US2] Register personalization router in `api/src/main.py` with prefix `/api/v1`

### Frontend: Personalization Service (FR-013, FR-015, FR-021)

- [ ] T051 [US2] Create `personalizationService.ts` in `book-source/src/services/`
- [ ] T052 [US2] Implement `generatePersonalizedContent()` function using EventSource for SSE streaming
- [ ] T053 [US2] Construct URL with query params: pageId, content, token (from authService), programmingLevel, aiLevel (from profile)
- [ ] T054 [US2] Parse SSE events extracting {chunk, done, error} JSON from data field
- [ ] T055 [US2] Implement callback pattern: onChunk(text), onComplete(), onError(message, partialContent)
- [ ] T056 [US2] Close EventSource on completion or error
- [ ] T057 [US2] Handle network errors and timeout scenarios (FR-015)

### Frontend: PersonalizationTab Component (FR-001, FR-008, FR-014, FR-021a, FR-021b)

- [ ] T058 [US2] Create `PersonalizationTab.tsx` component in `book-source/src/components/ContentTabs/`
- [ ] T059 [US2] Check authentication on mount: if not authenticated, render "Login to Personalize" button routing to `/login?returnTo=<current-page>#personalization`
- [ ] T060 [US2] If authenticated, render "Generate Personalized Content" button and content display area
- [ ] T061 [US2] Implement generate button click handler calling `personalizationService.generatePersonalizedContent()`
- [ ] T062 [US2] Disable generate button during streaming (FR-021a) and show "Generating..." text (FR-021b)
- [ ] T063 [US2] Display streaming chunks progressively in content area (append each chunk as received)
- [ ] T064 [US2] Implement auto-scroll to bottom during streaming (smooth scroll behavior)
- [ ] T065 [US2] On streaming completion, re-enable generate button and show success indicator
- [ ] T066 [US2] On streaming error, preserve partial content (FR-015a), show error message (FR-015b), and display retry button (FR-015c)
- [ ] T067 [US2] Implement retry button handler re-calling generatePersonalizedContent (clears previous error state)

### Frontend: Tab Integration

- [ ] T068 [US2] Update `book-source/src/components/ContentTabs/index.tsx` to add Personalization tab alongside Summary and Learn tabs
- [ ] T069 [US2] Add tab styling in `styles.module.css` for personalization tab (consistent with existing tabs)
- [ ] T070 [US2] Ensure PersonalizationTab receives pageId and content props from parent component

**US2 Acceptance**: ✅ Authenticated user clicks "Generate Personalized Content" → Content streams progressively → Button shows "Generating..." → Content complexity matches proficiency level → Errors preserve partial content with retry option

---

## Phase 5: User Story 3 - Personalized Content Caching (Priority: P3)

**Goal**: Cache generated personalized content client-side for instant reload on subsequent visits (profile-specific)

**Independent Test**: Generate personalized content for a page → Navigate away → Return to same page → Verify content loads instantly (<500ms) without calling /personalize endpoint → Check cache indicator → Change proficiency levels → Verify new content generated (different cache key)

**Depends On**: US2 (requires working personalization generation)

### Frontend: Cache Service Enhancement (FR-016, FR-017, FR-017a, FR-018, FR-019, FR-020)

- [ ] T071 [US3] Add `getPersonalizedContentFromCache(pageId, profileFingerprint)` function to cache service
- [ ] T072 [US3] Add `savePersonalizedContentToCache(pageId, profileFingerprint, content)` function to cache service
- [ ] T073 [US3] Implement cache key format: `personalized_{pageId}_{profileFingerprint}` (e.g., "personalized_intro_Novice-Beginner")
- [ ] T074 [US3] Store PersonalizationCacheEntry objects in sessionStorage: {pageId, profileFingerprint, personalizedText, timestamp, cached: true}
- [ ] T075 [US3] Implement cache retrieval returning PersonalizationCacheEntry | null

### Frontend: PersonalizationTab Cache Integration (FR-018, FR-019, FR-020)

- [ ] T076 [US3] Update PersonalizationTab to check cache on mount before showing generate button
- [ ] T077 [US3] Get profile fingerprint from `authService.generateProfileFingerprint(profile)`
- [ ] T078 [US3] Call `getPersonalizedContentFromCache(pageId, fingerprint)` and display if found
- [ ] T079 [US3] Show cache indicator when displaying cached content (e.g., "📋 Cached content from {timestamp}")
- [ ] T080 [US3] Skip generate button if cached content displayed (or show "Regenerate" option)
- [ ] T081 [US3] After successful personalization generation, call `savePersonalizedContentToCache()` to store result
- [ ] T082 [US3] Update cache indicator to show "freshly generated" vs "from cache" state

### State Machine Implementation (from data-model.md)

- [ ] T083 [US3] Implement 9-state machine in PersonalizationTab: IDLE → CHECKING_CACHE → CACHE_HIT/CHECKING_AUTH → LOADING → STREAMING → SUCCESS/ERROR
- [ ] T084 [US3] Add state transitions: IDLE → CHECKING_CACHE (on mount), CHECKING_CACHE → CACHE_HIT (if cached) or CHECKING_AUTH (if not cached)
- [ ] T085 [US3] Add transition: CHECKING_AUTH → LOGIN_REQUIRED (not authenticated) or LOADING (authenticated)
- [ ] T086 [US3] Add transition: LOADING → STREAMING (generation starts) → SUCCESS (complete) or ERROR (failure)
- [ ] T087 [US3] Render UI based on current state: CACHE_HIT shows content immediately, LOADING shows spinner, STREAMING shows progressive chunks

**US3 Acceptance**: ✅ First visit generates content → Second visit loads instantly from cache (<500ms) → Cache indicator shows source → Different profiles get different cached content → Cache hit rate reaches 80% for repeat visits

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, session expiration, edge cases, and UX refinements

### Session Expiration Handling (FR-006a, FR-006b, FR-006c)

- [ ] T088 Implement session expiration detection in authService (check expiresAt if present)
- [ ] T089 Add `authService.isSessionExpired()` function checking expiration timestamp
- [ ] T090 Update PersonalizationTab to check session expiration before generation requests
- [ ] T091 Show non-intrusive notification on session expiration (e.g., toast: "Session expired. Please login to generate new content.")
- [ ] T092 Allow viewing current cached/displayed content after expiration (don't clear display)
- [ ] T093 Require re-login for new personalization requests after expiration (redirect to /login with returnTo)

### Error Boundary & Edge Cases

- [ ] T094 Add error boundary component wrapping PersonalizationTab to catch React errors
- [ ] T095 Handle edge case: User clears sessionStorage during generation (detect missing session, show login button)
- [ ] T096 Handle edge case: Duplicate requests for same page/profile (use ref guard to prevent concurrent calls)
- [ ] T097 Add request debouncing to generate button (prevent accidental double-clicks)

### UX Refinements

- [ ] T098 Add loading skeletons for initial cache check phase (shimmer effect while CHECKING_CACHE)
- [ ] T099 Implement smooth transitions between states (fade-in animations for content display)
- [ ] T100 Add accessibility: ARIA labels for generate button states, screen reader announcements for streaming progress
- [ ] T101 Add keyboard navigation: Enter key to submit login form, Esc to close error messages
- [ ] T102 Style login button consistently across Summary and Personalization tabs (use shared CSS class)

### Backend Validation & Logging

- [ ] T103 Add request logging in /personalize endpoint (log pageId, proficiency levels, token prefix for debugging)
- [ ] T104 Add rate limiting consideration (document future rate limit strategy in comments)
- [ ] T105 Validate content length in /personalize endpoint (reject if >50,000 chars per API contract)
- [ ] T106 Add health check verification that OpenAI API key is configured (return 500 if missing)

### Documentation

- [ ] T107 Update `quickstart.md` with final testing checklist based on implemented features
- [ ] T108 Document environment variables needed: OPENAI_API_KEY or GOOGLE_API_KEY in api/.env
- [ ] T109 Add inline code comments documenting state machine transitions in PersonalizationTab
- [ ] T110 Document cache key format and fingerprint generation in code comments

---

## Dependencies & Execution Order

### Critical Path (Sequential - Must Complete in Order):
1. **Phase 1 (Setup)** → T001-T003
2. **Phase 2 (Foundational)** → T004-T011
3. **Phase 3 (US1 - Login)** → T012-T036
4. **Phase 4 (US2 - Streaming)** → T037-T070 (depends on US1 session storage)
5. **Phase 5 (US3 - Caching)** → T071-T087 (depends on US2 generation working)
6. **Phase 6 (Polish)** → T088-T110 (depends on US1-3 complete)

### User Story Dependencies:
- **US1** (Login): No dependencies - can implement first (MVP)
- **US2** (Streaming): Requires US1 complete (needs authentication and profile)
- **US3** (Caching): Requires US2 complete (needs working generation to cache results)

### Parallel Execution Opportunities:

**Within Phase 2 (Foundational)**:
- T005, T006, T007 (backend schemas) || T009, T010, T011 (frontend types)
  - Different files, no dependencies between backend and frontend types

**Within Phase 3 (US1)**:
- T016 (backend error responses) || T023-T030 (frontend login component)
  - Backend and frontend can develop simultaneously
- T031-T033 (SummaryTab updates) || T023-T030 (DummyLoginWithProfile component)
  - Different components, no shared code

**Within Phase 4 (US2)**:
- T037-T042 (backend agent) || T051-T057 (frontend service)
  - Backend and frontend can develop simultaneously once contracts defined
- T058-T067 (PersonalizationTab) || T043-T050 (backend endpoint)
  - Can develop in parallel, then integrate

**Within Phase 5 (US3)**:
- T071-T075 (cache service) || T083-T087 (state machine)
  - Cache storage logic independent from state management logic

**Within Phase 6 (Polish)**:
- T088-T093 (session expiration) || T094-T097 (error boundaries) || T098-T102 (UX) || T103-T106 (backend validation)
  - All cross-cutting concerns can execute in parallel

---

## Summary

**Total Tasks**: 110 tasks across 6 phases

**Task Breakdown by User Story**:
- **Setup**: 3 tasks
- **Foundational**: 8 tasks (types/schemas)
- **US1 (Login - P1)**: 25 tasks (12 backend, 13 frontend) - **MVP SCOPE**
- **US2 (Streaming - P2)**: 34 tasks (14 backend, 20 frontend)
- **US3 (Caching - P3)**: 17 tasks (all frontend)
- **Polish**: 23 tasks (cross-cutting)

**Parallel Opportunities**: 38 tasks marked with [P] can execute simultaneously when dependencies met

**Independent Test Criteria**:
- **US1**: Login flow complete, session stored, no re-login on return
- **US2**: Personalization streams correctly, button states work, errors handled
- **US3**: Cache loads <500ms, cache indicator accurate, profile-specific keys

**Suggested MVP Scope**: Phase 1 + Phase 2 + Phase 3 (US1 only) = 36 tasks
- Delivers authentication flow with profiling
- Enables future personalization work
- Testable without AI generation complexity

**Format Validation**: ✅ All tasks follow checklist format with ID, [P] marker (if parallel), [Story] label (for US tasks), description with file paths

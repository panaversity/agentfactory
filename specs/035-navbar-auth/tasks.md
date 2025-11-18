# Tasks: Navbar Authentication UI

**Input**: Design documents from `/specs/035-navbar-auth/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: No automated tests requested for this MVP - manual testing only (see quickstart.md)

**Organization**: Tasks grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Repository: `book-source/` (Docusaurus project root)
- Components: `book-source/src/theme/Navbar/`
- Services: `book-source/src/services/` (existing, no modifications)
- Types: `book-source/src/types/` (existing, no modifications)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus navbar override structure

- [ ] T001 Swizzle Docusaurus Navbar/Content component to `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T002 Verify swizzled component renders original navbar correctly (no UI breakage)
- [ ] T003 [P] Add CSS module for navbar auth button styles in `book-source/src/theme/Navbar/Content/styles.module.css`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core authentication state management that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T004 Import authService and types (AuthState, UserProfile) from existing services in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T005 Add React state hooks for authentication state: `isAuthenticated`, `userProfile`, `showLoginModal` in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T006 Implement useEffect hook to check initial auth state on component mount in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T007 Add storage event listener in useEffect to sync auth state changes in `book-source/src/theme/Navbar/Content/index.tsx`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Guest User Login (Priority: P1) 🎯 MVP

**Goal**: Enable guest users to click login button, fill profile form, and see authenticated state in navbar

**Independent Test**: Navigate to site as guest → Click "Login" button → Fill form → Submit → Verify button changes to user icon (see quickstart.md Test 1)

### Implementation for User Story 1

- [ ] T008 [US1] Import DummyLoginWithProfile component in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T009 [US1] Render conditional login button (when NOT authenticated) in navbar items array in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T010 [US1] Add onClick handler to login button to set `showLoginModal` state to true in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T011 [US1] Position login button in navbar items array before ColorModeToggle using array splice in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T012 [US1] Render DummyLoginWithProfile modal conditionally when `showLoginModal` is true in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T013 [US1] Add `onClose` callback to DummyLoginWithProfile to set `showLoginModal` to false in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T014 [US1] Add `onSuccess` callback to DummyLoginWithProfile to update auth state and close modal in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T015 [US1] Update auth state (isAuthenticated, userProfile) after successful login in handleLoginSuccess callback in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T016 [US1] Add ARIA label "Login to access personalized content" to login button in `book-source/src/theme/Navbar/Content/index.tsx`

**Checkpoint**: At this point, User Story 1 should be fully functional - guest users can log in and see authenticated navbar

---

## Phase 4: User Story 2 - Authenticated User Logout (Priority: P2)

**Goal**: Enable authenticated users to click logout button and return to guest state

**Independent Test**: Log in via User Story 1 → Click user icon/logout button → Verify button changes to "Login" and sessionStorage cleared (see quickstart.md Test 3)

### Implementation for User Story 2

- [ ] T017 [US2] Render conditional logout button (when authenticated) in navbar items array in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T018 [US2] Add Font Awesome fa-user-circle icon to logout button in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T019 [US2] Add screen-reader-only text "Logout" using sr-only CSS class in `book-source/src/theme/Navbar/Content/styles.module.css`
- [ ] T020 [US2] Implement handleLogout function to call authService.clearToken() in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T021 [US2] Update local state (isAuthenticated, userProfile) after logout in handleLogout function in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T022 [US2] Add onClick handler to logout button to call handleLogout in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T023 [US2] Add ARIA label "Logout" to logout button in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T024 [US2] Add aria-hidden="true" to user icon in logout button in `book-source/src/theme/Navbar/Content/index.tsx`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - full login/logout cycle functional

---

## Phase 5: User Story 3 - Visual Feedback and State Indication (Priority: P3)

**Goal**: Provide clear visual indicators of authentication state and button interactions

**Independent Test**: Observe button appearance in both states (logged in/out), test hover effects, verify icon visibility (see quickstart.md Test 2, Test 6)

### Implementation for User Story 3

- [ ] T025 [P] [US3] Add hover state styles for login button in `book-source/src/theme/Navbar/Content/styles.module.css`
- [ ] T026 [P] [US3] Add hover state styles for logout button in `book-source/src/theme/Navbar/Content/styles.module.css`
- [ ] T027 [P] [US3] Add focus visible styles for keyboard accessibility in `book-source/src/theme/Navbar/Content/styles.module.css`
- [ ] T028 [US3] Position logout button (with icon) in navbar items array before ColorModeToggle in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T029 [US3] Add type="button" attribute to both login and logout buttons in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T030 [US3] Verify button styling matches existing navbar items (GitHub link, theme toggle) in `book-source/src/theme/Navbar/Content/styles.module.css`

**Checkpoint**: All user stories should now be independently functional - login/logout with full visual feedback

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final validation

- [ ] T031 [P] Test login button positioning on desktop viewport (1920px width) using browser DevTools
- [ ] T032 [P] Test login button positioning on tablet viewport (768px width) using browser DevTools
- [ ] T033 [P] Test login button positioning on mobile viewport (375px width) using browser DevTools
- [ ] T034 Test keyboard navigation: Tab to focus login button, Enter to activate (see quickstart.md Test 4)
- [ ] T035 Test keyboard navigation: Space key to activate login button (alternative to Enter)
- [ ] T036 Test keyboard navigation: Escape key to close modal without logging in (see quickstart.md Test 4)
- [ ] T037 Test screen reader accessibility with NVDA (Windows) or VoiceOver (macOS) - verify ARIA labels announced (see quickstart.md Test 5)
- [ ] T038 Test session persistence: Log in → Navigate to different page → Refresh → Verify still logged in (see quickstart.md Test 2)
- [ ] T039 Test browser session timeout: Log in → Close browser → Reopen → Verify logged out (see quickstart.md Test 7)
- [ ] T040 Test multi-tab independence: Log in on Tab 1 → Open Tab 2 → Verify Tab 2 shows "Login" button (see quickstart.md Test 8)
- [ ] T041 Test logout in Tab 1 → Verify Tab 2 remains logged in (multi-tab independence validation)
- [ ] T042 Verify no console errors or warnings in browser DevTools during login/logout flow
- [ ] T043 Verify no UI layout breakage in navbar (GitHub link, search, theme toggle positions unchanged)
- [ ] T044 Run all quickstart.md validation tests (Tests 1-8) and verify all acceptance criteria pass
- [ ] T045 [P] Add inline code comments explaining swizzle pattern and auth state management in `book-source/src/theme/Navbar/Content/index.tsx`
- [ ] T046 [P] Document button positioning logic (array splice before ColorModeToggle) in code comments in `book-source/src/theme/Navbar/Content/index.tsx`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (T001-T003) - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational (T004-T007) completion
  - User stories can proceed sequentially in priority order (P1 → P2 → P3)
  - OR in parallel if different developers work on different stories
- **Polish (Phase 6)**: Depends on all user stories (Phase 3-5) being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Technically independent, but builds on US1 login flow for testing
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Enhances US1/US2 with visual feedback

### Within Each User Story

- **User Story 1**: T008 (import) → T009-T011 (button rendering) → T012-T015 (modal integration) → T016 (accessibility)
- **User Story 2**: T017-T019 (logout button rendering) → T020-T021 (logout logic) → T022-T024 (event handlers + accessibility)
- **User Story 3**: T025-T027 (CSS styles, parallelizable) → T028-T030 (component updates)

### Critical Path

1. T001 (swizzle) → T002 (verify)
2. T004-T007 (foundational state management)
3. T008-T016 (User Story 1 - login flow)
4. T017-T024 (User Story 2 - logout flow)
5. T025-T030 (User Story 3 - visual polish)
6. T031-T046 (comprehensive testing and validation)

### Parallel Opportunities

- **Setup Phase**: T001-T002 must be sequential, T003 can overlap with T002
- **Foundational Phase**: T004-T007 are sequential (state setup dependencies)
- **User Story 3 CSS**: T025-T027 can all run in parallel (different style rules)
- **Polish Testing**: T031-T033 (viewport tests) can run in parallel with T045-T046 (documentation)

---

## Parallel Example: User Story 3 (Visual Feedback)

```bash
# Launch all CSS styling tasks together:
Task: "Add hover state styles for login button in styles.module.css"
Task: "Add hover state styles for logout button in styles.module.css"
Task: "Add focus visible styles for keyboard accessibility in styles.module.css"

# These can be done by different developers or in parallel branches
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T007) **CRITICAL**
3. Complete Phase 3: User Story 1 (T008-T016)
4. **STOP and VALIDATE**: Run quickstart.md Test 1 (Guest Login Flow)
5. If Test 1 passes → MVP ready to demo (guest login functional)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (T008-T016) → Test independently (quickstart.md Test 1) → **MVP READY**
3. Add User Story 2 (T017-T024) → Test independently (quickstart.md Test 3) → **Full auth cycle ready**
4. Add User Story 3 (T025-T030) → Test independently (quickstart.md Test 6) → **Visual polish complete**
5. Run comprehensive testing (Phase 6: T031-T046) → **Production ready**

### Parallel Team Strategy

With multiple developers:

1. **Team completes Setup + Foundational together** (T001-T007)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T008-T016) - Login flow
   - **Developer B**: User Story 2 (T017-T024) - Logout flow (can start in parallel, test after US1)
   - **Developer C**: User Story 3 CSS (T025-T027) - Styling (fully parallel)
3. **Integration**: T028-T030 (User Story 3 component updates)
4. **Team validation**: Phase 6 testing (T031-T046)

---

## Task Count Summary

- **Phase 1 (Setup)**: 3 tasks
- **Phase 2 (Foundational)**: 4 tasks **← BLOCKS ALL USER STORIES**
- **Phase 3 (User Story 1 - P1 MVP)**: 9 tasks
- **Phase 4 (User Story 2 - P2)**: 8 tasks
- **Phase 5 (User Story 3 - P3)**: 6 tasks
- **Phase 6 (Polish & Testing)**: 16 tasks

**Total**: 46 tasks

**Parallelizable**: 6 tasks marked [P] (13% of total)
- T003 (CSS module setup)
- T025-T027 (User Story 3 CSS styles - 3 tasks)
- T031-T033 (Viewport testing - partially parallel)
- T045-T046 (Documentation - 2 tasks)

**MVP Scope** (User Story 1 only): 16 tasks (T001-T016)

---

## Notes

- No automated tests for MVP - all testing is manual via quickstart.md
- [P] tasks = different files or independent CSS rules, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently testable (see quickstart.md for test scenarios)
- Commit after each task or logical group (e.g., T008-T011 login button rendering)
- Stop at any checkpoint to validate story independently before proceeding
- All tasks reference exact file paths for LLM implementation
- Existing files (authService, DummyLoginWithProfile, types) are NOT modified - only imported/reused
- **Zero new dependencies**: Feature uses only existing Docusaurus theme components and Font Awesome icons

# Tasks: Metadata-Driven Slides Architecture

**Input**: Design documents from `/specs/035-metadata-driven-slides/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md

**Tests**: No automated tests required (manual testing approach per research.md)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- Project type: Docusaurus plugin (build-time MDX transformation)
- Plugin path: `book-source/plugins/remark-slides-metadata/`
- Test chapters: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/`
- Static files: `book-source/static/slides/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and plugin structure

- [ ] T001 Create plugin directory structure at book-source/plugins/remark-slides-metadata/
- [ ] T002 Install remark dependencies: unist-util-visit and mdast-util-to-string in book-source/package.json
- [ ] T003 [P] Create plugin entry point file at book-source/plugins/remark-slides-metadata/index.ts
- [ ] T004 [P] Create utility functions file at book-source/plugins/remark-slides-metadata/utils.ts
- [ ] T005 [P] Create transformer file at book-source/plugins/remark-slides-metadata/transformer.ts

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core plugin infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Implement path detection logic in book-source/plugins/remark-slides-metadata/utils.ts (detectPathType function)
- [ ] T007 [P] Implement path normalization logic in book-source/plugins/remark-slides-metadata/utils.ts (normalizePath function)
- [ ] T008 [P] Implement heading finder in book-source/plugins/remark-slides-metadata/utils.ts (findHeading function using unist-util-visit)
- [ ] T009 Implement import node generator in book-source/plugins/remark-slides-metadata/utils.ts (generateImportNode function)
- [ ] T010 [P] Implement JSX node generator in book-source/plugins/remark-slides-metadata/utils.ts (generateSlidesSection function)
- [ ] T011 Register plugin in book-source/docusaurus.config.ts under presets.classic.docs.remarkPlugins array

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Content Creator Adds Slides via Frontmatter (Priority: P1) 🎯 MVP

**Goal**: Content creators can add slides to any chapter by editing only frontmatter with local path, without any imports or JSX in markdown content

**Independent Test**: Add `slides: "slides/chapter-02-slides.pdf"` to Chapter 2 README frontmatter, verify PDF renders after "What You'll Learn" section during `npm start` without manual imports

### Implementation for User Story 1

- [ ] T012 [US1] Implement main transformer function in book-source/plugins/remark-slides-metadata/transformer.ts to parse frontmatter and extract slides field
- [ ] T013 [US1] Add "What You'll Learn" heading detection in transformer using findHeading utility
- [ ] T014 [US1] Implement AST node injection logic in transformer to insert import, heading, and JSX nodes after target heading
- [ ] T015 [US1] Add import deduplication check in transformer to avoid duplicate PDFViewer imports
- [ ] T016 [US1] Wire transformer to plugin export in book-source/plugins/remark-slides-metadata/index.ts
- [ ] T017 [US1] Test with Chapter 2: Update apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/README.md frontmatter to add `slides: "slides/chapter-02-slides.pdf"`
- [ ] T018 [US1] Remove manual PDFViewer import and JSX from Chapter 2 README to validate metadata approach
- [ ] T019 [US1] Start dev server (`npm start` in book-source/) and verify slides render correctly in Chapter 2

**Checkpoint**: At this point, User Story 1 should be fully functional - local path slides injection works end-to-end

---

## Phase 4: User Story 2 - Content Creator Uses Cloud URL Instead of Local Path (Priority: P1)

**Goal**: Content creators can reference slides hosted on CDN/cloud storage using full URL, system renders identically to local paths

**Independent Test**: Add `slides: "https://cdn.example.com/chapter-03.pdf"` to Chapter 3 README frontmatter, verify URL is used as-is in PDFViewer src prop

### Implementation for User Story 2

- [ ] T020 [US2] Update transformer in book-source/plugins/remark-slides-metadata/transformer.ts to use detectPathType utility before normalization
- [ ] T021 [US2] Update JSX generation in transformer to pass normalized path (with or without leading slash) based on path type
- [ ] T022 [US2] Test with Chapter 3: Update apps/learn-app/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/README.md frontmatter to add `slides: "https://example.com/test.pdf"` (test URL)
- [ ] T023 [US2] Verify in dev server that URL is passed directly to PDFViewer without modification
- [ ] T024 [US2] Test mixed scenario: Verify Chapter 2 (local) and Chapter 3 (URL) both work simultaneously during full site build

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - both local paths and cloud URLs supported

---

## Phase 5: User Story 3 - Developer Maintains Slides Across 84 Chapters (Priority: P2)

**Goal**: Developers can update slide rendering behavior (height, styling, download button) across all chapters from single location (plugin or component)

**Independent Test**: Modify PDFViewer component height prop default or plugin-generated JSX, verify changes apply to all chapters with slides metadata without touching markdown

### Implementation for User Story 3

- [ ] T025 [US3] Document centralized modification points in specs/035-metadata-driven-slides/quickstart.md maintenance section
- [ ] T026 [US3] Add configuration comments in book-source/plugins/remark-slides-metadata/utils.ts generateSlidesSection function for height and title customization
- [ ] T027 [US3] Test centralized change: Modify height value in generateSlidesSection from 700 to 800
- [ ] T028 [US3] Run full build (`npm run build` in book-source/) and verify all chapters with slides reflect new height
- [ ] T029 [US3] Revert test change and document that PDFViewer component changes also apply globally

**Checkpoint**: All user stories (1, 2, 3) should now be independently functional - centralized maintenance validated

---

## Phase 6: User Story 4 - Build System Validates Slide References (Priority: P3)

**Goal**: During build, system validates slide references and warns about broken references without failing build (graceful degradation)

**Independent Test**: Add invalid slide reference (`slides: "slides/nonexistent.pdf"`) to test chapter, verify warning logged but build succeeds

### Implementation for User Story 4

- [ ] T030 [US4] Add try-catch error handling in book-source/plugins/remark-slides-metadata/transformer.ts main function
- [ ] T031 [US4] Implement console.warn logging for missing "What You'll Learn" heading in transformer
- [ ] T032 [US4] Implement console.warn logging for malformed frontmatter (slides field not string) in transformer
- [ ] T033 [US4] Add file path to all warning messages using vfile.history or vfile.path
- [ ] T034 [US4] Test graceful degradation: Create test chapter without "What You'll Learn" heading, verify warning logged and build succeeds
- [ ] T035 [US4] Test graceful degradation: Add malformed slides field (number instead of string), verify warning logged and build succeeds
- [ ] T036 [US4] Run full production build (`npm run build`) and verify no build failures from plugin

**Checkpoint**: All user stories should now be independently functional - graceful degradation ensures robustness

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, validation, and finalization

- [ ] T037 [P] Update specs/035-metadata-driven-slides/quickstart.md with real examples from Chapters 2-4
- [ ] T038 [P] Add TypeScript type definitions in book-source/plugins/remark-slides-metadata/index.ts for plugin options (even if empty interface)
- [ ] T039 Test Chapter 4: Update apps/learn-app/docs/01-Introducing-AI-Driven-Development/04-nine-pillars/README.md frontmatter with slides metadata
- [ ] T040 Validate backward compatibility: Ensure existing chapters with manual PDFViewer JSX still work alongside metadata approach
- [ ] T041 Run full build performance test: Compare build time before and after plugin (<5% increase acceptable per research.md)
- [ ] T042 [P] Add plugin README at book-source/plugins/remark-slides-metadata/README.md with architecture overview
- [ ] T043 Run quickstart.md validation: Follow all examples in quickstart guide to ensure accuracy
- [ ] T044 Final full build test: `npm run build` in book-source/ should succeed with all chapters building correctly

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - Core functionality
  - User Story 2 (P1): Depends on User Story 1 (extends path handling)
  - User Story 3 (P2): Can start after User Story 1 complete (tests centralization)
  - User Story 4 (P3): Can start after User Story 1 complete (adds error handling)
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Depends on User Story 1 core transformer - Extends path detection logic
- **User Story 3 (P2)**: Depends on User Story 1 being functional - Tests centralized modifications
- **User Story 4 (P3)**: Depends on User Story 1 core logic - Adds error handling and warnings

### Within Each User Story

- Core transformer logic before integration testing
- Utility functions before transformer implementation
- Dev server testing before production build testing
- Single chapter testing before multi-chapter validation

### Parallel Opportunities

- **Setup (Phase 1)**: T003, T004, T005 can run in parallel (different files)
- **Foundational (Phase 2)**: T007, T008, T010 can run in parallel (different utility functions)
- **User Story 1**: T017, T018 can run in parallel (updating Chapter 2 README)
- **Polish (Phase 7)**: T037, T038, T042 can run in parallel (documentation tasks, different files)

---

## Parallel Example: Setup Phase

```bash
# Launch all setup file creation together:
Task: "Create plugin entry point file at book-source/plugins/remark-slides-metadata/index.ts"
Task: "Create utility functions file at book-source/plugins/remark-slides-metadata/utils.ts"
Task: "Create transformer file at book-source/plugins/remark-slides-metadata/transformer.ts"
```

## Parallel Example: Foundational Phase

```bash
# Launch utility function implementations together:
Task: "Implement path normalization logic in book-source/plugins/remark-slides-metadata/utils.ts"
Task: "Implement heading finder in book-source/plugins/remark-slides-metadata/utils.ts"
Task: "Implement JSX node generator in book-source/plugins/remark-slides-metadata/utils.ts"
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 Only)

1. Complete Phase 1: Setup (5 tasks, ~10 mins)
2. Complete Phase 2: Foundational (6 tasks, ~30 mins - CRITICAL)
3. Complete Phase 3: User Story 1 (8 tasks, ~45 mins - Core local path support)
4. Complete Phase 4: User Story 2 (5 tasks, ~20 mins - Cloud URL support)
5. **STOP and VALIDATE**: Test both local and cloud URLs work end-to-end
6. Deploy/demo if ready (core requirement satisfied)

**Rationale**: User Stories 1 and 2 are both P1 and deliver the core value (decouple slides from markdown, support both storage types). User Stories 3 and 4 add robustness but aren't blocking for initial deployment.

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready (~40 mins)
2. Add User Story 1 → Test independently with Chapter 2 → Deploy/Demo (local paths work!)
3. Add User Story 2 → Test independently with Chapter 3 → Deploy/Demo (cloud URLs work!)
4. Add User Story 3 → Validate centralized maintenance → Deploy/Demo (maintainability proven)
5. Add User Story 4 → Validate graceful degradation → Deploy/Demo (robustness ensured)
6. Each story adds value without breaking previous stories

### Sequential Implementation (Single Developer)

**Recommended approach** (follows priority order):

1. Phase 1 (Setup) - 5 tasks
2. Phase 2 (Foundational) - 6 tasks **CRITICAL BLOCKER**
3. Phase 3 (US1: Local paths) - 8 tasks **MVP CORE**
4. Phase 4 (US2: Cloud URLs) - 5 tasks **MVP CORE**
5. Phase 5 (US3: Centralized maintenance) - 5 tasks
6. Phase 6 (US4: Validation) - 7 tasks
7. Phase 7 (Polish) - 8 tasks

**Total**: 44 tasks, estimated 2-3 hours for complete implementation

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Manual testing approach (no automated tests per research.md decision)
- Dev server hot-reload enables rapid iteration during testing
- Commit after logical groups (per phase or per story)
- Stop at any checkpoint to validate story independently before proceeding
- Production build validation is final gate (Phase 7, Task 044)

## Success Criteria Validation

After all tasks complete, validate against spec.md success criteria:

- ✅ **SC-001**: Zero markdown content changes needed (validate with Chapters 2-4)
- ✅ **SC-002**: Local paths and URLs work transparently (validate mixed chapters)
- ✅ **SC-003**: Global updates via plugin/component only (test centralized change)
- ✅ **SC-004**: All 84 chapters build successfully (`npm run build` passes)
- ✅ **SC-005**: No performance degradation (compare build times)
- ✅ **SC-006**: Graceful degradation (test invalid references produce warnings, not failures)

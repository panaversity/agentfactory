# Tasks: Automatic i18n Translation with Gemini

**Input**: Design documents from `/specs/006-i18n-auto-translate-gemini/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are OPTIONAL - not explicitly requested in spec, focusing on manual validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- Plugin code: `robolearn-interface/plugins/docusaurus-plugin-auto-translate/`
- UI components: `robolearn-interface/src/components/`
- Styles: `robolearn-interface/src/css/`
- Config: `robolearn-interface/docusaurus.config.ts`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 [P] Create plugin directory structure at `robolearn-interface/plugins/docusaurus-plugin-auto-translate/`
- [ ] T002 [P] Create lib subdirectory at `robolearn-interface/plugins/docusaurus-plugin-auto-translate/lib/`
- [ ] T003 [P] Install dependencies: `@google/generative-ai`, `gray-matter`, `glob` in `robolearn-interface/package.json`
- [ ] T004 [P] Create `.translation-cache/` directory (add to `.gitignore`)
- [ ] T005 [P] Create `robolearn-interface/plugins/docusaurus-plugin-auto-translate/README.md` with plugin documentation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 [P] Implement `lib/cache.js` - File hashing (MD5/SHA256) and cache entry management
- [ ] T007 [P] Implement `lib/file-processor.js` - File reading, frontmatter parsing with gray-matter
- [ ] T008 [P] Implement `lib/i18n-structure.js` - Path mapping and Docusaurus doc ID normalization
- [ ] T009 [P] Implement `lib/translator.js` - Gemini API client initialization and basic API call
- [ ] T010 Configure Docusaurus i18n in `robolearn-interface/docusaurus.config.ts` - Add "ur" locale with RTL config
- [ ] T011 Add plugin to `robolearn-interface/docusaurus.config.ts` plugins array (disabled initially)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Automatic Translation During Build (Priority: P1) 🎯 MVP

**Goal**: Automatically translate English content to Urdu during build process, generating i18n folder structure

**Independent Test**: Run `npm run build` with English content in `docs/` and verify `i18n/ur/docusaurus-plugin-content-docs/current/` contains translated Urdu files

### Implementation for User Story 1

- [ ] T012 [US1] Implement main plugin `index.js` with `loadContent()` hook that scans `docs/` directory
- [ ] T013 [US1] Add file scanning logic in `index.js` - Find all `.md` files using glob pattern
- [ ] T014 [US1] Integrate cache check in `index.js` - For each file, check if translation exists and is up-to-date
- [ ] T015 [US1] Implement translation prompt in `lib/translator.js` - Format prompt to preserve code blocks and technical terms
- [ ] T016 [US1] Implement Gemini API call in `lib/translator.js` - Translate content with error handling
- [ ] T017 [US1] Implement file writing logic in `lib/file-processor.js` - Write translated content to `i18n/ur/docusaurus-plugin-content-docs/current/` structure
- [ ] T018 [US1] Implement cache update in `lib/cache.js` - Store translation metadata after successful translation
- [ ] T019 [US1] Enable plugin in `robolearn-interface/docusaurus.config.ts` - Set `enabled: true` in plugin config
- [ ] T020 [US1] Add environment variable support - Read `GEMINI_API_KEY` from `process.env` in plugin config
- [ ] T021 [US1] Test translation workflow - Run build and verify translations are generated

**Checkpoint**: At this point, User Story 1 should be fully functional - translations generated automatically during build

---

## Phase 4: User Story 2 - Language Toggle UI Component (Priority: P1)

**Goal**: Provide LanguageToggle component that switches between `/en/` and `/ur/` routes

**Independent Test**: Visit any docs page, click language toggle, verify URL changes to `/ur/` route and content displays in Urdu

### Implementation for User Story 2

- [ ] T022 [P] [US2] Create `src/components/LanguageToggle/index.tsx` - LanguageToggle component with locale switching logic
- [ ] T023 [P] [US2] Create `src/components/LanguageToggle/styles.module.css` - Component styling
- [ ] T024 [US2] Implement locale detection in `LanguageToggle/index.tsx` - Get current locale from Docusaurus context
- [ ] T025 [US2] Implement locale switching in `LanguageToggle/index.tsx` - Navigate to `/ur/` or `/en/` route on click
- [ ] T026 [US2] Implement language preference persistence in `LanguageToggle/index.tsx` - Store preference in localStorage
- [ ] T027 [US2] Integrate LanguageToggle into navbar - Swizzle Docusaurus Navbar or add custom component
- [ ] T028 [US2] Test LanguageToggle functionality - Verify locale switching works correctly

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - translations exist and users can switch languages

---

## Phase 5: User Story 3 - RTL Support for Urdu (Priority: P2)

**Goal**: Display Urdu content with right-to-left layout and proper typography

**Independent Test**: View any Urdu-translated page and verify text direction is RTL, fonts render correctly, layout adapts

### Implementation for User Story 3

- [ ] T029 [P] [US3] Create `src/css/rtl.css` - RTL direction styles and typography adjustments
- [ ] T030 [US3] Import RTL CSS in `src/css/custom.css` or theme configuration
- [ ] T031 [US3] Add RTL detection in theme - Apply `dir="rtl"` attribute to HTML when locale is "ur"
- [ ] T032 [US3] Implement code block LTR override in `rtl.css` - Ensure code blocks remain left-to-right
- [ ] T033 [US3] Test RTL layout - Verify Urdu content displays correctly with RTL layout

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should work - translations, language toggle, and RTL layout all functional

---

## Phase 6: User Story 4 - Translation Caching & Performance (Priority: P2)

**Goal**: Cache translations to avoid re-translating unchanged content, improving build performance and reducing API costs

**Independent Test**: Run two builds in a row with no content changes, verify second build skips translation (no API calls)

### Implementation for User Story 4

- [ ] T034 [US4] Enhance cache check logic in `lib/cache.js` - Compare file hash with cached hash
- [ ] T035 [US4] Implement cache hit logic in `index.js` - Skip translation if cache hit
- [ ] T036 [US4] Implement cache miss logic in `index.js` - Translate only when cache miss or file changed
- [ ] T037 [US4] Add cache invalidation in `lib/cache.js` - Remove cache entries for deleted source files
- [ ] T038 [US4] Add cache statistics logging - Log cache hit/miss rates during build
- [ ] T039 [US4] Test caching behavior - Verify second build uses cache and skips API calls

**Checkpoint**: All user stories should now be independently functional - translations, UI, RTL, and caching all working

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T040 [P] Add error handling for API failures - Log errors, skip file, continue build
- [ ] T041 [P] Add error handling for rate limits - Implement retry logic with exponential backoff
- [ ] T042 [P] Implement content chunking in `lib/translator.js` - Handle large files (>100KB) by chunking
- [ ] T043 [P] Add progress logging - Show translation progress during build (e.g., "Translating 5/100 files...")
- [ ] T044 [P] Add translation quality validation - Verify code blocks and technical terms are preserved
- [ ] T045 [P] Update plugin README.md - Document configuration options and usage
- [ ] T046 [P] Add UI label translation - Translate `code.json` files for navbar, footer labels
- [ ] T047 [P] Test edge cases - API unavailable, rate limits, large files, special characters
- [ ] T048 [P] Add cleanup logic - Remove translation files when source files are deleted
- [ ] T049 [P] Performance optimization - Parallelize API calls if possible (respect rate limits)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Story 1 (P1) can start immediately after Foundational
  - User Story 2 (P1) can start after Foundational (independent of US1)
  - User Story 3 (P2) depends on US1 (needs translations to exist) and US2 (needs LanguageToggle)
  - User Story 4 (P2) depends on US1 (needs translation logic to cache)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent of US1, but both are P1
- **User Story 3 (P2)**: Depends on US1 (translations must exist) and US2 (LanguageToggle must exist)
- **User Story 4 (P2)**: Depends on US1 (translation logic must exist to cache)

### Within Each User Story

- Core implementation before integration
- File I/O before API calls
- Cache logic before translation logic
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks (T001-T005) can run in parallel
- All Foundational tasks (T006-T011) can run in parallel (within Phase 2)
- User Stories 1 and 2 can start in parallel after Foundational (both P1)
- Polish tasks (T040-T049) can run in parallel
- Different lib modules can be implemented in parallel

---

## Parallel Example: User Story 1

```bash
# Launch foundational lib modules in parallel:
Task: "Implement lib/cache.js - File hashing and cache entry management"
Task: "Implement lib/file-processor.js - File reading, frontmatter parsing"
Task: "Implement lib/i18n-structure.js - Path mapping and doc ID normalization"
Task: "Implement lib/translator.js - Gemini API client initialization"

# Then integrate in main plugin:
Task: "Implement main plugin index.js with loadContent() hook"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Automatic Translation)
4. Complete Phase 4: User Story 2 (Language Toggle)
5. **STOP and VALIDATE**: Test both stories independently
6. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (Core translation works!)
3. Add User Story 2 → Test independently → Deploy/Demo (Users can switch languages!)
4. Add User Story 3 → Test independently → Deploy/Demo (RTL support!)
5. Add User Story 4 → Test independently → Deploy/Demo (Performance optimized!)
6. Add Polish → Final improvements

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Translation logic)
   - Developer B: User Story 2 (LanguageToggle UI)
3. After US1 and US2 complete:
   - Developer A: User Story 4 (Caching)
   - Developer B: User Story 3 (RTL support)
4. Both complete Polish tasks

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Environment variable `GEMINI_API_KEY` must be set for translation to work
- Cache directory `.translation-cache/` should be gitignored


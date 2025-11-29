# Tasks: Home Page Redesign

**Input**: Design documents from `/specs/001-home-page-redesign/`
**Prerequisites**: plan.md (completed), spec.md (completed)
**Branch**: `001-home-page-redesign`
**Generated**: 2025-11-29

**Tests**: Playwright e2e tests included (per spec.md Success Evals and plan.md Phase 9)

**Organization**: Tasks organized by implementation phase from plan.md, mapped to user stories from spec.md.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task serves (US1-US5)
- Include exact file paths in descriptions

## Path Conventions

```
robolearn-interface/src/
├── pages/index.tsx                    # Main page (MODIFY)
├── pages/index.module.css             # Page styles (MAJOR REDESIGN)
├── components/HomepageComponents/     # NEW component directory
├── hooks/                             # NEW hooks directory
└── css/custom.css                     # Design tokens (MODIFY)

robolearn-interface/docusaurus.config.ts       # Font preloading (MODIFY)
tests/                                 # NEW Playwright tests
```

---

## Phase 1: Setup (Design System Foundation)

**Purpose**: Establish design tokens, typography, and project structure - BLOCKS all other work

- [ ] T001 Create component directory structure at `robolearn-interface/src/components/HomepageComponents/`
- [ ] T002 Create hooks directory at `robolearn-interface/src/hooks/`
- [ ] T003 Create tests directory at `tests/`
- [ ] T004 [P] Add Industrial-Kinetic Futurism CSS variables to `robolearn-interface/src/css/custom.css`
- [ ] T005 [P] Add Google Fonts preload tags to `robolearn-interface/docusaurus.config.ts` headTags section
- [ ] T006 [P] Add font-face declarations with fallbacks to `robolearn-interface/src/css/custom.css`
- [ ] T007 [P] Add animation keyframes (fadeInUp, fadeInLeft, shine) to `robolearn-interface/src/css/custom.css`
- [ ] T008 [P] Add reduced-motion media query styles to `robolearn-interface/src/css/custom.css`
- [ ] T009 Create `useScrollReveal` hook at `robolearn-interface/src/hooks/useScrollReveal.ts`

**Checkpoint**: Design system ready - component development can begin

---

## Phase 2: User Story 1 - First-Time Visitor Landing (Priority: P1) 🎯 MVP

**Goal**: Hero section with animated grid, book tilt, technology badges, and CTAs - the first impression that determines conversion

**Independent Test**: Load home page → verify hero displays with animations, book cover tilts on hover, CTAs navigate correctly

### Components for US1

- [ ] T010 [P] [US1] Create AnimatedGrid component at `robolearn-interface/src/components/HomepageComponents/AnimatedGrid/index.tsx`
- [ ] T011 [P] [US1] Create AnimatedGrid styles at `robolearn-interface/src/components/HomepageComponents/AnimatedGrid/styles.module.css`
- [ ] T012 [P] [US1] Create HeroSection component at `robolearn-interface/src/components/HomepageComponents/HeroSection/index.tsx`
- [ ] T013 [P] [US1] Create HeroSection styles at `robolearn-interface/src/components/HomepageComponents/HeroSection/styles.module.css`

### Implementation for US1

- [ ] T014 [US1] Implement SVG grid pattern with scan-line animation in AnimatedGrid
- [ ] T015 [US1] Implement book cover with perspective tilt effect (transform: rotateY, translateY, scale) in HeroSection
- [ ] T016 [US1] Implement technology badges (ROS 2, Gazebo & Unity, NVIDIA Isaac) with blueprint styling in HeroSection
- [ ] T017 [US1] Implement primary CTA "Start Learning" with shine hover effect in HeroSection
- [ ] T018 [US1] Implement secondary CTA "View on GitHub" with outline style in HeroSection
- [ ] T019 [US1] Add staggered fade-in animations for hero content (title, badges, CTAs)
- [ ] T020 [US1] Integrate HeroSection into `robolearn-interface/src/pages/index.tsx` replacing existing hero

### Test for US1

- [ ] T021 [US1] Create Playwright test for hero section at `tests/home-page.spec.ts` (FR-001 to FR-006)

**Checkpoint**: Hero section complete - first-time visitors see polished landing experience

---

## Phase 3: User Story 2 - Course Curriculum Discovery (Priority: P1)

**Goal**: Modules section with 4 course modules, hover glows, and badges for advanced modules

**Independent Test**: Scroll to modules → verify 4 cards display with correct info, hover shows glow, badges on modules 3-4

### Components for US2

- [ ] T022 [P] [US2] Create ModuleCard component at `robolearn-interface/src/components/HomepageComponents/ModuleCard/index.tsx`
- [ ] T023 [P] [US2] Create ModuleCard styles at `robolearn-interface/src/components/HomepageComponents/ModuleCard/styles.module.css`

### Implementation for US2

- [ ] T024 [US2] Implement ModuleCard props interface (moduleNumber, icon, title, subtitle, description, topics, weekRange, highlighted, badge)
- [ ] T025 [US2] Implement card styling with dark secondary bg, subtle border, 8px radius
- [ ] T026 [US2] Implement hover glow effect (cyan for 1-2, amber for 3-4) with translateY(-4px)
- [ ] T027 [US2] Implement badge styling (absolute top-right, amber bg, uppercase JetBrains Mono)
- [ ] T028 [US2] Implement topic list with ▸ bullet styling in accent color
- [ ] T029 [US2] Add scroll-triggered reveal with stagger delay (0.1s per card) using useScrollReveal
- [ ] T030 [US2] Create modules section in `robolearn-interface/src/pages/index.tsx` with 4 ModuleCard instances
- [ ] T031 [US2] Add focus-visible styles matching hover state for keyboard accessibility

### Test for US2

- [ ] T032 [US2] Add Playwright tests for modules section to `tests/home-page.spec.ts` (FR-007 to FR-010)

**Checkpoint**: Modules section complete - visitors can explore full curriculum

---

## Phase 4: User Story 3 - Hardware Requirements Assessment (Priority: P2)

**Goal**: Hardware section with 3 tiers, cost estimates, and RECOMMENDED badge on Cloud+Edge

**Independent Test**: Scroll to hardware → verify 3 tiers display with costs, RECOMMENDED badge on tier 2

### Components for US3

- [ ] T033 [P] [US3] Create HardwareTier component at `robolearn-interface/src/components/HomepageComponents/HardwareTier/index.tsx`
- [ ] T034 [P] [US3] Create HardwareTier styles at `robolearn-interface/src/components/HomepageComponents/HardwareTier/styles.module.css`

### Implementation for US3

- [ ] T035 [US3] Implement HardwareTier props interface (tierNumber, title, subtitle, description, costEstimate, impactLabel, recommended)
- [ ] T036 [US3] Implement card styling with dark secondary bg, amber border for recommended tier
- [ ] T037 [US3] Implement RECOMMENDED badge (absolute top center, amber bg with shadow, uppercase)
- [ ] T038 [US3] Implement tier number badge (48x48 circle, cyan/amber border based on tier)
- [ ] T039 [US3] Implement impact label pill (cyan border, rgba bg)
- [ ] T040 [US3] Create hardware section in `robolearn-interface/src/pages/index.tsx` with 3 HardwareTier instances
- [ ] T041 [US3] Add scroll-triggered reveal animation using useScrollReveal

### Test for US3

- [ ] T042 [US3] Add Playwright tests for hardware section to `tests/home-page.spec.ts` (FR-014 to FR-016)

**Checkpoint**: Hardware section complete - students can assess equipment requirements

---

## Phase 5: Features Section & CTA (Supports US1-US3)

**Goal**: Features section with custom icons and CTA block for conversion

**Independent Test**: Scroll to features → verify 6 features with SVG icons, Core badges; verify CTA button works

### Components for Features

- [ ] T043 [P] Create FeatureCard component at `robolearn-interface/src/components/HomepageComponents/FeatureCard/index.tsx`
- [ ] T044 [P] Create FeatureCard styles at `robolearn-interface/src/components/HomepageComponents/FeatureCard/styles.module.css`
- [ ] T045 [P] Create CTABlock component at `robolearn-interface/src/components/HomepageComponents/CTABlock/index.tsx`
- [ ] T046 [P] Create CTABlock styles at `robolearn-interface/src/components/HomepageComponents/CTABlock/styles.module.css`

### Custom SVG Icons (6 geometric icons)

- [ ] T047 [P] Create EmbodiedIntelligence icon at `robolearn-interface/src/components/HomepageComponents/Icons/EmbodiedIntelligence.tsx`
- [ ] T048 [P] Create HumanCentered icon at `robolearn-interface/src/components/HomepageComponents/Icons/HumanCentered.tsx`
- [ ] T049 [P] Create ProductionReady icon at `robolearn-interface/src/components/HomepageComponents/Icons/ProductionReady.tsx`
- [ ] T050 [P] Create Conversational icon at `robolearn-interface/src/components/HomepageComponents/Icons/Conversational.tsx`
- [ ] T051 [P] Create SimToReal icon at `robolearn-interface/src/components/HomepageComponents/Icons/SimToReal.tsx`
- [ ] T052 [P] Create Interactive icon at `robolearn-interface/src/components/HomepageComponents/Icons/Interactive.tsx`

### Implementation for Features & CTA

- [ ] T053 Implement FeatureCard with icon wrapper (80x80 circle, cyan border), hover rotation
- [ ] T054 Implement "Core" badge styling for featured cards (absolute top-right, cyan bg)
- [ ] T055 Implement CTABlock with gradient bg, grid pattern overlay, glow button
- [ ] T056 Implement button hover: cyan → amber transition with box-shadow change
- [ ] T057 Create features section in `robolearn-interface/src/pages/index.tsx` with 6 FeatureCard instances
- [ ] T058 Create CTA section in `robolearn-interface/src/pages/index.tsx` with CTABlock instance
- [ ] T059 Add scroll-triggered reveal for features and CTA sections

### Tests for Features & CTA

- [ ] T060 Add Playwright tests for features section to `tests/home-page.spec.ts` (FR-011 to FR-013)
- [ ] T061 Add Playwright tests for CTA section to `tests/home-page.spec.ts` (FR-017 to FR-018)

**Checkpoint**: All 5 sections complete with industrial aesthetic

---

## Phase 6: User Story 4 - Mobile Responsive Experience (Priority: P2)

**Goal**: Full responsive design across all breakpoints (320px to 2560px)

**Independent Test**: View page at 375px → verify single column layout, no horizontal scroll, touch targets >= 44px

### Implementation for US4

- [ ] T062 [US4] Add responsive breakpoints to all component CSS modules (320px, 480px, 768px, 996px, 1440px)
- [ ] T063 [US4] Implement hero layout: grid → stack at 996px breakpoint in HeroSection styles
- [ ] T064 [US4] Implement modules grid: 4-col → 2-col → 1-col in page styles
- [ ] T065 [US4] Implement features grid: 3-col → 2-col → 1-col in page styles
- [ ] T066 [US4] Implement hardware grid: 3-col → 1-col in page styles
- [ ] T067 [US4] Implement CTA layout: flex-row → flex-column at 768px in CTABlock styles
- [ ] T068 [US4] Add touch target styles (min-height: 44px) for all interactive elements
- [ ] T069 [US4] Adjust typography scale for mobile (reduce font sizes appropriately)
- [ ] T070 [US4] Test all sections at 375px viewport - no horizontal scroll

### Test for US4

- [ ] T071 [US4] Add Playwright viewport tests at 375px, 768px, 1440px to `tests/home-page.spec.ts` (FR-024, FR-025)

**Checkpoint**: Fully responsive experience across all devices

---

## Phase 7: User Story 5 - Dark/Light Mode Compatibility (Priority: P3)

**Goal**: Page respects system color preferences while maintaining industrial aesthetic

**Independent Test**: Toggle system dark/light mode → verify page adapts appropriately

### Implementation for US5

- [ ] T072 [US5] Add light mode CSS variables to `robolearn-interface/src/css/custom.css` using `[data-theme='light']`
- [ ] T073 [US5] Adjust light mode colors to maintain brand identity (lighter bg, preserved accents)
- [ ] T074 [US5] Test contrast ratios in light mode meet WCAG AA (4.5:1)
- [ ] T075 [US5] Ensure component styles adapt to theme variable changes

### Test for US5

- [ ] T076 [US5] Add Playwright dark/light mode tests to `tests/home-page.spec.ts`

**Checkpoint**: Theme compatibility complete

---

## Phase 8: Accessibility & Performance (Cross-Cutting)

**Goal**: Achieve Lighthouse 90+ performance, 95+ accessibility

### Accessibility Implementation

- [ ] T077 Add skip link to main content at top of `robolearn-interface/src/pages/index.tsx`
- [ ] T078 Add ARIA labels to decorative elements (aria-hidden="true" on AnimatedGrid)
- [ ] T079 Add landmark roles (main, section with aria-labelledby)
- [ ] T080 Add focus-visible styles (2px cyan outline, 4px offset) to all interactive elements
- [ ] T081 Verify all text meets WCAG 2.1 AA contrast (4.5:1 normal, 3:1 large)

### Performance Implementation

- [ ] T082 Convert book cover to WebP format at `robolearn-interface/static/img/book-cover-page.webp`
- [ ] T083 Add explicit width/height to book cover image (prevent CLS)
- [ ] T084 Add loading="eager" to hero images, loading="lazy" to below-fold images
- [ ] T085 Verify all animations use GPU-accelerated properties only (transform, opacity)

### Tests

- [ ] T086 Create accessibility test suite at `tests/accessibility.spec.ts` using @axe-core/playwright
- [ ] T087 Run Lighthouse audit and verify performance >= 90, accessibility >= 95

**Checkpoint**: Performance and accessibility targets met

---

## Phase 9: Final Integration & Documentation

**Purpose**: Final assembly, cleanup, and documentation

- [ ] T088 Refactor `robolearn-interface/src/pages/index.tsx` to use all new components (replace old sections)
- [ ] T089 Remove unused CSS from `robolearn-interface/src/pages/index.module.css`
- [ ] T090 Create component README at `robolearn-interface/src/components/HomepageComponents/README.md`
- [ ] T091 Document design tokens in CSS comments
- [ ] T092 Run full Playwright test suite and verify all tests pass
- [ ] T093 Run Lighthouse audit and capture final scores
- [ ] T094 Visual review: verify all 5 visual motifs present (dark bg, cyan accents, grid animation, JetBrains Mono, blueprint styling)

**Checkpoint**: Implementation complete - ready for PR

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup (BLOCKS ALL)
    ↓
Phase 2: US1 Hero (depends on Phase 1)
    ↓
Phase 3: US2 Modules (depends on Phase 1)
    ↓
Phase 4: US3 Hardware (depends on Phase 1)
    ↓
Phase 5: Features & CTA (depends on Phase 1)
    ↓
Phase 6: US4 Responsive (depends on Phases 2-5)
    ↓
Phase 7: US5 Theming (depends on Phases 2-5)
    ↓
Phase 8: A11y & Perf (depends on Phases 2-7)
    ↓
Phase 9: Integration (depends on all above)
```

### User Story Dependencies

- **US1 (Hero)**: Can start after Phase 1 - No dependencies on other stories
- **US2 (Modules)**: Can start after Phase 1 - Parallel with US1
- **US3 (Hardware)**: Can start after Phase 1 - Parallel with US1, US2
- **US4 (Responsive)**: Depends on US1-US3 being complete (needs all sections)
- **US5 (Theming)**: Depends on US1-US3 being complete (needs all CSS)

### Parallel Opportunities

**Phase 1 (Setup)**: T004, T005, T006, T007, T008 can run in parallel

**Phase 2-4 (US1-US3)**: Can run in parallel after Phase 1:
```
Developer A: US1 (Hero) - T010-T021
Developer B: US2 (Modules) - T022-T032
Developer C: US3 (Hardware) - T033-T042
```

**Phase 5 (Features)**: T043-T052 (all component/icon creation) can run in parallel

---

## Parallel Example: Phase 2 (US1 Hero)

```bash
# Create components in parallel (different files):
T010: AnimatedGrid/index.tsx
T011: AnimatedGrid/styles.module.css
T012: HeroSection/index.tsx
T013: HeroSection/styles.module.css

# Then implement sequentially:
T014 → T015 → T016 → T017 → T018 → T019 → T020 → T021
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: US1 Hero (T010-T021)
3. **STOP and VALIDATE**: Test hero section independently
4. Deploy preview if ready

### Incremental Delivery

1. Phase 1 → Design system ready
2. Phase 2 (US1) → Hero complete → Deploy preview
3. Phase 3 (US2) → Modules complete → Deploy preview
4. Phase 4 (US3) → Hardware complete → Deploy preview
5. Phase 5 → Features + CTA complete → Deploy preview
6. Phase 6-8 → Polish → Final deploy

### Estimated Time Per Phase

| Phase | Tasks | Est. Time |
|-------|-------|-----------|
| 1 Setup | 9 | 2h |
| 2 US1 Hero | 12 | 3h |
| 3 US2 Modules | 11 | 2h |
| 4 US3 Hardware | 10 | 1.5h |
| 5 Features | 19 | 3h |
| 6 US4 Responsive | 10 | 2h |
| 7 US5 Theming | 5 | 1h |
| 8 A11y & Perf | 11 | 2h |
| 9 Integration | 7 | 1.5h |
| **Total** | **94** | **18h** |

---

## Summary

- **Total Tasks**: 94
- **User Stories**: 5 (mapped to tasks)
- **Parallel Opportunities**: 40+ tasks marked [P]
- **MVP Scope**: Phase 1-2 (21 tasks, ~5h)
- **Full Scope**: All phases (94 tasks, ~18h)

### Task Count by User Story

| Story | Tasks | Priority |
|-------|-------|----------|
| US1 (Hero) | 12 | P1 |
| US2 (Modules) | 11 | P1 |
| US3 (Hardware) | 10 | P2 |
| US4 (Responsive) | 10 | P2 |
| US5 (Theming) | 5 | P3 |
| Cross-cutting | 46 | - |

### Format Validation

✅ All tasks follow checklist format: `- [ ] [ID] [P?] [Story?] Description with file path`
✅ Task IDs sequential (T001-T094)
✅ [P] markers on parallelizable tasks
✅ [US1-US5] labels on user story tasks
✅ File paths included in descriptions

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] labels map tasks to user stories for traceability
- Each user story independently testable after completion
- Commit after each task or logical group
- Stop at any checkpoint to validate progress
- Run `npm run start` in robolearn-interface to preview changes locally

# Tasks: Integrate MIT Technology Review Article into Lesson 02

**Input**: Design documents from `/specs/033-lesson-02-mit-article/`
**Prerequisites**: plan.md (ready), spec.md (ready)

**Tests**: Not applicable (educational content enhancement, no code tests)

**Organization**: Tasks are grouped by user story to enable independent implementation and content validation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Lesson file**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/02-three-trillion-developer-economy.md`
- **Summary file**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/02-three-trillion-developer-economy.summary.md`
- **Visual assets**: `book-source/static/img/part-1/chapter-1/`

---

## Phase 1: Setup (Content Preparation)

**Purpose**: Prepare MIT TR article content and verify existing lesson structure

- [ ] T001 Read current lesson file `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/02-three-trillion-developer-economy.md` to understand existing structure (259 lines, 4 concepts)
- [ ] T002 [P] Read current summary file `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/02-three-trillion-developer-economy.summary.md` to understand current mental models (23 lines)
- [ ] T003 [P] Verify MIT TR article content from Phase 0 browser retrieval (David Rotman, March 25, 2023) is available for fact-checking

**Checkpoint**: Current lesson structure understood, ready for content integration

---

## Phase 2: User Story 1 - Understanding Job Transformation Specifics (Priority: P1) 🎯

**Goal**: Add specific job impact data so students can identify which jobs are affected and why higher-income cognitive work is more vulnerable

**Independent Test**: Student can read enhanced lesson and identify 3-5 specific job categories mentioned (writers, designers, financial analysts, blockchain engineers), explaining why higher-income jobs are more vulnerable

**Success Eval**: Eval-1 (Job Impact Specificity - Target: 80%+ students)

### Implementation for User Story 1

- [ ] T004 [US1] Create new subsection "Which Jobs Face Disruption? And Why?" after "The Calculation" section (~line 107) in lesson file. **Content**: OpenAI research showing 80% workforce exposure (clarify: not elimination), 19% heavily impacted jobs, specific categories (writers, designers, financial analysts, blockchain engineers), critical distinction that exposure ≠ displacement. Estimated +20-25 lines.
- [ ] T005 [US1] Add proper MIT TR citation on first mention: "David Rotman, Editor at Large for MIT Technology Review, 'Will AI Become a Tool of Oppression?' (published March 25, 2023)" with full URL
- [ ] T006 [US1] Verify all job impact claims against browser-retrieved MIT TR article text (fact-checking protocol from Phase 5 of plan): 80% workforce exposure, 19% heavily impacted, specific job categories
- [ ] T007 [US1] Audit new subsection for A1-A2 complexity compliance: section should add 1 new concept (job vulnerability categories), bringing section total to 3 concepts (within 5-7 limit)

**Checkpoint**: Job impact section complete, properly cited, fact-checked, and complexity-compliant. Students can now identify specific vulnerable job categories.

---

## Phase 3: User Story 2 - Recognizing Two Divergent Economic Futures (Priority: P1) 🎯

**Goal**: Help students understand that AI outcomes depend on societal choices (not predetermined), presenting both optimistic (upskilling tool) and pessimistic (inequality accelerator) scenarios

**Independent Test**: Present student with hypothetical AI tool and have them articulate both potential outcomes with specific examples from lesson

**Success Eval**: Eval-2 (Two Futures Articulation - Target: 85%+ students)

### Implementation for User Story 2

- [ ] T008 [US2] Restructure "Why This Matters: Disruption at Scale" section (~line 120-144) to introduce Two Economic Futures framework. **Structure**: Part 1 (keep existing acceleration data), Part 2 (NEW: Two futures framework with optimistic and pessimistic scenarios), Part 3 (NEW: Societal choice, not technological determinism). Estimated +30-40 lines.
- [ ] T009 [US2] Create SVG diagram "two-economic-futures-branching.svg" showing branching flowchart: Present (AI Tools Emerge) → Decision Point (How Do We Implement AI?) → Branch Left (Policy A: Broad Access → Optimistic Future: expansion, shared prosperity) and Branch Right (Policy B: Restricted → Pessimistic Future: polarization, inequality). Design notes: left branch green with upward arrows, right branch red with downward arrows, decision point highlighted yellow. File location: `book-source/static/img/part-1/chapter-1/two-economic-futures-branching.svg`
- [ ] T010 [US2] Add diagram to lesson with markdown image syntax and alt text: "Branching flowchart showing AI tools leading to decision point between two policy approaches, resulting in optimistic future (shared prosperity, growing developer population) or pessimistic future (concentrated wealth, polarized opportunities)"
- [ ] T011 [US2] Verify Two Futures framework claims against MIT TR article (fact-checking): Acemoglu/Johnson framework referenced, "we need to decide what that looks like" quote attribution
- [ ] T012 [US2] Audit enhanced section for A1-A2 complexity compliance: section should add 1 new consolidated concept (choice determines outcomes), bringing section total to 2 concepts (within 5-7 limit)

**Checkpoint**: Two Economic Futures framework integrated with visual aid, properly cited, and complexity-compliant. Students can now articulate both scenarios with examples.

---

## Phase 4: User Story 3 - Evidence-Based Optimism for Career Entry (Priority: P2)

**Goal**: Provide concrete empirical evidence (MIT productivity study) that AI creates opportunity rather than displacing workers, so beginners can commit to learning with confidence

**Independent Test**: Ask student to cite three pieces of evidence from lesson supporting "expansion, not displacement" thesis

**Success Eval**: Eval-3 (Evidence-Based Career Confidence - Target: 90%+ students)

### Implementation for User Story 3

- [ ] T013 [US3] Add new paragraph to "Acceleration Paradox" section, subsection "Developer Population Growing" (~after line 175). **Content**: MIT Noy/Zhang study findings showing ChatGPT helped least-skilled workers most = skill-floor reduction, democratization of development. Estimated +15-20 lines.
- [ ] T014 [US3] Create new subsection "The Turing Trap: Replacement vs. Augmentation" after "Acceleration Paradox", before "Historical Precedent" (~line 180-182). **Content**: Definition (AI designed to mimic human = replacement), Alternative (AI designed to amplify human = augmentation), Positioning choice (compete vs. partner with AI), Learning implication (what to learn: judgment vs. syntax). Estimated +15-20 lines.
- [ ] T015 [US3] Verify MIT study claims against MIT TR article (fact-checking): Noy/Zhang study correctly referenced, Brynjolfsson Turing Trap concept accurately explained
- [ ] T016 [US3] Audit both additions for A1-A2 complexity compliance: MIT study reinforces existing "democratization" concept (0 new concepts), Turing Trap adds 1 new concept (augmentation vs. replacement positioning), total section concepts = 2 (within 5-7 limit)

**Checkpoint**: Evidence-based optimism established with MIT study + Turing Trap framework. Students can now cite empirical evidence supporting expansion thesis.

---

## Phase 5: User Story 4 - Understanding Policy and Societal Context (Priority: P3)

**Goal**: Elevate lesson from "how to position yourself" to "how society should respond" by introducing policy debates and historical context

**Independent Test**: Student can summarize 2-3 policy implications mentioned in lesson and explain rationale

**Success Eval**: Eval-5 (Historical Context Application - Target: 70%+ students)

### Implementation for User Story 4

- [ ] T017 [US4] Enhance "Historical Precedent" section (~line 182-191) by adding new subsection after printing analogy. **Title**: "Why History Suggests Two Futures Are Possible". **Content**: Post-WWII era (automation + shared institutional choices = widely shared prosperity), Recent decades (automation + different institutional choices = concentrated wealth), Implication (AI outcomes depend on policy/choices, not tech alone). Use Acemoglu/Johnson Power & Progress framework. Estimated +12-18 lines.
- [ ] T018 [US4] Enhance "What This Means For You" section (~line 193-210) with strengthened positioning guidance referencing policy context. Estimated +5-10 lines.
- [ ] T019 [US4] Verify Power & Progress framework claims against MIT TR article (fact-checking): Acemoglu/Johnson historical examples correctly referenced, post-WWII vs. recent decades distinction accurate
- [ ] T020 [US4] Audit enhanced sections for A1-A2 complexity compliance: Power & Progress reinforces existing "choice determines outcomes" concept (0 new concepts), section remains within 5-7 limit

**Checkpoint**: Policy and historical context integrated. Students now understand this is a civic/political issue, not just individual career positioning.

---

## Phase 6: Cross-Story Enhancements (Affects All User Stories)

**Purpose**: Updates that serve multiple user stories

### Try With AI Enhancement (Eval-8)

- [ ] T021 [P] Add 3 new prompts to "Try With AI" section (~line 225-257). **Prompt 5 (US2)**: "The lesson describes two possible economic futures from AI. Which future appeals to you more, and why? Ask your AI to help you think through: What would you need to do differently if the optimistic future emerged? What about the pessimistic one? How does your answer change your learning priorities right now?" **Prompt 6 (US3)**: "Ask your AI to give you examples of positioning yourself as AI's partner vs. AI's competitor in software development. What specific skills make you compete with AI (things AI is already good at) vs. partner with AI (things humans uniquely provide)?" **Prompt 7 (US3)**: "Pick ONE research finding from this lesson (OpenAI job exposure study, MIT productivity study, or Acemoglu/Johnson historical examples). Ask your AI to help you explain to a skeptic why this specific evidence shows AI is expanding, not shrinking, the developer field. Practice making an evidence-based argument." Estimated +25-30 lines.

### Summary Enhancement (Eval-8)

- [ ] T022 [P] Update summary file `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/02-three-trillion-developer-economy.summary.md` to add 3 new mental models. **Mental Model 4**: "Two Economic Futures: AI outcomes depend on policy choices and societal decisions, not technological determinism. Optimistic path: broad access, upskilling, shared prosperity. Pessimistic path: restricted access, replacement, concentrated wealth." **Mental Model 5**: "Turing Trap: Building AI that mimics humans leads to replacement. Building AI that amplifies humans leads to augmentation. Your positioning choice determines career trajectory." **Mental Model 6**: "Power & Progress: Historical precedent shows technology impact depends on institutional choices (post-WWII shared prosperity vs. recent concentrated wealth)." Estimated +8-12 lines.

### Frontmatter Update

- [ ] T023 [P] Update lesson frontmatter YAML metadata (~line 1-53): Increment `new_concepts` from 4 to 7, update `cognitive_load.assessment` to reflect 3 new concepts added (job vulnerability categories, two futures framework, Turing Trap positioning), add new learning objective: "Recognize that AI's economic impact depends on societal choices, not technological determinism" (proficiency A2, Bloom level: Evaluate)

**Checkpoint**: All cross-story enhancements complete. Lesson now has 7 prompts, updated summary with 6 mental models, and accurate frontmatter metadata.

---

## Phase 7: Validation & Quality Assurance

**Purpose**: Comprehensive validation before publication (Phase 5 from plan)

### Fact-Checking (Eval-6: 100% Factual Accuracy)

- [ ] T024 Verify all 7 factual claims against browser-retrieved MIT TR article: (1) 80% workforce exposure, 19% heavily impacted [OpenAI research], (2) Job categories: writers, designers, financial analysts, blockchain engineers, (3) MIT Noy/Zhang study findings, (4) Brynjolfsson Turing Trap concept, (5) Acemoglu/Johnson Power & Progress framework, (6) David Rotman author attribution, (7) Publication date: March 25, 2023. Document verification in checklist format.

### Complexity Compliance Audit (Eval-7: 100% of Sections)

- [ ] T025 Audit section-by-section concept count: The Calculation (3 concepts: 2 existing + 1 job categories), Why This Matters (2 concepts: 1 existing + 1 two futures), Acceleration Paradox (2 concepts: 1 existing + 1 Turing Trap), Historical Precedent (0 new concepts, reinforces choice framework), What This Means For You (0 new concepts, reinforces positioning). **Total lesson concepts: 7 (within A1-A2 limit of 5-7) ✓**. Language audit: no unexplained jargon, concrete examples for complex concepts.

### Narrative Flow Testing (Eval-7: Organic Integration)

- [ ] T026 Read enhanced lesson end-to-end to test narrative coherence. **Transition quality checkpoints**: (1) Calculation → Job Impact (smooth pivot from aggregate value to specific categories), (2) Job Impact → Two Futures (natural progression: impact feeds futures discussion), (3) Two Futures → Paradox ("Futures depend on choices" connects to "developers growing"), (4) Paradox → Turing Trap ("Growth" assertion → "replacement risk" counternarrative), (5) Turing Trap → Precedent (strategic positioning → historical examples). **Pass criteria**: Lesson reads as unified whole with no abrupt shifts, new content feels organically integrated (not bolted-on).

### Citation Completeness Check (Eval-6: 100% Attribution)

- [ ] T027 Verify citation completeness: (1) Full citation on first MIT TR mention (author, publication, date, URL), (2) Secondary citations properly formatted throughout, (3) All article-sourced claims attributed, (4) Consistency in citation format. **Pass criteria**: 100% of claims properly cited.

### Success Evals Verification

- [ ] T028 Verify all 8 Success Evals can be achieved: (1) Eval-1: Students can identify 3+ job categories ✓ (US1 complete), (2) Eval-2: Students can articulate both futures with examples ✓ (US2 complete), (3) Eval-3: Students can cite 2+ empirical findings ✓ (US3 complete), (4) Eval-4: Students can explain Turing Trap ✓ (US3 complete), (5) Eval-5: Students can reference Power & Progress ✓ (US4 complete), (6) Eval-6: 100% factual accuracy ✓ (T024 complete), (7) Eval-7: 100% complexity compliance ✓ (T025 complete), (8) Eval-8: 3+ new reflection prompts ✓ (T021 complete)

**Checkpoint**: All validation complete. Lesson ready for publication.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **User Story 1 (Phase 2)**: Depends on Setup (T001-T003) - Adds job impact specificity
- **User Story 2 (Phase 3)**: Depends on Setup (T001-T003) - Adds two futures framework (can run parallel with US1)
- **User Story 3 (Phase 4)**: Depends on Setup (T001-T003) - Adds evidence-based optimism (can run parallel with US1, US2)
- **User Story 4 (Phase 5)**: Depends on Setup (T001-T003) - Adds policy context (can run parallel with US1, US2, US3)
- **Cross-Story Enhancements (Phase 6)**: Depends on US1-US4 completion - Adds Try With AI prompts, summary updates, frontmatter
- **Validation (Phase 7)**: Depends on all implementation complete - Final quality assurance before publication

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Setup - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Setup - Independent of US1 (different section)
- **User Story 3 (P2)**: Can start after Setup - Independent of US1, US2 (different section)
- **User Story 4 (P3)**: Can start after Setup - Independent of US1, US2, US3 (different section)

**All user stories are independently implementable and testable** (different lesson sections, no content dependencies)

### Within Each User Story

- Content addition before fact-checking
- Fact-checking before complexity audit
- All story tasks complete before moving to cross-story enhancements

### Parallel Opportunities

- **Setup phase**: All 3 tasks marked [P] can run in parallel (T002, T003 independent of T001)
- **User Stories 1-4**: All can run in parallel after Setup completes (different lesson sections, no dependencies)
- **Cross-Story Enhancements**: All 3 tasks marked [P] can run in parallel (T021, T022, T023 affect different files)
- **Maximum parallelism**: 4 user stories + 3 cross-story tasks = 7 tasks simultaneously (if team capacity allows)

---

## Parallel Example: All User Stories After Setup

```bash
# After Phase 1 (Setup) completes, launch all 4 user stories simultaneously:

Task T004-T007: "User Story 1 - Job Impact Specificity" (file: lesson sections ~line 107-120)
Task T008-T012: "User Story 2 - Two Futures Framework" (file: lesson sections ~line 120-144 + diagram)
Task T013-T016: "User Story 3 - Evidence-Based Optimism" (file: lesson sections ~line 175-182)
Task T017-T020: "User Story 4 - Policy Context" (file: lesson sections ~line 182-210)

# Each story works on different lesson sections, no file conflicts
# Once all 4 complete, launch cross-story enhancements in parallel:

Task T021: "Try With AI prompts" (file: lesson ~line 225-257)
Task T022: "Summary update" (file: summary.md)
Task T023: "Frontmatter update" (file: lesson YAML ~line 1-53)
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only - Both P1)

1. Complete Phase 1: Setup
2. Complete Phase 2: User Story 1 (Job Impact)
3. Complete Phase 3: User Story 2 (Two Futures)
4. Complete Phase 6: Cross-Story Enhancements (prompts, summary, frontmatter)
5. Complete Phase 7: Validation
6. **STOP and VALIDATE**: Test that students can achieve Eval-1 and Eval-2
7. Publish MVP if validation passes

### Incremental Delivery

1. Complete Setup → Context understood
2. Add US1 (Job Impact) → Students can identify vulnerable jobs → Validate independently
3. Add US2 (Two Futures) → Students understand choice framework → Validate independently
4. Add US3 (Evidence) → Students gain career confidence → Validate independently
5. Add US4 (Policy) → Students recognize civic dimension → Validate independently
6. Add Cross-Story Enhancements → Complete experience with prompts and summary
7. Run Validation → Final quality assurance → Publish

### Sequential Strategy (Single Author)

Recommended execution order for one content author:

1. Phase 1: Setup (T001-T003) - 15 minutes
2. Phase 2: User Story 1 (T004-T007) - 1.5 hours
3. Phase 3: User Story 2 (T008-T012) - 2.5 hours (includes diagram creation)
4. Phase 4: User Story 3 (T013-T016) - 1.5 hours
5. Phase 5: User Story 4 (T017-T020) - 1 hour
6. Phase 6: Cross-Story (T021-T023) - 1.5 hours
7. Phase 7: Validation (T024-T028) - 2 hours

**Total estimated time: 10 hours** (matches plan.md resource requirements)

---

## Notes

- [P] tasks = different files or sections, no dependencies, can run in parallel
- [Story] label (US1, US2, US3, US4) maps task to specific user story for traceability
- Each user story enhances different lesson section, enabling independent implementation
- No tests included (educational content, not code)
- Commit after each user story phase completion
- All claims must be verified against MIT TR article before publication (Eval-6: 100% factual accuracy)
- Final lesson: 380-420 lines (current 259 + 120-160 new), 7 total concepts (A1-A2 compliant)

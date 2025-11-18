# Tasks: Chapter 9 Markdown Redesign - Specification Language Approach

**Input**: Design documents from `/specs/034-chapter-9-markdown-redesign/`
**Prerequisites**: plan.md (lesson breakdown), spec.md (user stories and requirements)
**Target Directory**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/`

**Tests**: Not applicable for educational content (quality validation through validation-auditor agent instead)

**Organization**: Tasks are grouped by lesson (user story equivalent) to enable independent lesson implementation and validation.

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (e.g., L1, L2, L3, L4, L5)
- Include exact file paths in descriptions

## Path Conventions

- **Chapter directory**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/`
- **Lesson files**: `01-lesson-title.md`, `02-lesson-title.md`, etc.
- **Chapter README**: `README.md` in chapter directory

---

## Phase 1: Setup (Chapter Structure)

**Purpose**: Initialize chapter directory structure and validate prerequisites

- [ ] T001 Validate chapter directory exists at `book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/`
- [ ] T002 Read existing README.md to preserve chapter overview structure
- [ ] T003 Read existing lesson 04-code-blocks.md to identify content that violates no-code constraint (Python/Bash examples)
- [ ] T004 [P] Backup existing lessons to `specs/034-chapter-9-markdown-redesign/backup/` directory

**Checkpoint**: Directory structure validated, existing content analyzed

---

## Phase 2: Foundational (Chapter-Level Updates)

**Purpose**: Update chapter-level documentation and establish teaching framework

**⚠️ CRITICAL**: These tasks establish the specification-first modality for ALL lessons

- [ ] T005 Update README.md with new chapter overview (specification language focus, 5-lesson structure, no-code constraint)
- [ ] T006 [P] Document lesson ending protocol: ONLY "Try With AI" section permitted (no "What's Next", "Key Takeaways", "Summary")
- [ ] T007 [P] Create constitutional compliance checklist in `specs/034-chapter-9-markdown-redesign/validation-checklist.md`

**Checkpoint**: Chapter framework established - lesson implementation can now begin in parallel

---

## Phase 3: User Story 1 - Understand Markdown as Specification Language (Priority: P1) 🎯 Foundation

**Goal**: Lesson 1 teaches markdown basics as specification language (Stage 1: Manual Foundation)

**Independent Test**: Lesson includes specification examples only, zero code examples, manual validation exercises

**Mapped Requirements**: FR-007, FR-008, FR-009, SC-001, SC-003

### Implementation for Lesson 1

- [ ] T008 [L1] Create `01-markdown-as-specification-language.md` with frontmatter (duration: 45 min, concepts: 6, proficiency: A2, stage: 1)
- [ ] T009 [L1] Write "Introduction" section: Markdown as Intent Layer of SDD-RI (not formatting tool)
- [ ] T010 [L1] Write "Concept 1: Markdown as Structured Communication" section with specification vs implementation examples
- [ ] T011 [L1] Write "Concept 2: Heading Hierarchy for Document Structure" section with Feature > Sub-feature > Requirement nesting examples
- [ ] T012 [L1] Write "Concept 3: Manual Validation" section with correct/broken examples for students to identify
- [ ] T013 [L1] Write "Practice Exercise: Create Your First Specification" section (e-book reader OR recipe collection OR daily planner)
- [ ] T014 [L1] Write "Try With AI" section: Validation-focused (student creates markdown manually, AI checks syntax)
- [ ] T015 [L1] Validate zero code examples present (grep check for `def `, `function `, `import `, `const `, `#!/bin/bash`)
- [ ] T016 [L1] Validate concept count = 6 in frontmatter matches actual content
- [ ] T017 [L1] Validate NO sections after "Try With AI" (no "What's Next", "Key Takeaways")

**Checkpoint**: Lesson 1 complete, foundation established for manual markdown writing

---

## Phase 4: User Story 2 - Write Structured Requirements Using Lists (Priority: P1)

**Goal**: Lesson 2 teaches lists for requirements (Stage 2: AI Collaboration begins with Three Roles)

**Independent Test**: Lesson demonstrates Three Roles (AI as Teacher/Student/Co-Worker) through narrative, not labels

**Mapped Requirements**: FR-010, FR-011, FR-014, SC-004, SC-005

### Implementation for Lesson 2

- [ ] T018 [L2] Create `02-lists-for-requirements.md` with frontmatter (duration: 50 min, concepts: 7, proficiency: A2, stage: 2)
- [ ] T019 [L2] Write "Introduction" section: Lists for discrete requirements vs continuous prose
- [ ] T020 [L2] Write "Concept 1: Bullet Lists for Options" section with feature enumeration examples
- [ ] T021 [L2] Write "Concept 2: Numbered Lists for Sequential Steps" section with implementation sequence examples
- [ ] T022 [L2] Write "Concept 3: Option vs Sequence Distinction" section with when-to-use guidance
- [ ] T023 [L2] Write "Three Roles Demonstration" section: AI suggests list structure → Student refines with MVP constraints → Convergence produces superior organization
- [ ] T024 [L2] Write "Practice Exercise: Convert Feature Description to Lists" section (unstructured → structured markdown)
- [ ] T025 [L2] Write "Try With AI" section: Collaborative prompt where student receives AI suggestions and refines them
- [ ] T026 [L2] Validate Three Roles narrative present (phrases like "AI suggested...", "You refined by...", "Together you converged on...")
- [ ] T027 [L2] Validate zero code examples present (grep check)
- [ ] T028 [L2] Validate NO internal labels ("Stage 2", "Three Roles Framework" as section headers)
- [ ] T029 [L2] Validate concept count = 7 in frontmatter matches actual content

**Checkpoint**: Lesson 2 complete, AI collaboration demonstrated through lists

---

## Phase 5: User Story 3 - Use Code Blocks for Specifications (Not Code) (Priority: P1)

**Goal**: Lesson 3 teaches code blocks for specifications (Stage 2: AI Collaboration deepened with Three Roles)

**Independent Test**: Code blocks show expected outputs, API descriptions, feature requirements - ZERO implementation code

**Mapped Requirements**: FR-004, FR-006, FR-010, FR-012, FR-013, FR-015, SC-005

### Implementation for Lesson 3

- [ ] T030 [L3] Create `03-code-blocks-for-specifications.md` with frontmatter (duration: 55 min, concepts: 7, proficiency: A2, stage: 2)
- [ ] T031 [L3] Write "Introduction" section: Code blocks for specifications (expected outputs, API descriptions) NOT code
- [ ] T032 [L3] Write "Concept 1: Code Block Syntax (Triple Backticks)" section with specification examples
- [ ] T033 [L3] Write "Concept 2: Expected Output Specifications" section showing system behavior examples
- [ ] T034 [L3] Write "Concept 3: API Endpoint Descriptions" section showing request/response format without implementation
- [ ] T035 [L3] Write "Three Roles Iteration" section: AI suggests output structure → Student teaches domain constraints → Converge on clarity
- [ ] T036 [L3] Write "Practice Exercise: Add Expected Outputs to Feature Spec" section (building on Lesson 2 lists)
- [ ] T037 [L3] Write "Try With AI" section: Iteration-focused (student refines AI suggestions, demonstrates convergence)
- [ ] T038 [L3] Replace ALL code examples from existing Lesson 4 (04-code-blocks.md) with specification examples
- [ ] T039 [L3] Validate ZERO programming code examples (Python, Bash, JavaScript, TypeScript syntax)
- [ ] T040 [L3] Validate code blocks use specification context (preceded by "Expected output:", "API specification:", "Feature requirements:")
- [ ] T041 [L3] Validate Three Roles convergence loop demonstrated
- [ ] T042 [L3] Validate concept count = 7 in frontmatter matches actual content

**Checkpoint**: Lesson 3 complete, no-code constraint validated, code blocks reframed as specification tool

---

## Phase 6: User Story 4 - Create Reusable Specification Templates (Priority: P2)

**Goal**: Lesson 4 teaches links, images, and reusable template creation (Stage 3: Intelligence Design)

**Independent Test**: Lesson guides students to create at least one reusable template using Persona + Questions + Principles pattern

**Mapped Requirements**: FR-016, FR-017, FR-018, FR-019, SC-005

### Implementation for Lesson 4

- [ ] T043 [L4] Create `04-links-images-and-templates.md` with frontmatter (duration: 50 min, concepts: 5, proficiency: A2, stage: 3)
- [ ] T044 [L4] Write "Introduction" section: References and reusable intelligence in specifications
- [ ] T045 [L4] Write "Concept 1: Links for Reference Materials" section with `[text](url)` syntax and authoritative source examples
- [ ] T046 [L4] Write "Concept 2: Images for Diagrams" section with `![alt](url)` syntax (NO Mermaid per user decision)
- [ ] T047 [L4] Write "Concept 3: Creating Reusable Templates" section: Persona + Questions + Principles pattern
- [ ] T048 [L4] Write "Template Example: Feature Specification Template" section with complete template structure
- [ ] T049 [L4] Write "Practice Exercise: Create Your Own Template" section (feature template OR acceptance criteria template OR user story template)
- [ ] T050 [L4] Write "Try With AI" section: Template validation (AI provides feedback on template reusability)
- [ ] T051 [L4] Validate Persona + Questions + Principles pattern demonstrated in template example
- [ ] T052 [L4] Validate NO Mermaid diagram syntax present
- [ ] T053 [L4] Validate zero code examples present (grep check)
- [ ] T054 [L4] Validate concept count = 5 in frontmatter matches actual content

**Checkpoint**: Lesson 4 complete, intelligence design demonstrated through template creation

---

## Phase 7: User Story 5 - Write Complete System Specification (Capstone) (Priority: P1)

**Goal**: Lesson 5 capstone composes all skills (Stage 4: Spec-Driven Integration) - task management system specification

**Independent Test**: Capstone specification includes headings, lists, code blocks, links - validated through AI feedback without implementation

**Mapped Requirements**: FR-020, FR-021, FR-022, FR-023, SC-002, SC-005, SC-007

### Implementation for Lesson 5

- [ ] T055 [L5] Create `05-complete-system-specification.md` with frontmatter (duration: 75 min, concepts: 8, proficiency: A2, stage: 4)
- [ ] T056 [L5] Write "Introduction" section: Capstone project synthesizing all markdown skills
- [ ] T057 [L5] Write "Capstone Project: Task Management System Specification" section with clear scope (3-4 features: create, view, mark complete, delete)
- [ ] T058 [L5] Write "Component 1: System Overview Using Headings" section (hierarchical organization practice)
- [ ] T059 [L5] Write "Component 2: Feature List Using Lists" section (enumerated requirements practice)
- [ ] T060 [L5] Write "Component 3: Expected Behavior Using Code Blocks" section (output specifications practice)
- [ ] T061 [L5] Write "Component 4: Reference Materials Using Links" section (authoritative source practice)
- [ ] T062 [L5] Write "Validation Checklist" section: Specification completeness criteria (headings ✓, lists ✓, code blocks ✓, links ✓)
- [ ] T063 [L5] Write "Try With AI" section: AI feedback on specification clarity (NOT implementation) - iteration without code
- [ ] T064 [L5] Validate capstone scope appropriate for A2 tier (3-4 features, not overwhelming)
- [ ] T065 [L5] Validate capstone deliverable includes ALL components (headings, lists, code blocks, links)
- [ ] T066 [L5] Validate exercise explicitly states "Do NOT ask AI to implement code"
- [ ] T067 [L5] Validate zero code examples present (grep check)
- [ ] T068 [L5] Validate concept count = 8 in frontmatter matches actual content

**Checkpoint**: Lesson 5 complete, capstone demonstrates mastery of markdown as specification language

---

## Phase 8: Cross-Lesson Validation & Constitutional Compliance

**Purpose**: Validate chapter-wide compliance with constitutional frameworks and requirements

- [ ] T069 [P] Run grep validation across all 5 lessons for forbidden code patterns: `grep -rE "(def |function |import |const |#!/bin/bash)" book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/*.md`
- [ ] T070 [P] Validate lesson ending protocol: Each lesson ends with "## Try With AI" as ONLY final section (no "What's Next", "Key Takeaways", "Summary")
- [ ] T071 [P] Validate NO internal scaffolding labels in student-facing text: `grep -rE "(Stage [0-9]|Layer [0-9]|Three Roles (Framework|in Action))" *.md`
- [ ] T072 [P] Validate concept counts in frontmatter: L1=6, L2=7, L3=7, L4=5, L5=8 (all ≤ 7 except capstone)
- [ ] T073 [P] Validate Three Roles narrative present in Lessons 2-3 (not labels, narrative phrases)
- [ ] T074 [P] Validate Persona + Questions + Principles pattern in Lesson 4 template
- [ ] T075 [P] Validate capstone composition in Lesson 5 (uses skills from L1+L2+L3+L4)
- [ ] T076 Validate CommonMark compliance (all markdown syntax follows CommonMark standard, GFM variants noted)
- [ ] T077 Validate SDD-RI methodology positioning (markdown as Intent Layer explicitly stated in Lesson 1)
- [ ] T078 Create validation report in `specs/034-chapter-9-markdown-redesign/validation-report.md` documenting all checks

**Checkpoint**: All constitutional constraints validated, chapter ready for content-evaluation-framework assessment

---

## Phase 9: Polish & Meta-Documentation

**Purpose**: Finalize chapter documentation and create implementation artifacts

- [ ] T079 [P] Update chapter README.md with final lesson titles and durations
- [ ] T080 [P] Document anti-convergence variation from Chapter 8: Specification-first modality vs hands-on discovery
- [ ] T081 [P] Create learning pathway map showing 4-stage progression across 5 lessons
- [ ] T082 [P] Document intelligence harvest: Feature Specification Template (from Lesson 4) as reusable artifact
- [ ] T083 Generate implementation summary for PHR creation
- [ ] T084 Run final quality check using validation-auditor agent

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all lesson implementation
- **Lessons (Phase 3-7)**: All depend on Foundational phase completion
  - Lessons can then proceed in parallel (L1, L2, L3, L4, L5 are independent files)
  - Or sequentially in order (L1 → L2 → L3 → L4 → L5 for pedagogical coherence)
- **Validation (Phase 8)**: Depends on ALL lessons being complete
- **Polish (Phase 9)**: Depends on Validation completion

### Lesson Dependencies

- **Lesson 1 (L1)**: Can start after Foundational (Phase 2) - No dependencies on other lessons
- **Lesson 2 (L2)**: Can start after Foundational - Builds on L1 conceptually but file is independent
- **Lesson 3 (L3)**: Can start after Foundational - Builds on L2 conceptually but file is independent
- **Lesson 4 (L4)**: Can start after Foundational - Builds on L1-L3 conceptually but file is independent
- **Lesson 5 (L5)**: Can start after Foundational - Composes L1-L4 but file is independent

### Within Each Lesson

- Introduction section before concept sections
- Concept sections before practice exercises
- Practice exercises before "Try With AI"
- "Try With AI" MUST be final section
- Frontmatter validation after content complete

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all 5 lessons can be written in parallel (if team capacity allows)
- All validation tasks marked [P] can run in parallel (within Phase 8)
- All polish tasks marked [P] can run in parallel (within Phase 9)

---

## Parallel Example: Lesson Implementation

```bash
# After Foundational phase complete, launch all lessons in parallel:
Task: "Create 01-markdown-as-specification-language.md" [L1]
Task: "Create 02-lists-for-requirements.md" [L2]
Task: "Create 03-code-blocks-for-specifications.md" [L3]
Task: "Create 04-links-images-and-templates.md" [L4]
Task: "Create 05-complete-system-specification.md" [L5]

# Or validation tasks in parallel:
Task: "Run grep validation for code patterns" [P]
Task: "Validate lesson ending protocol" [P]
Task: "Validate concept counts in frontmatter" [P]
```

---

## Implementation Strategy

### Sequential Approach (Recommended for Pedagogical Coherence)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - establishes teaching framework)
3. Complete Phase 3: Lesson 1 (Foundation)
4. Complete Phase 4: Lesson 2 (AI Collaboration starts)
5. Complete Phase 5: Lesson 3 (AI Collaboration deepens)
6. Complete Phase 6: Lesson 4 (Intelligence Design)
7. Complete Phase 7: Lesson 5 (Capstone)
8. Complete Phase 8: Validation
9. Complete Phase 9: Polish

**Why sequential?** Each lesson builds conceptually on previous lessons. Testing progression ensures coherent learning pathway.

### Parallel Approach (If Multiple Content Authors Available)

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: Lesson 1 + Lesson 2
   - Author B: Lesson 3 + Lesson 4
   - Author C: Lesson 5 + Validation
3. Team reviews for consistency
4. Complete Polish together

---

## Notes

### Task Labeling

- **[P]** tasks = different files, no dependencies
- **[Lesson]** label maps task to specific lesson for traceability
- Each lesson should be independently completable once Foundational phase is complete

### Quality Gates

- **After each lesson**: Run grep validation to catch code examples early
- **After Phase 8**: Full constitutional compliance validation before polish
- **Before completion**: Run validation-auditor agent for comprehensive quality check

### Constitutional Compliance Reminders

- **Forbidden**: Code examples (Python, Bash, JavaScript, any programming syntax)
- **Forbidden**: Sections after "Try With AI" (no "What's Next", "Key Takeaways")
- **Forbidden**: Internal labels in student-facing text ("Stage 1/2/3/4", "Three Roles Framework")
- **Required**: Specification examples ONLY (feature lists, acceptance criteria, expected outputs)
- **Required**: A2 tier cognitive load (5-7 concepts per section, maximum 7 with scaffolding)
- **Required**: Three Roles demonstrated in Lessons 2-3 through narrative
- **Required**: Persona + Questions + Principles pattern in Lesson 4 template
- **Required**: Capstone composition in Lesson 5 (uses all markdown skills)

### Anti-Pattern Detection

- Watch for: Code-centric language ("implement this", "run this code")
- Watch for: Generic markdown tutorial language ("make pretty READMEs")
- Watch for: Missing specification context in code blocks
- Ensure: Specification-first language ("describe WHAT system does", "communicate intent to AI")

---

## Validation Checklist Summary

After all tasks complete, verify:

- ✅ Zero code examples across all 5 lessons (AT-001)
- ✅ All lessons end with "Try With AI" ONLY (AT-002)
- ✅ No internal scaffolding labels in student-facing text (AT-003)
- ✅ All code blocks show specifications, not implementation (AT-004)
- ✅ Lesson 1 demonstrates Stage 1 (Manual Foundation) (AT-005)
- ✅ Lessons 2-3 demonstrate Three Roles through narrative (AT-006)
- ✅ Lesson 4 demonstrates Intelligence Design with template (AT-007)
- ✅ Lesson 5 demonstrates Spec-Driven Integration with capstone (AT-008)
- ✅ Concept counts comply with A2 tier limits (AT-009)
- ✅ Heavy scaffolding present in all lessons (AT-010)
- ✅ Maximum 2 options at decision points (AT-011)
- ✅ Capstone includes all required components (AT-012)
- ✅ Capstone scope appropriate for A2 tier (AT-013)
- ✅ Capstone demonstrates specification clarity through AI feedback (AT-014)
- ✅ CommonMark compliance validated (AT-015)
- ✅ GFM variants noted where applicable (AT-016)
- ✅ Markdown positioned as Intent Layer of SDD-RI (AT-017)
- ✅ Lessons demonstrate how AI agents parse markdown structure (AT-018)
- ✅ Capstone shows specification → AI feedback workflow (AT-019)
- ✅ Lessons do NOT converge toward generic markdown tutorials (AT-020)
- ✅ Code blocks NOT treated as "for code only" (AT-021)

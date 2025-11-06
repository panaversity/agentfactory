# Tasks: Chapter 9 - Markdown: The Language of AI Communication

**Input**: Design documents from `/specs/001-chapter-9-markdown/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: No automated tests required for educational content (manual validation via exercises and assessments)

**Organization**: Tasks organized by user story (learning outcomes) to enable independent lesson development and validation

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (learning outcome) this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Path Conventions

**Chapter Location**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/`

**File Structure**:
```
09-markdown-language-of-ai/
├── 01-why-markdown-matters.md
├── 02-headings-hierarchy.md
├── 03-lists-organizing-ideas.md
├── 04-code-blocks-examples.md
├── 05-emphasis-links-first-spec.md
├── 06-complex-markdown-ai-companion.md
├── 07-scaling-markdown-orchestration.md
└── 08-full-aidd-cycle.md
```

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Initialize chapter directory structure and shared assets

- [ ] T001 Create chapter directory at `book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/`
- [ ] T002 [P] Create YAML frontmatter template for all 8 lessons with consistent metadata (chapter: 9, part: 3, sidebar_position, skills metadata fields)
- [ ] T003 [P] Create example markdown files directory at `09-markdown-language-of-ai/examples/` for code samples referenced across lessons

---

## Phase 2: Foundational (Prerequisites for All Lessons)

**Purpose**: Core teaching materials that ALL lessons depend on

**⚠️ CRITICAL**: No lesson writing can begin until this phase is complete

- [ ] T004 Create common mistakes reference document in `09-markdown-language-of-ai/examples/common-errors.md` covering edge cases from spec (empty headings, unclosed code blocks, invalid links, nested formatting conflicts)
- [ ] T005 [P] Create validation guide in `09-markdown-language-of-ai/examples/validation-checklist.md` for students to verify markdown renders correctly (GitHub preview, local render, AI parsing test)
- [ ] T006 [P] Create markdown syntax quick reference card in `09-markdown-language-of-ai/examples/syntax-reference.md` for Tier 1 patterns (headings, lists, code blocks, emphasis, links)

**Checkpoint**: Foundation ready - lesson writing can now begin in parallel

---

## Phase 3: User Story 1 - Write Clear Project Specifications (Priority: P1) 🎯 MVP

**Goal**: Students can write specification documents in markdown that AI agents can parse and implement from

**Independent Test**: Student writes README.md with headings, lists, and code blocks → AI agent successfully generates code from spec (SC-001: 90%+ achieve this)

**Lessons Covering US1**: Lessons 1-5 (Foundation + All Tier 1 skills)

### Lesson 1: Why Markdown Matters (Foundation)

- [ ] T007 [US1] Write lesson 1 content in `01-why-markdown-matters.md` covering:
  - Introduction: Markdown as specification language (FR-009, FR-018)
  - Concept 1: Markdown as structured text (human-readable, machine-parseable)
  - Concept 2: Markdown's role in AIDD Intent Layer (specs → AI reasoning → implementation)
  - Real-world context: GitHub README, AI prompt engineering, technical documentation
  - YAML frontmatter: `title`, `description`, `duration: 40min`, `proficiency: A1`, `concepts: 2`, `skills: [Understanding Markdown's Role (A1 - Remember), Recognizing Specification Intent (A1 - Understand)]`
- [ ] T008 [US1] Add "Try With AI" section to lesson 1: Student asks ChatGPT web "Why does markdown matter for working with AI coding agents?" and reflects on response alignment with lesson
- [ ] T009 [US1] Create discussion prompts for lesson 1 formative assessment: What is the Intent Layer? How does structured text help AI understand requirements?

### Lesson 2: Headings - Creating Document Hierarchy (Tier 1)

- [ ] T010 [P] [US1] Write lesson 2 content in `02-headings-hierarchy.md` covering:
  - Concept 1: `#` syntax (one `#` = Level 1 heading, two `##` = Level 2, etc.)
  - Concept 2: Hierarchy principle (Level 1 for title, Level 2 for sections, Level 3 for subsections)
  - Direct teaching: Show examples, explain when to use each level
  - Code Example 1: Simple README with proper hierarchy (Problem → Solution → Features)
  - Code Example 2: Wrong hierarchy (skipping levels - error pattern to avoid)
  - Exercise: Create heading structure for student's own project idea
  - YAML frontmatter: `duration: 40min`, `proficiency: A2`, `concepts: 2`, `skills: [Creating Heading Hierarchy (A2 - Apply), Understanding Document Structure (A2 - Understand)]`
- [ ] T011 [US1] Add "Try With AI" section to lesson 2: Student creates heading structure → shares with ChatGPT → asks "Is this structure clear for someone reading my project spec?"
- [ ] T012 [US1] Create syntax validation checklist for lesson 2: Correct `#` count, no empty headings, logical hierarchy (no Level 3 before Level 2)

### Lesson 3: Lists - Organizing Ideas (Tier 1)

- [ ] T013 [P] [US1] Write lesson 3 content in `03-lists-organizing-ideas.md` covering:
  - Concept 1: Unordered list syntax (`-` or `*` for bullet points)
  - Concept 2: Ordered list syntax (`1.`, `2.`, `3.` for numbered steps)
  - Concept 3: Selection principle (bullets for unordered features, numbers for sequential steps)
  - Direct teaching: When to use unordered vs ordered
  - Code Example 1: Feature list (unordered - no specific sequence)
  - Code Example 2: Installation steps (ordered - must follow sequence)
  - Code Example 3: Mixed markers error (inconsistent `-`, `*`, `+` - teach consistency)
  - Exercise: Create feature list (unordered) and installation steps (ordered) for project
  - YAML frontmatter: `duration: 40min`, `proficiency: A2`, `concepts: 3`, `skills: [Creating Unordered Lists (A2 - Apply), Creating Ordered Lists (A2 - Apply), Choosing List Type (A2 - Analyze)]`
- [ ] T014 [US1] Add "Try With AI" section to lesson 3: Student shares lists → asks ChatGPT "Are these lists clear? Should any be ordered instead of unordered?"
- [ ] T015 [US1] Create exercise template for lesson 3: Fill-in structure with [Your project feature 1], [Installation step 1], etc.

### Lesson 4: Code Blocks - Showing Examples (Tier 1)

- [ ] T016 [P] [US1] Write lesson 4 content in `04-code-blocks-examples.md` covering:
  - Concept 1: Fenced code blocks (triple backticks ` ``` ` opening and closing)
  - Concept 2: Language tags (` ```python `, ` ```typescript `, ` ```bash ` for syntax highlighting)
  - Concept 3: Inline code (single backticks `` `variable` `` for short snippets)
  - Concept 4: Code block purpose (show expected output, provide working examples)
  - Direct teaching: When to use inline vs. multi-line code blocks
  - Code Example 1: Expected output block (what program should produce)
  - Code Example 2: Python code example with language tag
  - Code Example 3: Inline code in paragraph explaining `print()` function
  - Code Example 4: Unclosed code block error (missing closing ` ``` `)
  - Exercise: Add code block showing expected output to student's project spec
  - YAML frontmatter: `duration: 45min`, `proficiency: A2`, `concepts: 4`, `skills: [Creating Fenced Code Blocks (A2 - Apply), Using Language Tags (A2 - Apply), Using Inline Code (A2 - Apply), Understanding Block Purpose (A2 - Understand)]`
- [ ] T017 [US1] Add "Try With AI" section to lesson 4: Student shows code block to ChatGPT → asks "Does this code block clarify what my program should do?"
- [ ] T018 [US1] Create validation checklist for lesson 4: Opening and closing ` ``` ` present, language tag specified, code renders with syntax highlighting

### Lesson 5: Emphasis & Links + First Complete Spec (Tier 1 Integration)

- [ ] T019 [P] [US1] Write lesson 5 content in `05-emphasis-links-first-spec.md` covering:
  - Concept 1: Emphasis syntax (`**bold**` for important terms, `*italic*` for emphasis)
  - Concept 2: Link syntax (`[text](url)` for references to documentation)
  - Concept 3: Specification structure (integrating all Tier 1 elements: headings, lists, code blocks, emphasis, links)
  - Direct teaching: When to bold vs. italicize, how to find good documentation URLs
  - Code Example 1: README with proper emphasis on key requirements
  - Code Example 2: Links to Python documentation, GitHub repos
  - Code Example 3: Invalid link syntax error (spaces in URL without encoding)
  - **Major Exercise**: Complete spec template with 6 sections (Problem, Solution, Features list, Expected Output code block, Reference links)
  - Exercise acceptance criteria: Proper heading hierarchy, feature list with 3+ items, code block with language tag, at least 1 working link
  - YAML frontmatter: `duration: 50min`, `proficiency: A2`, `concepts: 3`, `skills: [Applying Emphasis (A2 - Apply), Creating Links (A2 - Apply), Writing Specification Documents (A2 - Create), Understanding Spec Structure (A2 - Understand)]`
- [ ] T020 [US1] Add "Try With AI" section to lesson 5: Student submits complete spec.md to ChatGPT → asks "Is this specification clear enough for you to implement? What's missing?"
- [ ] T021 [US1] Create spec template file in `examples/spec-template.md` for lesson 5 major exercise with scaffolding: section headings provided, students fill content
- [ ] T022 [US1] Create assessment rubric for lesson 5 (SC-001 validation): Heading hierarchy correct (20%), Feature list complete (20%), Code block with language tag (20%), Working link (20%), Overall clarity (20%)

**Checkpoint**: At this point, User Story 1 (P1 - Write Specifications) is complete and independently testable via lesson 5 exercise submission

---

## Phase 4: User Story 2 - Read and Understand AI-Generated Documentation (Priority: P2)

**Goal**: Students can read markdown-formatted AI responses and extract key information (code language, error messages, action items)

**Independent Test**: Student correctly identifies code language and extracts 3 fix steps from formatted AI response (SC-002: 85%+ achieve 75%+ quiz score)

**Lessons Covering US2**: Lesson 6 (Tier 2 - AI Companion pattern)

### Lesson 6: Complex Markdown with AI Companion (Tier 2)

- [ ] T023 [US2] Write lesson 6 content in `06-complex-markdown-ai-companion.md` covering:
  - Concept 1: Tier 2 teaching pattern (student specifies WHAT → AI generates HOW → student validates)
  - Concept 2: Complex markdown types (tables with multiple columns, nested lists 3+ levels deep, YAML front matter)
  - Concept 3: Validation principle (always verify AI output renders correctly - don't blindly trust)
  - Teaching approach: Explain WHEN to use AI for complex syntax (memorizing table pipes is inefficient)
  - Code Example 1: Markdown table comparing Python vs TypeScript vs JavaScript (show specification request to AI, show generated output)
  - Code Example 2: Nested task list (Backend → [Database, API, Auth] and Frontend → [UI, State, Routing])
  - Code Example 3: YAML front matter for Docusaurus page (title, description, sidebar_position)
  - Reading AI output: How to identify code blocks by language tag in multi-block responses
  - Reading AI output: How to locate error sections by heading structure (## Root Cause, ## Fix Steps)
  - Reading AI output: How to extract action items from bulleted lists
  - Exercise 1: Request table from AI with specification "Create table comparing 3 tools with columns: Name, Purpose, When to Use"
  - Exercise 2: Request nested list from AI describing project structure
  - Exercise 3: Parse sample AI response (multiple code blocks) to identify Python vs TypeScript vs Bash
  - Comprehension quiz: Given AI markdown response, answer "What language is code block 2?" and "What are the 3 steps in Fix section?"
  - YAML frontmatter: `duration: 50min`, `proficiency: B1`, `concepts: 3`, `skills: [Specifying Complex Markdown (B1 - Create), Understanding AI's Role (A2 - Understand), Validating AI Output (A2 - Evaluate), Reading Structured Responses (A2 - Analyze)]`
- [ ] T024 [US2] Add "Try With AI" section to lesson 6: Student requests "Create a comparison table for [student's domain]" from ChatGPT → validates table renders correctly → refines request if needed
- [ ] T025 [US2] Create sample AI responses in `examples/ai-response-samples.md` for lesson 6 reading practice: Multi-block code response (Python + TypeScript), error explanation with sections, action items list
- [ ] T026 [US2] Create comprehension quiz for lesson 6 (SC-002 validation): 10 questions (5 on identifying code language, 5 on extracting structured information), passing score 75%+
- [ ] T027 [US2] Create exercise acceptance criteria for lesson 6 (SC-004 validation): AI-generated table renders correctly (3+ columns), student correctly specified WHAT (not HOW), student validated output before submission

**Checkpoint**: At this point, User Story 2 (P2 - Read AI Output) is complete and independently testable via lesson 6 quiz and exercises

---

## Phase 5: User Story 3 - Collaborate on Specifications Using GitHub (Priority: P3)

**Goal**: Students can create professional README.md files for GitHub that render correctly

**Independent Test**: Student creates README.md with standard sections (Title, Description, Installation, Usage) → pushes to GitHub → formatted README displays on repository homepage (SC-003: 80%+ achieve this)

**Lessons Covering US3**: Lessons 7-8 (Tier 3 orchestration + Full AIDD integration)

### Lesson 7: Scaling Markdown with AI Orchestration (Tier 3)

- [ ] T028 [US3] Write lesson 7 content in `07-scaling-markdown-orchestration.md` covering:
  - Concept 1: Orchestration mindset (you define STRATEGY, AI executes TACTICS)
  - Concept 2: Consistency principle (when generating multiple files, ensure uniform structure)
  - Concept 3: Scaling operations (10+ files, bulk conversions, multi-component documentation)
  - Teaching approach: When to orchestrate vs. manually create (threshold: 5+ similar items)
  - Code Example 1: Directing AI to generate 5 README files for multi-module project (specification: "Generate README.md for each of these 5 folders: [list]. Use sections: Title, Description, Installation, Usage. Customize content based on folder purpose.")
  - Code Example 2: Converting 10 .txt notes to markdown (specification: "Convert all files in /notes folder from .txt to .md. Detect headings from ALL CAPS lines. Detect code from indented blocks.")
  - Code Example 3: Creating Docusaurus site structure (specification: "Create docs/ folder with 3 chapters, each containing 3 lessons as separate .md files. Use naming: 01-chapter-name/01-lesson-name.md")
  - Exercise 1: Direct AI to generate 3+ README files with consistent structure for student's imagined multi-repo project
  - Exercise 2: Direct AI to create documentation set (index.md + 3 topic pages) with consistent format
  - Assessment: Submit 3+ generated files demonstrating consistency (same sections, similar formatting, appropriate customization)
  - YAML frontmatter: `duration: 50min`, `proficiency: B1`, `concepts: 3`, `skills: [Orchestrating AI Generation (B1 - Create), Ensuring Consistency (B1 - Evaluate), Directing AI Strategy (B1 - Analyze)]`
- [ ] T029 [US3] Add "Try With AI" section to lesson 7: Student orchestrates "Generate complete documentation for [student's project] with 5 pages following standard structure" → validates consistency across generated files
- [ ] T030 [US3] Create orchestration request templates in `examples/orchestration-prompts.md` for lesson 7: Batch README generation template, bulk conversion template, site structure template
- [ ] T031 [US3] Create consistency validation checklist for lesson 7: All files have same section headings? Content customized (not copy-paste)? Formatting uniform? Links work?

### Lesson 8: Full AIDD Cycle - Final Integration Project (MVP Demonstration)

- [ ] T032 [US3] Write lesson 8 content in `08-full-aidd-cycle.md` covering:
  - Concept 1: Full AIDD workflow (Write spec in markdown → AI generates code → Validate output → Identify gaps → Refine spec → Re-generate)
  - Integration: All skills from Lessons 1-7 applied in complete workflow
  - Real-world context: This is how professional developers work with AI coding agents
  - **Major Exercise**: 5-step AIDD cycle
    - Step 1: Write specification for simple CLI tool (e.g., "Todo list app with add, list, delete commands") using all Tier 1 markdown (headings, features list, expected output code block, reference links)
    - Step 2: Submit spec to AI agent (ChatGPT web: "Implement this specification") → receive generated Python code
    - Step 3: Validate code (Does it match spec? Are all features present? Does expected output match?)
    - Step 4: Identify gaps (What's missing? What doesn't match spec?)
    - Step 5: Refine specification → re-submit → validate improved output
  - GitHub integration: Create repository → add generated code → create README.md (Tier 1 markdown) → push → verify rendering
  - Exercise acceptance criteria: Complete 5-step cycle documented, GitHub repo created with README.md rendering correctly (SC-003), reflection on what markdown clarity improved AI output quality
  - YAML frontmatter: `duration: 60min`, `proficiency: B1+`, `concepts: 1`, `skills: [Completing Full AIDD Cycle (B1 - Create), Validating Code Against Spec (B1 - Evaluate), Iterative Spec Refinement (B1 - Analyze), Understanding Intent Layer (B1 - Understand)]`
- [ ] T033 [US3] Add "Try With AI" section to lesson 8: Student completes full AIDD cycle with their own project idea (not provided example) → documents cycle → shares GitHub repo link
- [ ] T034 [US3] Create AIDD cycle worksheet in `examples/aidd-cycle-worksheet.md` for lesson 8 major exercise: Structured template with sections for each step (Spec v1, Generated Code, Validation Notes, Gap Analysis, Spec v2, Improved Code)
- [ ] T035 [US3] Create assessment rubric for lesson 8 (SC-005 validation): Spec v1 complete (20%), Code generated (15%), Validation documented (20%), Gaps identified (20%), Spec refined (15%), GitHub README renders (10%)
- [ ] T036 [US3] Create GitHub submission checklist for lesson 8 (SC-003 + SC-011 validation): Repository created? README.md present? Headings render correctly? Lists format correctly? Code blocks highlight correctly? Links work?

**Checkpoint**: All user stories (US1, US2, US3) complete and independently functional. Students have portfolio evidence (GitHub repo with README) demonstrating markdown mastery.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and validation that affect all lessons

- [ ] T037 [P] Add "Common Mistakes" sidebar callouts to all 8 lessons referencing `examples/common-errors.md` (empty headings in lesson 2, unclosed code blocks in lesson 4, invalid links in lesson 5, etc.)
- [ ] T038 [P] Verify all 8 lessons end with ONLY "Try With AI" section (no separate "Key Takeaways" or "What's Next" sections per constitution)
- [ ] T039 [P] Verify all YAML frontmatter includes skills metadata fields: `skills: [{name, cefr_level, bloom_level, digcomp_area, measurable_outcome}]`
- [ ] T040 [P] Add cross-references between lessons: Lesson 5 references 2-4, Lesson 6 references 2-5, Lesson 8 references all previous
- [ ] T041 Validate reading level for all lessons using Flesch-Kincaid test (SC-008): Target Grade 7-8, flag any lessons above Grade 9 for simplification
- [ ] T042 [P] Create end-of-chapter quiz in `examples/chapter-quiz.md` covering: Syntax patterns (30%), Tier 1 vs 2 vs 3 usage (30%), AIDD workflow understanding (30%), Real-world application (10%) - 20 questions, passing score 80% (SC-007)
- [ ] T043 Verify all code examples in all lessons tested for correctness: Markdown renders as expected, language tags work, links are valid
- [ ] T044 [P] Create chapter introduction page explaining three-tier structure and how lessons build on each other
- [ ] T045 Validate constitutional compliance across all lessons: Tier 1 teaches directly (not "ask your AI")? Tier 2 teaches specification pattern? Tier 3 teaches orchestration? Max 5 concepts per section?

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all lesson writing
- **User Stories (Phases 3-5)**: All depend on Foundational phase completion
  - Lessons within US1 (Lessons 1-5): Sequential (1→2→3→4→5) due to progressive skill building
  - US2 (Lesson 6): Can start after Foundational, integrates with US1 skills
  - US3 (Lessons 7-8): Lesson 7 can start after Foundational, Lesson 8 depends on Lessons 1-7
- **Polish (Phase 6)**: Depends on all lessons being written

### User Story Dependencies

- **User Story 1 (P1 - Write Specs)**: Can start after Foundational (Phase 2) - No dependencies on other stories
  - Lessons 1-5 build progressively: Foundation → Headings → Lists → Code Blocks → Emphasis & Links → First Complete Spec
- **User Story 2 (P2 - Read AI Output)**: Can start after Foundational (Phase 2) - Integrates US1 skills but independently testable via quiz
  - Lesson 6 applies US1 skills in new context (reading AI responses vs writing specs)
- **User Story 3 (P3 - GitHub Collaboration)**: Can start after Foundational (Phase 2) - Integrates US1 + US2 but independently testable via GitHub submission
  - Lesson 7 applies orchestration mindset (new skill)
  - Lesson 8 synthesizes all skills in complete AIDD cycle

### Within Each User Story

- **US1 (Lessons 1-5)**: Sequential execution required (each lesson builds on previous)
  - Lesson 1 (foundation) must complete before Lesson 2
  - Lesson 2 (headings) must complete before Lesson 3
  - Lesson 3 (lists) must complete before Lesson 4
  - Lesson 4 (code blocks) must complete before Lesson 5
  - Lesson 5 (integration) must complete to satisfy US1 acceptance criteria
- **US2 (Lesson 6)**: Single lesson, no internal dependencies
- **US3 (Lessons 7-8)**: Lesson 8 depends on Lesson 7 and all previous lessons

### Parallel Opportunities

- Setup tasks (T001-T003): All marked [P] can run in parallel
- Foundational tasks (T004-T006): All marked [P] can run in parallel
- Within US1:
  - Lesson 1 writing (T007-T009): Sequential within lesson but can start immediately after Foundational
  - Lessons 2-4: Each lesson's tasks (T010-T012, T013-T015, T016-T018) can be parallelized if multiple authors
  - However, lessons MUST be completed in order due to progressive scaffolding
- US2 Lesson 6 tasks (T023-T027): Can parallelize sample creation, quiz creation, exercise template
- US3 tasks: Lesson 7 (T028-T031) and partial Lesson 8 (T032) can be parallelized if different authors
- Polish tasks (T037-T045): Most marked [P] can run in parallel across all lessons

---

## Parallel Example: Foundational Phase

```bash
# Launch all foundational tasks together:
Task: "Create common mistakes reference in examples/common-errors.md"
Task: "Create validation guide in examples/validation-checklist.md"
Task: "Create syntax quick reference in examples/syntax-reference.md"

# All three support files can be created simultaneously
```

## Parallel Example: Lesson 6 (User Story 2)

```bash
# Launch all support materials for lesson 6 together:
Task: "Create sample AI responses in examples/ai-response-samples.md"
Task: "Create comprehension quiz with 10 questions"
Task: "Create exercise acceptance criteria rubric"

# Main lesson content (T023) can be written while support materials are being created
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Lessons 1-5)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T006) - CRITICAL
3. Complete Phase 3: User Story 1 (T007-T022) - Lessons 1-5
4. **STOP and VALIDATE**: Test lesson 5 major exercise independently
   - Can students write valid spec.md with all Tier 1 elements?
   - Does AI successfully generate code from student specs?
   - SC-001 validation: 90%+ achieve success?
5. Deploy/demo Lessons 1-5 if ready (students can now write specifications)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Lessons 1-5 (US1) → Test independently → Deploy/Demo (Students can write specs - MVP!)
3. Add Lesson 6 (US2) → Test independently via quiz → Deploy/Demo (Students can read AI output)
4. Add Lessons 7-8 (US3) → Test independently via GitHub submission → Deploy/Demo (Students can collaborate on GitHub + complete AIDD cycle)
5. Polish phase → Final validation → Complete chapter ready

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together (6 tasks)
2. Once Foundational is done:
   - **Author A**: Lessons 1-2 (Foundation + Headings) - T007-T012
   - **Author B**: Lessons 3-4 (Lists + Code Blocks) - T013-T018
   - **Author C**: Lesson 5 (Integration + Major Exercise) - T019-T022
   - **Author D**: Lesson 6 (Tier 2 + Quiz) - T023-T027
   - **Author E**: Lessons 7-8 (Tier 3 + AIDD Cycle) - T028-T036
3. Lessons complete sequentially within each author's domain, integrate independently for testing

---

## Notes

### AI Native Teaching Methodology Integration

**This task list implements AI-Native Teaching across three tiers**:

1. **Tier 1 (Lessons 2-5)**: Book teaches foundational markdown syntax directly
   - Tasks T010-T022 ensure content demonstrates syntax with clear examples
   - Students practice manually (not "ask your AI what is a heading")
   - Validation: Students can write markdown without AI assistance

2. **Tier 2 (Lesson 6)**: AI Companion handles complex syntax
   - Tasks T023-T027 teach specification-driven requests to AI
   - Students specify WHAT they need → AI generates complex syntax → Students validate
   - Focus: Understanding when to delegate complexity to AI

3. **Tier 3 (Lesson 7)**: AI Orchestration for scaling operations
   - Tasks T028-T031 teach strategic direction of AI for multi-file work
   - Students define patterns and structure → AI executes repetitive work → Students ensure consistency
   - Focus: Orchestration mindset for large-scale tasks

4. **Integration (Lesson 8)**: Full AIDD workflow synthesis
   - Tasks T032-T036 demonstrate complete cycle: Spec → AI Implementation → Validation
   - Combines all three tiers in real-world application
   - Portfolio evidence: GitHub repository with professional README

### Lesson Closure Policy

**CRITICAL**: All lessons MUST end with single "Try With AI" section only.

- **Before AI tools taught** (Chapters 1-9): "Try With AI" uses ChatGPT web
- **After tool onboarding** (Chapters 10+): Instruct use of preferred tool (Gemini CLI, Claude CLI), optionally provide CLI and web variants
- **NO** separate "Key Takeaways" section
- **NO** separate "What's Next" section
- Constitution Principle 1 enforces this structure

### Task Format Compliance

- [P] marker: Different files, no dependencies within phase
- [Story] label: Maps to user stories from spec.md (US1, US2, US3)
- Exact file paths included in all task descriptions
- All tasks follow checklist format: `- [ ] [ID] [P?] [Story?] Description with path`

### Success Criteria Tracking

Tasks map directly to spec.md success criteria:

- **SC-001** (90%+ write specs): T022 assessment rubric for lesson 5 exercise
- **SC-002** (85%+ read AI output): T026 comprehension quiz for lesson 6
- **SC-003** (80%+ GitHub README): T036 GitHub submission checklist for lesson 8
- **SC-004** (75%+ direct AI complex markdown): T027 exercise validation for lesson 6
- **SC-005** (Full AIDD cycle): T035 assessment rubric for lesson 8
- **SC-007** (80%+ pass chapter quiz): T042 end-of-chapter quiz
- **SC-008** (Grade 7-8 reading level): T041 Flesch-Kincaid validation
- **SC-009** (70%+ completion rate): Tracked post-publication
- **SC-010** (80%+ submit "Try With AI"): Tracked post-publication
- **SC-011** (GitHub portfolio): T036 validates this outcome

### Independent Testing Per User Story

- **US1 Independent Test**: Lesson 5 exercise submission (T022 rubric)
- **US2 Independent Test**: Lesson 6 comprehension quiz (T026 passing score 75%+)
- **US3 Independent Test**: Lesson 8 GitHub submission (T036 checklist)

Each story delivers measurable value independently:
- Complete US1 only → Students can write specifications (foundation skill)
- Complete US1+US2 → Students can also read/validate AI output (safety skill)
- Complete US1+US2+US3 → Students can collaborate on GitHub + complete full AIDD cycle (professional workflow)

---

## Total Task Count: 45 tasks

- Setup: 3 tasks
- Foundational: 3 tasks (BLOCKING)
- User Story 1 (Lessons 1-5): 16 tasks
- User Story 2 (Lesson 6): 5 tasks
- User Story 3 (Lessons 7-8): 9 tasks
- Polish: 9 tasks

**Parallel opportunities identified**: 18 tasks marked [P] can run in parallel within their phases

**MVP scope** (US1 only): Tasks T001-T022 (25 tasks, ~60% of total effort)

**Estimated timeline**:
- MVP (US1): 2-3 weeks
- US2 addition: +1 week
- US3 addition: +1-2 weeks
- Polish: +1 week
- **Total: 5-7 weeks** for complete chapter

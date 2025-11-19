# Tasks: Chapter 13 Redesign - Introduction to Python (Story-Based Learning)

**Input**: Design documents from `/specs/014-chapter-13-redesign/`
**Prerequisites**: plan.md (✓ approved), spec.md (✓ completed), chapter-index.md, constitution.md v6.0.0
**Branch**: `014-chapter-13-redesign`
**Content Type**: Educational book chapter (Markdown lessons)
**Target Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/`

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project structure preparation and validation setup

- [ ] **T001** Create feature branch `014-chapter-13-redesign` from main
- [ ] **T002** [P] Backup existing Chapter 13 content to `specs/014-chapter-13-redesign/backup/` (preserve for reference)
- [ ] **T003** [P] Read Chapter 14 structure for reference patterns: `book-source/docs/04-Python-Fundamentals/14-data-types/`
- [ ] **T004** [P] Verify Python 3.14+ installed and UV working (prerequisite check from Chapter 12)
- [ ] **T005** Create test environment for code validation: `specs/014-chapter-13-redesign/code-tests/` directory structure

**Checkpoint**: Branch created, backup complete, reference materials ready, validation environment prepared

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core chapter infrastructure that MUST be complete before ANY lesson implementation

**⚠️ CRITICAL**: No lesson work can begin until this phase is complete

- [ ] **T006** Create Chapter 13 README.md at `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/readme.md`
  - **Content**: Learning journey map, prerequisites checklist, 6-lesson progression overview, connection to AI-native development, "What We're NOT Covering Yet" section
  - **Reference**: `specs/014-chapter-13-redesign/plan.md` Section: "README.md Structure"
  - **Constitutional Check**: Principles 1, 4, 7 (Specification Primacy, Coherent Structure, Minimal Content)
  - **Validation**: Matches Chapter 14 README quality; includes all required sections from plan

- [ ] **T007** Create `_category_.json` for Chapter 13 directory with sidebar position and label
  - **Content**: `{"position": 13, "label": "Introduction to Python", "collapsible": true, "collapsed": false}`
  - **Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/_category_.json`
  - **Purpose**: Docusaurus sidebar navigation

- [ ] **T008** [P] Create story elements document capturing narrative thread for all lessons
  - **Path**: `specs/014-chapter-13-redesign/STORY_ELEMENTS.md`
  - **Content**: Robot sandwich metaphor, pet name labels analogy, digital introduction card capstone
  - **Purpose**: Maintain story cohesion during parallel lesson implementation

- [ ] **T009** [P] Create validation scripts directory with grep patterns for forbidden content
  - **Path**: `specs/014-chapter-13-redesign/validation/`
  - **Scripts**: `grep-stage-labels.sh`, `grep-three-roles.sh`, `grep-section-endings.sh`
  - **Purpose**: Automated constitutional compliance checks (FR-017, EV-014)

- [ ] **T010** [P] Create code testing framework for all examples
  - **Path**: `specs/014-chapter-13-redesign/code-tests/test_all_examples.py`
  - **Purpose**: Execute every code snippet, capture output logs (FR-006, FR-019)
  - **Framework**: Python unittest or pytest with execution log capture

**Checkpoint**: Foundation ready - lesson implementation can now begin in parallel

---

## Phase 3: User Story 1 - First Programmer's Welcome (Priority: P1) 🎯 MVP

**Goal**: Introduce absolute beginners to programming and Python through story-based narrative. Student understands "what Python is" without intimidation.

**Independent Test**: Student can explain programming as "giving instructions to computers" and identify Python as a beginner-friendly language.

### Implementation for User Story 1

- [ ] **T011** [P] [US1] Implement Lesson 1: "What is Python? Your First Step into Programming"
  - **Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/01-what-is-python.md`
  - **Story Element**: "Teaching a Robot to Make Sandwiches"
  - **Concepts** (3 total - A2 compliant):
    1. Programming as instruction-giving
    2. Python as a programming language
    3. Why learn Python (motivation)
  - **Structure**:
    - Opening Hook: Robot sandwich story
    - What Is Programming? (WHAT concept)
    - Why Python? (WHY reasoning before syntax)
    - Real-World Applications (motivation)
    - For Curious Learners: How Python Runs Your Code (bytecode intro)
    - Try With AI (Stage 2 gentle intro)
  - **Constitutional Check**: Principles 1 (WHAT-WHY-HOW), 2 (A2 tier: 3 concepts), 6 (story-based modality)
  - **Code Examples**: None yet (foundation lesson)
  - **Duration**: 30-35 minutes
  - **Reference**: `specs/014-chapter-13-redesign/plan.md` Lines 101-188

- [ ] **T012** [P] [US1] Implement Lesson 2: "Your First Python Program"
  - **Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/02-first-python-program.md`
  - **Story Element**: "Your Robot Speaks Its First Words"
  - **Concepts** (2 total - A2 compliant):
    1. Running Python programs in terminal
    2. print() function for output
  - **Structure**:
    - Opening Hook: Robot speaks analogy
    - Running Python (terminal introduction)
    - The print() Function (WHAT-WHY-HOW)
    - Your First Program: "Hello, World!" (guided execution)
    - Common Mistakes (syntax errors, quote issues)
    - For Curious Learners: Indentation Importance
    - Try With AI (practice print() variations)
  - **Constitutional Check**: Principles 1, 2 (2 concepts), 3 (code tested)
  - **Code Examples**:
    - `print("Hello, World!")`
    - `print("My name is Alex")`
    - Multi-line print examples
  - **Testing**: T010 framework validates all examples execute
  - **Duration**: 30-35 minutes
  - **Reference**: `specs/014-chapter-13-redesign/plan.md` Lines 190-266

- [ ] **T013** [US1] Test User Story 1 independently
  - **Validation**: Student completes L1-L2, can explain "what Python is" and run first program
  - **Acceptance**: Matches spec.md User Story 1 acceptance scenarios
  - **Check**: EV-001 (explain Python), EV-002 (Hello World execution)

**Checkpoint**: User Story 1 complete - student understands programming concept and executed first Python program

---

## Phase 4: User Story 2 - Variable Discovery Journey (Priority: P1)

**Goal**: Teach variables conceptually (WHAT/WHY) before syntax (HOW). Student understands variables as "labeled memory" and grasps why type hints matter.

**Independent Test**: Student creates variables with type hints, modifies values, explains WHY variables need both names and types.

### Implementation for User Story 2

- [ ] **T014** [P] [US2] Implement Lesson 3: "Variables: Python's Memory"
  - **Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/03-variables-python-memory.md`
  - **Story Element**: "Your Robot Learns to Remember" + Pet name labels analogy
  - **Concepts** (3 total - A2 compliant):
    1. Variables as labeled memory boxes
    2. Creating variables with names
    3. Variable naming conventions
  - **Structure**:
    - Opening Hook: Robot memory story
    - What Are Variables? (concept before syntax)
    - Creating Variables (HOW after WHY)
    - Variable Naming Rules (PEP 8 basics)
    - Modifying Variables (reassignment)
    - For Curious Learners: Memory and id() Function
    - Try With AI (variable naming practice)
  - **Constitutional Check**: Principles 1 (WHAT-WHY-HOW), 2 (3 concepts), 7 (minimal content)
  - **Code Examples**:
    - `name = "Alex"`
    - `age = 25`
    - `is_student = True`
    - Naming rules violations
  - **Testing**: All examples tested via T010 framework
  - **Duration**: 35-40 minutes
  - **Reference**: `specs/014-chapter-13-redesign/plan.md` Lines 268-355

- [ ] **T015** [P] [US2] Implement Lesson 4: "Type Hints: Organizing Your Data"
  - **Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/04-type-hints-organizing-data.md`
  - **Story Element**: "Your Robot Organizes Its Knowledge"
  - **Concepts** (3 total - A2 compliant):
    1. Data types concept (int, str, float, bool preview)
    2. Type hint syntax (:int, :str)
    3. Why type hints matter (intent specification)
  - **Structure**:
    - Opening Hook: Robot organization analogy
    - What Are Data Types? (classification concept)
    - Type Hint Syntax (HOW with examples)
    - Why Type Hints Matter (intent over enforcement)
    - Type Hints vs Python's Flexibility (dynamic typing)
    - For Curious Learners: OOP Preview (type as class)
    - Try With AI (type hint practice scenarios)
  - **Constitutional Check**: Principles 1, 2 (3 concepts), 6 (story cohesion)
  - **Code Examples**:
    - `name: str = "Alex"`
    - `age: int = 25`
    - `price: float = 19.99`
    - `is_student: bool = True`
  - **Testing**: All examples validated
  - **Duration**: 35-40 minutes
  - **Reference**: `specs/014-chapter-13-redesign/plan.md` Lines 357-446

- [ ] **T016** [US2] Test User Story 2 independently
  - **Validation**: Student creates variables with type hints, explains purpose of both names and types
  - **Acceptance**: Matches spec.md User Story 2 acceptance scenarios
  - **Check**: EV-003 (create variable with type hint), EV-004 (explain WHY)

**Checkpoint**: User Story 2 complete - student masters variables and type hints conceptually

---

## Phase 5: User Story 3 - Building Confidence Through Capstone (Priority: P1)

**Goal**: Integrate all concepts (print, variables, type hints, input) into ONE coherent program. Student builds "digital introduction card" capstone.

**Independent Test**: Student creates introduction program independently with light AI guidance.

### Implementation for User Story 3

- [ ] **T017** [P] [US3] Implement Lesson 5: "User Input: Interactive Programs"
  - **Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/05-user-input-interactive-programs.md`
  - **Story Element**: "Your Robot Learns to Listen"
  - **Concepts** (2 total - A2 compliant):
    1. input() function
    2. Integration of print + variables + type hints + input
  - **Structure**:
    - Opening Hook: Robot listening story
    - The input() Function (WHAT-WHY-HOW)
    - Storing Input in Variables (integration practice)
    - Combining print() and input() (conversation flow)
    - For Curious Learners: Type Conversion Preview (input always returns str)
    - Try With AI (interactive program practice)
  - **Constitutional Check**: Principles 1, 2 (2 concepts), 4 (builds on prior lessons)
  - **Code Examples**:
    - `name: str = input("What is your name? ")`
    - `age: str = input("How old are you? ")`
    - Full program: ask name, greet user
  - **Testing**: All examples execute and capture logs
  - **Duration**: 30-35 minutes
  - **Reference**: `specs/014-chapter-13-redesign/plan.md` Lines 448-523

- [ ] **T018** [P] [US3] Implement Lesson 6: "Capstone - Personal Introduction Program"
  - **Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/06-capstone-personal-introduction.md`
  - **Story Element**: "Your Robot Introduces Itself" (resolution)
  - **Concepts** (0 new - synthesis only)
  - **Structure**:
    - Project Overview: Digital introduction card goal
    - Requirements: Name, age, favorite hobby, greeting output
    - Guided Planning: Pseudocode walkthrough
    - Implementation Guide: Step-by-step with checkpoints
    - Common Challenges: Debugging tips
    - Extension Ideas: Add more inputs, format output creatively
    - Try With AI (extend capstone with guidance)
  - **Constitutional Check**: Principles 4 (coherent structure), 5 (intelligence accumulation)
  - **Code Example**: Full capstone solution with comments
  - **Testing**: Capstone program executes successfully
  - **Duration**: 45-50 minutes
  - **Reference**: `specs/014-chapter-13-redesign/plan.md` Lines 525-596

- [ ] **T019** [US3] Test User Story 3 independently
  - **Validation**: Student completes capstone independently, demonstrates print/variables/types/input mastery
  - **Acceptance**: Matches spec.md User Story 3 acceptance scenarios
  - **Check**: EV-005 (capstone completion 70%+ target)

**Checkpoint**: User Story 3 complete - student has working capstone program demonstrating all Chapter 13 concepts

---

## Phase 6: User Story 4 - Curious Learner Exploration (Priority: P2)

**Goal**: Provide enrichment content for advanced students WITHOUT overwhelming main learning path.

**Independent Test**: Students CAN skip "For Curious Learners" sections and still master chapter OR explore and gain deeper understanding.

### Implementation for User Story 4

- [ ] **T020** [P] [US4] Verify all "For Curious Learners" sections are present and appropriately scoped
  - **Lessons with Sections**:
    - L1: "How Python Runs Your Code" (bytecode, compilation)
    - L2: "Indentation Importance" (significant whitespace)
    - L3: "Memory and id() Function" (object identity)
    - L4: "OOP Preview" (types as classes)
    - L5: "Type Conversion Preview" (input returns str)
  - **Validation**:
    - Each section clearly labeled "For Curious Learners"
    - Advanced concepts don't disrupt main narrative flow
    - Students can skip without losing core understanding
  - **Constitutional Check**: Principle 2 (progressive complexity), 7 (optional content)

- [ ] **T021** [US4] Test User Story 4 independently
  - **Validation**: Two test paths: (1) Skip all advanced sections, master core; (2) Read all, gain enrichment
  - **Acceptance**: Matches spec.md User Story 4 acceptance scenarios
  - **Check**: EV-010 (advanced sections present without overwhelming)

**Checkpoint**: User Story 4 complete - enrichment content available for curious learners

---

## Phase 7: User Story 5 - AI Collaboration Introduction (Priority: P2)

**Goal**: Introduce AI as "coding buddy" through "Try With AI" sections. Gentle Stage 2 introduction (manual foundation first).

**Independent Test**: Student uses AI for practice WITHOUT becoming dependent (can complete exercises manually).

### Implementation for User Story 5

- [ ] **T022** [P] [US5] Verify all "Try With AI" sections follow Stage 1→Stage 2 progression pattern
  - **Structure per Lesson**:
    - Clear prompt examples (copy-paste ready)
    - Expected output described
    - Safety notes (verify AI suggestions, don't just trust)
    - Reflection questions (how AI helped, what you learned)
  - **Lessons**:
    - L1: Analogies for programming concepts
    - L2: Practice print() variations
    - L3: Variable naming scenarios
    - L4: Type hint practice with validation
    - L5: Interactive program practice
    - L6: Capstone extensions with AI guidance
  - **Constitutional Check**: Stage 1 primary, Stage 2 gentle intro (CON-003, FR-011)

- [ ] **T023** [P] [US5] Validate "Try With AI" is FINAL section in all lessons (no content after)
  - **Validation**: Run grep audit from T009 validation scripts
  - **Check**: No "What's Next", "Summary", "Key Takeaways" after "Try With AI"
  - **Acceptance**: FR-010, EV-015 (section ending protocol)

- [ ] **T024** [US5] Test User Story 5 independently
  - **Validation**: Student completes "Try With AI" exercises, understands AI as practice partner (not solution generator)
  - **Acceptance**: Matches spec.md User Story 5 acceptance scenarios

**Checkpoint**: User Story 5 complete - AI collaboration introduced appropriately for Stage 1 chapter

---

## Phase 8: Quality Assurance & Constitutional Compliance

**Purpose**: Comprehensive validation before publication

### Code Validation

- [ ] **T025** [P] Execute all code examples and capture output logs
  - **Script**: Run `specs/014-chapter-13-redesign/code-tests/test_all_examples.py`
  - **Output**: Execution logs for every code snippet
  - **Attach**: Logs to each lesson as comments or separate file
  - **Validation**: FR-006, FR-019, EV-006 (all code executes in Python 3.14+)

- [ ] **T026** [P] Verify Python version compatibility
  - **Test**: Run examples in Python 3.14+ AND Python 3.10+ (backward compatibility check)
  - **Document**: Any version-specific features noted
  - **Validation**: Spec requirement: "Python 3.14+ with 3.10+ compatibility"

### Citation Validation

- [ ] **T027** [P] Create citation list for all Python technical claims
  - **Path**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/CITATIONS.md`
  - **Content**: Python.org docs, PEP references, official sources for every claim
  - **Examples**:
    - "Python is interpreted" → cite Python execution model docs
    - "Variables store references" → cite Python data model
    - "Type hints don't enforce types" → cite PEP 484
  - **Validation**: FR-007, EV-007 (all claims cited)

### Constitutional Compliance Audits

- [ ] **T028** [P] Run grep validation for forbidden patterns (internal scaffolding exposure)
  - **Script**: `specs/014-chapter-13-redesign/validation/grep-stage-labels.sh`
  - **Patterns**:
    - "Stage [0-9]" (forbidden in student text)
    - "Layer [0-9]" (internal only)
    - "Three Roles (Framework|in Action)" (pedagogical scaffolding)
  - **Expected Result**: ZERO matches in lesson files
  - **Validation**: FR-013, FR-017, EV-014 (scaffolding hidden)

- [ ] **T029** [P] Verify section ending protocol (lessons end with "Try With AI" only)
  - **Script**: `specs/014-chapter-13-redesign/validation/grep-section-endings.sh`
  - **Check**: No sections after "Try With AI" in any lesson
  - **Validation**: FR-010, FR-018, EV-015 (section structure)

- [ ] **T030** [P] Audit cognitive load per lesson
  - **Count**: New concepts in each lesson
  - **Expected**:
    - L1: 3 concepts ✓
    - L2: 2 concepts ✓
    - L3: 3 concepts ✓
    - L4: 3 concepts ✓
    - L5: 2 concepts ✓
    - L6: 0 concepts ✓
  - **Validation**: All within A2 limit (5-7 max); FR-004, EV-008 (cognitive load managed)

- [ ] **T031** [P] Verify story-based modality maintained throughout
  - **Check**: Each lesson has story element connecting to robot narrative thread
  - **Reference**: `specs/014-chapter-13-redesign/STORY_ELEMENTS.md`
  - **Validation**: FR-009, EV-009, EV-012 (story cohesion)

- [ ] **T032** [P] Validate WHAT-WHY-HOW sequence in every concept introduction
  - **Pattern**: Concept explained (WHAT) → Purpose/reasoning (WHY) → Syntax (HOW)
  - **Lessons**: L1 (programming), L2 (print), L3 (variables), L4 (types), L5 (input)
  - **Validation**: FR-002, EV-011, Principle 1 (Specification Primacy)

### Validation Agent Execution

- [ ] **T033** Run validation-auditor agent on complete chapter
  - **Agent**: `.claude/agents/validation-auditor.md`
  - **Checks**:
    1. Pedagogical effectiveness (story cohesion, WHAT-WHY pattern, progressive depth)
    2. Constitutional compliance (Principles 1-7 applied)
    3. Factual accuracy (code tested, claims cited)
    4. Cognitive load (A2 tier limits respected)
    5. Anti-patterns (no lecture-style convergence, scaffolding hidden, meta-commentary absent)
  - **Output**: Validation report with pass/fail for each dimension
  - **Validation**: FR-016 (comprehensive validation)

- [ ] **T034** Run factual-verifier agent on all technical claims
  - **Agent**: `.claude/agents/factual-verifier.md` (or integrated in validation-auditor)
  - **Focus**: Python behavior claims, version features, syntax rules
  - **Output**: Citation verification report
  - **Validation**: FR-007, EV-007 (factual accuracy)

### Integration Checks

- [ ] **T035** Verify Chapter 13 README.md completeness
  - **Sections Required**:
    - What You'll Master
    - Before You Start (Prerequisites)
    - How This Chapter Works (5-6 lesson progression)
    - Learning Philosophy (Concept Before Syntax)
    - Connection to AI-Native Development
    - Try With AI Throughout
    - What We're NOT Covering Yet
  - **Reference**: Chapter 14 README as quality standard
  - **Validation**: FR-015 (README explains learning journey)

- [ ] **T036** Test navigation and cross-references
  - **Links**: README → Lessons, Lessons → Next Lesson, Chapter 13 → Chapter 14
  - **Docusaurus Build**: Ensure no broken links
  - **Validation**: Functional navigation

- [ ] **T037** Verify Prerequisites from Chapter 12 are referenced
  - **Check**: README mentions Python 3.14+ installed, UV working, terminal access
  - **Link**: "Completed Chapter 12" prerequisite clear
  - **Validation**: Spec dependency AS-001 (Chapter 12 prerequisite)

**Checkpoint**: All quality gates passed - chapter ready for publication

---

## Phase 9: Documentation & Meta-Learning

**Purpose**: Capture reasoning, document decisions, create deployment artifacts

- [ ] **T038** [P] Create ADR (Architectural Decision Record) for key pedagogical choices
  - **Path**: `history/adr/XXXX-chapter-13-story-based-teaching-modality.md`
  - **Decisions**:
    1. Why 6 lessons (not 9) - concept density justified
    2. Why story-based modality - differentiation from Chapter 14
    3. Why robot metaphor - beginner accessibility
    4. Why WHAT-WHY-HOW sequence - specification primacy
  - **Script**: `.specify/scripts/bash/create-adr.sh` or manual creation
  - **Validation**: Principle 5 (intelligence accumulation)

- [ ] **T039** [P] Create PHR (Prompt History Record) for implementation workflow
  - **Path**: `history/prompts/014-chapter-13-redesign/XXXX-implementation-complete.green.prompt.md`
  - **Content**: LoopFlow v2.0 workflow summary, reasoning decisions, outcomes
  - **Script**: `.specify/scripts/bash/create-phr.sh` or manual
  - **Purpose**: Meta-learning capture for future chapters

- [ ] **T040** [P] Update chapter-index.md status for Chapter 13
  - **Path**: `specs/book/chapter-index.md`
  - **Change**: Status from "✅ Implemented" to "✅ Refined & Validated (Story-Based)"
  - **Note**: Similar to Chapter 7 entry showing refinement completion

- [ ] **T041** [P] Create implementation summary document
  - **Path**: `specs/014-chapter-13-redesign/IMPLEMENTATION_SUMMARY.md`
  - **Content**:
    - What was redesigned (from topic-based → story-based)
    - Key pedagogical improvements
    - Constitutional compliance highlights
    - Metrics (concept count, cognitive load, story cohesion)
    - Lessons learned for future chapters

**Checkpoint**: Documentation complete - meta-learning captured

---

## Phase 10: Deployment & Git Workflow

**Purpose**: Commit changes, create PR, merge to main

- [ ] **T042** Stage all changes in Chapter 13 directory
  - **Command**: `git add book-source/docs/04-Python-Fundamentals/13-introduction-to-python/`
  - **Files**: readme.md, 6 lesson files, _category_.json

- [ ] **T043** Stage specification and planning artifacts
  - **Command**: `git add specs/014-chapter-13-redesign/`
  - **Files**: spec.md, plan.md, tasks.md, checklists/, validation/

- [ ] **T044** Stage documentation artifacts
  - **Command**: `git add history/adr/XXXX-chapter-13-*.md history/prompts/014-chapter-13-redesign/`

- [ ] **T045** Create commit with descriptive message
  - **Message Template**:
    ```
    redesign(chapter-13): Story-based learning for absolute beginners

    - Redesigned Chapter 13 (Introduction to Python) using story-based narrative modality
    - 6 lessons with robot learning journey (differentiated from Chapter 14's analogy-based approach)
    - A2 tier cognitive load managed: avg 2.2 concepts/lesson (max 3)
    - WHAT-WHY-HOW sequence applied consistently (Principle 1: Specification Primacy)
    - All code examples tested in Python 3.14+ with execution logs
    - "For Curious Learners" sections provide enrichment without overwhelming
    - Constitutional compliance: Principles 1-7 validated

    Closes #XXX (if GitHub issue exists)

    🤖 Generated with LoopFlow v2.0 - Constitutional Reasoning Orchestrator

    Co-Authored-By: Claude <noreply@anthropic.com>
    ```
  - **Validation**: Follows git commit protocol from CLAUDE.md

- [ ] **T046** Push feature branch to remote
  - **Command**: `git push -u origin 014-chapter-13-redesign`

- [ ] **T047** Create Pull Request with detailed description
  - **Title**: `Redesign Chapter 13: Story-Based Learning for Absolute Beginners`
  - **Description**:
    - Summary of changes (topic-based → story-based)
    - User stories addressed (US1-US5)
    - Constitutional compliance highlights
    - Validation results (code tested, citations added, cognitive load managed)
    - Preview links (if Docusaurus preview available)
    - Reviewer checklist (test capstone, verify story cohesion, check no scaffolding exposure)
  - **Tool**: `gh pr create` or GitHub web interface
  - **Validation**: PR description explains reasoning, not just changes

- [ ] **T048** Address PR review feedback (if any)
  - **Iterate**: Fix issues, update lessons, re-run validations
  - **Commit**: Additional fixes with clear messages

- [ ] **T049** Merge PR to main after approval
  - **Method**: Squash merge or merge commit (per repo conventions)
  - **Validation**: CI/CD passes (Docusaurus build succeeds)

- [ ] **T050** Verify deployment to production
  - **URL**: Check `https://your-book-domain.com/docs/04-Python-Fundamentals/13-introduction-to-python/`
  - **Navigation**: Test lesson links, verify rendering
  - **Validation**: Chapter 13 live and accessible

**Checkpoint**: Chapter 13 redesign deployed to production ✅

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user story implementation
- **User Stories (Phases 3-7)**: All depend on Foundational completion
  - Can proceed in parallel IF multiple developers (T011-T018 are [P] marked where possible)
  - OR sequentially in priority order: US1 → US2 → US3 → US4 → US5
- **QA (Phase 8)**: Depends on all user stories being implemented
- **Documentation (Phase 9)**: Can run in parallel with QA (Phase 8)
- **Deployment (Phase 10)**: Depends on QA + Documentation completion

### User Story Dependencies

- **US1 (P1)**: Can start after Foundational (Phase 2) - Lessons 1-2
- **US2 (P1)**: Can start after Foundational - Lessons 3-4 (builds on US1 concepts but parallel implementation possible)
- **US3 (P1)**: Depends on US1+US2 completion - Lessons 5-6 (capstone integrates prior lessons)
- **US4 (P2)**: Verification task - depends on US1-US3 lessons having "For Curious Learners" sections
- **US5 (P2)**: Verification task - depends on US1-US3 lessons having "Try With AI" sections

### Within Each Lesson

**Sequential Execution Required**:
1. Implement lesson markdown (T011-T018)
2. Write code examples inline
3. Test code examples (T025-T026) - verify execution
4. Validate structure (T028-T032) - grep audits, cognitive load checks
5. Run validation agents (T033-T034)

**Parallel Opportunities**:
- T011, T012 can run in parallel (L1, L2 = different files)
- T014, T015 can run in parallel (L3, L4 = different files)
- T017, T018 can run in parallel (L5, L6 = different files)
- T025, T026, T027 can run in parallel (code tests, version tests, citations = independent)
- T028, T029, T030, T031, T032 can run in parallel (validation audits = different grep patterns)
- T038, T039, T040, T041 can run in parallel (documentation tasks = different files)

---

## Parallel Execution Example: User Story 1

```bash
# Launch Lesson 1 and Lesson 2 implementation together:
Task T011: "Implement Lesson 1: What is Python?"
Task T012: "Implement Lesson 2: Your First Python Program"

# Both lessons are independent story elements with different file paths
# Can be written in parallel by different developers or agents
```

---

## Implementation Strategy

### Recommended Sequential Approach (Single Developer/Agent)

**Rationale**: Story cohesion requires understanding narrative thread. Lessons build conceptually even though files are independent.

1. **Phase 1**: Setup (T001-T005) - ~30 minutes
2. **Phase 2**: Foundational (T006-T010) - ~2 hours
   - README.md is critical reference for all lessons
   - Story elements document ensures cohesion
3. **Phase 3-7**: User Stories (T011-T024) - Sequential by lesson
   - L1 → L2 (US1) - ~4 hours
   - L3 → L4 (US2) - ~4 hours
   - L5 → L6 (US3) - ~4 hours
   - US4 verification (~30 minutes)
   - US5 verification (~30 minutes)
   - **Total**: ~13 hours of implementation
4. **Phase 8**: QA (T025-T037) - ~3 hours
5. **Phase 9**: Documentation (T038-T041) - ~1 hour (parallel with QA)
6. **Phase 10**: Deployment (T042-T050) - ~1 hour

**Total Estimated Time**: ~20-22 hours (single developer/agent, sequential)

### Alternative Parallel Approach (Multiple Developers/Agents)

**Prerequisites**: Phase 1-2 complete, story elements documented

**Team A (Lessons 1-2 = US1)**:
- T011: Lesson 1 (2 hours)
- T012: Lesson 2 (2 hours)

**Team B (Lessons 3-4 = US2)**:
- T014: Lesson 3 (2 hours)
- T015: Lesson 4 (2 hours)

**Team C (Lessons 5-6 = US3)**:
- T017: Lesson 5 (2 hours)
- T018: Lesson 6 (2.5 hours)

**Team D (QA)**:
- T025-T037: Run validations as lessons complete

**Total Parallel Time**: ~8-10 hours (with 3-4 teams)

---

## Validation Checkpoints (Stop-and-Verify Points)

### Checkpoint 1: After Phase 2 (Foundational)
- ✓ README.md complete and matches Chapter 14 quality
- ✓ Story elements documented for narrative cohesion
- ✓ Validation scripts ready
- ✓ Code testing framework functional

**Decision**: Proceed to lesson implementation OR refine foundation

---

### Checkpoint 2: After User Story 1 (Lessons 1-2)
- ✓ Student can explain "what Python is" using story analogies
- ✓ "Hello, World!" program executes successfully
- ✓ Story-based modality evident (not lecture-style)
- ✓ WHAT-WHY-HOW sequence applied
- ✓ No scaffolding exposure

**Decision**: Continue to US2 OR refine US1 based on validation

---

### Checkpoint 3: After User Story 2 (Lessons 3-4)
- ✓ Variables explained as "labeled memory" conceptually
- ✓ Type hints justified (intent specification, not enforcement)
- ✓ Cognitive load within A2 limits (3 concepts per lesson)
- ✓ Story cohesion maintained (robot narrative thread continues)

**Decision**: Continue to US3 OR refine US2

---

### Checkpoint 4: After User Story 3 (Lessons 5-6)
- ✓ Capstone project integrates all concepts
- ✓ Student can build introduction program independently
- ✓ 6-lesson arc complete (Foundation → Rising Action → Capstone)
- ✓ All code examples tested and executable

**Decision**: Proceed to QA OR refine lessons based on capstone test results

---

### Checkpoint 5: After Phase 8 (QA)
- ✓ All validation audits passed (grep checks, cognitive load, story cohesion)
- ✓ validation-auditor agent confirms constitutional compliance
- ✓ factual-verifier confirms all claims cited
- ✓ Code execution logs attached
- ✓ No forbidden patterns detected

**Decision**: Proceed to deployment OR address validation failures

---

### Checkpoint 6: After Phase 10 (Deployment)
- ✓ PR merged to main
- ✓ Chapter 13 live on production site
- ✓ Navigation functional
- ✓ Docusaurus build succeeds

**Decision**: Chapter 13 redesign COMPLETE ✅

---

## Notes

- **[P] tasks**: Can run in parallel (different files, no dependencies)
- **[Story] labels**: Map tasks to specific user stories for traceability
- **Constitutional Principles**: Every lesson task references applicable principles
- **Validation Protocol**: Code tested → Citations added → Grep audits → Agent validation
- **Story Cohesion**: Maintained through STORY_ELEMENTS.md reference document
- **Cognitive Load**: Monitored continuously (A2 tier: max 5-7 concepts, lessons average 2.2)
- **Stage Progression**: Stage 1 primary (manual foundation), Stage 2 gentle intro ("Try With AI")
- **Anti-Convergence**: Story-based modality differentiated from Chapter 14's analogy-based approach

---

## Success Criteria (From Evals Section)

**This implementation succeeds when**:
1. ✅ 85%+ beginners can explain "what Python is" in own words (EV-001)
2. ✅ 90%+ write "Hello, World!" independently within 15 minutes (EV-002)
3. ✅ 80%+ create variables with type hints successfully (EV-003)
4. ✅ 75%+ explain WHY variables need names and types (EV-004)
5. ✅ 70%+ complete capstone project (EV-005)
6. ✅ All code executes in Python 3.14+ with logs (EV-006)
7. ✅ All claims cited from Python.org (EV-007)
8. ✅ Cognitive load managed at A2 tier (EV-008)
9. ✅ Story-based narrative maintained (EV-009)
10. ✅ "For Curious Learners" sections present (EV-010)
11. ✅ WHAT-WHY-HOW pattern applied (EV-011)
12. ✅ Story elements engage students (EV-012)
13. ✅ Progressive depth evident (EV-013)
14. ✅ Scaffolding hidden from students (EV-014)
15. ✅ Lessons end with "Try With AI" only (EV-015)

**Validation**: Metrics tracked through validation-auditor report + student completion data (post-deployment)

---

**Tasks Document Complete**: Ready for Phase 4 (Implementation) execution via `/sp.implement`

# Tasks: Chapter 22 - IO and File Handling

**Feature Branch**: `001-part-4-chapter-22`
**Input**: Design documents from `/specs/001-part-4-chapter-22/`
**Prerequisites**: plan.md ✅, spec.md ✅

**Organization**: Tasks are organized by lesson (which map to user stories) to enable independent implementation and validation.

**Tests**: NOT requested in specification - no test tasks included.

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, independent work)
- **[Lesson]**: Which lesson this task belongs to (L1=Lesson 1, L2=Lesson 2, etc.)
- File paths follow book structure: `book-source/docs/04-Part-4-Python-Fundamentals/22-io-file-handling/`

---

## Phase 1: Setup (Chapter Structure)

**Purpose**: Create directory structure and initialize chapter files

- [ ] T001 Create chapter directory at `book-source/docs/04-Part-4-Python-Fundamentals/22-io-file-handling/`
- [ ] T002 Create README.md with chapter overview, learning objectives, prerequisites, tools needed
- [ ] T003 [P] Create placeholder files for 5 lessons: `01-lesson-1.md` through `05-lesson-5.md`
- [ ] T004 [P] Create `_category_.json` with chapter metadata (position: 22, label: "Chapter 22: IO and File Handling")

**Checkpoint**: Chapter structure ready - lesson writing can begin in parallel

---

## Phase 2: Foundational (Shared Resources)

**Purpose**: Resources used across multiple lessons

- [ ] T005 Create `examples/` subdirectory for code sample files
- [ ] T006 [P] Create sample data files: `examples/sample.txt`, `examples/employees.csv`, `examples/notes.json`
- [ ] T007 [P] Document Python 3.14+ setup requirements in README.md

**Checkpoint**: Shared resources ready - lessons can reference common examples

---

## Phase 3: Lesson 1 - Console I/O and User Input Validation (Priority: P1) 🎯

**Goal**: Students can gather user input with validation, handle errors gracefully, and format output using f-strings

**Independent Test**: Lesson teaches input validation patterns; students complete exercise asking for age with retry on invalid input

**Maps to**: User Story 1 (Console Interaction with Validation)

### Lesson 1 Implementation

- [ ] T008 [L1] Write Lesson 1 introduction section (5 min) - why console I/O matters, connection to CLI programs
- [ ] T009 [L1] Write "Understanding input()" concept section (8 min) - how input() works, string return type, terminal behavior
- [ ] T010 [L1] Write "Type Conversion and Validation" concept section (10 min) - int(), float(), try/except, ValueError
- [ ] T011 [L1] Write "F-String Formatting" concept section (8 min) - f-string syntax, expressions, number formatting
- [ ] T012 [L1] Write "Error Recovery Loops" concept section (7 min) - retry logic, user-friendly messages
- [ ] T013 [P] [L1] Create Code Example 1.1 (Basic input() with Type Conversion) in lesson file with explanation
- [ ] T014 [P] [L1] Create Code Example 1.2 (Input Validation with Error Handling) with function pattern
- [ ] T015 [P] [L1] Create Code Example 1.3 (Formatted Output with Numbers) demonstrating f-string formatting
- [ ] T016 [P] [L1] Write 💬 AI Colearning Prompt (after Code Example 1.1) - explore type conversion mechanism
- [ ] T017 [P] [L1] Write 🎓 Instructor Commentary (after try/except) - "syntax is cheap, semantics is gold"
- [ ] T018 [P] [L1] Write 🚀 CoLearning Challenge (after Code Example 1.2) - email validation function
- [ ] T019 [P] [L1] Write ✨ Teaching Tip (before formatting section) - explore f-string variations with Claude Code
- [ ] T020 [L1] Write Practice Exercise 1 (name + favorite number with validation)
- [ ] T021 [L1] Write Practice Exercise 2 (input validation function for minimum length)
- [ ] T022 [L1] Write "Try With AI" section with 4 prompts (Remember/Understand → Apply → Analyze → Synthesize)
- [ ] T023 [L1] Add YAML frontmatter with lesson metadata (title, duration, CEFR level A2, skills taught)

**Checkpoint**: Lesson 1 complete - students can build validated console input programs

---

## Phase 4: Lesson 2 - File I/O Fundamentals with Context Managers (Priority: P1)

**Goal**: Students can safely read/write files using context managers, handle file exceptions, understand file modes

**Independent Test**: Lesson teaches file safety; students write program that reads file with error handling and writes to new file

**Maps to**: User Story 2 (Safe File Operations)

### Lesson 2 Implementation

- [ ] T024 [L2] Write Lesson 2 introduction section (5 min) - why files matter, persistence vs console I/O
- [ ] T025 [L2] Write "Context Managers and File Safety" concept section (10 min) - with statement, automatic cleanup
- [ ] T026 [L2] Write "File Modes" concept section (8 min) - 'r', 'w', 'a', 'r+' with file state examples
- [ ] T027 [L2] Write "Reading Methods" concept section (10 min) - read(), readline(), readlines() comparison
- [ ] T028 [L2] Write "Writing Files and Newlines" concept section (8 min) - write(), writelines(), '\n' handling
- [ ] T029 [L2] Write "File Exceptions" concept section (7 min) - FileNotFoundError, PermissionError, IOError
- [ ] T030 [P] [L2] Create Code Example 2.1 (Safe File Reading with Context Manager) with exception handling
- [ ] T031 [P] [L2] Create Code Example 2.2 (Different File Modes and Reading Methods) comparing read/readline/readlines
- [ ] T032 [P] [L2] Create Code Example 2.3 (Append Mode and File Growth) with log file pattern
- [ ] T033 [P] [L2] Create Code Example 2.4 (Error Handling for File Operations) with comprehensive exception handling
- [ ] T034 [P] [L2] Write 💬 AI Colearning Prompt (after Code Example 2.1) - explore context manager benefits
- [ ] T035 [P] [L2] Write 🎓 Instructor Commentary (after file modes) - specify intent, not syntax
- [ ] T036 [P] [L2] Write 🚀 CoLearning Challenge (after Code Example 2.3) - log file processing with search
- [ ] T037 [P] [L2] Write ✨ Teaching Tip (before error handling) - explore file errors with AI
- [ ] T038 [L2] Write Practice Exercise 1 (write 5 lines, read back with line numbers)
- [ ] T039 [L2] Write Practice Exercise 2 (append timestamped log message, handle missing file)
- [ ] T040 [L2] Write Practice Exercise 3 (error handling for missing file and permission errors)
- [ ] T041 [L2] Write "Try With AI" section with 4 prompts (Remember/Understand → Apply → Analyze → Synthesize)
- [ ] T042 [L2] Add YAML frontmatter with lesson metadata (title, duration, CEFR level A2-B1, skills taught)

**Checkpoint**: Lesson 2 complete - students can safely perform file I/O operations

---

## Phase 5: Lesson 3 - Cross-Platform Path Handling with pathlib (Priority: P2)

**Goal**: Students can use pathlib.Path for cross-platform paths, check existence, create directories, use glob patterns, understand path security

**Independent Test**: Lesson teaches pathlib; students create nested directory structure and list files by pattern

**Maps to**: User Story 3 (Cross-Platform Path Handling)

### Lesson 3 Implementation

- [ ] T043 [L3] Write Lesson 3 introduction section (5 min) - why pathlib essential, moving beyond hardcoded paths
- [ ] T044 [L3] Write "pathlib vs os.path" concept section (8 min) - advantages of Path objects
- [ ] T045 [L3] Write "Creating and Manipulating Paths" concept section (10 min) - Path(), `/` operator, properties
- [ ] T046 [L3] Write "Checking Existence and File Types" concept section (8 min) - exists(), is_file(), is_dir()
- [ ] T047 [L3] Write "Creating Directories" concept section (7 min) - mkdir(parents=True, exist_ok=True)
- [ ] T048 [L3] Write "Finding Files with Glob" concept section (7 min) - iterdir(), glob() patterns
- [ ] T049 [L3] Write "Path Security and Traversal Prevention" concept section (8 min) - resolve(), boundary validation
- [ ] T050 [P] [L3] Create Code Example 3.1 (Creating Paths with pathlib) showing Path objects and properties
- [ ] T051 [P] [L3] Create Code Example 3.2 (Checking File Existence and Type) with defensive pattern
- [ ] T052 [P] [L3] Create Code Example 3.3 (Creating Nested Directories) with loop for categories
- [ ] T053 [P] [L3] Create Code Example 3.4 (Finding Files with Glob Patterns) demonstrating file discovery
- [ ] T054 [P] [L3] Create Code Example 3.5 (Path Traversal Security) with validation function
- [ ] T055 [P] [L3] Write 💬 AI Colearning Prompt (after Code Example 3.1) - explore `/` operator behavior
- [ ] T056 [P] [L3] Write 🎓 Instructor Commentary (after cross-platform paths) - specify structure, not paths
- [ ] T057 [P] [L3] Write 🚀 CoLearning Challenge (after Code Example 3.3) - multi-project directory structure
- [ ] T058 [P] [L3] Write ✨ Teaching Tip (before security section) - explore path attacks with AI
- [ ] T059 [L3] Write Practice Exercise 1 (create nested directory with exists check)
- [ ] T060 [L3] Write Practice Exercise 2 (list files by extension using glob)
- [ ] T061 [L3] Write Practice Exercise 3 (path validation preventing traversal)
- [ ] T062 [L3] Write "Try With AI" section with 4 prompts (Remember/Understand → Apply → Analyze → Synthesize)
- [ ] T063 [L3] Add YAML frontmatter with lesson metadata (title, duration, CEFR level B1, skills taught)

**Checkpoint**: Lesson 3 complete - students can work with paths safely across platforms

---

## Phase 6: Lesson 4 - Structured Data Formats (CSV and JSON) (Priority: P2)

**Goal**: Students can read/write CSV and JSON with proper encoding, handle parsing errors, choose appropriate format

**Independent Test**: Lesson teaches data formats; students convert CSV to JSON with encoding

**Maps to**: User Story 4 (Structured Data Formats)

### Lesson 4 Implementation

- [ ] T064 [L4] Write Lesson 4 introduction section (5 min) - why structured formats matter, CSV vs JSON
- [ ] T065 [L4] Write "CSV Format and csv Module" concept section (10 min) - rows, headers, DictReader
- [ ] T066 [L4] Write "JSON Format and json Module" concept section (8 min) - objects, arrays, nested data
- [ ] T067 [L4] Write "Character Encoding (UTF-8)" concept section (8 min) - encoding importance, ensure_ascii
- [ ] T068 [L4] Write "Reading CSV Files" concept section (8 min) - csv.reader vs csv.DictReader
- [ ] T069 [L4] Write "Writing CSV Files" concept section (7 min) - headers, DictWriter
- [ ] T070 [L4] Write "JSON Serialization/Deserialization" concept section (7 min) - dump(), load(), pretty printing
- [ ] T071 [L4] Write "Error Handling for Formats" concept section (6 min) - JSONDecodeError, ValueError
- [ ] T072 [P] [L4] Create Code Example 4.1 (Reading CSV Files) comparing csv.reader and csv.DictReader
- [ ] T073 [P] [L4] Create Code Example 4.2 (Writing CSV Files) with DictWriter and headers
- [ ] T074 [P] [L4] Create Code Example 4.3 (Reading and Writing JSON) demonstrating serialization
- [ ] T075 [P] [L4] Create Code Example 4.4 (Handling Encoding for International Text) with emoji/Unicode
- [ ] T076 [P] [L4] Create Code Example 4.5 (Error Handling for Format Errors) with JSONDecodeError
- [ ] T077 [P] [L4] Write 💬 AI Colearning Prompt (after Code Example 4.1) - compare reader types
- [ ] T078 [P] [L4] Write 🎓 Instructor Commentary (after CSV/JSON) - specify data structure intent
- [ ] T079 [P] [L4] Write 🚀 CoLearning Challenge (after Code Example 4.2) - CSV→JSON transformation with filtering
- [ ] T080 [P] [L4] Write ✨ Teaching Tip (before encoding section) - explore encoding issues with AI
- [ ] T081 [L4] Write Practice Exercise 1 (read CSV, display formatted table)
- [ ] T082 [L4] Write Practice Exercise 2 (convert CSV to JSON, preserve international characters)
- [ ] T083 [L4] Write Practice Exercise 3 (error handling for corrupted CSV/JSON)
- [ ] T084 [L4] Write "Try With AI" section with 4 prompts (Remember/Understand → Apply → Analyze → Synthesize)
- [ ] T085 [L4] Add YAML frontmatter with lesson metadata (title, duration, CEFR level B1, skills taught)

**Checkpoint**: Lesson 4 complete - students can work with structured data formats

---

## Phase 7: Lesson 5 - Capstone Note-Taking App (Priority: P1 - Integration) 🎯

**Goal**: Students integrate all I/O concepts into complete CLI application with CRUD operations, search, and category organization

**Independent Test**: Students build working Note-Taking App handling 10-50 notes with menu-driven interface

**Maps to**: User Story 5 (Complete Note-Taking App)

### Lesson 5 Implementation

- [ ] T086 [L5] Write Lesson 5 introduction section (10 min) - application architecture, design overview
- [ ] T087 [L5] Write "Application Architecture Introduction" section - user flow diagram, components
- [ ] T088 [L5] Write "Setting Up Project Structure" section (10 min) - initialize notes/ directory, organization
- [ ] T089 [L5] Write "Core CRUD Functions" section (40 min) - get_all_notes(), save_note(), delete_note(), search_notes()
- [ ] T090 [L5] Write "Menu Loop Implementation" section (20 min) - main loop with input handling
- [ ] T091 [L5] Write "Error Handling and Validation" section (10 min) - input validation, file operation errors
- [ ] T092 [L5] Write "Testing and Refinement" section (10 min) - manual testing approach, edge cases
- [ ] T093 [P] [L5] Create Code Example 5.1 (Project Structure and Initialization) with security validation
- [ ] T094 [P] [L5] Create Code Example 5.2 (CRUD Functions) with complete implementations
- [ ] T095 [P] [L5] Create Code Example 5.3 (Menu Loop with Input Validation) showing full app structure
- [ ] T096 [P] [L5] Write 💬 AI Colearning Prompt (at project start) - explore data structure design decisions
- [ ] T097 [P] [L5] Write 🎓 Instructor Commentary (after CRUD functions) - specify app flow, not file handling
- [ ] T098 [P] [L5] Write 🚀 CoLearning Challenge (during development) - optimize for 1000 notes scenario
- [ ] T099 [P] [L5] Write ✨ Teaching Tip (during testing) - test edge cases with AI
- [ ] T100 [L5] Document Project Deliverables (app.py, helper functions, data directory, working features)
- [ ] T101 [L5] Write Success Criteria checklist (8 criteria from plan.md)
- [ ] T102 [L5] Write "Try With AI" section with 4 prompts (Remember/Understand → Apply → Analyze → Synthesize)
- [ ] T103 [L5] Add YAML frontmatter with lesson metadata (title, duration 90-120 min, CEFR level B1, skills taught)

**Checkpoint**: Lesson 5 complete - students have built complete file-based CLI application

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Chapter-level review, validation, and publication preparation

- [ ] T104 [P] Validate all code examples run correctly with Python 3.14+
- [ ] T105 [P] Verify all code uses modern syntax (type hints, pathlib, f-strings, context managers)
- [ ] T106 [P] Check all lessons end with ONLY "Try With AI" section (no summaries after)
- [ ] T107 [P] Verify cognitive load: max 7 concepts per lesson (Lessons 1-4), 0 new for Lesson 5
- [ ] T108 [P] Validate CoLearning elements distributed throughout lessons (not just at end)
- [ ] T109 [P] Confirm all 31 skills from plan.md have proficiency metadata in lesson frontmatter
- [ ] T110 [P] Test cross-platform compatibility (paths work on Windows, Mac, Linux)
- [ ] T111 Update chapter README.md with learning outcomes, prerequisites, estimated time
- [ ] T112 Create chapter index entry in parent directory's `_category_.json`
- [ ] T113 Run constitution compliance check (spec-first workflow, validation steps, graduated teaching pattern)
- [ ] T114 Verify all "Try With AI" prompts follow Bloom's progression (4 per lesson, 20 total)
- [ ] T115 Validate capstone project completable in 90-120 minutes for B1 learners
- [ ] T116 Cross-reference with Chapters 21 (exception handling) and 23 (datetime) for continuity
- [ ] T117 Final editorial review for voice, tone, and flow
- [ ] T118 Docusaurus build test to ensure no formatting errors

**Checkpoint**: Chapter 22 ready for publication

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) completion
- **Lessons (Phases 3-7)**: All depend on Foundational (Phase 2) completion
  - Lessons CAN proceed in parallel if multiple writers available
  - OR sequentially: Lesson 1 → Lesson 2 → Lesson 3 → Lesson 4 → Lesson 5
- **Polish (Phase 8)**: Depends on ALL lessons (Phases 3-7) complete

### Lesson Dependencies

- **Lesson 1 (Phase 3)**: Can start after Foundational - teaches console I/O foundation
- **Lesson 2 (Phase 4)**: Can start after Foundational - teaches file I/O (independent of Lesson 1 completion, but builds conceptually)
- **Lesson 3 (Phase 5)**: Can start after Foundational - teaches pathlib (independent of Lessons 1-2)
- **Lesson 4 (Phase 6)**: Can start after Foundational - teaches CSV/JSON (independent of Lessons 1-3)
- **Lesson 5 (Phase 7)**: Should start after Lessons 1-4 complete (integrates all concepts)

### Within Each Lesson

- Introduction → Concept sections → Code examples → CoLearning elements → Practice exercises → Try With AI → Frontmatter
- Code examples marked [P] can be written in parallel (independent)
- CoLearning elements marked [P] can be written in parallel (independent)
- "Try With AI" section must be last content item in lesson

### Parallel Opportunities

- **Phase 1**: Tasks T003-T004 can run in parallel
- **Phase 2**: Tasks T006-T007 can run in parallel
- **Lessons**: Code examples within each lesson can be written in parallel
- **Lessons**: CoLearning elements within each lesson can be written in parallel
- **Phase 8**: Most polish tasks (T104-T110) can run in parallel

---

## Parallel Example: Lesson 2 Code Examples

```bash
# Launch all code examples for Lesson 2 together:
Task T030: "Create Code Example 2.1 (Safe File Reading)"
Task T031: "Create Code Example 2.2 (Different File Modes)"
Task T032: "Create Code Example 2.3 (Append Mode)"
Task T033: "Create Code Example 2.4 (Error Handling)"

# Then launch all CoLearning elements together:
Task T034: "Write 💬 AI Colearning Prompt"
Task T035: "Write 🎓 Instructor Commentary"
Task T036: "Write 🚀 CoLearning Challenge"
Task T037: "Write ✨ Teaching Tip"
```

---

## Implementation Strategy

### Sequential Approach (Single Writer)

1. Complete Phase 1: Setup (4 tasks) → Chapter structure ready
2. Complete Phase 2: Foundational (3 tasks) → Shared resources ready
3. Complete Phase 3: Lesson 1 (16 tasks) → Console I/O lesson done
4. Complete Phase 4: Lesson 2 (19 tasks) → File I/O lesson done
5. Complete Phase 5: Lesson 3 (21 tasks) → Pathlib lesson done
6. Complete Phase 6: Lesson 4 (22 tasks) → CSV/JSON lesson done
7. Complete Phase 7: Lesson 5 (18 tasks) → Capstone lesson done
8. Complete Phase 8: Polish (15 tasks) → Chapter ready for publication

**Total Tasks**: 118 tasks

### Parallel Approach (Multiple Writers)

1. All: Complete Phase 1 + 2 (Setup + Foundational) → 7 tasks
2. Once Foundational complete:
   - Writer A: Lesson 1 (Phase 3) → 16 tasks
   - Writer B: Lesson 2 (Phase 4) → 19 tasks
   - Writer C: Lesson 3 (Phase 5) → 21 tasks
   - Writer D: Lesson 4 (Phase 6) → 22 tasks
3. After Lessons 1-4 complete:
   - Writer E: Lesson 5 (Phase 7) → 18 tasks (requires all concepts)
4. All: Phase 8 Polish → 15 tasks (can parallelize most validation)

### MVP First (Minimum Viable Product)

**MVP = Lesson 1 only** (smallest publishable unit):
1. Phase 1: Setup → 4 tasks
2. Phase 2: Foundational → 3 tasks
3. Phase 3: Lesson 1 complete → 16 tasks
4. Partial Phase 8: Basic validation → 5 tasks

**Total for MVP**: 28 tasks → Validates approach before full chapter

Then incrementally add Lessons 2, 3, 4, 5.

---

## Notes

- All code examples MUST use Python 3.14+ with modern type hints (`list[str]`, `dict[str, int]`, `X | None`)
- All file operations MUST use `with` context managers
- All path operations MUST use `pathlib.Path` (NOT `os.path`)
- All lessons MUST end with "Try With AI" section ONLY (no summaries or checklists after)
- CoLearning elements MUST be distributed throughout lesson (not just at end)
- Maximum 7 new concepts per lesson (A2-B1 cognitive load limit)
- Lesson 5 introduces 0 new concepts (integration only)
- All tasks reference exact file paths in `book-source/docs/04-Part-4-Python-Fundamentals/22-io-file-handling/`
- [P] tasks can run in parallel (different content sections, no dependencies)
- Commit after each lesson completes for checkpoint validation

---

## Success Metrics (From Specification)

After implementation complete, chapter must satisfy:

- ✅ **EVAL-001**: 75%+ can explain console I/O vs file I/O (concept quiz)
- ✅ **EVAL-004**: 85%+ can write file reading code with error handling (hands-on)
- ✅ **EVAL-005**: 75%+ can use pathlib for cross-platform operations (practical task)
- ✅ **EVAL-006**: 70%+ can work with CSV/JSON correctly (capstone requirement)
- ✅ **EVAL-007**: 80%+ complete capstone Note-Taking App
- ✅ **EVAL-008**: Average 3.5+ "Try With AI" prompts attempted per lesson
- ✅ **SC-005**: Capstone completable in 90 minutes for B1 learners

---

**Tasks Status**: ✅ READY FOR IMPLEMENTATION

This task list provides complete guidance for implementing all 5 lessons of Chapter 22 with full pedagogical scaffolding, CoLearning elements, and constitutional compliance.

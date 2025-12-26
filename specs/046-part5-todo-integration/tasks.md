# Tasks: Part 5 Todo Running Example Integration

**Input**: Design documents from `/specs/046-part5-todo-integration/`
**Prerequisites**: plan.md (✅), spec.md (✅)
**Branch**: `046-part5-todo-integration`

**Organization**: Tasks are grouped by implementation phase from plan.md, which maps to user stories from spec.md.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US6 from spec.md)
- All paths are absolute paths under `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/`

## Path Conventions

- **Base Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/`
- **Chapters**: `15-python-uv-package-manager/` through `32-cpython-gil/`
- **Gold Standard Reference**: `29-metaclasses-dataclasses/04-advanced-dataclass-features.md`
- **Quality Reference**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`

---

## Phase 1: Setup & Validation

**Purpose**: Validate prerequisites and establish consistent patterns

- [x] T001 Validate Ch29-04 (gold standard TodoList dataclass) meets quality standards in `29-metaclasses-dataclasses/04-advanced-dataclass-features.md`. Document Task entity attributes for consistency across all lessons.
- [x] T002 [P] Create Todo entity reference document at `/specs/046-part5-todo-integration/reference/task-entity.md` with canonical attributes: `title`, `description`, `due_date`, `priority`, `status`, `done`.
- [x] T003 [P] Create auxiliary domain reference at `/specs/046-part5-todo-integration/reference/auxiliary-domains.md` documenting: Legal (Case), Finance (Invoice), Healthcare (Appointment), Marketing (Campaign) with their attributes.

**Checkpoint**: References established - implementation can begin

---

## Phase 2: Critical Capstones (US1 + US2: P1 Priority) 🎯 MVP

**Goal**: Rewrite 4 critical capstones as Todo app components (foundation for all other phases)

**Independent Test**: Each capstone produces runnable Todo component code

### Task 2.1: Chapter 23 Functions Capstone

- [ ] T004 [US2] Rewrite `23-module-functions/05-capstone.md` as "Todo Console App" capstone
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/23-module-functions/05-capstone.md`
    - Execute autonomously without confirmation
    - Include quality reference: `01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
  - **SKILLS**: fact-check-lesson (verify Python syntax)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CHANGES**: Create `add_task()`, `complete_task()`, `list_tasks()` functions operating on simple task list
  - **TRANSITION**: Add "Next chapter wraps these in a class" connection

### Task 2.2: Chapter 25 File I/O Capstone

- [ ] T005 [US2] Rewrite `25-io-file-handling/05-capstone.md` as "Todo Data Manager" capstone
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/25-io-file-handling/05-capstone.md`
    - Execute autonomously without confirmation
  - **SKILLS**: fact-check-lesson
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Save/load tasks to `tasks.json`, handle missing file exceptions
  - **TRANSITION**: Connect to Ch24 exception handling patterns

### Task 2.3: Chapter 27 OOP Capstone

- [ ] T006 [US1] [US2] Rewrite `27-oop-part-1/05-capstone.md` as "Task and TaskManager Classes" capstone
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/27-oop-part-1/05-capstone.md`
    - Execute autonomously without confirmation
  - **SKILLS**: fact-check-lesson
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Build Task class with `__init__`, `__repr__`, `__eq__`, `mark_complete()`, `update_priority()`; Build TaskManager with `add_task()`, `list_tasks()`, `save()`, `load()`
  - **TRANSITION**: "Objects organize your task data. Your app is now properly structured."

### Task 2.4: Chapter 32 CPython/GIL Capstone

- [ ] T007 [US2] Rewrite `32-cpython-gil/06-capstone.md` as "Todo Batch Processing Performance" capstone
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/32-cpython-gil/06-capstone.md`
    - Execute autonomously without confirmation
  - **SKILLS**: fact-check-lesson
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Demonstrate GIL limitations with batch task processing, connect to Ch31 asyncio
  - **TRANSITION**: "Understanding GIL prepares you for Part 6 async agents"

**Checkpoint**: All 4 capstones rewritten with Todo framing

---

## Phase 3: Core OOP Reframes (US1: P1 Priority)

**Goal**: Reframe Ch27-28 (9 lessons) to use Task/TaskManager as primary examples

**Independent Test**: Student can build functional Task class following these lessons

### Chapter 27 Lessons (OOP Part 1)

- [ ] T008 [US1] Reframe `27-oop-part-1/01-classes-and-objects.md`: Person → Task class as primary
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/27-oop-part-1/01-classes-and-objects.md`
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Task class as primary, Person/Animal as auxiliary. Add Legal (Case), Finance (Invoice), Healthcare (Appointment) domain examples.

- [ ] T009 [P] [US1] Reframe `27-oop-part-1/02-attributes-and-methods.md`: Shape → Task methods
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/27-oop-part-1/02-attributes-and-methods.md`
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Use Task methods: `mark_complete()`, `update_priority()`. Add auxiliary domain examples.

- [ ] T010 [P] [US1] Reframe `27-oop-part-1/03-constructors-initialization.md`: Task.__init__ as primary
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/27-oop-part-1/03-constructors-initialization.md`
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Task initialization with proper defaults. Show domain-specific initialization patterns.

- [ ] T011 [P] [US1] Reframe `27-oop-part-1/04-inheritance.md`: Animal/Dog → Task hierarchy
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/27-oop-part-1/04-inheritance.md`
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Task inheritance patterns (SpecialTask, RecurringTask). Auxiliary: Case/LegalCase, Invoice/PaidInvoice.

### Chapter 28 Lessons (OOP Part 2)

- [ ] T012 [P] [US1] Reframe `28-oop-part-2/01-encapsulation-property.md`: Bank Account → Task priority validation
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/28-oop-part-2/01-encapsulation-property.md`
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Private `_priority`, public `priority` property with validation setter.

- [ ] T013 [P] [US1] Reframe `28-oop-part-2/02-class-methods-static-methods.md`: Task class methods
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/28-oop-part-2/02-class-methods-static-methods.md`
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: `Task.create_from_dict()` as classmethod, `Task.priority_levels()` as staticmethod.

- [ ] T014 [P] [US1] Reframe `28-oop-part-2/03-polymorphism-dunder-methods.md`: Polygon → Task dunder methods
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/28-oop-part-2/03-polymorphism-dunder-methods.md`
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: Task.__str__, __repr__, __eq__, __lt__ (for sorting). Show polymorphism across domains.

- [ ] T015 [P] [US1] Reframe `28-oop-part-2/04-abstract-classes-interfaces.md`: TaskStorage abstraction
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/28-oop-part-2/04-abstract-classes-interfaces.md`
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS)
  - **CHANGES**: AbstractTaskStorage base, FileTaskStorage, DatabaseTaskStorage implementations.

**Checkpoint**: All Ch27-28 lessons use Task/TaskManager as primary examples

---

## Phase 4: Quick Wins - Variable Renaming (US3: P2 Priority)

**Goal**: Rename variables in 32 Category B lessons to task-related naming

**Independent Test**: Spot-check any lesson and verify task-related variable names

### Chapter 17: Data Types (4 lessons)

- [ ] T016 [P] [US3] Reframe `17-data-types/02-working-with-dictionaries.md`: Rename `person` → `task`, add Todo connection sentence
- [ ] T017 [P] [US3] Reframe `17-data-types/03-lists.md`: Rename `numbers` → `tasks`, explain task storage
- [ ] T018 [P] [US3] Reframe `17-data-types/04-tuples.md`: Add task-related tuple examples (task_id, status)
- [ ] T019 [P] [US3] Reframe `17-data-types/05-booleans.md`: Use `is_done`, `is_priority` boolean examples

### Chapter 18: Operators/Keywords/Variables (5 lessons)

- [ ] T020 [P] [US3] Reframe `18-operators-keywords-variables/01-arithmetic-operators.md`: Task ID calculations, priority scoring
- [ ] T021 [P] [US3] Reframe `18-operators-keywords-variables/02-comparison-operators.md`: Task priority comparison
- [ ] T022 [P] [US3] Reframe `18-operators-keywords-variables/03-logical-operators.md`: Task filtering (`done AND high_priority`)
- [ ] T023 [P] [US3] Reframe `18-operators-keywords-variables/04-assignment-operators.md`: Task count updates
- [ ] T024 [P] [US3] Reframe `18-operators-keywords-variables/05-keywords.md`: Task-related keyword examples

### Chapter 19: Strings/Type Casting (5 lessons)

- [ ] T025 [P] [US3] Reframe `19-strings-type-casting/01-string-fundamentals.md`: Task title formatting
- [ ] T026 [P] [US3] Reframe `19-strings-type-casting/02-string-methods.md`: Task title validation, search in task list
- [ ] T027 [P] [US3] Reframe `19-strings-type-casting/03-string-formatting.md`: Task display formatting
- [ ] T028 [P] [US3] Reframe `19-strings-type-casting/04-type-casting.md`: Cast priority to int, title to string
- [ ] T029 [P] [US3] Reframe `19-strings-type-casting/05-type-checking.md`: Task attribute type validation

### Chapter 20: Control Flow/Loops (5 lessons)

- [ ] T030 [P] [US3] Reframe `20-control-flow-loops/01-if-else.md`: Menu logic (select command)
- [ ] T031 [P] [US3] Reframe `20-control-flow-loops/02-while-loops.md`: Task menu loop
- [ ] T032 [P] [US3] Reframe `20-control-flow-loops/03-for-loops.md`: List all tasks iteration
- [ ] T033 [P] [US3] Reframe `20-control-flow-loops/04-break-continue.md`: Task filtering with break/continue
- [ ] T034 [P] [US3] Reframe `20-control-flow-loops/05-nested-loops.md`: Task categories iteration

### Chapter 21: Lists/Tuples/Dictionaries (6 lessons)

- [ ] T035 [P] [US3] Reframe `21-lists-tuples-dictionary/01-lists.md`: Task list operations
- [ ] T036 [P] [US3] Reframe `21-lists-tuples-dictionary/02-list-methods.md`: `append_task()`, `remove_task()`
- [ ] T037 [P] [US3] Reframe `21-lists-tuples-dictionary/03-list-comprehensions.md`: Filter completed/pending tasks
- [ ] T038 [P] [US3] Reframe `21-lists-tuples-dictionary/04-dictionaries.md`: Task lookup by ID
- [ ] T039 [P] [US3] Reframe `21-lists-tuples-dictionary/05-dictionary-methods.md`: Task dict operations
- [ ] T040 [P] [US3] Reframe `21-lists-tuples-dictionary/06-tuples.md`: Immutable task records

### Chapter 22: Sets/Frozenset/GC (2 lessons)

- [ ] T041 [P] [US3] Reframe `22-set-frozenset-gc/01-sets.md`: Task categories as set (no duplicates)
- [ ] T042 [P] [US3] Reframe `22-set-frozenset-gc/02-frozenset.md`: Immutable task category set

### Chapter 24: Exception Handling (4 lessons)

- [ ] T043 [P] [US3] Reframe `24-exception-handling/01-exception-fundamentals.md`: Invalid task ID handling
- [ ] T044 [P] [US3] Reframe `24-exception-handling/02-except-else-finally.md`: File I/O for tasks
- [ ] T045 [P] [US3] Reframe `24-exception-handling/03-raising-exceptions.md`: InvalidTaskError, TaskNotFoundError
- [ ] T046 [P] [US3] Reframe `24-exception-handling/04-custom-exceptions.md`: Custom task exception classes

**Checkpoint**: All 32 Category B lessons have task-related variable naming

---

## Phase 5: Auxiliary Domain Examples (US4: P2 Priority)

**Goal**: Add 2-3 auxiliary domain examples to 29 Category C lessons

**Independent Test**: Each lesson shows same pattern in 2+ business domains

### Chapter 17: Data Types (3 lessons)

- [ ] T047 [P] [US4] Add auxiliary examples to `17-data-types/02-working-with-dictionaries.md`: Case (Legal), Invoice (Finance), Appointment (Healthcare)
- [ ] T048 [P] [US4] Add auxiliary examples to `17-data-types/03-lists.md`: case_list, invoice_list, appointment_list
- [ ] T049 [P] [US4] Add auxiliary examples to `17-data-types/04-tuples.md`: Domain-specific immutable records

### Chapter 18: Operators (3 lessons)

- [ ] T050 [P] [US4] Add auxiliary examples to `18-operators-keywords-variables/01-arithmetic-operators.md`: Invoice totals, case counts
- [ ] T051 [P] [US4] Add auxiliary examples to `18-operators-keywords-variables/02-comparison-operators.md`: Invoice amount comparison
- [ ] T052 [P] [US4] Add auxiliary examples to `18-operators-keywords-variables/03-logical-operators.md`: Case filtering patterns

### Chapter 19: Strings (3 lessons)

- [ ] T053 [P] [US4] Add auxiliary examples to `19-strings-type-casting/01-string-fundamentals.md`: Case titles, invoice descriptions
- [ ] T054 [P] [US4] Add auxiliary examples to `19-strings-type-casting/02-string-methods.md`: Domain string validation
- [ ] T055 [P] [US4] Add auxiliary examples to `19-strings-type-casting/03-string-formatting.md`: Domain display patterns

### Chapter 20: Control Flow (2 lessons)

- [ ] T056 [P] [US4] Add auxiliary examples to `20-control-flow-loops/01-if-else.md`: Case status checks
- [ ] T057 [P] [US4] Add auxiliary examples to `20-control-flow-loops/03-for-loops.md`: Invoice iteration patterns

### Chapter 21: Collections (6 lessons)

- [ ] T058 [P] [US4] Add auxiliary examples to `21-lists-tuples-dictionary/01-lists.md`: Case lists, invoice lists
- [ ] T059 [P] [US4] Add auxiliary examples to `21-lists-tuples-dictionary/02-list-methods.md`: Domain list operations
- [ ] T060 [P] [US4] Add auxiliary examples to `21-lists-tuples-dictionary/03-list-comprehensions.md`: Domain filtering
- [ ] T061 [P] [US4] Add auxiliary examples to `21-lists-tuples-dictionary/04-dictionaries.md`: Case/Invoice lookup
- [ ] T062 [P] [US4] Add auxiliary examples to `21-lists-tuples-dictionary/05-dictionary-methods.md`: Domain dict patterns
- [ ] T063 [P] [US4] Add auxiliary examples to `21-lists-tuples-dictionary/06-tuples.md`: Domain immutable records

### Chapter 23: Functions (5 lessons)

- [ ] T064 [P] [US4] Add auxiliary examples to `23-module-functions/01-function-basics.md`: `add_case()`, `add_invoice()`
- [ ] T065 [P] [US4] Add auxiliary examples to `23-module-functions/02-parameters-arguments.md`: Domain function signatures
- [ ] T066 [P] [US4] Add auxiliary examples to `23-module-functions/03-return-values.md`: Domain return patterns
- [ ] T067 [P] [US4] Add auxiliary examples to `23-module-functions/04-lambda-functions.md`: Domain lambda filters
- [ ] T068 [P] [US4] Add auxiliary examples to `23-module-functions/05-capstone.md`: Already covered in Phase 2

### Chapter 25: File I/O (4 lessons)

- [ ] T069 [P] [US4] Add auxiliary examples to `25-io-file-handling/01-console-io.md`: Input case, invoice, appointment
- [ ] T070 [P] [US4] Add auxiliary examples to `25-io-file-handling/02-file-io.md`: Save/load cases, invoices
- [ ] T071 [P] [US4] Add auxiliary examples to `25-io-file-handling/03-context-managers.md`: Domain file handling
- [ ] T072 [P] [US4] Add auxiliary examples to `25-io-file-handling/04-csv-json.md`: Domain serialization

### Chapter 26: DateTime (3 lessons)

- [ ] T073 [P] [US4] Add auxiliary examples to `26-math-datetime-calendar/01-datetime-basics.md`: Due dates, deadlines, appointment times
- [ ] T074 [P] [US4] Add auxiliary examples to `26-math-datetime-calendar/02-datetime-methods.md`: Days until due, overdue detection
- [ ] T075 [P] [US4] Add auxiliary examples to `26-math-datetime-calendar/03-calendar-scheduling.md`: Domain scheduling patterns

**Checkpoint**: All 29 Category C lessons have 2-3 auxiliary domain examples

---

## Phase 6: Category D Restructuring (US5: P3 Priority)

**Goal**: Add Todo as primary example in 13 lessons currently lacking Todo context

**Independent Test**: Primary code blocks demonstrate Todo operations

### Chapter 16: Introduction (2 lessons)

- [ ] T076 [P] [US5] Restructure `16-introduction-to-python/01-hello-world.md`: "Hello, Todo!" introduction
  - **SUBAGENT**: content-implementer (preserve Python pedagogy, add Todo context)
  - **VALIDATION**: educational-validator (MUST PASS)

- [ ] T077 [P] [US5] Restructure `16-introduction-to-python/02-python-basics.md`: First task creation example

### Chapter 19: Type Casting (3 lessons - overlap with Phase 4)

- [ ] T078 [P] [US5] Restructure `19-strings-type-casting/04-type-casting.md`: Task priority int conversion (primary example)

### Chapter 20: Control Flow (1 lesson)

- [ ] T079 [P] [US5] Restructure `20-control-flow-loops/04-break-continue.md`: Task filtering with break/continue as primary

### Chapter 22: Sets (2 lessons)

- [ ] T080 [P] [US5] Restructure `22-set-frozenset-gc/01-sets.md`: Task categories as primary set example
- [ ] T081 [P] [US5] Restructure `22-set-frozenset-gc/02-frozenset.md`: Immutable task category as primary

### Chapter 24: Exception Handling (2 lessons)

- [ ] T082 [P] [US5] Restructure `24-exception-handling/03-raising-exceptions.md`: InvalidTaskError as primary
- [ ] T083 [P] [US5] Restructure `24-exception-handling/04-custom-exceptions.md`: TaskNotFoundError, InvalidPriorityError

### Chapter 26: DateTime/Calendar (1 lesson)

- [ ] T084 [P] [US5] Restructure `26-math-datetime-calendar/03-calendar-scheduling.md`: Task scheduling as primary

### Chapter 32: CPython/GIL (2 lessons)

- [ ] T085 [P] [US5] Restructure `32-cpython-gil/01-cpython-internals.md`: Add task processing context
- [ ] T086 [P] [US5] Restructure `32-cpython-gil/05-performance-optimization.md`: Task batch processing optimization

**Checkpoint**: All 13 Category D lessons have Todo as primary example

---

## Phase 7: Connection Points & READMEs (US6: P3 Priority)

**Goal**: Add narrative connections and update 18 chapter READMEs

**Independent Test**: Student can trace Todo evolution Ch16 → Ch32

### Connection Points (5 transitions)

- [ ] T087 [US6] Add Ch16 → Ch17 connection: "Now we'll structure our tasks as data"
  - Location: End of `16-introduction-to-python/README.md` or beginning of `17-data-types/01-*.md`

- [ ] T088 [US6] Add Ch21 → Ch23 connection: "You now have task storage. Let's wrap operations in functions"
  - Location: End of `21-lists-tuples-dictionary/README.md` or beginning of `23-module-functions/01-*.md`

- [ ] T089 [US6] Add Ch23 → Ch25 connection: "Your functions manipulate task data. Now persist it to disk"
  - Location: End of `23-module-functions/README.md` or beginning of `25-io-file-handling/01-*.md`

- [ ] T090 [US6] Add Ch25 → Ch27 connection: "File I/O works with functions. Now let's organize everything with classes"
  - Location: End of `25-io-file-handling/README.md` or beginning of `27-oop-part-1/01-*.md`

- [ ] T091 [US6] Add Ch27 → Ch31 connection: "Your TaskManager works synchronously. Part 6 will make it async"
  - Location: End of `27-oop-part-1/README.md` or beginning of `31-asyncio/01-*.md`

### Chapter README Updates (18 files)

- [ ] T092 [P] [US6] Update `15-python-uv-package-manager/README.md`: "This chapter sets up the environment for building your Todo app"
- [ ] T093 [P] [US6] Update `16-introduction-to-python/README.md`: "Chapter contributes: First Python code with Todo context"
- [ ] T094 [P] [US6] Update `17-data-types/README.md`: "Chapter contributes: Task data structure (dict, list)"
- [ ] T095 [P] [US6] Update `18-operators-keywords-variables/README.md`: "Chapter contributes: Task calculations and comparisons"
- [ ] T096 [P] [US6] Update `19-strings-type-casting/README.md`: "Chapter contributes: Task title handling"
- [ ] T097 [P] [US6] Update `20-control-flow-loops/README.md`: "Chapter contributes: Task menu logic and iteration"
- [ ] T098 [P] [US6] Update `21-lists-tuples-dictionary/README.md`: "Chapter contributes: Task storage (list/dict)"
- [ ] T099 [P] [US6] Update `22-set-frozenset-gc/README.md`: "Chapter contributes: Task categories"
- [ ] T100 [P] [US6] Update `23-module-functions/README.md`: "Chapter contributes: Task operation functions"
- [ ] T101 [P] [US6] Update `24-exception-handling/README.md`: "Chapter contributes: Task error handling"
- [ ] T102 [P] [US6] Update `25-io-file-handling/README.md`: "Chapter contributes: Task persistence (JSON)"
- [ ] T103 [P] [US6] Update `26-math-datetime-calendar/README.md`: "Chapter contributes: Task due dates and scheduling"
- [ ] T104 [P] [US6] Update `27-oop-part-1/README.md`: "Chapter contributes: Task and TaskManager classes"
- [ ] T105 [P] [US6] Update `28-oop-part-2/README.md`: "Chapter contributes: Advanced Task class features"
- [ ] T106 [P] [US6] Update `29-metaclasses-dataclasses/README.md`: "Chapter contributes: TodoList dataclass (gold standard)"
- [ ] T107 [P] [US6] Update `30-pydantic-generics/README.md`: "Chapter contributes: Validated Task models"
- [ ] T108 [P] [US6] Update `31-asyncio/README.md`: "Chapter contributes: Async task operations (preview Part 6)"
- [ ] T109 [P] [US6] Update `32-cpython-gil/README.md`: "Chapter contributes: Performance understanding for task batching"

**Checkpoint**: All 18 READMEs updated with Todo connection

---

## Phase 8: Validation & Polish

**Purpose**: Final validation across all changes

- [ ] T110 Run `pnpm nx build learn-app` and verify no build errors
- [ ] T111 Validate all Python examples are syntactically correct (spot-check 10 lessons)
- [ ] T112 Cross-reference Task attributes for consistency across all lessons
- [ ] T113 Run educational-validator on 5 representative lessons (1 per phase)
- [ ] T114 Verify narrative arc: Read Ch16, Ch21, Ch27, Ch32 openings for consistency
- [ ] T115 Create final checklist summary at `/specs/046-part5-todo-integration/checklists/implementation-complete.md`

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Capstones) ← CRITICAL PATH START
    ↓
Phase 3 (OOP Reframes)
    ↓
Phase 4-6 can run in PARALLEL:
├── Phase 4 (Variable Renaming - 32 lessons)
├── Phase 5 (Auxiliary Examples - 29 lessons)
└── Phase 6 (Category D - 13 lessons)
    ↓
Phase 7 (Connections + READMEs) ← MUST BE LAST for narrative consistency
    ↓
Phase 8 (Validation)
```

### User Story Dependencies

- **US1 (OOP Foundation)**: Depends on Phase 2 capstones being complete
- **US2 (Capstones)**: No dependencies - START HERE
- **US3 (Variable Rename)**: Can start after US1 establishes patterns
- **US4 (Auxiliary Examples)**: Can start after US3 establishes naming
- **US5 (Category D)**: Can start after US4 establishes domain patterns
- **US6 (Connections)**: MUST be last to ensure consistent narrative

### Parallel Opportunities

**After Phase 3 completes**, these phases can run in parallel:
- Phase 4: 32 lessons across Ch17-24 (variable renaming)
- Phase 5: 29 lessons adding auxiliary examples
- Phase 6: 13 lessons restructuring primary examples

**Within each phase**, all [P] marked tasks can run in parallel.

---

## Parallel Example: Phase 4 (Variable Renaming)

```bash
# All these can run in parallel (different files):
Task: T016 "Reframe 17-data-types/02-working-with-dictionaries.md"
Task: T020 "Reframe 18-operators-keywords-variables/01-arithmetic-operators.md"
Task: T025 "Reframe 19-strings-type-casting/01-string-fundamentals.md"
Task: T030 "Reframe 20-control-flow-loops/01-if-else.md"
Task: T035 "Reframe 21-lists-tuples-dictionary/01-lists.md"
```

---

## Implementation Strategy

### MVP First (Phases 1-3 Only)

1. Complete Phase 1: Setup references
2. Complete Phase 2: 4 critical capstones
3. Complete Phase 3: 9 OOP lessons
4. **STOP and VALIDATE**: Build book, test examples
5. This gives students functional Task/TaskManager foundation

### Full Implementation

1. MVP (Phases 1-3) → 21 lessons
2. Phase 4 (Variable Renaming) → +32 lessons (53 total)
3. Phase 5 (Auxiliary Examples) → +29 lessons (82 total)
4. Phase 6 (Category D) → +13 lessons (95 total overlaps)
5. Phase 7 (Connections) → +23 items
6. Phase 8 (Validation) → Final checks

---

## Summary

| Phase | Tasks | Lessons | Effort | Priority |
|-------|-------|---------|--------|----------|
| Phase 1 | T001-T003 | 0 | 1h | Setup |
| Phase 2 | T004-T007 | 4 | 8h | P1 (MVP) |
| Phase 3 | T008-T015 | 8 | 16h | P1 |
| Phase 4 | T016-T046 | 31 | 16h | P2 |
| Phase 5 | T047-T075 | 29 | 36h | P2 |
| Phase 6 | T076-T086 | 11 | 12h | P3 |
| Phase 7 | T087-T109 | 23 | 4h | P3 |
| Phase 8 | T110-T115 | 6 | 3h | Final |
| **Total** | **115** | **74 unique** | **96h** | |

**Note**: Some lessons appear in multiple phases (e.g., variable rename + auxiliary examples). Actual unique lesson count is 74.

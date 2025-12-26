# Implementation Plan: Part 5 Todo Running Example Integration

**Branch**: `046-part5-todo-integration` | **Date**: 2025-12-26 | **Spec**: Issue #407
**Source**: Part 5 Audit (Chapters 15-32) for Running Example Integration
**Status**: Planning Phase

---

## Executive Summary

This plan organizes the integration of the Todo running example across Part 5 (Python Fundamentals, Chapters 15-32) into six distinct implementation phases. The running example approach builds a Todo console app incrementally, with each Python concept contributing to a cohesive student project. Auxiliary domain examples (legal, finance, healthcare, marketing) demonstrate pattern universality.

**Key Metrics:**
- **Total Lessons**: 74 lessons across 18 chapters
- **Effort Breakdown**: 96 total implementation hours (varies by phase)
- **Critical Path**: Phases 1 → 2 (must complete before later phases)
- **Parallelization**: Phases 3, 4, 5 can run in parallel after Phase 2 completes

**Phase Breakdown:**
| Phase | Type | Lessons | Hours | Duration |
|-------|------|---------|-------|----------|
| Phase 1 | Critical Capstones | 4 | 8 | 1-2 days |
| Phase 2 | Core OOP Reframes | 9 | 20 | 3-4 days |
| Phase 3 | Quick Wins (Variable Rename) | 32 | 16 | 2-3 days |
| Phase 4 | Auxiliary Examples | 29 | 36 | 5-7 days |
| Phase 5 | Category D Restructuring | 13 | 12 | 2-3 days |
| Phase 6 | Connection Points + READMEs | 5 + 18 | 4 | 1 day |
| **Total** | **All Phases** | **74** | **96** | **2-3 weeks** |

---

## Part 1: Technical Context

**Language/Version**: Python 3.11+
**Primary Concepts**: Core Python fundamentals (types, OOP, async)
**Testing Framework**: pytest (for validating code examples)
**Target Audience**: A2-B1 proficiency (beginner to intermediate)
**Project Scope**: Educational content (18 chapters, 74 lessons)
**Running Example**: Todo Console App (builds incrementally across all chapters)

### Todo App Architecture (Reference from Ch29)

**Gold Standard**: Chapter 29, Lesson 04 - `TodoList` dataclass
**Location**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/29-metaclasses-dataclasses/04-advanced-dataclass-features.md`

```python
from dataclasses import dataclass, field

@dataclass
class TodoList:
    name: str
    items: list[str] = field(default_factory=list)
    tags: dict[str, str] = field(default_factory=dict)
    priority: int = 5
```

---

## Part 2: Phase Breakdown and Implementation Details

### Phase 1: Critical Capstones (4 Lessons) — 8 hours

**Objective**: Rewrite capstone lessons to frame the complete Todo app as the culminating project for each major milestone.

**Lessons to Implement** (in order):

#### 1.1 Chapter 23, Lesson 05: Functions Capstone
- **File Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/23-module-functions/05-capstone.md`
- **Current State**: Generic function practice exercises
- **Change Type**: Major Rewrite
- **Effort**: 2 hours
- **Changes Required**:
  - Refocus capstone on `TaskManager` function interface:
    - `add_task(tasks: list, title: str, priority: str) -> None`
    - `list_tasks(tasks: list) -> str`
    - `complete_task(tasks: list, task_id: int) -> bool`
  - Build student-written functions that operate on simple task list (not yet class-based)
  - Frame as: "You now have the functional foundation for the Todo app. Next chapter, we'll wrap these in a class."
- **Success Criteria**:
  - Students write 3+ functions for task operations
  - Functions manipulate task list correctly
  - Clear transition statement to OOP

#### 1.2 Chapter 25, Lesson 05: File I/O Capstone
- **File Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/25-io-file-handling/05-capstone.md`
- **Current State**: Generic CSV/JSON file reading
- **Change Type**: Major Rewrite
- **Effort**: 2 hours
- **Changes Required**:
  - Capstone: Save and load tasks from JSON file
  - Students write code to:
    - Save task list to `tasks.json` (serialize)
    - Load tasks from `tasks.json` (deserialize)
    - Handle missing file gracefully (exception handling review)
  - Frame as: "Your Todo app now persists. Tasks survive app restarts."
  - Connect to Ch24 exception handling patterns
- **Success Criteria**:
  - Save/load functions work correctly
  - JSON format is clean and readable
  - Exception handling covers file not found

#### 1.3 Chapter 27, Lesson 05: OOP Part 1 Capstone
- **File Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/27-oop-part-1/05-capstone.md`
- **Current State**: Generic Person/Animal class examples
- **Change Type**: Major Rewrite
- **Effort**: 2 hours
- **Changes Required**:
  - Build `Task` class with:
    - `__init__`: id, title, done, priority, created_at
    - `__repr__`: readable task display
    - `__eq__`: compare tasks by id
    - Methods: `mark_complete()`, `update_priority()`
  - Build `TaskManager` class with:
    - `__init__`: tasks list
    - `add_task()`, `list_tasks()`, `get_task()`
  - Integrate with Ch25 file I/O: `TaskManager.save()`, `TaskManager.load()`
  - Frame as: "Objects organize your task data and operations. Your app is now properly structured."
- **Success Criteria**:
  - Task and TaskManager classes work together
  - Can save/load via JSON
  - All methods tested with example usage

#### 1.4 Chapter 32, Lesson 06: CPython/GIL Capstone
- **File Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/32-cpython-gil/06-capstone.md`
- **Current State**: Generic performance examples
- **Change Type**: Rewrite with Todo Focus
- **Effort**: 2 hours
- **Changes Required**:
  - Capstone: Batch operations on todo list (preview async for Part 6)
  - Demonstrate:
    - Single-threaded task processing (baseline)
    - Multi-threaded vs GIL limitations
    - Why asyncio (Ch31) is better for I/O
  - Frame as: "Understanding GIL prepares you for Part 6 async agents that process many tasks."
  - Connect backward to Ch31 asyncio
- **Success Criteria**:
  - Benchmark code shows GIL limitation
  - Clear explanation of when threading fails
  - Motivation for asyncio introduced

**Phase 1 Dependencies**: None (parallelizable)
**Phase 1 Validation**:
- All capstones have clear Todo framing
- Student code in examples is syntactically correct
- Backward and forward connections explicitly stated

---

### Phase 2: Core OOP Reframes (9 Lessons) — 20 hours

**Objective**: Reframe all OOP lessons (Ch27-28) to use Todo Task/TaskManager as primary example, moving generic Person/Animal examples to auxiliary.

**Chapters to Reframe**: Ch27 (OOP Part 1) and Ch28 (OOP Part 2)

#### 2.1 Chapter 27 Lessons (OOP Part 1)
- **Total Lessons**: 5 lessons + 1 quiz
- **Effort**: 12 hours (2.4 hours/lesson avg)

**Lesson 2.1a: Chapter 27, Lesson 01 - Classes and Objects**
- **File**: `27-oop-part-1/01-classes-and-objects.md`
- **Change Type**: Primary Example Replacement
- **Current**: Generic Person class
- **Action**:
  - Replace primary example with Task class
  - Keep Person/Animal as auxiliary examples (secondary)
  - Frame: "A Task object represents one todo item"
  - Add 3 auxiliary examples: Legal (Case class), Finance (Invoice class), Healthcare (Appointment class)
  - Estimated Effort: 2 hours

**Lesson 2.1b: Chapter 27, Lesson 02 - Attributes and Methods**
- **File**: `27-oop-part-1/02-attributes-and-methods.md`
- **Change Type**: Primary Example Replacement + Auxiliary Examples
- **Current**: Generic shapes
- **Action**:
  - Use Task methods: `mark_complete()`, `update_priority()`, `get_age()`
  - Add auxiliary examples showing same pattern in legal/finance/healthcare domains
  - Estimated Effort: 2 hours

**Lesson 2.1c: Chapter 27, Lesson 03 - Constructors and Initialization**
- **File**: `27-oop-part-1/03-constructors-initialization.md`
- **Change Type**: Primary Example Replacement
- **Action**:
  - Task.__init__ with proper defaults
  - Show how each domain initializes differently
  - Estimated Effort: 2 hours

**Lesson 2.1d: Chapter 27, Lesson 04 - Inheritance**
- **File**: `27-oop-part-1/04-inheritance.md`
- **Change Type**: Primary Example Replacement
- **Current**: Animal/Dog inheritance
- **Action**:
  - Task inheritance (unlikely, but if shown, use SpecialTask)
  - More important: TaskManager inheritance pattern
  - Auxiliary: Case/LegalCase, Invoice/PaidInvoice, etc.
  - Estimated Effort: 2 hours

**Lesson 2.1e: Chapter 27, Lesson 05 - Capstone** (Already in Phase 1)

#### 2.2 Chapter 28 Lessons (OOP Part 2)
- **Total Lessons**: 4 lessons (28-01 through 28-04)
- **Effort**: 8 hours (2 hours/lesson avg)

**Lesson 2.2a: Chapter 28, Lesson 01 - Encapsulation and Property**
- **File**: `28-oop-part-2/01-encapsulation-property.md`
- **Change Type**: Primary Example Replacement
- **Current**: Generic Bank Account
- **Action**:
  - Use Task: private `_priority`, public `priority` property with setter
  - Show validation: `task.priority = "urgent"` vs `task.priority = "xyz"` (invalid)
  - Auxiliary examples for each domain
  - Estimated Effort: 2 hours

**Lesson 2.2b: Chapter 28, Lesson 02 - Class Methods and Static Methods**
- **File**: `28-oop-part-2/02-class-methods-static-methods.md`
- **Change Type**: Primary Example Replacement
- **Action**:
  - Task.create_from_dict() as class method
  - Task.priority_levels() as static method
  - Auxiliary patterns in legal/finance
  - Estimated Effort: 2 hours

**Lesson 2.2c: Chapter 28, Lesson 03 - Polymorphism and Dunder Methods**
- **File**: `28-oop-part-2/03-polymorphism-dunder-methods.md`
- **Change Type**: Primary Example Replacement
- **Current**: Polygon area calculations
- **Action**:
  - Task.__str__, __repr__, __eq__, __lt__ (for sorting tasks)
  - Show how same dunder method works across domains (polymorphism)
  - Estimated Effort: 2 hours

**Lesson 2.2d: Chapter 28, Lesson 04 - Abstract Classes and Interfaces**
- **File**: `28-oop-part-2/04-abstract-classes-interfaces.md`
- **Change Type**: Primary Example Replacement
- **Action**:
  - AbstractTask base (if needed) or more likely: TaskStorage interface
  - FileTaskStorage, DatabaseTaskStorage (abstract pattern)
  - Estimated Effort: 2 hours

**Phase 2 Dependencies**: Phase 1 should be complete (so students understand full Todo app architecture)
**Phase 2 Validation**:
- Todo examples are primary (appear first) in all lessons
- Auxiliary examples show pattern universality
- No contradictions between lessons (e.g., Task attributes consistent)
- Students can implement Todo patterns from these lessons

---

### Phase 3: Quick Wins — Variable Renaming (32 Lessons) — 16 hours

**Objective**: Minor reframes (Category B) where lessons mention generic variable names but don't change fundamentally. Rename to Todo-aligned names and add 1-2 connecting sentences.

**Scope**: 32 lessons across multiple chapters where variable naming can align with Todo context

#### 3.1 Strategy
- Minimal code changes (variable renames only)
- Add 1-2 lines of narrative connecting to Todo app
- No pedagogical changes
- Examples: `items → tasks`, `data → todo_data`, `person_list → contact_list` (if not yet renamed)

#### 3.2 Lessons (Representative Examples)

**Chapter 17: Data Types** (4 lessons)
- 17-02: "Working with Dictionaries" → Rename `person` to `task`, add "This becomes the foundation of our Todo data model"
- 17-03: "Lists" → Rename `numbers` to `tasks`, explain task storage
- Effort per lesson: 0.5 hours

**Chapter 18: Operators/Keywords/Variables** (5 lessons)
- 18-01: "Arithmetic Operators" → Show task ID calculations
- 18-02: "Comparison Operators" → Task priority comparison
- 18-03: "Logical Operators" → Task filtering (done AND high-priority)
- Effort per lesson: 0.5 hours

**Chapter 19: Strings/Type Casting** (5 lessons)
- 19-01: "String Fundamentals" → Rename to show task title formatting
- 19-02: "String Methods" → Task title validation, search in task list
- Effort per lesson: 0.5 hours

**Chapter 20: Control Flow/Loops** (5 lessons)
- 20-01: "If/Else Statements" → Menu loop logic (select command)
- 20-02: "While Loops" → Task menu loop
- 20-03: "For Loops" → List all tasks
- Effort per lesson: 0.5 hours

**Chapter 21: Lists/Tuples/Dictionaries** (6 lessons)
- 21-01: "Lists" → Task list operations
- 21-02: "Dictionaries" → Task lookup by ID
- Effort per lesson: 0.5 hours

**Chapter 22: Sets/Frozenset/GC** (2 lessons)
- 22-01: "Sets" → Task categories as set (no duplicates)
- Effort per lesson: 0.5 hours

**Chapter 24: Exception Handling** (5 lessons)
- 24-01: "Exception Fundamentals" → Invalid task ID handling
- 24-02: "Except/Else/Finally" → File I/O for tasks
- Effort per lesson: 0.5 hours

**Phase 3 Dependencies**: Phase 2 should be complete (OOP foundation established)
**Phase 3 Parallelization**: All lessons can be done in parallel
**Phase 3 Validation**:
- Variable names are consistent across chapter (e.g., `tasks` used in all Ch20 lessons)
- No contradictions introduced
- Narrative connections are 1-2 lines (not disruptive)

---

### Phase 4: Auxiliary Domain Examples (29 Lessons) — 36 hours

**Objective**: Add 2-3 domain examples (Legal, Finance, Healthcare, Marketing) to lessons that have Todo but lack auxiliary examples.

**Scope**: 29 lessons that need auxiliary examples added (typically after main Todo example)

#### 4.1 Domain Mappings

| Domain | Primary Entity | Relevant Chapters |
|--------|---|---|
| **Legal** | Case/Contract | Ch17-21, Ch23-25, Ch27-30 |
| **Finance** | Invoice/Transaction | Ch17-21, Ch23-25, Ch27-30 |
| **Healthcare** | Appointment/Patient | Ch17-21, Ch23-25, Ch27-30 |
| **Marketing** | Campaign/Lead | Ch17-21, Ch23-25, Ch27-30 |

#### 4.2 Phase 4 Lessons (Representative)

**Chapter 21: Lists/Tuples/Dictionaries** (6 lessons requiring auxiliary examples)
- 21-01: Task list → Add case_list (Legal), invoice_list (Finance), appointment_list (Healthcare)
- 21-02: Task lookup → Show equivalent in legal/finance/healthcare domains
- 21-03: Dictionary operations → Case metadata, invoice details, etc.
- Effort per lesson: 1.5 hours

**Chapter 23: Functions** (5 lessons)
- 23-01: "Function Basics" → add_task(), add_case(), add_invoice(), etc.
- 23-02: "Parameters and Arguments" → Show consistency across domains
- 23-03: "Return Values" → Each domain returns same pattern
- Effort per lesson: 1.5 hours

**Chapter 25: File I/O** (4 lessons)
- 25-01: "Console I/O" → Input task, case, invoice, appointment
- 25-02: "File I/O" → Save/load tasks, cases, invoices, appointments
- 25-04: "CSV/JSON" → Each domain serializes to JSON consistently
- Effort per lesson: 1.5 hours

**Chapter 26: DateTime** (3 lessons)
- 26-01: DateTime basics → Task due date, case deadline, appointment time
- 26-02: DateTime methods → Calculate days until due, overdue detection
- Effort per lesson: 1.5 hours

**Chapter 29: Dataclasses** (5 lessons)
- 29-03: Intro to dataclasses → Task as @dataclass (primary), plus Case, Invoice, Appointment
- 29-04: Advanced dataclass features → Validation, __post_init__, serialization across domains
- Effort per lesson: 2 hours

**Chapter 30: Pydantic** (3 lessons)
- 30-01: "Pydantic Basics" → ValidatedTask, Case, Invoice models
- Effort per lesson: 2 hours

**Chapter 31: Asyncio** (2 lessons)
- 31-01: "Async Basics" → Async task operations, case lookups, etc.
- Effort per lesson: 2 hours

**Phase 4 Dependencies**: Phase 3 should be complete (variable naming consistent)
**Phase 4 Parallelization**: All lessons can be done in parallel (independent examples)
**Phase 4 Validation**:
- Each domain example follows same pattern as Todo (consistency)
- Auxiliary examples don't distract from main Python concept
- Students see universal applicability of pattern

---

### Phase 5: Category D Restructuring (13 Lessons) — 12 hours

**Objective**: Add running Todo example where lessons currently use unrelated examples (Category D: requires primary restructuring).

**Scope**: 13 lessons that currently lack any Todo framing and need substantial restructuring

#### 5.1 Lessons Requiring Restructuring (Representative)

**Chapter 16: Introduction to Python** (2 lessons)
- 16-01: Hello World → "Hello, Todo!"
- 16-02: Python Basics → First task creation
- Effort per lesson: 1 hour

**Chapter 19: Strings/Type Casting** (3 lessons)
- 19-01: Type casting → Cast task priority to int, title to string
- Effort per lesson: 1 hour

**Chapter 20: Control Flow** (1 lesson)
- 20-04: "Break/Continue" → Task filtering with break/continue
- Effort per lesson: 1 hour

**Chapter 22: Sets** (2 lessons)
- 22-02: "Frozenset" → Immutable task categories
- Effort per lesson: 0.75 hours

**Chapter 24: Exception Handling** (2 lessons)
- 24-03: "Raising Custom Exceptions" → InvalidTaskError, TaskNotFoundError
- Effort per lesson: 1 hour

**Chapter 26: DateTime** (1 lesson)
- 26-03: "Calendar/Scheduling" → Task scheduling
- Effort per lesson: 1 hour

**Chapter 32: CPython/GIL** (2 lessons)
- 32-01 through 32-05 (lessons 1-5) → Batch task processing (already Ch32-06 in Phase 1)
- Effort per lesson: 0.5 hours

**Phase 5 Dependencies**: Phase 4 should be complete (patterns established)
**Phase 5 Parallelization**: All lessons can be done in parallel
**Phase 5 Validation**:
- No existing content removed (add examples, don't replace)
- Todo example integrated naturally (not forced)
- Pedagogical explanation unchanged

---

### Phase 6: Connection Points and README Updates (23 items) — 4 hours

**Objective**: Add narrative connections between chapters and update README files to emphasize running example progression.

#### 6.1 Connection Points (5 Lessons) — 2 hours

**Inter-Chapter Transition Sentences**:

1. **Ch16 → Ch17** (Intro → Data Types)
   - "Now we'll structure our tasks as data: task titles are strings, completion status is boolean"
   - Location: End of Ch16 README or beginning of Ch17-01
   - Effort: 0.25 hours

2. **Ch21 → Ch23** (Collections → Functions)
   - "You now have task storage (list/dict). Let's wrap operations in functions for reusability"
   - Location: End of Ch21 README or beginning of Ch23-01
   - Effort: 0.25 hours

3. **Ch23 → Ch25** (Functions → File I/O)
   - "Your functions manipulate task data. Now persist it to disk"
   - Location: End of Ch23 README or beginning of Ch25-01
   - Effort: 0.25 hours

4. **Ch25 → Ch27** (File I/O → OOP)
   - "File I/O works with functions. Now let's organize everything with classes"
   - Location: End of Ch25 README or beginning of Ch27-01
   - Effort: 0.25 hours

5. **Ch27 → Ch31** (OOP → Asyncio)
   - "Your TaskManager works synchronously. Part 6 will make it async for agent coordination"
   - Location: End of Ch27 README or beginning of Ch31-01
   - Effort: 0.5 hours

#### 6.2 Chapter README Updates (18 Files) — 2 hours

**Update Each Chapter README** with:
1. **Running Example Connection** (1-2 sentences):
   - What Todo component does this chapter build?
   - How does it connect to prior chapters?

2. **Auxiliary Examples Callout** (optional):
   - Which domains are demonstrated in this chapter?

3. **Learning Progression Map** (optional):
   - Visual: `Ch16 → Ch17 → Ch21 → Ch23 → Ch25 → Ch27`

**Chapters with README files to update** (18 total):
- 15-python-uv-package-manager/README.md
- 16-introduction-to-python/README.md
- 17-data-types/README.md
- 18-operators-keywords-variables/README.md
- 19-strings-type-casting/README.md
- 20-control-flow-loops/README.md
- 21-lists-tuples-dictionary/README.md
- 22-set-frozenset-gc/README.md
- 23-module-functions/README.md
- 24-exception-handling/README.md
- 25-io-file-handling/README.md
- 26-math-datetime-calendar/README.md
- 27-oop-part-1/README.md
- 28-oop-part-2/README.md
- 29-metaclasses-dataclasses/README.md
- 30-pydantic-generics/README.md
- 31-asyncio/README.md
- 32-cpython-gil/README.md

**Effort**: ~7 minutes per README = 2 hours total

**Phase 6 Dependencies**: All prior phases complete
**Phase 6 Parallelization**: All README updates can be done in parallel
**Phase 6 Validation**:
- Each chapter has explicit running example connection
- Connection points create visible throughline (Ch16...Ch32)
- Student can trace Todo evolution

---

## Part 3: Implementation Workflow

### Critical Path Analysis

```
Phase 1 (Capstones)
    ↓
Phase 2 (OOP Reframes)
    ↓
Phase 3 (Variable Rename) ─→ Phase 4 (Auxiliary Examples)
                          ↘
                            Phase 5 (Category D)
                                ↓
                            Phase 6 (Connections + READMEs)
```

**Critical Path Duration**:
- Phase 1: 1-2 days (4 lessons, 8 hours)
- Phase 2: 3-4 days (9 lessons, 20 hours) — **Cannot start until Phase 1 done**
- Phase 3-5: Parallel tracks, 5-7 days total
  - Phase 3: 2-3 days (32 lessons, 16 hours)
  - Phase 4: 5-7 days (29 lessons, 36 hours) — **Longest track**
  - Phase 5: 2-3 days (13 lessons, 12 hours)
- Phase 6: 1 day (23 items, 4 hours) — **Must be last**

**Minimum Total Duration**: 14 days (2 weeks) with optimal parallelization

### Parallelization Strategy

**Weeks 1-2:**
- Week 1 (Days 1-2): Phase 1 (4 lessons, 1 person)
- Week 1 (Days 3-4): Phase 2 (9 lessons, 1 person working sequentially)
- Week 1 (Days 3-7): Parallel tracks:
  - Track A (Days 3-5): Phase 3 (32 lessons, 1 person, 16 hours)
  - Track B (Days 3-7): Phase 4 (29 lessons, 1 person, 36 hours) — **Longest**
  - Track C (Days 5-7): Phase 5 (13 lessons, 1 person, 12 hours)
- Week 2 (Day 1): Phase 6 (23 items, 1 person, 4 hours)

**With 2 People**: ~10 days (14 days ÷ 1.4 efficiency ratio)
**With 3+ People**: ~8-10 days (subject to communication overhead)

### Per-Phase Quality Checklist

**Phase 1 Capstones**:
- [ ] Each capstone has clear "Build the Todo app" framing
- [ ] Code examples are syntactically correct (tested)
- [ ] Transition statements connect to next chapter
- [ ] All 4 capstones work together (sequential progression)

**Phase 2 OOP Reframes**:
- [ ] Task and TaskManager are primary examples
- [ ] Generic examples moved to auxiliary (secondary)
- [ ] 3+ domain examples per lesson (Legal, Finance, Healthcare minimum)
- [ ] No contradictions in Task attributes (consistent across Ch27-28)

**Phase 3 Variable Renaming**:
- [ ] Variable names consistent across chapter (e.g., `tasks` everywhere in Ch20)
- [ ] Connection sentences are 1-2 lines max
- [ ] No pedagogical changes (only variable names + 1-2 sentences)

**Phase 4 Auxiliary Examples**:
- [ ] Each domain example uses same pattern as Todo
- [ ] Auxiliary examples don't distract (appear after primary Todo)
- [ ] 2-3 domains per lesson minimum
- [ ] Domain examples are complete and runnable

**Phase 5 Category D Restructuring**:
- [ ] Running example integrated naturally (not forced)
- [ ] No existing content removed
- [ ] Pedagogical explanation of Python concept unchanged

**Phase 6 Connections and READMEs**:
- [ ] Every README mentions running example
- [ ] Connection points create visible narrative arc
- [ ] README updates are brief (1-2 paragraphs max)

---

## Part 4: Lesson-by-Lesson Reference

### Phase 1: Capstone Lessons (4 lessons)

| Lesson | File Path | Current State | Changes | Effort |
|--------|-----------|---------------|---------|--------|
| Ch23-05 | 23-module-functions/05-capstone.md | Generic function practice | Rewrite as TaskManager function interface | 2h |
| Ch25-05 | 25-io-file-handling/05-capstone.md | Generic CSV/JSON | Rewrite as task save/load | 2h |
| Ch27-05 | 27-oop-part-1/05-capstone.md | Generic classes | Rewrite as Task/TaskManager OOP | 2h |
| Ch32-06 | 32-cpython-gil/06-capstone.md | Generic performance | Rewrite as batch task processing | 2h |

### Phase 2: OOP Reframes (9 lessons)

| Lesson | File Path | Primary Example | Auxiliary Domains | Effort |
|--------|-----------|-----------------|-------------------|--------|
| Ch27-01 | 27-oop-part-1/01-classes-and-objects.md | Person → Task | Legal (Case), Finance (Invoice), Healthcare (Appointment) | 2h |
| Ch27-02 | 27-oop-part-1/02-attributes-and-methods.md | Shape → Task | Same 3 domains | 2h |
| Ch27-03 | 27-oop-part-1/03-constructors-initialization.md | Generic | Task init | Same 3 domains | 2h |
| Ch27-04 | 27-oop-part-1/04-inheritance.md | Animal/Dog → Task hierarchy | Domain hierarchies | 2h |
| Ch28-01 | 28-oop-part-2/01-encapsulation-property.md | Bank Account → Task | Domain properties | 2h |
| Ch28-02 | 28-oop-part-2/02-class-methods-static-methods.md | Generic | Task class methods | Domain patterns | 2h |
| Ch28-03 | 28-oop-part-2/03-polymorphism-dunder-methods.md | Polygon area → Task methods | Domain dunder methods | 2h |
| Ch28-04 | 28-oop-part-2/04-abstract-classes-interfaces.md | Generic | TaskStorage abstraction | Domain storage patterns | 2h |

### Phase 3: Quick Wins (32 lessons) — Variable Renaming

| Chapter | Lessons | Change Type | Effort |
|---------|---------|-------------|--------|
| Ch17 | 4 lessons | Rename data structures to task-oriented | 2h (0.5h each) |
| Ch18 | 5 lessons | Rename to show task operations | 2.5h |
| Ch19 | 5 lessons | Rename strings to task titles | 2.5h |
| Ch20 | 5 lessons | Rename to show task loop logic | 2.5h |
| Ch21 | 6 lessons | Rename to task list/dict operations | 3h |
| Ch22 | 2 lessons | Rename to task categories | 1h |
| Ch24 | 4 lessons | Rename to task error handling | 2h |

**Total Phase 3**: 32 lessons, 16 hours

### Phase 4: Auxiliary Examples (29 lessons)

| Chapter | Lessons | Domains Added | Effort |
|---------|---------|---|--------|
| Ch17 | 3 lessons | Legal, Finance, Healthcare | 4.5h |
| Ch18 | 3 lessons | Same 3 domains | 4.5h |
| Ch19 | 3 lessons | Same 3 domains | 4.5h |
| Ch20 | 2 lessons | Same 3 domains | 3h |
| Ch21 | 6 lessons | Same 3 domains | 9h |
| Ch23 | 5 lessons | Same 3 domains | 7.5h |
| Ch25 | 4 lessons | Same 3 domains | 6h |
| Ch26 | 3 lessons | Same 3 domains | 4.5h |

**Total Phase 4**: 29 lessons, 36 hours (longest phase)

### Phase 5: Category D Restructuring (13 lessons)

| Chapter | Lessons | Current → Todo Reframe | Effort |
|---------|---------|-------|--------|
| Ch16 | 2 | Hello World → Todo basics | 2h |
| Ch19 | 3 | Type casting → Task data | 3h |
| Ch20 | 1 | Break/Continue → Task filtering | 1h |
| Ch22 | 2 | Frozenset → Task categories | 1.5h |
| Ch24 | 2 | Custom exceptions → Task errors | 2h |
| Ch26 | 1 | Calendar → Task scheduling | 1h |
| Ch32 | 2 | Batch processing → Task batching | 1h |

**Total Phase 5**: 13 lessons, 12 hours

### Phase 6: Connection Points and READMEs (23 items)

| Type | Items | Effort |
|------|-------|--------|
| Inter-chapter connections | 5 transition points | 1.25h |
| Chapter README updates | 18 README files | 2h |
| Quick wins + spot checks | Various | 0.75h |

**Total Phase 6**: 23 items, 4 hours

---

## Part 5: Risk Mitigation and Validation

### Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| **Inconsistent Task naming** | Students confused by `task` vs `todo` vs `item` | Establish naming convention in Phase 1; validate in Phase 2 |
| **Auxiliary examples distract** | Students focus on domain instead of Python concept | Keep auxiliary examples brief; place after primary example |
| **Breaking existing content** | Chapter breaks during update | Read full chapter before editing; test examples |
| **Schedule overruns in Phase 4** | Longest phase delays entire project | Start Phase 4 early (parallel to Phase 3); consider 2-person team |
| **READMEs feel disconnected** | Narrative arc not visible to students | Use consistent framing: "...contributes to Todo app" |

### Validation Checkpoints

**After Phase 1**:
- [ ] All 4 capstones build on each other (Ch23 functions → Ch25 file I/O → Ch27 OOP → Ch32 perf)
- [ ] Code examples compile and run
- [ ] Transition statements are present

**After Phase 2**:
- [ ] Task/TaskManager consistent across Ch27-28 (no attribute changes)
- [ ] Auxiliary examples use same pattern (not contradictory)
- [ ] OOP principles still clear (not obscured by Todo framing)

**After Phase 3**:
- [ ] Variable names consistent within each chapter
- [ ] No examples broken by variable renames

**After Phase 4**:
- [ ] Auxiliary examples follow domain logic (not generic)
- [ ] Each domain example is runnable Python code

**After Phase 5**:
- [ ] Running examples feel natural (not forced)
- [ ] Python concepts still the primary focus

**After Phase 6**:
- [ ] Student can trace Todo progression: Ch16 → Ch17 → ... → Ch32
- [ ] Each README mentions running example

### Regression Testing

**What to verify** (before final commit):
1. Build the book: `pnpm nx build learn-app` — No errors
2. Syntax check all Python examples: Run as standalone scripts
3. Cross-reference check: Task attributes consistent across lessons
4. Read-through: 2-3 lessons per phase to catch tone/pedagogy issues

---

## Part 6: Effort Estimation Breakdown

### By Phase

| Phase | Lessons | Hours | Days (8h/day) | Team Size |
|-------|---------|-------|---|---|
| Phase 1 | 4 | 8 | 1 | 1 person |
| Phase 2 | 9 | 20 | 2.5 | 1 person |
| Phase 3 | 32 | 16 | 2 | 2 people (parallel) |
| Phase 4 | 29 | 36 | 4.5 | 2 people (parallel) |
| Phase 5 | 13 | 12 | 1.5 | 2 people (parallel) |
| Phase 6 | 23 | 4 | 0.5 | 1 person |
| **TOTAL** | **74** | **96** | **12 days** | **1-2 people** |

### Realistic Schedule

**With 1 person** (sequential):
- Phase 1: Days 1-2
- Phase 2: Days 3-6
- Phase 3: Days 7-9
- Phase 4: Days 10-14
- Phase 5: Days 15-17
- Phase 6: Day 18
- **Total: 18 days (3.6 weeks)**

**With 2 people** (parallelized):
- Weeks 1: Phase 1 + Phase 2 (person A) + Phase 3 (person B)
- Week 2: Phase 4 + Phase 5 (parallel)
- Week 2 (end): Phase 6
- **Total: 10 days (2 weeks)**

**With 3+ people**: 8-10 days (subject to communication overhead)

---

## Part 7: Success Criteria

The implementation is complete when:

1. **Phase 1 Capstones** (4 lessons):
   - Each capstone focuses on building the Todo app
   - Code examples are correct and runnable
   - Clear connections between capstones

2. **Phase 2 OOP Reframes** (9 lessons):
   - Task and TaskManager are primary examples
   - 3+ auxiliary domain examples per lesson
   - No contradictions in Task structure

3. **Phase 3 Quick Wins** (32 lessons):
   - Variable names consistent within chapters
   - 1-2 sentence connections to Todo
   - No content removed

4. **Phase 4 Auxiliary Examples** (29 lessons):
   - Each domain example is correct and runnable
   - Auxiliary examples don't distract from core Python concept
   - Pattern consistency across domains

5. **Phase 5 Restructuring** (13 lessons):
   - Running examples integrated naturally
   - Pedagogical explanation of Python unchanged
   - No existing content removed

6. **Phase 6 Connections** (23 items):
   - Students can trace Todo progression across all chapters
   - Each chapter explicitly mentions its contribution
   - Narrative arc is clear and cohesive

**Final Validation**:
- [ ] Book builds without errors: `pnpm nx build learn-app`
- [ ] All Python examples are syntactically correct
- [ ] Students can follow Todo app evolution across Part 5
- [ ] Part 6 integration points are clear (bridge to TaskManager Agent)

---

## Appendix: Reference Materials

### Gold Standard Example
**Location**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/29-metaclasses-dataclasses/04-advanced-dataclass-features.md`

**TodoList Dataclass Reference**:
```python
from dataclasses import dataclass, field

@dataclass
class TodoList:
    name: str
    items: list[str] = field(default_factory=list)
    tags: dict[str, str] = field(default_factory=dict)
    priority: int = 5
```

### Quality Reference Lesson
**Location**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`

Use this for:
- YAML frontmatter structure
- Narrative opening (real-world hook)
- Code example depth
- "Try With AI" section quality
- Auxiliary example formatting

### File Paths (All Chapters)
```
/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/

15-python-uv-package-manager/
16-introduction-to-python/
17-data-types/
18-operators-keywords-variables/
19-strings-type-casting/
20-control-flow-loops/
21-lists-tuples-dictionary/
22-set-frozenset-gc/
23-module-functions/
24-exception-handling/
25-io-file-handling/
26-math-datetime-calendar/
27-oop-part-1/
28-oop-part-2/
29-metaclasses-dataclasses/
30-pydantic-generics/
31-asyncio/
32-cpython-gil/
```

### Domain-to-Entity Mapping (for Phases 2, 4)

**Legal Domain**:
- Entity: `Case` (or `LegalCase`)
- Attributes: `case_id`, `status` (open/closed/resolved), `filed_date`, `parties`
- Operations: `open_case()`, `close_case()`, `get_case_age()`

**Finance Domain**:
- Entity: `Invoice` (or `Transaction`)
- Attributes: `invoice_no`, `amount`, `paid`, `due_date`, `vendor`
- Operations: `mark_paid()`, `send_reminder()`, `calculate_overdue()`

**Healthcare Domain**:
- Entity: `Appointment` (or `PatientRecord`)
- Attributes: `appointment_id`, `patient_name`, `date_time`, `doctor`, `status`
- Operations: `schedule()`, `cancel()`, `get_reminders()`

**Marketing Domain**:
- Entity: `Campaign` (or `Lead`)
- Attributes: `campaign_id`, `name`, `status` (active/paused/completed), `budget`
- Operations: `launch()`, `pause()`, `calculate_roi()`

---

## Conclusion

This implementation plan organizes the Part 5 Todo running example integration into six manageable phases, with clear dependencies, parallelization opportunities, and concrete success criteria. The critical path runs through Phases 1 → 2, with Phases 3-5 parallelizable. Estimated completion: 10-18 days depending on team size and parallelization strategy.

**Next Steps**:
1. Assign team members to phases (recommend 2-person team for parallel execution)
2. Start Phase 1 immediately (capstones are foundational)
3. Begin Phase 2 after Phase 1 (OOP foundation required)
4. Parallel execution of Phases 3-5 while Phase 2 completes
5. Final validation and Phase 6 (connections/READMEs)

---

**Generated by**: chapter-planner v2.0.0
**Date**: 2025-12-26
**Status**: Ready for Implementation

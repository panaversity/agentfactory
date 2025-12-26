# Feature Specification: Part 5 Todo Running Example Integration

**Feature Branch**: `046-part5-todo-integration`
**Created**: 2025-12-26
**Status**: Draft
**Input**: Implement Todo Running Example Integration across Part 5 Python Fundamentals (Chapters 15-32) based on completed audit

**Related Issues**: #406 (Parent), #407 (Part 5 Spec), #411 (Audit Guidelines)
**Source Audit**: `specs/audit-part5-todo-integration.md`

---

## Assumed Knowledge

**What students know BEFORE Part 5**:
- Parts 1-4 concepts: AI-First Development paradigm, Agent Factory thesis, AI collaboration patterns
- No prior Python programming required (Part 5 is Python fundamentals)
- Understanding of what agents are and why we build them

**What Part 5 must explain from scratch**:
- Python language fundamentals (syntax, data types, control flow)
- Object-oriented programming concepts
- File I/O, exception handling, async programming
- Modern Python tooling (UV, Ruff, Pyright)

**What Part 5 builds toward (Part 6)**:
- TaskManager Agent that manages the Todo app built in Part 5
- The Todo console app IS the domain logic for Part 6's agent

**Proficiency Level**: A2 → B2 (progression across 18 chapters)

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Todo App Foundation in OOP Chapters (Priority: P1)

A student working through Chapter 27 (OOP Part 1) learns class design by building a `Task` class instead of the current `Dog` class. Each OOP concept (constructors, encapsulation, methods) directly applies to their evolving Todo application. By chapter end, they have a working Task class that becomes the foundation for Part 6's TaskManager Agent.

**Why this priority**: Chapter 27 has 4/5 lessons using non-Todo examples (Dog, BankAccount). These core OOP concepts are THE foundation for Part 6's agent. Without Task-based examples, students lack the domain model for their agent.

**Independent Test**: A student completing only Ch27 has a functional `Task` class with: `__init__`, properties, status validation, and methods like `complete()`. This class works standalone and can store/manipulate tasks.

**Acceptance Scenarios**:

1. **Given** a student reads Ch27-02 (Classes and Objects), **When** they see the primary example, **Then** it demonstrates `Task(title, description, due_date)` not `Dog(name, breed, age)`
2. **Given** a student completes Ch27-04 (Encapsulation), **When** they build the exercise, **Then** they create Task status validation with `@property` decorators
3. **Given** a student finishes Ch27 capstone, **When** they run their code, **Then** they have a working Task class with at least 5 methods

---

### User Story 2 - Critical Capstone Rewrites (Priority: P1)

A student completing a chapter capstone builds a component of their Todo console app, not an unrelated application. The Calculator Utility (Ch23-05) becomes "Todo Console App", the generic I/O capstone (Ch25-05) becomes "Todo Data Manager", and so on.

**Why this priority**: Capstones are integration points where all chapter concepts combine. If capstones use unrelated examples, students lose the connection between concepts and their growing Todo app.

**Independent Test**: Each capstone can be tested by running the resulting Todo component and verifying it performs the specified task (add tasks, save to JSON, etc.)

**Acceptance Scenarios**:

1. **Given** Ch23-05 (Functions capstone), **When** a student completes it, **Then** they have `add_task()`, `complete_task()`, `list_tasks()` functions in a working console app
2. **Given** Ch25-05 (I/O capstone), **When** a student completes it, **Then** they can save/load their todo list from a JSON file
3. **Given** Ch32-06 (CPython capstone), **When** a student completes it, **Then** they understand how their Todo app performs under concurrent load

---

### User Story 3 - Category B Minor Reframes (Priority: P2)

A student working through early chapters encounters task-related variable names and examples rather than generic placeholders. Instead of `name = "Alice"`, they see `task_title = "Review contract"`. The Python concepts remain identical, but the context builds familiarity with the Todo domain.

**Why this priority**: These are quick wins (32 lessons) requiring only variable renaming and 1-2 context sentences. Low effort, high consistency impact.

**Independent Test**: Spot-check any Category B lesson and verify primary examples use task-related naming (`task`, `tasks`, `priority`, `done`, `due_date`).

**Acceptance Scenarios**:

1. **Given** Ch16-03 (Variables), **When** reviewing examples, **Then** at least one example uses `task_title: str` or `is_done: bool`
2. **Given** Ch21-07 (Dict Basics), **When** reviewing examples, **Then** primary dict is `task = {"title": ..., "status": ...}` not `person = {"name": ...}`
3. **Given** any Category B lesson, **When** adding Todo context, **Then** existing explanations remain unchanged (only examples modified)

---

### User Story 4 - Auxiliary Domain Examples (Priority: P2)

A student learns that Python patterns apply universally by seeing 2-3 domain examples per lesson (Legal, Finance, Healthcare, Marketing) alongside the Todo primary example. This reinforces that they're learning transferable skills, not just "how to build a todo app."

**Why this priority**: Demonstrates pattern universality. Prevents tunnel vision on single domain. Prepares students for diverse agent projects in their careers.

**Independent Test**: Each lesson with auxiliary examples shows the same Python pattern applied to at least 2 different business domains.

**Acceptance Scenarios**:

1. **Given** Ch21-08 (Dict CRUD), **When** reviewing examples, **Then** Todo CRUD is primary, plus Invoice CRUD (finance) and Case CRUD (legal) as auxiliaries
2. **Given** any lesson with auxiliary examples, **When** comparing domains, **Then** the Python pattern is identical, only entity names differ
3. **Given** auxiliary domains, **When** checking consistency, **Then** Legal=Case/Contract, Finance=Invoice/Transaction, Healthcare=Appointment, Marketing=Campaign

---

### User Story 5 - Category D Primary Example Restructuring (Priority: P3)

A student in chapters with unrelated primary examples (calculator, shapes, generic math) now sees Todo-focused examples as the primary teaching vehicle. The unrelated examples become secondary "also applies to" demonstrations.

**Why this priority**: Highest effort (26 lessons) but ensures complete consistency. Some lessons require significant restructuring.

**Independent Test**: Primary code blocks in Category D lessons demonstrate Todo operations, with previous examples moved to "Also applies to:" sections.

**Acceptance Scenarios**:

1. **Given** Ch18-01 (Arithmetic), **When** viewing primary example, **Then** it shows `score = duration * priority_weight` not generic `x + y`
2. **Given** Ch28-02 (Polymorphism), **When** viewing hierarchy, **Then** it shows `Task → Project, RecurringTask` not `Shape → Circle, Rectangle`
3. **Given** any Category D lesson, **When** restructuring, **Then** Python explanations remain unchanged, only examples are replaced

---

### User Story 6 - Connection Points Between Chapters (Priority: P3)

A student sees explicit connections showing how their Todo app evolves across chapters. Ch17 says "This task dict becomes the foundation," Ch23 says "Now we wrap our task operations in functions," Ch27 says "Let's transform our dict-based tasks into a proper Task class."

**Why this priority**: Creates narrative thread. Students understand progression, not isolated lessons.

**Independent Test**: Each tier transition (Tier 1→2, 2→3, etc.) has explicit callout referencing previous Todo progress.

**Acceptance Scenarios**:

1. **Given** Ch20 (Control Flow), **When** introducing loops, **Then** opening references task storage from Ch21 preview
2. **Given** Ch27 (OOP), **When** introducing classes, **Then** opening states "Transform your dict-based tasks from Ch21 into proper classes"
3. **Given** each chapter README, **When** reviewing, **Then** it shows what Todo component this chapter contributes

---

### Edge Cases

- What happens when an existing example is pedagogically superior to a Todo version? Keep it as auxiliary, don't force Todo where it doesn't fit naturally.
- How does the system handle Ch29-04 which is ALREADY gold standard? Mark as Category A, validate it meets quality bar, no changes needed.
- What happens when a lesson has no natural Todo mapping (e.g., trigonometry in Ch26-05)? Add Todo context where possible, keep specialized content for domain relevance.
- How do we prevent "Todo fatigue" where every example feels repetitive? Use auxiliary domains (legal, finance, healthcare) to show pattern universality.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Every lesson file MUST have a Todo-related primary example OR be marked Category A (already aligned)
- **FR-002**: Category B lessons MUST have variable names changed to task-related naming (`task`, `tasks`, `priority`, `done`, `due_date`, `TaskManager`)
- **FR-003**: Category B lessons MUST include at least 1 connecting sentence to the Todo running example
- **FR-004**: Category C lessons MUST include 2-3 auxiliary domain examples (Legal, Finance, Healthcare, or Marketing)
- **FR-005**: Category D lessons MUST restructure primary examples to be Todo-focused, moving existing examples to auxiliary sections
- **FR-006**: Critical capstones (Ch23-05, Ch25-05, Ch27-05, Ch32-06) MUST be fully rewritten as Todo components
- **FR-007**: Python explanations and core pedagogical content MUST NOT be changed (only examples)
- **FR-008**: Each chapter README MUST document what Todo component that chapter contributes
- **FR-009**: Connection points MUST be added at tier transitions (Ch20, Ch23, Ch27, Ch30)
- **FR-010**: Ch29-04 (TodoList dataclass) MUST be validated as meeting quality standards (gold standard reference)
- **FR-011**: All changes MUST preserve existing YAML frontmatter structure
- **FR-012**: No marketing language or promises MUST be added to lessons

### Key Entities

- **Task**: The core entity with attributes: `title`, `description`, `due_date`, `priority`, `status`, `done`
- **TodoList**: Collection of tasks with operations: add, remove, complete, filter, sort
- **TaskManager**: Class orchestrating todo operations (OOP chapters)
- **Auxiliary Entities**: Case (legal), Invoice (finance), Appointment (healthcare), Campaign (marketing)

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of 87 Category B/C/D lessons updated with Todo integration
- **SC-002**: 100% of 4 critical capstones (Ch23-05, Ch25-05, Ch27-05, Ch32-06) fully rewritten
- **SC-003**: 100% of Category B lessons (32) have task-related variable naming
- **SC-004**: 100% of Category C lessons (29) have 2+ auxiliary domain examples
- **SC-005**: 100% of Category D lessons (26) have Todo as primary example
- **SC-006**: All 5 tier transition points have explicit connection sentences
- **SC-007**: 18 chapter READMEs updated with Todo component documentation
- **SC-008**: Ch29-04 validated as meeting gold standard quality
- **SC-009**: Zero Python explanation changes (only examples modified)
- **SC-010**: All changes pass educational-validator checks

---

## Scope Boundaries

### In Scope

- Part 5 only (Chapters 15-32)
- Modifying code examples and variable names
- Adding auxiliary domain examples
- Adding connection sentences
- Rewriting 4 critical capstones
- Updating chapter READMEs

### Out of Scope

- Part 6 or other parts (separate specification)
- Changing Python explanations or pedagogical approach
- Adding new lessons or removing existing content
- Modifying YAML frontmatter beyond what's needed
- Creating new quizzes or assessments
- Technical infrastructure changes

---

## Assumptions

1. The completed audit (`specs/audit-part5-todo-integration.md`) is accurate and complete
2. Ch29-04 (TodoList dataclass) meets quality standards and serves as reference
3. Existing lessons are pedagogically sound; only examples need updating
4. Auxiliary domains (Legal, Finance, Healthcare, Marketing) are sufficiently diverse
5. Students will complete chapters sequentially (Todo progression builds)
6. Part 6 TaskManager Agent will use the Task class from Part 5

---

## Dependencies

- **Upstream**: Completed audit (`specs/audit-part5-todo-integration.md`)
- **Reference**: Ch29-04 for gold standard example quality
- **Downstream**: Part 6 TaskManager Agent specification (will reference Part 5 Todo)

---

## Constraints

- Cannot change Python explanations (only examples)
- Cannot add marketing language or promises
- Cannot remove existing good examples (demote to auxiliary)
- Must maintain YAML frontmatter structure
- Must preserve lesson position numbers and file structure

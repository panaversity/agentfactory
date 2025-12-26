# Part 5 Audit: Todo Running Example Integration

**Related Issues:** #406 (Parent), #407 (Part 5 Spec), #411 (Audit Guidelines)
**Date:** 2025-12-26
**Author:** Claude (Audit Agent)

---

## Executive Summary

### Total Lessons Audited: 98 lessons across 18 chapters

| Category | Count | Percentage | Description |
|----------|-------|------------|-------------|
| **A** (Already Todo-aligned) | 11 | 11% | Uses task/list/item patterns - minimal changes |
| **B** (Minor reframe) | 32 | 33% | Change variable names, add Todo context |
| **C** (Add auxiliary examples) | 29 | 30% | Good examples, needs domain diversity |
| **D** (Needs Todo as primary) | 26 | 27% | Unrelated examples, needs reframing |

### Estimated Effort
- **Phase 1 (Category B - Quick wins):** ~16 hours (variable renaming, context sentences)
- **Phase 2 (Category C - Auxiliary examples):** ~24 hours (add 2-3 domain examples per lesson)
- **Phase 3 (Category D - Primary reframing):** ~40 hours (major example restructuring)
- **Total:** ~80 hours

### Critical Capstones Requiring Rewrite
1. **Ch23 Lesson 05** - Calculator Utility → Todo Console App
2. **Ch25 Lesson 05** - Generic I/O → Todo Data Manager
3. **Ch27 Lesson 05** - Game Character → Todo/Task System (add variant)
4. **Ch32 Lesson 06** - Multi-Agent → Todo Multi-Agent System

---

## Chapter-by-Chapter Breakdown

### Chapter 15: Python & UV Package Manager (9 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-why-uv | Package management concepts | **C** | Add: "Setting up your Todo app project" |
| 02-installing-uv | Installation instructions | **A** | Already universal |
| 03-creating-first-project | `my_project` example | **B** | Rename to `todo_app` |
| 04-managing-dependencies | Generic packages | **B** | Use todo-relevant deps (click, typer) |
| 05-running-code | `main.py` runner | **B** | Reference `todo_app/main.py` |
| 06-team-collaboration | Environment sync | **C** | Add: "Sharing Todo app with team" |
| 07-ruff-basics | Generic linting | **B** | Lint `todo.py` examples |
| 08-advanced-ruff | Configuration | **C** | Configure for todo codebase |
| 09-pyright | Type checking | **B** | Type-check Task models |

**Chapter 15 Summary:** Mostly Category B/C. Easy wins by renaming project examples to `todo_app`.

---

### Chapter 16: Introduction to Python (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-what-is-python | Language overview | **A** | Already conceptual |
| 02-installing-python | Installation | **A** | Universal |
| 03-variables-type-hints | `name`, `age`, `email` | **B** | Add: `task_title`, `priority`, `done` |
| 04-basic-syntax | "Hello, World!", Alice examples | **B** | Add: "Todo: Review contract" print |
| 05-capstone | Personal Info Collector | **B** | Reframe as "Task Info Collector" |

**Chapter 16 Summary:** The capstone already uses data collection patterns. Minor reframe to task collection.

---

### Chapter 17: Data Types (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-understanding-types | Age, email, library metaphor | **C** | Add task type classification example |
| 02-numeric-types | Price, temperature | **C** | Add: priority (int), duration (float) |
| 03-text-boolean-none | `is_student`, `user_phone` | **B** | Add: `is_done: bool`, `due_date: str \| None` |
| 04-collections-binary | Shopping list, RGB colors | **B** | Primary: `tasks: list[str]`, `task: dict[str, str]` |
| 05-type-utilities-capstone | User Profile Validator | **B** | Reframe as Task Validator |

**Chapter 17 Summary:** Core data type examples map directly to Task attributes. Straightforward reframe.

---

### Chapter 18: Operators, Keywords & Variables (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-arithmetic | Generic `x=10, y=3` math | **D** | Todo: `score = duration * priority_weight` |
| 02-comparison | Age eligibility, password length | **C** | Add: `priority > 3`, `due_date <= today` |
| 03-logical | Permission checks | **C** | Reframe: `(assigned_to_me and active) or is_admin` |
| 04-assignment | Counter `count += 1` | **B** | Rename: `task_count += 1`, `total_hours += duration` |
| 05-keywords-capstone | Type-safe calculator | **D** | → "Task Effort Calculator" |

---

### Chapter 19: Strings & Type Casting (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-string-fundamentals | `name: str = "Alice"` | **B** | Add: `task: str = "Complete report"` |
| 02-essential-methods | Data cleaning pipeline | **A** | Already ideal for task title cleaning |
| 03-f-string-formatting | Receipt, weather display | **B** | Add: `f"Task: {title} (Priority: {p})"` |
| 04-type-casting | `int("42")`, validation | **B** | Add: `priority: int = int(priority_str)` |
| 05-capstone | Contact Card Validator | **A** | Reframe as Todo Item Validator |

---

### Chapter 20: Control Flow & Loops (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-conditionals | Age eligibility, grades | **C** | Add: task status transitions |
| 02-match-case | Grade assignment | **C** | Add: `match task_status: case "pending":` |
| 03-loops | Generic `for i in range(10)` | **D** | Primary: `for task in tasks: process()` |
| 04-controlling-loops | Break/continue search | **C** | Add: skip completed, exit on found |
| 05-nested-control | Grid processing | **C** | Add: `for category in cats: for task in tasks:` |

---

### Chapter 21: Lists, Tuples & Dictionaries (11 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-intro-collections | Abstract comparison | **B** | Motivate with Todo app needs |
| 02-lists-basics | `numbers`, `names` | **B** | Primary: `tasks: list[str]` |
| 03-lists-mutability | Shopping cart CRUD | **A** | Already task management pattern |
| 04-lists-sorting | Generic sorting | **C** | Add: sort by priority, deadline |
| 05-list-comprehensions | Filtering/transforming | **C** | Add: `[t for t in tasks if t.priority > 3]` |
| 06-tuples | Coordinates, RGB | **C** | Add: `STATUSES = ("pending", "done")` |
| 07-dicts-basics | Person dict | **B** | Primary: `task = {"title": ..., "status": ...}` |
| 08-dicts-crud | Assignment, pop, del | **A** | Already task database operations |
| 09-dicts-iteration | Dict comprehensions | **C** | Add: `{id: t for id, t in tasks.items() if urgent}` |
| 10-choosing-structure | Comparison table | **C** | Add Todo decision framework |
| 11-capstone | Data processing pipeline | **B** | → Todo Processing Pipeline |

---

### Chapter 22: Sets, Frozensets & GC (6 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-set-basics | Visited cities, emails | **B** | Rename: `completed_tasks`, `assigned_tasks` |
| 02-set-operations | Team membership | **C** | Add: task categories across projects |
| 03-set-internals | User authentication | **D** | Primary: task ID lookup O(1) vs list O(n) |
| 04-frozensets | Permission system | **B** | Add: `task_categories: dict[frozenset, int]` |
| 05-garbage-collection | Generic list/dict | **D** | Add: Task class with circular refs |
| 06-memory-capstone | Object tracking | **B** | Add: categorize by task status |

---

### Chapter 23: Modules & Functions (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-modules-imports | math, random modules | **D** | Show: `from todo_ops import add_task` |
| 02-writing-functions | `add()`, `greet()` | **D** | Primary: `add_task()`, `filter_tasks()` |
| 03-parameters-returns | `create_user_account()` | **D** | Primary: `create_task(title, priority="normal")` |
| 04-scope-nested | Counter, discount | **C** | Add: task filter closures |
| 05-capstone | Calculator utility | **D** | **CRITICAL:** → Todo Console App |

---

### Chapter 24: Exception Handling (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-fundamentals | ValueError, TypeError | **C** | Add: task parsing errors |
| 02-except-else-finally | File operations | **B** | Show: `try: parse_task()` pattern |
| 03-raising-custom | Auth exceptions | **B** | Create: `TaskValidationError`, `TaskNotFoundError` |
| 04-error-strategies | CSV parsing | **D** | → Task CSV import strategies |
| 05-capstone | CSV parser | **D** | → Todo CSV Importer |

---

### Chapter 25: I/O & File Handling (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-console-io | Age/name input | **B** | Primary: `task_title = input("Enter task: ")` |
| 02-file-io | Generic read/write | **B** | Primary: `tasks.txt`, `completed.txt` |
| 03-pathlib | Path construction | **B** | Example: `todo_dir / "tasks.txt"` |
| 04-csv-json | Generic handling | **B** | Primary: task JSON, CSV export |
| 05-capstone | Generic I/O | **D** | **CRITICAL:** → Todo Data Manager |

---

### Chapter 26: Math, DateTime & Calendar (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-math-module | Circle area, sqrt | **D** | Add: task priority scoring |
| 02-time-epoch | Elapsed time | **B** | Primary: task creation timestamps |
| 03-date-time-objects | Conference scheduling | **B** | Strengthen: task due date parsing |
| 04-datetime-formatting | Deadline countdown | **A** | Already deadline-focused |
| 05-calendar-advanced | Calendar display, trig | **C** | Add: recurring task scheduling |

---

### Chapter 27: OOP Part 1 (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-what-is-oop | BankAccount | **D** | Replace: Task system scaling problem |
| 02-classes-objects | Dog class | **D** | Replace: `Task(name, description, due_date)` |
| 03-constructors | Dog with `__init__` | **D** | Replace: Task with required/optional attrs |
| 04-encapsulation | BankAccount methods | **D** | Replace: Task status validation |
| 05-capstone | Game Character | **C** | Add: Todo/Task system variant |

**Chapter 27: HIGHEST PRIORITY** - All core OOP concepts use non-Todo examples. Perfect fit for Task modeling.

---

### Chapter 28: OOP Part 2 (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-inheritance-mro | Animal → Dog | **D** | Replace: Task → Project, RecurringTask |
| 02-polymorphism | Shape hierarchy | **D** | Replace: Task interface (execute() method) |
| 03-composition | Car HAS-AN Engine | **D** | Replace: Task HAS-A Assignee, Priority |
| 04-special-methods | Vector class | **D** | Replace: Task `__str__`, `__lt__` for sorting |
| 05-capstone | Design patterns | **A** | Already agent management - add Todo examples |

---

### Chapter 29: Metaclasses & Dataclasses (5 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-metaclasses | type(), Dog | **D** | Show: Task metaclass validation |
| 02-metaclass-patterns | Plugin registration | **B** | Reframe: Task type auto-registration |
| 03-intro-dataclasses | Person dataclass | **D** | Replace: `@dataclass class Task` |
| 04-advanced-dataclasses | TodoList example | **A** | **GOLD STANDARD** - already uses TodoList! |
| 05-choosing-tools | Decision matrix | **C** | Add: Task system decision examples |

---

### Chapter 30: Pydantic & Generics (6 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-intro-pydantic | Book model | **B** | Primary: `Task` model validation |
| 02-advanced-pydantic | Product, UserAccount | **C** | Add: Invoice, PatientRecord auxiliary |
| 03-intro-generics | `get_first_item[T]` | **D** | Replace: `TodoList[T]`, `filter_tasks[T]` |
| 04-generic-classes | Stack[T], Cache[K,V] | **D** | Replace: `TodoQueue[T]`, `TaskCache` |
| 05-pydantic-ai | Recipe, LLM validation | **B** | Reframe: TodoItem schema from LLM |
| 06-capstone | Config manager | **C** | Add: Todo app configuration |

---

### Chapter 31: Asyncio (6 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-asyncio-foundations | `fetch_api()` | **A** | Already good - rename to `fetch_todo_service()` |
| 02-concurrent-tasks | TaskGroup, gather | **A** | Already good - rename to task sync examples |
| 03-advanced-patterns | Timeout, retry | **A** | Already good - apply to todo resilience |
| 04-cpu-bound-work | Heavy calculations | **C** | Add: task analytics (CPU-bound metrics) |
| 05-hybrid-workloads | Mixed patterns | **C** | Add: TodoAnalyzer (I/O + CPU) |
| 06-capstone | AI agent | **C** | Add: TodoAgent coordination |

---

### Chapter 32: CPython & GIL (6 lessons)

| Lesson | Current Example | Category | Changes Needed |
|--------|-----------------|----------|----------------|
| 01-what-is-cpython | Bytecode, memory | **C** | Add: `python todo_app.py` execution trace |
| 02-performance-evolution | Python history | **C** | Add: task list processing benchmarks |
| 03-traditional-gil | GIL concept | **D** | **CRITICAL:** Show GIL impact on task analysis |
| 04-free-threaded | Python 3.13+ | **C** | Add: parallel task processing speedup |
| 05-choosing-concurrency | Decision framework | **D** | Primary: Todo app concurrency decisions |
| 06-capstone | Multi-agent system | **D** | **CRITICAL:** → Todo Multi-Agent System |

---

## Connection Points (Todo Running Example Progression)

### Tier 1: Foundation (Chapters 15-19)
```
Ch15: Set up todo_app project with UV
Ch16: First `print("Todo: Review contract")` + type hints
Ch17: Task data types: str (title), bool (done), dict (task)
Ch18: Task calculations: priority scoring, status comparisons
Ch19: Task string processing: clean titles, format display
```

### Tier 2: Collections (Chapters 20-22)
```
Ch20: Task processing loops: for task in tasks, while pending
Ch21: Task storage: list[Task], dict[str, Task], tuple statuses
Ch22: Unique task IDs (set), category combinations (frozenset)
```

### Tier 3: Structure (Chapters 23-25)
```
Ch23: Task functions: add_task(), complete_task(), list_tasks()
Ch24: Task validation: TaskNotFoundError, TaskValidationError
Ch25: Task persistence: tasks.json, tasks.csv, pathlib paths
```

### Tier 4: OOP (Chapters 26-29)
```
Ch26: Task timestamps: due_date, created_at, timedelta
Ch27: Task class: __init__, status property, class methods
Ch28: Task hierarchy: Project, RecurringTask; composition
Ch29: @dataclass Task (Ch29-4 ALREADY DOES THIS!)
```

### Tier 5: Advanced (Chapters 30-32)
```
Ch30: Pydantic Task model with validation
Ch31: Async task sync with cloud services
Ch32: Todo app performance, multi-agent orchestration
```

---

## Implementation Priority

### Phase 1: Critical Capstones (HIGH IMPACT)
1. **Ch23-05**: Calculator → Todo Console App
2. **Ch25-05**: I/O Capstone → Todo Data Manager
3. **Ch27-05**: Add Todo/Task variant alongside Game Character
4. **Ch32-06**: Multi-Agent → Todo Multi-Agent System

### Phase 2: Core OOP Reframes (FOUNDATIONAL)
5. **Ch27** (all lessons): Dog/BankAccount → Task class
6. **Ch28-01 to 28-04**: Animal/Shape/Car → Task hierarchy
7. **Ch29-03**: Person dataclass → Task dataclass

### Phase 3: Quick Wins (Category B)
- Ch15: Rename `my_project` → `todo_app`
- Ch16: Add task variable examples
- Ch17: Add task type examples
- Ch18-04, Ch19-01/03/04: Variable renaming
- Ch21-02/07: List/dict task examples
- Ch25-01/02/03/04: Task I/O examples

### Phase 4: Auxiliary Domains (Category C)
For each lesson needing auxiliary examples, add:
- **Legal**: Case management, contract deadlines, filing systems
- **Finance**: Invoice tracking, payment schedules, budget items
- **Healthcare**: Appointment scheduling, patient records, prescriptions
- **Marketing**: Campaign tasks, lead tracking, content calendar

---

## Success Metrics

After implementation:
- [ ] Every chapter references previous Todo progress
- [ ] Ch29-04 (TodoList dataclass) is highlighted as "gold standard"
- [ ] Part 6 (TaskManager Agent) has clear foundation from Part 5
- [ ] Students can trace their Todo app from Ch15 → Ch32
- [ ] Each lesson has 1 Todo + 2-3 auxiliary domain examples

---

## Files Summary

**Chapters with most Category D lessons (priority order):**
1. Ch27 OOP Part 1: 4/5 lessons need reframe
2. Ch28 OOP Part 2: 4/5 lessons need reframe
3. Ch23 Modules/Functions: 4/5 lessons need reframe
4. Ch18 Operators: 2/5 lessons need reframe
5. Ch32 CPython/GIL: 3/6 lessons need reframe

**Chapters already well-aligned:**
1. Ch29-04: TodoList dataclass (GOLD STANDARD)
2. Ch31: Asyncio patterns (just rename examples)
3. Ch19: String methods (data cleaning patterns)
4. Ch21-03/08: List/dict CRUD (task management patterns)

---

*Generated by Claude Audit Agent - 2025-12-26*

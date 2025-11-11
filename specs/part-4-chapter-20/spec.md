# Feature Specification: Chapter 20 - Module and Functions

**Chapter Number**: 20
**Chapter Title**: Module and Functions
**Part**: 4 (Python Fundamentals)
**Created**: 2025-11-09
**Status**: Clarified & Ready for Planning
**CEFR Proficiency Level**: A2-B1 (Intermediate)
**Complexity Tier**: Intermediate (7 concepts max per lesson)
**Prerequisites**: Chapters 1-19 (AIDD principles, data types, control flow, loops, collections)

---

## Overview

Chapter 20 teaches students how to organize and reuse Python code through **modules** (file-based organization and imports) and **functions** (reusable code blocks with parameters, returns, and scope). Students learn to think in terms of modular, maintainable code using Python's built-in and custom modules, alongside writing functions that communicate intent through type hints and clear parameter/return specifications.

This chapter bridges foundational syntax (Chapters 12-19) with production-oriented patterns (Chapters 21+) by teaching students to **describe intent** through function signatures and **orchestrate code organization** through module imports.

---

## Learning Objectives (Alignment with Evals-First Approach)

By the end of Chapter 20, students will be able to:

1. **Understand module organization** — Explain why Python developers split code into modules and how imports work (knowledge/comprehension)
2. **Write reusable functions** — Create functions with clear parameters, return values, and type hints (application)
3. **Apply scope rules** — Predict variable behavior across function and module boundaries (application)
4. **Orchestrate code organization** — Build a multi-module project with clear separation of concerns (synthesis)
5. **Validate function behavior** — Use type hints and test code to ensure functions work as specified (evaluation)

---

## User Scenarios & Testing

### User Story 1 - Learning Module Imports (Priority: P1)

**Scenario**: A beginner developer learns that Python code is organized in modules (.py files) and that imports bring code from one module into another.

**Why this priority**: Understanding modules is foundational to Python development. Students cannot progress to later chapters (file I/O, packages, frameworks) without grasping how `import` statements work.

**Independent Test**: Can be fully tested by writing `import math` and using `math.sqrt()`, and verifying understanding of import variants (`from math import sqrt`, `import math as m`). Delivers the value: "I can reuse code from Python's built-in modules."

**Acceptance Scenarios**:

1. **Given** a Python script, **When** student imports a built-in module (e.g., `import math`), **Then** student can access functions from that module (e.g., `math.sqrt(25)`)
2. **Given** import statements like `import math`, `from math import sqrt`, `from math import sqrt as s`, **When** student reads code, **Then** student correctly predicts how each import variant is used
3. **Given** a custom module file (e.g., `helpers.py`), **When** student imports it (`import helpers` or `from helpers import func`), **Then** student can call functions from that custom module
4. **Given** the `from module import *` syntax, **When** student reads documentation, **Then** student understands why it's discouraged and when it's appropriate

---

### User Story 2 - Writing Functions with Intent (Priority: P1)

**Scenario**: A developer learns to write functions with clear parameter lists, return values, and type hints so other developers (and their AI collaborators) understand the intent without reading implementation.

**Why this priority**: Writing functions is THE core competency for intermediate Python developers. Type-annotated function signatures enable AI-driven development by making intent explicit.

**Independent Test**: Can be fully tested by writing a function like `def add(a: int, b: int) -> int: return a + b`, documenting what it does, and testing that it works correctly. Delivers the value: "I can write functions that are clear to read and safe to use."

**Acceptance Scenarios**:

1. **Given** a problem statement (e.g., "calculate the square of a number"), **When** student writes a function with parameters, return type, and a docstring, **Then** another developer can understand and use that function without reading the implementation
2. **Given** type hints in a function signature, **When** student or their AI reads the function, **Then** they know what data types are expected and returned
3. **Given** multiple parameters with different types, **When** student calls the function, **Then** Python enforces type safety and catches errors early
4. **Given** a function that returns multiple values, **When** student unpacks the return value, **Then** they correctly assign each value to variables

---

### User Story 3 - Understanding Scope and Nested Functions (Priority: P2)

**Scenario**: A developer learns that variables have scope (local, function, module) and that functions can be nested, preparing them for advanced patterns later.

**Why this priority**: Scope is critical for writing correct code and avoiding bugs. Nested functions prepare students for decorators (Chapter 26+) but are not deeply explored in Chapter 20.

**Independent Test**: Can be fully tested by writing a function that uses local and global variables, predicting behavior, and observing execution. Delivers the value: "I understand where variables exist and can predict code behavior."

**Acceptance Scenarios**:

1. **Given** a variable defined outside a function, **When** the function reads that variable, **Then** the function accesses the outer scope variable
2. **Given** a variable defined inside a function, **When** code outside the function tries to access it, **Then** Python raises a NameError (the variable doesn't exist in outer scope)
3. **Given** the `global` keyword, **When** a function uses `global x` and modifies `x`, **Then** the modification affects the module-level variable
4. **Given** a nested function, **When** the inner function reads a variable from the outer function, **Then** the variable is in the inner function's closure (prepare for Chapter 26+)

---

### User Story 4 - Building a Multi-Module Project (Priority: P2)

**Scenario**: A student integrates modules and functions into a real hands-on project, applying all concepts from Chapter 20 in a cohesive application.

**Why this priority**: Integration project reinforces learning and shows students why modules and functions matter in practice. Capstone solidifies intermediate skills before moving to exception handling and I/O.

**Independent Test**: Can be fully tested by building a Calculator Utility project with 2-3 modules (e.g., `math_operations.py`, `utils.py`, `main.py`), each with reusable functions. Delivers the value: "I can organize real code into modules and functions."

**Acceptance Scenarios**:

1. **Given** a project with multiple Python files, **When** the main script imports functions from helper modules, **Then** the project runs correctly and produces expected output
2. **Given** functions with clear intent (type hints, docstrings), **When** another person (or AI) reads the code, **Then** they understand what each function does without reading implementation
3. **Given** scope rules, **When** different modules define functions with the same name, **Then** Python correctly calls the right function based on import (no naming conflicts)
4. **Given** a completed project, **When** student runs tests on each function, **Then** all tests pass and the project is ready to share

---

### Edge Cases

- What happens when a student imports a module that doesn't exist? (ImportError)
- What happens when a function parameter name shadows a global variable?
- What happens when a student imports `*` from a module with many functions?
- What happens when a student forgets to include `return` in a function? (Returns None)
- What happens when a student calls a function before defining it? (NameError)
- What happens with circular imports (module A imports module B, which imports module A)?

---

## Requirements

### Functional Requirements

**FR-001**: Students MUST understand built-in modules
- Demonstrate knowledge of at least 3 built-in modules (math, random, os) and their purposes
- Show how to import and use functions from built-in modules

**FR-002**: Students MUST write functions with intent
- Create functions with clear parameter names, type hints, and return types
- Write docstrings that describe what the function does
- Apply function definitions to real problems (calculations, data processing)

**FR-003**: Students MUST understand function parameters and returns
- Use positional parameters, default parameters, and type hints
- Return single and multiple values from functions
- Unpack return values correctly

**FR-004**: Students MUST understand scope rules
- Distinguish between local and module-level variables
- Predict variable behavior across function boundaries
- Understand the `global` keyword (use case, when needed)

**FR-005**: Students MUST create custom modules
- Organize related functions into a Python file (.py module)
- Import and use functions from custom modules
- Apply the principle of separation of concerns

**FR-006**: Students MUST understand nested functions (awareness for advanced patterns)
- Define functions inside other functions
- Understand how inner functions access outer function variables (closure concept)
- Recognize this pattern as preparation for decorators (Chapter 26+)

**FR-007**: Students MUST demonstrate integration
- Build a multi-module project (Calculator Utility or similar)
- Apply modules, functions, parameters, returns, and scope in a cohesive project
- Validate the project with testing (manual or automated)

---

### Key Entities

**Module**: A .py file containing Python code (functions, classes, variables). Can be imported and reused.

**Function**: A reusable block of code that takes parameters and returns values. Communicates intent through type hints and docstrings.

**Parameter**: A variable in a function's definition that accepts values when the function is called.

**Return Value**: Data returned by a function to the caller. Specified with `->` type hint.

**Scope**: The region of code where a variable is defined and accessible. Levels: local (inside function), enclosing (nested function), global (module-level), built-in (Python's built-ins).

**Type Hint**: Annotation in function signature specifying expected input/output types (e.g., `def add(a: int, b: int) -> int`).

---

## Success Criteria

### Measurable Outcomes

**SC-001**: 90% of students write at least one custom module with 3+ functions and import those functions successfully
- Validates ability to organize code and understand import mechanics

**SC-002**: 85% of students write functions with type hints and docstrings on first attempt (with AI assistance as needed)
- Validates clarity of intent and ability to communicate function purpose

**SC-003**: Students correctly predict variable scope behavior in 80%+ of test scenarios (given global/local variable code, predict output)
- Validates understanding of scope rules

**SC-004**: Students build and test a multi-module project (Capstone) with all code working correctly
- Validates integration and practical application

**SC-005**: Students explain the difference between `import math`, `from math import sqrt`, and `from math import *` correctly
- Validates understanding of import variants

---

## Content Outline

Chapter 20 consists of 5 lessons (4 foundational + 1 capstone project):

### Lesson 1: Understanding Modules and Imports
- What is a module? (Why organize code?)
- Built-in modules (math, random, os)
- Import variants: `import`, `from X import Y`, `from X import Y as Z`, `from X import *`
- Reading and using documentation
- **AI-Native Learning Pattern**: Describe what you want to do → Ask AI to show import examples → Validate understanding with code

### Lesson 2: Writing Functions with Intent
- Function definition: `def`, parameters, return
- Type hints: `def func(param: Type) -> ReturnType`
- Docstrings: explaining function purpose
- Calling functions and handling return values
- **AI-Native Learning Pattern**: Describe the function you need → Ask AI to generate it → Validate behavior

### Lesson 3: Function Parameters and Returns
- Positional parameters (required, order-dependent)
- Default parameters (optional, with defaults)
- Multiple returns (tuples, unpacking)
- `*args` awareness (for future chapters, not deeply taught)
- **AI-Native Learning Pattern**: Explore parameter options with AI → Test different calling patterns

### Lesson 4: Scope and Nested Functions
- Variable scope: local vs. global vs. enclosing
- The `global` keyword (when and why)
- Nested functions and closures (awareness for Chapter 26+)
- Variable shadowing (mistakes and how to avoid)
- **AI-Native Learning Pattern**: Predict scope behavior → Run code → Learn from results

### Lesson 5: Building a Calculator Utility (Capstone Project)
- Integrate modules + functions + scope in a real project
- Multi-file organization (main.py, operations.py, utils.py)
- Testing and validation
- **AI-Native Learning Pattern**: Orchestrate AI to help build the project → Validate with testing

---

## Code Examples Specification

**6-8 code examples** across all 5 lessons:

1. **Built-in module usage** (math.sqrt, math.pi) — Understand module structure
2. **Custom module creation** (simple helpers.py) — Create own module
3. **Function definition with type hints** (def add(a: int, b: int) -> int) — Intent clarity
4. **Function with default parameters** (def greet(name, greeting="Hello")) — Parameter flexibility
5. **Multiple return values** (def divide(a, b) -> tuple[float, float]) — Unpacking returns
6. **Nested function and closure** (inner function accessing outer scope) — Prepare for decorators
7. **Variable scope** (global variable modified in function) — Understand scope rules
8. **Capstone project sketch** (multi-module calculator structure) — Integration example

---

## AI-Native Learning Approach (Part 4 Appropriate)

**Pattern Applied Throughout**: Describe Intent → Explore with AI → Validate Understanding → Learn from Errors

- **Describe Intent**: Students use type hints and docstrings to express what they want functions to do
- **Explore with AI**: Ask Claude Code/Gemini CLI "Show me how to import the math module," "Generate a function that calculates factorial"
- **Validate Understanding**: Test code with examples, predict scope behavior, verify function returns
- **Learn from Errors**: When errors occur, ask AI "Why did this ImportError happen?" or "Why is this variable undefined?"

**NOT Taught**: Formal Specification-Driven Development (that's Chapter 30+). Instead, students practice describing intent through type hints and clear code structure, preparing them for SDD later.

---

## Assumptions

1. **Students have completed Chapters 1-19**: They understand AIDD, data types, control flow, loops, and collections
2. **Python 3.14+**: Modern syntax with type hints as a core feature
3. **AI tool availability**: Students have access to Claude Code or Gemini CLI for exploration
4. **Module organization**: We teach file-based module structure (not packages with __init__.py — that's advanced)
5. **Capstone scope**: Calculator utility is simple enough for intermediate learners (not complex OOP or async)

---

## Complexity & Cognitive Load Validation

- **CEFR Level**: A2-B1 (intermediate learner)
- **Concepts per lesson**: Max 7 (intermediate tier limit)
  - Lesson 1: 5 concepts (module, import variants 3, documentation, built-in)
  - Lesson 2: 6 concepts (function def, parameters, returns, type hints, docstrings, calling)
  - Lesson 3: 5 concepts (positional params, defaults, returns, unpacking, *args awareness)
  - Lesson 4: 4 concepts (local scope, global scope, nested functions, closures)
  - Lesson 5: 7 concepts (integration, file organization, testing, module reuse, etc.)

- **Proficiency Progression**: A2 (foundational awareness) → B1 (apply in integrated projects)
- **Cognitive Load**: Balanced across lessons with hands-on practice and AI exploration

---

## Testing & Validation Strategy

**How we'll know students understand**:

1. **Lesson 1**: Students correctly predict output of import statements and can import + use built-in modules
2. **Lesson 2**: Students write functions with type hints matching requirements; docstrings are clear
3. **Lesson 3**: Students correctly call functions with positional/default parameters; returns are unpacked correctly
4. **Lesson 4**: Students predict scope behavior; global variables are used appropriately
5. **Lesson 5**: Capstone project runs without errors; multi-module organization works; testing validates correctness

---

## Notes for Planning Phase

- **Graduated Teaching Pattern**: Book explains foundational concepts (what modules are, why scope matters) → AI companion handles syntax exploration → AI orchestration for capstone project
- **CoLearning Elements**: Use 💬 prompts (conceptual exploration), 🎓 commentaries (semantics emphasis), 🚀 challenges (specification-driven thinking), ✨ tips (AI tool usage) throughout
- **Lesson Closure**: Every lesson ends with "Try With AI" section (4 progressive prompts, Bloom's levels: Remember → Understand → Apply → Synthesize)
- **No Forward References**: Don't mention decorators, OOP, async, or Chapter 26+ concepts directly; acknowledge they exist but teach foundationally in Chapter 20
- **Type Hints are Core**: Use modern Python 3.14+ type hints throughout (not optional, fundamental to intent clarity)

---

**Ready for /sp.clarify and /sp.plan**

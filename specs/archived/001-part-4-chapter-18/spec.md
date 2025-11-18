# Feature Specification: Chapter 18 - Lists, Tuples, and Dictionary

**Feature Branch**: `001-part-4-chapter-18`
**Created**: 2025-11-08
**Status**: Draft
**Input**: User description: "Write Chapter 18: Lists, Tuples, and Dictionary in Part 4"

**Chapter Context**:
- Number: 18
- Title: "Lists, Tuples, and Dictionary" (from chapter-index.md)
- File Name: `18-lists-tuples-dictionary/`
- Part: 4 (Python Fundamentals - Chapters 12-29)
- Complexity Tier: Intermediate (A2-B1)
- Prerequisites: Chapters 1-17 (especially Ch 14 Data Types, Ch 17 Control Flow & Loops)

## Success Evals *(defined FIRST - business-goal-aligned)*

### Measurable Outcomes

**Comprehension Evals**:
- **EVAL-001**: 75%+ of students can explain the difference between mutable (lists) and immutable (tuples) data structures in their own words
- **EVAL-002**: 80%+ of students can choose the correct data structure (list/tuple/dict) for a given problem scenario
- **EVAL-003**: 70%+ of students can identify when to use dict.get() vs bracket notation and explain why

**Skill Acquisition Evals**:
- **EVAL-004**: 80%+ of students can write type-hinted code using `list[str]`, `dict[str, int]`, `tuple[float, ...]` without errors
- **EVAL-005**: 75%+ of students can successfully complete the Data Processing Pipeline capstone (ingests data → processes with lists/dicts → outputs results)
- **EVAL-006**: 70%+ of students can write list/dict comprehensions for simple transformations

**Engagement Evals**:
- **EVAL-007**: 85%+ completion rate for all 10+ lessons (students don't drop off due to cognitive overload)
- **EVAL-008**: Students submit "Try With AI" prompt 4 responses in 70%+ of lessons (demonstrates active AI partnership)

**Accessibility Eval**:
- **EVAL-009**: Reading level stays at Grade 7-8 (appropriate for intermediate A2-B1 learners)

**AI Partnership Eval**:
- **EVAL-010**: 80%+ of students report using Claude Code/Gemini CLI to explore concepts beyond lesson content (demonstrates co-reasoning behavior)

## Topic Summary

Chapter 18 teaches Python's three fundamental collection data structures: **lists** (ordered, mutable sequences), **tuples** (ordered, immutable sequences), and **dictionaries** (unordered key-value mappings). Students learn when to use each structure, how to manipulate data with type hints and modern Python 3.14+ syntax, and how to build real-world data processing applications.

The chapter emphasizes **AI-Native Learning**: students describe their intent using type hints, explore concepts with their AI companion (Claude Code/Gemini CLI), validate understanding through testing, and learn from errors. By the end, students build a **Data Processing Pipeline** application that demonstrates all three data structures in a practical context, preparing them for real software development scenarios.

## Prerequisites

**Required Prior Knowledge** (from previous chapters):
- **Chapter 14 (Data Types)**: Understanding of int, float, str, bool, None, and basic type hints
- **Chapter 15 (Operators, Keywords, Variables)**: Variable assignment, operators, expressions
- **Chapter 16 (Strings and Type Casting)**: String manipulation, type conversion
- **Chapter 17 (Control Flow & Loops)**: if/elif/else, for loops, while loops, iteration concepts

**Assumed Skills** (from Part 1-3):
- AI-Native Learning 4-step pattern (describe intent → explore → validate → learn from errors)
- Using Claude Code or Gemini CLI as co-reasoning partner
- Reading error messages and asking AI for explanations
- Writing clear code with meaningful variable names

## Learning Objectives *(aligned with A2-B1 CEFR proficiency)*

By the end of this chapter, students will be able to:

1. **LO-001 (A2 - Understand)**: Explain the differences between lists, tuples, and dictionaries, including mutability, use cases, and performance characteristics
2. **LO-002 (A2-B1 - Apply)**: Create and manipulate lists, tuples, and dictionaries using modern Python 3.14+ syntax with proper type hints
3. **LO-003 (B1 - Apply)**: Choose the appropriate data structure (list/tuple/dict) for a given problem based on requirements (mutability, ordering, lookup speed)
4. **LO-004 (B1 - Apply)**: Use common data structure methods (append, pop, get, keys, values, items) correctly in practical scenarios
5. **LO-005 (B1 - Analyze)**: Write list comprehensions and dictionary comprehensions for data transformations
6. **LO-006 (B1 - Create)**: Build a complete Data Processing Pipeline application that ingests, transforms, and outputs data using all three structures
7. **LO-007 (B1 - Evaluate)**: Debug data structure errors by asking AI targeted questions and understanding error messages

## Content Outline *(lesson structure)*

### Lesson 1: Introduction to Collections - Why They Matter (A2)
**Concepts** (5 max): Collection concept, sequences vs mappings, mutability vs immutability, type hints for collections, when to use each structure
**Learning Objectives**: LO-001 (understand differences)
**AI Partnership**: Compare collections conceptually, explore use cases

### Lesson 2: Lists Part 1 - Creation and Basic Operations (A2)
**Concepts** (6 max): List literals, list() constructor, type hints `list[T]`, indexing (positive/negative), slicing syntax, len()
**Learning Objectives**: LO-002 (create and manipulate)
**AI Partnership**: Explore indexing edge cases, ask AI about slicing patterns

### Lesson 3: Lists Part 2 - Mutability and Modification (A2-B1)
**Concepts** (7 max): Mutability concept, item assignment, append(), extend(), insert(), remove(), pop(), clear()
**Learning Objectives**: LO-002, LO-004 (methods)
**AI Partnership**: Ask AI when to use append vs extend, explore method differences

### Lesson 4: Lists Part 3 - Sorting, Reversing, and Advanced Methods (B1)
**Concepts** (7 max): sort() method, sorted() function, reverse(), count(), index(), copy(), list aliasing vs copying
**Learning Objectives**: LO-002, LO-004
**AI Partnership**: Explore sort() vs sorted(), ask AI about aliasing pitfalls

### Lesson 5: List Comprehensions - Transforming Data (B1)
**Concepts** (6 max): Comprehension syntax, expression evaluation, filtering with if, nested comprehensions (brief), readability considerations
**Learning Objectives**: LO-005 (comprehensions)
**AI Partnership**: Ask AI to translate loops to comprehensions, explore readability tradeoffs

### Lesson 6: Tuples - Immutable Sequences (A2-B1)
**Concepts** (7 max): Tuple literals, tuple() constructor, type hints `tuple[T, ...]`, immutability concept, use cases (hashable keys, returns, unpacking), tuple methods (count, index), memory efficiency
**Learning Objectives**: LO-001, LO-002, LO-003 (when to use tuples)
**AI Partnership**: Ask AI when immutability is valuable, explore tuple unpacking patterns

### Lesson 7: Dictionaries Part 1 - Key-Value Mappings (A2)
**Concepts** (6 max): Key-value concept, dict literals, dict() constructor, type hints `dict[K, V]`, accessing values (bracket notation), KeyError, get() with defaults
**Learning Objectives**: LO-002 (create and manipulate dicts)
**AI Partnership**: Explore get() vs bracket notation, ask AI about KeyError handling

### Lesson 8: Dictionaries Part 2 - CRUD Operations (A2-B1)
**Concepts** (7 max): Adding keys, updating values, deleting keys (del), pop() method, popitem(), clear(), checking key existence (in operator), unique keys constraint
**Learning Objectives**: LO-002, LO-004 (dict methods)
**AI Partnership**: Ask AI about pop() vs del differences, explore duplicate key behavior

### Lesson 9: Dictionaries Part 3 - Iteration and Comprehensions (B1)
**Concepts** (7 max): Iterating keys, values(), items(), keys(), dict comprehensions syntax, filtering dicts, transforming dicts, nested dicts (brief intro)
**Learning Objectives**: LO-004, LO-005 (dict comprehensions)
**AI Partnership**: Ask AI to convert dict loops to comprehensions, explore nested dict patterns

### Lesson 10: Choosing the Right Structure (B1)
**Concepts** (7 max): Decision criteria (mutability, ordering, lookup speed, duplicates), performance characteristics (O(1) vs O(n)), when to use list vs tuple vs dict, common patterns, anti-patterns
**Learning Objectives**: LO-003 (choosing structure), LO-007 (debugging)
**AI Partnership**: Present scenarios and ask AI which structure to use, explore performance implications

### Lesson 11: Capstone - Data Processing Pipeline (B1)
**Concepts** (integration): Real-world application combining all three structures
**Learning Objectives**: LO-006 (build complete application)
**Project**: Build a Data Processing Pipeline that:
  - Ingests CSV-like data as strings (simulated file reading using multi-line strings)
  - Parses data into list of dicts (each dict = one record)
  - Processes data (filtering, transforming using comprehensions)
  - Aggregates results (e.g., word frequency counter, grouping by category)
  - Outputs processed results (formatted strings)
**AI Partnership**: Use AI to design data structure choices, debug issues, optimize processing logic

## Code Examples *(specifications for examples)*

### Example Spec Format
Each code example includes:
- **Purpose**: What concept it demonstrates
- **Complexity**: Simple (5-10 lines) / Moderate (10-20 lines) / Complex (20-30 lines)
- **Type Hints**: Full annotations using Python 3.14+ syntax
- **AI Prompt**: Suggested prompt for students to explore with AI

### Lists Examples (Lessons 2-5)

**EX-001**: List creation with type hints
**Purpose**: Show list literals, list() constructor, heterogeneous vs typed lists
**Complexity**: Simple (5-7 lines)
```python
# Students will see:
numbers: list[int] = [1, 2, 3, 4, 5]
names: list[str] = ["Alice", "Bob", "Charlie"]
mixed: list = [1, "hello", 3.14, True]  # heterogeneous (avoid in practice)
empty: list[float] = []
```
**AI Prompt**: "Why do we use type hints like list[int]? What happens if I add a string to a list[int]?"

**EX-002**: Indexing and slicing
**Purpose**: Positive/negative indexing, slicing syntax
**Complexity**: Simple (8-10 lines)
```python
fruits: list[str] = ["apple", "banana", "cherry", "date"]
# Indexing examples with explanations
```
**AI Prompt**: "Explain negative indexing. What does fruits[-1] return and why?"

**EX-003**: List mutation methods
**Purpose**: append, extend, insert, remove, pop
**Complexity**: Moderate (12-15 lines)
```python
shopping_cart: list[str] = ["milk", "eggs"]
# Demonstrate each mutation method with print() to show state changes
```
**AI Prompt**: "What's the difference between append() and extend()? When should I use each?"

**EX-004**: Sorting and reversing
**Purpose**: sort() vs sorted(), reverse() method
**Complexity**: Moderate (10-12 lines)
```python
scores: list[int] = [85, 92, 78, 95, 88]
# Show in-place sort() vs sorted() function
```
**AI Prompt**: "Explain the difference between scores.sort() and sorted(scores). Which one modifies the original list?"

**EX-005**: List comprehension basics
**Purpose**: Transform list using comprehension
**Complexity**: Moderate (8-10 lines)
```python
# Traditional loop version
squares_loop: list[int] = []
for x in range(1, 6):
    squares_loop.append(x**2)

# Comprehension version
squares_comp: list[int] = [x**2 for x in range(1, 6)]
```
**AI Prompt**: "Rewrite this loop as a list comprehension and explain how the syntax works."

**EX-006**: List comprehension with filtering
**Purpose**: Use if condition in comprehension
**Complexity**: Simple (5-7 lines)
```python
numbers: list[int] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
evens: list[int] = [n for n in numbers if n % 2 == 0]
```
**AI Prompt**: "How does the 'if' work in a list comprehension? Where does it go in the syntax?"

### Tuples Examples (Lesson 6)

**EX-007**: Tuple creation and immutability
**Purpose**: Show tuple literals, demonstrate immutability error
**Complexity**: Simple (8-10 lines)
```python
point: tuple[int, int] = (10, 20)
rgb: tuple[int, int, int] = (255, 128, 0)
# Try to modify (causes error) - teaching moment for immutability
```
**AI Prompt**: "Why do tuples exist if lists can do the same thing? What's the advantage of immutability?"

**EX-008**: Tuple unpacking
**Purpose**: Unpack tuple into variables
**Complexity**: Simple (6-8 lines)
```python
coordinates: tuple[float, float] = (40.7128, -74.0060)
latitude, longitude = coordinates
# Show multiple return values from tuple
```
**AI Prompt**: "Explain how tuple unpacking works. What happens if the number of variables doesn't match?"

**EX-009**: Using tuples as dict keys
**Purpose**: Show tuples are hashable (unlike lists)
**Complexity**: Moderate (10-12 lines)
```python
locations: dict[tuple[int, int], str] = {
    (0, 0): "Origin",
    (10, 20): "Point A",
    (30, 40): "Point B"
}
```
**AI Prompt**: "Why can tuples be dict keys but lists can't? What does 'hashable' mean?"

### Dictionaries Examples (Lessons 7-9)

**EX-010**: Dictionary creation and access
**Purpose**: Dict literals, bracket notation, get()
**Complexity**: Simple (8-10 lines)
```python
student: dict[str, str | int] = {
    "name": "Alice",
    "age": 20,
    "major": "Computer Science"
}
# Show bracket notation vs get() with default
```
**AI Prompt**: "When should I use student.get('grade', 'N/A') instead of student['grade']?"

**EX-011**: Dictionary CRUD operations
**Purpose**: Add, update, delete keys
**Complexity**: Moderate (12-15 lines)
```python
inventory: dict[str, int] = {"apples": 50, "bananas": 30}
# Demonstrate adding, updating, deleting with explanations
```
**AI Prompt**: "What happens if I try to delete a key that doesn't exist using pop()? How is it different from del?"

**EX-012**: Iterating dictionaries
**Purpose**: Show keys(), values(), items() iteration
**Complexity**: Moderate (10-12 lines)
```python
grades: dict[str, int] = {"Alice": 95, "Bob": 87, "Charlie": 92}
# Iterate three different ways
```
**AI Prompt**: "Explain the difference between iterating with .keys(), .values(), and .items(). When would I use each?"

**EX-013**: Dictionary comprehension
**Purpose**: Transform dict using comprehension
**Complexity**: Moderate (8-10 lines)
```python
temps_celsius: dict[str, int] = {"NY": 20, "LA": 25, "Chicago": 15}
temps_fahrenheit: dict[str, float] = {
    city: (temp * 9/5) + 32
    for city, temp in temps_celsius.items()
}
```
**AI Prompt**: "Convert this dict comprehension to a regular for loop. Which is more readable?"

**EX-014**: Word frequency counter (practical example)
**Purpose**: Real-world dict usage
**Complexity**: Complex (20-25 lines)
```python
text: str = """Python is great. Python is powerful.
Python is used for AI development."""
# Count word frequency using dict
```
**AI Prompt**: "How would you modify this to ignore case and punctuation? Ask AI to help design the logic."

### Capstone Example (Lesson 11)

**EX-015**: Data Processing Pipeline (complete application)
**Purpose**: Integrate all three structures in real-world scenario
**Complexity**: Complex (60-80 lines total across multiple functions/sections)
**Features**:
- Ingest: Parse multi-line CSV-like string into list[dict]
- Process: Filter/transform data using comprehensions
- Aggregate: Calculate statistics, group by category using dicts
- Output: Format results as human-readable strings

**AI Prompt**: "Design the data structure for this pipeline. Should each record be a dict or a tuple? Why?"

## Acceptance Criteria *(references evals)*

### Content Quality
- [ ] All lessons follow AI-Native Learning 4-step pattern (describe → explore → validate → learn)
- [ ] Each lesson has CoLearning elements (💬 AI Colearning Prompt, 🎓 Instructor Commentary, 🚀 CoLearning Challenge, ✨ Teaching Tip) throughout content
- [ ] Conversational tone maintained (you, your, we) - NOT documentation style
- [ ] No forward references to Ch 20+ concepts (functions, exceptions, file I/O, OOP)
- [ ] Type hints used throughout (list[T], dict[K,V], tuple[T,...])
- [ ] All code examples tested and run on Python 3.14+
- [ ] Lesson closure: ONLY "Try With AI" section (4 prompts, Bloom's progression) - NO summaries/checklists after

### Learning Objectives Alignment
- [ ] LO-001: Lesson 1, 6, 10 teach differences between structures (EVAL-001, EVAL-002)
- [ ] LO-002: Lessons 2-4, 6-8 teach creation and manipulation (EVAL-004)
- [ ] LO-003: Lesson 10 teaches choosing correct structure (EVAL-002)
- [ ] LO-004: Lessons 3-4, 8-9 teach common methods (EVAL-003)
- [ ] LO-005: Lessons 5, 9 teach comprehensions (EVAL-006)
- [ ] LO-006: Lesson 11 capstone builds complete application (EVAL-005)
- [ ] LO-007: Throughout, students debug with AI (EVAL-010)

### Cognitive Load Compliance
- [ ] Max 7 concepts per lesson (intermediate A2-B1 tier)
- [ ] Concepts build progressively (lists → tuples → dicts → integration)
- [ ] Each lesson focuses on ONE primary skill
- [ ] Capstone integrates (doesn't introduce new concepts)

### Evals Traceability
- [ ] EVAL-001-003: Comprehension assessed via "Try With AI" prompts and exercises
- [ ] EVAL-004-006: Skills assessed via code examples and capstone project
- [ ] EVAL-007: Engagement tracked via lesson completion analytics
- [ ] EVAL-008: AI partnership assessed via prompt response submission rates
- [ ] EVAL-009: Reading level validated via automated tools (target Grade 7-8)
- [ ] EVAL-010: AI partnership behavior tracked via student self-reporting

### Technical Standards
- [ ] Python version: 3.14+
- [ ] Type hints: Modern syntax (list[int], dict[str, float], tuple[str, ...], X | None)
- [ ] No security violations (no eval(), no hardcoded secrets, no shell=True)
- [ ] All code formatted with clear variable names and comments
- [ ] Error handling appropriate for intermediate learners (explain errors, ask AI for help)

### Complexity Tier Validation
- [ ] Intermediate (A2-B1) proficiency appropriate for Chapter 18 position
- [ ] Prerequisites from Ch 1-17 respected (no gaps in assumed knowledge)
- [ ] Lesson difficulty progression smooth (A2 → A2-B1 → B1 across 11 lessons)
- [ ] Capstone project appropriate for B1 level (integration, not overwhelming)

### AI Partnership Quality
- [ ] AI positioned as co-reasoning partner (not autocomplete tool)
- [ ] "Try With AI" prompts are specific and concrete (not generic "ask AI about X")
- [ ] "Expected outcome" provided for each prompt (clear learning targets)
- [ ] Prompts follow Bloom's progression (Remember → Understand → Apply → Analyze/Create)
- [ ] Students practice validation ("Does this code match my intent?")

## User Scenarios & Testing *(mandatory for educational content)*

### User Story 1 - Understanding Collection Fundamentals (Priority: P1)

**As an intermediate Python learner (A2-B1)**, I want to understand when to use lists vs tuples vs dictionaries, so I can make informed data structure choices in my programs.

**Why this priority**: Foundational concept - without understanding differences, students can't progress to applying these structures correctly. This is the core "aha moment" that unlocks the rest of the chapter.

**Independent Test**: Student can explain the three structures' differences (mutability, use cases, performance) in their own words and choose the correct structure for 3 given scenarios (80%+ accuracy targets EVAL-001, EVAL-002).

**Acceptance Scenarios**:

1. **Given** student has read Lesson 1 (Introduction to Collections), **When** asked "When would you use a tuple instead of a list?", **Then** student explains immutability and provides a valid use case (e.g., dict keys, function returns, data integrity)

2. **Given** student has completed Lessons 1-10, **When** presented with scenario "Store user profiles where each profile has name, email, age", **Then** student chooses dict with string keys and explains why (key-value lookup, meaningful field names)

3. **Given** student uses Claude Code/Gemini CLI, **When** asking "What's the performance difference between lists and dicts for lookup?", **Then** AI explains O(n) vs O(1) and student demonstrates understanding by choosing dict for frequent lookups

---

### User Story 2 - Manipulating Lists and Dicts with Type Hints (Priority: P2)

**As an intermediate Python learner (A2-B1)**, I want to create and manipulate lists and dictionaries using modern Python syntax with type hints, so I can write clear, type-safe code like professional developers.

**Why this priority**: Core technical skill - students must manipulate these structures correctly before building applications. Type hints are essential for AI-native development (clear intent communication).

**Independent Test**: Student writes type-hinted code that creates a list[str], adds/removes items, creates a dict[str, int], and accesses values using get(). Code runs without errors and types are correct (targets EVAL-004).

**Acceptance Scenarios**:

1. **Given** student has completed Lessons 2-4 (Lists), **When** asked to write code that creates a list of integers, adds 3 items, removes 1, and sorts the list, **Then** student writes correct code with `list[int]` type hint and uses appropriate methods (append, remove, sort)

2. **Given** student has completed Lessons 7-8 (Dicts), **When** asked to create a dict mapping city names to temperatures, add a new city, and safely retrieve a city's temperature with a default, **Then** student uses `dict[str, int]`, adds key correctly, and uses .get() with default value

3. **Given** student encounters a TypeError ("cannot add str to list[int]"), **When** asking AI "Why did this error occur?", **Then** AI explains type mismatch and student corrects code by converting string to int or changing type hint

---

### User Story 3 - Writing Comprehensions for Data Transformation (Priority: P3)

**As an intermediate Python learner (A2-B1)**, I want to write list and dictionary comprehensions, so I can transform data concisely and read professional Python code that uses comprehensions.

**Why this priority**: Important applied skill but not critical for MVP - students can process data with loops initially. Comprehensions are "nice to have" for writing idiomatic Python.

**Independent Test**: Student converts a 5-line for loop into a list comprehension and writes a dict comprehension to transform key-value pairs. Both run correctly (targets EVAL-006).

**Acceptance Scenarios**:

1. **Given** student has completed Lesson 5 (List Comprehensions), **When** shown a for loop that squares numbers, **Then** student rewrites it as a list comprehension with correct syntax: `[x**2 for x in range(10)]`

2. **Given** student has completed Lesson 9 (Dict Comprehensions), **When** asked to convert a dict of Celsius temperatures to Fahrenheit, **Then** student writes dict comprehension: `{city: (temp * 9/5) + 32 for city, temp in temps.items()}`

3. **Given** student asks AI "Is this comprehension readable?", **When** AI responds that it's too complex (e.g., nested comprehension with multiple conditions), **Then** student refactors to a regular loop or simplifies logic

---

### User Story 4 - Building the Data Processing Pipeline Capstone (Priority: P1)

**As an intermediate Python learner (A2-B1)**, I want to build a complete Data Processing Pipeline application that uses lists, tuples, and dictionaries together, so I can apply collection concepts in a real-world project and demonstrate practical skills.

**Why this priority**: Capstone validates all learning objectives - without building a complete application, students only have fragmented knowledge. This is the proof of competency that targets EVAL-005.

**Independent Test**: Student completes the Data Processing Pipeline capstone project that ingests CSV-like data, processes it with lists/dicts, and outputs results. All three structures are used appropriately (targets EVAL-005, EVAL-006).

**Acceptance Scenarios**:

1. **Given** student has completed Lessons 1-10, **When** starting Lesson 11 capstone, **Then** student can ingest multi-line CSV-like string data and parse it into a list of dicts (one dict per record)

2. **Given** student has parsed data into list[dict], **When** asked to filter records (e.g., "only records where age > 18"), **Then** student writes a list comprehension with if condition: `[record for record in data if record['age'] > 18]`

3. **Given** student needs to aggregate data (e.g., count occurrences by category), **When** processing list[dict], **Then** student uses a dict to store counts and updates values correctly: `counts[category] = counts.get(category, 0) + 1`

4. **Given** student completes the capstone project, **When** running the full pipeline, **Then** application outputs correctly formatted results (e.g., summary statistics, grouped data) without errors

---

### Edge Cases

- **Empty collections**: What happens when iterating an empty list/dict? (Should handle gracefully, no errors)
- **KeyError handling**: What happens when accessing a non-existent dict key with brackets? (Teach using .get() with defaults)
- **Type mismatch**: What happens when adding wrong type to typed list? (Python 3.14 doesn't enforce at runtime, but mypy/type checkers will warn - teach using AI to check)
- **Mutating during iteration**: What happens when modifying a list while looping over it? (Common pitfall - causes unexpected behavior, teach safe patterns)
- **Dict key types**: What types can be dict keys? (Must be hashable - int, str, tuple yes; list, dict no - teach with tuple example)
- **List aliasing**: What happens when doing `list2 = list1` (aliasing vs copying)? (Both variables point to same list - teach using .copy())
- **Large data performance**: What happens when processing 10,000+ items? (Teach asking AI about performance characteristics, mention O(n) vs O(1) conceptually)

## Requirements *(mandatory)*

### Functional Requirements

**Content Requirements**:
- **FR-001**: Chapter MUST contain 11 lessons covering lists, tuples, and dictionaries comprehensively
- **FR-002**: Each lesson MUST follow AI-Native Learning 4-step pattern (describe intent → explore → validate → learn from errors)
- **FR-003**: All code examples MUST use Python 3.14+ syntax with modern type hints (list[T], dict[K, V], tuple[T, ...])
- **FR-004**: All lessons MUST include CoLearning structural elements (💬 AI Colearning Prompt, 🎓 Instructor Commentary, 🚀 CoLearning Challenge, ✨ Teaching Tip) throughout content
- **FR-005**: All lessons MUST end with "Try With AI" section ONLY (4 prompts, Bloom's progression) - NO summaries/checklists after

**Pedagogical Requirements**:
- **FR-006**: Chapter MUST respect max 7 concepts per lesson (intermediate A2-B1 cognitive load limit)
- **FR-007**: Chapter MUST NOT forward reference concepts from Ch 20+ (functions, exceptions, file I/O, OOP)
- **FR-008**: Lesson progression MUST follow graduated complexity: A2 (Lessons 1-3) → A2-B1 (Lessons 4-6, 8-9) → B1 (Lessons 5, 7, 10-11)
- **FR-009**: Capstone (Lesson 11) MUST integrate concepts from previous lessons (NOT introduce new concepts)

**Technical Requirements**:
- **FR-010**: All code examples MUST be tested and run successfully on Python 3.14+
- **FR-011**: Code MUST NOT use security anti-patterns (eval(), shell=True, hardcoded secrets)
- **FR-012**: Type hints MUST use modern syntax (not legacy typing.List, typing.Dict)
- **FR-013**: Code MUST use f-strings for string formatting (not % or .format())

**AI Partnership Requirements**:
- **FR-014**: Chapter MUST position AI as co-reasoning partner (NOT autocomplete tool or documentation replacement)
- **FR-015**: "Try With AI" prompts MUST be specific and concrete (NOT generic "ask AI about X")
- **FR-016**: Each "Try With AI" prompt MUST include "Expected outcome" describing what student learns
- **FR-017**: Prompts MUST follow Bloom's taxonomy progression (Remember → Understand → Apply → Analyze/Create)

**Assessment Requirements**:
- **FR-018**: Chapter MUST include measurable success evals aligned with business goals (student learning outcomes)
- **FR-019**: Capstone project MUST validate LO-006 (ability to build complete application using all three structures)
- **FR-020**: "Try With AI" prompts MUST enable evaluation of EVAL-001-003 (comprehension)

### Key Entities *(educational content context)*

**Learning Content Entities**:
- **Lesson**: A learning module focusing on 1 primary skill, max 7 concepts, 3.5-4 hours study time, includes CoLearning elements and "Try With AI" section
- **Code Example**: Demonstrative code with purpose, complexity level, type hints, and AI exploration prompt
- **Exercise**: Practice activity embedded in lessons (not separate quizzes - active learning approach)
- **Capstone Project**: Integration project (Lesson 11) that validates competency across all learning objectives

**Assessment Entities**:
- **Success Eval**: Measurable criterion (percentage, completion rate) tied to business goal (student learning outcome)
- **Learning Objective**: Specific skill student will achieve, tagged with CEFR level and Bloom's taxonomy level
- **Acceptance Scenario**: Testable condition (Given/When/Then) validating student can perform a skill

**AI Partnership Entities**:
- **AI Colearning Prompt**: Conversational prompt embedded in lesson content encouraging AI exploration of concepts
- **Try With AI Prompt**: Structured prompt at lesson end (4 per lesson) with expected outcome specified
- **Expected Outcome**: Description of what student learns/understands after AI interaction

**Data Structure Entities** (taught in chapter):
- **List**: Ordered, mutable sequence with type hint list[T]
- **Tuple**: Ordered, immutable sequence with type hint tuple[T, ...]
- **Dictionary**: Unordered key-value mapping with type hint dict[K, V]

## Assumptions

1. **Student Prerequisites**: Students have successfully completed Chapters 1-17, including understanding of basic data types, control flow, and loops
2. **AI Tool Access**: Students have access to Claude Code or Gemini CLI as their AI companion (Part 2 setup completed)
3. **Python Environment**: Students have Python 3.14+ installed with a working development environment (Chapter 12 UV setup completed)
4. **Time Commitment**: Students can dedicate 3.5-4 hours per lesson (11 lessons × 4 hours = ~44 hours total chapter study time)
5. **Reading Level**: Students can read Grade 7-8 level English text comfortably (supports A2-B1 CEFR proficiency)
6. **Motivation**: Students are motivated to build real applications (capstone project requires initiative and problem-solving)
7. **AI Literacy**: Students understand how to ask AI clear questions (developed through Chapters 9-11 on prompting/context engineering)

## Dependencies

**Internal Dependencies** (other chapters):
- **Chapter 14 (Data Types)**: Foundation for understanding type hints and built-in types
- **Chapter 15 (Operators, Keywords, Variables)**: Variable assignment, expressions
- **Chapter 16 (Strings and Type Casting)**: String manipulation needed for text processing examples
- **Chapter 17 (Control Flow & Loops)**: For loops required for iterating collections

**External Dependencies**:
- **Python 3.14+**: Language version for modern type hint syntax
- **Claude Code or Gemini CLI**: AI companion tools for co-learning
- **Text Editor/IDE**: For writing and running Python code (VS Code, PyCharm, or similar)

**Future Chapters Building on Ch 18**:
- **Chapter 19 (Set, Frozen Set, GC)**: Additional collection types
- **Chapter 20 (Module and Functions)**: Functions will operate on collections
- **Chapter 21 (Exception Handling)**: Handling KeyError, IndexError from collections
- **Chapter 22 (IO and File Handling)**: Reading/writing collections to files
- **Chapter 24+ (OOP)**: Objects will contain collections as attributes

## Constraints

**Content Constraints**:
- Max 7 concepts per lesson (cognitive load limit for intermediate A2-B1)
- No forward references to Ch 20+ concepts (scope boundary)
- No use of functions until Ch 20 (architectural constraint - lessons use inline code or built-in functions only)
- Conversational tone required (not documentation style)

**Technical Constraints**:
- Python 3.14+ only (no legacy syntax support)
- Type hints mandatory (part of AI-Native Learning approach - clear intent communication)
- No external libraries (only Python standard library - students haven't learned pip/uv package management beyond Ch 12 basics)

**Pedagogical Constraints**:
- AI positioned as co-reasoning partner (not replacement for learning - constitutional principle)
- Students must actively engage (not passive reading - "Try With AI" prompts are mandatory)
- Validation-first approach (students test understanding before moving forward)

**Time Constraints**:
- Each lesson: 3.5-4 hours study time (not shorter - need depth for intermediate level)
- Total chapter: ~44 hours (11 lessons × 4 hours) - realistic for intermediate learners

**Platform Constraints**:
- Content rendered in Docusaurus (markdown format with frontmatter)
- Code blocks must be syntax-highlighted (``` python fences)
- Lessons displayed in sidebar navigation (ordered 01-11)

## Out of Scope

**Explicitly NOT included in Chapter 18**:

- **Functions and modules** (Ch 20): No function definitions, no imports beyond built-ins
- **Exception handling details** (Ch 21): Basic KeyError/IndexError mentioned but no try/except blocks
- **File I/O** (Ch 22): Capstone uses multi-line strings to simulate CSV data (not actual file reading)
- **Advanced OOP** (Ch 24-26): No classes, objects, methods (collections are built-in types)
- **Pydantic and advanced typing** (Ch 27): No TypedDict, no Pydantic models
- **Asyncio** (Ch 28): No async/await for concurrent collection processing
- **Performance optimization** (Ch 29): No CPython internals, no GIL discussion, no detailed performance profiling

**Other exclusions**:
- **Advanced comprehensions**: No nested comprehensions beyond brief mention (readability over cleverness)
- **Collections module**: No Counter, defaultdict, OrderedDict (standard library extensions deferred)
- **NumPy arrays**: No numeric computing libraries (out of scope for fundamentals)
- **Pandas DataFrames**: No data science tools (out of scope)
- **JSON parsing**: Mentioned conceptually but not implemented (needs file I/O from Ch 22)

## Risks

**Risk 1: Cognitive overload from 11 lessons**
- **Likelihood**: Medium (more lessons = more fatigue risk)
- **Impact**: High (students drop off, EVAL-007 fails)
- **Mitigation**: Strict 7-concept limit per lesson, progressive difficulty (A2→B1), frequent "Try With AI" breaks for active learning

**Risk 2: Students struggle with comprehensions**
- **Likelihood**: Medium (comprehensions are syntactically dense for intermediate learners)
- **Impact**: Medium (EVAL-006 fails but students can still use loops)
- **Mitigation**: Lesson 5 shows loop-to-comprehension transformation, AI prompts help decode syntax, emphasize readability over cleverness

**Risk 3: Type hints confuse students (runtime vs static checking)**
- **Likelihood**: Medium (type hints don't enforce at runtime in Python)
- **Impact**: Medium (students think `list[int]` prevents adding strings - misunderstanding)
- **Mitigation**: Explicitly teach type hints are "documentation + tooling support", show AI can check types, use mypy in "Try With AI" prompts

**Risk 4: Capstone project too complex for B1 level**
- **Likelihood**: Low (if well-scaffolded)
- **Impact**: High (EVAL-005 fails, students feel defeated)
- **Mitigation**: Capstone lesson provides step-by-step guidance, AI partnership for debugging, focus on integration (not novelty)

**Risk 5: No forward references constraint limits real-world examples**
- **Likelihood**: Low (collections have rich built-in methods)
- **Impact**: Low (examples might feel slightly artificial without functions)
- **Mitigation**: Use inline code creatively, acknowledge "we'll refactor this with functions in Ch 20", focus on data structures themselves

## Success Criteria *(technology-agnostic, measurable outcomes)*

### Measurable Outcomes

**Comprehension (Knowledge)**:
- **SC-001**: 75%+ of students can explain the difference between mutable (lists) and immutable (tuples) data structures in a short-answer assessment
- **SC-002**: 80%+ of students can identify the correct data structure (list/tuple/dict) for 5 given problem scenarios
- **SC-003**: 70%+ of students can articulate when to use dict.get() vs bracket notation and describe the tradeoff

**Skill Acquisition (Application)**:
- **SC-004**: 80%+ of students write syntactically correct type-hinted code using `list[str]`, `dict[str, int]`, `tuple[float, ...]` in practice exercises
- **SC-005**: 75%+ of students successfully complete the Data Processing Pipeline capstone project (all three structures used correctly, application runs without errors)
- **SC-006**: 70%+ of students can convert a 5-line for loop into an equivalent list comprehension

**Engagement (Behavior)**:
- **SC-007**: 85%+ lesson completion rate across all 11 lessons (students don't drop off)
- **SC-008**: 70%+ of students submit "Try With AI" prompt 4 responses in at least 8 out of 11 lessons (demonstrates active AI partnership)
- **SC-009**: Students spend 35-45 hours total on chapter (measured via LMS analytics or self-reporting) - indicates appropriate pacing

**AI Partnership (Behavior)**:
- **SC-010**: 80%+ of students report using Claude Code/Gemini CLI to explore concepts beyond lesson content (measured via end-of-chapter survey)
- **SC-011**: 75%+ of students report asking AI for error explanations at least 3 times during the chapter (demonstrates debugging partnership)

**Quality (Content)**:
- **SC-012**: Chapter content reads at Grade 7-8 level (validated via Flesch-Kincaid or similar readability tool)
- **SC-013**: 100% of code examples run successfully on Python 3.14+ without errors (technical validation)
- **SC-014**: 0 forward references to Ch 20+ concepts (architectural validation)

## Notes

**Key Design Decisions**:
1. **11 lessons (not 5-6)**: User preference for comprehensive coverage - acceptable per constitutional "10+ lessons okay" guidance
2. **Capstone as Lesson 11**: Integration project validates competency (not separate from lesson sequence)
3. **No functions until Ch 20**: Constitutional constraint means lessons use inline code or built-in functions only (e.g., len(), sorted(), enumerate())
4. **Real-world outcome**: Data Processing Pipeline demonstrates practical application (not toy examples)

**Alignment with AI-Native Learning**:
- Students describe intent via type hints (clear communication)
- Students explore via AI partnership (conceptual understanding)
- Students validate via testing code (hands-on practice)
- Students learn from errors via AI explanations (debugging mindset)

**Alignment with Constitution v3.0.2**:
- **Principle 12 (Cognitive Load Management)**: Max 7 concepts per lesson enforced
- **Principle 13 (Graduated Teaching Pattern)**: Book teaches foundations (what lists are), AI companion handles complex exploration (comprehensions), AI orchestration deferred to capstone
- **Evals-First Philosophy**: Success evals defined BEFORE content outline (professional pattern)
- **Bilingual Development**: Python focus in Part 4 (TypeScript in Part 9+)

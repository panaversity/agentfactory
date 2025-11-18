# Feature Specification: Chapter 14 - Data Types (AIDD-First Python Learning)

**Feature Branch**: `014-data-types`
**Created**: 2025-01-08
**Status**: Draft
**Input**: Write Chapter 14: Data Types in Part 4 - Comprehensive Python data types taught through AI partnership (AIDD-first approach, not traditional syntax memorization)

---

## Success Evals *(mandatory)*

### How We Know Students Understand Data Types

These are **business-goal-aligned success criteria** that measure learning outcomes:

1. **Comprehension Eval**: 75%+ can explain "what is a data type and why does Python care?" in their own words (measured via quiz/exercise submission)

2. **Skill Acquisition Eval**: 80%+ can use `isinstance()` to check types and explain results (measured via hands-on exercise)

3. **Type Exploration Eval**: 90%+ can ask AI "what can I do with this type?" and extract useful operations (measured via "Try With AI" exercise completion)

4. **Error Recovery Eval**: 85%+ can take a `TypeError`, ask AI for explanation, and fix the issue (measured via troubleshooting exercise)

5. **Interactive Explorer Outcome**: 100% build working type explorer that demonstrates understanding through AI partnership (measured via capstone project submission)

**Accessibility Eval**: Content maintains Grade 7-8 reading level (automated readability check)

**Engagement Eval**: 70%+ completion rate on "Try With AI" exercises (measured via platform analytics)

---

## Topic Summary *(mandatory)*

**Building on Chapter 13's Hello World Foundation**:

Chapter 13 teaches: What is Python + Installing Python + Hello World with `print()`

Chapter 14 builds on this by teaching:
1. **Variables with Type Hints**: Store data with specification-first code
2. **Data Types Foundation**: Understand types through AI partnership, not syntax memorization

Students arrive knowing `print()` from Hello World. Chapter 14 teaches them to **store data in variables** AND understand **what types are and why they matter**.

**Core Philosophy**: Types aren't syntax to memorize - they're concepts to explore. Students use Claude Code or Gemini CLI as their co-learning partner to discover "what can I do with this type?" through dialogue.

**Learning Outcomes**:
- Create variables with type hints (builds on Chapter 13's print())
- Understand type concept and type() function
- Work with core types (int, float, str, bool, None)
- Explore collections awareness (list, tuple, dict, set)
- Build simple type explorer demonstrating type operations

**AI-Native Learning Pattern**:
- **Describe intent**: Type hints describe what data means (`age: int` says "age is a number")
- **Explore with AI**: Ask AI "what can I do with this type?" instead of reading documentation
- **Validate together**: Use isinstance() and type() to check understanding
- **Learn from errors**: When TypeError occurs, ask AI "why?" to deepen learning

---

## Prerequisites *(mandatory)*

Students MUST have completed:

- **Chapters 1-4**: AI-Native development mindset (Nine Pillars, AI as learning partner)
- **Chapter 5-11**: AI tool literacy (Claude Code, Gemini CLI, prompting, context engineering)
- **Chapter 12**: Python UV package manager (installation and environment setup)
- **Chapter 13**: Introduction to Python (2 lessons: What is Python + Installing Python)

**Assumed Knowledge from Chapter 13**:
- What Python is and why it's used for AI development
- Python 3.14+ installed and working
- How to run Python code
- `print()` function - basic usage from Hello World
- How to use Claude Code or Gemini CLI for code exploration
- Terminal basics (from Chapter 7 - Bash Essentials)

**Chapter 14 Builds On This By Teaching**:
- Variables and assignment (NEW - not in Chapter 13)
- Type hints syntax (NEW - introduces specification-first code)
- Core data types (int, float, str, bool, None)
- Type exploration through AI dialogue
- Type validation with `type()` and `isinstance()`

**NOT Yet Covered** (deferred to later chapters):
- `input()` function (requires f-strings - too advanced)
- Operators deep dive (Chapter 15)
- String methods (Chapter 16)
- Collections methods (Chapters 18-19)

**NOT Required** (taught in later chapters):
- **Chapter 15**: Operators (arithmetic, comparison, logical) - taught next chapter
- **Chapter 16**: Strings deep dive - Chapter 14 covers str basics only
- **Chapter 17**: Control flow (if/else, loops) - not yet covered
- **Chapter 18**: Lists, Tuples, Dictionary - Chapter 14 introduces concept only
- **Chapter 19**: Set, Frozenset - Chapter 14 mentions, Chapter 19 deep dive
- **Chapter 20**: Functions - not yet covered
- **Chapter 21**: Exception handling - not yet covered
- **Chapter 22**: File operations - not yet covered

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Variables with Type Hints (Priority: P1)

**As a Python beginner**, I want to store data in variables with type hints, so I can write clear, specification-first code building on my Hello World knowledge.

**Why this priority**: Foundation skill - variables with type hints enable all future Python work. Type hints introduce specification-first thinking from first variables.

**Independent Test**: Student can create variables (int, float, str, bool) with type hints and use print() (from Ch 13) to display values.

**Acceptance Scenarios**:

1. **Given** a student completed Hello World (Chapter 13), **When** they learn variables, **Then** they create `age: int = 25` to store data

2. **Given** a student sees type hints, **When** they ask AI "why add `: int` to variables?", **Then** they understand type hints as embedded specifications

3. **Given** a student creates multiple variables, **When** they use different types (int, float, str, bool), **Then** they see Python categorizes data

4. **Given** a student uses print(), **When** they display variable values, **Then** they build on Chapter 13 Hello World to show stored data

---

### User Story 2 - Understanding Data Types through Numbers (Priority: P1)

**As a learner**, I want to understand what a data type is by exploring int and float through AI dialogue, so I grasp "type as capability" concept.

**Why this priority**: Foundational concept - understanding "what is a type?" is critical before learning specific types.

**Independent Test**: Student can explain "what is a type?" in own words, use type() function, and choose int vs float appropriately.

**Acceptance Scenarios**:

1. **Given** a student asks "what is a data type?", **When** AI explains types as categories determining operations, **Then** they understand type concept

2. **Given** a student uses type() function, **When** they inspect different values, **Then** they see Python categorizes data

3. **Given** a student learns int and float, **When** they ask AI "when should I use int vs float?", **Then** they understand counting (int) vs measuring (float)

4. **Given** a student encounters TypeError, **When** they ask AI "why did this fail?", **Then** they understand type mismatch and use isinstance() to validate

---

### User Story 3 - Strings and Booleans Understanding (Priority: P2)

**As a learner**, I want to understand str (text basics), bool (True/False), None, and type conversion, so I can work with text and boolean logic in my programs.

**Why this priority**: Essential for real programs (text input/output, conditional logic) - builds on numeric foundation.

**Independent Test**: Student can create strings with type hints, understand truthy/falsy values, convert between types safely, and use None appropriately.

**Acceptance Scenarios**:

1. **Given** a student learns str basics, **When** they create strings with type hints, **Then** they understand `name: str = "Alex"` syntax (deep dive in Chapter 16)

2. **Given** a student explores bool, **When** they ask AI "why is 0 falsy but -1 truthy?", **Then** they understand boolean evaluation context

3. **Given** a student needs type conversion, **When** they ask AI "how do I convert user input (string) to integer?", **Then** they use int() and understand when it fails

4. **Given** a student sees None, **When** they ask AI "what is None and when should I use it?", **Then** they understand "no value" concept

---

### User Story 4 - Collection Awareness (Priority: P3)

**As a learner**, I want **conceptual introduction** to collections (list, tuple, dict, set), so I know they exist and when to learn them deeply.

**Why this priority**: Awareness only - deep coverage happens in Chapters 18-19. Not critical for Chapter 14 completion.

**Independent Test**: Student can create basic list, tuple, dict, set with type hints, and knows where to find comprehensive coverage.

**Acceptance Scenarios**:

1. **Given** a student sees collections, **When** they ask AI "what's the difference between list, tuple, dict, and set?", **Then** they understand high-level purpose of each

2. **Given** a student learns type hints for collections, **When** they write `scores: list[int]`, **Then** they understand collection type hint syntax

3. **Given** a student asks about collection methods, **When** they see "defer to Chapter 18-19", **Then** they know where comprehensive coverage is

**CRITICAL**: This story provides **awareness only**, not mastery. Full coverage in Chapters 18-19.

---

### User Story 5 - Building Type Explorer with Core Types (Priority: P1)

**As a learner**, I want to build an interactive type explorer applying core type concepts (int, float, str, bool, None), so I solidify understanding through hands-on practice.

**Why this priority**: Capstone project integrating all Chapter 14 core concepts - critical for mastery before Chapter 15.

**Independent Test**: Student completes working type explorer for core types with type hints, isinstance() validation, type conversion, and AI-guided extensions.

**Acceptance Scenarios**:

1. **Given** a student has learned core types, **When** they build type explorer program, **Then** they apply int, float, str, bool, None with type hints

2. **Given** a student needs type validation, **When** they use isinstance(), **Then** they check types before operations

3. **Given** a student handles user input, **When** they convert types with int(), float(), str(), **Then** they handle conversion failures with AI help

4. **Given** a student has working explorer, **When** they ask AI "how can I improve this?", **Then** they extend it with AI-suggested features

---

### Edge Cases

- **Empty Collections**: What happens when `list`, `dict`, `set` are empty? (Falsy in boolean context)
- **Type Conversion Failures**: When does `int("abc")` fail? How to handle gracefully?
- **Integer Division**: What's the difference between `/` (float division) and `//` (integer division)?
- **Float Precision**: Why does `0.1 + 0.2 != 0.3` in Python? (IEEE 754 floating point)
- **Complex Number Operations**: What operations work on complex numbers vs real numbers?
- **None Type**: When is `None` used and how does it behave in boolean contexts?
- **Truthy/Falsy Confusion**: Why is `[] == False` false but `if []:` doesn't execute?
- **Bytes vs String**: When should you use `bytes` instead of `str`?

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST cover **core data types**: int, float, str, bool, None with type hints (defer collections deep dive to Ch 18-19)

- **FR-002**: Every type MUST be taught using AIDD teaching pattern: **Concept → Code Idea → Try With AI → Reasoning Pattern**

- **FR-003**: Chapter MUST include "Try With AI" section for every lesson following standardized 4-prompt format (Rule 7)

- **FR-004**: Chapter MUST include troubleshooting prompts for common `TypeError` scenarios (Rule 6 pattern: "Ask AI when errors occur")

- **FR-005**: All code examples MUST use Python 3.14+ type hints and modern syntax (f-strings, `list[int]`, `dict[str, int]`)

- **FR-006**: Chapter MUST reinforce AI-Native Learning pattern from Chapters 1-11 (describe intent → explore with AI → validate → learn from errors)

- **FR-007**: Chapter MUST respect chapter boundaries:
  - No forward references to Chapters 15-29 (operators, control flow, functions, classes, exceptions, file I/O)
  - Defer strings deep dive to Chapter 16
  - Defer collections (list, tuple, dict) to Chapter 18
  - Defer sets (set, frozenset) to Chapter 19

- **FR-008**: Students MUST build interactive type explorer for **core types only** (int, float, str, bool, None)

- **FR-009**: Lessons MUST apply cognitive load limits (max 5-7 concepts per lesson for A2-B1 tier)

- **FR-010**: Chapter MUST teach `isinstance()` for type validation and `type()` for type inspection

- **FR-011**: Chapter MUST explain type conversion (casting) with `int()`, `str()`, `float()`, `bool()` and when conversions fail

- **FR-012**: Chapter MUST explain truthy/falsy values in boolean contexts (0, empty string, None)

- **FR-013**: Chapter MUST introduce collections (list, tuple, dict, set) at **awareness level only** with type hints (`list[int]`, `dict[str, float]`)

### Key Entities *(data types taught)*

**Core Types (Comprehensive Coverage)**:

- **Numeric Types**:
  - `int` (integers: whole numbers)
  - `float` (floating-point: decimal numbers)
  - `complex` (complex numbers: brief mention - real + imaginary)

- **Text Type**:
  - `str` (strings: text basics - methods in Chapter 16)

- **Boolean Type**:
  - `bool` (True/False values)
  - Truthy/falsy evaluation

- **None Type**:
  - `None` (represents "no value")

**Collections (Awareness Only - Deep Dive in Chapters 18-19)**:

- **List** (`list`): Mutable ordered collection - basic creation with type hints
- **Tuple** (`tuple`): Immutable ordered collection - basic creation with type hints
- **Dictionary** (`dict`): Key-value pairs - basic creation with type hints
- **Set** (`set`): Unique values - basic creation with type hints

**CRITICAL**: Collections taught at awareness level only (create basic examples, understand type hints). Comprehensive coverage (methods, operations, use cases) deferred to Chapters 18-19.

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Learning Success**:

- **SC-001**: 75%+ of students can explain "what is a data type" in their own words (quiz metric)

- **SC-002**: 80%+ of students successfully use `isinstance()` to validate types in exercises (submission accuracy)

- **SC-003**: 90%+ of students complete "Try With AI" prompts and extract useful type operations (engagement metric)

- **SC-004**: 85%+ of students can troubleshoot `TypeError` by asking AI and fixing the issue (exercise completion)

- **SC-005**: 100% of students build working interactive type explorer demonstrating all type concepts (capstone submission)

**Content Quality**:

- **SC-006**: Chapter maintains Grade 7-8 reading level (automated Flesch-Kincaid check)

- **SC-007**: All code examples use Python 3.14+ type hints and modern syntax (technical review)

- **SC-008**: Zero forward references to Chapters 15-29 concepts (spec validation)

- **SC-009**: Every lesson includes exactly 4 "Try With AI" prompts with expected outcomes (format validation)

**Engagement**:

- **SC-010**: 70%+ completion rate on "Try With AI" exercises (platform analytics)

- **SC-011**: Students spend average 2-3 hours on chapter content (engagement tracking)

- **SC-012**: 80%+ of students report understanding types better through AI dialogue than traditional documentation (post-chapter survey)

---

## Content Outline *(detailed lesson structure)*

**CRITICAL - UPDATED SCOPE**:
- **Chapter 13 now teaches**: What is Python + Installing Python ONLY (2 lessons)
- **Chapter 14 must teach**: First program, variables, print(), type hints, AND data types
- Deep dives into specific types still deferred:
  - **Chapter 16**: Strings deep dive (methods, formatting, manipulation)
  - **Chapter 18**: Lists, Tuples, Dictionary (comprehensive coverage with methods)
  - **Chapter 19**: Set, Frozenset (set operations, mathematical sets)

**Chapter 14 Scope (Expanded)**:
1. First Python program (Hello World)
2. Variables and assignment
3. Type hints syntax introduction
4. Core data types (int, float, str, bool, None)
5. Basic awareness of collections (list, tuple, dict, set)

---

### Lesson 1: Variables and Type Hints - Storing Data with Specifications

**What Students Learn**: Variables, assignment, type hints syntax (builds on Chapter 13's Hello World)

**Key Concepts** (max 6):
1. Variables - storing data with names
2. Assignment operator (`=`)
3. Type hints syntax (`age: int = 25`)
4. Why type hints matter (specification-first thinking)
5. Multiple variables with different types
6. AI as code exploration partner

**Code Example Specifications**:
- **First Variables** (5 lines): Creating variables without type hints first
- **Variables with Type Hints** (8 lines): Same variables with type hints added
- **Print Variables** (6 lines): Using print() from Ch 13 to display variable values

**Try With AI** (4 prompts):
1. "You learned print() in Chapter 13. Now explain what a variable is and why we need them."
2. "What does `age: int = 25` mean? Why do we add `: int` to the variable?"
3. "Show me how to create variables for different types of data (numbers, text, true/false). What types exist?"
4. "Type hints describe what data means (`age: int` says 'age is a number'). How does this help me and AI understand code better than `age = 25` alone?"

**Expected Outcomes**:
1. Student creates variables with meaningful names
2. Student writes type hints for clarity
3. Student uses print() (from Ch 13) to display variables
4. Student understands type hints as embedded specifications

**Note**: Builds on Chapter 13's Hello World - students already know print()

---

### Lesson 2: Understanding Data Types - Integers and Floats

**What Students Learn**: Type concept, int, float, type() function, isinstance()

**Key Concepts** (max 7):
1. **What is a data type?** (category of data determining operations)
2. **Why types matter** (safety, clarity, what you can do with data)
3. `type()` function for inspecting types
4. `int` - whole numbers (no decimal point)
5. `float` - numbers with decimal points
6. When to use int vs float (counting vs measuring)
7. `isinstance()` for type checking

**Code Example Specifications**:
- **Type Inspector** (6 lines): Using type() to see what type data is
- **Int vs Float** (10 lines): Examples showing when to use each type
- **Type Checker** (6 lines): Using isinstance(value, int) for validation

**Try With AI** (4 prompts):
1. "What is a data type in Python? Why does Python care about types instead of treating all numbers the same?"
2. "What's the difference between int and float? When should I use each?"
3. "Show me the type() function. What does it tell me about my data?"
4. "I got 'TypeError: unsupported operand type(s)'. What happened and how do I fix it?"

**Expected Outcomes**:
1. Student understands type concept (category + capability)
2. Student uses type() to inspect data
3. Student chooses int vs float appropriately
4. Student validates types with isinstance()

---

### Lesson 3: Strings and Booleans - Text and Truth

**What Students Learn**: str (text basics only - Chapter 16 has deep dive), bool (True/False), truthy/falsy

**Key Concepts** (max 7):
1. `str` - text data (basics: quotes, concatenation - methods in Chapter 16)
2. Type hints for strings (`name: str = "Alex"`)
3. `bool` - True/False values
4. Boolean evaluation (True, False)
5. Truthy/falsy values (0 is falsy, non-zero is truthy)
6. `None` type - represents "no value"
7. Type conversion: str(), int(), float(), bool()

**Code Example Specifications**:
- **String Basics** (8 lines): Creating strings, simple concatenation (defer methods to Ch 16)
- **Boolean Logic** (10 lines): True/False, truthy/falsy evaluation
- **Type Conversion** (12 lines): Converting between int, float, str, bool safely

**Try With AI** (4 prompts):
1. "Explain truthy and falsy values. Why is 0 falsy but -1 truthy? What about empty strings?"
2. "When does int('abc') fail and int('123') succeed? What's the conversion rule?"
3. "What is None and when should I use it? How does it behave in boolean contexts?"
4. "I need to convert user input (string) to a number. How do I do this safely?"

**Expected Outcomes**:
1. Student creates strings with type hints (basics only)
2. Student understands boolean evaluation
3. Student converts between types safely
4. Student understands None type

---

### Lesson 4: Introduction to Collections - Overview Only

**What Students Learn**: **Awareness** of list, tuple, dict, set (details deferred to Chapters 18-19)

**CRITICAL**: This lesson provides **conceptual introduction only**. Deep coverage happens later:
- **Chapter 18**: Lists, Tuples, Dictionary (comprehensive with all methods)
- **Chapter 19**: Set, Frozenset (set operations, mathematical operations)

**Key Concepts** (max 7):
1. **Why collections exist**: Grouping related data
2. `list` - mutable ordered collection (basics: `[1, 2, 3]` - full coverage Ch 18)
3. `tuple` - immutable ordered collection (basics: `(1, 2, 3)` - full coverage Ch 18)
4. `dict` - key-value pairs (basics: `{"name": "Alex"}` - full coverage Ch 18)
5. `set` - unique values (basics: `{1, 2, 3}` - full coverage Ch 19)
6. Type hints for collections (`scores: list[int]` - introduced here, used throughout)
7. **Defer to future chapters**: Methods, operations, deep usage

**Code Example Specifications**:
- **Collection Basics** (12 lines): Creating list, tuple, dict, set with type hints
- **Type Hints for Collections** (8 lines): `list[int]`, `dict[str, int]` syntax
- **Collection Awareness** (10 lines): When to use each (conceptual only)

**Try With AI** (4 prompts):
1. "What's the difference between list, tuple, dict, and set? Give me a simple example of when I'd use each."
2. "Show me type hints for collections. What does `list[int]` mean? What about `dict[str, float]`?"
3. "I see collections have many methods. Where will I learn those in detail?"
4. "How do I choose the right collection type for my data? What questions should I ask?"

**Expected Outcomes**:
1. Student understands collections exist for grouping data
2. Student can create basic list, tuple, dict, set
3. Student writes type hints for collections
4. Student knows detailed coverage is in Chapters 18-19

---

### Lesson 5: Building a Type Explorer - Hands-On Practice

**What Students Learn**: Apply core type concepts (int, float, str, bool, None) with type hints and validation

**Key Concepts** (max 6):
1. Integrating core types (int, float, str, bool, None)
2. Type hints throughout program
3. Type validation with isinstance()
4. Type conversion with error handling
5. User interaction demonstrating types
6. AI-guided program improvement

**Code Example Specifications**:
- **Type Explorer Program** (25-30 lines): Interactive program exploring core types
  - Prompt user for input
  - Demonstrate type() inspection
  - Show type conversion
  - Validate with isinstance()
  - Display type information
  - Include AI extension points (comments: "Ask AI to add...")

**Try With AI** (4 prompts):
1. "Review my type explorer code. Am I using type hints correctly? What types am I demonstrating?"
2. "How can I improve this program to explore more type operations?"
3. "Add type conversion error handling. What happens if user enters invalid data?"
4. "What did I learn about types by building this? How does this help me in Chapter 15 (Operators)?"

**Expected Outcomes**:
1. Student builds working type explorer for core types
2. Student uses type hints throughout
3. Student validates types and handles conversion errors
4. Student reflects on type understanding

**Note**: Capstone does NOT cover collections in depth - that's Chapters 18-19

---

## Acceptance Criteria *(validation checklist)*

**Content Completeness**:

- [ ] All Python built-in types covered (numeric, boolean, sequence, mapping, set, binary, None)
- [ ] Every lesson uses AIDD teaching pattern (Concept → Code → Try → Reasoning)
- [ ] Every lesson has exactly 4 "Try With AI" prompts with expected outcomes (Rule 7)
- [ ] Troubleshooting prompts for TypeError scenarios included (Rule 6)
- [ ] Interactive type explorer capstone included

**Technical Accuracy**:

- [ ] All code examples use Python 3.14+ type hints
- [ ] Modern syntax only (f-strings, `list[int]`, `X | None`)
- [ ] All code examples tested and working
- [ ] No deprecated or legacy patterns

**Pedagogical Compliance**:

- [ ] No forward references to Chapters 15-29 (functions, classes, exceptions, files)
- [ ] AI-Native Learning pattern reinforced (describe intent → explore with AI → validate → learn from errors)
- [ ] Cognitive load limits respected (max 7-8 concepts per lesson)
- [ ] Complexity tier A2-B1 maintained
- [ ] Grade 7-8 reading level maintained

**AIDD Integration**:

- [ ] Students use AI as type exploration partner (not documentation lookup)
- [ ] "Try With AI" prompts encourage dialogue, not syntax memorization
- [ ] Troubleshooting teaches "ask AI when errors occur" pattern
- [ ] Type validation (isinstance) taught as specification-first skill

**Success Evals Alignment**:

- [ ] Content enables 75%+ comprehension (clear type concept explanations)
- [ ] Content enables 80%+ isinstance() usage (validation exercises included)
- [ ] Content enables 90%+ AI type exploration (engaging "Try With AI" prompts)
- [ ] Content enables 85%+ error recovery (troubleshooting prompts included)
- [ ] Content enables 100% capstone completion (clear project requirements)

---

## Complexity Tier *(pedagogical level)*

**A2-B1 (Intermediate Beginners)**

**Audience**:
- Universal (beginners AND professionals new to Python)
- AIDD-first learners (not traditional syntax memorization)

**Cognitive Load**:
- Max 7-8 concepts per lesson
- 6 lessons total (progressive complexity)
- Each concept explained through AI dialogue

**Teaching Approach**:
- Concept-first (understand WHAT and WHY before HOW)
- AI partnership (explore through dialogue)
- Validation-first (check types before operations)
- Troubleshooting with AI (errors are learning opportunities)

**Expected Proficiency Levels** (CEFR-aligned):

| Lesson | Type Concepts | Proficiency Level | Measurable Skill |
|--------|---------------|-------------------|------------------|
| 1 | Type concept, type() | A2 | Can explain "what is a type" in own words |
| 2 | Numeric, boolean, isinstance() | A2-B1 | Can validate types and explain errors |
| 3 | Sequences (str, list, tuple, range) | B1 | Can choose appropriate sequence type for task |
| 4 | Mapping (dict), sets | B1 | Can use dict/set for real-world data |
| 5 | Type conversion, None | B1 | Can convert types safely and handle failures |
| 6 | Interactive type explorer | B1 | Can integrate all type concepts in working program |

---

## Assumptions *(documented defaults)*

**Teaching Environment**:
- Students have Claude Code or Gemini CLI installed and configured (from Chapters 5-6)
- Students can write and run Python programs (from Chapter 13)
- Students understand basic type hints syntax (from Chapter 13)

**Content Scope**:
- Binary types (bytes, bytearray, memoryview) covered at overview level only (deep dive deferred to advanced topics)
- Number systems (ASCII, hex, octal, binary) mentioned briefly, not deep dive (save for advanced Python internals)
- Integer interning and memory internals skipped entirely (Chapter 29 - CPython topic)
- File handling with binary data skipped (Chapter 22 - I/O topic)
- UTF-8 encoding details skipped (Chapter 16 - Strings or Chapter 22 - I/O)

**Code Examples**:
- All examples self-contained (no external dependencies)
- Examples use print() for output (from Chapter 13)
- Examples use input() for user interaction (from Chapter 13)
- No examples require functions, classes, or exception handling (future chapters)

**Student Responsibilities**:
- Read lesson content
- Execute code examples
- Complete "Try With AI" prompts (ask AI, understand responses)
- Build capstone type explorer
- NOT responsible for memorizing syntax (AI partnership handles that)

---

## Notes *(clarifications and context)*

**Why AIDD-First Approach for Types?**

Traditional Python teaching: "Here are 12 data types. Memorize their methods."

AIDD-first approach: "Types define what you can DO with data. Use AI to explore what each type enables."

**Example of the difference**:

**Traditional**:
> "Strings have 47 methods: `.upper()`, `.lower()`, `.split()`, `.join()`, ... [memorize all]"

**AIDD-First**:
> "Strings are text. Ask your AI: 'What can I do with text in Python?' Discover operations as you need them."

**Pedagogical Benefits**:
1. **Reduces cognitive load**: Explore on-demand vs memorize upfront
2. **Builds AI partnership**: Students learn to ask questions, not look up docs
3. **Focuses on concepts**: Understand "why types exist" before "how to use them"
4. **Mirrors real work**: Professional developers ask AI about types constantly

**Ruthless Context Filtering AND Chapter Boundary Respect**:

Chapter 14 focuses on **core type concepts** + **collection awareness**. Deep dives deferred to later chapters.

✅ **CHAPTER 14 SCOPE** (Comprehensive Coverage):
- Type concept (what is a type, why it matters)
- Core types: int, float, str (basics), bool, None
- Type hints syntax (`age: int`, `name: str`, `scores: list[int]`)
- type() function for inspection
- isinstance() for validation
- Type conversion (int(), str(), float(), bool())
- Truthy/falsy values

✅ **CHAPTER 14 SCOPE** (Awareness Only - Full Coverage Later):
- Collections introduction: list, tuple, dict, set (basic creation, type hints)
- Forward reference to Chapters 18-19 for deep dive

❌ **DEFERRED TO LATER CHAPTERS**:
- **Chapter 15**: Operators (arithmetic, comparison, logical) - defer operator deep dive
- **Chapter 16**: Strings deep dive (methods, formatting, slicing, manipulation)
- **Chapter 17**: Control flow (if/else, loops) - not yet taught
- **Chapter 18**: Lists, Tuples, Dictionary (comprehensive: methods, operations, use cases)
- **Chapter 19**: Set, Frozenset (set operations, mathematical operations, GC basics)
- **Chapter 20**: Functions - not yet taught
- **Chapter 21**: Exception handling - not yet taught
- **Chapter 22**: File handling - not yet taught

❌ **SKIPPED** (Too Advanced or Out of Scope):
- Integer literals and interning (advanced Python internals)
- Memory space sharing details (Chapter 29 - CPython and GIL)
- File handling examples (Chapter 22 - IO and File Handling)
- Number systems deep dive (ASCII, hex, octal, binary) - keep minimal
- UTF-8 encoding details (Chapter 16 or Chapter 22)
- Binary types (bytes, bytearray, memoryview) - too advanced for Chapter 14

**Critical Design Rules Applied**:

1. **User Intent is Authority**: User requested core types first (int, float, str, bool) + collection awareness → spec reflects this priority
2. **Chapter Boundary Respect**: Strings deep dive (Ch 16), Collections deep dive (Ch 18-19) - Chapter 14 provides foundational awareness
3. **No Forward References**: Zero mentions of operators (Ch 15), control flow (Ch 17), functions (Ch 20), classes (Ch 24+)
4. **Ruthless Filtering**: Only Chapter 14 scope from context materials - deferred advanced topics
5. **Minimal Files**: This spec only (plan.md and tasks.md come later)
6. **Troubleshooting is AI Partnership**: "Ask AI when TypeError occurs" pattern
7. **Standardized Try With AI Format**: Exactly 4 prompts per lesson with expected outcomes

**Key Scope Decision**: Chapter 14 teaches **type concept + core types comprehensively** + **collection awareness**. This enables students to:
- Understand "what is a type and why it matters" (foundation)
- Work with int, float, str, bool, None confidently (immediate utility)
- Know collections exist and use type hints (`list[int]`) (future preparation)
- Be ready for Chapter 15 (Operators) with solid type foundation

---

## Next Steps

After spec approval:

1. **Run /sp.plan**: Break spec into detailed lesson-by-lesson implementation plan with CEFR proficiency levels
2. **Run /sp.tasks**: Generate implementation task checklist with acceptance criteria
3. **Optional - Run content-implementer**: Implement lessons with AIDD teaching pattern applied
4. **Run validation-auditor**: Validate technical accuracy and constitution alignment

**Spec Ready For**: Planning phase (`/sp.plan`)

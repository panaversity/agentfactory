# Feature Specification: Chapter 14 - Data Types (Beginner-Friendly Redesign)

**Feature Branch**: `013-chapter-14-redesign`
**Created**: 2025-01-15 (V2 - Corrected Scope)
**Status**: Draft
**Input**: "Rewrite Chapter 14 (Data Types) applying beginner-friendly pedagogy: explain WHAT each data type is and WHY you'd use it BEFORE showing syntax. Assume Chapter 13 taught print(), variables, basic syntax. Focus on Python's type system from context/13_chap12_to_29_specs/Lesson_02_Data_Types.md."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Beginner Understands Data Type Concepts Before Syntax (Priority: P1)

A learner (completed Chapter 13, knows variables/print()) needs to understand Python's type system.

**Current Problem**: Chapter 14 shows `num_int: int = 42` without explaining WHAT `int` means (whole numbers) or WHY you'd choose `int` over `float`.

**Desired Outcome**: Before seeing ANY code syntax, learner understands:
- **WHAT** a data type is (Python's classification system for data)
- **WHY** types matter (different operations for different types)
- **The 7 type categories**: Numeric (int/float/complex), Text (str), Boolean (bool), Collections (list/dict/tuple/set/range), Binary (bytes/bytearray/memoryview), Mapping (dict), Special (None)
- **Decision framework**: "What kind of data?" → type choice

**Why P1**: Without conceptual understanding, beginners memorize `int`/`float`/`str` as keywords, not as meaningful classifications. This is foundational.

**Independent Test**: Give learners 10 scenarios (age, price, email, yes/no, list of scores, image data) and ask them to choose correct type WITH REASONING. Success = 80%+ correct with explanations.

**Acceptance Scenarios**:

1. **Given** "What is a Data Type?" explanation, **When** asked, **Then** learner says "how Python classifies different kinds of data"
2. **Given** type categories, **When** shown age=25, price=19.99, name="Alice", **Then** learner explains int (whole #), float (decimal), str (text)
3. **Given** "Why types matter," **When** asked "Why can't 5 + 'hello' work?", **Then** learner explains types determine valid operations
4. **Given** type decision framework, **When** seeing new data, **Then** learner asks "Number? Text? True/False? Collection?" to determine type

---

### User Story 2 - Learner Masters Numeric Types with Clear Comparisons (Priority: P2)

After understanding type concepts, learner needs to distinguish int, float, complex.

**Current Problem**: Shows `int = 42`, `float = 3.14`, `complex = 2+3j` without explaining WHEN to use each or WHY Python has 3 numeric types.

**Desired Outcome**: Learner understands:
- **int**: Whole numbers (age, count, index). No decimals. Examples: 25, -5, 0
- **float**: Decimals (price, measurements, percentages). Examples: 19.99, 3.14, -0.5
- **complex**: Real + imaginary (scientific/engineering). Examples: 2+3j. Rare for beginners.
- **When to use which**: Age → int. Price → float. Complex rarely needed.

**Independent Test**: Learner correctly categorizes 15 numbers into int/float/complex and explains reasoning. Success = 90%+ correct.

**Acceptance Scenarios**:

1. **Given** int explanation, **When** asked "age = 25.5 or 25?", **Then** learner chooses int=25 (ages are whole numbers)
2. **Given** float explanation, **When** storing price, **Then** learner uses float for $19.99 (needs decimals)
3. **Given** complex explanation, **When** asked "Do beginners need complex?", **Then** learner says "rarely, mainly for science/engineering"
4. **Given** type() function, **When** running `type(42)`, **Then** learner sees `<class 'int'>` and understands Python's classification

---

### User Story 3 - Learner Understands All Python Types at Appropriate Depth (Priority: P3)

After numeric/bool/str basics, learner surveys Python's complete type system.

**Content Scope** (from Lesson_02_Data_Types.md):
- **Sequence Types**: str (text), list (mutable collection), tuple (immutable), range (number sequence)
- **Set Types**: set (unique, mutable), frozenset (unique, immutable)
- **Mapping**: dict (key-value pairs)
- **Binary Types**: bytes, bytearray, memoryview (file/network data)
- **None Type**: Represents absence of value
- **Advanced Concepts**: type(), id(), isinstance(), type casting, truthy/falsy, integer interning, number systems (binary/hex/octal/ASCII)

**Depth Strategy**:
- **Deep dive**: int, float, str, bool, None (beginners use daily)
- **Awareness**: list, dict, tuple, set (intro here, deep dive Chapter 18)
- **Brief exposure**: bytes, complex, frozenset, range (know they exist)
- **Advanced marked**: integer interning, number systems (for curious learners)

**Independent Test**: Learner can classify 20 data examples into correct types and knows which chapters cover deep dives. Success = 75%+ correct classification.

**Acceptance Scenarios**:

1. **Given** collection awareness, **When** seeing `[1, 2, 3]`, **Then** learner recognizes list but knows "Chapter 18 for details"
2. **Given** type casting, **When** needing int from "25", **Then** learner uses `int("25")`
3. **Given** truthy/falsy, **When** asked "Is empty list True or False?", **Then** learner says False (empty = falsy)
4. **Given** isinstance(), **When** checking type, **Then** learner uses `isinstance(age, int)`

---

### Edge Cases

- **What if learner skips concept explanations and jumps to syntax?** Include forward references: "Confused why int vs float? See 'When to Use Each Type' above"
- **What if learner is overwhelmed by 13 different types?** Use progressive disclosure: Core 5 types first (int/float/str/bool/None), collections awareness, advanced brief
- **How to handle type() vs isinstance() confusion?** Explain type() shows classification, isinstance() checks membership
- **What about type hints vs type enforcement?** Clarify Python doesn't enforce type hints (dynamic typing), they're documentation/tooling aids

## Requirements *(mandatory)*

### Functional Requirements

**Conceptual Foundations (BEFORE Syntax)**
- **FR-001**: Chapter 14 MUST begin with "What is a Data Type?" explaining Python's classification system
- **FR-002**: Chapter 14 MUST explain WHY types matter (different operations, data validation, memory efficiency)
- **FR-003**: Chapter 14 MUST present the 7 type categories overview BEFORE individual type details

**Numeric Types**
- **FR-004**: Chapter 14 MUST explain WHAT int is (whole numbers) and WHEN to use it (ages, counts, indices)
- **FR-005**: Chapter 14 MUST explain WHAT float is (decimals) and WHEN to use it (prices, measurements)
- **FR-006**: Chapter 14 MUST explain WHAT complex is (real+imaginary) and mark it "Advanced/Scientific"
- **FR-007**: Chapter 14 MUST show type() function to inspect types (e.g., `type(42)` → `<class 'int'>`)

**Text and Boolean**
- **FR-008**: Chapter 14 MUST explain str as text/sequence of characters with quote variations (single, double, triple)
- **FR-009**: Chapter 14 MUST explain bool as True/False for yes/no decisions

**Collections (Awareness Level)**
- **FR-010**: Chapter 14 MUST introduce list, tuple, dict, set, range with "deep dive in Chapter 18" messaging
- **FR-011**: Chapter 14 MUST show syntax examples but NOT deep methods (save for Chapter 18)

**Binary Types (Brief Exposure)**
- **FR-012**: Chapter 14 MUST briefly introduce bytes, bytearray, memoryview for file/network data
- **FR-013**: Binary types MUST be marked "Advanced" with example use cases (image files, network protocols)

**None Type**
- **FR-014**: Chapter 14 MUST explain None as representing absence of value (not 0, not empty string)
- **FR-015**: Chapter 14 MUST explain None as singleton (only one None object in Python)

**Type Utilities**
- **FR-016**: Chapter 14 MUST teach type() for inspecting types
- **FR-017**: Chapter 14 MUST teach id() for object identity
- **FR-018**: Chapter 14 MUST teach isinstance() for type checking
- **FR-019**: Chapter 14 MUST teach type casting with int(), float(), str(), bool()
- **FR-020**: Chapter 14 MUST explain implicit vs explicit type casting

**Advanced Concepts**
- **FR-021**: Chapter 14 MUST explain truthy/falsy values with examples (0, "", [], {}, None = False)
- **FR-022**: Chapter 14 MUST explain integer interning (-5 to 256 range) marked "Advanced/Curious Learners"
- **FR-023**: Chapter 14 MUST introduce number systems (binary, hex, octal, ASCII) marked "Advanced"

**Pedagogical Requirements**
- **FR-024**: ALL type explanations MUST follow pattern: WHAT it is → WHY use it → WHEN to choose → Code syntax
- **FR-025**: Code examples MUST appear AFTER conceptual explanations
- **FR-026**: Chapter 14 MUST reference Chapter 13 for print()/variables (don't re-teach)
- **FR-027**: Chapter 14 MUST use type hints in ALL examples to normalize the practice
- **FR-028**: Exercises MUST progress: type identification → type casting → real-world scenarios

### Key Entities

- **Data Type**: Python's classification system for data (numeric, text, boolean, collections, binary, None)
- **int**: Whole numbers without decimals (ages, counts, indices). Range: unlimited
- **float**: Numbers with decimal points (prices, measurements, scientific). IEEE 754 standard
- **complex**: Numbers with real + imaginary parts (scientific/engineering). Format: a+bj
- **str**: Text/sequence of characters. Immutable. Quote variations: '', "", ''', """
- **bool**: True or False values for yes/no decisions
- **list**: Ordered, mutable collection. Deep dive in Chapter 18
- **tuple**: Ordered, immutable collection. Deep dive in Chapter 18
- **dict**: Key-value pairs (mapping type). Deep dive in Chapter 18
- **set**: Unordered, unique values, mutable. Deep dive in Chapter 19
- **frozenset**: Unordered, unique values, immutable. Deep dive in Chapter 19
- **range**: Immutable sequence of numbers. Used in loops (Chapter 17)
- **bytes**: Immutable binary data (files, networks)
- **bytearray**: Mutable binary data
- **memoryview**: Efficient binary data access without copying
- **None**: Special type representing absence of value. Singleton object
- **type()**: Built-in function returning data type classification
- **id()**: Built-in function returning object's unique identifier
- **isinstance()**: Built-in function checking if object is instance of type
- **Type Casting**: Converting data from one type to another (int(), float(), str(), bool())

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 85% of learners can explain "what is a data type" in their own words after reading concept section
- **SC-002**: 80% of learners correctly classify 10 real-world data scenarios (age, price, email, yes/no) into appropriate types WITH REASONING
- **SC-003**: 90% of learners can distinguish int vs float: "age=25 (int), price=19.99 (float)" with correct reasoning
- **SC-004**: 75% of learners can identify truthy/falsy: 0, "", [], {}, None are False; non-zero/non-empty are True
- **SC-005**: 85% of learners can use type() to inspect types and understand output `<class 'int'>`
- **SC-006**: 80% of learners can perform type casting: convert "25" to int, 25 to str, 0 to bool
- **SC-007**: 70% of learners recognize collections (list, dict) but know "Chapter 18 for deep dive"
- **SC-008**: Learners report 40% less memorization (more conceptual understanding) vs current Chapter 14
- **SC-009**: 90% of learners complete exercises without asking "Why int vs float?" (concept was explained first)
- **SC-010**: 75% of learners can explain WHY types matter: "different types enable different operations"

## Assumptions *(optional)*

### Prerequisites
- Chapter 13 completed: Learners know print(), variables, assignment, type hints syntax
- Learners understand basic programming concepts (variables store data, functions do tasks)
- Learners have Python 3.14+ installed (Chapter 12)
- No prior knowledge of data types required

### Content Scope
- Chapter 14 focuses on TYPE SYSTEM (what types exist, when to use them)
- Does NOT cover: control flow (Chapter 17), functions (Chapter 20), deep collection methods (Chapter 18)
- Advanced topics (integer interning, number systems) included but marked "Advanced/Curious"

### Pedagogical Approach
- Concept-before-syntax: Explain WHAT/WHY/WHEN before showing code
- Progressive disclosure: Core types deep → collections awareness → advanced brief
- Real-world examples: age=int, price=float, email=str (not abstract x/y/z)
- Type hints normalized: Every example uses type hints

## Scope Boundaries *(optional)*

### In Scope

✅ Conceptual understanding of Python's type system
✅ Numeric types: int, float, complex (with WHEN to use each)
✅ Text type: str (quote variations, immutability)
✅ Boolean type: bool (True/False)
✅ Collections awareness: list, tuple, dict, set, range (syntax only, methods in Chapter 18)
✅ Binary types brief: bytes, bytearray, memoryview (marked Advanced)
✅ None type (singleton, represents absence)
✅ type(), id(), isinstance() functions
✅ Type casting: int(), float(), str(), bool() (implicit vs explicit)
✅ Truthy/falsy values
✅ Integer interning (marked Advanced)
✅ Number systems (binary, hex, octal, ASCII) (marked Advanced)
✅ Real vs imaginary parts of complex numbers
✅ UTF-8 encoding concept

### Out of Scope (Other Chapters)

❌ print() / variables / basic syntax → Chapter 13
❌ Control flow (if/else, loops) → Chapter 17
❌ Functions (def, parameters, return) → Chapter 20
❌ Deep collection methods (list.append(), dict.get()) → Chapter 18
❌ String methods (.split(), .join(), .upper()) → Chapter 16
❌ Set operations (union, intersection) → Chapter 19
❌ File I/O with binary data → Chapter 22
❌ Advanced type hints (Union, Optional, TypeVar) → Chapter 27
❌ Object-oriented programming → Chapters 24-25

### Explicitly NOT Included

❌ Arithmetic operators (+, -, *, /) → Chapter 15
❌ Comparison operators (==, !=, <, >) → Chapter 15
❌ Boolean operators (and, or, not) → Chapter 15
❌ String formatting (f-strings, format()) → Chapter 16
❌ List indexing/slicing → Chapter 18
❌ Dictionary comprehensions → Chapter 18
❌ Lambda functions → Chapter 20
❌ Decorators → Later chapters

## Dependencies *(optional)*

### Prerequisites
- **Chapter 12**: Python UV installed, Python 3.14+ working
- **Chapter 13**: print(), variables, type hints syntax, basic programs

### Related Content
- **Chapter 15**: Operators (will use types from Chapter 14)
- **Chapter 16**: Strings deep dive (builds on str from Chapter 14)
- **Chapter 17**: Control flow (uses bool from Chapter 14)
- **Chapter 18**: Collections deep dive (list, dict, tuple, set intro'd here)
- **Chapter 19**: Set operations (builds on set/frozenset from Chapter 14)
- **Chapter 27**: Advanced type hints (builds on type hint basics)

## Open Questions *(optional)*

None. All pedagogical decisions have clear defaults.

---

## Notes for Implementation

### Recommended Lesson Structure

**Lesson 1: Understanding Data Types (Concepts First)**
- What is a Data Type? (Python's classification system)
- Why Types Matter (operations, validation, efficiency)
- The 7 Type Categories overview
- Type Decision Framework ("What kind of data?")

**Lesson 2: Numeric Types (int, float, complex)**
- WHAT each is, WHY use it, WHEN to choose
- type() function
- Code examples AFTER concepts
- Common mistakes (25.5 in int, integer division)

**Lesson 3: Text, Boolean, and None**
- str (quotes, immutability)
- bool (True/False, truthy/falsy)
- None (singleton, absence)
- Practice exercises

**Lesson 4: Collections Awareness + Binary Types**
- list, tuple, dict, set, range (syntax only, "Chapter 18 for deep dive")
- bytes, bytearray, memoryview (marked Advanced)
- When you'll need each

**Lesson 5: Type Utilities + Advanced Concepts**
- type(), id(), isinstance()
- Type casting (int(), float(), str(), bool())
- Integer interning (Advanced)
- Number systems (Advanced)
- Capstone project: Type Explorer

### Key Pedagogical Principles

1. **Concept-before-syntax**: WHAT/WHY/WHEN before code examples
2. **Real-world scenarios**: age/price/email, not x/y/z
3. **Progressive disclosure**: Core 5 types deep → collections awareness → advanced brief
4. **Normalize type hints**: Use in every example
5. **Reference Chapter 13**: Don't re-teach print()/variables
6. **Mark advanced clearly**: Integer interning, number systems = "For Curious Learners"
7. **Forward references**: "Confused? See 'When to Use Each Type' above"
8. **Exercises progress**: Identify types → Cast types → Real-world scenarios

### Quality Checklist

- [ ] Concepts explained BEFORE syntax for every type
- [ ] WHAT/WHY/WHEN pattern used for int, float, str, bool
- [ ] type() function introduced early
- [ ] Collections shown but "Chapter 18 for deep dive" messaging
- [ ] Advanced topics marked "Advanced" or "For Curious Learners"
- [ ] Real-world examples (not abstract x/y/z)
- [ ] Type hints in ALL examples
- [ ] No re-teaching of Chapter 13 content (print, variables)
- [ ] Exercises test conceptual understanding, not memorization
- [ ] Success criteria are measurable

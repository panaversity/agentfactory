# Chapter Specification: Strings and Type Casting

**Feature Branch**: `016-ch16-strings-keywords-typecasting`
**Created**: 2025-11-08
**Status**: Draft
**Chapter**: 16 (Part 4: Python Fundamentals)
**Complexity Tier**: Beginner (A1-A2)
**CEFR Range**: A1-A2 (max 5 concepts per lesson)

## Success Evals *(FIRST - Business-Goal-Aligned)*

### Comprehension Evals

- **EVAL-001**: 80%+ of students can explain difference between strings and other data types after Lesson 1
- **EVAL-002**: 75%+ can correctly predict output of string operations (upper, lower, split, join) in quiz
- **EVAL-003**: 85%+ can identify when type casting is needed vs when it happens automatically

### Skill Acquisition Evals

- **EVAL-004**: 70%+ write valid string manipulation code using 5+ essential methods in capstone
- **EVAL-005**: 80%+ correctly convert between int/float/str/bool types without errors
- **EVAL-006**: 75%+ validate their string operations and type conversions using isinstance() and type()

### Engagement Evals

- **EVAL-007**: 85%+ completion rate for all 4 lessons (measured by "Try With AI" submissions)
- **EVAL-008**: 70%+ complete Interactive String Explorer capstone project

### Accessibility Evals

- **EVAL-009**: Reading level at Grade 7-8 (Flesch-Kincaid automated check)
- **EVAL-010**: No forward references to concepts beyond Chapter 16 scope

## Topic Summary

Chapter 16 teaches students how to work with text data (strings) and convert between different data types safely. This chapter builds on Chapter 14's introduction to data types by focusing specifically on string operations and type casting—two foundational skills needed for processing user input, formatting output, and building interactive programs.

Students learn to create strings, manipulate them with 5-7 essential methods, format output using f-strings, and convert between core scalar types (int, float, str, bool). The chapter emphasizes validation-first thinking: students describe their intent using type hints, explore string operations with their AI companion, validate results, and learn from errors.

The capstone project—Interactive String Explorer—demonstrates all concepts in a hands-on tool that validates and formats user input while demonstrating type conversions.

## Prerequisites

**Explicit Dependencies**:

- **Chapter 1-11**: AIDD fundamentals, AI-Native Learning pattern (describe intent → explore → validate → learn from errors)
- **Chapter 12**: Python UV package manager (Python environment setup)
- **Chapter 13**: Python basics (running code, basic syntax, print statements)
- **Chapter 14**: Data types (int, float, str, bool, None) and type hints
- **Chapter 15**: Variables, operators, and basic input/output

**Assumed Knowledge**:

- Students can run Python code using Claude Code or Gemini CLI
- Students understand what variables are and how to use type hints
- Students can use basic operators (+, *, ==) with different types
- Students know how to ask their AI companion questions about concepts

## Learning Objectives

**After completing this chapter, students will be able to:**

1. **LO-001**: Create and manipulate strings using 5-7 essential methods (upper, lower, split, join, replace, find, strip) with confidence
2. **LO-002**: Format strings using f-strings to create dynamic output from variables
3. **LO-003**: Convert between core scalar types (int ↔ float ↔ str ↔ bool) safely and validate conversions
4. **LO-004**: Describe intent using type hints (str, int, float, bool) and validate string operations with isinstance() and type()
5. **LO-005**: Build an Interactive String Explorer that demonstrates string operations and type casting with error handling

## Content Outline

### Lesson 1: String Fundamentals (A1 Level)

**Key Concepts** (5 max):

1. What strings are (sequences of characters)
2. Creating strings (single quotes, double quotes, triple quotes)
3. String immutability (strings cannot be changed after creation)
4. Basic string operations (concatenation with +, repetition with *)
5. String length and indexing basics

**Teaching Pattern**:

- Book teaches string creation and basic operations directly
- Students practice with AI companion for edge cases
- Validation: Use len(), type(), isinstance() to check understanding

**AI CoLearning Elements**:

- 💬 AI Colearning Prompt: "Explain what 'immutable' means for strings. Why can't we change a string after creating it?"
- 🎓 Instructor Commentary: "In AI-native development, understanding data immutability helps you design better data flows"
- 🚀 CoLearning Challenge: "Ask AI to show what happens when you try to modify a string character"
- ✨ Teaching Tip: "Use your AI to explore why Python chose to make strings immutable"

### Lesson 2: Essential String Methods (A2 Level)

**Key Concepts** (5 max):

1. Case transformation (upper(), lower())
2. String splitting and joining (split(), join())
3. Finding and replacing (find(), replace())
4. Whitespace handling (strip(), lstrip(), rstrip())
5. Method chaining (applying multiple methods in sequence)

**Teaching Pattern**:

- Book teaches 5-7 essential methods with clear examples
- AI companion helps explore method combinations
- Students describe intent ("I want to clean and capitalize user input") → AI suggests approach

**AI CoLearning Elements**:

- 💬 "How does split() work under the hood? Why does it return a list?"
- 🎓 "Syntax is cheap—semantics is gold. Understand WHAT methods do, not memorize syntax"
- 🚀 "Ask AI to generate examples combining 3+ string methods for text processing"
- ✨ "When you forget a method name, ask AI 'How do I remove whitespace from a string?'"

### Lesson 3: String Formatting with F-Strings (A2 Level)

**Key Concepts** (5 max):

1. F-string syntax (f"Hello, {name}!")
2. Embedding variables in strings
3. Formatting numbers ({value:.2f} for 2 decimal places)
4. Combining strings with expressions
5. When to use f-strings vs concatenation

**Teaching Pattern**:

- Book teaches f-string basics directly
- AI companion handles complex formatting patterns (students specify intent)
- Tier 2 (Complex Syntax): AI handles advanced formatting, student directs

**AI CoLearning Elements**:

- 💬 "Why are f-strings better than older formatting methods (% and .format())?"
- 🎓 "F-strings let you describe your output intent clearly—exactly the skill needed for AI-native development"
- 🚀 "Tell your AI: 'Create an f-string that formats a price with 2 decimals and currency symbol'"
- ✨ "For complex formatting, describe what you want and let AI handle the syntax"

### Lesson 4: Type Casting Fundamentals (A2-B1 Level)

**Key Concepts** (5 max):

1. Implicit vs explicit type casting
2. Converting strings to numbers (int("123"), float("3.14"))
3. Converting numbers to strings (str(42), str(3.14))
4. Boolean conversions (bool(""), bool("text"))
5. Handling conversion errors safely

**Teaching Pattern**:

- Book teaches core scalar type conversions directly
- Students validate conversions with isinstance() and type()
- AI companion helps troubleshoot conversion errors

**AI CoLearning Elements**:

- 💬 "Explain why int('3.14') fails but float('3.14') works"
- 🎓 "Type casting is about data transformation—a core pattern in all programming"
- 🚀 "Ask AI to show what happens when you convert invalid strings to numbers"
- ✨ "When you get ValueError, ask AI 'Why did this conversion fail and how do I fix it?'"

### Lesson 5: Capstone - Interactive String Explorer (B1 Level)

**Project**: Build a hands-on tool that:

1. Accepts user text input (with type validation)
2. Demonstrates all essential string methods
3. Shows type conversions between str/int/float/bool
4. Handles errors gracefully (invalid conversions)
5. Provides formatted output using f-strings

**Capstone Goals**:

- Integration of Lessons 1-4 concepts (not 5 NEW concepts)
- Real-world application: text processing and validation
- AI partnership: Student describes intent, AI helps implement, student validates
- Error literacy: Handle and learn from conversion errors

**AI CoLearning Elements**:

- 💬 "How would you structure this program to handle all the different operations?"
- 🎓 "Real programs combine concepts—this capstone shows how strings and type casting work together"
- 🚀 "Ask AI to help design the user interaction flow for your String Explorer"
- ✨ "When you get stuck, describe what you're trying to achieve and ask AI for suggestions"

## User Scenarios & Testing

### User Story 1 - String Manipulation Mastery (Priority: P1)

As a beginner Python learner, I want to confidently create and manipulate strings so I can process text data in my programs.

**Why this priority**: Strings are fundamental to nearly every program (user input, output, data processing). Without string skills, students can't build interactive programs.

**Independent Test**: Can be fully tested by having student complete exercises manipulating strings (uppercase conversion, splitting sentences, joining words) and delivers immediate value for building text-based programs.

**Acceptance Scenarios**:

1. **Given** a string "hello world", **When** student applies upper() method, **Then** result is "HELLO WORLD" and student understands strings are immutable
2. **Given** a sentence string, **When** student uses split(), **Then** result is a list of words and student can explain what happened
3. **Given** a list of words, **When** student uses join(), **Then** result is a single string with specified separator
4. **Given** a string with whitespace, **When** student uses strip(), **Then** whitespace is removed and student can validate using len()

---

### User Story 2 - Dynamic Output Formatting (Priority: P2)

As a student building interactive programs, I want to format output dynamically using f-strings so my programs can display personalized, readable messages to users.

**Why this priority**: Dynamic output is essential for user-friendly programs. Students need this to display variables, format numbers, and create readable output.

**Independent Test**: Student creates formatted output displaying variables, numbers with specific decimal places, and combined text/expressions—delivers value for any program with output.

**Acceptance Scenarios**:

1. **Given** variables name="Alice" and age=25, **When** student creates f"Hello, {name}! You are {age}.", **Then** output is "Hello, Alice! You are 25."
2. **Given** a price variable 19.5, **When** student formats as f"${price:.2f}", **Then** output is "$19.50"
3. **Given** multiple variables, **When** student combines them in one f-string, **Then** output is readable and correctly formatted

---

### User Story 3 - Safe Type Conversions (Priority: P1)

As a Python learner processing user input, I want to safely convert between types (int/float/str/bool) so my programs can handle different data correctly without crashing.

**Why this priority**: User input is always strings. Converting to numbers is essential for calculations. This is a core skill needed immediately.

**Independent Test**: Student converts string input to numbers, performs calculations, converts back to strings for output—delivers value for any program with numeric input/output.

**Acceptance Scenarios**:

1. **Given** string "42", **When** student converts to int using int("42"), **Then** result is integer 42 and can be used in calculations
2. **Given** string "3.14", **When** student converts to float using float("3.14"), **Then** result is float 3.14
3. **Given** number 99, **When** student converts to string using str(99), **Then** result is string "99" and can be concatenated
4. **Given** invalid string "abc", **When** student attempts int("abc"), **Then** ValueError occurs and student learns to validate input first

---

### User Story 4 - Type Validation (Priority: P3)

As a student learning AI-Native development, I want to validate data types using isinstance() and type() so I can check my understanding and debug type-related errors.

**Why this priority**: Validation is core to AI-Native Learning pattern (describe → explore → validate → learn from errors). Less critical than core skills but important for good practices.

**Independent Test**: Student uses isinstance() and type() to check string and number types, validates conversions, and understands when to use each—delivers value for debugging and learning.

**Acceptance Scenarios**:

1. **Given** variable x = "hello", **When** student checks isinstance(x, str), **Then** result is True and student understands type checking
2. **Given** converted number y = int("42"), **When** student checks type(y), **Then** result is <class 'int'> and student validates conversion succeeded
3. **Given** mixed variables, **When** student validates types before operations, **Then** student avoids type errors

---

### Edge Cases

- What happens when converting strings with whitespace to numbers (int(" 42 ") vs int("42"))?
- How does the system handle empty strings in split() and join()?
- What occurs when finding a substring that doesn't exist (find() returns -1)?
- How are boolean conversions handled for empty strings vs non-empty strings (bool("") is False, bool("text") is True)?
- What happens when concatenating strings vs adding numbers ("1" + "2" vs 1 + 2)?

## Requirements

### Functional Requirements

- **FR-001**: Chapter MUST teach string creation using single quotes, double quotes, and triple quotes (multiline)
- **FR-002**: Chapter MUST cover 5-7 essential string methods only (upper, lower, split, join, replace, find, strip)
- **FR-003**: Chapter MUST teach f-string formatting as the primary method (no %, no .format())
- **FR-004**: Chapter MUST teach type casting for core scalar types only (int ↔ float ↔ str ↔ bool)
- **FR-005**: Chapter MUST include validation techniques (isinstance(), type()) throughout all lessons
- **FR-006**: Chapter MUST apply AI-Native Learning pattern in all lessons (describe intent → explore → validate → learn from errors)
- **FR-007**: All lessons MUST end with "Try With AI" section ONLY (4 prompts, no summaries after)
- **FR-008**: Capstone MUST integrate concepts from Lessons 1-4 without introducing 5 NEW concepts
- **FR-009**: Chapter MUST use Python 3.14+ syntax and modern type hints throughout
- **FR-010**: Chapter MUST stay within A1-A2 cognitive load limits (max 5 concepts per lesson)
- **FR-011**: Chapter MUST NOT include: 40+ string methods, regex, string interning, collection conversions, escape sequences deep dive
- **FR-012**: Chapter MUST apply Graduated Teaching Pattern (Book teaches foundational → AI handles complex → student validates)
- **FR-013**: All code examples MUST include type hints and demonstrate clear intent
- **FR-014**: Chapter MUST use Part 4 appropriate language (NO "write specifications", use "describe intent")

### Key Entities

- **String**: Immutable sequence of characters; key operations: upper, lower, split, join, replace, find, strip; formatting via f-strings
- **Type Conversion**: Transformation between int, float, str, bool; validation via isinstance() and type()
- **String Method**: Operation on strings (transformation or query); returns new string or value (immutability)
- **F-String**: Formatted string literal allowing variable embedding and expression evaluation

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students can correctly manipulate strings using 5+ essential methods in under 5 minutes (measured via capstone exercise timing)
- **SC-002**: 80%+ of students successfully convert between str/int/float/bool types without errors in quiz
- **SC-003**: 75%+ complete Interactive String Explorer capstone demonstrating all chapter concepts
- **SC-004**: 85%+ can explain when to use upper vs lower vs strip vs split in interview-style questions
- **SC-005**: 70%+ can debug type conversion errors by asking AI targeted questions (measured via error resolution exercises)
- **SC-006**: Reading comprehension at Grade 7-8 level (Flesch-Kincaid automated check passes)
- **SC-007**: Chapter completion rate 85%+ (all 4 lessons + capstone)
- **SC-008**: 80%+ report feeling confident with strings and type casting in post-chapter survey

## Scope Boundaries

### In Scope

✅ String creation (single, double, triple quotes)
✅ Essential string methods (upper, lower, split, join, replace, find, strip)
✅ F-string formatting with variables and basic number formatting
✅ Core scalar type casting (int ↔ float ↔ str ↔ bool)
✅ Type validation (isinstance(), type())
✅ Error handling for invalid conversions (ValueError)
✅ Interactive String Explorer capstone project
✅ AI-Native Learning pattern throughout (describe → explore → validate → learn)

### Out of Scope

❌ Advanced string methods (40+ methods like capitalize, casefold, center, etc.)
❌ Regular expressions (regex patterns)
❌ String interning and memory optimization
❌ Collection type conversions (list ↔ tuple ↔ set ↔ dict) - Chapter 18-19
❌ Complex type (complex numbers)
❌ Escape sequences deep dive (focus on practical f-strings)
❌ String encoding and Unicode details
❌ Performance optimization of string operations
❌ Formal Specification-Driven Development (that's Part 5+)

## Assumptions

1. **Environment**: Students have Python 3.14+ installed via Chapter 12 (Python UV)
2. **AI Tools**: Students have access to Claude Code or Gemini CLI as co-reasoning partners
3. **Prior Knowledge**: Students completed Chapters 1-15 (AIDD fundamentals, Python basics, data types)
4. **Learning Style**: Students are comfortable with AI-Native Learning pattern (describe intent, explore, validate, learn from errors)
5. **Cognitive Load**: A1-A2 beginners can handle max 5 new concepts per lesson when concepts are well-explained with practice
6. **Teaching Pattern**: Book teaches foundational concepts directly; AI handles complex syntax; student validates all outputs
7. **Lesson Format**: All lessons end with "Try With AI" section ONLY (no summaries or checklists after)
8. **Code Standards**: All examples use type hints, f-strings, Python 3.14+ syntax

## Constraints

- **Cognitive Load**: Maximum 5 new concepts per lesson (beginner A1-A2 tier)
- **Lesson Count**: 4 foundational lessons + 1 capstone (total 5 lessons max)
- **Method Coverage**: Only 5-7 essential string methods (not all 40+)
- **Type Casting Scope**: Core scalar types only (int, float, str, bool)—no collections
- **Python Version**: 3.14+ (modern type hints: str, int, float, bool, list[T], dict[K,V])
- **No Forward References**: Cannot mention concepts from Chapters 17+ (loops, functions, classes, etc.)
- **Language Appropriateness**: Part 4 language (describe intent, not write specifications)
- **Lesson Closure**: "Try With AI" section ONLY—no summaries, checklists, or "what's next"
- **Reading Level**: Grade 7-8 (Flesch-Kincaid check)
- **AI CoLearning**: Must include 💬🎓🚀✨ elements throughout all lessons (not just at end)

## Dependencies

**Prerequisites**:

- Chapter 14 (Data Types): Students understand int, float, str, bool, None and basic type hints
- Chapter 15 (Operators, Keywords, Variables): Students can use operators and variables

**Future Chapters (do not reference)**:

- Chapter 17 (Control Flow and Loops): Will use string operations in conditionals and loops
- Chapter 18-19 (Collections): Will teach collection conversions (list ↔ tuple ↔ set)
- Chapter 20 (Functions): Will use string parameters and return values
- Chapter 30+ (Spec-Driven Development): Formal specification methodology

## Non-Goals

- Teaching all 40+ string methods (only 5-7 essential)
- Regular expressions or pattern matching
- String interning and memory optimization
- Collection type conversions (covered in Ch 18-19)
- Unicode and character encoding details
- Performance benchmarking of string operations
- String theory deep dives (just practical usage)
- Formal specification writing (that's Part 5+)

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Students overwhelmed by too many string methods | High - Cognitive overload for A1-A2 learners | Limit to 5-7 essential methods; defer advanced methods to future chapters |
| Type conversion errors cause frustration | Medium - Could discourage beginners | Teach validation-first approach; use isinstance() and type() extensively; show error handling patterns |
| F-strings vs older methods confusion | Low - Clear if only f-strings taught | Use ONLY f-strings; skip % and .format() entirely; explain why f-strings are modern standard |
| Scope creep (adding collections, regex, etc.) | High - Violates cognitive load limits | Ruthless filtering: Chapter title is "Strings and Type Casting"—stick to scope; mark out-of-scope explicitly |
| Forward references to loops/functions | Medium - Violates prerequisites | Review all examples to ensure no loops, functions, or Chapter 17+ concepts mentioned |

## Notes

**Pedagogical Approach**:

- This chapter applies Constitution Principle 13 (Graduated Teaching Pattern):
  - **Tier 1** (Foundational): Book teaches string basics, type concepts directly
  - **Tier 2** (Complex): AI handles complex formatting patterns (students specify intent)
  - **Tier 3** (Orchestration): Not applicable at A1-A2 level

**AI-Native Learning Integration**:

- Every lesson includes CoLearning elements (💬🎓🚀✨) throughout content
- "Try With AI" section at end of each lesson (4 prompts, Bloom's progression)
- Students learn WITH AI, not just USING AI
- AI positioned as co-reasoning partner, not autocomplete tool

**Alignment with Chapter Index**:

- Chapter title "Strings and Type Casting" is THE ANCHOR—all content must match
- File name: `16-strings-type-casting/` (from chapter-index.md)
- Part 4, Position 16 in sidebar

**Quality Assurance**:

- Technical review will validate A1-A2 proficiency levels
- Cognitive load check (max 5 concepts per lesson)
- Constitutional alignment (AI-Native Learning pattern, lesson closure, no forward references)
- Code quality (Python 3.14+, type hints, f-strings only)

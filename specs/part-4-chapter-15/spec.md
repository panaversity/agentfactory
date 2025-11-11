# Feature Specification: Chapter 15 - Operators, Keywords, and Variables

**Feature Branch**: `015-operators-keywords-variables`
**Created**: 2025-11-08
**Status**: Draft
**Input**: User description: "Write Chapter 15: Operators, Keywords, and Variables in Part 4"

**Chapter Context**:
- **Part**: 4 (Python Fundamentals - Chapters 12-29)
- **Complexity Tier**: Beginner (A1-A2 CEFR)
- **Prerequisites**: Chapters 1-14 (especially Chapter 14: Data Types)
- **Learning Pattern**: AI-Native Learning (describe intent → explore → validate → learn from errors)

---

## Success Criteria (Evals-First) *(mandatory)*

### Measurable Outcomes

These evaluation criteria connect directly to student learning goals and must be achieved for chapter success:

- **SC-001**: **Comprehension Eval** - 75%+ of students can explain what each of the 4 operator types does in plain language (no jargon required)
  - *Business Goal Connection*: Students who can explain concepts clearly demonstrate true understanding vs. memorization
  - *Measurement*: Post-lesson quiz with open-ended questions; manual review of 20% sample for coherence

- **SC-002**: **Skill Acquisition Eval** - 80%+ of students write code using arithmetic, comparison, logical, and assignment operators correctly without errors on first attempt
  - *Business Goal Connection*: Practical coding ability indicates job-readiness and reduces frustration
  - *Measurement*: Exercise completion rate tracking in learning platform; automated test pass rate

- **SC-003**: **Validation Eval** - 70%+ of students identify incorrect operator usage and explain why it's wrong (e.g., type mismatches, division by zero)
  - *Business Goal Connection*: Debugging skills are essential professional competency; reduces dependency on others
  - *Measurement*: "Find the bug" exercises with student explanations; rubric-scored for accuracy

- **SC-004**: **Keywords Eval** - 90%+ of students recognize reserved words from a list and avoid using them as variable names
  - *Business Goal Connection*: Prevents common syntax errors that frustrate beginners; demonstrates attention to language rules
  - *Measurement*: Multiple-choice quiz on keyword recognition; code review of student exercises for keyword misuse

- **SC-005**: **AI Partnership Eval** - 75%+ of students ask AI productive "what happens if" questions about operator behavior successfully
  - *Business Goal Connection*: AI co-learning partnership is the core methodology; students must learn to explore concepts with AI
  - *Measurement*: Prompt quality assessment (structured rubric); student self-report of useful AI responses

- **SC-006**: **Engagement Eval** - 80%+ of students complete all lessons in Chapter 15 (not just first lesson)
  - *Business Goal Connection*: Chapter completion indicates appropriate difficulty and engaging content
  - *Measurement*: Learning platform analytics showing lesson completion rates

- **SC-007**: **Accessibility Eval** - Content maintains Grade 7-8 reading level (Flesch-Kincaid) for non-native English speakers
  - *Business Goal Connection*: Global audience requires accessible English; reading level predicts comprehension
  - *Measurement*: Automated readability scoring on all lesson content

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Learn to Perform Math Operations (Priority: P1)

**As a beginner Python learner**, I want to **understand and use arithmetic operators (+, -, *, /, //, %, **)** so that **I can perform calculations in my programs**.

**Why this priority**: Arithmetic is the most foundational and immediately useful operator category. Students can write calculator-like programs from day 1. This is the "hello world" of data manipulation.

**Independent Test**: Can be fully tested by having students write a simple calculator that performs all 7 arithmetic operations and validates results by asking AI "Is this result correct?"

**Acceptance Scenarios**:

1. **Given** I have two numbers (e.g., 10 and 3), **When** I use the + operator, **Then** I get 13 and understand it adds values
2. **Given** I have two numbers (e.g., 10 and 3), **When** I use the - operator, **Then** I get 7 and understand it subtracts
3. **Given** I have two numbers (e.g., 10 and 3), **When** I use the * operator, **Then** I get 30 and understand it multiplies
4. **Given** I have two numbers (e.g., 10 and 3), **When** I use the / operator, **Then** I get 3.333... (float) and understand it divides
5. **Given** I have two numbers (e.g., 10 and 3), **When** I use the // operator, **Then** I get 3 (integer) and understand it does floor division
6. **Given** I have two numbers (e.g., 10 and 3), **When** I use the % operator, **Then** I get 1 (remainder) and understand modulus
7. **Given** I have two numbers (e.g., 10 and 3), **When** I use the ** operator, **Then** I get 1000 (10 cubed) and understand exponentiation
8. **Given** I have int and float types, **When** I perform arithmetic operations, **Then** I can use type() to verify result types and ask AI why types change

---

### User Story 2 - Make Decisions with Comparisons (Priority: P2)

**As a beginner Python learner**, I want to **understand and use comparison operators (==, !=, >, <, >=, <=)** so that **I can compare values and make decisions in my programs**.

**Why this priority**: Comparisons are essential for all conditional logic (covered in Ch 17). This chapter introduces the concept without requiring if/else statements yet - students just see True/False results.

**Independent Test**: Can be fully tested by having students compare different values and predict/verify True/False results, then ask AI to explain edge cases like comparing int to float.

**Acceptance Scenarios**:

1. **Given** I have two values, **When** I use == operator, **Then** I get True if equal, False otherwise
2. **Given** I have two values, **When** I use != operator, **Then** I get True if not equal, False if equal
3. **Given** I have two numbers, **When** I use > or < operators, **Then** I get True/False based on magnitude comparison
4. **Given** I have two numbers, **When** I use >= or <= operators, **Then** I get True/False including equality case
5. **Given** I compare int to float (e.g., 5 == 5.0), **When** using == operator, **Then** I understand Python compares values not types
6. **Given** I compare different types (e.g., "5" == 5), **When** using == operator, **Then** I get False and understand type matters for equality

---

### User Story 3 - Combine Conditions with Logic (Priority: P3)

**As a beginner Python learner**, I want to **understand and use logical operators (and, or, not)** so that **I can combine multiple conditions for complex decision-making**.

**Why this priority**: Logical operators build on comparisons to enable more sophisticated reasoning. They're essential for Chapter 17 (Control Flow) but are introduced here to keep cognitive load manageable.

**Independent Test**: Can be fully tested by having students combine comparison results using and/or/not and predict outcomes, then verify with Python and ask AI about truth table behavior.

**Acceptance Scenarios**:

1. **Given** I have two boolean values, **When** I use the `and` operator, **Then** I get True only if both are True
2. **Given** I have two boolean values, **When** I use the `or` operator, **Then** I get True if at least one is True
3. **Given** I have a boolean value, **When** I use the `not` operator, **Then** I get the opposite value
4. **Given** I combine comparisons (e.g., `x > 5 and x < 10`), **When** evaluating the expression, **Then** I understand it checks both conditions
5. **Given** I have nested logical operations (e.g., `not (x > 5 or y < 3)`), **When** evaluating, **Then** I can work with AI to understand evaluation order

---

### User Story 4 - Update Variables Efficiently (Priority: P4)

**As a beginner Python learner**, I want to **understand and use assignment operators (=, +=, -=, *=, /=)** so that **I can update variable values concisely without repetition**.

**Why this priority**: Assignment operators make code more readable and efficient. They're commonly used in loops (Ch 17) and are easier to learn now while students are focused on operators, rather than mixing with loop syntax later.

**Independent Test**: Can be fully tested by having students start with a variable, apply shorthand operators, and verify results match the expanded form (e.g., x += 5 is the same as x = x + 5).

**Acceptance Scenarios**:

1. **Given** I have a variable `x = 10`, **When** I use `x += 5`, **Then** x becomes 15 and I understand it's equivalent to `x = x + 5`
2. **Given** I have a variable, **When** I use `-=`, `*=`, `/=` operators, **Then** I understand they're shorthand for the corresponding arithmetic operation
3. **Given** I need to increment a counter, **When** I use `count += 1`, **Then** I recognize this is more concise than `count = count + 1`
4. **Given** I update a variable with shorthand, **When** I ask AI to show the expanded form, **Then** I verify they produce identical results

---

### User Story 5 - Recognize Reserved Words (Priority: P5)

**As a beginner Python learner**, I want to **understand what Python keywords are and why I can't use them as variable names** so that **I avoid syntax errors and understand Python's reserved words**.

**Why this priority**: Keywords are foundational knowledge that prevents common errors. This is "learn once, apply forever" content - best taught early before students develop bad habits.

**Independent Test**: Can be fully tested by showing students the keyword list, having them attempt to use a keyword as a variable name (seeing the error), and asking AI to explain why keywords are reserved.

**Acceptance Scenarios**:

1. **Given** I see the Python keyword list (import keyword; print(keyword.kwlist)), **When** reviewing the list, **Then** I recognize common words like `if`, `for`, `while`, `def`, `class`
2. **Given** I try to create a variable named `for`, **When** running the code, **Then** I get a SyntaxError and understand why
3. **Given** I need to name a variable related to a keyword, **When** choosing a name, **Then** I use alternatives like `for_loop` or `user_class` instead of reserved words
4. **Given** I'm unsure if a word is a keyword, **When** I ask AI or check keyword.kwlist, **Then** I can verify before using it

---

### Edge Cases

- **Division by zero**: What happens when a student uses `/` or `//` with 0 as divisor? (ZeroDivisionError - students should ask AI to explain)
- **Type mixing in operations**: What happens when adding int + str (e.g., 5 + "hello")? (TypeError - opportunity to discuss type safety)
- **Float precision**: What happens with 0.1 + 0.2? (Result is 0.30000000000000004 - students ask AI why floating point arithmetic has precision issues)
- **Boolean operators with non-boolean values**: What happens with `5 and 10` or `0 or "hello"`? (Truthiness concept - deferred to Ch 17, but AI can explain if students ask)
- **Operator precedence**: What happens with `2 + 3 * 4`? (Result is 14, not 20 - students should ask AI about order of operations)
- **Keyword case sensitivity**: What happens with `For` or `IF` instead of `for`/`if`? (Not keywords - Python is case-sensitive, but still bad practice)

---

## Requirements *(mandatory)*

### Functional Requirements

**Operators Coverage:**

- **FR-001**: Chapter MUST teach arithmetic operators (+, -, *, /, //, %, **) with clear examples showing each operator's behavior
- **FR-002**: Chapter MUST teach comparison operators (==, !=, >, <, >=, <=) with examples demonstrating True/False results
- **FR-003**: Chapter MUST teach logical operators (and, or, not) with examples showing boolean logic
- **FR-004**: Chapter MUST teach assignment operators (=, +=, -=, *=, /=) with examples showing shorthand equivalence to full expressions
- **FR-005**: All operator examples MUST use type hints consistently (e.g., `x: int = 10`, `result: float = x / 3`)

**Keywords Coverage:**

- **FR-006**: Chapter MUST explain what Python keywords are (reserved words with special meaning)
- **FR-007**: Chapter MUST show how to retrieve the keyword list using `import keyword; print(keyword.kwlist)`
- **FR-008**: Chapter MUST demonstrate what happens when attempting to use a keyword as a variable name (SyntaxError)
- **FR-009**: Chapter MUST explain WHY keywords are reserved (language syntax requires them for control flow, definitions, etc.)

**AI-Native Learning Pattern:**

- **FR-010**: Each lesson MUST include "Try With AI" section with exactly 4 progressive prompts
- **FR-011**: Each "Try With AI" prompt MUST have an explicit "Expected outcome" describing what students learn
- **FR-012**: Prompts MUST encourage exploration ("What happens if...?"), not just fact retrieval ("What is...?")
- **FR-013**: Prompts MUST include validation practice (e.g., "Is this result correct?" to build verification habits)
- **FR-014**: All lessons MUST end with "Try With AI" section ONLY - no summaries, key takeaways, or checklists after

**Lesson Structure:**

- **FR-015**: Each lesson MUST start with "What it is" explanation (plain language, 2-3 sentences)
- **FR-016**: Each lesson MUST include "Code Idea" section with minimal executable examples
- **FR-017**: Code examples MUST be progressive (start simple, add complexity gradually within lesson)
- **FR-018**: Maximum 5 concepts per lesson (respecting A1-A2 cognitive load limits)

**Content Constraints:**

- **FR-019**: Chapter MUST NOT include identity operators (is, is not) - deferred to advanced chapters covering memory concepts
- **FR-020**: Chapter MUST NOT include membership operators (in, not in) - requires collections knowledge from Ch 18-19
- **FR-021**: Chapter MUST NOT include bitwise operators (~, &, |, ^) - advanced topic requiring binary understanding
- **FR-022**: Chapter MUST NOT include walrus operator (:=) - advanced Python 3.8+ syntax
- **FR-023**: Chapter MUST NOT include chained comparisons in depth - mentioned as edge case only, not taught as core concept
- **FR-024**: Chapter MUST NOT include detailed PEP 8 naming conventions - complex rules handled by AI companion (Tier 2)
- **FR-025**: Chapter MUST NOT include `del` keyword or memory management concepts - advanced topics
- **FR-026**: Chapter MUST NOT reference Chapters 30+ or use "Specification-Driven Development" terminology (Part 5+ content)
- **FR-027**: Chapter MUST use Part 4 appropriate language: "describe your intent" NOT "write specifications"

**Python Standards:**

- **FR-028**: All code examples MUST run on Python 3.14+ (latest stable release)
- **FR-029**: All code examples MUST use modern type hints (e.g., `list[int]` not `List[int]`, `X | None` not `Optional[X]`)
- **FR-030**: All code examples MUST use f-strings for string formatting (not % or .format())
- **FR-031**: Code examples MUST NOT use eval(), shell=True, or hardcode secrets (security requirements)
- **FR-032**: Code examples MUST validate user input where appropriate (demonstrating safe coding habits)

**Capstone Project:**

- **FR-033**: Chapter MUST include a final lesson with hands-on "Calculator with Type Safety" project
- **FR-034**: Capstone project MUST demonstrate all 4 operator categories in one cohesive program
- **FR-035**: Capstone MUST include type validation (using type() or isinstance()) to teach verification habits
- **FR-036**: Capstone MUST be achievable by students at A1-A2 proficiency level (beginner-friendly scope)

### Key Entities *(Educational Content Structure)*

- **Operator**: A symbol that performs an operation on one or more values (operands)
  - **Categories**: Arithmetic, Comparison, Logical, Assignment
  - **Properties**: Symbol, purpose, example usage, result type
  - **Relationships**: Operators combine with data types from Chapter 14 to produce results

- **Keyword**: A reserved word in Python with special syntactic meaning
  - **Properties**: Word string, purpose in language, error if misused as identifier
  - **Examples**: if, for, while, def, class, import, return, True, False, None
  - **Relationships**: Keywords cannot be used as variable/function names (syntactic constraint)

- **Operand**: A value or variable that an operator acts upon
  - **Types**: Literal values (5, 3.14, "hello"), variables (x, count, total), expressions (x + 5)
  - **Type Safety**: Operands must be compatible with operator (can't add int + str without conversion)

- **Expression**: A combination of operands and operators that evaluates to a value
  - **Examples**: `5 + 3`, `x > 10`, `not (a and b)`, `total += price`
  - **Evaluation**: Follows operator precedence rules (PEMDAS for arithmetic)

---

## Success Criteria *(already defined above - moved to top per evals-first principle)*

See "Success Criteria (Evals-First)" section at the beginning of this specification.

---

## Assumptions

**About Students' Prior Knowledge:**

1. Students have completed Chapter 14 (Data Types) and understand int, float, str, bool, None, and type hints
2. Students can create variables with type hints (e.g., `x: int = 10`)
3. Students have access to Claude Code or Gemini CLI for AI-Native Learning exercises
4. Students can run Python code in an interpreter or IDE

**About Learning Context:**

1. Students are learning Python as their first programming language (beginner-friendly examples)
2. Students will practice in an interactive Python environment (REPL or notebook)
3. Lessons are completed sequentially (operator types build on each other)
4. Students have 3.5-4 hours total for this chapter (standard chapter time budget)

**About Content Delivery:**

1. Each lesson is designed for ~45-50 minutes of active learning
2. "Try With AI" exercises are completed with Claude Code/Gemini CLI assistance
3. Students validate their understanding by testing code and asking AI questions
4. Capstone project is completed after mastering the 4 core operator lessons

**Default Design Decisions** (no clarification needed):

1. **Operator Sequence**: Arithmetic → Comparison → Logical → Assignment (logical progression from simple math to complex logic to updates)
2. **Example Numbers**: Use small integers (10, 3, 5) for clarity and easy mental verification
3. **Type Hints**: Show in all examples to reinforce Chapter 14 learning and build good habits
4. **Error Handling**: Demonstrate errors as learning opportunities (e.g., division by zero) rather than avoiding them
5. **AI Prompts**: Progressive complexity (concept → application → edge case → synthesis) within each "Try With AI" section

---

## Out of Scope

**Content Explicitly Excluded from Chapter 15:**

- Identity operators (is, is not) and memory concepts (id()) - requires understanding of object references (Chapter 24+)
- Membership operators (in, not in) - requires collection types (Chapter 18-19)
- Bitwise operators (~, &, |, ^, <<, >>) - requires binary number system understanding (advanced topic)
- Walrus operator (:=) - advanced syntax for inline assignment
- Chained comparisons in depth (e.g., `10 < x < 20`) - mentioned as edge case but not taught systematically
- Operator overloading (__add__, __eq__, etc.) - OOP concept from Chapter 24-25
- Detailed PEP 8 style guide - comprehensive coverage deferred to later chapters, AI handles complex rules
- Variable naming convention exhaustive guide - students learn basics, AI companion handles edge cases
- `del` keyword and garbage collection - memory management is advanced (Chapter 19)
- Ternary operator (x if condition else y) - advanced syntax, not essential for beginners
- Augmented assignment for all operators (%=, //=, **=, etc.) - cover most common (+=, -=, *=, /=), mention others exist

**Teaching Approaches Excluded:**

- Formal "specification writing" language - this is Part 4, students use "describe intent" framing, not SDD terminology from Part 5+
- References to Chapters 30+ (Specification-Driven Development) - forward references violate beginner-friendly design
- Academic operator theory (unary vs. binary classification) - too theoretical, focus on practical usage
- Complex precedence rules in depth - basic PEMDAS mentioned, AI handles complex cases
- Truthiness and falsy values in depth - brief mention, full coverage in Chapter 17 (Control Flow)

**Implementation Details Excluded (Non-Functional):**

- Specific IDE or editor setup - students already have environment from Chapter 12-13
- Debugging tools or techniques - systematic debugging taught in later chapters
- Performance optimization of operators - premature for beginners
- Historical context of Python syntax - focus on current Python 3.14+

---

## Dependencies

**Hard Dependencies** (must be completed before Chapter 15):

- **Chapter 14: Data Types** - Students must understand int, float, str, bool, None, and type hints to use operators on these types
- **Chapter 13: Introduction to Python** - Students must know how to run Python code and see output
- **Chapter 12: Python UV** - Students must have Python installed and working

**Soft Dependencies** (helpful context but not blocking):

- **Chapters 1-11 (AIDD Fundamentals)** - Familiarity with AI-Native Learning pattern (describe → explore → validate → learn from errors)
- **Chapter 9: Markdown** - Helps students document their operator exploration in readable format
- **Chapter 10-11: Prompt & Context Engineering** - Makes "Try With AI" exercises more effective

**Future Dependencies** (chapters that build on Chapter 15):

- **Chapter 17: Control Flow and Loops** - Uses comparison and logical operators extensively in if/while/for statements
- **Chapter 18-19: Collections** - Membership operators (in, not in) taught there, building on Chapter 15 foundation
- **Chapter 20: Functions** - Uses operators within function bodies, assumes operator fluency
- **Chapter 24-25: OOP** - Operator overloading and custom operator behavior builds on understanding from Chapter 15

---

## Constraints

**Pedagogical Constraints:**

- **Cognitive Load**: Maximum 5 new concepts per lesson (A1-A2 CEFR beginner limit)
- **Chapter Length**: 4 foundational lessons + 1 capstone lesson (5 lessons total) - standard chapter structure
- **Time Budget**: 3.5-4 hours total chapter time (45-50 minutes per lesson)
- **Reading Level**: Grade 7-8 Flesch-Kincaid (accessible to non-native English speakers)

**Content Constraints:**

- **Python Version**: Must use Python 3.14+ syntax and features only
- **Type Hints**: Must appear in all code examples (reinforce Chapter 14, build professional habits)
- **No Forward References**: Cannot mention concepts from Chapters 16+ unless prerequisite relationship is unavoidable
- **Part 4 Language**: Use "AI-Native Learning" framing, NOT "Specification-Driven Development" (Part 5+ terminology)

**Lesson Structure Constraints:**

- **Closure Pattern**: ALL lessons must end with "Try With AI" section ONLY (constitutional mandate)
- **Prompt Count**: Exactly 4 prompts per lesson in "Try With AI" section (progressive complexity)
- **Expected Outcomes**: Each prompt must have explicit "Expected outcome" text (sets learning targets)
- **No Post-Closure Content**: Zero summaries, key takeaways, or checklists after "Try With AI" (violates cognitive load principles)

**Technical Constraints:**

- **Security**: No eval(), exec(), shell=True, or hardcoded secrets in any examples
- **Safety**: All code examples must be runnable without errors (except intentional error demonstrations)
- **Portability**: Code must work on Windows, macOS, and Linux without modification

**AI Tooling Constraints:**

- **Availability**: Assumes students have access to Claude Code OR Gemini CLI (not both required)
- **Prompting**: "Try With AI" exercises must work with both Claude and Gemini (no tool-specific prompts)
- **Validation**: Students must be able to validate AI responses by running code in their Python environment

---

## Risks & Mitigations

**Risk 1: Students confuse comparison operators with assignment**
- **Impact**: High - leads to bugs like `if x = 5:` instead of `if x == 5:`
- **Probability**: High - extremely common beginner mistake
- **Mitigation**: Explicitly contrast `=` (assignment) vs `==` (comparison) with side-by-side examples. Use clear error message when mistake is made. Include in "Try With AI" prompts.

**Risk 2: Division by zero not understood as error condition**
- **Impact**: Medium - students see ZeroDivisionError and panic, think they "broke" Python
- **Probability**: Medium - will occur when experimenting
- **Mitigation**: Demonstrate division by zero intentionally in lesson, normalize errors as learning opportunities. "Try With AI" prompt: "I got ZeroDivisionError. What does this mean and when might it happen in real programs?"

**Risk 3: Float precision surprises students**
- **Impact**: Medium - confusion when `0.1 + 0.2 != 0.3` exactly
- **Probability**: High - will occur during exploration
- **Mitigation**: Include in edge cases. "Try With AI" prompt: "Why does 0.1 + 0.2 give 0.30000000000000004?" Explain floating point limitation, defer deep dive to later chapters.

**Risk 4: Operator precedence confusion**
- **Impact**: Medium - wrong calculation results (e.g., `2 + 3 * 4` misunderstood)
- **Probability**: Medium - students from math backgrounds may handle well, others struggle
- **Mitigation**: Mention PEMDAS/BODMAS in arithmetic lesson. Encourage parentheses for clarity. "Try With AI" prompt: "Ask AI to explain order of operations and when to use parentheses."

**Risk 5: Too many operators overwhelms beginners**
- **Impact**: High - cognitive overload leads to drop-off
- **Probability**: Medium - 4 operator categories + keywords = potentially too much
- **Mitigation**: Strict 5-concept-per-lesson limit enforced. Spread operators across 4 separate lessons. Capstone reinforces without introducing new concepts. Skills proficiency mapping validates cognitive load.

**Risk 6: Students skip "Try With AI" exercises**
- **Impact**: High - miss core AI-Native Learning practice, fall back to memorization
- **Probability**: Medium - exercises optional, students may skip if pressed for time
- **Mitigation**: Make "Try With AI" visually distinct and explicitly state "This is not optional - AI partnership is how professionals learn." Track completion in platform analytics (SC-006).

**Risk 7: Type hint confusion from Chapter 14 not resolved**
- **Impact**: High - students copy type hints without understanding, creating bugs
- **Probability**: Medium - type hints are advanced for beginners
- **Mitigation**: Chapter 14 prerequisite validation. Quick review in Chapter 15 Lesson 1. Use type() in operator examples to verify result types. "Try With AI" prompt: "Why does this expression return float even though both operands are int?"

---

## Related Documents

- **Constitution**: `.specify/memory/constitution.md` (v3.0.2) - AI-Native Learning principles, graduated complexity guidelines
- **Chapter Index**: `specs/book/chapter-index.md` - Chapter 15 position in Part 4, prerequisite tracking
- **Chapter 14 Spec**: `specs/part-4-chapter-14/spec.md` - Data Types (prerequisite understanding)
- **Chapter 17 Spec** (future): Control Flow and Loops - will build on comparison/logical operators from Chapter 15
- **Output Style Guide**: `.claude/output-styles/lesson.md` - Lesson formatting requirements
- **Spec Template**: `.specify/templates/spec-template.md` - This specification follows this structure

---

## Notes

**Design Philosophy:**

This chapter represents a critical transition point: students move from **knowing data types exist** (Chapter 14) to **doing things with data** (Chapter 15). Operators are the verbs in Python's language - they make data active and useful.

The 4-operator-category structure follows a natural progression:
1. **Arithmetic** - most familiar (everyone knows 2 + 2)
2. **Comparison** - introduces True/False results (prepare for Chapter 17's if statements)
3. **Logical** - combines boolean values (AND/OR/NOT logic needed for complex conditions)
4. **Assignment** - efficiency upgrade (shorthand builds on arithmetic)

Keywords are included as the 5th concept because:
- They're orthogonal to operators (different concern)
- They're "learn once, remember forever" content
- Preventing keyword misuse now saves debugging time later
- Fits cognitive load limit (5 concepts total)

**AI-Native Learning Application:**

Operators are PERFECT for AI partnership because:
- **Exploratory learning**: "What happens if I divide int by int?" → AI explains why result is float
- **Edge case discovery**: "What happens with 0.1 + 0.2?" → AI explains floating point precision
- **Validation practice**: "Is this result correct?" → AI confirms or corrects, building verification habits
- **Conceptual deepening**: "Why does Python have both / and //?" → AI explains use cases for each

The "Try With AI" prompts in this chapter are designed to build confidence in using AI as a co-reasoning partner, not just a code generator. Students learn to ASK GOOD QUESTIONS - a meta-skill more valuable than memorizing operator tables.

**Ruthless Filtering Rationale:**

Many traditional Python tutorials teach 6-8 operator categories (arithmetic, comparison, logical, assignment, identity, membership, bitwise, special). This creates cognitive overload and most categories aren't used by beginners.

By filtering to 4 core categories + keywords, we:
- Respect A1-A2 cognitive load limits (5 concepts max)
- Teach what students will actually use in Chapters 16-20
- Defer advanced operators (is, in, bitwise) until students have the prerequisite knowledge (memory models, collections, binary math)
- Enable depth over breadth (students truly understand 4 categories rather than memorizing 8 superficially)

Identity operators (is, is not) are ESPECIALLY important to exclude because:
- They require understanding object identity vs. equality (Chapter 24+ OOP concept)
- They confuse beginners who think "is" is "equals" (it's not - it's memory location comparison)
- They're rarely needed in beginner code (== is almost always correct)
- Teaching them now would require explaining memory addresses and references - completely out of scope

**Validation Strategy:**

Every lesson includes type() usage to verify result types. This serves multiple purposes:
1. Reinforces Chapter 14's type hint learning
2. Builds validation-first thinking (never assume, always verify)
3. Reveals surprising behaviors (int / int → float, not int // int)
4. Demonstrates Python introspection capabilities
5. Prepares for isinstance() in later chapters

Students should develop the reflex: "Write code → Run it → Check type → Ask AI if surprised"

This validation-first mindset is professional development practice and core to AI-Native Learning.

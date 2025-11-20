# Chapter 15: Operators, Keywords, and Variables — Task Checklist

**Chapter Type**: Technical / Code-Focused
**Status**: Ready for Development
**Feature Branch**: `015-operators-keywords-variables`
**Spec Reference**: `specs/part-4-chapter-15/spec.md`
**Plan Reference**: `specs/part-4-chapter-15/plan.md`
**Owner**: [To be assigned]
**Estimated Effort**: 45–55 story points (approximately 36–44 hours of work)
**Target Timeline**: 2 weeks (1 week per 2–2.5 lessons, allowing for revision cycles)

---

## Overview & Success Criteria

This chapter requires creating 5 lessons teaching four operator categories (arithmetic, comparison, logical, assignment) plus keyword awareness. Each lesson follows the pattern: **What It Is → Code Examples → Try With AI**. No additional closure sections (summaries, key takeaways, checklists) appear after "Try With AI".

**Definition of Done** (for entire chapter):
- [ ] All 5 lessons created with consistent structure
- [ ] Each lesson has exactly 4 "Try With AI" prompts (20 total) with expected outcomes documented
- [ ] All code examples (12+) run on Python 3.14+ without errors
- [ ] All code includes type hints throughout
- [ ] Reading level validated as Grade 7-8 (Flesch-Kincaid automated check)
- [ ] Learning objectives aligned with CEFR proficiency levels (A1-A2 with B1 elements)
- [ ] Skills proficiency metadata embedded in lesson YAML frontmatter
- [ ] Cross-references to Chapter 14 (prerequisites) and Chapter 17 (future use) included
- [ ] All lessons end with "Try With AI" ONLY (lesson closure compliance)
- [ ] Technical review passed (code quality, accessibility, structure)
- [ ] Capstone project "Calculator with Type Safety" integrates all operator types

---

## Phase 1: Content Structure & Core Elements

### Lesson 1: Arithmetic Operators — Doing Math with Python

- [ ] **MUST**: Create lesson outline and learning objectives
  - Acceptance: Outline includes 5 clear learning objectives aligned with Bloom's (Understand + Apply)
  - Reference: `plan.md`, Lesson 1 section; `.claude/output-styles/lesson.md` for lesson structure
  - Effort: 1–2h

- [ ] **MUST**: Write "What It Is" introduction (5 min content)
  - Acceptance: 2–3 sentences explaining operators as actions on data; sets context
  - Reference: Lesson 1 outline; Chapter 14 prerequisite context
  - Effort: 0.5–1h

- [ ] **MUST**: Create Code Example 1 — Basic Arithmetic Operators (all 7)
  - Acceptance: All 7 operators (+, -, *, /, //, %, **) shown in single example; type hints on all variables; runs without error on Python 3.14+
  - Code: `x: int = 10; y: int = 3; result: int = x + y`, etc. (see plan.md for full code)
  - Effort: 1–2h

- [ ] **MUST**: Create Code Example 2 — Type Behavior with Division
  - Acceptance: Shows int/int → float; int//int → int; float operations return float; type() verification included
  - Code: Demonstrates division behavior, validates with type() (see plan.md)
  - Testing: Verify on Python 3.14+ (Windows, macOS, Linux)
  - Effort: 2–3h

- [ ] **MUST**: Create Code Example 3 — Operator Precedence
  - Acceptance: Shows PEMDAS order (multiplication before addition); demonstrates parentheses overriding; type hints throughout
  - Code: `result: int = 2 + 3 * 4` vs. `result: int = (2 + 3) * 4` (see plan.md)
  - Effort: 1–2h

- [ ] **MUST**: Design "Try With AI" Prompts for Lesson 1 (4 total)
  - Acceptance: 4 prompts (concept exploration, application, edge case, synthesis) with expected outcomes documented
  - Prompts:
    1. **Prompt 1 (Concept)**: "What makes / and // different?"
    2. **Prompt 2 (Application)**: "Build a Simple Calculator Problem" (price + tax)
    3. **Prompt 3 (Edge Case)**: "What breaks?" (division by zero, float precision)
    4. **Prompt 4 (Synthesis)**: "Connect to what I know" (Chapter 14 types + Chapter 15 operators)
  - Expected outcomes documented for each (see plan.md for full text)
  - Reference: Plan.md, Lesson 1 "Try With AI" section
  - Effort: 2–3h

- [ ] **SHOULD**: Add validation exercise with expected output
  - Acceptance: Exercise asks students to predict result, then verify with code/AI
  - Example: "Predict: 10 / 3, then verify with Python. What type is the result?"
  - Effort: 1–2h

- [ ] **NICE-TO-HAVE**: Add visual diagram showing operator symbols and operations
  - Acceptance: ASCII diagram or description mapping operators to math operations
  - Effort: 1h

**Lesson 1 Subtotal**: 11–18h

---

### Lesson 2: Comparison Operators — Making Decisions

- [ ] **MUST**: Create lesson outline and learning objectives
  - Acceptance: 5 clear learning objectives (explain operators, predict True/False, understand value equality, etc.)
  - Reference: Plan.md, Lesson 2
  - Effort: 1–2h

- [ ] **MUST**: Write "What It Is" introduction (5 min content)
  - Acceptance: Explains comparisons as "questions returning True/False"; sets stage for Chapter 17 preview
  - Effort: 0.5–1h

- [ ] **MUST**: Create Code Example 1 — All Six Comparison Operators
  - Acceptance: Shows ==, !=, >, <, >=, <= with simple examples; all with type hints; verifies each returns bool type
  - Code: `equal: bool = x == y`, etc., with type() verification (see plan.md)
  - Effort: 1–2h

- [ ] **MUST**: Create Code Example 2 — Type Equality vs. Value Equality
  - Acceptance: Shows 5 == 5.0 is True (value equality), "5" == 5 is False (type matters), demonstrates type checking
  - Code: Demonstrates int/float equality and int/str inequality (see plan.md)
  - Effort: 2–3h

- [ ] **MUST**: Create Code Example 3 — Comparisons with Real Data
  - Acceptance: Realistic scenario (age eligibility, password length); shows comparisons in context; all type hints
  - Code: Uses len() with comparisons; age checking scenario (see plan.md)
  - Effort: 1–2h

- [ ] **MUST**: Design "Try With AI" Prompts for Lesson 2 (4 total)
  - Acceptance: 4 prompts (concept, application, edge case, synthesis) with expected outcomes
  - Prompts:
    1. **Prompt 1 (Concept)**: "What are comparisons? Why True/False? How is == different from =?"
    2. **Prompt 2 (Application)**: "Check User Eligibility" (movie rating age check)
    3. **Prompt 3 (Edge Case)**: "Surprising Comparisons" (5 == 5.0, "5" == 5, True == 1, False == 0)
    4. **Prompt 4 (Synthesis)**: "Why do I need comparisons before if statements?"
  - Expected outcomes: Learn comparison returns bool; distinguish == from =; understand value equality; see Chapter 17 preview
  - Effort: 2–3h

- [ ] **SHOULD**: Add validation exercise predicting True/False
  - Acceptance: Exercise with 5+ comparisons; students predict, then verify
  - Effort: 1–2h

- [ ] **NICE-TO-HAVE**: Add truth table showing all operator results
  - Acceptance: Simple table: `x > y` with examples showing True and False cases
  - Effort: 1h

**Lesson 2 Subtotal**: 11–18h

---

### Lesson 3: Logical Operators — Combining Conditions

- [ ] **MUST**: Create lesson outline and learning objectives
  - Acceptance: Learning objectives for and/or/not; emphasize combining conditions; B1-level proficiency element
  - Reference: Plan.md, Lesson 3
  - Effort: 1–2h

- [ ] **MUST**: Write "What It Is" introduction (5 min content)
  - Acceptance: Explains logical operators combine True/False values; enables complex reasoning
  - Effort: 0.5–1h

- [ ] **MUST**: Create Code Example 1 — Three Logical Operators
  - Acceptance: and, or, not with truth table values; shows all type hints; type() verification for bool results
  - Code: Truth tables (True and True = True, etc.); see plan.md (see plan.md)
  - Effort: 1–2h

- [ ] **MUST**: Create Code Example 2 — Combining Comparisons with Logical Operators
  - Acceptance: Complex conditions like `(x > 5) and (x < 10)`; password strength check pattern; realistic context
  - Code: Range check, out-of-range check, password validity pattern (see plan.md)
  - Testing: Verify on Python 3.14+
  - Effort: 2–3h

- [ ] **MUST**: Create Code Example 3 — Evaluation Order with Parentheses
  - Acceptance: Shows parentheses control order; demonstrates complex boolean evaluation; type hints throughout
  - Code: Age + license + insurance example (see plan.md)
  - Effort: 1–2h

- [ ] **MUST**: Design "Try With AI" Prompts for Lesson 3 (4 total)
  - Acceptance: 4 prompts (concept, application, edge case, synthesis) with expected outcomes
  - Prompts:
    1. **Prompt 1 (Concept)**: "What's the difference between and and or? Why do we need not?"
    2. **Prompt 2 (Application)**: "Check User Permissions" (logged in AND member 7+ days)
    3. **Prompt 3 (Edge Case)**: "Complex Logic" (evaluate expressions with multiple operators)
    4. **Prompt 4 (Synthesis)**: "Why is this important for Chapter 17?"
  - Expected outcomes: Learn truth conditions; understand permission logic; evaluate expressions; see Chapter 17 connection
  - Effort: 2–3h

- [ ] **SHOULD**: Add truth table for all combinations of two boolean values
  - Acceptance: Clear table showing and/or results for T+T, T+F, F+T, F+F
  - Effort: 1h

- [ ] **NICE-TO-HAVE**: Add complexity progression (simple → complex expressions)
  - Acceptance: Start with `True and False` → progress to `(x > 5) and (y < 10)`
  - Effort: 1h

**Lesson 3 Subtotal**: 11–18h

---

### Lesson 4: Assignment Operators — Updating Variables Efficiently

- [ ] **MUST**: Create lesson outline and learning objectives
  - Acceptance: Learning objectives for =, +=, -=, *=, /=; emphasize shorthand for readability
  - Reference: Plan.md, Lesson 4
  - Effort: 1–2h

- [ ] **MUST**: Write "What It Is" introduction (5 min content)
  - Acceptance: Explains assignment operators update variables efficiently; combine operation with assignment
  - Effort: 0.5–1h

- [ ] **MUST**: Create Code Example 1 — Assignment Operators Comparison
  - Acceptance: Shows expanded form (count = count + 5) vs. shorthand (count += 5); demonstrates all 5 operators; all type hints
  - Code: Direct comparison of expanded vs. shorthand; all operators (see plan.md)
  - Testing: Verify on Python 3.14+
  - Effort: 1–2h

- [ ] **MUST**: Create Code Example 2 — Counter Pattern (Common Use)
  - Acceptance: Realistic counting scenario (items in loop); accumulation pattern (total += price); type hints
  - Code: Loop preparation (before Chapter 17); counting items; accumulating total (see plan.md)
  - Effort: 2–3h

- [ ] **MUST**: Create Code Example 3 — Type Behavior with Shorthand Assignment
  - Acceptance: Shows //= operator; *= with decimals; mixed type += behavior; validates all with type()
  - Code: Floor division assignment; float multiplication; type changes (see plan.md)
  - Effort: 1–2h

- [ ] **MUST**: Design "Try With AI" Prompts for Lesson 4 (4 total)
  - Acceptance: 4 prompts (concept, application, edge case, synthesis) with expected outcomes
  - Prompts:
    1. **Prompt 1 (Concept)**: "Why do I need += if I have =?"
    2. **Prompt 2 (Application)**: "Track a Running Total" (balance with deposits/withdrawals/interest)
    3. **Prompt 3 (Edge Case)**: "What breaks or surprises?" (x /= 2 type change; str concatenation with +=)
    4. **Prompt 4 (Synthesis)**: "Prepare for loops" (why count += 1 matters in Chapter 17 loops)
  - Expected outcomes: Learn readability benefit; apply to realistic finance scenario; discover type changes; preview Chapter 17
  - Effort: 2–3h

- [ ] **SHOULD**: Add validation exercise comparing expanded vs. shorthand
  - Acceptance: Exercise showing two equivalent ways to write code; students verify they produce same result
  - Effort: 1–2h

- [ ] **NICE-TO-HAVE**: Show all 5 operators with real-world analogy
  - Acceptance: Connect += to "add to", -= to "remove from", etc.
  - Effort: 1h

**Lesson 4 Subtotal**: 11–18h

---

### Lesson 5: Python Keywords and Capstone Project

- [ ] **MUST**: Create lesson outline and learning objectives
  - Acceptance: Learning objectives for keyword recognition, understanding why reserved, capstone integration
  - Reference: Plan.md, Lesson 5
  - Effort: 1–2h

- [ ] **MUST**: Write "What It Is" introduction (5 min content)
  - Acceptance: Explains keywords are reserved words; prevents common errors; introduces capstone integration
  - Effort: 0.5–1h

- [ ] **MUST**: Create Code Example 1 — Discovering Python Keywords
  - Acceptance: Shows `import keyword; keyword.kwlist` and `keyword.iskeyword()` usage; counts keywords; type hints
  - Code: Keyword list exploration; checking if word is keyword (see plan.md)
  - Testing: Verify on Python 3.14+
  - Effort: 1–2h

- [ ] **MUST**: Create Code Example 2 — Why Keywords are Reserved
  - Acceptance: Shows consequences of attempting keyword as variable; explains why each keyword is reserved; defensive checking pattern
  - Code: Error demonstration (commented out for safety); reserved word explanation (see plan.md)
  - Effort: 1–2h

- [ ] **MUST**: Create Capstone Project — "Calculator with Type Safety"
  - Acceptance: Complete, runnable calculator demonstrating all 4 operator types
  - Requirements:
    - [ ] Gets user input for two numbers (x2)
    - [ ] Performs all 7 arithmetic operations (+, -, *, /, //, %, **)
    - [ ] Uses comparison operators (at least one: ==, >, <)
    - [ ] Uses logical operators (at least one: and, or, not) for safety check (e.g., num2 != 0)
    - [ ] Uses assignment operators (at least one: +=, -=, *=, /=) for running total or calculation
    - [ ] Validates result types with type()
    - [ ] Includes keyword checking (import keyword)
    - [ ] All code has type hints
    - [ ] Includes input validation (check for zero before division)
    - [ ] Includes comments explaining each operator category
    - [ ] Runs without errors on Python 3.14+ (Windows, macOS, Linux)
  - Code: See plan.md for full capstone code (both full and simplified versions)
  - Reference: Plan.md, Lesson 5 capstone section
  - Effort: 4–6h

- [ ] **MUST**: Create Simplified Calculator Version (for pre-function students)
  - Acceptance: Version without function definitions (since Chapter 20 hasn't taught functions yet)
  - Code: Linear script with input validation; all 4 operator types (see plan.md)
  - Effort: 2–3h

- [ ] **MUST**: Design "Try With AI" Prompts for Lesson 5 (4 total)
  - Acceptance: 4 prompts (concept, application, edge case, synthesis) with expected outcomes
  - Prompts:
    1. **Prompt 1 (Concept)**: "What are keywords and why do I care?"
    2. **Prompt 2 (Application)**: "Build a Simple Calculator" (integrate all operators)
    3. **Prompt 3 (Edge Case)**: "What breaks?" (test with edge cases: negative, zero, text input)
    4. **Prompt 4 (Synthesis)**: "Connect all 4 operator types" (where does each appear? why both needed?)
  - Expected outcomes: Understand keyword restrictions; build working calculator; test edge cases; synthesize all 4 operator types
  - Effort: 2–3h

- [ ] **SHOULD**: Create sample output from running capstone
  - Acceptance: Show before/after running calculator; demonstrate all operators being used
  - Effort: 1h

- [ ] **NICE-TO-HAVE**: Create optional extension: "Error-Handling Calculator"
  - Acceptance: Version that handles division by zero gracefully (Chapter 21 preview, optional)
  - Effort: 2h

**Lesson 5 Subtotal**: 14–21h

---

## Phase 2: Practice & Validation Elements

### Cross-Lesson Quality Assurance

- [ ] **MUST**: Validate all code examples run on Python 3.14+
  - Acceptance: Each of 12+ code examples tested on at least 2 of 3 platforms (Windows, macOS, Linux)
  - Process: Run each example in fresh Python REPL; capture output; document any warnings
  - Platforms tested: macOS (primary), Windows (optional), Linux (optional)
  - Effort: 3–4h

- [ ] **MUST**: Verify all code includes type hints
  - Acceptance: 100% of variables and functions use type hints; search grep for lines without `: type` or `-> type`
  - Reference: `.specify/memory/constitution.md` Code Standards (Python 3.14+, type hints mandatory)
  - Effort: 1h (automated check)

- [ ] **MUST**: Validate cognitive load per lesson
  - Acceptance: Confirm each lesson ≤ 5 new concepts (A1-A2 limit)
    - L1: 5 (arithmetic operators as family)
    - L2: 5 (comparison operators + bool results)
    - L3: 3 (and, or, not)
    - L4: 5 (=, +=, -=, *=, /=)
    - L5: 2 (keywords) + integration (no new operators)
  - Reference: Plan.md, "Key Concepts" section per lesson
  - Effort: 0.5h (verification against plan)

- [ ] **MUST**: Create chapter README.md
  - Acceptance: Chapter overview, lists 5 lessons, explains "What You'll Learn"
  - Reference: `.claude/output-styles/lesson.md` for README format
  - Structure:
    - [ ] Chapter title: "Chapter 15: Operators, Keywords, and Variables"
    - [ ] 2–3 paragraph introduction explaining why operators matter
    - [ ] "What You'll Learn" bullet list (learning outcomes)
    - [ ] Prerequisites from Chapter 14
    - [ ] Lesson list with brief descriptions
    - [ ] Estimated time (3.5–4 hours total)
  - Effort: 2–3h

---

### Lesson Closure Pattern Validation

- [ ] **MUST**: Verify each lesson ends with "Try With AI" ONLY
  - Acceptance: Zero content after "Try With AI" section in all 5 lessons; no "Key Takeaways", "What's Next", "Summary"
  - Reference: Constitutional mandate (FR-014 in spec: "All lessons MUST end with Try With AI section ONLY")
  - Check: grep for any content following "Try With AI" heading
  - Effort: 0.5h

- [ ] **MUST**: Verify exactly 4 prompts per lesson
  - Acceptance: Each lesson has exactly 4 "Try With AI" prompts (20 total across chapter)
  - Format:
    - Prompt 1: Concept exploration
    - Prompt 2: Application
    - Prompt 3: Edge case discovery
    - Prompt 4: Synthesis & validation
  - Effort: 0.5h

- [ ] **MUST**: Document expected outcomes for all 20 prompts
  - Acceptance: Each prompt has explicit "Expected outcome" text (see plan.md examples)
  - Reference: Plan.md, all "Try With AI" sections
  - Effort: 1h (copy from plan, adapt to final lesson text)

- [ ] **SHOULD**: Validate prompt progression within each lesson
  - Acceptance: Prompts 1-4 show learning progression (simple → complex); no redundancy
  - Check: Manual review of all 20 prompts
  - Effort: 1h

---

### Technical Review Preparation

- [ ] **MUST**: Create test cases for capstone project
  - Acceptance: At least 5 test scenarios with expected outputs
  - Scenarios:
    1. Positive numbers (10, 3)
    2. Negative numbers (-10, -3)
    3. Zero and positive (0, 5)
    4. Zero as second number (5, 0) — tests division by zero handling
    5. Float inputs (10.5, 2.5)
  - Expected output: Documented for each scenario
  - Effort: 2–3h

- [ ] **MUST**: Validate chapter integration with prerequisites (Chapter 14)
  - Acceptance: All references to type hints, type(), isinstance() explained or prereq-noted
  - Check: Verify Chapter 14 learning objectives are assumed (not re-taught)
  - Effort: 1h

- [ ] **MUST**: Validate chapter preparation for Chapter 17
  - Acceptance: Comparison and logical operators clearly framed as "needed for if statements"
  - Check: Ensure Prompts 4 in Lessons 2-3 explicitly mention Chapter 17
  - Effort: 0.5h

- [ ] **SHOULD**: Accessibility check
  - Acceptance: Reading level Grade 7-8 (automated Flesch-Kincaid check before publication)
  - Process: Copy all lesson text to automated checker (e.g., hemingwayapp.com or readability tool)
  - Passing criteria: Average sentence length < 15 words; average word length < 5 characters (rough Guide)
  - Effort: 1–2h (manual review + potential rewrites for clarity)

- [ ] **SHOULD**: Cross-language review
  - Acceptance: No jargon without definition; analogies clear; cultural references inclusive
  - Check: Review for non-native English speaker comprehension
  - Effort: 1h

---

## Phase 3: Review & Integration

- [ ] **MUST**: Peer review for pedagogical clarity
  - Acceptance: Reviewer confirms:
    - Learning objectives are clear and measurable
    - Code examples teach the intended concept
    - "Try With AI" prompts match learning objectives
    - Progression from Lesson 1-5 is logical
  - Process: Share with another Chapter 14/13 writer or pedagogy reviewer
  - Effort: 2–3h (for reviewer) + 1–2h (for revisions)

- [ ] **MUST**: Peer review for technical accuracy
  - Acceptance: Reviewer confirms:
    - All code examples run without errors
    - Type hints are correct (can be validated by mypy)
    - Operator behavior is correct (int / int → float, etc.)
    - Edge cases are handled or flagged appropriately
  - Process: Share code with Python expert or run mypy type checker
  - Effort: 2–3h (for reviewer) + 1h (for fixes)

- [ ] **MUST**: Style guide compliance check
  - Acceptance: All lessons follow output style from `.claude/output-styles/lesson.md`
  - Checks:
    - [ ] YAML frontmatter present with skills metadata
    - [ ] H1 title for each lesson
    - [ ] Code blocks use triple backticks with `python` language tag
    - [ ] "Try With AI" section uses consistent formatting
    - [ ] Expected outcomes clearly documented
  - Effort: 1h (automated check for formatting)

- [ ] **SHOULD**: Cross-reference validation
  - Acceptance: All links to Chapter 14 (prerequisite) are correct file paths
  - Check: Verify path references in lesson files
  - Effort: 0.5h

- [ ] **SHOULD**: AI-tool neutrality check
  - Acceptance: "Try With AI" prompts work with Claude Code, Gemini CLI, and ChatGPT (not tool-specific)
  - Check: Review prompts for tool-specific language (avoid "Ask Claude", use "Ask your AI")
  - Effort: 0.5h

---

## Phase 4: Final Technical Review & Publication Readiness

- [ ] **MUST**: Technical review by subject-matter expert
  - Acceptance: Validator confirms:
    - All code examples correct (Python 3.14+ syntax)
    - Type hints are modern (`list[int]` not `List[int]`, `X | None` not `Optional[X]`)
    - f-strings used throughout (not % or .format())
    - No eval(), shell=True, or hardcoded secrets
    - Security best practices followed (input validation, error handling)
  - Reference: `.specify/memory/constitution.md` Code Standards (FR-028 to FR-032)
  - Effort: 3–4h (for technical reviewer)

- [ ] **MUST**: Learning outcome validation
  - Acceptance: Technical reviewer confirms each lesson achieves stated learning objectives
  - Check: Can students complete capstone using content taught in Lessons 1-5?
  - Effort: 2h

- [ ] **MUST**: Specification alignment check
  - Acceptance: Confirm chapter delivers all FR (Functional Requirements) from spec
  - Checklist:
    - [ ] FR-001: Arithmetic operators taught (+, -, *, /, //, %, **)
    - [ ] FR-002: Comparison operators taught (==, !=, >, <, >=, <=)
    - [ ] FR-003: Logical operators taught (and, or, not)
    - [ ] FR-004: Assignment operators taught (=, +=, -=, *=, /=)
    - [ ] FR-005: Type hints in all examples
    - [ ] FR-006: Keywords explained (what they are)
    - [ ] FR-007: Keyword list retrieval shown (import keyword)
    - [ ] FR-008: Keyword as variable name error demonstrated
    - [ ] FR-009: Why keywords reserved explained
    - [ ] FR-010 to FR-014: "Try With AI" structure validated
    - [ ] FR-015 to FR-017: Lesson structure (What It Is, Code Idea, progression)
    - [ ] FR-018: Max 5 concepts per lesson
    - [ ] FR-019 to FR-027: Content exclusions (no identity/membership/bitwise operators)
    - [ ] FR-028 to FR-032: Python standards (type hints, f-strings, security)
    - [ ] FR-033 to FR-036: Capstone project complete
  - Effort: 2–3h

- [ ] **MUST**: Success criteria (evals) alignment
  - Acceptance: Content delivery addresses all 7 success criteria (SC-001 through SC-007)
  - Verification:
    - [ ] SC-001: Operators explained in plain language (Lesson objectives + Prompt 1 each lesson)
    - [ ] SC-002: Code examples runnable without errors (all tested on Python 3.14+)
    - [ ] SC-003: Error detection taught (Lesson 2-3, Prompt 3 exploration)
    - [ ] SC-004: Keywords recognized (Lesson 5 + keyword checking)
    - [ ] SC-005: AI partnership prompts (all "Try With AI" sections)
    - [ ] SC-006: Engagement (capstone project completion tracked)
    - [ ] SC-007: Reading level Grade 7-8 (validated via automated check)
  - Effort: 1–2h

- [ ] **NICE-TO-HAVE**: Create instructor guide (optional)
  - Acceptance: Notes for instructors on:
    - Common student misconceptions (= vs. ==, division by zero fear)
    - Suggested pacing (time per section)
    - Extension ideas (what to do if students finish early)
    - Assessment rubric for capstone project
  - Effort: 2–3h

---

## Phase 5: Content Validation & Integration

- [ ] **MUST**: Docusaurus build test
  - Acceptance: Chapter builds successfully in Docusaurus without warnings/errors
  - Process: Add chapter files to `book-source/docs/` directory; run `npm run build`
  - Effort: 1–2h (troubleshooting any build issues)

- [ ] **MUST**: Visual inspection of rendered content
  - Acceptance: Content renders correctly in Docusaurus:
    - [ ] Code blocks display properly (syntax highlighting works)
    - [ ] Links to other chapters work
    - [ ] "Try With AI" sections are visually distinct
    - [ ] YAML frontmatter is hidden (not visible to readers)
  - Effort: 1h

- [ ] **MUST**: Final editorial polish
  - Acceptance: Professional tone, grammar check, consistent voice throughout chapter
  - Process: Read all 5 lessons end-to-end; catch typos, inconsistencies
  - Tools: Spell checker, grammar checker (Grammarly or similar)
  - Effort: 2–3h

- [ ] **SHOULD**: Student feedback (if possible)
  - Acceptance: Option to get feedback from 2–3 beginner students or instructors
  - Process: Have test readers work through Lessons 1-2; collect feedback on clarity
  - Effort: 3–4h (wait for feedback + incorporate)

---

## Acceptance Criteria (Definition of Done)

**All MUST Tasks Completed**:
- [ ] All 5 lessons created with "What It Is → Code Examples → Try With AI" structure
- [ ] Each lesson has exactly 4 "Try With AI" prompts with expected outcomes
- [ ] All lessons end with "Try With AI" ONLY (no summaries/checklists after)
- [ ] All code examples run on Python 3.14+ (tested on at least 1 platform)
- [ ] All code includes type hints
- [ ] Cognitive load limits verified (≤ 5 concepts per lesson)
- [ ] Chapter README.md created
- [ ] Keywords awareness (Lesson 5) and capstone project (both versions) complete
- [ ] Technical review passed (code quality, accessibility, structure)
- [ ] Specification alignment confirmed (all FR requirements met)
- [ ] Success criteria (evals) alignment confirmed (all SC requirements addressed)
- [ ] Docusaurus build test passed
- [ ] Final editorial polish complete

**Optional (SHOULD/NICE-TO-HAVE)**:
- [ ] Accessibility check complete (Grade 7-8 reading level validated)
- [ ] Cross-reference validation done
- [ ] Instructor guide created
- [ ] Student feedback incorporated

**Quality Gates** (before marking "Ready for Publication"):
1. ✅ All MUST tasks complete
2. ✅ Zero post-"Try With AI" content (lesson closure compliance)
3. ✅ All 20 prompts have expected outcomes documented
4. ✅ Capstone project runs without errors and integrates all 4 operator types
5. ✅ Technical review passed without critical issues
6. ✅ Spec alignment verified (all FR requirements)
7. ✅ Reading level validated (Grade 7-8 or clearer)

---

## Risk Mitigation & Dependencies

**Key Risks**:

| Risk | Mitigation | Owner |
|------|-----------|-------|
| R1: Code examples don't run on Python 3.14+ | Test all examples on Python 3.14 before finalizing | Lesson writer |
| R2: Type hints missing from examples | Automated grep check for `: type` and `-> type` | QA/reviewer |
| R3: "Try With AI" prompts are vague | Use examples from plan.md; ensure expected outcomes clear | Lesson writer |
| R4: Lesson exceeds 5 concept cognitive limit | Count concepts against plan.md; split if necessary | Lesson writer |
| R5: Reading level too high for target audience | Automated Flesch-Kincaid check; rewrite complex sentences | Editor |
| R6: Capstone project too complex for A1-A2 students | Test with beginner; simplify if needed; provide function-free version | Lesson writer |
| R7: Chapter doesn't prepare for Chapter 17 | Ensure Lessons 2-3 mention if/while/for coming; cross-ref in Prompts 4 | Lesson writer |

**Dependencies**:
- [ ] Chapter 14 (Data Types) must be complete and stable (✅ already validated as of Nov 8, 2025)
- [ ] Constitution v3.0.2 requirements understood (AI-Native Learning pattern, graduated complexity)
- [ ] Output style guide (`.claude/output-styles/lesson.md`) available for reference

---

## Timeline & Effort Estimation

**Total Estimated Effort**: 45–55 story points (approximately 36–44 hours)

**Phase Breakdown**:
1. **Phase 1 (Content Structure)**: 57–93h per lesson ≈ 14–23h per lesson × 5 = 70–115h
   - *Adjusted: 48–56h* (consolidated estimates, some tasks overlap)

2. **Phase 2 (Practice & Validation)**: 10–14h

3. **Phase 3 (Review & Integration)**: 10–14h

4. **Phase 4 (Technical Review & Publication)**: 8–12h

5. **Phase 5 (Content Validation & Integration)**: 6–10h

**Total**: 82–106h (approximately 2–2.5 weeks at 40 hours/week)

**Recommended Pacing**:
- Week 1: Lessons 1-2 (Phase 1 content creation)
- Week 2: Lessons 3-4 (Phase 1 content creation)
- Week 3: Lesson 5 + capstone (Phase 1), Phase 2-3 reviews
- Week 4: Phase 4 technical review, Phase 5 publication prep

---

## Follow-Ups & Next Steps

**Upon Completion of This Chapter**:

1. **Chapter 16 Planning** (Strings and Type Casting)
   - Builds on Chapter 15's operators (string concatenation with +)
   - Uses str() type conversion (casting)
   - Reference: Assume Chapter 15 operators are foundational

2. **Chapter 17 Planning** (Control Flow and Loops)
   - Uses comparison operators (if, while conditions)
   - Uses logical operators (complex conditions)
   - Uses assignment operators (+=  in loop counters)
   - Reference: Chapter 15 as prerequisite vocabulary

3. **Quality Assurance Feedback Loop**
   - Technical reviewer flags any issues (code, accessibility, structure)
   - Lesson writer addresses feedback within 2 business days
   - Iterate until review passes

4. **Publication & Analytics**
   - Track completion rates (SC-006 eval)
   - Monitor student feedback on difficulty/clarity
   - Adjust Lesson 5 capstone complexity if needed based on data

---

## Notes for Lesson Writer

**Before Starting**:

1. Read the approved spec (`specs/part-4-chapter-15/spec.md`) fully
2. Review this plan (`specs/part-4-chapter-15/plan.md`) for lesson structure
3. Reference Chapter 14 plan to understand prerequisite expectations
4. Review `.claude/output-styles/lesson.md` for YAML frontmatter and formatting

**Key Reminders**:

- **Type hints are not optional** — every variable and function has type hint
- **"Try With AI" is the closure** — no summaries, key takeaways, or checklists after
- **Part 4 language** — say "describe intent" not "specify requirements" (Part 5+ terminology)
- **Errors are learning opportunities** — ZeroDivisionError, TypeError are intentional demos
- **Validation-first** — every lesson teaches students to verify results with type()
- **Concrete examples** — use small numbers (10, 3) that students can mentally verify
- **Four prompts per lesson** — Concept → Application → Edge Case → Synthesis progression

**File Structure**:

```
specs/part-4-chapter-15/
├── spec.md          [approved specification]
├── plan.md          [this implementation plan]
├── tasks.md         [this task checklist]
└── (lesson files created during implementation)

book-source/docs/04-Python-Fundamentals/15-operators-keywords-variables/
├── readme.md        [chapter overview]
├── 01-arithmetic-operators.md
├── 02-comparison-operators.md
├── 03-logical-operators.md
├── 04-assignment-operators.md
└── 05-keywords-and-capstone.md
```

---

**Status**: ✅ Ready for Lesson Writer Execution
**Next Action**: Assign to content-implementer subagent; proceed with Phase 1 (Lesson 1 content creation)
**Quality Gate**: All acceptance criteria must be met before marking chapter complete
**Estimated Completion**: 2–3 weeks (depending on revision cycles)


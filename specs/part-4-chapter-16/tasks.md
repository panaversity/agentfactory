# Chapter 16: Strings and Type Casting — Task Checklist

**Chapter Type**: Technical / Code-Focused
**Status**: Ready for Implementation (Lesson-Writer Phase)
**Feature Branch**: `016-ch16-strings-keywords-typecasting`
**Owner**: [To be assigned]
**Estimated Effort**: 32–40 story points (4–5 days, 1 implementer)
**Spec Reference**: `specs/part-4-chapter-16/spec.md`
**Plan Reference**: `specs/part-4-chapter-16/plan.md`

---

## Task List by Phase

### Phase 1: Chapter Infrastructure & Lesson 1

#### Task 1.1: CREATE Chapter README.md (Chapter-Level Overview)

- [ ] **MUST**: Create `docs/book-source/docs/part-4-python-fundamentals/chapter-16-strings-and-type-casting/README.md`
  - Acceptance: README.md exists at correct path (uppercase), follows `.claude/output-styles/chapter-readme.md` template
  - Sections: Chapter Overview, Learning Objectives, What You'll Build, Prerequisites, How to Navigate This Chapter
  - Include chapter-level success evals (EVAL-001 through EVAL-010 from spec)
  - Content: Professional, engaging, Grade 7-8 reading level
  - Cross-references: Links to Chapters 14-15 (prerequisites) and Chapter 17 (next)
  - Effort: 2–3 hours

#### Task 1.2: CREATE Lesson 1 Outline and Code Examples

- [ ] **MUST**: Create `docs/book-source/docs/part-4-python-fundamentals/chapter-16-strings-and-type-casting/lesson-1-string-fundamentals.md`
  - Acceptance: File exists, follows `.claude/output-styles/lesson.md` template
  - Content structure: (from plan.md, Lesson 1 section)
    - Opening hook: "Why strings matter" (2-3 min)
    - Learning objective clearly stated
    - 5 code examples (from plan.md Examples 1.1–1.5) with explanations
    - CoLearning elements (💬🎓🚀✨) integrated throughout
    - **5 validation checkpoints** after Examples 1.2, 1.3, 1.5, and end
    - "Try With AI" section ONLY (4 prompts with expected outcomes)
    - No summary, key takeaways, or "what's next" sections
  - Code examples: All include type hints, f-strings, isinstance() validation
  - Pedagogical: Clear explanations of WHY (not just HOW); immutability emphasized
  - Testing: All code examples validated to run on Python 3.14+
  - Effort: 4–5 hours

#### Task 1.3: WRITE Code Examples for Lesson 1

- [ ] **MUST**: Write, test, and validate all 5 code examples from plan (Examples 1.1–1.5)
  - Acceptance: Each example is syntactically correct, runs without errors, outputs expected results
  - Requirements:
    - All variables have type hints
    - All use f-strings for formatting
    - All use isinstance() or type() for validation
    - All include inline comments explaining intent
    - Cross-platform validated (Mac, Linux, Windows)
  - Tools: Python 3.14+ interpreter; code validation with mypy or similar
  - Create `assets/ch16/lesson-1-examples.py` with all examples (for reference/testing)
  - Effort: 2–3 hours

#### Task 1.4: DESIGN "Try With AI" Prompts for Lesson 1

- [ ] **MUST**: Write 4 progressive "Try With AI" prompts for Lesson 1 (from plan.md)
  - Acceptance: 4 prompts present, follow Bloom's progression (recall → understand → apply → synthesize)
  - Requirements:
    - Prompt 1: Recall/Understand—"Strings vs Other Types"
    - Prompt 2: Apply—"String Manipulation Task"
    - Prompt 3: Analyze—"Why Immutability Matters"
    - Prompt 4: Synthesize—"Connect to Chapter 14"
  - Each includes: Prompt text, expected outcome (what student learns)
  - Safety note included (AI limitations, when to ask questions)
  - Tools referenced: ChatGPT web (Part 4 pre-tools phase)
  - Effort: 1–2 hours

---

### Phase 2: Lessons 2–3

#### Task 2.1: CREATE Lesson 2 (Essential String Methods)

- [ ] **MUST**: Create `docs/book-source/docs/part-4-python-fundamentals/chapter-16-strings-and-type-casting/lesson-2-essential-string-methods.md`
  - Acceptance: File exists, follows lesson.md template
  - Content: (from plan.md, Lesson 2)
    - Opening hook: "Transforming text with methods"
    - Learning objective
    - 5 code examples (Examples 2.1–2.5 from plan)
    - CoLearning elements throughout
    - Validation checkpoints after Examples 2.1, 2.2, 2.5
    - "Try With AI" section (4 prompts—no additional sections)
  - Code examples: Demonstrate case, split/join, find/replace, strip, chaining
  - Testing: All examples validated on Python 3.14+
  - Effort: 4–5 hours

#### Task 2.2: WRITE Code Examples for Lesson 2

- [ ] **MUST**: Write, test, validate all 5 code examples (Examples 2.1–2.5)
  - Acceptance: Each example runs correctly; demonstrates intended concept
  - Special focus: Split/join demonstrate that operations return different types (list vs string)
  - Cross-platform validation: Python 3.14+, Mac/Linux/Windows
  - Create `assets/ch16/lesson-2-examples.py` with all examples
  - Effort: 2–3 hours

#### Task 2.3: DESIGN "Try With AI" Prompts for Lesson 2

- [ ] **MUST**: Write 4 progressive prompts for Lesson 2
  - Prompt 1: Recall/Understand—"What Do These Methods Do?"
  - Prompt 2: Apply—"Text Processing Task"
  - Prompt 3: Analyze—"Method Behavior and Edge Cases"
  - Prompt 4: Synthesize—"Real-World Text Processing"
  - Each with expected outcomes and safety notes
  - Effort: 1–2 hours

#### Task 2.4: CREATE Lesson 3 (F-String Formatting)

- [ ] **MUST**: Create `docs/book-source/docs/part-4-python-fundamentals/chapter-16-strings-and-type-casting/lesson-3-string-formatting-with-f-strings.md`
  - Acceptance: File exists, follows lesson.md template
  - Content: (from plan.md, Lesson 3)
    - Opening: "Creating dynamic output with intent"
    - Learning objective
    - 5 code examples (Examples 3.1–3.5 from plan)
    - CoLearning elements throughout
    - Validation checkpoints
    - "Try With AI" section (4 prompts—no additional sections)
  - Code examples: Demonstrate f-string basics, expressions, number formatting, intent-first design
  - Emphasis: Intent-first approach prepares for specification-thinking (Part 5)
  - Testing: All examples validated
  - Effort: 4–5 hours

#### Task 2.5: WRITE Code Examples for Lesson 3

- [ ] **MUST**: Write, test, validate all 5 code examples (Examples 3.1–3.5)
  - Acceptance: Examples demonstrate f-string syntax, expressions, format specifiers
  - Special focus: Example 3.5 introduces "intent-first" thinking (foundation for specs)
  - Cross-platform validation: Python 3.14+
  - Create `assets/ch16/lesson-3-examples.py` with all examples
  - Effort: 2–3 hours

#### Task 2.6: DESIGN "Try With AI" Prompts for Lesson 3

- [ ] **MUST**: Write 4 progressive prompts for Lesson 3
  - Prompt 1: Recall/Understand—"F-String Basics"
  - Prompt 2: Apply—"Format Output Task"
  - Prompt 3: Analyze—"Format Specifier Patterns"
  - Prompt 4: Synthesize—"Clear Output Design"
  - Focus on helping students think about intent (what output should communicate)
  - Effort: 1–2 hours

---

### Phase 3: Lesson 4 and Capstone

#### Task 3.1: CREATE Lesson 4 (Type Casting)

- [ ] **MUST**: Create `docs/book-source/docs/part-4-python-fundamentals/chapter-16-strings-and-type-casting/lesson-4-type-casting-fundamentals.md`
  - Acceptance: File exists, follows lesson.md template
  - Content: (from plan.md, Lesson 4)
    - Opening: "Converting between types safely"
    - Learning objective
    - 5 code examples (Examples 4.1–4.5 from plan)
    - CoLearning elements throughout
    - Validation emphasis (validation before conversion)
    - Validation checkpoints
    - "Try With AI" section (4 prompts—no additional sections)
  - Code examples: String→int/float, number→string, boolean conversions, error handling, validation patterns
  - Special focus: validation-first approach (validate input BEFORE converting)
  - Testing: All examples validated; error cases explicitly tested
  - Effort: 4–5 hours

#### Task 3.2: WRITE Code Examples for Lesson 4

- [ ] **MUST**: Write, test, validate all 5 code examples (Examples 4.1–4.5)
  - Acceptance: Examples demonstrate conversions and validation patterns
  - Special requirements:
    - Example 4.2: Show TypeError and ValueError; explain errors as information
    - Example 4.5: Introduce validation patterns (isdigit(), isinstance())
  - Cross-platform validation: Python 3.14+; test edge cases (whitespace, negatives, empty strings)
  - Create `assets/ch16/lesson-4-examples.py` with all examples
  - Effort: 2–3 hours

#### Task 3.3: DESIGN "Try With AI" Prompts for Lesson 4

- [ ] **MUST**: Write 4 progressive prompts for Lesson 4
  - Prompt 1: Recall/Understand—"Type Conversion Basics"
  - Prompt 2: Apply—"Parse User Input Safely"
  - Prompt 3: Analyze—"Type Conversion Edge Cases"
  - Prompt 4: Synthesize—"Real-World Type Safety Pattern"
  - Emphasis on validation, error discovery, and designing robust code
  - Effort: 1–2 hours

#### Task 3.4: CREATE Lesson 5 (Capstone: Interactive String Explorer)

- [ ] **MUST**: Create `docs/book-source/docs/part-4-python-fundamentals/chapter-16-strings-and-type-casting/lesson-5-capstone-interactive-string-explorer.md`
  - Acceptance: File exists, follows lesson.md template
  - Content: (from plan.md, Lesson 5)
    - Opening: "Building a real tool with all Chapter 16 skills"
    - Learning objective (integration, not new concepts)
    - Project description: What the explorer should do
    - Phase-by-phase breakdown:
      - Phase 1: Design (10 min)—describe intent
      - Phase 2: Implementation (25 min)—build with AI
      - Phase 3: Testing (15 min)—validate with edge cases
      - Phase 4: Reflection (5-10 min)—connect to objectives
    - 3 code examples (from plan Examples 5.1–5.3): Basic, Enhanced, Best-Practices
    - CoLearning elements throughout
    - "Try With AI" section (4 prompts—no additional sections)
  - Code examples: Pseudocode design, basic working version, enhanced with validation
  - Effort: 5–6 hours

#### Task 3.5: WRITE Code Examples for Lesson 5

- [ ] **MUST**: Write, test, validate all 3 code examples (Examples 5.1–5.3)
  - Acceptance:
    - Example 5.1: Pseudocode/design intent (not runnable, just structure)
    - Example 5.2: Complete, working basic explorer (demonstrates all Ch 16 concepts)
    - Example 5.3: Enhanced explorer with validation and error handling
  - Requirements:
    - All examples include type hints
    - All use f-strings for output
    - All demonstrate validation
    - Example 5.3 shows function concepts lightly (for reference; not required knowledge)
  - Cross-platform validation: Python 3.14+
  - Create `assets/ch16/lesson-5-examples.py` with working examples
  - Create `assets/ch16/string-explorer-starter.py` (template for students)
  - Effort: 3–4 hours

#### Task 3.6: DESIGN "Try With AI" Prompts for Lesson 5

- [ ] **MUST**: Write 4 progressive prompts for Lesson 5
  - Prompt 1: Recall/Understand—"Connect Concepts"
  - Prompt 2: Apply—"Design and Build"
  - Prompt 3: Analyze—"Improve and Edge Cases"
  - Prompt 4: Synthesize—"Reflect and Extend"
  - Focus on design-before-implementation and reflection
  - Effort: 1–2 hours

---

### Phase 4: Integration & Review

#### Task 4.1: CREATE Unified Code Assets File

- [ ] **SHOULD**: Create `assets/ch16/all-code-examples.py` with:
  - All 18 code examples from Lessons 1–4 (Examples 1.1–4.5)
  - Organized by lesson, clearly commented
  - Each example runnable independently
  - Acceptance: File runs without errors; examples produce expected output
  - Use: Reference for students, testing baseline
  - Effort: 1 hour

#### Task 4.2: VERIFY Cognitive Load and Pedagogical Alignment

- [ ] **SHOULD**: Conduct self-check:
  - [ ] Each lesson has exactly 5 new concepts (or fewer)
  - [ ] Concepts clearly defined in content
  - [ ] Scaffolding visible (A1 → A2 → B1 progression)
  - [ ] No forward references to Chapters 17+
  - [ ] All code examples have type hints
  - [ ] All "Try With AI" prompts follow Bloom's progression
  - [ ] Capstone uses 0 new concepts (integration only)
  - Effort: 1–2 hours

#### Task 4.3: ACCESSIBILITY and INCLUSIVITY Check

- [ ] **SHOULD**: Verify:
  - [ ] Reading level approximately Grade 7-8 (Flesch-Kincaid)
  - [ ] Examples use diverse, inclusive names (not just "Alice" and "Bob")
  - [ ] Examples reflect real-world use cases (user input, formatting, conversions)
  - [ ] Language is clear; jargon is defined
  - [ ] No gatekeeping language ("just", "obviously", "everyone knows")
  - [ ] Examples are relatable to students with various backgrounds
  - Effort: 1 hour

#### Task 4.4: CROSS-REFERENCE Validation

- [ ] **SHOULD**: Verify cross-references:
  - [ ] All lessons reference Chapter 14 (data types) appropriately
  - [ ] All lessons reference Chapter 15 (operators) where relevant
  - [ ] Capstone references Chapters 14–16
  - [ ] No references to Chapters 17–29
  - [ ] Links to previous chapter files are correct
  - Effort: 30 minutes

#### Task 4.5: DOCUSAURUS Build Test

- [ ] **SHOULD**: Test Docusaurus build:
  - [ ] All markdown files parse correctly (no syntax errors)
  - [ ] Navigation structure is correct (lessons appear in sidebar)
  - [ ] Internal cross-links work (Chapter 14, 15 links resolve)
  - [ ] Code blocks render correctly (syntax highlighting)
  - [ ] No broken images or assets
  - Effort: 1–2 hours

---

### Phase 5: Review & Approval Checkpoints

#### Task 5.1: TECHNICAL REVIEW (Code Quality & Correctness)

- [ ] **MUST**: Technical review by code reviewer:
  - Acceptance: All code examples pass review without critical/major issues
  - Checks:
    - [ ] All code examples run on Python 3.14+ without errors
    - [ ] Type hints are correct and complete (no `Any` types)
    - [ ] F-strings used consistently (no % or .format())
    - [ ] No forward references to undefined concepts
    - [ ] Error handling patterns are appropriate
    - [ ] Cross-platform compatibility verified (Mac, Linux, Windows)
  - Output: Technical review report with any issues to fix
  - Effort: 2–3 hours (reviewer)

#### Task 5.2: PEDAGOGICAL REVIEW (Learning Effectiveness)

- [ ] **MUST**: Pedagogical review by instructor/educator:
  - Acceptance: Learning objectives are clear and measurable; scaffolding sound
  - Checks:
    - [ ] Each lesson has single, measurable learning objective
    - [ ] Concepts progress logically (A1 → A2 → B1)
    - [ ] CoLearning elements are integrated (not added at end)
    - [ ] "Try With AI" prompts guide discovery (not just questions)
    - [ ] Validation checkpoints are appropriate and clear
    - [ ] Capstone integrates Lessons 1–4 without new concepts
  - Output: Pedagogical review report with recommendations
  - Effort: 2–3 hours (reviewer)

#### Task 5.3: SPEC ALIGNMENT VERIFICATION

- [ ] **MUST**: Verify all spec evals are addressed:
  - [ ] EVAL-001 (80%+ explain string vs types): Lesson 1 addresses
  - [ ] EVAL-002 (75%+ predict outputs): Lessons 2 validation checkpoints
  - [ ] EVAL-003 (85%+ identify type casting): Lesson 4 addresses
  - [ ] EVAL-004 (70%+ write valid code): Capstone project
  - [ ] EVAL-005 (80%+ convert types): Lesson 4 examples and validation
  - [ ] EVAL-006 (75%+ validate operations): All lessons use isinstance()/type()
  - [ ] EVAL-007 (85%+ completion): All lessons have "Try With AI" section
  - [ ] EVAL-008 (70%+ capstone): Lesson 5 demonstrates integration
  - [ ] EVAL-009 (Grade 7-8 reading): Accessibility check (Task 4.3)
  - [ ] EVAL-010 (No forward refs): Verification check (Task 4.2)
  - Output: Alignment checklist confirming all evals covered
  - Effort: 1 hour

#### Task 5.4: ADDRESS Review Feedback

- [ ] **SHOULD**: If review identifies issues:
  - [ ] Critical issues: MUST be fixed before approval
  - [ ] Major issues: Should be fixed (human decides priority)
  - [ ] Minor issues: Nice-to-fix (human decides)
  - [ ] For each fix: Update lesson, re-test code, re-verify alignment
  - [ ] Effort: Depends on feedback; estimate 2–4 hours

#### Task 5.5: FINAL APPROVAL GATE

- [ ] **MUST**: Confirm completion:
  - [ ] All lessons exist and follow lesson.md template
  - [ ] All code examples validated (Python 3.14+, cross-platform)
  - [ ] All "Try With AI" prompts present (4 per lesson, Bloom's progression)
  - [ ] Technical review passed (no critical/major issues)
  - [ ] Pedagogical review passed (objectives clear, scaffolding sound)
  - [ ] Spec evals verification complete
  - [ ] No forward references; reading level appropriate
  - [ ] Ready for deployment
  - Effort: 30 minutes (approval authority)

---

## Acceptance Criteria (Definition of Done)

**For Chapter 16 to be COMPLETE, ALL of the following must be true**:

### Content Completeness
- [ ] All 5 lessons created and published (Lessons 1–5)
- [ ] Chapter README.md created and linked
- [ ] All lessons follow `.claude/output-styles/lesson.md` template
- [ ] All lessons in correct directory: `docs/book-source/docs/part-4-python-fundamentals/chapter-16-strings-and-type-casting/`

### Code Quality
- [ ] All 18+ code examples run without errors on Python 3.14+
- [ ] All code examples include type hints on every variable
- [ ] All code examples use f-strings only (no %, no .format())
- [ ] All code examples include validation (isinstance() or type())
- [ ] All examples tested on Mac, Linux, and Windows
- [ ] Code assets created: `assets/ch16/` with all examples

### Pedagogical Quality
- [ ] Each lesson has single, measurable learning objective
- [ ] Each lesson introduces exactly 5 new concepts (or fewer)
- [ ] Concepts clearly defined and scaffolded (A1 → A2 → B1)
- [ ] CoLearning elements (💬🎓🚀✨) integrated throughout each lesson
- [ ] Validation checkpoints present after key examples
- [ ] "Try With AI" section at lesson end ONLY (4 prompts, Bloom's progression)
- [ ] No summary, key takeaways, or "what's next" after "Try With AI"
- [ ] Capstone (Lesson 5) integrates Lessons 1–4 without new concepts

### Specification Alignment
- [ ] All spec evals (EVAL-001 through EVAL-010) are addressed
- [ ] No forward references to Chapters 17+
- [ ] Only 5-7 string methods taught (upper, lower, split, join, replace, find, strip)
- [ ] Only core scalar types (int, float, str, bool) in type casting
- [ ] Only f-strings for formatting (no %, .format())
- [ ] Validation-first approach emphasized throughout
- [ ] Part 4 language used (describe intent, not write specifications)

### Accessibility & Inclusivity
- [ ] Reading level Grade 7-8 (Flesch-Kincaid check)
- [ ] Examples use diverse, inclusive names
- [ ] Jargon defined when introduced
- [ ] No gatekeeping language
- [ ] Examples reflect real-world use cases

### Reviews Completed
- [ ] Technical review: Passed (all code valid)
- [ ] Pedagogical review: Passed (objectives clear, scaffolding sound)
- [ ] Spec alignment: Verified (all evals addressed)
- [ ] Docusaurus build: Tested (no errors, links work)

---

## Risk Management & Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| **Code examples fail on Python 3.14+** | High - Blocks publication | Test all examples on 3.14+; cross-platform (Mac/Linux/Windows); use mypy for type checking |
| **Cognitive load exceeds 5 concepts per lesson** | High - Violates spec | Count concepts explicitly; group related items; defer advanced to future chapters |
| **"Try With AI" prompts are vague** | Medium - Students confused | Make prompts concrete; include expected outcomes; reference specific concepts |
| **Forward references to Chapter 17+** | High - Violates scope | Audit all examples; search for: loops, functions, classes, collections, try/except |
| **CoLearning elements missing or weak** | Medium - Misses pedagogy | Include 💬🎓🚀✨ markers; test that students can engage with each element |
| **Reading level exceeds Grade 8** | Medium - Accessibility issue | Use Flesch-Kincaid checker; keep sentences short; define jargon |
| **Capstone introduces new concepts** | High - Violates design | Capstone uses ONLY Lessons 1–4 skills; verify no new methods, types, or patterns |
| **Lessons too long** | Medium - Cognitive overload | Target 48–54 min per lesson; review pacing; break long sections |
| **Integration between lessons unclear** | Low-Medium | Show how each lesson builds on prior; use explicit prerequisites |

---

## Follow-Ups & Next Steps

**Immediately After Chapter 16 Completes**:

1. **Deploy to Docusaurus**: Push to main branch; trigger production build
2. **Update Chapter Index**: Mark Chapter 16 as COMPLETE in `specs/book/chapter-index.md`
3. **Validate Skills Registry**: Update `specs/part-4-skills-registry.md` with Ch 16 skills and proficiency levels
4. **Prepare Chapter 17 Specs**: Begin planning Chapter 17 (Control Flow) which depends on Ch 16 type knowledge

**Longer-Term**:

1. **Monitor Learner Performance**: Track EVAL achievement (80%+ for EVAL-001, 75%+ for EVAL-002, etc.)
2. **Gather Feedback**: Collect student comments on difficulty, clarity, engagement
3. **Maintain & Update**: Keep examples current with Python 3.15+ as it releases
4. **Extend Skills Registry**: Link Chapter 16 skills to future chapters that build on them

---

## Effort Estimation Summary

**By Phase**:
- **Phase 1** (Ch Intro + Lesson 1): 9–12 hours
- **Phase 2** (Lessons 2–3): 12–15 hours
- **Phase 3** (Lesson 4 + Capstone): 15–18 hours
- **Phase 4** (Integration): 4–6 hours
- **Phase 5** (Review): 6–10 hours

**Total**: 46–61 hours (6–8 days, 1 developer + reviewers)

**Breakdown by Activity**:
- Content writing: 20–25 hours
- Code example writing & testing: 12–15 hours
- "Try With AI" prompt design: 5–8 hours
- Reviews & refinement: 8–12 hours
- Build/deployment: 1–2 hours

**Resource Requirements**:
- 1 Lesson-Writer (implementing all lessons)
- 1 Technical Reviewer (code quality, cross-platform testing)
- 1 Pedagogical Reviewer (learning effectiveness, scaffolding)
- 1 Approver (final gate)

---

**This task checklist is PRODUCTION-READY for the lesson-writer subagent to execute systematically.**

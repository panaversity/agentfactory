# Specification Template Reference

**Chapter**: 31 | **Part**: 5 | **Status**: Reference Document

---

## Introduction

A **specification** is the foundation of Spec-Driven Development. It defines **WHAT** needs to be built before determining **HOW** to build it. In AI-native development, specifications serve as the interface between human intent and AI execution.

### Why Specifications Matter

**The Cascade Effect**: Quality flows downhill from specification to code.

```
Clear Spec → Clear Plan → Clear Tasks → Working Code
Vague Spec → Vague Plan → Confused Tasks → Broken Code
```

**Key Insight**: The time you invest in writing a clear specification saves exponentially more time during implementation. AI tools can only build what you can clearly describe.

---

## Complete Specification Template

Every Spec-Kit Plus specification has **6 required components**:

1. **Overview** — What are you building? Why does it matter?
2. **Scope** — What's in vs. out?
3. **Requirements** — What must the system do?
4. **Acceptance Criteria** — How do we verify it works?
5. **Constraints** — What limits us?
6. **Success Criteria** — How do we know we succeeded?

---

## Annotated Example: Calculator Specification

### Component 1: Overview

**What goes here**: Problem statement + value proposition (2-3 paragraphs)

**Why it matters**: Sets context for ALL downstream decisions. If AI (or humans) don't understand the problem, they can't solve it correctly.

**Example**:

```markdown
## Overview

**Problem**: Students learning Python need a simple project to practice
AI-driven development without being overwhelmed by domain complexity.
Existing calculator tutorials teach syntax, not specification-first thinking.

**Solution**: Build a command-line calculator that demonstrates Spec-Kit Plus
workflow end-to-end. The calculator will support 5 core operations (add, subtract,
multiply, divide, power) with proper error handling and type preservation.

**Value**: This project teaches specification-first development through a familiar
domain, allowing students to focus on the SDD workflow rather than learning
complex business logic.
```

**Annotations**:
- ✅ **Good**: Explains WHY calculator (not just WHAT calculator)
- ✅ **Good**: Identifies target user (students learning Python)
- ✅ **Good**: States value proposition (teaches SDD workflow)
- ❌ **Bad**: "Build a calculator" (no context, no why, no value)

---

### Component 2: Scope

**What goes here**: Explicit in-scope features + explicit out-of-scope features

**Why it matters**: Prevents scope creep. AI needs boundaries. If you don't say "graphical UI is out of scope," AI might build one.

**Example**:

```markdown
## Scope

### In Scope
- 5 core operations: add, subtract, multiply, divide, power
- Command-line interface (CLI) only
- Type preservation (int + int = int, float + float = float)
- Error handling: division by zero, invalid input, overflow
- Calculation history (store last 10 operations)
- Basic input validation

### Out of Scope
- Graphical user interface (GUI)
- Advanced scientific functions (sin, cos, log, etc.)
- Multi-user support or authentication
- Database persistence (history stored in memory only)
- Network/API functionality
- Mobile or web versions
```

**Annotations**:
- ✅ **Good**: Both in-scope AND out-of-scope are explicit
- ✅ **Good**: Out-of-scope prevents AI from building unnecessary features
- ✅ **Good**: Scope is realistic for a learning project (5 operations, not 50)
- ❌ **Bad**: "Build a calculator with basic features" (what's "basic"?)

**Common Mistake**: Only listing in-scope features. You MUST explicitly list out-of-scope to prevent AI from over-engineering.

---

### Component 3: Requirements

**What goes here**: Functional requirements (behaviors) + Non-functional requirements (qualities)

**Why it matters**: This is the detailed WHAT. Each requirement becomes a plan component and task unit.

**Example**:

```markdown
## Requirements

### Functional Requirements

**FR-001: Addition Operation**
- System shall add two numbers and return the sum
- Supports integers and floating-point numbers
- Preserves type: int + int = int, float + any = float

**FR-002: Subtraction Operation**
- System shall subtract second number from first number
- Supports integers and floating-point numbers
- Preserves type: int - int = int, float - any = float

**FR-003: Multiplication Operation**
- System shall multiply two numbers and return the product
- Supports integers and floating-point numbers
- Preserves type: int * int = int, float * any = float

**FR-004: Division Operation**
- System shall divide first number by second number
- Supports integers and floating-point numbers
- Returns float for all divisions (3 / 2 = 1.5, not 1)
- Raises ValueError with message "Cannot divide by zero" when divisor is 0

**FR-005: Power Operation**
- System shall raise first number to power of second number
- Supports positive and negative exponents
- Supports fractional exponents (square root: x^0.5)
- Returns float for negative exponents (2^-1 = 0.5)

**FR-006: Calculation History**
- System shall store last 10 calculations in memory
- Each history entry includes: operation, operands, result, timestamp
- User can view history via "history" command
- History clears on program exit (no persistence)

**FR-007: Input Validation**
- System shall validate all inputs are numeric
- System shall reject empty inputs
- System shall provide clear error messages for invalid inputs

### Non-Functional Requirements

**NFR-001: Performance**
- Each calculation shall complete in < 100ms
- History retrieval shall complete in < 50ms

**NFR-002: Usability**
- Error messages shall be user-friendly (no technical jargon)
- CLI prompts shall clearly indicate expected input
- Help command shall list all available operations

**NFR-003: Code Quality**
- All code shall have Python 3.13+ type hints
- All functions shall have docstrings
- Code shall pass mypy type checking
```

**Annotations**:
- ✅ **Good**: Each requirement is numbered (FR-001, FR-002, etc.) for traceability
- ✅ **Good**: Requirements are specific (not "calculator should be fast" but "< 100ms")
- ✅ **Good**: Edge cases are explicit (division by zero, negative exponents)
- ✅ **Good**: Non-functional requirements included (performance, usability, quality)
- ❌ **Bad**: "Calculator should handle errors gracefully" (what errors? how gracefully?)

**Common Mistake**: Vague language like "fast", "user-friendly", "robust". Use numbers and specific behaviors.

---

### Component 4: Acceptance Criteria

**What goes here**: SMART criteria (Specific, Measurable, Achievable, Relevant, Time-bound) that define "done"

**Why it matters**: These become your validation checklist. If code passes all acceptance criteria, the feature is complete.

**Example**:

```markdown
## Acceptance Criteria

### AC-001: Addition Operation Works Correctly
**Given** two numbers
**When** user selects addition operation
**Then** system returns correct sum with type preservation
- ✓ Test: 2 + 3 = 5 (int + int = int)
- ✓ Test: 2.5 + 3.5 = 6.0 (float + float = float)
- ✓ Test: 2 + 3.5 = 5.5 (int + float = float)

### AC-002: Division by Zero is Handled Safely
**Given** division operation with divisor = 0
**When** user attempts calculation
**Then** system raises ValueError with message "Cannot divide by zero"
- ✓ Test: 5 / 0 → ValueError("Cannot divide by zero")
- ✓ Test: Program does not crash
- ✓ Test: User can continue with next calculation

### AC-003: Type Preservation Works as Specified
**Given** any operation with two integers
**When** calculation is performed
**Then** result is integer (except division, which always returns float)
- ✓ Test: 5 + 3 = 8 (type: int)
- ✓ Test: 5 * 3 = 15 (type: int)
- ✓ Test: 5 / 2 = 2.5 (type: float, special case)
- ✓ Test: 2^3 = 8 (type: int)

### AC-004: History Stores Last 10 Calculations
**Given** user has performed 10+ calculations
**When** user views history
**Then** system displays last 10 operations with correct details
- ✓ Test: After 15 operations, history shows operations 6-15 (last 10)
- ✓ Test: Each entry includes: operation type, operands, result, timestamp
- ✓ Test: History is ordered newest-to-oldest

### AC-005: Error Messages are User-Friendly
**Given** invalid input
**When** user enters non-numeric value
**Then** system displays clear error message without technical jargon
- ✓ Test: Input "abc" → "Error: Please enter a valid number"
- ✓ Test: NOT "ValueError: invalid literal for int() with base 10: 'abc'"
- ✓ Test: User can retry after error

### AC-006: Performance Meets Requirements
**Given** any calculation request
**When** operation is executed
**Then** response time is < 100ms
- ✓ Test: Measure 100 operations; all complete in < 100ms
- ✓ Test: History retrieval < 50ms

### AC-007: Code Quality Standards Met
**Given** implemented calculator code
**When** quality checks are run
**Then** all quality standards pass
- ✓ Test: mypy type checking passes with no errors
- ✓ Test: All functions have type hints
- ✓ Test: All functions have docstrings
- ✓ Test: Test coverage ≥ 80%
```

**Annotations**:
- ✅ **Good**: Each criterion is testable (pass/fail, not subjective)
- ✅ **Good**: Uses "Given-When-Then" format (clear scenarios)
- ✅ **Good**: Includes specific test cases with expected values
- ✅ **Good**: Covers edge cases (division by zero, type preservation, error handling)
- ❌ **Bad**: "Calculator should work correctly" (not testable)

**Common Mistake**: Acceptance criteria that are opinions ("user-friendly") rather than tests. Every criterion must be verifiable.

---

### Component 5: Constraints

**What goes here**: Limitations that affect implementation (time, budget, technical, regulatory)

**Why it matters**: Helps AI and humans make realistic decisions. If you have 2 weeks and $0 budget, AI won't suggest a cloud-hosted solution.

**Example**:

```markdown
## Constraints

### Time Constraints
- Project must be completable in 15-18 hours (Chapter 31 duration)
- Students are learning workflow, not building production systems

### Technical Constraints
- Must use Python 3.13+ standard library only (no external dependencies beyond testing)
- Must run on Windows, macOS, and Linux
- No database installation required (memory-only history)
- Must work in terminal/CLI environments

### Scope Constraints
- Domain complexity is intentionally limited (5 operations) to focus on SDD workflow
- No advanced mathematical functions (calculator is learning vehicle, not scientific tool)

### Skill Constraints
- Target audience: intermediate Python developers (know basics, learning SDD)
- Assumes Python 3.13+ installed
- Assumes terminal/CLI familiarity
```

**Annotations**:
- ✅ **Good**: Explains WHY constraints exist (learning vehicle, not production)
- ✅ **Good**: Technical constraints are specific (Python 3.13+, standard library)
- ✅ **Good**: Acknowledges target audience skill level
- ❌ **Bad**: "Build it quickly and cheaply" (not specific)

**Common Mistake**: Forgetting to document implicit constraints. If you're building for students, that's a skill constraint!

---

### Component 6: Success Criteria

**What goes here**: Measurable outcomes that define project success (different from acceptance criteria, which are feature-level)

**Why it matters**: These are project-level goals. Acceptance criteria say "feature works." Success criteria say "project achieved its purpose."

**Example**:

```markdown
## Success Criteria

### SC-001: Students Complete Full SDD Workflow
**Measure**: 90%+ of students complete Constitution → Specify → Clarify → Plan → Tasks → Implement phases
**Why**: The goal is teaching workflow, not just building a calculator

### SC-002: Students Demonstrate Specification Quality Impact
**Measure**: Students can articulate how their spec quality affected plan/task/code quality
**Evidence**: Reflection document shows cascade effect understanding

### SC-003: Calculator Passes All Acceptance Criteria
**Measure**: All 7 acceptance criteria (AC-001 through AC-007) pass
**Why**: Working code validates that workflow was followed correctly

### SC-004: Students Create Complete Artifact Set
**Measure**: Each student produces: Constitution, Spec, Plan, Tasks, 2-3 ADRs, 8-10 PHRs, Code
**Why**: Demonstrates mastery of all Spec-Kit Plus components

### SC-005: Validation Skills Demonstrated
**Measure**: Students can validate AI-generated code against acceptance criteria
**Evidence**: Validation report with pass/fail for each criterion

### SC-006: Foundation for Chapter 32 Established
**Measure**: Students understand single-component workflow before learning multi-component patterns
**Why**: Chapter 32 builds on Chapter 31 workflow mastery
```

**Annotations**:
- ✅ **Good**: Success criteria are project-level (not feature-level like acceptance criteria)
- ✅ **Good**: Each criterion has a measurable outcome
- ✅ **Good**: Explains WHY each success criterion matters
- ✅ **Good**: Distinguishes between "calculator works" (AC-003) and "students learned" (SC-001, SC-002)
- ❌ **Bad**: "Project is successful" (not measurable)

**Common Mistake**: Confusing acceptance criteria (feature-level: "add operation works") with success criteria (project-level: "students learned SDD workflow").

---

## Specification Quality Checklist

Use this checklist to validate your specification before proceeding to planning:

### Overview
- [ ] Explains what you're building
- [ ] Explains why it matters (problem + value)
- [ ] Target user is clear
- [ ] 2-3 paragraphs (not 2 pages)

### Scope
- [ ] In-scope features listed explicitly
- [ ] Out-of-scope features listed explicitly
- [ ] Boundaries are clear (prevents scope creep)
- [ ] Scope is realistic for timeframe

### Requirements
- [ ] Functional requirements are specific behaviors
- [ ] Non-functional requirements cover quality attributes (performance, usability, security)
- [ ] Each requirement is numbered for traceability
- [ ] Requirements are testable (not opinions)
- [ ] Edge cases are documented

### Acceptance Criteria
- [ ] Each criterion is SMART (Specific, Measurable, Achievable, Relevant, Time-bound)
- [ ] Each criterion is testable (pass/fail)
- [ ] Criteria use Given-When-Then format
- [ ] Specific test cases are included
- [ ] Criteria cover all major requirements

### Constraints
- [ ] Time constraints documented
- [ ] Technical constraints listed
- [ ] Budget/resource constraints noted
- [ ] Skill constraints acknowledged
- [ ] Constraints explain why they exist

### Success Criteria
- [ ] Project-level goals defined (not feature-level)
- [ ] Each criterion is measurable
- [ ] Success criteria explain WHY they matter
- [ ] Criteria align with project purpose

### Overall Quality
- [ ] No vague language ("fast", "user-friendly", "robust" without specifics)
- [ ] No forward references without explanation
- [ ] Consistent terminology throughout
- [ ] Ready to hand to planning phase

---

## Good vs. Bad Specification Examples

### Bad Example (Vague)
```markdown
## Overview
Build a calculator that's easy to use and works well.

## Requirements
- Add numbers
- Subtract numbers
- Handle errors gracefully
- Be fast and reliable
```

**Problems**:
- "Easy to use" is subjective
- "Works well" is not measurable
- "Handle errors gracefully" doesn't specify which errors or how
- "Fast" without a number is not testable

---

### Good Example (Specific)
```markdown
## Overview
Build a command-line calculator supporting 5 core operations (add, subtract,
multiply, divide, power) with type preservation and error handling. Target
users: students learning Spec-Driven Development who need a familiar domain
to practice workflow without domain complexity overhead.

## Requirements
**FR-001: Addition Operation**
- System shall add two numbers (int or float) and return sum
- Type preservation: int + int = int, float + any = float
- Example: 2 + 3 = 5 (int), 2.5 + 3.5 = 6.0 (float)

**NFR-001: Performance**
- Each calculation shall complete in < 100ms
- Measured via automated tests (100 operations)
```

**Why This Works**:
- Specific numbers (< 100ms, not "fast")
- Explicit type rules (int + int = int)
- Examples show expected behavior
- Measurable via automated tests

---

## How to Use This Template

### Step 1: Start with Overview
Write 2-3 paragraphs explaining:
- What problem you're solving
- Why your solution matters
- Who will use it

### Step 2: Define Scope
List explicitly:
- What you WILL build (in-scope)
- What you WILL NOT build (out-of-scope)

### Step 3: Write Requirements
For each major feature:
- Give it a number (FR-001, FR-002, etc.)
- Describe behavior in detail
- Include edge cases
- Add non-functional requirements (performance, security, usability)

### Step 4: Create Acceptance Criteria
For each requirement:
- Write Given-When-Then scenario
- Add specific test cases
- Make it pass/fail testable

### Step 5: Document Constraints
Think about:
- Time (how long do you have?)
- Budget (what resources are available?)
- Technical (what platform/language/tools?)
- Skills (what does your team know?)

### Step 6: Define Success Criteria
Ask yourself:
- How will I know this project succeeded? (not just "it works")
- What measurable outcomes prove success?
- Why does this project exist?

### Step 7: Review & Refine
Run through the quality checklist above.

---

## Next Steps

Once your specification is complete:
1. **Use `/sp.clarify`** to get AI feedback on gaps and ambiguities
2. **Iterate** based on feedback (specs are never "one and done")
3. **Get approval** from stakeholders before planning
4. **Proceed to `/sp.plan`** to generate implementation plan

**Remember**: Time spent on specification saves exponentially more time during implementation. A clear spec enables AI to generate correct code.

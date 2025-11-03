---
title: "Implementation & Validation (/sp.implement)"
chapter: 31
lesson: 7
duration_minutes: 150

skills:
  - name: "Implementation Orchestration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving, Content Creation"
    measurable_at_this_level: "Student uses /sp.implement; AI generates code and tests; student reviews and understands output"

  - name: "Validation Mastery"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply, Analyze, Evaluate"
    digcomp_area: "Safety, Problem-Solving"
    measurable_at_this_level: "Student systematically validates code: reads, understands, tests, verifies against spec"

  - name: "AI Feedback & Refinement"
    proficiency_level: "B1-B2"
    category: "Soft"
    bloom_level: "Apply, Analyze, Evaluate"
    digcomp_area: "Communication & Problem-Solving"
    measurable_at_this_level: "Student provides feedback to AI on failed criteria; sees AI refine; understands iteration"

  - name: "Specification Quality Reflection"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze, Evaluate"
    digcomp_area: "Problem-Solving, Safety"
    measurable_at_this_level: "Student evaluates: Did AI understand spec? If code failed, spec ambiguity or AI error? Proposes improvements"

learning_objectives:
  - objective: "Use /sp.implement to generate code, validate against acceptance criteria, and provide feedback for refinement"
    proficiency_level: "B1-B2"
    bloom_level: "Apply, Analyze, Evaluate"
    assessment_method: "Given spec, plan, and tasks, student uses /sp.implement, validates code against acceptance criteria, identifies pass/fail, provides feedback for iteration"

cognitive_load:
  new_concepts: 10
  assessment: "10 new concepts (Workflow, Artifacts, Validation Checklist, Acceptance Criteria Validation, Code Comprehension, Test Coverage, Security, Spec-Code Alignment, Iteration, AIDD Loop) at B1-B2 limit ✓"

differentiation:
  extension_for_advanced: "Validate production code; perform security audit; analyze performance; propose optimization"
  remedial_for_struggling: "Simplified validation checklist (5 items instead of 10); focus on pass/fail of main criteria only"
---

# Lesson 7: Implementation & Validation (/sp.implement)

## Opening Hook

You have:
- A clear specification (Lessons 1-4)
- A detailed plan with phases (Lesson 5)
- Atomic tasks with dependencies (Lesson 6)

Now comes the moment of truth: **Does AI understand your spec well enough to generate working code?**

`/sp.implement` orchestrates code generation. It takes your specification, plan, and tasks, and generates Python code that (hopefully) passes all acceptance criteria.

But here's the key: **AI-generated code is not automatically correct. Your job is validation.** This lesson teaches you how to validate code systematically, identify failures, and provide feedback for refinement.

This is the AIDD loop in action: **Intent (spec) → Generation (code) → Validation (your job) → Refinement (iterate) → Trust.**

---

## Part A: The AIDD Loop (20 min)

### What Is AIDD?

AIDD = **AI-Driven Development**. It's a cycle where humans and AI collaborate:

```
1. INTENT (You write specification)
   ↓
2. GENERATION (AI generates code from spec)
   ↓
3. VALIDATION (You verify code works)
   ↓
4. FEEDBACK (You tell AI what failed)
   ↓
5. REFINEMENT (AI improves based on feedback)
   ↓
6. TRUST (You deploy with confidence)
```

**Key principle**: AI is the executor. You are the validator. Neither works alone; together they produce working code faster than either could alone.

### What /sp.implement Does

`/sp.implement` orchestrates the first two steps of the AIDD loop:

1. **Takes your artifacts**: Specification, plan, tasks
2. **Generates code**: AI writes Python code for each task
3. **Generates tests**: AI writes unit tests and integration tests
4. **Produces documentation**: Function docstrings, README updates
5. **Returns for validation**: Everything for you to review

Your job in Lesson 7: **Complete steps 3-5 of the AIDD loop** (validation, feedback, refinement).

### Risk: AI Can Hallucinate

AI is powerful but imperfect. It can:
- Misunderstand spec ambiguities (your job: catch and clarify)
- Generate code with edge case bugs (your job: test and find them)
- Miss security issues (your job: audit before deploying)
- Optimize in wrong direction (your job: verify it meets acceptance criteria)

**This is why validation is non-negotiable.** You validate not because AI is evil, but because AI is literal. If your spec says "Response time ≤ 1 second," AI might not think to test with 1 million records.

---

## Part B: Run /sp.implement (20 min)

### Running the Command

Like prior lessons, `/sp.implement` runs **within Claude Code**:

1. Open Claude Code
2. Paste your specification, plan, and tasks
3. Run `/sp.implement`
4. Review generated artifacts

### Example Output

When you run `/sp.implement` on your calculator spec/plan/tasks, you get:

```
GENERATED ARTIFACTS:

[calculator.py]
"""Calculator module with arithmetic operations and history."""

def add(a: float, b: float) -> float:
    """Add two numbers."""
    return a + b

def subtract(a: float, b: float) -> float:
    """Subtract two numbers."""
    return a - b

def multiply(a: float, b: float) -> float:
    """Multiply two numbers."""
    return a * b

def divide(a: float, b: float) -> float:
    """Divide two numbers. Raises ValueError on division by zero."""
    if b == 0:
        raise ValueError("ERROR: Cannot divide by zero")
    return a / b

... [more functions] ...

[test_calculator.py]
"""Unit tests for calculator module."""

import pytest
from calculator import add, subtract, multiply, divide

def test_add_positive():
    assert add(2, 3) == 5

def test_add_negative():
    assert add(-2, -3) == -5

... [more tests] ...

[history.py]
"""History management for calculator."""

import json
from pathlib import Path

def save_calculation(operation: str, operands: list, result: float):
    """Save calculation to history file."""
    history_file = Path("history.json")
    # Load existing history or create empty list
    if history_file.exists():
        with open(history_file, 'r') as f:
            history = json.load(f)
    else:
        history = []

    # Append new calculation
    history.append({
        "operation": operation,
        "operands": operands,
        "result": result,
        "timestamp": str(datetime.now())
    })

    # Enforce 50-item limit
    if len(history) > 50:
        history = history[-50:]

    # Save back to file
    with open(history_file, 'w') as f:
        json.dump(history, f, indent=2)

... [more code] ...

SUMMARY:
Generated Files:
- calculator.py (245 lines, 6 functions)
- history.py (120 lines, 4 functions)
- test_calculator.py (180 lines, 15 test functions)
- README.md (updated with usage)

Test Results (Initial):
- test_calculator.py: 15 pass, 0 fail
- test_history.py: 8 pass, 0 fail
- Coverage: 92% (exceeds 80% target)

Ready for validation.
```

### First Observations (Before Validating)

Look at what was generated. Ask yourself:

1. **Structure**: Code looks organized? Functions have docstrings?
2. **Completeness**: Are all required functions present?
3. **Quality**: Type hints used? Error handling present?
4. **Tests**: Were tests generated? Do they look reasonable?
5. **Coverage**: Does coverage report indicate thorough testing?

**Don't run code yet.** Read it first. Understand what the code does before you test it.

---

## Part C: Validation Deep-Dive (80 min)

### Validation Phase 1: Code Reading (15 min)

**Purpose**: Understand the code before running it.

**Process**:
1. Open each generated file
2. Read the code (line by line, not skimming)
3. Ask: "What does this function do? Why is it implemented this way?"
4. Look for:
   - Type hints (present? complete?)
   - Docstrings (present? clear?)
   - Error handling (how are errors caught?)
   - Edge cases (what about boundary conditions?)

**Red Flags** (things to question):
- ❌ Missing type hints
- ❌ No docstrings
- ❌ No error handling (try/except blocks)
- ❌ Hardcoded values (magic numbers)
- ❌ Unsafe operations (subprocess, eval, os.system)

**Green Flags** (things to appreciate):
- ✅ Complete type hints
- ✅ Clear docstrings with parameter descriptions
- ✅ Proper error handling
- ✅ Comments explaining non-obvious logic
- ✅ Security best practices (input validation)

**Example Code Reading**:

```python
def divide(a: float, b: float) -> float:
    """Divide two numbers. Raises ValueError on division by zero."""
    if b == 0:
        raise ValueError("ERROR: Cannot divide by zero")
    return a / b
```

**Reading Notes**:
- ✅ Type hints: a: float, b: float, return: float (complete)
- ✅ Docstring: Explains what it does and what it raises
- ✅ Error handling: Checks for zero divisor before dividing
- ✅ Clear: Code is simple and understandable
- ❓ Question: Is ValueError the right exception type? (Yes, Python convention)

### Validation Phase 2: Comprehension Check (10 min)

**Purpose**: Ensure you understand the generated code well enough to explain it.

**Process**:
For each major function/module, explain in your own words:
1. What's the input? (parameters)
2. What's the output? (return value)
3. What does it do? (sequence of steps)
4. Why is it implemented this way? (design rationale)

**If you can't explain it**, ask AI for clarification before proceeding.

**Example Comprehension Check**:

**Function**: `save_calculation(operation, operands, result)`

**My Explanation**:
1. **Input**: operation name, list of operands, calculated result
2. **Output**: Nothing (saves to file)
3. **What it does**:
   - Load existing history from JSON file (or create empty list)
   - Append new calculation record with timestamp
   - Enforce 50-item limit (keep last 50 only)
   - Write back to file
4. **Why**: Persistent storage, limit prevents unbounded growth

**If unclear**: "AI, can you explain why you enforce the 50-item limit here? Is that the right approach?"

### Validation Phase 3: Test Execution (15 min)

**Purpose**: Run the code and see if tests pass.

**Process**:
1. In Claude Code or local terminal, run tests:
   ```bash
   python -m pytest test_calculator.py -v
   python -m pytest test_history.py -v
   ```

2. Check results:
   - Do all tests pass?
   - Are there any failures?
   - Is coverage adequate (80%+)?

3. If tests fail:
   - Note the failure message
   - Understand why it failed (code bug or test issue?)
   - Mark for refinement phase

**Example Test Results**:
```
test_calculator.py::test_add_positive PASSED
test_calculator.py::test_add_negative PASSED
test_calculator.py::test_multiply_zero PASSED
test_calculator.py::test_divide_by_zero PASSED [checks for ValueError]
...
15 passed in 0.23s

Coverage report:
- calculator.py: 95% coverage
- history.py: 88% coverage
Overall: 92%
```

**Good!** All tests pass, coverage is strong.

### Validation Phase 4: Acceptance Criteria Verification (20 min)

**Purpose**: Verify each acceptance criterion from your spec is satisfied.

**Process**:

Create a validation matrix for each criterion:

| Acceptance Criterion | Test/Verification | Pass/Fail | Notes |
|---|---|---|---|
| "Calculation response ≤ 100ms" | Time 1000 operations | ✅ Pass | Average: 0.5ms |
| "Division by zero error" | Test divide(5, 0) | ✅ Pass | Raises ValueError with message |
| "History limited to 50" | Add 100, verify 50 remain | ✅ Pass | FIFO works correctly |
| "All 6 operations work" | Unit tests for each | ✅ Pass | add, sub, mul, div, exp, sqrt |
| "History persists across restarts" | Save, kill process, load | ✅ Pass | JSON file persisted |

**If a criterion fails** (❌):
- Understand why it failed
- Is it a code bug? Spec ambiguity? Test issue?
- Mark for refinement phase

**Example Criterion Verification**:

**Criterion**: "Response time for all operations ≤ 100ms"

**Verification**:
```python
import time

start = time.time()
for i in range(1000):
    add(i, i+1)
elapsed = time.time() - start
avg_time = (elapsed / 1000) * 1000  # Convert to ms

print(f"Average time per operation: {avg_time:.2f}ms")
assert avg_time <= 100, f"Too slow: {avg_time}ms"
```

**Result**: ✅ Pass (0.5ms average)

### Validation Phase 5: Security & Quality Review (10 min)

**Purpose**: Catch security and quality issues.

**Security Checklist**:
- [ ] No hardcoded secrets (passwords, API keys, tokens)
- [ ] Input validation present (edge cases handled)
- [ ] Error messages don't leak information
- [ ] No dangerous functions (eval, exec, subprocess without validation)
- [ ] File I/O is safe (no path traversal vulnerabilities)

**Quality Checklist**:
- [ ] Type hints complete (all function parameters and returns)
- [ ] Docstrings present and clear
- [ ] Code style consistent (follows PEP 8)
- [ ] Performance acceptable (no unnecessary loops or recursion)
- [ ] Error handling comprehensive (all possible failures caught)

**Example Security Check** (calculator code):

```python
# GOOD: Safe file I/O
history_file = Path("history.json")  # Relative path, fixed filename
with open(history_file, 'r') as f:
    history = json.load(f)

# BAD (example of what to avoid):
user_filename = input("Enter filename: ")
with open(user_filename, 'r') as f:  # Vulnerable! Could load any file
    history = json.load(f)
```

**Result**: ✅ Pass (calculator uses safe file I/O)

### Validation Phase 6: Refinement (if needed) (10 min)

**Purpose**: If validation finds failures, iterate toward passing.

**Process**:

1. **Collect failures**: List all criteria that failed
2. **Analyze root cause**:
   - Is it a code bug? (AI generated incorrectly)
   - Is it a spec ambiguity? (Your spec was unclear)
   - Is it a test issue? (Test is wrong, not code)
3. **Decide action**:
   - **Code bug**: Provide feedback to AI ("`divide()` doesn't handle negative results correctly. Fix it.")
   - **Spec ambiguity**: Refine specification, then ask AI to regenerate
   - **Test issue**: Fix the test, rerun
4. **Regenerate**: Run `/sp.implement` again with feedback
5. **Revalidate**: Repeat validation process

**Example Refinement**:

**Failure**: History limit enforcement doesn't work

**Analysis**: Test shows that after adding 100 items, history has 100 not 50

**Root Cause**: AI code appends to list but doesn't limit properly

**Feedback to AI**:
```
Criterion failed: "History limited to 50 calculations"
Problem: save_calculation() isn't enforcing the limit
Current behavior: History grows to 100 items
Expected behavior: Only last 50 items retained (FIFO)
Can you fix the limit enforcement?
```

**AI Refines**:
```python
def save_calculation(operation: str, operands: list, result: float):
    # ... load and append ...
    # Enforce 50-item limit
    history = history[-50:]  # Keep only last 50
    # ... save to file ...
```

**Revalidate**: Rerun test → ✅ Pass

---

## Part D: Student Practice (50 min)

### Exercise 1: Run /sp.implement

**Instructions**:

1. Open Claude Code
2. Paste your specification, plan, and tasks
3. Run `/sp.implement`
4. Review what was generated

**Documentation**:
- What files were created?
- What's the total lines of code?
- How many tests were generated?
- What's the initial test pass/fail count?

### Exercise 2: Code Reading & Comprehension

**Instructions**:

Pick 3 major functions from generated code. For each:

1. Read the code (no running)
2. Understand it (explain in your own words)
3. Document:
   - What does this function do?
   - What are the inputs/outputs?
   - Do type hints look complete?
   - Is there error handling?

### Exercise 3: Validate Against Acceptance Criteria

**Instructions**:

Create a validation matrix (like above) for your acceptance criteria.

For each criterion:
1. Understand what it's asking for
2. Design a test to verify it
3. Run the test (or code manually)
4. Record Pass/Fail

**If all pass**: Excellent! Code is validation-ready.

**If any fail**: Note the failures. We'll address them in Exercise 4.

### Exercise 4: Provide Refinement Feedback (if needed)

**Instructions** (only if failures exist):

For each failed criterion:
1. Understand the failure
2. Ask: "Is this a code bug or spec ambiguity?"
3. Provide feedback to AI:
   ```
   Criterion failed: [criterion name]
   Expected: [what should happen]
   Actual: [what actually happened]
   Please: [what to fix]
   ```
4. Ask AI to regenerate
5. Revalidate

---

## Part E: Reflection & Cascade Completion (10 min)

### The Complete AIDD Cycle

You've now completed the full cycle:

```
1. INTENT (Specification - Lessons 1-4) ✅
2. GENERATION (Code from /sp.implement) ✅
3. VALIDATION (This lesson) ✅
4. FEEDBACK (Refinement if needed) ✅
5. REFINEMENT (AI regenerates) ✅
6. TRUST (Deploy validated code) →
```

### The Cascade Effect - Complete

From Lesson 1 to now:

```
Lesson 1: SMART Criteria (Specific, Measurable, Achievable, Relevant, Time-bound)
  ↓
Lesson 3: Complete Specification (6 sections with SMART criteria)
  ↓
Lesson 4: Refined Spec (/sp.specify finds gaps, you clarify)
  ↓
Lesson 5: Clear Plan (/sp.plan generates from clear spec)
  ↓
Lesson 6: Atomic Tasks (/sp.tasks decomposes plan into work units)
  ↓
Lesson 7: Working Code (/sp.implement generates, you validate)
  ↓
Result: Specification quality → Plan quality → Task quality → Code quality
```

This is why specification-driven development works. Clarity compounds at every level.

---

## Try With AI

**Tool**: Claude Code with `/sp.implement` command + validation
**Duration**: 15-20 minutes

### Workflow

1. **Run `/sp.implement`** on your artifacts
2. **Review generated code**: Read for understanding (no running yet)
3. **Ask Claude**: "Explain this function: [pick a complex function]"
4. **Create validation matrix**: For each acceptance criterion, design verification
5. **Run tests**: Execute generated tests
6. **Verify criteria**: Check each criterion manually or with tests
7. **If failures exist**: Provide feedback to AI, ask for refinement
8. **Reflect**: "How did clear specifications enable AI to generate working code?"

### Expected Outcomes

- You see the full AIDD cycle in action
- You understand validation as a critical professional skill
- You recognize how specification clarity cascades to code quality
- You're ready for the capstone project with complete mastery
- You have working code that you understand and validated

### Safety & Ethics Note

**Remember**: Never deploy code you haven't read and understood. AI is powerful but literal. Your job is:

1. **Read** (understand what code does)
2. **Validate** (verify it meets acceptance criteria)
3. **Audit** (check for security issues)
4. **Iterate** (refine if failures found)
5. **Deploy** (with confidence in code quality)

This discipline separates production-ready code from "it works in a demo."

---

## Checkpoint: You're Ready for the Capstone Project!

Before starting the capstone, verify you can do these:

- [ ] Run `/sp.implement` on your complete artifacts
- [ ] Read and understand generated code (explain functions in your own words)
- [ ] Create a validation matrix for all acceptance criteria
- [ ] Run tests and verify they pass
- [ ] Manually verify acceptance criteria
- [ ] Provide feedback if criteria fail
- [ ] Iterate (regenerate) if needed
- [ ] Understand the complete AIDD cycle (intent → generation → validation → feedback → refinement → trust)
- [ ] Articulate why specification clarity matters for code generation quality

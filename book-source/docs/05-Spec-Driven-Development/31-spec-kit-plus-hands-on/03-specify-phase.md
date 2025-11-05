---
title: "Specify Phase — Writing Complete Specifications"
chapter: 31
lesson: 3
duration_minutes: 120

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Business First Thinking (Pre-Specification Collaboration)"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can conduct informal human-AI conversation about success criteria BEFORE formal specification"

  - name: "Writing SMART Acceptance Criteria"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can convert vague requirements into Specific, Measurable, Achievable, Relevant, Time-bound criteria"

  - name: "Complete Specification Writing"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write specification with Overview, Scope, Requirements, Acceptance Criteria, Constraints"

  - name: "Identifying Edge Cases"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify edge cases (division by zero, negative exponents, type mismatches, boundary conditions)"

  - name: "Distinguishing User Stories from Acceptance Criteria"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain that user stories describe intent while acceptance criteria describe testable verification"

learning_objectives:
  - objective: "Conduct informal evals-first discussion with AI to clarify success criteria before writing formal specification"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Completion of pre-specification conversation with AI companion"

  - objective: "Write complete calculator specification with 5 operations and clear acceptance criteria"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Specification completeness review (all required sections included)"

  - objective: "Convert vague requirements into SMART acceptance criteria"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "SMART criteria validation (Specific, Measurable, Achievable, Relevant, Time-bound)"

  - objective: "Identify and document edge cases for calculator operations"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Edge case documentation completeness and relevance"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Business first thinking, SMART criteria, Specification structure, Edge cases, User stories vs criteria) within B1 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Write specifications for 2 additional projects; compare how specification style differs based on project type (CLI tool vs library vs API)"
  remedial_for_struggling: "Use provided specification template with examples; focus only on the 5 core operations before exploring edge cases"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Specify Phase — Writing Complete Specifications

Welcome to the most important phase: **Specification**. This is where you translate your vague ideas ("I want a calculator") into crystal-clear requirements that AI can build from.

Here's the paradigm shift: In AI-native development, **your ability to write a clear specification is more valuable than your ability to write code**. Bad code can be refactored — but a bad spec breaks everything downstream. In this lesson, you’ll learn how to collaborate with AI to define success and turn it into a complete, testable specification.

---

## Thinking What Success Looks Like

Before writing a specification, professional teams define criteria for evaluating whether the solution succeeds. This is NOT a formal phase; it's an informal human-AI conversation that clarifies intent.

Not "does it compile?" or "do tests pass?" (those are technical validation). Real evals are **business/user criteria** that define success:

- **For a calculator**:
  - ✅ "Can perform basic arithmetic (add, subtract, multiply, divide)"
  - ✅ "Handles edge cases without crashing (division by zero, negative exponents)"
  - ✅ "Results are accurate to 6+ decimal places"
  - ✅ "Error messages are clear enough for users to understand what went wrong"

- **For a user-facing feature**:
  - ✅ "Users can complete the task in < 30 seconds"
  - ✅ "Error messages help users recover without contacting support"
  - ✅ "Experienced and new users succeed at same rate"

### Use Pre-Specification Conversation to Build Specification V0

Here's how professionals work:

**Step 1: YOU Drive the Exploration Through Questions**

**Critical**: YOU ask the questions to explore the problem space. The AI doesn't interview you; you use the AI to think through requirements.

```
You: "I want to build a calculator. What operations should a basic calculator include?"

AI: "Common operations are add, subtract, multiply, divide, and power (exponentiation).
     Those five operations cover most basic arithmetic needs."

You: "What edge cases do I need to consider for these operations?"

AI: "Key edge cases:
     - Division by zero (should error)
     - 0^0 (mathematically undefined)
     - Type mixing (int and float inputs)"

You: "How precise do the results need to be?"

AI: "For a basic calculator, 6 decimal places minimum is standard:
     - Enough for most practical calculations
     - Avoids floating-point display noise
     - Matches calculator conventions"

You: "So wht success looks like?"
```

**Step 2: Formalize Into Specification**

That conversation becomes the specification's acceptance criteria. The spec says, "Here's what the calculator must do" (derived from the conversation you just had). Now you can Specify (The WHAT):
- **Purpose:** Define what users need in plain English
- **Focus:** User journeys, acceptance criteria, success metrics
- **Avoid:** Technical details, code structure, implementation
- **Output:** spec.md file

```bash
You: "/sp.specify Build Basic Calculator -Let's formalize our discussion into a specification."
```

---

## Write Your Calculator Specification

Now it's your turn. Use the structure above to write your complete calculator specification.

### Step 1: Start the Conversation

Open your AI companion (Claude Code) in your calculator-project directory and have this conversation:

```
I'm writing a specification for a calculator Python library.
Let me clarify what success looks like with you:

1. What operations should my calculator support?
2. What edge cases should I handle?
3. What's my definition of "correct" for floating-point results?
4. How should the calculator interface work? (Library vs CLI?)
5. What should happen with invalid inputs?

[Have the conversation with your AI companion]
```

### Step 2: Create Your Specification File

```bash
/sp.specify Basic calculator operations with full testing. Let's formalize our discussion into a specification.

User journeys:
- Add two numbers (positive, negative, zero, decimals)
- Subtract two numbers (all combinations)
- Multiply two numbers (including edge cases)
- Divide two numbers (we'll handle division by zero later)

Acceptance criteria:
- All operations work with whole numbers and decimals
- All operations return correct results
- All operations have full test coverage
- All functions use Python 3.12+ type hints
- All functions have clear docstrings

Success metrics:
- 100% test coverage for all operations
- Type checking passes with mypy
- Code follows our constitution rules
```

### Step 3: Review the Specification in New Branch

Use the above command **Agent:**
- Creates new feature branch automatically
- Generates comprehensive spec file
- Defines user scenarios and edge cases
- Establishes acceptance criteria
- Sets up testing requirements

In SDD, we think at the FEATURE level (what user needs), not the FUNCTION level (code details).

### Step 4: Verify Completeness

Check that your specification:

- [ ] Has clear overview explaining the calculator
- [ ] Clearly states in-scope (5 operations, error handling) and out-of-scope
- [ ] Includes 5+ user stories
- [ ] Documents edge cases (at least 3-4 per operation)
- [ ] References Constitution constraints
- [ ] Is written clearly enough that another developer could implement from it

---

## Validation: The Cascade Effect

Now test your specification's quality by asking: **Will this spec produce a good plan?**

A good spec has:
- ✅ Crystal-clear operations (no ambiguity)
- ✅ Explicit edge cases (no surprises during planning)
- ✅ AI can build acceptance tests directly from them
- ✅ Constraint alignment (specifications respect Constitution)

A bad spec has:
- ❌ Vague operations ("should work correctly" - what's "correct"?)
- ❌ Missing edge cases (surprises emerge during implementation)
- ❌ Ambiguous criteria ("handle errors"—how?)
- ❌ Ignores Constitution (specification asks for things Constitution forbids)

**Action**: Read your specification aloud. Does it sound clear? Would a developer understand exactly what to build?

---

## Try With AI: Get Specification Feedback

Now let's use your AI companion to validate your specification and ensure it's clear enough for planning.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)

**Context**: Your calculator specification (written in `specs/calculator/spec.md`)

**Goal**: Validate that your specification is specific, measurable, and complete

### Prompt Set (Copy-Paste Ready)

**Prompt 1 — Specification Completeness**

Copy and paste this into Claude Code:

```
I've written a specification for my calculator Python library.
Please review it for completeness and clarity:

1. Is every operation fully specified? (add, subtract, multiply, divide, power)
2. Are acceptance criteria SMART (Specific, Measurable, Achievable, Relevant)?
3. Did I cover edge cases adequately?
4. Would a developer know exactly what to build from this spec?

Here's my specification:
@specs/

Please identify any ambiguities or missing details.
```
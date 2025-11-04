---
title: "Implementation and Human in the Loop Validations"
chapter: 31
lesson: 6
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

# Implementation and Human in the Loop Validations

You have:
- A clear specification
- A detailed plan with phases
- Atomic tasks with dependencies

Now comes the moment of truth: **Does AI understand your spec well enough to generate working code?**

`/sp.implement` orchestrates code generation. It takes your specification, plan, and tasks, and generates code that (hopefully) passes all acceptance criteria.

But here's the key: **AI-generated code is not automatically correct. Your job is validation.** This lesson teaches you how to validate code systematically, identify failures, and provide feedback for refinement.

---

## What /sp.implement Does?

`/sp.implement`

1. **Takes your artifacts**: Specification, plan, tasks
2. **Generates code**: AI writes Python code for each task
3. **Generates tests**: AI writes unit tests and integration tests
4. **Produces documentation**: Function docstrings, README updates
5. **Returns for validation**: Everything for you to review

## Risk: AI Can Hallucinate

AI is powerful but imperfect. It can:
- Misunderstand spec ambiguities (your job: catch and clarify)
- Generate code with edge case bugs (your job: test and find them)
- Miss security issues (your job: audit before deploying)
- Optimize in wrong direction (your job: verify it meets acceptance criteria)

**This is why validation is non-negotiable.** You validate not because AI is evil, but because AI is literal. If your spec says "Response time ≤ 1 second," AI might not think to test with 1 million records.

---

## Run /sp.implement

Like prior lessons, `/sp.implement` runs **within Claude Code** or your AI Companion setup with SpecifyPlus:

1. Open Claude Code/Gemini CLI
2. Run `/sp.implement`

```
/sp.implement let's complete Phase 1

Rules: tests first, smallest diff, keep public API stable within a phase.
After each task: run tests, update checklist, note deltas to spec if needed Mark completed tasks in tasks.md
```

**Agent Does:**

- Implements each task in sequence
- Runs tests after each task
- Updates task completion status
- Shows progress and test results
- Handles any implementation issues

3. Review generated artifacts

**Example for Addition Operation:**

1. Agent writes test_add_positive_numbers(), test_add_negative_numbers(), etc.
2. Agent runs pytest → All fail (RED) ✓
3. Human reviews tests and approves
4. Agent writes add() function implementation
5. Agent runs pytest → All pass (GREEN) ✓
6. Human reviews code and approves
7. Commit "feat: Add addition operation with tests"
8. Move to subtraction operation

This systematic approach ensures quality at each step!

## Verification Steps

### Step 1: Run Complete Test Suite

**Your Prompt:**

```
Run the complete test suite and show me the results.
Include coverage report to verify we meet the constitution requirements.
```

**Agent Does:**

- Runs `uv run pytest -v --cov=calculator --cov-report=term-missing`
- Shows all tests passing
- Displays coverage report (should be 100%)
- Confirms constitution requirements met

### Step 2: Type Checking

**Your Prompt:**

```
Run mypy to verify all type hints are correct.
```

**Agent Does:**

- Runs `uv run mypy src/`
- Shows type checking results
- Confirms no type errors

### Step 3: Code Quality Check

**Your Prompt:**

```
Run ruff to check code quality and formatting.
```

**Agent Does:**

- Runs `uv run ruff check src/ tests/`
- Shows linting results
- Confirms code follows standards

## Expected Artifacts (Generated by Agent)

Your agent will create the appropriate spec artifacts under your project's specs directory (for example, a folder for this feature), along with source code and tests organized per your constitution.

## Ship It with `/sp.git.commit_pr`

Once every task is complete and all verification checks are green, hand the Git work to the agentic workflow command `/sp.git_commit_pr`

- Run `/sp.git.commit_pr Wrap up the basic calculator feature.` (or omit the argument and let it infer the summary)
- Approve the generated branch, commit message, and PR details before announcing completion
- If the command surfaces blockers (e.g., missing credentials), resolve them and re-run so the PR accurately reflects the finished loop

You've completed your first full spec loop! The basic operations are now implemented with comprehensive testing and documentation. 

## Try With AI

**Tool**: Your AI Companion with `/sp.implement` command + validation

### Workflow

1. **Run `/sp.implement`** on your artifacts
2. **Review generated code**: Read for understanding (no running yet)
3. **Ask**: "Explain this function: [pick a complex function]"
4. **Create validation matrix**: For each acceptance criterion, design verification
5. **Run tests**: Execute generated tests
6. **Verify criteria**: Check each criterion manually or with tests
7. **If failures exist**: Provide feedback to AI, ask for refinement
8. **Reflect**: "How did clear specifications enable AI to generate working code?"
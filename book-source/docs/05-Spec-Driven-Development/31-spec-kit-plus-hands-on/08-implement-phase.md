---
title: "Implement Phase - AI-Driven Code Generation and Validation"
chapter: 31
lesson: 8
duration_minutes: 150

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using /sp.implement Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run /sp.implement to orchestrate code generation for tasks"

  - name: "Code Review and Comprehension"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can read AI-generated code and verify it matches specification before acceptance"

  - name: "Validation Against Acceptance Criteria"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student systematically validates each acceptance criterion (test, verify, sign-off)"

  - name: "Understanding PHR Auto-Creation"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student understands PHRs are auto-created by system, knows where to find them, when to request explicit PHRs"

  - name: "Executing Checkpoint Pattern During Implementation"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student reviews→tests→approves→commits for each implementation phase"

learning_objectives:
  - objective: "Use /sp.implement to orchestrate AI-driven code generation for calculator tasks"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful implementation orchestration with checkpoint reviews"

  - objective: "Validate AI-generated code against specification acceptance criteria"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Systematic validation of each criterion with pass/fail evidence"

  - objective: "Understand PHR auto-creation and know where to find them"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Navigation to PHR files and understanding their content"

  - objective: "Identify and request explicit PHRs when system auto-creation might miss important decisions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Judgment about when to request PHR creation"

cognitive_load:
  new_concepts: 10
  assessment: "10 new concepts (Implement command, code generation, code review, validation protocol, acceptance criteria verification, PHR auto-creation, PHR locations, explicit PHR requests, checkpoint pattern execution, error handling during implementation) within B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Request explicit PHRs for complex decisions; analyze generated PHRs for quality; run full test suite before committing"
  remedial_for_struggling: "Focus on core operations first; validate against top 3-4 acceptance criteria before diving into edge cases"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Implement Phase - AI-Driven Code Generation and Validation

This is it: **Implementation**. Everything you've done-specification, planning, tasking-leads to this moment.

`/sp.implement` orchestrates AI code generation. The agent generates code, you review it, you validate against acceptance criteria, you commit. Then the next task.

This lesson teaches two critical skills:
1. **Code validation** - How to review AI-generated code
2. **PHR auto-creation** - Understanding automatic documentation of AI collaboration

---

## What Does /sp.implement Do? 

### The Implement Command

`/sp.implement` analyzes your tasks and generates:
- **Code implementing each task**
- **Tests validating the code**
- **Documentation (docstrings, comments)**

It works task-by-task, respecting your checkpoint pattern.

### How Implementation Works

**Input**: Your specifications, plans, tasks

**Agent's Process**:
1. Read spec, plan, and current task
2. Generate code matching the specification
3. Include type hints, docstrings, error handling
4. Generate tests verifying acceptance criteria
5. Output code + tests, ready for human review

**Your Process**:
1. Review generated code
2. Understand what it does
3. Verify acceptance criteria
4. Approve or request changes
5. Ask to Commit to git
6. Tell agent: "Next task"

#### 💬 AI Colearning Prompt
> "Why is the 5-step validation protocol systematic (Read → Check Spec → Run Tests → Manual Test → Approve) instead of just 'run tests and commit'? What issues does this prevent?"

---

## The Validation Protocol

Validation is NOT just "does it work?" It's systematic verification against your specification.

### The 5-Step Validation Process

**Step 1: Read and Understand**

Read the generated code without running anything:
- Do you understand what it does?
- Does it follow your Constitution (type hints, docstrings)?
- Is the logic clear or does it seem hacky?

**RED FLAG**: If you don't understand the code, don't approve it. Ask the agent to explain or simplify.

**Step 2: Check Against Specification**

Compare code to your specification:
- Does it do what the spec says?
- Does it handle the edge cases you specified?
- Does it match the error handling strategy (exceptions)?

**RED FLAG**: If code does something the spec doesn't mention, question it.

**Step 3: Run Acceptance Criteria Tests**

Run the generated tests:
- All tests pass?
- Coverage adequate?
- Edge cases included?

**RED FLAG**: Any failing tests = don't approve. Agent fixes and retries.

**Step 4: Manual Testing (Optional)**

**Step 5: Review and Approve**

If all checks pass:
- Mark as approved
- Ask to Commit to git
- Provide feedback to agent on quality
- Request next task

---

## PHRs - Automatic Documentation 

While ADRs capture architectural decisions, PHRs capture collaboration and implementation decisions. Together, they form the project’s explainable memory.

### What Are PHRs?

**PHR** = **Prompt History Record**

A PHR automatically documents:
- What prompt you gave the agent
- What the agent responded with
- What decision was made
- When it happened

PHRs are auto-created for all `/sp.` commands and Important clarifications during coding

### Where Are PHRs Stored?

```
history/prompts/
├── calculator/
│   ├── 001-specify-phase.md          (auto-created by /sp.specify)
│   ├── 002-clarify-phase.md          (auto-created by /sp.clarify)
│   ├── 003-plan-phase.md             (auto-created by /sp.plan)
│   ├── 004-tasks-phase.md            (auto-created by /sp.tasks)
│   ├── 005-implement-phase-pt1.md    (auto-created by /sp.implement)
└── general/
    └── [Other non-feature PHRs]
```

### What You Do With PHRs

**You don't create them.** You:
1. **Know they exist** (understand they're being created automatically)
2. **Know where to find them** (`history/prompts/<feature>/`)
3. **Review them later** (for learning, compliance, debugging)
4. **Request explicit PHRs** (only when system might miss something)

### When to Request Explicit PHRs

Normally, the system auto-creates PHRs for every `/sp.*` command and major decisions. But occasionally you might ask:

```
Agent, this debugging session was complex and taught me something important
about floating-point precision. Can you record this as a PHR for future reference?

[Describe what you learned]
```

**When to request**:
- ✅ Novel problem-solving approach
- ✅ Non-obvious error resolution
- ✅ Complex tradeoff decision
- ✅ Learning moment worth preserving

**When NOT to request**:
- ❌ Routine coding (PHRs already auto-created)
- ❌ Simple bug fixes (already captured in git history)
- ❌ Repeated issues (first occurrence captured, repeats unnecessary)

### Your Interaction With PHRs

**During Implementation**:
- You don't think about PHRs; agent creates them automatically
- Focus on reviewing code and validating

**After Implementation**:
- Browse `history/prompts/calculator/` to see all implementation decisions
- Review PHRs to understand "why" decisions were made
- Use for documentation, compliance, or learning

**If System Misses Something**:
- "Record this debugging session as PHR"
- Agent creates explicit PHR for that decision
- Rare; most things are auto-captured

#### 🎓 Expert Insight
> In AI-native development, validation is where human judgment creates value. AI can generate syntactically correct code that completely misses the specification intent. The 5-step protocol (Read → Spec Check → Test → Manual → Approve) catches three critical failure modes: (1) AI misunderstood requirements, (2) tests pass but don't cover edge cases, (3) code works but violates Constitution standards. Professional developers NEVER skip validation—the 10 minutes you invest prevents hours of debugging production failures.

#### 🤝 Practice Exercise

> **Ask your AI**: "I'm about to implement Task 1 (add operation) using /sp.implement. Walk me through what the 5-step validation protocol should check: (1) What Constitution standards should I verify in the generated code? (2) What specific acceptance criteria from my spec should the tests cover? (3) What edge cases must be tested (negative numbers, zero, floats vs ints)? (4) If tests pass but I don't understand the code, should I approve it? Why or why not? Then explain how PHRs will automatically capture this implementation session."

**Expected Outcome**: Your AI should identify Constitution standards (type hints, docstrings, error handling), map spec acceptance criteria to test cases (e.g., "add(5, 3) = 8.0", "add(-2, 5) = 3.0"), list calculator-specific edge cases, explain why understanding code is mandatory before approval (can't maintain what you don't understand), and clarify that PHRs auto-capture this workflow without manual intervention.

---

## Implementing Your Calculator (50 minutes)

Now let's implement your calculator using the checkpoint pattern.

- **Step 1: Run /sp.implement**

In Claude Code, from your calculator-project directory:

```
/sp.implement

My calculator tasks are at specs/calculator/tasks.md

Please implement tasks 1-3 (core operations: add, subtract, multiply):

1. Implement each operation with full type hints and docstrings
2. Generate comprehensive tests (unit + edge case + error handling)
3. Verify 100% code coverage for each operation
4. Output ready for my review

After I review and approve, I'll request the next tasks.
```

- **Step 2: Review Generated Code**

Your Review Checklist:
- [ ] Code is understandable (clear variable names, readable logic)
- [ ] Type hints present on all functions
- [ ] Docstrings present and clear
- [ ] Follows Constitution standards
- [ ] Handles edge cases specified
- [ ] Error handling matches your error strategy
- [ ] Tests cover all acceptance criteria

- **Step 3: Ask Agent to Run Tests**

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

- **Step 4: Validate Acceptance Criteria**

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


### Step 4: Approve and Commit

- If all checks pass run `/sp.git.commit_pr`
- Continue Implementation (Divide, Power, Tests, Docs)
- Repeat the checkpoint pattern for remaining tasks.

---

## Common Mistakes

### Mistake 1: Accepting AI Code Without Reading It First

**The Error**: AI generates code → You immediately commit without review

**Why It's Wrong**: AI makes mistakes (missing error handling, hardcoded values, security issues). Blind trust leads to bugs.

**The Fix**: Validation protocol (5-step checklist):
1. **Read without running** - Understand what code does
2. **Ask questions** - "Why this approach?" "What does this line do?"
3. **Check against spec** - Does it match acceptance criteria?
4. **Run tests** - Do all tests pass?
5. **Review security** - Any hardcoded secrets? Input validation?

### Mistake 2: Requesting Too Many Features at Once

**The Error**: "Implement all 5 operations + tests + error handling + logging in one go"

**Why It's Wrong**: Violates checkpoint pattern. No opportunity to review incrementally.

**The Fix**: One task at a time:
- Implement add() → Review → Commit → Next task
- Not: Implement everything → Review 1000 lines → Hope it works

---

## Try With AI: Reflect on Implementation and Decisions

Use your AI companion to reflect on your implementation and capture important decisions.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)

**Context**: Your completed implementation code and tests

**Goal**: Validate implementation quality and reflect on key decisions captured in PHRs

:::tip ⚠️ Learning WITH AI (Not Generating FROM AI)

**What this exercise teaches:**
- ❌ **DON'T ask**: "Write more code for me"
- ❌ **DON'T ask**: "Add more features automatically"
- ✅ **DO ask**: "Does this code match my specification?"
- ✅ **DO ask**: "What decisions were captured in PHRs?"
- ✅ **DO ask**: "Are there security issues I should address?"

**Your role**: Validate implementation, review generated code, verify acceptance criteria
**AI's role**: Answer questions about code, explain PHRs, identify potential issues

:::

### Prompt Set (Copy-Paste Ready)

**Prompt 1 - Implementation Quality**

Copy and paste this into Claude Code:

```
I've completed implementation of core operations (add, subtract, multiply).

Summary:
- 3 functions implemented
- Type hints included
- Docstrings included
- 15 tests written, all passing
- 100% coverage achieved

Review my code at: calculator/operations.py
Review my tests at: tests/test_operations.py

Is the implementation quality good? Any suggestions for improvement?
What patterns from this implementation should I maintain for the remaining operations?
```

**Prompt 2 - Decision Capture**

After quality review, ask:

```
During implementation, we made several design decisions.
These decisions are being captured in PHRs automatically.
If I need to understand "why" something was implemented this way in the future,
where would I look?
```

**Prompt 3 - PHR Exploration**

Finally, ask:

```
Can you help me understand the PHRs created during my calculator implementation?

Which files in history/prompts/calculator/ were auto-created?
What does each one capture?

And if I had discovered something surprising during implementation
(like floating-point precision issues), should I request an explicit PHR?
When is that warranted?
```

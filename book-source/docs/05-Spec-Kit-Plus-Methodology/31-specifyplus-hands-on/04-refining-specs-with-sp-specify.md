---
title: "Refining Specs with /sp.specify"
chapter: 31
lesson: 4
duration_minutes: 90

skills:
  - name: "Specification Refinement"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving, Content Creation"
    measurable_at_this_level: "Student uses /sp.specify to identify gaps; iterates on spec; produces clearer version"

  - name: "AI Feedback Interpretation"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Communication & Problem-Solving"
    measurable_at_this_level: "Student reads AI feedback on completeness; understands gap rationale; applies feedback meaningfully"

  - name: "Iterative Clarity"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student recognizes specs are iterative; each iteration adds clarity; provides rationale for revisions"

learning_objectives:
  - objective: "Use /sp.specify to analyze specification completeness, understand AI feedback, and iterate on specifications to improve clarity"
    proficiency_level: "B1"
    bloom_level: "Apply, Analyze"
    assessment_method: "Given a specification, student uses /sp.specify, reviews feedback, refines spec, articulates which gaps were filled and why"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (Tool Usage, Feedback Interpretation, Iteration, Gap Categories, Version Control, AI Partnership, Cascade Improvement) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Refine specifications for large-scale systems; handle conflicting feedback from multiple stakeholders"
  remedial_for_struggling: "Simplified gap categories (only 3 types); guided feedback interpretation checklist"
---

# Lesson 4: Refining Specs with /sp.specify

## Opening Hook

You've written a complete specification. It has all six sections. It feels done. But is it **really** ready for implementation?

This is where `/sp.specify` enters the picture. This SpecifyPlus tool analyzes your specification and identifies gaps—ambiguities you might have missed, incomplete assumptions, untestable criteria.

The magic isn't that `/sp.specify` fixes your spec. It doesn't. The magic is that it surfaces gaps so **you** can decide which ones matter and refine accordingly. This lesson teaches you how to use this feedback loop to iterate toward clarity.

---

## Part A: Introduction to /sp.specify (15 min)

### What /sp.specify Does

`/sp.specify` is a SpecifyPlus command that analyzes your specification against best practices and returns feedback.

**What it checks**:
- Are all six specification sections present?
- Are requirements specific (not vague)?
- Are acceptance criteria measurable (testable)?
- Are assumptions explicit?
- Are constraints realistic?
- Is there alignment between Overview and Success Criteria?
- Are there missing edge cases?

**What it returns**:
- List of identified gaps (ambiguities, missing sections, unclear terms)
- Suggested improvements (how to clarify)
- Questions for you to answer ("What's the maximum file size?" "How long can a calculation history be?")

### Where It Runs

**Important**: `/sp.specify` runs **within Claude Code** or your AI development environment. It's not a terminal command.

When you invoke `/sp.specify`:
1. Open Claude Code (or your AI tool)
2. Paste your specification
3. Run the `/sp.specify` command within the tool
4. Review the AI's analysis
5. Refine your spec based on feedback

### Why This Matters

Writing specs in isolation, you're blind to gaps in your own thinking. `/sp.specify` provides an external perspective. It catches:
- Assumptions you didn't realize you were making ("Does history persist across restarts?")
- Ambiguous language you didn't notice ("fast" appears in three different contexts but means different things)
- Missing edge cases ("What happens if user tries to divide by zero?")

---

## Part B: Demo — Running /sp.specify (20 min)

### Example Specification (Before Refinement)

Let's start with a calculator spec that's pretty good but has some gaps:

```markdown
# Calculator Specification

## Overview
Build a CLI calculator that performs arithmetic operations and stores results.

## Scope
### In Scope
- Add, subtract, multiply, divide operations
- Show calculation history
- Clear history command

### Out of Scope
- Advanced functions (sin, cos, etc.)
- Graphical interface

## Requirements
### Functional
1. User can enter two numbers and an operation
2. System displays the result
3. System stores results in history
4. User can view history
5. User can clear history

### Non-Functional
1. Fast calculation
2. User-friendly error handling

## Acceptance Criteria
1. Calculation is fast
2. Errors are displayed clearly
3. History works

## Constraints
- Built in Python
- No external libraries

## Success Criteria
- Users are happy
- No calculation errors
```

### Running /sp.specify (Simulated Feedback)

When you run `/sp.specify` on this specification, you get feedback:

```
SPECIFICATION ANALYSIS REPORT

STATUS: Gaps Identified (6 issues)

GAPS IDENTIFIED:

1. VAGUE LANGUAGE - Acceptance Criteria
   - "Calculation is fast" - not measurable
   - Suggested: "Response time ≤ 100ms for all operations"

2. MISSING ASSUMPTION - History
   - Does history persist across calculator restarts?
   - Suggested: Clarify in Scope or Constraints

3. VAGUE LANGUAGE - Non-Functional Requirement
   - "User-friendly error handling" - not specific
   - Suggested: "Display error message in format: ERROR: [operation] failed"

4. MISSING EDGE CASE - Arithmetic
   - Division by zero not addressed
   - Suggested: "What happens when user divides by zero? (error? exception?)"

5. UNTESTABLE CRITERIA - Success
   - "Users are happy" - how do you measure this?
   - Suggested: "Users report ≥4/5 satisfaction on survey" or "Zero bug reports"

6. INCOMPLETE CONSTRAINT
   - Maximum history size not specified
   - Suggested: "History limited to 50 calculations" or "Unlimited"

SUMMARY:
Your spec is on the right track (all 6 sections present).
Address these 6 gaps to improve clarity for implementation.

PRIORITY:
CRITICAL (blocks implementation):
- Acceptance Criteria (items 1, 3, 5)
- Edge cases (item 4)

IMPORTANT (causes confusion):
- Assumptions (item 2)
- Constraints (item 6)
```

### Observing the Cascade

Notice what happened: Gaps in acceptance criteria → unclear what "done" means → implementation confusion.

---

## Part C: Student Practice (45 min)

### Exercise 1: Run /sp.specify on Your Spec

**Instructions**:

1. Open Claude Code (or your AI companion tool)
2. Copy your specification from Lesson 3 and paste it into Claude Code
3. Run the `/sp.specify` command (syntax: `/sp.specify`)
4. Read the analysis report

**What to Do**:
- Take a screenshot or copy the feedback
- Count how many gaps were identified
- Note the CRITICAL vs. IMPORTANT gaps

### Exercise 2: Analyze the Feedback

**Instructions**: For each gap identified, understand WHY it matters.

**For each gap, answer**:
1. What's the gap? (What did AI identify as missing or vague?)
2. Why does it matter? (How would this confusion affect implementation?)
3. Is it critical? (Blocks implementation or just improves clarity?)
4. Can I fix it? (Is the suggested improvement reasonable?)

**Example** (from the demo above):

**Gap**: "Calculation is fast" is not measurable

**Why it matters**: A developer might assume "< 1 second" while you meant "< 100ms". They implement differently. Validation fails.

**Is it critical**: Yes. Acceptance criteria validation depends on this being specific.

**Can I fix it**: Yes. Changed to "Response time ≤ 100ms for all operations"

### Exercise 3: Refine Your Spec

**Instructions**: Choose 3 critical gaps from your feedback. For each, refine your specification to address it.

**Refinement Process**:
1. **Read the gap**: What's missing or vague?
2. **Understand the impact**: How would this cause problems?
3. **Refine the spec**: Add clarity, remove ambiguity, answer the question
4. **Document the change**: "Changed from 'fast calculation' to 'response time ≤ 100ms'"

**Examples of refinements**:

| Gap | Refined Section | Change |
|-----|-----------------|--------|
| "User-friendly error handling" is vague | Non-Functional Requirements | Changed to: "When user enters invalid input, system displays error message within 500ms in format: ERROR: [operation] failed" |
| Maximum history size not specified | Constraints | Added: "History limited to 50 calculations; older entries removed (FIFO)" |
| Division by zero not addressed | Acceptance Criteria | Added: "When user attempts division by zero, system displays: 'ERROR: Cannot divide by zero. Enter a new expression.'" |

### Exercise 4: Run /sp.specify Again

**Instructions**:

1. Copy your refined specification
2. Paste into Claude Code
3. Run `/sp.specify` again
4. Compare feedback (should improve!)

**Comparison**:
- First run: 6 gaps identified
- Second run: Hopefully 2-3 gaps remaining (and less critical)
- Pattern: Specification quality improves with iteration

---

## Part D: Cascade Demonstration (10 min)

### How Spec Refinement Cascades

Watch what happens when your specification improves:

**Vague Spec → /sp.specify identifies 6 gaps → You refine spec → Better spec**

When you move to planning (Lesson 5):

**Better Spec → /sp.plan generates clearer plan with explicit phases**

When you move to tasks (Lesson 6):

**Clearer Plan → /sp.tasks decompose into well-structured, testable tasks**

When you generate code (Lesson 7):

**Clear Tasks → /sp.implement generates code that passes validation**

This is the **cascade effect in action**: Spec quality → Plan quality → Task quality → Code quality.

---

## Try With AI

**Tool**: Claude Code with `/sp.specify` command (or ChatGPT web as fallback)
**Duration**: 10-15 minutes

### Workflow

1. **Copy your specification** into Claude Code
2. **Run `/sp.specify`**: Paste the specification and request analysis
3. **Review the feedback**: Identify 3+ gaps
4. **Prioritize gaps**: Which are critical vs. important?
5. **Refine your spec**: Address 1-2 gaps
6. **Re-run `/sp.specify`**: Notice improvement
7. **Reflect**: "How did refinement improve clarity?"

### Expected Outcomes

- You experience `/sp.specify` as a real iteration tool, not a demo
- You understand how feedback surfaces gaps in your thinking
- You see the cascade effect: better spec → better planning → better implementation
- You're ready for `/sp.plan` (Lesson 5) with a well-refined specification
- You recognize that iteration toward clarity is normal and valuable

### Safety & Ethics Note

**Remember**: AI feedback surfaces possible gaps. You're responsible for deciding which gaps matter for YOUR project. Don't blindly accept all feedback. If `/sp.specify` suggests something you disagree with, you can skip it—but understand the tradeoff.

Example: If `/sp.specify` asks "How many languages should the UI support?" and you reply "English only," that's a valid clarification. If you don't clarify and someone later implements 5 languages, that's wasted work.

---

## Checkpoint: Are You Ready for Lesson 5?

Before moving forward, verify you can do these:

- [ ] Run `/sp.specify` on your specification within Claude Code
- [ ] Understand what each identified gap means
- [ ] Distinguish between critical gaps (block implementation) and important gaps (improve clarity)
- [ ] Refine your specification to address at least 3 gaps
- [ ] Re-run `/sp.specify` and see improvement
- [ ] Explain how better specifications cascade to better planning
- [ ] Have a specification that's genuinely "ready for planning"

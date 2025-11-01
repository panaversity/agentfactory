---
title: "Capstone Part 1: Decompose Your Spec Into Independent Features"
chapter: 32
lesson: 8
lesson_type: "capstone_part_1"
duration: "2.5 hours"
skills:
  - name: "Feature Decomposition"
    proficiency: "B1"
    category: "Technical"
  - name: "Dependency Analysis"
    proficiency: "B1"
    category: "Technical"
  - name: "Parallel Execution Planning"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Decompose a complex specification into independent, parallelizable features (B1)"
  - "Identify critical dependencies and blocking relationships (B1)"
  - "Design work packages for team parallel execution (B1)"
---

# Capstone Part 1: Decompose Your Spec Into Independent Features

## What You're About to Do

This is the first part of Chapter 27's capstone project. In this part, you'll take the grading system specification you created in Chapter 26, Lesson 9, and prepare it for a multi-person team by decomposing it into independent features.

This is **specification preparation**—the work a technical lead or architect does before handing work to team members.

---

## Part 1: Gather Your Existing Specification

You need your grading system specification from Chapter 26, Lesson 9. This specification should include:
- Feature description (what the system does)
- Input requirements (submission format, rubric format)
- Output requirements (grade format, feedback format)
- Acceptance criteria (how you know it works)
- Constraints (response time, reliability)
- Error handling (what happens when things go wrong)

**If you haven't completed Chapter 26, Lesson 9, do that first.**

Once you have your spec, review it carefully. Make notes:
- What parts of your spec are clear?
- Where are ambiguities?
- What parts could be refined?

---

## Part 2: Decompose Into Independent Features

Your grading system likely consists of multiple independent features that can be built in parallel. Your job is to identify them.

### Step 1: Identify Feature Boundaries

Ask your AI companion:

> **Prompt**: "I have a grading system specification. Here's what it says: [paste your spec]. I need to decompose this into 2-3 independent features that can be built in parallel. For each feature, tell me: (1) What does it do? (2) What's its input? (3) What's its output? (4) How do other features depend on it? (5) How long would it take to implement?"

Your companion will help you identify natural boundaries. Common features for a grading system:

**Feature 1: Grading Engine**
- **What**: Takes a submission and rubric, calculates a grade
- **Input**: submission_text, rubric_criteria
- **Output**: grade (0-100), explanation
- **Dependencies**: None (first feature)
- **Estimated Time**: 4-6 hours implementation + 2 hours testing

**Feature 2: Feedback Generator**
- **What**: Takes a submission, grade, and rubric, generates personalized feedback
- **Input**: submission_text, grade_result, rubric_criteria
- **Output**: feedback (200-500 words)
- **Dependencies**: Depends on Feature 1 output format
- **Estimated Time**: 4-6 hours implementation + 2 hours testing

**Optional Feature 3: Grading Analytics**
- **What**: Tracks grading patterns and generates reports
- **Input**: Graded submissions, grades, feedback
- **Output**: Reports (grade distribution, feedback patterns)
- **Dependencies**: Depends on Features 1 & 2
- **Estimated Time**: 6-8 hours implementation + 2 hours testing

### Step 2: Validate Feature Independence

For each feature pair, ask:

> **Prompt**: "I've decomposed my grading system into these features: [list them]. For each pair, tell me: Can Feature A be implemented independently of Feature B? What's the minimum information Feature A needs from Feature B?"

This validates that features can truly be built in parallel.

### Step 3: Document Feature Specifications

For each feature, write a **standalone specification** that includes:

**Format: Feature Specification Template**

```markdown
# Feature [Number]: [Feature Name]

## Overview
[1-2 sentences describing what this feature does]

## Inputs
[Specify exact input format and constraints]
Example:
  submission_text: string (1-5000 characters)
  rubric_criteria: array of objects
    - criterion_name: string
    - weight: number (0-1)

## Outputs
[Specify exact output format and constraints]
Example:
  grade: number (0-100)
  explanation: string (100-500 characters)
  timestamp: ISO 8601 datetime

## Acceptance Criteria
- [ ] All inputs validated and error cases handled
- [ ] All outputs match specification exactly
- [ ] Calculation logic is correct and testable
- [ ] Error messages are clear
- [ ] Performance meets constraint: <5 second response time

## Error Cases
[List what happens when things go wrong]
- If submission_text is empty: Return 400 error
- If rubric_criteria is invalid: Return 400 error
- If calculation times out: Return 503 Service Unavailable

## Constraints
- Response time: <5 seconds for typical submissions
- Availability: Must handle 99.9% of requests
- Scalability: Must support 100 concurrent requests

## Implementation Notes
- Write tests for each acceptance criterion
- Include docstrings on all functions
- Use type hints on all function parameters and returns
- Log key decisions for debugging

## Testing Strategy
- Unit tests: Test grade calculation with known inputs
- Integration tests: Test with full submission + rubric
- Edge case tests: Empty submission, extreme rubric weights, etc.
```

### Step 4: Define Integration Contracts

The key to parallel development is defining exactly how features connect.

Create a document called **"Integration Contracts"** that specifies:

```markdown
# Integration Contracts

## Feature 1 → Feature 2

Feature 1 (Grading Engine) produces output in this format:
```json
{
  "submission_id": "uuid-string",
  "grade": 85,
  "explanation": "Clear writing with minor grammatical errors.",
  "rubric_used": {
    "criteria": [
      {"name": "Grammar", "weight": 0.3, "score": 8},
      {"name": "Clarity", "weight": 0.4, "score": 9},
      {"name": "Organization", "weight": 0.3, "score": 8}
    ]
  },
  "timestamp": "2025-11-01T10:30:00Z"
}
```

Feature 2 (Feedback Generator) expects input in this format:
```json
{
  "submission_id": "uuid-string",
  "submission_text": "...",
  "grade_result": { /* Feature 1 output */ },
  "rubric_criteria": [ /* original rubric */ ]
}
```

**What this means**:
- Feature 1 developer: Produce output with these exact fields
- Feature 2 developer: You'll receive input in this format
- Both developers: Don't change this contract without approval

## Error Handling Contract

If Feature 1 fails:
- Feature 2 receives error in this format: `{"error": "Grading failed", "status": 503}`
- Feature 2 behavior: Log error, return error to user, don't attempt feedback generation

If Feature 2 fails:
- User receives error: `{"error": "Feedback unavailable", "status": 503}`
- Grading still succeeds; feedback is optional

This contract prevents cascading failures.
```

---

## Part 3: Prepare for Team Handoff

Now you're preparing to hand these specs to team members (or AI companions in the capstone).

### Step 1: Create a Specification Package

Create a folder/document called **"Grading System - Team Implementation Package"** that includes:

1. **README** (orientation for the team)
   - Overview of what we're building
   - Key timeline and milestones
   - How specs will be used

2. **Specification for Feature 1** (standalone)
   - Give this to whoever implements Feature 1

3. **Specification for Feature 2** (standalone)
   - Give this to whoever implements Feature 2

4. **Integration Contracts** (shared)
   - Both features need this
   - This is how they coordinate

5. **Testing Specification** (shared)
   - How we'll validate features work individually
   - How we'll validate integration

6. **FAQ** (anticipated questions)
   - "Can I change the data format?"
   - "What if the spec is unclear?"
   - "How do I ask questions?"

### Step 2: Review Your Package for Clarity

Ask your companion:

> **Prompt**: "I've prepared a specification package for a team to implement my grading system. Here's the package: [summarize what you've prepared]. Read this package from the perspective of a developer who's never worked with me before. What's unclear? What questions would I ask? What would confuse me?"

Your companion will identify gaps. Address them:
- If the integration contract is unclear, clarify it
- If a feature spec is vague, add more detail
- If the testing strategy is unclear, add examples

### Step 3: Prepare Your Handoff Plan

Create a document describing how you'll hand off the specs:

**Specification Handoff Plan**

```markdown
# How We'll Use Specifications

## Specification Review (Before Implementation)
When: [Date/Time]
Who: [Implementers] + [You as Spec Lead]
What: Review specifications and integration contracts
Goal: Confirm everyone understands the spec before we start coding
Decision: Is the spec clear enough to implement? (YES/NO)

## Implementation Phase (While Building)
Duration: [X hours]
Process:
- Feature 1 developer implements from their spec
- Feature 2 developer implements from their spec
- Both reference integration contracts
- If anyone finds spec unclear, message Spec Lead for clarification

## Integration Phase (When Both Complete)
When: [Date]
What: We integrate Feature 1 and Feature 2
Test: Does Feature 1's output match Feature 2's input expectations?
Result: Working integrated system or identified spec bugs

## Reflection (After Complete)
When: [Date]
What: Did specs work? Where was clarity missing?
Goal: Learn how to write better specs next time
```

---

## Part 4: Validate Your Decomposition

Before you hand this off, validate your decomposition.

### Step 1: Check Feature Independence

For each feature pair:
- Can Feature A be completely implemented without seeing Feature B's code?
- Does Feature A need to know anything about Feature B besides the output format?
- If answer is yes, your decomposition is good.

### Step 2: Check Integration Clarity

Can someone read:
- Feature 1 spec + Integration contract → Understand exactly what to produce?
- Feature 2 spec + Integration contract → Understand exactly what to expect?

If yes, your specs are ready.

### Step 3: Check Completeness

Does your Feature Spec answer these questions?
- What are the exact inputs? (format, constraints, examples)
- What are the exact outputs? (format, constraints, examples)
- How do you know it's correct? (acceptance criteria)
- What happens when it fails? (error cases)
- How fast must it be? (constraints)
- How reliable must it be? (availability, SLA)

If yes, you're ready.

---

## Part 5: Create a Decomposition Summary

Write a one-page summary that explains your decomposition:

**Decomposition Summary**

```markdown
# Grading System: Decomposition Summary

## Why We Decomposed This Way

We decomposed the grading system into [X] features because:
1. [Reason 1: e.g., "Grading and feedback are independent concerns"]
2. [Reason 2: e.g., "Features can be built in parallel"]
3. [Reason 3: e.g., "Clear integration contracts exist"]

## Features

### Feature 1: [Name]
- Purpose: [What it does]
- Owner: [Who builds it]
- Depends on: [Features it needs]
- Is depended on by: [Features that need it]
- Estimated time: [Hours to build + test]

### Feature 2: [Name]
- Purpose: [What it does]
- Owner: [Who builds it]
- Depends on: [Features it needs]
- Is depended on by: [Features that need it]
- Estimated time: [Hours to build + test]

## Integration Points

Feature 1 → Feature 2:
- Feature 1 produces: [Output format summary]
- Feature 2 expects: [Input format summary]
- Validation: Integration test will verify they match

## Risks & Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| [Risk 1] | [High/Med/Low] | [Impact] | [How to prevent] |

## Timeline

- Specification review: [Date]
- Implementation: [Start] - [End] (parallel)
- Integration testing: [Date]
- Final validation: [Date]
```

---

## Part 6: Reflection

Answer these questions:

1. **How did you decompose your system?** Why did you choose these features?

2. **What was the hardest part?** Identifying features? Defining contracts? Writing specs?

3. **What's unclear** in your decomposition that you'd like to refine?

4. **How confident are you** that team members could implement these specs without asking you constant questions?

5. **What would make these specs clearer?** More examples? More detail? Diagrams?

---

## What's Next

You've completed Part 1 of the capstone. You now have:
- Decomposed specifications for 2-3 independent features
- Clear integration contracts
- A specification package ready to hand off
- A decomposition summary explaining your decisions

In **Capstone Part 2**, you'll implement these specifications in parallel with two AI companions. You'll see whether your specs were clear enough to guide independent development.

Before moving on, make sure:
- [ ] Each feature has a standalone specification
- [ ] Integration contracts are crystal clear
- [ ] Your specification package is complete
- [ ] You've validated clarity (no obvious ambiguities)
- [ ] You have a handoff plan

**Save all your specifications. You'll use them in Part 2.**

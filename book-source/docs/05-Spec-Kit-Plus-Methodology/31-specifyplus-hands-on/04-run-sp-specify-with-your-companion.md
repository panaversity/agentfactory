---
title: "Run /sp.specify With Your Companion"
chapter: 31
lesson: 4
duration: "1.5 hours"
skills:
  - name: "Specification Automation"
    proficiency: "A2"
    category: "Technical"
  - name: "CLI Tool Usage"
    proficiency: "A2"
    category: "Technical"
  - name: "AI Tool Orchestration"
    proficiency: "B1"
    category: "Technical"
learning_objectives:
  - "Execute the /sp.specify CLI command to generate structured specifications (A2)"
  - "Interpret AI-generated specification output (A2)"
  - "Refine AI-generated specs through iterative prompting (B1)"
---

# Run /sp.specify With Your Companion

## What You're About to Discover

You've built your spec manually—asking questions, refining criteria, writing clearly. Now you're going to use SpecifyPlus's specification refinement tool: `/sp.specify`.

This command analyzes your specification and **points out gaps, ambiguities, and assumptions you didn't know you made.** You're going to run it, see what it reveals, and then collaborate with your companion to refine your spec based on the feedback.

The magic moment: you'll see that clarity isn't one-time. It's iterative.

---

## Part 1: Prepare Your Specification

First, make sure your `specs/grading-system/spec.md` has at least these sections filled in:

```markdown
---
title: "Grading System"
feature: "grading-system"
date_created: "2025-11-01"
---

# Specification: Grading System

## 1. Overview
A system that helps a teacher assign numerical scores (0-100)
and automated feedback to student assignments. Students submit
essays and code, teachers define grading rubrics, and the system
evaluates submissions against those rubrics.

## 4. Scope
### In Scope
- Teachers can create and edit rubrics with 3-5 criteria
- Students can submit essay files (.txt) and code files (.py)
- System generates scores (0-100) based on rubric criteria
- System provides feedback (50-200 words) specific to the rubric
- Teachers can review and adjust grades after assignment
- Grade history is stored per student per assignment

### Out of Scope
- User authentication or security (phase 2)
- Multi-teacher support (phase 2)
- Student-facing feedback portal (phase 2)

## 5. Success Criteria
- Each assignment can be graded in < 1 minute
- Feedback is specific to the rubric (not generic)
- Grades are stored persistently
- Teachers can edit a grade after assignment
- System works with files up to 10k words (essays) and 1k lines (code)
```

Save this. You now have a **partial but functional specification** that you've thought through carefully.

---

## Part 2: Run /sp.specify

**In your terminal, navigate to your grading-system directory:**

```bash
cd grading-system
```

**Run the specify command:**

```bash
/sp.specify
```

(Or if that doesn't work in your shell, try: `uvx specifyplus specify`)

**What happens**:

The tool reads your `specs/grading-system/spec.md`, analyzes it, and returns:

1. **A summary of what it found** (all the good stuff you did)
2. **Gaps and ambiguities** (things you didn't think about or were vague on)
3. **Clarification prompts** (questions to answer to make the spec stronger)

---

## Part 3: Analyze the Output With Your Companion

Instead of trying to interpret the /sp.specify output alone, you're going to ask your companion to help you understand it.

**Paste the output and ask:**

```
I ran /sp.specify on my grading system specification.
Here's what came back:

[paste the entire output]

Can you help me understand this?

1. What gaps did it identify?
2. For each gap, why is it important to fill?
3. Which gaps are critical (I must fix) vs. nice-to-have?
4. How would I address each gap?
```

Your companion will help you interpret something like:

> **Gaps identified:**
> 1. No specification of WHO uses the system
>    - (Critical) You assumed "one teacher" but didn't specify it formally
> 2. No error handling specified
>    - (Critical) What happens if a file upload fails?
> 3. No database/storage mechanism specified
>    - (Important) Where do grades actually get stored?
> 4. No integration points with external systems
>    - (Nice-to-have) Does this system integrate with other tools?
> 5. Feedback generation mechanism is unclear
>    - (Critical) Is feedback AI-generated or template-based?
>
> **Why they matter:**
> - If you don't specify WHO, you'll build for one user but need to support 10
> - Error handling prevents the system from crashing silently
> - Storage mechanism affects how you build the entire architecture
> - Feedback generation completely changes complexity and cost
>
> **How to fix them:**
> - Add a "Users" section specifying the audience
> - Add an "Error Handling" section with specific scenarios
> - Add a "Technical Stack" section specifying data storage
> - Add a "Feedback Generation" section specifying the mechanism

---

## Part 4: Refine Your Spec Based on Feedback

Now you're going to address the critical gaps. **Add these new sections to your spec:**

### Add: Users Section

```markdown
## 2. Users & Audience

### Primary User
- One teacher (Ms. Garcia) using the system for a single class
- Basic technical skills (can upload files, use web/CLI)
- Does not need to code or configure advanced features

### Secondary Users (Future)
- Future: Multiple teachers in a school
- Future: Student-facing feedback access
- Future: Parent dashboards

### Current Phase
- Single teacher only
- No multi-user synchronization needed
```

### Add: Error Handling Section

```markdown
## 6. Error Handling & Edge Cases

### File Upload Errors
- File too large: Return error "File exceeds 10,000 words (essays) or 1,000 lines (code)"
- Invalid format: Return error "Only .txt (essays) and .py (code) files accepted"
- Duplicate upload: Allow re-upload, overwrite previous version with warning

### Grading Errors
- Rubric criteria missing: Error "Rubric must have 3-5 criteria"
- Invalid score: Error "Score must be 0-100"
- File not found: Error "Student submission not found. Upload it first."

### All Errors
- Include timestamp of error
- Include actionable next step
- Log to file for debugging (not shown to user)
```

### Add: Technical Stack Section

```markdown
## 7. Technical Stack & Storage

### Data Storage
- Format: JSON files (no database initially)
- Storage location: ./data/grades.json, ./data/rubrics.json
- Backup: Manual backup (will be automated in phase 2)

### Feedback Generation
- Mechanism: Template-based with basic text analysis
- Not AI-generated (to keep costs low and results predictable)
- Feedback templates are built into the system
- Can be customized by teacher via YAML configuration

### External Dependencies
- None required for MVP (minimum viable product)
- Python 3.9+
- Standard library only (no pip packages)
```

---

## Part 5: Run /sp.specify Again

**After adding these sections, run /sp.specify again:**

```bash
/sp.specify
```

**What happens**:

The tool re-analyzes your spec. You should see:

1. Fewer gaps (because you filled them in)
2. Different feedback (pointing to remaining ambiguities)
3. A stronger "readiness for planning" assessment

**Ask your companion to compare:**

```
I ran /sp.specify again after refining my spec.
Here's the new output:

[paste the new output]

How much better is this second iteration?
What gaps remain that are critical?
Is this spec ready to move to planning?
```

---

## Part 6: Understand What /sp.specify Is Teaching You

The /sp.specify command isn't just pointing out gaps. It's teaching you to think like a professional software architect:

1. **Specification is iterative.** First draft won't be complete. That's okay. The tool helps you find what's missing.

2. **Gaps cascade.** If you don't specify "who uses this," you'll make wrong decisions in error handling, scale, and security.

3. **Clarity is measurable.** /sp.specify can analyze your spec and give you a score (maybe "78% ready for planning"). That score comes from checking specific things: clear inputs, clear outputs, identified assumptions, specified constraints, etc.

4. **Professional tools enforce rigor.** /sp.specify won't let you move to planning with a vague spec. It forces clarity. That feels annoying while you're writing, but it saves time downstream.

---

## Part 7: Create Your Specification Refinement Checklist

Based on what /sp.specify found, create a checklist for your next spec:

```markdown
## Specification Quality Checklist

- [ ] Overview is one sentence and makes sense to non-technical person
- [ ] Users section specifies WHO is using this
- [ ] Scope section has clear "In Scope" and "Out of Scope"
- [ ] Success criteria are SMART (Specific, Measurable, Achievable, Relevant, Time-bound)
- [ ] Technical constraints are realistic
- [ ] Error handling covers the main failure modes
- [ ] Storage mechanism is specified
- [ ] External dependencies are listed
- [ ] Assumptions are documented
- [ ] Ambiguities have been resolved

When I run /sp.specify, it should show:
- Zero critical gaps
- Zero ambiguities about inputs/outputs
- "Ready for planning" verdict
```

---

## Try With AI

You're going to practice the refinement cycle by running /sp.specify on your spec and addressing its feedback.

### Prompt 1: Simulate /sp.specify
```
I have a specification for a grading system. I've filled in the
Overview, Scope, and Success Criteria. Here's what I have:

[paste your current spec.md]

If you were an automated tool analyzing this spec for gaps,
what would you find? What's missing or ambiguous?

For each gap, tell me:
1. What's missing?
2. Why is it important?
3. How would I fix it?
```

**Expected outcome**: Your companion acts as /sp.specify and points out gaps. You see what an automated analysis would catch.

### Prompt 2: Prioritize Fixes
```
Here are the gaps you found:

[list of gaps from Prompt 1]

Which of these MUST I fix before moving to planning?
Which are nice-to-have?
In what order should I fix them?
```

**Expected outcome**: Your companion helps you prioritize. You focus on critical issues first, which is how professional teams work.

### Prompt 3: Validate Fixes
```
I've updated my spec to address the critical gaps:

[paste updated spec.md]

Now, if you were analyzing this spec again, what gaps remain?
Is this spec ready for the planning phase?
```

**Expected outcome**: You iterate until your spec is strong. This is the real specification-driven workflow: spec → analyze → refine → repeat until ready.

---

## Safety & Next Steps

You've just experienced the **specification refinement cycle**. This is professional software development: clear requirements → automated analysis → targeted refinement → stronger spec.

The key insight: **Clarity is a process, not a destination.** Your first draft doesn't need to be perfect. But you need the discipline to refine it until gaps are gone.

In the next lesson, you're going to run `/sp.plan` and watch your refined specification transform into an implementation plan. You'll see how spec quality flows directly into plan quality.

**Next**: Your clear specification will generate a clear plan that breaks your project into manageable pieces.

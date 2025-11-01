---
title: "Build Your Grading System Spec End-to-End"
chapter: 31
lesson: 7
duration: "3 hours"
skills:
  - name: "End-to-End Specification"
    proficiency: "B1"
    category: "Technical"
  - name: "Complex System Modeling"
    proficiency: "B1"
    category: "Technical"
  - name: "SDD Loop Application"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Build a complete grading system specification from problem to implementation (B1)"
  - "Apply Spec-Kit Plus tools to a real-world problem (B1)"
  - "Execute the full SDD loop: Spec → Plan → Decompose → Validate (B1)"
---

# Build Your Grading System Spec End-to-End

## What You're About to Do

You've learned the individual skills:
- Coaching clarity (Lesson 1)
- Using SpecifyPlus structure (Lesson 2)
- Writing SMART criteria (Lesson 3)
- Refining with /sp.specify (Lesson 4)
- Planning with /sp.plan (Lesson 5)
- Decomposing into tasks with /sp.tasks (Lesson 6)

Now you're going to **integrate all of these into one complete project**: building a **production-quality specification for a grading system**, running it through all the SpecifyPlus tools, and delivering something ready for implementation.

By the end, you'll have:
1. A complete `spec.md` (the specification)
2. A generated `plan.md` (the implementation plan)
3. A generated `tasks.md` (the atomic tasks)
4. A specification validation checklist proving it's ready

---

## Part 1: Start Fresh (Optional) or Complete Your Existing Spec

If you started a grading-system project in earlier lessons, open it. Otherwise, start a new one:

```bash
uvx specifyplus init grading-system-final
cd grading-system-final
```

You're now going to create a **complete specification** that addresses all the components we've discussed.

---

## Part 2: Write Your Complete Specification

Open `specs/grading-system/spec.md` and fill in all nine sections completely.

Use everything you've learned:
- Lesson 1: Six components of clarity
- Lesson 3: SMART acceptance criteria
- Lesson 4: Comprehensive gaps (users, error handling, technical stack)

**Here's a complete, production-quality specification to use as your guide:**

```markdown
---
title: "Student Assignment Grading System"
feature: "grading-system"
version: "1.0"
date_created: "2025-11-01"
---

# Specification: Student Assignment Grading System

## 1. Overview

A system that helps a single teacher grade and provide feedback on
student assignments. Teachers define grading rubrics with 3-5 criteria,
students submit essays (text files) and code (Python files), and the
system generates numerical scores (0-100) and targeted feedback based
on each rubric. Teachers can review, adjust, and access historical
grades for all students in a semester.

**Target Use Case**: A high school or university instructor with one
class (20-30 students) who wants to grade consistently and quickly
using defined rubrics.

## 2. Users & Audience

### Primary User
- **Ms. Garcia** (fictional high school teacher)
- Technical skill level: Basic (can open files, use command line or web interface)
- Pain point: Currently spends 2-3 hours grading 30 essays; wants to speed this up and ensure consistency
- Success metric: Can grade an essay with rubric feedback in < 1 minute

### Secondary Users (Future Scope - Phase 2)
- Multiple teachers (not in MVP)
- Students accessing feedback (not in MVP)
- Parents viewing grades (not in MVP)

### Current Phase Constraints
- Single teacher only
- No user authentication required
- No multi-user synchronization

## 3. Prerequisites

Before implementing this system, the implementer should have:
- Basic Python proficiency (functions, classes, dictionaries)
- Understanding of data persistence (JSON files)
- Familiarity with command-line interfaces
- No external libraries required (uses Python 3.9+ stdlib only)

## 4. Learning Objectives

After building this system, the implementer will be able to:
1. Design a data model for educational assessment (rubrics, grades, feedback)
2. Implement file upload validation with size and format constraints
3. Create template-based feedback generation (not AI-based)
4. Build persistent data storage using JSON
5. Design a command-line interface for teacher workflows
6. Test educational software for edge cases (empty submissions, oversized files)
7. Apply specification-driven development to real projects

## 5. In-Scope Features

### Rubric Definition
- Teacher can create a new rubric with 3-5 criteria
- Each criterion has: name (1-50 chars), description (0-100 words), weight (0-100%)
- Rubrics are saved and retrievable by name
- Teacher can edit or delete rubrics
- Rubric changes apply to future grading (not retroactive)

### Student Submission
- Students upload essay files (.txt, max 10,000 words)
- Students upload code files (.py, max 1,000 lines)
- System validates file format before accepting
- System validates file size before accepting
- Submission metadata stored: student name, file, timestamp, assignment ID
- Oversized or invalid files rejected with clear error message

### Grade Generation & Feedback
- System evaluates submission against selected rubric
- Generates numerical score (0-100) based on rubric criteria
- Generates feedback text (50-200 words minimum)
- Feedback must address at least 2 rubric criteria explicitly
- Feedback must include 1-2 specific examples from the submission
- Feedback uses template-based generation (not AI-generated)
- Grading completes in < 3 seconds per submission

### Grade Management
- Teacher can view all grades
- Teacher can filter grades by student name or assignment ID
- Teacher can edit a grade after it's assigned (within same session)
- Grade history is maintained (show original grade, date changed, new grade)
- Grades cannot be deleted (audit trail)

### Data Persistence
- All data (rubrics, submissions, grades) persisted to JSON files
- Data survives application restart
- Manual backup possible (files are human-readable JSON)

## 6. Out-of-Scope Features

**Explicitly NOT included in MVP:**

- User authentication or login
- Multi-teacher support
- Student-facing interface or feedback access
- Integration with learning management systems (Canvas, Blackboard, etc.)
- Database (using JSON files instead)
- AI-generated feedback (template-based only)
- Analytics, dashboards, or reporting
- Email notifications
- Plagiarism detection
- Grade weighting or final grade calculation

These are planned for Phase 2-3, but not required for MVP.

## 7. Success Criteria

The system is successful when:

1. **Speed**: Teacher can grade an assignment (evaluate against rubric, generate feedback) in < 1 minute
2. **Consistency**: Same submission evaluated twice produces same score
3. **Specificity**: Feedback addresses at least 2 rubric criteria; not generic
4. **Usability**: Teacher can complete full workflow (define rubric, upload submission, generate grade, view history) without documentation
5. **Reliability**: Grades persist across application restart
6. **Accuracy**: Score matches criteria (if essay addresses all criteria, score ≥ 80)
7. **Data Integrity**: Historical grades cannot be modified or deleted (audit trail)

## 8. Technical Constraints & Assumptions

### Technical Stack
- Language: Python 3.9+
- Storage: JSON files (no database)
- Interface: Command-line interface (no web UI for MVP)
- External dependencies: None (Python stdlib only)
- Testing: pytest for unit tests

### Performance Budgets
- Rubric creation: < 30 seconds
- File upload: < 2 seconds
- Grade generation: < 3 seconds per submission
- Grade retrieval: < 1 second

### Data Constraints
- Essays: Maximum 10,000 words (.txt files only)
- Code: Maximum 1,000 lines (.py files only)
- Rubric criteria: 3-5 per rubric
- Feedback: 50-200 words per submission
- Student limit: 100 students per semester (no hard limit, but untested beyond this)

### Assumptions
- Teacher knows what criteria should be in a rubric
- Students can upload files to the system
- Feedback templates are sufficient (AI generation not required)
- Single-threaded usage (no concurrent grading)
- Automated feedback without human review is acceptable

## 9. Error Handling & Edge Cases

### File Upload Errors
| Scenario | User Sees | System Does |
|----------|-----------|-------------|
| File too large (essay > 10k words) | "Error: Essay exceeds 10,000 words (X words submitted). Keep it brief." | Rejects upload, suggests trimming |
| File too large (code > 1k lines) | "Error: Code exceeds 1,000 lines (X lines submitted). Simplify." | Rejects upload |
| Invalid file type (.pdf instead of .txt) | "Error: Only .txt (essays) and .py (code) files accepted. You uploaded .pdf." | Rejects upload |
| Empty file | "Error: Submission is empty. Upload a non-empty file." | Rejects upload |

### Grading Errors
| Scenario | User Sees | System Does |
|----------|-----------|-------------|
| Rubric missing | "Error: Select a rubric before grading." | Prevents grading without rubric |
| Rubric has < 3 criteria | "Error: Rubric must have 3-5 criteria. Current: X." | Prevents creating invalid rubric |
| Invalid score (not 0-100) | "Error: Score must be 0-100. You entered: X." | Rejects invalid score |

### Data Errors
| Scenario | System Does |
|----------|-------------|
| grades.json corrupted | Shows error and suggests restoring from backup |
| Student submitted twice | Keeps original, warns about overwrite |

## 10. Dependencies & Integration Points

### Internal Dependencies
- Rubric management must be built before grading
- File upload handler must be built before grading
- Grade storage must be built before grading can persist

### External Dependencies
- None for MVP (uses only Python stdlib)

### Future Integration Points (Phase 2)
- Canvas/Blackboard LMS integration
- Email notifications
- Student dashboards

## 11. Risks & Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|-----------|
| Feedback generation too simplistic | Teacher unhappy with output | Medium | Use comprehensive feedback templates; allow manual editing |
| Data loss from JSON corruption | Teacher loses grade history | Low | Document manual backup; implement auto-backup in Phase 2 |
| Performance degrades with 100+ students | Slow grade retrieval | Low | Cache grade data; optimize JSON loading |
| Teacher forgets rubric criteria | Grades inconsistent | Medium | Display rubric during grading; require confirmation |

## 12. Acceptance Criteria (QA Checklist)

The system meets acceptance criteria when:

- [ ] Rubric can be created with 3-5 criteria and saved to JSON
- [ ] Rubric can be loaded from JSON and displayed
- [ ] Rubric criteria are editable and changes persist
- [ ] Essays (.txt, max 10k words) accepted; larger/invalid files rejected
- [ ] Code (.py, max 1k lines) accepted; larger/invalid files rejected
- [ ] Grade generated in < 3 seconds with score (0-100) and feedback (50-200 words)
- [ ] Feedback addresses ≥2 rubric criteria and includes ≥1 example from submission
- [ ] Grades saved to JSON and retrievable by student/assignment
- [ ] Grade history maintained (original grade, edits, timestamps)
- [ ] All errors have clear, actionable messages
- [ ] Unit tests pass with > 80% code coverage
- [ ] README with setup and usage instructions provided
- [ ] System works on Windows, Mac, and Linux

```

---

## Part 3: Validate Your Specification

Before moving forward, make sure your spec is complete. Use this checklist:

```markdown
## Specification Validation Checklist

- [ ] Overview is 2-3 sentences and explains what this system does
- [ ] Users section specifies who will use this (Ms. Garcia)
- [ ] Learning objectives are specific and measurable
- [ ] In-Scope section lists all features clearly
- [ ] Out-of-Scope section is explicit (no ambiguity about what's NOT included)
- [ ] Success criteria are measurable (< 1 minute, > 80% accuracy)
- [ ] Technical constraints are realistic (JSON, Python stdlib, no database)
- [ ] Error handling covers main failure modes (oversized files, invalid formats)
- [ ] Risks and mitigations are identified
- [ ] Acceptance criteria are testable (checkboxes, specific outcomes)

When all boxes are checked, your spec is ready for planning.
```

---

## Part 4: Run the Full SpecifyPlus Workflow

Now you're going to run all the tools in sequence:

### Step 1: Run /sp.specify

```bash
/sp.specify
```

This will analyze your spec for gaps. You might see feedback like:

> "Your spec is well-structured. Minor gaps found:
> - Consider adding a 'Data Models' section showing Rubric and Grade structures
> - Error messages should be in a separate file or config (optional for MVP)
>
> Overall: Ready for planning with these minor enhancements."

**Interpret the feedback**: If it's critical, refine your spec. If it's nice-to-have (like the data models section), you can skip it for MVP and add it later.

### Step 2: Run /sp.plan

```bash
/sp.plan
```

This generates your implementation plan, including:
- Phase breakdown (Foundation, Core Features, Polish)
- Learning objectives for each lesson
- Dependencies between lessons
- Risk mitigations

**What you're looking for**: Does the plan match your vision? Are the phases in a logical order?

### Step 3: Run /sp.tasks

```bash
/sp.tasks
```

This generates your task breakdown:
- 6-8 atomic tasks (2-4 hours each)
- Clear acceptance criteria for each
- Dependency graph

**What you're looking for**: Can you hand these to a developer (or use them with AI) and they'd know exactly what to build?

---

## Part 5: Create Your Project Artifact

You now have three documents:

1. **spec.md** — What you're building (specification)
2. **plan.md** — How you'll build it (implementation plan)
3. **tasks.md** — The atomic work units (task breakdown)

These three documents together represent a **professional specification-driven project**.

**Create a summary document:**

```markdown
# Grading System Project Summary

## What Is This?
A specification-driven software project demonstrating AI-native development methodology.

## Artifacts Included

### 1. Specification (spec.md)
The complete requirements: what this system does, who uses it, success criteria,
constraints, assumptions, error handling, and acceptance criteria.

**Quality Level**: Production-ready (could hand to a development team)

### 2. Implementation Plan (plan.md)
How to build the specification in phases: foundation (data models), core features
(grading engine), and polish (testing/documentation).

**Quality Level**: Phase-ready (tells implementers what to build first)

### 3. Task Breakdown (tasks.md)
Atomic work units (2-4 hours each) with clear acceptance criteria and dependencies.

**Quality Level**: Build-ready (implementer can start coding immediately)

## Why This Structure Matters

```
Vague Idea
  ↓ (Coached to clarity)
Specification
  ↓ (Analyzed for gaps)
Refined Specification
  ↓ (Transformed to plan)
Implementation Plan
  ↓ (Decomposed into tasks)
Atomic Tasks
  ↓ (Ready to build)
Built Project
```

Each step depends on the clarity upstream.

## Validation

This specification is ready for implementation because:

- [ ] Spec has all 12 sections and is complete
- [ ] /sp.specify shows no critical gaps
- [ ] Plan phases are logical and sequenced
- [ ] Tasks are atomic (2-4 hours each)
- [ ] Acceptance criteria are testable
- [ ] Risks are identified and mitigated
- [ ] No assumptions are hidden

## What Comes Next

With this specification, plan, and task breakdown:

1. **Option 1**: A human developer could build this
2. **Option 2**: An AI (Claude Code, etc.) could build this with your specifications
3. **Option 3**: You could build this with AI assistance, using the tasks as guidance

All three would produce similar results because the specification is clear.
```

---

## Part 6: Reflect on Your Learning

You've just completed the full specification-driven development workflow. **Write a reflection:**

```markdown
## Learning Reflection: Building a Spec End-to-End

### What Was the Biggest Insight?
[Write 2-3 sentences about the main thing you learned]

### How Did the Process Change From Lesson 1?
[Compare your vague idea (Lesson 1) to your production spec]

### What Would Happen If You Skipped Specification?
[What problems would you face if you just tried to build this without clarity?]

### How Will This Change How You Think About Projects?
[Will you use specs for your own projects? Why or why not?]

### What's the Real Skill You Learned?
[Is it Python? Is it SpecifyPlus? Or is it something deeper about clarity and planning?]
```

---

## Try With AI

You're going to complete the specification-driven workflow end-to-end with your companion's help.

### Prompt 1: Complete Your Specification
```
I'm writing a complete specification for a grading system.
I've filled in sections 1-7, but I'm stuck on sections 8-12
(technical constraints, error handling, risks, acceptance criteria).

Can you help me think through:
1. What technical constraints are realistic for this project?
2. What errors could happen and how should I handle them?
3. What are the top 3 risks?
4. What would good acceptance criteria look like?

Here's what I have so far:

[paste your incomplete spec]
```

**Expected outcome**: Your companion helps you flesh out the remaining sections.

### Prompt 2: Interpret the SpecifyPlus Feedback
```
I ran /sp.specify on my completed specification.
Here's what it returned:

[paste the output]

What does this feedback mean?
Which suggestions are critical (I must fix)?
Which are nice-to-have?
Is my spec ready for planning?
```

**Expected outcome**: You understand what gaps exist and prioritize fixes.

### Prompt 3: Validate Your Full Project
```
I now have:
- Complete specification (spec.md)
- Implementation plan (plan.md)
- Task breakdown (tasks.md)

Looking at all three together, tell me:
1. Is everything in the tasks covered by the plan?
2. Is everything in the plan justified by the specification?
3. Could someone build this system from these documents alone?
4. What would a developer ask for that's missing?
```

**Expected outcome**: You validate that the three documents work together cohesively.

---

## Safety & What's Next

You've completed the **core lesson sequence**. You now know how to:
1. Coach clarity (Lesson 1)
2. Use SpecifyPlus (Lesson 2)
3. Write SMART criteria (Lesson 3)
4. Refine with tools (Lesson 4)
5. Plan implementation (Lesson 5)
6. Decompose tasks (Lesson 6)
7. Build end-to-end (Lesson 7)

**What comes next:**

- **Mini-Project 1** (next lesson): Build a Python calculator spec-first and IMPLEMENT it
- **Mini-Project 2** (final lesson): Write a production-grade specification for a real-world grading system

These mini-projects let you apply the spec-first workflow to real code.

---

## Your Artifact: The Three-Document Specification

Save your three documents. This is professional-grade software planning. You can:

- Hand it to a developer: they'll know what to build
- Use it with AI: it will generate better code
- Refer to it for months: it's your project's source of truth

This is the power of specification-driven development.

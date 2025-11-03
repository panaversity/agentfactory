---
title: "Complete Specification Writing"
chapter: 31
lesson: 3
duration_minutes: 120

skills:
  - name: "Specification Writing"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand, Apply"
    digcomp_area: "Content Creation, Communication"
    measurable_at_this_level: "Student writes complete spec with all 6 components; components are related and consistent"

  - name: "Requirements Clarity"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Communication & Problem-Solving"
    measurable_at_this_level: "Student identifies ambiguous requirements and clarifies them; explains relationships between requirements"

  - name: "Acceptance Criteria Mastery"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student writes detailed acceptance criteria using SMART framework; criteria are testable and complete"

learning_objectives:
  - objective: "Write complete specifications with all required components and understand the relationship between components"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Given a feature description, student writes complete specification.md with all sections filled; each section is clear and internally consistent"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Specification Components, Component Relationships, Template as Tool, Ready for Planning, Revision Cycle) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Write specifications for multi-service architectures; include non-functional requirements for scale, security, compliance"
  remedial_for_struggling: "Provided outline with fill-in-the-blank sections; example specification with each section annotated"
---

# Lesson 3: Complete Specification Writing

## Opening Hook

You've learned to write SMART acceptance criteria (Lesson 1) and explored the project structure (Lesson 2). Now comes the real challenge: **How do you put it all together into a complete specification?**

A specification isn't just acceptance criteria. It's a complete description of what you're building: the problem you're solving, the requirements you're meeting, the constraints you're working within, and the criteria for success.

This lesson shows you how to write complete specifications that are so clear that anyone (human or AI) can read them and understand exactly what needs to be built.

---

## Part A: Specification Anatomy (30 min)

### The Six Components of a Complete Specification

A complete specification has six key sections. Let's explore each and understand how they work together.

#### **1. Overview: The "Why" and "What"**

This section answers: "What problem are we solving? Why does it matter?"

**Purpose**: Help readers understand the business context, not just the technical requirements.

**What to Include**:
- Problem statement (what's broken or missing?)
- Value statement (why does this matter?)
- Success vision (what does success look like?)

**Example**:
```markdown
## Overview

### Problem
Teachers spend 2-3 hours per week manually entering grades into spreadsheets.
This is error-prone, time-consuming, and prevents timely student feedback.

### Value
An automated grading system will save teachers time and enable faster feedback
to students, improving learning outcomes.

### Success Vision
Teachers can upload assignment submissions, view automatically calculated grades,
and generate grade reports in under 15 minutes per week.
```

#### **2. Scope: The Boundaries**

This section clarifies what's **in** and **out** of scope. This is critical because it prevents scope creep and misalignment.

**Purpose**: Everyone knows what's being built and what's explicitly not being built.

**What to Include**:
- **In-Scope**: Features we WILL build
- **Out-of-Scope**: Features we will NOT build (yet or ever)
- **Assumptions**: What we assume is true (environment, user knowledge, etc.)

**Example**:
```markdown
## Scope

### In Scope
- Upload student submissions (file or text)
- Define grading rubric with weighted categories
- Calculate final grade from rubric scores
- Display grades to students
- Export grade report as CSV

### Out of Scope
- Email notifications (Phase 2)
- Mobile app (Phase 2)
- Plagiarism detection (future)
- Integration with learning management system (Phase 2)

### Assumptions
- Teachers use standard web browsers (Chrome, Firefox, Safari)
- Python 3.13+ is available on deployment machine
- File uploads are under 50MB each
- Up to 100 students per teacher (Phase 1)
```

#### **3. Requirements: The "What to Build"**

This section lists all functional and non-functional requirements. Functional requirements are what the system does. Non-functional requirements are how well it does it.

**Purpose**: Comprehensive list of behaviors the system must exhibit.

**Functional Requirements** (behaviors):
- "User can upload submission files"
- "System calculates weighted grades"
- "User can export grade report"

**Non-Functional Requirements** (performance, reliability, security, scalability):
- "Response time ≤ 1 second for grade calculations"
- "System supports 1,000 concurrent users"
- "All student data encrypted at rest"
- "99.9% uptime SLA"

**Example**:
```markdown
## Requirements

### Functional
1. Teachers can upload student submissions (docx, pdf, txt)
2. Teachers can define grading rubric with custom categories
3. Teachers can assign scores to submissions based on rubric
4. System calculates weighted final grade from category scores
5. Students can view their grades and rubric feedback
6. Teachers can export grade report (CSV or PDF)
7. Teachers can view class-wide grade statistics

### Non-Functional
1. Performance: Grade calculation ≤ 500ms even with 100 students
2. Reliability: System handles file upload errors gracefully (max retry 3x)
3. Security: All passwords hashed with bcrypt; sessions expire after 1 hour
4. Scalability: Initial support for 100 students per teacher (design for 1000)
5. Accessibility: WCAG 2.1 AA compliance (text alternatives, keyboard navigation)
```

#### **4. Acceptance Criteria: The "How to Verify"**

This is where your SMART criteria from Lesson 1 go! These are testable conditions that prove the system works.

**Purpose**: Define exactly what "done" means. Each criterion is verifiable (pass/fail).

**Example**:
```markdown
## Acceptance Criteria

### Grading Accuracy
- When teacher assigns scores to a 3-category rubric (homework 30%, participation 20%, exam 50%),
  the system calculates final grade to ±0.1% accuracy
- Criterion: Test with 50 student submissions; verify accuracy against manual calculation

### Performance
- Grade calculation for 100 students completes in ≤ 500ms
- Criterion: Load test with 100 student submissions; measure calculation time

### Student Feedback
- Students view rubric feedback within 1 second of accessing their grades
- Criterion: Time from page load to rubric display ≤ 1 second
```

#### **5. Constraints: The "Limitations"**

This section defines what limits the solution: time, budget, technical, legal, or domain constraints.

**Purpose**: Be realistic about what's possible.

**What to Include**:
- **Time**: "Must ship by Q1 end"
- **Budget**: "Max 40 engineering hours"
- **Technical**: "Only Python; no external ML libraries"
- **Domain**: "Only for K-12 education (not higher ed yet)"
- **Legal**: "Must comply with FERPA (student data privacy)"

**Example**:
```markdown
## Constraints

### Time
- MVP must ship by end of Week 4
- Teachers can view grades by Week 2 (interim milestone)

### Technical
- Python 3.13+ only
- No external ML libraries (calculate grades with basic math only)
- SQLite for data storage (no database server required)

### Domain
- Designed for K-12 teachers (not higher education)
- Supports standard grading (0-100 scale)
- Does not support +/- grading or pass/fail systems (Phase 2)

### Legal
- Must comply with FERPA (student data privacy regulations)
- All data exported with student consent only
```

#### **6. Success Criteria: The "We Won"**

This section defines how you know you've succeeded. These are higher-level outcomes, not technical requirements.

**Purpose**: Align success with business goals, not just feature checklists.

**What to Include**:
- Measurable outcomes (user adoption, time saved, error reduction)
- User satisfaction indicators
- Performance targets

**Example**:
```markdown
## Success Criteria

### Adoption
- 10+ teachers using system in pilot (Week 2)
- Teachers report ≥ 4/5 satisfaction (Week 3)

### Time Saved
- Teachers save ≥ 2 hours per week (vs. manual grading)
- Grade feedback delivered to students within 24 hours (vs. 1 week)

### Accuracy
- Zero calculation errors in pilot (100% accuracy)
- All student data secured (zero unauthorized access)

### Reliability
- 99.9% uptime during school hours (7am-5pm)
- All uploaded files recoverable (backup + restore verified)
```

### How Components Relate

These six sections form a coherent whole:

```
Overview (Why?)
  ↓
Scope (What's in/out?)
  ↓
Requirements (What must it do?)
  ↓
Acceptance Criteria (How to verify?)
  ↓
Constraints (What limits us?)
  ↓
Success Criteria (Did we succeed?)
```

Each section flows from the previous. The Overview sets context. Requirements detail what addresses that context. Acceptance Criteria verify requirements are met. Success Criteria measure if the whole effort succeeded.

### What Makes a Spec "Ready for Planning"

A specification is ready to move to the plan phase when:

1. **Overview is clear**: Someone unfamiliar with the project can understand the problem in 2 minutes
2. **Scope is explicit**: "In" and "Out" lists are complete; no ambiguity
3. **Requirements are testable**: Each requirement can be verified (pass/fail)
4. **Acceptance Criteria are SMART**: From Lesson 1 — Specific, Measurable, Achievable, Relevant, Time-bound
5. **Constraints are realistic**: Timelines, budgets, and technical limits are grounded in reality
6. **Success Criteria are measurable**: You can run a report and say "Yes, we succeeded" or "No, we didn't"

If your spec has all six sections filled, you're ready. If one section is missing or vague, you need to revise.

---

## Part B: Write a Complete Spec (50 min)

### Choose Your Project

**Option A: Python Calculator** (Simpler)
- Features: add, subtract, multiply, divide, exponentiation, square root, history
- Scope: CLI application, JSON file storage
- Complexity: 2-3 hours implementation

**Option B: Grading System** (More Complex)
- Features: submit assignments, grade rubric, calculate final grades, view feedback, export reports
- Scope: Simple web interface, SQLite storage
- Complexity: 3-4 hours implementation

### Exercise: Write Your Complete Specification

**Instructions**: Choose one project. Using the six components above, write a complete specification. Use the provided template structure.

**Template Structure**:

```markdown
# Specification: [Project Name]

## Overview
[Problem statement, value statement, success vision — 3-5 sentences]

## Scope
### In Scope
[4-6 bullet points of features you WILL build]

### Out of Scope
[3-5 bullet points of features you will NOT build]

### Assumptions
[2-3 assumptions about environment, user knowledge, constraints]

## Requirements
### Functional
[6-8 features the system must have]

### Non-Functional
[3-5 performance, security, reliability, scalability requirements]

## Acceptance Criteria
[5-10 SMART criteria using the framework from Lesson 1]

## Constraints
### Time
[Deadline and milestones]

### Technical
[Technology stack, limitations, dependencies]

### Domain
[What you're NOT supporting yet]

### Legal/Compliance
[Regulatory or policy constraints]

## Success Criteria
[3-5 measurable outcomes indicating success]
```

### Sub-Exercise: Self-Review Checklist

After writing, review your spec:

**Overview**:
- [ ] Clear problem statement (1-2 sentences)?
- [ ] Someone unfamiliar with the project understands in 2 minutes?

**Scope**:
- [ ] At least 4 "in-scope" features listed?
- [ ] At least 3 "out-of-scope" items explicitly excluded?
- [ ] 2-3 realistic assumptions documented?

**Requirements**:
- [ ] 6+ functional requirements?
- [ ] 3+ non-functional requirements (performance, security, scalability)?
- [ ] Each requirement testable (can you verify it)?

**Acceptance Criteria**:
- [ ] 5+ criteria using SMART framework (from Lesson 1)?
- [ ] Each criterion includes how to test/verify?
- [ ] Specific numbers or pass/fail conditions?

**Constraints**:
- [ ] Timeline explicit (deadline, milestones)?
- [ ] Technical stack clear (Python? Web? Database?)?
- [ ] What's explicitly NOT in Phase 1?

**Success Criteria**:
- [ ] 3+ measurable outcomes?
- [ ] Aligned with business goals (not just technical checklists)?
- [ ] Can be verified with a test or survey?

### Sub-Exercise: Peer Review

**Instructions**: Trade specs with a partner (or a colleague).

**Reviewer's Task**:
- Read the spec aloud in your own words (2-3 minutes)
- Answer these questions:
  - Do you understand the problem?
  - What's being built? What's not?
  - How would you know if it's done?
- Provide feedback: "This is unclear because..." or "This is great because..."

**Author's Task**:
- Listen without defending
- Ask: "What would make this clearer?"
- Revise based on feedback (even one peer's confusion is a sign of ambiguity)

---

## Part C: Revision and Iteration

### Specs Improve Through Feedback

Write this down: **No specification is perfect on first draft.** Great specs go through iterations:

1. **Draft 1**: Initial attempt, captures main ideas
2. **Feedback Round 1**: Peer review identifies 2-3 ambiguities
3. **Draft 2**: Clarify overview, add missing scope items, refine requirements
4. **Feedback Round 2**: Different reviewer (or same peer) says "Much better! Just clarify X"
5. **Draft 3**: Final version, clear and ready for planning

This is normal. This is good. Iteration leads to clarity.

---

## Part D: Key Insight — Templates Force Completeness

Write this down: **The specification template isn't bureaucracy. It's a thinking tool.**

By filling out all six sections, you force yourself to:
- Define the **why** (Overview)
- Set **boundaries** (Scope)
- List **what to build** (Requirements)
- Define **how to verify** (Acceptance Criteria)
- Acknowledge **limitations** (Constraints)
- Measure **success** (Success Criteria)

Projects that skip this step often fail because they later discover they were building different things. "I thought you meant X, but you meant Y." The specification prevents that miscommunication.

---

## Try With AI

**Tool**: ChatGPT web (or your AI companion if already set up)
**Duration**: 10-15 minutes

### Prompts

1. **Review Completeness**:
   ```
   Here's my specification: [paste your complete spec]

   Does this specification have all six sections?
   (Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria)

   What's missing or unclear?
   ```

2. **Validate Ambiguity**:
   ```
   I wrote this requirement: [paste one requirement from your spec]

   Is this ambiguous? Could two developers interpret this differently?
   How would you make it more specific?
   ```

3. **Test Clarity**:
   ```
   Here's my specification overview and scope:
   [paste Overview and Scope sections]

   If you were implementing this, would you understand what to build and what NOT to build?
   What questions would you ask?
   ```

### Expected Outcomes

- You see how AI can help validate specification completeness
- You understand what "clear enough" means for implementation
- You identify ambiguities in your own writing
- You're ready for Lesson 4 with a solid, complete specification
- You understand specifications aren't one-time documents—they improve through feedback

### Safety & Ethics Note

**Remember**: A specification is your contract with the implementer (human or AI). If it's unclear, implementation will be unclear. Specifications are the most important artifact you create. Invest time in clarity.

---

## Checkpoint: Are You Ready for Lesson 4?

Before moving forward, verify you can do these:

- [ ] Write all six specification sections (Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria)
- [ ] Explain how each section relates to the others
- [ ] Write acceptance criteria using SMART framework (from Lesson 1)
- [ ] Identify ambiguous requirements and clarify them
- [ ] Receive peer feedback and revise your specification
- [ ] Understand that "ready for planning" means all six sections are clear and complete

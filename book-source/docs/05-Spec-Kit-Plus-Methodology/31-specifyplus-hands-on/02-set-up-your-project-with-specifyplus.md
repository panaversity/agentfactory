---
title: "Set Up Your Project With SpecifyPlus"
chapter: 31
lesson: 2
duration: "1.5 hours"
skills:
  - name: "Tool Installation"
    proficiency: "A2"
    category: "Technical"
  - name: "Project Initialization"
    proficiency: "A2"
    category: "Technical"
  - name: "Environment Configuration"
    proficiency: "A2"
    category: "Technical"
learning_objectives:
  - "Install Spec-Kit Plus tooling and dependencies (A2)"
  - "Initialize a new project with specification structure (A2)"
  - "Understand directory organization for SDD workflows (A2)"
---

# Set Up Your Project With SpecifyPlus

## What You're About to Discover

In Lesson 1, you discovered the six components of a good specification by coaching your companion. Now you're going to see how SpecifyPlus—a professional tool—organizes those six components into a formal structure.

You won't learn about SpecifyPlus by reading documentation. Instead, you'll **use it**, explore the files it creates, and ask your companion to explain why it's organized that way. By the end, you'll understand the tool by using it—which is much faster than reading about it.

---

## Part 1: Initialize Your SpecifyPlus Project

You're going to use the grading system idea from Lesson 1. Let's formalize it with SpecifyPlus.

**Open your terminal and run:**

```bash
uvx specifyplus init grading-system
```

This command does several things:
1. Creates a new directory called `grading-system`
2. Initializes it as a SpecifyPlus project
3. Creates a folder structure with templates for specification, planning, and tasks

**What you should see:**

```
grading-system/
├── .specify/
│   ├── templates/
│   │   ├── spec-template.md
│   │   ├── plan-template.md
│   │   └── tasks-template.md
│   ├── memory/
│   │   └── constitution.md
│   └── config.json
├── specs/
│   └── grading-system/
│       ├── spec.md
│       ├── plan.md
│       └── tasks.md
└── history/
    └── prompts/
```

---

## Part 2: Explore the Structure With Your Companion

Now you're going to explore what was created. **Don't try to understand it from this lesson—ask your companion to explain it.**

**Give your companion this prompt:**

```
I just ran:
  uvx specifyplus init grading-system

This created a folder structure. Here's what I see:

grading-system/
├── .specify/
│   ├── templates/
│   │   ├── spec-template.md
│   │   ├── plan-template.md
│   │   └── tasks-template.md
│   ├── memory/
│   │   └── constitution.md
│   └── config.json
├── specs/
│   └── grading-system/
│       ├── spec.md
│       ├── plan.md
│       └── tasks.md
└── history/
    └── prompts/

Can you explain:
1. Why these three directories (.specify, specs, history)?
2. What goes in each?
3. Why are there three files in specs/grading-system (spec.md, plan.md, tasks.md)?
```

Your companion will explain something like:

> ".specify/ holds all the tools and templates that guide your work.
>
> - templates/ has the formats you'll fill in (spec, plan, tasks)
> - memory/ has the constitution—your project's values and principles
> - config.json has settings
>
> specs/ is where YOUR specification documents live:
> - spec.md: What are you building? (the six components from Lesson 1)
> - plan.md: How will you build it? (the architecture and design)
> - tasks.md: Break the plan into atomic work units you can actually build
>
> history/ records every decision and prompt you use. This creates a learning record.
>
> Together, these three files represent the progression: Specification → Planning → Implementation Tasks."

**Key insight**: This structure **forces a workflow**. You can't jump to tasks without a plan. You can't have a good plan without a clear specification. The folder structure *teaches* you the right order.

---

## Part 3: Open Your First Template

Navigate to your new project:

```bash
cd grading-system
```

Now open `specs/grading-system/spec.md` in your text editor. You'll see something like this:

```markdown
---
title: "Grading System"
feature: "grading-system"
date_created: "2025-11-01"
---

# Specification: Grading System

## 1. Overview
[Brief description of what you're building]

## 2. Prerequisites
[What knowledge/setup is required before this project?]

## 3. Learning Objectives
[What will the implementer learn/be able to do?]

## 4. Scope
### In Scope
[What IS included]

### Out of Scope
[What is NOT included]

## 5. Success Criteria
[How do we know it works?]

## 6. Technical Constraints
[Performance, security, scalability requirements]

## 7. Assumptions
[What are we taking for granted?]

## 8. Risks & Mitigations
[What could go wrong? How do we handle it?]

## 9. Acceptance Criteria
[Specific, testable conditions for completion]
```

---

## Part 4: Compare Lesson 1 to SpecifyPlus Structure

Now you're going to map what you learned in Lesson 1 to what SpecifyPlus expects.

**Here's the comparison:**

| What You Discovered (Lesson 1) | What SpecifyPlus Calls It |
|--------------------------------|---------------------------|
| **Overview** (What are we building?) | Section 1: Overview |
| **Inputs** (What goes in?) | Implicit in Scope (In Scope section) |
| **Outputs** (What comes out?) | Implicit in Scope (In Scope section) |
| **Success Criteria** (How do we know it works?) | Section 5: Success Criteria |
| **Constraints** (What are the boundaries?) | Sections 6-7: Constraints and Assumptions |
| **Assumptions** (What are we taking for granted?) | Section 7: Assumptions |

**The insight**: The six components you discovered naturally are embedded in SpecifyPlus, but organized more formally. SpecifyPlus adds more structure (Prerequisites, Learning Objectives, Acceptance Criteria) because it's designed for professional projects that need clarity on multiple dimensions.

---

## Part 5: Discover the Workflow

**Give your companion this prompt:**

```
I'm looking at the SpecifyPlus structure. I see three files
that go in sequence: spec.md → plan.md → tasks.md

What's the relationship between them? Why is this order important?
Can I skip directly to tasks.md without writing plan.md?

Also: what makes a good specification good enough to move to planning?
```

Your companion will explain something like:

> "Great question. Here's the cascade:
>
> **spec.md** answers: WHAT are we building? (the problem and requirements)
>
> **plan.md** answers: HOW will we build it? (the design, architecture, phases)
> - Plan flows FROM the spec
> - Plan shows how your spec requirements will be implemented
> - If your spec is vague, the plan will be vague too
>
> **tasks.md** answers: WHO does WHAT by WHEN? (the atomic work units)
> - Tasks flow FROM the plan
> - Each task is small enough for one person to do (ideally 2-4 hours)
> - If your plan is unclear, tasks will be either too big or redundant
>
> You cannot skip steps. If you try to write tasks without a plan,
> you'll miss connections between them. If you write a plan without
> a spec, you'll plan the wrong thing.
>
> For a spec to be 'ready for planning,' it needs:
> - Clear overview that makes sense to a non-technical person
> - All inputs and outputs explicitly named
> - Success criteria that are measurable (not 'good' but 'scores 100/100')
> - Constraints that are realistic
> - Assumptions that could be challenged"

---

## Part 6: Understand Why This Structure Matters

You've now seen three key things:

### 1. The Folder Structure Forces a Workflow
- You *can't* write good tasks without a good plan
- You *can't* write a good plan without a clear spec
- The folders literally organize this progression

### 2. The Templates Guide Your Thinking
- Each template has sections that ask the right questions
- You're not writing from a blank page; you're filling in a framework
- The framework reflects the thinking of dozens of software teams

### 3. The Specification → Plan → Tasks Cascade
- Specification quality determines plan quality
- Plan quality determines task quality
- Poor upstream → chaos downstream

This is professional software development. It's not magic. It's **structure**.

---

## Part 7: Customize Your Project

Now you're going to personalize your SpecifyPlus project with your own information.

**Open `specs/grading-system/spec.md` and fill in the Overview section:**

```markdown
## 1. Overview

A system that helps a teacher assign numerical scores (0-100) and
automated feedback to student assignments. Students submit essays
and code, teachers define grading rubrics, and the system evaluates
submissions against those rubrics.
```

**Fill in the Scope section:**

```markdown
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
- Analytics or reporting dashboards (phase 3)
```

**Fill in Success Criteria:**

```markdown
## 5. Success Criteria
- Each assignment can be graded in < 1 minute
- Feedback is specific to the rubric (not generic)
- Grades are stored persistently
- Teachers can edit a grade after assignment
- System works with files up to 10k words (essays) and 1k lines (code)
```

Don't fill in every section yet. Just these three. This is enough to see how your specs take shape.

---

## Part 8: Your Specification Artifact

**Save your partially-filled spec.md.** You now have:
- A SpecifyPlus project initialized
- The six components from Lesson 1 formalized into a framework
- A partially-filled specification that guides what comes next

This artifact—your spec.md file—is going to grow as you move through the next lessons. It's the source of truth for everything you build.

---

## Try With AI

You're going to explore the SpecifyPlus structure in your own project by asking your companion questions about why things are organized the way they are.

### Prompt 1: Understand the Templates
```
I have a SpecifyPlus project initialized. I see three template files:
- spec-template.md
- plan-template.md
- tasks-template.md

Can you walk me through what each template is asking for?
For each template, tell me:
1. What's the primary question it answers?
2. What sections does it have?
3. Why are those sections in that order?
```

**Expected outcome**: Your companion explains the thinking behind the templates. This gives you context for why they exist, not just what they contain.

### Prompt 2: Map Your Spec to the Template
```
I've started filling in my spec.md with the grading system.
I've completed the Overview, Scope, and Success Criteria sections.

What sections should I fill in next? What's the natural order?
And what would a complete, ready-for-planning spec look like?
```

**Expected outcome**: Your companion gives you a roadmap for completing the spec. You understand not just WHAT to fill in, but WHY and IN WHAT ORDER.

### Prompt 3: Understand the Cascade
```
Now I understand spec.md. But I'm curious: what would a plan.md
look like for a grading system? What would it contain that spec.md
doesn't? Can you show me a brief example?
```

**Expected outcome**: You get a glimpse of what comes next, which helps you understand why the spec needs to be so clear. You see the connection between the three files.

---

## Safety & Next Steps

You've now moved from abstract understanding ("What's a spec?") to concrete usage ("Here's my SpecifyPlus project, and here's my spec.md file").

In the next lesson, you'll focus on the most important skill: **writing success criteria that are so clear that even an AI can't misinterpret them.**

**Next**: You'll learn how to make acceptance criteria SMART—and you'll see what happens when criteria are vague vs. clear by watching your companion build different things based on your spec quality.

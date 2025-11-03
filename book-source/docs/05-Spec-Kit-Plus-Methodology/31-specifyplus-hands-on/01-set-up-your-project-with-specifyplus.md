---
title: "Set Up Your Project With SpecifyPlus"
chapter: 31
lesson: 1
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

This lesson introduces how SpecifyPlus organizes specifications and the end-to-end SDD workflow.

You won't learn about SpecifyPlus by reading documentation. Instead, you'll use it, explore the files it creates, and ask your companion to explain why it's organized that way. By the end, you'll understand the tool by using it—which is much faster than reading about it.

---

## Part 1: Initialize Your SpecifyPlus Project

You're going to use the grading system idea for hands-on practice. Let's formalize it with SpecifyPlus.

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

Now you're going to explore what was created. Don't try to understand it from this lesson—ask your companion to explain it.

**Give your companion this prompt:**

```
I just ran:
  uvx specifyplus init grading-system

This created a folder structure. Here's what I see:

[folder tree]

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
> - spec.md: What are you building?
> - plan.md: How will you build it?
> - tasks.md: Break the plan into atomic work units you can actually build
>
> history/ records every decision and prompt you use. This creates a learning record.
>
> Together, these three files represent the progression: Specification → Planning → Implementation Tasks."

**Key insight**: This structure forces a workflow. You can't jump to tasks without a plan. You can't have a good plan without a clear specification. The folder structure teaches you the right order.

References:
- Setup: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/00_setup/readme.md`
- PHR: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/01_phr/readme.md`
- Spec: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/03_spec/readme.md`

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

## Part 4: Discover the Workflow

Give your companion this prompt:

```
I'm looking at the SpecifyPlus structure. I see three files
that go in sequence: spec.md → plan.md → tasks.md

What's the relationship between them? Why is this order important?
Can I skip directly to tasks.md without writing plan.md?

Also: what makes a good specification good enough to move to planning?
```

---

## Part 5: Customize Your Project

Open `specs/grading-system/spec.md` and fill in the Overview, Scope, and Success Criteria sections (keep examples Python-only for MVP).

---

## Acceptance Checklist (Lesson 1)

- [ ] Initialized a project (or reviewed structure) with `.specify/`, `specs/`, `history/`
- [ ] Located templates: `spec-template.md`, `plan-template.md`, `tasks-template.md`
- [ ] Opened `specs/<feature>/spec.md` and identified core sections
- [ ] Can explain why the order is Spec → Plan → Tasks

---

## Safety & Next Steps

Next lesson: write SMART acceptance criteria so even an AI can't misinterpret them.

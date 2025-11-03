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

This lesson introduces how SpecifyPlus organizes specifications and the end-to-end SDD workflow.

You won't learn about SpecifyPlus by reading documentation. Instead, you'll use it, explore the files it creates, and ask your companion to explain why it's organized that way. By the end, you'll understand the tool by using it—which is much faster than reading about it.

---

## Part 1: Initialize Your SpecifyPlus Project

You're going to use the grading system idea for hands-on practice. Let's formalize it with SpecifyPlus.

**Open your terminal and run:**

```bash
uvx specifyplus init grading-system
```

Select your AI Companion and Terminal (Bash or Powersheel)

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
```

---

## Part 2: Explore the Structure With Your Companion

Now you're going to explore what was created. Don't try to understand it from this lesson—ask your companion to explain it.

**Give your companion this prompt:**

```
I just ran:
  uvx specifyplus init grading-system

This created a folder structure.

Can you explain:
1. Why these directories?
2. What goes in each?
3. Why are there three files?
```

Your companion will explain something like:

> ".specify/ holds all the tools and templates that guide your work.
>
> - templates/ has the formats you'll fill in (spec, plan, tasks)
> - memory/ has the constitution—your project's values and principles

Together, these three files represent the progression: Specification → Planning → Implementation Tasks."

**Key insight**: This structure forces a workflow. You can't jump to tasks without a plan. You can't have a good plan without a clear specification. The folder structure teaches you the right order.

---

## Part 3: Open Your First Template

Navigate to your new project:

```bash
cd grading-system
```

---

## Part 4: Discover the Workflow


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

---
title: "Installation and Setup"
chapter: 14
lesson: 2
duration_minutes: 30
proficiency_level: "A2"
cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (directory structure, .specify folder, .claude folder, specs organization) within A2 limit of 7 ✓"

learning_objectives:
  - objective: "Create a Spec-Kit Plus project folder structure"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successfully create project folders with correct hierarchy"

  - objective: "Understand the purpose of .specify/, .claude/, and specs/ directories"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explain where each type of artifact belongs"

  - objective: "Locate the constitution file in .specify/memory/"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Open and read constitution.md from correct location"

  - objective: "Verify your AI companion can access the project folder"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successfully open project folder in AI tool and confirm readability"

skills:
  - name: "Project Structure Creation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"

  - name: "Directory Purpose Understanding"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"

generated_by: "content-implementer v1.0.0"
source_spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Installation and Setup

In Lesson 1, you learned what Spec-Kit Plus is: a framework for capturing both working artifacts AND reusable intelligence. Now you'll set up your first project.

This lesson is simple: you'll create a folder structure that tells your AI companion where to find decisions, specifications, and artifacts. No complex installations—just folders and files.

By the end of this lesson, you'll have:
- A project folder ready for your research paper
- Clear directory structure so artifacts go in the right places
- Your AI companion able to access and read the entire project
- Understanding of where each type of thinking gets saved

---

## Overview: What You're Creating

You'll create a project folder for writing a research paper using Spec-Kit Plus workflow. The structure looks like this:

```
my-research-paper/
├── .specify/
│   └── memory/
│       └── constitution.md
├── .claude/
│   └── skills/
├── specs/
│   └── research-paper/
│       ├── spec.md
│       ├── plan.md
│       └── tasks.md
└── paper.md
```

**What each folder does**:

- **`.specify/memory/`** — Stores your project constitution (the decision framework for your entire project). Think of it as your project's rulebook: "When we make tradeoffs, what matters most?"

- **`.claude/skills/`** — Stores reusable skills your AI companion creates. Right now it's empty, but later it will hold prompts and frameworks you can reuse on future projects.

- **`specs/research-paper/`** — Stores your specifications, plans, and tasks for the research paper project. These are the thinking documents that guide your AI companion's work.

- **`paper.md`** — Your actual research paper file (the deliverable).

---

## Step 1: Create the Project Folder

Open your terminal and run:

```bash
mkdir my-research-paper
cd my-research-paper
```

You now have a folder called `my-research-paper`. Inside, you'll create the subdirectories.

---

## Step 2: Create the .specify Directory Structure

The `.specify/memory/` folder stores your project constitution. Create it:

```bash
mkdir -p .specify/memory
```

The `-p` flag means "create parent folders if they don't exist," so this creates both `.specify/` AND `memory/` in one command.

**What does this folder do?**

Your `.specify/memory/` folder will contain a file called `constitution.md`. This file captures the principles that guide your entire project:

- What kind of research paper are you writing? (Academic, technical, popular science?)
- What's your audience? (PhD students, industry practitioners, interested amateurs?)
- What quality standards matter most? (Rigor, clarity, novelty, practical application?)
- How do you evaluate tradeoffs? (If a section could be deeper OR clearer, which wins?)

The constitution is written ONCE at the project start, then referenced throughout. When your AI companion needs to make decisions, it reads the constitution and knows what matters most in your project.

---

## Step 3: Create the .claude Directory Structure

The `.claude/` folder stores things that help your AI companion work with your project. Create it:

```bash
mkdir -p .claude/skills
```

**What does this folder do?**

Right now, `skills/` is empty. But as you work through this chapter, you'll create reusable skills—collections of prompts and reasoning patterns that work for similar problems. By the end of Chapter 14, you might have skills like:

- "Section Writer Skill" (knows how to write clear paper sections)
- "Research Validator Skill" (checks that sources are credible)
- "Outline Refiner Skill" (improves paper structure)

These skills live in `.claude/skills/` so they're easy to find and reuse in future projects.

---

## Step 4: Create the specs Directory Structure

Specifications are the thinking documents that guide your project. Create the folder:

```bash
mkdir -p specs/research-paper
```

**What does this folder do?**

Inside `specs/research-paper/`, you'll create three documents:

**`spec.md`** — What are you building?
- Clear statement of the paper's purpose
- Target audience
- Scope (what's in, what's out)
- Success criteria (how do you know this paper is done?)

**`plan.md`** — How will you build it?
- Architecture of the paper (intro, lit review, methodology, results, discussion, conclusion)
- Key sections and their relationships
- Dependencies between sections

**`tasks.md`** — What are the specific, actionable steps?
- Write introduction section
- Conduct literature review
- Analyze research findings
- Each task is small, completable, and testable

You'll create these files in later lessons. For now, just understand that this is where they live.

---

## Step 5: Create the paper.md File

This is your actual research paper deliverable. Create an empty file:

```bash
touch paper.md
```

You'll write your paper in this file as you work through the specification, planning, and task execution phases.

---

## Step 6: Verify Your Folder Structure

Let's confirm everything was created correctly. Run:

```bash
find . -type d -o -type f | sort
```

You should see:

```
.
./.claude
./.claude/skills
./.specify
./.specify/memory
./paper.md
./specs
./specs/research-paper
```

**Explanation of what you see**:

- **.specify/** and **.claude/** start with dots because they're configuration folders (hidden by default in file explorers, but very important)
- **specs/research-paper/** is a regular folder where your thinking documents go
- **paper.md** is your deliverable (the actual research paper you'll write)

---

## Step 7: Create a Minimal Constitution

Now you'll create your first constitution—the decision framework for your research paper project. This is a starter version (you'll expand it in Lesson 3 using `/sp.constitution`).

Create the file:

```bash
touch .specify/memory/constitution.md
```

Open it in your text editor and add:

```markdown
# Constitution: Research Paper Project

## Core Principle

This research paper prioritizes **clarity for the target audience** over academic formality. Readers should understand complex ideas without prior expertise in the field.

## Decision Framework

When we make tradeoffs in this project:

1. **Clarity > Formality** — Explain ideas clearly, even if less academic
2. **Rigor > Brevity** — Spend extra space on careful reasoning
3. **Evidence > Speculation** — Every claim backed by citation or demonstration

## Success Criteria

The paper succeeds when:
- A reader with basic knowledge can understand the main argument
- Every claim is supported with evidence
- The structure guides the reader logically
```

**Why create this now?**

You're establishing what your project values. When your AI companion helps you write the specification in Lesson 3, it will read this constitution and align its recommendations with your values.

---

## Understanding Where Each Artifact Goes

Here's the mental model for Spec-Kit Plus artifact organization:

| When you... | It goes in... |
|-------------|---------------|
| Define project values | `.specify/memory/constitution.md` |
| Write a specification | `specs/research-paper/spec.md` |
| Create an implementation plan | `specs/research-paper/plan.md` |
| Break plan into tasks | `specs/research-paper/tasks.md` |
| Create reusable skills | `.claude/skills/` |
| Write the actual paper | `paper.md` |

This structure keeps everything organized so you can:
1. Find past decisions when making new ones
2. Reuse skills on future projects
3. See how specifications led to final deliverables

---

## Try With AI

Now verify that your AI companion can access and read your project folder.

**Setup:** Open your terminal in the project folder and launch Claude Code or Gemini CLI.

```bash
cd my-research-paper
claude
```

(Or `gemini` if using Gemini CLI)

**Prompt 1: Verify Project Access**
> "I've created a Spec-Kit Plus project structure for a research paper. Walk me through what you see in the project folder. Show me the directory structure and explain what each folder (.specify/, .claude/, specs/) will contain."

**Expected Response**: Your AI should describe your folder structure, explaining that .specify/memory/ holds the constitution, specs/ holds thinking documents, and .claude/skills/ will hold reusable patterns.

**Prompt 2: Understand the Constitution**
> "Read the constitution I wrote in .specify/memory/constitution.md and tell me: What does this constitution value most? If you were writing the specification for my research paper, how would this constitution guide your recommendations?"

**Expected Response**: Your AI should understand that clarity matters most, and explain how that would influence spec decisions.

**Prompt 3: Preview Next Steps**
> "Based on my project structure and constitution, what should I do next to start writing my research paper specification? What questions should I answer?"

**Expected Response**: Your AI should suggest using `/sp.specify` or asking about paper purpose, target audience, scope, and success criteria—preparing you for Lesson 3.

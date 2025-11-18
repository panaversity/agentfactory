---
title: "Headings - Creating Document Hierarchy"
description: "Learning to structure specifications with clear heading hierarchy"
sidebar_label: "Headings - Creating Document Hierarchy"
sidebar_position: 2
chapter: 9
lesson: 2
duration_minutes: 40
proficiency: "A2"
concepts: 2

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Creating Heading Hierarchy"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create proper heading hierarchy in specification documents"

  - name: "Understanding Document Structure"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why heading hierarchy matters for AI parsing and human readability"

learning_objectives:
  - objective: "Create proper heading hierarchy using hash symbols"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Practice exercise with validation checklist"

  - objective: "Identify when to use each heading level (1-4)"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Error identification exercise"

cognitive_load:
  new_concepts: 2
  assessment: "2 new concepts (hash syntax and hierarchy principle) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore semantic HTML heading structure (h1-h6) and how markdown headings map to HTML; analyze accessibility implications for screen readers"
  remedial_for_struggling: "Focus on just two levels (# for title, ## for sections); practice with simple 3-section documents before adding level 3 headings"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-chapter-9-markdown/spec.md"
created: "2025-11-06"
last_modified: "2025-11-07"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.1"
---

# Headings - Creating Document Hierarchy

Imagine trying to find information in a document that's just one long wall of text. You'd scroll endlessly, hunting for the part you need. Now imagine that same document with clear sections: "Problem," "Solution," "Features," "Installation." Suddenly you can scan it in seconds.

In markdown, headings create this structure. They organize your document into sections that both humans and AI agents can quickly understand. When you write a specification, headings tell the AI: "This is the problem. These are the features. This is what the output should look like."

This lesson teaches you how to create clear document structure using headings.

---

## Concept 1: Using Hash Symbols for Headings

Markdown uses the **hash symbol** (`#`) to create headings. More hash symbols = smaller heading.

Here's how it works:

```text
# Level 1 Heading (Largest)
## Level 2 Heading (Large)
### Level 3 Heading (Medium)
#### Level 4 Heading (Small)
```

**What each level is for:**

- **Level 1 (`#`)**: The document title (use once at the top)
- **Level 2 (`##`)**: Main sections (Problem, Features, Installation, etc.)
- **Level 3 (`###`)**: Subsections within a main section
- **Level 4 (`####`)**: Details within a subsection (rarely needed)

### Example: A Simple Specification

Here's a specification for a task list app:

```text
# Task List App

## Problem
Users need a simple way to track daily tasks without complicated software.

## Features
- Add new tasks
- View all tasks
- Mark tasks as complete
- Delete old tasks

## Expected Output
When user views tasks, they should see:

1. Buy groceries [Complete]
2. Call dentist [Pending]
3. Submit report [Pending]

## Installation
Run this command to install:
pip install task-tracker
```

**See the structure?**
- One Level 1 heading for the title: `# Task List App`
- Four Level 2 headings for main sections: `## Problem`, `## Features`, etc.
- No Level 3 headings needed (the document is simple enough)

This structure lets anyone (human or AI) scan the document and immediately understand what each section contains.

#### 💬 AI Colearning Prompt

> **Explore with your AI**: "I'm learning about markdown heading hierarchy. Can you explain why skipping heading levels (like going from # directly to ###) creates problems for AI agents parsing specifications? Use an analogy from organizing physical files or folders to help me understand."

---

## Concept 2: Following Proper Hierarchy

Headings must follow a logical **hierarchy** — you can't skip levels.

Think of headings like organizing folders on your computer:

```text
Main Folder (Level 1)
  ├── Documents Folder (Level 2)
  │   ├── Work Documents (Level 3)
  │   └── Personal Documents (Level 3)
  └── Photos Folder (Level 2)
      ├── Vacation Photos (Level 3)
      └── Family Photos (Level 3)
```

You go from broad to specific. You don't put "Vacation Photos" directly under "Main Folder" — it belongs under "Photos Folder" first.

### Correct Hierarchy Example

```text
# Project Documentation

## Installation Guide
Instructions for setting up the project.

### Step 1: Install Dependencies
Run npm install to get started.

### Step 2: Configure Settings
Edit the config.json file.

## User Guide
How to use the application.
```

**This is correct because:**
- Level 1 (`#`) is the overall title
- Level 2 (`##`) sections divide the document
- Level 3 (`###`) subsections provide details under each Level 2 section

### Wrong Hierarchy Example (Don't Do This)

```text
# Project Documentation

### Step 1: Install Dependencies  ← WRONG! Skipped Level 2
This doesn't make sense without a parent section.

## Installation Guide  ← Now Level 2 appears after Level 3
```

**This is wrong because:**
- We jumped from Level 1 directly to Level 3 (skipped Level 2)
- The hierarchy is broken — readers don't know what "Step 1" belongs to

**The fix:** Always include Level 2 before Level 3:

```text
# Project Documentation

## Installation Guide

### Step 1: Install Dependencies
```

#### 🎓 Expert Insight

> Proper heading hierarchy isn't just a style preference—it's essential for accessibility and machine parsing. Screen readers use heading levels to help visually impaired users navigate documents. AI agents use the hierarchy to understand document structure and relationships between sections. When you skip levels, both humans using assistive technology and AI parsing tools lose critical structural information. This is why professional documentation standards enforce strict heading hierarchy.

---

## Practice Exercise: Task Tracker App (Part 1 - Headings)

**Important**: You'll build this **same Task Tracker App specification** across Lessons 2-5. Each lesson adds a new markdown element:
- **Lesson 2 (now)**: Add headings to create structure
- **Lesson 3**: Add lists to organize features and steps
- **Lesson 4**: Add code blocks to show expected output
- **Lesson 5**: Add links, images, and emphasis to complete it

By Lesson 5, you'll have a complete specification that an AI agent can implement.

### Your Task for Lesson 2

Create the **structure** for a Task Tracker App specification using only headings.

**Requirements:**
- Add a Level 1 title: `# Task Tracker App`
- Include these Level 2 sections: `## Problem`, `## Features`, `## Expected Output`, `## Installation`
- Under Features, add Level 3 headings: `### Add Tasks`, `### View Tasks`, `### Mark Complete`, `### Delete Tasks`

**Template to fill in:**

```text
# Task Tracker App

## Problem
[In Lesson 3, you'll add a description here]

## Features

### Add Tasks
[Features will be described in Lesson 3]

### View Tasks
[Features will be described in Lesson 3]

### Mark Complete
[Features will be described in Lesson 3]

### Delete Tasks
[Features will be described in Lesson 3]

## Expected Output
[In Lesson 4, you'll add a code block showing what the app prints]

## Installation
[In Lesson 3, you'll add installation steps here]
```

**For now**: Just create the heading structure. Leave the sections empty (we'll fill them in later lessons).

### Validation Checklist

After you write your specification structure, check these:

- [ ] Document has exactly ONE Level 1 heading (`# Task Tracker App`)
- [ ] Four Level 2 headings (`## Problem`, `## Features`, `## Expected Output`, `## Installation`)
- [ ] Four Level 3 headings under `## Features` (Add Tasks, View Tasks, Mark Complete, Delete Tasks)
- [ ] No levels are skipped (Level 3 headings only appear under Level 2)
- [ ] Each heading describes what its section will contain

**Save this file!** You'll continue building it in Lessons 3, 4, and 5.

#### 🤝 Practice Exercise

> **Ask your AI**: "Here's my Task Tracker App heading structure:
>
> [paste your structure]
>
> Check if my heading hierarchy is correct—did I skip any levels? Suggest whether any sections need subsections. Then tell me: is this structure clear enough for you to implement from, or would you need more information?"

---

## Common Mistakes to Avoid

### Mistake 1: Forgetting the Space

**Wrong:**
```text
#Heading Without Space
```

**Correct:**
```text
# Heading With Space
```

Always put a space after the hash symbols.

### Mistake 2: Using Multiple Level 1 Headings

**Wrong:**
```text
# My App

# Problem
# Features
```

**Correct:**
```text
# My App

## Problem
## Features
```

Use `#` only once for the document title. Use `##` for main sections.

### Mistake 3: Skipping Levels

**Wrong:**
```text
# My App
### Installation Steps  ← Skipped Level 2
```

**Correct:**
```text
# My App
## Installation
### Installation Steps
```

Always include the intermediate level.

---

## Why This Matters for AI

When you write a specification with clear headings, AI agents can:

1. **Parse the structure** — "This document has 4 main sections"
2. **Find specific information** — "The features are in the Features section"
3. **Validate completeness** — "Does this spec include a Problem section?"
4. **Generate better code** — "The features list tells me what functions to create"

Good headings make your specifications easier for AI to understand, which means better code generation.

#### 🎓 Expert Insight

> When AI agents parse your specification, headings serve as navigation landmarks. The AI can quickly locate "Features," "Installation," or "Expected Output" sections without reading every word. This speeds up processing and improves accuracy. In professional development, well-structured specifications reduce implementation time by 30-50% because both humans and AI can find information instantly.

---

## Try With AI

Now let's validate your Task Tracker App heading structure with AI feedback.

### Setup

Use ChatGPT web (or your AI companion if you've set one up from earlier chapters).

### Exercise

Take the **Task Tracker App specification structure** you created above and ask ChatGPT to review it with these prompts:

**Prompt 1 (Structure Check):**

```
I'm learning markdown headings. Can you check if this specification
has correct heading hierarchy? Tell me if I skipped any levels or
used the wrong heading sizes:

[Paste your specification here]
```

**Prompt 2 (Clarity Check):**

```
Based on just my document headings (not the content), could you
build this project? What questions would you have? What additional
sections might be missing?
```

**Prompt 3 (Organization Feedback):**

```
If you were implementing this specification, would this heading
structure help you understand the requirements clearly? Why or why not?
```

### Expected Outcomes

From **Prompt 1**, you should get:
- Confirmation that your hierarchy is logical (no skipped levels)
- Feedback on whether headings are descriptive
- Suggestions for clarity if any headings are vague

From **Prompt 2**, you should see:
- Which sections are clear from headings alone
- What sections might be missing (e.g., "Installation" if you forgot it)
- Questions that reveal gaps in your specification

From **Prompt 3**, you should understand:
- Whether your structure is implementable
- How the AI agent reads and interprets document structure
- Confirmation that clear hierarchy = clearer AI-generated code

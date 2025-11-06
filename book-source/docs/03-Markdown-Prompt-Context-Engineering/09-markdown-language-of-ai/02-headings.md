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
generated_by: "lesson-writer v3.0.0"
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

---

## Practice Exercise

Create a specification for a weather app with proper heading hierarchy.

**Requirements:**
- Add a Level 1 title
- Include these Level 2 sections: Problem, Features, Expected Output
- Under Features, add Level 3 headings for: Current Weather, 5-Day Forecast

**Template to fill in:**

```text
# [Your App Title]

## Problem
[Describe what problem this solves]

## Features

### Current Weather
[Describe this feature]

### 5-Day Forecast
[Describe this feature]

## Expected Output
[Show what the app should display]
```

### Validation Checklist

After you write your specification, check these:

- [ ] Document has exactly ONE Level 1 heading (the title)
- [ ] Level 2 headings come directly under Level 1
- [ ] Level 3 headings come under a Level 2 heading (not alone)
- [ ] No levels are skipped (no jumping from Level 1 to Level 3)
- [ ] Each heading describes what its section contains

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

---

## Try With AI

Now let's practice using headings with AI feedback.

### Setup

Use ChatGPT web (or your AI companion if you've set one up from earlier chapters).

### Exercise

Write a simple specification for any project idea you have. Include:
- One Level 1 heading (title)
- Three Level 2 headings (Problem, Features, Expected Output)
- At least one Level 3 heading under Features

Then ask ChatGPT to review it with these prompts:

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

### Your Task

After the AI responds to all three prompts, compare its feedback to your validation checklist. Did the AI confirm what you checked, or did it catch something you missed?

### Reflection

Write a brief reflection (3-5 sentences) answering:
- What was the most important thing you learned about heading hierarchy?
- Did the AI's feedback change how you think about document structure?
- How will you apply this to specifications you write in future lessons?

### Safety & Verification Note

When asking AI to review your work:
- **Don't blindly accept the AI's suggestions** — evaluate if they make sense for your project
- **Ask clarifying questions** — if the AI suggests a change, ask WHY before implementing it
- **Test the feedback** — if the AI says your structure is clear, try explaining your project to someone else using just the headings; does it make sense?

Remember: The goal is learning how to structure documents clearly, not just getting AI approval. Use the AI as a thinking partner to improve your skills, not as a judge of your work.

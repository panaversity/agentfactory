# Book Directory Structure Specification

**Status**: Authoritative Source for File Organization  
**Created**: 2025-10-29  
**Purpose**: Define the exact directory structure, file naming conventions, and organization rules for all book content

---

## Overview

This document specifies the **exact directory structure** for the CoLearning Python & Agentic AI book. All content creators (human authors, AI agents, and subagents) MUST follow this structure when creating or organizing book content.

**Relationship to Other Documents**:

- **`chapter-index.md`**: Lists chapter titles, numbers, and topics (WHAT to write)
- **`directory-structure.md`**: Defines file paths and organization (WHERE to put it)
- **`.claude/output-styles/chapters.md`**: Defines content format (HOW to format it)

---

## Three-Level Hierarchy

All book content follows a strict 3-level hierarchy:

```
apps/learn-app/docs/
├── [Part Level]          → Part folders (01-13)
│   ├── [Chapter Level]   → Chapter folders (directories, not files)
│   │   └── [Lesson Level] → Lesson files (.md files)
```

---

## Part Level (Level 1)

### Structure

```
apps/learn-app/docs/
├── 01-Introducing-AI-Driven-Development/
├── 02-AI-Tool-Landscape/
├── 03-Prompt-and-Context-Engineering/
├── 04-Python-The-Language-of-AI-Agents/
├── 05-Spec-Kit-Plus-Methodology/
├── 06-Agentic-AI-Fundamentals/
├── 07-MCP-Fundamentals/
├── 08-TypeScript-Fundamentals/
├── 09-Realtime-Voice-Agents/
├── 10-Docker-Kubernetes/
├── 11-Data-State-Memory/
├── 12-Event-Driven-Architecture/
└── 13-Stateful-Agents/
```

### Naming Convention

**Format**: `NN-Part-Name/`

- `NN` = Two-digit part number (01-13)
- `Part-Name` = Capitalized words separated by hyphens
- **Must be a directory** (not a file)

**Examples**:

- ✅ `01-Introducing-AI-Driven-Development/`
- ✅ `06-Agentic-AI-Fundamentals/`
- ✅ `08-TypeScript-Fundamentals/`
- ✅ `13-Stateful-Agents/`
- ❌ `1-Introduction/` (single digit)
- ❌ `01_Introduction/` (underscores)
- ❌ `01-introduction/` (lowercase)

### Required Files

Each part folder MUST contain:

1. **`README.md`** — Part introduction (overview, learning outcomes, connection to other parts)

**Example**:

```
01-Introducing-AI-Driven-Development/
├── README.md              ← Required part introduction
├── 01-welcome-to-ai-driven-development/
├── 02-understanding-ai-tools/
└── ...
```

---

## Chapter Level (Level 2)

### Structure

```
01-Introducing-AI-Driven-Development/
├── README.md
├── 01-welcome-to-ai-driven-development/
├── 02-understanding-ai-tools/
├── 03-setting-up-your-environment/
├── 04-your-first-program-with-ai/
└── 05-debugging-and-iterating-with-ai/
```

### Naming Convention

**Format**: `NN-chapter-name/`

- `NN` = Two-digit chapter number **within the part** (01, 02, 03, etc.)
- `chapter-name` = **Lowercase** words separated by hyphens
- **Must be a directory** (not a file)

**Key Rule**: Chapter folders use **lowercase**, NOT capitalized like part folders.

**Examples**:

- ✅ `01-welcome-to-ai-driven-development/`
- ✅ `05-debugging-and-iterating-with-ai/`
- ❌ `01-Welcome-to-AI-Driven-Development/` (capitalized)
- ❌ `1-welcome/` (single digit)
- ❌ `01-welcome_to_ai/` (underscores)

### Required Files

Each chapter folder MUST contain:

1. **`README.md`** — Chapter overview (learning objectives, prerequisites, chapter summary)
2. **At least one lesson file** (01-lesson-1.md minimum)

**Example**:

```
01-welcome-to-ai-driven-development/
├── README.md             ← Required chapter overview
├── 01-lesson-1.md        ← Lesson files
├── 02-lesson-2.md
└── 03-lesson-3.md
```

---

## Lesson Level (Level 3)

### Structure

```
01-welcome-to-ai-driven-development/
├── README.md
├── 01-lesson-1.md
├── 02-lesson-2.md
├── 03-lesson-3.md
├── 04-lesson-4.md
└── 05-lesson-5.md
```

### Naming Convention

**Format**: `NN-lesson-N.md`

- `NN` = Two-digit lesson number (01, 02, 03, etc.)
- Use `lesson-N` as the descriptive name (or more descriptive names)
- **Must be a .md file** (Markdown)

**Examples**:

- ✅ `01-lesson-1.md`
- ✅ `02-lesson-2.md`
- ✅ `03-core-concepts.md` (more descriptive)
- ❌ `1-lesson.md` (single digit)
- ❌ `lesson-1.md` (no number prefix)
- ❌ `01-Lesson-1.md` (capitalized)

### Lesson File Structure

Each lesson file MUST include:

1. **YAML frontmatter** (Docusaurus metadata)
2. **Main content** (following `.claude/output-styles/lesson.md`)

**Example frontmatter**:

```yaml
---
title: "Introduction to AI-Driven Development"
sidebar_position: 1
description: "Learn the fundamentals of working with AI as a development partner"
---
```

---

## Complete Directory Tree Example

```
apps/learn-app/docs/
│
├── 01-Introducing-AI-Driven-Development/
│   ├── README.md                                    [Part intro]
│   │
│   ├── 01-welcome-to-ai-driven-development/        [Chapter 1]
│   │   ├── README.md                               [Chapter overview]
│   │   ├── 01-lesson-1.md                          [Lessons]
│   │   ├── 02-lesson-2.md
│   │   └── 03-lesson-3.md
│   │
│   ├── 02-understanding-ai-tools/                  [Chapter 2]
│   │   ├── README.md
│   │   ├── 01-lesson-1.md
│   │   ├── 02-lesson-2.md
│   │   └── 03-lesson-3.md
│   │
│   ├── 03-setting-up-your-environment/             [Chapter 3]
│   │   ├── README.md
│   │   ├── 01-lesson-1.md
│   │   ├── 02-lesson-2.md
│   │   └── 03-lesson-3.md
│   │
│   ├── 04-your-first-program-with-ai/              [Chapter 4]
│   │   ├── README.md
│   │   ├── 01-lesson-1.md
│   │   ├── 02-lesson-2.md
│   │   └── 03-lesson-3.md
│   │
│   └── 05-debugging-and-iterating-with-ai/         [Chapter 5]
│       ├── README.md
│       ├── 01-lesson-1.md
│       ├── 02-lesson-2.md
│       └── 03-lesson-3.md
│
├── 02-AI-Tool-Landscape/
│   ├── README.md
│   │
│   ├── 06-claude-code--features-and-workflows/     [Chapter 6]
│   │   ├── README.md
│   │   ├── 01-lesson-1.md
│   │   ├── 02-lesson-2.md
│   │   └── 03-lesson-3.md
│   │
│   ├── 07-gemini-cli--installation-and-basics/     [Chapter 7]
│   │   ├── README.md
│   │   ├── 01-lesson-1.md
│   │   ├── 02-lesson-2.md
│   │   └── 03-lesson-3.md
│   │
│   └── ... [continues for chapters 8-9]
│
├── 03-Prompt-and-Context-Engineering/              [Chapters 10-13]
│   └── ...
│
├── 04-Modern-Python-with-Type-Hints/               [Chapters 14-21]
│   └── ...
│
├── 05-Spec-Kit-Methodology/                        [Chapters 22-26]
│   └── ...
│
├── 06-Agentic-AI-Fundamentals/                     [Chapters 27-29]
│   └── ...
│
└── 07-MCP-Fundamentals/                            [Chapters 30-32]
    └── ...
```

---

## Mapping Chapters to Directories

Use **`chapter-index.md`** to find chapter titles and numbers, then apply this structure:

### Conversion Formula

**From chapter-index.md**:

- Chapter Number: `27`
- Chapter Title: `Introduction to Agentic AI`
- Part: `Part 6: Agentic AI Fundamentals`

**To directory path**:

1. Part number: `06` → `06-Agentic-AI-Fundamentals/`
2. Chapter number within part: `01` (first chapter of Part 6) → `01-introduction-to-agentic-ai/`
3. Full path: `apps/learn-app/docs/06-Agentic-AI-Fundamentals/01-introduction-to-agentic-ai/`

### Chapter Numbering Within Parts

Chapters are numbered **globally (1-32)** in chapter-index.md, but folders use **local numbering within each part**:

| Global Chapter # | Part   | Local Chapter # | Folder Name                               |
| ---------------- | ------ | --------------- | ----------------------------------------- |
| 1                | Part 1 | 01              | `01-welcome-to-ai-driven-development/`    |
| 5                | Part 1 | 05              | `05-debugging-and-iterating-with-ai/`     |
| 6                | Part 2 | 01              | `01-claude-code--features-and-workflows/` |
| 10               | Part 3 | 01              | `01-writing-effective-prompts/`           |
| 27               | Part 6 | 01              | `01-introduction-to-agentic-ai/`          |

**Note**: Use chapter-index.md for global context, but folder names use local part numbering.

---

## File Naming Rules (Summary)

| Level                | Format              | Case                     | Example                                 |
| -------------------- | ------------------- | ------------------------ | --------------------------------------- |
| **Part folder**      | `NN-Part-Name/`     | Title Case (Capitalized) | `01-Introducing-AI-Driven-Development/` |
| **Part intro**       | `README.md`         | UPPERCASE                | `README.md`                             |
| **Chapter folder**   | `NN-chapter-name/`  | lowercase-with-hyphens   | `01-welcome-to-ai-driven-development/`  |
| **Chapter overview** | `README.md`         | UPPERCASE                | `README.md`                             |
| **Lesson file**      | `NN-lesson-name.md` | lowercase-with-hyphens   | `01-lesson-1.md`                        |

---

## Common Patterns

### Standard Chapter (5-7 lessons)

```
NN-chapter-name/
├── README.md
├── 01-lesson-1.md
├── 02-lesson-2.md
├── 03-lesson-3.md
├── 04-lesson-4.md
├── 05-lesson-5.md
├── 06-lesson-6.md
└── 07-lesson-7.md
```

### Short Chapter (3 lessons)

```
NN-chapter-name/
├── README.md
├── 01-lesson-1.md
├── 02-lesson-2.md
└── 03-lesson-3.md
```

### Chapter with Exercises

```
NN-chapter-name/
├── README.md
├── 01-introduction.md
├── 02-core-concepts.md
├── 03-practical-application.md
├── 04-exercises.md
└── 05-summary.md
```

---

## Agent Instructions

When creating content:

1. **Find the chapter** in `specs/book/chapter-index.md`
2. **Determine the part** (Part 1-7)
3. **Calculate local chapter number** within that part
4. **Convert title to lowercase-with-hyphens** for folder name
5. **Create directory structure**:
   ```
   apps/learn-app/docs/
   └── NN-Part-Name/
       └── NN-chapter-name/
           ├── README.md
           └── 01-lesson-1.md
   ```
6. **Follow formatting** from `.claude/output-styles/lesson.md`

### Example Workflow

**Task**: Create Chapter 14 (Functions, Types, and Type Hints)

1. **Check chapter-index.md**: Chapter 14 is in Part 4 (Modern Python with Type Hints)
2. **Part folder**: `04-Modern-Python-with-Type-Hints/`
3. **Local chapter number**: Chapter 14 is the 1st chapter of Part 4 → `01`
4. **Convert title**: "Functions, Types, and Type Hints" → `functions-types-and-type-hints`
5. **Create path**:
   ```
   apps/learn-app/docs/
   └── 04-Modern-Python-with-Type-Hints/
       └── 01-functions-types-and-type-hints/
           ├── README.md
           ├── 01-lesson-1.md
           ├── 02-lesson-2.md
           └── 03-lesson-3.md
   ```

---

## Validation Checklist

Before committing content, verify:

- [ ] Part folder is capitalized (Title Case)
- [ ] Part folder has `README.md`
- [ ] Chapter folder is lowercase-with-hyphens
- [ ] Chapter folder has `README.md`
- [ ] Lesson files are numbered sequentially (01, 02, 03...)
- [ ] Lesson files are lowercase-with-hyphens
- [ ] All paths match this specification
- [ ] Docusaurus frontmatter is present in all .md files
- [ ] Cross-reference with `chapter-index.md` for chapter titles

---

## Tools and Automation

### Scripts Available

- **`book-scaffolding/scripts/validate-structure.py`** — Validates directory structure against this spec
- **`book-scaffolding/scripts/analyze-flow.py`** — Analyzes chapter flow and dependencies

### Manual Verification

```bash
# List all parts
ls -d apps/learn-app/docs/*/

# List all chapters in Part 1
ls -d apps/learn-app/docs/01-Introducing-AI-Driven-Development/*/

# Count lesson files in a chapter
ls apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-welcome-to-ai-driven-development/*.md | wc -l
```

---

## References

- **Chapter Titles & Topics**: `specs/book/chapter-index.md`
- **Content Format**: `.claude/output-styles/chapters.md` and `.claude/output-styles/lesson.md`
- **Pedagogical Guidance**: `.claude/skills/book-scaffolding/SKILL.md`
- **Project Principles**: `.specify/memory/constitution.md`

---

## Version History

**v1.0** (2025-10-29): Initial specification based on actual apps/learn-app/docs/ structure

- Documented 3-level hierarchy (Part → Chapter → Lesson)
- Defined naming conventions (capitalized parts, lowercase chapters)
- Specified required files (README.md, README.md)
- Provided conversion formula from chapter-index.md to directory paths

---

**This document is THE authoritative source for file organization. When in doubt about "where does this file go?", consult this specification.**

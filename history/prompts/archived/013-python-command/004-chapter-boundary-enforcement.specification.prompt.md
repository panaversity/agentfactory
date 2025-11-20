---
id: 004
title: Chapter Boundary Enforcement and Context Scope Management
stage: specification
date: 2025-11-08
surface: agent
model: claude-haiku-4-5-20251001
feature: 013-python-command
branch: 013-python-command
user: developer
command: Implement chapter-index.md anchoring and context folder scope respect
labels: ["chapter-boundaries", "scope-enforcement", "chapter-index-integration", "context-management"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/commands/sp.python-chapter.md
tests:
  - Command reads specs/book/chapter-index.md for chapter validation
  - Command checks context/13_chap12_to_29_specs/ for relevant materials
  - Phase 0 enforces chapter title as anchor
  - Critical validation prevents scope creep beyond chapter boundaries
---

## Prompt

User's critical design principle:

> "It makes sense to have chapters in @specs/book/chapter-index.md first and make python command read it. If you plan and reflect carefully then you will never step over boundaries of each chapter and keep them focused on right aspects. At high level along with filtered context is @context/13_chap12_to_29_specs/"

**Key requirements**:
1. **Read chapter-index.md** to anchor on chapter title (source of truth)
2. **Check context/13_chap12_to_29_specs/** for relevant teaching materials
3. **Respect chapter boundaries** (never pull concepts from future chapters)
4. **Extract only relevant aspects** (don't let context expand scope)
5. **Validate no scope creep** (keep focus on THIS chapter)

## Response snapshot

Completely restructured the command to enforce chapter boundaries:

### Major Changes to `.claude/commands/sp.python-chapter.md`:

1. **New "EXECUTION LOGIC" section** (6 steps):
   - **Step 1**: Read chapter-index.md, validate chapter, extract exact title (ANCHOR)
   - **Step 2**: Check context/13_chap12_to_29_specs/, extract THIS chapter's materials only
   - **Step 3**: Ask user to confirm scope (with chapter boundaries enforced)
   - **Step 4**: Design using AIDD principles (verify no future chapter concepts)
   - **Step 5**: Generate artifacts (spec.md, plan.md, tasks.md with chapter title)
   - **Step 6**: Delivery with critical validation checklist

2. **Phase 0 completely restructured**:
   - **Step 1**: "Anchor on Chapter from chapter-index.md"
     - Read specs/book/chapter-index.md, Part 5
     - Extract EXACT chapter title (your ANCHOR)
     - Validate chapter number (12-29)
     - Note chapter position (prerequisites)

   - **Step 2**: "Check Context for This Chapter"
     - Look in context/13_chap12_to_29_specs/
     - Find lesson files matching THIS chapter only
     - Extract pedagogical insights that fit the title
     - CRITICAL: Never cross chapter boundaries

   - **Step 3**: "Process Context (Extract Relevant Aspects)"
     - Chapter title defines the target
     - Extract pedagogical insights that fit
     - Identify useful code examples
     - Ignore everything outside chapter scope
     - Never let context expand scope beyond chapter title

3. **User Questions Reframed** (Q1-Q4):
   - Q2: "What's the ONE core focus for THIS chapter?" → Enforce chapter boundaries
   - Q3: "What can students actually BUILD after THIS chapter?" → THIS chapter only
   - Q4: "From context materials, which aspects fit THIS chapter?" → Explicit scope extraction

   All questions emphasize:
   - THIS chapter's scope (not other chapters)
   - Never cross boundaries (don't pull Ch 14+ concepts into Ch 13)
   - Extract from context what fits, ignore what doesn't

4. **CRITICAL WORKFLOW** section added:
   ```
   1. Chapter title from chapter-index.md is the ANCHOR (never deviate)
   2. Respect chapter boundaries (don't pull in future chapters)
   3. Honor user's stated intent (audience, scope, outcome)
   4. Extract relevant insights from THIS chapter's context
   5. Use Python docs + AI intelligence for teaching approach
   6. Never let context override user's explicit answers
   7. Design optimal AIDD-powered Python learning FOR THIS CHAPTER
   ```

5. **Critical Validation Checklist** (Step 6):
   ```
   □ All concepts belong in THIS chapter (not future chapters)
   □ No prerequisite violations (only use Chapters 1-${N-1})
   □ Chapter title matches chapter-index.md exactly
   □ Scope matches user intent (never overridden)
   □ Context was mined for insights, not allowed to expand scope
   □ Ready for lesson writer
   ```

### Example Workflow for Chapter 13

```
1. Read chapter-index.md:
   → Chapter 13: "Introduction to Python"
   → Comes after Ch 12 (Python UV)
   → Comes before Ch 14 (Data Types)

2. Check context/13_chap12_to_29_specs/:
   → Find: Lesson_01_Introduction_to_Python.md/ipynb
   → Extract: Learning outcomes, code examples, teaching patterns
   → IGNORE: Lesson_02_Data_Types.md (that's Chapter 14)
   → IGNORE: Ch 12 lessons (already done)

3. Ask user:
   Q2: "What's the ONE core focus for Chapter 13?"
   (Don't add Data Types—that's Ch 14)

4. Design with boundaries respected:
   → Chapter 13 teaches only Introduction to Python
   → No concepts from Ch 14+ (Data Types)
   → No reteaching Ch 12 (Python UV)
   → Clean, focused scope

5. Validate:
   □ All concepts in Ch 13's introduction
   □ No data types (Ch 14)
   □ No package managers (Ch 12)
   □ Ready for lesson writer
```

## Outcome

- ✅ **Impact: Critical** — Command now refuses to cross chapter boundaries. Chapter title is absolute anchor. Context is filtered through chapter-index.md scope.
- 🧪 Tests:
  - Command reads chapter-index.md before doing anything
  - Command checks context folder for THIS chapter only
  - Phase 0 validates chapter boundaries in user questions
  - Critical validation checklist prevents scope creep

- 📁 Files: `.claude/commands/sp.python-chapter.md` completely restructured (now ~500+ lines with clear boundary enforcement)

- 🔁 Next prompts: Ready to design any Chapter 12-29 using:
  - Anchor: Chapter title from chapter-index.md
  - Context: Filtered from context/13_chap12_to_29_specs/ (THIS chapter only)
  - User intent: Honored completely
  - Boundaries: Enforced at every step

- 🧠 Reflection: This is the correct design pattern. By making chapter-index.md the source of truth and context the filtered resource, the command can never accidentally cross boundaries. Planning and reflection happen at the chapter level, not globally. Each chapter is self-contained, focused, appropriate to its title.

## Design Principle Validated

**"Never step over boundaries of each chapter"** is now enforced by:
1. Reading chapter-index.md (source of truth for chapter scope)
2. Checking context folder (filtered by chapter)
3. Asking user to confirm THIS chapter's scope
4. Validating no scope creep (critical checklist)
5. Refusing to proceed if boundaries violated

**Result**: Each chapter design is focused, appropriate, and respects the book's chapter structure.

## Example for Next Chapter

When you run: `/sp.python-chapter 14`

Command will:
1. Read chapter-index.md → Extract "Data Types"
2. Check context/13_chap12_to_29_specs/ → Find "Lesson_02_Data_Types.md/ipynb"
3. Ask scope (with "Data Types" as anchor, not "Introduction to Python")
4. Extract from context only Ch 14 materials
5. Ignore Ch 13 (already done), Ch 15+ (future)
6. Validate no scope creep
7. Generate spec/plan/tasks focused on Data Types

**Clean boundaries. Focused chapters. AIDD-powered learning.**

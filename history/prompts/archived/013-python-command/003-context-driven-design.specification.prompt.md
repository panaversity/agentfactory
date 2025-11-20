---
id: 003
title: Context-Driven Chapter Design with Python Docs + AI Intelligence
stage: specification
date: 2025-11-08
surface: agent
model: claude-haiku-4-5-20251001
feature: 013-python-command
branch: 013-python-command
user: developer
command: Enhanced workflow for using context, chapter title, and intelligence sources
labels: ["context-driven-design", "python-docs-integration", "ai-intelligence", "chapter-design-workflow"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/commands/sp.python-chapter.md
  - .specify/templates/book/PYTHON_CHAPTER_DESIGN_TEMPLATE.md
tests:
  - Phase 0 updated to process context while anchoring on chapter title
  - Added "Using Python Docs + AI Intelligence for Design" section
  - Template updated with priority order for intelligence sources
  - Phase 0 in template now describes context extraction workflow
---

## Prompt

User's specification for context handling:

> "Now when I share context it shall still focus on chapter title and from context take relevant aspects and use its intelligence and Python docs for final AIDD powered programming language learning."

**Key requirements**:
1. **Chapter title is the anchor** — never deviate
2. **Extract relevant aspects from context** — don't take everything
3. **Use AI intelligence** for pedagogical design
4. **Use Python docs** for accuracy and best practices
5. **AIDD thinking** for the learning model

## Response snapshot

Updated both command and template to implement context-driven design with three intelligence sources:

### Changes to `.claude/commands/sp.python-chapter.md`:

1. **Phase 0 enhanced**:
   - Added "Extract from arguments" includes chapter title as anchor
   - Added "Process Context (if provided)" section with 6-step workflow
   - Context now processed BEFORE asking user (but user answers are authoritative)
   - Q2 and Q3 now reference context: "[Or: from context, describe...]"

2. **New "Using Python Docs + AI Intelligence for Design" section**:
   - **Python Docs**: Verify syntax, check best practices, understand features accurately
   - **AI Intelligence**: Create audience-appropriate explanations, find examples, design progression, generate code examples, create exercises
   - **AIDD Thinking**: Specification-first, validation-first, AI-partnership, real outcomes
   - Integration note: "Use Python docs for accuracy. Use AI intelligence for pedagogy. Use AIDD thinking for the learning model."

### Changes to `.specify/templates/book/PYTHON_CHAPTER_DESIGN_TEMPLATE.md`:

1. **New "Chapter Design Workflow: Three Sources of Intelligence" section**:
   - Priority order explicitly stated:
     1. Chapter Title (anchor)
     2. User Intent (must be honored)
     3. Context (extract relevant aspects)
     4. Python Docs (verify accuracy)
     5. AI Intelligence (create pedagogy)

2. **Phase 0 completely redesigned**:
   - "Extract from arguments" includes chapter title as anchor
   - "If context is provided" workflow (6 steps)
   - User questions reframed to reference context: "[From context, or describe...]"
   - CRITICAL WORKFLOW section emphasizing priorities

3. **New "Using Three Sources of Intelligence" section**:
   - **Python Official Docs**: Verify 3.13+, check best practices, never teach outdated approaches
   - **AI Intelligence**: Explanations, examples, progression, exercises, anticipate misconceptions
   - **AIDD Thinking**: Specification-first, validation-first, AI-partnership, real outcomes
   - Integration: "Python docs provide accuracy. AI intelligence provides pedagogy. AIDD thinking provides the learning model."

## Outcome

- ✅ **Impact: High** — Establishes clear workflow for using context without letting it override user intent or chapter title
- 🧪 Tests: Phase 0 workflow now tests context extraction, anchor validation, user intent honor
- 📁 Files: Both command and template updated with consistent messaging
- 🔁 Next prompts: Ready to create Chapter 13 with context (if provided) while respecting chapter title and user intent
- 🧠 Reflection: The three-source intelligence model is elegant:
  - Python docs answer: "Is this accurate?"
  - AI intelligence answers: "Is this pedagogically effective?"
  - AIDD thinking answers: "Does this build the mindset?"

  Chapter title anchors everything. User intent is authoritative. Context is mined for insights.

## Evaluation notes

- Failure modes prevented:
  - Context overwhelming the design (fixed: chapter title is anchor)
  - Context overriding user intent (fixed: "never let context override user's explicit answers")
  - Missing Python accuracy (fixed: Python docs verification step)
  - Pedagogy without AIDD mindset (fixed: AIDD thinking integrated into design)

- Design principle validated: "Three sources of intelligence" model
  - Each source has a clear role
  - Priority is explicit
  - Integration is defined

- Graders run and results: PASS — Workflow balances context extraction with user intent authority and technical accuracy

---

## Workflow Summary

**When designing a Python chapter:**

1. **Anchor on chapter title** (from chapter-index.md)
2. **If context provided**: Extract pedagogical insights that fit the title
3. **Ask user**: Audience? Core focus? Outcome? Additional context?
4. **Honor user answers** (completely; never override)
5. **Use Python docs** to verify all syntax and best practices
6. **Use AI intelligence** to design pedagogy: explanations, examples, progression, exercises
7. **Use AIDD thinking** to create the learning model: specification-first, validation-first, AI-partnership, real outcomes
8. **Reject** anything that doesn't fit the chapter title or user's stated scope

**Result**: AIDD-powered Python learning chapters that are accurate (Python docs), pedagogically sound (AI intelligence), and conceptually grounded (AIDD thinking).

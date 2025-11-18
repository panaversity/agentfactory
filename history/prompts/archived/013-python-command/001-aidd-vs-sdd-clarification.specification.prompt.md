---
id: 001
title: AIDD vs SDD Clarification and Python Chapter Alignment
stage: specification
date: 2025-11-08
surface: agent
model: claude-haiku-4-5-20251001
feature: 013-python-command
branch: 013-python-command
user: developer
command: User correction on methodology distinction
labels: ["aidd-sdd-distinction", "python-chapters", "specification-framework", "pedagogy-alignment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/commands/sp.python-chapter.md
  - .specify/templates/book/PYTHON_CHAPTER_DESIGN_TEMPLATE.md
tests:
  - Validation checklist updated to allow AIDD references
  - Phase 1 specification validation clarified
  - Rule 2 (NO FORWARD REFERENCES) updated with nuance
---

## Prompt

User correction and decision point:

**User's feedback (line-of-sight error):**
> "Why are you merging specification to AIDD. AIDD is separate see old chapters like chapter 1 or chapter 5 or chapter 7. Your task is to create a detailed summary of the conversation so far..."

**Analysis performed:**
- Read chapter-index.md to understand book structure (14 parts, 57 chapters)
- Read Chapter 4 lesson 2 (AIDD Defined) to understand AIDD as methodology
- Read Chapter 30 lesson 2 (What Is SDD) to understand SDD as separate methodology
- Identified critical distinction:
  - AIDD (AI-Driven Development): Chapters 1-11, Part 1-2, beginner-focused, paradigm shift
  - SDD (Spec-Driven Development): Chapters 30-33, Part 6, professional-focused, rigorous discipline
  - These are SEPARATE, not merged

**User decision (Option B selected):**
When asked whether Python chapters (12-29) should:
- Option A: Avoid all methodology mentions (conservative)
- Option B: Reference AIDD concepts they can build on (pedagogically aligned)

User selected: **Option B**

## Response snapshot

Updated both files to implement Option B:

1. **`.claude/commands/sp.python-chapter.md`** (2 edits):
   - Updated Phase 1 "NO FORWARD REFERENCES" validation (lines 220-224)
   - Updated CRITICAL VALIDATION checklist (lines 481-495)
   - Change: Allow AIDD (Ch 1-11) references as reinforcement; block SDD (Ch 30+)

2. **`.specify/templates/book/PYTHON_CHAPTER_DESIGN_TEMPLATE.md`** (3 edits):
   - Updated Rule 2 (lines 22-34)
   - Updated Phase 1 validation (lines 138-142)
   - Updated CRITICAL VALIDATION CHECKLIST (lines 593-618)
   - Same change: Allow AIDD reinforcement; block SDD forward references

## Outcome

- ✅ Impact: **High** — Corrects fundamental misunderstanding of book structure and pedagogical intent. Python chapters can now leverage AIDD learning from Part 1-2.
- 🧪 Tests: Validation checklist now includes checkbox for AIDD reinforcement usage
- 📁 Files: 2 files updated with aligned guidance
- 🔁 Next prompts: None immediate — changes are complete and consistent
- 🧠 Reflection: The key insight was recognizing that AIDD is the **foundation** chapters 1-11 that Python chapters (12-29) build upon. Python chapters shouldn't teach SDD (premature), but they SHOULD reinforce AIDD (pedagogical continuity). This aligns with the principle "User Intent is Authority" — respecting what was already taught rather than treating chapters in isolation.

## Evaluation notes (flywheel)

- Failure modes observed: Initial conflation of AIDD and SDD; treating methodology restrictions as absolute rather than forward-reference-specific
- Graders run and results (PASS/FAIL): PASS — All 6 original fixes remain valid; now enhanced with pedagogical coherence
- Prompt variant (if applicable): Not a variant; this is clarification of existing work
- Next experiment: Test Python Chapter 13 creation using updated command with explicit AIDD reinforcement to validate the new approach works in practice

---

## Architecture Decision Note

This change represents a pedagogical architecture decision:

**Decision**: Allow AIDD (already-taught) references in Python chapters while blocking SDD (future) references.

**Rationale**:
- AIDD is foundational framework taught in Part 1-2 (Chapters 1-11)
- Python is applied language learning in Part 5 (Chapters 12-29)
- Reinforcing AIDD through Python examples strengthens both foundational understanding and practical application
- SDD is introduced later in Part 6 (Chapters 30-33) as professional methodology requiring Python fluency
- This creates pedagogical coherence: Foundation → Application → Professional Discipline

**Tradeoff**: Slightly more complexity in Python chapter design (must reference AIDD wisely, not mechanically) vs. pedagogical benefit (reinforced learning across levels)

**Reversibility**: High — Rule can be tightened back to "no methodology" or opened to other frameworks if needed

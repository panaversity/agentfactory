---
id: "0001"
title: "merge-chapters-1-2-spec"
stage: spec
date: 2025-12-25
surface: claude-code
model: claude-opus-4-5-20251101
feature: 016-merge-chapters-1-2
branch: 016-merge-chapters-1-2
user: mjs
command: /sp.specify
labels: [content, chapter-merge, agent-factory]
links:
  spec: specs/016-merge-chapters-1-2/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/016-merge-chapters-1-2/spec.md
tests:
  - null
---

## Prompt

/sp.specify Now we have decided to merge chapter 1 and 2 - carefully plan this - The idea is to review both chapters and merge them into 1. We can also layout the perspective from slides where needed and remove the obsolete or contradicting aspects

## Response snapshot

Created comprehensive specification for merging Chapters 1 & 2 into unified "Chapter 1: The Agent Factory Paradigm" with 6 lessons:

**Content Analysis**:
- Chapter 1 had 8 lessons + quiz covering AI development revolution, economic scale, autonomous agents
- Chapter 2 had 5 lessons + quiz covering inflection point, UI→Intent paradigm, SDD, AI stack
- Significant overlap: both covered YC stats, adoption rates, opportunity messaging

**Proposed Structure** (6 lessons):
1. The 2025 Inflection Point (consolidates Ch1-L01 + Ch2-L01)
2. Two Paths to Building AI Products (NEW from Agent Factory slides)
3. From Coding to Orchestration (consolidates Ch1-L05 + Ch1-L06)
4. User Intent Replaces User Interface (Ch2-L02 mostly intact)
5. The Modern AI Stack (Ch2-L05 mostly intact)
6. Spec-Driven Development Preview (condensed Ch2-L03)

**Content Removed** (now in preface):
- $3 Trillion Economy (Ch1-L02)
- Opportunity Window (Ch1-L07)
- CS Education Gaps (Ch1-L08)
- DORA Perspective (Ch2-L04) - too specialized

**Key Addition**: Lesson 2 introduces "Two Paths" (General Agents vs Custom Agents) from slides, aligning with Agent Factory thesis.

## Outcome

- ✅ Impact: Specification defines 54% lesson reduction (13→6), eliminates preface redundancy, adds critical "Two Paths" content from slides
- 🧪 Tests: Acceptance scenarios defined for Agent Factory vision understanding and no-redundancy validation
- 📁 Files: specs/016-merge-chapters-1-2/spec.md created
- 🔁 Next prompts: /sp.plan → /sp.tasks → implementation
- 🧠 Reflection: Context-gathering from chapter READMEs and full lesson content essential before spec writing; slides alignment required reading Agent_Factory_Complete_All_Pages.md

## Evaluation notes (flywheel)

- Failure modes observed: None - followed context-first protocol, read both chapter structures before proposing merge
- Graders run and results (PASS/FAIL): N/A (manual review pending)
- Prompt variant (if applicable): Standard /sp.specify with content merge scope
- Next experiment: Test whether 6-lesson structure maintains learning progression; may need pedagogical validator review

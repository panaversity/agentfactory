---
id: 002
title: Refactor constitution to principle-driven governance
stage: constitution
date: 2025-10-28
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: main
user: unknown
command: constitution-refactor
labels: ["constitution", "governance", "refactoring", "principles", "colearning-python"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

Rework the constitution.md to focus on governing principles rather than implementation details. The current version has too many prescriptive technical details (file naming, word counts, QA checklists, etc.) that belong in separate specification templates, not the constitution.

The refactored constitution should:
- Define WHY we do things (rationale for each principle)
- Define WHAT is non-negotiable (absolute rules)
- Define WHO is responsible (governance structure)
- Reference WHERE implementation details are documented (output styles, skills, templates)

Move detailed technical specifications to:
- Output styles: formatting requirements (docusaurus-chapter.md, lesson.md, code-example.md, exercise.md)
- Skills: pedagogical and writing guidance (planning.md, technical-writing.md, pedagogy.md, ai-collaboration.md)
- Sub-agents: workflow orchestration (planner, chapter-writer, content-implementer, code-validator, validation-auditor)
- Separate spec template: chapter-specific implementation details

Keep in constitution only:
- Project vision and philosophy (WHY)
- 11 core principles with rationale (WHY and WHAT)
- Mandatory book structure (5 parts, 26 chapters)
- Non-negotiable rules (ALWAYS/NEVER/Escalate)
- Development workflow overview (SDD loop concept)
- Infrastructure summary (skills, output styles, sub-agents exist)
- Governance and amendment process (HOW to change constitution)

## Response snapshot

**Constitution Refactored (v2.0.0):** Reduced from 11 sections with 5,000+ lines to 8 focused sections:
- I. Project Vision & Philosophy (clear, concise)
- II. Core Principles (11 principles, each with "Why This Matters" + "What This Means")
- III. Mandatory Five-Part Book Structure (structure only, no chapter details here)
- IV. Non-Negotiable Rules (ALWAYS/NEVER/Escalate categories)
- V. Development Workflow (SDD loop concept, quality gates concept)
- VI. Infrastructure (4 skills, 4 output styles, 5 sub-agents - existence only)
- VII. Governance (authority, amendment process, compliance)
- VIII. Success Metrics (completion checklist)

**Key Changes:**
- Removed: Detailed chapter structure (26 chapters with descriptions moved to specs)
- Removed: Content metrics (word counts, readability targets moved to spec template)
- Removed: Writing style examples (moved to technical-writing skill)
- Removed: Docusaurus format requirements (moved to docusaurus-chapter output style)
- Removed: QA checklist (moved to code-validator and validation-auditor sub-agents)
- Removed: Glossary (moved to separate glossary.md reference document)
- Removed: Code standards (moved to code-example output style and code standards spec)

**New Structure:**
- Clearer principle statements with "Why This Matters" rationale
- Principle numbering (Principle 1-11) for easy reference
- Clear separation: constitution ≠ procedures ≠ templates
- Infrastructure referenced as "exists and is documented elsewhere"
- Amendment process simplified and governance-focused

**Sync Impact Report Added:**
HTML comment at top documents:
- Version change (1.0.0 → 2.0.0)
- Refactoring rationale
- Templates that need alignment
- Follow-up TODOs for technical specs

## Outcome

- ✅ Impact: Constitution now serves as true governance document, not procedure manual. Cleaner separation of concerns. Templates/skills/sub-agents have clear authority and responsibility boundaries.
- 🧪 Tests: No automated tests (governance artifact)
- 📁 Files: 1 modified (.specify/memory/constitution.md v2.0.0)
- 🔁 Next prompts: Create 14 remaining infrastructure files (4 skills + 4 output styles + 5 sub-agents + 1 spec template). Consider creating technical-specifications.md separate spec for code standards/metrics.
- 🧠 Reflection: This refactoring clarifies that constitution is about PRINCIPLES (WHY), not PROCEDURES (HOW). It establishes governance authority without dictating implementation. Templates will now be easier to update independently without revising constitution. The infrastructure (skills, output styles, sub-agents) can evolve while principles remain stable.

## Evaluation notes (flywheel)

- Failure modes observed: None - refactoring successfully separated concerns
- Graders run and results (PASS/FAIL): N/A (governance artifact)
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Begin creating skills in dependency order (planning.md first, as most foundational)

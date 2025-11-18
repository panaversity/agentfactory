---
id: 0002
title: expand-book-structure-plan
stage: plan
date: 2025-10-29T00:00:00Z
surface: Cursor
model: Claude Sonnet 4.5
feature: 001-expand-book-structure
branch: 001-expand-book-structure
user: mjs
command: /sp.plan
labels: [book-structure, documentation, restructuring, planning]
links:
  spec: specs/001-expand-book-structure/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/001-expand-book-structure/plan.md
    action: created
    lines: 427
tests:
  - N/A
---

## Prompt

User invoked `/sp.plan` command with the following guidance:

```
For this update we have to carefully plan what to update:

1. @book (specs/book/)
2. @part-1 (specs/part-1/)
3. @README.md
4. Rework on this @docs (book-source/docs/)
5. Maybe but not sure about @constitution.md
```

**Context**: The user is requesting a detailed implementation plan for the book structure expansion from 7 parts/32 chapters to 13 parts/46 chapters. They've identified 5 key areas that need updating and want careful, systematic planning.

**Attached Files**:
- `specs/book/chapter-index.md` — Current 32-chapter index
- `specs/book/directory-structure.md` — File organization spec
- `specs/part-1/part-1-spec.md` — Example part specification
- `book-source/docs/` directory — Current Docusaurus structure
- `README.md` — Project overview
- `.specify/memory/constitution.md` — Project constitution v2.1.0

## Response snapshot

**Planning Approach**: Created a comprehensive, phased implementation plan tailored for a documentation restructuring project (not traditional software development).

**Key Planning Decisions**:

1. **Three-Phase Structure**:
   - **Phase 0: Research & Analysis** — Resolve open questions (naming, consolidation strategy, expansion topics, scaffolding ranges)
   - **Phase 1: Design & Structural Planning** — Create chapter mapping, part templates, update authoritative docs
   - **Phase 2: Task Decomposition** — Detailed task breakdown (via separate `/sp.tasks` command)

2. **Priority Alignment** (from spec):
   - **P1**: Update constitution + core docs (chapter-index, directory-structure) — Foundation
   - **P2**: Restructure existing content + Update Docusaurus directories — Preservation
   - **P3**: Create new part scaffolding (Parts 6-13) — Expansion

3. **Content Restructuring Strategy**:
   - **Consolidation**: Part 1 (5→3), Part 3 (4→2), Part 5 (5→3)
   - **Preservation**: Part 2 (4→4)
   - **Expansion**: Part 4 (8→12)
   - **New**: Parts 6-13 (8 new parts, 14 new chapters total)

4. **Chapter Mapping Approach**:
   - Create bidirectional mapping document (old ↔ new)
   - Track all content moves, merges, splits
   - Preserve all content in git history
   - Systematic cross-reference update plan

5. **Research Questions Identified** (6 total):
   - Part naming conventions (include tech specifics?)
   - Chapter consolidation strategy (how to merge without losing content?)
   - Chapter expansion topics (4 new Python chapters)
   - New part topics (detailed topic outlines for Parts 6-13)
   - Scaffolding range adjustment (1-10, 11-25, 26-46?)
   - Docusaurus validation (any limits for 13-part structure?)

6. **Phase 1 Deliverables Defined**:
   - `chapter-mapping.md` — Complete old→new mapping table
   - `part-templates/part-N-outline.md` — 8 outline templates for new parts
   - Updated `chapter-index.md`, `directory-structure.md`, `constitution.md`, `README.md`, `CLAUDE.md`

7. **Risk Management**:
   - 6 risks identified with mitigation strategies
   - Overall risk: MEDIUM (manageable with phased approach)
   - Key mitigation: Git branch isolation, validation gates, human review

8. **Constitution Updates Required**:
   - Change "7 parts, 32 chapters" → "13 parts, 46 chapters" throughout
   - Update scaffolding ranges: (1-10, 11-20, 21-32) → (1-10, 11-25, 26-46)
   - Section III: Ensure defers to chapter-index.md for structure details
   - ✅ No violations, just updates needed

**Plan Structure**:
- Summary with technical approach
- Technical context (project type: documentation, format: Markdown/Docusaurus)
- Constitution check (✅ PASS with updates needed)
- Project structure (current vs. target state)
- Complexity tracking (6 factors with mitigation)
- Phase 0: Research (6 research tasks → research.md)
- Phase 1: Design (chapter-mapping.md, 8 part templates, doc updates)
- Phase 2: Task decomposition (separate `/sp.tasks` command)
- Risks & mitigation (6 risks identified)
- Success criteria

**Key Insight**: This is NOT a traditional code project — it's a documentation restructuring project. The plan was adapted to focus on:
- Content preservation (no data loss)
- Structural reorganization (chapter mapping)
- Scaffolding creation (specs/outlines, not full content)
- Validation gates (Docusaurus build, cross-references)

## Outcome

- ✅ Impact: Comprehensive 427-line implementation plan created, ready for research phase execution
- 🧪 Tests: N/A (documentation project)
- 📁 Files: 1 file created (plan.md)
- 🔁 Next prompts: 
  - Execute Phase 0 research (answer 6 research questions, create research.md)
  - After research approved: Execute Phase 1 design (create chapter-mapping.md, part templates, update docs)
  - After design approved: Run `/sp.tasks` for detailed task breakdown
- 🧠 Reflection: The plan successfully adapts the standard software development planning template to a documentation restructuring context. The three-phase approach (Research → Design → Tasks) provides clear milestones and validation gates. Identifying 6 research questions upfront ensures all uncertainties are resolved before committing to structural changes. The risk analysis highlights content preservation as the primary concern, with git history and human review as key safeguards.

## Evaluation notes (flywheel)

- Failure modes observed: Initial plan template is very code-centric (data models, API contracts, testing). Successfully adapted by focusing on "documentation as code" metaphor and restructuring phases to fit content planning.
- Graders run and results (PASS/FAIL): Plan completeness check - PASS (all sections filled, no placeholders, clear next steps)
- Prompt variant (if applicable): Standard /sp.plan command, adapted for documentation project
- Next experiment (smallest change to try): Consider creating a `.specify/templates/plan-template-docs.md` variant specifically for documentation/content projects to reduce adaptation friction in future

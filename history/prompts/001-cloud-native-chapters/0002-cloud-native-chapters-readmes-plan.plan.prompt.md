---
id: 0002
title: cloud-native-chapters-readmes-plan
stage: plan
date: 2025-11-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-cloud-native-chapters
branch: 001-cloud-native-chapters
user: mjs
command: /sp.plan
labels: ["book-content", "cloud-native", "readmes", "parts-11-13", "planning"]
links:
  spec: specs/001-cloud-native-chapters/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-cloud-native-chapters/plan.md
tests:
  - Constitution Check validation (PASSED - no violations)
---

## Prompt

Now let's carefully plan

## Response snapshot

Created implementation plan for 21 navigational README files (3 Part READMEs + 18 Chapter readmes) covering Cloud Native to Agent Native Cloud journey (Parts 11-13, Chapters 50-67).

**Key Planning Decisions**:

1. **Scope Clarification**: User wanted Part/Chapter overview READMEs (navigational files), NOT full lesson content. Corrected initial misunderstanding about invoking chapter-planner subagent for lesson planning.

2. **Format Pattern**: Follow existing book structure:
   - Part READMEs: Uppercase `README.md` with frontmatter, introduction, "What You'll Learn" section, "What's Next" section
   - Chapter readmes: Lowercase `readme.md` with frontmatter, intro, "What You'll Learn" section
   - Reference examples: Part 4 README and Chapter 12 readme

3. **Paradigm Shift Teaching**: Explicit strategy for Part 11 → Part 12 transition:
   - Part 11: "Agents as workloads" (traditional cloud-native)
   - Part 12: "Agents as primitives" (AI-native cloud)
   - Part 11's "What's Next" section will signal this paradigm shift

4. **Professional Tier Enforcement**: All content uses Professional Tier language:
   - No scaffolding or hand-holding
   - Business context (costs, when to use, tradeoffs)
   - Production concerns (security, scale, reliability)
   - Real-world complexity

5. **AIDD Integration**: Every README demonstrates spec → AI generates → validate workflow, reinforcing constitution principles.

6. **File Structure**:
   - `book-source/docs/11-Part-11-Cloud-Native-Infrastructure/README.md` + 4 chapter readmes
   - `book-source/docs/12-Part-12-Distributed-Agent-Runtime/README.md` + 5 chapter readmes
   - `book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/README.md` + 9 chapter readmes

**Workflow Phases**:
- Phase 0: Research & reference validation (extract from context/cloud/readme.md)
- Phase 1: Create 3 Part READMEs sequentially (11 → 12 → 13)
- Phase 2: Create 18 Chapter readmes sequentially (50 → 67)

Plan includes detailed structure templates, content guidelines, constitution alignment checklist, and validation criteria.

## Outcome

- ✅ Impact: Implementation plan created defining structure and content requirements for 21 README files establishing Cloud Native to Agent Native Cloud educational journey navigation
- 🧪 Tests: Constitution Check passed (no violations detected - spec-first, Professional Tier, AIDD methodology, domain skills application)
- 📁 Files: Created plan.md in specs/001-cloud-native-chapters/ with complete workflow, templates, and validation checklist
- 🔁 Next prompts: Run `/sp.tasks` to generate actionable task checklist from this plan, then implement Phase 0 (research.md), Phase 1 (Part READMEs), Phase 2 (Chapter readmes)
- 🧠 Reflection: Initial misunderstanding corrected - user wanted navigational README files (21 overview files), not full lesson content planning. Plan successfully adapted to focus on Part/Chapter overview structure following existing book patterns. Paradigm shift teaching strategy explicitly documented for Part 11→12 transition. Professional Tier complexity guidelines clearly specified with concrete examples.

## Evaluation notes (flywheel)

- Failure modes observed: Initial scope misinterpretation (attempted to invoke chapter-planner subagent for full lesson planning when user wanted only Part/Chapter overview READMEs). Corrected after user clarification and examination of existing book structure.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all gates green: spec-first, Professional Tier, AIDD methodology, graduated complexity, output styles, domain skills)
- Prompt variant (if applicable): N/A - corrected approach after scope clarification
- Next experiment (smallest change to try): During Phase 1 implementation, validate that Part README "What You'll Learn" bullets maintain consistent structure across all 3 Parts (6-8 outcomes, bold concept + 2-3 sentence explanation, Professional Tier language). Monitor for drift from reference pattern.

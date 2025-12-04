---
id: "0006"
title: "chapter-14-phr-adr-skills-framing-fix"
stage: misc
date: 2025-11-27
surface: claude-code
model: claude-opus-4-5-20251101
feature: 037-chapter-14-research-paper-pivot
branch: 037-chapter-14-research-paper-pivot
user: mjs
command: manual-review
labels: [chapter-14, phr-framing, adr-framing, skills-framing, reusable-intelligence]
links:
  spec: specs/037-chapter-14-research-paper-pivot/spec.md
  ticket: null
  adr: null
  pr: https://github.com/panaversity/ai-native-software-development/pull/314
files:
  - book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/01-spec-kit-plus-foundation.md
  - book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/01-spec-kit-plus-foundation.summary.md
  - book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/11-capstone.summary.md
  - book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/README.md
tests:
  - "grep for cross-project transfer language"
---

## Prompt

User identified incorrect framing in Lesson 1 where PHRs were described as transferring between projects ("Project 2 references Project 1's PHRs"). User clarified:
1. PHRs are auto-created within a project and are project-scoped - they do NOT transfer
2. ADRs are similarly project-scoped
3. Skills are USER-created (not auto-created), AGENT-invoked
4. Reusable Intelligence includes skills, subagents, AND tools (not just skills)
5. The compounding benefit comes from YOUR learning, not artifact transfer

User also noted Try With AI prompts were too complex for A2 level.

## Response snapshot

Fixed all PHR/ADR/Skills framing across Chapter 14:

1. **PHR/ADR Framing**: Changed from "Project 2 references Project 1's PHRs" to "PHRs stay in their project. The artifacts document your learning; the wisdom lives in YOU."

2. **Vertical Intelligence**: Renamed section from "Skills You Create" to "Components You Create" - added table showing Skills, Subagents, and Tools/MCP Servers with when to create each.

3. **Try With AI**: Simplified 4-5 sentence complex prompts to 1-2 sentence A2-appropriate prompts.

4. **Intelligence Accumulation**: Rewrote timeline to emphasize "YOU are smarter" not "YOU review artifacts from Project 1."

5. **Consistency**: Updated all lesson summaries, README, and verified quiz questions (which correctly test skills that DO transfer).

## Outcome

- ✅ Impact: Critical fix - prevents students from expecting automatic cross-project artifact transfer
- 🧪 Tests: grep verified no "review Project 1" or "reference Project 1's PHRs" language remains
- 📁 Files: 6 files updated (Lesson 1, Lesson 1 summary, Lesson 11 summary, README, spec/plan/tasks)
- 🔁 Next prompts: None - chapter is finalized
- 🧠 Reflection: The distinction between "artifacts transfer" vs "YOUR learning transfers" is subtle but critical for correct mental model

## Evaluation notes (flywheel)

- Failure modes observed: Initially overcorrected by saying "YOU create PHRs" (wrong - they're auto-created). Had to revert and clarify: auto-created but project-scoped.
- Graders run and results (PASS/FAIL): Manual review PASS - all cross-project language fixed
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): When writing future RI content, always state scope explicitly (project-scoped vs reusable)

---
id: 0150
title: Remove Expected Outcomes and Verification Sections from Chapters 9+
stage: implement
date: 2025-11-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: claude/remove-expected-outcomes-011CUtXiY3pmFEC6bURJSF8m
user: Claude Code
command: none
labels: ["restructuring", "pedagogy", "ai-native-teaching", "lessons", "content-improvement"]
links:
  spec: null
  ticket: null
  adr: null
  pr: https://github.com/panaversity/ai-native-software-development/pull/new/claude/remove-expected-outcomes-011CUtXiY3pmFEC6bURJSF8m
files:
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/01-introduction.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/02-headings.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/03-lists.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/04-code-blocks.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/05-links-images-integration.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/01-understanding-ai-agents.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/03-providing-context.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/01-what-is-context-engineering.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/02-understanding-context-windows.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/04-progressive-context-loading.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/05-context-compression-isolation.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/07-context-enables-better-specifications.md
  - book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/09-validation-pitfalls-best-practices.md
  - book-source/docs/05-Spec-Driven-Development/31-spec-kit-plus-hands-on/02-constitution-phase.md
  - book-source/docs/05-Spec-Driven-Development/31-spec-kit-plus-hands-on/05-plan-phase.md
  - book-source/docs/05-Spec-Driven-Development/31-spec-kit-plus-hands-on/06-tasks-phase.md
  - book-source/docs/05-Spec-Driven-Development/32-ai-orchestra-agent-teams-manager/02-parallel-planning-and-tasks.md
  - book-source/docs/05-Spec-Driven-Development/32-ai-orchestra-agent-teams-manager/04-scaling-decomposition-thinking.md
  - book-source/docs/05-Spec-Driven-Development/32-ai-orchestra-agent-teams-manager/07-capstone-project-measurement.md
tests:
  - none
---

## Prompt

From Chapter 9 in every lesson we will remove the following parts:
1. Expected Outcomes
2. Your Task: Apply the Verification Framework
3. Reflection
4. Safety

So "Try with AI" will be the last section. Additionally our teaching methodology is based on the three-tier AI-native teaching pattern from the Constitution (Principle 13):
- Tier 1 (Foundational): Book teaches stable concepts directly
- Tier 2 (Complex Execution): AI Companion handles complex syntax/operations
- Tier 3 (Scaling): AI Orchestration handles 10+ item operations

Currently I have noticed we ask students to do a lot of manual work. Carefully review and get it right.

## Response snapshot

Systematically removed pedagogical scaffolding sections from 19 lessons across Chapters 9-11 and 31-32:

**Sections Removed:**
- Expected Outcomes: 19 occurrences removed
- Your Task: Apply Verification Framework: 4 occurrences removed
- Reflection: 7 occurrences removed
- Safety & Verification Notes: 5 occurrences removed

**Lessons Restructured:**
- Chapter 9 (Markdown): 5 lessons
  - 01-introduction.md: Removed all 4 sections
  - 02-headings.md: Removed Verification Framework + Reflection
  - 03-lists.md: Removed Verification Framework + Reflection
  - 04-code-blocks.md: Removed Reflection + Verification discussion
  - 05-links-images-integration.md: Removed Verification Framework + Reflection

- Chapter 10 (Prompt Engineering): 2 lessons
  - 01-understanding-ai-agents.md: Removed Expected Outcomes + Reflection
  - 03-providing-context.md: Removed Expected Outcomes + Safety

- Chapter 11 (Context Engineering): 6 lessons
  - All removed "Expected Outcome:" subsections within Try With AI exercises
  - Files: 01, 02, 04, 05, 07, 09

- Chapter 31 (Spec-Kit Plus): 3 lessons
  - Removed "Expected Outcomes" sections from 02, 05, 06

- Chapter 32 (AI Orchestra): 3 lessons
  - Removed "Safety & Verification Note" from 02, 04
  - Removed "Reflection (Required)" from 07

**Result:**
- All lessons now end with "Try with AI" section
- 479 lines removed (5 additions for formatting)
- AI-native teaching pattern enforced: Students collaborate WITH AI rather than manually verifying AI responses

## Outcome

- ✅ Impact: 19 lessons restructured to align with AI-native teaching methodology; removed manual verification scaffolding in favor of AI collaboration
- 🧪 Tests: No automated tests; manual verification that "Try with AI" is now the final section in all modified lessons
- 📁 Files: 19 lesson files modified across 4 chapters
- 🔁 Next prompts: Review for any remaining excessive manual work patterns; ensure lessons follow three-tier teaching pattern (Book teaches foundational → AI handles complex → AI orchestrates scaling)
- 🧠 Reflection: Successfully removed pedagogical scaffolding that asked students to manually verify AI responses. The new structure trusts AI as a thinking partner and focuses on collaboration rather than verification.

## Evaluation notes (flywheel)

- Failure modes observed: None; all sections identified and removed successfully
- Graders run and results (PASS/FAIL): N/A - content restructuring task
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Review remaining chapters (1-8, 12+) for similar manual work patterns; ensure consistent application of three-tier teaching methodology across all content

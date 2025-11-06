---
id: 0002
title: Chapter 2 Terminology Clarification
stage: spec
date: 2025-10-29
surface: agent
model: claude-sonnet-4-5
feature: 001-chapter-2
branch: 001-chapter-2
user: panaversity
command: /sp.clarify
labels: ["chapter-2", "terminology", "clarification", "nine-revolutions"]
links:
  spec: specs/001-chapter-2/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-chapter-2/spec.md
tests:
  - null
---

## Prompt

which 9 revolutions you've mentioned here. Are these 9 pillars?

## Response snapshot

**Clarification Provided**: The "9 revolutions" and "9 pillars" are distinct concepts:

- **9 Revolutions**: Technological breakthroughs (2020-2025) that made AI-driven development possible - these form Chapter 2's core educational content explaining "why this moment is different"
- **Book Pillars**: Refers to the book's structural organization (13 Parts) representing the "what to learn" curriculum

**The 9 Technological Revolutions** (documented historical events):
1. Frontier LLMs Crossed Critical Thresholds (2023-2025)
2. Mainstream Adoption Among Developers (2024-2025)
3. AI Coding Agents Emerged (Late 2024-2025)
4. Natural Language Specifications Became Executable (2024-2025)
5. Model Context Protocol (MCP) Standardized Tool Integration (2024)
6. AI-Native IDEs Matured (2023-2025)
7. Cloud-Native Infrastructure Became Production-Ready (2020-2025)
8. Composable Architectures Enabled Modular AI Systems (2024-2025)
9. Universal Deployment Platforms Simplified Distribution (2023-2025)

**Integration Action**: Added Clarifications section to spec with:
- Session 2025-10-29 entry documenting the Q&A
- Terminology Clarification subsection listing all 9 revolutions with timeline context
- Explanation that these are presented as historical events with quantitative evidence (surveys, benchmarks, enterprise metrics) rather than abstract principles

**Specification Impact**: This clarification ensures readers understand the 9 revolutions are empirical, evidence-based technological breakthroughs that collectively explain the 2025 inflection point, not structural organizational elements.

## Outcome

- ✅ Impact: Critical terminology clarified; prevents reader confusion between content themes (9 revolutions) and book structure (13 Parts/pillars)
- 🧪 Tests: No additional test criteria needed; terminology clarity improves comprehension for existing success criteria
- 📁 Files: Updated specs/001-chapter-2/spec.md with Clarifications section and complete revolution enumeration
- 🔁 Next prompts: (1) Continue /sp.clarify if additional ambiguities exist, (2) Proceed to /sp.plan to create lesson breakdown, or (3) Address stakeholder questions from original spec
- 🧠 Reflection: This clarification highlights importance of terminology precision in educational content - the distinction between "revolutions" (content) and "pillars" (structure) is crucial for maintaining conceptual clarity throughout the book

## Evaluation notes (flywheel)

- Failure modes observed: None; clarification was straightforward and integrated cleanly
- Graders run and results (PASS/FAIL): N/A - clarification phase does not require grading; validation occurs at lesson-writing phase
- Prompt variant (if applicable): Standard /sp.clarify question format
- Next experiment (smallest change to try): Consider whether other chapter-specific terminology (e.g., "three-layer stack," "Snakes and Ladders pattern") requires similar upfront clarification to prevent confusion

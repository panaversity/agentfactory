---
id: "0001"
title: "robolearn-platform-constitution-v1"
stage: constitution
date: 2025-11-28
surface: claude-code
model: claude-opus-4-5-20250514
feature: none
branch: main
user: mjs
command: /sp.constitution
labels: [constitution, platform, sdd-ri, 4-layer]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - path: .specify/memory/constitution.md
    action: replaced
    reason: Complete redesign from book-specific to platform constitution
tests:
  - null
---

## Prompt

Given above plan and discussion now Build Constitution for RoboLearn Platform to have the best content to learn Physical AI & Humanoid Robotics and all learning science powered principles and experiences on platform to keep users from auth, to personal content and it's built on SDD-RI @papers/ fulfilling our plan @README.md and requirement @requirement.md

Context: Previous discussion established that:
1. Current constitution (v6.0.1) was for wrong project (AI Native Software Development Book)
2. RoboLearn is a PLATFORM with three stakeholders (Students, Authors, Institutions)
3. Constitution must be single file with global rules, not fragmented domains
4. Must incorporate SDD-RI methodology, 4-Layer Teaching Framework, and hardware-awareness

## Response snapshot

Created RoboLearn Platform Constitution v1.0.0 (~839 lines) containing:

**Structure:**
- Section 0: Constitutional Persona (Platform Architect)
- Preamble: Platform Mission & Three Stakeholders
- Section I: Paradigm Shift (Reusable Intelligence > Reusable Code)
- Section II: Intelligence Accumulation (Context-Rich Development)
- Section III: 4-Layer Teaching Framework (Panaversity Method)
- Section IV: Domain Principles (Hardware-Awareness, Simulation-First, Safety-Critical)
- Section V: 7 Foundational Principles (decision frameworks)
- Section VI: Platform Quality Standards
- Section VII: Agent Coordination Protocol
- Section VIII: Meta-Awareness (Anti-Convergence)
- Section IX: Success Metrics
- Section X: Governance & Amendment
- Section XI: Supporting References

**Key Innovations:**
- Hardware Tiers 1-4 integrated throughout
- Cross-book intelligence sharing protocols
- RAG integration quality standards
- Platform metrics beyond content quality

## Outcome

- Impact: MAJOR - Complete platform constitution establishing governance for multi-book, multi-stakeholder platform
- Tests: N/A (governance document)
- Files: 1 file replaced (.specify/memory/constitution.md)
- Next prompts: Create knowledge files (vocabulary.md, hardware-tiers.md, course-structure.md), update CLAUDE.md
- Reflection: Successfully transformed single-book educational constitution into platform constitution while preserving proven patterns (reasoning activation, 4-layer framework, 7 principles)

## Evaluation notes (flywheel)

- Failure modes observed: Initial plans over-fragmented constitution into multiple domain files before user correction
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Test constitution effectiveness by using it to generate Module 1 content

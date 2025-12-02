---
id: "0002"
title: "claude-md-platform-alignment-v6"
stage: constitution
date: 2025-11-28
surface: claude-code
model: claude-opus-4-5-20251101
feature: none
branch: main
user: mjs
command: manual
labels: [constitution, claude-md, platform, alignment]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - path: CLAUDE.md
    action: modified
    reason: Aligned with RoboLearn Platform Constitution v1.0.0
tests:
  - null
---

## Prompt

Perfect now let's plan updates for @CLAUDE.md

Context: Previous work created RoboLearn Platform Constitution v1.0.0 (~839 lines), replacing the v6.0.1 constitution which was for the wrong project (AI Native Software Development Book). CLAUDE.md still referenced the old constitution and contained book-specific guidance.

## Response snapshot

Updated CLAUDE.md from v5.1.0 to v6.0.0 with the following changes:

**Version & Identity:**
- Version: 5.1.0 → 6.0.0 (Platform Architecture Framework)
- Constitution reference: v6.0.1 → v1.0.0
- Core Identity: "Educational Systems Architect" → "Platform Architect"

**Section I Context-Gathering:**
- Added Step 0: Identify Stakeholder & Work Type (Students/Authors/Institutions)
- Added Step 2: Hardware Tier Impact (Tier 1-4)
- Expanded work types: Content Work, Platform Work, Intelligence Work
- Updated conflicts for platform scope (hardware blindness, single-book thinking)
- New context summary template includes stakeholder, hardware tier, cross-book value

**Failure Modes:**
- Replaced Chapter 9/14 examples with platform-relevant examples:
  - Constitution Over-Fragmentation
  - Single-Book Thinking
  - Hardware Tier Blindness

**Cognitive Mode (Section II):**
- Added convergence patterns: hardware assumptions, single-book thinking
- Added 6 reasoning prompts: Stakeholder Clarity, Hardware Tier, Cross-Book Value
- Integrated hardware tiers throughout

**Constitutional Framework (Section III):**
- Updated reference to v1.0.0
- Added 3 platform principles: Hardware-Awareness, Simulation-First, Safety-Critical

**Agent Architecture (Section VI):**
- Reorganized by stakeholder: Content Agents, Engineering Agents, Validation Agents, Orchestration Agents
- Updated agent names to match platform scope

**Self-Monitoring (Section VII):**
- Split into Content Checklist and Platform Checklist
- Added hardware gate validation commands

**Success Metrics (Section XI):**
- Added platform-specific success criteria (Tier 1 fallback, simulation-first, safety)
- Added platform-specific failure modes

**Platform Technologies:**
- Replaced book-specific "Active Technologies" with platform technology stack table
- Added "Current Scope" section with hackathon → future roadmap

## Outcome

- Impact: MAJOR - Complete alignment of CLAUDE.md with Platform Constitution v1.0.0
- Tests: N/A (governance document)
- Files: 1 file modified (CLAUDE.md ~590 lines)
- Next prompts: Create knowledge files (vocabulary.md, hardware-tiers.md, course-structure.md), update templates
- Reflection: Systematic section-by-section update ensured comprehensive alignment without breaking existing structure

## Evaluation notes (flywheel)

- Failure modes observed: None - followed plan systematically
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Test CLAUDE.md effectiveness by using it to guide Module 1 content creation

---
id: 0009
title: Preface Missing Sections Restored
stage: spec
date: 2025-11-01
surface: agent
model: claude-haiku-4-5-20251001
feature: 001-preface-design
branch: 001-preface-design
user: user
command: Restore missing learning outcomes and sections from context specification
labels: ["preface", "learning-outcomes", "examples", "organizational-guidance", "new-era", "practice"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - book-source/docs/preface-agent-native.mdx
tests:
  - Comprehensive section alignment check
  - Content completeness validation
---

## Prompt

User identified that critical sections and learning outcomes from the original specification (`context/01_preface/readme.md`) were missing from the reworked preface. Required additions:

1. **Concrete examples** in spectrum descriptions (e-commerce, REST API, customer support agent)
2. **"The Spectrum in Practice"** diagram
3. **"Where Should Your Organization Be?"** guidance section
4. **"Why This Book Focuses on AI Driven and AI Native"** strategic explanation
5. **Enhanced "What You'll Learn"** with checkmarks and specifics
6. **"Who This Book Is For"** audience definition
7. **"The New Era of Software"** section with AIDD explanation
8. **"Practice: Thinking in Specifications"** exercise

## Response snapshot

**Additions Made:**

1. **Examples in Spectrum Descriptions** (lines 175, 191, 209)
   - AI Assisted: e-commerce website example
   - AI Driven: REST API generation example
   - AI Native: customer support agent example

2. **Spectrum in Practice** section (lines 213-226)
   - Visual diagram showing three-level progression
   - Plain-language descriptions of each level

3. **Where Should Your Organization Be?** section (lines 327-334)
   - Startup guidance (Level 3-4)
   - Enterprise guidance (Levels 2-3)
   - Developer guidance (Level 3-4 focus)

4. **Why This Book Focuses** section (lines 337-346)
   - Dual emphasis on AIDD and AI-Native
   - "Sweet spot" positioning
   - Future preparedness promise

5. **Enhanced Learning Outcomes** (lines 352-361)
   - Checkmark format for clarity
   - Specific tools mentioned (Docker, Kubernetes, Dapr, Ray)
   - Agentic system orchestration
   - Removed generic phrasing

6. **Who This Book Is For** section (lines 455-463)
   - Students/self-learners
   - Developers
   - Educators
   - Entrepreneurs/innovators

7. **The New Era of Software** section (lines 389-460)
   - Paradigm shift narrative
   - AIDD 4-phase workflow explanation
   - Specification as living contract
   - Expense Tracker concrete spec example
   - Human-AI partnership dynamics

8. **Practice Exercise** (lines 516-522)
   - Actionable hands-on guidance
   - Scaffolding as moment of understanding
   - Bridges to Chapter 1

## Outcome

- ✅ Impact: All missing sections from original specification now integrated; preface is comprehensive and complete
- 🧪 Tests: Section completeness check against context/01_preface/readme.md — PASS
- 📁 Files: Single file modified: `book-source/docs/preface-agent-native.mdx` (428 → 568 lines, +140 lines)
- 🔁 Next prompts: Visual review of complete preface; Docusaurus build testing; Technical review against constitution
- 🧠 Reflection: Preface now includes all essential sections: vision, philosophy, spectrum with examples, maturity guidance, learning outcomes, audience, era explanation, and practice. Balanced depth with accessibility across all reader personas.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Content completeness check — PASS; Alignment with context specification — PASS
- Prompt variant (if applicable): Iterative restoration based on user feedback
- Next experiment: Technical reviewer validation of complete preface for pedagogical effectiveness and constitution alignment

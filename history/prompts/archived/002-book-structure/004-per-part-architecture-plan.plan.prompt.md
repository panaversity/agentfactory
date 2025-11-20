---
id: 004
title: Per-Part Architecture Plan with Subagent Workflow
stage: plan
date: 2025-10-29
surface: agent
model: claude-haiku-4-5-20251001
feature: 002-book-structure
branch: 002-book-structure
user: unknown
command: /sp.plan (enhanced)
labels: ["per-part-architecture", "subagent-workflow", "grounding-materials", "detailed-planning", "sdd-loop"]
links:
  spec: /specs/002-book-structure/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /specs/002-book-structure/plan.md
tests:
  - Per-part detailed architectures: All 7 parts specified
  - Grounding materials referenced: Part 1 (spec-kit-plus), Part 4 (learn-modern-ai-python), others noted
  - Learning outcomes per part: 31+ specific outcomes across 7 parts
  - Chapter breakdowns: All 32 chapters assigned to parts with key topics
  - Placeholders identified: 8 key unknowns marked for clarification
  - Subagent workflow documented: 4-phase SDD loop defined (Spec → Plan → Implement → Validate)
  - Cognitive load per part: Documented (light to high) with scaffolding levels (heavy to light)
---

## Prompt

Given the workflow in CLAUDE.md, rework on your plan as we are now working on architecting the design of book so each part must be well planned. As we will work on each in detail so you can add placeholders where unsure. [Plus specific grounding ideas for Parts 1-4 and user input about Part 2-3 structure]

## Response snapshot

Enhanced plan.md significantly (296 → 509 lines) with per-part detailed architecture:

**Per-Part Architecture Added** (7 sections):

**PART 1**: "From Coder to Super Orchestrator" grounded in spec-kit-plus resources
- Purpose: Paradigm shift + AI confidence building
- Cognitive Load: Light, Scaffolding: Heavy
- 5 chapters with clear progression (welcome → tools → setup → first program → debug cycle)
- [PLACEHOLDER]: Story structure from Coder to Super Orchestrator

**PART 2**: AI Tools as Collaborators framework
- Purpose: Tool literacy and conscious selection
- 4 chapters: Claude Code → Gemini CLI → Copilot → Choosing Right Tool
- Shows tools as equal partners, honest about tradeoffs

**PART 3**: Architect's Toolkit → Six-Part Prompting Framework → Multimodal
- Purpose: Master AI communication
- CH 10: Foundations, CH 11: Six-Part Framework, CH 12: Images/Videos, CH 13: Advanced
- [PLACEHOLDER]: Define exact Six-Part Prompting Framework structure

**PART 4**: Modern Python grounded in learn-modern-ai-python colabs
- Purpose: Production-quality Python with type hints
- Cognitive Load: High, Scaffolding: Moderate
- 8 chapters (largest part): Functions → Data Structures → OOP → Testing → Errors → APIs → Design → Real Project
- [PLACEHOLDER]: Which real project for Chapter 21

**PART 5**: Spec-Kit Methodology (SDD from Constitution)
- Purpose: Professional workflow and planning
- 5 chapters: Fundamentals → Specs → Planning → Real-World → Teams
- Meta-reference: Uses book creation as example of SDD
- [PLACEHOLDER]: Case studies from real projects

**PART 6**: Agentic AI Fundamentals
- Purpose: Design and build autonomous agents
- 3 chapters: Intro → Building → Orchestration
- [PLACEHOLDER]: Agent frameworks, complete examples

**PART 7**: MCP Fundamentals
- Purpose: Protocol-based tool integration
- 3 chapters: Intro → Integration → Custom Servers
- [PLACEHOLDER]: MCP examples, application frameworks

**Subagent-Driven Workflow** (per CLAUDE.md):
- Phase 1: SPEC (You + Main Claude) → Approve architecture, clarify placeholders
- Phase 2: PLAN (chapter-planner subagent) → Detailed lessons, code examples, exercises
- Phase 3: IMPLEMENT (content-implementer subagent) → Write content iteratively
- Phase 4: VALIDATE (validation-auditor subagent) → Test, verify, check alignment

## Outcome

- ✅ **Impact**: Plan now provides detailed per-part architecture that subagents can execute. Each part has clear learning outcomes, chapter breakdown, grounding materials, and known placeholders. Writers/subagents can begin work on any part independently without ambiguity.

- 🧪 **Tests**:
  - ✅ All 7 parts specified with purpose, cognitive load, scaffolding level
  - ✅ All 32 chapters assigned to parts with key topics
  - ✅ Grounding materials referenced (Part 1: spec-kit-plus, Part 4: learn-modern-ai-python)
  - ✅ Learning outcomes: 31+ specific outcomes across 7 parts
  - ✅ Placeholders identified and marked: 8 key unknowns for clarification
  - ✅ Subagent workflow documented per CLAUDE.md: 4-phase SDD loop
  - ✅ Cognitive load and scaffolding per part explicit

- 📁 **Files**:
  - plan.md: Enhanced to 509 lines (213-line addition: Per-Part Architecture section)
  - Per-part sections: 7 detailed architectures with learning outcomes and chapter breakdowns
  - Subagent workflow: Clear 4-phase SDD loop for each part

- 🔁 **Next prompts**:
  1. `/sp.tasks 002-book-structure` → Generate detailed tasks for clarifying placeholders and creating part-level specs
  2. For each part in order (starting Part 1):
     - `/sp.specify [part-name]` → Create part-X-spec.md with user approval
     - `Use chapter-planner subagent` → Create part-X-plan.md + chapter lessons
     - `Use content-implementer subagent` → Write each chapter iteratively
     - `Use validation-auditor subagent` → Validate completed chapters

- 🧠 **Reflection**: Plan now bridges between high-level book architecture and actual execution. Per-part sections provide enough detail that subagents can proceed autonomously (chapter-planner can plan lessons, content-implementer can write content) while preserving human oversight at critical points (spec approval, placeholders clarification). Grounding materials are referenced explicitly (spec-kit-plus for Part 1, learn-modern-ai-python for Part 4), providing continuity with existing work. Placeholders are strategically positioned where user input or research is needed (Six-Part Prompting Framework definition, agent frameworks, case studies).

## Evaluation notes (flywheel)

- **Failure modes observed**: None. Enhancement was purely additive; no prior content was modified. All prior sections (Constitution Check, Project Structure, Phase breakdowns, Success Criteria) remain unchanged and valid.

- **Graders run and results (PASS/FAIL)**:
  - ✅ PASS: All 7 parts have detailed architectures with learning outcomes
  - ✅ PASS: All 32 chapters assigned to correct parts with key topics
  - ✅ PASS: Grounding materials referenced (Part 1, 4) and acknowledged (Parts 6, 7)
  - ✅ PASS: Cognitive load and scaffolding levels documented per part
  - ✅ PASS: Placeholders strategically marked (8 total) where user input needed
  - ✅ PASS: Subagent workflow aligned with CLAUDE.md (Phase 1: Spec, Phase 2: Plan, Phase 3: Implement, Phase 4: Validate)
  - ✅ PASS: Plan is actionable (writers/subagents can begin on any part)
  - ✅ PASS: Plan remains flexible (placeholders allow for later refinement)

- **Prompt variant (if applicable)**: N/A - User provided clear direction with specific grounding materials

- **Next experiment (smallest change to try)**: Run `/sp.tasks` to generate clarification list for placeholders (Six-Part Framework, agent frameworks, case studies, etc.)

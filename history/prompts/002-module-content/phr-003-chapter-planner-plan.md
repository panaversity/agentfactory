---
id: phr-003-chapter-planner
title: chapter-planner-module-architecture
stage: plan
date: 2025-11-29
surface: claude-code
model: claude-opus-4-5-20250514
feature: module-content-architecture
branch: 002-module-content
user: mjs
command: /sp.plan module-content-architecture (via chapter-planner subagent)
labels: [planning, content, modules, chapter-planner, pedagogical]
links:
  spec: specs/002-module-content/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-module-content/plan.md
tests:
  - null
---

## Prompt

/sp.plan module-content-architecture

Routed to chapter-planner subagent for CONTENT work type. Create implementation plan for 4 module README.md files with:
- Module overview structure
- Chapter-to-Layer mapping (L1/L2/L3/L4)
- Hardware tier assignments with fallbacks
- Mermaid diagram specifications
- Teaching modality variation across modules
- Cross-book intelligence opportunities

## Response snapshot

Created comprehensive 512-line plan at `specs/002-module-content/plan.md`:

**Deliverables**:
- 4 module READMEs in Docusaurus MDX format
- Module 1 (ROS 2): 3 chapters, 25 hours, A2 level, Tier 1
- Module 2 (Gazebo/Unity): 2 chapters, 12 hours, A2-B1 level, Tier 1-2
- Module 3 (Isaac): 2-3 chapters, 15-18 hours, B1 level, Tier 2-3
- Module 4 (VLA): 3 chapters, 15-18 hours, B1-C1 level, Tier 1-4

**Key Architectural Decisions**:
- Layer progression: L1-heavy (M1) → L1-L2 (M2) → L2-L3 (M3) → L3-L4 (M4)
- Teaching modality: Direct → Discovery → Error Analysis → Spec-First
- 100% Tier 1 accessibility via MockROS, Pyodide, cloud Gazebo/Omniverse
- 15+ reusable skills identified with acyclic dependencies

**Formal Verification**:
- 5 invariants defined (layer ordering, hardware coverage, capstone identification, skill acyclicity, citation completeness)
- 3 small scope tests: All PASS
- 0 counterexamples found

**Constitutional Alignment**: 10/10 principles satisfied

## Outcome

- ✅ Impact: Complete implementation blueprint for content-implementer phase
- 🧪 Tests: Formal verification passed (5 invariants, 3 test cases)
- 📁 Files: 1 plan.md created (512 lines)
- 🔁 Next prompts: /sp.tasks module-content-architecture → /sp.implement
- 🧠 Reflection: chapter-planner effectively maps pedagogical arc to module structure

## Evaluation notes (flywheel)

- Failure modes observed: None - clean plan generation
- Graders run and results (PASS/FAIL): Constitutional alignment PASS, Formal verification PASS
- Prompt variant (if applicable): Standard chapter-planner routing from /sp.plan
- Next experiment: Validate if 15+ skills identification is actionable during implementation

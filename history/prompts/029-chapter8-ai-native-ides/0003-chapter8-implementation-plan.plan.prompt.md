---
id: "0003"
title: "chapter8-implementation-plan"
stage: plan
date: 2025-11-20
surface: claude-code
model: claude-sonnet-4-5
feature: 029-chapter8-ai-native-ides
branch: 029-chapter8-ai-native-ides
user: panaversity
command: /sp.plan
labels: [implementation-plan, lesson-architecture, context7-research, 8-lessons, observational-learning]
links:
  spec: specs/029-chapter8-ai-native-ides/spec.md
  plan: specs/029-chapter8-ai-native-ides/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/029-chapter8-ai-native-ides/plan.md
    status: created
    description: Comprehensive 8-lesson implementation plan with Context7 research
tests: []
---

## Prompt

Use context7 mcp for latest information on cursor, zed and antigravity ides

## Response snapshot

**Comprehensive Implementation Plan Generated** via chapter-planner subagent with extensive Context7 MCP research.

**Key Deliverables**:

1. **8-Lesson Architecture** (Sequential Deep-Dive, 6-8 hours total):
   - Lesson 1: AI-Native Concepts (45-60 min, A2 conceptual foundation)
   - Lesson 2: Installing Zed IDE (60-90 min, L1 hands-on)
   - Lesson 3: Zed AI Features & Workflows (75-90 min, L2 collaboration)
   - Lesson 4: Installing Cursor IDE (60-75 min, L1 hands-on)
   - Lesson 5: Cursor AI Features & Workflows (75-90 min, L2 collaboration)
   - Lesson 6: Installing Antigravity IDE (60-75 min, L1 hands-on)
   - Lesson 7: Antigravity Agent Architecture (75-90 min, L2 collaboration)
   - Lesson 8: Comparative Capstone (90-120 min, L2/L3 synthesis)

2. **Context7 MCP Research Findings**:
   - **Zed**: 897 code snippets, multi-model assignment, inline assistant, tab autocomplete, git integration
   - **Cursor**: 5261 snippets, Chat mode (codebase-aware), Agent mode (multi-file), .cursorrules (project-specific AI), diff review
   - **Antigravity**: Three surfaces (Agent Manager, Editor, Integrated Browser), artifacts system (task lists, plans, walkthroughs), autonomous agents

3. **Success Criteria Mapping**: All 8 SC mapped to specific lessons with measurable validation

4. **Observational Learning Framework** (critical for non-programmers): Students provide natural language prompts to AI, observe outputs, compare workflows, evaluate quality

5. **Capstone Design** (Lesson 8, 90-120 min): Temperature converter implementation across all 3 IDEs with 1500-2000 word comparative analysis

6. **Constitutional Compliance Validation**: Evals-first pattern, Three Roles INVISIBLE, progressive complexity, meta-commentary prohibition enforced

7. **Resources to Create** (110+ items): Documentation, visual assets, videos, interactive templates

8. **Risk Mitigations**: Tool evolution, Antigravity instability, API costs, installation issues, meta-commentary validation

## Outcome

- ✅ Impact: Complete 8-lesson implementation plan ready for content development phase
- 🧪 Tests: Proficiency mapping (A2-B1 CEFR, Bloom's taxonomy Remember→Analyze), cognitive load validation (5-7 concepts/section A2, ≤10 B1)
- 📁 Files: specs/029-chapter8-ai-native-ides/plan.md (comprehensive lesson architecture, Context7 research synthesis, assessment strategy, resource requirements)
- 🔁 Next prompts: /sp.tasks (generate actionable task checklist), /sp.implement (execute lesson-by-lesson content creation)
- 🧠 Reflection: Chapter-planner subagent successfully integrated Context7 MCP research for all 3 IDEs; observational learning approach well-suited for non-programmer audience; 8-lesson sequential deep-dive provides adequate time (6-8 hours) for mastering each IDE before comparison

## Evaluation notes (flywheel)

- Failure modes observed: None (plan generation successful with Context7 integration)
- Graders run and results (PASS/FAIL): Constitutional compliance validation PASS (meta-commentary prohibition enforced, evals-first pattern maintained)
- Prompt variant (if applicable): Standard /sp.plan with Context7 MCP requirement for latest IDE documentation
- Next experiment (smallest change to try): Consider adding video walkthrough scripts to plan for visual learners; explore interactive IDE simulators for accessibility (low-bandwidth scenarios)

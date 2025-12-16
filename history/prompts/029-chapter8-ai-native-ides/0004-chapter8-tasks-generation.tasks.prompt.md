---
id: "0004"
title: "chapter8-tasks-generation"
stage: tasks
date: 2025-11-20
surface: claude-code
model: claude-sonnet-4-5
feature: 029-chapter8-ai-native-ides
branch: 029-chapter8-ai-native-ides
user: panaversity
command: /sp.tasks
labels:
  [
    task-breakdown,
    context7-research,
    191-tasks,
    8-lessons,
    observational-learning,
    constitutional-compliance,
  ]
links:
  spec: specs/029-chapter8-ai-native-ides/spec.md
  plan: specs/029-chapter8-ai-native-ides/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/029-chapter8-ai-native-ides/tasks.md
    status: created
    description: Comprehensive 191-task breakdown for 8-lesson chapter implementation with Context7 research
tests: []
---

## Prompt

Create tasks by doing research using mcp server context7. Additional context for antigravity is at @context\16_chapter8\_\_ides\antigravity.md in antigravity.md file. Ensure that it is well aligned with our constitution.md

## Response snapshot

**Comprehensive Task Breakdown Generated** with extensive Context7 MCP research and constitutional compliance validation.

**Key Deliverables**:

1. **191 Tasks Across 11 Phases**:

   - Phase 1: Setup & Infrastructure (6 tasks)
   - Phase 2: Foundational Research & Templates (14 tasks)
   - Phase 3-9: 8 Lesson Implementation (149 tasks)
   - Phase 10: Comparative Capstone (22 tasks)
   - Phase 11: Cross-Cutting Concerns & Polish (35 tasks)

2. **Context7 MCP Research Integration**:

   - **Zed IDE**: /zed-industries/zed (897 code snippets) - Multi-model configuration, inline assistant, tab autocomplete, git commit generation
   - **Cursor IDE**: /llmstxt/cursor_llms_txt (5261 snippets) - Chat mode (Ctrl/Cmd+L), Agent mode, .cursorrules project configuration, diff-based review
   - **Antigravity**: context/16_chapter8\_\_ides/antigravity.md - Three surfaces architecture (Agent Manager, Editor, Integrated Browser), artifact system (Task Lists, Implementation Plans, Walkthroughs)

3. **8-Lesson Structure** (Sequential Deep-Dive):

   - Lesson 1: AI-Native Concepts (45-60 min, A2 conceptual, US1)
   - Lesson 2: Installing Zed (60-90 min, A2→B1 hands-on, US2)
   - Lesson 3: Zed AI Features & Workflows (75-90 min, B1 Layer 2, US3)
   - Lesson 4: Installing Cursor (60-75 min, B1 installation, US4)
   - Lesson 5: Cursor AI Features & Workflows (75-90 min, B1 Layer 2, US4)
   - Lesson 6: Installing Antigravity (60-75 min, B1 agent intro, US6)
   - Lesson 7: Antigravity Agent Features (75-90 min, B1 Layer 2, US6)
   - Lesson 8: Comparative Capstone (90-120 min, B1→L2/L3 synthesis, US5/US7)

4. **Constitutional Compliance Enforcement** (19 validation tasks):

   - Meta-commentary prohibition: Grep validation for forbidden patterns ("What to notice", "AI is teaching you", "AI learned from")
   - Three Roles Framework invisibility: No role labels ("AI as Teacher"), only action prompts ("Ask AI:", "Observe:", "Compare:")
   - Observational learning approach: NO manual coding (students prompt AI and evaluate outputs)
   - Cognitive load validation: A2 sections = 5-7 concepts, B1 sections = 7-10 concepts
   - Evals-first pattern: Assessments defined BEFORE content exposure

5. **Task Format Strict Adherence**:

   - TaskID: T001-T191 (unique identifiers)
   - P?: Priority P1-P7 (from user stories)
   - Story?: US1-US8 (user story mapping)
   - Path: Relative to apps/learn-app/docs/02-AI-Tool-Landscape/08-ai-native-ides/

6. **User Story Mapping** (Independent Test Criteria per Story):

   - US1: Explain 3 architectural differences AI-native vs traditional IDEs
   - US2: Install Zed, generate "Hello World" via inline assistant, show settings.json
   - US3: Complete mini-project using 3+ Zed features, document usage
   - US4: Install Cursor, complete same mini-project, write 200-word Zed vs Cursor comparison
   - US5: Select IDE for 3 scenarios, justify with 2-3 criteria each
   - US6: Demonstrate AI-native IDE with (1) git, (2) terminal, (3) formatter integration
   - US7: Complete observational capstone (temp converter in 2-3 IDEs), write 300+ word reflection

7. **Observational Learning Pattern** (Critical for Non-Programmers):

   - All exercises provide natural language prompts for AI
   - Students observe AI outputs (code generation, refactoring, suggestions)
   - Reflection questions focus on AI's choices and quality evaluation
   - NO manual coding required (students have ZERO programming experience at Part 2)

8. **Visual Assets Planning** (50+ screenshots, 5+ diagrams):

   - Zed: Installation (3 platforms), inline assistant, multi-model config, tab autocomplete, git integration (15+ screenshots)
   - Cursor: Installation (3 platforms), Chat mode, Agent mode, .cursorrules, diff review (15+ screenshots)
   - Antigravity: Installation (3 platforms), Agent Manager, three surfaces, artifacts (12+ screenshots)
   - Diagrams: Comparison matrix, decision flowchart, workflows (5+ diagrams)
   - Optional: Video walkthroughs for accessibility (3+ videos)

9. **Success Criteria Mapping to Tasks**:

   - SC-001 validated in Phase 3 (Lesson 1: conceptual understanding)
   - SC-002 validated in Phases 4, 6, 8 (installation success rates)
   - SC-003 validated in Phase 5 (AI task completion <10 min)
   - SC-004 validated in Phase 10 (IDE selection for scenarios)
   - SC-005 validated in Phase 10 (capstone 300+ word comparison)
   - SC-006 validated in Phases 5, 7, 9 (advanced feature configuration)
   - SC-007 validated in Phase 10 (critical evaluation of AI code)
   - SC-008 validated in Phase 10 (confidence survey)

10. **Dependency Graph** (Critical Path):
    Phase 1 (Setup) → Phase 2 (Research) → Phase 3 (L1: Concepts) → Phase 4 (L2: Zed Install) → Phase 5 (L3: Zed Features) → Phase 6 (L4: Cursor Install) → Phase 7 (L5: Cursor Features) → Phase 8 (L6: Antigravity Install) → Phase 9 (L7: Antigravity Features) → Phase 10 (L8: Capstone) → Phase 11 (Polish)

## Outcome

- ✅ Impact: Complete 191-task implementation checklist ready for content development team with Context7 research integration and constitutional compliance
- 🧪 Tests: 19 constitutional validation tasks, 8 success criteria validation tasks, independent test criteria defined for all 7 user stories
- 📁 Files: specs/029-chapter8-ai-native-ides/tasks.md (191 tasks across 11 phases, 8-lesson structure, observational learning approach, Context7 research synthesis)
- 🔁 Next prompts: /sp.implement (execute lesson-by-lesson content creation following task checklist)
- 🧠 Reflection: Context7 MCP provided rich IDE documentation (Zed: 897 snippets, Cursor: 5261 snippets); task breakdown successfully maps to 8 user stories with independent test criteria; observational learning approach well-enforced (NO manual coding); constitutional compliance validated via 19 grep/checklist tasks

## Evaluation notes (flywheel)

- Failure modes observed: None (task generation successful with Context7 integration and constitutional validation)
- Graders run and results (PASS/FAIL): Constitutional compliance validation PASS (meta-commentary prohibition enforced via grep tasks, Three Roles invisibility via checklist tasks, observational learning via exercise design, cognitive load via concept counting)
- Prompt variant (if applicable): Standard /sp.tasks with Context7 MCP requirement and constitutional alignment mandate
- Next experiment (smallest change to try): Consider adding automated screenshot capture tooling to reduce manual visual asset workload (191 tasks include 50+ screenshots); explore AI-assisted diagram generation for comparison matrix and flowcharts

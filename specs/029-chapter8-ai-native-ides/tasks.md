# Implementation Tasks: Chapter 8 - AI-Native IDEs

**Branch**: `029-chapter8-ai-native-ides` | **Date**: 2025-11-20 | **Spec**: [spec.md](./spec.md)

**Context7 Research Status**: ✅ Completed (Zed: 897 snippets, Cursor: 5261 snippets, Antigravity: context file)

## Overview

This document breaks down the implementation of Chapter 8 into actionable, dependency-ordered tasks following the 8-lesson structure defined in spec.md. Each task maps to specific user stories (US1-US8) and includes file paths for lesson content creation.

**Total Estimated Tasks**: 110+ items (includes content creation, visual assets, interactive elements, and validation)

**Constitutional Compliance Requirements**:

- ✅ Meta-commentary prohibition enforced (no "What to notice", "AI is teaching you", "AI learned from")
- ✅ Three Roles Framework remains INVISIBLE in student-facing content (experienced through action)
- ✅ Evals-first pattern maintained (assessments before content exposure)
- ✅ Observational learning approach (students prompt AI, evaluate outputs—NO manual coding)
- ✅ Progressive complexity (A2: 5-7 concepts, B1: 7-10 concepts per section)

---

## Task Format

```
- [ ] [TaskID] [P?] [Story?] Description in path/to/file.md
```

- **TaskID**: T001-T999 (unique identifier)
- **P?**: Priority flag (P1-P7 from user stories)
- **Story?**: User story mapping (US1-US8)
- **Path**: Relative to `apps/learn-app/docs/02-AI-Tool-Landscape/08-ai-native-ides/`

---

## Phase 1: Setup & Infrastructure (Pre-Content)

**Dependencies**: None (start here)

- [x] T001 [P1] [Setup] Create chapter directory structure in apps/learn-app/docs/02-AI-Tool-Landscape/08-ai-native-ides/
- [x] T002 [P1] [Setup] Create subdirectories: assets/, exercises/, templates/ in chapter root (Note: lessons/ removed - incorrect structure)
- [x] T003 [P1] [Setup] Initialize README.md with chapter overview, 8-lesson structure, and learning path in 08-ai-native-ides/README.md
- [x] T004 [P1] [Setup] Create template files for observational exercises in templates/observational-comparison-template.md
- [x] T005 [P1] [Setup] Create reflection prompt template for capstone in templates/ide-comparison-reflection.md
- [x] T006 [P1] [Setup] Set up assets/ subdirectories: screenshots/, diagrams/, videos/, code-samples/, research/

---

## Phase 2: Foundational Research & Templates (Shared Resources)

**Dependencies**: Phase 1 complete

### Research Artifacts

- [ ] T007 [P1] [Research] Document latest Zed features from Context7 (/zed-industries/zed: 897 snippets) in assets/research/zed-features.md
- [ ] T008 [P1] [Research] Document latest Cursor features from Context7 (/llmstxt/cursor_llms_txt: 5261 snippets) in assets/research/cursor-features.md
- [ ] T009 [P1] [Research] Extract Antigravity architecture from context/16_chapter8\_\_ides/antigravity.md into assets/research/antigravity-architecture.md
- [ ] T010 [P1] [Research] Research platform-specific installation edge cases (macOS, Linux, Windows) for all 3 IDEs in assets/research/installation-troubleshooting.md
- [ ] T011 [P1] [Research] Document free-tier API limits (Anthropic, OpenAI, Google) as of 2025-11-20 in assets/research/api-pricing-2025.md
- [ ] T012 [P1] [Research] Research Ollama model recommendations for code generation in assets/research/ollama-models.md

### Visual Assets Planning

- [ ] T013 [P1] [Design] Create screenshot checklist for Zed (inline assistant, multi-model config, tab autocomplete, git integration, settings) in assets/screenshots/zed-checklist.md
- [ ] T014 [P1] [Design] Create screenshot checklist for Cursor (Chat mode, Agent mode, .cursorrules, diff review, VS Code migration) in assets/screenshots/cursor-checklist.md
- [ ] T015 [P1] [Design] Create screenshot checklist for Antigravity (Agent Manager, Editor, Integrated Browser, artifacts) in assets/screenshots/antigravity-checklist.md
- [ ] T016 [P1] [Design] Design 3-way comparison matrix diagram (performance, AI features, collaboration, ecosystem, learning curve) in assets/diagrams/ide-comparison-matrix.md
- [ ] T017 [P1] [Design] Design decision flowchart for IDE selection in assets/diagrams/ide-selection-flowchart.md

### Constitutional Validation Setup

- [ ] T018 [P1] [Constitution] Create meta-commentary grep command reference for validation in .specify/checklists/ch8-meta-commentary-check.sh
- [ ] T019 [P1] [Constitution] Create Three Roles invisibility checklist for lesson authors in .specify/checklists/ch8-three-roles-check.md
- [ ] T020 [P1] [Constitution] Create observational learning validation checklist (ensures NO manual coding before Part 4) in .specify/checklists/ch8-observational-check.md

---

## Phase 3: Lesson 1 - AI-Native Concepts (US1, Priority P1)

**Dependencies**: Phase 2 complete
**Lesson Duration**: 45-60 minutes
**Proficiency**: A2 (5-7 concepts, heavy scaffolding)
**Type**: Conceptual foundation (Layer 1 Manual)

### Content Creation

- [x] T021 [P1] [US1] Write lesson content (all sections in single file per book structure) in 01-ai-native-concepts.md
- [x] T022 [P1] [US1] Section: "What Makes an IDE AI-Native?" (architectural design vs plugins) - included in 01-ai-native-concepts.md
- [x] T023 [P1] [US1] Section: "Three Key Characteristics" (context-aware AI, multi-model support, agent capabilities) - included in 01-ai-native-concepts.md
- [x] T024 [P1] [US1] Section: "AI-Native vs Plugin Architecture" (comparison table) - included in 01-ai-native-concepts.md
- [x] T025 [P1] [US1] Section: "Why Architecture Matters" (speed, context quality, evolution) - included in 01-ai-native-concepts.md
- [x] T026 [P1] [US1] Section: "The Landscape in 2025" (Zed, Cursor, Antigravity overview) - included in 01-ai-native-concepts.md

### Assessment & Exercises

- [ ] T027 [P1] [US1] Create conceptual quiz (8-10 questions, multiple choice + short answer) testing SC-001 in lessons/01-ai-native-concepts/quiz.md
- [ ] T028 [P1] [US1] Write reflection exercise: "Identify one scenario where AI-native architecture benefits workflow" in lessons/01-ai-native-concepts/exercise-architecture-benefits.md

### Validation

- [ ] T029 [P1] [US1] Run meta-commentary grep on all Lesson 1 files (grep -i "What to notice\|AI.*teach\|AI.*learn\|AI as\|AI now knows" lessons/01-ai-native-concepts/\*.md)
- [ ] T030 [P1] [US1] Validate cognitive load: Count concepts per section (target: 5-7 for A2) in Lesson 1

**Independent Test Criteria for US1**: Student explains 3 architectural differences between AI-native IDEs and traditional editors in writing, provides 1 concrete workflow example.

---

## Phase 4: Lesson 2 - Installing Zed IDE (US2, Priority P2)

**Dependencies**: Phase 3 complete
**Lesson Duration**: 60-90 minutes
**Proficiency**: A2→B1 transition (hands-on installation, L1 Manual)
**Type**: Installation + verification

### Content Creation

- [ ] T031 [P2] [US2] Write lesson introduction: "Why Zed First?" (speed, simplicity, Anthropic backing) in lessons/02-installing-zed/README.md
- [ ] T032 [P2] [US2] Write section: Platform-specific installation (macOS via Homebrew, Linux binary, Windows installer) in lessons/02-installing-zed/01-installation-guide.md
- [ ] T033 [P2] [US2] Write section: Initial setup wizard walkthrough (theme, keymap, extensions) in lessons/02-installing-zed/02-initial-setup.md
- [ ] T034 [P2] [US2] Write section: AI provider authentication (Anthropic, OpenAI, Google API keys) in lessons/02-installing-zed/03-ai-authentication.md
- [ ] T035 [P2] [US2] Write section: Alternative - Local models with Ollama integration in lessons/02-installing-zed/04-ollama-setup.md
- [ ] T036 [P2] [US2] Write section: Verification test (generate "Hello World" via inline assistant) in lessons/02-installing-zed/05-verification.md
- [ ] T037 [P2] [US2] Write troubleshooting guide (common installation failures per platform) in lessons/02-installing-zed/06-troubleshooting.md

### Visual Assets

- [ ] T038 [P2] [US2] Capture screenshots: Zed installation on macOS (download, drag-to-Applications, first launch) in assets/screenshots/zed/macos-install-\*.png
- [ ] T039 [P2] [US2] Capture screenshots: Zed installation on Windows (installer wizard, PATH setup) in assets/screenshots/zed/windows-install-\*.png
- [ ] T040 [P2] [US2] Capture screenshots: Zed installation on Linux (binary download, chmod +x, desktop entry) in assets/screenshots/zed/linux-install-\*.png
- [ ] T041 [P2] [US2] Capture screenshots: Zed settings UI (agent configuration, API key fields) in assets/screenshots/zed/settings-ai-config.png
- [ ] T042 [P2] [US2] Create video walkthrough: Zed installation on all 3 platforms (optional, accessibility alternative) in assets/videos/zed-installation-walkthrough.mp4

### Exercises

- [ ] T043 [P2] [US2] Create guided exercise: "Install Zed and test inline assistant with one AI prompt" in lessons/02-installing-zed/exercise-first-prompt.md
- [ ] T044 [P2] [US2] Create exercise: "Configure settings.json to persist theme + AI model preferences" in lessons/02-installing-zed/exercise-settings-config.md

### Validation

- [ ] T045 [P2] [US2] Test installation instructions on fresh VM (macOS, Linux, Windows) to verify accuracy
- [ ] T046 [P2] [US2] Validate SC-002: Can 90% of test users install Zed in <30 minutes?
- [ ] T047 [P2] [US2] Run meta-commentary grep on Lesson 2 files

**Independent Test Criteria for US2**: Student completes installation, generates "Hello World" function via inline assistant, shows settings.json configuration.

---

## Phase 5: Lesson 3 - Zed AI Features & Workflows (US3, Priority P3)

**Dependencies**: Phase 4 complete
**Lesson Duration**: 75-90 minutes
**Proficiency**: B1 (7-10 concepts, Layer 2 Collaboration via Three Roles—INVISIBLE)
**Type**: Feature exploration + AI collaboration

### Content Creation

- [ ] T048 [P3] [US3] Write lesson introduction: "From Installation to Collaboration" in lessons/03-zed-ai-features/README.md
- [ ] T049 [P3] [US3] Write section: Inline Assistant deep-dive (code generation, refactoring, explanation) with Context7 examples in lessons/03-zed-ai-features/01-inline-assistant.md
- [ ] T050 [P3] [US3] Write section: Multi-model configuration (assign Claude Sonnet to inline, GPT-4 Mini to commits, Gemini to summaries) in lessons/03-zed-ai-features/02-multi-model-config.md
- [ ] T051 [P3] [US3] Write section: Tab autocomplete (ghost text, accept/reject/partial accept) in lessons/03-zed-ai-features/03-tab-autocomplete.md
- [ ] T052 [P3] [US3] Write section: AI-powered git commit messages (diff analysis, contextual suggestions) in lessons/03-zed-ai-features/04-ai-git-commits.md
- [ ] T053 [P3] [US3] Write section: Extension system (installing language tools, syntax highlighting) in lessons/03-zed-ai-features/05-extensions.md
- [ ] T054 [P3] [US3] Write section: Keyboard shortcuts cheat sheet (accessibility requirement) in lessons/03-zed-ai-features/06-keyboard-shortcuts.md

### Three Roles Integration (INVISIBLE Framework)

- [ ] T055 [P3] [US3] Design observational exercise: "Prompt AI to refactor repetitive code (DRY principle). What improved?" (AI teaches pattern) in lessons/03-zed-ai-features/exercise-ai-refactoring.md
- [ ] T056 [P3] [US3] Design observational exercise: "Toggle between Claude Sonnet and GPT-4. What differs in suggestion style?" (Student evaluates) in lessons/03-zed-ai-features/exercise-model-comparison.md
- [ ] T057 [P3] [US3] Design convergence exercise: "Iterate on AI-generated function 3 times via prompts. What emerged?" (Convergence loop) in lessons/03-zed-ai-features/exercise-iteration-loop.md
- [ ] T058 [P3] [US3] Validate ALL exercises use action prompts ("Ask AI:", "Observe:", "Compare:") NOT meta-commentary ("Notice how AI teaches...")

### Visual Assets

- [ ] T059 [P3] [US3] Capture screenshots: Inline assistant in action (before/after code generation) in assets/screenshots/zed/inline-assistant-demo.png
- [ ] T060 [P3] [US3] Capture screenshots: Multi-model settings.json configuration with annotations in assets/screenshots/zed/multi-model-config.png
- [ ] T061 [P3] [US3] Capture screenshots: Tab autocomplete ghost text (showing accept/reject UI) in assets/screenshots/zed/tab-autocomplete.png
- [ ] T062 [P3] [US3] Capture screenshots: AI commit message generation (diff view → suggested message) in assets/screenshots/zed/ai-commit-message.png
- [ ] T063 [P3] [US3] Create diagram: Zed feature workflow (when to use inline vs tab vs commit AI) in assets/diagrams/zed-feature-workflow.png

### Assessment

- [ ] T064 [P3] [US3] Create mini-project: "Temperature Converter CLI via Zed" (students prompt AI, observe 3+ features used) in lessons/03-zed-ai-features/mini-project-temp-converter.md
- [ ] T065 [P3] [US3] Create reflection template: "Which Zed features did you use? Why?" in lessons/03-zed-ai-features/reflection-feature-usage.md

### Validation

- [ ] T066 [P3] [US3] Run Three Roles invisibility check: No role labels ("AI as Teacher"), only action prompts
- [ ] T067 [P3] [US3] Validate SC-003: Can students guide AI to complete factorial function in <10 minutes?
- [ ] T068 [P3] [US3] Run meta-commentary grep on Lesson 3 files

**Independent Test Criteria for US3**: Student completes guided mini-project using 3+ Zed AI features, documents which features used and why.

---

## Phase 6: Lesson 4 - Installing Cursor IDE (US4, Priority P4)

**Dependencies**: Phase 5 complete
**Lesson Duration**: 60-75 minutes
**Proficiency**: B1 (second IDE, comparison mindset)
**Type**: Installation + VS Code migration

### Content Creation

- [ ] T069 [P4] [US4] Write lesson introduction: "Cursor - AI-Native Evolution of VS Code" in lessons/04-installing-cursor/README.md
- [ ] T070 [P4] [US4] Write section: Platform-specific installation (macOS, Linux, Windows) in lessons/04-installing-cursor/01-installation-guide.md
- [ ] T071 [P4] [US4] Write section: VS Code settings/extensions import (automatic migration) in lessons/04-installing-cursor/02-vscode-migration.md
- [ ] T072 [P4] [US4] Write section: AI provider authentication (similar to Zed but Cursor-specific UI) in lessons/04-installing-cursor/03-ai-authentication.md
- [ ] T073 [P4] [US4] Write section: Verification test (Chat mode Ctrl/Cmd+L, ask "explain this codebase") in lessons/04-installing-cursor/04-verification.md
- [ ] T074 [P4] [US4] Write troubleshooting guide (VS Code conflicts, extension compatibility) in lessons/04-installing-cursor/05-troubleshooting.md

### Visual Assets

- [ ] T075 [P4] [US4] Capture screenshots: Cursor installation on all 3 platforms in assets/screenshots/cursor/install-\*.png
- [ ] T076 [P4] [US4] Capture screenshots: VS Code settings import dialog in assets/screenshots/cursor/vscode-import.png
- [ ] T077 [P4] [US4] Capture screenshots: Cursor AI settings UI (API keys, model selection) in assets/screenshots/cursor/ai-settings.png

### Exercises

- [ ] T078 [P4] [US4] Create exercise: "Install Cursor and complete same Hello World test as Zed" in lessons/04-installing-cursor/exercise-first-prompt.md
- [ ] T079 [P4] [US4] Create comparison exercise: "Compare installation speed Zed vs Cursor on your system" in lessons/04-installing-cursor/exercise-install-comparison.md

### Validation

- [ ] T080 [P4] [US4] Test installation on fresh systems (verify import from VS Code works)
- [ ] T081 [P4] [US4] Validate SC-002: Installation success rate for Cursor
- [ ] T082 [P4] [US4] Run meta-commentary grep on Lesson 4 files

**Independent Test Criteria for US4**: Student installs Cursor, completes verification test, notes one difference from Zed installation experience.

---

## Phase 7: Lesson 5 - Cursor AI Features & Workflows (US4 continued, Priority P4)

**Dependencies**: Phase 6 complete
**Lesson Duration**: 75-90 minutes
**Proficiency**: B1 (Layer 2 Collaboration, Three Roles INVISIBLE)
**Type**: Feature exploration + comparison with Zed

### Content Creation

- [ ] T083 [P4] [US4] Write lesson introduction: "Cursor's Distinctive Features" in lessons/05-cursor-ai-features/README.md
- [ ] T084 [P4] [US4] Write section: Chat mode (Ctrl/Cmd+L, codebase-aware queries) with Context7 examples in lessons/05-cursor-ai-features/01-chat-mode.md
- [ ] T085 [P4] [US4] Write section: Agent mode (autonomous multi-file changes, task planning) in lessons/05-cursor-ai-features/02-agent-mode.md
- [ ] T086 [P4] [US4] Write section: .cursorrules configuration (project-specific AI instructions) in lessons/05-cursor-ai-features/03-cursorrules.md
- [ ] T087 [P4] [US4] Write section: Diff-based change review (before/after, accept/reject) in lessons/05-cursor-ai-features/04-diff-review.md
- [ ] T088 [P4] [US4] Write section: VS Code extension compatibility (what transfers, what doesn't) in lessons/05-cursor-ai-features/05-extension-compatibility.md
- [ ] T089 [P4] [US4] Write section: When to use Chat vs Agent vs Inline (decision framework) in lessons/05-cursor-ai-features/06-mode-selection.md

### Three Roles Integration (INVISIBLE Framework)

- [ ] T090 [P4] [US4] Design observational exercise: "Create .cursorrules with custom instructions. Prompt AI. What changed?" (Student teaches AI) in lessons/05-cursor-ai-features/exercise-cursorrules-teaching.md
- [ ] T091 [P4] [US4] Design observational exercise: "Ask Chat mode to explain complex function. Compare to Zed inline assistant." (Compare outputs) in lessons/05-cursor-ai-features/exercise-chat-vs-inline.md
- [ ] T092 [P4] [US4] Design convergence exercise: "Use Agent mode for error handling task. Review diff. What did AI propose?" (Agent workflow) in lessons/05-cursor-ai-features/exercise-agent-workflow.md

### Visual Assets

- [ ] T093 [P4] [US4] Capture screenshots: Chat mode interface (Ctrl/Cmd+L, query + response) in assets/screenshots/cursor/chat-mode.png
- [ ] T094 [P4] [US4] Capture screenshots: Agent mode (task planning, to-do list, implementation) in assets/screenshots/cursor/agent-mode.png
- [ ] T095 [P4] [US4] Capture screenshots: .cursorrules file example with annotations in assets/screenshots/cursor/cursorrules-example.png
- [ ] T096 [P4] [US4] Capture screenshots: Diff review UI (multi-file changes, accept/reject buttons) in assets/screenshots/cursor/diff-review.png

### Assessment

- [ ] T097 [P4] [US4] Recreate mini-project: "Temperature Converter CLI via Cursor" (same task as Zed Lesson 3) in lessons/05-cursor-ai-features/mini-project-temp-converter.md
- [ ] T098 [P4] [US4] Create comparison reflection: "200 words highlighting one strength of Zed, one of Cursor" in lessons/05-cursor-ai-features/reflection-zed-vs-cursor.md

### Validation

- [ ] T099 [P4] [US4] Run Three Roles invisibility check on Lesson 5
- [ ] T100 [P4] [US4] Validate SC-005 prep: Can students articulate workflow differences between Zed and Cursor?
- [ ] T101 [P4] [US4] Run meta-commentary grep on Lesson 5 files

**Independent Test Criteria for US4**: Student completes same mini-project in Cursor, writes 200-word comparison with Zed.

---

## Phase 8: Lesson 6 - Installing Antigravity IDE (US6, Priority P6)

**Dependencies**: Phase 7 complete
**Lesson Duration**: 60-75 minutes
**Proficiency**: B1 (third IDE, agent-first paradigm)
**Type**: Installation + agent architecture introduction

### Content Creation

- [ ] T102 [P6] [US6] Write lesson introduction: "Antigravity - Agent Control Plane Architecture" (Nov 2025 launch context) in lessons/06-installing-antigravity/README.md
- [ ] T103 [P6] [US6] Write section: Platform-specific installation (macOS, Linux, Windows free download) in lessons/06-installing-antigravity/01-installation-guide.md
- [ ] T104 [P6] [US6] Write section: Initial setup (Agent Manager, workspace creation) in lessons/06-installing-antigravity/02-initial-setup.md
- [ ] T105 [P6] [US6] Write section: AI provider configuration (Google AI, Anthropic, OpenAI) in lessons/06-installing-antigravity/03-ai-authentication.md
- [ ] T106 [P6] [US6] Write section: Three surfaces overview (Agent Manager, Editor, Integrated Browser) from antigravity.md in lessons/06-installing-antigravity/04-three-surfaces.md
- [ ] T107 [P6] [US6] Write section: Verification test (create first agent, observe task list artifact) in lessons/06-installing-antigravity/05-verification.md
- [ ] T108 [P6] [US6] Write troubleshooting guide (new product instability warnings from Risk section) in lessons/06-installing-antigravity/06-troubleshooting.md

### Visual Assets

- [ ] T109 [P6] [US6] Capture screenshots: Antigravity installation on all 3 platforms in assets/screenshots/antigravity/install-\*.png
- [ ] T110 [P6] [US6] Capture screenshots: Agent Manager interface (create agent, workspace selection) in assets/screenshots/antigravity/agent-manager.png
- [ ] T111 [P6] [US6] Capture screenshots: Three surfaces side-by-side (Agent Manager, Editor, Browser) in assets/screenshots/antigravity/three-surfaces.png

### Exercises

- [ ] T112 [P6] [US6] Create exercise: "Install Antigravity and create first agent with simple task" in lessons/06-installing-antigravity/exercise-first-agent.md
- [ ] T113 [P6] [US6] Create comparison exercise: "How does Antigravity installation differ from Zed and Cursor?" in lessons/06-installing-antigravity/exercise-install-comparison.md

### Validation

- [ ] T114 [P6] [US6] Test installation on fresh systems (verify Nov 2025 public version availability)
- [ ] T115 [P6] [US6] Validate SC-002: Installation success rate for Antigravity
- [ ] T116 [P6] [US6] Run meta-commentary grep on Lesson 6 files

**Independent Test Criteria for US6**: Student installs Antigravity, creates first agent, observes task list artifact generation.

---

## Phase 9: Lesson 7 - Antigravity Agent Architecture & Features (US6 continued, Priority P6)

**Dependencies**: Phase 8 complete
**Lesson Duration**: 75-90 minutes
**Proficiency**: B1 (Layer 2 Collaboration, agent-driven paradigm)
**Type**: Feature exploration + agent workflow

### Content Creation

- [ ] T117 [P6] [US6] Write lesson introduction: "Agent-First Development Paradigm" in lessons/07-antigravity-agent-features/README.md
- [ ] T118 [P6] [US6] Write section: Artifact system deep-dive (Task Lists, Implementation Plans, Walkthroughs) from antigravity.md in lessons/07-antigravity-agent-features/01-artifact-system.md
- [ ] T119 [P6] [US6] Write section: Agent Manager workflows (creating agents, parallel task execution) in lessons/07-antigravity-agent-features/02-agent-manager.md
- [ ] T120 [P6] [US6] Write section: Antigravity Editor (tab autocomplete, agent sidebar) in lessons/07-antigravity-agent-features/03-editor-features.md
- [ ] T121 [P6] [US6] Write section: Integrated Browser (agent inside Chrome, research workflows) in lessons/07-antigravity-agent-features/04-integrated-browser.md
- [ ] T122 [P6] [US6] Write section: Context-aware editing (how agents track progress) in lessons/07-antigravity-agent-features/05-context-awareness.md
- [ ] T123 [P6] [US6] Write section: When to use Antigravity vs Zed vs Cursor (agent-driven tasks vs inline edits) in lessons/07-antigravity-agent-features/06-use-cases.md

### Three Roles Integration (INVISIBLE Framework)

- [ ] T124 [P6] [US6] Design observational exercise: "Create agent for feature implementation. Review implementation plan artifact. What did agent plan?" in lessons/07-antigravity-agent-features/exercise-implementation-plan.md
- [ ] T125 [P6] [US6] Design observational exercise: "Compare agent-generated code to Cursor Agent mode output. What differs?" in lessons/07-antigravity-agent-features/exercise-agent-comparison.md
- [ ] T126 [P6] [US6] Design convergence exercise: "Iterate on agent task via feedback. What improved?" in lessons/07-antigravity-agent-features/exercise-agent-iteration.md

### Visual Assets

- [ ] T127 [P6] [US6] Capture screenshots: Task List artifact example in assets/screenshots/antigravity/task-list-artifact.png
- [ ] T128 [P6] [US6] Capture screenshots: Implementation Plan artifact with annotations in assets/screenshots/antigravity/implementation-plan.png
- [ ] T129 [P6] [US6] Capture screenshots: Walkthrough artifact (final deliverable) in assets/screenshots/antigravity/walkthrough-artifact.png
- [ ] T130 [P6] [US6] Capture screenshots: Parallel task execution in Agent Manager in assets/screenshots/antigravity/parallel-tasks.png
- [ ] T131 [P6] [US6] Create diagram: Antigravity workflow (agent creation → planning → implementation → walkthrough) in assets/diagrams/antigravity-workflow.png

### Assessment

- [ ] T132 [P6] [US6] Recreate mini-project: "Temperature Converter CLI via Antigravity" (third IDE for capstone prep) in lessons/07-antigravity-agent-features/mini-project-temp-converter.md
- [ ] T133 [P6] [US6] Create reflection: "How does agent-driven workflow differ from inline/chat modes?" in lessons/07-antigravity-agent-features/reflection-agent-paradigm.md

### Validation

- [ ] T134 [P6] [US6] Run Three Roles invisibility check on Lesson 7
- [ ] T135 [P6] [US6] Validate SC-006: Can students configure advanced feature (agent creation)?
- [ ] T136 [P6] [US6] Run meta-commentary grep on Lesson 7 files

**Independent Test Criteria for US6**: Student completes mini-project via Antigravity, demonstrates understanding of artifact system.

---

## Phase 10: Lesson 8 - Comparative Capstone "Try With AI" (US5, US7, Priority P5/P7)

**Dependencies**: Phases 3-9 complete (all previous lessons)
**Lesson Duration**: 90-120 minutes
**Proficiency**: B1→L2/L3 (synthesis, critical evaluation)
**Type**: Observational capstone + selection framework

### Content Creation

- [ ] T137 [P5] [US5] Write lesson introduction: "Putting It All Together - IDE Selection & Comparison" in lessons/08-comparative-capstone/README.md
- [ ] T138 [P5] [US5] Write section: Selection criteria framework (performance, AI features, collaboration, ecosystem, learning curve) in lessons/08-comparative-capstone/01-selection-framework.md
- [ ] T139 [P5] [US5] Write section: Decision matrix walkthrough (3-way Zed/Cursor/Antigravity comparison) in lessons/08-comparative-capstone/02-decision-matrix.md
- [ ] T140 [P5] [US5] Write section: Applying criteria to project scenarios (3 hypothetical scenarios) in lessons/08-comparative-capstone/03-scenario-analysis.md
- [ ] T141 [P7] [US7] Write capstone activity: "Temperature Converter Observational Comparison" (prompt all 3 IDEs, compare outputs) in lessons/08-comparative-capstone/04-capstone-activity.md
- [ ] T142 [P7] [US7] Write section: Critical evaluation guidelines (AI output quality, security, correctness) in lessons/08-comparative-capstone/05-critical-evaluation.md
- [ ] T143 [P7] [US7] Write section: Reflection template (1500-2000 word structured comparison) in lessons/08-comparative-capstone/06-reflection-template.md

### Capstone Activity Design (OBSERVATIONAL - NO MANUAL CODING)

- [ ] T144 [P7] [US7] Create starter prompt template: "Create temperature converter Celsius/Fahrenheit with input validation" in lessons/08-comparative-capstone/capstone-prompt-template.md
- [ ] T145 [P7] [US7] Create observation checklist: "What to document per IDE" (response speed, code structure, understandability, AI interaction) in lessons/08-comparative-capstone/observation-checklist.md
- [ ] T146 [P7] [US7] Create comparison dimensions: (1) AI output quality, (2) IDE interaction pattern, (3) Response speed, (4) Code readability in lessons/08-comparative-capstone/comparison-dimensions.md
- [ ] T147 [P7] [US7] Create structured reflection questions (8-10 prompts guiding 1500-2000 word analysis) in lessons/08-comparative-capstone/reflection-questions.md

### Visual Assets

- [ ] T148 [P5] [US5] Finalize 3-way comparison matrix (performance, features, collaboration, ecosystem, curve) in assets/diagrams/final-comparison-matrix.png
- [ ] T149 [P5] [US5] Finalize decision flowchart (if speed→Zed, if familiarity→Cursor, if agent-driven→Antigravity) in assets/diagrams/final-decision-flowchart.png
- [ ] T150 [P7] [US7] Create capstone walkthrough screenshots: Same prompt in all 3 IDEs in assets/screenshots/capstone/temp-converter-\*.png

### Assessment

- [ ] T151 [P5] [US5] Create scenario exercise: "Choose IDE for 3 project types, justify with 2+ criteria each" in lessons/08-comparative-capstone/exercise-scenario-selection.md
- [ ] T152 [P7] [US7] Create capstone rubric: Grading criteria for 1500-2000 word comparison (SC-005 validation) in lessons/08-comparative-capstone/capstone-rubric.md
- [ ] T153 [P7] [US7] Create self-assessment survey: Confidence in AI-integrated development tools (SC-008) in lessons/08-comparative-capstone/self-assessment-survey.md

### Validation

- [ ] T154 [P7] [US7] Validate SC-005: Does capstone require 300+ word structured comparison with AI output differences?
- [ ] T155 [P7] [US7] Validate SC-007: Does capstone include critical evaluation of AI-generated code?
- [ ] T156 [P5] [US5] Validate SC-004: Can students select appropriate IDE for 3 scenarios with 2+ criteria each?
- [ ] T157 [P7] [US7] Run meta-commentary grep on Lesson 8 files
- [ ] T158 [P7] [US7] Run observational learning check: NO manual coding required, only AI prompting + evaluation

**Independent Test Criteria for US5**: Student selects IDE for 3 hypothetical projects, justifies with 2-3 specific criteria per choice.

**Independent Test Criteria for US7**: Student completes capstone (temperature converter observational comparison), writes 300+ word reflection referencing specific features.

---

## Phase 11: Cross-Cutting Concerns & Polish

**Dependencies**: Phases 3-10 complete (all lessons drafted)

### Integration & Toolchain (US6)

- [ ] T159 [P6] [US6] Write integration guide: Git workflows in all 3 IDEs (stage, commit, push, diff view) in lessons/integration-guide/01-git-integration.md
- [ ] T160 [P6] [US6] Write integration guide: Terminal usage (integrated vs external) in lessons/integration-guide/02-terminal-usage.md
- [ ] T161 [P6] [US6] Write integration guide: Formatter configuration (Black, Prettier, rustfmt) in lessons/integration-guide/03-formatters.md
- [ ] T162 [P6] [US6] Write integration guide: VS Code extension migration to Cursor in lessons/integration-guide/04-extension-migration.md

### Accessibility & Platform-Specific Content

- [ ] T163 [P1] [Accessibility] Create keyboard shortcuts reference for all 3 IDEs in assets/keyboard-shortcuts-reference.md
- [ ] T164 [P1] [Accessibility] Test all lessons with screen reader (NVDA/JAWS on Windows, VoiceOver on macOS)
- [ ] T165 [P1] [Accessibility] Add alt-text to ALL screenshots (110+ images across lessons)
- [ ] T166 [P2] [Platform] Validate platform-specific instructions tested on real systems (macOS, Linux, Windows)

### Visual Assets Completion

- [ ] T167 [P1] [Visual] Complete all screenshot captures (Zed: 15+, Cursor: 15+, Antigravity: 12+, Capstone: 5+)
- [ ] T168 [P1] [Visual] Complete all diagrams (comparison matrix, decision flowchart, workflows: 5+ total)
- [ ] T169 [P1] [Visual] Optional: Create video walkthroughs for all 3 IDE installations (accessibility alternative)

### Code Samples & Templates

- [ ] T170 [P3] [Code] Create temperature converter starter code (Python, simple CLI) in assets/code-samples/temp-converter-starter.py
- [ ] T171 [P7] [Code] Create example AI outputs for capstone (Zed version, Cursor version, Antigravity version) in assets/code-samples/capstone-examples/
- [ ] T172 [P1] [Templates] Finalize observational comparison template with examples in templates/observational-comparison-template.md
- [ ] T173 [P1] [Templates] Finalize reflection prompt template with rubric in templates/ide-comparison-reflection.md

### Constitutional Compliance Final Validation

- [ ] T174 [P1] [Constitution] Run meta-commentary grep across ALL 8 lessons (grep -ri "What to notice\|AI.*teach\|AI.*learn\|AI as\|AI now knows" lessons/)
- [ ] T175 [P1] [Constitution] Run Three Roles invisibility check across Lessons 3, 5, 7, 8 (no role labels, only action prompts)
- [ ] T176 [P1] [Constitution] Run observational learning check: Validate NO manual coding required before Part 4 (all exercises are AI-prompting + evaluation)
- [ ] T177 [P1] [Constitution] Run cognitive load check: A2 sections have 5-7 concepts, B1 sections have 7-10 concepts
- [ ] T178 [P1] [Constitution] Run evals-first check: Assessments defined BEFORE content exposure in each lesson

### Documentation & Navigation

- [ ] T179 [P1] [Docs] Update chapter README.md with complete lesson list, time estimates, prerequisites in 08-ai-native-ides/README.md
- [ ] T180 [P1] [Docs] Create chapter navigation (prev/next links between lessons) in each lesson README.md
- [ ] T181 [P1] [Docs] Create glossary of AI-native IDE terms in 08-ai-native-ides/glossary.md
- [ ] T182 [P1] [Docs] Create FAQ section addressing common issues (API costs, platform support, VS Code conflicts) in 08-ai-native-ides/faq.md
- [ ] T183 [P1] [Docs] Create resource list: Official docs, community forums, troubleshooting channels in 08-ai-native-ides/resources.md

### Testing Success Criteria

- [ ] T184 [P1] [Test] Validate SC-001: Can students explain "AI-native" in 2-3 sentences without brands?
- [ ] T185 [P2] [Test] Validate SC-002: 90% installation success rate in <30 minutes (test with 10+ users)
- [ ] T186 [P3] [Test] Validate SC-003: Students guide AI to complete factorial function in <10 minutes
- [ ] T187 [P5] [Test] Validate SC-004: Students select IDE for 3 scenarios with 2+ criteria each
- [ ] T188 [P7] [Test] Validate SC-005: Capstone produces 300+ word structured comparison
- [ ] T189 [P6] [Test] Validate SC-006: Students configure 1+ advanced feature (multi-model, .cursorrules, agent)
- [ ] T190 [P7] [Test] Validate SC-007: Students identify 1+ instance where AI code needed correction
- [ ] T191 [P8] [Test] Validate SC-008: 80% report increased confidence via survey

---

## Dependency Graph (Execution Order)

**Critical Path**:

```
Phase 1 (Setup)
  ↓
Phase 2 (Research & Templates)
  ↓
Phase 3 (Lesson 1: Concepts) → Phase 4 (Lesson 2: Zed Install) → Phase 5 (Lesson 3: Zed Features)
  ↓
Phase 6 (Lesson 4: Cursor Install) → Phase 7 (Lesson 5: Cursor Features)
  ↓
Phase 8 (Lesson 6: Antigravity Install) → Phase 9 (Lesson 7: Antigravity Features)
  ↓
Phase 10 (Lesson 8: Capstone)
  ↓
Phase 11 (Cross-Cutting & Polish)
```

**Parallel Work Opportunities**:

- Visual assets (T038-T042, T075-T077, T109-T111) can be captured concurrently with content writing
- Accessibility tasks (T163-T165) can proceed once screenshots exist
- Validation tasks (T174-T178) can run incrementally after each lesson phase

---

## Success Metrics Tracking

### Per Lesson:

- [ ] Lesson 1: SC-001 validated (conceptual understanding)
- [ ] Lesson 2: SC-002 validated (installation success)
- [ ] Lesson 3: SC-003 validated (AI task completion <10 min)
- [ ] Lesson 4-5: SC-004 prep (comparison mindset)
- [ ] Lesson 6-7: SC-006 validated (advanced config)
- [ ] Lesson 8: SC-004, SC-005, SC-007, SC-008 validated (capstone + selection + evaluation + confidence)

### Constitutional Compliance:

- [ ] Zero meta-commentary violations (grep returns no matches)
- [ ] Three Roles invisible (no role labels, only action prompts)
- [ ] Observational approach enforced (no manual coding before Part 4)
- [ ] Cognitive load appropriate (A2: 5-7 concepts, B1: 7-10)
- [ ] Evals-first pattern maintained (assessments before content)

### Deliverables:

- [ ] 8 complete lessons (README + subsections per lesson)
- [ ] 110+ tasks completed (content, visuals, exercises, validation)
- [ ] 50+ screenshots captured and annotated
- [ ] 5+ diagrams created (comparison matrix, flowcharts, workflows)
- [ ] 3+ video walkthroughs (optional, accessibility)
- [ ] Full observational capstone with rubric and reflection template

---

## Notes for Implementers

### "Try With AI" Policy for Lesson Authors

**CRITICAL**: This chapter teaches students to observe and evaluate AI-generated code, NOT to write code manually. Students at Part 2 have ZERO programming experience.

**All exercises must follow observational learning pattern**:

1. Provide clear natural language prompt for students to give AI
2. Specify what to observe in AI's output
3. Include reflection questions about AI's choices
4. Compare outputs across IDEs (not student's manual code vs AI)

**Example CORRECT exercise**:

```
**Exercise**: Prompt AI to refactor repetitive code
1. Open the provided Python file with repeated validation logic
2. Ask AI: "Refactor this code to follow DRY principle"
3. Observe: What pattern did AI introduce? (loop, function, class method?)
4. Reflect: Would you accept this change? Why/why not?
```

**Example INCORRECT exercise** (violates observational approach):

```
**Exercise**: Refactor the following code manually, then ask AI for suggestions
[This requires manual coding skills students don't have yet]
```

### Context7 Research Integration

All IDE features documented in this task list derive from:

- **Zed**: /zed-industries/zed (897 code snippets, multi-model config, inline assistant, tab autocomplete, git integration)
- **Cursor**: /llmstxt/cursor_llms_txt (5261 snippets, Chat mode, Agent mode, .cursorrules, diff review)
- **Antigravity**: context/16_chapter8\_\_ides/antigravity.md (three surfaces, artifact system, agent architecture)

Refer to assets/research/\*.md for detailed feature documentation.

### Version Stability Note

Antigravity launched Nov 18, 2025 (2 days before spec creation). Expect more frequent updates/breaking changes than mature Zed/Cursor. Design lessons with version-agnostic concepts (agent architecture principles) over specific UI details. Maintain errata document for known changes.

---

**END OF TASKS.MD**

Total Tasks: 191
Estimated Completion Time: 6-8 weeks (1-2 content creators working full-time)
Priority: P1 tasks first (foundational), then sequential lesson phases (P2→P3→P4→P5→P6→P7), polish last (P8)

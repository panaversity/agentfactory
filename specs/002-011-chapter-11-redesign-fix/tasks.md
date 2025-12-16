# Implementation Tasks: Chapter 11 Context Engineering Redesign

**Feature**: 002-011-chapter-11-redesign-fix
**Created**: 2025-11-18
**Status**: Ready for Implementation

---

## Phase 0: Setup & Preparation (Tasks 001-010)

### T001: Verify specification completeness

- **Owner**: Human review
- **Input**: spec.md, plan.md
- **Output**: Specification approval
- **Test**: All sections complete, no NEEDS CLARIFICATION
- **Status**: ⬜ Pending

### T002: Read chapter-index.md for context

- **Owner**: Implementation agent
- **Input**: apps/learn-app/docs/chapter-index.md
- **Output**: Chapter 11 metadata (Part 3, B1 tier, prerequisites)
- **Test**: Confirm Part 3 (no programming), B1 proficiency
- **Status**: ⬜ Pending

### T003: Read existing Chapter 11 README

- **Owner**: Implementation agent
- **Input**: apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/README.md
- **Output**: Current chapter structure
- **Test**: Understand current organization
- **Status**: ⬜ Pending

### T004: Backup existing lessons

- **Owner**: Implementation agent
- **Action**: Copy all existing lesson files to backup/002-011-chapter-11-old/
- **Output**: Backup directory with 9 lesson files
- **Test**: All files copied, originals preserved
- **Status**: ⬜ Pending

### T005: Read external research (Anthropic article)

- **Owner**: Implementation agent
- **Input**: Anthropic article URL (from spec.md)
- **Output**: Key concepts (extraction, consolidation, memory generation)
- **Test**: Concepts match spec.md Section 3 (Research Integration)
- **Status**: ⬜ Pending

### T006: Read external research (GitHub spec)

- **Owner**: Implementation agent
- **Input**: GitHub spec URL (from spec.md)
- **Output**: Pedagogical patterns (compare-and-contrast, multi-session)
- **Test**: Patterns match spec.md Section 3 (Research Integration)
- **Status**: ⬜ Pending

### T007: Read external research (Google PDF)

- **Owner**: Implementation agent
- **Input**: P3-Context Engineering: Sessions & Memory.pdf
- **Output**: Sessions architecture, memory generation pipeline
- **Test**: Concepts match spec.md Section 3 (Research Integration)
- **Status**: ⬜ Pending

### T008: Create chapter directory structure

- **Owner**: Implementation agent
- **Action**: Verify apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/ exists
- **Output**: Directory ready
- **Test**: Directory exists and writable
- **Status**: ⬜ Pending

### T009: Create skills directory for new skills

- **Owner**: Implementation agent
- **Action**: Verify .claude/skills/ exists for memory-file-architecture and tool-selection-framework
- **Output**: Skills directory ready
- **Test**: Directory exists
- **Status**: ⬜ Pending

### T010: Review constitutional constraints

- **Owner**: Implementation agent
- **Input**: .specify/memory/constitution.md
- **Output**: 7 principles understood (Spec Primacy, Progressive Complexity, Factual Accuracy, etc.)
- **Test**: All principles listed in plan.md Section VII
- **Status**: ⬜ Pending

---

## Phase 1: Lesson 1 Implementation (Manual Foundation) (Tasks 011-025)

### T011: Create Lesson 1 frontmatter

- **Owner**: content-implementer
- **Input**: plan.md Lesson 1 spec
- **Output**: YAML frontmatter (title, sidebar_position, learning objectives, estimated_time, proficiency_level)
- **Test**: All required fields present
- **Status**: ⬜ Pending

### T012: Write Lesson 1 introduction

- **Owner**: content-implementer
- **Content**: "The Problem You're About to Solve" section
- **Constraint**: NO code examples, Markdown-only context
- **Test**: Introduces context windows without programming
- **Status**: ⬜ Pending

### T013: Write Lesson 1 core concept (Context Windows)

- **Owner**: content-implementer
- **Content**: Explain context windows as "AI working memory"
- **Example**: Use markdown session note analogy
- **Test**: B1-appropriate, no code
- **Status**: ⬜ Pending

### T014: Write Lesson 1 core concept (Token Counting)

- **Owner**: content-implementer
- **Content**: Rule of thumb: 1 word ≈ 1.2 tokens
- **Example**: Manual calculation using session note
- **Test**: Students can estimate tokens manually
- **Status**: ⬜ Pending

### T015: Write Lesson 1 core concept (Utilization Percentage)

- **Owner**: content-implementer
- **Content**: (tokens_used / token_limit) × 100
- **Example**: 23K / 200K = 11.5%
- **Test**: Simple arithmetic, no code
- **Status**: ⬜ Pending

### T016: Write Lesson 1 core concept (Context Limits)

- **Owner**: content-implementer
- **Content**: Claude Sonnet 4.5 (200K standard, 1M extended), Gemini 1.5 Pro (2M)
- **Source**: Anthropic docs, Google docs (cite accurately)
- **Test**: Factual-verifier passes
- **Status**: ⬜ Pending

### T017: Write Lesson 1 core concept (Warning Thresholds)

- **Owner**: content-implementer
- **Content**: 70% (plan checkpoint), 80% (create soon), 90% (create NOW)
- **Example**: Traffic light analogy (green/yellow/red)
- **Test**: Clear, memorable thresholds
- **Status**: ⬜ Pending

### T018: Write Lesson 1 Exercise 1 (Session Note Template)

- **Owner**: content-implementer
- **Content**: Provide markdown template for session notes
- **Task**: Students write session note for hypothetical bug fix
- **Test**: NO AI assistance (Layer 1 manual)
- **Status**: ⬜ Pending

### T019: Write Lesson 1 Exercise 2 (Manual Token Estimation)

- **Owner**: content-implementer
- **Content**: Given session note, estimate tokens manually
- **Task**: Count words, multiply by 1.2, calculate utilization
- **Test**: Simple arithmetic, reinforces concept
- **Status**: ⬜ Pending

### T020: Write Lesson 1 Exercise 3 (Threshold Recognition)

- **Owner**: content-implementer
- **Content**: Given 3 utilization percentages, identify warning level
- **Task**: 45% (safe), 75% (plan), 85% (act now)
- **Test**: Applies threshold concept
- **Status**: ⬜ Pending

### T021: Write Lesson 1 "Try With AI" section

- **Owner**: content-implementer
- **Content**: 4 prompts for AI collaboration on context tracking
- **Test**: Prompts appropriate for post-manual-foundation practice
- **Status**: ⬜ Pending

### T022: Validate Lesson 1 concept count

- **Owner**: Implementation agent
- **Action**: Count distinct concepts in Lesson 1
- **Test**: 7-10 concepts (B1 limit)
- **Status**: ⬜ Pending

### T023: Validate Lesson 1 NO code constraint

- **Owner**: Implementation agent
- **Action**: Search for programming code blocks
- **Test**: grep -E "^(from |import |def |class |function |const |let |var )" returns empty
- **Status**: ⬜ Pending

### T024: Validate Lesson 1 Layer 1 approach

- **Owner**: Implementation agent
- **Action**: Verify exercises are manual (NO AI)
- **Test**: Exercises 1-3 do not use AI collaboration
- **Status**: ⬜ Pending

### T025: Write Lesson 1 to file

- **Owner**: content-implementer
- **Output**: apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/01-context-windows-token-counting.md
- **Test**: File exists, renders correctly in Docusaurus
- **Status**: ⬜ Pending

---

## Phase 2: Lesson 2 Implementation (Manual Foundation) (Tasks 026-040)

### T026: Create Lesson 2 frontmatter

- **Status**: ⬜ Pending

### T027: Write Lesson 2 introduction

- **Content**: Compare-and-contrast pedagogy (healthy vs degraded session)
- **Status**: ⬜ Pending

### T028: Write Lesson 2 core concept (Degradation Symptoms)

- **Content**: 7 symptoms (repetitive, forgotten, performance drop, generic, lost context, confusion, reduced quality)
- **Status**: ⬜ Pending

### T029: Write Lesson 2 core concept (Manual Tracking)

- **Content**: Degradation checklist template
- **Status**: ⬜ Pending

### T030: Write Lesson 2 Example 1 (Healthy Session Transcript)

- **Content**: Conversation showing good context management
- **Status**: ⬜ Pending

### T031: Write Lesson 2 Example 2 (Degraded Session Transcript)

- **Content**: Conversation showing degradation symptoms
- **Status**: ⬜ Pending

### T032: Write Lesson 2 Exercise 1 (Symptom Identification)

- **Task**: Mark symptoms present/absent in given transcripts
- **Test**: NO AI (manual practice)
- **Status**: ⬜ Pending

### T033: Write Lesson 2 Exercise 2 (Create Degradation Checklist)

- **Task**: Students create personalized checklist
- **Status**: ⬜ Pending

### T034: Write Lesson 2 Exercise 3 (Compare and Contrast)

- **Task**: Analyze 2 transcripts side-by-side
- **Status**: ⬜ Pending

### T035: Write Lesson 2 "Try With AI" section

- **Status**: ⬜ Pending

### T036: Validate Lesson 2 concept count (7-10)

- **Status**: ⬜ Pending

### T037: Validate Lesson 2 NO code constraint

- **Status**: ⬜ Pending

### T038: Validate Lesson 2 Layer 1 approach

- **Status**: ⬜ Pending

### T039: Validate Lesson 2 research integration (GitHub spec compare-and-contrast)

- **Status**: ⬜ Pending

### T040: Write Lesson 2 to file (02-degradation-symptoms-manual-tracking.md)

- **Status**: ⬜ Pending

---

## Phase 3: Lesson 3 Implementation (AI Collaboration, Three Roles) (Tasks 041-060)

### T041: Create Lesson 3 frontmatter

- **Status**: ⬜ Pending

### T042: Write Lesson 3 introduction

- **Content**: Transition from manual to AI collaboration
- **Status**: ⬜ Pending

### T043: Write Lesson 3 core concept (Progressive Loading Pattern)

- **Content**: Foundation → Current → On-Demand phases
- **Source**: Anthropic article (smallest set of high-signal tokens)
- **Status**: ⬜ Pending

### T044: Write Lesson 3 core concept (Foundation Loading)

- **Content**: Always-loaded context (10-15% of window)
- **Example**: CLAUDE.md, architecture.md, decisions.md
- **Status**: ⬜ Pending

### T045: Write Lesson 3 core concept (Current Loading)

- **Content**: Task-relevant context (20-30%)
- **Status**: ⬜ Pending

### T046: Write Lesson 3 core concept (On-Demand Loading)

- **Content**: Fetch as needed (reserve 30%)
- **Status**: ⬜ Pending

### T047: Write Lesson 3 Three Roles demonstration (AI as Teacher)

- **Content**: AI suggests progressive loading pattern student didn't know
- **Example**: "For your documentation project, try loading foundation (patterns) first, then current (chapter files), then on-demand (examples)"
- **Test**: AI teaches NEW pattern
- **Status**: ⬜ Pending

### T048: Write Lesson 3 Three Roles demonstration (AI as Student)

- **Content**: Student corrects AI's loading suggestion with project constraint
- **Example**: "Actually, our `conventions.md` is critical foundation, not on-demand"
- **Test**: Student teaches AI project-specific knowledge
- **Status**: ⬜ Pending

### T049: Write Lesson 3 Three Roles demonstration (AI as Co-Worker)

- **Content**: Iterate toward optimal loading strategy (2+ rounds)
- **Example**: Round 1: AI suggests, Round 2: Student refines, Round 3: Converge on two-tier strategy
- **Test**: Complete convergence loop
- **Status**: ⬜ Pending

### T050: Write Lesson 3 Exercise 1 (Design Foundation Loading)

- **Task**: Given project structure, identify foundation files
- **Method**: Collaborative prompt with AI
- **Status**: ⬜ Pending

### T051: Write Lesson 3 Exercise 2 (Design Current Loading)

- **Task**: Given task description, identify task-relevant files
- **Status**: ⬜ Pending

### T052: Write Lesson 3 Exercise 3 (Calculate Reserved Capacity)

- **Task**: Foundation (15%) + Current (25%) + On-Demand (?) = Reserve 30%
- **Status**: ⬜ Pending

### T053: Write Lesson 3 "Try With AI" section

- **Status**: ⬜ Pending

### T054: Validate Lesson 3 concept count (7-10)

- **Status**: ⬜ Pending

### T055: Validate Lesson 3 NO code constraint

- **Status**: ⬜ Pending

### T056: Validate Lesson 3 Three Roles complete

- **Test**: Teacher, Student, Co-Worker all demonstrated with examples
- **Status**: ⬜ Pending

### T057: Validate Lesson 3 research integration (Anthropic article)

- **Test**: "Smallest set of high-signal tokens" cited
- **Status**: ⬜ Pending

### T058: Validate Lesson 3 Layer 2 transition

- **Test**: Students USE AI collaboration (not manual)
- **Status**: ⬜ Pending

### T059: Write Lesson 3 to file (03-progressive-loading-strategy.md)

- **Status**: ⬜ Pending

### T060: Create validation report for Lesson 3

- **Content**: Document Three Roles evidence
- **Status**: ⬜ Pending

---

## Phase 4: Lesson 4 Implementation (Compression, Three Roles) (Tasks 061-078)

### T061: Create Lesson 4 frontmatter

- **Status**: ⬜ Pending

### T062: Write Lesson 4 introduction

- **Content**: When to compress vs when to continue
- **Status**: ⬜ Pending

### T063: Write Lesson 4 core concept (Compression Triggers)

- **Content**: IF utilization > 80% AND duration > 60min THEN compress
- **Status**: ⬜ Pending

### T064: Write Lesson 4 core concept (Checkpoint Structure)

- **Content**: Architecture decisions, Progress summary, Next steps, Context to preserve
- **Source**: Anthropic article (extraction + consolidation)
- **Status**: ⬜ Pending

### T065: Write Lesson 4 core concept (Extraction)

- **Content**: Pull key facts from session
- **Status**: ⬜ Pending

### T066: Write Lesson 4 core concept (Consolidation)

- **Content**: Compress into <600 tokens
- **Status**: ⬜ Pending

### T067: Write Lesson 4 Three Roles (AI as Teacher)

- **Content**: AI suggests checkpoint creation based on symptoms
- **Status**: ⬜ Pending

### T068: Write Lesson 4 Three Roles (AI as Student)

- **Content**: Student teaches AI what to preserve in checkpoint
- **Status**: ⬜ Pending

### T069: Write Lesson 4 Three Roles (AI as Co-Worker)

- **Content**: Iterate toward concise checkpoint (2+ rounds)
- **Status**: ⬜ Pending

### T070: Write Lesson 4 Exercise 1 (Identify Compression Trigger)

- **Task**: Given 3 scenarios, mark which need compression
- **Status**: ⬜ Pending

### T071: Write Lesson 4 Exercise 2 (Create Checkpoint)

- **Task**: Collaborative prompt to create CHECKPOINT.md
- **Status**: ⬜ Pending

### T072: Write Lesson 4 Exercise 3 (Session Restart)

- **Task**: Practice restarting with checkpoint
- **Status**: ⬜ Pending

### T073: Write Lesson 4 "Try With AI" section

- **Status**: ⬜ Pending

### T074: Validate Lesson 4 concept count (7-10)

- **Status**: ⬜ Pending

### T075: Validate Lesson 4 Three Roles complete

- **Status**: ⬜ Pending

### T076: Validate Lesson 4 research integration (Anthropic extraction/consolidation)

- **Status**: ⬜ Pending

### T077: Write Lesson 4 to file (04-context-compression-session-restart.md)

- **Status**: ⬜ Pending

### T078: Create validation report for Lesson 4

- **Status**: ⬜ Pending

---

## Phase 5: Lesson 5 Implementation (Isolation, Three Roles) (Tasks 079-096)

### T079: Create Lesson 5 frontmatter

- **Status**: ⬜ Pending

### T080: Write Lesson 5 introduction

- **Content**: When to isolate vs when to continue mixed
- **Status**: ⬜ Pending

### T081: Write Lesson 5 core concept (Task Similarity Scoring)

- **Content**: Scoring algorithm (domain +30, models +20, service +20, routes +15, tests +15)
- **Status**: ⬜ Pending

### T082: Write Lesson 5 core concept (Isolation Decision)

- **Content**: IF similarity < 50% THEN isolate
- **Status**: ⬜ Pending

### T083: Write Lesson 5 core concept (Context Pollution)

- **Content**: Mixed patterns lead to degradation
- **Status**: ⬜ Pending

### T084: Write Lesson 5 Three Roles (AI as Teacher)

- **Content**: AI suggests when to start new session
- **Status**: ⬜ Pending

### T085: Write Lesson 5 Three Roles (AI as Student)

- **Content**: Student teaches AI project-specific similarity criteria
- **Status**: ⬜ Pending

### T086: Write Lesson 5 Three Roles (AI as Co-Worker)

- **Content**: Iterate toward isolation decision (2+ rounds)
- **Status**: ⬜ Pending

### T087: Write Lesson 5 Exercise 1 (Calculate Similarity Score)

- **Task**: Given 2 tasks, score similarity
- **Status**: ⬜ Pending

### T088: Write Lesson 5 Exercise 2 (Decide Isolation)

- **Task**: Apply decision rule to 3 scenarios
- **Status**: ⬜ Pending

### T089: Write Lesson 5 Exercise 3 (Multi-Session Workflow)

- **Task**: Design session structure for complex project
- **Status**: ⬜ Pending

### T090: Write Lesson 5 "Try With AI" section

- **Status**: ⬜ Pending

### T091: Validate Lesson 5 concept count (7-10)

- **Status**: ⬜ Pending

### T092: Validate Lesson 5 Three Roles complete

- **Status**: ⬜ Pending

### T093: Validate Lesson 5 research integration (GitHub multi-session patterns)

- **Status**: ⬜ Pending

### T094: Write Lesson 5 to file (05-context-isolation-parallel-tasks.md)

- **Status**: ⬜ Pending

### T095: Create validation report for Lesson 5

- **Status**: ⬜ Pending

### T096: Checkpoint: Lessons 1-5 complete

- **Test**: All Layer 1-2 lessons written, Three Roles demonstrated in L3-5
- **Status**: ⬜ Pending

---

## Phase 6: Lesson 6 Implementation (Memory Files, Skill Creation) (Tasks 097-115)

### T097: Create Lesson 6 frontmatter

- **Status**: ⬜ Pending

### T098: Write Lesson 6 introduction

- **Content**: Transition from session-level to project-level persistence
- **Status**: ⬜ Pending

### T099: Write Lesson 6 core concept (Memory Files)

- **Content**: CLAUDE.md, architecture.md, decisions.md
- **Source**: Google PDF (memory generation pipeline)
- **Status**: ⬜ Pending

### T100: Write Lesson 6 core concept (CLAUDE.md Structure)

- **Content**: Project patterns, coding conventions, AI preferences
- **Status**: ⬜ Pending

### T101: Write Lesson 6 core concept (architecture.md Structure)

- **Content**: System components, dependencies, constraints
- **Status**: ⬜ Pending

### T102: Write Lesson 6 core concept (decisions.md Structure)

- **Content**: ADRs (Date, Decision, Reasoning, Alternatives)
- **Status**: ⬜ Pending

### T103: Write Lesson 6 core concept (Memory Update Strategy)

- **Content**: Read at start, update at end, merge conflicts
- **Status**: ⬜ Pending

### T104: Write Lesson 6 Exercise 1 (Create CLAUDE.md)

- **Task**: Design memory file for hypothetical project
- **Status**: ⬜ Pending

### T105: Write Lesson 6 Exercise 2 (Create architecture.md)

- **Status**: ⬜ Pending

### T106: Write Lesson 6 Exercise 3 (Create decisions.md)

- **Task**: Write 3 ADRs for architectural decisions
- **Status**: ⬜ Pending

### T107: Write Lesson 6 "Try With AI" section

- **Status**: ⬜ Pending

### T108: Create memory-file-architecture skill

- **Owner**: Implementation agent
- **Location**: .claude/skills/memory-file-architecture/
- **Content**: Persona (Memory architect), Questions (What patterns? What decisions?), Principles (Extract, Consolidate, Persist)
- **Status**: ⬜ Pending

### T109: Validate memory-file-architecture skill structure

- **Test**: Contains Persona + Questions + Principles sections
- **Status**: ⬜ Pending

### T110: Integrate memory-file-architecture skill in Lesson 6

- **Content**: "Try With AI" prompts invoke skill
- **Status**: ⬜ Pending

### T111: Validate Lesson 6 concept count (7-10)

- **Status**: ⬜ Pending

### T112: Validate Lesson 6 research integration (Google PDF memory generation)

- **Status**: ⬜ Pending

### T113: Validate Lesson 6 Layer 3 approach

- **Test**: Reusable skill created
- **Status**: ⬜ Pending

### T114: Write Lesson 6 to file (06-memory-files-persistent-intelligence.md)

- **Status**: ⬜ Pending

### T115: Document memory-file-architecture skill in README

- **Status**: ⬜ Pending

---

## Phase 7: Lesson 7 Implementation (Tool Selection, Skill Creation) (Tasks 116-133)

### T116: Create Lesson 7 frontmatter

- **Status**: ⬜ Pending

### T117: Write Lesson 7 introduction

- **Content**: Choosing right tool for context requirements
- **Status**: ⬜ Pending

### T118: Write Lesson 7 core concept (Claude Code)

- **Content**: 200K context, deep reasoning, focused implementation
- **Status**: ⬜ Pending

### T119: Write Lesson 7 core concept (Gemini CLI)

- **Content**: 2M context, broad exploration, pattern analysis
- **Status**: ⬜ Pending

### T120: Write Lesson 7 core concept (Tool Selection Decision Logic)

- **Content**: Codebase size, reasoning depth, complexity
- **Source**: GitHub spec (guardrails, decision criteria)
- **Status**: ⬜ Pending

### T121: Write Lesson 7 core concept (Multi-Phase Workflows)

- **Content**: Gemini (explore) → Claude (implement)
- **Status**: ⬜ Pending

### T122: Write Lesson 7 Exercise 1 (Evaluate 5 Scenarios)

- **Task**: Apply decision logic to recommend tool
- **Status**: ⬜ Pending

### T123: Write Lesson 7 Exercise 2 (Design Multi-Phase Workflow)

- **Task**: Plan exploration + implementation phases
- **Status**: ⬜ Pending

### T124: Write Lesson 7 "Try With AI" section

- **Status**: ⬜ Pending

### T125: Create tool-selection-framework skill

- **Owner**: Implementation agent
- **Location**: .claude/skills/tool-selection-framework/
- **Content**: Persona (Tool advisor), Questions (Codebase size? Complexity?), Principles (Match tool to task)
- **Status**: ⬜ Pending

### T126: Validate tool-selection-framework skill structure

- **Status**: ⬜ Pending

### T127: Integrate tool-selection-framework skill in Lesson 7

- **Status**: ⬜ Pending

### T128: Validate Lesson 7 concept count (7-10)

- **Status**: ⬜ Pending

### T129: Validate Lesson 7 research integration (GitHub decision criteria)

- **Status**: ⬜ Pending

### T130: Validate Lesson 7 Layer 3 approach

- **Test**: Reusable skill created
- **Status**: ⬜ Pending

### T131: Write Lesson 7 to file (07-tool-selection-framework.md)

- **Status**: ⬜ Pending

### T132: Document tool-selection-framework skill in README

- **Status**: ⬜ Pending

### T133: Checkpoint: Both skills created

- **Test**: memory-file-architecture + tool-selection-framework in .claude/skills/
- **Status**: ⬜ Pending

---

## Phase 8: Lesson 8 Implementation (Integrated Debugging) (Tasks 134-150)

### T134: Create Lesson 8 frontmatter

- **Status**: ⬜ Pending

### T135: Write Lesson 8 introduction

- **Content**: Apply all techniques from Lessons 1-7
- **Status**: ⬜ Pending

### T136: Write Lesson 8 Scenario 1 (High Utilization)

- **Content**: 85% utilization, session > 90min
- **Task**: Diagnose and fix using compression (Lesson 4)
- **Status**: ⬜ Pending

### T137: Write Lesson 8 Scenario 2 (Degradation Symptoms)

- **Content**: AI repeating suggestions, forgotten patterns
- **Task**: Identify symptoms (Lesson 2), apply progressive loading (Lesson 3)
- **Status**: ⬜ Pending

### T138: Write Lesson 8 Scenario 3 (Context Pollution)

- **Content**: Mixed tasks, cross-contaminated patterns
- **Task**: Apply isolation (Lesson 5)
- **Status**: ⬜ Pending

### T139: Write Lesson 8 Scenario 4 (Lost Intelligence)

- **Content**: New session, forgot project patterns
- **Task**: Load memory files (Lesson 6)
- **Status**: ⬜ Pending

### T140: Write Lesson 8 Exercise 1 (Diagnose Scenario 1)

- **Status**: ⬜ Pending

### T141: Write Lesson 8 Exercise 2 (Diagnose Scenario 2)

- **Status**: ⬜ Pending

### T142: Write Lesson 8 Exercise 3 (Diagnose Scenario 3)

- **Status**: ⬜ Pending

### T143: Write Lesson 8 Exercise 4 (Diagnose Scenario 4)

- **Status**: ⬜ Pending

### T144: Write Lesson 8 "Try With AI" section

- **Status**: ⬜ Pending

### T145: Validate Lesson 8 concept count (7-10)

- **Status**: ⬜ Pending

### T146: Validate Lesson 8 integration completeness

- **Test**: References Lessons 1-7 techniques
- **Status**: ⬜ Pending

### T147: Validate Lesson 8 research integration (Google PDF production considerations)

- **Status**: ⬜ Pending

### T148: Write Lesson 8 to file (08-hands-on-debugging-optimization.md)

- **Status**: ⬜ Pending

### T149: Checkpoint: Lessons 1-8 complete

- **Test**: All foundation, collaboration, intelligence, and validation lessons written
- **Status**: ⬜ Pending

### T150: Pre-capstone validation checkpoint

- **Test**: Students have accumulated knowledge for capstone
- **Status**: ⬜ Pending

---

## Phase 9: Lesson 9 Implementation (Capstone Spec-Driven) (Tasks 151-170)

### T151: Create Lesson 9 frontmatter

- **Status**: ⬜ Pending

### T152: Write Lesson 9 introduction

- **Content**: Orchestrate all techniques in specification
- **Status**: ⬜ Pending

### T153: Write Lesson 9 core concept (Specification Quality)

- **Content**: Intent, Success Criteria, Functional Requirements, Architecture, Algorithms, Non-Goals
- **Status**: ⬜ Pending

### T154: Write Lesson 9 core concept (Intent Statement)

- **Content**: WHAT the tool does (not HOW)
- **Status**: ⬜ Pending

### T155: Write Lesson 9 core concept (Success Criteria)

- **Content**: Measurable, falsifiable criteria
- **Status**: ⬜ Pending

### T156: Write Lesson 9 core concept (System Architecture)

- **Content**: 6 components (Context Monitor, Checkpoint Engine, Similarity Analyzer, Memory Manager, Tool Selector, Orchestrator)
- **Status**: ⬜ Pending

### T157: Write Lesson 9 Example (Anti-Pattern Spec)

- **Content**: Show vague spec as what NOT to do
- **Status**: ⬜ Pending

### T158: Write Lesson 9 Example (Implementation-Ready Spec)

- **Content**: Show complete spec as target
- **Status**: ⬜ Pending

### T159: Write Lesson 9 Specification Checklist

- **Content**: Intent clarity, Success criteria measurable, Components complete, NO implementation code
- **Status**: ⬜ Pending

### T160: Write Lesson 9 Capstone Exercise

- **Task**: Write 3-5 page specification for context-aware development tool
- **Constraint**: ZERO implementation code
- **Status**: ⬜ Pending

### T161: Write Lesson 9 Peer Review Exercise

- **Task**: Exchange specs, identify ambiguities/missing details
- **Status**: ⬜ Pending

### T162: Write Lesson 9 Validation Checklist

- **Content**: Map capstone to all Lessons 1-8
- **Status**: ⬜ Pending

### T163: Write Lesson 9 "Try With AI" section

- **Status**: ⬜ Pending

### T164: Validate Lesson 9 concept count (7-10)

- **Status**: ⬜ Pending

### T165: Validate Lesson 9 ZERO implementation code

- **Test**: Capstone exercise forbids code, uses plain English algorithms
- **Status**: ⬜ Pending

### T166: Validate Lesson 9 Layer 4 approach

- **Test**: Specification-only, orchestrates Lessons 1-8
- **Status**: ⬜ Pending

### T167: Validate Lesson 9 constitutional compliance (Specification Primacy)

- **Test**: Intent before implementation enforced
- **Status**: ⬜ Pending

### T168: Write Lesson 9 to file (09-capstone-spec-driven-orchestration.md)

- **Status**: ⬜ Pending

### T169: Validate capstone completeness

- **Test**: Students can write complete spec using accumulated knowledge
- **Status**: ⬜ Pending

### T170: Checkpoint: All 9 lessons complete

- **Status**: ⬜ Pending

---

## Phase 10: Chapter README and Cross-Cutting (Tasks 171-185)

### T171: Create new README.md

- **Content**: Chapter overview, learning objectives, lesson structure
- **Status**: ⬜ Pending

### T172: Write README learning objectives

- **Content**: 6 objectives (manual tracking, progressive loading, compression/isolation, memory files, tool selection, spec writing)
- **Status**: ⬜ Pending

### T173: Write README lesson navigation

- **Content**: Table with all 9 lessons, layers, concepts
- **Status**: ⬜ Pending

### T174: Write README research foundation

- **Content**: Cite Anthropic article, GitHub spec, Google PDF
- **Status**: ⬜ Pending

### T175: Write README success criteria

- **Content**: How students know they've mastered chapter
- **Status**: ⬜ Pending

### T176: Validate README completeness

- **Test**: All sections present
- **Status**: ⬜ Pending

### T177: Cross-lesson consistency check

- **Test**: Progressive loading (L3) referenced in compression (L4), memory files (L6) referenced in debugging (L8)
- **Status**: ⬜ Pending

### T178: Lesson ending protocol check

- **Test**: All lessons end with "Try With AI" ONLY (no "What's Next", no "Success")
- **Status**: ⬜ Pending

### T179: Three Roles demonstration validation

- **Test**: Lessons 3, 4, 5 all show Teacher/Student/Co-Worker with convergence loops
- **Status**: ⬜ Pending

### T180: Cognitive load validation

- **Test**: All lessons 7-10 concepts maximum
- **Status**: ⬜ Pending

### T181: NO code constraint validation (Lessons 1-8)

- **Test**: grep -r "^from \|^import \|^def \|^class " lessons/0[1-8]\*.md returns empty
- **Status**: ⬜ Pending

### T182: Research citation accuracy check

- **Test**: All context window sizes, research concepts correctly cited
- **Status**: ⬜ Pending

### T183: Skill documentation check

- **Test**: Both skills (memory-file-architecture, tool-selection-framework) documented in README or lessons
- **Status**: ⬜ Pending

### T184: Write README to file

- **Output**: apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/README.md
- **Status**: ⬜ Pending

### T185: Checkpoint: Chapter infrastructure complete

- **Test**: README + 9 lessons + 2 skills all in place
- **Status**: ⬜ Pending

---

## Phase 11: Validation and Quality Assurance (Tasks 186-200)

### T186: Invoke validation-auditor agent

- **Input**: All 9 lesson files + README
- **Output**: Validation report (Technical Correctness, Pedagogical Effectiveness, Factual Accuracy, Accessibility)
- **Status**: ⬜ Pending

### T187: Invoke factual-verifier agent

- **Input**: All research citations
- **Output**: Verification report (context window sizes, research concepts, external sources)
- **Status**: ⬜ Pending

### T188: Review validation-auditor findings

- **Action**: Categorize issues (MAJOR, MINOR, OPTIONAL)
- **Status**: ⬜ Pending

### T189: Fix MAJOR issues

- **Priority**: HIGH
- **Action**: Address all MAJOR issues immediately
- **Status**: ⬜ Pending

### T190: Fix MINOR issues

- **Priority**: MEDIUM
- **Action**: Address MINOR issues if time permits
- **Status**: ⬜ Pending

### T191: Document OPTIONAL improvements

- **Action**: Record for future iteration
- **Status**: ⬜ Pending

### T192: Re-run validation after fixes

- **Test**: All MAJOR issues resolved
- **Status**: ⬜ Pending

### T193: Constitutional compliance verification

- **Test**: All 7 principles validated (Spec Primacy, Progressive Complexity, Factual Accuracy, Coherent Structure, Intelligence Accumulation, Anti-Convergence, Minimal Content)
- **Status**: ⬜ Pending

### T194: Acceptance test execution

- **Test**: Run all tests from spec.md Section 8 (Acceptance Tests)
- **Status**: ⬜ Pending

### T195: Test-001: NO code in Lessons 1-8

- **Status**: ⬜ Pending

### T196: Test-002: All examples use Markdown + prompts

- **Status**: ⬜ Pending

### T197: Test-003: Three Roles in Lessons 3-5

- **Status**: ⬜ Pending

### T198: Test-004: Capstone specification-only

- **Status**: ⬜ Pending

### T199: Test-005: External research integrated

- **Status**: ⬜ Pending

### T200: Test-006: B1 cognitive load limits

- **Status**: ⬜ Pending

---

## Phase 12: Meta-Learning and Documentation (Tasks 201-210)

### T201: Create PHR (Prompt History Record)

- **Stage**: plan
- **Feature**: 002-011-chapter-11-redesign-fix
- **Title**: "Chapter 11 Redesign Plan"
- **Status**: ⬜ Pending

### T202: Fill PHR metadata

- **Content**: ID, title, stage, date, feature, branch, user, labels
- **Status**: ⬜ Pending

### T203: Fill PHR prompt text

- **Content**: Full user request (verbatim)
- **Status**: ⬜ Pending

### T204: Fill PHR response text

- **Content**: Concise summary of plan generated
- **Status**: ⬜ Pending

### T205: Fill PHR outcome section

- **Content**: Impact, tests, files, next prompts, reflection
- **Status**: ⬜ Pending

### T206: Fill PHR evaluation notes

- **Content**: Failure modes, grader results, prompt variant, next experiment
- **Status**: ⬜ Pending

### T207: Validate PHR completeness

- **Test**: No placeholders, all sections filled
- **Status**: ⬜ Pending

### T208: Create implementation summary

- **Content**: Total lines written, lessons created, skills created, validation passed
- **Status**: ⬜ Pending

### T209: Document lessons learned

- **Content**: What worked (research integration), what didn't (initial Layer 4 mistake), prevention (context-first protocol)
- **Status**: ⬜ Pending

### T210: Final checkpoint: Chapter 11 redesign complete

- **Test**: README + 9 lessons + 2 skills + validation passed + PHR created
- **Status**: ⬜ Pending

---

## Summary Statistics

- **Total Tasks**: 210
- **Phase 0 (Setup)**: 10 tasks
- **Phase 1-9 (9 Lessons)**: 160 tasks (average 18 per lesson)
- **Phase 10 (README)**: 15 tasks
- **Phase 11 (Validation)**: 15 tasks
- **Phase 12 (Documentation)**: 10 tasks

**Estimated Implementation Time**: 4-5 sessions

- Session 1: Setup + Lessons 1-2 (Tasks 001-040)
- Session 2: Lessons 3-5 (Tasks 041-096)
- Session 3: Lessons 6-7 + Skills (Tasks 097-133)
- Session 4: Lessons 8-9 (Tasks 134-170)
- Session 5: README + Validation + PHR (Tasks 171-210)

**Critical Path Dependencies**:

- Setup (T001-T010) → All lessons
- Lesson 1 → Lesson 2 (manual foundation sequence)
- Lessons 1-2 → Lesson 3 (manual before AI)
- Lesson 3 → Lessons 4-5 (progressive loading foundation)
- Lessons 3-5 → Lesson 6 (what to persist)
- Lessons 1-7 → Lesson 8 (integrated debugging)
- Lessons 1-8 → Lesson 9 (capstone orchestration)
- All lessons → Validation (T186-T200)

**Quality Gates**:

- After T025: Lesson 1 complete (manual foundation)
- After T096: Lessons 1-5 complete (Three Roles demonstrated)
- After T133: Skills created (Layer 3 intelligence)
- After T170: All lessons complete
- After T200: All acceptance tests pass
- After T210: Chapter ready for publication

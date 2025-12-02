---
id: "0003"
title: "platform-commands-update-v3"
stage: constitution
date: 2025-11-28
surface: claude-code
model: claude-opus-4-5-20251101
feature: none
branch: main
user: mjs
command: manual
labels: [constitution, commands, platform, orchestration]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - path: .claude/commands/sp.orchestrate.md
    action: created
    reason: Replaced sp.loopflow.v2.md with platform-aware orchestrator
  - path: .claude/commands/sp.plan.md
    action: replaced
    reason: Added context-aware planner routing
  - path: .claude/commands/sp.implement.md
    action: replaced
    reason: Added context-aware implementer routing
  - path: .claude/commands/sp.loopflow.v2.md
    action: deleted
    reason: Replaced by sp.orchestrate.md
tests:
  - null
---

## Prompt

Now review 1. .claude/commands/sp.loopflow.v2.md so it's now sp.orchestrate and do what it does now but not just for book - other engineering features as well - it;s our platform. 2. .claude/commands/sp.plan.md so this now can invoke chapter planner or engineering planner or general planning subagent depeinging on the context. and 3. .claude/commands/sp.implement.md now invoke content implementor for authoring tasks and other relevant agents as needed

## Response snapshot

Updated three slash commands for platform-wide orchestration:

### 1. `/sp.orchestrate` (NEW - replaces `/sp.loopflow.v2`)

**v3.0 Platform Transformation:**
- FROM: Book-focused content orchestration only
- TO: Platform orchestration for content, engineering, and infrastructure

**Key Features:**
- Context analysis in Phase 0 (stakeholder, work type, hardware tier)
- Routing matrix for different work types:
  - CONTENT → chapter-planner + content-implementer + educational-validator
  - ENGINEERING → general-purpose + test suite
  - PLATFORM → rag-builder/scaffolder + integration tests
  - INTELLIGENCE → direct skill/agent creation
- Hardware tier awareness (Tier 1-4)
- Cross-book intelligence assessment

### 2. `/sp.plan` (UPDATED)

**Context-Aware Routing:**
- CONTENT work → `chapter-planner` subagent
  - Pedagogical arc (Foundation → Mastery)
  - Layer progression (L1 → L4)
  - Hardware tier requirements per lesson
- ENGINEERING/PLATFORM work → `general-purpose` subagent
  - Technical architecture
  - Component decomposition
  - Test strategy

**Work-Type-Specific Artifacts:**
- Content: plan.md (lessons), research.md
- Engineering: plan.md (architecture), data-model.md, contracts/

### 3. `/sp.implement` (UPDATED)

**Context-Aware Implementation:**
- CONTENT → `content-implementer` subagent
  - 4-layer framework
  - Hardware tier gates (`<HardwareGate>`, `<CloudFallback>`)
  - Constitutional validation via `educational-validator`
- ENGINEERING → `general-purpose` subagent
  - TDD approach
  - Test validation
- PLATFORM → specialized agents
  - RAG → rag-builder
  - Scaffolding → scaffolder
  - Auth/deployment → general-purpose

**Validation by Work Type:**
- Content: educational-validator (constitutional compliance)
- Engineering: test suite (unit, integration, coverage)
- Platform: integration tests, deployment validation

## Outcome

- Impact: MAJOR - Platform-wide orchestration replacing book-only workflow
- Tests: N/A (orchestration commands)
- Files: 3 files modified/created, 1 file deleted
- Next prompts: Test commands with actual content and engineering tasks
- Reflection: Commands now serve all three stakeholders (Students, Authors, Institutions) with appropriate routing

## Evaluation notes (flywheel)

- Failure modes observed: None - straightforward update
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Use /sp.orchestrate for Module 1 content AND RAG implementation to validate routing works correctly

# Tasks: Lesson 09 — Compiling MCP to Skills

**Input**: Design documents from `/specs/045-lesson-09-compiling-mcp-skills/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Target File**: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/09-compiling-mcp-to-skills.md`

**Organization**: Tasks grouped by user story (lesson sections map to user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different sections, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US4 from spec)

## Path Conventions

- **Lesson file**: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/09-compiling-mcp-to-skills.md`
- **Spec reference**: `specs/045-lesson-09-compiling-mcp-skills/spec.md`
- **Plan reference**: `specs/045-lesson-09-compiling-mcp-skills/plan.md`
- **Canonical skill format**: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/06-agent-skills.md`
- **MCP lesson reference**: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/08-mcp-integration.md`

---

## Phase 1: Setup

**Purpose**: Prepare lesson file and verify cross-references

- [x] T001 Read existing placeholder at `09-compiling-mcp-to-skills.md` to understand current frontmatter structure
- [x] T002 [P] Read `06-agent-skills.md` to extract canonical SKILL.md format (directory structure, YAML frontmatter, sections)
- [x] T003 [P] Read `08-mcp-integration.md` to extract exact MCP commands students have used (Playwright, Context7)
- [x] T004 Verify frontmatter in placeholder matches spec requirements (lesson: 9, sidebar_position: 9, duration: 25)

**Checkpoint**: Cross-references verified, ready to write content

---

## Phase 2: Foundational Content

**Purpose**: Opening sections that frame the problem and solution (US1 foundation)

- [x] T005 [US1] Write Section 1: "The Problem — MCP Token Bloat" (~3 min read) including:
  - Sentry example (8,000 tokens at startup) with Armin Ronacher quote
  - Compounding problem explanation (multiple MCPs × data volume)
  - "You're paying for tools you're not using" insight
  - Source citations (Anthropic blog, Armin Ronacher, SmartScope)

- [x] T006 [US1] Write Section 2: "The Solution — Code Execution Pattern" (~3 min read) including:
  - Code execution pattern explanation (write code locally vs call MCP directly)
  - Four-step workflow diagram: Introspect → Compile → Generate → Optimize
  - Token reduction claim (80-98%, 150,000 → 2,000 tokens) with source
  - Skills as "lazy loading" for expertise concept

**Checkpoint**: Problem and solution framed, student understands WHY

---

## Phase 3: User Story 2 — Introspection Exercise (Priority: P2)

**Goal**: Student can introspect an MCP server and identify its tools

**Independent Test**: Student has artifact showing introspected tools with token estimates

- [x] T007 [US2] Write Section 3: "Hands-On: Introspect Your MCP Server" (~5 min exercise) including:
  - MCP choice guidance (Context7 for beginner, Playwright for advanced)
  - Exact introspection prompt from plan.md Section VI
  - Expected output format (tool name, description, parameters, token estimate)
  - Scope selection guidance (choose 3-5 tools to compile)

- [x] T008 [US2] Add troubleshooting table for introspection issues:
  - "MCP not found" → verify with `claude mcp list`
  - "Output truncated" → "Continue with remaining tools"
  - "Token estimates seem high" → normal, justifies compilation

- [x] T009 [US2] Embed Three Roles invisibly (NO role labels):
  - Claude shows tools and costs (AI teaches)
  - Student identifies needed tools (student teaches context)
  - Convergence on scope

**Checkpoint**: Section 3 complete, student can introspect MCP

---

## Phase 4: User Story 3 — Compilation Exercise (Priority: P1)

**Goal**: Student creates working compiled skill using skill-creator

**Independent Test**: File exists at `.claude/skills/[name]/SKILL.md`, Claude discovers it

- [x] T010 [US3] Write Section 4: "Hands-On: Compile to Skill" (~10 min exercise) including:
  - skill-creator invocation prompt from plan.md Section VI
  - Step-by-step compilation workflow (6 steps from plan)
  - Expected artifacts diagram showing directory structure
  - Iteration loop guidance (generate → test → refine → converge)

- [x] T011 [US3] Add canonical format verification checklist:
  - [x] File location: `.claude/skills/[skill-name]/SKILL.md`
  - [x] YAML frontmatter: name, description, version
  - [x] Description uses formula: Action + Input + Output + Triggers
  - [x] Sections: "When to Use", "Procedure", "Output Format"

- [x] T012 [US3] Add troubleshooting table for compilation issues:
  - "skill-creator not available" → go back to Lesson 04
  - "Script syntax errors" → use compatible language
  - "Skill not discovered" → check file path

- [x] T013 [P] [US3] Verify SKILL.md format matches Lesson 06 canonical source exactly. **Cross-ref**: Compare against `06-agent-skills.md` lines 84-120.

- [x] T014 [US3] Embed Three Roles invisibly in iteration loop:
  - Claude generates initial skill
  - Student tests with actual workflow
  - Claude refines based on feedback
  - Convergence on optimized skill

**Checkpoint**: Section 4 complete, student can compile MCP to skill

---

## Phase 5: User Story 4 — Validation Exercise (Priority: P2)

**Goal**: Student measures token reduction and documents results

**Independent Test**: Comparison table showing ≥30% token reduction

- [x] T015 [US4] Write Section 5: "Hands-On: Validate Token Reduction" (~5 min exercise) including:
  - Baseline measurement prompt (use MCP directly)
  - Optimized measurement prompt (use compiled skill)
  - Calculation formula: (Baseline - Optimized) / Baseline × 100%
  - Documentation template from plan.md Section VI

- [x] T016 [US4] Add reflection questions:
  - "Was the compilation effort worth the token savings?"
  - "What would make compilation NOT worth it?"
  - "How did your understanding change through iteration?"

**Checkpoint**: Section 5 complete, student has measured results

---

## Phase 6: Decision Framework

**Purpose**: Enable students to decide when to compile vs use direct MCP (SC-005)

- [x] T017 Write Section 6: "Decision Framework: When to Compile" (~2 min read) including:
  - Decision matrix table from plan.md Section III (7 scenarios)
  - Decision flowchart (verbal from plan)
  - "Compile when..." criteria summary

**Checkpoint**: Decision framework complete

---

## Phase 7: Try With AI

**Purpose**: Extension prompts for continued learning

- [x] T018 Write Section 7: "Try With AI" including 5 exploration prompts:
  1. Extend to second MCP server
  2. Optimize existing skill further
  3. Create skill suite for workflow (bridges to Lesson 10)
  4. Apply decision framework to MCP inventory
  5. Plan MCP evolution strategy

**Policy**: Section must be titled exactly "Try With AI" (no "Key Takeaways" or "What's Next"). Students use their preferred AI companion tool.

**Checkpoint**: Extension prompts complete

---

## Phase 8: Polish & Cross-Cutting

**Purpose**: Final validation and quality checks

- [x] T019 Add source citations section at end with links:
  - [Anthropic: Code Execution with MCP](https://www.anthropic.com/engineering/code-execution-with-mcp)
  - [Armin Ronacher: Skills vs Dynamic MCP Loadouts](https://lucumr.pocoo.org/2025/12/13/skills-vs-mcp/)
  - [SmartScope: MCP Code Execution Deep Dive](https://smartscope.blog/en/blog/mcp-code-execution-agent-design/)

- [x] T020 Run Three Roles invisibility check: `grep -i "What to notice\|AI.*teach\|AI.*learn\|AI as\|AI now knows" 09-compiling-mcp-to-skills.md` must return NO matches

- [x] T021 Verify all MCP commands match Lesson 08:
  - `claude mcp add --transport stdio playwright npx @playwright/mcp@latest`
  - `claude mcp add --transport stdio context7 npx @upstash/context7-mcp`

- [x] T022 Update frontmatter metadata:
  - generated_by: "content-implementer v1.0.0 (045-lesson-09-compiling-mcp-skills)"
  - source_spec: "specs/045-lesson-09-compiling-mcp-skills/spec.md"
  - version: "1.0.0"

- [x] T023 Final read-through for cognitive load (5 new concepts max) and flow

**Checkpoint**: Lesson complete, all validations pass

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - read references first
- **Phase 2 (Foundational)**: Depends on Phase 1 - problem/solution framing
- **Phase 3-5 (User Stories)**: Depend on Phase 2 - can proceed sequentially
- **Phase 6-7 (Framework + Try With AI)**: Depend on Phases 3-5
- **Phase 8 (Polish)**: Depends on all content complete

### User Story Dependencies

- **US1 (Problem Understanding)**: Addressed in Phase 2 (foundation)
- **US2 (Introspection)**: Phase 3 - depends on US1 conceptual foundation
- **US3 (Compilation)**: Phase 4 - depends on US2 introspection output
- **US4 (Validation)**: Phase 5 - depends on US3 compiled skill

### Parallel Opportunities

Within Phase 1:
```bash
# These can run in parallel:
Task T002: Read 06-agent-skills.md for canonical format
Task T003: Read 08-mcp-integration.md for MCP commands
```

Within Phase 4:
```bash
# T013 can run parallel with other US3 tasks:
Task T013: Verify SKILL.md format matches Lesson 06
```

---

## Implementation Strategy

### MVP First (Sections 1-4)

1. Complete Phase 1: Setup and cross-references
2. Complete Phase 2: Problem and Solution sections
3. Complete Phase 4: Compilation exercise (core learning outcome)
4. **STOP and VALIDATE**: Can student compile MCP to skill?
5. If yes, continue with remaining phases

### Incremental Delivery

1. Phase 1-2 → Foundation ready
2. + Phase 3 → Introspection works
3. + Phase 4 → Compilation works (MVP complete!)
4. + Phase 5 → Validation works
5. + Phase 6-7 → Decision framework and extensions
6. + Phase 8 → Polish complete

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 23 |
| **Phase 1 (Setup)** | 4 |
| **Phase 2 (Foundation)** | 2 |
| **Phase 3 (US2 Introspection)** | 3 |
| **Phase 4 (US3 Compilation)** | 5 |
| **Phase 5 (US4 Validation)** | 2 |
| **Phase 6 (Decision Framework)** | 1 |
| **Phase 7 (Try With AI)** | 1 |
| **Phase 8 (Polish)** | 5 |
| **Parallel Opportunities** | 4 tasks marked [P] |
| **MVP Scope** | T001-T014 (Phases 1-4) |

---

## Notes

- All content tasks target single file: `09-compiling-mcp-to-skills.md`
- Cross-reference verification critical to prevent format drift
- Three Roles must stay invisible (grep validation in T020)
- Canonical skill format from Lesson 06 is authoritative
- MCP commands from Lesson 08 must match exactly

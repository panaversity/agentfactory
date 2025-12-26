# Feature Specification: Chapter 39 - Agent Skills & MCP Code Execution

**Feature Branch**: `047-chapter-39-skills-mcp`
**Created**: 2025-12-26
**Status**: Draft
**Proficiency Level**: B2 (from Part 6 positioning)

## The Goal

**Build Skills that Implement the Code Execution Pattern**

Students learn to create skills that don't just give advice—they EXECUTE. The skill orchestrates code execution (MCP calls, scripts, data processing) to autonomously solve problems.

```
Skill (intelligence) → orchestrates execution → analyzes results → iterates until solved
```

## The Learning Arc

| Phase | What Students Build | Example |
|-------|---------------------|---------|
| **MCP First** (Lessons 3-4) | Skills that wrap MCP servers | Like `fetching-library-docs` wrapping Context7 |
| **Scripts Next** (Lessons 5-6) | Skills that write + execute scripts | Data processing, file manipulation |
| **Full Workflows** (Lesson 7) | Skills that orchestrate everything | MCP + scripts + iteration + error recovery |
| **Ship It** (Lesson 8) | Complete, shippable skill | Capstone agent capability |

**Why MCP first?** Students already know MCP from Chapters 37-38. It's familiar. Once they see the pattern with MCP, scripts become obvious.

## What This Chapter DOES NOT Cover (Already Taught)

**From Chapter 5**: Basic SKILL.md, three-level loading, simple skill creation, code execution concept
**From Chapter 37**: MCP architecture, transports, client configuration
**From Chapter 38**: Building MCP servers (tools, resources, prompts)

**This chapter**: Building skills that USE those MCP servers intelligently + extend to script execution.

## Assumed Knowledge

- SKILL.md structure and YAML frontmatter (Chapter 5, Lesson 6)
- MCP fundamentals and client configuration (Chapter 37)
- Building MCP servers (Chapter 38)
- Python fundamentals (Part 5)

## User Scenarios & Testing

### User Story 1 - Build Skill that Wraps MCP (Priority: P1)

A developer wants to create a skill like `fetching-library-docs` that wraps an MCP server (e.g., Context7) and adds intelligence: when to call it, how to filter results, how to handle errors.

**Why this priority**: MCP is familiar from prior chapters. This is the first concrete example of the code execution pattern.

**Acceptance Scenarios**:

1. **Given** an existing MCP server, **When** the developer creates a wrapping skill, **Then** the skill triggers on relevant prompts and orchestrates MCP calls effectively
2. **Given** MCP returns large results, **When** the skill processes them, **Then** it filters to only relevant data (token efficiency)

---

### User Story 2 - Build Skill that Executes Scripts (Priority: P1)

A developer wants to create a skill that writes Python/Bash scripts, executes them, analyzes results, and iterates. This extends beyond MCP to general code execution.

**Why this priority**: Once MCP pattern is understood, scripts are the natural generalization. This is the full code execution pattern.

**Acceptance Scenarios**:

1. **Given** a data processing task, **When** the skill is activated, **Then** it writes code, executes it, and returns processed results
2. **Given** script execution fails, **When** the skill receives the error, **Then** it diagnoses and retries with corrected code

---

### User Story 3 - Build Skill with Write-Execute-Analyze Loop (Priority: P2)

A developer wants their skill to implement the full iterative pattern: write code → execute → analyze results → refine → repeat until solved.

**Why this priority**: The loop is what makes agents truly autonomous. Single-shot execution isn't enough.

**Acceptance Scenarios**:

1. **Given** a complex analysis task, **When** the skill runs, **Then** it iterates at least 3 times refining its approach
2. **Given** partial results, **When** analyzed by the skill, **Then** it identifies gaps and generates additional code to fill them

---

### User Story 4 - Build Skill with Proper Error Recovery (Priority: P2)

A developer wants their skill to handle failures gracefully: syntax errors, runtime errors, timeouts, missing data.

**Why this priority**: Production skills must be robust. Errors are inevitable.

**Acceptance Scenarios**:

1. **Given** a syntax error in generated code, **When** execution fails, **Then** the skill parses the error and fixes the code
2. **Given** a timeout, **When** execution is interrupted, **Then** the skill logs state and can resume

---

### User Story 5 - Ship Complete Code-Execution Skill (Priority: P3)

A student completes the capstone by building a full skill that: wraps relevant MCP/scripts, implements execution loops, handles errors, and can be shared/sold.

**Why this priority**: Capstone proves mastery. Students leave with a shippable asset.

**Acceptance Scenarios**:

1. **Given** a domain specification, **When** the skill is built, **Then** it autonomously solves problems in that domain
2. **Given** the completed skill, **When** packaged, **Then** it can be installed in any Claude Code project

---

### Edge Cases

- MCP server unavailable or returns errors
- Script execution exceeds time/memory limits
- Infinite loops in write-execute-analyze cycle
- Large result sets overwhelming context
- Partial execution state when interrupted

## Requirements

### Functional Requirements

**Advanced Skill Patterns (Lessons 1-2)** — *Foundation for execution skills*:
- **FR-001**: Lessons MUST teach persona patterns for execution skills (identity, expertise domain, behavioral constraints)
- **FR-002**: Lessons MUST explain Questions + Principles framework for encoding decision logic
- **FR-003**: Lessons MUST demonstrate skill composition and dependencies
- **FR-004**: Lessons MUST provide hands-on exercise creating a domain skill with persona and principles

**Skills that Wrap MCP (Lessons 3-4)** — *First example of code execution pattern*:
- **FR-005**: Lessons MUST analyze existing skills like `fetching-library-docs` and `browsing-with-playwright`
- **FR-006**: Lessons MUST teach when/how to wrap an MCP server in a skill
- **FR-007**: Lessons MUST demonstrate result filtering for token efficiency
- **FR-008**: Lessons MUST include exercise: build a skill wrapping an MCP server from Chapter 38

**Skills that Execute Scripts (Lessons 5-6)** — *Generalize beyond MCP*:
- **FR-009**: Lessons MUST teach the write-execute-analyze loop pattern
- **FR-010**: Lessons MUST cover script generation (Python, Bash)
- **FR-011**: Lessons MUST demonstrate error recovery (syntax vs runtime errors)
- **FR-012**: Lessons MUST address safety: what scripts should/shouldn't do
- **FR-013**: Lessons MUST include exercise: build a data processing skill

**Full Workflow Orchestration (Lesson 7)** — *Combine everything*:
- **FR-014**: Lessons MUST demonstrate skills that use MCP + scripts together
- **FR-015**: Lessons MUST teach iteration and convergence criteria
- **FR-016**: Lessons MUST address production concerns (logging, state, recovery)

**Capstone (Lesson 8)** — *Ship it*:
- **FR-017**: Capstone MUST be spec-driven (L4 layer)
- **FR-018**: Capstone skill MUST implement full code execution pattern
- **FR-019**: Capstone MUST be packageable and shippable

### Key Entities

- **Execution Skill**: A skill that orchestrates code execution (MCP calls, scripts, data processing)
- **MCP Wrapper Skill**: Skill that adds intelligence layer on top of an MCP server
- **Script Execution Skill**: Skill that writes and executes code autonomously
- **Write-Execute-Analyze Loop**: Iterative pattern for autonomous problem-solving

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students can analyze an existing MCP-wrapping skill (like `fetching-library-docs`) and explain its pattern
- **SC-002**: Students can build a skill that wraps an MCP server with proper triggering and result filtering
- **SC-003**: Students can build a skill that writes, executes, and iterates on Python scripts
- **SC-004**: Students can implement error recovery in their execution skills
- **SC-005**: Students complete capstone: a shippable skill implementing the full code execution pattern
- **SC-006**: 90% of exercises completed using AI collaboration
- **SC-007**: Chapter maintains B2 proficiency level

## Lesson Structure

| Lesson | Title | Layer | What Students BUILD |
|--------|-------|-------|---------------------|
| 1 | Advanced Skill Patterns | L1 (Manual) | Understand persona/principles for execution skills |
| 2 | Skill Composition | L2 (Collaboration) | Create skill with dependencies and references |
| 3 | Anatomy of MCP-Wrapping Skills | L1 (Manual) | Analyze `fetching-library-docs`, `browsing-with-playwright` |
| 4 | Build Your MCP-Wrapping Skill | L2 (Collaboration) | Wrap an MCP server from Chapter 38 |
| 5 | Script Execution Fundamentals | L1 (Manual) | Understand write-execute-analyze pattern |
| 6 | Build Script-Execution Skill | L2 (Collaboration) | Data processing skill with iteration |
| 7 | Full Workflow Orchestration | L3 (Intelligence) | Skill combining MCP + scripts + error recovery |
| 8 | Capstone: Shippable Agent Skill | L4 (Spec-Driven) | Complete, packageable execution skill |

## Dependencies

- Chapter 5: Skills fundamentals
- Chapter 37: MCP fundamentals
- Chapter 38: Building MCP servers (students have servers to wrap)
- Part 5: Python fundamentals

## Out of Scope

- Enterprise scaling (future chapter)
- Skill marketplace/monetization (Part 7)
- Multi-language execution beyond Python/Bash (mention as extension)

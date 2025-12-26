# Feature Specification: Chapter 36 - Claude Agent SDK: Building Digital FTEs

**Feature Branch**: `001-claude-agent-sdk`
**Created**: 2025-12-26
**Status**: Draft
**Input**: User description: "Chapter 36: Claude Agent SDK - Building Digital FTEs"

## Chapter Context

**Part**: 6 - AI Native Software Development
**Chapter Number**: 36
**Directory**: `apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/`
**Proficiency Level**: B1-B2 (Intermediate to Upper-Intermediate)

### Running Example

**TaskManager Agent** - A task management agent built progressively through the chapter, consistent with other SDK chapters (34, 35) in Part 6. This agent demonstrates:
- Creating and managing tasks
- Persisting state across sessions with file checkpointing
- Integrating with external tools via MCP
- Using Skills for domain expertise
- Handling user permissions with canUseTool
- Cost tracking for production billing

### Why Claude Agent SDK is the Top Contender

The Claude Agent SDK provides capabilities NO other SDK offers:

| Feature | Claude SDK | OpenAI SDK | Google ADK |
|---------|------------|------------|------------|
| Battle-tested tools (Read, Edit, Bash, etc.) | ✅ Same as Claude Code | New | New |
| Skills ecosystem (filesystem-based SKILL.md) | ✅ Native | ❌ | ❌ |
| Custom Slash Commands (/command) | ✅ .claude/commands/*.md | ❌ | ❌ |
| File Checkpointing (rewindFiles) | ✅ Undo any file change | ❌ | ❌ |
| 8+ Hook Events (PreToolUse, PostToolUse, etc.) | ✅ Full lifecycle | Partial | Partial |
| Session Forking (branch conversations) | ✅ | ❌ | ❌ |
| Native MCP Integration | ✅ First-class | ❌ | ❌ |
| System Prompt Presets (inherit Claude Code) | ✅ | ❌ | ❌ |
| CLAUDE.md Project Context | ✅ Automatic loading | ❌ | ❌ |
| Per-Message Cost Tracking | ✅ total_cost_usd | Per-request | Per-request |
| Bash Sandbox | ✅ Programmatic | ❌ | ❌ |
| Structured JSON Output | ✅ outputFormat | ✅ | ✅ |
| canUseTool Runtime Permissions | ✅ Dynamic per-call | ❌ | ❌ |
| Context Compaction (/compact) | ✅ Long-running agents | ❌ | ❌ |

### Differentiation from Chapter 5

| Chapter 5 (Claude Code Features) | Chapter 36 (Claude Agent SDK) |
|----------------------------------|-------------------------------|
| Using Claude Code as end-user tool | Programmatic SDK for building Custom Agents |
| CLI interaction patterns | Python/TypeScript API integration |
| Skills/Subagents/Hooks as user | Skills/Subagents/Hooks as developer |
| Configuration files (settings.json) | Code-driven configuration (ClaudeAgentOptions) |
| Interactive permission prompts | Programmatic canUseTool callbacks |

## Assumed Knowledge

**What students know BEFORE this chapter**:
- Python fundamentals (Parts 5: Chapters 15-32) - async/await, decorators, dataclasses
- Claude Code CLI usage (Chapter 5) - tools, permissions, MCP concepts, Skills, Subagents
- MCP fundamentals (Chapter 37 - may be taught concurrently)
- Agent architecture concepts (Chapter 33)
- OpenAI Agents SDK patterns (Chapter 34) - for comparison

**What this chapter must explain from scratch**:
- Claude Agent SDK architecture (query function, ClaudeSDKClient)
- How SDK differs from Claude API client library
- SDK-specific patterns: hooks, permission modes, session management
- Custom tool creation with @tool decorator
- Subagent orchestration in code
- **UNIQUE FEATURES** (not in other SDKs):
  - Agent Skills via `settingSources`
  - Custom Slash Commands via filesystem
  - File Checkpointing and rewindFiles()
  - Cost tracking with total_cost_usd
  - System prompt presets (claude_code)
  - canUseTool for runtime permission decisions
  - Sandbox configuration for Bash
  - Context compaction for long-running agents

## User Scenarios & Testing *(mandatory)*

### User Story 1 - First Agent with Query Function (Priority: P1)

A domain expert completes Chapter 36 Lesson 1-2 and understands what the Claude Agent SDK is, how it differs from the Claude API, and can write a basic agent using the `query()` function.

**Why this priority**: Foundation for all subsequent learning. Without understanding query() and ClaudeAgentOptions, students cannot proceed.

**Independent Test**: Student can explain the SDK architecture and write a conceptual agent script using query() with appropriate options.

**Acceptance Scenarios**:

1. **Given** a student who completed Python fundamentals, **When** they finish Lesson 2, **Then** they can articulate why SDK is different from API and write a basic query() call
2. **Given** a student reading code examples, **When** they see ClaudeAgentOptions, **Then** they understand allowed_tools, permission_mode, and max_turns

---

### User Story 2 - Tool Mastery and Permissions (Priority: P1)

A domain expert understands the built-in tools (Read, Edit, Write, Bash, Glob, Grep, WebSearch, WebFetch, Task) and can configure permission modes to control agent behavior.

**Why this priority**: Tool access and permissions are core to safe agent development. Critical for production use.

**Independent Test**: Student can configure a read-only research agent vs a full-access coding agent using appropriate tool restrictions.

**Acceptance Scenarios**:

1. **Given** a student learning tools, **When** they review tool capabilities, **Then** they can select appropriate tools for their use case
2. **Given** security requirements, **When** student configures permissions, **Then** they understand default/acceptEdits/bypassPermissions tradeoffs

---

### User Story 3 - Agent Skills in Code (Priority: P1) ★ UNIQUE

A domain expert can load and use filesystem-based Agent Skills (SKILL.md files) programmatically via `settingSources` to give their agent domain expertise.

**Why this priority**: Skills are THE key differentiator for Claude Agent SDK - no other SDK has this. Essential for Digital FTEs with domain expertise.

**Independent Test**: Student can configure an agent to load Skills from `.claude/skills/` and use them.

**Acceptance Scenarios**:

1. **Given** existing Skills in .claude/skills/, **When** student configures settingSources: ['project'], **Then** agent has access to all project skills
2. **Given** a domain-specific skill, **When** agent encounters relevant task, **Then** skill triggers automatically

---

### User Story 4 - Session Management and Checkpointing (Priority: P2) ★ UNIQUE

A domain expert can implement session persistence, resumption, forking, AND file checkpointing to maintain state and undo file changes.

**Why this priority**: File checkpointing is UNIQUE to Claude SDK - enables "undo" for agents.

**Independent Test**: Student can capture checkpoint UUID, make file changes, then rewindFiles() to restore.

**Acceptance Scenarios**:

1. **Given** a multi-turn task, **When** student implements session capture, **Then** agent can resume from previous state
2. **Given** file checkpointing enabled, **When** agent makes bad file changes, **Then** student can rewindFiles() to restore
3. **Given** experimental branches needed, **When** student forks a session, **Then** original session remains unchanged

---

### User Story 5 - Subagent Orchestration (Priority: P2)

A domain expert can define and invoke subagents programmatically using AgentDefinition to parallelize work and isolate context.

**Why this priority**: Subagents are key to complex agent architectures and the Task tool pattern from Chapter 5.

**Independent Test**: Student can define a code-reviewer subagent and invoke it from main agent.

**Acceptance Scenarios**:

1. **Given** a complex task, **When** student creates specialized subagents, **Then** each subagent has appropriate tool restrictions
2. **Given** parallel work needs, **When** subagents execute, **Then** they return synthesized results to orchestrator

---

### User Story 6 - Lifecycle Hooks (Priority: P2)

A domain expert can implement ALL lifecycle hooks (PreToolUse, PostToolUse, UserPromptSubmit, Stop, SubagentStop, PreCompact, PermissionRequest) for complete control.

**Why this priority**: Hooks provide production-ready control without modifying agent logic. More events than any other SDK.

**Independent Test**: Student can implement hooks for security, logging, and permission handling.

**Acceptance Scenarios**:

1. **Given** security requirements, **When** PreToolUse hook fires, **Then** dangerous commands are blocked
2. **Given** audit needs, **When** PostToolUse hook fires, **Then** all tool usage is logged
3. **Given** long-running session, **When** PreCompact fires, **Then** student can archive transcript before compaction

---

### User Story 7 - Custom Slash Commands (Priority: P2) ★ UNIQUE

A domain expert can create custom slash commands via filesystem (.claude/commands/*.md) that are available in their SDK agents.

**Why this priority**: Slash commands are UNIQUE to Claude SDK - enable reusable agent behaviors.

**Independent Test**: Student can create a /review command and use it programmatically.

**Acceptance Scenarios**:

1. **Given** .claude/commands/review.md exists, **When** agent is configured with settingSources, **Then** /review command is available
2. **Given** command with arguments, **When** student sends "/review src/auth.py", **Then** command executes with argument

---

### User Story 8 - Custom MCP Tools (Priority: P2)

A domain expert can create custom tools using @tool decorator and create_sdk_mcp_server for domain-specific functionality.

**Why this priority**: Enables extending agent capabilities beyond built-in tools - key for Digital FTEs.

**Independent Test**: Student can create a weather lookup tool and integrate it with their agent.

**Acceptance Scenarios**:

1. **Given** domain expertise, **When** student creates custom tool, **Then** tool is available in agent's toolset
2. **Given** external API integration, **When** tool is invoked, **Then** results are returned to agent

---

### User Story 9 - Runtime Permissions with canUseTool (Priority: P3) ★ UNIQUE

A domain expert can implement dynamic permission decisions using canUseTool callback that evaluates each tool call at runtime.

**Why this priority**: canUseTool is UNIQUE - enables context-aware security decisions.

**Independent Test**: Student can implement a canUseTool that allows Write only to specific directories.

**Acceptance Scenarios**:

1. **Given** canUseTool callback, **When** Write tool targets /sandbox/, **Then** allow
2. **Given** canUseTool callback, **When** Write tool targets /config/, **Then** deny with message
3. **Given** canUseTool with updatedInput, **When** tool executes, **Then** input is modified

---

### User Story 10 - Cost Tracking (Priority: P3) ★ UNIQUE

A domain expert can track per-message token usage and costs for billing using the SDK's cost tracking features.

**Why this priority**: Production Digital FTEs need cost tracking. SDK provides total_cost_usd.

**Independent Test**: Student can implement a CostTracker that bills users per session.

**Acceptance Scenarios**:

1. **Given** assistant messages, **When** usage data arrives, **Then** student can track tokens per step
2. **Given** result message, **When** session completes, **Then** total_cost_usd is available

---

### User Story 11 - Multi-Turn Conversations with ClaudeSDKClient (Priority: P3)

A domain expert can use ClaudeSDKClient for streaming input, multi-turn conversations, images, and interrupts.

**Why this priority**: Enables richer interaction patterns for production agents.

**Independent Test**: Student can implement a conversation loop with context persistence using ClaudeSDKClient.

**Acceptance Scenarios**:

1. **Given** multi-turn conversation, **When** using ClaudeSDKClient, **Then** context is maintained across turns
2. **Given** image analysis need, **When** student uploads image via streaming input, **Then** agent processes visual content
3. **Given** runaway agent, **When** student calls interrupt(), **Then** agent stops gracefully

---

### User Story 12 - Production Patterns (Priority: P3)

A domain expert understands hosting patterns (ephemeral, long-running, hybrid), compaction for long sessions, and sandbox configuration.

**Why this priority**: Bridges learning to real-world deployment of Digital FTEs.

**Independent Test**: Student can select appropriate hosting pattern and configure sandbox for their use case.

**Acceptance Scenarios**:

1. **Given** one-off task agent, **When** designing architecture, **Then** student selects ephemeral pattern
2. **Given** long-running session, **When** context grows large, **Then** student uses /compact or PreCompact hook
3. **Given** Bash execution needs, **When** configuring sandbox, **Then** student understands network/file restrictions

---

### Edge Cases

- What happens when session ID is invalid or expired?
- How does agent handle tool execution timeout?
- What happens when MCP server connection fails?
- How to handle API rate limits gracefully?
- What happens when context window approaches limit (compaction)?
- What if file checkpointing is enabled but no backups exist?
- How to handle canUseTool returning deny mid-execution?
- What if Skill loading fails due to malformed SKILL.md?

## Requirements *(mandatory)*

### Functional Requirements

**Lesson Structure Requirements**:
- **FR-001**: Chapter MUST contain 14-16 lessons following 4-Layer Teaching Method progression
- **FR-002**: Each lesson MUST include YAML frontmatter with skills, learning objectives, and cognitive load
- **FR-003**: Lessons MUST use TaskManager Agent as running example throughout
- **FR-004**: All code examples MUST be conceptual/documented (no API key required for testing)
- **FR-005**: Each lesson MUST include 3 "Try With AI" prompts with "What you're learning" explanations

**Core SDK Content Requirements**:
- **FR-006**: Chapter MUST explain SDK architecture (query(), ClaudeSDKClient, message types)
- **FR-007**: Chapter MUST cover all built-in tools with practical usage guidance
- **FR-008**: Chapter MUST explain all permission modes with security tradeoffs
- **FR-009**: Chapter MUST demonstrate session management (resume, fork)
- **FR-010**: Chapter MUST teach subagent definition with AgentDefinition
- **FR-011**: Chapter MUST cover ALL hook events (not just PreToolUse/PostToolUse)
- **FR-012**: Chapter MUST show custom MCP tool creation with @tool decorator
- **FR-013**: Chapter MUST include streaming input mode patterns
- **FR-014**: Chapter MUST cover production hosting patterns

**UNIQUE FEATURES Content Requirements** (differentiators from other SDKs):
- **FR-015**: Chapter MUST teach Agent Skills loading via settingSources
- **FR-016**: Chapter MUST teach custom Slash Commands via .claude/commands/
- **FR-017**: Chapter MUST teach file checkpointing and rewindFiles()
- **FR-018**: Chapter MUST teach cost tracking with total_cost_usd
- **FR-019**: Chapter MUST teach canUseTool for runtime permission decisions
- **FR-020**: Chapter MUST teach system prompt presets (claude_code with append)
- **FR-021**: Chapter MUST teach sandbox configuration for Bash
- **FR-022**: Chapter MUST teach context compaction for long-running agents
- **FR-023**: Chapter MUST teach structured output with outputFormat

**Pedagogical Requirements**:
- **FR-024**: L1 lessons MUST build vocabulary and mental models before code
- **FR-025**: L2 lessons MUST demonstrate AI collaboration patterns
- **FR-026**: L3/L4 lessons MUST connect to Digital FTE monetization
- **FR-027**: Chapter MUST differentiate clearly from Chapter 5 (user vs developer perspective)
- **FR-028**: Chapter MUST compare with OpenAI/Google SDKs to highlight unique features

**Quality Requirements**:
- **FR-029**: All statistics and dates MUST be fact-checked via WebSearch
- **FR-030**: Chapter quiz MUST have 15-20 questions covering key concepts including unique features
- **FR-031**: Each lesson MUST have summary file (.summary.md)

### Key Entities

- **Agent**: Autonomous system that uses tools to accomplish tasks
- **Query**: Single execution of agent loop with prompt and options
- **Session**: Persistent conversation context that can be resumed or forked
- **Checkpoint**: File state snapshot that can be restored via rewindFiles()
- **Subagent**: Specialized agent spawned by orchestrator for focused tasks
- **Hook**: Callback function executed at specific lifecycle points
- **Skill**: Filesystem-based expertise module (SKILL.md) loaded via settingSources
- **Slash Command**: Custom command defined in .claude/commands/*.md
- **MCP Tool**: Custom tool exposed via Model Context Protocol
- **Permission Mode**: Security setting controlling tool auto-approval
- **canUseTool**: Callback for dynamic per-call permission decisions

## Lesson Structure (Proposed - 15 Lessons)

| #  | Title | Layer | Focus | Unique? |
|----|-------|-------|-------|---------|
| 1  | What is the Claude Agent SDK? | L1 | SDK vs API, unique advantages | - |
| 2  | Your First Agent with query() | L1 | Basic query(), ClaudeAgentOptions | - |
| 3  | Built-in Tools Deep Dive | L1 | All 9 tools, when to use each | - |
| 4  | Permission Modes and Security | L2 | default/acceptEdits/bypass, canUseTool | ★ canUseTool |
| 5  | Agent Skills in Code | L2 | settingSources, loading SKILL.md | ★ Skills |
| 6  | Custom Slash Commands | L2 | .claude/commands/, arguments | ★ Commands |
| 7  | Session Management | L2 | Resume, fork, session IDs | ★ Fork |
| 8  | File Checkpointing | L2 | enableFileCheckpointing, rewindFiles() | ★ Checkpointing |
| 9  | Subagents for Parallel Work | L2 | AgentDefinition, Task tool | - |
| 10 | Lifecycle Hooks | L3 | All 8 hook events, HookMatcher | - |
| 11 | Custom MCP Tools | L3 | @tool decorator, create_sdk_mcp_server | - |
| 12 | ClaudeSDKClient and Streaming | L3 | Multi-turn, images, interrupts | - |
| 13 | Cost Tracking and Billing | L3 | Per-message usage, total_cost_usd | ★ Cost |
| 14 | Production Patterns | L4 | Hosting, sandbox, compaction | ★ Sandbox |
| 15 | TaskManager: Complete Agent | L4 | Capstone integrating ALL unique features | - |
| 16 | Chapter Quiz | - | Assessment (15-20 questions) | - |

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 90% of students can explain the difference between Claude Agent SDK and Claude API after Lesson 1
- **SC-002**: 85% of students can write a valid query() call with appropriate options after Lesson 2
- **SC-003**: 80% of students can configure tool restrictions for their use case after Lesson 3
- **SC-004**: 80% of students can implement canUseTool for runtime permissions after Lesson 4
- **SC-005**: 80% of students can load Agent Skills via settingSources after Lesson 5
- **SC-006**: 75% of students can create custom slash commands after Lesson 6
- **SC-007**: 80% of students can implement session resumption and forking after Lesson 7
- **SC-008**: 75% of students can implement file checkpointing with rewindFiles() after Lesson 8
- **SC-009**: 75% of students can define a custom subagent after Lesson 9
- **SC-010**: 75% of students can implement ALL hook types after Lesson 10
- **SC-011**: 70% of students can create a custom MCP tool after Lesson 11
- **SC-012**: 70% of students can implement cost tracking after Lesson 13
- **SC-013**: 70% of students pass chapter quiz with 80%+ score
- **SC-014**: Students can articulate 5+ unique features of Claude SDK vs OpenAI/Google
- **SC-015**: Chapter enables students to architect a Digital FTE using Claude Agent SDK with all unique features

### Assumptions

- Students have Python async/await experience from Chapter 31
- Students understand MCP concepts from Chapter 37 (may be concurrent)
- Official documentation at platform.claude.com/docs/en/agent-sdk/ is accurate as of Dec 2025
- Claude Code CLI is a prerequisite runtime dependency
- No API key will be provided; examples are conceptual/documented
- TaskManager pattern is consistent with Chapters 34-35

## External References

- **Official Docs**: https://platform.claude.com/docs/en/agent-sdk/overview
- **Python Reference**: https://platform.claude.com/docs/en/agent-sdk/python
- **TypeScript Reference**: https://platform.claude.com/docs/en/agent-sdk/typescript
- **Skills**: https://platform.claude.com/docs/en/agent-sdk/skills
- **Slash Commands**: https://platform.claude.com/docs/en/agent-sdk/slash-commands
- **File Checkpointing**: https://platform.claude.com/docs/en/agent-sdk/file-checkpointing
- **Cost Tracking**: https://platform.claude.com/docs/en/agent-sdk/cost-tracking
- **Skill Created**: .claude/skills/building-with-claude-agent-sdk

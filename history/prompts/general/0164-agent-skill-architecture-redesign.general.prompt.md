---
id: 0164
title: agent-skill-architecture-redesign
stage: general
date: 2025-12-03
surface: claude-code
model: opus-4-5
feature: none
branch: main
user: mjs
command: direct
labels: [agents, skills, architecture, frontmatter]
links:
  spec: none
  ticket: none
  adr: none
  pr: none
files:
  - .claude/agents/content-implementer.md
  - .claude/agents/chapter-planner.md
  - .claude/agents/assessment-architect.md
  - .claude/agents/validation-auditor.md
  - .claude/agents/pedagogical-designer.md
  - .claude/agents/educational-validator.md
  - .claude/agents/super-orchestra.md
  - .claude/agents/factual-verifier.md
  - .claude/agents/spec-architect.md
  - .claude/skills/canonical-format-checker/SKILL.md
tests:
  - none
---

## Prompt

User identified that subagents were not connected to skills per official Claude Code documentation. Requested plan using Plan agent and papers/prompting-practices-claude.md to determine which subagents get which skills and if any skills/agents need to be added or removed.

Official Claude Code subagent format supports `skills:` frontmatter field that auto-loads skills when agent is invoked:
```yaml
---
name: your-sub-agent-name
description: Description of when this subagent should be invoked
tools: tool1, tool2, tool3
model: sonnet
permissionMode: default
skills: skill1, skill2
---
```

## Response snapshot

### Analysis Completed

Used Plan agent to create comprehensive Agent-Skill Connection Architecture based on:
1. Claude 4.x Prompting Best Practices (long-horizon reasoning, agentic workflows, default-to-action)
2. 4-Layer Teaching Method alignment
3. Workflow integration points
4. Behavioral patterns (`default_to_action`, `investigate_before_acting`)

### Changes Implemented

**9 Agents Updated with `skills:` frontmatter**:

| Agent | Skills Auto-Loaded |
|-------|-------------------|
| content-implementer | ai-collaborate-teaching, code-example-generator, technical-clarity |
| chapter-planner | book-scaffolding, learning-objectives, concept-scaffolding |
| assessment-architect | assessment-builder, quiz-generator, exercise-designer |
| validation-auditor | content-evaluation-framework, technical-clarity, summary-generator |
| pedagogical-designer | skills-proficiency-mapper, concept-scaffolding, learning-objectives |
| educational-validator | content-evaluation-framework |
| super-orchestra | book-scaffolding, learning-objectives, skills-proficiency-mapper, technical-clarity, concept-scaffolding, session-intelligence-harvester |
| factual-verifier | [] (empty - specialized agent) |
| spec-architect | [] (empty - specialized agent) |

**1 New Skill Created**:
- `canonical-format-checker` - Prevents format drift by ensuring content follows canonical sources

### Key Decisions

1. Specialized agents (factual-verifier, spec-architect) get empty `skills: []` to keep them focused
2. validation-auditor gets `summary-generator` for post-implementation summaries
3. super-orchestra gets `session-intelligence-harvester` for systematic harvesting
4. Max 3-6 skills per agent to avoid overload

## Outcome

- Impact: All 9 agents now support skill auto-loading per official Claude Code format
- Tests: none (frontmatter changes)
- Files: 10 files modified/created
- Next prompts: Test agent invocations to verify skill auto-loading works
- Reflection: Agent-skill connections enable more efficient workflows by having relevant skills ready when agents are invoked

## Evaluation notes (flywheel)

- Failure modes observed: None - agents previously had no skill connections
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Test content-implementer invocation to verify ai-collaborate-teaching skill loads

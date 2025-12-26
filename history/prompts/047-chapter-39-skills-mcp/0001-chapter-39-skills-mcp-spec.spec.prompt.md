---
id: 0001
title: chapter-39-skills-mcp-spec
stage: spec
date: 2025-12-26
surface: claude-code
model: claude-opus-4-5-20251101
feature: 047-chapter-39-skills-mcp
branch: 047-chapter-39-skills-mcp
user: mjs
command: /sp.specify
labels: [chapter, educational-content, skills, mcp, code-execution]
links:
  spec: specs/047-chapter-39-skills-mcp/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/047-chapter-39-skills-mcp/spec.md
  - specs/047-chapter-39-skills-mcp/checklists/requirements.md
tests:
  - null
---

## Prompt

/sp.specify Chapter 39: Agent Skills & MCP Code Execution

## Context
This chapter is in Part 6 (AI-Native Software Development). Students have completed:
- Chapter 5: Claude Code Skills fundamentals (SKILL.md, MCP integration basics as USER)
- Chapter 38: Custom MCP Servers (building MCP servers)

This chapter bridges Skills and MCP to enable BUILDER capability - taking students from using skills/MCP to creating and shipping them.

## Official Sources
- AgentSkills.io: https://agentskills.io/integrate-skills (open format for AI agent skills)
- MCP Official: https://modelcontextprotocol.io/docs/getting-started/intro
- E2B MCP for code execution: https://e2b.dev/docs/mcp
- Anthropic code execution pattern: https://www.anthropic.com/engineering/code-execution-with-mcp

## Chapter Structure from README
8 lessons covering:
1. Skills Architecture Deep Dive
2. Skill Composition Patterns
3. Code Execution Fundamentals
4. MCP Code Execution Servers
5. Execution Workflow Patterns
6. Safety & Resource Management
7. Skills + Execution Integration
8. Capstone: Data Analysis Agent

## Response snapshot

Created comprehensive specification for Chapter 39 with:
- 7 prioritized user stories (P1-P3) covering skill creation, code execution, composition, safety, and integration
- 22 functional requirements organized by lesson topic
- 8 measurable success criteria
- Lesson structure with pedagogical layer progression (L1→L4)
- Assumed knowledge section mapping prerequisites
- Edge cases and out-of-scope items documented

Key decisions:
- B2 proficiency level (Part 6 positioning)
- E2B as primary code execution sandbox
- Skills for strategy, code execution for tactics pattern
- Capstone builds spec-driven data analysis agent

## Outcome

- ✅ Impact: Full chapter specification created with clear requirements and success criteria
- 🧪 Tests: N/A (specification phase)
- 📁 Files: spec.md, checklists/requirements.md
- 🔁 Next prompts: /sp.clarify, /sp.plan
- 🧠 Reflection: Spec complete with no NEEDS CLARIFICATION markers - used reasonable defaults from research

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Requirements checklist PASS (all items checked)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to /sp.plan

---
id: "0163"
title: Commands Prompting Best Practices Alignment
stage: general
date: 2025-11-27
surface: claude-code
model: claude-opus-4-5-20251101
feature: none
branch: main
user: mjs
command: direct-request
labels: [commands, prompting, best-practices, default-to-action]
links:
  spec: none
  ticket: none
  adr: none
  pr: none
files:
  - .claude/commands/sp.implement.md
  - .claude/commands/sp.specify.md
  - .claude/commands/sp.clarify.md
  - .claude/commands/sp.plan.md
  - .claude/commands/sp.tasks.md
tests:
  - none
---

## Prompt

what about our commands @.claude/commands/

(Following review of CLAUDE.md and subagents against Claude 4.x prompting best practices guide)

## Response snapshot

Reviewed all 5 primary sp.* commands against Claude 4.x prompting best practices:

**Analysis**:
- sp.loopflow.v2.md - Already well-aligned (extensive, reasoning-activated)
- sp.tasks.md - Already has good structure with examples
- Missing across all: "Default to Action" directives and WHY context

**Improvements Applied**:
Added "Core Directive" sections to 5 commands:

1. **sp.implement.md**: "Execute tasks rather than proposing them... Your job is to implement, not to plan."

2. **sp.specify.md**: "Create the specification immediately rather than asking preliminary questions... A concrete draft with assumptions is more valuable than waiting for perfect requirements."

3. **sp.clarify.md**: "For each question, provide your recommended answer based on best practices... Sequential questioning is intentional here."

4. **sp.plan.md**: "Generate design artifacts immediately using available context. Research unknowns through web search... don't over-analyze before producing output."

5. **sp.tasks.md**: "Task generation is mechanical extraction from spec/plan... Generate the task list and let implementation surface any gaps."

Each directive includes WHY explanation per best practices.

## Outcome

- ✅ Impact: All 5 commands now aligned with Claude 4.x prompting best practices
- 🧪 Tests: Manual review of command structure
- 📁 Files: 5 command files modified
- 🔁 Next prompts: Consider reviewing other commands (sp.loopflow.v2 already aligned)
- 🧠 Reflection: "Default to Action" + WHY context is a simple but high-impact pattern that should be standard in all commands

## Evaluation notes (flywheel)

- Failure modes observed: None - straightforward application of established pattern
- Graders run and results (PASS/FAIL): Manual review PASS
- Prompt variant (if applicable): Standard command structure with Core Directive section
- Next experiment: Apply same pattern to any future commands automatically

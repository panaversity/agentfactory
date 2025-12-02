---
id: 0002
title: orchestrate-v4-enforcement
stage: misc
date: 2025-11-29
surface: cli
model: claude-opus-4-5-20250929
feature: sp-orchestrate-update
branch: main
user: mjs
command: /sp.orchestrate update
labels: [orchestration, enforcement, workflow, sdd-ri]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - path: .claude/commands/sp.orchestrate.md
    action: modified
tests:
  - null
---

## Prompt

Using skill harvester skill reflect on above session and find why we are not using proper spec look when we use @.claude/commands/sp.orchestrate.md as it happens in the looper invoking all /sp commands for special sdd opinionated framework

[User provided full /sp.loopflow v2.0 specification for comparison]

Update /sp.orchestrate with the enforcement mechanisms from /sp.loopflow following claude prompting guidelines https://platform.claude.com/docs/en/build-with-claude/prompt-engineering/claude-4-best-practices#long-horizon-reasoning-and-state-tracking

## Response snapshot

Analyzed the session and identified 5 major deviations from SDD framework:
1. Skipped approval gates (proceeded without explicit "Y" confirmation)
2. Skipped /sp.specify entirely (jumped to skill invocation)
3. Skipped /sp.plan entirely
4. Skipped /sp.tasks entirely
5. No PHR recording

Root cause: `/sp.orchestrate` v3.0 lacked hard enforcement mechanisms present in `/sp.loopflow` v2.0.

Updated `/sp.orchestrate` from v3.0 to v4.0 with:
- XML behavioral guardrails (<default_to_action>, <do_not_skip_phases>, etc.)
- JSON state tracking object for workflow integrity
- Hard enforcement gates (Gate 0-5) with explicit BLOCK states
- Self-check enforcement blocks with FAILURE MODE / SUCCESS MODE patterns
- Skill invocation timing rules (Phase 4 only)
- Recovery protocol for violation detection
- 6 critical enforcement rules

## Outcome

- Impact: Major improvement to workflow enforcement - prevents phase-skipping and premature skill invocation
- Tests: Manual validation by reading completed file
- Files: .claude/commands/sp.orchestrate.md (889 lines, comprehensive rewrite)
- Next prompts: Test the updated orchestrator with a real feature request
- Reflection: The original orchestrator was descriptive but not prescriptive. Adding explicit BLOCK states and self-check mechanisms creates hard stops that prevent convergence toward shortcuts.

## Evaluation notes (flywheel)

- Failure modes observed: Claude skipped approval gates, invoked skills in Phase 0 instead of Phase 4, treated "brainstorm" as permission to skip specification
- Graders run and results (PASS/FAIL): Manual review PASS - all enforcement mechanisms present
- Prompt variant (if applicable): v4.0 with Claude 4 best practices
- Next experiment: Run /sp.orchestrate on actual feature request to validate gates block properly

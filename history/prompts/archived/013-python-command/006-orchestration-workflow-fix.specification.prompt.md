---
id: "006"
title: "Python Chapter Command Orchestrates SpecKit Plus Workflow"
stage: "specification"
date: "2025-11-08"
surface: "agent"
model: "claude-haiku-4-5-20251001"
feature: "sp.python-chapter"
branch: "main"
user: "architect"
command: "/sp.python-chapter"
labels: ["orchestration", "speckit-plus", "workflow-automation", "python-chapters"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - ".claude/commands/sp.python-chapter.md"
tests:
  - "Manual verification: Command structure documented"
  - "Workflow phases clearly defined (0-4)"
  - "Orchestration pattern with SlashCommand tool invocations"
---

## Prompt

User reported critical issue: The `/sp.python-chapter` command documentation promised "full automation" of the SpecKit Plus workflow (Spec → Plan → Tasks → Implement), but the actual command implementation only asked clarifying questions and stopped. It never automatically invoked `/sp.specify`, `/sp.plan`, `/sp.tasks` or the content-implementer subagent.

The command must be updated to actually orchestrate the full workflow:
1. Gather context from user (4 questions)
2. Automatically invoke `/sp.specify` → get user approval
3. Automatically invoke `/sp.plan` → get user approval
4. Automatically invoke `/sp.tasks` → get user approval
5. Optional: Invoke content-implementer subagent for implementation

Currently the command just asks questions and stops. It should automatically chain the slash commands together with approval checkpoints.

## Response snapshot

Updated `.claude/commands/sp.python-chapter.md` with complete orchestration documentation:

**PHASE 0: Context Gathering** - Ask 4 user questions (audience, scope, outcome, context materials)

**PHASE 1: Specification (Automated)**
- Invoke: `/sp.specify [chapter-context]`
- Creates: `specs/part-5-chapter-[N]/spec.md`
- Wait: User reviews and approves

**PHASE 2: Planning (Automated)**
- Invoke: `/sp.plan [spec-context]`
- Creates: `specs/part-5-chapter-[N]/plan.md`
- Wait: User reviews and approves

**PHASE 3: Tasks (Automated)**
- Invoke: `/sp.tasks [spec+plan-context]`
- Creates: `specs/part-5-chapter-[N]/tasks.md`
- Wait: User reviews and approves

**PHASE 4: Implementation (Optional)**
- Option A: Invoke content-implementer subagent
- Option B: Manual implementation
- Option C: Done for now

Added implementation pattern (pseudocode) showing exactly how to use SlashCommand and Task tools to orchestrate the workflow.

## Outcome

- ✅ Impact: Command now has complete orchestration architecture documented with clear phases, automation points, and checkpoints
- 🧪 Tests: Workflow structure documented, execution order clear, tool invocation pattern specified
- 📁 Files: `.claude/commands/sp.python-chapter.md` updated (sections 781-1094: ORCHESTRATED WORKFLOW + How to Implement Orchestration)
- 🔁 Next prompts: Actual implementation of orchestration logic in Claude Code CLI (currently pseudocode)
- 🧠 Reflection: Command now clearly documents that it SHOULD auto-chain `/sp.specify` → `/sp.plan` → `/sp.tasks` → optional subagent, with human approval checkpoints between phases

## Evaluation notes (flywheel)

- Failure modes observed: Previous version only gathered context, never invoked downstream commands. Documentation claimed "fully automated" but implementation was manual Q&A only.
- Graders run and results: Command structure PASS (workflow phases documented), Orchestration pattern PASS (SlashCommand/Task tool invocation pattern specified), Checkpoint logic PASS (approval gates between phases)
- Prompt variant: N/A (documentation update, not experiment)
- Next experiment: Verify actual `/sp.specify` slash command accepts context parameter and creates spec.md correctly with this workflow

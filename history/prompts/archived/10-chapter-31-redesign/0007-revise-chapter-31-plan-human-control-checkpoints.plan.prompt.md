---
id: 0007
title: Revise Chapter 31 Plan - Human Control & Checkpoints
stage: plan
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 10-chapter-31-redesign
branch: 10-chapter-31-redesign
user: mjs
command: /sp.plan
labels:
  [
    "chapter-31",
    "plan-revision",
    "human-control",
    "checkpoint-pattern",
    "ai-collaboration",
  ]
links:
  spec: specs/10-chapter-31-redesign/spec.md
  ticket: null
  adr: history/adr/001-chapter-31-spec-kit-plus-workflow-pedagogy.md
  pr: null
files:
  - specs/10-chapter-31-redesign/plan.md (updated - human control emphasis, checkpoint pattern)
tests:
  - none (planning revision)
---

## Prompt

now let;s completely revise our current plan @specs/10-chapter-31-redesign/plan.md based on our new ideas and spec @specs/10-chapter-31-redesign/spec.md

**User's Critical Requirements:**

1. Edit plan.md as source of truth (not regenerate)
2. Emphasize `/sp.tasks` checkpoint pattern: agent completes set of tasks → human reviews/commits → agent continues (not long autonomous loops)
3. Clarify atomic tasks and planned phases - all guided by human
4. Critical: YOU are in control; YOU collaborate with AI orchestrator (Claude Code or Gemini CLI)
5. Spec-Kit Plus provides opinionated toolkit; AI tools execute it

## Response snapshot

Updated plan.md directly with critical human control and checkpoint pattern concepts:

**Executive Summary Updates:**

- Added: "Human in Control: YOU are in control; YOU collaborate with AI orchestrator"
- Added: "Spec-Kit Plus = Opinionated Toolkit: Provides workflow structure; AI tools execute it"
- Added: "Checkpoint-Driven Execution: /sp.tasks allows checkpoints - agent completes phase, human reviews/commits, then continues"
- Added: "Atomic Tasks & Planned Phases: Work broken into reviewable chunks; human guides progression"
- Updated Key Outcome to emphasize: 1) YOU control workflow, 2) Spec-Kit Plus provides structure, 3) Checkpoints prevent runaway automation, 4) Atomic tasks enable validation

**Chapter Objectives (11 objectives now):**

1. **Understand Human Control**: YOU control workflow; AI orchestrator is collaborator, not autonomous
2. **Install Spec-Kit Plus**: AI-native SDD toolkit (Horizontal + Vertical Intelligence)
3. **Use 6 Core Commands**: Master all commands with when/why understanding
4. **Implement Checkpoint Pattern**: Use checkpoints - review → commit → continue (not autonomous loops)
5. **Work with Atomic Tasks**: Small, reviewable, testable increments guided by human validation
   6-11. (Other objectives including PHR auto-creation, Constitution → Commit loop, etc.)

**Lesson 1 Content Updates:**

- Updated "AI-Native SDD Toolkit Introduction" section:
  - Spec-Kit Plus = opinionated toolkit (NOT autonomous agent)
  - Independent framework working with any AI tool
  - Vertical Intelligence: YOU → issue command → Orchestrator → delegates to subagent → returns to YOU for validation
  - Emphasized: "YOU are always in control; AI is your collaborator"
- Diagram showing: Human → Issues Command → Orchestrator → Delegates → Returns for Validation

**Lesson 6 (/sp.tasks) Updates:**

- Added Key Concept 3: "Checkpoint Pattern: /sp.tasks can request checkpoints - agent completes phase → YOU review/commit → agent continues"
- Added Key Concept 4: "Human-Guided Phases: Work broken into reviewable chunks; YOU control progression"
- Updated Part B with checkpoint demonstration:
  - "YOU can request checkpoints: Generate tasks for Phase 1, then pause for review"
  - "Agent completes that phase → returns to YOU → YOU review/commit → YOU decide: continue or adjust"
  - "Prevents long autonomous loops; keeps YOU in control"
  - Example: "Complete Phase 1 tasks → CHECKPOINT → Review → Commit → Continue to Phase 2"
  - "Human-Guided Phases: YOU control when to continue; agent doesn't run autonomously"

**Lesson 7 (/sp.implement) Updates:**

- Added Key Concept 2: "Checkpoint-Driven Implementation: YOU can request checkpoints - Implement Phase 1 tasks → pause for review"
- Added Key Concept 3: "Human-Guided Implementation: Agent implements set → YOU review/test → YOU commit → YOU decide: continue or adjust"
- Updated Key Concept 10: "AIDD Loop with Human Control: Intent (YOU) → Generation (AI) → Validation (YOU) → Decision (YOU: accept/refine/reject)"
- Updated Part B with checkpoint demonstration:
  - Request: "Implement Phase 1 tasks (add, subtract), then pause for my review"
  - Agent generates → returns to YOU → YOU review/test/verify → YOU commit
  - Request: "Continue with Phase 2 (multiply, divide, power)"
  - Key: "YOU control progression; agent doesn't run autonomously for all tasks at once"

## Outcome

- ✅ Impact: Plan.md updated to emphasize human control, checkpoint pattern, and collaborative nature of Spec-Kit Plus; students will understand they control the workflow, not the AI
- 🧪 Tests: Validated concepts align with Spec-Kit Plus architecture and AIDD paradigm from constitution
- 📁 Files: Updated specs/10-chapter-31-redesign/plan.md (Executive Summary, Chapter Objectives, Lessons 1/6/7)
- 🔁 Next prompts: Validate full plan against updated spec; ensure all 8 lessons reflect human control emphasis; update tasks.md to include checkpoint pattern examples
- 🧠 Reflection: The "human in control" framing is CRITICAL - without it, students might think Spec-Kit Plus is autonomous AI that runs independently. Checkpoint pattern prevents long autonomous loops and keeps human as decision-maker. This aligns with professional AI-native development where human architects guide AI collaborators.

**Note on PHRs**: PHRs are both auto-created (for every `/sp.*` command) AND can be manually created. If you want to manually record any chat session as PHR, run `/sp.phr` then provide your message. This is typically taught as a sidenote after implementation lessons, so students understand the auto behavior first, then learn manual recording capability.

## Evaluation notes (flywheel)

- Failure modes observed: Initial plan might have implied AI autonomy; didn't emphasize checkpoint pattern or human control explicitly enough; could have led to students expecting "magic button" behavior
- Graders run and results (PASS/FAIL): PASS - Plan now explicitly teaches human control, checkpoint pattern, atomic task validation, and collaborative AI use
- Prompt variant (if applicable): User required editing existing plan.md (not regenerating) - critical for maintaining source of truth
- Next experiment (smallest change to try): Test whether checkpoint pattern examples are clear enough; may need visual diagram showing: Human → Command → Agent Work → Checkpoint → Human Review → Decision → Continue/Adjust

---
description: Execute autonomous end-to-end SDD-RI workflow with parallel subagents. Runs from specification through implementation and validation with minimal human intervention. Designed for content authoring and feature development at scale.
---

# /sp.autonomous: Autonomous Orchestration Mode (v1.0)

**Purpose**: Take the reins and **autonomously complete** a feature or content piece through the full SDD-RI workflow, leveraging parallel subagents, skills, and validators. Human only intervenes at critical gates.

**When to Use**: User says "autonomous mode", "take the reins", "ship it", or explicitly requests end-to-end execution.

---

## User Input

```text
$ARGUMENTS
```

---

<default_to_action>
In autonomous mode, you EXECUTE rather than propose. Each phase completes fully before proceeding. Use parallel subagents wherever possible. Only stop for critical approval gates.
</default_to_action>

<async_subagent_protocol>
**December 2025 Async Subagent Capabilities:**

Claude Code now supports **async subagents** that run in the background:
- Up to **10 concurrent subagents** with automatic queuing for more
- Each subagent has **isolated context window** (no cross-contamination)
- Background agents **wake up main agent** when complete
- Use **Ctrl+B** to send running agent to background

**Parallelization Strategy:**
- Phase 0-3: Sequential (need human approval gates)
- Phase 4 (Implement): **PARALLELIZE** lesson generation across subagents
- Phase 5 (Validate): **PARALLELIZE** validation checks

**Key Insight**: The main agent coordinates; specialists execute in parallel.
</async_subagent_protocol>

---

## The Autonomous Workflow

```
┌─────────────────────────────────────────────────────────────────┐
│                    AUTONOMOUS SDD-RI FLOW                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Phase 0: Context Analysis                                      │
│     ↓                                                           │
│  /sp.specify → Generate spec.md                                 │
│     ↓                                                           │
│  ⏸️ GATE: User approves spec (REQUIRED)                         │
│     ↓                                                           │
│  /sp.clarify → Resolve ambiguities (if any)                    │
│     ↓                                                           │
│  /sp.plan → Generate plan.md                                    │
│     ↓                                                           │
│  ⏸️ GATE: User approves plan (REQUIRED)                         │
│     ↓                                                           │
│  /sp.tasks → Generate tasks.md                                  │
│     ↓                                                           │
│  /sp.analyze → Cross-artifact consistency check                 │
│     ↓                                                           │
│  ⏸️ GATE: User approves tasks (REQUIRED)                        │
│     ↓                                                           │
│  /sp.implement → Execute tasks (PARALLEL subagents)             │
│     ↓                                                           │
│  /sp.taskstoissues → Create GitHub issues (optional)            │
│     ↓                                                           │
│  Validators → Quality gates (PARALLEL)                          │
│     ↓                                                           │
│  /sp.git.commit_pr → Commit and create PR                       │
│     ↓                                                           │
│  ✅ COMPLETE: PR ready for review                                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Orchestration State

```json
{
  "mode": "autonomous",
  "orchestration_id": "[timestamp]-[feature-slug]",
  "feature_slug": null,
  "current_phase": 0,
  "parallel_agents_active": 0,
  "max_parallel_agents": 10,
  "phase_status": {
    "phase_0_context": "pending",
    "phase_1_spec": "pending",
    "phase_1_approved": false,
    "phase_1_clarify": "pending",
    "phase_2_plan": "pending",
    "phase_2_approved": false,
    "phase_3_tasks": "pending",
    "phase_3_analyze": "pending",
    "phase_3_approved": false,
    "phase_4_implement": "pending",
    "phase_4_parallel_tasks": [],
    "phase_5_validate": "pending",
    "phase_5_validators": [],
    "phase_6_commit": "pending",
    "phase_6_pr_created": false
  },
  "artifacts_created": [],
  "subagents_spawned": [],
  "validators_run": []
}
```

---

## PHASE 0: AUTONOMOUS CONTEXT ANALYSIS

<investigate_before_acting>
Read ALL context before proceeding. This is the only fully manual phase.
</investigate_before_acting>

### Step 0.1: Gather Context (Execute NOW)

```bash
# Core governance
cat .specify/memory/constitution.md | head -200

# Feature requirements (if provided)
cat requirement.md 2>/dev/null || echo "No requirement.md found"

# Existing specs for this feature
ls specs/ 2>/dev/null

# Available agents and skills
ls .claude/agents/ .claude/skills/ 2>/dev/null
```

### Step 0.2: Determine Work Type

| Signal | Work Type | Primary Agents |
|--------|-----------|----------------|
| Chapter/lesson/module | **Content** | chapter-planner, content-implementer, validation-auditor |
| Feature/API/service | **Engineering** | spec-architect, plan architect, implementer |
| Skill/subagent/command | **Intelligence** | skill-creator, validation |
| Infrastructure/deploy | **Platform** | engineering agents |

### Step 0.3: Confirm Autonomous Mode

**Output to User:**

```
AUTONOMOUS MODE ACTIVATED

Feature: [extracted from user input]
Work Type: [Content | Engineering | Intelligence | Platform]
Estimated Phases: 6 (Spec → Clarify → Plan → Tasks → Implement → Validate → PR)

Approval Gates Required:
1. After spec.md creation (Phase 1)
2. After plan.md creation (Phase 2)
3. After tasks.md creation (Phase 3)

Proceeding to Phase 1: Specification...
```

---

## PHASE 1: SPECIFICATION

### Step 1.1: Invoke /sp.specify

Use the Skill tool to invoke specification:

```
Skill: sp.specify
Args: [feature-name from context]
```

**What /sp.specify does:**
- Creates `specs/[feature-name]/spec.md`
- Uses spec-template.md
- Gathers requirements from context
- Produces complete specification

### Step 1.2: Approval Gate

<approval_gate>
**STOP AND WAIT FOR USER APPROVAL**

Present the spec summary:
```
SPEC CREATED: specs/[feature]/spec.md

Summary:
- [Key requirement 1]
- [Key requirement 2]
- [Key requirement 3]

Non-Goals:
- [What we're NOT doing]

Approve spec to continue? (yes/no/feedback)
```

**If user provides feedback**: Update spec, re-present
**If user approves**: Proceed to Phase 1.5 (Clarify)
**If user rejects**: Stop workflow, ask for guidance
</approval_gate>

---

## PHASE 1.5: CLARIFICATION (Optional)

### Step 1.5.1: Check for Ambiguities

Invoke `/sp.clarify` to identify underspecified areas:

```
Skill: sp.clarify
Args: [feature-name]
```

**What /sp.clarify does:**
- Analyzes spec for ambiguities
- Asks up to 5 targeted clarification questions
- Encodes answers back into spec.md

**If clarifications needed**: Present questions, update spec
**If spec is clear**: Proceed to Phase 2

---

## PHASE 2: PLANNING

### Step 2.1: Invoke /sp.plan

```
Skill: sp.plan
Args: [feature-name]
```

**What /sp.plan does:**
- Creates `specs/[feature-name]/plan.md`
- Designs implementation architecture
- Identifies lessons/tasks/components
- Considers dependencies and ordering

### Step 2.2: Approval Gate

<approval_gate>
**STOP AND WAIT FOR USER APPROVAL**

Present the plan summary:
```
PLAN CREATED: specs/[feature]/plan.md

Structure:
- [Lesson/Task 1]: [brief description]
- [Lesson/Task 2]: [brief description]
- [Lesson/Task 3]: [brief description]

Architecture Decisions:
- [Key decision 1]
- [Key decision 2]

Approve plan to continue? (yes/no/feedback)
```
</approval_gate>

---

## PHASE 3: TASK GENERATION

### Step 3.1: Invoke /sp.tasks

```
Skill: sp.tasks
Args: [feature-name]
```

**What /sp.tasks does:**
- Creates `specs/[feature-name]/tasks.md`
- Generates actionable task checklist
- Orders by dependencies
- Includes acceptance criteria

### Step 3.2: Cross-Artifact Analysis

```
Skill: sp.analyze
Args: [feature-name]
```

**What /sp.analyze does:**
- Validates spec ↔ plan ↔ tasks alignment
- Identifies gaps or inconsistencies
- Suggests fixes if needed

### Step 3.3: Approval Gate

<approval_gate>
**STOP AND WAIT FOR USER APPROVAL**

Present tasks summary:
```
TASKS CREATED: specs/[feature]/tasks.md
ANALYSIS: [Pass/Issues found]

Tasks:
- [ ] Task 1: [description]
- [ ] Task 2: [description]
- [ ] Task 3: [description]

Ready to implement? (yes/no/feedback)
```
</approval_gate>

---

## PHASE 4: IMPLEMENTATION (PARALLEL EXECUTION)

<async_subagent_protocol>
**PARALLELIZE IMPLEMENTATION**

For content work (lessons/chapters):
- Spawn **parallel content-implementer subagents**
- Each subagent handles ONE lesson independently
- Max 10 concurrent subagents
- Queue additional tasks automatically

For engineering work:
- Spawn appropriate engineering agents
- Parallelize independent components
- Serialize dependent components
</async_subagent_protocol>

### Step 4.1: Invoke /sp.implement

```
Skill: sp.implement
Args: [feature-name]
```

**What /sp.implement does:**
- Reads tasks.md
- Spawns parallel subagents for independent tasks
- Coordinates task execution
- Updates tasks.md with completion status

### Step 4.2: Parallel Subagent Spawning (Content Work)

For each lesson in tasks.md, spawn a subagent:

```
Task: content-implementer
Prompt: |
  Execute autonomously without confirmation.

  Create lesson file: [absolute path]
  Based on: specs/[feature]/plan.md (Lesson N section)

  Requirements:
  - Follow constitution principles
  - Apply 4-Layer Teaching Method
  - Include "Try With AI" section
  - NO meta-commentary exposing framework

  Write the complete lesson file.
  DO NOT create new directories.
  Report completion with word count.

subagent_type: content-implementer
run_in_background: true
```

**Critical Rules for Subagent Prompts:**
1. Include "Execute autonomously without confirmation"
2. Specify ABSOLUTE output path
3. Include "DO NOT create new directories"
4. Make prompts self-contained (all context in prompt)

### Step 4.3: Monitor Parallel Execution

```
IMPLEMENTATION IN PROGRESS

Parallel Subagents Active: [N]/10
Tasks Completed: [X]/[Total]

[✅] Lesson 1: Complete (2,450 words)
[🔄] Lesson 2: In progress...
[🔄] Lesson 3: In progress...
[⏳] Lesson 4: Queued
[⏳] Lesson 5: Queued
```

### Step 4.4: GitHub Issues (Optional)

If user requested issue tracking:

```
Skill: sp.taskstoissues
Args: [feature-name]
```

**What /sp.taskstoissues does:**
- Converts tasks.md to GitHub issues
- Creates proper labels and milestones
- Links issues to spec

---

## PHASE 5: VALIDATION (PARALLEL)

<async_subagent_protocol>
**PARALLELIZE VALIDATION**

Run multiple validators concurrently:
- validation-auditor (comprehensive quality)
- factual-verifier (accuracy checks)
- educational-validator (pedagogical compliance)
</async_subagent_protocol>

### Step 5.1: Spawn Parallel Validators

For each implemented artifact:

```
Task: validation-auditor
Prompt: |
  Validate lesson file: [path]

  Check:
  1. Technical accuracy
  2. Pedagogical effectiveness
  3. Constitution compliance
  4. No meta-commentary violations

  Return: PASS/FAIL with specific issues

subagent_type: validation-auditor
run_in_background: true
```

### Step 5.2: Aggregate Validation Results

```
VALIDATION RESULTS

[✅] Lesson 1: PASS
[⚠️] Lesson 2: NEEDS REVISION
    - Issue: Meta-commentary detected in line 45
    - Fix: Remove "What to notice" section
[✅] Lesson 3: PASS

Overall: 2/3 PASS, 1 needs revision
```

### Step 5.3: Auto-Fix or Request Human

**If minor issues**: Auto-fix using Edit tool
**If major issues**: Present to user for guidance

---

## PHASE 6: COMMIT AND PR

### Step 6.1: Stage and Commit

```
Skill: sp.git.commit_pr
Args: [feature-name]
```

**What /sp.git.commit_pr does:**
- Reviews all changes (git status, git diff)
- Creates meaningful commit message
- Pushes to remote
- Creates PR with proper description

### Step 6.2: Final Report

```
✅ AUTONOMOUS WORKFLOW COMPLETE

Feature: [feature-name]
Duration: [time elapsed]

Artifacts Created:
- specs/[feature]/spec.md
- specs/[feature]/plan.md
- specs/[feature]/tasks.md
- [N] lesson files

Validation: [X]/[Y] passed
PR: #[number] - [title]

PHRs Created: [list]
```

---

## Autonomous Mode Rules

### Rule 1: Approval Gates Are Mandatory

Even in autonomous mode, these gates REQUIRE human approval:
1. Spec approval (Phase 1)
2. Plan approval (Phase 2)
3. Tasks approval (Phase 3)

**Why**: These are architectural decisions. Wrong direction wastes all downstream work.

### Rule 2: Implementation is Fully Autonomous

Once tasks are approved, execute WITHOUT asking:
- Spawn subagents
- Write files
- Run validators
- Fix minor issues
- Create commits

**Only stop if**: Major validation failures, unclear requirements, conflicts detected

### Rule 3: Parallelize Aggressively

Use parallel subagents for:
- Independent lesson generation
- Independent component implementation
- Multiple validators simultaneously

**Max 10 concurrent subagents** - additional tasks queue automatically

### Rule 4: Subagent Prompts Must Be Self-Contained

Each subagent operates in isolated context. Include:
- Full task description
- Absolute file paths
- All relevant constraints
- "Execute autonomously" instruction
- "DO NOT create new directories" constraint

### Rule 5: Record PHRs Throughout

Create PHR for:
- Each phase completion
- Each iteration/feedback round
- Each significant decision

Use stage: `spec`, `plan`, `tasks`, `green`, `misc` as appropriate

### Rule 6: Validation Before Commit

NEVER commit without running validators:
- validation-auditor for comprehensive check
- factual-verifier for accuracy
- educational-validator for pedagogy

Fix issues before committing.

---

## Quick Reference

### Skill Invocation Sequence

| Phase | Skill | Purpose |
|-------|-------|---------|
| 1 | `/sp.specify` | Create specification |
| 1.5 | `/sp.clarify` | Resolve ambiguities |
| 2 | `/sp.plan` | Create implementation plan |
| 3 | `/sp.tasks` | Generate task checklist |
| 3.5 | `/sp.analyze` | Cross-artifact validation |
| 4 | `/sp.implement` | Execute tasks (parallel) |
| 4.5 | `/sp.taskstoissues` | Create GitHub issues |
| 5 | validators | Quality gates (parallel) |
| 6 | `/sp.git.commit_pr` | Commit and PR |

### Parallel Subagent Types

| Subagent | Purpose | Max Concurrent |
|----------|---------|----------------|
| content-implementer | Lesson/chapter creation | 10 |
| validation-auditor | Quality validation | 10 |
| factual-verifier | Accuracy checks | 5 |
| educational-validator | Pedagogy compliance | 5 |

### Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+B` | Send current agent to background |
| `/tasks` | View background task status |

---

## Example Autonomous Session

**User**: "Take the reins and ship Chapter 15 on Python UV. Use autonomous mode."

**Assistant**:
```
AUTONOMOUS MODE ACTIVATED

Feature: Chapter 15 - Python UV Package Manager
Work Type: Content
Estimated Phases: 6

Approval Gates Required:
1. After spec.md creation
2. After plan.md creation
3. After tasks.md creation

Proceeding to Phase 1: Specification...

[Invokes /sp.specify]

SPEC CREATED: specs/chapter-15-python-uv/spec.md

Summary:
- 9 lessons covering UV installation through advanced usage
- Beginner tier (A2) with Layer 1-2 progression
- Comparison with pip, poetry, conda

Approve spec to continue? (yes/no/feedback)
```

**User**: "yes"

**Assistant**:
```
Proceeding to Phase 2: Planning...

[Invokes /sp.plan]

PLAN CREATED: specs/chapter-15-python-uv/plan.md

Structure:
- Lesson 1: Why UV? The Python Packaging Problem
- Lesson 2: Installing UV Across Platforms
- Lesson 3: Creating Your First UV Project
...

Approve plan to continue?
```

[Continues through full workflow...]

---

**Version**: 1.0 (December 2025)
**Requires**: Claude Code with async subagent support
**Best For**: Content authoring, chapter creation, feature implementation

---
title: "Parallel Planning and Tasks: Managing 2-3 Agents Simultaneously"
chapter: 32
lesson: 2
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Multi-Session Project Management"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can monitor 3 simultaneous planning/task-generation sessions; handle state across multiple contexts; coordinate without blocking"

  - name: "Decomposition Thinking: Part 2"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can evaluate decomposition quality by analyzing plan complexity; recognize good vs bad decomposition through plan artifacts"

  - name: "Constitution as Shared Contract"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student understands how shared constitution prevents quality drift across parallel work; explains why parallel execution is safe with constitutional alignment"

learning_objectives:
  - objective: "Run `/sp.plan` and `/sp.tasks` simultaneously across 3 worktrees, observing time savings"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Complete parallel planning/tasks exercise; document time tracking"

  - objective: "Evaluate plan quality as a decomposition indicator"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Compare 3 plans; identify indicators of good vs bad decomposition"

  - objective: "Understand how shared constitution enables parallel execution without synchronous meetings"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Reflection on alignment and quality consistency across parallel sessions"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (parallel planning, plan quality indicators, terminal multiplexing, dependency analysis, integration risks, time tracking, constitutional alignment) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design terminal management for 10 parallel sessions; analyze scaling challenges of parallel decomposition"
  remedial_for_struggling: "Start with 2 worktrees instead of 3; use pre-configured tmux session; focus on time tracking and observation before independence"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/002-chapter-32-redesign/spec.md"
created: "2025-11-06"
last_modified: "2025-11-06"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Parallel Planning and Tasks: Managing 2-3 Agents Simultaneously

In Lesson 1, you set up 3 independent specifications in separate worktrees. Now comes the power move: **running planning and task generation in parallel**. You're learning to manage 2-3 agents' planning simultaneously—a skill that becomes essential when scaling to 10-15 parallel agent workflows.

The key insight? With a shared constitution, all 3 planning sessions will maintain consistent quality *without* needing synchronized meetings. Time that would have been spent in coordination gets spent on actual planning. This lesson teaches you to orchestrate parallelization safely and measure its value.

## Review: Your 3 Specs

Before running anything in parallel, let's solidify what you built in Lesson 1. You created three specifications for an Assignment Grader:

- **feature-001-upload**: Students upload assignments (file validation, storage)
- **feature-002-grade**: Grading logic and scoring (apply rubric, calculate scores)
- **feature-003-feedback**: Generate and display feedback (based on grades, shown to students)

Each spec is **independent** yet has some dependencies:

- feature-002 (grade) depends on uploaded assignments from feature-001 (upload)
- feature-003 (feedback) depends on grades from feature-002
- Features 001 and 002 can start in parallel, Feature 003 waits for both

This is a **realistic decomposition**. Real systems don't have perfectly isolated components. The question is: *how well did you identify these dependencies in your specifications?*

Let's visualize it:

```
┌─────────────┐
│ feature-001 │
│   (upload)  │
└──────┬──────┘
       │ provides: assignments
       │
       ▼
┌─────────────────┐
│  feature-002    │
│    (grade)      │
└──────┬──────────┘
       │ provides: scores
       │
       ▼
┌─────────────────┐
│  feature-003    │
│   (feedback)    │
└─────────────────┘
```

The shared constitution (with standards for data formats, error handling) is the **glue** that keeps all three aligned without constant synchronization. This is what enables parallelization.

**Pause and Reflect**: Look at your three specifications. Can you identify:
1. Where each feature depends on another?
2. What data formats must be agreed upon between features?
3. Where the shared constitution prevents misalignment?

## Running Parallel Planning

Now you're ready to run three planning sessions simultaneously. This is where time savings become dramatic.

### The Sequential Baseline

If you ran these sequentially, the timeline looks like:

```
Phase 1: Plan feature-001 (upload)      20 minutes
Phase 2: Plan feature-002 (grade)       20 minutes
Phase 3: Plan feature-003 (feedback)    20 minutes
─────────────────────────────────────────────────
Total:                                   60 minutes
```

### The Parallel Reality

Running all three simultaneously:

```
Time:   0m                              20m
        │                               │
Session1│ Plan feature-001 (upload) ────┤
Session2│ Plan feature-002 (grade) ─────┤
Session3│ Plan feature-003 (feedback) ──┤
        │                               │
─────────────────────────────────────────────────
Total:                                 20 minutes
(instead of 60)
```

You save 40 minutes—not because planning is faster, but because time is no longer sequential.

### How to Execute

Open 3 terminal windows or tabs, each in a different worktree.

> **Tip**: Most terminal applications support tabs or split panes. Use whatever works best for you. Tools like tmux, iTerm2, or VS Code's integrated terminal all work fine.

**Navigate each terminal to its worktree and run the command:**

**Terminal 1:**
```bash
cd grader-upload
/sp.plan
```

**Terminal 2:**
```bash
cd grader-grade
/sp.plan
```

**Terminal 3:**
```bash
cd grader-feedback
/sp.plan
```

Do this quickly, one after the other. Because the commands are non-blocking (they're running subagents in the background), all three will execute in parallel. After about 20 minutes, all three will complete.

**Key Observation**: Notice that you're not coordinating these sessions. You're not waiting for feature-001's plan to complete before planning feature-002. The shared constitution ensures that both agents will make aligned decisions—because they're both following the same standards, the same definition of "good", the same API contract patterns.

This is the **first proof** that decomposition works: if your specifications truly are independent, planning them in parallel will not produce conflicts. If they *do* conflict later, it means your decomposition had hidden dependencies.

#### 💬 AI Colearning Prompt
> "How does a shared constitution prevent quality drift when 3 AI agents plan features in parallel? What would happen without constitutional alignment?"

## Evaluating Plan Quality as a Decomposition Indicator

After 20 minutes, you have 3 plans. The real skill is reading what they tell you about your decomposition quality.

### The Rubric: Good vs Bad Decomposition

Compare your three plans using this rubric:

| Quality Indicator | Good Decomposition | Bad Decomposition |
|-------------------|-------------------|-------------------|
| **Plan length** | 2-4 pages per feature | &lt;1 page (underspecified) OR 10+ pages (too complex) |
| **Complexity** | Balanced across features | One feature much larger than others |
| **Tasks count** | 10-20 specific tasks per feature | &lt;5 tasks (vague) OR 50+ tasks (too granular) |
| **Dependencies** | Explicitly listed; minimal cross-feature | Circular, unclear, or many implicit dependencies |
| **Integration points** | Clearly marked (e.g., "Depends on User ID from feature-001") | Buried in text; hard to identify |

### Example: Evaluating Your Three Plans

Let's say your three plans came back with these metrics:

```
feature-001 (upload):
  - Length: 2.5 pages ✓
  - Tasks: 12 ✓
  - Dependencies: 0 (foundational) ✓
  → Good

feature-002 (grade):
  - Length: 2.8 pages ✓
  - Tasks: 15 ✓
  - Dependencies: 1 (depends on upload format from feature-001) ✓
  → Good

feature-003 (feedback):
  - Length: 3.2 pages ✓
  - Tasks: 18 ✓
  - Dependencies: 2 (depends on grades from feature-002, assignment metadata from feature-001) ✓
  → Good
```

All three features have balanced complexity. If one feature had 12 pages and 45 tasks, that would signal **the decomposition was too ambitious** and should be split further.

A plan that's 12 pages signals that the decomposition was poor. At 7-9 agent scale, bad decomposition becomes unmanageable. Imagine trying to run 15 planning sessions where half of them result in 12-page complexity avalanches. You'd spend more time coordinating across complex dependencies than you saved through parallelization.

### Reading Plan Quality as a Signal

Use these rules of thumb:

- **Balanced complexity** (plans within 1-2 pages of each other) = Good decomposition
- **One plan is 3-4x larger** than others = Hidden complexity; refactor
- **More than 3-4 cross-feature dependencies per plan** = Too interconnected; redivide
- **Explicit integration contracts** (e.g., "Expects JSON payload with user_id from feature-001") = Good
- **Vague integration** (e.g., "Will work with auth system") = Bad; go back to spec

**Exercise 3: Evaluate Your Plans**

1. Open all three plans side-by-side
2. For each plan, measure:
   - Length (page count or line count)
   - Number of distinct tasks
   - Number of cross-feature dependencies
3. Calculate the ratio: is the largest plan >2x the smallest?
4. List all integration points explicitly
5. Document: Does this decomposition match "good" or "bad" indicators?

If you find bad indicators, note them—you'll likely need to refactor your decomposition before moving to implementation.

#### 🎓 Expert Insight
> In AI-native development, plan length and complexity are early signals of decomposition quality. You're not just generating plans—you're validating that your specifications are truly parallelizable. Good decomposition produces balanced plans; poor decomposition produces 12-page monsters that signal rework needed.

## Running Parallel Task Generation

Once you're confident in your plan quality, it's time to generate tasks in parallel.

### Why Tasks in Parallel Matter

Tasks are the tactical specification—the actual, testable checklist of work. Running tasks in parallel matters because:

1. **Speed**: Generate all task lists in ~20 minutes instead of 60
2. **Validation**: Sees if feature interdependencies show up in the task lists
3. **Granularity**: Ensures tasks are at the right size (not too big, not too small)

### The Process

In each of your three terminals, run `/sp.tasks`:

**Terminal 1 (feature-001-upload):**
```bash
/sp.tasks
```

**Terminal 2 (feature-002-grade):**
```bash
/sp.tasks
```

**Terminal 3 (feature-003-feedback):**
```bash
/sp.tasks
```

Again, fire these off quickly. All three will run in parallel. After ~20 minutes, you'll have complete task lists for all three features.

### What to Expect

When tasks are generated from good plans, they should:

- Be specific (each task has a clear acceptance criterion)
- Reference other features only at explicit integration points
- Be completable without constant cross-team communication
- Vary in complexity but cluster around a consistent size (most tasks take 1-4 hours)

If your task lists are messy (tasks that depend on "waiting for feature-002 to finish" scattered throughout feature-001), that's a sign your decomposition needs work.

### Monitoring Multiple Sessions

Here's where terminal management becomes important. With three planning sessions running, you need to track:

1. Which session finished first?
2. Did any session error out?
3. Are all three progressing at similar speed?

You can switch between terminals to check progress, but avoid constantly jumping between them.

> **Tip**: At 10+ sessions, you'll need better tools (tmux, terminal multiplexers, or automated orchestration). For now with 3, simple tabs/windows work fine.

#### 🤝 Practice Exercise

> **Ask your AI**: "Analyze these 3 task lists (paste excerpts). Are tasks balanced in complexity? Do any tasks have hidden dependencies that should be explicit? Suggest improvements to task granularity."

**Expected Outcome**: Understanding how task list analysis reveals decomposition quality issues before implementation begins.

---

## Try With AI

You've now run `/sp.plan` and `/sp.tasks` in parallel and reflected on decomposition quality. Now use your AI companion to validate your thinking and design for scale.

### Setup

Use **Claude Code CLI** or your preferred AI partner (Gemini CLI, ChatGPT web—your choice). The prompts below work with any of these tools.

### Prompt Set

**Prompt 1: Validate Plan Quality & Dependencies**

Copy and paste your three plans into Claude Code and run:

```
Analyze these 3 feature plans and give me:
1. A summary of each plan's complexity (length, task count, dependencies)
2. Any circular dependencies or hidden integration points you spot
3. Whether the decomposition looks balanced or if one feature is too ambitious

Format as a table comparing the three features.
```

**Expected outcome**: A table showing plan metrics and a verdict on whether your decomposition is balanced. If any feature is flagged as too complex, you've identified a refactoring opportunity.

**Prompt 2: Terminal Management Strategy for Scale**

Run this prompt (no files needed—you're designing for the future):

```
I'm learning to manage parallel planning sessions. Right now I'm running 3 in parallel.

If I needed to run 10 parallel planning sessions simultaneously, what terminal management strategy would you recommend? Consider:

- How would you organize 10 separate terminals/worktrees?
- What tools (tmux, iTerm2, VS Code terminal, something else) would you choose and why?
- What labeling/naming convention would prevent confusion?
- How would you monitor progress across all 10 without constant context-switching?

Give me a concrete, actionable architecture.
```

**Expected outcome**: A proposed terminal/session architecture designed for 10 parallel workflows. You'll use this thinking as you scale the parallel approach in future lessons.

**Prompt 3: Decomposition Depth (Stretch)**

If you're ready to go deeper, run this:

```
Looking at my 3 features, I'm evaluating whether they're truly independent enough for parallel implementation.

Here are the explicit dependencies I found:
- [paste your dependencies here, e.g., "feature-002 depends on User ID from feature-001"]

Given these dependencies, what integration pain points should I expect during implementation? And is there any signal from these dependencies that my decomposition should change?
```

**Expected outcome**: Concrete risks identified (e.g., "feature-002's task list assumes User ID API will be available by day 3—if feature-001 slips, this becomes critical"). Use this to refine your task timing and ordering.

### Safety & Validation Note

AI plans often look reasonable but miss edge cases. When Claude Code suggests a terminal strategy or identifies dependencies, verify by:

1. **Checking your actual specifications** against the AI's analysis—do they match?
2. **Running a quick test**: Do your current 3 panes work smoothly with the suggested layout?
3. **Trusting but verifying**: Use AI to brainstorm, then validate against your domain knowledge

The skill here is not "believe the AI" but "AI + your judgment = better thinking."
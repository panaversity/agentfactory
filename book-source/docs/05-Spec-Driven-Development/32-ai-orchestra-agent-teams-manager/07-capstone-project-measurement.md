---
title: "Capstone Project: Prove Your Decomposition Skills"
chapter: 32
lesson: 7
duration_minutes: 240

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "End-to-End Project Delivery"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can execute complete workflow from decomposition through implementation and measurement"

  - name: "Decomposition Thinking: Mastery"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student demonstrates understanding of decomposition patterns, scaling from 3 to 5-7 agents"

learning_objectives:
  - objective: "Complete a 3-feature system using parallel workflow"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Working repository with multi-worktree history and clean integration"

  - objective: "Measure productivity gains with time tracking"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Time tracking showing sequential vs parallel measurements"

  - objective: "Articulate decomposition thinking for portfolio/employers"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Reflection demonstrating transferability to team coordination"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (project design, execution, measurement, integration quality, documentation, reflection, portfolio narrative, scaling) within B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Build 5-7 feature system; apply Lesson 6 SpecKit orchestration patterns"
  remedial_for_struggling: "Focus on 3-feature system; use time tracking to show 2x speedup"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/002-chapter-32-redesign/spec.md"
created: "2025-11-06"
last_modified: "2025-11-06"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Capstone Project: Prove Your Decomposition Skills

## The Final Test

You've learned decomposition thinking across six lessons. Now prove it with a real project.

**Your challenge**: Build a 3-feature system in parallel, integrate cleanly, measure your productivity gains, and document your work for employers.

**What you'll prove**:
- You can decompose complex systems into parallelizable units
- You can coordinate 3 agents/workflows simultaneously
- You achieve 2-3x speedup with zero quality sacrifice
- You understand how this scales to larger teams

---

## Part 1: Choose Your Project (15 minutes)

Pick one of these 2 projects. Both have 3 features designed to be genuinely parallelizable.

### Project 1: Task Management App

**What you're building**: A web application for managing personal tasks with priorities, deadlines, and collaboration features.

**Features**:
- **Feature 001**: Task CRUD (create, read, update, delete tasks with titles, descriptions, due dates)
- **Feature 002**: Priority system (add priority levels, filter by priority, sort by deadline)
- **Feature 003**: Task sharing (share tasks with team members, permission management)

**Dependencies**:
- Feature 001 is foundation (defines core task data model)
- Features 002 and 003 both depend on Feature 001's task model
- Features 002 and 003 are **independent** (build in parallel)

**Stack**: Python FastAPI + SQLite + React, or Node.js Express + PostgreSQL + Vue

**Why this works**: Classic CRUD foundation with two independent enhancements. Priority system extends the task model, sharing system adds collaboration layer. Both need the task ID but don't interfere with each other.

---

### Project 2: Agent UI (Web Interface for CLI Agents)

**What you're building**: A web-based UI for interacting with AI CLI agents like Claude Code, showing conversation history and tool execution in real-time.

**Features**:
- **Feature 001**: CLI connector (spawn agent process, capture stdout/stderr, send commands)
- **Feature 002**: Chat interface (display conversation history, message input, markdown rendering)
- **Feature 003**: Tool visualizer (show tool calls in sidebar, file diffs, execution status)

**Dependencies**:
- Feature 001 is foundation (connects to CLI agent)
- Features 002 and 003 both depend on Feature 001's message stream
- Features 002 and 003 are **independent** (build in parallel)

**Stack**: Python FastAPI + WebSocket + React, or Node.js + Socket.io + Vue

**Why this works**: Meta project - you're building a UI for the very tools you've been using! CLI connector provides message stream, chat interface displays conversation, tool visualizer shows what's happening. Both consume the same stream but render different views.

**Bonus**: You've been using Claude Code CLI throughout this chapter. Now you understand it well enough to build a UI for it.

---

### Verify Parallelizability

Before starting, answer:
1. Can Feature 001 be built independently? (Should be YES)
2. Can Features 002 and 003 be built simultaneously? (Should be YES)
3. Are integration contracts clear? (Can you define what each feature needs?)

If all YES, proceed to execution.

#### 💬 AI Colearning Prompt
> "Why is decomposition quality more important than implementation speed in multi-agent orchestration? What breaks when decomposition is poor but agents are fast?"

---

## Part 2: Execute Full Workflow (150 minutes)

Use the complete workflow from Lessons 1-3.

### Phase 1: Setup & Specification (30 minutes)

**Step 1: Create Worktrees**

```bash
git worktree add worktrees/feature-001-[name] -b feature-001-[name]
git worktree add worktrees/feature-002-[name] -b feature-002-[name]
git worktree add worktrees/feature-003-[name] -b feature-003-[name]
```

**Step 2: Write Specs (In Parallel)**

Open 3 terminals, one per worktree:

```bash
# Terminal 1 (feature-001)
/sp.specify

# Terminal 2 (feature-002)
/sp.specify

# Terminal 3 (feature-003)
/sp.specify
```

**What to include in each spec**:
- User stories explaining feature purpose
- Success criteria (how will you know it works?)
- Integration contract: What data does this feature need? What does it provide?

**Time estimate**: 20-30 minutes total (all 3 in parallel)

---

### Phase 2: Planning & Tasks (30 minutes)

**In each worktree, run**:

```bash
# All 3 terminals in parallel
/sp.plan
/sp.tasks
```

**Observe**: Are plan sizes similar? Or is one much larger? Unbalanced plans indicate poor decomposition.

**Time estimate**: 25-30 minutes total (all 3 in parallel)

---

### Phase 3: Implementation (90 minutes)

**In each worktree, run**:

```bash
# All 3 terminals in parallel
/sp.implement
```

**Monitor progress**: Check every 10-15 minutes. If one fails, debug while others continue.

**Expected time**: ~60-90 minutes (longest feature determines total time)

---

### Phase 4: Integration (30 minutes)

Merge in dependency order:

```bash
# Step 1: Merge Feature 001
git checkout main
git merge feature-001-[name]
# Run tests

# Step 2: Merge Feature 002
git merge feature-002-[name]
# Resolve conflicts (should be minimal)
# Run tests

# Step 3: Merge Feature 003
git merge feature-003-[name]
# Resolve conflicts (should be minimal)
# Run tests

# Step 4: End-to-end testing
# Test that all 3 features work together
```

**Document conflicts**: How many? In which files? What did they reveal?

Clean merges (0-1 conflicts) = excellent decomposition.

### Phase 5: Cleanup Worktrees (5 minutes)

After merging, remove worktrees to keep your repository clean:

```bash
# Remove each worktree (after branches are merged)
git worktree remove worktrees/feature-001-[name]
git worktree remove worktrees/feature-002-[name]
git worktree remove worktrees/feature-003-[name]

# Verify removal
git worktree list
# Should only show main repository

# Optional: Delete merged branches
git branch -d feature-001-[name]
git branch -d feature-002-[name]
git branch -d feature-003-[name]
```

**Best practice**: Remove worktrees immediately after merging to prevent clutter.

---

## Part 3: Measurement (30 minutes)

Quantify your productivity gains.

### Time Tracking Worksheet

| Phase | Sequential Estimate | Parallel Actual | Notes |
|-------|---------------------|-----------------|-------|
| Specification (all 3) | 60 min | ___ min | |
| Planning & Tasks (all 3) | 75 min | ___ min | |
| Implementation (all 3) | 180 min | ___ min | Longest feature determines time |
| Integration | 30 min | ___ min | |
| Testing | 30 min | ___ min | |
| **TOTAL** | **375 min** | **___ min** | **___x speedup** |

**Calculate speedup**:
```
Speedup = Sequential Total ÷ Parallel Total
        = 375 ÷ [your actual]
```

**Target**: 2-3x speedup for well-decomposed systems.

**If speedup < 2x**, likely causes:
- Merge conflicts (poor decomposition)
- Unbalanced features (one took much longer)
- Hidden dependencies (integration issues)

---

## Part 4: Documentation (30 minutes)

Create portfolio-worthy documentation.

### README.md (Required)

Include:
- **What you built** (3 features, purpose)
- **How you decomposed it** (dependency diagram, integration contracts)
- **Results** (time tracking table, speedup calculation, merge conflicts)
- **Key insight** (what made parallelization possible?)
- **How to run** (installation, tests, startup)

### Reflection (Required)

Write 2-3 paragraphs answering:

1. **What made your features parallelizable?**
   - Which design decisions enabled parallel work?
   - What would have broken if decomposition was poor?

2. **What did merge conflicts reveal?**
   - 0 conflicts = excellent decomposition
   - 3+ conflicts = features weren't independent enough

3. **How does this scale?**
   - What changes at 5-7 features?
   - How does this apply to human teams?

---

## Part 5: Portfolio Narrative (20 minutes)

Craft your 2-minute pitch for employers.

### What NOT to Say

❌ "I used git worktrees to run 3 sessions in parallel"
❌ "I learned terminal management"

These are **tool descriptions**, not strategic insights.

### What TO Say

✅ "I demonstrated decomposition thinking — breaking complex systems into parallelizable units with clear contracts. I built a [project name] with 3 independent features, achieved [X]x speedup with zero quality sacrifice. More importantly, I learned that good specs eliminate coordination overhead. This pattern scales from 3 AI agents to 15-person teams."

### Your Pitch Structure

**Opening**: "I built [project] using decomposition thinking — a pattern that enables 1 person to coordinate multiple autonomous agents (or teams) with minimal coordination overhead."

**Design**: "The critical insight was that the 3 features had clear dependencies. I designed specs with explicit integration contracts, so each feature could be developed autonomously."

**Execution**: "We built all 3 features in parallel. Implementation time: [X] minutes vs [Y] minutes sequential = [Z]x speedup. [N] merge conflicts, same test coverage, zero quality sacrifice."

**Strategic Insight**: "This isn't about coding faster — it's about orchestrating teams. The same decomposition thinking coordinates AI agents, human developers, or distributed teams. Good decomposition eliminates meetings and lets teams work asynchronously."

**Closing**: "I'm a creative orchestrator. I can decompose complex systems and coordinate autonomous teams at scale."

**Practice this out loud.** Record yourself. Refine.

#### 🎓 Expert Insight
> In AI-native development, your portfolio isn't about code you typed—it's about systems you orchestrated. Decomposition thinking, specification quality, and coordination capability are the skills employers value. Metrics matter: "coordinated 3-agent teams with 2.5x speedup and 1 merge conflict" demonstrates strategic orchestration, not syntax memorization.

---

## Completion Checklist

### Code
- [ ] 3 features implemented and working
- [ ] All tests passing
- [ ] 0-2 merge conflicts (documented)
- [ ] Clean commit history showing parallel work

### Documentation
- [ ] README.md with overview and results
- [ ] Time tracking worksheet with actual measurements
- [ ] Reflection essay (2-3 paragraphs)
- [ ] Portfolio narrative (2-minute pitch)

### GitHub Repository
- [ ] Public repository created
- [ ] Clear file structure
- [ ] CI/CD tests passing (optional but recommended)
- [ ] "How to Run" section works

---

## Extension: 5-Agent Capstone (Optional)

Want to prove you can coordinate 5-7 agents? Extend your capstone project.

### 5-Feature System Design

Choose one project and add 2 more features:

**Task Management App** → Add:
- **Feature 004**: Task categories (organize tasks by projects/tags)
- **Feature 005**: Activity log (track all task changes with timestamps)

**Agent UI** → Add:
- **Feature 004**: Session history (list past conversations, search/filter)
- **Feature 005**: Settings panel (customize UI themes, notification preferences)

### Extension Requirements

- Use contracts from Lesson 5 to define integration points
- Use completion hooks from Lesson 5 for async coordination
- All 5 features must integrate cleanly (0-4 merge conflicts acceptable)
- Measure and document 3-5x speedup

### Modified Time Estimates

| Phase | Sequential (5 features) | Parallel (5 features) | Expected Speedup |
|-------|------------------------|----------------------|------------------|
| Total | 625 min (10.5 hrs) | 150-200 min (2.5-3.3 hrs) | **3-4x** |

Completing this extension proves you can coordinate 5-7 agents—the skill level described in the chapter spec.

---

## Scaling Reflection: From 3 to 5-7 Agents

You built a 3-feature system. What changes at larger scale?

| Aspect | 3 Features | 5-7 Features |
|--------|-----------|--------------|
| Specification time | 20-30 min | 40-60 min |
| Merge conflicts | 0-1 | 2-4 |
| Integration testing | 20 min | 45-60 min |
| Manual coordination | Easy | Gets tedious |
| Solution | Manual worktrees | Integration contracts (Lesson 5) + orchestration (Lesson 6) |

**Key insight**: At 3 features, manual coordination works. At 5-7 features, you need explicit contracts. At 10-15 features, manual coordination becomes impossible — you need orchestration.

**The bottleneck is ALWAYS decomposition**, not tools.

---

## Congratulations!

You've completed Chapter 32: *AI Orchestra - Agent Teams Manager*.

**You learned**:
- Decomposition thinking (break complex problems into parallelizable units)
- Task management (coordinate 3 agents/workflows simultaneously)
- Integration contracts (define clear boundaries between features)
- Scaling patterns (what works at 3 vs 5-7 vs 10-15 agents)

**You proved**:
- You can build real systems with 2-3x productivity gains
- Clean specs eliminate coordination overhead
- Good decomposition improves code quality

**You are now**: A creative orchestrator who can coordinate autonomous teams at scale.

Not because you're a faster coder. But because you understand how **clear specs eliminate meetings**, **good decomposition enables autonomous work**, and **this pattern scales to larger teams**.

#### 🤝 Practice Exercise

> **Ask your AI**: "Review my capstone project metrics (paste time tracking and merge conflict data). Analyze what my measurements reveal about decomposition quality. Suggest what I should improve before scaling to 5-7 features."

**Expected Outcome**: Data-driven feedback on decomposition effectiveness and actionable recommendations for scaling.

---

## Try With AI

Use your AI companion to refine your portfolio narrative:

```
Review my capstone project portfolio narrative. Does it emphasize
decomposition thinking and strategic capability (not tool proficiency)?

[PASTE YOUR 2-MINUTE PITCH]

Give me specific feedback on what works and what to strengthen.
```

Also ask:

```
I measured [X]x speedup with [N] merge conflicts on my 3-feature project.
What does this tell me about my decomposition quality? What would change
if I built 5-7 features?
```

**Expected**: AI feedback on narrative clarity and scaling insights based on your actual measurements.

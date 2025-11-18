---
title: "Git Worktrees & Parallel Specifications"
chapter: 32
lesson: 1
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Git Worktrees for Parallel Development"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create, list, navigate, and remove multiple git worktrees; understand isolation mechanism; perform basic worktree operations independently with scaffolding"

  - name: "Parallel Specification Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write specifications for independent features simultaneously; design integration contracts between specs; coordinate parallel work through shared constitution; identify when features are parallelizable"

  - name: "Decomposition Thinking: Part 1"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can recognize what makes features parallelizable; understand that clear feature boundaries enable autonomous work; identify when decomposition is effective"

learning_objectives:
  - objective: "Set up and manage 3 git worktrees for simultaneous feature development"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successful creation and navigation of 3 worktrees; output of `git worktree list` showing all 3"

  - objective: "Run parallel `/sp.specify` workflows across 3 features without conflicts"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Completion of 3 parallel specifications in feature-specific directories with PHR auto-routing verified"

  - objective: "Design integration contracts that define how features connect"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Documentation of integration points between 3 specs showing data formats and dependencies"

  - objective: "Understand how parallel specification design enables 7-9 agent orchestration"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Reflection on scaling from 3 to 7-9 agents; analysis of what changes in workflow"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (git worktree, feature isolation, parallel specification, integration contracts, PHR routing, feature numbering, decomposition thinking) within A2/B1 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Scale to 5+ worktrees simultaneously; design decomposition strategy for large monorepo; explore worktree integration with CI/CD pipelines"
  remedial_for_struggling: "Start with 2 worktrees instead of 3; use provided integration contract template; pair with peer during parallel specification phase"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/002-chapter-32-redesign/spec.md"
created: "2025-11-06"
last_modified: "2025-11-06"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Git Worktrees & Parallel Specifications

## The Problem: Sequential Bottleneck

You're building a simple **Assignment Grader** with 3 features:
1. **Upload** - Students upload assignments
2. **Grade** - Grading logic and scoring
3. **Feedback** - Generate and display feedback

**Traditional sequential approach:**
- Write Upload spec (30 min) → Wait → Write Grade spec (30 min) → Wait → Write Feedback spec (30 min)
- **Total time: 90 minutes**
- Only one person/agent working at a time
- Everyone else blocked

**Parallel approach:**
- Write all 3 specs simultaneously in isolated workspaces
- **Total time: 30 minutes**
- 3 agents working concurrently
- 67% time reduction

This lesson teaches you **orchestration mindset**: how to manage multiple AI agent sessions working in parallel. You'll learn strategic thinking, not command memorization.

---

## Git Worktrees: Quick Concept

A **git worktree** is an isolated workspace connected to the same repository.

**Analogy:** Multiple desks in one office. Each desk has its own work in progress, but everyone uses the same filing cabinet (git history).

**The structure:**
```
assignment-grader/.git/          <- Shared git repository
grader-upload/                   <- Worktree 1 (branch: feature-001-upload)
  specs/feature-001/
grader-grade/                    <- Worktree 2 (branch: feature-002-grade)
  specs/feature-002/
grader-feedback/                 <- Worktree 3 (branch: feature-003-feedback)
  specs/feature-003/
```

**Key property: Complete isolation**
- Changes in worktree 1 don't affect worktree 2 or 3
- Each worktree on different branch
- No merge conflicts during development
- All sharing same git history

**When you need worktrees:**
- Multiple features being specified/built in parallel
- Want to avoid branch switching chaos
- Need true isolation between work streams

**Time savings at scale:**
| Features | Sequential | Parallel | Speedup |
|----------|-----------|----------|---------|
| 3 features | 90 min | 30 min | **3x** |
| 7 features | 210 min | 30 min | **7x** |
| 10 features | 300 min | 30 min | **10x** |

This is why worktrees matter for AI agent orchestration.

#### 💬 AI Colearning Prompt
> "Why do git worktrees use shared history but isolated file systems? What would break if they shared file systems too?"

---

## Let AI Set Up Your Worktrees

You understand the concept. Now let AI handle the complex setup.

**Open Claude Code (or your AI assistant):**

```bash
cd your-project
claude
```

**Tell your AI:**

```
I need to work on 3 features in parallel for an Assignment Grader:
1. Upload feature (feature-001-upload) - students upload assignments
2. Grade feature (feature-002-grade) - grading logic and scoring
3. Feedback feature (feature-003-feedback) - generate and display feedback

Create a new .trees directory and inside it set up 3 git worktrees for these features:
- Create worktree directories
- Initialize feature branches
- Add specs/ folder in each
- Verify isolation

Show me what you're doing at each step and confirm when complete.
```

**What AI will do:**
1. Check your current git state
2. Create 3 isolated worktrees with proper structure
3. Initialize branches (feature-001-upload, feature-002-grade, feature-003-feedback)
4. Create directory structure in each
5. Run verification commands
6. Show you `git worktree list` output

**While AI works, observe the approach:**
- How does AI break the task into steps?
- What safety checks does AI run before executing?
- How does AI verify each worktree is isolated?
- What commands does AI choose and why?

**This is the mindset you're learning:**

```
Your Goal → AI Decomposes → AI Executes → AI Validates → You Verify
```

You're not learning git commands. You're learning how to orchestrate AI to solve problems.

#### 🎓 Expert Insight
> In AI-native development, you don't memorize git worktree commands—you understand when parallel isolation solves coordination problems. Your job: recognize when decomposition enables parallelization, then orchestrate AI to handle the mechanics.

**After AI finishes, verify the result:**

```
Show me proof all 3 worktrees are isolated:
1. Run git worktree list
2. Show branches in each worktree
3. Demonstrate that changing a file in worktree 1 doesn't affect worktrees 2-3
```

**Expected validation output:**
```
/path/to/grader-upload      abc1234 [feature-001-upload]
/path/to/grader-grade       def5678 [feature-002-grade]
/path/to/grader-feedback    ghi9012 [feature-003-feedback]
```

Each worktree on its own branch, completely isolated.

---

## Adapting /sp.specify for Worktrees

There's one problem: **`/sp.specify` creates NEW branches by default**.

But we already created branches when setting up worktrees (feature-001-upload, feature-002-grade, feature-003-feedback)!

**The conflict:**
- Worktrees: Already on feature branches
- `/sp.specify`: Tries to create new branches
- Result: Workflow breaks

**The solution: Customize your local command**

SpecKit Plus commands are just markdown files you can edit. Let's adapt `/sp.specify` to detect existing branches.

### Step 1: Find Your Command File

The command file location depends on your AI agent:

**For Claude Code users:**
```bash
.claude/commands/sp.specify.md
```

**For other AI agents:**
```bash
# Check your agent's commands directory
# Common patterns:
.ai/commands/sp.specify.md
.aider/commands/sp.specify.md
```

### Step 2: Add Worktree Detection

Open `.claude/commands/sp.specify.md` (or equivalent) and add this **BEFORE step 1** (before "Generate a concise short name"):

```markdown
0. **Detect existing feature branch** (for git worktree workflows):

   a. Check current branch:
   
      CURRENT_BRANCH=$(git branch --show-current 2>/dev/null)

      # Check if branch matches: NNN-name or feature-NNN-name
      if [[ "$CURRENT_BRANCH" =~ ^([0-9]+-|feature-[0-9]+-).*$ ]]; then
        echo "EXISTING_FEATURE_BRANCH_DETECTED"
      fi

   b. If existing feature branch detected:
      - Extract feature number and short-name from branch name
      - Set `FEATURE_DIR=specs/{number}-{name}`
      - Set `SPEC_FILE={FEATURE_DIR}/spec.md`
      - **Skip steps 1-2** (branch already exists)
      - **Proceed to step 3** (load spec template)

      Example:
      - Current branch: `feature-001-upload`
      - Extract: number=001, name=upload
      - Create spec at: `specs/001-upload/spec.md`
      - Do NOT create new branch

   c. If NO existing feature branch:
      - Proceed normally with steps 1-2 (create new branch)
```

### Step 3: Update Step 2 (Branch Creation)

Find step 2 in your command file ("Check for existing branches before creating new one") and add this at the beginning:

```markdown
2. **Check for existing branches before creating new one**:

   **If step 0 detected existing branch**:
   - Skip this entire step (branch already exists)
   - Proceed to step 3

   **Otherwise** (standard workflow):

   a. First, fetch all remote branches...
   [rest of existing step 2...]
```

### Step 4: Test Your Modification

**Test 1: Verify worktree workflow**

```bash
# In worktree grader-upload (on branch feature-001-upload)
cd grader-upload
claude

# Tell Claude:
/sp.specify "Assignment upload feature: students upload files"
```

**Expected behavior:**
```
✓ Detected existing branch: feature-001-upload
✓ Using feature number: 001
✓ Using short name: upload
✓ Creating spec at: specs/001-upload/spec.md
✓ Skipping branch creation (already on feature branch)

Proceeding to specification creation...
```

**Test 2: Verify standard workflow still works**

```bash
# In main project (on main/master branch)
cd my-project
claude

/sp.specify "Add user authentication"
```

**Expected behavior:**
```
(No existing feature branch detected)
Generating short name: user-auth
Checking for existing branches...
Creating new branch: 002-user-auth
```

### Why This Works

You're teaching your AI agent to be **context-aware**:

- **In worktree** → Already on feature branch → Skip creation
- **Not in worktree** → Create new branch → Standard flow

This is a key skill: **adapting tools to match your workflow**, not forcing workflows to match tools.

---

## Running Parallel Specifications

Now run specs in all 3 worktrees simultaneously. You'll manage 3 Claude agent sessions in parallel.

**Prerequisites:**
- ✅ 3 worktrees created (grader-upload, grader-grade, grader-feedback)
- ✅ `/sp.specify` command modified to detect existing branches (previous section)

**Step 1: Open 3 terminal windows**

Open 3 separate terminal windows or tabs. How you do this depends on your terminal application.

> **Tip**: Most terminals support tabs (Cmd+T on Mac, Ctrl+Shift+T on Linux) or split panes. Use whatever works for you.

Arrange them so you can see all 3 (side by side, or tiled).

**Step 2: Navigate each terminal to its worktree**

**Terminal 1:**
```bash
cd grader-upload
claude
```

**Terminal 2:**
```bash
cd grader-grade
claude
```

**Terminal 3:**
```bash
cd grader-feedback
claude
```

Now you have 3 Claude agent sessions running, each in its own worktree.

**Step 3: Run /sp.specify in each Claude session**

Start these within 1 minute of each other to run truly in parallel:

**Terminal 1 (Claude session):**
```
/sp.specify "Assignment upload feature: students can upload files, validate formats, store submissions"
```

**Terminal 2 (Claude session):**
```
/sp.specify "Grading feature: apply rubric, calculate scores, handle edge cases"
```

**Terminal 3 (Claude session):**
```
/sp.specify "Feedback feature: generate feedback based on scores, display to students"
```

**What you'll observe:**
- ✅ Each agent detects existing branch (feature-001-upload, etc.)
- ✅ Each skips branch creation automatically
- ✅ Each creates spec in correct location (specs/001-upload/, specs/002-grade/, specs/003-feedback/)
- ✅ All 3 agents working simultaneously
- ✅ Complete isolation between agents - no conflicts

**Skills you're learning:**
- Managing multiple AI agent sessions in parallel
- Coordinating concurrent work across isolated environments
- Validating that parallel execution works correctly

**Time tracking:**
- Note when you started all 3 (should be within 1 minute)
- Check completion times in each terminal
- Verify total time: ~30 minutes (not 90)

**At scale:**
| Agents | Sequential Time | Parallel Time | Speedup |
|--------|----------------|---------------|---------|
| 3 agents | 90 min | 30 min | **3x** |
| 7 agents | 210 min | 30 min | **7x** |
| 10 agents | 300 min | 30 min | **10x** |

This manual coordination prepares you for Lesson 2, where you'll learn to manage planning and tasks in parallel.

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a 4th worktree for feature-004-notifications. Then explain how git ensures the 4 worktrees stay synchronized with shared history while maintaining file isolation."

**Expected Outcome**: Understanding of git's isolation mechanism and practical experience adding worktrees to existing parallel workflows.

---

## Try With AI

Now use AI to validate your understanding and explore scaling.

**Validate worktree setup:**

```
I just set up 3 git worktrees for an Assignment Grader:
- grader-upload (upload feature)
- grader-grade (grading logic)
- grader-feedback (feedback generation)

Run git worktree list and verify:
1. All 3 worktrees exist
2. Each is on its own branch
3. They're completely isolated

Then demonstrate isolation by:
- Creating a file in worktree 1
- Verifying it doesn't appear in worktrees 2-3
```

**Explore scaling:**

```
I'm currently managing 3 Claude agent sessions with git worktrees.
How would I scale this to 10 agents working on 10 features?

What changes:
1. How many terminal windows would I need?
2. How would I keep track of which session is which?
3. What tools could help manage 10 parallel sessions?
4. At what point does manual coordination become unmanageable?
```

**Your job:** Understand the scaling challenges NOW (with 3 agents) to prepare for LATER (automated orchestration in lessons 6-8).
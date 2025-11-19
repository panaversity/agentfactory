---
title: "Progressive Loading Strategy"
description: "Design three-tier loading framework to prevent degradation through intelligent context allocation"
sidebar_label: "Progressive Loading Strategy"
sidebar_position: 3
chapter: 11
lesson: 3
duration_minutes: 50
proficiency: "B1"
concepts: 6

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Progressive Loading Design"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student designs Foundation/Current/On-Demand loading strategy for projects"

learning_objectives:
  - objective: "Design three-tier progressive loading framework for project context"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Loading strategy document with tier allocation and rationale"

  - objective: "Balance preloading efficiency with context waste through collaborative iteration"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Optimization analysis of loading strategy tradeoffs"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (three loading tiers, allocation strategy, context budget, preloading tradeoffs) within B1 limit of 7-10 ✓"

differentiation:
  extension_for_advanced: "Research context caching strategies; analyze dynamic loading patterns in production AI systems; design adaptive loading based on task type detection"
  remedial_for_struggling: "Focus on binary categorization: Separate files into 'Always need' vs 'Sometimes need' before learning three-tier framework"
---

# Progressive Loading Strategy

In Lessons 1-2, you learned to track context utilization manually and recognize degradation symptoms before they become severe. You practiced estimating tokens, identifying warning thresholds, and diagnosing issues from conversation transcripts.

Now it's time to shift from reactive diagnosis to **proactive strategy**. Instead of waiting for degradation and then fixing it, you'll design loading strategies that prevent it in the first place.

In this lesson, you'll work with AI to design a three-tier progressive loading framework that manages context allocation intelligently: **Foundation** (always loaded), **Current** (task-specific), and **On-Demand** (fetched as needed). Through collaborative iteration, you'll discover how to balance preloading efficiency with context waste, adapting strategies to your project's unique constraints.

## The Context Allocation Problem

Imagine starting a documentation project with 50 files. Your instinct might be: "Load everything so AI has full context." But loading all 50 files immediately creates problems:

1. **Context saturation**: You've used 60% of context before even starting work
2. **Wasted capacity**: Only 10 files are relevant to today's task
3. **On-demand handicap**: When you need a reference file mid-session, there's no room to load it

**The alternative**: Load selectively. But how do you decide what to load when?

This is where **progressive loading strategy** becomes essential.

## The Three-Tier Loading Model

Progressive loading divides your context into three tiers, each serving a different purpose:

### Tier 1: Foundation (Always Loaded)

**Purpose**: Core patterns that apply to **all** tasks in this project.

**What belongs here**:
- Project conventions (CLAUDE.md)
- System architecture (architecture.md)
- Key decisions (decisions.md)
- Critical reference files that guide every task

**Size target**: 10-15% of context window (~20-30K tokens for Claude Sonnet)

**Why this matters**: Foundation provides consistent context across sessions. Without it, AI starts every session from zero, requiring you to re-establish patterns repeatedly.

**Example Foundation for documentation project**:
```markdown
Foundation Files (Always Load):
- CLAUDE.md: Markdown conventions, heading levels, link structure
- chapter-index.md: Overall book structure, cross-references
- style-guide.md: Writing patterns, terminology standards

Estimated tokens: ~25K (12.5% of 200K context)
```

### Tier 2: Current (Task-Specific)

**Purpose**: Files relevant to **today's specific task**.

**What belongs here**:
- Files you're actively editing
- Related documentation that informs this work
- Dependencies that this task touches

**Size target**: 20-30% of context window (~40-60K tokens)

**Why this matters**: Current tier provides focused context for the work at hand without overwhelming the session with everything that *might* be relevant.

**Example Current for specific task**:
```markdown
Task: Write user guide for Chapter 11 (Markdown syntax)

Current Files (Task-Specific):
- chapter-09-outline.md: Structure for this chapter
- markdown-syntax-reference.md: Technical details
- previous-chapter-examples.md: Patterns to follow

Estimated tokens: ~50K (25% of 200K context)

Total so far: Foundation (25K) + Current (50K) = 75K (37.5%)
```

### Tier 3: On-Demand (Fetch As Needed)

**Purpose**: Reference files loaded only when AI explicitly requests them.

**What belongs here**:
- Examples from other chapters (fetch if AI asks "Can I see example X?")
- Technical specifications (load when needed for validation)
- Archive files (older decisions, legacy patterns)

**Size reserved**: 30% of context window (~60K tokens)

**Why this matters**: On-demand reserve prevents preloading waste. You've protected capacity for mid-session needs without committing it upfront.

**Example On-Demand strategy**:
```markdown
On-Demand Reference Library:
- Other chapter files (50+ files available)
- Historical examples
- Extended specifications

Reserve: 60K tokens (30% of context)

When to fetch:
- AI asks: "Can I see how Chapter 3 handled this?"
- You realize: "This needs validation against spec"
- Mid-session: "We need an example from earlier work"

Total capacity plan:
- Foundation: 25K (12.5%)
- Current: 50K (25%)
- Conversation: ~45K (22.5%)
- On-Demand reserve: 60K (30%)
- Safety buffer: 20K (10%)
= 200K total
```

## Collaborative Loading Strategy Design

You're working on a documentation project with this structure:

```
docs/
├── CLAUDE.md (project patterns)
├── chapter-index.md (book structure)
├── style-guide.md (writing standards)
├── 01-introduction/
│   ├── outline.md
│   ├── lesson-1.md
│   └── lesson-2.md
├── 02-fundamentals/
│   ├── outline.md
│   ├── lesson-1.md
│   └── lesson-2.md
└── ... (20 more chapters with similar structure)
```

Your current task: Write `02-fundamentals/lesson-1.md`.

### Exploring Loading Options

You start a new Claude Code session:

**Your request:**
"I'm working on a documentation project with 50+ markdown files. Today I'm writing lesson 1 for the Fundamentals chapter. What files should I load to give you the right context without wasting tokens?"

**AI's recommendation:**
"For efficient context management, I'd suggest a three-tier approach:

**Foundation (always load)**: CLAUDE.md, chapter-index.md, style-guide.md — these establish project-wide patterns

**Current (today's task)**: 02-fundamentals/outline.md, any completed lessons in chapter 02 for consistency

**On-Demand (fetch if needed)**: Examples from other chapters, only when we need specific reference patterns

This keeps your initial load around 30-40K tokens (15-20% utilization), leaving plenty of room for our conversation and any additional files we discover we need."

### Discovering Project Constraints

**Your response:**
"That makes sense as a general approach, but our style-guide.md changes frequently as we refine standards. Should it still be Foundation, or would that risk loading stale versions?"

**AI's adaptation:**
"Good point. For frequently-changing files, Foundation isn't ideal—you'd load outdated patterns. Let's adjust:

**Foundation (stable)**: CLAUDE.md (core patterns rarely change), chapter-index.md (structure is stable)

**Current (include fresh version)**: style-guide.md (fetch latest each session), 02-fundamentals/outline.md, completed lesson files

This way, style-guide.md is always current, and your Foundation stays small (~15K tokens instead of 25K)."

### Refining Through Iteration

**Your clarification:**
"Actually, for this project, chapter-index.md is critical for understanding cross-references between chapters. It's large (~10K tokens), but without it, you might suggest links that don't align with our structure. I'm willing to spend 10K of my Foundation budget there."

**AI's final recommendation:**
"Understood. Your Foundation allocation reflects what's most valuable for **your** project:

**Foundation (project-critical)**:
- CLAUDE.md: ~8K tokens (core conventions)
- chapter-index.md: ~10K tokens (cross-reference structure — critical despite size)
Total Foundation: ~18K tokens (~9%)

**Current (task-specific)**:
- style-guide.md: ~7K tokens (fresh version each session)
- 02-fundamentals/outline.md: ~5K tokens
- 01-introduction/lesson-2.md: ~6K tokens (previous lesson for consistency check)
Total Current: ~18K tokens (~9%)

**Conversation space**: ~80K tokens available (40%)
**On-Demand reserve**: ~60K tokens (30%)
**Safety buffer**: ~24K tokens (12%)

This loading strategy prioritizes what you identified as critical (chapter-index.md for cross-references) while staying well under 70% utilization."

### What Emerged from Collaboration

Through three rounds of iteration, you arrived at a loading strategy that:
- Started with AI's general recommendation (Foundation/Current/On-Demand)
- Incorporated your domain knowledge (style-guide.md changes frequently)
- Refined based on project priorities (chapter-index.md is critical despite size)

The final strategy balances efficiency (minimal Foundation) with effectiveness (critical reference loaded).

## Building Your Loading Strategy Template

Now that you've seen the collaborative pattern, create a reusable template for your own projects:

```markdown
# Context Loading Strategy for [Project Name]

## Foundation (Always Loaded)
**Purpose**: Patterns that apply to ALL tasks

1. [File 1] — [Why this is foundation] — Est: [X] tokens
2. [File 2] — [Why this is foundation] — Est: [Y] tokens

Total Foundation: ~[Z] tokens (~[%] of 200K context)

**Foundation Decision Criteria**:
- Does this apply to every task? (If no → Move to Current or On-Demand)
- Does this change frequently? (If yes → Consider Current instead)
- Is this file large? (If over 15K tokens, justify the foundation spend)

## Current (Task-Specific)
**Purpose**: Files for TODAY'S task only

For task: [Describe today's work]

1. [File A] — [Why needed for this task] — Est: [X] tokens
2. [File B] — [Why needed for this task] — Est: [Y] tokens

Total Current: ~[Z] tokens (~[%] of 200K context)

**Current Decision Criteria**:
- Is this file directly related to today's task?
- Will I actively reference or edit this today?
- Can I defer loading until I explicitly need it? (If yes → On-Demand)

## On-Demand (Reference Library)
**Purpose**: Available but not preloaded

Reference Categories:
- [Category 1]: [What's available] — Fetch when: [condition]
- [Category 2]: [What's available] — Fetch when: [condition]

Reserved capacity: ~[X] tokens (~30% of context)

**On-Demand Decision Criteria**:
- Is this "might need" rather than "definitely need"? (On-Demand)
- Can AI request this mid-session? (On-Demand)
- Is this a backup reference? (On-Demand)

## Context Budget Summary
- Foundation: [X]K tokens ([%])
- Current: [Y]K tokens ([%])
- Conversation estimate: [Z]K tokens ([%])
- On-Demand reserve: [A]K tokens ([%])
- Safety buffer: [B]K tokens ([%])
= 200K total

**Validation**: Total preload (Foundation + Current) should be < 70K tokens (35%)
```

## Practice Exercises

### Exercise 1: Classify Files into Tiers

You're working on a web development project with these files:

```
project/
├── README.md (project overview, 3K tokens)
├── architecture.md (system design, 8K tokens)
├── CLAUDE.md (coding conventions, 5K tokens)
├── api/
│   ├── auth.py (authentication routes, 12K tokens)
│   ├── users.py (user management, 10K tokens)
│   └── products.py (product catalog, 15K tokens)
├── tests/
│   ├── test_auth.py (8K tokens)
│   └── test_users.py (7K tokens)
└── docs/
    ├── api-reference.md (20K tokens)
    └── deployment-guide.md (12K tokens)
```

**Today's task**: Add password reset functionality to `api/auth.py`.

**Your assignment**:
1. Classify each file into Foundation, Current, or On-Demand
2. Calculate total Foundation + Current token usage
3. Justify your Foundation choices (why always-loaded?)
4. Identify which On-Demand files you might fetch mid-session

### Exercise 2: Optimize an Overloaded Strategy

A colleague shows you their loading strategy:

```
Foundation (Always):
- CLAUDE.md: 5K tokens
- architecture.md: 8K tokens
- README.md: 3K tokens
- api-reference.md: 20K tokens
- deployment-guide.md: 12K tokens
- All test files: 15K tokens combined

Total Foundation: 63K tokens (31.5% of context)

Current (Task-specific):
- auth.py: 12K tokens
- users.py: 10K tokens

Total Current: 22K tokens

TOTAL PRELOAD: 85K tokens (42.5%)
```

**Problems**:
- Foundation is too large (should be 10-15%, currently 31.5%)
- Some files in Foundation aren't needed for every task

**Your assignment**:
1. Identify which Foundation files should move to Current or On-Demand
2. Redesign the loading strategy to reduce Foundation to 15-20K tokens
3. Explain your reasoning for each reclassification

### Exercise 3: Design for Multi-Task Session

You're planning a session with THREE tasks:

**Task A**: Fix bug in authentication (needs: auth.py, test_auth.py)
**Task B**: Update API documentation (needs: api-reference.md, auth.py, users.py)
**Task C**: Write deployment instructions (needs: deployment-guide.md, architecture.md)

**Your assignment**:
1. Can you use the same Foundation/Current split for all three tasks?
2. If not, should you:
   - Create three separate sessions (isolated tasks)?
   - Restart session with new Current files between tasks?
   - Load everything upfront (risking high utilization)?
3. Design the optimal strategy with justification

## Try With AI

Now practice designing progressive loading strategies collaboratively with Claude Code.

### Setup
Open Claude Code. Have your project structure ready (list of files with estimated sizes).

### Prompt Set

**Prompt 1 — Initial Strategy Design:**
```
I'm working on a [project type] with [N] files. Here's my project structure:
[List key files and sizes]

Today's task: [Describe specific work]

Help me design a Foundation/Current/On-Demand loading strategy. Which files should always be loaded (Foundation), which are specific to today's task (Current), and which should wait for explicit need (On-Demand)?
```

**Prompt 2 — Constraint Teaching:**
```
Your Foundation suggestion included [File X], but that file changes frequently in our workflow. For files that update often, should they be in Foundation or Current? How does that affect the strategy?
```

**Prompt 3 — Priority Clarification:**
```
Actually, [File Y] is critical for understanding our project's cross-references, even though it's large (~[Z] tokens). I'm willing to include it in Foundation despite the size. How should we adjust the budget to accommodate this?
```

**Prompt 4 — Strategy Validation:**
```
Here's my final loading strategy:
[Paste your Foundation/Current/On-Demand plan with token estimates]

Validate this:
- Is my Foundation too large? (Should be under 20K tokens)
- Is my Current allocation reasonable for the task?
- Have I reserved enough On-Demand capacity? (30% minimum)
- What's my total preload percentage? (Should be under 35%)
```

### Expected Outcomes

Through collaborative iteration, you should:
- Design a loading strategy tailored to your project's constraints
- Understand **why** certain files belong in each tier
- Discover tradeoffs you hadn't considered (frequently-changing Foundation files, oversized critical files)
- Arrive at a strategy that balances efficiency with effectiveness

**Safety Note**: Trust your domain knowledge. If AI suggests moving a critical file to On-Demand but you know you'll reference it constantly, override that suggestion. You understand your project better than AI's general recommendations.


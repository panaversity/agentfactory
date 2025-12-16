---
title: "Tool Selection Framework"
description: "Compare Claude Code and Gemini CLI capabilities to select the right tool for different development scenarios"
sidebar_label: "Tool Selection Framework"
sidebar_position: 7
chapter: 11
lesson: 7
duration_minutes: 75
proficiency: "B1"
concepts: 6

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "AI Tool Selection Decision Framework"
    proficiency_level: "B1"
    category: "Intelligence Design"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student evaluates task characteristics and selects appropriate AI tool (Claude Code vs Gemini CLI)"

learning_objectives:
  - objective: "Compare Claude Code and Gemini CLI across context window, reasoning depth, and workflow integration"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Tool comparison matrix with capability analysis"

  - objective: "Apply decision framework to select appropriate tool for development scenarios"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Tool selection decisions for 5 development scenarios with justification"

  - objective: "Create reusable tool-selection-framework skill using Persona + Questions + Principles"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Skill document demonstrating decision framework pattern"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (tool capabilities, context tradeoffs, reasoning depth, file selection, task classification, sequential tool use) within B1 limit of 7-10 ✓"

differentiation:
  extension_for_advanced: "Research additional AI coding tools (GitHub Copilot, Cursor, Aider); create comprehensive tool selection matrix across 10+ dimensions; design automated tool recommendation system"
  remedial_for_struggling: "Focus on binary decision: 'Do I need to load 100+ files?' If yes → Gemini CLI, if no → Claude Code, before learning complex selection criteria"
---

# Tool Selection Framework — Claude Code vs Gemini CLI

## The Problem You're About to Solve

You're facing a developer's dilemma.

Your team just inherited a massive codebase: **150,000 lines of legacy Python**. The previous team documented nothing. The architecture is unclear. Before you can modernize it, you need to **understand what you're looking at**.

You have two tools available:

**Option A: Claude Code**
- Deep reasoning capability
- Focused development workflow
- But: Limited context window (200,000 tokens standard)

**Option B: Gemini CLI**
- Massive context window (2 million tokens)
- Can load entire large codebases at once
- Different interface, less IDE integration

**The Dilemma**: Do you use Claude Code's superior reasoning for architectural analysis? Or Gemini CLI's context breadth to see the full picture?

**Different answer for different situations.**

This lesson teaches you to **recognize task characteristics, evaluate tool capabilities, and select the right tool for your situation** — sometimes Claude Code, sometimes Gemini CLI, sometimes both in sequence.

This is Layer 2-3 Hybrid: **AI Collaboration (evaluating options) + Intelligence Design (creating reusable framework)**.

## Tool Comparison: Verified 2025 Specifications

Understanding your tool options requires accurate, current specifications.

### Claude Code (Anthropic)

| Capability | Standard | Extended (Beta) |
|-----------|----------|---|
| **Context Window** | 200,000 tokens | 1,000,000 tokens (tier 4+) |
| **Output Capacity** | 64,000 tokens | 64,000 tokens |
| **Reasoning Depth** | Superior for complex architectural decisions | Superior |
| **File Selection** | Progressive loading (you choose files) | Progressive loading |
| **Typical Session Length** | 1-4 hours (depending on file size) | 4-8+ hours |
| **Cost Model** | Input + output tokens billed | Input + output tokens billed |

**What 200K tokens means**:
- ~500 pages of documentation
- ~50 typical Python files (2000 lines each)
- ~100+ typical JavaScript files
- Complete single-module projects

**What 1M tokens means (extended)**:
- Multiple large modules
- Complete system documentation
- Extended refactoring across modules
- Multi-service architecture exploration

**Best for**:
- Focused development on specific features
- Deep architectural reasoning about complex systems
- Long sessions with persistent context (using memory files from Lesson 6)
- When you need explicit control over which files are loaded

### Gemini CLI (Google)

| Capability | Standard | Extended |
|-----------|----------|---|
| **Context Window** | 128,000 tokens | 2,000,000 tokens |
| **File Handling** | Load files specified at startup | Load entire large codebases |
| **Reasoning Approach** | Pattern analysis across files | Whole-system pattern analysis |
| **File Selection** | You list files to include | Load everything, AI analyzes |
| **Typical Use** | Utility scripts, small projects | Large codebase exploration |
| **Code File Capacity** | 10,000+ lines of code | 500,000+ lines of code |

**What 2M tokens means**:
- Entire 100K+ line codebases
- Complete documentation + code
- Multiple services + test suite
- Full architectural pattern analysis

**Best for**:
- Understanding unfamiliar large systems
- Broad pattern analysis across entire codebase
- Refactoring decisions that span multiple modules
- Legacy system exploration and documentation

### Key Differences at a Glance

| Dimension | Claude Code | Gemini CLI |
|-----------|-------------|-----------|
| **Reasoning** | Deep, step-by-step | Broad pattern analysis |
| **Context Control** | High (you choose files) | Low (load all, AI analyzes) |
| **IDE Integration** | Excellent (Claude Code IDE) | CLI-based |
| **Session Duration** | Flexible (1-8+ hours with memory) | Single-session analysis |
| **Error Recovery** | Good (can refine approach) | Good (access to full codebase) |
| **Typical Workflow** | "Implement this feature" | "Understand this codebase" |

## Decision Framework: Which Tool Should I Use?

Your choice depends on **task characteristics**: scope, complexity, reasoning need, context budget.

### Dimension 1: Task Scope — How Much Code?

**Small scope** (1-5 files, &lt;10K lines):
- **Choose**: Claude Code
- **Why**: 200K context is more than enough. You get IDE integration and deep reasoning.
- **Example**: Implement single endpoint in FastAPI module

**Medium scope** (5-50 files, 10K-100K lines):
- **Choose**: Claude Code with progressive loading (Lesson 3)
- **Why**: Progressive loading keeps context focused. Deep reasoning for architectural decisions.
- **Example**: Refactor authentication module across 20 files

**Large scope** (100+ files, 100K+ lines):
- **Choose**: Gemini CLI for exploration, then Claude Code for implementation
- **Why**: First understand architecture with Gemini, then implement details with Claude
- **Example**: Understand inherited 150K-line legacy system, then modernize

### Dimension 2: Task Complexity — How Much Reasoning?

**Low complexity** (straightforward execution):
- **Choose**: Either tool works
- **Prefer**: Claude Code (better IDE integration)
- **Example**: Add new database column, update tests

**Medium complexity** (architectural decisions needed):
- **Choose**: Claude Code
- **Why**: Superior reasoning for tradeoff analysis
- **Example**: Design session management strategy (JWT vs sessions vs OAuth)

**High complexity** (understand patterns across entire system):
- **Choose**: Gemini CLI
- **Why**: Need to see full codebase patterns at once
- **Example**: Identify code duplication patterns across 100+ files

### Dimension 3: Context Budget — Can I Afford to Load Everything?

**Abundant context** (small codebase, simple architecture):
- **Choose**: Claude Code
- **Load**: Entire relevant codebase at once
- **Example**: 20-file project, load all files in one session

**Limited context** (large codebase, must be selective):
- **Choose**: Claude Code with memory files
- **Load**: Essential context only (foundation + current work), use on-demand loading
- **Example**: 500-file project, load foundation (10 files) + current task (5 files), fetch others as needed

**Massive codebase** (want to see everything):
- **Choose**: Gemini CLI
- **Load**: Entire codebase
- **Example**: Legacy 100K-line system, load all files, analyze patterns

### Dimension 4: File Selection Control — Do I Need to Be Selective?

**High control needed** (I know which files matter):
- **Choose**: Claude Code
- **Why**: You manually select files, AI focuses on what you choose
- **Example**: "I only care about payment processing, ignore UI code"

**Low control acceptable** (AI should see everything):
- **Choose**: Gemini CLI
- **Why**: Load entire codebase, AI finds relevant patterns automatically
- **Example**: "Here's the codebase. Find all database-related code."

## Decision Tree: Pick Your Tool

Follow this decision tree for any AI-assisted development task:

```
Task: "I need help with my codebase"

START HERE: How many lines of code?
├─ 1-10K lines?
│  └─> Consider Claude Code (plenty of context)
│      ├─ Need deep architectural reasoning?
│      │  └─> CHOOSE: Claude Code
│      └─ Just need exploratory analysis?
│         └─> CHOOSE: Either (Claude Code for IDE, Gemini CLI if massive)
│
├─ 10K-100K lines?
│  └─> How much of it do you need in context?
│      ├─ Only specific modules (you can point to them)?
│      │  └─> CHOOSE: Claude Code with progressive loading
│      └─ Need to see patterns across whole system?
│         └─> CHOOSE: Explore with Gemini CLI first, implement with Claude Code
│
└─ 100K+ lines?
   └─> What's your goal?
       ├─ Understand architecture?
       │  └─> CHOOSE: Gemini CLI (2M context)
       │      Then: Switch to Claude Code for specific implementation
       │
       ├─ Implement focused feature?
       │  └─> CHOOSE: Claude Code with memory files
       │      Load: Foundation (10 files) + current task (5 files)
       │      Use: Lesson 6 memory files for architectural context
       │
       └─ Refactor across multiple modules?
          └─> CHOOSE: Two-phase
              Phase 1: Gemini CLI to understand current patterns
              Phase 2: Claude Code to implement refactoring
```

## Real-World Scenario Analysis

Let's apply the framework to five real situations:

### Scenario 1: Implement New Endpoint (Small Scope, High Reasoning)

**Situation**: FastAPI project with 20 files. You need to implement a new `/reports` endpoint. It requires database queries, caching decisions, and authorization logic.

**Analysis**:
- Scope: Small (20 files, only 3-5 matter)
- Complexity: Medium (architectural decisions about caching, auth)
- Context budget: Abundant (200K tokens handles everything)
- File control: High (you know exactly which files matter)

**Tool choice**: **Claude Code**
- Load files: models/Report.py, services/auth.py, services/cache.py, routes/, main.py
- Deep reasoning needed for: "Where should caching logic live?" "How to integrate with existing auth?"
- IDE integration helps: Implement, test, iterate quickly

### Scenario 2: Refactor Entire API Validation (Medium-Large Scope, Moderate Reasoning)

**Situation**: FastAPI project with 50 files. All endpoints validate input differently. You need to implement consistent validation framework. Must understand all endpoints before deciding approach.

**Analysis**:
- Scope: Large (50 files, need to see validation patterns across all 30 endpoints)
- Complexity: High (architectural decision about validation strategy)
- Context budget: Constrained (can't load all endpoints at once in depth)
- File control: Need selective (some endpoints are irrelevant)

**Tool choice**: **Two-phase approach**

*Phase 1 — Gemini CLI*: "Load entire codebase, show me all unique validation patterns across all 30 endpoints. What are the 5 most common patterns?"
- Gemini loads all 50 files
- Identifies pattern categories
- Delivers: "You have 5 validation patterns, here's the breakdown"

*Phase 2 — Claude Code*: "Based on these 5 patterns, design unified validation framework. Then implement it across all endpoints."
- Claude handles detailed reasoning about framework design
- Loads: validation/, routes/, models/ (essential files)
- Implements framework with IDE support

### Scenario 3: Understand Legacy System (Very Large, Exploration Priority)

**Situation**: Inherited 150K-line Python monolith. Previous team left no docs. You need to understand architecture before proposing modernization.

**Analysis**:
- Scope: Massive (150 files, 150K lines)
- Complexity: High (need to understand patterns, dependencies, architecture)
- Context budget: Impossible with Claude Code alone (200K tokens insufficient)
- File control: Not applicable (need to see everything)

**Tool choice**: **Gemini CLI**
- 2M context handles entire codebase
- Prompts:
  - "What's the overall architecture?" (Gemini analyzes all 150 files)
  - "What are the biggest code duplication problems?" (Cross-file analysis)
  - "What's the module dependency graph?" (System-level understanding)
- Output: Complete architectural overview
- Next step: Use Claude Code for specific refactoring once you understand what to change

### Scenario 4: Emergency Bug Fix (Unknown Module, Time Pressure)

**Situation**: Critical bug in production. Module you've never seen. 100-file codebase. Bug must be fixed in 2 hours.

**Analysis**:
- Scope: Large (100 files, but only 3-5 relevant to bug)
- Complexity: Medium (understand bug, implement fix)
- Context budget: Good if selective (Claude Code is enough if you load right files)
- File control: Critical (load only files related to bug)

**Tool choice**: **Claude Code with smart file selection**
- Work with Claude to identify which files contain the bug
- Load only those files + related dependencies
- Example loading: "Payment webhook handler, Stripe models, database transaction layer"
- Claude's reasoning helps: "Here's the bug: you're not checking signature before processing webhook"
- IDE integration: Make the fix, test, deploy quickly

### Scenario 5: Multi-Day Feature Development (Deep Work, Session Continuity)

**Situation**: Building new feature over 5 days. Requires deep architectural understanding. Lots of sessions, must maintain context across days.

**Analysis**:
- Scope: Medium-large (30 files, but evolving as you build)
- Complexity: High (architectural decisions, multiple iterations)
- Context budget: Persistent across sessions (use memory files)
- File control: High (you know which files matter for this feature)

**Tool choice**: **Claude Code + Memory Files (Lesson 6)**
- Day 1: Design architecture, document in memory files
- Day 2: Implement core logic (Claude reads memory from Day 1)
- Day 3: Add integration (Claude uses decisions from Days 1-2)
- Day 4-5: Testing and refinement (full context persistence)
- IDE integration keeps workflow smooth
- Memory files from Lesson 6 eliminate re-explanation

## Building the Reusable Skill: tool-selection-framework

You've learned to evaluate tools. Now let's create a **reusable skill** for future projects.

### What Goes Into the Skill

The **tool-selection-framework** skill captures:

1. **Verified specifications table** (Claude Code vs Gemini CLI, updatable as specs change)
2. **Decision criteria** (task scope, complexity, context budget, file control)
3. **Decision tree** (flowchart for tool selection)
4. **Scenario examples** (5+ real situations with analysis)
5. **Hybrid approach guidance** (when to use both tools sequentially)

### Using the Skill in Future Projects

**For ANY new development task**:

1. Ask yourself: "How many files? What complexity? What's my reasoning need?"
2. Reference tool-selection-framework decision tree
3. Follow framework to select appropriate tool
4. Execute with confidence (you've evaluated the decision)

This is **reusable intelligence** — the decision-making process you learned here applies to any future tool selection challenge.

## Try With AI

Ready to select the right AI tool for your development task and maximize effectiveness?

**🔍 Explore Tool Capabilities and Selection Criteria:**
> "Compare Claude Code (200K-1M context, deep reasoning, progressive loading) vs Gemini CLI (2M context, broad analysis). For each tool, explain strengths and ideal use cases. When would I choose Claude Code for a 100-file codebase vs. Gemini CLI? What about a 500-file codebase?"

**🎯 Practice Tool Selection Decision:**
> "I have a [size] codebase with [description]. My task: [specific goal]. Apply the decision framework: evaluate task scope (files/lines), complexity (reasoning depth needed), context budget (can I fit it?), and file control (selective vs. comprehensive). Recommend Claude Code or Gemini CLI with reasoning. Ask clarifying questions about my project."

**🧪 Test Two-Phase Strategy:**
> "Design a two-phase approach for: 'Understand and modernize a 150K-line legacy Python monolith'. Phase 1 (Exploration): What should I use Gemini CLI for (architectural analysis, pattern identification, dependency mapping)? Phase 2 (Implementation): What should I use Claude Code for (focused refactoring, feature implementation)? How do insights from Phase 1 inform Phase 2?"

**🚀 Apply Multi-Day Tool Strategy:**
> "Plan a 5-day development sprint for [your actual project and feature]. Which tool for Day 1 (architecture understanding)? Which for Days 2-4 (implementation)? How do I integrate memory files (Lesson 6) with tool switching? Create a day-by-day tool usage plan with session handoff protocols."

---


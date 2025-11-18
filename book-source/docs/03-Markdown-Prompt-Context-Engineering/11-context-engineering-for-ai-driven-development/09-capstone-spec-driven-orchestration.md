---
title: Lesson 9 - Capstone Spec-Driven Orchestration
sidebar_position: 9
chapter: 11
lesson: 9
learning_objectives:
  - Write complete, implementation-ready specification for context-aware system
  - Orchestrate accumulated intelligence from Lessons 1-8
  - Validate specification completeness and clarity
  - Demonstrate spec-first mastery (intent before implementation)
estimated_time: 120 minutes
proficiency_level: B1
generated_by: content-implementer v1.0.0
source_spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
created: 2025-01-18
version: 1.0.0
---

# Lesson 9: Capstone — Spec-Driven Context Orchestration

## The Problem You're About to Solve

Over 8 lessons, you've learned context engineering **techniques**:

- Recognize degradation symptoms
- Apply progressive loading
- Create checkpoints
- Isolate parallel tasks
- Build persistent memory files
- Select appropriate tools
- Diagnose and fix context problems

Now comes the **capstone challenge**: **Design a system that orchestrates ALL these techniques automatically.**

Imagine a CLI tool that:
- Monitors context utilization in real-time
- Automatically creates checkpoints when degradation appears
- Intelligently isolates parallel tasks
- Reads persistent memory files to recover context
- Recommends tool selection (Claude Code vs Gemini CLI)
- Prevents context pollution before it happens

**But here's the constraint: You can't implement it yet.**

Instead, you'll write a **complete specification** that could be handed to another developer who has never taken this course, and they could build the system from your spec alone.

This is Layer 4: **Spec-Driven Integration — Specification before implementation.**

## Understanding Specification Quality: What Makes a Spec "Implementation-Ready"?

### The Anti-Pattern: Vague Specs

Most specs fail because they're too vague:

```markdown
# Specification: Context-Aware CLI Tool

## What It Does
A CLI tool that helps manage context in AI development sessions.

## Features
- Monitors context
- Creates checkpoints
- Prevents issues
- Recommends tools

## Technology
- Python 3.10+
- FastAPI for API
- PostgreSQL for storage
- Redis for caching

## Deployment
- Docker container
- Deploy to cloud
- Users run locally

## Success
- Developers are happy
- Productivity improves
- System is reliable
```

**Problems with this spec**:
- "Help manage context" — Too vague (what exactly?)
- "Creates checkpoints" — How? When? What triggers?
- "Prevents issues" — Which issues? How detected?
- "Developers are happy" — Not measurable
- "Technology" section describes HOW, not WHAT

### The Pattern: Implementation-Ready Specs

A complete specification answers every question without dictating implementation:

```markdown
# Specification: Context-Aware Development Tool

## Intent (WHAT the tool does)
A system that monitors AI development sessions, automatically applies
context engineering strategies (progressive loading, compression, isolation,
persistence), and recommends tool selection without human intervention.

Developer benefit: Session continuity across days. No manual context management.

## Success Criteria (Measurable, Falsifiable)
- 80%+ development sessions maintain context utilization under 70%
- Zero manual re-explanation of project patterns in new sessions
- Automatic memory file updates capture 90%+ of architectural decisions
- Tool recommends appropriate tool (Claude Code vs Gemini CLI) with 85%+ accuracy

## Functional Requirements (Observable Behavior)

### Requirement 1: Real-Time Context Monitoring
When: Continuously during session
What: Track token count, utilization percentage
Measure: Report utilization every 30 seconds
Trigger: Alert when crossing 70%, 80%, 90% thresholds

### Requirement 2: Automatic Checkpoint Creation
When: Utilization exceeds 80% AND session duration > 60 minutes
What: Extract architectural decisions, progress summary, next steps
How: Generate checkpoint in <2 minutes
Size: Checkpoint &lt;600 tokens
Output: Save as CHECKPOINT.md in project

### Requirement 3: Task Isolation Detection
When: User starts new task
What: Measure semantic similarity to current context (0-100%)
Trigger: If similarity &lt;50%, recommend isolation
User choice: Accept recommendation (create new session) or continue mixed

### Requirement 4: Memory File Persistence
When: Session starts, session ends
What: Read CLAUDE.md, architecture.md, decisions.md at start
Update: Append new decisions to decisions.md at end
Conflict: If decisions file changed externally, merge (new supersedes old)

### Requirement 5: Tool Selection Recommendation
When: User describes task
What: Evaluate scope (lines of code, file count) and complexity
Recommend: Claude Code (focused work) or Gemini CLI (exploration)
Reasoning: Explain why chosen

## System Architecture (Components + Interactions, NO Implementation Details)

### Components

**Context Monitor** (Component 1)
- Responsibility: Track session token usage, utilization percentage
- Inputs: Token count from AI session, time elapsed
- Outputs: Utilization metrics, threshold alerts
- Algorithm: Calculate (tokens_used / token_limit) * 100

**Checkpoint Engine** (Component 2)
- Responsibility: Detect when checkpoint needed, generate checkpoint
- Inputs: Utilization > 80%, session duration > 60 min
- Triggers: Both conditions met → create checkpoint
- Outputs: CHECKPOINT.md file (architecture + progress + next steps)
- Algorithm: Summarize session activity in <600 tokens

**Task Similarity Analyzer** (Component 3)
- Responsibility: Evaluate if new task relates to current context
- Inputs: Current session files + new task description
- Outputs: Similarity score (0-100%), isolation recommendation
- Algorithm: Scoring criteria:
  - Same business domain? (+30 points)
  - Same database models? (+20 points)
  - Same external service? (+20 points)
  - Same API routes? (+15 points)
  - Shared test suite? (+15 points)
  - Score: Sum of matched criteria
  - Isolated if score &lt;50

**Memory File Manager** (Component 4)
- Responsibility: Load project context at session start, update at end
- Inputs: CLAUDE.md, architecture.md, decisions.md from project
- Outputs: Loaded context (ready for AI session)
- Behavior:
  - Session start: Read files, inject into session prompt
  - Session end: Append new decisions to decisions.md
  - Conflict: If decisions.md changed, merge (new decision supersedes)

**Tool Selector** (Component 5)
- Responsibility: Recommend Claude Code vs Gemini CLI
- Inputs: Task scope (codebase size), complexity assessment
- Outputs: Tool recommendation + reasoning
- Decision logic:
  - If codebase &lt;50K lines: Recommend Claude Code
  - If codebase 50K-500K lines: Evaluate complexity
    - If deep architectural reasoning needed: Claude Code
    - If broad pattern analysis needed: Gemini CLI
  - If codebase > 500K lines: Recommend Gemini CLI
  - If multi-phase (explore + implement): Recommend both (Gemini first, Claude second)

**Orchestrator** (Component 6)
- Responsibility: Coordinate all components, manage system flow
- Inputs: User commands, component outputs
- Flow:
  1. Session start → Memory Manager loads context
  2. During session → Context Monitor tracks utilization
  3. Monitor alert (80%) → Checkpoint Engine creates checkpoint
  4. New task request → Similarity Analyzer evaluates
  5. If isolation recommended → Offer new session
  6. If tool selection needed → Tool Selector recommends
  7. Session end → Memory Manager updates context
- Output: User-facing recommendations and prompts

## Progressive Loading Algorithm (From Lesson 3)

How the system applies progressive loading:

**Phase 1: Foundation Loading**
- Auto-load: Project configuration, core models, API setup
- Size: ~10-15% of codebase
- Timing: Session start

**Phase 2: Current Work Loading**
- When: User specifies task
- Logic: Identify task-relevant files (database models, routes, tests)
- Size: 20-30% of codebase
- Timing: After task identified

**Phase 3: On-Demand Fetching**
- When: AI requests file not yet loaded
- Logic: Fetch file dynamically, add to context
- Size: Variable (1-5 files per request)
- Timing: During session, as needed

**Decision Logic**:
- Foundation constant (always loaded)
- Current varies by task
- Total target: &lt;70% utilization after Foundation + Current
- Reserve: 30% for on-demand fetching

## Compression/Isolation Decision Logic (From Lessons 4-5)

**Compression Triggers** (Same task, context constraints):
```
IF utilization > 80%
AND session_duration > 60 minutes
AND task_status IN ('in_progress', 'near_completion')
THEN create checkpoint and restart session
```

**Isolation Triggers** (Different task, context separation):
```
IF similarity_score &lt;50%
AND new_task_complexity > current_context_complexity
THEN recommend isolated session

Similarity calculation:
  - shared_domain: +30
  - shared_models: +20
  - shared_service: +20
  - shared_routes: +15
  - shared_tests: +15
  - total &lt;50: ISOLATE
```

## Memory File Update Strategy (From Lesson 6)

**Reading at Session Start**:
```
Session initialization:
1. Check for CLAUDE.md in project root
2. Check for architecture.md in project root
3. Check for decisions.md in project root
4. Inject all three into session prompt as context
5. Note: "Memory files loaded, system has full project context"
```

**Updating at Session End**:
```
Session completion:
1. Identify new architectural decisions made
2. If new decision: Append ADR to decisions.md
3. If new pattern discovered: Update CLAUDE.md
4. If architecture changed: Update architecture.md
5. Commit: "Updated memory files with [N] new decisions"
```

**Conflict Resolution**:
```
When updating decisions.md:
- Check if file modified externally (compare hash)
- If modified: Merge strategy
  - New decisions from this session: Add
  - External changes: Preserve
  - Conflicts: New session's decision supersedes (newer understanding wins)
- Record: "Merged [N] external decisions into memory file"
```

## Multi-Agent Coordination (From Six Components Framework)

When task spans 3+ distinct domains:

**Multi-Agent Pattern**:
```
1. Orchestrator receives complex task (spans 3+ domains)
2. Orchestrator spawns specialized agents:
   - Agent 1: Authentication domain
   - Agent 2: Payment processing domain
   - Agent 3: Notification domain
3. Each agent receives:
   - Isolated context (only relevant to their domain)
   - Memory file extract (patterns for their domain)
   - Task description (their piece of the work)
4. Agents work in parallel
5. Orchestrator receives outputs:
   - Agent 1: 2K tokens (auth implementation)
   - Agent 2: 2K tokens (payment implementation)
   - Agent 3: 2K tokens (notification implementation)
6. Orchestrator compresses each to 1K tokens (summary)
7. Orchestrator merges into unified output
8. System continues with 3K tokens of combined work
```

## Specification Completion Checklist

Use this checklist to verify your spec is complete:

**Intent & Clarity**
- [ ] Intent statement: 1-2 paragraphs, crystal clear
- [ ] WHAT the tool does (not HOW)
- [ ] Developer benefit stated
- [ ] No implementation details (no Python, Docker, databases)

**Success Criteria**
- [ ] All criteria measurable (include numbers)
- [ ] All criteria falsifiable (can test if met or not)
- [ ] No vague terms ("works well", "improves productivity")

**Functional Requirements**
- [ ] Each requirement is testable
- [ ] Each requirement specifies: When? What? How measured?
- [ ] Triggers are explicit (not vague)
- [ ] Outputs are specific (not abstract)

**System Architecture**
- [ ] 6 components identified
- [ ] Each component has: Responsibility, Inputs, Outputs, Algorithm
- [ ] Component interactions shown (who talks to whom?)
- [ ] No implementation-specific details

**Algorithms**
- [ ] Progressive loading: Foundation/Current/On-Demand phases
- [ ] Compression trigger: Explicit conditions (80% utilization + 60min)
- [ ] Isolation trigger: Similarity score calculation
- [ ] Memory file updates: Read/write/merge logic

**Multi-Agent Coordination**
- [ ] When to spawn agents (3+ domains)
- [ ] How agents receive context (isolated + memory extracts)
- [ ] How outputs merge (compression + unification)

**Completeness**
- [ ] Spec is 3-5 pages (complete but concise)
- [ ] Zero implementation code (no Python, no SQL, no pseudo-code)
- [ ] Spec answers: "Can someone build this without asking questions?"

## Capstone Project: Write Your Specification

### Your Task

Write a complete specification for the context-aware development tool described above.

**Structure Your Spec**:

```markdown
# Context-Aware Development Tool Specification

## Intent
[1-2 paragraphs explaining WHAT the tool does and why]

## Success Criteria
[Measurable, falsifiable success criteria]

## Functional Requirements
[Observable behaviors, testable outcomes]

## System Architecture
[6 components: Context Monitor, Checkpoint Engine, Task Similarity Analyzer,
Memory File Manager, Tool Selector, Orchestrator]

## Algorithms
[Progressive loading, compression/isolation decision logic,
memory file updates, multi-agent coordination]

## Non-Goals
[What the tool does NOT do]
```

**Constraints**:

1. **Specification-Only**: ZERO implementation code
   - No Python, TypeScript, JavaScript
   - No SQL, database schemas
   - No API endpoints (can describe them in words, not code)
   - No Dockerfiles, deployment scripts
   - No pseudo-code that looks like real code

2. **Intent Before Implementation**: Focus on WHAT, not HOW
   - Specify *behaviors* (when X happens, system does Y)
   - Avoid *technologies* (no "use Redis for caching" unless essential to spec)
   - Avoid *architecture choices* (no "microservices vs monolith" unless needed for requirements)

3. **Completeness**: Answer all critical questions
   - When does each component trigger?
   - What are the inputs and outputs?
   - How are decisions made (decision logic)?
   - How do components interact?
   - What happens in error cases?

4. **Length**: 3-5 pages
   - Enough detail to build from
   - Concise enough to stay focused
   - No filler or redundancy

### Exercise: Peer Review

**Pair with a peer** (ideally from different cohort). Exchange specs.

**Reviewer's Task**:
1. Read the spec **without any other context**
2. Ask yourself: "Could I build this system from this spec alone?"
3. Document your findings:
   - ✅ What's clear and complete?
   - ❓ What's ambiguous or unclear?
   - ❌ What's missing?

**Reviewer's Questions to Ask**:
- "What triggers checkpoint creation? What's inside a checkpoint?"
- "How does similarity score work? What counts as 'shared domain'?"
- "What goes in memory files? When are they read vs updated?"
- "How do components communicate? What order do they execute in?"
- "What happens if file loading fails? Context monitor unavailable?"

**Revision Cycle**:
1. Receive peer feedback
2. Revise spec based on unclear/missing areas
3. Iterate until peer says: "Yes, I could build this"

### Validation: Test Against Learning Objectives

Your spec should demonstrate mastery of all 8 lessons:

**Lesson 1-2: Degradation Recognition** → Spec includes: "Context Monitor tracks utilization, alerts at 70%/80%/90%"

**Lesson 3: Progressive Loading** → Spec includes: "Foundation/Current/On-Demand phases, target &lt;70% utilization"

**Lesson 4: Compression** → Spec includes: "Checkpoint Engine creates checkpoint when 80% utilization + 60min"

**Lesson 5: Isolation** → Spec includes: "Task Similarity Analyzer recommends isolation when score &lt;50%"

**Lesson 6: Memory Files** → Spec includes: "Memory File Manager loads CLAUDE.md/architecture.md/decisions.md at start, updates at end"

**Lesson 7: Tool Selection** → Spec includes: "Tool Selector recommends Claude Code vs Gemini CLI based on codebase size"

**Lesson 8: Debugging** → Spec implies: "System handles failures gracefully (conflicts, missing files, unavailable components)"

**Lesson 9: Spec Quality** → Your spec IS implementation-ready (clear intent, testable requirements, complete architecture)

## Try With AI

**Setup**: You're ready to write your capstone spec.

**Prompt Set**:

**Prompt 1: Get Clarification on Complex Concepts**
```
I'm writing a specification for a context-aware CLI tool.

Help me clarify these concepts:
1. What should the "Checkpoint Engine" include? What goes inside a checkpoint?
2. How does "Task Similarity Analyzer" calculate similarity? What criteria matter?
3. What's the difference between updating CLAUDE.md vs architecture.md vs decisions.md?
4. How do the 6 components interact? What's the execution order?

Provide clear, concise explanations (not implementation details).
```

**Prompt 2: Review Draft Sections**
```
I've written the Intent and Success Criteria sections of my spec:

[Paste your Intent section]

[Paste your Success Criteria section]

Is this spec-quality? Specifically:
- Is the Intent clear about WHAT without HOW?
- Are Success Criteria measurable and falsifiable?
- Did I accidentally include implementation details?
- What's missing or ambiguous?

Suggest improvements.
```

**Prompt 3: Validate Completeness**
```
I've completed my full specification for the context-aware CLI tool.

Validate completeness using this checklist:
- [ ] Intent is clear (1-2 paragraphs)
- [ ] Success criteria are measurable
- [ ] Functional requirements are testable
- [ ] 6 components defined with responsibilities
- [ ] Algorithms specified (progressive loading, compression, isolation)
- [ ] Multi-agent coordination described
- [ ] Zero implementation code present
- [ ] Spec is 3-5 pages

[Paste your complete spec]

Where are the gaps? What needs more detail?
```

**Expected Outcomes**:
- Prompt 1: Clear conceptual understanding of each component
- Prompt 2: Feedback on Intent/Criteria quality
- Prompt 3: Completeness assessment and gaps identification

**Safety Note**: Your specification should never include actual credentials, API keys, or sensitive data. Use placeholders like "STRIPE_API_KEY" instead of real keys.

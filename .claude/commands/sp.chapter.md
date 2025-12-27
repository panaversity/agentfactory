---
description: Execute research-first chapter creation. Builds expertise skill BEFORE writing content. Use for new technical chapters requiring deep understanding of frameworks/libraries.
handoffs:
  - label: Skip to Content
    agent: sp.autonomous
    prompt: Skip skill creation, proceed with content
    send: true
---

# /sp.chapter: Research-First Chapter Creation (v1.5)

## ⛔ HARD GATES (EXECUTION BLOCKED WITHOUT THESE) ⛔

**These are PRE-EXECUTION checks. If ANY fails, you MUST NOT proceed.**

### GATE 1: Determine Entry Mode

```
PARSE user input:
  IF input contains "Issue #" OR "issue" OR references GitHub issues:
    MODE = "ISSUE_EXECUTION"
    → GATE 1A applies
  ELSE:
    MODE = "FRESH_START"
    → Proceed to Phase A (skill creation)
```

### GATE 1A: Issue Execution Requires Spec (BLOCKING)

```
IF MODE = "ISSUE_EXECUTION":
  1. Extract chapter slug from issues (e.g., "40-fastapi-for-agents")
  2. CHECK: Does specs/chapter-[slug]/spec.md exist?

  IF spec.md DOES NOT EXIST:
    ⛔ STOP IMMEDIATELY
    OUTPUT: "Cannot execute issues without specification.
             Run '/sp.chapter \"Chapter N: Title\"' from scratch first,
             OR create spec manually at: specs/chapter-[slug]/spec.md"
    → EXIT (do not proceed)

  IF spec.md EXISTS:
    → Verify plan.md exists (WARNING if missing)
    → Verify tasks.md exists (WARNING if missing)
    → Proceed to Phase B, Step B.7 (Implementation)
```

### GATE 2: Context Handoff & Autonomous Execution

```
THINKING FRAMEWORK (not hardcoded rules)

Skills and subagents start fresh - they don't inherit your context.
Apply these principles dynamically to ANY skill/subagent invocation:

═══════════════════════════════════════════════════════════════════
PRINCIPLE 0: USE THE COMMANDS - "Don't bypass, invoke"
═══════════════════════════════════════════════════════════════════

The spec loop commands (/sp.specify, /sp.clarify, /sp.plan, etc.)
ARE the way to do the work. You don't bypass them by doing it yourself.

❌ WRONG: "Spec is missing, let me write spec.md directly"
   Write(specs/chapter-40/spec.md)  ← Bypasses templates, quality gates

✅ RIGHT: "Spec is missing, let me invoke /sp.specify with context"
   Skill: sp.specify
   Args: |
     [full context gathered]

The commands exist because they:
  - Apply correct templates
  - Enforce quality standards
  - Create consistent artifacts
  - Can be validated by other commands

If you write artifacts directly, you bypass all of that.

═══════════════════════════════════════════════════════════════════
PRINCIPLE 1: BEFORE INVOKING - "What does it need to succeed?"
═══════════════════════════════════════════════════════════════════

Ask yourself:
  → What files does this skill need to read?
  → What decisions have I already made that it needs to know?
  → What constraints or requirements apply?
  → What quality standard should it match?

Then PASS ALL OF THAT in Args. If you spent time gathering context,
that context must appear in Args or it's wasted.

❌ /sp.specify                    ← Skill knows nothing
✅ /sp.specify with 20-line Args  ← Skill has full context

═══════════════════════════════════════════════════════════════════
PRINCIPLE 2: AFTER COMPLETION - "Did it actually work?"
═══════════════════════════════════════════════════════════════════

Ask yourself:
  → What should have been created?
  → Does the file exist where expected?
  → Does the content match what I asked for?
  → Are there placeholders or missing sections?

READ YOUR OUTPUT. Verify it meets requirements.
If it doesn't, fix it yourself and continue.

═══════════════════════════════════════════════════════════════════
PRINCIPLE 3: AUTONOMOUS FLOW - "Keep moving unless stuck"
═══════════════════════════════════════════════════════════════════

Execute the full workflow without pausing for confirmation:
  invoke → verify → continue → invoke → verify → continue → done

PAUSE ONLY for genuine ambiguity:
  - Multiple valid paths, need human to choose direction
  - Conflicting requirements that can't be auto-resolved
  - Validation failure you cannot fix yourself

NEVER PAUSE for:
  - "Should I proceed?" ← Just proceed
  - "Is this okay?" ← Verify it yourself
  - "Ready for next step?" ← Take the next step

If you're asking permission after every step, the user might as
well do it manually. Be autonomous.

═══════════════════════════════════════════════════════════════════
PRINCIPLE 4: SELF-CORRECT - "Fix it, don't ask about it"
═══════════════════════════════════════════════════════════════════

If output is wrong or incomplete:
  → Fix it yourself
  → Re-invoke if necessary
  → Continue with workflow
  → Only escalate if you genuinely cannot resolve it

The goal is: User gives input → Agent delivers complete result
Not: User gives input → Agent asks 50 questions → User does half the work
```

### GATE 3: Subagent File Writing Verification (MANDATORY)

```
FOR content-implementer subagents:

✅ REQUIRED behavior:
   - Subagent writes file directly via Write tool
   - Subagent returns: "✅ Wrote [path] ([N] lines)"
   - Orchestrator VERIFIES file exists: ls -la [path]

SUBAGENT PROMPT MUST INCLUDE:
  "Execute autonomously. Write file directly. Return confirmation only (~50 lines)."

AFTER SUBAGENT RETURNS:
  1. Check file exists: ls -la [path]
  2. IF file missing:
     - Agent definition may have parsing issues
     - Run /agents in Claude Code, verify "All tools" selected
     - Ensure single-line description in agent .md file
     - Restart session if config was recently changed
  3. IF file exists: proceed to validation

AGENT DEFINITION REQUIREMENTS:
  ✅ Single-line description (no description: |)
  ✅ No tools: field in YAML (use /agents UI instead)
  ✅ All tools selected in Claude Code /agents UI
```

### GATE 4: Validation Before Commit (BLOCKING)

```
BEFORE any git commit:

REQUIRED validators (via Task tool with subagent_type):
  - educational-validator (per lesson)
  - validation-auditor (chapter-wide)
  - factual-verifier (chapter-wide)

IF validators not invoked:
  ⛔ STOP - Cannot commit without validation

IF any validator returns FAIL:
  ⛔ STOP - Cannot commit with validation failures
  → Fix issues, re-validate, then commit
```

### GATE 5: Phase A Skill Creation (BLOCKING for Technical Chapters)

```
FOR technical chapters (Part 6-7) teaching frameworks/SDKs/tools:

BEFORE Phase B (content creation):
  1. IDENTIFY the PRIMARY framework/SDK being taught
     Example: "FastAPI for Agents" → PRIMARY = FastAPI

  2. CHECK skill existence:
     Does .claude/skills/building-with-[framework]/SKILL.md exist?

  3. IF skill DOES NOT EXIST:
     ⛔ MUST create skill BEFORE writing content
     → Use Phase A workflow (research → create → test → validate)
     → Skill becomes expertise source for content

  4. IF skill EXISTS but chapter covers NEW topics:
     → Update skill with new reference materials
     Example: Chapter covers security + SQLModel
     → Add references/security.md
     → Add references/sqlmodel.md

SKILL STRUCTURE (one skill per framework):
  .claude/skills/building-with-fastapi/
    ├── SKILL.md              # Core FastAPI patterns
    └── references/
        ├── security.md       # JWT, OAuth, rate limiting (topic)
        ├── sqlmodel.md       # Database integration (topic)
        ├── testing.md        # pytest patterns (topic)
        └── streaming.md      # SSE patterns (topic)

WRONG: building-with-fastapi-security (over-fragmented)
RIGHT: building-with-fastapi + references/security.md

WHY THIS GATE EXISTS:
- Chapter 40 incident: No skill created, content from memory
- Skill creation IS the research that prevents hallucination
```

### GATE 6: Clarification Before Implementation (BLOCKING)

```
BEFORE implementation (Phase B, Step B.7):

IF /sp.clarify was NOT invoked:
  ⛔ STOP - Must clarify ambiguities first

MINIMUM clarifications for technical chapters:
  - Technology versions (FastAPI 0.109+? SQLModel 0.0.14+?)
  - Scope boundaries (what's in vs out?)
  - Prerequisite assumptions (what do readers already know?)
  - Running example alignment (how does TaskManager connect?)

IF spec seems "clear enough":
  → Still invoke /sp.clarify
  → Response can be "Spec is complete, no clarifications needed"
  → But the invocation MUST happen
```

---

**Purpose**: Build deep expertise BEFORE writing content. Creates a programmatic skill for the technical domain, tests it on real projects, then uses that expertise to write high-quality chapter content.

**When to Use**: Creating new technical chapters (Part 6-7) that teach frameworks, SDKs, or tools.

---

## ⛔ ENFORCEMENT RULES (READ FIRST) ⛔

**These rules are NON-NEGOTIABLE. Violations cause quality drift.**

**⚠️ SEE HARD GATES ABOVE** - Gates 1-4 are BLOCKING requirements that must pass BEFORE any work begins.

### Rule 1: NO SKIPPING STEPS
Every step marked "EXECUTE - DO NOT SKIP" MUST be executed with the FULL prompt shown.
- ❌ FORBIDDEN: Outputting just `/sp.specify` without executing
- ❌ FORBIDDEN: Skipping clarification because "spec looks complete"
- ❌ FORBIDDEN: Combining multiple steps to "save time"
- ✅ REQUIRED: Execute each step, wait for completion, verify output, proceed

### Rule 2: NO ABBREVIATED PROMPTS
Each Skill/Task invocation MUST include the FULL prompt context shown.
- ❌ FORBIDDEN: `Skill: sp.tasks Args: chapter-37` (too short)
- ✅ REQUIRED: Full multi-line prompt with all context, requirements, paths

### Rule 3: NO QUALITY SHORTCUTS
- ❌ FORBIDDEN: Skipping validators because "content looks good"
- ❌ FORBIDDEN: Writing content directly instead of using subagents
- ❌ FORBIDDEN: Skipping PHR creation because "we're almost done"
- ✅ REQUIRED: Every validator must run and pass before commit

### Rule 4: PROGRESS REPORTING
After EACH step, you MUST report:
```
✅ Step [X]: [Name] COMPLETE
   Created: [file path or "N/A"]
   Summary: [1-2 sentences]
   Verified: [checklist items checked]
   Next: Step [X+1]
```

### Rule 5: FAILURE RESPONSE
If ANY step fails:
1. DO NOT proceed to next step
2. Report the failure with specific error
3. Fix the issue
4. Re-run the failed step
5. Only proceed when step passes

---

## User Input

```text
$ARGUMENTS
```

---

## The Two-Phase Approach

```
┌─────────────────────────────────────────────────────────────────────┐
│                    PHASE A: SKILL RESEARCH & CREATION               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. Use EXISTING skills for research:                               │
│     ├── researching-with-deepwiki (repo architecture)              │
│     ├── fetching-library-docs (API patterns via Context7)          │
│     └── WebSearch (community patterns)                              │
│  2. Build NEW programmatic skill with:                              │
│     ├── Persona (expert identity)                                   │
│     ├── Logic (decision trees, workflows)                           │
│     ├── Context (prerequisites, setup)                              │
│     ├── MCP (tool integrations)                                     │
│     ├── Data/Knowledge (API patterns in references/)               │
│     └── Safety & Guardrails                                         │
│  3. Use creating-skills to build properly                           │
│  4. Test skill on TaskManager project                               │
│  5. Validate and commit skill                                       │
│                                                                     │
│  OUTPUT: .claude/skills/building-with-[framework]/SKILL.md          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                    PHASE B: CHAPTER CREATION                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  With deep expertise encoded in skill:                              │
│  /sp.specify → /sp.clarify → /sp.plan → /sp.tasks →                │
│  /sp.analyze → /sp.taskstoissues → /sp.implement →                 │
│                                                                     │
│  ⛔ MANDATORY VALIDATION GATE (Step B.8) ⛔                          │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ Run in PARALLEL:                                             │   │
│  │ • educational-validator (per lesson, 8x parallel)            │   │
│  │ • validation-auditor (chapter-wide)                          │   │
│  │ • factual-verifier (chapter-wide)                            │   │
│  │ • pedagogical-designer (chapter-wide)                        │   │
│  │                                                               │   │
│  │ ALL MUST PASS before proceeding                               │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  → update tasks.md (close issues) → /sp.git.commit_pr              │
│                                                                     │
│  The skill IS the research - no hallucination risk                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## PHASE A: SKILL RESEARCH & CREATION

### Step A.1: Identify the Technical Domain

From user input, extract:
- **Framework/SDK**: e.g., "OpenAI Agents SDK", "Google ADK", "Anthropic Agents Kit"
- **Chapter Number**: e.g., "Ch 34", "Ch 37"
- **Running Example**: TaskManager Agent (from Part 6 framing)

### Step A.2: Research Using EXISTING Skills

**Use these skills in sequence:**

#### A.2.1: Repository Architecture (researching-with-deepwiki)

```
Skill: researching-with-deepwiki

Questions to ask DeepWiki:
- "Analyze the architecture of github.com/[org]/[repo]"
- "How is the agent/tool system implemented in github.com/[org]/[repo]?"
- "What design patterns are used in github.com/[org]/[repo]?"
- "Show the directory structure of github.com/[org]/[repo]"
```

**Capture:**
- Core architecture patterns
- Key abstractions (Agent, Runner, Tool, etc.)
- How components interact
- Example implementations in the repo

#### A.2.2: API Documentation (fetching-library-docs)

```bash
# Use the token-efficient shell pipeline
bash .claude/skills/fetching-library-docs/scripts/fetch-docs.sh \
  --library [framework] \
  --topic "getting started" \
  --verbose

bash .claude/skills/fetching-library-docs/scripts/fetch-docs.sh \
  --library [framework] \
  --topic "agents" \
  --mode code
```

**Capture:**
- Official API signatures
- Code examples from docs
- Configuration patterns
- Error handling patterns

#### A.2.3: Community Patterns (WebSearch)

```
WebSearch queries:
- "[framework] production examples 2024"
- "[framework] best practices"
- "[framework] vs [alternative] comparison"
- "[framework] common mistakes pitfalls"
- "[framework] with MCP integration"
```

**Capture:**
- Real-world usage patterns
- Community-discovered gotchas
- Integration patterns
- Performance considerations

### Step A.3: Build the New Skill

**Use creating-skills skill:**

```
Skill: creating-skills

Create a skill for [framework] with:
1. Name: building-with-[framework]
2. Description: "Use when building agents with [framework]..."
3. Structure per anatomy below
```

#### Skill Structure Required

```
.claude/skills/building-with-[framework]/
├── SKILL.md
│   ├── Frontmatter (name, description)
│   └── Body
│       ├── ## Persona
│       │   └── Expert identity and voice
│       ├── ## When to Use
│       │   └── Triggering conditions
│       ├── ## Core Concepts
│       │   └── Key abstractions (from DeepWiki)
│       ├── ## Decision Logic
│       │   └── When to use what pattern
│       ├── ## Workflow
│       │   └── Step-by-step implementation
│       ├── ## MCP Integration
│       │   └── How to connect with MCP servers
│       ├── ## Safety & Guardrails
│       │   └── What to avoid, error handling
│       └── ## TaskManager Example
│           └── How to build TaskManager with this
│
├── references/
│   ├── api-patterns.md      # From fetching-library-docs
│   ├── architecture.md      # From researching-with-deepwiki
│   └── community-wisdom.md  # From WebSearch
│
└── scripts/
    └── verify.py            # Validate skill works
```

#### Skill Components Detail

**1. Persona (WHO is this skill?)**
```markdown
## Persona

You are a [Framework] expert with production experience.
You understand both official patterns and community wisdom.
You've built TaskManager-style agents multiple times.
```

**2. Logic (WHEN to use what?)**
```markdown
## Decision Logic

| Situation | Pattern | Why |
|-----------|---------|-----|
| Simple single-purpose | Basic Agent | Less overhead |
| Multi-step workflow | Agent with handoffs | Clear responsibility |
| Tool-heavy operations | MCP integration | Standard connectivity |
| Streaming responses | SSE pattern | User experience |
```

**3. Context (WHAT does it need?)**
```markdown
## Prerequisites

Before building, verify:
- [ ] Python 3.11+ installed
- [ ] API key configured: `export [FRAMEWORK]_API_KEY=...`
- [ ] Dependencies: `uv add [framework]`
```

**4. MCP Integration (HOW to connect?)**
```markdown
## MCP Integration

### Connecting to MCP Servers

[Framework] connects to MCP via:

\`\`\`python
# Pattern from official docs
from [framework] import Agent, MCPServerStdio

agent = Agent(
    name="TaskManager",
    mcp_servers=[
        MCPServerStdio(command="uvx", args=["todo-mcp"])
    ]
)
\`\`\`
```

**5. Safety & Guardrails**
```markdown
## Safety

### NEVER
- Expose API keys in code or logs
- Skip error handling for API calls
- Ignore rate limits
- Trust user input without validation

### ALWAYS
- Use environment variables for secrets
- Wrap API calls in try/except
- Implement exponential backoff
- Validate and sanitize inputs
```

**6. TaskManager Example**
```markdown
## TaskManager Implementation

Complete example building TaskManager with [Framework]:

\`\`\`python
# Full working example from research
[Code from fetching-library-docs + community patterns]
\`\`\`
```

### Step A.4: Test the Skill

**Create test project:**

```bash
mkdir -p /tmp/test-[framework]-taskmanager
cd /tmp/test-[framework]-taskmanager
uv init
uv add [framework]
```

**Use the skill to build TaskManager:**

```
"Using the building-with-[framework] skill, create a TaskManager agent
that can add, list, and complete tasks."
```

**Validation criteria:**
- [ ] Skill triggers on relevant prompts
- [ ] Provides accurate API patterns
- [ ] TaskManager code compiles/runs
- [ ] No hallucinated methods or classes

### Step A.5: Validate and Commit

```bash
# Validate skill structure
python3 .claude/skills/creating-skills/scripts/verify.py \
  .claude/skills/building-with-[framework]

# If valid, commit
git add .claude/skills/building-with-[framework]
git commit -m "feat(skill): add [framework] expertise skill

Research sources:
- DeepWiki: github.com/[org]/[repo]
- Context7: [framework] docs
- Community: [key sources]

Tested on: TaskManager agent implementation"
```

### Step A.6: Create PHR for Phase A

**⚠️ MANDATORY**: Document the skill creation work.

```
Skill: sp.phr
Args: "Created [framework] expertise skill with research from DeepWiki, Context7, and community sources"
```

---

## PHASE B: CHAPTER CREATION

**⛔ CRITICAL: EVERY STEP IS MANDATORY. DO NOT SKIP OR ABBREVIATE. ⛔**

Each step below MUST be executed with the FULL prompt shown. Simply outputting a command name (e.g., "/sp.specify") without executing it is a FAILURE.

**Progress Tracking Required**: After each step, report:
```
✅ Step B.N: [Step Name] COMPLETE
   Created: [file path]
   Content: [brief summary]
   Next: Step B.N+1
```

---

### Step B.1: Specification (EXECUTE - DO NOT SKIP)

**Invoke the Skill tool with FULL context:**

```
Skill: sp.specify
Args: |
  Create specification for Chapter [N]: [Title]

  CONTEXT (from Phase A skill):
  - Framework/SDK: [framework name]
  - Expertise skill created: .claude/skills/building-with-[framework]/SKILL.md
  - Running example: TaskManager Agent
  - Target proficiency: [A1|A2|B1|B2|C1|C2]

  REQUIRED IN SPEC:
  1. Chapter overview with learning outcomes
  2. Lesson breakdown (7-10 lessons typical)
  3. Layer progression: L1 (manual) → L2 (collaboration) → L3 (skill) → L4 (orchestration)
  4. Prerequisites from earlier chapters
  5. Reference to expertise skill for accurate API patterns
  6. Images/videos available for this chapter (list paths)

  OUTPUT: specs/chapter-[N]-[slug]/spec.md

  Execute autonomously. DO NOT ask "Should I proceed?"
```

**After completion, verify**:
- [ ] spec.md exists at correct path
- [ ] Contains 7-10 lesson titles
- [ ] Layer progression is explicit
- [ ] References expertise skill

---

### Step B.2: Clarification (EXECUTE - DO NOT SKIP)

**Invoke the Skill tool to resolve ambiguities:**

```
Skill: sp.clarify
Args: |
  Review and clarify specification: specs/chapter-[N]-[slug]/spec.md

  CHECK FOR:
  1. Ambiguous lesson scope (too broad/narrow?)
  2. Missing prerequisites
  3. Unclear proficiency expectations
  4. Technology version specifications
  5. Exercise complexity alignment

  ASK UP TO 5 clarification questions if needed.
  ENCODE answers back into spec.md.

  If spec is already clear, confirm: "Spec is complete, no clarifications needed."
```

**After completion, verify**:
- [ ] Any ambiguities resolved
- [ ] Spec updated with clarifications (if any)

---

### Step B.3: Planning (EXECUTE - DO NOT SKIP)

**Invoke with chapter-planner subagent (NOT general planning):**

```
Task subagent_type=chapter-planner
Prompt: |
  Create implementation plan for: specs/chapter-[N]-[slug]/spec.md

  LEVERAGE EXPERTISE SKILL:
  - Read: .claude/skills/building-with-[framework]/SKILL.md
  - Use skill's decision logic for lesson structure
  - Use skill's examples for "Try With AI" sections
  - Use skill's safety notes for guardrails

  PLAN MUST INCLUDE:
  1. Lesson-by-lesson breakdown with:
     - Title and learning objectives
     - Layer (L1/L2/L3/L4)
     - Proficiency level (CEFR)
     - Key concepts from expertise skill
     - Estimated duration
  2. Pedagogical arc (Foundation → Practice → Integration → Mastery)
  3. Cognitive load assessment per lesson
  4. Content dependencies and ordering
  5. Images/videos to include (from available assets)

  QUALITY REFERENCE:
  Match structure of: apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/

  OUTPUT: specs/chapter-[N]-[slug]/plan.md

  Execute autonomously without confirmation.
```

**After completion, verify**:
- [ ] plan.md exists at correct path
- [ ] Each lesson has layer and proficiency specified
- [ ] Pedagogical arc is clear
- [ ] References expertise skill

---

### Step B.4: Task Generation (EXECUTE - DO NOT SKIP)

**Generate actionable task list:**

```
Skill: sp.tasks
Args: |
  Generate tasks from plan: specs/chapter-[N]-[slug]/plan.md

  FOR EACH LESSON, CREATE TASK WITH:
  - Task ID: T[N].L[X]
  - Description: "Create lesson X: [Title]"
  - Output path: apps/learn-app/docs/[part]/[chapter]/[lesson-slug].md
  - Dependencies: List prior lessons if any
  - Acceptance criteria:
    * Full YAML frontmatter (skills, learning_objectives, cognitive_load)
    * 3 "Try With AI" prompts with explanations
    * Evidence blocks for all code
    * Ends with activity section (no summary after)

  ADDITIONAL TASKS:
  - T[N].README: Chapter README.md
  - T[N].ASSESS: Chapter assessment/quiz
  - T[N].VALIDATE: Run all validators

  OUTPUT: specs/chapter-[N]-[slug]/tasks.md
```

**After completion, verify**:
- [ ] tasks.md exists with all lessons as tasks
- [ ] Each task has acceptance criteria
- [ ] Output paths are absolute

---

### Step B.5: Cross-Artifact Analysis (EXECUTE - DO NOT SKIP)

**Validate consistency across artifacts:**

```
Skill: sp.analyze
Args: |
  Analyze artifacts for chapter: specs/chapter-[N]-[slug]/

  CHECK:
  1. spec.md ↔ plan.md alignment (all lessons in both?)
  2. plan.md ↔ tasks.md alignment (all tasks for all lessons?)
  3. Proficiency consistency (same levels throughout?)
  4. Dependency correctness (no circular dependencies?)
  5. Output path validity (directories exist?)

  REPORT:
  - Gaps found
  - Inconsistencies
  - Recommended fixes

  If issues found, FIX THEM before proceeding.
```

**After completion, verify**:
- [ ] All artifacts aligned
- [ ] Issues fixed (if any were found)

---

### Step B.6: GitHub Issues (EXECUTE - DO NOT SKIP)

**Convert tasks to trackable issues:**

```
Skill: sp.taskstoissues
Args: |
  Create GitHub issues from: specs/chapter-[N]-[slug]/tasks.md

  FOR EACH TASK:
  - Create issue with task description
  - Add labels: content, chapter-[N], lesson
  - Add to milestone: Chapter [N]
  - Include acceptance criteria in issue body

  RETURN: List of created issue numbers
```

**After completion, verify**:
- [ ] Issues created in GitHub
- [ ] Issue numbers recorded in tasks.md

---

### Step B.7: Implementation (EXECUTE - DO NOT SKIP)

**⛔ USE SUBAGENTS - DO NOT WRITE CONTENT DIRECTLY ⛔**

```
Skill: sp.implement
Args: |
  Implement chapter: specs/chapter-[N]-[slug]/

  EXECUTION RULES:
  1. For EACH lesson task, spawn content-implementer subagent
  2. Each subagent writes directly to output path
  3. Subagent returns confirmation only (~50 lines), NOT full content
  4. After each lesson, VERIFY file exists: ls -la [path]
  5. If file missing, check GATE 3 troubleshooting
  6. Fix any validation failures before moving to next lesson

  SUBAGENT PROMPT TEMPLATE:
  ```
  Task subagent_type=content-implementer
  Prompt: |
    Execute autonomously without confirmation.

    Create lesson: [Title]
    Output path: [ABSOLUTE PATH]
    DO NOT create new directories.

    EXPERTISE SOURCE:
    Read: .claude/skills/building-with-[framework]/SKILL.md
    Use accurate API patterns from this skill.

    QUALITY REFERENCE:
    Match: apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md

    REQUIRED:
    - Full YAML frontmatter (skills, learning_objectives, cognitive_load, differentiation)
    - 3 "Try With AI" prompts with "What you're learning" explanations
    - Evidence blocks (Output:) for all executable code
    - Safety note at end
    - NO sections after "Try With AI" (no Summary, no What's Next)

    Write the file directly using the Write tool.
    Return ONLY: "✅ Wrote [path] ([N] lines)"
  ```

  VERIFICATION: After subagent returns, run: ls -la [path]

  PROGRESS: After each lesson verified, update tasks.md to mark [X] complete.
```

**After completion, verify**:
- [ ] All lesson files created at correct paths
- [ ] All tasks marked complete in tasks.md
- [ ] Educational-validator passed for each lesson

### Step B.8: Validation (MANDATORY GATE - DO NOT SKIP)

**⛔ BLOCKING REQUIREMENT**: Content MUST pass ALL validators before commit.

**Launch validators in parallel with FULL prompts:**

```
# Per-lesson validation (spawn N parallel agents for N lessons)
FOR EACH lesson file in chapter:
  Task subagent_type=educational-validator
  Prompt: |
    Validate lesson: [ABSOLUTE PATH TO LESSON]

    READ SKILLS FIRST (MANDATORY):
    1. Read .claude/skills/content-evaluation-framework/SKILL.md
    2. Read .claude/skills/ai-collaborate-teaching/SKILL.md
    3. Read .claude/skills/learning-objectives/SKILL.md

    CHECK:
    1. Framework invisibility (no "AI as Teacher", "Part 2:", meta-commentary)
    2. Evidence presence (70%+ code has Output: blocks)
    3. Structural compliance (ends with "Try With AI", no sections after)
    4. Proficiency alignment (uses proficiency_level, matches tier)
    5. Three Roles demonstration (student teaches AI at some point)

    RETURN: PASS with summary, or FAIL with specific line numbers and fixes

# Chapter-wide validation (parallel with per-lesson)
Task subagent_type=validation-auditor
Prompt: |
  Validate chapter: apps/learn-app/docs/[part]/[chapter]/

  Apply 6-category weighted rubric:
  - Technical Accuracy (30%)
  - Pedagogical Effectiveness (25%)
  - Writing Quality (20%)
  - Structure & Organization (15%)
  - AI-First Teaching (10%)
  - Constitution Compliance (Pass/Fail gate)

  RETURN: Overall score and per-category breakdown

Task subagent_type=factual-verifier
Prompt: |
  Verify all facts in chapter: apps/learn-app/docs/[part]/[chapter]/

  FOR EACH lesson:
  1. Extract all statistics, dates, quotes
  2. WebSearch to verify accuracy
  3. Flag any unverified or incorrect claims

  RETURN: List of verified claims and unverified claims

Task subagent_type=pedagogical-designer
Prompt: |
  Validate learning progression in: apps/learn-app/docs/[part]/[chapter]/

  CHECK:
  1. Layer progression L1→L2→L3→L4 is honored
  2. Cognitive load appropriate per proficiency tier
  3. Concept scaffolding builds properly
  4. No prerequisite violations

  RETURN: PASS or FAIL with specific issues
```

**GATE CRITERIA - ALL MUST PASS:**
| Validator | Pass Criteria | Failure Action |
|-----------|---------------|----------------|
| `educational-validator` | All 5 checks pass per lesson | Fix issues, re-validate lesson |
| `validation-auditor` | Score ≥ 80% weighted average | Identify weak areas, improve |
| `factual-verifier` | Zero unverified claims | WebSearch and fix all claims |
| `pedagogical-designer` | Progression validated | Reorder or add bridging content |

**IF ANY VALIDATOR FAILS:**
1. ⛔ DO NOT proceed to Step B.9
2. Read validation report carefully
3. Fix ALL identified issues in the affected files
4. Re-run ONLY the failed validators
5. Repeat until all pass
6. Only then proceed

---

### Step B.9: Close GitHub Issues (EXECUTE - DO NOT SKIP)

**Update task tracking:**

```bash
# Read tasks.md to get issue numbers
# For each completed task with issue number:
gh issue close [issue-number] --comment "Completed in [commit-sha]

Files created:
- [list of lesson files]

Validation results:
- educational-validator: PASS
- validation-auditor: [score]%
- factual-verifier: [N] claims verified
- pedagogical-designer: PASS"
```

**After completion, verify**:
- [ ] All issues closed
- [ ] Comments include validation results

---

### Step B.10: Create PHRs (EXECUTE - DO NOT SKIP)

**⚠️ MANDATORY**: Document ALL chapter work with PHRs.

```
# PHR for Phase A (skill creation)
Skill: sp.phr
Args: |
  Created [framework] expertise skill for Chapter [N]

  Research sources used:
  - DeepWiki: [repo]
  - Context7: [library docs]
  - Community: [key sources]

  Skill output: .claude/skills/building-with-[framework]/SKILL.md
  Tested on: TaskManager implementation

# PHR for Phase B (chapter implementation)
Skill: sp.phr
Args: |
  Implemented Chapter [N]: [Title]

  Lessons created: [count]
  Files: [list of paths]
  Validation results:
  - educational-validator: [X]/[N] lessons passed
  - validation-auditor: [score]%
  - factual-verifier: [N] claims verified
  - pedagogical-designer: PASS

  Issues closed: [list of issue numbers]
```

**After completion, verify**:
- [ ] PHR files created in history/prompts/
- [ ] Both Phase A and Phase B documented

---

### Step B.11: Commit and PR (EXECUTE - DO NOT SKIP)

**Create atomic commit and PR:**

```
Skill: sp.git.commit_pr
Args: |
  Commit and create PR for Chapter [N]: [Title]

  COMMIT MESSAGE FORMAT:
  feat(content): add Chapter [N] - [Title]

  Lessons:
  - [List of lesson titles]

  Skills created:
  - building-with-[framework]

  Validation: All validators passed
  PHRs: [PHR IDs]

  PR BODY:
  ## Summary
  - [N] lessons teaching [framework]
  - Layer progression: L1→L2→L3→L4
  - Proficiency: [tier]

  ## Validation Results
  - educational-validator: ✅ All lessons passed
  - validation-auditor: [score]%
  - factual-verifier: [N] claims verified
  - pedagogical-designer: ✅ Progression validated

  ## Files Changed
  [List of files]
```

**After completion, verify**:
- [ ] Commit created with proper message
- [ ] PR created with validation summary
- [ ] All files included

---

## EXISTING SKILLS INVENTORY

### Research Skills (Phase A)
| Skill | Purpose | When to Use |
|-------|---------|-------------|
| `researching-with-deepwiki` | Repo architecture via DeepWiki MCP | Understanding SDK internals |
| `fetching-library-docs` | API docs via Context7 (77% token savings) | Official API patterns |

### Building Skills (Phase A)
| Skill | Purpose | When to Use |
|-------|---------|-------------|
| `creating-skills` | Proper skill structure & validation | Building the new skill |
| `mcp-builder` | MCP server patterns | If SDK needs MCP server |

### Content Skills (Phase B)
| Skill | Purpose | When to Use |
|-------|---------|-------------|
| `ai-collaborate-teaching` | Three Roles Framework | L2 lesson design |
| `exercise-designer` | Deliberate practice exercises | Each lesson |
| `learning-objectives` | Bloom's/CEFR alignment | Lesson planning |
| `content-evaluation-framework` | Quality rubric | Before commit |

### Validation Skills (Phase B)
| Skill | Purpose | When to Use |
|-------|---------|-------------|
| `canonical-format-checker` | Format drift prevention | Teaching platform patterns |
| `code-validation-sandbox` | Code example validation | Before finalizing |

---

## EXAMPLE: Chapter 34 (OpenAI Agents SDK)

### Phase A Research

```bash
# 1. DeepWiki for architecture
"Analyze github.com/openai/openai-agents-python architecture"

# 2. Context7 for API
bash scripts/fetch-docs.sh --library openai-agents --topic "agent creation"
bash scripts/fetch-docs.sh --library openai-agents --topic "tools"

# 3. WebSearch for community
"OpenAI Agents SDK production examples 2024"
"OpenAI Agents SDK vs LangChain comparison"
```

### Phase A Skill Output

```
.claude/skills/building-with-openai-agents/
├── SKILL.md
├── references/
│   ├── api-patterns.md (Runner, Agent, Tool, Handoff)
│   ├── architecture.md (from DeepWiki)
│   └── community-wisdom.md (best practices)
└── scripts/
    └── verify.py
```

### Phase B Content Output

```
apps/learn-app/docs/06-AI-Native-Software-Development/34-openai-agents-sdk/
├── 01-what-is-openai-agents-sdk.md
├── 02-creating-your-first-agent.md
├── 03-tools-and-function-calling.md
├── 04-handoffs-and-multi-agent.md
├── 05-taskmanager-with-openai.md
├── 06-mcp-integration.md
├── 07-error-handling-safety.md
└── README.md
```

---

## QUALITY GATES

### Phase A → Phase B Transition

| Check | Requirement | How to Verify |
|-------|-------------|---------------|
| Skill validates | verify.py passes | `python3 scripts/verify.py` |
| Skill triggers | Test prompts activate it | Manual test |
| TaskManager works | Code runs successfully | Execute test project |
| No hallucinations | All APIs in official docs | Compare with Context7 output |

**If any check fails**: Fix skill before proceeding to content.

### Phase B: Content → Commit Gate (MANDATORY)

| Validator | Scope | Pass Criteria |
|-----------|-------|---------------|
| `educational-validator` | Per lesson (parallel) | All 5 checks pass |
| `validation-auditor` | Chapter-wide | ≥80% weighted score |
| `factual-verifier` | Chapter-wide | Zero unverified claims |
| `pedagogical-designer` | Chapter-wide | Progression validated |

**⛔ BLOCKING**: Cannot commit chapter content without ALL validators passing.

**If any validator fails**:
1. Identify specific issues from validator output
2. Fix content in affected lessons
3. Re-run ONLY the failed validators
4. Repeat until all pass
5. Only then proceed to commit

---

**Version**: 1.5 (December 2025) - Added 6 HARD GATES with blocking enforcement for all failure modes from Chapter 40 incident
**Required Skills**: researching-with-deepwiki, fetching-library-docs, creating-skills
**Required Validators**: educational-validator, validation-auditor, factual-verifier, pedagogical-designer
**Best For**: Technical chapters teaching frameworks/SDKs (Part 6-7)

---

## CHANGELOG

### v1.5 (2025-12-27)
**ADDITIONAL FIXES**: Added 2 more hard gates after further analysis of Chapter 40 incident:
- Phase A (skill creation) was completely skipped
- No clarification questions were asked
- No skills created for fastapi-security, sqlmodel domains

**New gates:**
- GATE 5: Phase A Skill Creation (BLOCKING for technical chapters)
- GATE 6: Clarification Before Implementation (BLOCKING)

### v1.4 (2025-12-27)
**CRITICAL FIX**: Added 4 hard gates after Chapter 40 incident where:
- Spec loop was bypassed entirely (no spec.md, plan.md, tasks.md)
- Skills were not invoked via Skill tool
- Subagents were not invoked via Task tool with proper subagent_type
- Content was written directly without orchestration

**New sections:**
- GATE 1: Entry mode detection (fresh start vs issue execution)
- GATE 1A: Spec existence check (BLOCKING for issue execution)
- GATE 2: Skill invocation enforcement (no mentioning without invoking)
- GATE 3: Subagent file writing protocol (write directly, return confirmation)
- GATE 4: Validation before commit (validators must run and pass)

### v1.3 (2025-12-26)
- Complete rewrite of Phase B with explicit prompts
- NO SHORTCUTS policy
- Progress tracking requirements

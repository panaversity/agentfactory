---
description: Execute research-first chapter creation. Builds expertise skill BEFORE writing content. Use for new technical chapters requiring deep understanding of frameworks/libraries.
handoffs:
  - label: Skip to Content
    agent: sp.autonomous
    prompt: Skip skill creation, proceed with content
    send: true
---

# /sp.chapter: Research-First Chapter Creation (v1.0)

**Purpose**: Build deep expertise BEFORE writing content. Creates a programmatic skill for the technical domain, tests it on real projects, then uses that expertise to write high-quality chapter content.

**When to Use**: Creating new technical chapters (Part 6-7) that teach frameworks, SDKs, or tools.

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
│  validators → update tasks.md (close issues) →                      │
│  /sp.git.commit_pr                                                  │
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

---

## PHASE B: CHAPTER CREATION

**Now with verified skill as knowledge source:**

### Step B.1: Specification

```
Skill: sp.specify
Args: "Chapter [N]: [Title]"

Include in spec:
- Reference skill: .claude/skills/building-with-[framework]
- Running example: TaskManager Agent
- Auxiliary examples: Legal, Finance, Healthcare agents
```

### Step B.2: Clarification

```
Skill: sp.clarify
Args: [feature-name]
```

### Step B.3: Planning

```
Skill: sp.plan
Args: [feature-name]

Plan should leverage:
- Skill's decision logic for lesson structure
- Skill's examples for "Try With AI" sections
- Skill's safety notes for guardrails
```

### Step B.4: Task Generation

```
Skill: sp.tasks
Args: [feature-name]
```

### Step B.5: Analysis

```
Skill: sp.analyze
Args: [feature-name]
```

### Step B.6: GitHub Issues

```
Skill: sp.taskstoissues
Args: [feature-name]
```

### Step B.7: Implementation

```
Skill: sp.implement
Args: [feature-name]

Each lesson subagent has access to:
- The framework skill (building-with-[framework])
- Content skills (ai-collaborate-teaching, exercise-designer)
- Validation skills (content-evaluation-framework)
```

### Step B.8: Validation

Run in parallel:
- `validation-auditor` - Comprehensive quality
- `educational-validator` - Pedagogical compliance
- `factual-verifier` - Accuracy (easier because skill is verified)

### Step B.9: Close Issues

```bash
# For each completed task
gh issue close [issue-number] --comment "Completed in [commit]"
```

### Step B.10: Commit and PR

```
Skill: sp.git.commit_pr
Args: [feature-name]
```

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

---

**Version**: 1.0 (December 2025)
**Required Skills**: researching-with-deepwiki, fetching-library-docs, creating-skills
**Best For**: Technical chapters teaching frameworks/SDKs (Part 6-7)

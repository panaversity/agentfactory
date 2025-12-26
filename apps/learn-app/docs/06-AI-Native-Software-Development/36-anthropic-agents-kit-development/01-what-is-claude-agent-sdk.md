---
sidebar_position: 1
title: "What is the Claude Agent SDK?"
description: "Understand how Claude Agent SDK differs from the Claude API (Anthropic Client SDK) and why it's the foundation for building autonomous agents and Digital FTEs."
keywords: [Claude Agent SDK, agent development, autonomous agents, tool execution, Digital FTE]
chapter: 36
lesson: 1
duration_minutes: 20

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Agent SDK vs API Paradigms"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can explain the fundamental difference between Client SDK (manual tool loop) and Agent SDK (autonomous tool execution)"

  - name: "Recognizing Claude SDK Differentiation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can identify 5+ unique features that set Claude Agent SDK apart from OpenAI and Google SDKs"

  - name: "Applying Claude SDK for Digital FTE Development"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can articulate why Claude SDK is the optimal choice for building production-grade Digital FTEs"

learning_objectives:
  - objective: "Articulate the difference between Claude Agent SDK (autonomous tool execution) and Claude API (manual tool loop management)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student explains why Agent SDK eliminates implementation burden and reduces error surface"

  - objective: "Identify 5+ unique features of Claude Agent SDK that differentiate it from OpenAI SDK and Google ADK"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Comparison table completion and feature capability matching"

  - objective: "Understand why Claude Agent SDK positions you to build Digital FTEs with minimal overhead"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student articulates how skills ecosystem and file checkpointing support production deployment"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (SDK vs API, tool execution, skills ecosystem, file checkpointing, runtime permissions) within B1 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Compare Claude SDK architecture with OpenAI SDK's native_tools implementation; analyze why Claude's approach reduces complexity"
  remedial_for_struggling: "Focus on single key difference: With Client SDK you write the loop, with Agent SDK Claude handles it. Connect to familiar analogy (database ORMs handling queries vs raw SQL)"
---

# What is the Claude Agent SDK?

You've used Claude Code. You've seen it read files, run commands, search the web, and write code autonomously. Now you're going to understand what powers that autonomy—and build your own agents using the same foundations.

The Claude Agent SDK is fundamentally different from the Claude API (the Anthropic Client SDK). If you've worked with OpenAI's SDK or Google's API, the distinction will feel profound. This difference shapes everything you'll build in this chapter.

Let's start with the core distinction, because it determines which tools you'll reach for in production.

## The Mental Model: Two Different Paradigms

When you use the **Claude API (Client SDK)**, you're implementing the agent loop yourself:

```
Your code:
1. Send prompt to Claude
2. Claude returns tool calls ("write this file")
3. You execute the tools
4. Send results back to Claude
5. Claude responds or requests more tools
6. Repeat until done
```

You control the loop. Claude generates instructions. You manage execution.

**With Claude Agent SDK**, Claude handles the loop:

```
Your code:
1. Send prompt to Claude
2. Claude observes the filesystem, runs commands, and takes action
3. Claude reports back with results
4. Repeat until task completes
```

Claude controls the loop. You specify the task. Claude manages execution.

**The Question This Raises**: Why does this distinction matter?

When you implement the loop yourself, you're responsible for:
- Parsing Claude's tool requests correctly
- Handling tool execution errors gracefully
- Managing state between iterations
- Deciding when the agent should stop
- Implementing security guardrails

That's a substantial surface area for bugs. You're not writing the agent logic—you're building infrastructure that the agent logic depends on.

With Agent SDK, Claude handles that infrastructure. You focus on:
- What the agent should do (specification)
- What tools it has access to (permissions)
- How to evaluate whether it succeeded (validation)

**This is why Claude Agent SDK is called "SDK" not "API".** It's not just a connection to Claude's models. It's a complete agent runtime that handles the complexity you'd otherwise implement yourself.

## Comparison: Claude SDK vs Client SDK vs Competitors

Here's how Claude Agent SDK compares to other approaches:

| **Capability** | **Claude Agent SDK** | **Claude API (Client SDK)** | **OpenAI Agents SDK** | **Google ADK** |
|---|---|---|---|---|
| **Tool Execution** | Claude autonomous | You implement loop | Swanson native_tools | Google Gemini 2.5 native |
| **Skills Ecosystem** | Yes (SKILL.md files) | No | No | No |
| **File Checkpointing** | Yes (rewindFiles()) | No | No | No |
| **Runtime Permissions** | Yes (canUseTool callback) | No | No (static allowlist) | No (static allowlist) |
| **Custom Commands** | Yes (.claude/commands/) | No | No | No |
| **Cost Tracking** | Per-message token usage | Per-message token usage | Per-message token usage | Per-message token usage |
| **System Prompt Presets** | Yes (claude_code preset) | No | No | No |
| **Session Management** | Persistent (resume=session_id) | Single-turn | Single-turn | Single-turn |
| **MCP Integration** | Native (mcp_servers=) | Basic | Requires wrapper | Basic |
| **Multi-Turn Conversations** | ClaudeSDKClient context preservation | Manual context management | Built-in conversation history | Built-in |

**What do these differences mean for building Digital FTEs?**

### Skills Ecosystem: Reusable Intelligence

With Claude Agent SDK, you can package domain expertise into **skills**—filesystem-based documents that give your agent specialized knowledge.

Instead of including everything in a system prompt:

```python
# WITHOUT SKILLS ECOSYSTEM
options = ClaudeAgentOptions(
    system_prompt="""You are a code review expert.
    Consider these security patterns: [20 KB of guidelines]
    Consider these performance patterns: [15 KB of guidelines]
    Consider these testing patterns: [10 KB of guidelines]"""
)
```

You organize it as reusable skills:

```python
# WITH SKILLS ECOSYSTEM
options = ClaudeAgentOptions(
    setting_sources=["project"],  # Load from .claude/skills/
    allowed_tools=["Skill"]
)
```

Then your agent accesses `/skills/security-review.md`, `/skills/performance-patterns.md`, etc. as needed. Skills compose. They're versionable. Teams can share them.

**Why this matters for Digital FTEs**: Your product can expose different skills to different customers. One Digital FTE loads the "aggressive optimization" skill. Another loads the "conservative reliability" skill. Same agent, different expertise.

### File Checkpointing: Undo Capability

Claude Agent SDK tracks file changes and lets you rewind:

```python
# Rewind files to previous checkpoint
await client.rewind_files(checkpoint_id)
```

This is extraordinarily valuable for agents that might:
- Accidentally delete critical files
- Make speculative changes that need reverting
- Need to try multiple approaches

**Why this matters for Digital FTEs**: A buggy agent doesn't destroy a customer's codebase—it reverts. The agent can be bold in exploration because failure is recoverable.

### Runtime Permissions: Dynamic Decision-Making

With `canUseTool` callback, your agent makes permission decisions at runtime based on context:

```python
async def can_use_tool(tool: str, input: dict, context: dict):
    # DENY writes to config files
    if tool == "Write" and "/config/" in input.get("file_path", ""):
        return {"behavior": "deny", "message": "Config files protected"}

    # ALLOW with modified path (sandbox writes)
    if tool == "Write" and input.get("file_path", "").startswith("/tmp"):
        return {"behavior": "allow", "updatedInput": input}

    return {"behavior": "allow", "updatedInput": input}
```

OpenAI and Google SDKs use static allowlists. Claude SDK can reason about context.

**Why this matters for Digital FTEs**: Your product can implement sophisticated permission policies. "Allow writes to test files but not production files." "Allow API calls to staging environment but not production." The agent respects these policies without modification.

### Cost Tracking: Per-Message Economics

All SDKs track token usage. Claude Agent SDK integrates cost tracking:

```python
async for message in query(prompt="Task", options=opts):
    if message.type == "result":
        print(f"Total cost: ${message.total_cost_usd:.4f}")
```

**Why this matters for Digital FTEs**: You're selling agent capability. Cost transparency drives pricing decisions. Knowing that a particular workflow consumes $0.47 per execution lets you offer it profitably.

## Why Claude Agent SDK Is The Top Contender For Digital FTEs

The fundamental insight: **Claude Agent SDK is the only SDK designed for production autonomous agents.**

OpenAI SDK and Google ADK are designed for:
- Single-turn queries with tool use
- Stateless interactions
- Direct API consumption

Claude Agent SDK is designed for:
- Multi-turn agent workflows
- Stateful sessions with recovery
- Production deployment with guardrails

**For building Digital FTEs—agents that work 24/7 for customers—this distinction is critical.**

Consider a Digital FTE that reviews pull requests for code quality:

**Without Agent SDK** (OpenAI approach):
- You implement the agent loop
- Each code review is a separate session
- If the agent makes a mistake modifying code, you recover manually
- Permissions are static (can write or cannot write)
- Cost tracking requires parsing API responses

**With Agent SDK**:
- Claude handles the loop
- Session persists across multiple PRs
- File changes are checkpointed and reversible
- Permissions adapt based on which PR branch is being reviewed
- Cost is transparent per review

The Agent SDK removes infrastructure burden. You focus on domain expertise (what makes a good code review?). Claude handles the agent machinery.

## The Skills Ecosystem: Compound Intelligence

This is where Claude Agent SDK truly differentiates for Digital FTEs.

A skill is a markdown file with:
- **Persona**: How should the agent think about this domain?
- **Logic**: What decision frameworks apply?
- **Context**: What prerequisites matter?
- **Data/Knowledge**: What patterns exist?

Skills compose. A code review agent might load:

- `/skills/security-review.md` — Security patterns and threat modeling
- `/skills/python-best-practices.md` — Python-specific guidance
- `/skills/test-driven-development.md` — Testing philosophy

Each skill is reusable across agents. Skills are versionable. Teams share and improve them.

**Why this matters**: You're not building one agent. You're building an agency—a collection of specialized agents sharing domain expertise through skills.

## What You're Building In This Chapter

This chapter teaches you to:

1. **Install and configure** Claude Agent SDK
2. **Understand tools** available (Read, Write, Edit, Bash, Glob, Grep, WebSearch, WebFetch, Task)
3. **Manage permissions** using permission modes and canUseTool callbacks
4. **Build session-aware agents** that maintain state across interactions
5. **Compose subagents** for parallel processing
6. **Create skills** that encode reusable intelligence
7. **Deploy production agents** with monitoring and cost tracking

By the end, you'll have built a production-grade agent that could become a Digital FTE.

## Try With AI

Use Claude Code (or any Claude Agent SDK environment) for these prompts.

### Prompt 1: Experience the Agent Loop Difference

```
I have a directory with 50 Python files. I want to count how many
have a specific function called "validate_input". I'll describe the
task, you execute it autonomously without me implementing a loop.
Read all files in the current directory, search for "validate_input"
function definitions, and report a summary.
```

**What you're learning**: How Claude Agent SDK executes complex multi-step tasks without you managing the iteration loop. Notice that you specified *what* (find validate_input), not *how* (iterate through files, parse Python, etc.).

### Prompt 2: Understand Tool Composition

```
Now I want you to create a small test file that validates the validate_input
function exists in at least 3 of those files. Write a script that imports
from those files and tests the function. Show me the test results.
```

**What you're learning**: How Agent SDK composes multiple tools (Read → understand files → Write new test file → Bash execute tests). Each tool is autonomous. You don't implement orchestration.

### Prompt 3: Compare With Manual API Loop

```
Explain to me: If I were using the Claude API (Client SDK) to do this same
task, what code would I need to write to implement the agent loop? What's
different about how Agent SDK handles it?
```

**What you're learning**: The architectural difference. With Client SDK, you'd write code to: 1) Parse Claude's tool requests, 2) Execute the tools yourself, 3) Send results back, 4) repeat until done. Agent SDK eliminates steps 2 and 4 from your responsibility.

---

**Safety Note**: As you build agents with the SDK, remember that agent autonomy comes with responsibility. Always test agents in non-production environments first. Use permission callbacks to restrict what agents can do. File checkpointing exists exactly for this—so you can safely experiment knowing changes are reversible.

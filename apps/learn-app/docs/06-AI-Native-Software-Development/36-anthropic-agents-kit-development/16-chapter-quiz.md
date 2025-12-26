---
sidebar_position: 16
title: "Chapter Quiz: Claude Agent SDK Mastery"
description: "Test your understanding of Claude Agent SDK's 8 unique features and production patterns. Assess your readiness to build production-grade Digital FTEs using the complete SDK."
keywords: [quiz, assessment, claude agent sdk, self-test, mastery check, digital fte]
chapter: 36
lesson: 16
duration_minutes: 30
assessment_type: summative
questions_count: 20
bloom_distribution:
  remember: 5
  understand: 5
  apply: 5
  analyze: 3
  evaluate: 2
---

# Chapter Quiz: Claude Agent SDK Mastery

You've now learned all eight unique features that differentiate Claude Agent SDK from other agent development frameworks. This quiz tests your understanding of those features and your ability to apply them in production scenarios.

The quiz has 20 questions spanning the Bloom's taxonomy progression from **Remember** (foundational knowledge) through **Evaluate** (judgment about design choices). Each question maps to specific SDK features or production patterns covered in this chapter.

**How to use this quiz:**

1. Read each question carefully
2. Think through your answer before revealing the solution
3. Review the explanation even if you answered correctly—it reinforces learning
4. Track your score using the scoring guide below
5. Use results to identify areas for deeper review

---

## Scoring Guide

| Score Range | Assessment | Recommended Action |
|---|---|---|
| **18-20** (90-100%) | Master | Ready to build production Digital FTEs; consider advanced optimization patterns |
| **15-17** (75-89%) | Proficient | Ready for production with senior review; revisit 1-2 concepts for deepening |
| **12-14** (60-74%) | Developing | Return to lessons covering missed concepts; practice with simple agents first |
| **9-11** (45-59%) | Emerging | Work through lessons sequentially; focus on Layer 2 collaboration patterns before Layer 4 production |
| **Below 9** (<45%) | Foundation Building | Start with Lesson 1 and build systematically; ensure each feature is understood before moving forward |

---

## Questions

### Question 1 (Remember)

Which tool allows you to capture and restore file state during agent execution?

<details>
<summary>Show answer</summary>

**Answer:** `rewindFiles()`

**Explanation:** File checkpointing with `rewindFiles()` is a unique Claude Agent SDK feature that allows you to restore files to a previous checkpoint. OpenAI SDK and Google ADK lack this capability, making it essential for building resilient agents that can safely explore multiple approaches.

**Chapter Reference:** Lesson 8 - File Checkpointing

</details>

---

### Question 2 (Remember)

What is the name of the Claude Agent SDK's built-in permission system that lets you decide whether to allow or deny tool execution at runtime?

<details>
<summary>Show answer</summary>

**Answer:** `canUseTool()` callback

**Explanation:** `canUseTool()` is a runtime permission decision function that evaluates each tool call and returns true/false to allow or deny execution. This is unique to Claude Agent SDK and provides fine-grained control over agent actions, preventing unauthorized operations even when the agent requests them.

**Chapter Reference:** Lesson 4 - Permission Modes and Security

</details>

---

### Question 3 (Remember)

What file path pattern does Claude Agent SDK use to load reusable Agent Skills?

<details>
<summary>Show answer</summary>

**Answer:** `.claude/skills/*.md` (SKILL.md files from filesystem)

**Explanation:** Claude Agent SDK automatically loads SKILL.md files from the `.claude/skills/` directory through the `settingSources` parameter. This filesystem-based skills loading is unique to Claude Agent SDK and enables agents to access accumulated organizational knowledge from your project's skill library.

**Chapter Reference:** Lesson 5 - Agent Skills and Code

</details>

---

### Question 4 (Remember)

What do slash commands in Claude Agent SDK represent?

<details>
<summary>Show answer</summary>

**Answer:** Custom command patterns defined in `.claude/commands/*.md` files

**Explanation:** Slash commands (like `/debug`, `/test`, `/review`) are custom commands you define in `.claude/commands/` that agents can recognize and execute. This pattern is unique to Claude Agent SDK and allows you to create domain-specific workflows that agents can invoke, building reusable command patterns for your team.

**Chapter Reference:** Lesson 6 - Custom Slash Commands

</details>

---

### Question 5 (Remember)

Which ClaudeAgentOptions parameter must be set to True to enable file checkpointing?

<details>
<summary>Show answer</summary>

**Answer:** `enable_file_checkpointing=True`

**Explanation:** The `enable_file_checkpointing` parameter in ClaudeAgentOptions activates checkpoint tracking for all file operations. Without this enabled, the agent's file modifications are not captured as checkpoints, preventing recovery operations.

**Chapter Reference:** Lesson 8 - File Checkpointing

</details>

---

### Question 6 (Understand)

Explain the difference between ClaudeSDKClient and the query() function in Claude Agent SDK.

<details>
<summary>Show answer</summary>

**Answer:**
- **ClaudeSDKClient** is a streaming client that maintains connection state, allows multiple sequential queries, and provides access to lifecycle hooks and checkpoint recovery
- **query()** is a convenience function that runs a single query with default options and returns results

The choice depends on whether you need persistent state (ClaudeSDKClient) or simple one-off execution (query()).

**Chapter Reference:** Lesson 12 - SDK Client and Streaming

</details>

---

### Question 7 (Understand)

How do Agent Skills differ from generic AI prompts or system instructions?

<details>
<summary>Show answer</summary>

**Answer:**
Agent Skills are structured SKILL.md files with:
- **Persona** - Expert identity and reasoning stance
- **Logic** - Decision trees for when/how to apply the skill
- **Context** - Prerequisites and setup requirements
- **MCP** - Tool integrations specific to the skill
- **Data** - Patterns and knowledge the skill encodes
- **Safety** - Guardrails and what to avoid

Generic prompts are text instructions without this structure. Skills are executable knowledge that agents can load, reason about, and apply systematically.

**Chapter Reference:** Lesson 5 - Agent Skills and Code

</details>

---

### Question 8 (Understand)

Why is `canUseTool()` different from simple allowlist/blocklist permission systems?

<details>
<summary>Show answer</summary>

**Answer:**
Allowlists/blocklists make simple yes/no decisions: "Agent can use Read tool, cannot use Bash."

`canUseTool()` makes context-aware decisions:
- "Agent can use Bash to run tests, but only in /test/ directory, not /production/"
- "Agent can use Edit on .md files, but not on .json config files"
- "Agent can only use cost-tracking tools if total_cost_usd < $10"

This runtime evaluation of each specific tool call enables security policies that simple allowlists cannot express.

**Chapter Reference:** Lesson 4 - Permission Modes and Security

</details>

---

### Question 9 (Understand)

What are lifecycle hooks and what problems do they solve in agent systems?

<details>
<summary>Show answer</summary>

**Answer:**
Lifecycle hooks are callbacks that fire at key moments during agent execution:
- `onSessionStart()` - Initialize state when session begins
- `onMessageReceived()` - React to incoming messages
- `onToolCall()` - Intercept tool calls before execution
- `onToolResult()` - Process tool results before sending to agent
- `onSessionEnd()` - Cleanup when session completes

They solve:
- **Stateful operations** - Initialize databases, load context
- **Monitoring** - Track execution without modifying agent code
- **Validation** - Inspect tool calls/results before processing
- **Integration** - Connect to external systems (logging, analytics)

**Chapter Reference:** Lesson 10 - Lifecycle Hooks

</details>

---

### Question 10 (Understand)

How does context compaction solve the long-running agent problem?

<details>
<summary>Show answer</summary>

**Answer:**
Long-running agents accumulate message history, causing:
- Increasing context window usage (more tokens, higher cost)
- Slower response times (more messages to process)
- Memory pressure on the system

Context compaction solves this by:
1. Summarizing old message history into a concise recap
2. Replacing verbose message chains with summaries
3. Keeping recent messages (last 5-10) for immediate context
4. Allowing agents to continue for days/weeks without context overflow

This enables persistent agents that handle thousands of interactions.

**Chapter Reference:** Lesson 7 - Session Management and Context

</details>

---

### Question 11 (Apply)

You're building an agent that reads customer data files. Design a `canUseTool()` policy that prevents the agent from accidentally deleting customer data while still allowing legitimate read/edit operations.

<details>
<summary>Show answer</summary>

**Answer:**

```python
def canUseTool(tool_name: str, tool_input: dict) -> bool:
    if tool_name == "Read":
        # Allow reading customer files
        return True

    if tool_name == "Edit":
        # Allow editing only .csv and .txt files (not .py or .sql)
        file_path = tool_input.get("file_path", "")
        return file_path.endswith((".csv", ".txt"))

    if tool_name == "Bash":
        # Allow safe read commands, block all delete/remove commands
        command = tool_input.get("command", "")
        dangerous_patterns = ["rm ", "delete", "truncate", "drop"]
        return not any(pattern in command for pattern in dangerous_patterns)

    # Block all other tools
    return False
```

**Why this works:**
- Read is always safe, so allow it
- Edit restricted to data formats, not code
- Bash allowed only for safe commands, blocking dangerous patterns
- Other tools blocked by default

**Chapter Reference:** Lesson 4 - Permission Modes and Security

</details>

---

### Question 12 (Apply)

Your agent is tracking API call costs. Write the configuration that enables cost tracking in ClaudeAgentOptions and explain how you'd use the cost data.

<details>
<summary>Show answer</summary>

**Answer:**

```python
from claude_agent_sdk import ClaudeAgentOptions, ClaudeSDKClient

options = ClaudeAgentOptions(
    model="claude-opus-4-5-20251101",  # Required for cost tracking
    # Cost tracking is automatic in ClaudeSDKClient
)

async with ClaudeSDKClient(options) as client:
    async for message in client.query("Your task here"):
        if hasattr(message, 'total_cost_usd'):
            print(f"Cost so far: ${message.total_cost_usd:.4f}")

        # Stop if costs exceed budget
        if message.total_cost_usd > 5.00:
            break
```

**How to use cost data:**
1. **Budget enforcement** - Stop execution when costs exceed threshold
2. **Economic modeling** - Calculate unit economics for Digital FTE pricing
3. **Optimization** - Identify expensive operations to optimize
4. **Customer billing** - Charge customers based on actual agent costs

**Chapter Reference:** Lesson 13 - Cost Tracking and Billing

</details>

---

### Question 13 (Apply)

You need to implement a custom slash command `/debug` that agents can invoke to run debugging. Write the SKILL.md structure for this command.

<details>
<summary>Show answer</summary>

**Answer:**

Create `.claude/commands/debug.md`:

```markdown
---
command: /debug
category: development
description: "Run debugging for the current codebase"
---

# /debug Command

## Persona

You are a debugging expert. Think like a systems diagnostician—
identify root causes, not just symptoms.

## When to Use

- Exception traces appear
- Tests fail mysteriously
- Performance issues occur
- Integration fails unexpectedly

## Logic

1. Examine error messages for root cause signals
2. Check related code sections for logical errors
3. Run targeted tests to isolate the problem
4. Propose minimal fix (not refactoring)

## Safe Operations

- Read error logs
- Run test suite
- Inspect configuration
- NOT modify production code without approval

## Example Usage

Agent receives: `/debug Exception in auth service`

Agent:
1. Reads auth service code
2. Runs auth tests to reproduce
3. Identifies root cause
4. Proposes minimal fix
```

**Why this works:**
- Defines when command applies (persona + logic)
- Constrains dangerous operations (safety)
- Gives examples agents can learn from
- Makes debugging reproducible across sessions

**Chapter Reference:** Lesson 6 - Custom Slash Commands

</details>

---

### Question 14 (Apply)

You're implementing session forking to explore two different approaches. Write pseudocode showing how to fork a session, pursue an alternative approach, and optionally merge findings back.

<details>
<summary>Show answer</summary>

**Answer:**

```python
import asyncio
from claude_agent_sdk import ClaudeSDKClient, ClaudeAgentOptions

async def session_forking_pattern():
    # Main session
    main_options = ClaudeAgentOptions(
        allowed_tools=["Read", "Edit", "Bash"],
        enable_file_checkpointing=True
    )

    async with ClaudeSDKClient(main_options) as main_session:
        # Do initial work
        await main_session.query("Analyze the codebase")

        # Get checkpoint ID after analysis
        analysis_checkpoint = "checkpoint-uuid-here"

        # Fork a parallel session for alternative approach
        fork_options = ClaudeAgentOptions(
            allowed_tools=["Read", "Edit", "Bash"],
            enable_file_checkpointing=True
        )

        async with ClaudeSDKClient(fork_options) as fork_session:
            # Fork explores different implementation
            await fork_session.query("Try refactoring with async/await")
            fork_results = []
            async for msg in fork_session.receive_response():
                fork_results.append(msg)

        # Back in main session - either keep fork results or revert
        if fork_results_better:
            # Keep fork results
            print("Fork results adopted")
        else:
            # Revert to main approach
            await main_session.rewind_files(analysis_checkpoint)
            print("Reverted to main approach")
```

**When to use:**
- Testing multiple implementation strategies
- Risk mitigation (try risky approach in fork, keep main stable)
- Performance comparison (measure both approaches)
- Architectural decision making

**Chapter Reference:** Lesson 7 - Session Management and Context

</details>

---

### Question 15 (Apply)

Design a callback function for the `onToolResult()` lifecycle hook that validates agent outputs before processing them. What problems does this solve?

<details>
<summary>Show answer</summary>

**Answer:**

```python
from claude_agent_sdk import ClaudeAgentOptions

def on_tool_result(tool_name: str, result: dict) -> dict:
    """Validate and sanitize tool results before returning to agent."""

    if tool_name == "Bash":
        # Validate command output
        output = result.get("stdout", "")

        # Check for error signals
        if "error" in output.lower() or result.get("exit_code") != 0:
            return {
                "output": output,
                "validation_warning": "Command returned non-zero exit code"
            }

        # Truncate massive outputs that could inflate token usage
        if len(output) > 10000:
            return {
                "output": output[:10000] + "\n[... truncated ...]",
                "validation_warning": "Output truncated to prevent context overflow"
            }

    if tool_name == "Read":
        # Validate file contents
        content = result.get("content", "")
        if len(content) > 50000:  # Very large file
            return {
                "content": content[:50000] + "\n[... file truncated ...]",
                "validation_warning": "Large file truncated"
            }

    return result

options = ClaudeAgentOptions(
    on_tool_result=on_tool_result
)
```

**Problems this solves:**
1. **Token overflow** - Truncate results that would bloat context
2. **Error detection** - Alert agent to failed operations
3. **Data validation** - Ensure results meet expected format
4. **Sanitization** - Remove sensitive data before sending to agent
5. **Performance** - Short-circuit expensive operations based on validation

**Chapter Reference:** Lesson 10 - Lifecycle Hooks

</details>

---

### Question 16 (Analyze)

Compare the three permission modes (default, acceptEdits, bypassPermissions). For each, identify a scenario where it's appropriate and a risk if misused.

<details>
<summary>Show answer</summary>

**Answer:**

| Mode | When Appropriate | Risk if Misused |
|------|---|---|
| **Default** (canUseTool() every call) | Building agents that will access production systems or user data | Slowdown from repeated permission checks; bottleneck if checks are expensive |
| **acceptEdits** (human pre-approves, agent executes) | Collaborative workflows where human wants to see plan before execution; code review agents | Human approves without fully reading; agent executes wrong instructions anyway |
| **bypassPermissions** (no checks, execute directly) | Ephemeral agents in development/testing; fully isolated environments | Agent inadvertently breaks things; security breach if credentials exposed; no audit trail |

**Architectural principle:**
- **Production** → Default (every action checked)
- **Collaborative** → acceptEdits (human reviews plan)
- **Ephemeral/Test** → bypassPermissions (speed over safety)

**Red flag:** Using bypassPermissions in production means agent actions are unguarded.

**Chapter Reference:** Lesson 4 - Permission Modes and Security

</details>

---

### Question 17 (Analyze)

Describe the relationship between Agent Skills, Slash Commands, and Subagents. How do they differ and when would you use each?

<details>
<summary>Show answer</summary>

**Answer:**

| Component | What It Is | Use When | Example |
|---|---|---|---|
| **Agent Skill** | Reusable knowledge + decision logic | You want primary agent to know domain patterns | Security skill: "Think like security engineer when reviewing code" |
| **Slash Command** | Specialized workflow agent can invoke | You want to trigger domain-specific operations | `/debug` - run debugging workflow |
| **Subagent** | Autonomous agent managing single responsibility | You need independent decision-making on a task | Code review subagent that evaluates pull requests autonomously |

**How they compose:**
```
Main Agent
├── Loads Skills (knows patterns)
├── Invokes Slash Commands (delegates to workflows)
└── Manages Subagents (delegates to specialists)
```

**Decision framework:**
- **Knowledge needed?** → Create Skill
- **Workflow to trigger?** → Create Slash Command
- **Independent authority?** → Create Subagent

**Real example:**
```python
# Main agent loads security skill
options.setting_sources = ["/project/.claude/skills/security.md"]

# Main agent can invoke specialized workflow
agent receives: "/debug-security" command
agent invokes slash command

# Main agent delegates to code review subagent
from claude_agent_sdk import AgentDefinition, Task

review_subagent = AgentDefinition(
    name="code-reviewer",
    role="evaluate pull requests"
)

task = Task(subagent=review_subagent, goal="review PR #123")
```

**Chapter Reference:** Lessons 5, 6, 9 - Skills, Commands, Subagents

</details>

---

### Question 18 (Analyze)

An agent is building a microservice with 8 files. Explain how file checkpointing helps you handle the following scenarios:
1. Agent makes good progress but then introduces a critical bug
2. You want to compare two different architectural approaches
3. You need to audit what changes the agent made

<details>
<summary>Show answer</summary>

**Answer:**

**Scenario 1: Critical bug partway through**

Without checkpointing: Manually revert 8 files one by one, re-run first part of work

With checkpointing:
```python
# After initial analysis (good state)
good_checkpoint = "uuid-after-analysis"

# Agent works, encounters bug after refactoring 4 files
# You notice the bug

await client.rewindFiles(good_checkpoint)  # Back to good state
# Agent continues with different approach
```

**Scenario 2: Compare architectures**

Without checkpointing: Try architecture A, manually revert, try architecture B

With checkpointing:
```python
checkpoint_before = await client.getCurrentCheckpoint()

# Agent tries monolithic approach (Architecture A)
await client.query("Build monolithic auth system")

# Check results, decide to compare
await client.rewindFiles(checkpoint_before)

# Agent tries microservices approach (Architecture B)
await client.query("Build microservices auth system")

# Compare both approaches' results
```

**Scenario 3: Audit trail**

Checkpointing provides:
- **Timestamp** - when each checkpoint was created
- **File state** - exact content at each checkpoint
- **Sequence** - progression of changes
- **Recovery points** - where agent succeeded/failed

```python
# Query checkpoint history
history = await client.getCheckpointHistory()
for checkpoint in history:
    print(f"{checkpoint.id}: {checkpoint.timestamp}")
    print(f"  Files changed: {checkpoint.file_list}")
    print(f"  Agent state: {checkpoint.agent_status}")
```

**Why this matters:** Checkpointing enables safe exploration without fear of permanent damage.

**Chapter Reference:** Lesson 8 - File Checkpointing

</details>

---

### Question 19 (Evaluate)

You're designing a Digital FTE that manages sensitive customer data. Choose between three architectural approaches:
1. Default permissions with `canUseTool()` callback
2. acceptEdits mode with human approval each step
3. Subagent approach with limited tool access

Justify your choice considering: security, throughput, user experience, and auditability.

<details>
<summary>Show answer</summary>

**Answer:**

**Recommended: Hybrid Approach (Default + Subagent separation)**

Use Default permissions with `canUseTool()` for data access, delegating sensitive operations to restricted subagents:

```python
# Main agent
main_options = ClaudeAgentOptions(
    allowed_tools=["Read"],  # Main can only read
    can_use_tool=strict_canUseTool
)

# Sensitive operations delegated to subagent
delete_subagent = AgentDefinition(
    name="data-deletion",
    allowed_tools=["Edit", "Bash"],  # Subagent can modify
    description="Handles data deletion with audit logging"
)

# Main agent delegates to subagent for sensitive ops
task = Task(subagent=delete_subagent, goal="Delete customer #123 data")
```

**Comparison:**

| Approach | Security | Throughput | UX | Auditability |
|---|---|---|---|---|
| **Default + canUseTool()** | High (per-call checks) | Medium (checks add latency) | Poor (no human oversight) | Good (all actions logged) |
| **acceptEdits** | Highest (human reviews) | Low (human bottleneck) | Good (humans see plans) | Best (human + system audit) |
| **Subagent isolation** | Very High (compartmentalization) | High (parallel execution) | Fair (less transparency) | Excellent (subagent logs) |

**Why hybrid works best:**
- **Security**: Compartmentalization (subagents can't access data they don't need)
- **Throughput**: No human bottleneck; parallel subagent execution
- **Auditability**: Subagent logs + main agent logs create audit trail
- **User experience**: Humans see high-level summaries, not every decision

**Trade-off**: Less transparency than acceptEdits, but acceptable for automated workflows

**Chapter Reference:** Lessons 4, 9 - Permission Modes, Subagents

</details>

---

### Question 20 (Evaluate)

You need to decide between deploying your agent as: (a) ephemeral short-lived process, (b) long-running service with context compaction, or (c) session-fork architecture with persistent state. Evaluate based on: cost, latency, reliability, and state management.

<details>
<summary>Show answer</summary>

**Answer:**

**Decision Framework:**

| Pattern | Best For | Cost | Latency | Reliability | State |
|---|---|---|---|---|---|
| **Ephemeral** | One-off tasks, low volume, development | Lowest (spin up, run, shut down) | High (cold starts) | Medium (stateless, simple) | None (fresh start each time) |
| **Long-Running + Compaction** | Persistent service, high volume, 24/7 | Medium (always running) | Low (warm connection) | High (recovery from failures) | Built-in (context summary) |
| **Session Forking** | Exploratory work, A/B testing, learning | Higher (multiple sessions) | Medium (parallel branches) | Highest (multiple paths) | Complex (branch tracking) |

**Recommendation by Use Case:**

**Use Ephemeral if:**
- Task runs once/day or less
- Each invocation is independent
- Example: Scheduled data export, nightly report generation
- Pricing model: Pay-per-execution

**Use Long-Running + Compaction if:**
- Service is always available (24/7)
- High request volume throughout day
- State persists across requests (user context, preferences)
- Example: Customer support agent, personal assistant
- Pricing model: Subscription ($199/month)

**Use Session Forking if:**
- You need to explore multiple solutions
- Cost of human review justifies complexity
- Testing architectural decisions
- Example: Code architecture exploration, feature design
- Pricing model: Enterprise hourly billing

**Real Example - Customer Support Agent:**

```python
# Use long-running + compaction
options = ClaudeAgentOptions(
    enable_context_compaction=True,
    max_context_before_compaction=50000,
    persistence_backend="redis"  # Persist state across requests
)

# Agent handles 100+ customer requests per day
# Context compaction prevents token overflow
# State retained for multi-turn conversations
```

**Cost Analysis:**

If handling 100 requests/day:
- **Ephemeral**: 100 cold starts = overhead
- **Long-running**: Single warm connection = efficient
- **Forking**: Only for exploration (add 20% overhead for research phase)

**Conclusion:** Choose based on access pattern:
- **Sparse** (few times/day) → Ephemeral
- **Dense** (constant requests) → Long-running
- **Exploratory** (testing) → Forking during development, then switch to production pattern

**Chapter Reference:** Lessons 7, 14 - Session Management, Production Patterns

</details>

---

## What's Next

**If you scored 18-20:**
Congratulations! You have mastery-level understanding of Claude Agent SDK. You're ready to:
- Design and implement production Digital FTEs
- Make sophisticated architecture decisions
- Mentor others on SDK patterns

Consider exploring advanced topics:
- Multi-model orchestration (routing tasks to different models)
- Custom MCP server integration
- Advanced permission policies for team workflows

**If you scored 15-17:**
Solid proficiency. You can build and deploy agents, but deepen your understanding of 1-2 features. Consider:
- Picking one SDK feature you scored lower on
- Building a small project using that feature
- Reviewing the corresponding lesson again with fresh eyes

**If you scored 12-14:**
Good foundation, but more practice needed. Build a simple agent that uses 3-4 SDK features:
- Start with Read, Edit, Bash tools
- Add one permission mode (canUseTool)
- Add one lifecycle hook
- Complete a working project before moving to complex patterns

**If you scored below 12:**
Return to foundational lessons and build incrementally. The SDK has eight features; try mastering one at a time:
1. Start with basic query() and built-in tools
2. Add canUseTool() for permission control
3. Add Agent Skills for reusable knowledge
4. Add lifecycle hooks for observability
5. Add file checkpointing for recovery
6. Continue with remaining features

You have solid conceptual understanding. Hands-on practice will solidify the knowledge.

---

**Ready to build?** Create a small Digital FTE that serves a specific domain. Use this quiz as a reference for which features to include. Start simple, then add complexity as you build confidence.

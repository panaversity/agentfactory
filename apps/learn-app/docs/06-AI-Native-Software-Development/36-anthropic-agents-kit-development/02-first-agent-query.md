---
sidebar_position: 2
title: "Your First Agent with query()"
description: "Write your first autonomous agent using the query() function. Understand ClaudeAgentOptions parameters, message types, and how agents execute tools."
keywords: [query, ClaudeAgentOptions, agent messages, async/await, tool execution]
chapter: 36
lesson: 2
duration_minutes: 25

# HIDDEN SKILLS METADATA
skills:
  - name: "Writing Basic query() Calls"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write a query() call with prompt and ClaudeAgentOptions, iterate over messages, and identify message types correctly"

  - name: "Configuring ClaudeAgentOptions for Agent Behavior"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain what allowed_tools, permission_mode, max_turns, and cwd parameters do, and choose appropriate values for specific workflows"

  - name: "Processing Agent Message Types"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can correctly identify and extract data from assistant messages, tool_use messages, and result messages using type checking patterns"

learning_objectives:
  - objective: "Write a basic query() call with async/await that executes an autonomous agent task"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code execution: Student's query() call correctly streams messages without errors"

  - objective: "Configure ClaudeAgentOptions to control tool access (allowed_tools, permission_mode, max_turns)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Configuration task: Student sets up options that match specific workflow constraints"

  - objective: "Extract and process different message types from the agent response stream"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Message handling: Student correctly identifies message types and extracts result data"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (query, async/await, ClaudeAgentOptions, allowed_tools, permission_mode, message types) within B1-B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Explore session management with resume parameter; implement custom permission callbacks using can_use_tool; design multi-turn conversations with ClaudeSDKClient"
  remedial_for_struggling: "Focus on the basic query pattern first; practice identifying message types by running simple examples; break down async/await pattern step-by-step"
---

# Your First Agent with query()

You're about to write your first autonomous agent in 10 lines of code.

Unlike traditional APIs where you send a request and get a response, agents work differently: They receive a task, reason about what to do, execute tools autonomously, evaluate results, and iterate until the task is complete. Your job is to start the agent and process the stream of messages it produces.

The `query()` function is your entry point to this autonomous loop. By the end of this lesson, you'll understand how to configure it, iterate over its message stream, and extract results.

## Installation and Authentication

### Install the SDK

```bash
# Python
pip install claude-agent-sdk

# TypeScript
npm install @anthropic-ai/claude-agent-sdk
```

### Set Up API Access

The SDK authenticates using environment variables. Three options exist:

**Option 1: Anthropic API (Recommended for development)**

```bash
export ANTHROPIC_API_KEY=your-api-key-here
```

Get your API key at [console.anthropic.com](https://console.anthropic.com/keys).

**Option 2: Amazon Bedrock (Production-grade)**

```bash
export CLAUDE_CODE_USE_BEDROCK=1
export AWS_PROFILE=your-profile
```

**Option 3: Google Vertex AI**

```bash
export CLAUDE_CODE_USE_VERTEX=1
export GOOGLE_PROJECT_ID=your-project
export GOOGLE_REGION=us-central1
```

For this lesson, use Option 1 (Anthropic API). Authentication will complete in seconds when you first call `query()`.

## Understanding query()

The `query()` function is an async generator—it streams messages as the agent executes.

### Basic Pattern: Python

```python
import asyncio
from claude_agent_sdk import query, ClaudeAgentOptions

async def main():
    async for message in query(
        prompt="Find all Python files in the current directory and count their lines",
        options=ClaudeAgentOptions(
            allowed_tools=["Bash", "Glob"],
            permission_mode="default"
        )
    ):
        print(f"Message type: {message.type}")

asyncio.run(main())
```

**Output:**
```
Message type: assistant
Message type: tool_use
Message type: assistant
Message type: result
Done: Found 47 Python files totaling 18,294 lines of code
```

The stream ends when `message.type == "result"` appears, signaling agent completion.

The agent:
1. **assistant**: Claude receives the prompt and generates a plan
2. **tool_use**: Claude decides to use Bash/Glob to find files
3. **assistant**: Claude processes the results and generates output
4. **result**: The final answer is delivered

### Basic Pattern: TypeScript

```typescript
import { query } from "@anthropic-ai/claude-agent-sdk";

async function main() {
  for await (const message of query({
    prompt: "Find all Python files in the current directory and count their lines",
    options: {
      allowedTools: ["Bash", "Glob"],
      permissionMode: "default"
    }
  })) {
    console.log(`Message type: ${message.type}`);
  }
}

main();
```

**Output:**
```
Message type: assistant
Message type: tool_use
Message type: assistant
Message type: result
Done: Found 47 Python files totaling 18,294 lines of code
```

Same message flow as Python. The async iterator pattern (`for await...of`) is the TypeScript equivalent of Python's `async for`.

## ClaudeAgentOptions: Controlling Agent Behavior

`ClaudeAgentOptions` is a configuration object that controls what tools the agent can use and how it executes.

### Parameter 1: allowed_tools

Specifies which tools the agent can invoke. The Agent SDK provides these built-in tools:

| Tool | What It Does |
|------|--------------|
| `Read` | Read any file (text, images, PDFs, Jupyter notebooks) |
| `Write` | Create new files |
| `Edit` | Make precise edits to existing files (character-by-character) |
| `Bash` | Run terminal commands and scripts |
| `Glob` | Find files matching patterns (`**/*.ts`, `src/**/test*.py`) |
| `Grep` | Search file contents with regex |
| `WebSearch` | Search the web |
| `WebFetch` | Fetch and parse web pages |
| `Task` | Spawn subagents for parallel work |

**Example: Code review agent that can read and search, but not modify**

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Grep", "Glob"],
    # ... other config
)
```

**Why restrict tools?** An agent with access to only Read and Grep cannot accidentally delete files. This is a security principle: **Start with deny-all, then allowlist only what's necessary.**

### Parameter 2: permission_mode

Controls how the agent prompts for permission before executing certain operations.

| Mode | Behavior | When to Use |
|------|----------|------------|
| `default` | Prompts user before file edits, Bash commands, deletions | Interactive sessions where you want approval for each operation |
| `acceptEdits` | Auto-approves file edits, still prompts for dangerous operations | Trusted workflows where edits are safe but Bash needs approval |
| `bypassPermissions` | No prompts; agent executes without asking | Automated pipelines, production systems (use with caution) |

**Example: Auto-approve file edits for code formatting**

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Edit"],
    permission_mode="acceptEdits"  # Edits approved; Bash still requires permission
)
```

### Parameter 3: max_turns

Limits the number of agent iterations. The agent counts each cycle of reasoning → tool use → evaluation as one turn.

**Why limit turns?** Prevents runaway agents that loop indefinitely:

```python
options = ClaudeAgentOptions(
    allowed_tools=["Bash", "Read"],
    max_turns=5  # Stop after 5 reasoning cycles maximum
)
```

Without `max_turns`, an agent could theoretically run forever. Set a limit appropriate to your task:
- **Simple tasks** (search, format): `max_turns=3`
- **Standard tasks** (refactor, debug): `max_turns=5-10`
- **Complex tasks** (architecture design): `max_turns=15`

### Parameter 4: cwd (Current Working Directory)

Specifies where the agent executes Bash commands and file operations.

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Bash"],
    cwd="/path/to/project"  # All file operations relative to this directory
)
```

This prevents the agent from accidentally modifying system files outside your project.

## Processing Message Types

When you iterate over `query()` messages, three types appear:

### Message Type 1: assistant

The agent's reasoning—its plan before tool execution.

```python
async for message in query(prompt="Task", options=opts):
    if message.type == "assistant":
        print(f"Agent thinks: {message.content}")
```

**Output:**
```
Agent thinks: I should first find Python files in the directory, then count
their lines. Let me start by using Glob to find all .py files.
```

The `content` field contains the agent's natural language reasoning. Use this to understand what the agent is planning to do.

### Message Type 2: tool_use

The agent has decided to use a tool. This message contains what tool and what parameters.

```python
async for message in query(prompt="Task", options=opts):
    if hasattr(message, "tool_use") and message.tool_use:
        for tool_call in message.tool_use:
            print(f"Using {tool_call.name} with {tool_call.input}")
```

**Output:**
```
Using Glob with {'pattern': '**/*.py'}
Using Bash with {'command': 'wc -l *.py | tail -1'}
```

You won't typically act on this message—the SDK handles tool execution automatically. But you can use it for logging or monitoring what the agent is doing.

### Message Type 3: result

The final result. This is what you extract for application use.

```python
async for message in query(prompt="Task", options=opts):
    if message.type == "result":
        print(f"Final answer: {message.result}")
        # Output: "I found 45 Python files totaling 12,450 lines of code"
```

The `result` field contains the agent's final output—the answer to your prompt.

## Complete Example: Reading Code Quality

This example shows a complete agent that analyzes code quality:

**Python (with message processing)**

```python
import asyncio
from claude_agent_sdk import query, ClaudeAgentOptions

async def analyze_code_quality():
    """Analyze code quality in a project."""

    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash", "Glob"],
        permission_mode="default",
        max_turns=5,
        cwd="/path/to/your/project"
    )

    prompt = """
    Analyze code quality in this project:
    1. Find all Python files
    2. Check for obvious style issues (long functions, missing docstrings)
    3. Report a summary with recommendations
    """

    message_count = 0
    async for message in query(prompt=prompt, options=options):
        message_count += 1

        if message.type == "assistant":
            # Agent is thinking; we can log this for debugging
            print(f"[Turn {message_count}] Agent reasoning...")

        elif hasattr(message, "tool_use") and message.tool_use:
            # Agent is using tools; useful for monitoring
            print(f"[Turn {message_count}] Using tools...")

        elif message.type == "result":
            # Extract final result
            print(f"\n=== Analysis Complete ===")
            print(message.result)
            return message.result

asyncio.run(analyze_code_quality())
```

**Output:**
```
[Turn 1] Agent reasoning...
[Turn 2] Using tools...
[Turn 3] Agent reasoning...
[Turn 4] Using tools...
[Turn 5] Agent reasoning...

=== Analysis Complete ===
Found 23 Python files. Issues detected:
- 4 functions exceeding 50 lines (recommendation: break into smaller functions)
- 8 files missing module docstrings
- Average line length: 68 chars (good)

Priority actions:
1. Add docstrings to auth.py and database.py
2. Refactor calculate_total() in utils.py (78 lines)
```

**TypeScript equivalent:**

```typescript
import { query } from "@anthropic-ai/claude-agent-sdk";

async function analyzeCodeQuality() {
  const options = {
    allowedTools: ["Read", "Bash", "Glob"],
    permissionMode: "default",
    maxTurns: 5,
    cwd: "/path/to/your/project"
  };

  const prompt = `
    Analyze code quality in this project:
    1. Find all Python files
    2. Check for obvious style issues
    3. Report a summary with recommendations
  `;

  let messageCount = 0;
  for await (const message of query({ prompt, options })) {
    messageCount++;

    if (message.type === "assistant") {
      console.log(`[Turn ${messageCount}] Agent reasoning...`);
    } else if ("toolUse" in message && message.toolUse) {
      console.log(`[Turn ${messageCount}] Using tools...`);
    } else if (message.type === "result") {
      console.log(`\n=== Analysis Complete ===`);
      console.log(message.result);
      return message.result;
    }
  }
}

analyzeCodeQuality();
```

**Output:**
```
[Turn 1] Agent reasoning...
[Turn 2] Using tools...
[Turn 3] Agent reasoning...
[Turn 4] Using tools...
[Turn 5] Agent reasoning...

=== Analysis Complete ===
Found 23 Python files. Issues detected:
- 4 functions exceeding 50 lines
- 8 files missing module docstrings
- Average line length: 68 chars (good)
```

## Key Differences from the Client SDK

If you've used the Anthropic Client SDK (`anthropic` package), the Agent SDK works fundamentally differently:

| Feature | Client SDK | Agent SDK |
|---------|-----------|----------|
| **Tool Loop** | You implement tool execution | SDK handles it automatically |
| **Async Pattern** | Single-turn `client.messages.create()` | Streaming `async for message in query()` |
| **Tools Available** | Limited (vision, code execution) | Rich (file operations, bash, search, web) |
| **Session Continuity** | Manual with token tracking | Built-in with `resume` parameter |
| **Permissions** | No built-in permission system | Three modes: default, acceptEdits, bypassPermissions |
| **Subagents** | Manual task spawning | Native `Task` tool for parallel agents |

The Agent SDK is designed for autonomous agents with long execution cycles. The Client SDK is designed for chatbots with human turns. Pick the right tool for your use case.

## Try With AI

Use your AI companion (Claude, ChatGPT, Gemini, or similar) to explore query() in action.

### Prompt 1: Set Up Your First Agent

```
I want to write my first Claude Agent SDK agent. Help me:
1. Create a simple Python script that uses query() to read a file from my project
   and summarize its contents
2. Walk me through the ClaudeAgentOptions I need (what tools, what permission mode)
3. Show me how to iterate through messages and extract the final result

Here's my project structure: [paste your directory structure or describe it]
```

**What you're learning**: How to structure a basic query() call with appropriate configuration for your workflow, and how to handle the message stream it produces.

### Prompt 2: Debug Message Processing

```
I ran a query() and got these message types in my output:
[paste the message types you saw: assistant, tool_use, result, etc.]

For each message type, explain:
- What does it tell me about what the agent was doing?
- Which message type should I extract data from?
- How would I access the actual results/answers?

Show me the Python code pattern for checking each message type.
```

**What you're learning**: How to identify different message types flowing from the agent and correctly extract results without processing noise.

### Prompt 3: Design Permissions for Your Use Case

```
I need to build an agent for this workflow: [describe your task]

The agent should be able to:
- Read files: [what types, what directories?]
- Modify files: [if needed, what kinds of changes?]
- Run commands: [what bash operations are necessary?]

Help me:
1. Choose the right set of allowed_tools
2. Pick the right permission_mode (default, acceptEdits, bypassPermissions)
3. Set max_turns to something reasonable for this task
4. Design a prompt that communicates clear intent without over-specifying

Show me the complete ClaudeAgentOptions configuration.
```

**What you're learning**: How to think about agent permissions as a security principle, and how to balance automation (acceptEdits, bypassPermissions) with safety (default, restricted tools).

### Safety Note

When you give agents access to Write/Edit/Bash and `permission_mode=bypassPermissions`, the agent can modify or delete files without asking. Start conservatively: Use `allowed_tools=["Read", "Glob", "Grep"]` first to observe how the agent behaves. Gradually expand permissions as you verify the agent works reliably.

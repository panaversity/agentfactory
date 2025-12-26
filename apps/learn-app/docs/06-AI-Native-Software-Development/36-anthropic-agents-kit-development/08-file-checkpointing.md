---
sidebar_position: 8
title: "File Checkpointing: Recovering from Agent Mistakes"
description: "Enable file checkpointing to capture agent execution snapshots and use rewindFiles() to recover from mistakes. Build resilient agents that can safely explore multiple approaches without destroying code."
keywords: [file checkpointing, rewindFiles, error recovery, agent resilience, undo, session management]
chapter: 36
lesson: 8
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Enabling File Checkpointing"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can configure enable_file_checkpointing=True and understand how checkpoints track file state"

  - name: "Capturing Checkpoint UUIDs During Execution"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can identify UserMessage objects with UUID fields during agent execution and store checkpoint IDs for later recovery"

  - name: "Recovering File State with rewindFiles()"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can use rewindFiles() to restore previous file states and resume agent sessions from recovery points"

learning_objectives:
  - objective: "Enable file checkpointing in ClaudeAgentOptions to track file changes during agent execution"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Configuration task: Student enables checkpointing with correct parameters"

  - objective: "Identify checkpoint UUIDs from UserMessage objects during agent streaming"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Code execution: Student captures checkpoint IDs from agent message stream"

  - objective: "Use rewindFiles() to recover file state and continue execution from known-good checkpoint"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Recovery task: Student reverts files to checkpoint and demonstrates successful recovery"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (enable_file_checkpointing, checkpoint UUID, UserMessage, session recovery, rewindFiles, error recovery pattern) within B1-B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Implement checkpoint strategy before risky operations; design multi-checkpoint workflows for comparing agent approaches; analyze checkpoint overhead vs recovery benefit"
  remedial_for_struggling: "Focus on basic enable/rewind pattern first; practice identifying UUIDs in message streams; use simple examples with 2-3 file changes before complex scenarios"
---

# File Checkpointing: Recovering from Agent Mistakes

Your agent just modified 12 files and made a critical error in one of them. A traditional agent system? The changes are permanent. You're reverting manually, manually, manually.

The Claude Agent SDK does something radical: **It snapshots file state at execution checkpoints. If your agent makes a mistake, you rewind.**

This is the difference between agents you can let loose on real projects and agents you have to watch constantly. File checkpointing makes agents resilient.

## The Problem: Risky Exploration

Agents are bold. They explore multiple solutions. They refactor code. They try different architectures. This exploration is valuable—it discovers approaches you wouldn't think of.

But exploration is risky:

- Agent decides to restructure your authentication flow
- Makes changes to 8 files
- One file has a syntax error
- Now you have 8 files in an inconsistent state

Without checkpointing, you're manually reverting. With checkpointing, you rewind to the moment before exploration started and try a different approach.

### Real-World Scenario: Code Refactoring

```python
# Your agent receives this request:
# "Refactor our auth module from callbacks to async/await"

# The agent:
# 1. Analyzes current code (Checkpoint A)
# 2. Starts refactoring auth.py
# 3. Updates middleware.py
# 4. Updates error handling
# 5. Syntax error in session_manager.py (Checkpoint B)

# WITHOUT CHECKPOINTING:
# You manually revert session_manager.py,
# then discover auth.py has a logical error,
# then middleware.py doesn't compile,
# then you give up and revert manually

# WITH CHECKPOINTING:
# await client.rewind_files(checkpoint_a)
# Agent resumes with a different approach
```

Checkpointing lets your agent be exploratory without being destructive.

## Enabling File Checkpointing

File checkpointing is disabled by default. Enable it in `ClaudeAgentOptions`:

### Basic Enablement

```python
from claude_agent_sdk import ClaudeAgentOptions, ClaudeSDKClient
import asyncio

async def main():
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Edit", "Bash"],
        enable_file_checkpointing=True,
        extra_args={"replay-user-messages": None}  # Required to get checkpoint UUIDs
    )

    async with ClaudeSDKClient(options) as client:
        await client.query("Refactor the auth module to use async/await")

        async for message in client.receive_response():
            print(f"Message type: {message.type}")
            # More on capturing UUIDs below

asyncio.run(main())
```

**Output:**
```
Message type: assistant
Message type: tool_use
Message type: assistant
Message type: result
```

That's it. Checkpointing is now active. Every file change the agent makes is snapshots with a checkpoint UUID.

### Configuration Explained

| Parameter | Purpose | Required? |
|-----------|---------|-----------|
| `enable_file_checkpointing=True` | Activate checkpoint tracking | Yes |
| `extra_args={"replay-user-messages": None}` | Include checkpoint UUIDs in messages | Yes (for recovery) |

**Why `replay-user-messages`?** This flag tells the SDK to emit UserMessage objects that contain checkpoint UUIDs. Without it, you can still use checkpointing, but you won't know the checkpoint IDs to rewind to.

## Capturing Checkpoint UUIDs

File states are identified by checkpoint UUIDs. A new UUID is generated each time the agent executes a tool that modifies files.

### Identifying UserMessage Objects

```python
from claude_agent_sdk import ClaudeSDKClient, ClaudeAgentOptions, UserMessage

async def main():
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Edit"],
        enable_file_checkpointing=True,
        extra_args={"replay-user-messages": None}
    )

    checkpoints = {}

    async with ClaudeSDKClient(options) as client:
        await client.query("Add error handling to auth.py")

        async for message in client.receive_response():
            # Capture checkpoints from UserMessage objects
            if isinstance(message, UserMessage) and message.uuid:
                checkpoint_id = message.uuid
                checkpoints[checkpoint_id] = {
                    "timestamp": message.timestamp or "unknown",
                    "description": "After agent tool execution"
                }
                print(f"Checkpoint created: {checkpoint_id}")

            if message.type == "result":
                print(f"Agent completed. Created {len(checkpoints)} checkpoints")
                break

asyncio.run(main())
```

**Output:**
```
Checkpoint created: 550e8400-e29b-41d4-a716-446655440000
Checkpoint created: 550e8400-e29b-41d4-a716-446655440001
Checkpoint created: 550e8400-e29b-41d4-a716-446655440002
Agent completed. Created 3 checkpoints
```

Each UUID represents a specific file state. If anything goes wrong after checkpoint `550e8400-e29b-41d4-a716-446655440001`, you can rewind to that exact state.

### Storing Checkpoints Strategically

In production, you'd store checkpoints with semantic meaning:

```python
checkpoints = {}

async for message in client.receive_response():
    if isinstance(message, UserMessage) and message.uuid:
        # Store with description for later reference
        checkpoints["after_authentication"] = message.uuid

        # Or store chronologically
        if "checkpoints" not in checkpoints:
            checkpoints["checkpoints"] = []
        checkpoints["checkpoints"].append({
            "id": message.uuid,
            "timestamp": message.timestamp,
            "order": len(checkpoints["checkpoints"])
        })
```

**Best practice**: Save checkpoint IDs to a file or database during long-running agent sessions. This way, if your program crashes, you can still recover.

## Recovering with rewindFiles()

When an agent makes a mistake, use `rewindFiles()` to restore files to a previous checkpoint.

### Basic Recovery Pattern

```python
from claude_agent_sdk import ClaudeSDKClient, ClaudeAgentOptions, UserMessage
import asyncio

async def main():
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Edit"],
        enable_file_checkpointing=True,
        extra_args={"replay-user-messages": None}
    )

    last_good_checkpoint = None

    async with ClaudeSDKClient(options) as client:
        # Task 1: Initial refactoring
        await client.query("Refactor database.py to use connection pooling")

        async for message in client.receive_response():
            if isinstance(message, UserMessage) and message.uuid:
                last_good_checkpoint = message.uuid

            if message.type == "result":
                result = message.result
                print(f"Task 1 result: {result}")

                # Check if refactoring succeeded
                if "error" in result.lower():
                    print(f"Error detected! Rewinding to checkpoint: {last_good_checkpoint}")

                    # Rewind files to known-good state
                    await client.rewind_files(last_good_checkpoint)
                    print("Files recovered. Ready for retry.")
                else:
                    print("Refactoring successful!")

                break

asyncio.run(main())
```

**Output:**
```
Task 1 result: Added connection pooling. All tests pass.
Refactoring successful!
```

If there had been an error:
```
Task 1 result: Syntax error in pool configuration
Error detected! Rewinding to checkpoint: 550e8400-e29b-41d4-a716-446655440001
Files recovered. Ready for retry.
```

All file changes after that checkpoint are undone instantly.

### Recovering and Retrying

```python
async def refactor_with_recovery(client, task_description):
    """Execute a refactoring with automatic recovery on error."""

    checkpoints = []

    # Capture starting checkpoint
    await client.query("")  # Get initial state
    async for message in client.receive_response():
        if isinstance(message, UserMessage) and message.uuid:
            checkpoints.append(message.uuid)
        if message.type == "result":
            break

    starting_checkpoint = checkpoints[-1] if checkpoints else None

    # Attempt refactoring
    await client.query(task_description)

    success = False
    async for message in client.receive_response():
        if isinstance(message, UserMessage) and message.uuid:
            checkpoints.append(message.uuid)

        if message.type == "result":
            if "error" not in message.result.lower():
                success = True
                print(f"Success: {message.result}")
            else:
                print(f"Failed: {message.result}")
                print(f"Recovering to checkpoint: {starting_checkpoint}")
                await client.rewind_files(starting_checkpoint)
            break

    return success

# Usage
async with ClaudeSDKClient(options) as client:
    success = await refactor_with_recovery(
        client,
        "Refactor our logging to use structured JSON format"
    )

    if not success:
        print("Refactoring failed and files were recovered.")
        # Now try a different approach
        await client.query("Create a new logging module in parallel without modifying existing code")
```

## Checkpoint Strategy: When to Checkpoint

Checkpointing happens automatically, but you should understand when to use recovery strategically.

### Pattern 1: Risky Operations

Checkpoint BEFORE attempting risky modifications:

```python
async with ClaudeSDKClient(options) as client:
    # Get baseline checkpoint
    baseline_checkpoint = None

    # Baseline operation
    await client.query("Read the entire codebase and summarize architecture")
    async for message in client.receive_response():
        if isinstance(message, UserMessage) and message.uuid:
            baseline_checkpoint = message.uuid
        if message.type == "result":
            break

    print(f"Baseline checkpoint: {baseline_checkpoint}")

    # NOW attempt risky operation
    await client.query("Refactor dependency injection throughout the codebase")

    async for message in client.receive_response():
        if message.type == "result":
            if "error" in message.result:
                # Recover to baseline
                await client.rewind_files(baseline_checkpoint)
                print("Recovered to baseline. Ready to try different approach.")
            break
```

### Pattern 2: Multi-Approach Comparison

Compare multiple refactoring approaches by checkpointing:

```python
async with ClaudeSDKClient(options) as client:
    approaches = {}

    # Approach A: Event-driven refactor
    approach_a_checkpoint = None
    await client.query("Refactor to event-driven architecture")
    async for message in client.receive_response():
        if isinstance(message, UserMessage) and message.uuid:
            approach_a_checkpoint = message.uuid
        if message.type == "result":
            approaches["event-driven"] = {
                "checkpoint": approach_a_checkpoint,
                "result": message.result
            }
            break

    # Rewind to baseline for Approach B
    await client.rewind_files(baseline_checkpoint)

    # Approach B: Service-oriented refactor
    approach_b_checkpoint = None
    await client.query("Refactor to service-oriented architecture")
    async for message in client.receive_response():
        if isinstance(message, UserMessage) and message.uuid:
            approach_b_checkpoint = message.uuid
        if message.type == "result":
            approaches["service-oriented"] = {
                "checkpoint": approach_b_checkpoint,
                "result": message.result
            }
            break

    # Compare both approaches
    print("Approach A (Event-Driven):")
    print(approaches["event-driven"]["result"])
    print("\nApproach B (Service-Oriented):")
    print(approaches["service-oriented"]["result"])

    # Keep the better one, rewind the other
    if "event-driven" in approaches["event-driven"]["result"].lower():
        print("Event-driven looks better. Keeping that checkpoint.")
        # approaches["event-driven"]["checkpoint"] is now the active state
    else:
        await client.rewind_files(approaches["event-driven"]["checkpoint"])
        print("Service-oriented looks better. Reverted to that checkpoint.")
```

## Checkpoint Overhead and Session Management

File checkpointing has minimal overhead (snapshots are efficient), but understanding session lifecycle matters.

### Session Persistence with Checkpoints

```python
# Checkpoints persist across session boundaries
# Save session ID and checkpoint during execution

session_id = None
final_checkpoint = None

async with ClaudeSDKClient(options) as client:
    await client.query("Initial task")

    async for message in client.receive_response():
        if hasattr(message, 'subtype') and message.subtype == 'init':
            session_id = message.session_id

        if isinstance(message, UserMessage) and message.uuid:
            final_checkpoint = message.uuid

        if message.type == "result":
            break

# Save these
print(f"Session: {session_id}, Checkpoint: {final_checkpoint}")

# Later: Resume and potentially rewind
async with ClaudeSDKClient(
    ClaudeAgentOptions(
        enable_file_checkpointing=True,
        resume=session_id
    )
) as client:
    # Files are still in state from final_checkpoint

    # Try new approach
    await client.query("Implement feature X")

    # If that fails, rewind to final_checkpoint
    async for message in client.receive_response():
        if message.type == "result" and "error" in message.result:
            await client.rewind_files(final_checkpoint)
            print("Reverted to saved checkpoint")
        break
```

## Why This Matters for Digital FTEs

Checkpointing is the **resilience mechanism** that makes agents safe for production:

**Without checkpointing**: A buggy agent modifies 50 files incorrectly. Your customer's codebase is corrupted. You manually fix it. Trust is destroyed.

**With checkpointing**: A buggy agent modifies 50 files. You detect the error immediately. One command reverts everything. Customer never knows anything went wrong. Agent tries a different approach. Success.

This is why checkpoint-enabled agents can be **bold**. They can explore aggressive refactorings, complex architectural changes, and risky optimizations—because failure is instantly recoverable.

## Try With AI

Use Claude Code or the Agent SDK for these checkpointing exercises.

### Prompt 1: Enable Checkpointing and Capture UUIDs

```
Set up a Claude Agent SDK session with file checkpointing enabled.
Create a small Python file with intentionally poor formatting (inconsistent
indentation, long lines, unclear variable names). Then ask the agent to:
1. Reformat the file properly
2. Show me how to capture checkpoint UUIDs during execution
3. Display the checkpoint IDs for reference

Your task: Write the code that enables checkpointing, runs the refactoring,
and prints all captured checkpoint UUIDs.
```

**What you're learning**: How to configure checkpointing and identify the UUID objects that represent file states. Notice that each tool execution creates a new checkpoint—you can recover to any of them.

### Prompt 2: Recover from a Deliberate Mistake

```
Using the same Python file from Prompt 1, ask the agent to:
1. Add a new function that has a syntax error (intentionally buggy)
2. Capture the checkpoint ID BEFORE adding the function
3. Let the agent try to fix the error
4. If the fix fails, show how rewindFiles() would recover to the
   pre-broken state

Your task: Demonstrate checkpoint capture → file modification → recovery.
Print the checkpoint IDs at each step so I can see the sequence.
```

**What you're learning**: The recovery pattern—how to detect when an agent task fails and use `rewindFiles()` to instantly restore files to a known-good state. This is the core resilience mechanism for production agents.

### Prompt 3: Compare Multiple Refactoring Approaches

```
Create a multi-approach comparison:
1. Start with a basic TODO app with global state
2. Refactor to approach A: Dependency injection (checkpoint this state)
3. Rewind files to before approach A
4. Refactor to approach B: Observer pattern (checkpoint this state)
5. Show me the code differences between the two approaches

Your task: Implement the multi-approach pattern—refactoring to two different
architectures from the same starting point, comparing which one is cleaner.
Store checkpoint IDs so you can switch between them.

**Safety note**: Always capture a baseline checkpoint before risky refactorings.
This is how production agents safely explore multiple solution paths.
```

**What you're learning**: Strategic checkpoint use—how teams would use checkpointing in real projects to safely explore architectural alternatives and compare solutions before committing to one.

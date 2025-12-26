---
sidebar_position: 7
title: "Session Management"
chapter: 36
lesson: 7
duration_minutes: 30
description: "Capture, resume, and fork agent sessions. Build agents that remember everything and explore multiple paths in parallel."
keywords: ["Claude Agent SDK", "Session Management", "Session Persistence", "Session Forking", "Long-Running Agents", "Context Preservation", "State Management", "Digital FTE Workflows"]

# HIDDEN SKILLS METADATA
skills:
  - name: "Session Capture and Resumption"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Technical Communication"
    measurable_at_this_level: "Student can capture a session ID from an agent query, store it, and resume the same session later with full context preservation"

  - name: "Session Forking for Parallel Exploration"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can fork a session at a checkpoint to explore alternative approaches, keep the original unchanged, and compare parallel outcomes"

  - name: "Long-Running Agent State Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can design agent workflows that maintain state across multiple invocations, identify where session boundaries occur, and preserve critical context for Digital FTE applications"

learning_objectives:
  - objective: "Extract session IDs from agent initialization messages and store them for later resumption"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student captures session_id from init message and verifies it works to resume the session"

  - objective: "Resume a previous session and continue work with full context preserved"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student resumes session and asks agent about earlier work—agent recalls full context"

  - objective: "Fork sessions to explore alternative approaches while preserving the original state"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student forks a session, explores approach A in fork, keeps original at checkpoint, compares outcomes"

cognitive_load:
  new_concepts: 7
  assessment: "Session capture, session resumption, session fork, checkpoint, context preservation, state lifecycle, parallel exploration. Concepts chunk naturally around the session lifecycle pattern and the unique forking capability."

differentiation:
  extension_for_advanced: "Implement session checkpointing with multiple forks—branch a session into 3 parallel explorations (A/B/C test approaches), then merge learnings. Build a session replay system that replays previous decisions in a new session. Implement session compaction to reduce transcript size while preserving critical context for long-running agents."
  remedial_for_struggling: "Start with simple session capture and resumption (no forking). Focus on understanding the session lifecycle: create → capture ID → resume → verify context. Use a checklist: 'Is the session_id captured?' → 'Does resume accept it?' → 'Does the agent remember?' Defer forking until comfortable with basic resumption."

---

# Session Management

Your agent remembers everything. Your sessions persist. Let's learn how to architect long-running agents that maintain state across days or weeks.

A **session** is a persistent conversation context. When you start an agent, a session begins. The agent accumulates knowledge, context, and decisions. When the session ends, that context is lost—unless you capture the session ID and resume later.

But here's the powerful part: You can fork a session. Imagine your agent reaches a decision point: "Should we refactor this module (Approach A) or rewrite from scratch (Approach B)?" Instead of choosing blindly, fork the session and explore both paths in parallel. Keep the original at the checkpoint. Compare what emerged. Then decide.

This capability transforms agents from one-shot tools into long-running team members that work 24/7 across weeks or months, accumulating expertise and maintaining continuity.

## Understanding Sessions

A session has three lifecycle events:

1. **Init** (creation): Agent starts, new session_id generated
2. **Resume** (continuation): Existing session_id provided, full context restored
3. **Fork** (branching): Session copied at checkpoint, two independent branches

Every agent query starts a session unless you explicitly resume one.

### Session Capture

When you call `query()`, the agent streams back messages. The first message contains initialization data—including the session ID.

```python
import asyncio
from claude_agent_sdk import query, ClaudeAgentOptions

async def main():
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash"]
    )

    session_id = None

    async for message in query(
        prompt="Analyze the test failures in tests/",
        options=options
    ):
        # Capture session ID from init message
        if hasattr(message, 'subtype') and message.subtype == 'init':
            session_id = message.session_id
            print(f"Session created: {session_id}")

        # Print other messages as they arrive
        if hasattr(message, 'content'):
            print(message.content)

    # Save session_id for later
    print(f"Save this ID to resume: {session_id}")

asyncio.run(main())
```

**Output:**
```
Session created: session_abc123def456
[Agent analyzes test failures...]
Save this ID to resume: session_abc123def456
```

The session_id is a unique identifier. Store it in a database, config file, or environment variable. You'll need it to resume.

### Resuming Sessions

Hours later (or days later), resume that exact session with full context preserved:

```python
async def main():
    session_id = "session_abc123def456"  # Retrieved from storage

    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash"],
        resume=session_id  # Resume this specific session
    )

    async for message in query(
        prompt="What were the critical failures we found?",
        options=options
    ):
        if hasattr(message, 'content'):
            print(message.content)

asyncio.run(main())
```

**What happens internally:**

1. SDK sends request with `resume: session_id`
2. Agent system restores full conversation history
3. Agent recalls everything from previous interactions
4. Agent responds with complete context awareness

The agent remembers:
- Files it analyzed previously
- Conclusions it reached
- Decisions made
- Constraints identified

This is **context preservation**—the opposite of starting fresh.

## Session Forking: The Unique Power

Standard tools can resume sessions. The Claude Agent SDK adds something unique: **forking**.

Session forking creates two independent branches from a checkpoint. The original session remains unchanged. Perfect for exploring alternatives.

### Use Case: A/B Testing Approaches

Your agent reaches a critical decision: "How should we optimize this database query?"

- **Approach A**: Index the most-queried columns
- **Approach B**: Implement query caching
- **Approach C**: Refactor to reduce queries

Instead of choosing one and hoping it works, fork:

```python
import asyncio
from claude_agent_sdk import query, ClaudeAgentOptions

async def analyze_approach(fork_session_id: str, approach: str) -> str:
    """Analyze one approach in a forked session"""
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash"],
        resume=fork_session_id
    )

    async for message in query(
        prompt=f"Implement and test {approach}. Measure performance improvements.",
        options=options
    ):
        if hasattr(message, 'result'):
            return message.result

async def main():
    # Start with original session analyzing the problem
    session_id = None
    async for message in query(
        prompt="Analyze database query performance bottleneck",
        options=ClaudeAgentOptions(allowed_tools=["Read", "Bash"])
    ):
        if hasattr(message, 'subtype') and message.subtype == 'init':
            session_id = message.session_id

    # Fork 1: Approach A (Indexing)
    fork_a = session_id + "_fork_indexing"
    result_a = await analyze_approach(fork_a, "query indexing strategy")
    print(f"Indexing approach result:\n{result_a}\n")

    # Fork 2: Approach B (Caching)
    fork_b = session_id + "_fork_caching"
    result_b = await analyze_approach(fork_b, "query caching strategy")
    print(f"Caching approach result:\n{result_b}\n")

    # Fork 3: Approach C (Refactoring)
    fork_c = session_id + "_fork_refactor"
    result_c = await analyze_approach(fork_c, "query refactoring strategy")
    print(f"Refactoring approach result:\n{result_c}\n")

    print("All three approaches explored in parallel. Compare outcomes and decide.")

asyncio.run(main())
```

**What happened:**

1. Original session analyzed the problem and identified it
2. Three independent forks branched from that checkpoint
3. Each fork explored one approach independently
4. All three could run in parallel (or sequentially)
5. You compare outcomes and choose the best path

**Original session remains unchanged** at the problem-analysis checkpoint. You can resume it again later, create different forks, or take a different branch entirely.

---

## Session Lifecycle for Long-Running Digital FTEs

Digital FTEs (Digital Full-Time Employees) need to persist for weeks or months. Session management is how they maintain continuity.

### Example: Email Triage Digital FTE

Imagine an AI agent that processes customer support emails. It needs to:

1. Remember customers it's helped before
2. Know about unresolved issues
3. Apply lessons learned from previous interactions
4. Build expertise over time

```python
import asyncio
from datetime import datetime
from claude_agent_sdk import query, ClaudeAgentOptions

class EmailTriageAgent:
    def __init__(self, session_storage_file: str):
        self.storage_file = session_storage_file
        self.session_id = self._load_session_id()

    def _load_session_id(self) -> str:
        """Load session_id from storage, or create new session"""
        try:
            with open(self.storage_file, 'r') as f:
                return f.read().strip()
        except FileNotFoundError:
            return None  # New session

    def _save_session_id(self, session_id: str):
        """Save session_id for next run"""
        with open(self.storage_file, 'w') as f:
            f.write(session_id)

    async def process_email(self, email_content: str) -> str:
        """Process one email in ongoing session"""
        options = ClaudeAgentOptions(
            allowed_tools=["Read", "Write", "Bash"],
            system_prompt="""You are an email triage specialist.
Your job: Categorize emails, identify urgent issues, draft responses.
Remember context from previous emails you've processed.
Build knowledge about recurring issues."""
        )

        # Resume existing session (or create new)
        if self.session_id:
            options.resume = self.session_id

        result = None
        async for message in query(prompt=email_content, options=options):
            # Capture session_id on first run
            if hasattr(message, 'subtype') and message.subtype == 'init':
                if not self.session_id:
                    self.session_id = message.session_id
                    self._save_session_id(self.session_id)

            if hasattr(message, 'result'):
                result = message.result

        return result

async def main():
    agent = EmailTriageAgent("agent_session.txt")

    emails = [
        "Customer: Database is down. Can't access portal. Critical!",
        "Customer: Account locked. Password reset not working.",
        "Customer: Billing discrepancy. Charged twice this month.",
    ]

    for i, email in enumerate(emails, 1):
        print(f"\n--- Email {i} ---")
        print(f"Input: {email}")
        response = await agent.process_email(email)
        print(f"Agent response:\n{response}")
        print(f"(Session ID persisted: {agent.session_id})")

asyncio.run(main())
```

**Output:**
```
--- Email 1 ---
Input: Customer: Database is down. Can't access portal. Critical!
Agent response:
Category: CRITICAL
Priority: P1
Suggested response: "We're investigating the database outage immediately..."
(Session ID persisted: session_xyz789)

--- Email 2 ---
Input: Customer: Account locked. Password reset not working.
Agent response:
Category: URGENT
Priority: P2
Context: [Agent recalls Email 1 context] This is different from the critical outage
Suggested response: "Let me reset your account directly..."
(Session ID persisted: session_xyz789)

--- Email 3 ---
Input: Customer: Billing discrepancy. Charged twice this month.
Agent response:
Category: STANDARD
Priority: P3
Context: [Agent recalls Emails 1-2] This is unrelated to infrastructure issues
Suggested response: "I'll investigate your billing..."
(Session ID persisted: session_xyz789)
```

**The agent:**
- Processed three emails with ONE session
- Accumulated context across emails
- Made decisions based on previous interactions
- Built expertise (recognizing patterns)
- **Persisted state** for next run

This is how Digital FTEs work. Not as one-shot tasks. As long-running team members that accumulate expertise.

---

## Checkpointing and Rollback

Sessions can be large. After processing 1000 emails, the transcript grows. You can checkpoint sessions—save critical state at moments—and rollback if needed.

```python
from claude_agent_sdk import ClaudeSDKClient, ClaudeAgentOptions

async def main():
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Edit", "Bash"],
        enable_file_checkpointing=True  # Enable checkpointing
    )

    checkpoint_id = None

    async with ClaudeSDKClient(options) as client:
        # Do work
        await client.query("Refactor the authentication module")
        async for msg in client.receive_response():
            if hasattr(msg, 'uuid'):
                checkpoint_id = msg.uuid
                print(f"Checkpoint saved: {checkpoint_id}")

        # More work
        await client.query("Run security audit on modified code")
        async for msg in client.receive_response():
            print(msg)

    # Later: Rollback to checkpoint if needed
    async with ClaudeSDKClient(options) as client:
        await client.query("")
        async for msg in client.receive_response():
            # Rewind to checkpoint (before the security audit)
            await client.rewind_files(checkpoint_id)
            print("Rolled back to checkpoint")
            break

asyncio.run(main())
```

---

## State Persistence Patterns for Digital FTEs

When building Digital FTEs, consider these patterns:

### Pattern 1: Session + Database

Store critical state in a database, session for conversation context:

```python
import json
from datetime import datetime

class KnowledgeBase:
    """Store decisions, learned patterns, customer history"""
    def __init__(self, db_file: str):
        self.db_file = db_file

    def log_decision(self, decision_id: str, content: dict):
        """Persist critical decisions outside session"""
        with open(self.db_file, 'a') as f:
            f.write(json.dumps({
                "timestamp": datetime.now().isoformat(),
                "id": decision_id,
                **content
            }) + "\n")

async def agent_with_knowledge():
    kb = KnowledgeBase("decisions.jsonl")
    agent = EmailTriageAgent("session.txt")

    # Agent processes email using session context
    response = await agent.process_email("Customer complaint about outage")

    # Critical decision logged to database (survives session resets)
    kb.log_decision(
        "critical_outage",
        {"issue": "Database down", "impact": "All users affected", "resolution": "Restore backup"}
    )
```

### Pattern 2: Session Handoff

Transfer a session to another agent:

```python
async def specialist_handoff(session_id: str, specialist_type: str):
    """Hand off session to specialized agent"""
    options = ClaudeAgentOptions(
        resume=session_id,
        system_prompt=f"You are a {specialist_type} specialist. Review previous context and take over."
    )

    async for message in query(
        prompt="Take over this case and provide specialist perspective",
        options=options
    ):
        print(message)

# In email triage workflow:
# 1. General triage agent processes email
# 2. If technical issue: handoff to technical specialist (same session_id)
# 3. If billing issue: handoff to billing specialist (same session_id)
```

---

## Try With AI

Build three practical session management workflows for your domain.

### Prompt 1: Design Session Capture and Resumption

```
I want to build an agent that processes [your domain] tasks.
The agent will start, do work, then be invoked again days later.

For this scenario:
1. What data should the agent capture about the first session?
2. How would you store the session_id so it can be retrieved later?
3. When resuming, what context should the agent remember?
4. What could go wrong if you DON'T use sessions (start fresh each time)?

Write Python code that:
- Captures the session_id after the first query
- Saves it to a config file
- Resumes the same session in a second query
- Verifies the agent remembers the first interaction

Include both the initial session code and the resumption code.
```

**What you're learning**: Session persistence is the foundation for long-running Digital FTEs. You're designing how state flows across invocations.

### Prompt 2: Fork Sessions for A/B Testing

```
Your agent reaches a decision point:
[Describe a specific decision in your domain, e.g., "Should we use algorithm A or algorithm B?"]

Design a session forking workflow:
1. How would you fork the session into 3 branches (one per approach)?
2. What would each fork do differently?
3. How would you compare the results?
4. Which approach would you recommend and why?

Write Python code that:
- Creates the initial session (analyzing the problem)
- Forks into 3 branches
- Each branch explores one approach
- Collects results and compares outcomes
```

**What you're learning**: Parallel exploration is how you make better decisions. Instead of guessing, you test multiple paths in parallel and choose based on evidence.

### Prompt 3: Design a Long-Running Digital FTE Workflow

```
Design a Digital FTE for [your domain] that needs to:
1. [Goal 1, e.g., "Process 50 customer emails daily"]
2. [Goal 2, e.g., "Build expertise about recurring issues"]
3. [Goal 3, e.g., "Improve response quality over weeks"]

For this agent:
1. How would you use sessions to maintain continuity?
2. What critical decisions would you checkpoint?
3. What would you store in a database vs in the session?
4. How would you handle rollbacks if something goes wrong?
5. What metrics would you track to measure improvement over time?

Show a complete class/system design that:
- Manages the session lifecycle
- Persists critical state
- Supports checkpointing
- Improves over time through accumulated context
```

**What you're learning**: This is Digital FTE architecture—designing agents that work as long-term team members, not one-off tools. Session management is how you build agents that are trustworthy enough to run unsupervised for days or weeks.

**Safety Note**: Session state grows over time. Monitor session size and implement compaction or archival strategies for very long-running agents. Never store sensitive customer data in sessions—persist it in encrypted databases instead. Sessions are conversation context, not data storage.

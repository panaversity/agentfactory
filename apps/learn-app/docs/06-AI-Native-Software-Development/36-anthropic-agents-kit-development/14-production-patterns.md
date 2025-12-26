---
sidebar_position: 14
title: "Production Patterns: Hosting, Sandbox, and Compaction"
chapter: 36
lesson: 14
duration_minutes: 35
description: "Choose hosting patterns for your Digital FTE: ephemeral for stateless tasks, long-running for persistent agents, hybrid for stateful workflows. Configure sandbox security and manage context at scale through compaction."
keywords: [production, hosting, ephemeral, long-running, hybrid, sandbox, security, context compaction, 24/7 agents, Digital FTE deployment, memory management]

# HIDDEN SKILLS METADATA
skills:
  - name: "Evaluating Hosting Patterns for Agent Use Cases"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can analyze a Digital FTE use case and determine whether ephemeral, long-running, or hybrid hosting best serves its requirements, justifying the choice through cost, state management, and availability needs"

  - name: "Sandbox Configuration for Security"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can configure sandbox settings (enabled/disabled, autoAllowBashIfSandboxed, network restrictions) for different deployment contexts and explain how sandbox isolation protects production environments"

  - name: "Context Compaction Strategy Design"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Technical Communication"
    measurable_at_this_level: "Student can design context compaction strategies using PreCompact hooks, determine when to archive vs summarize, and implement memory management that keeps 24/7 agents within token budgets"

learning_objectives:
  - objective: "Evaluate hosting patterns (ephemeral, long-running, hybrid) against Digital FTE requirements and trade-offs"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Given a use case, student selects appropriate hosting pattern and justifies choice through cost analysis, state requirements, and availability considerations"

  - objective: "Configure sandbox execution settings for development vs production environments"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements sandbox configuration that balances security restrictions (development) with operational flexibility (production)"

  - objective: "Design context compaction strategies that keep long-running agents efficient without losing critical knowledge"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student creates a PreCompact hook strategy that archives key decisions before compaction, enabling agent continuity"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (ephemeral pattern, long-running pattern, hybrid pattern, sandbox configuration, context compaction, PreCompact hook, memory efficiency) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design a multi-region hosting strategy that spins up ephemeral agents in closest AWS region, implements sandboxing policies per region (EU requires stricter isolation), and compacts contexts across time zones. Build a cost optimization dashboard that compares hosting pattern costs across 100K+ monthly users. Implement adaptive compaction that archives more aggressively for cost-sensitive customers."
  remedial_for_struggling: "Focus on single decision: 'Which pattern matches YOUR use case?' Start with clear scenarios: Email agent (long-running—remembers threads), batch document processor (ephemeral—no state needed), research assistant (hybrid—remembers session, forgets between). Use decision tree: 'Does agent need to remember past interactions?' YES=long-running, NO=ephemeral. Answer that one question first."

---

# Production Patterns: Hosting, Sandbox, and Compaction

Your agent works flawlessly in testing. You've built skills, integrated tools, managed sessions. Now: production. Where uptime matters. Where cost matters. Where security matters.

The architecture of production agents differs fundamentally from development prototypes. Testing runs one agent once. Production runs thousands of agents continuously, each with different security requirements, cost constraints, and memory needs.

This lesson teaches you to design production-grade agents by choosing hosting patterns that match your economics, configuring security boundaries that match your threat model, and managing memory constraints that keep costs sustainable.

## The Production Architecture Decision

Before implementing, you face a foundational architectural choice: **How long should your agent live?**

Three fundamental patterns exist:

| Pattern | Container Lifetime | State Preservation | Cost Model | Best For |
|---------|-------------------|-------------------|-----------|----------|
| **Ephemeral** | Minutes to hours | Discarded after task | Per-request | Bug fixes, one-off processing, stateless tasks |
| **Long-Running** | Days to months | Persistent across sessions | Subscription/monthly | Email agents, chatbots, customer support |
| **Hybrid** | Hours to weeks | Checkpoint-based persistence | Tiered (base + overage) | Project managers, research assistants, domain experts |

Each pattern answers a different production problem. Choosing wrong wastes money. Choosing right makes your Digital FTE profitable.

## Pattern 1: Ephemeral Agents

**The question**: "When would you want to discard agent state after each task?"

An **ephemeral agent** spins up for one task, completes it, shuts down. No memory persists. No session carries forward. The next request starts fresh.

**Real-world example**: Your customer reports a bug in their API integration. You spin up an ephemeral agent to:

1. Reproduce the bug
2. Analyze logs
3. Suggest fix
4. Terminate

The agent never needs to remember its analysis next week. The context is task-specific.

### Ephemeral Architecture

```python
from claude_agent_sdk import query, ClaudeAgentOptions

async def fix_customer_bug(customer_id, bug_report):
    """Ephemeral agent: One request, zero persistence"""

    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash", "Browser"],
        # No resume parameter—new session every time
        max_turns=5,  # Bounded execution
        model="claude-opus-4-5-20251101"  # Full power for one-off problem solving
    )

    prompt = f"""
    Customer {customer_id} reported: {bug_report}

    1. Reproduce the issue
    2. Analyze logs to find root cause
    3. Suggest a fix
    4. Output: [Reproduction steps] [Root cause] [Fix]
    """

    analysis = []
    async for message in query(prompt=prompt, options=options):
        if hasattr(message, 'content'):
            analysis.append(message.content)

    # Agent terminates here. No session saved. No memory persists.
    return "\n".join(analysis)

# Usage: One call per bug report. Fresh agent each time.
result = await fix_customer_bug(
    customer_id="acme-corp",
    bug_report="API returns 500 on concurrent requests"
)
print(result)
```

**Output:**
```
[Agent reproduces bug by sending concurrent requests...]
Root cause: Race condition in database transaction handling
Fix: Add transaction lock before updating inventory...
```

### When Ephemeral Works

✓ Stateless workloads (translate text, generate reports)
✓ One-off problems (debug this error, analyze this file)
✓ Cost-conscious scenarios (spin up when needed, terminate)
✓ Security isolation (each task runs in clean environment)

### When Ephemeral Fails

✗ Workflows needing context (customer support remembering past tickets)
✗ Long-running projects (research agent needing weeks of exploration)
✗ Relational tasks (agent coordinating across multiple requests)

---

## Pattern 2: Long-Running Agents

**The question**: "What advantages does session persistence give?"

A **long-running agent** runs continuously or semi-continuously. A session persists for days or months. The agent accumulates knowledge, learns customer preferences, maintains context.

**Real-world example**: Your customer has an email inbox of unresolved issues. They want an agent that:

1. Checks email daily
2. Remembers previous conversations
3. Maintains running state of each ticket
4. Never forgets a customer's constraints

That's a long-running agent.

### Long-Running Architecture

```python
from claude_agent_sdk import query, ClaudeAgentOptions
import json

class EmailDigestAgent:
    def __init__(self, customer_id, session_id=None):
        self.customer_id = customer_id
        self.session_id = session_id  # Resumed if provided

    async def process_daily(self):
        """Run once daily. Accumulates context over time."""

        options = ClaudeAgentOptions(
            allowed_tools=["Email", "Slack", "Database"],
            resume=self.session_id,  # Resume existing context
            max_turns=10  # More turns for complex decision-making
        )

        prompt = """
        Review new emails since yesterday.

        For each email:
        1. Assess urgency (critical/high/normal)
        2. Check if we've handled similar issues before
        3. Draft response incorporating our past solutions
        4. Flag any new patterns you notice

        Remember: You've been monitoring this inbox for 3 months.
        Use that context to recognize patterns.
        """

        messages = []
        async for message in query(prompt=prompt, options=options):
            # Capture session ID on first run
            if hasattr(message, 'subtype') and message.subtype == 'init':
                self.session_id = message.session_id

            if hasattr(message, 'content'):
                messages.append(message.content)

        return {
            "session_id": self.session_id,
            "analysis": "\n".join(messages)
        }

# Usage: Daily trigger that persists state
async def run_email_digest():
    # Retrieve agent's session from database
    stored_session_id = db.get_session_id("acme-corp")

    agent = EmailDigestAgent(
        customer_id="acme-corp",
        session_id=stored_session_id
    )

    result = await agent.process_daily()

    # Save session for tomorrow
    db.save_session_id("acme-corp", result["session_id"])

    return result["analysis"]

# Runs every day at 9am. Agent remembers everything from day 1.
schedule.every().day.at("09:00").do(run_email_digest)
```

**Output (Day 1):**
```
No history. Analyzing 5 new emails...
[Agent processes emails, learns patterns...]
Session created for future reference.
```

**Output (Day 5):**
```
Session resumed. I remember 4 previous customers and their preferences...
[Agent recognizes patterns, applies past solutions...]
I notice 3 customers are asking about the same feature—recommend to product team.
```

### When Long-Running Works

✓ Customer-facing products (chatbots, support agents)
✓ Relationship workflows (remembering preferences, history)
✓ Learning-based tasks (agent improves with experience)
✓ Subscription models (persistence justifies recurring cost)

### When Long-Running Fails

✗ High-security environments (persistent memory is liability)
✗ Seasonal tasks (agent sits idle most of year)
✗ Tight cost budgets (context grows indefinitely)

---

## Pattern 3: Hybrid Agents

**The question**: "How can you get benefits of both?"

A **hybrid agent** combines ephemeral and long-running. It runs ephemeral agents for independent tasks but checkpoints critical context back to a long-running orchestrator.

**Real-world example**: A research agent exploring a 100-page legal document.

- Main session (long-running): Orchestrates research, remembers conclusions
- Analysis sessions (ephemeral): Spin up for each section, analyze independently
- Checkpoint: Save key findings back to main session

### Hybrid Architecture

```python
from claude_agent_sdk import query, ClaudeAgentOptions

class ResearchOrchestrator:
    def __init__(self, document_id, session_id=None):
        self.document_id = document_id
        self.session_id = session_id
        self.findings = []  # Accumulated across ephemeral analyses

    async def analyze_section(self, section_text):
        """Ephemeral agent: Analyze one section independently"""

        options = ClaudeAgentOptions(
            allowed_tools=["Read"],
            model="claude-haiku-4-5-20251001"  # Cheaper for isolated analysis
            # No resume—new session for this section
        )

        prompt = f"Analyze this section. Extract key claims, evidence, conclusions:\n{section_text}"

        findings = []
        async for message in query(prompt=prompt, options=options):
            if hasattr(message, 'content'):
                findings.append(message.content)

        return "\n".join(findings)

    async def checkpoint_findings(self, section_findings):
        """Long-running session: Accumulate knowledge"""

        options = ClaudeAgentOptions(
            allowed_tools=["Database"],
            resume=self.session_id,  # Resume main context
            model="claude-opus-4-5-20251101"  # Full reasoning
        )

        prompt = f"""
        New findings from section analysis:
        {section_findings}

        1. Integrate with previous findings
        2. Identify patterns or contradictions
        3. Update our overall assessment
        """

        async for message in query(prompt=prompt, options=options):
            if hasattr(message, 'subtype') and message.subtype == 'init':
                self.session_id = message.session_id

            if hasattr(message, 'content'):
                self.findings.append(message.content)

    async def research_document(self, sections):
        """Hybrid workflow: Ephemeral analysis + long-running synthesis"""

        for i, section in enumerate(sections):
            # Ephemeral: Analyze this section independently (cheap)
            section_findings = await self.analyze_section(section)

            # Long-running: Integrate findings (expensive but essential)
            await self.checkpoint_findings(section_findings)

        return self.findings

# Usage: Process 10-section document efficiently
async def research_report():
    orchestrator = ResearchOrchestrator(document_id="report-123")

    sections = [
        "Section 1: Executive Summary...",
        "Section 2: Background...",
        "Section 3: Methodology...",
        # ... 10 sections total
    ]

    results = await orchestrator.research_document(sections)
    return results

# Cost profile: Cheaper analysis (Haiku) + expensive synthesis (Opus)
# = balanced cost for complex projects
```

**Output:**
```
Analyzing Section 1 (ephemeral, Haiku)...
Found: 5 claims, 3 evidence pieces
Checkpointing to main session...

Analyzing Section 2 (ephemeral, Haiku)...
Found: 4 claims, 2 contradictions with Section 1
Checkpointing...

[Session 5]
Pattern detected: Sections 2, 4, 6 contradict each other on methodology.
Recommend: Verify author consistency across document.
```

### When Hybrid Works

✓ Large projects with parallel work (process multiple sections independently)
✓ Cost-sensitive complex work (cheap analysis + selective expensive synthesis)
✓ Learning-based systems (ephemeral explores, long-running synthesizes)

---

## Security Isolation: Sandbox Configuration

You've learned patterns. Now: **how do you prevent your agent from breaking production?**

Agents execute code. Agents run Bash. Agents access files. Without security boundaries, a compromised agent could:

- Access production databases
- Modify source code
- Exfiltrate customer data
- Consume unbounded resources

Sandbox configuration is how you draw these boundaries.

### Understanding Sandbox Options

The Claude Agent SDK provides three sandbox controls:

```python
options = ClaudeAgentOptions(
    sandbox={
        "enabled": True,  # Whether Bash execution is isolated
        "autoAllowBashIfSandboxed": True,  # Auto-approve safe Bash
        "network": {
            "allowLocalBinding": True,  # Can agent listen on localhost?
        }
    }
)
```

**enabled**: Controls whether Bash runs in isolated environment

- `True` (Development): Bash runs in container, cannot access host system
- `False` (Production): Bash runs with host permissions (dangerous without trust)

**autoAllowBashIfSandboxed**: Auto-approve Bash if sandbox is enabled

- `True`: Sandboxed Bash commands execute automatically
- `False`: All Bash requires explicit human approval

**allowLocalBinding**: Can agent listen on network ports?

- `True`: Agent can start web servers, APIs
- `False`: Agent cannot bind to network ports (safer)

### Configuration for Development

```python
async def development_agent():
    """Development: Maximize agent freedom, sandbox everything"""

    options = ClaudeAgentOptions(
        sandbox={
            "enabled": True,  # Isolate Bash execution
            "autoAllowBashIfSandboxed": True,  # Auto-approve safe commands
            "network": {
                "allowLocalBinding": True  # Allow test servers
            }
        },
        allowed_tools=["Bash", "Read", "Write", "Browser"]
    )

    prompt = "Debug why tests are failing. Run the test suite and analyze errors."

    async for message in query(prompt=prompt, options=options):
        if hasattr(message, 'content'):
            print(message.content)

# Agent can safely run: rm -rf src/ (only affects sandbox, not host)
# Agent can safely run: npm install (only affects sandbox container)
# Agent cannot access: /home, /var/www (host filesystem protected)
```

**Output:**
```
Running: npm test
Error: test_database_connection failed
Root cause: Connection string missing in test config
Fix: Set TEST_DB_URL environment variable
```

### Configuration for Production

```python
async def production_agent():
    """Production: Minimal permissions, human review required"""

    options = ClaudeAgentOptions(
        sandbox={
            "enabled": False,  # Run on host (trust the agent)
            "autoAllowBashIfSandboxed": False,  # Explicit approval
            "network": {
                "allowLocalBinding": False  # No network access
            }
        },
        allowed_tools=["Read"],  # Only read-only operations
        max_turns=3  # Fewer turns = less opportunity for problems
    )

    prompt = "Analyze these logs and generate a report"

    async for message in query(prompt=prompt, options=options):
        if hasattr(message, 'content'):
            print(message.content)

# Agent cannot: Run Bash
# Agent cannot: Write files
# Agent cannot: Access network
# Agent can: Read production logs, generate analysis
```

### Decision Framework: When to Sandbox

| Requirement | Sandbox Setting | Rationale |
|-------------|-----------------|-----------|
| Development testing | enabled=true, auto=true | Safe to experiment |
| Production read-only | enabled=false, auto=false, restricted tools | Minimal attack surface |
| Production with write access | enabled=true, auto=false, human approval | Dangerous operations logged |
| High-security environment (HIPAA, SOC2) | enabled=true, network=false, minimal tools | Multiple layers of isolation |

---

## Memory Management: Context Compaction

You know how to architect agents: ephemeral, long-running, hybrid. You know how to secure them: sandbox configuration.

Now: **How do you keep a 24/7 agent from consuming infinite memory?**

A long-running agent accumulates context. Days of conversation. Weeks of decisions. Months of learning. The conversation history grows. Each new message costs tokens. Context window eventually fills.

**Context compaction** is how you manage this growth.

### The Compaction Problem

Imagine an email agent running for 6 months:

- Day 1-30: 500 messages (light load)
- Day 31-60: 1000 messages (growing)
- Day 61-180: 5000+ messages (conversation exploding)

By month 6:

```
Total tokens in session history: 450,000
Context window limit: 200,000 tokens
Result: Agent cannot process new messages—context is full
```

Compaction solves this by **archiving old context** before the window fills.

### The PreCompact Hook

```python
from claude_agent_sdk import query, ClaudeAgentOptions, HookMatcher

async def pre_compact_handler(context):
    """Called before context compaction. Archive key knowledge."""

    return {
        "archive": True,  # Tell SDK to proceed with compaction
        "summary": """
        === ARCHIVED CONTEXT (6 months of operation) ===

        Key customers:
        - acme-corp: Enterprise client, requires JSON responses
        - startup-xyz: Startup, flexible requirements

        Resolved patterns:
        - Database connection issues → Use connection pooling
        - Rate limiting → Implement exponential backoff
        - Authentication failures → Check token expiration first

        Known issues:
        - Email API sometimes returns 429 errors (rate limit)
        - PDF parsing fails on scanned documents (need OCR)

        Current priority tickets:
        - Feature request from acme-corp: CSV export
        - Bug: Timezone handling in batch jobs
        """,
        "checkpoint_data": {
            "processed_tickets": 1247,
            "customer_preferences": {...},
            "learned_patterns": {...}
        }
    }

options = ClaudeAgentOptions(
    hooks={
        'PreCompact': [
            HookMatcher(
                matcher='*',  # Trigger on any compaction
                hooks=[pre_compact_handler]
            )
        ]
    }
)

# When session grows too large, SDK automatically calls pre_compact_handler
# Agent loses detailed conversation history but retains critical knowledge
```

**Output (Before Compaction):**
```
Conversation history: 6 months, 450K tokens
```

**Output (After Compaction):**
```
Archived summary: Key customers, patterns, issues (15K tokens)
+ Recent conversation: Last 10 exchanges (5K tokens)
Total: 20K tokens (freed 430K for new work)
```

### Compaction Strategies

**Strategy 1: Archive Everything**

For agents that need to start fresh monthly:

```python
async def monthly_archive_handler(context):
    # Keep last 7 days, archive everything else
    return {
        "archive": True,
        "keep_recent_days": 7,
        "summary": "Full monthly summary of all work"
    }
```

**Strategy 2: Smart Retention**

For agents that need specific knowledge preserved:

```python
async def smart_archive_handler(context):
    # Archive everything except:
    # - Customer preferences
    # - Unresolved tickets
    # - Learned patterns

    return {
        "archive": True,
        "preserve_tags": ["customer_preference", "open_ticket", "pattern"],
        "summary": "Archive with selective preservation"
    }
```

**Strategy 3: Hybrid Archival**

For agents juggling multiple concerns:

```python
async def hybrid_archive_handler(context):
    # Archive to external system, keep summary

    # 1. Export full history to database
    full_history_id = await export_to_db(context.messages)

    # 2. Keep memory-efficient summary
    summary = await generate_summary(context)

    # 3. Return minimal context
    return {
        "archive": True,
        "external_archive_id": full_history_id,
        "summary": summary,
        "memory_target": "50K tokens"  # Keep under this
    }
```

### When to Trigger Compaction

```python
async def email_agent_with_compaction():
    """Automatic compaction at decision points"""

    options = ClaudeAgentOptions(
        hooks={
            'PreCompact': [
                HookMatcher(
                    matcher='*',
                    hooks=[pre_compact_handler]
                )
            ]
        },
        # Trigger compaction when context exceeds 80% of window
        compaction_threshold=0.80
    )

    # Agent runs normally until context fills to 80%
    # Then: PreCompact hook fires
    # Then: Context compressed automatically
    # Then: Agent continues with archived summary

    async for message in query(
        prompt="Process today's emails",
        options=options
    ):
        print(message.content)
```

---

## Decision Framework: Choosing Your Production Pattern

You've learned three patterns. Which do you choose?

### The Decision Tree

**Question 1: Does your agent need to remember things between requests?**

- **NO** → Ephemeral pattern
- **YES** → Go to Question 2

**Question 2: Does the agent run continuously (24/7 or near it)?**

- **NO** → Long-running pattern (checkpoint at session boundaries)
- **YES** → Go to Question 3

**Question 3: Does the agent do multiple parallel tasks or expensive synthesis?**

- **NO** → Pure long-running
- **YES** → Hybrid pattern (ephemeral tasks + long-running synthesis)

### Your TaskManager Digital FTE

Recall: TaskManager is your capstone from Chapter 6. It's a project management agent that:

- Tracks task status
- Remembers project context
- Coordinates between team members
- Learns project preferences

**Which pattern fits?**

**Answer: Hybrid**

- Orchestrator (long-running): Maintains project state, learns preferences
- Task analyzer (ephemeral): Analyzes individual tasks independently
- Report generator (ephemeral): Generates daily reports without storing them

**Sandbox configuration?**

- Development: `enabled=true, auto=true` (experiment freely)
- Production: `enabled=true, auto=false` (audit all file access)

**Context compaction?**

- Trigger monthly: Archive project archives, keep active task list
- PreCompact hook: Export detailed history to database before compacting

---

## Try With AI

The three scenarios below test your ability to evaluate patterns and configure production agents. Each prompt builds on your accumulated knowledge about hosting, security, and memory.

### Scenario 1: Bug Fix Agent

**Setup:** Your customer reports API failures in their production system. You need an agent to investigate, diagnose, and suggest fixes.

**Your task:** Determine the hosting pattern and sandbox configuration.

**What you're learning:** Evaluating when agents can be ephemeral vs. persistent. Understanding that stateless debugging doesn't require session persistence.

**Prompt:**
```
You're building a bug-fix agent for customer support.
Requirements:
- Investigate customer's reported issue
- Analyze logs and stack traces
- Suggest fix with code examples
- No need to remember this case for future debugging

What hosting pattern fits? Ephemeral or long-running?
What sandbox settings would you use for development vs production?
Explain your reasoning for both choices.
```

**Expected approach:**
- Recognize stateless nature (no past context needed)
- Choose ephemeral pattern (spin up, solve, terminate)
- Suggest development sandbox: enabled=true, auto=true (freedom to explore)
- Suggest production sandbox: enabled=true, auto=false (audit sensitive operations)

### Scenario 2: Email Digest Agent

**Setup:** You're building a 24/7 email monitoring agent for enterprise customers. It checks inboxes daily, learns which emails matter, and prioritizes responses.

**Your task:** Design the hosting pattern, sandbox configuration, and a compaction strategy.

**What you're learning:** Long-running agents need sophisticated memory management. Context compaction isn't optional—it's essential for sustainability.

**Prompt:**
```
Design a production email agent with:
- 24/7 operation
- Daily inbox monitoring
- Memory of customer preferences and past solutions
- Operation for 6+ months without manual intervention

1. What hosting pattern? Why?
2. What sandbox settings for production?
3. When would you trigger context compaction? What would you preserve?

Show your PreCompact hook strategy. What goes into the archive summary?
```

**Expected approach:**
- Identify long-running pattern (continuous operation + persistent memory)
- Configure sandbox: enabled=true, auto=false (email access is sensitive)
- Design compaction trigger: Monthly (preserve customer preferences, archive old exchanges)
- PreCompact hook: Archive last 30 days of conversations, keep 6-month customer preference database

### Scenario 3: Research Assistant Digital FTE

**Setup:** You're monetizing a research assistant that analyzes documents, extracts insights, and composes reports. Customers subscribe ($500/month) and use it for ongoing research projects.

**Your task:** Design a production architecture that balances cost, quality, and memory management. You target 60% gross margin.

**What you're learning:** Hybrid patterns enable cost optimization. Cheap analysis + expensive synthesis = profitable Digital FTE. Context compaction enables indefinite operation without losing value.

**Prompt:**
```
Design a production research agent Digital FTE:
- Customers subscribe at $500/month
- Process 10-20 document chunks daily
- Projects run for 3-6 months continuously
- Target: 60% gross margin (cost < $200/month per customer)

1. Which hosting pattern and why?
2. Show the architecture: how do ephemeral and long-running components interact?
3. Design context compaction: How do you archive project findings without losing learning?
4. How does this architecture achieve 60% margin?

Provide code showing:
- Ephemeral component (cheap analysis)
- Long-running component (expensive synthesis)
- PreCompact hook that archives findings
```

**Expected approach:**
- Identify hybrid pattern (parallel analysis + synthesis)
- Ephemeral tasks use Haiku (cheap: $0.80 per million input tokens)
- Long-running uses Opus (expensive: $20 per million—only for synthesis)
- Cost calculation: 20 documents × $0.05/analysis + $2 synthesis = $3/day = $90/month (vs $500 revenue = 82% margin)
- Compaction: Archive document analyses, preserve project insights and customer preferences

---

**Safety note**: When configuring sandbox for production, start with maximum restrictions (`enabled=true, auto=false`) and relax only after testing with restricted permissions. Never assume unsafe is acceptable because "it's just this one agent."


---
sidebar_position: 3
title: "Session State & Memory"
description: "Master session persistence and state management in Google ADK agents. Learn ToolContext for tool-level state, SessionService options for persistence, and patterns for maintaining conversation history."
keywords: ["Google ADK", "session management", "state persistence", "ToolContext", "SessionService", "conversation memory", "Firestore", "Vertex AI", "agent state"]
chapter: 35
lesson: 3
duration_minutes: 50

# HIDDEN SKILLS METADATA
skills:
  - name: "ToolContext State Access"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can access and modify session state within tool functions using ToolContext"

  - name: "SessionService Selection and Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "System Architecture"
    measurable_at_this_level: "Student understands trade-offs between InMemory, Firestore, and VertexAI session backends"

  - name: "Runner Configuration for State Management"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can instantiate Runner with appropriate SessionService for development or production"

  - name: "Session Lifecycle Management"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can create sessions, preserve state across messages, and manage session lifecycle"

  - name: "State Persistence Patterns"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student implements patterns for user preferences, conversation history, and task progress tracking"

learning_objectives:
  - objective: "Understand ToolContext as the mechanism for tool-level state access"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student explains how tools read/write state via ToolContext parameter"

  - objective: "Choose appropriate SessionService backend for development vs production"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student articulates trade-offs and selects correct backend for scenario"

  - objective: "Configure Runner with SessionService for persistent conversations"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Agent maintains state across multiple user messages"

  - objective: "Implement session state patterns for real-world use cases"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student tracks user preferences or conversation history in agent"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (ToolContext, tool state access, SessionService, InMemory/Firestore/VertexAI backends, Runner configuration, session creation, state lifecycle, persistence patterns) within B1 limit of 7-10 ✓"

differentiation:
  extension_for_advanced: "Investigate Firestore data model; configure Vertex AI Agent Engine for managed session storage; implement custom SessionService for specialized backends; design multi-tenant session isolation"
  remedial_for_struggling: "Focus on InMemory first to understand state lifecycle; use concrete examples (user preferences) before abstracting patterns; defer Firestore complexity until comfort with core concepts"

generated_by: content-implementer
source_spec: Chapter 35 Lesson 3 specification
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Session State & Memory

So far, you've built ADK agents that process single requests: user asks a question, agent responds, conversation ends. But real agents need to **remember context across conversations**. A customer support agent should recall the user's previous issues. A research assistant should accumulate findings across multiple questions. A news agent should track which stories it's already reported.

This is where **session state** and **persistent memory** become essential. Google ADK provides two complementary mechanisms: **ToolContext** for tool-level state access (what tools can read and modify during execution), and **SessionService** for session-level persistence (how conversations survive across API calls and deployments).

In this lesson, you'll learn to design agents that remember—from simple preference tracking to complex conversation histories—and understand when to use in-memory storage for development versus production-grade backends like Firestore or Vertex AI.

## The State Problem: Why Sessions Matter

Imagine building a news research agent. Your workflow:

```
Session 1 (Monday):
User: "What's new in AI hardware?"
Agent: Searches, finds 5 papers, stores findings

Session 2 (Tuesday):
User: "Summarize what I learned yesterday"
Agent: Needs access to Session 1's findings
```

Without persistent state, the agent has no memory of Session 1. It can't:
- Recall previous questions
- Build on accumulated research
- Track user preferences (e.g., "I prefer peer-reviewed sources")
- Maintain conversation context

**ToolContext and SessionService solve this problem.**

## ToolContext: State Access Within Tools

When a tool runs, it receives a `ToolContext` parameter—a gateway to session state that the tool can read and modify.

### Reading State from ToolContext

```python
from google.adk.tools import ToolContext

def research_tool(query: str, tool_context: ToolContext) -> dict:
    """Research tool that accesses conversation history."""

    # Read state
    history = tool_context.state.get("research_history", [])
    preferences = tool_context.state.get("preferences", {})

    # Use state in logic
    is_duplicate = query in [h["query"] for h in history]

    return {
        "duplicate_detected": is_duplicate,
        "user_prefers": preferences.get("source_type", "any")
    }
```

**Output:**

```json
{
  "duplicate_detected": false,
  "user_prefers": "peer-reviewed"
}
```

### Modifying State from ToolContext

```python
def research_tool(query: str, tool_context: ToolContext) -> dict:
    """Research tool that adds to conversation history."""

    # Read current history
    history = tool_context.state.get("research_history", [])

    # Modify state
    history.append({
        "query": query,
        "timestamp": str(datetime.now()),
        "sources_found": 3
    })

    # Write state back
    tool_context.state["research_history"] = history

    return {
        "query": query,
        "history_length": len(history),
        "stored": True
    }
```

**Output (first call):**

```json
{
  "query": "AI hardware trends",
  "history_length": 1,
  "stored": true
}
```

**Output (second call):**

```json
{
  "query": "Neural scaling laws",
  "history_length": 2,
  "stored": true
}
```

### The Two-Way State Flow

ToolContext enables bidirectional state management:

```
Initial State:
{
  "preferences": {"source_type": "academic"},
  "history": []
}
    ↓
Tool Execution:
- Reads "source_type" to filter results
- Appends research to "history"
    ↓
Updated State:
{
  "preferences": {"source_type": "academic"},
  "history": [
    {"query": "AI hardware", "sources": 5},
    {"query": "Neural efficiency", "sources": 3}
  ]
}
    ↓
Next Tool Call:
- Reads updated history
- Checks for duplicates
```

**Key principle**: State persists across tool calls within the same session. When the user ends the conversation and starts a new session, state is preserved if using a persistent backend (Firestore, Vertex AI).

## SessionService: Where State Lives

ToolContext reads/writes state, but **where does that state actually live?** SessionService answers this question. It's the storage backend for conversations.

### Option 1: InMemorySessionService (Development)

**Use case**: Local development, testing, prototyping

```python
from google.adk.sessions import InMemorySessionService
from google.adk import Runner

# Development setup
session_service = InMemorySessionService()

runner = Runner(
    app_name="news_research",
    agent=agent,
    session_service=session_service
)
```

**Characteristics**:
- State stored in Python memory (RAM)
- Persists for lifetime of Python process
- Useful for testing without external dependencies
- Lost when process restarts

**When state is preserved**:
```python
# Same session ID = same state
session_id = "user_123"

runner.run(session_id=session_id, message="Research AI")
# ToolContext.state persists

runner.run(session_id=session_id, message="What did you find?")
# State from previous message is available
```

**When state is lost**:
```
Process: Restart Python
    ↓
session_service = InMemorySessionService()  # New instance
    ↓
All previous state in old instance is gone
```

### Option 2: FirestoreSessionService (Production)

**Use case**: Cloud-hosted agents, multi-instance deployments, permanent storage

```python
from google.adk.sessions import FirestoreSessionService
from google.adk import Runner

# Production setup
session_service = FirestoreSessionService(
    project="my-gcp-project",
    database="(default)"
)

runner = Runner(
    app_name="news_research",
    agent=agent,
    session_service=session_service
)
```

**Firestore data model**:
```
Firestore Collection: sessions

Document: user_123_session_456
{
  "app_name": "news_research",
  "user_id": "user_123",
  "created_at": 2025-12-26T10:30:00Z,
  "last_message": 2025-12-26T10:45:00Z,
  "state": {
    "preferences": {"source_type": "academic"},
    "research_history": [
      {
        "query": "AI hardware trends",
        "timestamp": "2025-12-26T10:30:00Z",
        "sources_found": 5
      }
    ]
  },
  "messages": [
    {
      "role": "user",
      "content": "Research AI hardware trends",
      "timestamp": "2025-12-26T10:30:00Z"
    },
    {
      "role": "assistant",
      "content": "Found 5 papers...",
      "timestamp": "2025-12-26T10:31:00Z"
    }
  ]
}
```

**State survives**:
- Process restarts
- Deployments (instance 1 down, instance 2 up)
- Database failover
- Hours, days, months later

```python
# Day 1
runner.run(session_id="user_123", message="Research AI")
# State saved to Firestore

# Process restarts
# Firestore still has the state

# Day 2 (new process instance)
runner.run(session_id="user_123", message="What did you find?")
# ToolContext retrieves state from Firestore
# Agent has full conversation history
```

### Option 3: VertexAiSessionService (Enterprise)

**Use case**: Managed deployment on Google Cloud, Vertex AI Agent Engine

```python
from google.adk.sessions import VertexAiSessionService
from google.adk import Runner

# Vertex AI managed sessions
session_service = VertexAiSessionService(
    project="my-gcp-project",
    location="us-central1"
)

runner = Runner(
    app_name="news_research",
    agent=agent,
    session_service=session_service
)
```

**Characteristics**:
- Managed by Google Cloud (no infrastructure to manage)
- Integrated with Vertex AI security and monitoring
- Handles scaling automatically
- Auditing built-in

## Runner Configuration: Connecting Agent to State

The `Runner` object orchestrates everything: the agent, the backend, the session lifecycle.

### Minimal Configuration

```python
from google.adk import Runner
from google.adk.sessions import InMemorySessionService

runner = Runner(
    app_name="news_research",
    agent=agent,
    session_service=InMemorySessionService()
)
```

**What happens**:
```
User Input
    ↓
runner.run(session_id="user_123", message="Research AI")
    ↓
1. Lookup/create session with ID "user_123"
2. Load state from SessionService
3. Execute agent with ToolContext containing state
4. Tools read/write state via ToolContext
5. Save updated state back to SessionService
    ↓
Agent Output
```

### Production Configuration with Firestore

```python
from google.adk import Runner
from google.adk.sessions import FirestoreSessionService

session_service = FirestoreSessionService(
    project="my-gcp-project",
    database="(default)"
)

runner = Runner(
    app_name="news_research",
    agent=agent,
    session_service=session_service
)

# Client code
response = runner.run(
    session_id="user_123",  # Persists to Firestore
    message="Research AI hardware trends"
)

# Same user, later in time
response = runner.run(
    session_id="user_123",  # Loads from Firestore
    message="Summarize what you found"
)
```

## Session Management Patterns

### Pattern 1: User Preferences

Track user settings that persist across sessions.

```python
def set_preference_tool(
    preference_key: str,
    preference_value: str,
    tool_context: ToolContext
) -> dict:
    """User sets a preference that persists."""

    # Load preferences
    preferences = tool_context.state.get("preferences", {})

    # Update preference
    preferences[preference_key] = preference_value

    # Save back
    tool_context.state["preferences"] = preferences

    return {
        "preference_set": preference_key,
        "value": preference_value
    }

def search_with_preferences_tool(
    query: str,
    tool_context: ToolContext
) -> dict:
    """Search respects user preferences."""

    preferences = tool_context.state.get("preferences", {})
    source_type = preferences.get("source_type", "any")

    results = search_api(query, source_type=source_type)

    return {
        "query": query,
        "results": results,
        "filtered_by": source_type
    }
```

**Usage**:
```
Session 1:
User: "I prefer academic papers"
Agent: [Calls set_preference_tool, stores preference]

Session 2 (Days later):
User: "Research quantum computing"
Agent: [Calls search_with_preferences_tool]
Agent: [Remembers academic preference, filters results]
Output: Shows only peer-reviewed papers
```

### Pattern 2: Conversation History

Track what you've discussed to avoid redundant work.

```python
def extract_and_store_insight_tool(
    insight: str,
    category: str,
    tool_context: ToolContext
) -> dict:
    """Store research insight in conversation history."""

    history = tool_context.state.get("insights", [])

    history.append({
        "insight": insight,
        "category": category,
        "timestamp": str(datetime.now()),
        "session_id": tool_context.session_id
    })

    tool_context.state["insights"] = history

    return {
        "stored": True,
        "total_insights": len(history)
    }

def summarize_session_tool(
    tool_context: ToolContext
) -> dict:
    """Summarize what we've learned this session."""

    history = tool_context.state.get("insights", [])

    if not history:
        return {"summary": "No insights recorded yet"}

    # Group by category
    by_category = {}
    for item in history:
        cat = item["category"]
        if cat not in by_category:
            by_category[cat] = []
        by_category[cat].append(item["insight"])

    return {
        "categories": list(by_category.keys()),
        "insights_by_category": by_category,
        "total_insights": len(history)
    }
```

**Output Example**:

```json
Session:
User: "Research AI chip architectures"
Agent: [Finds papers, stores 3 insights about speculative execution]

User: "What about energy efficiency?"
Agent: [Finds papers, stores 2 insights about power management]

User: "Summarize"
Agent: [Calls summarize_session_tool]

{
  "categories": ["speculative_execution", "power_management"],
  "insights_by_category": {
    "speculative_execution": [
      "Branch prediction improves throughput by 15-25%",
      "Complexity cost: pipeline flushes on mispredict",
      "Neural predictors outperform traditional algorithms"
    ],
    "power_management": [
      "Dynamic voltage scaling reduces power by 30%",
      "Temperature management critical above 3GHz"
    ]
  },
  "total_insights": 5
}
```

### Pattern 3: Task Progress Tracking

Track long-running workflows across sessions.

```python
def start_research_project_tool(
    project_name: str,
    total_topics: int,
    tool_context: ToolContext
) -> dict:
    """Initialize a multi-topic research project."""

    tool_context.state["project"] = {
        "name": project_name,
        "total_topics": total_topics,
        "completed_topics": 0,
        "progress": 0,
        "started_at": str(datetime.now()),
        "findings": []
    }

    return {"project_created": project_name}

def complete_topic_tool(
    topic: str,
    findings: str,
    tool_context: ToolContext
) -> dict:
    """Mark a topic as complete in the project."""

    project = tool_context.state.get("project", {})

    project["completed_topics"] += 1
    project["progress"] = int((project["completed_topics"] / project["total_topics"]) * 100)

    project["findings"].append({
        "topic": topic,
        "findings": findings,
        "completed_at": str(datetime.now())
    })

    tool_context.state["project"] = project

    return {
        "progress": project["progress"],
        "completed": f"{project['completed_topics']}/{project['total_topics']}"
    }
```

**Usage**:
```
Session 1:
User: "Research 5 AI topics: LLMs, Agents, RAG, Fine-tuning, Evals"
Agent: [Calls start_research_project_tool with total_topics=5]

Agent: [Researches LLMs, calls complete_topic_tool]
Output: Progress 20% (1/5)

Session 2 (Next day):
Runner loads state, project progress is preserved
Agent: [Continues with next topic, calls complete_topic_tool]
Output: Progress 40% (2/5)

Sessions 3-5: Continue until 100%
```

## Comparison: State Management in OpenAI vs Google ADK

| Aspect | OpenAI SDK | Google ADK |
|--------|-----------|-----------|
| **Tool state access** | Via `context_variables` dict | Via `ToolContext.state` |
| **State modification** | Explicit return + merge | Direct state mutation |
| **Session persistence** | User-managed (custom code) | SessionService built-in (InMemory, Firestore, VertexAI) |
| **Multi-session support** | Manual session tracking | Runner handles session lifecycle |
| **Production backend** | Requires custom implementation | Firestore or VertexAI ready |
| **Learning curve** | Requires session design | SessionService pattern clear |

**Key difference**: OpenAI requires you to manually save/load state (using context_variables). Google ADK provides SessionService to handle persistence for you.

## Try With AI

**Setup**: You're building a research assistant that accumulates findings across multiple conversations with the same user.

### Prompt 1: State Access Pattern

Ask AI to explain how ToolContext works:

```
I'm building a tool that needs to access conversation history and user preferences
from previous sessions. Show me the pattern for:
1. Reading state from ToolContext
2. Modifying state
3. What happens to state after the tool returns

Use a concrete example: a research tool that checks if a query was already researched
and adds it to history if it's new.
```

**What you're learning**: Understanding the bidirectional state flow through ToolContext and how state persists within a session.

### Prompt 2: SessionService Selection

Ask AI to help choose the right backend:

```
I'm making a decision about session storage. My agent needs to:
- Run during development (local testing)
- Scale to production (100+ concurrent users)
- Persist user data for months
- Never lose conversation history

Compare InMemorySessionService, FirestoreSessionService, and VertexAiSessionService.
For each, explain: when to use it, what you must manage, how state survives restarts.
```

**What you're learning**: Trade-offs between different backends and how to match them to requirements.

### Prompt 3: Implementing a Real Pattern

Ask AI to help implement state tracking:

```
I want to build a research agent that:
1. Allows users to set a preference: "I prefer academic sources"
2. Remembers this preference across sessions
3. Uses it to filter search results
4. Tracks 'topics researched' to avoid duplicates

Show me:
- The tool that sets the preference
- The tool that searches with preference filtering
- The tool that checks for duplicate topics
- How these work together with ToolContext and SessionService
```

**What you're learning**: Real-world state patterns and how to compose multiple tools that share and manage state.


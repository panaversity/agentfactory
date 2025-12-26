---
sidebar_position: 1
title: "Your First ADK Agent"
description: "Install Google's Agent Development Kit, create your first Gemini-powered agent, and understand the declarative approach that makes ADK production-ready."
keywords: ["Google ADK", "Gemini agents", "Agent Development Kit", "agent setup", "Python agents", "declarative agents"]
chapter: 35
lesson: 1
duration_minutes: 50

# HIDDEN SKILLS METADATA
skills:
  - name: "ADK Installation and Project Setup"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Technical Problem-Solving"
    measurable_at_this_level: "Student can install google-adk and scaffold a project with adk create"

  - name: "Agent Class Instantiation with Gemini"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can create Agent with name, instructions, and model configuration"

  - name: "Google Search Tool Integration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can add google_search to agent tools and invoke via agent execution"

  - name: "ADK Development Commands (run, web)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Development Tools"
    measurable_at_this_level: "Student can use adk run for terminal interaction and adk web for debugging UI"

  - name: "Declarative vs Procedural Agent Design"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can contrast declarative (ADK YAML/config) with procedural (OpenAI SDK imperative)"

learning_objectives:
  - objective: "Install Google ADK and scaffold a new agent project"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful project creation with adk create"

  - objective: "Configure Gemini API keys and understand backend options (Google AI vs Vertex AI)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Agent authenticates and responds without errors"

  - objective: "Build a declarative agent with instructions and built-in tools"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Agent responds to prompts using google_search tool"

  - objective: "Use adk run for terminal interaction and adk web for visual debugging"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student navigates Events tab, sees tool calls and model reasoning"

  - objective: "Understand how declarative design differs from OpenAI's imperative SDK pattern"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student articulates key differences: config-driven vs code-driven, workflow agents vs handoffs"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (ADK installation, Agent class, declarative config, Gemini models, google_search tool, adk run, adk web, Vertex AI vs Google AI) within B1 limit of 10 concepts - PASS"

differentiation:
  extension_for_advanced: "Explore agent.yaml YAML configuration syntax; configure multi-agent systems; investigate Vertex AI backend setup for production"
  remedial_for_struggling: "Start with adk web UI first to visualize execution; defer google_search integration until basic agent pattern is comfortable; use gemini-2.5-flash model (free tier) instead of exploring model options"

generated_by: content-implementer
source_spec: Chapter 35 Lesson 1 specification
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Your First ADK Agent

Imagine you're building a news research system that autonomously gathers information about emerging technologies. You don't want to write imperative code orchestrating every API call and data flow. Instead, you declare what the agent should do—research, synthesize, and report—and let the framework handle the execution. This is where Google's Agent Development Kit shines.

You've built agents with OpenAI's procedural SDK in Chapter 34, where you construct agents imperatively through code. Google ADK takes a different philosophy: **declarative agent design**. You specify agent structure through configuration and type annotations, and ADK handles the orchestration. This approach scales better to complex multi-agent systems and integrates deeply with Google Cloud's production infrastructure.

This lesson walks you through installation, your first ADK agent powered by Gemini, and how the declarative approach differs from the SDK pattern you already know. By the end, you'll understand when to choose ADK's structured approach over OpenAI's flexibility.

## Installation and Project Setup

Before building agents, you need Google ADK installed and your development environment configured.

### Step 1: Install Google ADK

Open your terminal and install the google-adk package:

```bash
pip install google-adk
```

This installs the core ADK framework with dependencies. Verify the installation worked:

```bash
pip show google-adk
```

**Output:**
```
Name: google-adk
Version: X.Y.Z  # Check PyPI for current version
Location: /path/to/your/python/site-packages
```

If installation succeeds, you're ready to scaffold a project.

### Step 2: Create Your First Agent Project

ADK provides a scaffolding tool to initialize agent projects with best practices. Run:

```bash
adk create --type=code my_news_agent
```

This creates a project directory with the following structure:

```
my_news_agent/
├── agent.py          # Main agent code
├── agent.yaml        # Agent configuration (optional, for declarative definition)
├── .env              # Environment variables (API keys, project IDs)
├── __init__.py       # Package marker
└── requirements.txt  # Python dependencies
```

Navigate into your project:

```bash
cd my_news_agent
```

### Step 3: Configure Your API Key

ADK supports two authentication backends:

**Option A: Google AI (Free Tier)**

Get a free API key from https://ai.google.dev/. Then create a `.env` file in your project:

```
GOOGLE_API_KEY=your-api-key-here
```

**Option B: Vertex AI (Google Cloud)**

If you have a Google Cloud project with Vertex AI enabled, set:

```
GOOGLE_GENAI_USE_VERTEXAI=true
GOOGLE_CLOUD_PROJECT=your-project-id
```

For this lesson, Option A (Google AI with free key) is simplest. ADK automatically loads `.env` when you run agents.

### Step 4: Verify Configuration

Create a verification script `verify_setup.py`:

```python
import os
from google.genai import Client

api_key = os.getenv('GOOGLE_API_KEY')
if not api_key:
    print("ERROR: GOOGLE_API_KEY not set in .env")
else:
    print(f"API Key found: {api_key[:20]}...")
    # Optional: Test client initialization
    client = Client(api_key=api_key)
    print("Google AI client initialized successfully")
```

**Output:**
```
API Key found: AIzaSyD...
Google AI client initialized successfully
```

## Your First Agent

Now let's build a working agent. This introduces ADK's declarative approach.

### The Declarative Pattern

Unlike OpenAI's imperative SDK (define agent → run → extract result), ADK uses a **declarative model**:

1. **Define the Agent** — Specify agent properties (name, instructions, tools) through code or YAML
2. **Add Tools** — Equip agent with capabilities (google_search, custom functions)
3. **Run with CLI** — Use `adk run` or `adk web` to interact

The key difference: ADK separates agent **definition** from **execution**, making configuration reusable across environments.

### Hello World Agent

Create a file `agent.py`:

```python
from google.adk.agents import Agent
from google.adk.tools import google_search

# Define the agent declaratively
root_agent = Agent(
    name="ai_news_agent",
    model="gemini-2.5-flash",
    instruction="You are an AI News Assistant. Use Google Search to find recent AI news and summarize findings clearly.",
    tools=[google_search]
)
```

That's it. No runner object needed. No manual orchestration. ADK handles execution.

### Running Your Agent

Now use ADK's CLI tools to interact with your agent.

**Terminal Interaction:**

```bash
adk run agent.py
```

This starts an interactive terminal session. Type your queries:

```
You: What are the latest developments in AI agents?

Agent: I'll search for recent AI agent developments for you.
[Searching for "latest AI agent developments 2025"...]

Based on recent news, here are key developments:
1. Multi-agent systems becoming mainstream with frameworks like ADK
2. Voice agents gaining adoption with Gemini's Live API
3. Enterprise deployment through Vertex AI Agent Engine
...
```

**Web Interface (Recommended for Learning):**

```bash
adk web
```

This opens a web UI at `http://localhost:8000` with a chat interface AND an Events tab showing agent reasoning.

The Events tab is crucial for understanding how your agent works:
- **Tool calls**: See which tools the agent used and when
- **Model reasoning**: Watch the agent's thinking process
- **Parameters**: Inspect what arguments were passed to each tool
- **Results**: See what each tool returned

This transparency is one of ADK's core strengths for production agents.

## Understanding the Declarative Approach

Let's break down why ADK's design matters.

### Agent Definition (Code)

```python
root_agent = Agent(
    name="ai_news_agent",                     # Human-readable identifier
    model="gemini-2.5-flash",                 # Gemini model to use
    instruction="You are an AI News Assistant...",  # Agent behavior (singular!)
    tools=[google_search]                     # Agent capabilities
)
```

**Compare to OpenAI SDK** (Chapter 34):

```python
# OpenAI: Imperative approach
agent = Agent(
    name="NewsResearcher",
    instruction="You are..."
    # Tools added separately through definitions or context
)

# Then you orchestrate:
result = Runner.run_sync(agent, "What's new in AI?")
print(result.final_output)
```

**ADK difference:**

- Agent configuration is self-contained (name, instructions, tools together)
- Execution is abstracted into CLI commands, not code
- This separates concerns: developers define agents; operators run them

### Tool Integration

Google ADK provides built-in tools:

- **google_search** — Web search without needing API keys
- **code_execution** — Python code running in sandboxed environment
- **file_access** — Read/write files safely

You simply add them to the `tools` list:

```python
from google.adk.tools import google_search
from google.adk.code_executors import BuiltInCodeExecutor

root_agent = Agent(
    name="research_assistant",
    model="gemini-2.5-flash",
    instruction="Research questions and validate answers with code.",
    tools=[google_search],
    code_executor=BuiltInCodeExecutor()  # Enables code execution
)
```

Custom tools (your own functions) come later in Lesson 2.

### Gemini Model Selection

ADK defaults to `gemini-2.5-flash`, a fast, efficient model suitable for most agent tasks. If you want different models:

```python
from google.adk.agents import Agent
from google.adk.tools import google_search

root_agent = Agent(
    name="ai_news_agent",
    model="gemini-2.0-flash-live-001",  # For voice/streaming support
    instruction="You are an AI News Assistant.",
    tools=[google_search]
)
```

**Key model choices:**

- `gemini-2.5-flash` — Fast, good for tooling (default)
- `gemini-2.0-flash-live-001` — Voice/video streaming support
- `gemini-1.5-pro` — Higher reasoning capability (slower)

## Comparing ADK to OpenAI SDK

By now you've seen both approaches. Here's how they differ:

| Aspect | OpenAI SDK | Google ADK |
|--------|-----------|-----------|
| **Design Pattern** | Imperative (code orchestrates execution) | Declarative (config specifies agent) |
| **Tool Definition** | Pydantic models, explicit function wrapping | Type annotations, auto-wrapped functions |
| **Execution** | `Runner.run_sync()` in code | `adk run` / `adk web` CLI |
| **State Management** | `context_variables` parameter passing | `ToolContext` for persistent state (Lesson 3) |
| **Multi-Agent** | Handoffs (flexible but manual) | Workflow agents (deterministic pipelines) |
| **Safety** | Input/output validation | 6 callback types for granular control |
| **Debugging** | Print statements or external logging | Events tab in `adk web` UI |
| **Deployment** | Self-hosted anywhere | Vertex AI Agent Engine (managed) |

**When to choose ADK:**

- Building production agents at scale (Vertex AI integration)
- Needing transparent debugging (Events tab)
- Creating deterministic workflows (Workflow agents)
- Working in Google Cloud ecosystem

**When to choose OpenAI SDK:**

- Building simple, one-off agents
- Needing maximum flexibility for custom orchestration
- Integrating with non-Google services exclusively
- Team familiar with Python imperative patterns

## Common Setup Issues

**Issue: "ModuleNotFoundError: No module named 'google.adk'"**

Your environment doesn't have ADK installed. Install it:

```bash
pip install google-adk
```

**Issue: "GOOGLE_API_KEY not found" or authentication fails**

Your `.env` file isn't set up or isn't in the right location. Verify:

```bash
cat .env  # Check if file exists and has GOOGLE_API_KEY=...
```

If missing, create `.env` in your project root (same directory as `agent.py`).

**Issue: "google_search tool not found"**

You're importing from the wrong module. Use:

```python
from google.adk.tools import google_search
```

NOT `from google.genai.tools import ...`

**Issue: adk web doesn't open or shows "Connection refused"**

The web server is running on `localhost:8000` but your browser can't reach it. Try:

```bash
# Check if server is running
curl http://localhost:8000

# If that fails, restart adk web
adk web --port=8001  # Use different port
```

**Issue: Agent takes 30+ seconds to respond**

This is expected. ADK agents:
1. Send your prompt to Google's servers
2. Call google_search tool (which makes HTTP requests)
3. Process search results
4. Generate response

Network latency compounds. This is normal for cloud-based agents.

## Try With AI

### Prompt 1: Domain-Specific News Agent

```
Create an ADK agent that specializes in researching one specific domain
(choose: climate technology, space exploration, cybersecurity, healthcare AI, or renewable energy).

Write the agent definition with:
- Clear instructions emphasizing the domain focus
- The google_search tool
- Specific constraints (e.g., "only report peer-reviewed sources" or "focus on recent breakthroughs")

Run the agent with: adk run

Ask it: "What's the most significant news in [your domain] this month?"

What does the agent prioritize in its search results? How do the instructions shape its focus?
```

**What you're learning:** How declarative agent specifications drive search behavior and information prioritization

### Prompt 2: Comparing ADK vs OpenAI Pattern

```
Think about an agent you might build: a customer feedback analyzer that researches
company reputation and competitive positioning.

Write the agent definition for ADK:
- Name and instructions
- Tools needed (google_search, maybe others)
- Model choice (explain why)

Then, imagine writing the same agent with OpenAI SDK from Chapter 34.
What would be different?

Specifically consider:
- How would you orchestrate the search (manual loop vs declarative definition)?
- How would you see the agent's reasoning (Events tab vs print statements)?
- How would you deploy it (Vertex AI vs self-hosted)?

Which would you choose for this use case? Why?
```

**What you're learning:** The architectural tradeoffs between declarative (ADK) and imperative (OpenAI) agent design

### Prompt 3: Debugging with adk web

```
Create a simple agent that researches AI news.

Run it with: adk web

Open http://localhost:8000 in your browser and click the Events tab.

Ask the agent: "What are the latest breakthroughs in AI agent architectures?"

In the Events tab, examine:
1. How many times did google_search get called?
2. What search queries did the agent construct?
3. What results did search return?
4. How did the agent synthesize results into its response?

Compare this visibility to what you'd see with OpenAI SDK (just final_output).
Why is this transparency important for production agents?
```

**What you're learning:** Production agent debugging through declarative tool transparency and reasoning visibility

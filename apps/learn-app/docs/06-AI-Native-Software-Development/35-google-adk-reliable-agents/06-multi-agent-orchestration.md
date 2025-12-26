---
sidebar_position: 6
title: "Multi-Agent Orchestration"
description: "Build production-grade multi-agent systems using AgentTool to compose specialist agents into coordinated workflows, with structured output schemas enforcing consistency."
keywords: ["multi-agent", "AgentTool", "agent composition", "Pydantic schemas", "orchestration", "specialist agents", "coordinator pattern", "structured output"]
chapter: 35
lesson: 6
duration_minutes: 90

# HIDDEN SKILLS METADATA
skills:
  - name: "AgentTool for Agent Composition"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can wrap agents as tools and invoke them from coordinator agents"

  - name: "Pydantic Schema Definition for Structured Output"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can design Pydantic models enforcing structured agent outputs"

  - name: "Multi-Agent Workflow Architecture"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can design coordinator-specialist patterns for complex multi-agent systems"

  - name: "Error Resilience in Agent Composition"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement graceful degradation when agent components fail"

  - name: "Output Schema Validation and Type Safety"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can validate agent outputs against schemas and handle schema violations"

learning_objectives:
  - objective: "Implement multi-agent systems where specialist agents encapsulate domain expertise"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Build coordinator agent that delegates to multiple specialist agents successfully"

  - objective: "Design Pydantic schemas that enforce structured outputs from agent orchestration"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create AINewsReport schema and validate agent output matches schema"

  - objective: "Architect coordinator-specialist patterns where coordinator stays in control"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Compare ADK AgentTool (coordinator control) vs OpenAI handoffs (control transfer)"

  - objective: "Implement error resilience so multi-agent systems continue when individual components fail"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Handle missing data gracefully using placeholder values, no workflow halting"

cognitive_load:
  new_concepts: 7
  assessment: "7 core concepts (AgentTool, Pydantic BaseModel, output_schema parameter, specialist agents, coordinator pattern, structured validation, error resilience) appropriate for B2 with 90-minute duration. Complexity increases through hands-on examples building complete multi-agent news system."

differentiation:
  extension_for_advanced: "Implement dynamic agent selection based on task type; add multi-level agent hierarchies (coordinator → sub-coordinators → specialists); design agent communication protocols for complex workflows"
  remedial_for_struggling: "Start with simple 2-agent system (coordinator + specialist); practice Pydantic model definition separately before integration; visualize agent call flow with diagrams before coding"

generated_by: content-implementer
source_spec: Chapter 35 Lesson 6 specification
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Multi-Agent Orchestration

In earlier lessons, you built individual agents with specialized tools and instructions. But the real power emerges when you compose multiple agents into coordinated systems where each agent owns a specific domain.

Imagine this workflow:

1. A coordinator agent receives a request: "Research AI news and generate an audio podcast"
2. The coordinator searches for recent news, enriches it with financial context
3. The coordinator recognizes audio generation is a specialty—not its job
4. The coordinator delegates to a podcaster agent with structured data
5. The podcaster agent generates audio and returns a confirmation
6. The coordinator assembles everything and returns the result

This is **multi-agent orchestration**—dividing work across specialized agents while keeping the coordinator in control of the overall workflow.

Google ADK provides **AgentTool**, which lets you wrap agents as tools. Unlike OpenAI's handoffs (which transfer control entirely), AgentTool keeps the coordinator in charge. This pattern scales elegantly to complex multi-agent systems.

This lesson teaches you to architect these systems, use Pydantic schemas to enforce consistent structured output, and handle failures gracefully when components break.

## The Coordinator-Specialist Pattern

Before diving into code, let's understand the architectural pattern that makes multi-agent systems reliable.

### Specialist Agents: Focused Expertise

A specialist agent owns one domain:

- **Podcaster Agent**: Takes text and generates audio (one job)
- **Data Enricher Agent**: Takes news and adds financial context (one job)
- **Summarizer Agent**: Condenses long text into bullet points (one job)

Each specialist:

- Has focused instructions (not "do everything")
- Owns specific tools (not a kitchen sink of capabilities)
- Returns structured data (not free-form text)
- Can fail independently without crashing the whole system

### Coordinator Agent: Orchestration

The coordinator agent:

- Receives the user request
- Decides which specialists to invoke and when
- Maintains control (doesn't hand off execution)
- Combines specialist outputs into final result
- Handles failures gracefully

**The critical difference from OpenAI SDK:**

| ADK | OpenAI SDK |
|-----|-----------|
| Coordinator uses `AgentTool(agent=specialist)` to call specialist | Coordinator executes `transfer_to(agent)`, handing off control |
| Coordinator stays in charge and receives specialist output | Control transfers entirely—coordinator has no say in specialist execution |
| Coordinator can invoke multiple specialists and decide next steps | Handoff is one-way—hard to compose 3+ agents |

## Building Your First Multi-Agent System

Let's build the AI News Podcast system end-to-end. You'll create a podcaster specialist, then a coordinator agent that delegates to it.

### Step 1: Define the Podcaster Specialist

```python
from google.adk.agents import Agent
from google.adk.tools import code_execution

# Specialist agent: convert text to podcast audio
podcaster_agent = Agent(
    name="podcaster_agent",
    model="gemini-2.0-flash",
    instruction="""
    You are an Audio Generation Specialist.

    Your job: Take a text script and convert it to podcast audio.

    Rules:
    - Use clear, engaging voice instructions
    - Break long text into sections
    - Add natural pauses for readability
    - Return ONLY the audio generation command, not explanations

    You have access to code_execution tool to generate audio files.
    """,
    tools=[code_execution]
)
```

The podcaster agent:
- Has **one job**: Generate audio from text
- **Clear instructions** that prevent it from doing other tasks
- **Specific tools** (code_execution for file generation)

### Step 2: Define Pydantic Schemas for Structured Output

Before building the coordinator, define what structured outputs look like. This enforces consistency across the system.

```python
from pydantic import BaseModel, Field
from typing import List

class NewsStory(BaseModel):
    """A single news story with financial context."""
    company: str = Field(description="Company name mentioned in news")
    ticker: str = Field(description="Stock ticker symbol or 'N/A' if not found")
    summary: str = Field(description="One-sentence summary of the news")
    financial_context: str = Field(description="Current stock price and percentage change, or 'N/A'")

class AINewsReport(BaseModel):
    """Structured report of AI news - output of coordinator agent."""
    title: str = Field(default="AI Research Report")
    report_summary: str = Field(description="High-level findings from research")
    stories: List[NewsStory] = Field(description="List of news stories with context")
    podcast_generated: bool = Field(default=False, description="Whether podcast audio was created")
```

**Why Pydantic schemas matter:**

Without schema:
```python
agent_output = "Some news about OpenAI... They're doing AI research..."
# Is this valid? Is it complete? Does it match what coordinator expects?
# Unknown. Coordinator has to parse it as free-form text.
```

With schema:
```python
report = AINewsReport(
    title="AI Research Report",
    report_summary="OpenAI released new research...",
    stories=[
        NewsStory(
            company="OpenAI",
            ticker="PRIVATE",
            summary="Released research on reasoning models",
            financial_context="N/A"
        )
    ],
    podcast_generated=True
)
# Type-safe. Validated. Coordinator knows exactly what it has.
```

### Step 3: Build the Coordinator Agent

Now create the coordinator that uses AgentTool to delegate to the podcaster:

```python
from google.adk.agents import Agent
from google.adk.tools.agent_tool import AgentTool, google_search
from google.genai import Client

# Wrap the podcaster agent as a tool
podcaster_tool = agent_tool.AgentTool(agent=podcaster_agent)

# Coordinator agent
coordinator_agent = Agent(
    name="ai_news_researcher",
    model="gemini-2.5-flash",
    instruction="""
    You are an AI News Researcher and Podcast Producer.

    Your job:
    1. Search for recent AI news
    2. Enrich stories with financial context
    3. Delegate to podcaster_agent to generate audio
    4. Return structured report with news stories

    Critical rules:
    - If financial data is missing, use "N/A" (don't halt workflow)
    - Always delegate to podcaster_agent for audio generation
    - Structure output as JSON matching AINewsReport schema
    """,
    tools=[google_search, podcaster_tool],
    output_schema=AINewsReport  # Enforce structured output
)
```

**Key details:**

1. **`agent_tool.AgentTool()`**: Wraps the podcaster agent as a callable tool
2. **Added to `tools` list**: Coordinator can invoke it like any other tool
3. **`output_schema=AINewsReport`**: ADK validates coordinator output against the schema

### Step 4: Run the Multi-Agent System

```python
from google.adk import Runner

async def run_news_system():
    """Execute the multi-agent news podcast system."""

    # User request
    user_request = "Research recent AI breakthroughs and create a podcast script"

    print("Starting multi-agent orchestration...")
    print(f"Request: {user_request}\n")

    # Run coordinator (which will invoke specialist as needed)
    result = await Runner.run_async(
        agent=coordinator_agent,
        user_input=user_request
    )

    # Result is guaranteed to match AINewsReport schema
    report = result.final_output

    # Type-safe access to structured data
    print(f"Report Title: {report.title}")
    print(f"Summary: {report.report_summary}\n")

    print(f"Stories ({len(report.stories)}):")
    for story in report.stories:
        print(f"  - {story.company} ({story.ticker})")
        print(f"    {story.summary}")
        print(f"    Financial: {story.financial_context}\n")

    print(f"Podcast Generated: {report.podcast_generated}")

# Execute
await run_news_system()
```

**Output:**
```
Starting multi-agent orchestration...
Request: Research recent AI breakthroughs and create a podcast script

Report Title: AI Research Report
Summary: Recent breakthroughs in reasoning models and multimodal AI systems

Stories (2):
  - OpenAI (PRIVATE)
    Released advanced reasoning models showing improved performance on complex tasks
    Financial: N/A

  - Google (GOOGL)
    Announced Gemini 2.0 with native multimodal capabilities
    Financial: $176.50 (+2.3%)

Podcast Generated: True
```

Notice the structure: coordinator searched, enriched data, delegated audio generation, and returned type-safe output—all without manual orchestration code.

## Handling Failures Gracefully

Real systems fail. The news search might return no results. The financial API might be down. A specialist agent might crash. Production multi-agent systems must continue working when components break.

### Pattern: Graceful Degradation with Placeholders

```python
from typing import Optional

class NewsStory(BaseModel):
    """News story with optional financial context."""
    company: str
    ticker: str = Field(default="N/A", description="Use N/A if ticker not found")
    summary: str
    financial_context: str = Field(default="N/A", description="Use N/A if API fails")

class AINewsReport(BaseModel):
    """Report that continues even if specialist agents fail."""
    stories: List[NewsStory]
    podcast_generated: bool = Field(default=False)
    error_summary: Optional[str] = Field(default=None)
```

### Resilient Coordinator Pattern

```python
from google.adk.agents import Agent

resilient_coordinator = Agent(
    name="resilient_news_researcher",
    model="gemini-2.5-flash",
    instruction="""
    You are a resilient news research coordinator.

    Instructions for handling failures:

    1. If google_search returns no results:
       - Report what you found (even if empty)
       - Don't halt the workflow

    2. If financial data is unavailable:
       - Use "N/A" for ticker and financial_context
       - Continue with the story anyway

    3. If podcaster_agent fails:
       - Set podcast_generated = false
       - Return the stories you found
       - Note the error in error_summary

    Critical Rule: A failure in one component NEVER stops the entire workflow.
    Always return best-effort results.

    Return structured output matching AINewsReport schema.
    """,
    tools=[google_search, podcaster_tool],
    output_schema=AINewsReport
)
```

This design ensures:
- Missing ticker data doesn't crash the system (use "N/A")
- Financial API down doesn't halt research (continue with empty context)
- Podcast generation failure doesn't lose news stories (report what we found)

## Comparing ADK AgentTool to OpenAI Handoffs

This is the crucial architectural distinction:

### OpenAI SDK: Handoff Pattern (Control Transfer)

```python
# Chapter 34 pattern - handoff transfers control
root_agent = Agent(
    name="coordinator",
    instruction="Delegate to podcaster when needed",
    tools=[
        transfer_to(podcaster_agent)  # Control leaves coordinator
    ]
)

# Flow: coordinator → transfer to podcaster → podcaster owns execution
# Coordinator can't see what podcaster is doing
# Coordinator can't intervene if podcaster makes wrong choice
```

**Consequences:**
- Coordinator loses visibility into specialist execution
- Hard to compose 3+ agents (handoff chains are fragile)
- Difficult to implement fallbacks or error handling
- Each handoff is a control transfer, not a tool invocation

### Google ADK: AgentTool Pattern (Coordinator Control)

```python
# ADK pattern - agent remains coordinator
coordinator_agent = Agent(
    name="ai_news_researcher",
    instruction="Search, enrich, delegate to podcaster, return results",
    tools=[
        google_search,
        agent_tool.AgentTool(agent=podcaster_agent)  # Control stays with coordinator
    ]
)

# Flow: coordinator calls podcaster_tool → gets result → decides next step
# Coordinator sees the podcast result and can decide what to do with it
# Easy to handle failures: if podcaster fails, coordinator continues
```

**Advantages:**
- Coordinator stays in control of the workflow
- Easy to see what specialists returned
- Simple to implement error handling
- Composing many agents is straightforward

**Decision Table:**

| Scenario | ADK AgentTool | OpenAI Handoff |
|----------|---|---|
| **2-agent system** | Excellent | Fine |
| **3+ agent system** | Excellent | Fragile |
| **Error handling** | Natural (agent returns error) | Difficult (control already transferred) |
| **Coordinator visibility** | Full (sees all agent outputs) | Limited (after handoff happens) |
| **Complex workflows** | Scales well | Manual orchestration needed |

## Complete Multi-Agent Example: AI News Podcast System

Here's the complete, working system combining everything:

```python
from google.adk.agents import Agent, Runner
from google.adk.tools import google_search, code_execution, agent_tool
from pydantic import BaseModel, Field
from typing import List
import json

# ============ PYDANTIC SCHEMAS ============
class NewsStory(BaseModel):
    """Single news story with context."""
    company: str = Field(description="Company name")
    ticker: str = Field(default="N/A", description="Stock ticker or N/A")
    summary: str = Field(description="One-sentence summary")
    financial_context: str = Field(default="N/A", description="Stock price and change")

class AINewsReport(BaseModel):
    """Final structured report."""
    title: str = Field(default="AI Research Report")
    report_summary: str
    stories: List[NewsStory]
    podcast_generated: bool = Field(default=False)

# ============ SPECIALIST AGENT ============
podcaster_agent = Agent(
    name="podcaster_agent",
    model="gemini-2.0-flash",
    instruction="""
    You are an Audio Generation Specialist.
    Your ONLY job: Convert text scripts to podcast format.

    When given a script:
    1. Break it into natural sections
    2. Add audio cues for emphasis
    3. Return podcast-ready markdown

    Don't do anything else. If asked non-podcast questions, refuse politely.
    """,
    tools=[code_execution]
)

# ============ COORDINATOR AGENT ============
# Wrap specialist as tool
podcaster_tool = agent_tool.AgentTool(agent=podcaster_agent)

coordinator_agent = Agent(
    name="ai_news_researcher",
    model="gemini-2.5-flash",
    instruction="""
    You research AI news and produce podcast content.

    Workflow:
    1. Search for recent AI news using google_search (3-5 stories)
    2. For each story, identify the company and get ticker symbol if available
    3. If financial data is missing, use "N/A" and continue (don't stop)
    4. Structure findings into NewsStory objects
    5. Delegate to podcaster_agent to generate audio format

    Error handling: If any step fails, use placeholder data ("N/A") and continue.
    Never halt the entire workflow for one missing piece of data.

    Return structured output EXACTLY matching AINewsReport schema.
    """,
    tools=[google_search, podcaster_tool],
    output_schema=AINewsReport
)

# ============ EXECUTION ============
async def main():
    """Run the multi-agent news podcast system."""

    request = "Research AI breakthroughs this month and generate podcast"

    print("=== Multi-Agent News Podcast System ===\n")
    print(f"Request: {request}\n")

    result = await Runner.run_async(
        agent=coordinator_agent,
        user_input=request
    )

    # Result is guaranteed to match AINewsReport schema
    report: AINewsReport = result.final_output

    print(f"📰 {report.title}")
    print(f"Summary: {report.report_summary}\n")

    print(f"Stories Found: {len(report.stories)}")
    for i, story in enumerate(report.stories, 1):
        print(f"\n{i}. {story.company} ({story.ticker})")
        print(f"   {story.summary}")
        print(f"   Financial: {story.financial_context}")

    print(f"\n🎙️ Podcast Generated: {report.podcast_generated}")

# Run the system
if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
```

## Try With AI

You've built agents and used tools before. Now you're composing agents into systems. Use your existing ADK setup from previous lessons.

### Prompt 1: Design a Multi-Agent Research System

```
I want to build a research system with specialist agents for different domains.

Design (don't code yet, just describe):
- What specialist agents would you create?
- What does each specialist do?
- What data does the coordinator need to pass to specialists?
- How would you structure the output from each specialist?

Example: Building a "Startup Due Diligence" system that researches:
1. Company financial health
2. Market opportunity
3. Competitive landscape
4. Team credibility

For each area, define:
- A specialist agent (what's their focus?)
- Input data the coordinator provides
- Output schema they return
- How the coordinator combines these into a final report

This will help you think about agent responsibilities before coding.
```

**What you're learning**: Architectural thinking about multi-agent system design—dividing work based on specialization and defining interfaces (schemas) between components

### Prompt 2: Implement Multi-Agent System with AgentTool

```
Build a multi-agent system using AgentTool:

1. Create a "Data Summarizer" specialist agent that:
   - Takes long text passages
   - Extracts 3-5 key points
   - Returns structured list of points

2. Create a "Summary Validator" specialist agent that:
   - Takes a list of key points
   - Checks for redundancy
   - Returns deduplicated points

3. Create a coordinator agent that:
   - Receives user input (long article text)
   - Delegates to Data Summarizer to extract points
   - Delegates to Validator to deduplicate
   - Returns final structured list

Define Pydantic schemas for:
- KeyPoint (text, confidence)
- SummaryOutput (key_points: List[KeyPoint])

Run the system with a sample article about AI agents.
Observe how the coordinator stays in control throughout.
```

**What you're learning**: Practical multi-agent composition where coordinator orchestrates specialist outputs and maintains control of workflow—the core ADK advantage over OpenAI handoffs

### Prompt 3: Add Error Resilience to Multi-Agent System

```
Starting with your coordinator + 2 specialists system from Prompt 2:

Add error handling so the system continues when components fail.

Implement these failure scenarios:
1. Summarizer returns empty list (no points extracted)
   → Coordinator should continue with empty list, not halt
2. Validator fails to connect
   → Coordinator should skip validation and return raw summary
3. Invalid JSON from specialist
   → Coordinator should use placeholder data "No data available"

For each scenario:
- Show the coordinator instructions handling this gracefully
- Use placeholder values so workflow continues
- Return results with error_count field indicating what failed

Then test: Does your system continue when a specialist fails?
Or does it halt the entire workflow?

(Production multi-agent systems must continue—this is non-negotiable.)
```

**What you're learning**: Resilience patterns that distinguish production multi-agent systems from fragile prototypes—graceful degradation with placeholder data ensures the whole workflow doesn't break when individual components fail

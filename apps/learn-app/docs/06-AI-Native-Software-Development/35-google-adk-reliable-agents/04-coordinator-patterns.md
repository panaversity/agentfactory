---
sidebar_position: 4
title: "Coordinator Patterns"
description: "Master background agent processing with the coordinator-dispatcher pattern. Learn how to offload complex work to run silently while keeping user interaction responsive and structured."
keywords: ["coordinator pattern", "dispatcher pattern", "background processing", "agent coordination", "file persistence", "asynchronous agents", "callback patterns"]
chapter: 35
lesson: 4
duration_minutes: 55

# HIDDEN SKILLS METADATA
skills:
  - name: "Coordinator-Dispatcher Architecture"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can identify when to use coordinator pattern vs direct agent interaction; can articulate the two-message flow (acknowledgment + completion)"

  - name: "File Persistence Tool Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can write a file persistence tool with error handling and status reporting; can use pathlib for cross-platform paths"

  - name: "Silent Background Processing in Agents"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can design agent instructions that process work silently between acknowledgment and final completion message"

  - name: "Structured Report Generation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Technical Problem-Solving"
    measurable_at_this_level: "Student can create markdown-based report templates with consistent formatting and information hierarchy"

learning_objectives:
  - objective: "Understand when coordinator-dispatcher pattern improves user experience over direct tool execution"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student can articulate the problem (raw data overwhelming users) and solution (structured background processing)"

  - objective: "Design and implement file persistence tools with error handling"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student writes a tool that saves content to files with proper error handling and returns status"

  - objective: "Write coordinator agent instructions that manage two-message interaction pattern"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Agent produces acknowledgment message, processes silently, then completion message with file location"

  - objective: "Create structured markdown report templates for agent output"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student generates reports with consistent structure (headlines, metadata, summaries)"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (coordinator-dispatcher pattern, file persistence tool, pathlib, markdown reporting, two-message interaction, silent processing, error handling, status reporting) within B1 limit of 10 concepts - PASS"

differentiation:
  extension_for_advanced: "Implement coordinator with task queuing (Celery, RQ); add webhook callbacks for async completion notification; explore distributed file storage (S3, GCS); implement progress streaming within silent phase"
  remedial_for_struggling: "Start with simple file writing before error handling; practice pathlib.Path operations separately; use simpler YAML/JSON reports before markdown; practice two-message flow pattern with print statements before integrating with agent"

generated_by: content-implementer
source_spec: Chapter 35 Lesson 4 specification (DeepLearning.AI L4)
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Coordinator Patterns

Imagine your AI agent researches breaking news across dozens of sources, compiles market data, analyzes trends, and synthesizes findings into a structured report. If you ask the agent to do all this in real-time and stream everything back to you immediately, you'll be drowning in verbose output while waiting for the final analysis.

The **coordinator-dispatcher pattern** solves this by separating user-facing communication from background work. The agent acknowledges your request immediately, processes everything silently in the background, saves structured results to persistent storage, and finally confirms completion with the output location.

This pattern is essential for production agents because it:
1. **Improves UX**: Users see immediate feedback and a final summary, not a firehose of intermediate steps
2. **Enables persistence**: Work persists across sessions (files, databases, APIs)
3. **Scales processing**: Complex multi-step work runs without blocking user interaction
4. **Provides evidence**: Structured output files serve as audit trails and re-usable artifacts

In this lesson, you'll implement a coordinator agent that researches AI news, enriches it with financial data, and saves results to a markdown report—all while keeping the user experience clean and focused.

## The Coordinator-Dispatcher Problem

### Raw Output Overwhelms Users

When agents execute research directly and stream results back, users see everything:

```
Agent: Searching for AI news...
[Tool: google_search | "latest AI news 2025"]
Result: 47 headlines found...
Result: Processing headline 1 of 47...
Result: Extracting company info...
[Tool: get_financial_context | "OpenAI stock price"]
Result: OPENAI (NVDA): $890.25 (+2.3%)...
Result: Analyzing market impact...
Result: Writing report...
```

This is technically transparent, but pedagogically terrible for users. They don't care about the plumbing—they want the final research report.

### Solution: Two-Message Architecture

The coordinator pattern uses exactly two user-facing messages:

1. **Acknowledgment** (immediate): "I'll research that for you. This might take a moment."
2. **Completion** (after silent processing): "Done! Results saved to ai_research_report.md"

Everything between acknowledgment and completion happens silently, invisible to the user.

```
User: "Research the latest AI news and save a report"
     ↓
[Acknowledgment Message]
Agent: "Okay, I'll research that. This might take a moment."
     ↓
[Silent Background Processing — All work happens here]
Agent internally:
  - Searches for news (invisible)
  - Enriches with financial data (invisible)
  - Formats structured report (invisible)
  - Saves to file (invisible)
     ↓
[Completion Message]
Agent: "All done. Report saved to ai_research_report.md"
```

This keeps the user interaction clean while the agent does heavy lifting silently.

## File Persistence Tool

The coordinator pattern requires a tool that saves structured output to files. Let's build one.

### Design Specification

Before writing code, specify what this tool should do:

**Intent**: Save markdown-formatted content to files in the current working directory

**Success Criteria**:
- Accepts filename and markdown content
- Validates filename (adds .md extension if missing)
- Writes to current working directory using pathlib for cross-platform compatibility
- Returns status dict with success/error message
- Handles exceptions gracefully (full path in output, error messages clear)

**Constraints**:
- Always writes to current directory (no path traversal)
- Overwrites existing files silently
- Uses UTF-8 encoding

### Implementation

Here's the file persistence tool:

```python
import pathlib
from typing import Dict

def save_news_to_markdown(filename: str, content: str) -> Dict[str, str]:
    """
    Saves markdown content to a file in the current directory.

    This tool is designed for coordinator agents that process work silently
    and save structured results. It enables persistent storage of agent output.

    Args:
        filename: Name of file to save (e.g., 'ai_news.md').
                  If filename doesn't end with .md, extension is added automatically.
        content: Markdown-formatted string to write to file

    Returns:
        Dictionary with operation status:
        {
            "status": "success" | "error",
            "message": Human-readable status message,
            "filepath": Absolute path to saved file (on success only)
        }

    Example:
        >>> result = save_news_to_markdown(
        ...     "research_report",
        ...     "# AI News\n\n- Breaking: New model released"
        ... )
        >>> print(result["message"])
        Successfully saved to /Users/you/project/research_report.md
    """
    try:
        # Validate and normalize filename
        if not filename:
            return {
                "status": "error",
                "message": "Filename cannot be empty"
            }

        # Add .md extension if missing
        if not filename.endswith(".md"):
            filename += ".md"

        # Get current directory using pathlib (cross-platform safe)
        current_directory = pathlib.Path.cwd()
        file_path = current_directory / filename

        # Write content to file with UTF-8 encoding
        file_path.write_text(content, encoding="utf-8")

        # Return success with absolute path
        return {
            "status": "success",
            "message": f"Successfully saved to {file_path.resolve()}"
        }

    except Exception as e:
        # Catch any IO errors and return error status
        return {
            "status": "error",
            "message": f"Failed to save file: {str(e)}"
        }
```

### What This Tool Does

**Specification → Implementation**:
- "Validates filename" → Checks for empty string, adds .md extension
- "Uses pathlib" → `pathlib.Path.cwd()` and `/` operator (cross-platform safe)
- "Returns status dict" → Always returns `{"status": "...", "message": "..."}`
- "Handles exceptions" → try/except catches IO errors

**Key design decisions**:
1. **Always returns dict, never raises exceptions** → Agent can check status and adapt
2. **Absolute path in message** → User knows exactly where file was saved
3. **Auto-adds .md extension** → Reduces user burden (they can pass "report" or "report.md")
4. **UTF-8 encoding** → Handles special characters in markdown consistently

### Using This Tool in ADK

Add the tool to your agent:

```python
from google.adk.agents import Agent
from google.adk.tools import google_search, get_financial_context

# Import your custom tool
from your_module import save_news_to_markdown

agent = Agent(
    name="ai_news_coordinator",
    model="gemini-2.5-flash",
    instruction="""...""",
    tools=[google_search, get_financial_context, save_news_to_markdown]
)
```

The agent will now have access to the persistence tool.

## Coordinator Agent Instructions

Now let's write the coordinator agent that uses the two-message pattern plus silent processing.

### Specification

**Intent**: Research AI news and financial data, then save structured report

**Required Behavior**:
1. First message: Acknowledge request
2. Silent phase: Execute all research tools (invisible to user)
3. Final message: Confirm completion with file location

**Key constraint**: No streaming or intermediate output between acknowledgment and completion

### Implementation

Here's the coordinator agent with proper instructions:

```python
from google.adk.agents import Agent
from google.adk.tools import google_search, get_financial_context
from your_module import save_news_to_markdown

root_agent = Agent(
    name="ai_news_research_coordinator",
    model="gemini-2.5-flash",
    instruction="""
You are a Background AI Research Coordinator. Your job is to research and save reports silently.

## Your Identity and Constraints

You operate in the background, not as a conversational assistant. You accept one research request,
process everything silently, and save results to a file. The user should see exactly two messages
from you: acknowledgment and completion.

## Required Two-Message Interaction

IMPORTANT: Follow this pattern strictly.

1. **First Message (Acknowledgment)**: Immediately acknowledge the request
   "Okay, I'll research [topic]. This might take a moment."

2. **Silent Processing Phase**: Execute all your work here (invisible to user)
   - Use google_search to find recent news
   - Use get_financial_context to gather market data
   - Format results into structured markdown
   - Call save_news_to_markdown to persist output
   - Do NOT send any intermediate messages or updates

3. **Final Message (Completion)**: Confirm the work is done
   "All done. Report saved to [filepath]"

## Work Guidelines

### Research Phase (Silent)
- Search for: "AI industry news 2025", "AI startup funding", "AI model releases"
- Gather context on the top 3-5 stories by relevance
- For each story, note: Company, Date, Impact

### Formatting Phase (Silent)
Use this markdown structure:

    # AI Industry News Report

    ## Top Headlines

    ### 1. [Headline]
    - **Company:** [Name] ([Ticker])
    - **Date:** [YYYY-MM-DD]
    - **Summary:** [2-3 sentences on impact and significance]

    ### 2. [Headline]
    - **Company:** [Name]
    - **Date:** [YYYY-MM-DD]
    - **Summary:** [Summary]

    ## Market Context
    - Key sector trends
    - Investment activity
    - Regulatory news

### Persistence Phase (Silent)
- Call save_news_to_markdown("ai_research_report", formatted_content)
- Check the returned status
- If status=="success", use the filepath in completion message
- If status=="error", report the error in completion message

## Safety Notes
- Never print intermediate steps
- Never stream partial research results
- Never ask for clarification once research begins
- Always structure output as markdown (never raw text or JSON)
- Always persist results to file before completing
    """,
    tools=[google_search, get_financial_context, save_news_to_markdown]
)
```

### Why This Instruction Design Works

**Two-message constraint**: The instruction explicitly says "two messages: acknowledgment and completion." This teaches the agent the pattern through explicit direction.

**Silent processing emphasis**: Phrases like "invisible to user," "Do NOT send any intermediate messages," and repeated "Silent" headings reinforce that work happens without streaming.

**Structured format**: The markdown template removes ambiguity about output format. The agent doesn't have to decide what to write—it follows the template.

**Tool-aware workflow**: The instruction references specific tools (google_search, get_financial_context, save_news_to_markdown) in the order they should be used, guiding the agent's reasoning.

## Structured Report Schema

The coordinator saves output using a markdown report template. Let's understand the structure:

### The Template

```markdown
# AI Industry News Report

Generated: 2025-01-15 | Focus: Recent developments

## Executive Summary

Brief overview of key themes from this research period.

## Top Headlines

### 1. [Headline]
- **Company:** [Name] ([Stock Ticker])
- **Date:** [YYYY-MM-DD]
- **Category:** [Model Release | Funding | Partnership | Regulation]
- **Summary:** [2-3 sentences on what happened and why it matters]
- **Market Impact:** [Stock movement, funding valuation, industry implications]

### 2. [Next Headline]
- **Company:** [Name]
- **Date:** [YYYY-MM-DD]
- **Category:** [Category]
- **Summary:** [Summary]
- **Market Impact:** [Impact]

## Market Context

### Funding Activity
- [Fund round]: $X million for [company] ([brief note])

### Regulatory News
- [Event]: [Brief description of regulatory action]

### Technology Trends
- [Trend]: [What's emerging, adoption rate]

## Key Metrics

| Metric | This Period | Previous | Trend |
|--------|------------|----------|-------|
| News stories | 12 | 8 | ↑ 50% |
| Funding announcements | 5 | 3 | ↑ 67% |
| Model releases | 2 | 1 | ↑ 100% |

## Next Steps

Based on these developments, watch for:
- [Upcoming expected development]
- [Regulatory event to monitor]
```

### Design Rationale

**Why markdown?**
- Human readable (not JSON)
- Versioning friendly (diffs work well)
- GitHub-renderable (can be committed to repos)
- No special tools needed to view (just a text editor)

**Why this specific structure?**
- **Headlines first** → User's most important information is at the top
- **Metadata per story** → Date, company, ticker enable follow-up research
- **Market Impact section** → Connects news to practical consequences
- **Metrics table** → Shows comparative trends (up/down from previous)

**Why these fields?**
- **Company + Ticker** → User can quickly look up stock performance
- **Category** → Helps user prioritize (funding vs regulation affect decisions differently)
- **Date** → Essential for news (recency affects relevance)
- **Summary** → Explains significance, not just facts

## Putting It Together: Complete Coordinator

Here's a complete, working coordinator that processes research silently:

```python
from google.adk.agents import Agent
from google.adk.tools import google_search, get_financial_context
import pathlib
from typing import Dict

# File persistence tool
def save_news_to_markdown(filename: str, content: str) -> Dict[str, str]:
    try:
        if not filename.endswith(".md"):
            filename += ".md"
        file_path = pathlib.Path.cwd() / filename
        file_path.write_text(content, encoding="utf-8")
        return {
            "status": "success",
            "message": f"Successfully saved to {file_path.resolve()}"
        }
    except Exception as e:
        return {"status": "error", "message": f"Failed: {str(e)}"}

# Coordinator agent
coordinator = Agent(
    name="ai_news_research_coordinator",
    model="gemini-2.5-flash",
    instruction="""
You are a Background AI Research Coordinator that processes research requests silently.

## Two-Message Pattern

1. **Acknowledgment**: "Okay, I'll research the latest AI news and trends. This might take a moment."
2. **Silent Processing**: Use google_search and get_financial_context to research. Format markdown. Save file.
3. **Completion**: "All done. Report saved to ai_research_report.md"

Do NOT send intermediate messages or ask for clarification.

## Work Instructions

Search for:
1. Latest AI model releases (this month)
2. Major funding announcements (last 30 days)
3. Enterprise AI adoption news

Structure as markdown with Headlines, Market Context, and Key Metrics.

Call save_news_to_markdown("ai_research_report", formatted_content) to persist.

Always follow the two-message pattern.
    """,
    tools=[google_search, get_financial_context, save_news_to_markdown]
)

# Use the coordinator:
# result = coordinator.run("Research the latest AI news and trends")
# The agent will acknowledge, process silently, save the report, and complete.
```

## Try With AI

Now you'll apply the coordinator pattern to a real research scenario. You'll experience how the two-message interaction keeps the user experience clean while the agent processes complex work silently.

### Setup

You'll research emerging AI capabilities from the last 90 days. The coordinator will gather news, enrich with financial context, structure the findings, and save a markdown report.

**What you're learning:**
- How coordinator-dispatcher separates user-facing communication from background work
- When to use silent processing vs streaming results (and why the choice matters)
- How structured output (markdown templates) removes ambiguity from agent work

### Prompt 1: Understand the Pattern

Ask AI: "I'm building a research coordinator agent that needs to research AI news, gather financial data, and save a report. The user should only see two messages: acknowledgment and completion. What are the key challenges in this design, and how do I structure the agent instructions to enforce the two-message pattern?"

**What you're learning:**
- AI articulates the pattern problem (intermediate messages disrupt UX)
- AI explains how instructions can guide the two-message behavior
- You internalize why structure matters in agent design

### Prompt 2: Design the File Persistence Tool

Ask AI: "Write a Python function that saves markdown content to a file with proper error handling. It should return a status dict so the agent can know if the save succeeded or failed. The function should handle edge cases like missing filenames or permission errors gracefully."

Compare AI's response to the implementation we provided above:
- Does AI include error handling?
- Does it return status information (not raise exceptions)?
- Does it use pathlib for cross-platform paths?
- Are the docstring and type hints clear enough for agent use?

**What you're learning:**
- How to design tools that agents can reason about (returning status vs raising exceptions)
- Why error handling is essential when agents make decisions based on tool output
- How documentation helps agents use tools correctly

### Prompt 3: Validate Your Understanding

Ask AI: "I have a coordinator agent that searches for news and saves reports. Between the acknowledgment and completion messages, it should search, enrich with financial data, format a markdown report, and persist to a file. Write the agent instructions that make this two-message pattern explicit and prevent the agent from sending intermediate messages."

Compare AI's instructions to the one provided in this lesson:
- Does it explicitly mention "two messages"?
- Does it say "silent" or "invisible" for the processing phase?
- Does it reference the specific tools in order?
- Does it guide markdown formatting?
- Does it mention checking tool status?

Evaluate: Which approach is clearer for the agent? Where does AI's version improve on this lesson's pattern? Where does this lesson's pattern provide better guidance?

**What you're learning:**
- How explicit instruction structure guides agent behavior
- The importance of naming patterns ("two-message," "silent processing") in agent prompts
- How to balance prescriptive guidance (follow this format) with flexibility (implement as you see fit)

### Reflection

After working with AI on these prompts:

- **Pattern recognition**: Can you identify coordinator-dispatcher in other agent workflows you've seen?
- **Trade-offs**: When would you use coordinator (silent processing) vs streaming (real-time feedback)? When do each serve the user better?
- **Implementation**: If you added a third tool (like audio generation), how would you maintain the two-message pattern while the agent does more work silently?
- **Generalization**: This pattern works for research, report generation, and file persistence. What other workflows could benefit from the coordinator pattern?

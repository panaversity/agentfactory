---
sidebar_position: 3
title: "Predictable Pipelines"
description: "Build deterministic multi-agent workflows with SequentialAgent and ParallelAgent"
keywords: [google adk, sequential agent, parallel agent, workflow agents, multi-agent, deterministic]
chapter: 35
lesson: 3
duration_minutes: 50

# HIDDEN SKILLS METADATA
skills:
  - name: "SequentialAgent Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can build a pipeline where agents execute in guaranteed order"

  - name: "ParallelAgent Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can build concurrent agents that merge results"

  - name: "Workflow vs LLM Routing Decision"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can articulate when to use workflow agents vs LLM handoffs"

learning_objectives:
  - objective: "Build SequentialAgent pipelines with deterministic execution order"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates researcher→writer→editor pipeline"

  - objective: "Build ParallelAgent for concurrent execution with merged results"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates parallel fact-check + sentiment analysis agents"

  - objective: "Choose between workflow agents and LLM routing based on reliability needs"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student explains tradeoffs in their own words"

cognitive_load:
  new_concepts: 2
  assessment: "2 new concepts (SequentialAgent, ParallelAgent) within B1 limit (7-10 concepts)"

differentiation:
  extension_for_advanced: "Nest SequentialAgent inside ParallelAgent; design complex DAG workflows with conditional branching"
  remedial_for_struggling: "Focus on SequentialAgent only; understand linear pipelines before concurrency"
---

# Predictable Pipelines

Your content creation system needs three stages: research, write, edit. You prompt Claude with a vague request. It decides to skip research. You get a draft that needs heavy revision. You prompt again. This time it runs all three stages. The quality varies wildly.

With LLM routing, the model decides routing at runtime. Sometimes it's brilliant. Sometimes it skips steps. You can't predict it. You can't test it.

**This is where workflow agents change everything.**

## The Problem with LLM Routing

Most multi-agent systems use LLM routing: The model decides when to hand off to the next agent.

```python
from google.adk.agents import LlmAgent

content_agent = LlmAgent(
    name="content_manager",
    model="gemini-2.5-flash",
    instruction="""You manage content creation. You can:
    1. Research topics (call research_agent)
    2. Write drafts (call writer_agent)
    3. Edit content (call editor_agent)

    Decide which agent to use based on the request.""",
    tools=[research_agent, writer_agent, editor_agent]
)
```

**Output:**

```
User: "Create an article about AI agents"
→ Model decides to call writer_agent directly (skips research)
→ Article lacks depth
→ User is unhappy

User: "Create an article about AI agents"
→ Model decides to call all three agents
→ Takes 3x longer
→ Same-quality output, wasted time
```

The problems:

1. **Unpredictability**: Same input, different routing decisions
2. **Uncontrollable order**: Model might call editor before research
3. **Hard to test**: Behavior varies per run; eval cases are unreliable
4. **Silent failures**: Agent skips a step, you don't know why
5. **Impossible to debug**: "Why did it skip research?" → Hard to answer

**LLM routing gives you flexibility. It costs you reliability.**

## Workflow Agents: Deterministic Order

Google ADK provides two workflow agent types that guarantee execution order:

1. **SequentialAgent**: Execute sub-agents in guaranteed sequence
2. **ParallelAgent**: Execute sub-agents concurrently, then merge results

Both are **deterministic**. Same input → same execution order, every time.

### SequentialAgent: Guaranteed Pipeline

Think of SequentialAgent like a manufacturing pipeline. Stage 1 completes, then Stage 2 starts. Stage 2 sees Stage 1's output. This continues until all stages finish.

```python
from google.adk.agents import LlmAgent, SequentialAgent
from google.adk.tools import google_search

researcher = LlmAgent(
    name="researcher",
    model="gemini-2.5-flash",
    instruction="Research the topic thoroughly. Gather facts, sources, and key insights. Be comprehensive.",
    tools=[google_search]
)

writer = LlmAgent(
    name="writer",
    model="gemini-2.5-flash",
    instruction="Write a clear, engaging article based on the research provided. Use the facts and sources from research. Structure with intro, body, conclusion."
)

editor = LlmAgent(
    name="editor",
    model="gemini-2.5-flash",
    instruction="Edit the article for clarity, grammar, flow, and impact. Improve writing quality without changing meaning."
)

content_pipeline = SequentialAgent(
    name="content_pipeline",
    description="Research a topic, write an article, then edit for quality. Always in that order.",
    sub_agents=[researcher, writer, editor]
)
```

**Output:**

```
User: "Write an article about AI agents"

Step 1 (researcher):
- Searches for information on AI agents
- Compiles facts, statistics, use cases
- Produces research document

Step 2 (writer):
- Receives researcher's output
- Writes comprehensive article using facts
- Produces draft article

Step 3 (editor):
- Receives draft from writer
- Edits for clarity and quality
- Produces final article

Result: Always researcher → writer → editor. Guaranteed.
```

**Why this matters**: Every time you use the pipeline, it follows the same sequence. No surprises.

### ParallelAgent: Concurrent Execution

Sometimes stages are independent. They don't need the output from each other. They can run simultaneously.

ParallelAgent executes all sub-agents at the same time, then combines their results.

```python
from google.adk.agents import LlmAgent, ParallelAgent

fact_checker = LlmAgent(
    name="fact_checker",
    model="gemini-2.5-flash",
    instruction="Review the article for factual accuracy. Check all claims, statistics, dates. Flag any inaccuracies or unsupported claims."
)

sentiment_analyzer = LlmAgent(
    name="sentiment_analyzer",
    model="gemini-2.5-flash",
    instruction="Analyze the emotional tone of the article. Is it neutral, positive, negative? Is the tone appropriate for the audience?"
)

quality_checker = LlmAgent(
    name="quality_checker",
    model="gemini-2.5-flash",
    instruction="Check structure and readability. Does it have clear intro? Logical flow? Strong conclusion? Is it too long or too short?"
)

analysis = ParallelAgent(
    name="article_analysis",
    description="Run fact-checking, sentiment analysis, and quality assessment simultaneously.",
    sub_agents=[fact_checker, sentiment_analyzer, quality_checker]
)
```

**Output:**

```
User: "Analyze this article"

All three agents run simultaneously:
[fact_checker] ← starts immediately
[sentiment_analyzer] ← starts immediately
[quality_checker] ← starts immediately

When all complete, results merge:
{
  "factual_accuracy": "All claims verified",
  "emotional_tone": "Neutral and authoritative",
  "structure": "Clear intro, logical flow, strong conclusion"
}

Total execution time ≈ time of slowest agent (instead of sum of all times)
```

**Why this matters**: When analyses are independent, run them in parallel. Faster results, same quality.

## When to Use Each Pattern

Use this decision framework:

| Scenario | Pattern | Why |
|----------|---------|-----|
| **Order matters** | SequentialAgent | Each stage depends on previous output |
| **Steps are independent** | ParallelAgent | Stages don't need each other's results |
| **Routing varies by input** | LLM routing (Agent with sub_agents) | Need flexibility for different requests |
| **Order + flexibility needed** | Nested workflow agents | Sequential stages with parallel branches inside |

### Example: When to Use Sequential

Research → Write → Edit requires sequential order because:
- Writer needs researcher's findings
- Editor needs writer's draft
- Can't optimize by running in parallel (they're dependencies)

```python
content_pipeline = SequentialAgent(
    name="content_pipeline",
    description="Research, write, edit—in that order.",
    sub_agents=[researcher, writer, editor]
)
```

### Example: When to Use Parallel

Analyzing a completed article doesn't require sequential execution because:
- Fact-checker works on article text
- Sentiment analyzer works on article text
- Quality checker works on article text
- They don't depend on each other

```python
analysis = ParallelAgent(
    name="analysis",
    description="Check facts, sentiment, and structure in parallel.",
    sub_agents=[fact_checker, sentiment_analyzer, quality_checker]
)
```

## Testing Workflow Agents

Because workflow agents execute deterministically, you can write reliable eval cases.

### Eval Case for Sequential Pipeline

```json
{
  "eval_set_id": "content_pipeline_tests",
  "eval_cases": [
    {
      "eval_id": "research_write_edit_sequence",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Write an article about AI agents in business"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Here's your article about AI agents in business:"}],
            "role": "model"
          },
          "intermediate_data": {
            "agent_sequence": ["researcher", "writer", "editor"],
            "tool_uses": [
              {"agent": "researcher", "tool": "google_search"},
              {"agent": "writer", "action": "compose article"},
              {"agent": "editor", "action": "polish and refine"}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "content_pipeline",
        "user_id": "test_user"
      }
    }
  ]
}
```

**Output:**

```
Test passes when:
- Agents execute in order: researcher → writer → editor
- Researcher called google_search tool
- Writer received research data
- Editor received draft from writer
- Final response contains polished article
```

### Eval Case for Parallel Pipeline

```json
{
  "eval_set_id": "parallel_analysis_tests",
  "eval_cases": [
    {
      "eval_id": "parallel_fact_sentiment_quality",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Analyze this article for accuracy, tone, and structure"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Analysis complete. Here are the results:"}],
            "role": "model"
          },
          "intermediate_data": {
            "parallel_agents": ["fact_checker", "sentiment_analyzer", "quality_checker"],
            "results_merged": true
          }
        }
      ],
      "session_input": {
        "app_name": "article_analysis",
        "user_id": "test_user"
      }
    }
  ]
}
```

**Output:**

```
Test passes when:
- All three agents execute (in parallel)
- Each agent completes and returns results
- Results merge into final response
- No agent blocks the others
```

**Why testing matters**: With eval cases, you verify that:
1. Sequential pipelines maintain order (not skipping steps)
2. Parallel agents truly run concurrently
3. Results merge correctly
4. The pipeline behaves the same way every time

## Nesting Workflow Agents: Complex Pipelines

For sophisticated systems, you can nest workflow agents:

```python
# Sequential stages
research_write_pipeline = SequentialAgent(
    name="content_creation",
    sub_agents=[researcher, writer]
)

# Parallel analysis
analysis_pipeline = ParallelAgent(
    name="analysis",
    sub_agents=[fact_checker, sentiment_analyzer]
)

# Combined: Create content, THEN analyze it
full_pipeline = SequentialAgent(
    name="full_workflow",
    sub_agents=[research_write_pipeline, analysis_pipeline]
)
```

**Output:**

```
Execution order:
1. researcher (parallel with writer)
2. writer (receives researcher output)
3. fact_checker AND sentiment_analyzer (parallel)
4. quality_checker
5. final_editor

This creates a directed acyclic graph (DAG) of workflows.
```

## The Reliability Advantage

Compare three approaches:

| Approach | Predictability | Testability | Performance |
|----------|---|---|---|
| **LLM Routing** | ❌ Varies per run | ❌ Hard to test | ⚠ Unpredictable |
| **SequentialAgent** | ✅ Guaranteed order | ✅ Easy to eval | ⚠ Linear time |
| **ParallelAgent** | ✅ All concurrent | ✅ Easy to eval | ✅ Optimal for independent work |

**The key insight**: Workflow agents trade flexibility for reliability. You sacrifice "the model decides" in exchange for "you decide the process."

For production systems, predictability beats flexibility. You want to know exactly what will happen.

## Safety Note

When designing parallel workflows, ensure sub-agents don't conflict. Two agents modifying the same resource simultaneously can cause race conditions:

```python
# PROBLEMATIC: Both agents modify the same file
parallel = ParallelAgent(
    name="bad_parallel",
    sub_agents=[
        writer_agent,  # Modifies document.md
        editor_agent   # Also modifies document.md
    ]
)
```

Solution: Use sequential order for operations that conflict, or add locking mechanisms.

```python
# CORRECT: Sequential for conflicting operations
pipeline = SequentialAgent(
    name="good_pipeline",
    sub_agents=[
        writer_agent,  # Modifies document first
        editor_agent   # Then modifies the result
    ]
)
```

## Try With AI

Use your AI companion to deepen your understanding of workflow design.

### Prompt 1: Design Your Pipeline

```
I need to build a [content creation / data analysis / code review]
system with multiple agents. Help me identify the stages that MUST
happen in order.

Ask me these questions to clarify dependencies:
1. What's the first stage that has no prerequisites?
2. What can't start until the previous stage finishes?
3. What stages could run in parallel without waiting?
4. What's the final stage that depends on all others?

Based on my answers, suggest whether I should use
SequentialAgent, ParallelAgent, or a combination.
```

**What you're learning**: Dependency analysis—the core skill for designing reliable workflows. This determines whether you use sequential, parallel, or nested pipeline designs.

### Prompt 2: Compare Approaches

```
I'm deciding between using ADK workflow agents (SequentialAgent,
ParallelAgent) or OpenAI SDK with LLM routing for my system.

The task is: [describe your use case].

Help me think through these tradeoffs:
1. What happens if a critical stage is skipped?
2. How would I test this system?
3. How would I know if something went wrong?
4. What's more important: flexibility or predictability?

Which approach would you recommend and why?
```

**What you're learning**: Framework-agnostic thinking about reliability architecture. Understanding that workflow patterns apply across tools—the abstraction matters more than the implementation.

### Prompt 3: Debug a Workflow Issue

```
My SequentialAgent pipeline sometimes produces inconsistent results.
The researcher finds good data, but the writer seems to ignore it.

Help me diagnose:
1. How do I trace data flow between agents?
2. What instrumentation should I add to see what each agent
   receives and produces?
3. How do I verify that researcher output is actually reaching
   the writer?
4. What eval cases would catch this problem?
```

**What you're learning**: Multi-agent debugging—understanding agent-to-agent communication. Learning to add observability so you can see what's happening inside pipelines.

Workflow agents are the foundation of production AI systems. They trade the flexibility of LLM routing for the reliability of deterministic execution. Every stage happens. Every stage happens in order. Every stage is testable. That's how you build systems you can trust.

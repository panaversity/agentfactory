---
sidebar_position: 7
title: "Workflow Agents: Deterministic Pipelines"
description: "Move beyond LLM routing to deterministic workflows with SequentialAgent, ParallelAgent, and LoopAgent for predictable, testable multi-step processes."
keywords: ["workflow agents", "sequential agent", "parallel agent", "loop agent", "deterministic pipelines", "agent orchestration", "ADK", "Google agents"]
chapter: 35
lesson: 7
duration_minutes: 55

# HIDDEN SKILLS METADATA
skills:
  - name: "SequentialAgent Pipeline Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can design and implement sequential agent pipelines with ordered execution"

  - name: "ParallelAgent Concurrent Execution"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can identify independent tasks and implement parallel execution"

  - name: "LoopAgent Iterative Refinement"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can implement iterative improvement with exit conditions and safety limits"

  - name: "Workflow Agent Pattern Selection"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate requirements and choose appropriate workflow agent type"

  - name: "Testability and Predictability in Agent Design"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Software Quality"
    measurable_at_this_level: "Student understands trade-offs between LLM routing flexibility and workflow agent predictability"

learning_objectives:
  - objective: "Understand why deterministic workflows matter in production agent systems"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student articulates when routing is too flexible and when determinism is required"

  - objective: "Design and implement SequentialAgent for fixed-order pipelines (research → write → edit)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates working sequential pipeline with execution order guaranteed"

  - objective: "Design and implement ParallelAgent for independent concurrent tasks"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student identifies independent tasks and implements parallel execution"

  - objective: "Design and implement LoopAgent for quality-based iteration with safety limits"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates iterative agent with clear exit condition and max_iterations constraint"

  - objective: "Select appropriate workflow agent type based on requirements (sequential vs parallel vs iterative)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student evaluates multiple scenarios and justifies pattern choice"

cognitive_load:
  new_concepts: 9
  assessment: "9 concepts (SequentialAgent, deterministic vs flexible routing, pipeline ordering, ParallelAgent, task independence, LoopAgent, exit conditions, max_iterations, pattern selection) within B1 limit of 10 concepts - PASS"

differentiation:
  extension_for_advanced: "Explore nested workflow agents (SequentialAgent containing ParallelAgent); implement custom exit conditions beyond exit_loop; investigate callback integration with workflow agents for observability"
  remedial_for_struggling: "Start with simple 2-agent sequential pipelines before 3-agent examples; focus on ParallelAgent first (simpler concept); defer max_iterations tuning until core pattern is comfortable"

generated_by: content-implementer
source_spec: Chapter 35 Lesson 7 specification
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Workflow Agents: Deterministic Pipelines

You've built multi-agent systems where the LLM decides what to do next. "Research is done? Hand off to writer. Writing is done? Hand off to editor. Unless... maybe we need more research? Or different research?" The agent's flexibility is powerful, but it's also unpredictable. You can't guarantee the order of operations. Testing becomes guesswork. Debugging takes hours because execution paths vary on every run.

Production systems often need the opposite: **guaranteed order**. Research always happens before writing. Financial data always enriches news before podcasting. You complete one phase fully before moving to the next. Some tasks should run in parallel (fact-checking and sentiment analysis happen simultaneously, not sequentially). Some processes improve through iteration—run a draft, evaluate, refine, run again until quality meets criteria.

Google ADK provides three workflow agent types for exactly these patterns. They sacrifice LLM flexibility for deterministic execution, predictable testing, and guaranteed orchestration.

## The Problem with Pure LLM Routing

Let's establish why determinism matters by examining what LLM routing actually does.

### How LLM Routing Works

In your multi-agent system from Lesson 6, you have:

```python
dispatcher = LlmAgent(
    name="dispatcher",
    instruction="Route the request to the appropriate agent: research, enrich, or podcast.",
    tools=[
        AgentTool(researcher),
        AgentTool(enricher),
        AgentTool(podcaster)
    ]
)
```

The LLM examines the request and decides: "I'll call researcher first." Then it examines the result and decides: "Now I should call enricher." Then: "Now podcaster."

**The flexibility:**
- Agent can skip enrichment if research is thorough
- Agent can revisit research if enrichment discovers gaps
- Agent can branch to unexpected paths based on content
- Agent adapts to novel inputs you didn't anticipate

**The problem:**
- Execution order is unpredictable
- Testing requires diverse prompts to reveal behavior
- Debugging why the agent skipped a step requires understanding LLM reasoning
- Performance is inefficient (agent reasons at each decision point)
- You cannot guarantee phases complete in order

### When Routing Fails

Imagine your AI News Podcast agent processes "breaking AI news about Deepseek's new model."

**Run 1:**
Research → Enrich with financial data → Generate podcast

**Run 2 (same input):**
Research → Research again → Enrich → Generate podcast (LLM thought initial research was incomplete)

**Run 3:**
Research → Enrich → Podcast → Realize podcast is too brief → Research again (LLM changed its mind)

Each run is different. You can't test it deterministically. You can't guarantee quality. You can't optimize performance because the execution path is unpredictable.

### Trade-off: Flexibility vs Predictability

| Dimension | LLM Routing | Workflow Agents |
|-----------|----------|-----------------|
| **Flexibility** | High (agent adapts to novel inputs) | Low (fixed execution order) |
| **Predictability** | Low (execution varies per run) | High (guaranteed order) |
| **Testability** | Hard (many paths to test) | Easy (one path per agent) |
| **Performance** | Slower (LLM reasons at each step) | Faster (no intermediate decisions) |
| **Debugging** | Hard (why did agent skip?") | Easy (follows prescribed order) |
| **Use case** | Novel, adaptive problems | Standardized, repeatable workflows |

**The insight**: You don't choose routing OR workflow agents. You use routing for novel decisions and workflow agents for standardized processes.

## SequentialAgent: Fixed-Order Pipelines

When you need guaranteed execution order, use **SequentialAgent**. It runs sub-agents in strict sequence: first completes → second starts → third starts.

### The Pattern: Research → Write → Edit

Imagine a content creation pipeline:

1. **Researcher** digs into a topic, gathers sources
2. **Writer** takes research and creates article
3. **Editor** refines for clarity and accuracy

This order is mandatory. The writer needs research output. The editor needs the article. You cannot parallelize (editor cannot run during research). You cannot skip steps. You need guarantee of sequence.

### Implementation

```python
from google.adk.agents import LlmAgent, SequentialAgent

# Define individual agents
researcher = LlmAgent(
    name="researcher",
    model="gemini-2.5-flash",
    instruction="Research the topic thoroughly using available tools. Gather multiple sources. Provide comprehensive research findings.",
    tools=[google_search]
)

writer = LlmAgent(
    name="writer",
    model="gemini-2.5-flash",
    instruction="Write a well-structured article based on the research provided. Use clear headings and evidence from sources.",
    tools=[]  # No tools needed, uses research output
)

editor = LlmAgent(
    name="editor",
    model="gemini-2.5-flash",
    instruction="Edit the article for clarity, grammar, and accuracy. Ensure sources are properly attributed.",
    tools=[]  # No tools needed, reviews article
)

# Compose into sequential pipeline
pipeline = SequentialAgent(
    name="content_pipeline",
    description="Research topic, write article, then edit for publication",
    sub_agents=[researcher, writer, editor]
)
```

### How Sequential Execution Works

When you run the pipeline:

```python
result = pipeline.run("Write an article about AI agents in enterprise software")
```

ADK executes:

1. **Pass input to researcher**: "Write an article about AI agents..."
2. **Researcher completes**: Returns research findings
3. **Pass research output to writer**: "Here's the research: [findings]. Now write the article."
4. **Writer completes**: Returns written article
5. **Pass article to editor**: "Here's the article: [content]. Now edit it."
6. **Editor completes**: Returns edited version
7. **Return final result**: Edited article

**Key property**: Each agent receives the previous agent's output as input. The chain is deterministic.

### Specifying Data Flow (SequentialAgent with Context)

By default, each agent receives text output from the previous agent. For structured data, use **ToolContext**:

```python
from google.adk.agents import SequentialAgent
from google.adk.tools import ToolContext

# Define agents that share structured context
researcher = LlmAgent(
    name="researcher",
    instruction="Research the topic and provide findings as JSON.",
    tools=[google_search]
)

writer = LlmAgent(
    name="writer",
    instruction="Access research findings from context['research']. Write article based on it.",
    tools=[]
)

# Create shared context that persists across agents
context = ToolContext()

pipeline = SequentialAgent(
    name="content_pipeline",
    sub_agents=[researcher, writer],
    context=context  # Shared across pipeline
)
```

In this pattern, agents can explicitly access shared context rather than relying on text output parsing.

## ParallelAgent: Concurrent Execution

When tasks are independent, run them simultaneously with **ParallelAgent**. It executes all sub-agents concurrently and waits for all to complete before returning.

### The Pattern: Fact-Check AND Sentiment Analysis in Parallel

Imagine analyzing a news article:

1. **Fact-checker** verifies claims against reliable sources
2. **Sentiment analyzer** evaluates tone and bias
3. Both tasks are independent—neither needs output from the other
4. Parallel execution is 2x faster than sequential

### Implementation

```python
from google.adk.agents import LlmAgent, ParallelAgent

fact_checker = LlmAgent(
    name="fact_checker",
    model="gemini-2.5-flash",
    instruction="Verify factual claims in the text. Identify which claims are supported and which are unsupported. Rate confidence in each verification.",
    tools=[google_search]
)

sentiment_analyzer = LlmAgent(
    name="sentiment_analyzer",
    model="gemini-2.5-flash",
    instruction="Analyze the emotional tone and potential bias of the text. Identify loaded language, emotional appeals, and perspective.",
    tools=[]
)

podcast_metadata = LlmAgent(
    name="metadata_extractor",
    model="gemini-2.5-flash",
    instruction="Extract podcast metadata: potential title, key topics, target audience, recommended length.",
    tools=[]
)

# Run all three concurrently
analysis = ParallelAgent(
    name="content_analysis",
    description="Analyze article for factuality, sentiment, and podcast potential simultaneously",
    sub_agents=[fact_checker, sentiment_analyzer, podcast_metadata]
)
```

### Execution Model

When you run:

```python
result = analysis.run("Apple announces revolutionary AI capabilities...")
```

ADK executes:

```
Time 0s:   fact_checker.run(), sentiment_analyzer.run(), podcast_metadata.run() all START
Time 2s:   fact_checker completes → results available
Time 3s:   sentiment_analyzer completes → results available
Time 3.5s: podcast_metadata completes → all done
Time 3.5s: ParallelAgent returns combined results
```

Compare to sequential (would take 2s + 3s + 3.5s = 8.5s). Parallel takes 3.5s—2.4x faster.

### Handling Results

ParallelAgent returns results as a dictionary, one per agent:

```python
results = analysis.run("Article text...")

# Access individual results
fact_checks = results['fact_checker']
sentiment = results['sentiment_analyzer']
metadata = results['metadata_extractor']
```

### When to Parallelize

Parallelize when:
- Tasks are independent (output from one doesn't feed into another)
- Task duration is significant (parallelizing 0.1s tasks isn't worth overhead)
- Latency matters (time-sensitive operations)

Don't parallelize when:
- Tasks depend on each other (Task B needs Task A's output)
- Tasks are very fast (<100ms) (parallelization overhead exceeds savings)
- You need ordered results (Sequential guarantees order; Parallel doesn't)

## LoopAgent: Iterative Refinement

When quality improvement requires iteration—"run, evaluate, refine, run again until acceptable"—use **LoopAgent**. It repeats a task, evaluating exit conditions after each iteration.

### The Pattern: Generate Draft, Check Quality, Refine Until Good

Imagine generating a podcast script:

1. **Generator** creates initial podcast script
2. **Evaluator** checks if script is good enough
3. If not: Refine and try again
4. If yes: Done
5. **Safety**: Stop after 5 iterations regardless

### Implementation

```python
from google.adk.agents import LlmAgent, LoopAgent
from google.adk.tools import exit_loop

# Agent that generates AND can self-improve
podcast_generator = LlmAgent(
    name="podcast_generator",
    model="gemini-2.5-flash",
    instruction="Generate a podcast script. If you see [REFINE] in context, improve the script based on feedback. When script is good enough, call exit_loop.",
    tools=[exit_loop]
)

# Agent that evaluates (optional for complex evaluations)
script_evaluator = LlmAgent(
    name="script_evaluator",
    model="gemini-2.5-flash",
    instruction="Review podcast script. Check: Is pacing good? Are transitions smooth? Is tone consistent? If issues exist, provide specific feedback for refinement.",
    tools=[]
)

# Iterative pipeline
refinement_loop = LoopAgent(
    name="podcast_refinement",
    description="Generate podcast script, evaluate, refine until quality threshold met",
    sub_agents=[podcast_generator],
    max_iterations=5  # Safety limit
)
```

### How LoopAgent Executes

When you run:

```python
result = refinement_loop.run("Create a 10-minute podcast about AI agents")
```

ADK executes:

```
Iteration 1:
  - podcast_generator runs with input
  - Generates script v1
  - exit_loop not called → continues

Iteration 2:
  - podcast_generator receives feedback from evaluator
  - Improves script v2
  - exit_loop not called → continues

Iteration 3:
  - podcast_generator refines script v3
  - Calls exit_loop → STOP

Result: script v3
```

### Exit Condition Design

The loop exits when:
1. **Agent calls exit_loop** — Agent determines quality is sufficient
2. **max_iterations reached** — Safety timeout (prevents infinite loops)

Whichever happens first.

### Complex Loop Example: Solver with Feedback

```python
from google.adk.agents import LlmAgent, LoopAgent
from google.adk.tools import exit_loop

solver = LlmAgent(
    name="problem_solver",
    model="gemini-2.5-flash",
    instruction="""
    Solve the given problem step-by-step.

    After solving:
    1. Verify your answer makes sense
    2. Check for edge cases
    3. If solution is complete and verified, call exit_loop
    4. If unsure, ask for clarification
    """,
    tools=[exit_loop, code_execution]  # Can write and run code
)

iterative_solver = LoopAgent(
    name="iterative_problem_solver",
    description="Solve problems iteratively, validating at each step",
    sub_agents=[solver],
    max_iterations=10
)
```

Here the solver can write code, run it, inspect results, and decide if it's correct.

## Decision Framework: Choosing Your Pattern

You now have three tools: Sequential, Parallel, Loop. How do you choose?

### The Decision Tree

**Question 1: Do tasks depend on each other?**
- YES: Use SequentialAgent (Task B needs Task A's output)
- NO: Go to Question 2

**Question 2: Can tasks run at the same time?**
- YES: Use ParallelAgent (independent tasks, run concurrently)
- NO: Use SequentialAgent (still need ordering)

**Question 3: Is the process iterative?**
- YES: Use LoopAgent (run, evaluate, refine until success)
- NO: Chose one of the above

### Decision Table

| Scenario | Pattern | Reason |
|----------|---------|--------|
| Research → Write → Edit | SequentialAgent | Fixed order, each step needs previous output |
| Fact-check + Sentiment + Metadata simultaneously | ParallelAgent | Independent analyses, run concurrently |
| Generate draft → Evaluate → Refine until good | LoopAgent | Iterative improvement with exit condition |
| Daily report: extract data → format → email | SequentialAgent | Pipeline steps must happen in order |
| Image analysis: detect objects + read text + recognize faces | ParallelAgent | Independent vision tasks |
| Optimize configuration: try setting → test → adjust until passing | LoopAgent | Iterative optimization |
| Research → Write, but writing needs financial data enriched separately | Nested: Sequential containing Parallel | Some steps sequential, others parallel |

### When to Still Use LLM Routing

Workflow agents remove flexibility. Sometimes you need it:

- **Novel inputs requiring adaptive logic** — LLM routing can branch unpredictably based on content
- **Uncertain execution paths** — If you don't know which agents to call until you analyze the request
- **Mixed deterministic + flexible workflows** — Use Sequential/Parallel for structured parts, routing for decision points

**Hybrid approach** (Lesson 6 advanced):

```python
# LLM decides WHICH workflow to invoke
dispatcher = LlmAgent(
    name="dispatcher",
    instruction="Determine if this is a research-heavy request or optimization-heavy request. Call appropriate_workflow_agent.",
    tools=[
        AgentTool(research_pipeline),      # SequentialAgent
        AgentTool(optimization_loop),       # LoopAgent
    ]
)
```

## Testing Workflow Agents

Deterministic workflow agents unlock testing possibilities that LLM routing cannot provide.

### Test Example: SequentialAgent

```python
# Setup
pipeline = SequentialAgent(
    name="content_pipeline",
    sub_agents=[researcher, writer, editor]
)

# Test: Verify pipeline output contains research sources
result = pipeline.run("Write about AI agents")
assert "source" in result.lower(), "Editor should preserve source attribution"
assert "research" not in result, "Final output shouldn't expose raw research structure"

# Test: Verify execution order (research comes first)
# In a real system, you'd inspect execution logs/events
events = pipeline.get_execution_events()
research_time = [e.timestamp for e in events if e.agent == "researcher"][0]
writer_time = [e.timestamp for e in events if e.agent == "writer"][0]
assert research_time < writer_time, "Researcher must execute before writer"
```

### Test Example: ParallelAgent

```python
import time

analysis = ParallelAgent(
    name="content_analysis",
    sub_agents=[fact_checker, sentiment_analyzer, metadata_extractor]
)

# Verify parallelization (should be faster than sum of individual times)
start = time.time()
result = analysis.run("Article text...")
elapsed = time.time() - start

# If truly parallel, should be ~max_time, not sum_time
assert elapsed < 8, "Parallel should be faster than sequential sum"
```

### Test Example: LoopAgent

```python
# Test: Verify loop exits on success condition
refinement = LoopAgent(
    name="refinement",
    sub_agents=[generator],
    max_iterations=5
)

result = refinement.run("Generate podcast")

# Verify exit happened (script should be final version)
assert "podcast" in result.lower()
assert len(result) > 100, "Generator should produce substantial output"

# Verify max_iterations is enforced
events = refinement.get_execution_events()
iterations_run = len([e for e in events if e.type == "iteration_start"])
assert iterations_run <= 5, "Loop should not exceed max_iterations"
```

## Composite Patterns

Real systems combine these patterns.

### Parallel Within Sequential

```python
# Research phase
researcher = LlmAgent(...)

# Analysis phase (parallel)
analysis = ParallelAgent(
    name="analysis",
    sub_agents=[fact_checker, sentiment_analyzer]
)

# Writing phase
writer = LlmAgent(...)

# Full pipeline: Research → (Fact-check + Sentiment in parallel) → Write
full_pipeline = SequentialAgent(
    name="full_pipeline",
    sub_agents=[researcher, analysis, writer]
)
```

### Loop Within Parallel

```python
# Two independent iterative tasks running in parallel
loop1 = LoopAgent(
    name="title_optimization",
    sub_agents=[title_generator],
    max_iterations=3
)

loop2 = LoopAgent(
    name="description_optimization",
    sub_agents=[description_generator],
    max_iterations=3
)

# Run both optimization loops simultaneously
parallel_optimization = ParallelAgent(
    name="content_optimization",
    sub_agents=[loop1, loop2]
)
```

## Try With AI

### Prompt 1: Sequential Pipeline Design

```
Design a SequentialAgent for code review workflow:
1. Analyzer: Analyzes code for bugs, performance issues, design patterns
2. Suggester: Proposes improvements based on analysis
3. Integrator: Integrates suggestions into improved version

Write the ADK code defining these three agents and composing them into a SequentialAgent.

Consider:
- What instruction focuses each agent on its role?
- How does output from Analyzer inform Suggester?
- What should Integrator produce as final output?
- What tools might each agent need?

Run it with: adk run agent.py

Input: A Python function with clear room for improvement

What does the sequential approach guarantee that LLM routing cannot?
```

**What you're learning:** Sequential dependency and data flow in multi-agent systems

### Prompt 2: Identifying Parallelizable Tasks

```
Consider a research system that needs to:
1. Search for academic papers on a topic
2. Analyze headlines from recent news
3. Check trending discussions on social media
4. Compile market data from financial sources

Which of these can run in parallel? Which must be sequential?

For the parallelizable subset, write a ParallelAgent that runs them concurrently.

Consider:
- Do any tasks depend on outputs from others?
- Which tasks would benefit from concurrent execution?
- How would you combine results from all four sources?
- What would happen if you tried to make all four parallel?

Ask the parallel agent: "What's the current state of AI adoption in enterprise?"

What information appears in the results? Does parallel execution reveal patterns that sequential might miss?
```

**What you're learning:** Task independence analysis and concurrent execution benefits

### Prompt 3: Iterative Refinement with Exit Conditions

```
Design a LoopAgent that iteratively improves an email draft for professionalism:

1. Generator: Creates an initial email response
2. Evaluator: Checks if email is professional, clear, and concise
3. Refiner: Improves based on feedback

The generator should improve the email until it meets quality criteria OR max 4 iterations is reached.

Implement:
- Generator agent with instructions to improve based on feedback
- Clear exit_loop condition (when is email "good enough"?)
- max_iterations safety limit

Run it with: adk run agent.py

Input: A casual, unprofessional email draft that needs refinement

Then ask: "I need to reply to my CEO about the project delay. Here's my initial response: [draft]"

How many iterations did it take to produce a professional response? What changed across iterations? When did the agent decide the email was ready?
```

**What you're learning:** Iterative improvement patterns with measurable quality criteria

---

**Key Insight**: ADK gives you BOTH flexibility and determinism. Use LLM routing when the process is novel and adaptive. Use workflow agents when the process is standardized and repeatable. The most powerful production systems combine both—routing for strategy, workflows for execution.

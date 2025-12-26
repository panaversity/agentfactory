---
sidebar_position: 9
title: "Subagents for Parallel Work"
description: "Define specialized subagents and invoke them via the Task tool to decompose complex problems into parallel workflows—the foundation for production Digital FTE orchestration."
keywords: [subagents, Task tool, parallel execution, agent composition, orchestration, Digital FTE]
chapter: 36
lesson: 9
duration_minutes: 35

# HIDDEN SKILLS METADATA
skills:
  - name: "Subagent Definition and Configuration"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can define specialized subagents with AgentDefinition, configure tool restrictions per subagent, and reason about tool isolation and model selection"

  - name: "Task-Based Parallel Execution"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can decompose workflows into parallel subagent invocations using the Task tool and synthesize results from independent concurrent work"

  - name: "Orchestration Architecture Decisions"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can decide when to use subagents vs single-agent approaches, reason about context isolation, and evaluate tradeoffs between specialization and coordination"

learning_objectives:
  - objective: "Define specialized subagents using AgentDefinition with appropriate tool restrictions and model selection"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student creates subagent definitions matching specific domain workflows"

  - objective: "Invoke subagents via Task tool for parallel work and synthesize results into coherent outputs"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements parallel execution patterns with proper result aggregation"

  - objective: "Analyze tradeoffs between single-agent and multi-agent architectures for complex workflows"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student evaluates architectural decisions with reasoning about specialization, context isolation, and failure modes"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (subagent definition, tool restrictions, model selection, Task tool, parallel execution, context isolation, result synthesis, orchestration patterns) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design multi-level subagent hierarchies where subagents spawn their own subagents; analyze communication patterns and context flow through agent hierarchies; evaluate scheduling strategies for resource-constrained environments"
  remedial_for_struggling: "Focus on simple two-subagent pattern (code reviewer + test runner) before multi-level hierarchies; use concrete examples of when subagents solve real problems vs introducing complexity; compare to familiar parallel programming concepts (threads, processes)"
---

# Subagents for Parallel Work

You've built single agents. You've learned to configure tools. Now you're going to solve a harder problem: **decomposing complex workflows into specialized agents that work in parallel**.

This is where orchestration begins. This is how you build production Digital FTEs that handle multi-faceted problems autonomously.

## The Problem: Single Agent Bottlenecks

Imagine you're building a Digital FTE that reviews code before deployment. The agent needs to:

1. **Security analysis**: Check for authentication bypasses, SQL injection patterns, credential leaks
2. **Performance analysis**: Identify N+1 queries, memory leaks, inefficient algorithms
3. **Testing coverage**: Verify new code has tests, check test quality
4. **Documentation validation**: Ensure API docs match implementation

A single agent could do all of this. But there's a cost:

**The Context Problem**

Each domain requires different expertise:
- Security analysis needs threat modeling frameworks
- Performance analysis needs algorithmic complexity knowledge
- Testing requires understanding coverage metrics and test patterns
- Documentation needs API specification standards

If you pack all this expertise into one system prompt, you're competing for space in the agent's context window. The security patterns are competing with test coverage metrics for attention. The agent must reason about unrelated domains simultaneously.

**The Efficiency Problem**

These tasks are independent. Security analysis doesn't depend on testing coverage. Performance analysis doesn't require documentation review. But a single agent processes sequentially: analyze → analyze → analyze → done.

You could run them in parallel if they were separate agents.

**The Specialization Problem**

A security expert thinks differently than a performance engineer. If one agent must excel at both, you're training a generalist instead of leveraging specialists. The agent's reasoning is diluted across unrelated domains.

## The Solution: Subagents for Specialization and Parallelism

Subagents are focused agents optimized for specific tasks. Instead of one agent doing everything, you define multiple agents with different configurations:

```python
from claude_agent_sdk import AgentDefinition, ClaudeAgentOptions, query

# Main orchestrator agent
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Grep", "Task"],  # Task required to invoke subagents
    agents={
        # Security specialist - only has tools for code analysis
        "security-reviewer": AgentDefinition(
            description="Expert security analyst identifying vulnerabilities, authentication bypasses, and credential leaks",
            prompt="""You are a security expert. Analyze code for:
- Authentication/authorization bypasses
- SQL injection and XSS patterns
- Credential exposure (API keys, secrets in code)
- Input validation failures
Focus on HIGH and CRITICAL severity issues.""",
            tools=["Read", "Grep", "Glob"],  # Read-only tools
            model="sonnet"  # Balanced performance/cost for analysis
        ),

        # Performance specialist - can execute and measure
        "performance-analyst": AgentDefinition(
            description="Performance engineer identifying algorithmic inefficiencies and resource bottlenecks",
            prompt="""You are a performance engineer. Analyze code for:
- O(n²) or worse algorithmic patterns
- N+1 query problems in database code
- Memory leaks and unbounded allocations
- Cache misses and inefficient data structures
Suggest optimizations with estimated impact.""",
            tools=["Read", "Bash", "Glob"],  # Can execute for profiling
            model="haiku"  # Faster for straightforward analysis
        ),

        # Testing specialist - focuses on coverage and quality
        "testing-analyst": AgentDefinition(
            description="Testing expert evaluating test coverage and quality",
            prompt="""You are a testing expert. Analyze for:
- Test coverage of new code (aim for 80%+)
- Test quality (are assertions meaningful?)
- Edge cases and error paths
- Flaky test patterns
Report coverage percentage and suggest specific test improvements.""",
            tools=["Read", "Bash", "Glob"],
            model="haiku"
        ),
    }
)

# Orchestration flow
async def review_code(pull_request_path: str):
    """Review code through parallel specialist analysis"""

    prompt = f"""Review the code changes in {pull_request_path}.

Use your subagents to parallelize the review:
1. Invoke "security-reviewer" subagent to find security issues
2. Invoke "performance-analyst" subagent to identify performance problems
3. Invoke "testing-analyst" subagent to evaluate test coverage
4. Synthesize all three reports into a comprehensive review"""

    async for message in query(prompt=prompt, options=options):
        if hasattr(message, "result"):
            print(message.result)
```

**What's happening here:**

1. **Each subagent is specialized**: Security reviewer thinks about threats. Performance analyst thinks about efficiency. Testing analyst thinks about coverage.

2. **Each has different tools**: Security reviewer gets read-only tools (Read, Grep). Performance analyst gets execution (Bash) for profiling. Testing analyst can run tests.

3. **Each has different model**: Sonnet for complex reasoning (security), Haiku for straightforward analysis (performance, testing) saves cost.

4. **Task tool enables parallelism**: When the orchestrator invokes all three subagents, they work simultaneously instead of sequentially.

This is the pattern for production Digital FTEs: **Decompose → Specialize → Parallelize → Synthesize**.

## AgentDefinition: Configuring Subagents

When you define a subagent, you're answering four questions:

### 1. What's This Agent's Specialty?

The `description` field tells the orchestrator why this agent exists:

```python
AgentDefinition(
    description="Expert security analyst identifying vulnerabilities..."
)
```

This description is visible to the orchestrator agent. When the orchestrator needs security analysis, it knows which subagent to invoke.

### 2. What's the Agent's Reasoning Framework?

The `prompt` field is the system prompt—the agent's core instructions:

```python
AgentDefinition(
    prompt="""You are a security expert focused on:
- Authentication bypasses
- Injection attacks
- Credential exposure
Think like a red-team operator: what would an attacker exploit?"""
)
```

This isn't generic ("be a security expert"). It's specific about what security means in this context. It activates the right reasoning pattern.

### 3. What Tools Does This Agent Have Access To?

The `tools` field restricts what the agent can do:

```python
AgentDefinition(
    tools=["Read", "Grep", "Glob"]  # Read-only analysis
)
```

vs.

```python
AgentDefinition(
    tools=["Read", "Bash", "Edit"]  # Can modify code
)
```

This is **context isolation**. A security reviewer doesn't need to modify code; it just needs to identify issues. A code formatter needs Edit but not WebSearch.

By restricting tools, you:
- **Reduce blast radius**: If a subagent goes wrong, it can't do unintended damage
- **Improve focus**: The agent isn't distracted by tools it doesn't need
- **Enable composition**: Multiple agents can work on the same codebase without conflicts

### 4. Which Model Should This Agent Use?

The optional `model` field specifies compute:

```python
# Complex reasoning → use Sonnet
"security-reviewer": AgentDefinition(
    prompt="Find authentication bypasses...",
    model="sonnet"  # More capable, slightly higher cost
)

# Straightforward analysis → use Haiku
"testing-analyst": AgentDefinition(
    prompt="Report test coverage percentages...",
    model="haiku"  # Faster, lower cost
)
```

**When to choose which model:**

- **Sonnet**: Complex multi-step reasoning, tricky edge cases, architectural decisions
- **Haiku**: Pattern recognition, straightforward analysis, high-volume processing

For the testing-analyst example: "Tell me test coverage percentage and suggest missing tests" is straightforward pattern matching. Haiku handles this well. But "Find authentication bypasses" requires adversarial thinking—Sonnet excels here.

## Invoking Subagents: The Task Tool

The **Task tool** is how the orchestrator invokes subagents:

```python
async def orchestrate_review(code_path: str):
    """Invoke multiple subagents in parallel"""

    prompt = f"""Analyze code in {code_path}.

Review this from three perspectives:
1. Security: Invoke the "security-reviewer" subagent
2. Performance: Invoke the "performance-analyst" subagent
3. Testing: Invoke the "testing-analyst" subagent

Combine findings into a single review report."""

    async for message in query(prompt=prompt, options=options):
        print(message)
```

When the orchestrator encounters "invoke the security-reviewer subagent", the Task tool:

1. **Spawns a new agent**: Independent context window
2. **Executes with subagent configuration**: Uses "security-reviewer"'s tools and prompt
3. **Returns results**: Subagent completes and returns findings
4. **Continues orchestrator**: Main agent now has security findings

**Key insight**: Task invocations happen in PARALLEL. All three subagents work simultaneously, not sequentially.

```
Timeline WITHOUT Subagents:
Time 0     → Security analysis (5 min)
Time 5     → Performance analysis (3 min)
Time 8     → Testing analysis (2 min)
Time 10    → DONE

Total: 10 minutes

Timeline WITH Subagents:
Time 0-5   → Security analysis (5 min) ----\
             Performance analysis (3 min)   --> All parallel
             Testing analysis (2 min)     /
Time 5     → DONE

Total: 5 minutes (2x faster for independent tasks)
```

## Result Synthesis: Combining Parallel Outputs

When you invoke multiple subagents in parallel, each returns results independently. The orchestrator must synthesize them:

```python
async def synthesize_review(code_path: str):
    """Orchestrator invokes three subagents and synthesizes results"""

    prompt = f"""Review code at {code_path}.

Step 1: Get security analysis
- Invoke "security-reviewer" subagent with: "Analyze this code for security issues"
- Save the security report

Step 2: Get performance analysis
- Invoke "performance-analyst" subagent with: "Check for performance problems"
- Save the performance report

Step 3: Get testing coverage
- Invoke "testing-analyst" subagent with: "Evaluate test coverage"
- Save the coverage report

Step 4: Synthesize
Combine all three reports into a single review following this structure:

## Security Issues
[Insert security report findings]

## Performance Concerns
[Insert performance findings]

## Testing Coverage
[Insert coverage assessment]

## Overall Recommendation
[Synthesize across all three domains: Is this code ready to merge?]"""

    async for message in query(prompt=prompt, options=options):
        if hasattr(message, "result"):
            print(message.result)
```

**The key pattern**: Orchestrator → [Invoke subagents] → [Collect results] → [Synthesize into output]

This requires the orchestrator to be smart about:
- **Requesting specific outputs** from subagents (so results are structured)
- **Aggregating findings** across domains
- **Making decisions** based on combined analysis

## Context Isolation: Why It Matters

When you define subagents with restricted tools, you're creating context isolation:

**Without isolation**:

```python
# Single agent with all tools
agent = AgentDefinition(
    tools=["Read", "Edit", "Bash", "WebSearch", "Grep"]
)
```

The agent's context window contains documentation for ALL tools. It must reason about which tool to use for each step. If an attack exploits the agent to use WebSearch, it can reach outside your codebase.

**With isolation**:

```python
"security-reviewer": AgentDefinition(
    tools=["Read", "Grep", "Glob"]  # Only analysis tools
)

"code-formatter": AgentDefinition(
    tools=["Read", "Edit"]  # Only formatting tools
)
```

Each subagent is isolated:
- Security reviewer can't edit code (can't introduce bugs)
- Formatter can't search the web (can't fetch malware)
- Each has smaller context → faster reasoning

## Orchestration Patterns: When to Use Subagents

**Use subagents when:**

1. **Tasks are independent**: Code review (security, performance, testing) are independent—no task depends on another's output

2. **Specialization matters**: Each domain has its own expertise, patterns, and reasoning framework

3. **Parallelism saves time**: Independent tasks can run simultaneously—total time is the longest single task, not the sum

4. **Tool isolation is necessary**: Different agents need different capabilities (some read-only, some write, some execute)

5. **Context windows are tight**: Packing everything into one system prompt exceeds token budgets or dilutes reasoning

**Don't use subagents when:**

1. **Tasks are sequential**: Task B depends on Task A's output. Parallelism doesn't help. Single orchestrator agent handles ordering better.

2. **Coordination overhead exceeds benefit**: Spawning 10 subagents to do 10 two-minute tasks costs more in spawn overhead than it saves in time.

3. **Shared reasoning required**: The agent needs unified understanding across domains. Splitting into specialists breaks the reasoning.

4. **State management is complex**: Subagents can't easily share mutable state. If coordination requires synchronized updates, single agent is simpler.

## Architecture Decision: Single Agent vs. Orchestrator + Subagents

**Scenario: Building a Digital FTE for Code Review**

| Decision | Single Agent | Orchestrator + Subagents |
|----------|--------------|-------------------------|
| **Model** | ClaudeAgentOptions with all tools | ClaudeAgentOptions with Task tool + specialized AgentDefinitions |
| **Tool Access** | `tools=["Read", "Bash", "Grep", "Edit"]` (all-in-one) | Orchestrator: `["Read", "Grep", "Task"]` / Subagents: restricted per specialty |
| **Reasoning Load** | One agent reasons about security, performance, and testing simultaneously | Each agent specializes; orchestrator coordinates |
| **Context Efficiency** | Competes for space: security patterns vs. performance patterns vs. test patterns | Isolated: security agent gets threat models, performance agent gets optimization patterns |
| **Parallelism** | N/A - sequential analysis | Parallel: Security + Performance + Testing simultaneously |
| **Execution Time** | 10 minutes (5 + 3 + 2 sequentially) | 5 minutes (5 parallel, since 5 > 3 and 5 > 2) |
| **Failure Mode** | One agent goes wrong, everything fails | One subagent fails, orchestrator may recover by ignoring that perspective |
| **Coordination Cost** | None | Orchestrator must synthesize three reports |
| **Best For** | Simple workflows, sequential dependencies | Complex workflows, independent specialists, time-sensitive |

**The trade-off: Specialization + Parallelism vs. Coordination Complexity**

For production Digital FTEs handling multi-faceted problems (code review, audit compliance, customer support), the specialization and parallelism outweigh coordination overhead.

## Code Example: A Complete Subagent Architecture

Here's a production pattern for a code review Digital FTE:

```python
from claude_agent_sdk import AgentDefinition, ClaudeAgentOptions, query

# Define specialized subagents
review_options = ClaudeAgentOptions(
    allowed_tools=["Read", "Grep", "Bash", "Task"],
    agents={
        "security-reviewer": AgentDefinition(
            description="Security specialist identifying vulnerabilities",
            prompt="""You are a security expert. Review code for:
1. Authentication/Authorization Flaws
2. Injection Attacks (SQL, Command, Template)
3. Sensitive Data Exposure (credentials, PII)
4. Broken Access Control
Report findings as: SEVERITY | ISSUE | LOCATION | RECOMMENDATION""",
            tools=["Read", "Grep", "Glob"],
            model="sonnet"
        ),

        "performance-reviewer": AgentDefinition(
            description="Performance specialist identifying bottlenecks",
            prompt="""You are a performance engineer. Review for:
1. Algorithmic Complexity (O(n²) or worse)
2. N+1 Query Patterns (database)
3. Memory Leaks (unbounded allocations)
4. Inefficient Caching
Report findings as: IMPACT | PATTERN | LOCATION | OPTIMIZATION""",
            tools=["Read", "Bash", "Glob"],
            model="haiku"
        ),

        "test-coverage-analyzer": AgentDefinition(
            description="Testing specialist evaluating coverage",
            prompt="""You are a testing expert. Evaluate:
1. Test Coverage % for new code (target 80%+)
2. Test Quality (meaningful assertions)
3. Edge Cases (error paths, boundary conditions)
4. Flaky Test Patterns
Report findings as: METRIC | FINDING | COVERAGE % | RECOMMENDATIONS""",
            tools=["Read", "Bash", "Glob"],
            model="haiku"
        ),
    }
)

# Orchestrator function
async def review_pull_request(repo_path: str, pr_branch: str):
    """Comprehensive code review using parallel specialist agents"""

    orchestration_prompt = f"""Perform a comprehensive code review for PR on {pr_branch} in {repo_path}.

Execute in parallel (invoke all three subagents simultaneously):

1. Security Review: Invoke "security-reviewer" subagent
   Prompt: "Review {repo_path} on branch {pr_branch} for security vulnerabilities"

2. Performance Review: Invoke "performance-reviewer" subagent
   Prompt: "Analyze {repo_path} on branch {pr_branch} for performance problems"

3. Test Coverage: Invoke "test-coverage-analyzer" subagent
   Prompt: "Check test coverage for {repo_path} on branch {pr_branch}"

Collect all results, then synthesize into unified review:

## PR Review Report

### Security Assessment
[Insert security findings]

### Performance Assessment
[Insert performance findings]

### Test Coverage Assessment
[Insert coverage findings]

### Overall Recommendation
Status: [APPROVED / NEEDS CHANGES / BLOCKED]
Reasoning: [Synthesis of all three perspectives]"""

    async for message in query(prompt=orchestration_prompt, options=review_options):
        if hasattr(message, "result"):
            print(message.result)

# Output example:
# ## PR Review Report
#
# ### Security Assessment
# HIGH | SQL Injection Risk | database.py:45 | Use parameterized queries
# MEDIUM | Hardcoded API Key | config.py:10 | Move to environment variable
# ...
```

**Key patterns demonstrated:**

1. **Specialization through prompts**: Each subagent gets domain-specific reasoning framework
2. **Tool restriction**: Security reviewer gets read-only tools; performance reviewer gets Bash for profiling
3. **Model selection**: Sonnet for complex reasoning (security), Haiku for straightforward analysis
4. **Parallel invocation**: Task tool invokes all three simultaneously
5. **Synthesis**: Orchestrator combines results into unified output

## Try With AI

**Setup**: You have a Python function with performance issues and incomplete test coverage. You want to review it through multiple specialized perspectives simultaneously.

### Prompt 1: Design Your Subagent Architecture

Ask AI to design the subagent configuration:

```
I need to review code changes for:
- Security issues (authentication, injection, credential leaks)
- Performance problems (N+1 queries, memory inefficiency)
- Testing gaps (coverage below 80%)
- Documentation accuracy (API docs match implementation)

Design an AgentDefinition for each specialist. For each:
1. What should the prompt instruct this agent to focus on?
2. What tools does this agent actually need?
3. Should we use Sonnet or Haiku for cost/performance tradeoffs?

I want these specialists to work in PARALLEL, so be specific about what each agent should output so I can synthesize results easily.
```

**What you're learning**: You're collaborating with AI to make architectural decisions about specialization, tool restrictions, and model selection. AI suggests patterns you might not have considered (maybe: "Testing specialist doesn't need Bash—test results from the repo are sufficient").

### Prompt 2: Implement the Orchestration Pattern

Once you have the subagent definitions, ask AI to implement the orchestrator:

```
Given these subagent definitions:
- [Paste your security-reviewer, performance-reviewer, testing-analyzer definitions]

Write the main orchestrator function that:
1. Invokes all three subagents in parallel using the Task tool
2. Collects their results
3. Synthesizes findings into a unified review report

The output should be structured as:
## Security Findings
## Performance Issues
## Testing Assessment
## Recommendation

Include specific patterns for how to request consistent output from each subagent.
```

**What you're learning**: You're seeing how to compose parallel agents into a coordinated workflow. AI refines your orchestration logic and helps structure subagent prompts so results are easy to synthesize.

### Prompt 3: Evaluate Your Architecture

After implementing, ask AI to critique your design:

```
Here's my subagent architecture:
[Paste your complete AgentDefinition setup and orchestration code]

Evaluate this design:
1. Did I restrict tools appropriately for each specialist?
2. Are there tasks I decomposed that should stay together?
3. Are there subagents I created that add overhead instead of value?
4. If I needed to add a "documentation reviewer" specialist, how would that change the architecture?

What would you change to make this more production-ready for a Digital FTE?
```

**What you're learning**: You're moving beyond implementation to architectural reasoning. AI helps you think through specialization tradeoffs, composition patterns, and scalability. This is the reasoning framework for designing orchestrated agents.

**Important**: Keep subagent definitions simple and focused. The temptation is to create many specialized subagents ("security-reviewer", "performance-reviewer", "documentation-reviewer", "accessibility-reviewer"...). Start with 2-3 specialists. More specialists mean more coordination complexity. More complexity means harder to debug when something goes wrong. Earn additional specialists through experience with what works.

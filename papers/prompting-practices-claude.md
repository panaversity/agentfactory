# Claude 4.x Prompting Best Practices

**Source**: https://platform.claude.com/docs/en/build-with-claude/prompt-engineering/claude-4-best-practices

## 1) Long-Horizon Reasoning & State Tracking

**Core Capability**: Claude 4.5 excels at "long-horizon reasoning tasks with exceptional state tracking capabilities," maintaining context across extended sessions through incremental progress.

**Key Strategies**:
- Use structured formats (JSON) for tracked data like test results
- Employ unstructured text for progress notes
- Leverage git for state checkpoints across sessions
- Emphasize incremental work over attempting everything simultaneously

**Context Awareness Prompt**:
```
Your context window will be automatically compacted as it approaches its limit,
allowing you to continue working indefinitely from where you left off. Therefore,
do not stop tasks early due to token budget concerns. As you approach your token
budget limit, save your current progress and state to memory before the context
window refreshes.
```

## 2) Agentic Workflows

**Multi-Context Window Framework**:
- First window: establish framework (write tests, create setup scripts)
- Subsequent windows: iterate using a structured todo-list
- Create tests in `tests.json` format before implementation begins
- Build setup scripts (`init.sh`) to prevent repeated work

**Starting Fresh Strategy**:
When clearing context windows, prompt Claude to rediscover state:
```
Call pwd; you can only read and write files in this directory.
Review progress.txt, tests.json, and the git logs.
Manually run through a fundamental integration test before moving on.
```

## 3) Tool Use Patterns

**Explicit Action Instructions**: Claude requires specific direction to implement rather than suggest.

**Less effective**: "Can you suggest changes to improve this function?"

**More effective**: "Change this function to improve its performance."

**Proactive Action Prompt**:
```xml
<default_to_action>
By default, implement changes rather than only suggesting them. If the user's
intent is unclear, infer the most useful likely action and proceed, using tools
to discover any missing details instead of guessing.
</default_to_action>
```

## 4) Default to Action Guidance

**Conservative Alternative**:
```xml
<do_not_act_before_instructions>
Do not jump into implementation or change files unless clearly instructed. When
intent is ambiguous, default to providing information, research, and
recommendations rather than taking action. Only proceed with edits when
explicitly requested.
</do_not_act_before_instructions>
```

**Parallel Tool Calling**:
```xml
<use_parallel_tool_calls>
If you intend to call multiple tools with no dependencies, make all independent
calls in parallel. Prioritize simultaneous tool calls whenever possible to
increase speed. Never use placeholders or guess missing parameters.
</use_parallel_tool_calls>
```

## 5) Investigation Before Acting

**Code Exploration Requirement**:
```
ALWAYS read and understand relevant files before proposing code edits. Do not
speculate about code you have not inspected. If the user references a specific
file/path, you MUST open and inspect it before explaining or proposing fixes.
```

**Hallucination Minimization**:
```xml
<investigate_before_answering>
Never speculate about code you have not opened. If the user references a
specific file, you MUST read the file before answering. Make sure to investigate
and read relevant files BEFORE answering questions about the codebase.
</investigate_before_answering>
```

---

## Additional High-Impact Practices

**Be Explicit**: "Create an analytics dashboard. Include as many relevant features and interactions as possible. Go beyond the basics to create a fully-featured implementation."

**Add Context for Motivation**: Explain *why* behavior matters; Claude generalizes from explanations.

**Encourage Complete Context Usage**:
```
This is a very long task, so plan your work clearly. Spend your entire output
context working on the taskjust ensure you don't run out with uncommitted work.
```

---

## XML Tags Used in /sp.orchestrate v4.2

Based on these best practices, the orchestrator uses:

| Tag | Purpose |
|-----|---------|
| `<default_to_action>` | Implement rather than suggest |
| `<investigate_before_acting>` | Read files before editing |
| `<use_parallel_tool_calls>` | Maximize parallel tool usage |
| `<skill_and_tool_usage>` | Skills usable in ALL phases |
| `<sdd_workflow_gates>` | Approval gates between phases |
| `<agent_discovery>` | Dynamic agent/skill discovery |
| `<orchestration_insight>` | Distributed systems thinking |
| `<approval_gate>` | Explicit blocking points |
| `<enforcement_check>` | Self-monitoring checkpoints |
| `<phr_recording_protocol>` | Mandatory PHR recording |
| `<recovery_protocol>` | Violation recovery guidance |

---
sidebar_position: 1
title: "The Reliability Mindset"
description: "Learn evaluation-first development with Google ADK - write tests before agents"
keywords: [google adk, agent evaluation, adk eval, evaluation-first, reliable agents]
chapter: 35
lesson: 1
duration_minutes: 45

# HIDDEN SKILLS METADATA
skills:
  - name: "Evaluation-First Mindset"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why writing eval cases before agent code leads to more reliable agents"

  - name: "JSON Eval Case Anatomy"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can identify the components of an ADK eval case file (query, expected_tool_use, reference)"

  - name: "TaskManager Domain Understanding"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can map TaskManager operations to eval case expectations"

learning_objectives:
  - objective: "Explain why evaluation-first development leads to more reliable agents"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student articulates the difference between demo agents and production agents"

  - objective: "Identify the structure of ADK eval case JSON files"
    proficiency_level: "B1"
    bloom_level: "Remember"
    assessment_method: "Student correctly labels eval case components (query, expected_tool_use, reference)"

  - objective: "Write valid eval cases for TaskManager operations"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates 3 eval cases manually that define expected tool calls"

cognitive_load:
  new_concepts: 2
  assessment: "2 concepts (evaluation-first mindset, JSON eval case anatomy) within B1 limit (7-10 concepts) - Low load for foundation lesson"

differentiation:
  extension_for_advanced: "Write additional eval cases testing edge cases (empty task list, duplicate task names)"
  remedial_for_struggling: "Focus on one eval case pattern; use template with placeholders to fill in"
---

# The Reliability Mindset

You just launched an AI agent. In demos, it's flawless. Users love it. Then you deploy to production.

Within hours, the agent starts behaving unpredictably. It completes tasks that should be deleted. It misses edge cases. It works in some contexts and fails in others. You spend days debugging. You rewrite the agent three times. Users lose trust.

What went wrong? You built a demo agent, not a production agent. The difference isn't features—**it's reliability engineering**.

**This is where evaluation-first development changes everything.**

## The Demo vs. Production Gap

Think about the last software system you used that felt truly reliable. It wasn't reliable because the developers were brilliant. It was reliable because the developers had systematic ways to catch failures **before users encountered them**.

Most tutorials teach agent development like this:

1. Write agent code
2. Run it in terminal, ask it questions
3. If it works → Ship it
4. If broken → Debug and repeat

This is **demo development**. It's optimized for "does this look good in controlled conditions?" not "will this work reliably in production?"

Production-grade agents need testing first. But here's the twist: agent testing is different from traditional code testing.

Traditional code testing validates:
- Does function return expected value?
- Are outputs formatted correctly?
- Do edge cases fail gracefully?

Agent testing validates something harder:
- Does the agent understand what to do?
- Does it call the right tools?
- Does it handle ambiguous inputs gracefully?
- Does it recognize when it's confused?

**ADK's solution is `adk eval`—evaluation cases you write BEFORE building the agent.**

## Why Tests Come First (Not Last)

In traditional development, tests are written last. They're quality assurance—a gate you pass before shipping.

In agent development, tests are **design documents**. They define the contract: "Here's what this agent MUST do."

Writing tests first forces clarity:

- **You define success explicitly**: Not "the agent should handle task completion well" but "when user says 'Mark task 3 done', agent must call complete_task with id=3"
- **You catch requirements gaps early**: Writing the test reveals missing details before you write agent code
- **You have measurable progress**: As you build the agent, tests pass/fail, showing exactly what works and what doesn't
- **You prevent scope creep**: Tests define the boundary; anything not in tests is "not required"

This is evaluation-first development. **Write the specification as executable test cases. Then build the agent to pass those tests.**

## ADK Eval Cases: The Specification Format

An ADK eval case is a JSON file that specifies:
- What a user says (query)
- What tool calls should happen (expected_tool_use)
- What response pattern should emerge (reference)

Here's the simplest example:

```json
{
  "eval_set_id": "taskmanager_basics",
  "eval_cases": [
    {
      "eval_id": "add_task_basic",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Add a task called 'Buy groceries'"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Task 'Buy groceries' added successfully"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "add_task", "args": {"title": "Buy groceries"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "task_manager",
        "user_id": "test_user"
      }
    }
  ]
}
```

**Output:**

```
Eval case "add_task_basic" passes when:
- User input: "Add a task called 'Buy groceries'"
- Agent calls: add_task tool with title="Buy groceries"
- Agent responds: "Task 'Buy groceries' added successfully"
```

Let's break down each component:

### Component 1: eval_id

A unique identifier for this test case. Use descriptive names:
- `add_task_basic` — adding a simple task
- `add_task_with_description` — adding task with more details
- `delete_task_missing` — delete operation on non-existent task

**Purpose**: When tests fail, you know exactly which scenario broke.

### Component 2: conversation

An array representing the entire interaction. For TaskManager, it's usually a single turn (user asks → agent responds). But ADK supports multi-turn conversations for complex agents.

Each turn contains:
- `user_content`: What the user said
- `final_response`: What the agent should respond with
- `intermediate_data.tool_uses`: What tools the agent should call

### Component 3: tool_uses

**The most critical part.** This is where you specify what tool calls MUST happen.

```json
"tool_uses": [
  {"name": "add_task", "args": {"title": "Buy groceries"}}
]
```

This means: "When user says this, the agent MUST call the add_task tool with these arguments."

**Why this matters**: This is the contract. If your agent calls the wrong tool, or calls it with wrong arguments, the test fails. No excuses.

### Component 4: reference

The expected response text. This is looser than tool_uses (the agent might phrase the response differently), but it validates that the agent explains what happened.

**Why both tool_uses AND reference?**:
- `tool_uses` validates the agent understood the intent
- `reference` validates the agent can explain the outcome

Together they prevent silent failures (agent calls the tool but doesn't explain what happened).

## Writing Your First Eval Cases

Let's design three eval cases for TaskManager. We're NOT building the agent yet. We're defining what "correct" means.

### Eval Case 1: Add a Task

User request: "Add a task called 'Buy groceries'"

What should happen?
- The agent understands "add task"
- The agent extracts the title "Buy groceries"
- The agent calls the `add_task` tool
- The agent confirms the action

```json
{
  "eval_id": "add_task_simple",
  "conversation": [
    {
      "user_content": {
        "parts": [{"text": "Add a task called 'Buy groceries'"}],
        "role": "user"
      },
      "final_response": {
        "parts": [{"text": "Task 'Buy groceries' added successfully"}],
        "role": "model"
      },
      "intermediate_data": {
        "tool_uses": [
          {"name": "add_task", "args": {"title": "Buy groceries"}}
        ]
      }
    }
  ],
  "session_input": {
    "app_name": "task_manager",
    "user_id": "test_user"
  }
}
```

**Output:**

Test passes if:
- Agent called `add_task` with `title="Buy groceries"`
- Agent responded confirming the task was added

### Eval Case 2: List Tasks

User request: "Show my tasks"

What should happen?
- The agent understands "show/list"
- The agent calls `list_tasks` tool
- The agent displays the task list

```json
{
  "eval_id": "list_tasks_default",
  "conversation": [
    {
      "user_content": {
        "parts": [{"text": "Show my tasks"}],
        "role": "user"
      },
      "final_response": {
        "parts": [{"text": "Here are your tasks:"}],
        "role": "model"
      },
      "intermediate_data": {
        "tool_uses": [
          {"name": "list_tasks", "args": {}}
        ]
      }
    }
  ],
  "session_input": {
    "app_name": "task_manager",
    "user_id": "test_user"
  }
}
```

**Output:**

Test passes if:
- Agent called `list_tasks` tool
- Agent acknowledged the request in its response

### Eval Case 3: Complete a Task

User request: "Mark the first task as done"

What should happen?
- The agent understands "mark done / complete"
- The agent identifies which task (first task = task_id: 1)
- The agent calls `complete_task` with the correct ID
- The agent confirms

```json
{
  "eval_id": "complete_task_by_position",
  "conversation": [
    {
      "user_content": {
        "parts": [{"text": "Mark the first task as done"}],
        "role": "user"
      },
      "final_response": {
        "parts": [{"text": "Task marked as completed"}],
        "role": "model"
      },
      "intermediate_data": {
        "tool_uses": [
          {"name": "complete_task", "args": {"task_id": 1}}
        ]
      }
    }
  ],
  "session_input": {
    "app_name": "task_manager",
    "user_id": "test_user"
  }
}
```

**Output:**

Test passes if:
- Agent called `complete_task` with `task_id=1`
- Agent confirmed the task was completed

## Why This Matters: The Reliability Contract

Here's what just happened: You wrote specifications as executable tests. No agent code. No implementation. Just clarity about what "correct" means.

Now you have three things:

1. **A contract**: "Agent must do X, Y, Z"
2. **A measurement**: "Eval case passes or fails"
3. **A guide**: Agent developers can read these tests and understand expectations

Compare this to vague requirements like "Agent should handle task management well." That's a feeling. These test cases are requirements.

When you build the agent, you'll:
1. Run `adk eval` and see all three tests fail (because agent doesn't exist)
2. Write agent code
3. Run `adk eval` again
4. Each test passes as you implement that capability
5. When all tests pass, you're done

**Reliable code doesn't come from hope. It comes from specifications that define "correct" and validation that proves it.**

## The Reliability Mindset in Practice

A developer using demo development thinks: "I'll try this approach and see if it works."

A developer using evaluation-first development thinks: "I've defined what success looks like. Now I'll build to pass those tests."

The second developer will:
- Ship agents with fewer bugs
- Debug faster (tests show exactly what failed)
- Handle edge cases (if not in tests, they write tests first)
- Never ship "worked in my demo" failures

This is the difference between fragile and reliable agents.

**The lesson**: Before you write a single line of agent code, write the eval cases. Define the contract. Measure success. Build with confidence.

## Try With AI

Use your AI companion to deepen your understanding of evaluation-first thinking.

### Prompt 1: Challenge Your Mental Model

```
I just learned about writing eval cases BEFORE building agents.
It feels slower—like writing tests before code. But the lesson
says it's actually faster and more reliable. Help me understand:

Why is evaluation-first development faster than code-first
development? What specifically makes it faster? Aren't I writing
twice as much (specs + code)?
```

**What you're learning**: Critical thinking about software methodology. Understanding tradeoffs between specification clarity and development speed—a core skill in spec-driven development.

### Prompt 2: Design Eval Cases for Your Domain

```
I work with [your domain: customer service / data analysis / code review / sales].
Help me design 3 eval cases for an AI agent that would be useful
in my domain. Ask me clarifying questions about:

1. What would the user ask the agent?
2. What tool should the agent call?
3. What's the expected outcome the user would see?

Then help me write those 3 eval cases in JSON format like the TaskManager example.
```

**What you're learning**: Pattern transfer—taking the eval case structure from TaskManager and applying it to your specific domain. This is how specification-first development becomes your default approach.

### Prompt 3: Find the Gap

```
Here's an eval case I wrote for my agent:

[paste a simple eval case]

I built the agent to pass this test, and it does. But in production,
users reported that [describe a failure scenario]. The agent doesn't
handle [edge case].

What eval case should I have written to catch this failure? What was
missing from my original specification?
```

**What you're learning**: The power of specification completeness. When production failures happen, the question isn't "was my code wrong?" but "what did my spec miss?" This is how reliability engineers think.

When building AI agents, reliability comes from clear specifications defined as executable tests—eval cases you run before the agent exists. As you proceed through this chapter, every agent you build will follow this pattern: specification (as eval cases) first, implementation second, validation continuous.

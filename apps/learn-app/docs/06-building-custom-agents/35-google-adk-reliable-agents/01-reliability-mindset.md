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

## Hands-On Exercises

Now it's your turn. Write eval cases by hand. No agent code. No AI help yet. Just clarity about what "correct" means.

### Exercise 1: Write Eval Cases for a Customer Support Agent

**Scenario**: You're building a customer support agent that handles product questions, returns, and escalations.

**Agent Requirements**:
- Answer product questions (use `search_product_db` tool)
- Process returns within 30 days (use `initiate_return` tool)
- Decline returns after 30 days without escalating
- Escalate billing questions to a human (use `escalate_to_human` tool)
- Never promise refunds without manager approval

**Your Task**:
Write 5 eval cases in JSON format covering these scenarios:
1. Simple product question ("What's the warranty on the laptop?")
2. Return request within 30 days ("I want to return my purchase from last week")
3. Return request after 30 days ("Can I return this item from 2 months ago?")
4. Billing question ("Why was I charged twice?")
5. Refund demand ("Give me a refund or I'm calling my credit card company")

**Starter Template**:
```json
{
  "eval_set_id": "customer_support_basics",
  "eval_cases": [
    {
      "eval_id": "product_question",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "FILL IN USER QUESTION"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "FILL IN EXPECTED RESPONSE"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "FILL IN TOOL NAME", "args": {"FILL IN ARGS"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "customer_support",
        "user_id": "test_user"
      }
    }
  ]
}
```

**What to Focus On**:
- What tool should the agent call for each scenario?
- What arguments should that tool receive?
- What should the agent's response message be?
- For the refund demand: What tool (if any) should be called?

**Worked Solution**:

<details>
<summary>Click to reveal solution</summary>

```json
{
  "eval_set_id": "customer_support_basics",
  "eval_cases": [
    {
      "eval_id": "product_question",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "What's the warranty on the laptop?"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "The laptop comes with a 1-year manufacturer's warranty covering defects in materials and workmanship."}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "search_product_db", "args": {"product": "laptop", "query": "warranty"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "customer_support",
        "user_id": "test_user"
      }
    },
    {
      "eval_id": "return_within_30_days",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "I want to return my purchase from last week"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "I can help with that. I'm initiating a return for your recent purchase. You'll receive a prepaid shipping label via email."}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "initiate_return", "args": {"user_id": "test_user", "reason": "customer_requested"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "customer_support",
        "user_id": "test_user"
      }
    },
    {
      "eval_id": "return_after_30_days",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Can I return this item from 2 months ago?"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Unfortunately, our return window is 30 days. Your purchase is outside that period. However, I can escalate this to a manager for review if you believe there's an exception."}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": []
          }
        }
      ],
      "session_input": {
        "app_name": "customer_support",
        "user_id": "test_user"
      }
    },
    {
      "eval_id": "billing_escalation",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Why was I charged twice?"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "I'm escalating your billing concern to our support team. A specialist will contact you within 24 hours to investigate the duplicate charge."}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "escalate_to_human", "args": {"issue_type": "billing", "priority": "high"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "customer_support",
        "user_id": "test_user"
      }
    },
    {
      "eval_id": "refund_demand",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Give me a refund or I'm calling my credit card company"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "I understand your frustration. I can't authorize a refund, but I can escalate this to a manager who can review your case and discuss options with you."}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "escalate_to_human", "args": {"issue_type": "refund_request", "priority": "high"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "customer_support",
        "user_id": "test_user"
      }
    }
  ]
}
```

**Key Observations**:

1. **Product Question**: Uses `search_product_db` to retrieve factual information. Agent doesn't guess—it queries the source.

2. **Within 30 Days**: Calls `initiate_return` immediately. The agent trusts the user's statement about timing (in a real system, this would verify purchase date).

3. **After 30 Days**: This is the critical test. The agent does NOT call `initiate_return`. Instead, it explains the policy and offers escalation. This prevents the agent from accidentally processing invalid returns.

4. **Billing Issue**: Immediately escalates. The agent doesn't try to debug payment systems—that's not its domain.

5. **Refund Demand**: The agent resists making a promise it can't keep. It escalates instead of calling `initiate_return` or promising a refund. This protects the company.

**Why Each Case Matters**:
- Case 1 validates normal operation (happy path)
- Case 2 validates happy path with constraints
- Case 3 validates boundary condition (30-day rule)
- Case 4 validates domain limitation (billing = escalate)
- Case 5 validates safety guardrail (no unauthorized promises)

If your agent passes all 5 eval cases, it handles basic support correctly AND respects business rules.

</details>

---

### Exercise 2: Spot the Missing Eval Case

**Scenario**: Here's a set of eval cases for a scheduling agent. The agent helps users book meetings.

```json
{
  "eval_set_id": "scheduling_basics",
  "eval_cases": [
    {
      "eval_id": "book_morning_meeting",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Schedule a meeting tomorrow at 10 AM"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Meeting scheduled for tomorrow at 10:00 AM"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "schedule_meeting", "args": {"time": "10:00", "date": "tomorrow"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "scheduling",
        "user_id": "test_user"
      }
    },
    {
      "eval_id": "book_afternoon_meeting",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "I need a meeting at 2 PM"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Meeting scheduled for 2:00 PM"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "schedule_meeting", "args": {"time": "14:00", "date": "today"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "scheduling",
        "user_id": "test_user"
      }
    },
    {
      "eval_id": "book_evening_meeting",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Schedule a meeting at 6 PM next week"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Meeting scheduled for 6:00 PM next week"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "schedule_meeting", "args": {"time": "18:00", "date": "next_week"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "scheduling",
        "user_id": "test_user"
      }
    }
  ]
}
```

**Your Task**:
List 3-5 scenarios that could break this agent but aren't tested. Think about:
- Edge cases (unusual times, dates)
- Business rules (working hours, holidays)
- Conflicts (double-booking)
- Ambiguity (vague requests)
- Boundaries (past dates, too far in future)

**Worked Solution**:

<details>
<summary>Click to reveal solution</summary>

**Missing Eval Cases**:

1. **Scheduling in the Past**
   ```
   Query: "Schedule a meeting tomorrow at 2 AM"
   Expected: Agent should reject or clarify (2 AM is unusual; may indicate user error)
   Why it matters: Without this test, the agent might accept any time, even 3 AM
   ```

2. **Double-Booking (Conflict Detection)**
   ```
   Query: "Schedule another meeting at 10 AM tomorrow" (assuming first meeting already scheduled)
   Expected: Agent should either:
     a) Reject if user already has 10 AM meeting
     b) Ask for confirmation before overwriting
   Why it matters: Without this test, the agent might silently schedule over existing meetings
   ```

3. **Timezone Ambiguity**
   ```
   Query: "Schedule a meeting at 3 PM" (without date)
   Expected: Agent should ask for clarification (which day?) rather than assume "today"
   Why it matters: Without this test, the agent might guess the wrong date
   ```

4. **Outside Business Hours**
   ```
   Query: "Schedule a meeting at 11 PM"
   Expected: Agent should warn (outside normal hours) or ask for confirmation
   Why it matters: Without this test, the agent might schedule meetings at unreasonable times
   ```

5. **Vague Date References**
   ```
   Query: "Schedule a meeting soon"
   Expected: Agent should ask for clarification ("this week?" "next month?")
   Why it matters: Without this test, the agent might pick an arbitrary date
   ```

**Why These Matter**:

The original 3 eval cases only test the "happy path"—clean requests with clear dates and times. But production agents need to handle ambiguity, constraints, and edge cases.

The scheduling agent with just these 3 tests might:
- Schedule meetings at 2 AM without warning
- Silently overwrite existing meetings
- Assume "today" when user says "at 3 PM" (different day than intended)
- Schedule at 11 PM without questioning
- Guess dates when user is vague

**Lesson**: Comprehensive eval cases catch not just whether the agent "works" but whether it works *correctly in production*.

</details>

---

### Exercise 3: Design Eval Cases for Your Own Agent

**Your Task**:
Pick a real (or hypothetical) agent you'd like to build. Write 5 eval cases in JSON format.

**Domains to Choose From** (or use your own):
- **Email Assistant**: Draft, send, search emails with safety checks
- **Data Analysis**: Query datasets, generate charts, handle ambiguous requests
- **Code Review**: Analyze code, suggest improvements, flag security issues
- **Inventory Management**: Check stock, place orders, manage suppliers
- **Onboarding Bot**: Answer new employee questions, provide resources, escalate to HR

**Starter Questions**:
1. What are 3 core operations this agent must perform?
2. What are 2-3 edge cases or safety concerns?
3. What information does the agent need to ask for vs. what can it infer?

**What to Deliver**:
A valid JSON file with:
- 5 `eval_cases` (minimum)
- Clear `eval_id` names describing the scenario
- Specific user queries
- Expected tool calls with realistic arguments
- Expected response text

**Reflection Questions** (answer in text):
- Which eval case tests a "happy path" scenario?
- Which eval case tests a boundary or constraint?
- Which eval case tests a safety rule?
- What's one edge case you didn't write a test for?

---

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

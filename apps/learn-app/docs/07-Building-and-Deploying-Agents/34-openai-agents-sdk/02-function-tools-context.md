---
sidebar_position: 2
title: "Function Tools & Context Objects"
description: "Create function tools with the @function_tool decorator and design context objects for persistent state across tool calls."
keywords: [function_tool, decorator, type_hints, Pydantic, RunContextWrapper, context_mutation, state_persistence]
chapter: 34
lesson: 2
duration_minutes: 70

skills:
  - name: "Function Tool Definition"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can create @function_tool with type hints and docstrings"

  - name: "Context Object Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Data Management"
    measurable_at_this_level: "Student can create Pydantic context model and pass to Runner"

  - name: "State Persistence Patterns"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can mutate context within tool functions and observe persistence"

learning_objectives:
  - objective: "Create function tools with @function_tool decorator and proper type hints"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Tool executes and agent uses it correctly"

  - objective: "Design Pydantic context objects for agent state management"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Context model validates input and persists state across calls"

  - objective: "Access and mutate context within tool functions using RunContextWrapper"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "State modifications persist across multiple sequential tool calls"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (@function_tool, type hints, docstrings, Pydantic BaseModel, RunContextWrapper, context.context property, mutations) within B1 limit ✓"

differentiation:
  extension_for_advanced: "Design complex context model for multi-agent workflow with nested state"
  remedial_for_struggling: "Start with simple tool without context; add context after tool works independently"
---

# Function Tools & Context Objects

Think of an agent without tools like a person who can only talk. They can reason, discuss, and advise—but they cannot *do* anything. They cannot check weather, book flights, update databases, or accomplish real work.

Tools are how agents take action.

But tools alone aren't enough. A cashier can process transactions, but without a register that *remembers* the running total, each transaction happens in isolation. Context is the register. It holds state that persists across tool calls, so the agent can build on its own previous actions.

In this lesson, you'll learn to create both: function tools that agents can call, and context objects that remember what has happened so far.

## Understanding Function Tools

A function tool is a Python function that an agent can invoke. It's marked with the `@function_tool` decorator, which tells the OpenAI Agents SDK to expose it to the agent.

Let's start simple:

```python
from agents import Agent, Runner, function_tool

@function_tool
def get_weather(city: str) -> str:
    """Get the current weather for a city.

    Args:
        city: The name of the city.
    """
    # In a real system, this would call a weather API
    return f"The weather in {city} is sunny, 72°F."

agent = Agent(
    name="WeatherBot",
    instructions="Help users check the weather. Use the get_weather tool to answer questions about weather in different cities.",
    tools=[get_weather],
)

result = Runner.run_sync(agent, "What's the weather in Tokyo?")
print(result.final_output)
```

**Output:**
```
The weather in Tokyo is sunny, 72°F.
```

Notice three things:

1. **Type hints matter**: `city: str` and `-> str` tell the SDK what arguments the agent should pass and what it will receive back.
2. **Docstrings are required**: The docstring becomes part of the agent's tool definition. A clear docstring helps the agent understand when and how to use the tool.
3. **Tools are just functions**: No special SDK knowledge needed—if you can write a Python function, you can create a tool.

The agent reads the tool definition, decides it needs to check the weather, calls `get_weather("Tokyo")`, receives the string back, and incorporates that into its response.

## Context Objects: Carrying State Forward

Now imagine a more complex scenario. You're building a task management agent. Users should be able to:
- Set a current project
- Add tasks to that project
- See how many tasks are in the project

Without context, each tool call would happen in isolation. The agent couldn't remember which project the user was working on, because there would be no place to store that information across calls.

Context solves this. You define a context object (typically using Pydantic) that holds whatever state you need:

```python
from pydantic import BaseModel
from agents import Agent, Runner, function_tool, RunContextWrapper

class TaskManagerContext(BaseModel):
    user_id: str | None = None
    current_project: str | None = None
    tasks_added: int = 0

@function_tool
def set_project(context: RunContextWrapper[TaskManagerContext], project_name: str) -> str:
    """Set the current project for adding tasks.

    Args:
        project_name: The name of the project to switch to.
    """
    context.context.current_project = project_name
    return f"Now working on project: {project_name}"

@function_tool
def add_task(context: RunContextWrapper[TaskManagerContext], task_name: str) -> str:
    """Add a task to the current project.

    Args:
        task_name: The description of the task to add.
    """
    if context.context.current_project is None:
        return "Error: No project selected. Use set_project first."
    context.context.tasks_added += 1
    return f"Added task '{task_name}' to {context.context.current_project}. Total tasks: {context.context.tasks_added}"

agent = Agent(
    name="TaskManager",
    instructions="""You help users manage their tasks and projects.
    When a user wants to switch projects, use set_project.
    When they want to add a task, use add_task.
    Always confirm what you've done.""",
    tools=[set_project, add_task],
)

# Create context and pass it to Runner
ctx = TaskManagerContext(user_id="user-123")
result = Runner.run_sync(
    agent,
    "Switch to 'Website Redesign' project and add two tasks: 'Update homepage' and 'Fix footer'",
    context=ctx,
)
print(result.final_output)
print(f"\nContext after: {ctx}")
```

**Output:**
```
I've switched to the 'Website Redesign' project and added both tasks. You now have 2 tasks in this project.

Context after: TaskManagerContext(user_id='user-123', current_project='Website Redesign', tasks_added=2)
```

This is the key insight: the context object `ctx` is **mutated** during the agent run. When `set_project` executes, it modifies `context.context.current_project`. When `add_task` executes, it increments `context.context.tasks_added`. After the run completes, your code can inspect the modified context to see what happened.

## How Context Flow Works

Here's what happens behind the scenes:

```
1. You create a context object: ctx = TaskManagerContext(user_id="user-123")
   ↓
2. You pass it to Runner: Runner.run_sync(agent, "...", context=ctx)
   ↓
3. Agent receives user input and decides to call set_project("Website Redesign")
   ↓
4. The set_project function receives: RunContextWrapper[TaskManagerContext]
   ↓
5. Inside set_project, you access the context: context.context.current_project = "Website Redesign"
   ↓
6. The modification happens in-place on your original ctx object
   ↓
7. Agent later calls add_task("Update homepage")
   ↓
8. The add_task function can see current_project is set (persisted from step 5)
   ↓
9. After the run, your code sees ctx.current_project = "Website Redesign" and ctx.tasks_added = 2
```

The critical detail: `context.context` is the actual context object you passed in. Modifications are real, persistent, and visible to your code after the run completes.

## Real-World Example: Multi-Step Workflow

Here's a practical example from airline customer service. The context tracks passenger information that persists across multiple tool calls:

```python
from pydantic import BaseModel
from agents import Agent, Runner, function_tool, RunContextWrapper

class AirlineContext(BaseModel):
    passenger_name: str | None = None
    confirmation_number: str | None = None
    seat_number: str | None = None

@function_tool
def lookup_booking(context: RunContextWrapper[AirlineContext], confirmation_number: str) -> str:
    """Look up a booking by confirmation number."""
    context.context.confirmation_number = confirmation_number
    # In production, this would query a database
    context.context.passenger_name = "John Doe"
    return f"Found booking {confirmation_number} for John Doe. Current seat: 12A"

@function_tool
def change_seat(context: RunContextWrapper[AirlineContext], new_seat: str) -> str:
    """Change the seat for the current booking."""
    if context.context.confirmation_number is None:
        return "Error: No booking loaded. Look up a booking first."
    context.context.seat_number = new_seat
    return f"Seat changed to {new_seat} for booking {context.context.confirmation_number}"

agent = Agent(
    name="AirlineAgent",
    instructions="""You help customers manage their flight bookings.
    First, use lookup_booking with their confirmation number.
    Then, if they want to change seats, use change_seat.
    Always confirm the action.""",
    tools=[lookup_booking, change_seat],
)

ctx = AirlineContext()
result = Runner.run_sync(
    agent,
    "My confirmation is ABC123. I'd like to change my seat to 14F.",
    context=ctx,
)
print(result.final_output)
print(f"\nContext: confirmation={ctx.confirmation_number}, seat={ctx.seat_number}, name={ctx.passenger_name}")
```

**Output:**
```
I've looked up your booking and changed your seat to 14F. You're all set!

Context: confirmation=ABC123, seat=14F, name=John Doe
```

Notice that `lookup_booking` runs first (and sets `passenger_name`), then `change_seat` runs (and can trust that `confirmation_number` was set by the previous call). The context connects the two steps.

## Design Patterns for Context

### Pattern 1: Simple Accumulator

Track counts or totals:

```python
class CounterContext(BaseModel):
    click_count: int = 0

@function_tool
def click_button(context: RunContextWrapper[CounterContext]) -> str:
    context.context.click_count += 1
    return f"Button clicked. Total clicks: {context.context.click_count}"
```

### Pattern 2: State Machine

Track what mode you're in:

```python
class GameContext(BaseModel):
    game_state: str = "menu"  # "menu", "playing", "game_over"
    score: int = 0

@function_tool
def start_game(context: RunContextWrapper[GameContext]) -> str:
    context.context.game_state = "playing"
    return "Game started!"

@function_tool
def end_game(context: RunContextWrapper[GameContext]) -> str:
    if context.context.game_state != "playing":
        return "Error: Game is not running"
    context.context.game_state = "game_over"
    return f"Game over! Final score: {context.context.score}"
```

### Pattern 3: List Accumulation

Collect items over time:

```python
from typing import List

class ShoppingContext(BaseModel):
    items: List[str] = []
    total_price: float = 0.0

@function_tool
def add_to_cart(context: RunContextWrapper[ShoppingContext], item: str, price: float) -> str:
    context.context.items.append(item)
    context.context.total_price += price
    return f"Added {item}. Cart total: ${context.context.total_price:.2f}"
```

## Common Mistakes

**Mistake 1: Forgetting to access `.context`**
```python
# WRONG
context.current_project = "Website"  # This won't work!

# RIGHT
context.context.current_project = "Website"  # Access via .context property
```

**Mistake 2: Creating context but not passing it**
```python
# WRONG
ctx = TaskManagerContext()
result = Runner.run_sync(agent, "...")  # Context is created but not passed!

# RIGHT
ctx = TaskManagerContext()
result = Runner.run_sync(agent, "...", context=ctx)  # Pass it explicitly
```

**Mistake 3: Expecting context to be sent to the LLM**
```python
# Context is NEVER sent to the LLM. If you need the agent to know something,
# put it in instructions or use a tool to fetch it.

# If you want the agent to use current state:
@function_tool
def get_status(context: RunContextWrapper[MyContext]) -> str:
    return f"Current project: {context.context.current_project}"

# Then the agent calls get_status to learn the state
```

## Type Safety with Context

Pydantic provides automatic validation. If your context expects a `str`, Pydantic will reject non-strings:

```python
class ValidatedContext(BaseModel):
    count: int  # Must be an int
    name: str   # Must be a string
    active: bool = True  # Default value, can be overridden

# This works:
ctx = ValidatedContext(count=5, name="Project A")

# This fails with a validation error:
ctx = ValidatedContext(count="five", name="Project A")  # count must be int!
```

Use this to catch mistakes early. If your tool tries to assign the wrong type to a context field, Pydantic will raise an error immediately.

## Summary: Tools and Context Together

Tools give agents the ability to do things. Context gives tools (and agents) the ability to remember what happened and build on it.

**Tools**:
- Decorated with `@function_tool`
- Have type hints (parameter types and return type)
- Have docstrings that explain their purpose
- Execute your code when the agent calls them

**Context**:
- Defined as a Pydantic `BaseModel`
- Passed to `Runner.run_sync()` via the `context=` parameter
- Accessed in tools via `RunContextWrapper[YourContextType]`
- Mutated in-place; changes persist for subsequent tool calls and are visible to your code afterward

When you combine them, you create agents that can take real actions and remember what they've done.

---

## Try With AI

### Prompt 1: Your First Tool

```
Create a function tool called calculate_sum that takes two integers (a and b)
and returns their sum. Add it to an Agent and test it with the input "What is 5 plus 3?"
```

**What you're learning:** The `@function_tool` pattern with type hints and how agents invoke tools to answer questions.

### Prompt 2: Context Model Design

```
Design a context model for a shopping cart system. It should have: user_id (string),
items (list of item names), and total_price (float). Create tools to add_item (takes item name and price)
and get_total (returns current total). Create an agent that can add items and report the total.
```

**What you're learning:** Pydantic model design and how context flows between tool calls in the same agent run.

### Prompt 3: State Persistence Verification

```
Run your shopping cart agent from Prompt 2 with this input: "Add a book for $15, then add a pen for $3,
then tell me the total." Print the context object after the run. Verify that total_price accumulated correctly—
it should be $18.00, not just $3.00 (the last item). This proves context persists across tool calls.
```

**What you're learning:** How context mutations persist across multiple sequential tool calls and how your code can inspect the final state after the agent run completes.

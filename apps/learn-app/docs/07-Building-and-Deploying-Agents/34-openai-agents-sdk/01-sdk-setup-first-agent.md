---
sidebar_position: 1
title: "SDK Setup & First Agent"
description: "Install OpenAI Agents SDK, configure your environment, and build your first working agent in Python."
keywords: ["OpenAI Agents SDK", "agent setup", "Python environment", "API key configuration", "first agent", "LiteLLM"]
chapter: 34
lesson: 1
duration_minutes: 55

# HIDDEN SKILLS METADATA
skills:
  - name: "SDK Installation and Environment Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Technical Problem-Solving"
    measurable_at_this_level: "Student can install SDK and configure API keys without errors"

  - name: "Agent Class Instantiation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can create Agent with name and instructions and run synchronously"

  - name: "Understanding Runner and Result Objects"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can use Runner.run_sync() and extract final_output from results"

  - name: "Alternative Model Providers with LiteLLM"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Resource Management"
    measurable_at_this_level: "Student can configure agents with non-OpenAI models using LitellmModel"

learning_objectives:
  - objective: "Install OpenAI Agents SDK and verify installation with hello world"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful agent creation and response printing"

  - objective: "Configure environment variables and API keys securely"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Agent can authenticate and respond without credential errors"

  - objective: "Understand the Agent → Runner → Result execution pattern"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student can explain the three-component pattern and use each correctly"

  - objective: "Use LiteLLM to run agents on free model alternatives"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Agent responds correctly using Claude or Gemini instead of GPT"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (SDK installation, API key management, Agent class, Runner.run_sync, result handling, LiteLLM integration) within B1 limit of 10 concepts - PASS"

differentiation:
  extension_for_advanced: "Explore runner configurations (max_turns, timeout), compare response quality across different LiteLLM models, investigate streaming with run_streamed"
  remedial_for_struggling: "Start with hello world agent first; skip LiteLLM until basic pattern is comfortable. Use synchronous examples only."

generated_by: content-implementer
source_spec: specs/047-ch34-openai-agents-sdk
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# SDK Setup & First Agent

Imagine you're building a customer support system that handles inquiries 24/7. You don't want to hire staff sitting idle between requests. Instead, you deploy a Digital FTE—a fully autonomous agent that reasons about customer problems, researches solutions, and responds in your company's voice. This is where you build that system.

In Chapter 33, you learned what agents are: systems that loop through reasoning, action, and observation to solve problems. Now you'll build one. The OpenAI Agents SDK is production-grade software for exactly this—minimal abstractions, maximum control. It's what powers custom agents at scale.

This lesson walks you through setup, your first working agent, and how to use it with free model alternatives. By the end, you'll have a functioning Digital FTE prototype that runs locally and can be deployed anywhere.

## Installation and Setup

Before you can build agents, you need the SDK installed and your environment configured.

### Step 1: Install the SDK

Open your terminal and install the openai-agents package:

```bash
pip install openai-agents
```

This installs the core SDK with essential dependencies. Verify the installation worked:

```bash
pip show openai-agents
```

**Output:**
```
Name: openai-agents
Version: 0.1.0
Location: /path/to/your/python/site-packages
```

### Step 2: Configure Your API Key

The SDK needs authentication to call OpenAI models. Set your API key as an environment variable:

```bash
export OPENAI_API_KEY=sk-proj-your-actual-key-here
```

On Windows, use:

```cmd
set OPENAI_API_KEY=sk-proj-your-actual-key-here
```

Or in Python, before running agents:

```python
import os
os.environ['OPENAI_API_KEY'] = 'sk-proj-your-actual-key-here'
```

Never hardcode API keys in your source code. Environment variables keep them secure.

### Verify Configuration

Create a simple verification script to confirm your environment is ready:

```python
import os
from agents import Agent, Runner

# Check if API key exists
api_key = os.getenv('OPENAI_API_KEY')
if not api_key:
    print("ERROR: OPENAI_API_KEY not set")
else:
    print(f"API Key found: {api_key[:20]}...")  # Show only first 20 chars
```

**Output:**
```
API Key found: sk-proj-abc123def456ghi...
```

## Your First Agent

Now let's build a working agent. This is the hello world of agent development.

### The Three-Part Pattern

Every agent follows this pattern:

1. **Agent** — Define what the agent is and how it should behave
2. **Runner** — Execute the agent with a user prompt
3. **Result** — Extract the agent's response

### Hello World Agent

Create a file called `first_agent.py`:

```python
from agents import Agent, Runner

# Step 1: Define the Agent
agent = Agent(
    name="Greeter",
    instructions="You are a friendly assistant. Greet the user warmly and ask what you can help with."
)

# Step 2: Run the Agent
result = Runner.run_sync(agent, "Hello!")

# Step 3: Extract and print the result
print(result.final_output)
```

**Expected Output:**
```
Hello! It's wonderful to meet you! I'm here to help. What can I assist you with today?
```

The exact wording varies because the LLM generates responses, but the structure is consistent:
- The agent greets warmly (matching instructions)
- The agent asks to help (matching instructions)
- The agent responds to your input (responding to "Hello!")

Run this script:

```bash
python first_agent.py
```

If you see an agent response, your SDK is working.

### Understanding the Pattern

Let's break down what happened:

**Agent Creation**:
```python
agent = Agent(
    name="Greeter",                    # Human-readable identifier
    instructions="You are a friendly..." # Controls agent behavior
)
```

The instructions field is critical. It tells the agent how to think. Better instructions produce better agents. Vague instructions create unreliable behavior.

**Execution**:
```python
result = Runner.run_sync(agent, "Hello!")
```

`Runner.run_sync()` means "run this agent synchronously" (wait for completion before continuing). This is the simplest execution pattern. The second argument is your input—the user's message to the agent.

**Result Extraction**:
```python
print(result.final_output)
```

`result.final_output` contains the agent's final response as a string. The result object has other properties (number of turns, tokens used, reasoning steps), but `final_output` is what you need for basic usage.

## Customizing Your Agent

Let's build a slightly more sophisticated example. Create a file called `custom_agent.py`:

```python
from agents import Agent, Runner

# Technical Support Agent
support_agent = Agent(
    name="TechSupport",
    instructions="""You are a helpful technical support agent for a software company.
Your job is to:
1. Listen to the customer's problem
2. Ask clarifying questions if needed
3. Provide clear, step-by-step solutions
4. Confirm the issue is resolved

Always be patient and professional. If you don't know something,
suggest the customer contact the support team."""
)

# Run with a customer problem
customer_issue = "I keep getting a 503 error when trying to upload files"
result = Runner.run_sync(support_agent, customer_issue)

print(result.final_output)
```

**Expected Output:**
```
I understand you're experiencing a 503 error during file uploads - that's frustrating!
A 503 error typically means the server is temporarily unavailable. Let me help you
troubleshoot this.

First, could you tell me:
1. What file size are you trying to upload?
2. Is this happening with all file types or specific ones?
3. When did this start - was it working before?

In the meantime, here are some quick things to try:
- Wait 5 minutes and try again (the server might recover)
- Clear your browser cache and try a different browser
- Try uploading from a different network (to rule out ISP issues)

Let me know the answers to those questions and we can dig deeper!
```

Notice how the agent:
- Acknowledges the problem
- Explains what 503 means
- Asks clarifying questions
- Provides immediate troubleshooting steps

The instructions directly shaped this behavior. Change the instructions, and the agent behaves differently.

## Using LiteLLM for Free Models

If you don't have an OpenAI API key or want to use free alternatives, LiteLLM lets you run agents on Claude, Gemini, and other models.

### Install LiteLLM Support

```bash
pip install "openai-agents[litellm]"
```

This adds LiteLLM as an optional dependency.

### Using Anthropic's Claude

First, get a Claude API key from https://console.anthropic.com. Then create `agent_with_claude.py`:

```python
from agents import Agent, Runner
from agents.extensions.models.litellm_model import LitellmModel

# Create agent using Claude instead of GPT
agent = Agent(
    name="ClaudeAgent",
    instructions="You are a thoughtful assistant powered by Claude. Provide helpful, accurate responses.",
    model=LitellmModel(
        model="anthropic/claude-3-5-sonnet-20241022",
        api_key="your-anthropic-api-key"
    )
)

# Run it the same way
result = Runner.run_sync(agent, "Explain the concept of a loop in programming")
print(result.final_output)
```

**Expected Output:**
```
A loop is a fundamental programming concept that lets you repeat a block of code
multiple times without writing it out repeatedly.

Think of it like an assembly line:
- You have a set of instructions (the loop body)
- You apply them to each item in a sequence
- Once you finish one item, you move to the next

Python has two main types of loops:

1. **while loops** - repeat until a condition becomes false:
   ```
   count = 0
   while count < 5:
       print(count)
       count += 1
   ```

2. **for loops** - iterate through items in a sequence:
   ```
   for number in range(5):
       print(number)
   ```

Loops prevent repetitive code and handle large datasets efficiently.
```

### Important Note: Disable Tracing for Non-OpenAI Models

When using LiteLLM with non-OpenAI models, disable tracing to avoid errors:

```python
from agents import Agent, Runner, set_tracing_disabled
from agents.extensions.models.litellm_model import LitellmModel

# Disable OpenAI tracing for non-OpenAI models
set_tracing_disabled(True)

agent = Agent(
    name="ClaudeAgent",
    instructions="You are a helpful assistant.",
    model=LitellmModel(model="anthropic/claude-3-5-sonnet-20241022")
)

result = Runner.run_sync(agent, "Hello!")
print(result.final_output)
```

The `set_tracing_disabled(True)` call prevents the SDK from trying to log to OpenAI's tracing system, which only works with OpenAI models.

## Common Setup Issues

**Issue: "ModuleNotFoundError: No module named 'agents'"**

Your Python environment doesn't have the SDK installed. Install it:
```bash
pip install openai-agents
```

**Issue: "Authentication failed" or "Invalid API key"**

Your API key isn't set correctly. Verify:
```bash
echo $OPENAI_API_KEY  # On macOS/Linux
echo %OPENAI_API_KEY%  # On Windows
```

If empty, set it again:
```bash
export OPENAI_API_KEY=sk-proj-your-key
```

**Issue: "LitellmModel not found"**

You haven't installed the litellm extra:
```bash
pip install "openai-agents[litellm]"
```

**Issue: Agent runs but takes 30+ seconds**

This is normal for API calls. The agent is:
1. Sending your prompt to OpenAI/Claude's servers
2. Waiting for model inference
3. Receiving and parsing the response

Network latency adds up. Later lessons cover streaming and optimization.

## Next Steps

You now have:
- A working SDK installation
- Your first agent prototype
- Knowledge of how to configure API keys
- Ability to use alternative models via LiteLLM

Chapter 34 lessons explore:
- **Lesson 2**: Building agents with tools (calling APIs, databases)
- **Lesson 3**: Multi-agent systems and handoffs
- **Lesson 4**: Production patterns (guardrails, error handling, structured outputs)

For now, practice the three-part pattern (Agent → Runner → Result) until it feels natural. This pattern underlies everything else in agent development.

## Try With AI

### Prompt 1: Personalized Agent

```
Create an agent that acts as a personal assistant for a specific profession
(pick one: doctor, architect, teacher, lawyer, or journalist).
Write the agent definition with appropriate instructions, run it with
Runner.run_sync(), and capture the response.

What should this agent's instructions emphasize to do the job well?
```

**What you're learning:** How instructions shape agent behavior and specialization

### Prompt 2: Instructions Experiment

```
Create the same agent twice: once with vague instructions
("Be helpful") and once with detailed instructions
(specific persona, job responsibilities, communication style).
Run both with the same user prompt and compare the responses.

What differences do you notice? Which is better?
```

**What you're learning:** The relationship between specification quality and agent reliability

### Prompt 3: Model Switching

```
Set up an agent using OpenAI (gpt-4o) and another using
LiteLLM with Claude or Gemini. Give them the same instructions
and ask the same question. Compare:
- Response quality
- Speed
- Cost differences (OpenAI vs free tier)

Which would you use for your Digital FTE and why?
```

**What you're learning:** How to evaluate models for specific agent use cases

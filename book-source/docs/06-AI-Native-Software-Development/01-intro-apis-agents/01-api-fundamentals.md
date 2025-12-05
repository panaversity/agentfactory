---
sidebar_position: 2
title: "Lesson 1.1: API Fundamentals"
description: "Understand what APIs are using a restaurant analogy and verify your OpenAI API key works"
keywords: [API, OpenAI, authentication, request-response, JSON]
---

# Lesson 1.1: API Fundamentals

**Duration**: 30-40 minutes
**Difficulty**: Beginner (A1-A2)

## Learning Goal

By the end of this lesson, you will:
- Understand what APIs are and why they matter
- Know how your code communicates with AI services
- Understand the difference between stateless and stateful APIs
- Successfully verify that your OpenAI API key works

---

## What You'll Learn

When you ask ChatGPT a question, something happens behind the scenes. Your message travels to OpenAI's servers, gets processed by an AI model, and a response comes back. This communication happens through an **API**.

**API** stands for **Application Programming Interface**. Think of it as a communication channel between your code and another service. Here's what you need to know:

1. **APIs are bridges**: They let different software talk to each other
2. **Requests go out**: Your code sends a message asking for something
3. **Responses come back**: The service sends an answer
4. **API keys identify you**: Like showing ID to enter a building (for paid/private APIs)

:::info Key Concept
An API is like a waiter in a restaurant — it takes your order to the kitchen and brings back your food. You don't need to know how the kitchen works; you just need to communicate your request.
:::

### Types of APIs You'll Use

In this course, you'll work with two types:

1. **Public APIs** (no key needed): Weather data, random facts, public information
2. **Authenticated APIs** (key required): OpenAI, Gemini, and other AI services

---

## Key Points

Before we continue, let's establish five foundational ideas:

- **APIs are everywhere**: Every time an app loads weather data, shows tweets, or processes payments, it's using APIs
- **Request/Response is the pattern**: You send a request, you get a response — it's a two-way conversation
- **Authentication matters**: API keys prove you're allowed to use the service (and track your usage)
- **JSON is the language**: Data travels back and forth in JSON format — a simple way to structure information
- **Stateless vs Stateful**: Chat Completions (stateless) gives you full control; Responses API (stateful) manages context for you

---

## Simple Analogy

:::tip The Restaurant Analogy
Imagine you're at a restaurant:

1. **You** (the client) look at the menu and decide what you want
2. **The waiter** (the API) takes your order to the kitchen
3. **The kitchen** (OpenAI's servers) prepares your food
4. **The waiter** brings back your meal (the AI's response)

You don't need to know how to cook. You don't need to enter the kitchen. You just need to:
- Know what to order (your prompt)
- Communicate clearly with the waiter (use the API correctly)
- Have a way to pay (your API key)

Your code works exactly the same way with OpenAI:
- Your code sends a prompt to OpenAI (placing an order)
- OpenAI processes it with an AI model (the kitchen works)
- OpenAI sends back a completion (your meal arrives)
:::

---

## Real-World API Example

Before we jump to AI APIs, let's see a simple public API in action. This weather API requires no key:

```python
import requests

# Get current weather for London
url = "https://api.open-meteo.com/v1/forecast?latitude=51.5&longitude=-0.12&current_weather=true"
response = requests.get(url, timeout=10)
weather = response.json()

print(f"Temperature: {weather['current_weather']['temperature']}°C")
print(f"Wind Speed: {weather['current_weather']['windspeed']} km/h")
```

**What's happening:**
1. We import the `requests` library (for making HTTP calls)
2. We define the API URL with parameters (latitude, longitude)
3. We send a GET request and wait for response
4. We parse the JSON response and extract data

This pattern — **request → response → parse** — is the same for ALL APIs, including OpenAI.

---

## OpenAI APIs: Stateless vs Stateful

OpenAI offers two main ways to interact with their models:

### Chat Completions API (Stateless)

Each request is independent — the AI has no memory of previous messages:

```python
from openai import OpenAI
client = OpenAI()

# First message
response1 = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[{"role": "user", "content": "My name is Alex"}]
)

# Second message - AI doesn't remember the first!
response2 = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[{"role": "user", "content": "What's my name?"}]
)
# AI will say: "I don't know your name"
```

**Why stateless?** You control the context. You must send conversation history each time.

### Responses API (Stateful)

OpenAI maintains conversation state for you using `previous_response_id`:

```python
from openai import OpenAI
client = OpenAI()

# First message
response1 = client.responses.create(
    model="gpt-4.1",
    store=True,
    input=[{"role": "user", "content": [{"type": "input_text", "text": "My name is Alex"}]}]
)

# Second message - uses previous_response_id to maintain context
response2 = client.responses.create(
    model="gpt-4.1",
    store=True,
    previous_response_id=response1.id,
    input=[{"role": "user", "content": [{"type": "input_text", "text": "What's my name?"}]}]
)
# AI will say: "Your name is Alex"
```

### Which Should You Use?

| Feature | Chat Completions | Responses API |
|---------|-----------------|---------------|
| Memory | You manage it | OpenAI manages it |
| Control | Full control | Less control |
| Use Case | Agents, custom apps | Simple chatbots |
| DocuBot | ✅ We'll use this | Not for agents |

**For DocuBot**: We'll use Chat Completions because agents need full control over context and memory.

---

## 🤖 Apply to DocuBot Project

Time to put this knowledge into action! You'll create a simple test to verify your OpenAI API key works.

### Task

Test your OpenAI API key with a simple API call:

1. **Create a file** called `test_api.py` in your project folder
2. **Import OpenAI** — add `from openai import OpenAI` at the top
3. **Create a client** — add `client = OpenAI()`
4. **Make a chat completion call** — use `client.chat.completions.create()`
5. **Print the response** — display what the AI says

### Outcome

*Confirmed working API key. You see a response from OpenAI in your terminal, proving your development setup is correct.*

---

## 💡 Hints

Work through these hints progressively. Try each step before moving to the next hint.

<details>
<summary><strong>Hint 1</strong> (Conceptual)</summary>

Make sure you have your `OPENAI_API_KEY` set up. The OpenAI client looks for this environment variable automatically.

If you don't have a key yet:
1. Go to [platform.openai.com](https://platform.openai.com)
2. Sign in or create an account
3. Navigate to API keys section
4. Create a new key (starts with `sk-`)

</details>

<details>
<summary><strong>Hint 2</strong> (Getting Started)</summary>

Your file should start like this:

```python
from openai import OpenAI

client = OpenAI()
```

The `OpenAI()` client automatically reads your API key from the environment variable `OPENAI_API_KEY`.

</details>

<details>
<summary><strong>Hint 3</strong> (The API Call)</summary>

To make a chat completion, use:

```python
response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "user", "content": "Say hello!"}
    ]
)
```

The `messages` list is how you send your prompt. Each message has a `role` (who's speaking) and `content` (what they say).

</details>

<details>
<summary><strong>Hint 4</strong> (Getting the Response)</summary>

The response object contains a lot of information. To get just the AI's text:

```python
print(response.choices[0].message.content)
```

This navigates to: `response` → `choices` (list) → first item `[0]` → `message` → `content` (the text)

</details>

<details>
<summary><strong>Hint 5</strong> (Complete Solution)</summary>

Here's the full working code:

```python
from openai import OpenAI

client = OpenAI()

response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "user", "content": "Say hello!"}
    ]
)

print(response.choices[0].message.content)
```

Run this with `python test_api.py`. You should see a friendly greeting from the AI!

</details>

---

## 📝 Starter Code

Copy this into your `test_api.py` file and fill in the missing part:

```python
# test_api.py - Verify your OpenAI API key works
# This is your first interaction with an AI API!

from openai import OpenAI

# Create a client (automatically uses OPENAI_API_KEY from environment)
client = OpenAI()

# Your code here: make a chat completion call
# Use client.chat.completions.create() with:
#   - model: "gpt-4o-mini"
#   - messages: a list with one user message

# Then print response.choices[0].message.content
```

---

## Troubleshooting

If something goes wrong, here are the most common issues and solutions:

### Error: 401 Authentication Failed

**What it means**: Your API key is missing, incorrect, or expired.

**How to fix**:
- Check your API key is set: `echo $OPENAI_API_KEY` (Mac/Linux) or `echo %OPENAI_API_KEY%` (Windows)
- Verify the key starts with `sk-`
- Make sure there are no extra spaces or quotes
- Check your key at [platform.openai.com/api-keys](https://platform.openai.com/api-keys)

### Error: Module Not Found

**What it means**: The OpenAI package isn't installed.

**How to fix**:
```bash
# If using UV (recommended)
uv add openai

# If using pip
pip install openai
```

### Error: Network/Connection Issues

**What it means**: Your computer can't reach OpenAI's servers.

**How to fix**:
- Check your internet connection
- If behind a corporate firewall, ask IT about API access
- Try again in a few minutes

### Windows-Specific Notes

- Use PowerShell, not Command Prompt
- To set environment variable temporarily:
  ```powershell
  $env:OPENAI_API_KEY="sk-your-key-here"
  python test_api.py
  ```

### macOS/Linux Notes

- To set environment variable temporarily:
  ```bash
  export OPENAI_API_KEY="sk-your-key-here"
  python test_api.py
  ```

---

## Try With AI

:::tip Practice with ChatGPT
Since we haven't set up coding tools yet, use [ChatGPT](https://chat.openai.com) to explore:

1. "Explain the HTTP request/response cycle in simple terms"
2. "What's the difference between an API key and a password?"
3. "Why do APIs use JSON format?"

This reinforces your understanding through conversation!
:::

---

## What's Next

Congratulations! You've:
- ✅ Understood what APIs are (communication bridges)
- ✅ Learned the request/response pattern
- ✅ Verified your OpenAI API key works
- ✅ Made your first AI API call

In the next lesson, you'll learn why we need to build an **agent** instead of just using a chatbot like ChatGPT.

---

**Next**: [Lesson 1.2 — LLM vs Agent](./02-llm-vs-agent.md)

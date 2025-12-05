---
sidebar_position: 6
title: "Chapter 1 Quiz"
description: "Test your understanding of APIs, LLMs vs Agents, environment setup, and system architecture"
---

# Chapter 1 Quiz

Test your understanding of the foundational concepts covered in Chapter 1.

---

## Instructions

- Answer all 10 questions
- Each question has one correct answer
- Review the relevant lesson if you're unsure

---

### Question 1: API Fundamentals

**What does API stand for?**

- A) Advanced Programming Interface
- B) Application Programming Interface
- C) Automated Program Integration
- D) Application Process Interface

<details>
<summary>Show Answer</summary>

**B) Application Programming Interface**

An API is a communication channel that allows different software applications to talk to each other.

</details>

---

### Question 2: API Analogy

**In the restaurant analogy, what does the "waiter" represent?**

- A) The OpenAI server
- B) Your code (the client)
- C) The API
- D) The JSON response

<details>
<summary>Show Answer</summary>

**C) The API**

The waiter (API) takes your order (request) to the kitchen (server) and brings back your food (response). The API is the intermediary that handles communication.

</details>

---

### Question 3: API Authentication

**What is the purpose of an API key?**

- A) To encrypt your messages
- B) To authenticate your identity and track usage
- C) To speed up API responses
- D) To format data as JSON

<details>
<summary>Show Answer</summary>

**B) To authenticate your identity and track usage**

API keys prove you're authorized to use the service and allow the provider to track how much you're using the API.

</details>

---

### Question 4: LLM Definition

**What is a Large Language Model (LLM)?**

- A) A database that stores conversations
- B) A text prediction system trained on vast amounts of data
- C) A tool that can search the internet
- D) A memory system for AI applications

<details>
<summary>Show Answer</summary>

**B) A text prediction system trained on vast amounts of data**

LLMs like GPT-4 predict what text should come next based on the input they receive. They're trained on massive datasets to understand and generate human-like text.

</details>

---

### Question 5: LLM vs Agent

**What makes an agent different from a basic LLM?**

- A) Agents are faster at generating text
- B) Agents have tools, memory, and can take actions
- C) Agents don't use language models
- D) Agents only work with documents

<details>
<summary>Show Answer</summary>

**B) Agents have tools, memory, and can take actions**

An agent combines an LLM with additional capabilities: tools (to perform actions), memory (to remember context), and a reasoning loop (to complete complex tasks).

</details>

---

### Question 6: Agent Loop

**What happens in an agent's reasoning loop?**

- A) The agent generates one response and stops
- B) The agent plans, acts, observes, and repeats until the task is done
- C) The agent only searches for documents
- D) The agent waits for user input between each step

<details>
<summary>Show Answer</summary>

**B) The agent plans, acts, observes, and repeats until the task is done**

The agent loop allows the agent to break down complex tasks, execute steps, observe results, and continue working until the objective is achieved.

</details>

---

### Question 7: Environment Setup

**What is the primary benefit of using UV over pip?**

- A) UV works on more operating systems
- B) UV is 10-100x faster and handles virtual environments automatically
- C) UV is made by OpenAI
- D) UV doesn't require Python installed

<details>
<summary>Show Answer</summary>

**B) UV is 10-100x faster and handles virtual environments automatically**

UV is a modern package manager that significantly speeds up dependency installation and automatically manages virtual environments and Python versions.

</details>

---

### Question 8: Environment Variables

**Where should you store your OpenAI API key?**

- A) Directly in your Python code
- B) In a .env file that's not committed to Git
- C) In the pyproject.toml file
- D) In the README.md file

<details>
<summary>Show Answer</summary>

**B) In a .env file that's not committed to Git**

API keys are secrets that should never be committed to version control. The .env file keeps them separate and should be listed in .gitignore.

</details>

---

### Question 9: System Architecture

**What does RAG stand for in the context of DocuBot?**

- A) Random Access Generation
- B) Retrieval-Augmented Generation
- C) Rapid Agent Gateway
- D) Real-time API Generation

<details>
<summary>Show Answer</summary>

**B) Retrieval-Augmented Generation**

RAG combines retrieval (searching documents) with generation (creating responses). This allows the AI to answer questions based on your specific documents rather than just its training data.

</details>

---

### Question 10: Component Understanding

**Which component serves as the "brain" of DocuBot that decides what actions to take?**

- A) Vector Database (Qdrant)
- B) Frontend (ChatKit)
- C) Agent (OpenAI Agents SDK)
- D) Backend (FastAPI)

<details>
<summary>Show Answer</summary>

**C) Agent (OpenAI Agents SDK)**

The Agent processes user questions, decides which tools to use, maintains conversation context, and generates responses. It's the intelligent core of the system.

</details>

---

## Scoring

| Score | Rating |
|-------|--------|
| 10/10 | Excellent! Ready for Chapter 2 |
| 8-9/10 | Great understanding, minor review needed |
| 6-7/10 | Good start, review lessons for missed concepts |
| Below 6 | Re-read the chapter before continuing |

---

## What's Next

If you scored 8 or above, you're ready for **Chapter 2: Your First Agent** where you'll write your first real agent code!

If you scored below 8, review the lessons for the questions you missed:
- Questions 1-3: [Lesson 1.1 — API Fundamentals](./01-api-fundamentals.md)
- Questions 4-6: [Lesson 1.2 — LLM vs Agent](./02-llm-vs-agent.md)
- Questions 7-8: [Lesson 1.3 — Environment Setup](./03-environment-setup.md)
- Questions 9-10: [Lesson 1.4 — Architecture Overview](./04-architecture-overview.md)

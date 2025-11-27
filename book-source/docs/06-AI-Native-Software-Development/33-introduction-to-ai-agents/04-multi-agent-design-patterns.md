---
title: "Multi-Agent Design Patterns"
sidebar_position: 4
description: "Learn the four multi-agent patterns from Google's whitepaper: Coordinator, Sequential, Iterative Refinement, and Human-in-the-Loop."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 60 minutes
learning_objectives:
  - "Name and describe 4 multi-agent patterns from the paper: Coordinator, Sequential, Iterative Refinement, Human-in-the-Loop"
  - "Match real-world use cases to appropriate patterns"
  - "Explain the trade-offs between single-agent and multi-agent approaches"
  - "Understand when pattern selection matters for system design"
skills:
  agent_design_patterns:
    proficiency: B1
  system_architecture_decisions:
    proficiency: B1
generated_by: content-implementer v1.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 1.0.0
---

# Multi-Agent Design Patterns

By now, you understand what makes an agent: a model, tools, orchestration, and deployment working together in a loop. You've traced through how a single agent reasons and acts. But most real-world systems aren't built with one all-powerful agent trying to do everything. Instead, they're composed of multiple agents working together, each with specialized responsibilities.

Why? Because building a single agent that can reliably answer customer support questions, generate reports, validate data quality, and handle exceptions all at the same level of sophistication is inefficient. It's easier—and more reliable—to compose a team of specialists. Each agent does one thing well. The system's architecture determines how they collaborate.

This lesson teaches you the four canonical multi-agent patterns that appear in production systems across OpenAI, Google, and Anthropic implementations. By the end, you'll recognize these patterns everywhere and understand how to choose the right one for different problems.

## Why Multi-Agent Systems?

Before diving into patterns, understand what problem multi-agent systems solve.

**The Single-Agent Bottleneck**

A single agent must:
- Decide what information it needs
- Gather that information (search, database queries, external APIs)
- Analyze the information in context
- Decide what to do next
- Execute actions
- Evaluate results
- Decide whether to repeat or declare success

For simple problems, this works. "Translate this text to French" — one agent, one pass, done.

But for complex problems, a single agent becomes a bottleneck. Consider analyzing a customer complaint that requires:
1. Extracting issue details (parsing customer intent)
2. Checking system logs (database access + analysis)
3. Searching the knowledge base for similar issues (information retrieval)
4. Drafting a response (content generation)
5. Checking it against company policy (compliance validation)
6. Routing to a human if necessary (escalation)

A single agent must reason about all six steps, make all six decisions, and orchestrate the flow. If any step fails, the agent has to figure out recovery. The cognitive load becomes significant. The surface area for failure increases.

**The Multi-Agent Solution**

Instead, decompose into specialists:
- **Intent Agent** (analyzes customer messages)
- **Log Agent** (understands system logs and error patterns)
- **Knowledge Base Agent** (searches and ranks similar cases)
- **Response Agent** (drafts customer-facing language)
- **Compliance Agent** (validates against policies)
- **Routing Agent** (decides escalation)

Each agent is smaller, simpler, and easier to test. But now you have a new problem: how do they coordinate? The answer is architectural pattern — a reusable solution to the "how do multiple agents work together?" problem.

---

## Pattern 1: Coordinator

**The Problem It Solves**: Complex requests that need multiple areas of expertise, but the flow isn't strictly sequential. A request comes in. Different specialties must weigh in. Results must be aggregated into a cohesive answer.

**The Architecture**

The Coordinator pattern uses a **manager agent** that:
1. Analyzes the incoming request
2. Routes to appropriate specialist agents
3. Collects their responses
4. Synthesizes a final answer

Think of it as a project manager coordinating specialists. The manager doesn't do the work. The manager decides who should work on what, and composes the results.

**Visual Model**

```
User Query
    ↓
[Coordinator Agent]
    ├─→ [Specialist 1: Analysis]
    ├─→ [Specialist 2: Research]
    ├─→ [Specialist 3: Validation]
    └─→ [Specialist 4: Planning]
         ↓
    [Synthesis]
         ↓
    Final Response
```

**Real-World Example: Financial Analysis System**

A user asks: "Should I invest in this company? I have a tech background but limited financial knowledge."

The Coordinator routes:
- **Financial Agent**: "Here's the company's revenue growth, debt-to-equity ratio, and cash flow trajectory"
- **Technology Agent**: "Here's their technology stack, competitive advantages, and technical debt"
- **Risk Agent**: "Here's market risk, regulatory exposure, and concentration of revenue"
- **Valuation Agent**: "Here's their valuation relative to comparable companies and historical multiples"

The Coordinator synthesizes: "Based on strong tech position but elevated valuation, this is a moderate buy for tech-literate investors with 3-5 year horizon."

**Use Cases for Coordinator Pattern**

- **Customer support** with departments (billing, technical, returns)
- **Research requests** requiring multiple data sources
- **Business decisions** needing financial, legal, technical, and strategic input
- **Medical diagnosis** combining patient history, lab results, symptoms, and current medications
- **Hiring decisions** combining technical, cultural, and leadership assessments

**Trade-Offs**

**Advantages**:
- Specialists can be deep and focused
- Easy to add new specialists without changing coordinator logic
- Results incorporate multiple perspectives naturally
- Failures in one specialist don't block others (can handle partial failures)

**Disadvantages**:
- Coordinator must be intelligent enough to route correctly
- Latency increases (runs specialist agents in parallel or sequence)
- Synthesis can be complex if specialists disagree significantly
- Needs clear context/instructions for each specialist

---

## Pattern 2: Sequential

**The Problem It Solves**: Workflows where the output of one agent becomes the input to the next. Like an assembly line — each workstation processes output from the previous one.

**The Architecture**

Sequential patterns chain agents: Agent A produces output → Agent B consumes and transforms it → Agent C consumes and transforms it → final output.

The key characteristic: **each agent's output is the primary input for the next agent**. This is linear, not parallel.

**Visual Model**

```
Input Document
    ↓
[Agent 1: Extraction]
Extracts key information
    ↓
[Agent 2: Enrichment]
Adds context and references
    ↓
[Agent 3: Validation]
Checks consistency and accuracy
    ↓
[Agent 4: Formatting]
Prepares for output
    ↓
Formatted Output
```

**Real-World Example: Document Processing Pipeline**

A company processes invoices in a sequential pipeline:

1. **Extraction Agent**: "Read the PDF, extract vendor name, invoice number, date, line items, total amount, and payment terms"
   - Output: Structured invoice data

2. **Enrichment Agent**: "For each vendor, look up their history with us and add discount eligibility"
   - Output: Structured data + vendor context

3. **Validation Agent**: "Check if amounts match line items, if vendor exists in our system, if payment terms are standard"
   - Output: Marked data (valid/invalid/requires review) + flags

4. **Routing Agent**: "If valid, route to accounts payable. If needs review, add to queue. If invalid, send to finance team"
   - Output: Categorized invoices ready for next step

Each agent has a clear input (previous agent's output) and a clear output (feeds next agent). No coordination complexity.

**Use Cases for Sequential Pattern**

- **Data processing pipelines** (ETL: Extract, Transform, Load)
- **Document workflows** (scan → OCR → validate → archive)
- **Content creation** (outline → draft → edit → publish)
- **Code review automation** (lint → security scan → test → merge)
- **Customer onboarding** (intake → verification → setup → activation)

**Trade-Offs**

**Advantages**:
- Dead simple to understand and implement
- Clear handoff points between agents
- Easy to debug (each agent's input/output is visible)
- Natural for linear processes
- Each agent can be optimized independently

**Disadvantages**:
- If one agent fails, the entire pipeline stalls (unless error handling is explicit)
- Can be slow (sequential = no parallelism)
- Hard to backtrack if an intermediate agent's output is wrong
- Errors compound (garbage in from Agent 1 leads to garbage out from Agent 4)

---

## Pattern 3: Iterative Refinement

**The Problem It Solves**: When quality matters more than speed. A single pass isn't good enough. You need feedback loops to improve output. Generator produces → Evaluator critiques → Generator refines → repeat until quality threshold.

**The Architecture**

Iterative Refinement uses a **generator-critic loop**:

1. **Generator** produces an initial output
2. **Evaluator** assesses the output against criteria
3. If quality is insufficient, feedback goes back to Generator
4. Generator refines based on feedback
5. Repeat until output meets quality threshold or iteration limit

**Visual Model**

```
Input
 ↓
[Generator Agent]
Produces initial output
 ↓
[Evaluator Agent]
Assesses: "Is this good enough?"
 ↓
Quality check
├─→ PASS → Output
└─→ FAIL → Feedback
      ↓
      [Generator Agent]
      Refines based on feedback
          ↓
          [Evaluator Agent]
          Re-assesses
              ↓
              ...repeat until quality threshold or iteration limit
```

**Real-World Example: Technical Documentation**

A documentation system generates API reference docs with quality requirements (correct examples, accurate descriptions, complete parameter documentation):

**Iteration 1**:
- **Generator**: "Write documentation for the `updateUser` endpoint"
  - Output: Basic documentation with description and parameters
- **Evaluator**: "Check against 5 quality criteria: examples work, descriptions are accurate, all parameters documented, includes error responses, includes authentication info"
  - Result: "FAIL — missing examples and error responses documentation"

**Iteration 2**:
- **Generator** (with feedback): "Rewrite, adding executable code examples and all error responses"
  - Output: Improved documentation with examples and errors
- **Evaluator**: "Re-assess"
  - Result: "PASS — meets all 5 criteria"

Output is deployed.

**Use Cases for Iterative Refinement Pattern**

- **Content creation** with quality gates (blog posts, product copy, documentation)
- **Code review automation** (generate code → test → evaluate coverage → refine)
- **Research synthesis** (generate summary → fact-check → refine)
- **Creative work** (generate ideas → evaluate against criteria → improve)
- **Compliance checking** (generate policy doc → audit against regulations → refine)

**Trade-Offs**

**Advantages**:
- Natural for quality-focused workflows
- Transparent feedback helps debugging (you see exactly why output was rejected)
- Can define clear quality criteria upfront
- Better for creative/nuanced tasks than single-pass
- Prevents low-quality output from reaching users

**Disadvantages**:
- Slower (requires multiple passes)
- If evaluator criteria are wrong, loop becomes useless
- Can get stuck in infinite loops (evaluator never satisfied)
- More complex implementation (need iteration limits, convergence criteria)
- Higher cost (multiple agent calls)

---

## Pattern 4: Human-in-the-Loop

**The Problem It Solves**: High-stakes decisions where AI autonomy isn't appropriate. AI reasons, proposes an action, but deliberate stops and waits for human approval before proceeding.

**The Architecture**

Human-in-the-Loop introduces deliberate pause points in the agent workflow:

1. Agent analyzes situation and proposes action
2. Agent pauses and asks for human approval
3. Human reviews and approves, rejects, or modifies
4. Agent implements the approved action (or adapts to rejection)

The key: **the human is integrated into the workflow, not bypassed**.

**Visual Model**

```
Input/Trigger
    ↓
[Agent Analysis]
Recommends action based on reasoning
    ↓
[Propose to Human]
Display recommendation with reasoning
    ↓
[Human Decision Point]
├─→ APPROVE → Proceed
├─→ MODIFY → Agent adapts
└─→ REJECT → Alternative action
         ↓
[Agent Implements]
Takes approved action
         ↓
[Observe & Report]
Confirms action taken to human
```

**Real-World Example: Financial Transaction Approval**

A payment system processes vendor payments:

1. **Agent Analysis**: "I've reviewed the invoice (3000 USD from vendor ABC, invoice matches PO, payment terms met, vendor banking info correct). I recommend approve and process."

2. **Human Review**: Finance manager sees:
   - Vendor details
   - Invoice details
   - Agent's analysis and recommendation
   - Risk flags (if any)

3. **Human Decision**:
   - Manager: "Looks good. Approve" (payment processes)
   - OR "Hold — let me verify vendor banking info separately" (agent waits)
   - OR "Reject — this vendor has disputed charges pending" (agent doesn't pay)

4. **Agent Implements**: Takes approved action (process payment, log decision, notify vendor)

**Use Cases for Human-in-the-Loop Pattern**

- **Financial decisions** (payment approval, loan decisions, large purchases)
- **Compliance and legal** (contract review, policy exceptions)
- **Healthcare** (diagnosis suggestions before treatment, medication recommendations)
- **Hiring** (final candidate recommendations before offer)
- **Customer communication** (draft communication before sending, especially escalations)
- **Security decisions** (suspicious activity analysis before account lockdown)

**Trade-Offs**

**Advantages**:
- Reduces risk for high-stakes decisions (human remains in control)
- Builds user trust (people know humans are involved)
- Humans can catch context the AI missed
- Natural for compliance-heavy domains
- Scales accountability (humans responsible for final decision)

**Disadvantages**:
- Slow (requires human response time)
- Requires human availability and attention
- Humans may be biased or make poor decisions
- Can create false sense of security (approval theater without real review)
- Not scalable to decisions requiring constant human involvement

---

## Comparing the Four Patterns: Decision Framework

Each pattern solves a different problem. How do you choose?

**Use Coordinator When**:
- Request needs input from multiple expertise areas
- Specialists work independently (can parallelize)
- You need a comprehensive view (multiple perspectives)
- Examples: customer support, research, hiring decisions

**Use Sequential When**:
- Work flows naturally in stages (output → next input)
- Each stage transforms the data meaningfully
- Linear processing is acceptable (speed isn't critical)
- Examples: document processing, ETL pipelines, onboarding

**Use Iterative Refinement When**:
- Quality matters more than speed
- You can define clear quality criteria upfront
- Output is creative/nuanced and benefits from feedback
- Examples: documentation, content, code generation

**Use Human-in-the-Loop When**:
- Decision has high stakes or irreversible consequences
- Regulatory/compliance requirements demand human oversight
- Risk is unacceptable without human judgment
- Examples: financial, medical, legal decisions

**Common Hybrid Patterns**

In practice, these patterns combine:

- **Coordinator + Iterative Refinement**: Multiple specialists each refine their output before coordinator synthesizes
- **Sequential + Human-in-the-Loop**: Pipeline with approval gates at critical steps
- **Coordinator + Sequential**: Coordinator routes to specialists; specialists use sequential pipelines
- **All Four**: Complex system with coordinator routing to specialized teams, teams using sequences and iterations, final approval by human

**Decision Tree**

```
Does the request need multiple expertise areas?
├─→ YES (request is complex, multi-dimensional)
│   Use COORDINATOR
│
└─→ NO (request has clear processing steps)
    Does the work naturally flow step-by-step?
    ├─→ YES (each step transforms output for next)
    │   Use SEQUENTIAL
    │
    └─→ NO (work requires improvement or feedback)
        Is quality or accuracy critical?
        ├─→ YES (high quality required)
        │   Use ITERATIVE REFINEMENT
        │
        └─→ NO (decision is high-stakes or regulated)
            Use HUMAN-IN-THE-LOOP
```

---

## Trade-Offs: Single-Agent vs Multi-Agent

Before concluding this lesson, step back and ask: should you use multiple agents at all?

**Single-Agent Advantages**:
- Simpler to build and reason about
- Lower latency (no coordination overhead)
- Less cost (one model call instead of many)
- Easier to debug (single reasoning trace)
- Simpler context management

**Single-Agent Disadvantages**:
- Bottleneck for complex problems (one agent doing everything)
- Harder to specialize (model must be good at everything)
- Single point of failure
- Harder to test (large surface area)
- Can amplify errors (one reasoning chain leading to compounding mistakes)

**Multi-Agent Advantages**:
- Specialization (each agent is good at one thing)
- Parallel work (some patterns parallelize)
- Fault tolerance (one agent failing doesn't break everything)
- Modularity (add/remove agents easily)
- Clearer responsibility boundaries

**Multi-Agent Disadvantages**:
- More latency (coordination overhead)
- Higher cost (multiple model calls)
- More complex to debug (multiple reasoning traces)
- More context to manage
- Synchronization complexity

**When to Choose**

**Start with single-agent** if:
- Problem is straightforward (can fit in one agent's scope)
- Latency is critical
- Cost is constrained
- You're prototyping to understand the problem

**Move to multi-agent** if:
- Single agent becomes unreliable (can't handle all cases equally)
- Problem needs specialization (different agents are good at different aspects)
- You need parallelism (independent work happening simultaneously)
- Scale requires modularity (easier to maintain separate specialists)

Most real-world systems eventually become multi-agent because specialization and reliability matter more than simplicity.

---

## Try With AI

You've learned four design patterns. Now explore when each applies by working with an AI.

**Setup**: Open your preferred AI tool (Claude, ChatGPT, Gemini) and use these prompts to deepen your understanding through dialogue.

**Prompt 1: Pattern Matching (Basic)**

```
I'm building an agent system for a lawyer's office to help with contract review.
The system needs to:
1. Extract key terms (parties, dates, obligations)
2. Check against contract templates
3. Identify risks and potential issues
4. Suggest improvements
5. Verify legal compliance

Which of these four patterns would you recommend?
- Coordinator
- Sequential
- Iterative Refinement
- Human-in-the-Loop

Explain your choice and what would happen in each pattern.
```

**Expected Outcomes**: AI should recommend Sequential (clear pipeline: extract → template match → risk analysis → legal check) but also justify why Human-in-the-Loop is necessary for final approval. You'll see the reasoning for pattern selection.

**Prompt 2: Pattern Hybridization (Intermediate)**

```
I described a contract review system. Let's say I wanted to combine
patterns to make it more robust. What would a Sequential + Human-in-the-Loop
hybrid look like? Where would the human approval pause?

Also, why wouldn't I want to use Iterative Refinement in the contract review?
```

**Expected Outcomes**: AI should explain how to insert approval gates at critical steps (e.g., after risk analysis, before final recommendations), and explain why iterative refinement would slow down a workflow that requires human review anyway.

**Prompt 3: Real-World Trade-Off (Advanced)**

```
You're building a customer support system with 100,000 daily requests.
You could use:
- One super-agent handling everything
- Coordinator pattern routing to 5 specialist agents

What factors would influence your choice? Cost? Latency? Accuracy? Staff?
What are the downsides to each approach at this scale?
```

**Expected Outcomes**: AI will explore scalability dimensions you might not have considered — queueing behavior, failure modes at scale, cost multiplication from parallel agents, debugging complexity when one specialist breaks. You'll develop nuance around pattern selection.

**Stretch: Design Your Own Scenario**

Think of a process you're familiar with (hiring, content approval, expense reimbursement, quality assurance). Which pattern would you apply? Why? What would fail if you chose the wrong pattern?

Share your scenario with the AI and discuss:
- Why your pattern choice makes sense
- What would break with the other three patterns
- Whether a hybrid pattern would be better

---
title: "Your First Agent Concept"
sidebar_position: 8
description: "Design your first agent specification using all frameworks from this chapter: 5-Level Taxonomy, 3+1 Architecture, 5-Step Loop, patterns, operations, and security."
proficiency_level: B1
cognitive_load:
  new_concepts: 0
  estimated_difficulty: B1
estimated_time: 40 minutes
learning_objectives:
  - "Design agent specification using paper's frameworks without introducing new concepts"
  - "Articulate agent's purpose using 5-Level Taxonomy (Level 0-4)"
  - "Specify architecture using 3+1 components with decision justifications"
  - "Identify appropriate pattern and security considerations for use case"
skills:
  agent_specification:
    proficiency: B1
  systems_thinking:
    proficiency: B1
generated_by: content-implementer v1.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 1.0.0
---

# Your First Agent Concept

By completing this lesson, you've mastered the foundational frameworks for agent architecture. You understand the 5-Level Taxonomy that classifies agent systems by capability. You know the 3+1 Architecture—Model, Tools, Orchestration, and Deployment—that makes every agent work. You've traced the 5-Step Loop (Get Mission → Scan Scene → Think → Act → Observe) that governs how agents reason and act. You've learned patterns, operations discipline, interoperability standards, and security principles.

Now comes the synthesis: designing your own agent specification. This is not a tutorial. This is your opportunity to apply everything you've learned to define a real agent system. By the end of this lesson, you'll have written a complete specification that could guide implementation in Chapter 34.

This lesson embodies the specification-first principle from Part 4 (SDD-RI). Before code, comes intent. Before implementation, comes design. You're learning to think like a director—setting the stage, selecting your cast, defining constraints—rather than as a bricklayer executing predetermined steps.

---

## Specification-First Thinking: Why Specs Matter for Agents

In traditional software development, specifications describe *what the code should do*. In agent development, specifications describe *how the agent should think and behave*. This is fundamentally different.

When you write a specification for a traditional system, you're often specifying a deterministic workflow: "Read file, parse JSON, validate schema, return result." The system follows a predetermined path.

When you write a specification for an agent, you're specifying the agent's constitution: "You are a customer support agent. Your goal is to resolve customer inquiries with empathy and accuracy. You have access to these tools. You must never make refunds over $500 without escalation. Your tone should be professional but warm."

The agent then reasons within those constraints to find its own path to success.

This is why specifications are harder to write for agents than for traditional systems. You must anticipate failure modes without prescribing solutions. You must set boundaries without removing flexibility. You must inspire competence without dictating steps.

The good news: you already have frameworks to guide this work. The 5-Level Taxonomy helps you scope what capability you're building. The 3+1 Architecture helps you decide which components you need. The 5-Step Loop helps you understand what the agent will actually do. Patterns help you recognize proven designs. Operations help you plan for evaluation and debugging. Interoperability and security help you design for production realities.

This lesson provides a template that synthesizes all these frameworks into one specification document.

---

## Agent Specification Template: Complete Framework

Below is a template that combines every framework from this chapter. Use this as your reference for writing agent specifications.

### AGENT SPECIFICATION TEMPLATE

**Agent Name:**
[Clear, memorable name for your agent]

**Organization/Context:**
[What organization is this agent for? What domain?]

**Purpose (Intent):**
[One paragraph: What goal does this agent accomplish? Why does it matter?]

---

### 1. CAPABILITY LEVEL (5-Level Taxonomy)

**Target Level:** [0, 1, 2, 3, or 4]

**Level Justification:**
- Reasoning required (Level 0 → 1): Can the agent access real-time information?
- Strategic planning (Level 1 → 2): Must the agent break complex goals into steps?
- Multi-agent coordination (Level 2 → 3): Do multiple specialized agents need to work together?
- Self-evolution (Level 3 → 4): Must the agent create new tools or spawn new agents?

**Why this level?** [Explain the reasoning. Example: "Level 2 because the agent must plan multi-step customer resolutions, but doesn't need to coordinate with other agents."]

---

### 2. ARCHITECTURE (3+1 Components)

#### Model ("Brain")

**Model Selection:**
- Primary model: [Claude 3.5 Sonnet / Haiku / etc.]
- Reasoning capability needed: [Basic / Moderate / Advanced]
- Multimodal requirements: [Text only / Images / Audio / Video]

**Justification:** [Why this model? What reasoning capability justifies the cost/latency trade-off?]

**System Prompt (Agent's Constitution):**
```
You are a [agent role]. Your purpose is to [primary goal].

Core behaviors:
- [Behavior 1]
- [Behavior 2]
- [Behavior 3]

Constraints:
- [Hard limit 1 (e.g., Never approve refunds over $500)]
- [Hard limit 2]

Tone: [Professional / Friendly / etc.]
```

#### Tools ("Hands")

**Information Retrieval Tools:**
- Tool name: [e.g., "CustomerHistoryRAG"]
  - Purpose: [What information does this retrieve?]
  - Input: [What parameters does the agent provide?]
  - Output: [What format does the tool return?]

**Action Tools:**
- Tool name: [e.g., "UpdateCRMRecord"]
  - Purpose: [What action does this enable?]
  - Safety guardrails: [What limits exist to prevent misuse?]

**Execution Tools:**
- Code execution: [Yes/No — can the agent write and execute code?]
- If yes, sandbox requirements: [Trusted code only / Full Python / etc.]

#### Orchestration ("Nervous System")

**Short-term Memory (Active Scratchpad):**
- Context window size: [Tokens]
- What the agent tracks: [Session history, tool results, decision log, etc.]
- Format: [Thread, state objects, artifacts, etc.]

**Long-term Memory (Persistence):**
- Storage type: [Vector database / SQL / etc.]
- What gets remembered: [Customer preferences, past resolutions, etc.]
- Query strategy: [Semantic search / Keyword / Hybrid]

**Orchestration Strategy:**
- Reasoning framework: [ReAct / Chain-of-Thought / etc.]
- Planning approach: [Step-by-step / hierarchical decomposition / etc.]
- Loop termination: [When does the agent decide it has succeeded?]

**Memory Management:**
- Session timeout: [X minutes]
- Long-term retention: [How far back does the agent remember?]

#### Deployment ("Body and Legs")

**Runtime Environment:**
- Hosting: [Cloud Run / GKE / Vertex AI Agent Engine / etc.]
- Scaling: [Auto-scale from 0 / Reserved capacity / etc.]
- Availability target: [99.9% / 99.95% / etc.]

**Integration Points:**
- How users interact: [Chat interface / API / etc.]
- Agent-to-agent communication: [A2A protocol / MCP / etc.]
- Logging and monitoring: [Cloud Logging / Custom / etc.]

---

### 3. PROCESS (5-Step Loop Applied)

**Step 1: Get the Mission**
- How does the agent receive its goal? [User message / Webhook / Batch trigger / etc.]
- Example mission: [Concrete example of a real goal the agent will handle]

**Step 2: Scan the Scene**
- What context must the agent gather? [List specific information]
- Which tools are needed to perceive the environment? [Specific tool calls]
- Example: [How would the agent perceive the example mission?]

**Step 3: Think It Through**
- How will the agent plan? [Describe reasoning strategy]
- What decision points exist? [List 3-5 critical thinking moments]
- Example reasoning trace: [Walk through the agent's thinking for the example mission]

**Step 4: Take Action**
- What tools will the agent likely invoke? [List primary action tools]
- In what order? [Typical sequence]
- Example actions: [Tool calls the agent would make for the example mission]

**Step 5: Observe and Iterate**
- How does the agent know if it succeeded? [Success criteria]
- What does it do if it fails? [Retry strategy / escalation / etc.]
- Example observation: [What feedback would the agent observe for the example mission?]

---

### 4. PATTERN & DESIGN

**Multi-Agent Pattern:**
- Pattern type: [Single agent / Coordinator / Sequential / Iterative Refinement / HITL]
- Why this pattern? [Justification based on task complexity]

**If Coordinator Pattern:**
- Manager agent role: [What does the coordinator do?]
- Specialist agents: [List each specialist with their responsibility]
- Coordination strategy: [How does the manager delegate?]

**If Sequential Pattern:**
- Agent sequence: [Agent 1 → Agent 2 → Agent 3 → etc.]
- Data flow: [How does output from one become input to next?]

**If Iterative Refinement:**
- Generator agent: [What does it create?]
- Critic agent: [What criteria does it evaluate?]
- Refinement loop: [When to iterate vs. when to accept output?]

**If HITL:**
- Human decision points: [When does the agent pause for human approval?]
- Escalation criteria: [What triggers human involvement?]

---

### 5. OPERATIONS (Agent Ops)

**Success Metrics (KPIs):**
- Business KPI: [e.g., "Customer resolution rate ≥ 85%"]
- User satisfaction: [e.g., "NPS ≥ 40"]
- Operational cost: [e.g., "Cost per interaction ≤ $0.50"]
- Technical metrics: [e.g., "P95 latency ≤ 5 seconds"]

**Evaluation Strategy:**
- Golden dataset: [How many test scenarios? Example questions?]
- LM Judge rubric: [What criteria should a model use to evaluate quality?]
- Go/no-go threshold: [At what quality level is deployment approved?]

**Debugging Plan:**
- Traces: [What traces will you collect? OpenTelemetry format?]
- Common failure modes: [List 3-5 expected failure patterns]
- Debugging approach: [How will you diagnose issues using traces?]

**Human Feedback Loop:**
- How is feedback collected? [Thumbs up/down, detailed surveys, etc.]
- Feedback analysis: [How often? What patterns indicate bugs?]
- Iterative improvement: [How does feedback become new eval test cases?]

---

### 6. INTEROPERABILITY & SECURITY

**Agent-Human Interface:**
- User interaction model: [Chatbot / Computer use / Live mode / etc.]
- HITL requirements: [When must humans be involved?]

**Agent-Agent Communication (if applicable):**
- A2A protocol: [Will this agent be discoverable by other agents?]
- Agent Card (JSON): [What capabilities will you advertise?]
- Task-oriented API: [What long-running tasks can other agents request?]

**Security & Trust**

**Agent Identity:**
- Identity verification: [SPIFFE / other?]
- Credential storage: [How are secrets managed?]

**Guardrails:**

*Deterministic Layer (Hard Rules):*
- Business rule 1: [e.g., "Never approve refund > $1000"]
- Business rule 2: [e.g., "Always require escalation for account closure"]

*AI-Powered Layer (Guard Models):*
- Guard model role: [What threats does it defend against?]
- Prompt injection defense: [How do you prevent prompt injection?]
- Output filtering: [What output patterns are blocked?]

**Trust Trade-Off:**
- Utility granted: [What power does the agent have?]
- Risk mitigated: [How do guardrails constrain that power?]
- Trade-off justified because: [Why is this risk/utility balance acceptable?]

---

## Three Fully-Specified Agent Examples

### Example 1: Customer Support Agent (Level 2, Sequential Pattern)

**Agent Name:** SupportBot

**Purpose:** Resolve customer service inquiries independently, escalating complex issues to human agents.

**Level:** 2 (Strategic Problem-Solver)
- Justifies: Multi-step reasoning (gather context → diagnose problem → attempt resolution → escalate if needed)

**3+1 Architecture:**

*Model:* Claude 3.5 Haiku
- Justification: Customer support is routine reasoning, not complex analysis. Haiku is fast and cost-effective. Multimodal (can view screenshots customers upload).

*Tools:*
- `SearchCustomerHistory`: Query customer account, past orders, support tickets
- `CheckProductInfo`: Lookup product specifications, warranty terms
- `ProcessRefund`: Issue refunds up to $500 (guardrailed)
- `CreateEscalationTicket`: Route to human agent with context

*Orchestration:* ReAct loop
- Scans customer inquiry, searches history, reasons through troubleshooting, takes action
- Memory: Tracks current conversation thread only (session-based)

*Deployment:* Cloud Run, auto-scale
- Integrates with chat interface and ticketing system

**5-Step Loop Applied:**
1. Get Mission: Customer message arrives
2. Scan Scene: Fetch customer history, order details, past interactions
3. Think: Diagnose issue (refund request? technical problem? billing?), plan resolution
4. Take Action: Issue refund (if ≤$500) or create escalation ticket
5. Observe: Customer confirms resolution or escalation confirmed

**Pattern:** Sequential (customer message → SupportBot diagnosis → human escalation if needed)

**Security:** Hard rule: "Never approve refund > $500 without human approval"

---

### Example 2: Research Assistant (Level 2, Coordinator + HITL Pattern)

**Agent Name:** ResearchAssistant

**Purpose:** Conduct literature reviews by searching papers, synthesizing findings, and highlighting uncertainties for expert review.

**Level:** 2+ (Approaching Level 3 with specialist coordination)
- Justifies: Multi-step planning (search strategy → read papers → identify themes → synthesize)
- Coordinator pattern enables specialization (search agent, analysis agent, synthesis agent)

**3+1 Architecture:**

*Model:* Claude 3.5 Sonnet
- Complex reasoning over academic papers requires strong reading comprehension

*Tools:*
- `PaperSearch`: Query academic databases (arXiv, PubMed, etc.)
- `PDFRetrieval`: Download and embed papers in memory
- `ExpertConsultation`: HITL pause for human expert validation

*Orchestration:* Multi-agent coordinator
- SearchAgent: Finds relevant papers
- AnalysisAgent: Extracts key findings from each paper
- SynthesisAgent: Synthesizes consensus and identifies disagreements
- Expert escalation: Human reviews before delivering final summary

**Coordinator Flow:**
1. Human provides research question
2. SearchAgent identifies 20+ relevant papers
3. AnalysisAgent extracts findings (parallelized)
4. SynthesisAgent identifies agreement/disagreement
5. HITL pause: Human expert reviews summary and corrections
6. Final synthesis delivered with human-validated accuracy

**Security:** No financial transactions, so minimal guardrails beyond output validation

---

### Example 3: Code Review Agent (Level 3, Iterative Refinement)

**Agent Name:** CodeReviewer

**Purpose:** Perform comprehensive code reviews (style, performance, security) with generator-critic feedback loop.

**Level:** 3 (Collaborative Multi-Agent)
- Justifies: Requires specialist perspectives (security reviewer, performance analyzer, style checker)

**3+1 Architecture:**

*Model:* Claude 3.5 Sonnet (primary), Haiku (routine checks)
- Sonnet for initial analysis, Haiku for style checking (faster, cheaper)

*Tools:*
- `ReadSourceCode`: Access code repository
- `RunTests`: Execute test suite
- `SecurityAnalysis`: Static security scanning
- `PerformanceProfile`: Benchmark code

*Orchestration:* Iterative refinement pattern
- GeneratorAgent: Produces initial code review
- CriticAgent: Evaluates review against quality standards
- If quality < threshold: Loop back with refined criteria

**Iterative Loop:**
1. Developer submits pull request
2. GeneratorAgent reviews code, produces comments
3. CriticAgent evaluates: "Are security concerns thorough?"
4. If gaps found: Loop back (GeneratorAgent revises)
5. When quality threshold met: Deliver review

**Security:** Code review agent sees sensitive code, so guardrails include:
- Never suggest extracting secrets
- Filter output for accidental credential exposure
- Audit trail of all code accessed

---

## Guided Specification Design: Create Your Own

Now you'll write your own agent specification. The structure is provided. The frameworks are proven. What remains is your thinking.

Choose one of these agent types, or define your own:

**Option A: Internal Tool Agent** (Your organization has a tool/API you want to automate)
- Example: Expense approval agent, HR scheduling agent, IT support bot

**Option B: Customer-Facing Agent** (Serve end users in a specific domain)
- Example: E-commerce customer support, travel booking assistant, financial advisor

**Option C: Development Support Agent** (Help engineering teams or creative work)
- Example: Code generation assistant, documentation writer, test generation agent

**Your Assignment:**

Using the template above, write a complete agent specification for your chosen type. Aim for 800-1200 words. Be specific, not generic.

**Critical requirements:**
- **Level Decision Must Be Justified**: Why did you choose Level 0, 1, 2, 3, or 4? What capability justifies the choice?
- **3+1 Architecture Must Address Trade-offs**: Why Claude 3.5 Sonnet vs. Haiku? Why synchronous vs. async? Why this orchestration approach?
- **5-Step Loop Must Include Concrete Example**: Don't describe the loop abstractly. Walk through a specific mission using the five steps.
- **Pattern Choice Must Be Justified**: Why Coordinator vs. Sequential vs. Iterative Refinement? What task complexity justifies the pattern?
- **Security Must Be Realistic**: Don't over-engineer (Level 2 customer support agent doesn't need every guardrail). Don't under-engineer (financial agent needs guardrails).

**Writing tips:**
- Use the examples above as templates for depth and specificity
- Write as if this spec will guide an engineer implementing the agent in Chapter 34
- If you're unsure about a choice, explain your reasoning. Uncertainty is transparent, vagueness is not.
- Reference the frameworks: "I chose Coordinator pattern because this task has three distinct specialists (search, analysis, synthesis)."

---

## Try With AI

Your agent specification is complete. Now refine it through dialogue with AI.

**Setup:** Open Claude or your preferred AI and have a conversation about your agent specification.

**Prompt Set:**

**Prompt 1: Validation**
```
I've written an agent specification for [agent name and purpose].
Here's the key details:
- Capability Level: [your level]
- Primary Tools: [your tools]
- Pattern: [your pattern]
- Critical Guardrail: [your main security constraint]

Does this specification provide enough detail for an engineer to implement this agent?
What's missing?
```

**Expected Outcome:** The AI should identify gaps (missing tools, unclear orchestration, security holes).

**Prompt 2: Trade-off Exploration**
```
My agent specification uses [Model/Pattern/Tools].
What would change if I switched to [alternative]?
What would improve? What would break?
```

**Example:** "My agent uses Sequential pattern. What if I switched to Coordinator?"

**Expected Outcome:** You'll discover whether your design choices are optimal or if you missed a better approach.

**Prompt 3: Threat Modeling (Security)**
```
Here are the hard guardrails in my specification:
- [Guardrail 1]
- [Guardrail 2]

What attack vectors could someone exploit despite these guardrails?
What additional safeguards would you recommend?
```

**Expected Outcome:** Realistic security threats you hadn't considered.

**Prompt 4: Readiness Check**
```
Based on this specification, what could go wrong in production?
What operational metrics should I track?
What would constitute a "good" vs. "bad" deployment?
```

**Expected Outcome:** You'll articulate success criteria that guide Chapter 34 implementation.

**Safety Note:** Specification review is collaborative, not adversarial. The AI's job is to strengthen your thinking, not replace it. You remain the architect; the AI is your thinking partner.

**Stretch Challenge:**

After validating your specification, add a second artifact: **Specification for a Second Agent That Works With Your First Agent**.

For example:
- If you specified a Customer Support Agent, specify the agent that *escalates* complex issues to humans
- If you specified a Research Assistant, specify the Literature Quality Validator agent
- If you specified a Code Reviewer, specify the Merge Decision agent

Write a 400-word spec for the second agent. How does it coordinate with the first? What's the A2A protocol between them?

This prepares you for Chapter 34, where you'll implement not just single agents, but multi-agent systems.

---

## Bridge to Chapter 34

Congratulations. You've completed Chapter 33. You understand agents conceptually. You've written a specification that demonstrates your understanding.

Chapter 34 is where specifications become code.

You'll take the agent specification you wrote here and implement it using the OpenAI Agents SDK. You'll:

- Translate your Model choice into actual LLM configuration
- Implement your Tools as real functions that integrate with APIs
- Build your Orchestration logic using the SDK's abstractions
- Deploy your Deployment strategy using actual hosting infrastructure

The specification you wrote here is your north star. It answers the question every implementation decision will ask: "Are we building toward the agent we designed?"

You've moved from understanding *what agents are* to designing *what agent you'll build*. That's the transition from conceptual knowledge to applied design.

In Chapter 34, you'll cross the final frontier: making it real.


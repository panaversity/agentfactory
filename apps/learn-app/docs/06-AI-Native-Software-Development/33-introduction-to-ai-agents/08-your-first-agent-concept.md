---
title: "Your First Agent Concept"
sidebar_position: 8
description: "Design your first agent specification using all frameworks from this chapter."
proficiency_level: B1
cognitive_load:
  new_concepts: 0
  estimated_difficulty: B1
estimated_time: 30 minutes
learning_objectives:
  - "Design an agent specification using chapter frameworks"
  - "Apply 5-Level Taxonomy, 3+1 Architecture, patterns, and security"
  - "Create specifications that guide implementation"
skills:
  agent_specification:
    proficiency: B1
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 2.0.0
---

# Your First Agent Concept

You've mastered the foundational frameworks. Now synthesize them by designing your own agent specification.

This lesson is not a tutorial—it's your opportunity to apply everything you've learned. By the end, you'll have written a complete specification that could guide implementation in Chapters 34-36.

## Specification-First Thinking

Traditional software specs describe *what code does*:
"Read file, parse JSON, validate schema, return result."

Agent specs describe *how the agent thinks and behaves*:
"You are a customer support agent. Your goal is to resolve inquiries with empathy. You can issue refunds up to $500. Escalate anything larger."

The difference is profound. Traditional specs prescribe a deterministic path. Agent specs define constraints within which the agent reasons to find its own path.

This makes agent specs harder to write. You must:
- Anticipate failure modes without prescribing solutions
- Set boundaries without removing flexibility
- Enable competence without dictating steps

The frameworks you've learned help structure this thinking.

## Agent Specification Template

Use this structure for any agent:

### Section 1: Purpose & Capability Level

**Agent Name**: [Clear, memorable name]

**Purpose**: [One paragraph: What does this agent accomplish? Why does it matter?]

**Capability Level** (5-Level Taxonomy):
- **Level 0**: Simple processor (no reasoning)
- **Level 1**: Router with retrieval (RAG)
- **Level 2**: Strategic planner (multi-step reasoning)
- **Level 3**: Multi-agent coordinator
- **Level 4**: Self-evolving (creates tools/agents)

**Why this level?** [Justify based on task complexity]

### Section 2: Architecture (3+1)

**Model (Brain)**:
- Which LLM? Why?
- Cost/capability trade-off reasoning
- Multimodal requirements?

**Tools (Hands)**:
- Information retrieval tools (what can it read?)
- Action tools (what can it do?)
- Guardrails on each tool (what limits?)

**Orchestration (Nervous System)**:
- Memory strategy (session-only vs persistent)
- Reasoning approach (ReAct, Chain-of-Thought, Reflection)
- Success criteria (how does it know when it's done?)

**Deployment (Body)**:
- Where does it run?
- How do users access it?
- Scaling requirements?

### Section 3: Process (5-Step Loop)

Walk through a concrete example using the 5-Step Loop:

1. **Get Mission**: How does the agent receive its goal?
2. **Scan Scene**: What context must it gather?
3. **Think Through**: How does it plan?
4. **Take Action**: What tools does it use?
5. **Observe & Iterate**: How does it verify success?

### Section 4: Pattern

**Multi-agent pattern**: [Single / Coordinator / Sequential / Iterative Refinement / HITL]

**Why this pattern?** [Justify based on task structure]

If multi-agent:
- What does each agent do?
- How do they coordinate?
- What's the data flow?

### Section 5: Security

**Deterministic guardrails** (hard limits):
- [Rule 1]
- [Rule 2]

**Guard model checks** (contextual evaluation):
- [Check 1]
- [Check 2]

**Trust trade-off**: What power does it have? How is risk mitigated?

**Compromise plan**: What happens if this agent is breached?

## Example Specification: Customer Support Agent

### Purpose & Level

**Agent Name**: SupportBot

**Purpose**: Resolve customer service inquiries autonomously, escalating complex or high-value issues to human agents. Reduce support queue while maintaining customer satisfaction.

**Capability Level**: 2 (Strategic planner)

**Why Level 2**: Requires multi-step reasoning: gather customer context → diagnose problem → determine resolution path → execute or escalate. Doesn't need multi-agent coordination—a single agent with good tools handles this scope.

### Architecture

**Model**: Claude 3.5 Haiku
- Justification: Customer support is routine reasoning with high volume. Haiku balances speed, cost, and capability. Multimodal enabled for screenshot analysis when customers share images.

**Tools**:
- `SearchCustomerHistory`: Query orders, past tickets, account status
- `CheckProductInfo`: Lookup specs, warranty, return policy
- `ProcessRefund`: Issue refunds ≤$500 (hard limit in tool)
- `CreateEscalation`: Route to human with full context

**Orchestration**:
- Memory: Session-based (current conversation only)
- Reasoning: ReAct loop with explicit planning
- Success: Issue resolved OR escalation created with context

**Deployment**:
- Cloud Run with auto-scaling
- Chat widget on customer portal
- API for integration with existing ticketing system

### Process (5-Step Loop Example)

**Scenario**: Customer asks "I ordered a laptop but received a tablet. Order #45678."

**Step 1 - Get Mission**: Resolve wrong-item shipment for order #45678.

**Step 2 - Scan Scene**:
- Call `SearchCustomerHistory("45678")` → Order found: laptop ordered, tablet shipped
- Call `CheckProductInfo("laptop_sku")` → In stock

**Step 3 - Think Through**:
- Wrong item confirmed (laptop vs tablet)
- Options: Refund + new order, or exchange
- Customer wants the laptop → exchange is best
- Need shipping label for return

**Step 4 - Take Action**:
- Call `CreateExchange(order_id="45678", return_item="tablet", ship_item="laptop")`
- Generate return shipping label
- Send confirmation email

**Step 5 - Observe**:
- Exchange created successfully
- Customer has return label and new delivery estimate
- Mission complete

### Pattern

**Pattern**: Single agent with Human-in-the-Loop escalation

**Why**: Task complexity doesn't require multiple specialists. Single agent handles most cases. HITL triggers for: refunds >$500, account closure, legal threats, angry customers (detected by sentiment).

### Security

**Deterministic guardrails**:
- Refund cap: $500/transaction, $2000/day per agent
- Order access: Only orders matching authenticated customer
- No account deletion capability

**Guard model checks**:
- Before refund: Is request pattern consistent with legitimate use?
- Email content: Scan for phishing patterns before sending
- Escalation: Flag if customer mentions legal action or media

**Trust trade-off**: Agent has financial authority (refunds) and customer data access. Risk mitigated by caps, authentication, and audit logging.

**Compromise plan**: Revoke API credentials → Review audit log for unauthorized refunds → Customer notification if data accessed → Maximum exposure: $2000/day × rate limit.

## Your Assignment

Choose an agent type and write a complete specification:

**Option A: Internal Tool Agent**
- Expense approval bot
- IT support assistant
- HR scheduling agent

**Option B: Customer-Facing Agent**
- E-commerce support
- Travel booking assistant
- Financial advisor

**Option C: Development Support Agent**
- Code review assistant
- Documentation writer
- Test generation agent

### Requirements

Your specification must include:

1. **Purpose and capability level** with justification (why this level?)
2. **3+1 Architecture** with trade-off reasoning (why this model? why these tools?)
3. **5-Step Loop walkthrough** with a concrete example (not abstract)
4. **Pattern choice** with justification (why not other patterns?)
5. **Security design** realistic for the use case (not over-engineered, not under-engineered)

### Writing Tips

- Write as if this spec will guide an engineer implementing the agent
- Be specific: "refund up to $500" not "can issue refunds"
- Justify decisions: "Claude Haiku because high volume + routine reasoning" not just "Claude Haiku"
- Include a realistic example—actually walk through the 5-Step Loop
- Reference the frameworks: "Level 2 because multi-step planning required but not multi-agent coordination"

## Try With AI

Use Claude, ChatGPT, or Gemini to validate and strengthen your specification.

> "I've written an agent spec with Capability Level [X], Tools [Y], Pattern [Z], and Guardrails [W]. Does this provide enough detail for an engineer to implement? What's missing?"

**Expected**: The AI identifies gaps—missing tools, unclear orchestration, security holes. Use this to strengthen your spec.

> "My agent specification uses [Sequential/Coordinator/etc] pattern. What would change if I switched to [alternative]? What would improve? What would break?"

**Expected**: Trade-off exploration. Discover whether your design choice is optimal or if there's a better pattern.

> "Here are the guardrails in my specification: [list them]. What attack vectors could someone exploit despite these guardrails? What additional safeguards would you recommend?"

**Expected**: Threat modeling you hadn't considered. Realistic security analysis.

> "Based on my specification, what could go wrong in production? What operational metrics should I track? What constitutes success vs failure?"

**Expected**: Production readiness thinking. Agent Ops criteria that guide implementation.

> "Walk through my 5-Step Loop example and identify where my agent might fail. What happens if the tool call fails? What if the customer changes their request mid-conversation?"

**Expected**: Edge case analysis. Robustness thinking.

**Key insight**: The spec is your thinking made explicit. If you can't specify it clearly, you don't understand it well enough to build it. Use AI to find the gaps in your understanding.

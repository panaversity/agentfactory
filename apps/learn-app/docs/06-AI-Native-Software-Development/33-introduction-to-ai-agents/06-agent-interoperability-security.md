---
title: "Agent Interoperability & Security"
sidebar_position: 6
description: "How agents communicate (A2A Protocol) and how to secure them (identity, guardrails, defense in depth)."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 20 minutes
learning_objectives:
  - "Explain how agents discover and communicate with each other"
  - "Design security frameworks using identity, guardrails, and guard models"
skills:
  agent_security:
    proficiency: B1
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 2.0.0
---

# Agent Interoperability & Security

Agents need to work together and stay secure. This lesson covers how agents communicate across systems and how to protect them from misuse.

## Agent-to-Agent Communication

### The Problem

You build a customer support agent. It needs to:
- Check order status (OrderAgent)
- Process refunds (BillingAgent)
- Schedule callbacks (CalendarAgent)

How do these agents find each other? How do they communicate? How does one agent know what another can do?

### A2A Protocol

The Agent-to-Agent (A2A) protocol standardizes agent communication.

**Core concepts:**

**Agent Cards**: JSON descriptions of capabilities
```json
{
  "name": "BillingAgent",
  "description": "Handles billing operations",
  "capabilities": ["process_refund", "query_balance", "update_payment"],
  "inputs": {
    "process_refund": {
      "customer_id": "string",
      "amount": "number",
      "reason": "string"
    }
  },
  "limits": {
    "max_refund": 500,
    "rate_limit": "100/hour"
  },
  "endpoint": "https://billing.internal/agent"
}
```

Agent Cards answer: "What can this agent do? What does it need? What are its limits?"

**Task Delegation**: One agent asks another to perform work
```
SupportAgent → BillingAgent: "Process refund $50 for customer 123, reason: damaged item"
BillingAgent: {accepts task, validates inputs}
BillingAgent: {processes refund}
BillingAgent → SupportAgent: "Refund complete. Confirmation: RF-456"
```

**Progress Updates**: Streaming status for long tasks
```
SupportAgent → ResearchAgent: "Compile report on customer trends Q4"
ResearchAgent: {status: "accepted"}
ResearchAgent: {status: "gathering data", progress: 25%}
ResearchAgent: {status: "analyzing", progress: 60%}
ResearchAgent: {status: "complete", result: [report]}
```

### Why A2A Matters

Without standards, every agent integration is custom. With A2A:
- Agents discover each other's capabilities automatically
- Communication follows predictable patterns
- Error handling and retries work consistently
- New agents plug into existing ecosystems

## Agent Security

### Agents as a New Principal Class

Traditional security has two principals:
- **Users**: Humans with identities and permissions
- **Services**: Software with API keys and scopes

Agents are a third class. They:
- Act autonomously on behalf of systems
- Make decisions without human approval
- Have their own identity and permissions
- Can be compromised or manipulated

**Why this matters**: A compromised agent can do damage proportional to its permissions. Security design limits that blast radius.

### The Trust Trade-Off

Every capability given to an agent introduces risk.

| Capability | Utility | Risk |
|------------|---------|------|
| Read customer data | Personalized service | Data exposure |
| Issue refunds | Autonomous resolution | Financial loss |
| Execute code | Powerful automation | System compromise |
| Access external APIs | Rich integrations | Data exfiltration |

Security design balances utility against risk. An agent with no capabilities is useless. An agent with unlimited capabilities is dangerous.

### Defense in Depth

Two layers of protection work together:

**Layer 1: Deterministic Guardrails**

Hard limits enforced by infrastructure. Can't be bypassed by clever prompting.

Examples:
- **Rate limits**: 100 requests per minute maximum
- **Resource caps**: $500 maximum per refund
- **Scope limits**: Can only access customer database, not employee database
- **Action restrictions**: Can read files but not delete them

```
BillingAgent attempts: process_refund($750)
Infrastructure blocks: "Amount exceeds $500 limit"
```

The agent can't talk its way past these limits. They're enforced at the infrastructure layer, not the reasoning layer.

**Layer 2: AI-Powered Guard Models**

A separate LLM evaluates agent actions for context-dependent risks.

```
Agent wants to: Send email to customer
Guard model checks:
  - Is the content appropriate? ✓
  - Is the recipient a valid customer? ✓
  - Does this look like spam/phishing? ✓
Decision: Allow
```

```
Agent wants to: Send email to 10,000 addresses
Guard model checks:
  - Is this bulk send authorized? ✗
  - Does this match normal patterns? ✗
Decision: Block, escalate for review
```

Guard models catch context-dependent issues that hard rules can't anticipate:
- Social engineering attempts
- Unusual patterns (agent suddenly accessing files it never touched before)
- Output that's technically allowed but inappropriate

### Agent Identity

Each agent needs verifiable identity:

**Credentials**: How the agent proves who it is
- API keys, certificates, or tokens
- Rotated regularly
- Scoped to specific capabilities

**Permissions**: What the agent can do
- Principle of least privilege: only what's needed
- Explicit grants, not implicit access
- Reviewed and audited regularly

**Audit Log**: What the agent has done
- Every action recorded
- Traceable to specific agent identity
- Enables forensics when things go wrong

**Compromise response**: When an agent is compromised:
1. Revoke its credentials immediately
2. Review audit logs to assess damage
3. Blast radius limited to that agent's permissions
4. Other agents unaffected

This is why least-privilege matters. An agent with access to everything makes a breach catastrophic. An agent with minimal access makes a breach containable.

## Designing Secure Agent Systems

Follow this process for any agent you build:

### Step 1: Define Capabilities
What does this agent need to do? List specific actions.

Example for a Support Agent:
- Read customer order history
- Issue refunds up to $500
- Create escalation tickets
- Send confirmation emails

### Step 2: Identify Risks
For each capability, what could go wrong?

| Capability | Risk | Severity |
|------------|------|----------|
| Read order history | Data exposure to wrong customer | Medium |
| Issue refunds | Fraudulent refunds | High |
| Create tickets | Spam/abuse | Low |
| Send emails | Phishing, spam | Medium |

### Step 3: Set Deterministic Guardrails
Hard limits for high-severity risks:

- Refund cap: $500 per transaction, $2000 per day
- Order access: Only orders matching authenticated customer
- Email rate: Maximum 10 emails per hour

### Step 4: Design Guard Model Checks
Contextual evaluation for medium-severity risks:

- Before sending email: Check content for phishing patterns
- Before refund: Verify refund reason matches order issues
- Unusual patterns: Alert if accessing orders outside normal hours

### Step 5: Plan for Compromise
What happens if this agent is compromised?

- Credential rotation: Can we revoke and reissue quickly?
- Blast radius: What's the maximum damage with current permissions?
- Detection: How would we know if the agent is acting maliciously?
- Recovery: How do we undo damage?

## Try With AI

Use Claude, ChatGPT, or Gemini to design agent security.

> "A BillingAgent can issue refunds up to $500 and query account balances. Design its Agent Card: name, capabilities, inputs, outputs, and guardrails."

**Expected**: Structured Agent Card with capabilities (process_refund, query_balance), inputs (customer_id, amount, reason), outputs (confirmation_id, status), guardrails ($500/transaction, $5000/day, rate limits).

> "If BillingAgent is compromised, what's the maximum damage an attacker could do? How does agent identity help contain it?"

**Expected**: Maximum damage = $500 × rate limit. Credential revocation stops attack immediately. Audit log identifies abuse. Other agents unaffected.

> "A billing action over $250 should go through a guard model before execution. What should the guard model check? Give me 5 specific checks."

**Expected**: Is amount reasonable for this customer's history? Is this a duplicate of recent request? Does refund reason match order data? Is request timing suspicious? Does the request come from a verified session?

> "Design a defense-in-depth strategy for an agent that can execute code in a sandbox. What deterministic guardrails and guard model checks would you implement?"

**Expected**: Guardrails: Sandboxed environment, no network access, execution timeout, memory limits. Guard checks: Does code match expected patterns? Is it trying to access sensitive data? Does it attempt to escape sandbox?

**Key insight**: Every capability is a risk. Security design manages the trust trade-off—enough power to be useful, enough limits to be safe.

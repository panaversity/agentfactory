---
sidebar_position: 13
title: "Cost Tracking and Billing"
description: "Track per-message token usage, extract total costs from agent executions, and design billing models that enable Digital FTE monetization through subscription, success fee, and usage-based pricing."
keywords: [cost tracking, billing, token usage, total_cost_usd, monetization, pricing models, Digital FTE]
chapter: 36
lesson: 13
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Real-Time Token and Cost Tracking"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can extract per-message token usage from message streams, aggregate costs across sessions, and implement cost tracking dashboards for agent operations"

  - name: "Total Cost USD Extraction and Aggregation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can extract total_cost_usd from result messages, understand cost calculation methodology, and differentiate between per-message and cumulative costs"

  - name: "Digital FTE Billing Model Design"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can design billing models aligned with cost structure (subscription, success fee, usage-based), calculate break-even pricing, and optimize cost-to-revenue ratio for Digital FTE monetization"

learning_objectives:
  - objective: "Extract per-message token usage and total_cost_usd from Claude Agent SDK message streams"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements cost tracking that captures every message's token count and cost contribution"

  - objective: "Understand how Claude Agent SDK calculates and reports total cost across agent executions"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student explains the relationship between per-message costs, token pricing tiers, and cumulative total_cost_usd"

  - objective: "Design subscription, success-fee, and usage-based billing models for Digital FTE monetization"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Student evaluates pricing models against target margin (60%+), customer acquisition cost, and break-even timeline"

  - objective: "Implement cost optimization strategies to improve Digital FTE margin through max_turns, model selection, and context pruning"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student demonstrates cost reduction tactics that maintain quality while improving profitability"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (per-message usage tracking, total_cost_usd, subscription billing, success-fee model, usage-based pricing, cost optimization) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design tiered pricing with volume discounts; analyze competitor pricing (OpenAI, Google); optimize profit margins for different customer segments; model customer lifetime value and churn scenarios"
  remedial_for_struggling: "Focus on single concept: 'Every message costs money. The SDK tells you exactly how much.' Establish cost-to-revenue relationship before multi-model comparisons; use concrete pricing scenarios (e.g., $10/month subscription with $0.50 cost per customer per month = 95% margin)"
---

# Cost Tracking and Billing

Your Digital FTE will work 24/7. It will process thousands of requests. Every API call costs money. Every token sent to Claude costs money.

If you don't track these costs precisely, you'll ship a product that loses money on every customer.

This lesson teaches you to see the economics of your agent—and design pricing that ensures profitability.

## The Economics Problem

When you run an agent, costs accumulate invisibly:

- **Per-message cost**: Each message to Claude API costs based on tokens sent and received
- **Model selection cost**: Haiku (2-3x cheaper) vs Opus (5-10x more expensive)
- **Context window cost**: Longer context = more tokens = higher cost
- **Iteration cost**: More turns (max_turns) = more messages = more cost

A customer using your Digital FTE for 10 interactions doesn't think about the cost. But you should: each interaction might cost $0.05, $0.50, or $5.00 depending on how efficiently your agent works.

**The margin problem**: If your customer pays $99/month for your Digital FTE, but it costs you $150 in API calls to service them, you lose money. You need cost visibility.

**The pricing problem**: You can't price intelligently without knowing your cost basis. Subscription? Pay-per-use? Success fee? Your choice depends on understanding what your agent actually costs to run.

## The Solution: Real-Time Cost Tracking

The Claude Agent SDK gives you exact cost data in every message. You extract it. You track it. You make it visible.

### Specification: Cost Tracking System

**Intent**: Capture every dollar your Digital FTE costs so you can track profitability per customer, per workflow, and overall.

**Success Criteria**:
- ✓ Per-message token usage visible as messages stream
- ✓ Total cost accumulated across full agent execution
- ✓ Cost tracked by customer, workflow, or time period
- ✓ Cost alerts when exceeding thresholds
- ✓ Historical cost data supports pricing analysis

**Constraints**:
- Must extract costs in real-time (no post-execution analysis delays)
- Must handle partial message streams (agent may be interrupted)
- Must distinguish model costs (Haiku 2-3x cheaper than Opus)

### Pattern: Extracting Per-Message Usage

Every message in the agent stream has cost data. You iterate through the stream and extract it:

```python
import asyncio
from claude_agent_sdk import query, ClaudeAgentOptions

async def track_agent_cost():
    """Track costs as agent executes in real-time."""
    total_tokens = 0
    total_cost = 0
    message_costs = []

    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash", "Glob"],
        permission_mode="acceptEdits"
    )

    async for message in query(
        prompt="Analyze the codebase structure and identify test coverage gaps",
        options=options
    ):
        # Extract per-message token usage
        if message.type == "assistant" and hasattr(message, "usage"):
            input_tokens = message.usage.get("input_tokens", 0)
            output_tokens = message.usage.get("output_tokens", 0)
            message_total = input_tokens + output_tokens
            total_tokens += message_total

            # Store for analysis
            message_costs.append({
                "message_id": getattr(message, "id", None),
                "input_tokens": input_tokens,
                "output_tokens": output_tokens,
                "total_tokens": message_total,
                "timestamp": getattr(message, "timestamp", None)
            })

            print(f"Message {len(message_costs)}: {message_total} tokens")

        # Extract final cost after execution completes
        if message.type == "result" and hasattr(message, "total_cost_usd"):
            total_cost = message.total_cost_usd
            print(f"\nExecution Summary:")
            print(f"  Total tokens: {total_tokens}")
            print(f"  Total cost: ${total_cost:.4f}")

    return {
        "total_tokens": total_tokens,
        "total_cost_usd": total_cost,
        "message_breakdown": message_costs
    }

# Run and see real costs
result = asyncio.run(track_agent_cost())
print(f"\nFinal cost: ${result['total_cost_usd']:.4f}")
```

**Output:**
```
Message 1: 1247 tokens
Message 2: 892 tokens
Message 3: 1501 tokens

Execution Summary:
  Total tokens: 3640
  Total cost: $0.0438
```

### Pattern: Understanding total_cost_usd ★

The `total_cost_usd` field is the key metric for monetization. It's the final answer to "How much did this agent execution cost?"

```python
async def billing_from_costs():
    """Extract costs and map to customer billing."""
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Grep"],
        permission_mode="acceptEdits"
    )

    async for message in query(
        prompt="Find all TODO comments in the project",
        options=options
    ):
        # Only process the final result message
        if message.type == "result" and hasattr(message, "total_cost_usd"):
            agent_cost = message.total_cost_usd

            # Example: Subscription with 60% margin requirement
            customer_id = "customer_acme_corp"
            monthly_budget = 99.00  # What customer pays monthly
            max_cost_per_execution = monthly_budget * 0.40  # Reserve 40% margin

            if agent_cost > max_cost_per_execution:
                print(f"WARNING: Cost ${agent_cost:.4f} exceeds budget ${max_cost_per_execution:.4f}")
                print("This customer needs a premium plan or optimization")
            else:
                print(f"Cost ${agent_cost:.4f} - Margin preserved ✓")

            return {
                "customer_id": customer_id,
                "execution_cost": agent_cost,
                "margin_status": "OK" if agent_cost < max_cost_per_execution else "EXCEEDED"
            }

# This is how billing systems work: cost → margin calculation → pricing decision
asyncio.run(billing_from_costs())
```

**Output:**
```
Cost $0.0438 - Margin preserved ✓
```

### Pattern: Cost Aggregation by Customer

In production, you track costs per customer to understand profitability:

```python
from datetime import datetime, timedelta
import json

class CustomerBillingTracker:
    """Track agent costs per customer for subscription billing."""

    def __init__(self, storage_path: str = "billing_data.json"):
        self.storage_path = storage_path
        self.data = self._load_data()

    def _load_data(self) -> dict:
        """Load existing billing data."""
        try:
            with open(self.storage_path, "r") as f:
                return json.load(f)
        except FileNotFoundError:
            return {}

    def _save_data(self):
        """Persist billing data."""
        with open(self.storage_path, "w") as f:
            json.dump(self.data, f, indent=2)

    def record_execution(self, customer_id: str, cost_usd: float, execution_type: str):
        """Record a single agent execution cost for a customer."""
        if customer_id not in self.data:
            self.data[customer_id] = {
                "total_cost": 0,
                "executions": [],
                "created_date": datetime.now().isoformat()
            }

        self.data[customer_id]["total_cost"] += cost_usd
        self.data[customer_id]["executions"].append({
            "cost": cost_usd,
            "type": execution_type,
            "timestamp": datetime.now().isoformat()
        })

        self._save_data()
        return self.data[customer_id]

    def monthly_cost(self, customer_id: str, month_back: int = 0) -> float:
        """Calculate customer's total cost for a specific month."""
        if customer_id not in self.data:
            return 0.0

        target_date = datetime.now() - timedelta(days=30 * month_back)
        month_start = target_date.replace(day=1)
        month_end = (month_start + timedelta(days=32)).replace(day=1)

        total = 0.0
        for execution in self.data[customer_id]["executions"]:
            exec_date = datetime.fromisoformat(execution["timestamp"])
            if month_start <= exec_date < month_end:
                total += execution["cost"]

        return total

    def profitability(self, customer_id: str, subscription_price: float) -> dict:
        """Calculate if customer is profitable."""
        monthly_cost = self.monthly_cost(customer_id)
        margin = subscription_price - monthly_cost
        margin_pct = (margin / subscription_price * 100) if subscription_price else 0

        return {
            "customer_id": customer_id,
            "monthly_cost": round(monthly_cost, 4),
            "subscription_price": subscription_price,
            "margin": round(margin, 2),
            "margin_pct": round(margin_pct, 1),
            "profitable": margin > 0
        }

# Usage in billing system
tracker = CustomerBillingTracker()

# After each agent execution, record the cost
tracker.record_execution(
    customer_id="customer_acme_corp",
    cost_usd=0.0438,
    execution_type="code_analysis"
)
tracker.record_execution(
    customer_id="customer_acme_corp",
    cost_usd=0.0312,
    execution_type="documentation_check"
)

# Check profitability
profitability = tracker.profitability(
    customer_id="customer_acme_corp",
    subscription_price=99.00
)
print(f"Customer margin: ${profitability['margin']:.2f} ({profitability['margin_pct']}% margin)")
```

**Output:**
```
Customer margin: $98.93 (99.9% margin)
```

This is healthy profitability. Costs stay well below subscription price.

## Billing Models for Digital FTEs

With cost visibility, you can price intelligently. Different models work for different agents:

### Model 1: Subscription Billing

**How it works**: Fixed monthly price. Customer uses agent as much as they want.

**Best for**: Predictable workflows (code review, security analysis, content moderation)

**Pricing calculation**:
```
Monthly subscription = (Target margin % × Average monthly cost per customer) / (1 - Target margin %)

Example:
- Average customer uses agent 20 times/month
- Cost per execution: $0.05
- Monthly cost per customer: $1.00
- Target margin: 70%
- Monthly subscription: $1.00 / (1 - 0.70) = $3.33

Set subscription at $9.99/month → 90% margin
```

**Pros**: Predictable revenue, simple billing
**Cons**: Requires cost discipline; high-usage customers become margin drains

### Model 2: Success-Fee Billing

**How it works**: Customer pays commission on value delivered (e.g., 2% of savings, $5 per lead)

**Best for**: High-ROI agents (lead generation, cost savings, revenue impact)

**Pricing calculation**:
```
Success fee = Value delivered × Commission %

Example: Legal contract review
- Agent saves customer $500 per contract reviewed (avoids expensive mistakes)
- Commission: 2% × $500 = $10 per contract
- Cost to run agent: $0.15
- Margin: $10 - $0.15 = $9.85 (98%+)
```

**Pros**: Unlimited upside, aligned incentives, justifiable pricing
**Cons**: Requires outcome tracking, customer trust

### Model 3: Usage-Based Billing

**How it works**: Customer pays per execution or per token consumed

**Best for**: Variable-workload agents (document analysis, research, real-time classification)

**Pricing calculation**:
```
Price per execution = (Cost per execution) / (1 - Desired margin %)

Example:
- Cost to run: $0.25
- Desired margin: 70%
- Price: $0.25 / (1 - 0.70) = $0.83 per execution
- Set price at $1.00 per execution → 75% margin
```

**Pros**: Fair to customers, scales with usage, easy to implement
**Cons**: Customer cost uncertainty, requires usage tracking

### Model 4: Hybrid Billing

**How it works**: Base subscription + usage overage fees

**Best for**: Most Digital FTEs (subscriptions handle baseline, overages capture high-volume users)

**Pricing calculation**:
```
Base subscription: $49/month (covers 50 executions)
Overage: $0.50 per execution beyond 50

Customer A (10 executions): Pays $49
Customer B (100 executions): Pays $49 + (50 × $0.50) = $74
Average margin: 65-75%
```

**Pros**: Predictable base revenue, captures high-value customers
**Cons**: More complex billing, requires tracking

## Cost Optimization: Maximizing Margin

Lower costs = higher margins. Here are the levers:

### Optimization 1: max_turns to Prevent Runaway Costs

An agent with unlimited iterations can spiral into expensive loops. Set `max_turns` to bound costs:

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Bash", "Glob"],
    max_turns=5,  # Maximum 5 message iterations
    permission_mode="acceptEdits"
)

async for message in query(
    prompt="Debug the authentication service",
    options=options
):
    if message.type == "result":
        # Agent stops after 5 turns, even if incomplete
        # This guarantees cost ≤ (Cost per message × 5)
        total_cost = message.total_cost_usd
        print(f"Bounded cost execution: ${total_cost:.4f}")
```

**Effect**: Reduces cost variability. Some tasks may fail, but costs are predictable.

### Optimization 2: Model Selection by Task

Use cheaper models for simple tasks, expensive models for complex reasoning:

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Grep"],
    agents={
        "classifier": AgentDefinition(
            description="Simple classification task",
            prompt="Classify this document as: [urgent|normal|low-priority]",
            tools=["Read"],
            model="haiku"  # 2-3x cheaper, fast for simple tasks
        ),
        "analyzer": AgentDefinition(
            description="Complex reasoning required",
            prompt="Analyze security implications of this code",
            tools=["Read", "Grep"],
            model="sonnet"  # Better reasoning for complex analysis
        )
    }
)

# Classification: $0.01 per execution (using Haiku)
# Security analysis: $0.08 per execution (using Sonnet)
# Overall: 2x cheaper than using Sonnet for everything
```

**Effect**: Reduces cost by 50-75% for simple tasks by model selection.

### Optimization 3: Context Pruning

Longer context = more tokens = higher cost. Remove old messages:

```python
from claude_agent_sdk import ClaudeSDKClient

async def prune_context_for_cost():
    """Long-running agent that prunes old messages to save costs."""
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash"],
        # Add setting to auto-prune old messages
        extra_args={"context-pruning-strategy": "aggressive"}
    )

    async with ClaudeSDKClient(options) as client:
        # First task: large project analysis
        await client.query("Analyze project structure")
        async for msg in client.receive_response():
            print(f"Cost: {msg.total_cost_usd:.4f}")

        # Prune old messages before next task
        # This keeps context window small and costs down
        await client.prune_messages()

        # Second task: much cheaper because context is smaller
        await client.query("Generate summary")
        async for msg in client.receive_response():
            print(f"Cost after pruning: {msg.total_cost_usd:.4f}")  # Lower
```

**Effect**: Reduces cost 30-50% in long-running sessions by removing irrelevant history.

## Digital FTE Monetization: Connecting Cost to Revenue

Cost tracking enables pricing. Pricing enables revenue. Revenue enables iteration.

Here's the monetization flow:

**Phase 1: Track Costs**
- Measure real execution costs per customer
- Identify cost drivers (which features/workflows cost most?)
- Establish baseline economics

**Phase 2: Set Pricing**
- Calculate break-even price based on target margin
- Choose model (subscription, success fee, usage)
- Test pricing with early customers

**Phase 3: Monitor Profitability**
- Track monthly margin per customer
- Identify unprofitable use cases
- Optimize high-cost workflows

**Phase 4: Scale Revenue**
- Acquire customers at profitable pricing
- Reinvest margin into product improvements
- Expand to new use cases

Example: A code review Digital FTE

| Step | Metric | Value |
|------|--------|-------|
| 1. Track costs | Avg cost per code review | $0.12 |
| 2. Set pricing | Subscription (30 reviews/month) | $14.99/month |
| 3. Profitability | Margin per customer | $10.35 (69%) |
| 4. Scale | 100 customers → revenue | $1,499/month |
| 5. Reinvest | Improve agent quality | $500/month → R&D |

This is how Digital FTEs become sustainable revenue.

## Try With AI

Build a cost-aware billing system for your Digital FTE.

### Prompt 1: Cost Tracking Architecture

Ask Claude: "I'm building a Digital FTE that analyzes code repositories. I need a Python system that tracks how much each customer's usage costs me. The agent runs multiple times per customer per month. How should I structure a cost tracking system that records total_cost_usd, aggregates by customer, and calculates monthly margin? Show me the class design and key methods."

**What you're learning**: How to design systems that make costs visible and actionable.

### Prompt 2: Pricing Model Recommendation

Based on the cost data you're tracking, ask Claude: "My code analysis Digital FTE costs an average of $0.08 per execution. Typical customers run it 40 times per month. I want 70% margin. Should I price this as subscription ($25/month), usage-based ($1.50 per execution), or success-fee (10% of time saved)? What are the tradeoffs and how do I validate which model customers prefer?"

**What you're learning**: How to evaluate pricing models against cost structure and customer preferences.

### Prompt 3: Cost Optimization Audit

Bring your implementation code and ask Claude: "Here's my agent configuration. It's costing too much. Help me audit the costs: Which parts are expensive? Should I change max_turns? Can I use cheaper models for any tasks? Can I optimize the prompts to get the same results with fewer tokens? Show me specific changes with estimated cost reduction."

**What you're learning**: How to optimize margins without sacrificing quality.

---

**What emerged from iteration**: A cost-conscious approach to agent development where economics inform design. You're not just building agents—you're building profitable products.


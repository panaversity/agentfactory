---
title: "Agent Ops"
sidebar_position: 5
description: "Learn how to operate AI agents in production: LM-as-Judge evaluation, Golden Datasets, OpenTelemetry traces, and human feedback loops."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 45 minutes
learning_objectives:
  - "Explain why traditional testing doesn't work for agents"
  - "Describe the LM-as-Judge evaluation approach and how to build evaluation rubrics"
  - "Explain how to debug agent behavior using OpenTelemetry traces"
  - "Understand human feedback loops and their role in continuous improvement"
skills:
  agent_evaluation:
    proficiency: B1
  agent_debugging:
    proficiency: B1
  operational_frameworks:
    proficiency: B1
generated_by: content-implementer v1.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 1.0.0
---

# Agent Ops

You've learned what agents are, how they're built, and the patterns that shape multi-agent systems. You understand the architecture and the loop. But here's the question that separates prototype systems from production ones: **How do you know if your agent is actually working?**

Traditional software testing breaks down when you're working with AI agents. You can't write a unit test that asserts `output == expected` because the agent's response is probabilistic—it might phrase the answer differently each time, find a better solution path, or take unexpected approaches that are actually correct. This means you need an entirely different operational framework.

This lesson introduces **Agent Ops**—the discipline of operating AI agents in production. It answers critical questions: How do you measure if an agent is achieving its goal? How do you debug why an agent made a poor decision? How do you improve the system based on what you observe? By the end, you'll understand why Agent Ops isn't optional—it's the difference between systems that work and systems that people trust.

---

## Why Agent Testing Is Different

Before diving into Agent Ops frameworks, recognize why traditional testing fails for agents.

### The Unit Test Problem

In traditional software, you write unit tests like this:

```python
def test_add_function():
    assert add(2, 3) == 5
    assert add(-1, 1) == 0
    assert add(0, 0) == 0
```

This works because the function's behavior is deterministic. Given the same input, you always get the same output. You test enough cases to feel confident in the function's correctness.

### Why This Breaks for Agents

An agent's response isn't deterministic. Given the same customer question, an agent might:

**Response 1**: "Your order #12345 shipped on November 20th and should arrive by November 25th."

**Response 2**: "Your order is on its way. It was sent on November 20th and the estimated delivery is November 25th."

**Response 3**: "Shipment confirmed for order #12345. Expected delivery: November 25th."

All three are correct. All three communicate the same information. But `response != expected_response` because the phrasing is different. You can't test with `assert response == "expected text"` because that assertion fails even though the agent performed correctly.

**Deeper problem**: What if the agent provides information but in the wrong format? Or includes accurate but irrelevant details? Or gets a fact wrong but the overall recommendation is sound? Traditional assertions can't capture these nuanced judgments.

This is why enterprises building agent systems realized: **You need a judge who can evaluate quality holistically, not a test suite checking for exact matches.**

---

## Measuring What Matters

Before designing an evaluation system, identify what success actually looks like.

### Define Your KPIs

An agent's effectiveness isn't about code correctness. It's about whether the agent accomplishes its goal. That requires metrics. The Google whitepaper emphasizes:

> "Define success metrics before building the evaluation system. What proves your agent delivers value?"

**Example: Customer Support Agent KPIs**

- **Goal Completion**: Did the agent successfully answer the customer's question? (70% target)
- **User Satisfaction**: Would the customer rate this response as helpful? (75%+ target)
- **Latency**: Did the agent respond within acceptable time? (<2 seconds target)
- **Cost**: What's the per-interaction cost? ($0.10 target)

Notice these aren't technical metrics (error rate, uptime). They're business metrics (goal completion, user satisfaction). This is the paradigm shift in Agent Ops: **You measure agent success the way you measure human employee success—did they accomplish the job well?**

### Frame Like an A/B Test

Think about Agent Ops improvements like A/B testing:

- **Baseline**: Current agent completes goals 65% of the time
- **Hypothesis**: Adding long-term memory improves completion to 75%
- **Test**: Run 100 conversations with memory enabled
- **Measure**: Did goal completion improve to 75%?

This frames Agent Ops as experimentation, not guessing. You change something (add memory, improve prompts, refine tools), measure the impact, and decide go/no-go based on whether the metrics improved.

---

## LM as Judge: Evaluating Agent Quality

The breakthrough that makes Agent Ops practical: **You can use a language model to evaluate another language model's output.**

### The Core Idea

Instead of writing a test that checks if output equals an expected string, you give a model a *rubric*—criteria for judging quality—and ask it to score the agent's response.

**Example Rubric for Customer Support Response**:

```
Evaluate the agent's customer support response using this rubric:

1. Accuracy (0-10 points)
   - Does the response contain correct factual information?
   - Are there any errors in order details, dates, or status?

2. Relevance (0-10 points)
   - Does the response directly address what the customer asked?
   - Is there unnecessary information included?

3. Clarity (0-10 points)
   - Is the response easy to understand?
   - Would a customer understand the next steps?

4. Helpfulness (0-10 points)
   - Does the response go beyond the minimum to help the customer?
   - Are there proactive suggestions or clarifications?

Total: Divide by 4 to get 0-10 point average score.
Go/No-go: Scores 8+ = acceptable, 7 or below = needs improvement

Evaluate this response: [AGENT RESPONSE HERE]
```

**Why This Works**

The language model evaluating the agent has:
- **Semantic understanding**: It understands whether information is correct, not just whether text matches exactly
- **Context awareness**: It understands whether the response fits the situation appropriately
- **Nuance handling**: It can judge responses that are correct but phrased differently, or correct but incomplete

This is fundamentally different from unit tests, which can only check exact matches.

### Building Effective Rubrics

The quality of your rubric determines evaluation quality. The Google whitepaper emphasizes:

**Rule 1: Be Specific**

**Bad rubric**: "Does the response sound helpful?"
- Too vague. The evaluator LM has to guess what "helpful" means.

**Good rubric**: "Does the response provide the customer with enough information to understand their next step without asking follow-up questions?"
- Specific criterion. Clear what counts as success.

**Rule 2: Include Anchors (Examples)**

**Weak rubric**: "Is the information accurate?"

**Strong rubric**:
```
Accuracy: Is the information accurate?
- 10 points: All factual claims are correct, dates match order records
- 7 points: Most claims correct, minor date discrepancy but correct month/year
- 4 points: Factual errors in status or dates, but overall direction correct
- 0 points: Multiple inaccuracies or contradicts order records
```

Examples help the evaluator LM understand the difference between a 7-point response and a 10-point response.

**Rule 3: Align Rubric to Your KPIs**

If your KPI is "user satisfaction," your rubric should measure things that drive satisfaction:
- Information accuracy
- Clarity of explanation
- Evidence of understanding the customer's situation
- Helpfulness (proactive suggestions)

Don't measure things that don't drive your metrics. "Code efficiency" doesn't belong in a customer support rubric.

---

## Golden Datasets: Building Your Evaluation Foundation

Now you have a way to evaluate agent responses (LM-as-Judge). But you need something to evaluate. This is where **golden datasets** come in.

### What Is a Golden Dataset?

A golden dataset is a curated collection of:
- **Ideal questions** (the kinds of queries your agent should handle)
- **Correct responses** (examples of what good answers look like)
- **Expected behaviors** (standards the agent should meet)

Think of it as a graded exam key. When a student takes an exam, you compare their answers to the key. Similarly, when you evaluate an agent, you compare its responses to your golden dataset examples.

### Building a Golden Dataset

**Step 1: Collect Real Examples**

Start with actual customer interactions or use cases:
- "What's my order status?"
- "How do I reset my password?"
- "Can I return an item from my purchase?"
- "What payment methods do you accept?"

**Step 2: Define Correct Responses**

For each question, write what a correct response should look like:

```
Question: "Where is my order #47382?"

Correct Response (Golden Standard):
"Order #47382 was shipped on November 15th via FedEx with tracking number
794853764920. The package is currently in transit and should arrive by
November 20th. You can track the shipment here: [link]."

Why This Answer Works:
- Confirms the order number (confirms understanding)
- Provides shipping date (answers the question)
- Includes carrier and tracking number (useful details)
- States expected delivery (helpful context)
- Provides tracking link (enables next step)
```

**Step 3: Validate as "Golden"**

Have humans verify these responses are actually good. This is critical. If your golden dataset contains mediocre or incorrect responses, your evaluation system will optimize for mediocrity.

### Using Golden Datasets in Evaluation

Once you have a golden dataset, your evaluation workflow looks like:

```
1. Run agent on a golden question
2. Collect agent's response
3. Run LM-as-Judge evaluation using your rubric
4. Compare score to baseline
5. If scores drop below threshold → flag for investigation
6. If scores improve → commit the change
```

This is **metrics-driven development**: You make changes (better prompts, additional tools, refined instructions) and only keep changes that improve your metrics.

---

## Debugging Agent Behavior: OpenTelemetry Traces

Here's a common problem: Your agent's score dropped from 8.2 to 6.7. Your LM-as-Judge says the response is now worse. But why? What changed?

Without visibility into what happened inside the agent, you're blind. This is where **traces** come in.

### What Is a Trace?

A trace is a **step-by-step recording of everything an agent did**. It captures:
- What prompt was sent to the model
- What the model was asked to reason about
- What tool was called
- What parameters were passed to the tool
- What the tool returned
- How the model responded to the result
- What happened next

Traces are like security camera footage for agent behavior—you can rewind and see exactly what happened.

### OpenTelemetry Standard

OpenTelemetry is an open standard for capturing traces (and metrics and logs). It's what Google Cloud uses, what AWS uses, what major observability platforms use.

**Why use a standard?** Because then your traces work across platforms. You're not locked into one vendor's proprietary format.

### A Concrete Example: Tracing an Order-Lookup Failure

Your agent is supposed to look up order #12345, but it returns the wrong information. Here's what a trace reveals:

```
TRACE: Customer Support Agent - Order Lookup

1. [MODEL] Received input: "Where is my order #12345?"

2. [REASONING] Model decides: "I need to find the order first, then get shipping info"

3. [TOOL CALL] Invokes: lookup_order(order_id="12345")
   Response: {
     "order_id": "12345",
     "status": "shipped",
     "shipping_carrier": "FedEx",
     "tracking_number": "794853764920"
   }

4. [REASONING] Model reasons: "I have the order. Now I need the expected delivery date."

5. [TOOL CALL] Invokes: get_shipping_estimate(tracking_number="794853764920")
   Response: ERROR - Invalid tracking number format

6. [RECOVERY] Model reasons: "The estimate lookup failed. I'll provide the information I have."

7. [RESPONSE] Generates: "Your order is shipped on FedEx with tracking 794853764920.
                           (Unable to fetch delivery estimate at this moment.)"

8. [QUALITY] LM-as-Judge scores: 6.7
   - Accuracy: 9/10 (correct tracking info)
   - Completeness: 4/10 (missing delivery estimate)
   - Clarity: 8/10 (clear but incomplete)
```

**Now you see the problem**: The `get_shipping_estimate` tool is rejecting valid tracking numbers. The agent tried to recover by providing partial information, but that hurt the score.

**Fix**: Either fix the tracking number format being passed, or provide the delivery estimate through a different method.

Without the trace, you'd be guessing. With it, the problem is obvious.

### In Practice: Google Cloud Trace Visualization

Google Cloud provides visual trace exploration where you can:
- Click on each step to see inputs and outputs
- Search for failures ("Show me traces where get_shipping_estimate failed")
- Compare traces from before/after a code change
- Identify patterns ("This error happens 23% of the time when...")

This transforms debugging from guessing to investigating. It's the difference between "why is the agent broken?" and "here's exactly what went wrong and where."

---

## Cherish Human Feedback: The Most Valuable Data Source

Here's something the Google whitepaper emphasizes that many teams overlook: **Human feedback is your most valuable improvement signal.**

Why? Because humans have context that automated metrics miss. A human might notice:
- "The response is technically correct but misses what the customer actually needed"
- "The tone is inappropriate even though the information is accurate"
- "There's a risk in what the agent recommended that metrics don't capture"

### The Feedback Loop

The whitepaper describes a virtuous cycle:

```
1. Agent Response
   ↓
2. Human Reviews & Provides Feedback
   "This was helpful" or "This missed X" or "This recommendation is risky"
   ↓
3. Feedback Captured (in structured format)
   ↓
4. Issue Replicated (reproduce the scenario that caused the problem)
   ↓
5. Add to Evaluation Dataset
   "Here's a scenario the agent should handle differently going forward"
   ↓
6. Retrain or Refine Agent
   ↓
7. Re-Evaluate (confirm improvement)
   ↓
   Back to Step 1 (next user feedback)
```

**The Magic**: Each piece of human feedback becomes a data point preventing future recurrence.

### Capturing Feedback Effectively

The key is structure. You can't just collect "I liked this" or "This was bad." You need:

**Structured Feedback Form**:
```
Feedback Submission

Was this response helpful?
  [ ] Yes, completely helpful
  [ ] Mostly helpful, minor issues
  [ ] Partially helpful, significant gaps
  [ ] Not helpful

If you selected "No" or "Partially": What was missing or incorrect?
  [ ] Missing information
  [ ] Inaccurate information
  [ ] Unclear explanation
  [ ] Inappropriate tone
  [ ] Other: ___________

What would have made this better?
  [Text field for specific suggestion]

Would you like to provide additional context?
  [Optional detailed explanation]
```

This structure makes feedback actionable. "Missing information" is data. "Other" with no detail is noise.

### From Feedback to Prevention

Once you have structured feedback:

**Step 1: Categorize**
- Identify patterns. "Missing information" shows up 47% of the time. "Inaccurate information" happens 12% of the time.

**Step 2: Replicate**
- Take a piece of feedback ("agent didn't mention return shipping") and create a test case
- Confirm the agent still fails on this case

**Step 3: Add to Golden Dataset**
```
Question: "I want to return an item from my order. What do I need to do?"

Golden Response:
"To return your item, here's the process:
1. Package the item in original condition
2. Print the return label here: [link]
3. Drop at any shipping location within 30 days
4. You'll receive a refund within 5-7 business days after we receive it

Is there anything else about the return process?"

Why This Matters:
This golden standard ensures the agent mentions return shipping cost
in future responses. Feedback indicated customers found it important.
```

**Step 4: Verify Improvement**
- Re-run the agent on this scenario
- Confirm LM-as-Judge score improves

---

## Putting It Together: An Agent Ops Workflow

Here's how these pieces fit into a real operational system:

### Week 1: Baseline Measurement

You have an agent in production. Before making any changes:

1. **Define KPIs**: Goal completion (target: 75%), user satisfaction (target: 4.2/5), latency (<2s)

2. **Build Golden Dataset**: Collect 50 representative customer questions with correct responses

3. **Create Evaluation Rubric**: Criteria for accuracy, completeness, clarity, helpfulness

4. **Run Baseline**: Evaluate agent on all 50 golden questions using LM-as-Judge
   - Result: 72% goal completion, 4.0/5 satisfaction

### Week 2: Improvement Hypothesis

You notice latency is creeping up (currently 1.8s, should be <2s). You hypothesize:

> "If we increase the context window limit by 50%, the agent will have better information and reach decisions faster."

You make the change and re-evaluate.

### Week 3: Measurement & Decision

Run LM-as-Judge on the same 50 golden questions:
- Goal completion: 74% (improved by 2 points)
- Satisfaction: 4.1/5 (improved by 0.1)
- Latency: 1.6s (improved!)

**Decision: Keep the change.** All metrics improved.

### Week 4: Human Feedback Collection

Collect human feedback from production conversations:
- 23 positive ratings
- 5 negative ratings with feedback:
  - "Missed that customer has existing support ticket" (3 mentions)
  - "Response was technically correct but tone felt dismissive" (2 mentions)

### Week 5: Iterate

From feedback, you identify a gap: The agent doesn't check for existing support history. You:

1. Replicate: Confirm agent fails on scenarios where customer has previous ticket
2. Add to golden dataset: Create test case for this scenario
3. Refine agent: Update prompt to "Always check support history first"
4. Re-evaluate: Run all 50 golden questions + new ones
   - New metric: "Support history awareness" now 95% (was 0%)

---

## Try With AI

You've learned the Agent Ops discipline. Now apply it to a use case you care about.

**Setup**: Choose an agent-like system you interact with regularly (customer support chatbot, scheduling assistant, research tool) and design how you'd evaluate its quality.

### Activity 1: Define KPIs

Pick a real system. What does success look like?

**Example: Document Search Agent**
- Goal Completion: Did the agent find relevant documents?
- Speed: Did it respond within 3 seconds?
- Relevance: Are the top 3 results actually related to the query?

**Your turn**:
Choose your system. Define 3-4 KPIs with measurable targets:
- [KPI 1]: [specific, measurable target]
- [KPI 2]: [specific, measurable target]
- [KPI 3]: [specific, measurable target]

Ask your AI: "I want to evaluate [my system]. Here are my KPIs. What metrics should I measure to confirm the agent is succeeding?"

Expected outcome: Your AI should suggest concrete metrics (not just "quality" but specific observable measures).

---

### Activity 2: Build an Evaluation Rubric

Using your KPIs, create a rubric an LM could use to judge the agent's performance.

**Structure**:
```
Criterion: [KPI name]
Measurement: [what you're evaluating]

10 points: [description of excellent performance with example]
7 points: [description of acceptable performance with example]
4 points: [description of poor performance with example]
0 points: [description of failure with example]
```

**Example for Relevance**:
```
Criterion: Document Relevance
Measurement: Are the returned documents actually related to the search query?

10 points: Top 3 results are directly relevant. Documents contain exact terms from query or clear synonyms.
7 points: Top 3 results mostly relevant. 2 of 3 are directly relevant; 1 is tangentially related.
4 points: Mixed relevance. 1 of 3 is relevant; others are loosely connected.
0 points: Results are unrelated to the query.
```

Ask your AI: "Here are my KPIs for evaluating [system]. Help me create a rubric that an LM-as-Judge could use. Make it specific with anchors."

Expected outcome: A detailed rubric with clear differentiation between scoring levels.

---

### Activity 3: Design a Golden Dataset

Create 5 test cases representing scenarios your agent should handle well.

**Format for Each Case**:
```
Scenario: [context/question]
Expected Response Quality:
  - What information should be included?
  - What should the tone be?
  - What's the minimum acceptable response?
Golden Standard (Example Response):
  [write an example response that scores 8+ on your rubric]
```

**Example for Customer Support**:
```
Scenario: Customer asks "Where is my order?"
Expected Response Quality:
  - Must include order number confirmation
  - Must include shipping status
  - Should include tracking number if shipped
  - Tone should be helpful and friendly

Golden Standard:
"Your order #45821 has been shipped! It left our warehouse on Nov 15
via UPS with tracking number 1Z999AA10123456784. You can track it here:
[link]. Expected delivery is Nov 20. Thank you for your patience!"
```

Ask your AI: "Help me create 5 realistic test cases for [my system]. Each should be a scenario where the agent should excel."

Expected outcome: Representative test cases that cover the agent's main use cases.

---

### Activity 4: Trace a Failure

Imagine your agent performed poorly on one of your golden dataset scenarios. Design what debugging visibility you'd need.

**Create a Trace Outline**:
```
Agent Receives: [input that caused failure]

Step 1: [What reasoning happened?]
Step 2: [What tool was called?]
Step 3: [What was the response?]
Step 4: [How did the agent respond to that?]
Step 5: [Final output and quality score]

Root Cause: [Where did things go wrong?]
Fix: [How would you prevent this?]
```

Ask your AI: "Imagine my agent failed on this scenario: [describe a failure]. Walk me through what a trace might reveal about why it failed, and what visibility I'd need to debug it."

Expected outcome: Understanding of how observability helps diagnose agent problems.

---

### Activity 5: Design a Feedback Loop

Human feedback is how agents improve. Design a system to capture it:

**Feedback Capture Design**:
- What question would you ask users to understand if the agent succeeded?
- What structured options would help users give actionable feedback?
- How would you categorize feedback so it drives improvements?

Example:
```
"Was this response helpful?"
  [ ] Yes, solved my problem
  [ ] Mostly helpful, minor issues
  [ ] Not helpful, missing key information
  [ ] Not helpful, incorrect information

If "No" or "Mostly": What was the problem?
  [ ] Missing information I needed
  [ ] Information was incorrect
  [ ] Response was unclear
  [ ] Suggested wrong action
  [ ] Other: ___

What would have made this better?
  [Text field for detail]
```

Ask your AI: "Design a feedback collection system for [my system] that captures structured data I can use to improve the agent. What questions should I ask? How should I structure responses?"

Expected outcome: A feedback mechanism that generates actionable data for agent improvement.

---

### Optional Stretch Challenge

Create a complete Agent Ops workflow for your system:

1. What's the baseline quality (estimate from your rubric)?
2. What improvement would you try first?
3. How would you measure whether it worked?
4. What human feedback would tell you there's a gap?
5. How would you prevent that gap from recurring?

Ask your AI: "Walk me through an Agent Ops iteration cycle for [my system]. Starting from baseline, what's one improvement I'd try, how I'd measure it, and how I'd use feedback to prevent problems?"

Expected outcome: A concrete operational workflow you could actually implement if you were building this agent.

---

## Summary

Agent Ops transforms agent development from guesswork to systematic improvement:

**The Core Concepts**:
- **LM-as-Judge**: Use language models to evaluate agent outputs holistically, not just check for exact matches
- **Golden Datasets**: Build curated test cases with ideal responses that define what success looks like
- **OpenTelemetry Traces**: Record every step an agent takes so you can debug failures precisely
- **Human Feedback Loops**: Capture structured feedback, replicate issues, add to evaluation dataset, prevent recurrence

**The Mindset Shift**:
Before Agent Ops, you asked: "Is my agent correct?" (hard to answer)
With Agent Ops, you ask: "Is my agent improving toward my business KPIs?" (measurable and concrete)

This is what separates prototype agents from production systems. You can build a prototype in an afternoon. But operating it reliably, improving it continuously, maintaining quality as use cases change—that's what Agent Ops enables.

The teams building the most reliable agent systems now aren't the ones with the most advanced models. They're the ones with the best operational discipline. That discipline starts here.

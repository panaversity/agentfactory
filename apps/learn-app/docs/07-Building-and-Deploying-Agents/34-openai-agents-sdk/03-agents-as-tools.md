---
sidebar_position: 3
title: "Agents as Tools & Orchestration"
description: "Convert agents to tools, orchestrate multi-agent workflows, and build specialized teams that delegate work to each other."
keywords: ["agent.as_tool", "multi-agent orchestration", "tool_name", "tool_description", "custom_output_extractor", "specialist agents", "manager agent", "agent composition"]
chapter: 34
lesson: 3
duration_minutes: 90

# HIDDEN SKILLS METADATA
skills:
  - name: "Agent-to-Tool Conversion"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can convert agent to tool using as_tool() with custom tool name and description"

  - name: "Custom Output Extraction"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Data Transformation"
    measurable_at_this_level: "Student can write custom_output_extractor to transform sub-agent output for parent agent consumption"

  - name: "Multi-Agent Orchestration Pattern"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "System Architecture"
    measurable_at_this_level: "Student can design manager agent that coordinates 2+ specialist agents through tool composition"

  - name: "Specialist Agent Design"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "System Design"
    measurable_at_this_level: "Student can identify which functions should become specialist agents vs remain as tools"

  - name: "Agent Factory Composition"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "System Architecture"
    measurable_at_this_level: "Student can evaluate tradeoffs between handoffs vs agents-as-tools for different workflow patterns"

learning_objectives:
  - objective: "Use as_tool() to convert an agent into a callable tool for another agent"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successfully creates manager agent that calls specialist agent as tool"

  - objective: "Write custom_output_extractor to control what data flows from sub-agent to parent agent"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Extractor correctly transforms sub-agent output into format parent agent expects"

  - objective: "Design multi-agent orchestration workflows where specialist agents divide responsibility"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Manager agent successfully delegates to 2+ specialists and synthesizes results"

  - objective: "Analyze tradeoffs between agents-as-tools (manager pattern) and handoffs (peer pattern)"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student can explain when to use each pattern and why for given workflow"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (agent.as_tool, tool_name, tool_description, custom_output_extractor, manager pattern, specialist pattern, orchestration, composition) - within B1-B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design dynamic agent composition with conditional specialist selection based on input; implement agent pool pattern for load distribution"
  remedial_for_struggling: "Start with 2-specialist manager; move to 3+ specialists after pattern is comfortable. Skip custom_output_extractor initially."

generated_by: content-implementer
source_spec: specs/047-ch34-openai-agents-sdk
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Agents as Tools & Orchestration

Imagine you're building a customer support system, but the problems are complex. Some customers need technical help. Others need billing assistance. Still others have shipping questions. A single agent trying to handle all three would be like hiring a generalist accountant to also repair helicopters. They'd be mediocre at everything.

The solution: specialist agents. You build three experts—a Technical Specialist, a Billing Specialist, and a Shipping Specialist. Then you build a Manager Agent that listens to the customer and routes them to the right specialist. The manager doesn't solve problems. The manager delegates.

This is what agents as tools enable. You convert specialized agents into tools that a manager agent can call. The manager orchestrates the workflow. The specialists execute their domains. Together, they deliver better results than any single agent could.

This is the foundation of Digital FTE design: composable agents that work together.

## The Agent as Tool Pattern

In Chapter 33, you learned that tools are functions an agent can call. But here's the powerful insight: **an agent itself can be converted into a tool**.

Think about it. An agent is a reasoning system that takes input and produces output. A tool is something that produces output when called. Agents fit the tool interface perfectly.

Here's the basic pattern:

```python
from agents import Agent, Runner

# Step 1: Define a specialist agent
research_agent = Agent(
    name="Researcher",
    instructions="You are a research expert. Find accurate information on topics and provide well-sourced summaries."
)

# Step 2: Convert it to a tool
research_tool = research_agent.as_tool(
    tool_name="do_research",
    tool_description="Research a topic and provide accurate, sourced findings"
)

# Step 3: Give the tool to a manager agent
manager_agent = Agent(
    name="Manager",
    instructions="You coordinate research tasks. When asked about a topic, use the research tool to find information.",
    tools=[research_tool]
)

# Step 4: Run the manager
result = Runner.run_sync(manager_agent, "Research the environmental impact of electric vehicles")
print(result.final_output)
```

**Output:**
```
Based on my research, electric vehicles have a complex environmental story:

Manufacturing: EV battery production requires significant energy (typically equivalent to 1-2 years of driving emissions). However, this is offset within 2-3 years of normal driving as EVs produce zero tailpipe emissions.

Well-to-Wheel: Even accounting for electricity generation methods, EVs produce 50-70% fewer emissions than gasoline vehicles over their lifetime in regions with clean power grids. In coal-heavy regions, the advantage is smaller but still significant.

End-of-Life: Battery recycling is improving. Current methods recover 90%+ of materials, reducing the need for new mining.

Conclusion: EVs are environmentally beneficial, particularly in regions transitioning to renewable energy.
```

What happened here:

1. The research agent ran independently, reasoning about the topic
2. It produced a detailed response
3. The manager agent received that response as tool output
4. The manager incorporated it into its own response

The manager never had to do research itself. It delegated to a specialist.

## Naming and Describing Your Agent Tools

When you convert an agent to a tool, you control how the manager agent understands it through two fields:

**tool_name**: How the manager calls it
- Should be lowercase, underscores (Python convention)
- Examples: `do_research`, `analyze_sentiment`, `generate_summary`
- The manager chooses this based on understanding what's available

**tool_description**: What the tool does
- Should be clear and concise (1-2 sentences)
- Should explain what the tool accepts and returns
- The manager reads this to decide when to use the tool

Here's an example with multiple specialist agents:

```python
from agents import Agent, Runner

# Define three specialists
financial_analyst = Agent(
    name="FinanceExpert",
    instructions="You are a financial analyst. Analyze financial metrics, profitability, and economic trends."
)

risk_analyst = Agent(
    name="RiskExpert",
    instructions="You are a risk analyst. Identify threats, red flags, and potential problems."
)

market_analyst = Agent(
    name="MarketExpert",
    instructions="You are a market analyst. Assess market conditions, competition, and opportunities."
)

# Convert each to a tool with clear names and descriptions
financial_tool = financial_analyst.as_tool(
    tool_name="analyze_financials",
    tool_description="Analyze company financials including revenue, profitability, cash flow, and growth rates"
)

risk_tool = risk_analyst.as_tool(
    tool_name="assess_risks",
    tool_description="Identify business risks, market threats, regulatory concerns, and vulnerabilities"
)

market_tool = market_analyst.as_tool(
    tool_name="evaluate_market",
    tool_description="Analyze market dynamics, competitive landscape, and growth opportunities"
)

# Create a manager that uses all three
investment_manager = Agent(
    name="InvestmentAdvisor",
    instructions="""You are an investment advisor. When asked to evaluate a company or investment:
1. Use analyze_financials to understand financial health
2. Use assess_risks to identify threats
3. Use evaluate_market to understand competitive position
4. Synthesize all three perspectives into a recommendation.""",
    tools=[financial_tool, risk_tool, market_tool]
)

result = Runner.run_sync(investment_manager, "Should I invest in TechStartup Inc?")
print(result.final_output)
```

**Output:**
```
Based on my comprehensive analysis, here's my investment recommendation:

Financial Analysis: TechStartup shows strong growth (40% YoY) with improving margins (now at 15%). However, they're burning cash and have 18 months of runway remaining.

Risk Assessment: Key risks include market saturation (5 competitors just entered), key person dependency (CEO holds critical IP), and regulatory uncertainty in their sector. These are significant concerns.

Market Position: The market is growing 25% annually, but competition is fierce and intensifying. Their differentiation is unclear.

Recommendation: HOLD. The company has potential, but the combination of cash burn, intense competition, and key risks makes this a higher-risk investment than their growth metrics alone suggest. Wait for profitability before investing, or only invest if you're willing to accept high volatility.
```

Notice how the tool names (`analyze_financials`, `assess_risks`, `evaluate_market`) are specific and action-oriented. The descriptions are detailed enough that the manager understands what each tool provides.

## Custom Output Extractors

By default, when a manager agent calls a specialist agent as a tool, it receives the specialist's complete output. But sometimes you want to transform that output before passing it to the parent agent.

This is where custom_output_extractor comes in. It's a function that processes the specialist's output and returns only what the parent needs.

Here's a realistic example from financial research:

```python
from agents import Agent, Runner, RunResult, RunResultStreaming

# Specialist agent that produces detailed analysis
financial_specialist = Agent(
    name="FinancialAnalyst",
    instructions="""Provide comprehensive financial analysis including:
    - Detailed revenue breakdown by segment
    - Margin analysis over time
    - Cash flow projections
    - Risk factors
    - Investment thesis

    Be thorough. Include numbers, context, and detailed explanation."""
)

# Function to extract just the summary
def extract_financial_summary(result: RunResult | RunResultStreaming) -> str:
    """Extract just the investment thesis from detailed analysis."""
    # In a real system, you might parse structured output
    # For now, we take the first 300 characters (the executive summary)
    full_output = str(result.final_output)
    summary = full_output.split('\n')[0]  # First line is usually the thesis
    return summary

# Convert to tool with custom extractor
financial_tool = financial_specialist.as_tool(
    tool_name="financial_analysis",
    tool_description="Get financial analysis of a company",
    custom_output_extractor=extract_financial_summary
)

# Manager that writes a report
report_writer = Agent(
    name="ReportWriter",
    instructions="""You write investment reports. For each company:
1. Get financial analysis (this will be a brief summary)
2. Incorporate the summary into a 2-paragraph report
3. Provide clear investment recommendation""",
    tools=[financial_tool]
)

result = Runner.run_sync(report_writer, "Write a report on CloudCorp Inc")
print(result.final_output)
```

**Output:**
```
CloudCorp Inc demonstrates strong financial fundamentals with 35% YoY revenue growth and expanding margins. Their cloud infrastructure business is well-positioned for continued expansion as enterprises migrate to distributed systems. Revenue diversification across three customer segments reduces concentration risk.

The company is currently unprofitable but trending toward breakeven in Q3 2025. Strong cash reserves (18 months runway) provide adequate capital for growth initiatives. We recommend this as a BUY for growth-oriented investors with moderate risk tolerance.
```

The key insight: the financial specialist provided exhaustive detail. But the report writer only needed the thesis. The custom_output_extractor acted as a filter, passing only the relevant information forward. This keeps the report writer focused and prevents it from getting lost in unnecessary detail.

## The Manager-Specialist Architecture

Here's a complete, realistic example: a TaskManager system with three specialists.

```python
from agents import Agent, Runner, function_tool

# Specialist 1: Planning Agent
planner = Agent(
    name="Planner",
    instructions="""You break complex tasks into steps.
    When given a goal, create a numbered plan with clear steps.
    Be specific: instead of "implement feature", say "implement user authentication with JWT"."""
)

# Specialist 2: Execution Agent
executor = Agent(
    name="Executor",
    instructions="""You implement tasks.
    Given a specific task, provide implementation details, code examples, and technical approach.
    Focus on practical, working solutions."""
)

# Specialist 3: Validation Agent
validator = Agent(
    name="Validator",
    instructions="""You review completed work.
    Check for correctness, missing edge cases, and potential issues.
    Provide constructive feedback on what works and what needs improvement."""
)

# Convert specialists to tools
planning_tool = planner.as_tool(
    tool_name="create_plan",
    tool_description="Break a complex goal into numbered steps"
)

execution_tool = executor.as_tool(
    tool_name="implement_task",
    tool_description="Implement a specific task with technical details"
)

validation_tool = validator.as_tool(
    tool_name="validate_work",
    tool_description="Review work for correctness and suggest improvements"
)

# Manager Agent: Orchestrates the workflow
task_manager = Agent(
    name="TaskManager",
    instructions="""You manage task execution workflows.

    When given a goal:
    1. Use create_plan to break it into steps
    2. Use implement_task for each step
    3. Use validate_work to review the implementation
    4. Provide a summary of the completed work

    Each step builds on previous steps. Pass relevant context forward.""",
    tools=[planning_tool, execution_tool, validation_tool]
)

# Run the orchestration
result = Runner.run_sync(
    task_manager,
    "Build a simple login system for a web application"
)

print(result.final_output)
```

**Output:**
```
I've orchestrated the development of a simple login system. Here's what happened:

Planning Phase:
1. Design user schema (username, email, password hash, created_at)
2. Implement registration endpoint (/api/register)
3. Implement login endpoint (/api/login with JWT token generation)
4. Add password hashing with bcrypt
5. Create session management logic
6. Add input validation and error handling

Implementation Phase:
[The Executor specialist provided detailed code for each step, including Flask endpoints, password hashing, JWT token generation]

Validation Phase:
[The Validator reviewed the code and flagged:]
- Missing HTTPS enforcement (security issue)
- No rate limiting on login attempts (vulnerability to brute force)
- Password validation too weak (should require 8+ chars, mixed case)
- Good: proper password hashing, secure token generation

Summary:
The login system is structurally sound but needs security hardening. Recommendations:
1. Add rate limiting to prevent brute force attacks
2. Strengthen password requirements
3. Add HTTPS enforcement in production
4. Consider adding 2FA for additional security

The system is ready for development but requires these security improvements before production deployment.
```

Notice how the Manager orchestrated the entire workflow:
- Planner created the roadmap
- Executor implemented based on that roadmap
- Validator reviewed and provided feedback
- Manager synthesized everything into a coherent summary

Each specialist had expertise. The manager had orchestration responsibility. Together they produced something better than any single agent could.

## Agents as Tools vs Handoffs

You might remember handoffs from the SKILL.md file. When should you use agents as tools (manager pattern) vs handoffs (peer pattern)?

| Situation | Pattern | Why |
|-----------|---------|-----|
| **Manager needs control** | Agents as Tools | Manager decides the workflow; specialists execute |
| **One agent routes to others** | Handoffs | Receiving agent takes full responsibility |
| **Multiple calls in sequence** | Agents as Tools | Manager keeps context across multiple specialist calls |
| **Linear routing** | Handoffs | Simple: "Go to specialist, they finish, done" |
| **Complex synthesis** | Agents as Tools | Manager needs to combine multiple specialist outputs |
| **Customer support triage** | Handoffs | "You talk to billing", final handoff to specialist |

For the TaskManager example above, agents as tools was the right choice because the Manager needed to:
1. Call Planner
2. Call Executor (multiple times, once per step)
3. Call Validator on the results
4. Synthesize everything

With handoffs, the workflow would have been fragmented. The Manager couldn't coordinate across multiple Executor calls or synthesize Validator feedback at the end.

## Building Reusable Specialist Agents

When designing specialists for your Digital FTE, think about reusability. A good specialist agent:

1. **Has a focused domain**: Billing specialist handles billing. Not also shipping.
2. **Has clear instructions**: "You analyze X and return Y." No ambiguity.
3. **Produces structured output**: Predictable format that parent agents can use.
4. **Can be used in multiple contexts**: The same billing specialist used by the customer support agent AND the accounting agent.

Here's a template for building reusable specialists:

```python
from agents import Agent

# Template for reusable specialist
def create_specialist_agent(domain: str, instructions: str) -> Agent:
    """Create a specialized agent for a specific domain."""
    return Agent(
        name=f"{domain}Specialist",
        instructions=f"""You are a {domain.lower()} specialist.

Your responsibilities:
{instructions}

Key principles:
- Be thorough and accurate
- Provide specific, actionable information
- Flag any limitations or uncertainties
- Format output clearly for consumption by other agents"""
    )

# Example specialists
billing_specialist = create_specialist_agent(
    domain="Billing",
    instructions="""
    - Calculate accurate invoice amounts
    - Apply discounts correctly
    - Handle refunds and credits
    - Provide itemized billing breakdowns"""
)

support_specialist = create_specialist_agent(
    domain="Support",
    instructions="""
    - Troubleshoot customer issues
    - Provide step-by-step solutions
    - Escalate complex issues appropriately
    - Confirm resolution with customer"""
)
```

These specialists can now be used in multiple contexts. The customer support manager uses both. The billing dashboard uses the billing specialist. They're modular, reusable components of your Digital FTE ecosystem.

## Try With AI

### Prompt 1: Your First Orchestration

```
Create a simple orchestration with a manager agent and 2 specialist agents:
- ContentSpecialist: Writes articles
- EditorSpecialist: Reviews and improves articles

Manager should ask ContentSpecialist to write a short article about Python loops,
then ask EditorSpecialist to improve it. Show both outputs.

What did the Editor change that the Content specialist missed?
```

**What you're learning:** How to structure agent-as-tool relationships and see how specialists improve on each other's work

### Prompt 2: Custom Output Extraction

```
Create a financial analyst agent that produces very detailed output (500+ words).
Write a custom_output_extractor that takes just the first 50 words (the summary).
Create a report writer agent that uses this as a tool.
Run the report writer and compare:
- What the financial analyst produced (full output)
- What the report writer received (extracted summary)

Why was extraction useful here?
```

**What you're learning:** How custom_output_extractor controls information flow between agents, preventing information overload

### Prompt 3: Design a 3-Specialist System

```
Design a customer complaint handling system with 3 specialists:
1. AnalysisSpecialist: Understands what the complaint is about
2. SolutionSpecialist: Recommends solutions
3. EmpathySpecialist: Writes a response that acknowledges the customer's feelings

Create the manager and all 3 specialists. Test with:
"I've been waiting 2 weeks for my order and it's still not here. I'm frustrated."

Did the 3 specialists working together produce a better response than any single agent?
```

**What you're learning:** How to decompose complex problems into specialist responsibilities and how manager orchestration produces better outcomes

### Core Concept
Every agent has four components: Model (brain for reasoning), Tools (hands for action), Orchestration (nervous system for planning/memory), and Deployment (body for accessibility). Understanding this architecture helps you design agents and debug failures.

### Key Mental Models
- **3+1 Architecture**: Model, Tools, and Orchestration are the core trio; Deployment wraps them for production use
- **Component → Failure Mapping**: Wrong reasoning = Model problem; can't execute = Tool problem; lost context = Orchestration problem
- **Model Tiering**: Match model capability to task complexity—fast/cheap for routing, premium for complex analysis

### Critical Patterns
- Tools have defined inputs, outputs, and *permissions*—guardrails prevent misuse
- Orchestration manages planning (breaking goals into steps), memory (short-term and long-term), and reasoning strategy
- Three reasoning strategies: ReAct (reason-act-observe loop), Chain-of-Thought (think before acting), Reflection (evaluate and revise)
- When debugging, identify which component failed by tracing the agent's decision flow

### Common Mistakes
- Giving agents unlimited tool access without permission boundaries
- Ignoring memory requirements—agents need context tracking across steps
- Choosing models based on capability alone, ignoring cost and latency trade-offs
- Treating orchestration as automatic—it requires deliberate design decisions

### Connections
- **Builds on**: Agent definition and 5-Level Taxonomy (Lesson 1)
- **Leads to**: The 5-Step Operational Loop (Lesson 3)—how agents execute within this architecture

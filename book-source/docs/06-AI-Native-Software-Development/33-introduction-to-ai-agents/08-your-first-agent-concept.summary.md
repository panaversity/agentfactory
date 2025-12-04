### Core Concept
Agent specs describe *how the agent thinks and behaves*, not deterministic steps. The spec defines constraints within which the agent reasons to find its own path. If you can't specify it clearly, you don't understand it well enough to build it.

### Key Mental Models
- **Traditional vs Agent Specs**: Traditional prescribes a path ("parse X, validate Y"). Agent specs define intent + constraints ("resolve inquiries, max $500 refund").
- **Spec as Constitution**: Anticipate failure modes without prescribing solutions; set boundaries without removing flexibility
- **Synthesis Test**: A complete spec applies all frameworks—taxonomy level, architecture, 5-step loop, pattern, security

### Critical Patterns
- **Specification Template** (5 sections):
  1. Purpose & Capability Level (with justification)
  2. 3+1 Architecture (model, tools, orchestration, deployment with trade-offs)
  3. 5-Step Loop (concrete example walkthrough)
  4. Pattern (single/coordinator/sequential/iterative/HITL with justification)
  5. Security (guardrails, guard checks, trust trade-off, compromise plan)
- Be specific: "refund up to $500" not "can issue refunds"
- Justify every decision: "Haiku because high volume + routine reasoning"

### Common Mistakes
- Writing abstract specs without concrete 5-Step Loop examples
- Justifying capability level without reasoning ("Level 2" vs "Level 2 because multi-step planning but not multi-agent coordination")
- Over-engineering security for low-risk agents or under-engineering for high-risk ones
- Missing compromise plan—what happens when the agent is breached?

### Connections
- **Builds on**: All Chapter 33 lessons—this synthesizes every framework into one deliverable
- **Leads to**: Chapters 34-36—implementation using the specification you designed

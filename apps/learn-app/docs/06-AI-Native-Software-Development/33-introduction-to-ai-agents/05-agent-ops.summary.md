### Core Concept
Building agents is easy; operating them reliably is hard. Agent Ops is evaluation, debugging, and continuous improvement through four pillars: LM-as-Judge, Golden Datasets, Traces, and Human Feedback Loops.

### Key Mental Models
- **Many Valid Outputs**: Traditional testing (input → expected output) breaks for agents. Evaluation must assess "good enough" across valid variations.
- **Mindset Shift**: From "Is my agent correct?" to "Is my agent improving toward my KPIs?"
- **Trace → Root Cause**: When agents fail, traces show exactly which step broke—no more guessing

### Critical Patterns
- **LM-as-Judge**: Use an LLM with a rubric to evaluate outputs at scale (10 points: solves problem; 0 points: wrong information)
- **Golden Datasets**: Curated test cases with expected elements (not exact wording). Run after every change; if scores drop, you broke something.
- **Traces**: Record every step—tool calls, reasoning, results. Enable root cause analysis and pattern recognition.
- **Feedback Loop**: User reports problem → find trace → identify root cause → fix → add to golden dataset → verify with judge

### Common Mistakes
- Testing agents like traditional software (expecting exact outputs)
- Building golden datasets too small (start with 20-50, aim for 100+)
- Not adding every fixed bug as a new test case
- Ignoring traces when debugging—they show the exact failure point

### Connections
- **Builds on**: Multi-Agent Patterns (Lesson 4)—Agent Ops applies to single and multi-agent systems
- **Leads to**: Interoperability & Security (Lesson 6)—operational discipline extends to secure, connected agents

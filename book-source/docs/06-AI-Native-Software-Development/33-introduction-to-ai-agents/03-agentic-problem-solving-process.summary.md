### Core Concept
Every agent follows the same 5-Step Loop: Get Mission → Scan Scene → Think Through → Take Action → Observe and Iterate. This loop runs until the goal is achieved or the agent determines it cannot proceed.

### Key Mental Models
- **5-Step Loop**: Universal operational pattern—understand goal, gather context, plan approach, execute, verify results
- **Context Engineering**: Agent quality depends on context quality. Focused context = better agent. More tools ≠ better agent.
- **Failure → Step Mapping**: When agents fail, trace which step broke to diagnose the root cause

### Critical Patterns
- Diagnose failures by step: Wrong goal (Step 1), missing info (Step 2), bad reasoning (Step 3), tool error (Step 4), didn't recognize done (Step 5)
- Provide focused, relevant context—specific error messages beat "it doesn't work"
- Clarifying questions are context engineering—help by answering precisely
- The loop is universal: applies to customer support agents, Claude Code, self-driving cars, your own problem-solving

### Common Mistakes
- Giving agents access to everything instead of focused, relevant context
- Providing vague requests that break Step 1 (Get Mission)
- Pointing agents to wrong files, breaking Step 2 (Scan Scene)
- Not defining clear success criteria, breaking Step 5 (Observe)

### Connections
- **Builds on**: 3+1 Architecture (Lesson 2)—the loop operates within that structure
- **Leads to**: Multi-Agent Design Patterns (Lesson 4)—how multiple agents coordinate their loops

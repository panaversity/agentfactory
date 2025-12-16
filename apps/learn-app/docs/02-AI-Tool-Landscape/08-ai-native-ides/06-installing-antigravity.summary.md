### Core Concept
Antigravity introduces a **control plane architecture** for AI agents—instead of asking AI to write code within an editor, you spawn agents that work autonomously across three distinct surfaces (Agent Manager, Editor, Integrated Browser), allowing parallel human-AI work and plan-before-code workflows.

### Key Mental Models
- **Three-Surface Separation**: Agent Manager (spawn and monitor agents), Editor (code collaboration), Integrated Browser (agent-controlled testing)—enables you and agents to work in parallel without blocking each other
- **Artifact-Driven Workflow**: Agents generate Task List → Implementation Plan → Walkthrough artifacts. You approve plans *before* agents code, not after—specification quality over speed
- **Agent as Workforce**: Instead of a single AI assistant, you manage multiple specialized agents with custom instructions, autonomy levels, and model choices

### Critical Patterns
- Create agents with clear instructions that define behavior across all tasks (like `.cursorrules` but per-agent)
- Set autonomy to "Ask Sometimes" for learning—agent pauses at key decisions for your approval
- Review Task List and Implementation Plan artifacts before approving—catch wrong approaches early
- Set artifact verbosity to "Detailed" to see agent reasoning
- Artifacts live in `~/.antigravity/workspaces/[workspace]/artifacts/[agent]/`

### Common Mistakes
- Approving plans without reading them—the whole point is catching issues before implementation
- Setting autonomy to "Full Auto" before trusting agent behavior—start with "Ask Sometimes"
- Ignoring the three surfaces—working only in Editor misses Antigravity's parallel workflow advantage
- Forgetting that Antigravity requires constant internet connection for DeepMind API calls

### Connections
- **Builds on**: Zed (inline AI assistance) and Cursor (autonomous code generation with diffs)—Antigravity represents a third architectural approach
- **Leads to**: Advanced artifact workflows, parallel task execution, and building complete projects with agent-driven development

### Core Concept
The modern AI development stack is a three-layer architecture that separates concerns and enables interoperability. Unlike 2024's monolithic tool silos, you can now independently choose and swap frontier models, IDEs, and development agents without breaking your workflow.

### Key Mental Models
- **Three-Layer Separation**: Layer 1 (Frontier Models) provides intelligence/reasoning, Layer 2 (AI-First IDEs) provides your workspace interface, Layer 3 (Development Agents) provides autonomous orchestration. Each layer has a clear job; swap any without affecting others.
- **USB Analogy for MCP**: Model Context Protocol is like USB—before it, every tool had custom integrations; with it, any MCP-compatible tool can access your codebase, terminal, and context through a standard interface.
- **Architecture vs Tools**: The three-layer *pattern* is durable even when specific tools change. Learn the architectural concept (why layers separate), not just tool names.

### Critical Patterns
- **Layer 1 (Frontier Models)**: GPT-5, Claude Opus 4, Gemini 2.5 Pro—handles reasoning and code generation
- **Layer 2 (AI-First IDEs)**: VS Code + extensions, Cursor, Windsurf—where you write code and interact with AI
- **Layer 3 (Development Agents)**: Claude Code CLI, Aider, Devin—autonomous multi-step task execution
- **MCP compatibility**: Enables tool switching without workflow changes; choose MCP-compatible tools for future flexibility

### Common Mistakes
- Treating AI tools as monolithic products instead of composable layers (2024 thinking)
- Learning specific tool features instead of underlying architectural patterns (knowledge becomes obsolete)
- Ignoring MCP compatibility when selecting tools (creates future vendor lock-in)
- Waiting for the "perfect" tool instead of learning the pattern that transfers across tools

### Connections
- **Builds on**: AI development transformation and economic scale (Chapter 1-2)
- **Leads to**: Practical tool setup, Claude Code CLI usage, and Spec-Driven Development workflows

### Core Concept
Claude Code's **CLI is frontend-agnostic**—it can work with any OpenAI-compatible backend through API routing. This architecture enables a free alternative using Google's Gemini API while maintaining identical Claude Code functionality (subagents, skills, MCP servers, hooks).

### Key Mental Models
- **Three-Layer Architecture**: CLI (interface) → Router (format translator) → API (AI backend)—understanding this separation reveals how tools can be composed beyond vendor lock-in
- **Backend Abstraction**: The Claude Code CLI doesn't care what AI responds—it cares about the API format. Routers translate between Anthropic and OpenAI API standards
- **Environment Variables as Secrets**: API keys stored in shell config files (`~/.zshrc`, `~/.bashrc`) persist across sessions without hardcoding sensitive data

### Critical Patterns
- **Daily workflow requires two terminals**: Router starts first (Terminal 1: `ccr start`), then Claude Code connects through it (Terminal 2: `ccr code`)
- **Verify each layer independently**: Check Node.js version, router configuration, API key presence—failures cascade, so isolate the broken layer
- **Copy-paste setup over understanding**: For procedural tasks, get it working first, understand the architecture later (remedial differentiation)

### Common Mistakes
- Forgetting to start the router before launching Claude Code (connection fails silently)
- Hardcoding API keys directly in config files instead of using environment variables (security risk)
- Using wrong shell config file (`.zshrc` vs `.bashrc`)—check `echo $SHELL` first
- Expecting router to auto-start (it's a manual step each session)

### Connections
- **Alternative to**: Lesson 2's official Claude Max subscription—both paths teach identical Claude Code skills
- **Leads to**: Lessons 4-9 work identically regardless of which setup path you chose (persistent context, MCP, subagents, skills, hooks, configuration)

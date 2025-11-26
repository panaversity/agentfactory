### Core Concept
Installation means three separate components: Spec-Kit Plus framework (`pip install specifyplus`), Playwright MCP browser automation tool (`npm install -g @anthropic-ai/mcp-playwright`), and configuration of your AI tool (Claude Desktop or Gemini CLI) to access both. Session persistence must be enabled to maintain browser login state across video generation runs.

### Key Mental Models
- **Framework ≠ AI Tool**: Spec-Kit Plus is the methodology; Claude Code/Gemini are the AI tools that execute it. Both required, neither sufficient alone
- **Global vs Local Installation**: Playwright MCP must be installed globally (`-g` flag) so Claude Desktop can access it system-wide
- **Session Persistence = Reusable Login**: Browser cookies saved to disk mean you log into Google once, then stay logged in for 10+ video generations

### Critical Patterns
- **Initialize project structure**: Always use `specifyplus init` to create `.specify/`, `.claude/`, and other required directories—never create manually
- **Verify at each step**: Test Spec-Kit Plus commands (`/sp.` autocomplete), confirm Playwright MCP appears in AI tool, validate session persistence configuration
- **Platform-specific paths**: MCP config lives at `~/.claude/claude_desktop_config.json` (not home directory, not `.config/`)

### Common Mistakes
- Installing Playwright MCP locally per-project instead of globally—Claude Desktop won't find it
- Creating MCP config in wrong location or with JSON syntax errors—config silently fails to load
- Skipping project initialization and manually creating folder structure—misses critical `.specify/` templates and scripts
- Forgetting to restart Claude Desktop after MCP configuration changes

### Connections
- **Builds on**: Understanding Spec-Kit Plus architecture (Lesson 1)
- **Leads to**: Creating project Constitution that governs all video generation work (Lesson 3)

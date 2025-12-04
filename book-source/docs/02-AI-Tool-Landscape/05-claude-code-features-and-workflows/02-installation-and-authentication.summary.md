### Core Concept
Terminal-integrated AI fundamentally changes your development workflow—Claude Code operates directly in your workspace where it reads your actual files, proposes real changes, and executes commands. This eliminates the context-switching friction of browser-based tools where you describe problems and manually copy-paste solutions.

### Key Mental Models
- **Workspace Integration vs Context-Switching**: Web-based AI is a consultant you visit; terminal-integrated AI is a pair programmer in your environment
- **Authentication Economics**: Claude.ai (subscription) vs Console API (pay-per-use) represent different cost models—your usage patterns determine which saves money
- **Safety Boundaries**: Claude Code has your permissions. Start sessions in project directories (not system directories) and review commands before approving

### Critical Patterns
- Choose installation method by platform: curl/Homebrew (macOS), PowerShell (Windows), npm (cross-platform with Node.js 18+)
- Verify installation with `claude --version` before proceeding to authentication
- Select authentication based on your account: Claude.ai subscription (Option 1, simpler) or Console API key (Option 2, pay-per-use)
- Test setup with: `claude "Hello! Can you confirm Claude Code is working?"`

### Common Mistakes
- Running Claude Code in system directories (~/Library, /etc, C:\Windows) instead of project folders—Claude Code can modify files where you run it
- Console API users: Not setting usage limits at console.anthropic.com/settings/limits, leading to unexpected bills
- Approving commands without review, especially `sudo` or administrative operations—Claude Code asks for approval, but you're responsible for understanding what you approve

### Connections
- **Builds on**: Lesson 1's distinction between passive AI tools and agentic collaboration
- **Leads to**: Lesson 3 (free Gemini alternative) or Lesson 4 (CLAUDE.md project context)

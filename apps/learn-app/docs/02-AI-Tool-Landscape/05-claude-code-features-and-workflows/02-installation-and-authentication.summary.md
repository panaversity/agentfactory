### Core Concept
Terminal-integrated AI fundamentally changes your development workflow—Claude Code operates directly in your workspace where it reads your actual files, proposes real changes, and executes commands. This eliminates the context-switching friction of browser-based tools where you describe problems and manually copy-paste solutions.

### Key Mental Models
- **Workspace Integration vs Context-Switching**: Web-based AI is a consultant you visit; terminal-integrated AI is a pair programmer in your environment
- **Platform-Specific Installation**: Each platform (Windows/macOS/Linux) has a recommended primary method optimized for that environment—reducing decision fatigue from evaluating all global options
- **Authentication Economics**: Three cost models exist: Claude App (subscription), Console API (pay-per-use), Enterprise (dedicated capacity)—your usage patterns and organizational requirements determine which to use
- **Safety Boundaries**: Claude Code has your permissions. Start sessions in project directories (not system directories) and review commands before approving

### Critical Patterns
- **Windows**: Requires WSL or Git for Windows (Claude Code needs bash). WSL (recommended) → Git Bash → npm (if Node.js available)
- **macOS**: Homebrew (recommended) → curl/bash → npm (if Node.js available)
- **Linux/WSL**: curl/bash (recommended) → npm (if Node.js available)
- **npm cross-platform**: Available on all platforms if Node.js 18+ installed
- Verify installation with `claude --version` before proceeding to authentication
- Select authentication based on your account type:
  - Method 1: Claude App (Pro/Max/Team subscription) - most common, unified access
  - Method 2: Console API (API credits, pay-per-use) - developers, usage-based billing
  - Method 3: Enterprise (Bedrock, Vertex AI, Foundry) - organizations with cloud infrastructure
- Test setup with: `claude "Hello! Can you confirm Claude Code is working?"`

### Common Mistakes
- Running Claude Code in system directories (~/Library, /etc, C:\Windows) instead of project folders—Claude Code can modify files where you run it
- Windows users: Trying to run Claude Code without WSL or Git for Windows—Claude Code requires a bash shell, the installer alone isn't enough
- Console API users: Not setting usage limits at console.anthropic.com/settings/limits, leading to unexpected bills
- Approving commands without review, especially `sudo` or administrative operations—Claude Code asks for approval, but you're responsible for understanding what you approve
- Alpine Linux users: Forgetting to install libgcc/libstdc++ and configure `USE_BUILTIN_RIPGREP=0`, causing runtime errors

### Connections
- **Builds on**: Lesson 1's distinction between passive AI tools and agentic collaboration
- **Leads to**: Lesson 3 (free Gemini alternative) or Lesson 4 (CLAUDE.md project context)

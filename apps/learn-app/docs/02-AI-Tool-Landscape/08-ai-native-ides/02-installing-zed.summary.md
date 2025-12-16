### Core Concept
Zed provides the fastest path to AI-native development—a Rust-based IDE with built-in AI assistance that removes friction between thinking and generating code.

### Key Mental Models
- **Speed = Friction Removal**: Sub-200ms startup and instant AI responses let you stay in flow state; lag breaks the human-AI collaboration loop
- **Provider Flexibility**: Zed connects to multiple AI providers (Anthropic, OpenAI, Google, Ollama)—choose based on cost, privacy, or capability needs
- **Verify Before Building**: Installation isn't complete until you've triggered AI generation and seen it work; untested setup causes mysterious failures later

### Critical Patterns
- Install via package manager: `brew install zed` (macOS), `curl https://zed.dev/install.sh | sh` (Linux), `winget install zed` (Windows)
- Trigger inline assistant: `Cmd+K` / `Ctrl+K` then `Cmd+/` / `Ctrl+/` to request code generation
- Configure via `settings.json`: theme, font size, tab size, formatter, and default AI model
- API key authentication: paste key from provider console into Zed's settings

### Common Mistakes
- Skipping verification: assuming installation worked without testing AI generation leads to debugging during actual work
- Wrong model selection: choosing expensive models for simple tasks or weak models for complex generation
- Forgetting API key regeneration: when "Invalid API Key" appears, the fix is regenerating at the provider—not reconfiguring Zed
- Expecting instant first response: initial requests take longer as the connection establishes

### Connections
- **Builds on**: Terminal basics (Chapter 7) for running installation commands
- **Leads to**: Building real projects with AI collaboration (Lesson 3)

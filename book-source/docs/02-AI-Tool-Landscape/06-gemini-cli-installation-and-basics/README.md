# Chapter 6: Google Gemini CLI: Open Source and Everywhere

Google didn't just follow—they went big. Gemini CLI is fully open source under Apache 2.0 license, bringing the power of Gemini directly into developers' terminals with built-in tools for Google Search grounding, file operations, shell commands, and web fetching.

What makes Gemini CLI particularly compelling is its accessibility. Developers get 60 model requests per minute and 1,000 requests per day at no charge simply by logging in with a personal Google account. Through eight progressive lessons organized in three tiers, you'll move from installation through advanced customization—mastering not just Gemini CLI, but the judgment to choose the right AI tool for every development scenario.

This chapter isn't about replacing Claude Code. It's about understanding **when to use each tool** and building the judgment to choose the right AI assistant for every development scenario you encounter.

## What You'll Learn

By the end of this chapter, you'll understand:

- **The open source advantage**: Why Apache 2.0 licensing matters for AI development tools and how Gemini CLI's openness enables customization, forking, and community-driven evolution
- **Tool selection framework**: Six objective dimensions (licensing, cost, context window, extensibility, interface, support) for evaluating AI assistants based on project needs, not vendor marketing
- **Installation and authentication**: Three installation methods (npm global, npx temporary, version-specific tags) and OAuth-based Google login providing 60 requests/minute and 1,000 requests/day free tier
- **Interface navigation**: The Gemini CLI terminal interface (ASCII logo, input box, status bar) and how to interact through natural conversation, slash commands, and shell mode
- **Built-in tool ecosystem**: File operations, shell integration, web fetching, and Google Search grounding—and when to combine tools for complex tasks
- **Context management**: How to use `/clear` for hard resets, `/compress` for smart summarization, `/chat save/resume` for conversation branching, and GEMINI.md files for persistent memory across sessions
- **Configuration mastery**: The 7-level hierarchy (CLI flags → project settings → global settings → system defaults) and how to use `.env` files, project-specific settings, and security best practices
- **Custom automation**: Creating reusable slash commands with TOML files and injection patterns (`{{args}}`, `!{shell}`, `@{file}`) to automate repetitive workflows
- **MCP integration**: Connecting external capabilities (databases, APIs, browsers) through the Model Context Protocol using `gemini mcp add/list/remove` commands and OAuth authentication
- **Extensions and security**: Building extension bundles, filtering tools with allowlists/blocklists, and integrating with VS Code for diff-based editing
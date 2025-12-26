### Core Concept
AGENTS.md is a README for AI agents—a Markdown file that tells agents your project's coding conventions, build commands, testing requirements, and security rules so they work correctly without guessing.

### Key Mental Models
- **README vs AGENTS.md**: README is for humans (what the project is), AGENTS.md is for agents (how to work on it)
- **Hierarchy Rule**: Nearest AGENTS.md wins—root file for project-wide rules, subdirectory files for overrides
- **Adaptability = Monetization**: A Digital FTE that respects client conventions can be sold to 100 organizations without customization

### Critical Patterns
- Include: build commands, code style, testing requirements, security rules, commit formats
- Plain Markdown with no required schema—easy to write, agents parse naturally
- Filename must be `AGENTS.md` (uppercase)
- Monorepos use multiple AGENTS.md files at different directory levels
- 60,000+ projects adopted; supported by Claude Code, Cursor, Copilot, VS Code, Devin, Zed

### Common Mistakes
- Omitting AGENTS.md and expecting agents to guess your conventions
- Putting only high-level descriptions instead of actionable commands
- Forgetting security considerations (API keys, input sanitization)
- Using a single AGENTS.md for a monorepo with different package standards

### Connections
- **Builds on**: MCP for tool connectivity (Lesson 2)
- **Leads to**: goose reference implementation (Lesson 4)

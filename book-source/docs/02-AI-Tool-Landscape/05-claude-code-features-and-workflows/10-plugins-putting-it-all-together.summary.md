### Core Concept
Plugins are bundled packages that combine multiple Claude Code capabilities—skills, agents, hooks, commands, and MCP—into single installable units. Check marketplaces before building custom solutions; reuse is strategic, reinvention is waste.

### Key Mental Models
- **Composition Over Creation**: Existing plugins solve common workflow needs (design, testing, document processing)—leverage community intelligence before building custom
- **Bundled Intelligence**: A plugin's manifest declares its components, allowing one install to add skills, agents, hooks, commands, and MCP together
- **Marketplace-First Discovery**: Start with Anthropic's skills repository (`anthropics/skills`) and custom marketplaces before considering custom development

### Critical Patterns
- Add marketplaces: `/plugin marketplace add anthropics/skills`
- Install bundles: `/plugin install example-skills@anthropic-agent-skills`
- List available: `/plugin marketplace list`
- Plugin manifest (`plugin.json`) declares all components with paths
- Skills install to `.claude/skills/` and activate automatically when Claude detects matching requests

### Common Mistakes
- Building capabilities from scratch without checking what already exists in marketplaces
- Creating "mega-skills" that do too much instead of focused, composable skills Claude can combine
- Writing vague skill descriptions without specific trigger scenarios and examples
- Treating components as separate (skills here, agents there) instead of bundling related capabilities together

### Connections
- **Builds on**: All Claude Code features (CLAUDE.md, MCP, subagents, skills, hooks, settings) from Lessons 1-8
- **Leads to**: Custom plugin creation and distribution strategies (Part 5+)

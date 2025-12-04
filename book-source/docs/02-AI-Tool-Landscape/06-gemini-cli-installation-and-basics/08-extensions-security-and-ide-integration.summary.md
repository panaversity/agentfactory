### Core Concept
Extensions transform complex multi-step Gemini CLI setups into single-command installations—packaging MCP servers, custom commands, context files, and settings into shareable bundles that turn 45-minute manual configuration into 2-minute automated deployment.

### Key Mental Models
- **Extension as Complete Bundle**: An extension isn't just one thing; it's MCP servers + commands + GEMINI.md context + settings wrapped together for consistent team deployment
- **Install vs Link**: Installed extensions copy files (update via command); linked extensions reference originals (changes apply on restart)—choose based on whether you're developing or consuming
- **Scope Control**: Extensions can be enabled/disabled globally (user scope) or per-project (workspace scope), letting you maintain different toolsets for different contexts

### Critical Patterns
- Install from GitHub: `gemini extensions install https://github.com/username/extension-name`
- Manage extensions: `list`, `update <name>`, `disable <name> --scope workspace`, `uninstall <name>`
- Create new: `gemini extensions new <name> <template>` with templates: `context`, `custom-commands`, `mcp-server`, `exclude-tools`
- Core structure: `gemini-extension.json` (config) + `commands/` (slash commands) + `GEMINI.md` (context)
- Cross-platform paths: Use `${extensionPath}`, `${workspacePath}`, `${/}` variables in configurations

### Common Mistakes
- Forgetting to restart Gemini CLI after extension changes—MCP servers and commands only load at startup
- Using wrong URL format (`github.com/...` instead of `https://github.com/...`) causes installation failures
- Creating extensions for solo use when individual MCP servers and commands would be simpler—extensions shine for team sharing and complex multi-component setups

### Connections
- **Builds on**: MCP servers, custom commands, and GEMINI.md context from earlier lessons
- **Leads to**: Complete toolkit for packaging and sharing Gemini CLI environments with teams and study groups

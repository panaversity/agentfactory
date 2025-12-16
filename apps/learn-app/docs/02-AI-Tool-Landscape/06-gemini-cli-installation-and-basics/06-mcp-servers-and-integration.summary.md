### Core Concept
MCP (Model Context Protocol) transforms AI tools from isolated assistants into connected powerhouses—one universal standard that lets Gemini CLI browse websites, access databases, and interact with external systems through a single protocol that works across all AI tools.

### Key Mental Models
- **USB for AI**: Before USB, every device needed a custom cable. Before MCP, every AI built custom integrations. MCP is the universal standard—build once, use everywhere.
- **Advisor → Executor**: Without MCP, AI tells you what to do manually. With MCP, AI does it for you and returns results.
- **Phone Directory of Specialists**: MCP servers are specialists Gemini calls automatically—Playwright for browsing, Context7 for documentation, custom servers for anything else.

### Critical Patterns
- Add servers via CLI: `gemini mcp add playwright npx @playwright/mcp@latest`
- Check status: `gemini mcp list` shows connected/disconnected state
- Remove when done: `gemini mcp remove server-name`
- Authenticate protected services: `/mcp auth server-name` opens browser OAuth flow
- Combine capabilities in single prompts: Playwright browses, Context7 fetches docs, file tools save results

### Common Mistakes
- Forgetting to verify server status when connections fail—always run `gemini mcp list` first
- Adding MCP servers from untrusted sources—stick to official npm packages and verified repositories
- Attempting OAuth for public websites—Playwright browsing Amazon or Wikipedia needs no authentication
- Not removing unused servers—audit regularly with the security checklist

### Connections
- **Builds on**: Built-in tools (Lesson 3), Configuration & Settings (Lesson 5)
- **Leads to**: Custom MCP server development, advanced multi-tool workflows

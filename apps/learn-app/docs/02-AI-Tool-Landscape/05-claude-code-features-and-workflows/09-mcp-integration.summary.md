### Core Concept
MCP (Model Context Protocol) extends Claude Code's reach beyond your local files to external systems—websites, documentation, APIs, databases—through standardized, permission-controlled connections.

### Key Mental Models
- **Phone Directory**: MCP servers are "approved specialists" Claude can call—a web browser expert (Playwright), a documentation specialist (Context7), a database consultant. Each handles one external capability.
- **Local vs External Boundary**: Without MCP, Claude sees only your files. With MCP, Claude can reach the outside world—but only through explicit, secure channels you've configured.
- **Just-in-Time Research**: MCP servers like Context7 fetch current documentation on demand, replacing bookmark collections and documentation hunting with conversational requests.

### Critical Patterns
- Install MCP servers with `claude mcp add --transport stdio [name] npx [package]`
- Verify registered servers with `claude mcp list`
- Use Playwright MCP for web browsing: "Use the Playwright MCP to browse [site] and find [criteria]"
- Use Context7 MCP for docs: "Use Context7 to fetch the latest documentation about [topic]"
- Iterate naturally: refine results with follow-up requests ("filter to long-sleeve", "show only Prime-eligible")

### Common Mistakes
- Expecting Claude to access websites or APIs without first adding an MCP server—Claude cannot reach external systems by default
- Using untrusted MCP servers—only install servers from reputable sources since they can access the internet
- Pasting secrets into files instead of using environment variables or system keychain prompts
- Forgetting that web content changes—if browsing fails, Claude may need to adjust navigation steps

### Connections
- **Builds on**: CLAUDE.md (Lesson 3) for local project context; core Claude Code operations
- **Leads to**: Advanced MCP servers (PostgreSQL, GitHub, Filesystem), custom MCP server creation for team-specific needs

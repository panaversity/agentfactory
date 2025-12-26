### Core Concept
MCP (Model Context Protocol) is the universal standard for connecting AI agents to external tools and data—write one MCP server, and any MCP-compatible agent (Claude, ChatGPT, Gemini) can use it.

### Key Mental Models
- **USB for AI Agents**: Just as USB standardized device connections, MCP standardizes agent-to-tool connections
- **Three Primitives**: Resources (data to read—eyes), Tools (functions to call—hands), Prompts (templates—playbook)
- **Host → Client → Server**: The LLM app (Host) contains a connector (Client) that talks to external services (Servers)

### Critical Patterns
- Resources = what agents can see (files, database records, API responses)
- Tools = what agents can do (search, create, update, send)
- Prompts = reusable interaction templates (summarizing, analyzing, reporting)
- Transport: stdio for local tools, HTTP/SSE for remote services
- MCP enables the "Act" power—without it, agents can only think, not do

### Common Mistakes
- Building custom integrations per AI platform instead of one MCP server
- Confusing Resources (read-only data) with Tools (executable actions)
- Thinking you need to implement transport yourself—SDKs handle this

### Connections
- **Builds on**: AAIF governance and open standards (Lesson 1)
- **Leads to**: AGENTS.md for project context (Lesson 3)

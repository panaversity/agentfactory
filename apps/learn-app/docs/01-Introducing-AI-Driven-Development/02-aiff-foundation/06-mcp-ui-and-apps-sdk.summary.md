### Core Concept
Agent interfaces are evolving beyond chat—Apps SDK lets you build rich applications inside ChatGPT today (800M+ users), while MCP Apps Extension (SEP-1865) is standardizing interactive UI across all MCP hosts for cross-platform portability.

### Key Mental Models
- **Text → Structured → Interactive**: Evolution from chat to buttons, forms, and visualizations—stage 3 is the competitive frontier
- **MCP = Brain, Interface = Face**: Same Digital FTE can have different "faces" for different distribution channels (ChatGPT app, web dashboard, CLI)
- **Apps SDK Today, MCP Apps Tomorrow**: Use Apps SDK for immediate ChatGPT distribution; MCP Apps Extension will unify interfaces across all hosts

### Critical Patterns
- MCP Apps Extension (SEP-1865): `ui://` URI scheme for pre-declared UI templates in sandboxed iframes
- Collaboration pattern: MCP-UI (Postman, Shopify, HuggingFace) + OpenAI Apps SDK → standardized extension
- Security by design: iframe sandboxing, template pre-review, auditable JSON-RPC logging, user consent for tool calls
- Marketplace monetization: 800M+ users, low customer acquisition cost, platform handles billing

### Common Mistakes
- Thinking chat is sufficient when rich UI would dramatically improve UX and close more deals
- Waiting for MCP Apps Extension instead of shipping with Apps SDK today (immediate distribution beats future portability)
- Building platform-specific interfaces without text-only fallbacks for hosts without UI support
- Ignoring security communication—enterprise clients buy trust before capability

### Connections
- **Builds on**: MCP protocol (Lesson 2), four revenue models from Chapter 1 (Marketplace path)
- **Leads to**: Part 2 AI Tool Landscape (putting standards into practice), Part 6 Custom Agents (building products with interfaces)

### Core Concept
MCP servers load ALL tool definitions at startup (8,000+ tokens each), creating token bloat. Compiling MCP to skills reduces context consumption by 80-98% while preserving full functionality—transforming expensive tool definitions into lean SKILL.md files (~100-200 tokens).

### Key Mental Models
- **Token Bloat**: MCP eagerly loads everything; you're paying for tools you're not using. A 2-hour workflow might waste 50,000+ tokens just from repeated definition loading.
- **Code Execution Pattern**: Instead of calling MCP tools directly through Claude's context, write code locally that calls the tools—Claude orchestrates, scripts execute.
- **Progressive Disclosure**: Skills load only when relevant, scripts execute only when needed. Same capability, fraction of the context cost.
- **Compilation Workflow**: Introspect (extract tool definitions) → Compile (create lean SKILL.md) → Generate (write executable scripts) → Validate (measure token savings).

### Critical Patterns
- Introspect MCP: "What tools does [server] provide? For each: name, description, parameters, output."
- Compile to skill: "Create a SKILL.md with YAML frontmatter (name, description, version) and sections: When to Use, Procedure, Output Format."
- Place skills at `.claude/skills/[name]/SKILL.md`
- Use skill-creator to automate: "Use skill-creator to compile [MCP server] to a skill for [use case]."
- Validate savings: Compare token counts before (direct MCP) vs after (compiled skill).

### Common Mistakes
- Compiling one-off MCP calls—compilation overhead isn't worth it for single use
- Creating skills for rapidly-changing APIs—compiled skills become stale quickly
- Forgetting the YAML frontmatter (name, description, version)—Claude won't discover the skill
- Writing overly broad descriptions—Claude can't determine when to activate

### Decision Framework
- **Use direct MCP**: One-off queries, low-token servers (<500 tokens), rapidly-changing APIs
- **Compile to skill**: Repeated workflows (3+ calls), high-token servers (5,000+), stable tool sets, team sharing, privacy-sensitive data

### Connections
- **Builds on**: Skills (Lesson 05-06) for SKILL.md format; MCP Integration (Lesson 08) for Playwright/Context7 examples
- **Leads to**: Subagents and Orchestration (Lesson 10) for composing skills into multi-agent workflows

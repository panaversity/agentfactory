### Core Concept
Gemini CLI has two memory systems: short-term **context** (conversation within a session, limited to 1M tokens) and long-term **memory** (GEMINI.md files that persist across sessions). Managing both effectively keeps your AI informed without repetition.

### Key Mental Models
- **Context = RAM, Memory = Disk**: Context is fast but volatile (lost on close); GEMINI.md is persistent but static (always loads)
- **Hierarchical Loading**: Multiple GEMINI.md files merge—user preferences + project conventions + directory-specific rules combine automatically
- **Token Budget**: Every message, file, and GEMINI.md content consumes from your 1M token limit—keep persistent memory lean

### Critical Patterns
- Create `GEMINI.md` at project root with team conventions, architecture, and setup commands
- Use `/clear` for fresh start (different topic), `/compress` to free tokens (same topic)
- Use `/chat save name` before interruptions, `/chat resume name` to restore full context
- Use `/memory refresh` after editing GEMINI.md files externally

### Common Mistakes
- Putting temporary debugging notes in GEMINI.md (use `/chat save` instead—it's session-specific)
- Closing terminal without saving—conversation is lost unless you run `/chat save`
- Bloated GEMINI.md files that waste tokens on every session start

### Connections
- **Builds on**: Basic Gemini CLI navigation and prompting (Chapter 6, Lessons 1-3)
- **Leads to**: MCP servers and extended tool capabilities

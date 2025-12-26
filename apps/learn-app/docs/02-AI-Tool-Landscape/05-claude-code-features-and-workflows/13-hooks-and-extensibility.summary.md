### Core Concept
Hooks are automated scripts that run when specific events occur in Claude Code—eliminating repetitive setup and follow-up tasks so you focus on creative work instead of manual routines.

### Key Mental Models
- **Event-Trigger-Action**: When X happens (session starts, file edited, tool runs) → Y command executes automatically
- **Automation vs. Capabilities**: Hooks add *automation* to workflows; Skills and MCP add *new capabilities*—they're complementary, not overlapping
- **Predictable → Strategic**: Automate the predictable parts of your workflow to free attention for the thinking only humans can do

### Critical Patterns
- Four hook events: `PreToolUse` (before tool runs), `PostToolUse` (after tool completes), `SessionStart` (session opens), `SessionEnd` (session closes)
- Hooks live in `.claude/settings.json` with structure: event type → matcher (optional) → command action
- Start simple: a SessionStart hook that echoes project context gives immediate value
- Co-design custom hooks through AI collaboration: you provide workflow context, AI suggests patterns, iterate together

### Common Mistakes
- Confusing hooks with skills/MCP/plugins—hooks automate behavior, they don't add new capabilities
- Feeling pressure to customize immediately—Claude Code works perfectly without hooks; master basic workflows first
- Worrying that hook errors will break your session—they're non-blocking and logged for later investigation

### Connections
- **Builds on**: CLAUDE.md configuration, MCP external connections, Skills custom commands
- **Leads to**: Advanced hook building and debugging (Part 5)

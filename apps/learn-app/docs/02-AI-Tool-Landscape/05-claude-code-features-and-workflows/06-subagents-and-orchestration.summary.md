### Core Concept
Subagents are specialized AI assistants with isolated context windows. Each subagent handles one focused task, completes it, and returns results to the main Claude Code session—enabling complex work through orchestrated delegation rather than cluttered single-context conversations.

### Key Mental Models
- **Orchestrator/Specialist**: Claude Code acts as a project manager coordinating a team of specialist subagents—one for research, one for code review, one for testing—each expert in their domain
- **One Task, One Completion**: Subagents don't persist. They receive a goal, work independently, return results, and hand control back. Like sending a specialist to research something—they return with a report, then you continue with your main assistant
- **Context Isolation = Clean Context**: Each subagent starts fresh without clutter from other conversations, producing more focused, higher-quality results

### Critical Patterns
- Use `/agents` → "Create new agent" to build custom subagents through Claude's guided workflow
- Describe what the subagent should do and when it should be used; Claude generates the configuration
- Subagents can activate automatically (Claude recognizes task complexity) or explicitly ("Use the startup-planner subagent to...")
- Subagent files live in `.claude/agents/` (project) or `~/.claude/agents/` (personal)
- The built-in Plan subagent automatically activates for complex multi-step tasks, researching your codebase before proposing changes

### Common Mistakes
- Expecting subagents to remember previous conversations—each invocation starts with clean context
- Trying to continue dialogue with a subagent after it completes—control returns to main Claude Code
- Overusing subagents for simple tasks that don't need specialist focus
- Creating overly broad subagents instead of focused specialists (defeats the purpose of context isolation)

### Connections
- **Builds on**: Claude Code installation, CLAUDE.md project memory, understanding of AI collaboration patterns
- **Leads to**: Multi-subagent orchestration workflows, advanced delegation patterns, building domain-specific subagent suites

### Core Concept
CLAUDE.md is a markdown file in your project root that Claude Code automatically loads at session start—giving your AI companion persistent project context without you repeating explanations every time.

### Key Mental Models
- **Context Friction → Productivity Loss**: Every session without persistent context forces re-explanation; CLAUDE.md eliminates this friction
- **Specify Once, Benefit Always**: One-time setup (10-15 minutes) provides automatic context loading forever
- **Iterative Refinement**: Your first CLAUDE.md draft improves through AI review (suggests missing sections) and your domain knowledge (adds team-specific constraints)

### Critical Patterns
- Place CLAUDE.md in project root (same level as `.git`, `package.json`, `pyproject.toml`)
- Include 6 standard sections: Project Overview, Technology Stack, Directory Structure, Coding Conventions, Key Commands, Important Notes
- Ask Claude to generate initial CLAUDE.md: `"Help me create a CLAUDE.md file for this project based on what you see in the codebase"`
- Verify auto-loading by starting new session and asking about your tech stack—Claude should know without you telling it
- Refine iteratively: Claude suggests missing sections → you add team-specific patterns → converge on complete context

### Common Mistakes
- File not in project root (must be same directory as `.git`, not in subdirectory)
- Wrong filename (must be exactly `CLAUDE.md`, case-sensitive)
- Testing in same session (auto-loading only happens when starting new Claude Code session)
- Including task-specific details (CLAUDE.md is for project-wide context, not individual task instructions)

### Connections
- **Builds on**: Claude Code installation and authentication (Lesson 2)
- **Leads to**: Subagents and Skills (Lessons 5-6) inherit and extend CLAUDE.md context

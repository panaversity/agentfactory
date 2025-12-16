### Core Concept
Claude Code uses a three-level settings hierarchy (user → project → local) where more specific settings override more general ones, enabling both team standards and personal customization without conflicts.

### Key Mental Models
- **Specificity Wins**: More specific context always overrides more general—local overrides project overrides user
- **Three Layers of Context**: User settings = personal defaults across all projects; Project settings = team-shared standards committed to git; Local settings = private experimentation that stays on your machine
- **Configuration as Organizational Intelligence**: The hierarchy mirrors how teams organize knowledge—individual preferences, shared agreements, and safe experimentation spaces

### Critical Patterns
- Three file locations: `~/.claude/settings.json` (user), `.claude/settings.json` (project), `.claude/settings.local.json` (local)
- Precedence order: local > project > user (most specific wins)
- Commit `.claude/settings.json` to version control; add `.claude/settings.local.json` to `.gitignore`
- Use user level for personal preferences, project level for team standards, local level for temporary experiments

### Common Mistakes
- Deleting the `.claude/` directory thinking it's unimportant—this resets all project configuration
- Committing `.claude/settings.local.json` to git, exposing personal/temporary settings to the team
- Using project level for personal experiments instead of local level (pollutes team standards)

### Connections
- **Builds on**: CLAUDE.md for project context (Lesson 3), Skills for capability customization (Lesson 6)
- **Leads to**: Part 5 detailed settings configuration and team workflow policies

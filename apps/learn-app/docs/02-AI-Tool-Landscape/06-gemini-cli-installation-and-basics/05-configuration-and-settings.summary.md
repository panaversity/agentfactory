### Core Concept
Gemini CLI's 7-level configuration hierarchy lets project settings automatically override your personal defaults—so you can work on multiple projects with different requirements without manually switching settings each time.

### Key Mental Models
- **Hierarchy Precedence**: Higher levels always win (CLI flags → env vars → .env → project → workspace → user → system). When settings conflict, the more specific one applies.
- **Three-File Security Pattern**: `.env` (secrets, gitignored) + `.env.example` (template, committed) + `.gitignore` (protection). This keeps credentials out of version control while documenting what's needed.
- **Variable Substitution**: Use `${VAR_NAME}` in settings.json to reference secrets stored safely in `.env` files.

### Critical Patterns
- Project-level `.gemini/settings.json` overrides user-level `~/.gemini/settings.json`
- Environment variables use `${DATABASE_URL}` syntax in configuration files
- Create `.gitignore` BEFORE creating `.env` to prevent accidental commits
- `.env.example` shows structure without real values—safe for version control

### Common Mistakes
- Hardcoding secrets directly in `settings.json` (use environment variable references instead)
- Forgetting to add `.env` to `.gitignore` before creating the file (secrets end up in git history)
- Variable name mismatches between `.env` and `settings.json` (names are case-sensitive)

### Connections
- **Builds on**: Gemini CLI installation and authentication (Lessons 1-4)
- **Leads to**: MCP Servers & Integration (Lesson 6)

### Core Concept
Custom slash commands transform repetitive multi-line prompts into short, reusable shortcuts. Type `/learn Python` instead of crafting the same 100-character prompt every time—consistent quality with minimal effort.

### Key Mental Models
- **Prompt → Command**: Any effective prompt pattern you discover can become a permanent, reusable command
- **Placeholder Replacement**: `{{args}}` works like fill-in-the-blank—one command serves unlimited topics
- **Folder → Namespace**: Directory structure (`study/plan.toml`) creates organized command categories (`/study:plan`)

### Critical Patterns
- Every command needs two fields: `description` (shown in `/help`) and `prompt` (sent to Gemini)
- Use triple quotes `"""` for multi-line prompts in TOML
- `{{args}}` gets replaced with whatever you type after the command name
- Commands live in `~/.gemini/commands/` (create with `mkdir -p`)
- Subfolders create namespaced commands: `study/review.toml` → `/study:review`

### Common Mistakes
- Wrong location: File must be in `~/.gemini/commands/`, not elsewhere
- Missing quotes: `description = Learn` fails; use `description = "Learn"`
- Wrong braces: `{args}` won't work—must use double braces `{{args}}`
- Case sensitivity: `Learn.toml` differs from `learn.toml` on Mac/Linux

### Connections
- **Builds on**: Gemini CLI installation and built-in tools (Chapter 6, lessons 1-3)
- **Leads to**: Building personal AI workflows and reusable prompt libraries

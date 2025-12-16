### Core Concept
Ruff configuration in `pyproject.toml` standardizes code quality rules across teams—enabling shared standards (line length, rule categories), format-on-save automation, and consistent enforcement through version-controlled settings rather than individual developer decisions.

### Key Mental Models
- **Configuration as Contract**: `pyproject.toml` version-controlled; teammates get identical rules automatically, preventing "your style" vs. "my style" debates
- **Rule Category Selection**: Enable/disable broad categories (E for style, F for safety, I for imports, B for bugs) rather than fine-grained rule tweaking, keeping decisions tractable
- **TOML Syntax Delegation**: AI generates configuration; students understand "what to configure" (line length, target Python version, rule categories) not "how to write TOML"
- **Editor Integration Automation**: Format-on-save in Zed removes manual `ruff format` step, keeping code clean during development without cognitive overhead

### Critical Patterns
- **Basic Configuration Workflow**: Tell AI preferences (line length, Python version, rule set), receive TOML, paste into file, verify with `ruff check`
- **Selective Rule Ignoring**: `# noqa` comments (line-level) for exceptions, `ignore` list in config (project-wide) for team decisions

### AI Collaboration Keys
- AI generates complete configurations matching team preferences without requiring TOML fluency
- AI resolves tool conflicts (Ruff + Pyright disagreements) through coordinated configuration

### Common Mistakes
- Memorizing TOML syntax instead of asking AI for generation
- Committing inconsistent configurations (defeats team standardization)
- Ignoring entire rule categories when single-rule exceptions needed (too permissive)
- Format-on-save not working in editor (LSP/settings configuration needed)

### Connections
- **Builds on**: Basic Ruff usage, understanding pyproject.toml role, editor configuration (Zed)
- **Leads to**: Type checking configuration (Pyright), complete quality toolchain integration, CI/CD automation

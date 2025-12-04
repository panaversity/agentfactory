### Core Concept
Configuration and secrets (API keys, passwords, tokens) must live in environment variables or `.env` files—never hardcoded in code. If secrets end up in your code and reach a repository, anyone with access gets your credentials.

### Key Mental Models
- **Temporary vs. Persistent**: `export` sets values for the current terminal session only; `.env` files persist across restarts
- **Separation of Secrets from Code**: Code reads configuration from the environment; the environment holds the actual values—this keeps secrets out of version control
- **Verification Before Trust**: Always verify configuration is set (`echo $VAR`) and protected (`.gitignore`) before assuming it's safe

### Critical Patterns
- Use `export VAR="value"` for temporary testing configuration
- Use `echo $VAR` to verify a variable is set correctly
- Create `.env` files for persistent configuration that survives terminal restarts
- Add `.env` to `.gitignore` to prevent accidental commits to version control
- Read secrets in code via environment: `os.getenv('API_KEY')` not hardcoded strings

### Common Mistakes
- Hardcoding secrets directly in code (`API_KEY = "sk-abc123..."`)—exposes credentials if code is shared
- Forgetting that `export` variables disappear when terminal closes—not suitable for persistent needs
- Creating `.env` without adding it to `.gitignore`—secrets still end up in repositories

### Connections
- **Builds on**: File operations and redirection (earlier lessons in Chapter 7)
- **Leads to**: Using API keys securely in AI tool configurations

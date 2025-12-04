### Core Concept
Installation is a simple, deterministic task requiring direct commands (no AI needed), but environment troubleshooting (PATH configuration, permission issues) becomes valuable when problems arise—learning this distinction shapes when to use AI for execution vs. diagnosis.

### Key Mental Models
- **Task Categorization**: Simple, deterministic tasks (installation) use direct commands; complex, ambiguous problems (troubleshooting) leverage AI for diagnosis and context-aware fixes
- **PATH Environment Registry**: Computer's command discovery system; installation scripts automatically add UV to PATH, making it accessible system-wide without manual configuration
- **Installation Verification Checkpoint**: Closing/reopening terminal (forces PATH reload) and running `uv --version` creates binary success/failure signal, cleanly separating installation from post-install issues

### Critical Patterns
- **Installation Security Pattern**: `curl <url> | sh` downloads and executes immediately; production practice inspects script before execution
- **Platform-Specific Variation**: macOS/Linux use curl + bash; Windows uses PowerShell; each has different PATH configuration and restart requirements

### AI Collaboration Keys
- AI diagnoses platform-specific PATH issues, permission errors, network failures with context-aware suggestions
- AI explains installation mechanisms rather than reciting steps, building understanding for future tool installations

### Common Mistakes
- Not restarting terminal after installation (PATH reload required)
- Assuming "command not found" always means failed installation (often PATH configuration issue)
- Running installation scripts without understanding what `curl | sh` does (security concern)

### Connections
- **Builds on**: Basic terminal navigation, understanding platform differences (Windows/Mac/Linux)
- **Leads to**: Creating UV projects, managing dependencies, running code in isolated environments

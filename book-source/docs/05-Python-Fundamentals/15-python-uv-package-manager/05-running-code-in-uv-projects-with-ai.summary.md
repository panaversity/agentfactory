### Core Concept
`uv run` activates project environments automatically (without manual activation), enabling scripts and tests to access project-specific packages in isolation—eliminating environment mismatches and "works on my machine" problems through transparent environment switching.

### Key Mental Models
- **Automatic Environment Activation**: `uv run` invisibly activates `.venv/` and its packages; unlike legacy `source .venv/bin/activate`, no manual shell configuration needed, enabling seamless development
- **Isolation Transparency**: System Python and project-specific packages coexist; `python` fails (system can't find requests), `uv run python` succeeds (uses project environment)—demonstrates isolation value concretely
- **Command Polymorphism**: `uv run python script.py`, `uv run pytest`, `uv run uvicorn` all follow identical pattern—once learned, applies to any project tool
- **Error Diagnostics**: ModuleNotFoundError indicates missing dependency (add with `uv add`), not environment misconfiguration, enabling direct fixes

### Critical Patterns
- **Multi-Project Coexistence Proof**: Project A uses requests 2.28, Project B uses 2.31; `uv run` in each project confirms isolation—no conflicts, no overwriting
- **Test Execution in Context**: `uv run pytest` runs tests with project-specific packages, matching production environment exactly

### AI Collaboration Keys
- AI helps diagnose environment errors, distinguishing between "package not installed" and "environment not activated"
- AI walks through test execution and debugging when failures occur in isolated environments

### Common Mistakes
- Running `python` directly instead of `uv run python` (system Python doesn't have project packages)
- Trying to manually activate `.venv/` (unnecessary with `uv run`, confuses process)
- Assuming permission errors on executables mean environment issues (usually require `uv run` pattern)

### Connections
- **Builds on**: Project creation, dependency management, understanding virtual environments
- **Leads to**: Writing and testing Python code, team collaboration, production deployments

### Core Concept
A UV project is a structured folder with configuration (`pyproject.toml`) and isolated packages (`.venv/`) that tell Python what your code needs—replacing legacy `requirements.txt` with modern, metadata-rich standards supporting development vs. production dependency distinction.

### Key Mental Models
- **Project Structure Abstraction**: `uv init` generates complete structure (pyproject.toml, .python-version, src/, .venv/) automatically; students understand purpose (project metadata, version pinning, isolation) not manual construction
- **Virtual Environment Isolation**: Each project gets separate "toolbox" preventing version conflicts between projects (Project A's pandas 1.5 coexists with Project B's pandas 2.0)
- **Modern vs. Legacy Configuration**: `pyproject.toml` (current standard) vs. `requirements.txt` (legacy); modern approach supports project metadata, tool configuration, development dependencies
- **Specification vs. Installation Distinction**: `pyproject.toml` lists what you need; `uv.lock` pins exact versions; `.venv/` holds actual installed packages

### Critical Patterns
- **Production Lean Principle**: .venv/ not committed to git (50+ MB, recreated from lockfile in seconds per teammate); git tracks pyproject.toml + uv.lock only
- **Dependency Transitive Chain**: `uv add requests` installs requests + 4+ dependencies automatically; UV resolves entire tree, students only specify direct needs

### AI Collaboration Keys
- AI explains pyproject.toml sections and virtual environment benefits using analogies (separate toolboxes, project recipes)
- AI helps understand why environment isolation matters through concrete multi-project examples

### Common Mistakes
- Committing .venv/ to git (unnecessary, bloats repository)
- Confusing specification (what you ask for) with installation (what gets downloaded)
- Assuming Python projects share packages globally (violates team development patterns)

### Connections
- **Builds on**: Understanding dependencies, tool installation (UV itself)
- **Leads to**: Managing dependencies (add/remove/update), running code in isolated environments, team collaboration with lockfiles

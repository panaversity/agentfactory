### Core Concept
Reproducible environments solve "works on my machine" through two-file coordination—`pyproject.toml` expresses constraints (flexible), `uv.lock` pins exact versions (strict)—enabling teammates to recreate identical environments from lockfiles and ensuring version consistency across developers, branches, and deployments.

### Key Mental Models
- **File Duality Philosophy**: `pyproject.toml` (recipe: what you want roughly) vs. `uv.lock` (exact meal: tested exact versions); both required—flexibility for individual development, exactness for team synchronization
- **Lockfile Reproducibility Contract**: `uv.lock` guarantees Maria gets your exact environment when cloning; `uv sync` reads lockfile and recreates `.venv/` identically, no surprises
- **Git Integration Pattern**: Commit both configuration files (enable team sync), ignore `.venv/` (everyone regenerates locally), ensure `.gitignore` auto-configured by `uv init`
- **Deployment Lean Principle**: `uv sync --no-dev` installs production dependencies only, removing pytest/black from production, reducing attack surface and startup time

### Critical Patterns
- **Onboarding Speed**: New developer clones, runs `uv sync` (30 seconds), runs `uv run pytest` (verify environment)—ready to develop without documentation
- **Lockfile Conflict Resolution**: Merge conflicts in `uv.lock` resolved by `uv lock` (regenerates consistently), not manual editing

### AI Collaboration Keys
- AI guides through team scenarios: adding dependencies mid-project, syncing after pulls, handling lockfile conflicts
- AI explains workflow differences between individual development (flexible `pyproject.toml`) and team synchronization (strict `uv.lock`)

### Common Mistakes
- Committing `.venv/` (50+ MB, breaks clones, forces regeneration anyway)
- Manually editing `uv.lock` (auto-generation ensures consistency)
- Forgetting to commit updated `uv.lock` (teammates get stale environment)
- Using `uv add` without committing lockfile changes (breaks reproducibility contract)

### Connections
- **Builds on**: Project creation, dependency management, understanding isolation and environment configuration
- **Leads to**: Production deployments, CI/CD integration, team workflows at scale

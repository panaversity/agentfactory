# Contributing to AI Native Software Development

This is an **Nx monorepo** containing multiple projects with different technology stacks:
- **JS/TS projects** (Docusaurus, plugins) - managed with `pnpm`
- **Python projects** (MCP server) - managed with `uv`

## Prerequisites

- Node.js 20+
- pnpm 9.12+ (`npm install -g pnpm`)
- Python 3.12+
- uv (`curl -LsSf https://astral.sh/uv/install.sh | sh`)

## Initial Setup

```bash
# Clone the repository
git clone https://github.com/panaversity/ai-native-software-development.git
cd ai-native-software-development

# Install JS/TS dependencies (from root)
pnpm install

# Install Python dependencies (from root - uv workspace handles all Python projects)
uv sync --all-extras
```

## Project Structure

```
/
├── apps/
│   ├── learn-app/           # Docusaurus site (JS/TS)
│   └── panaversity-fs-py/   # MCP server (Python)
├── libs/
│   └── docusaurus/          # Shared Docusaurus plugins (JS)
├── nx.json                  # Nx workspace config
├── package.json             # Root JS dependencies
├── pnpm-workspace.yaml      # pnpm workspace config (JS/TS)
├── pyproject.toml           # uv workspace config (Python)
└── uv.lock                  # Python lockfile (all projects)
```

### Workspace Architecture

This monorepo uses **two workspace systems** running in parallel:

| Stack | Package Manager | Workspace Config | Lockfile |
|-------|-----------------|------------------|----------|
| JS/TS | pnpm | `pnpm-workspace.yaml` | `pnpm-lock.yaml` |
| Python | uv | `pyproject.toml` ([tool.uv.workspace]) | `uv.lock` |

**Nx** orchestrates tasks across both stacks but doesn't manage dependencies.

## Running Commands

### Using Nx (Recommended)

Run tasks for specific projects from the **root**:

```bash
# Build a specific project
pnpm nx build learn-app
pnpm nx build panaversity-fs-py

# Run tests
pnpm nx test panaversity-fs-py

# Run linting
pnpm nx lint learn-app

# Run multiple targets
pnpm nx run-many -t build test

# Run affected (only changed projects)
pnpm nx affected -t build test lint
```

### Direct Commands (Per Project)

#### JS/TS Projects (learn-app, plugins)

```bash
cd apps/learn-app
pnpm dev          # Start dev server
pnpm build        # Production build
pnpm lint         # Run ESLint
```

#### Python Projects (panaversity-fs-py)

```bash
cd apps/panaversity-fs-py
make test         # Run tests
make lint         # Run ruff linter
make format       # Format with ruff
make build        # Build package
```

## Adding Dependencies

### JS/TS Dependencies

**From root (preferred):**
```bash
# Add to specific project
pnpm add <package> --filter learn-app
pnpm add -D <package> --filter learn-app  # dev dependency

# Add to root (shared tooling only)
pnpm add -D <package> -w
```

**From project directory:**
```bash
cd apps/learn-app
pnpm add <package>
```

### Python Dependencies

**Always from project directory:**
```bash
cd apps/panaversity-fs-py

# Add runtime dependency
uv add <package>

# Add dev dependency
uv add --group dev <package>

# Sync after changes
uv sync --extra dev
```

## Creating New Projects

### Option 1: JS/TS App with Nx Generators (Recommended)

First, add the appropriate Nx plugin:

```bash
# From root - add plugin for your tech stack
pnpm nx add @nx/next    # Next.js apps
pnpm nx add @nx/react   # React apps
pnpm nx add @nx/node    # Node/Express backends
```

Then generate the app:

```bash
# Generate new app
pnpm nx g @nx/next:app apps/my-new-app
pnpm nx g @nx/node:app apps/my-api

# Verify it shows in graph
pnpm nx graph --focus=my-new-app
```

### Option 2: Manual JS/TS Project

```bash
# Create directory
mkdir -p apps/my-new-app
cd apps/my-new-app

# Initialize package.json
pnpm init

# Add to pnpm workspace (already configured via apps/*)
```

Create `apps/my-new-app/project.json`:
```json
{
  "name": "my-new-app",
  "projectType": "application",
  "targets": {
    "build": {
      "command": "pnpm build",
      "options": { "cwd": "apps/my-new-app" }
    },
    "dev": {
      "command": "pnpm dev",
      "options": { "cwd": "apps/my-new-app" }
    }
  }
}
```

### Option 3: Python Project (Manual Setup)

Python projects use uv workspaces for dependency management and manual `project.json` for Nx:

```bash
# 1. Create directory and initialize
mkdir -p apps/my-python-app
cd apps/my-python-app
uv init

# 2. Add dev dependencies
uv add --group dev pytest ruff mypy
cd ../..

# 3. Add to uv workspace (edit root pyproject.toml)
```

Edit `pyproject.toml` at root to include the new project:
```toml
[tool.uv.workspace]
members = [
    "apps/panaversity-fs-py",
    "apps/my-python-app",  # <-- Add this line
]
```

```bash
# 4. Sync all Python dependencies from root
uv sync --all-extras
```

Create `apps/my-python-app/project.json` for Nx integration:
```json
{
  "name": "my-python-app",
  "projectType": "application",
  "targets": {
    "build": {
      "command": "uv build",
      "options": { "cwd": "apps/my-python-app" }
    },
    "test": {
      "command": "uv run --extra dev pytest",
      "options": { "cwd": "apps/my-python-app" }
    },
    "lint": {
      "command": "uv run --extra dev ruff check .",
      "options": { "cwd": "apps/my-python-app" }
    },
    "format": {
      "command": "uv run --extra dev ruff format .",
      "options": { "cwd": "apps/my-python-app" }
    }
  }
}
```

#### Shared Python Libraries

To create a shared Python library that other Python apps can import:

```bash
mkdir -p libs/auth-common-py
cd libs/auth-common-py
uv init --lib
cd ../..
```

Add to workspace and then reference in dependent projects:
```toml
# apps/my-python-app/pyproject.toml
[project]
dependencies = ["auth-common-py"]

[tool.uv.sources]
auth-common-py = { workspace = true }
```

### Option 4: Shared Library

```bash
# JS/TS shared library
pnpm nx add @nx/js
pnpm nx g @nx/js:lib libs/shared-utils

# Or manually
mkdir -p libs/my-lib
# Add package.json + project.json
```

### Verify New Project

After adding any project:

```bash
# Verify Nx recognizes it
pnpm nx show projects

# View in project graph
pnpm nx graph

# Test the targets work
pnpm nx build my-new-app
pnpm nx test my-new-app
```

## Nx Tips

### View Project Graph
```bash
pnpm nx graph
```

### Check What's Affected
```bash
pnpm nx affected:graph
```

### Reset Cache
```bash
pnpm nx reset
```

### View Project Details
```bash
pnpm nx show project learn-app
pnpm nx show project panaversity-fs-py
```

## CI/CD

GitHub Actions runs on every PR:
- `nx affected -t lint test build` - Only runs tasks for changed projects
- Uses Nx caching for faster builds

## Common Issues

### "Module not found" in Python tests
Make sure to install dev dependencies from root:
```bash
uv sync --all-extras
```

### pnpm lockfile conflicts
```bash
pnpm install --no-frozen-lockfile
```

### Nx cache issues
```bash
pnpm nx reset
```

## Questions?

Open an issue or discussion on GitHub.

# Developer & Book Writer Guide

This guide explains how to work with the Nx monorepo structure.

## Quick Start

```bash
# 1. Clone the repository
git clone https://github.com/panaversity/ai-native-software-development.git
cd ai-native-software-development

# 2. Install dependencies (uses pnpm, not npm)
pnpm install

# 3. Start the dev server
nx serve learn-app

# 4. Open http://localhost:3000
```

## Project Structure

```
storage/
├── apps/
│   ├── learn-app/           # Docusaurus website (THE book)
│   │   ├── docs/            # Book content (markdown files)
│   │   ├── src/             # React components
│   │   ├── static/          # Static assets
│   │   └── docusaurus.config.ts
│   └── panaversity-fs-py/   # Python MCP server
│       ├── src/             # Python source
│       ├── tests/           # Pytest tests
│       └── scripts/         # Utility scripts
├── libs/docusaurus/         # Shared Docusaurus plugins
│   ├── remark-interactive-python/
│   ├── remark-content-enhancements/
│   ├── plugin-og-image/
│   ├── plugin-structured-data/
│   ├── summaries-plugin/
│   └── panaversityfs-plugin/
├── nx.json                  # Nx workspace configuration
├── pnpm-workspace.yaml      # pnpm workspace definition
└── pnpm-lock.yaml           # Locked dependencies
```

## For Book Writers

### Where to Edit Content

All book content lives in:

```
apps/learn-app/docs/
├── 01-Part-Name/
│   └── 01-chapter-name/
│       ├── README.md        # Chapter overview
│       └── 01-lesson.md     # Individual lessons
└── chapter-index.md         # Master index
```

### Workflow

1. **Edit content** in `apps/learn-app/docs/`
2. **Preview changes**: `nx serve learn-app`
3. **Build to verify**: `nx build learn-app`
4. **Commit and push**

### Key Files for Context

When working on chapters, Claude Code reads:

- `apps/learn-app/docs/chapter-index.md` - Chapter metadata
- `apps/learn-app/docs/[part]/[chapter]/README.md` - Chapter structure

## For Developers

### Common Commands

```bash
# === Website (learn-app) ===
nx serve learn-app        # Start dev server
nx build learn-app        # Production build
nx lint learn-app         # TypeScript checks

# === Python MCP Server ===
nx test panaversity-fs-py   # Run pytest
nx lint panaversity-fs-py   # Run linting
nx serve panaversity-fs-py  # Start MCP server

# === Workspace ===
nx show projects          # List all projects
nx graph                  # Visualize dependencies
nx affected -t build      # Build only changed projects
```

### Dependency Graph

```
learn-app ──depends on──┬── remark-interactive-python
                        ├── remark-content-enhancements
                        ├── plugin-og-image
                        ├── plugin-structured-data
                        ├── summaries-plugin
                        └── panaversityfs-plugin

panaversity-fs-py (standalone - no JS dependencies)
```

**What this means:**

- Change a plugin → learn-app automatically rebuilds
- Change Python code → only panaversity-fs-py tests run
- Change docs only → minimal rebuild

### How Nx Affected Works

Instead of building everything, Nx only builds what changed:

```bash
# Example: You edited a Python file
nx affected -t test --base=main
# Result: Only panaversity-fs-py tests run

# Example: You edited a plugin
nx affected -t build --base=main
# Result: Plugin + learn-app both rebuild (dependency chain)
```

### Caching

Nx caches build outputs. Second builds are instant:

```bash
nx build learn-app        # First: 45 seconds
nx build learn-app        # Second: 0.5 seconds (cached)
```

To clear cache: `nx reset`

## CI/CD

### What Happens on PR

1. `ci.yml` runs `nx affected -t lint test build`
2. Only affected projects are checked
3. Results shown in PR checks

### What Happens on Merge to Main

1. `deploy.yml` triggers
2. Builds `apps/learn-app`
3. Deploys to GitHub Pages

## Troubleshooting

### "Command not found: nx"

```bash
# Make sure you installed dependencies
pnpm install

# Use npx if nx isn't in PATH
npx nx serve learn-app
```

### "Module not found" errors

```bash
# Clear Nx cache and reinstall
nx reset
rm -rf node_modules
pnpm install
```

### Python tests failing

```bash
# Ensure uv is installed
pip install uv

# Run tests directly
cd apps/panaversity-fs-py
uv run pytest tests/ -v
```

### Build cache seems stale

```bash
# Reset Nx cache
nx reset

# Or skip cache for one run
nx build learn-app --skip-nx-cache
```

## Migration Notes

### Old Path → New Path

| Old                    | New                       |
| ---------------------- | ------------------------- |
| `book-source/`         | `apps/learn-app/`         |
| `apps/learn-app/docs/` | `apps/learn-app/docs/`    |
| `book-source/plugins/` | `libs/docusaurus/`        |
| `panaversity-fs/`      | `apps/panaversity-fs-py/` |
| `npm install`          | `pnpm install`            |
| `npm run build`        | `nx build learn-app`      |

### npm → pnpm

All commands now use pnpm:

```bash
# Old
npm install
npm run build

# New
pnpm install
nx build learn-app  # or: pnpm build (from apps/learn-app/)
```

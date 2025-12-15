# Nx Monorepo Structure Analysis & Design

**Date**: 2025-12-15
**Repository**: panaversity/ai-native-software-development
**Analyzer**: monorepo-agent (Analysis Mode)
**Status**: Design Complete - Ready for Phase 1 Implementation

---

## Executive Summary

The Panaversity repository has been analyzed and a complete Nx monorepo structure has been designed. This document serves as the single source of truth for Nx workspace architecture.

**Key findings**:
- **Complexity**: MEDIUM (2 production projects + 6 plugins, polyglot setup)
- **Recommendation**: Adopt Nx with incremental 3-phase migration
- **Timeline**: ~8-10 hours for complete migration
- **Risk**: HIGH in Phase 3 (CI/CD), LOW in Phases 1-2

---

## Repository Snapshot

### Current Structure

```
storage/
├── apps/learn-app/              # Docusaurus + 4 plugins (887M)
├── panaversity-fs/          # Python MCP server (150M)
├── docs/                     # Architecture documentation
├── context/                  # Context files
├── history/                  # PHRs and learning history
├── papers/                   # Research papers
├── specs/                    # Feature specifications
└── [other dirs]             # Config, hidden, etc.
```

### Projects Identified

| Project | Type | Tech Stack | Status | Build Tool |
|---------|------|-----------|--------|-----------|
| `book-source` | App | Docusaurus 3.9.2 + React 19 | Production (GitHub Pages) | npm |
| `plugins/remark-interactive-python` | Lib | JavaScript | Active | npm |
| `plugins/docusaurus-panaversityfs-plugin` | Lib | JavaScript | Active | npm |
| `plugins/remark-content-enhancements` | Lib | JavaScript | Active | npm |
| `plugins/docusaurus-plugin-og-image-generator` | Lib | JavaScript | Active | npm |
| `plugins/docusaurus-plugin-structured-data` | Lib | JavaScript | Active | npm |
| `plugins/docusaurus-summaries-plugin` | Lib | JavaScript | Active | npm |
| `panaversity-fs` | App | Python 3.13 + FastAPI + MCP | Production (Docker) | uv |

**Total**: 8 projects (2 apps + 6 libs)

---

## Why Nx?

### Comparison with Alternatives

| Criterion | Nx | Turborepo | Bazel | Lerna |
|-----------|----|-----------|----|-------|
| **MCP Support** | ✅ Official | ❌ No | ❌ No | ❌ No |
| **Polyglot** | ✅ Native + Executors | ❌ Node.js only | ✅ Yes | ❌ Node.js only |
| **Learning Curve** | 📊 1 day | 📊 1 day | 📊 3-6 months | 📊 1 day |
| **Cache Sharing** | ✅ Nx Cloud | ✅ Cloud | ✅ Built-in | ❌ Manual |
| **CI Integration** | ✅ `nx affected` | ✅ turbo | ⚠️ Complex | ❌ Manual |
| **Production Tested** | ✅ Yes (millions) | ✅ Yes | ⚠️ Google/Meta | ❌ Legacy |

**Decision**: Nx is the best fit for this polyglot, production-active repository.

---

## Target Architecture

### Phase 1 Result (Foundation)

```
storage/
├── nx.json                  ✅ New
├── pnpm-workspace.yaml      ✅ New
├── .nxignore                ✅ New
├── package.json             ✅ Updated
│
├── apps/
│   ├── website/             ✅ Moved from book-source
│   │   ├── project.json     ✅ New (minimal)
│   │   ├── package.json     (existing)
│   │   └── docs/, src/, static/, ...
│   │
│   └── panaversity-fs-py/   ✅ Moved from panaversity-fs
│       ├── project.json     ✅ New (minimal)
│       ├── pyproject.toml   (existing)
│       └── src/, tests/, ...
│
├── libs/
│   ├── remark-interactive-python/
│   ├── docusaurus-panaversityfs-plugin/
│   ├── remark-content-enhancements/
│   ├── docusaurus-plugin-og-image/
│   ├── docusaurus-plugin-structured-data/
│   └── docusaurus-summaries-plugin/
│
└── tools/
    ├── executors/
    │   ├── python-executor/
    │   │   ├── schema.json
    │   │   ├── impl.ts
    │   │   └── package.json
    │   └── project.json
    │
    └── scripts/
        ├── hydrate-book.py
        ├── ingest-book.py
        └── project.json
```

### Key Design Decisions

**Decision 1: Plugins as First-Class Projects**
- Plugins moved from `apps/learn-app/plugins/` to `libs/*/`
- Each has its own `package.json` and `project.json`
- Enables independent versioning and caching

**Decision 2: Python Isolation via Custom Executor**
- Python remains completely isolated (no Node.js dependencies)
- Custom Nx executor at `tools/executors/python-executor/`
- Allows orchestrating Python targets from Nx CLI

**Decision 3: pnpm for All Node.js**
- Consolidate from mixed npm/pnpm to pnpm only
- Define workspace in `pnpm-workspace.yaml`
- Enables efficient monorepo dependency resolution

**Decision 4: Incremental Migration**
- Phase 1: Directory reorganization (safe)
- Phase 2: Configuration & testing (safe)
- Phase 3: CI/CD integration (high-risk, tested on branch first)

---

## Files Created

### Configuration Files (5)

1. **nx.json**
   - Workspace configuration
   - Named inputs for caching (npmInputs, pythonInputs, docusaurusInputs)
   - Target defaults (cache settings, dependencies)

2. **pnpm-workspace.yaml**
   - Defines workspace packages
   - Enables monorepo dependency hoisting

3. **.nxignore**
   - Patterns excluded from Nx
   - Prevents re-scanning documentation, .git, etc.

4. **package.json** (root, updated)
   - Workspace-level scripts
   - Nx + Docusaurus dev dependencies
   - packageManager constraint (pnpm)

5. **tools/executors/project.json**
   - Build configuration for Python executor tool

### Project Configuration Files (8)

6. **apps/website/project.json**
   - Docusaurus build, serve, deploy targets
   - Hydrate/ingest targets for MCP integration
   - Lint, typecheck, test targets

7. **apps/panaversity-fs-py/project.json**
   - Python build, test, lint, typecheck targets
   - Database migration targets
   - Custom executor usage (see #8-10)

8-13. **libs/*/project.json** (6 plugin files)
   - Standard npm library targets
   - Build, test, lint configurations

14. **tools/scripts/project.json**
   - Shared build scripts
   - No build targets (content only)

### Python Executor Files (3)

15. **tools/executors/python-executor/schema.json**
   - Defines input parameters for Python executor
   - Properties: command, cwd, env

16. **tools/executors/python-executor/impl.ts**
   - Executor implementation
   - Runs Python commands in Nx context

17. **tools/executors/python-executor/package.json**
   - Executor package definition

---

## Migration Phases

### Phase 1: Foundation (30 min - 1 hr)
**Risk**: LOW | **Changes**: Directory structure + config files

**Steps**:
1. Create `apps/`, `libs/`, `tools/` directories
2. Move projects to new locations
3. Create minimal `project.json` files
4. Create `nx.json`, `pnpm-workspace.yaml`, `.nxignore`
5. Update root `package.json`
6. Run `pnpm install`
7. Validate: `nx show projects` → 8 projects

**CI Impact**: None (local changes only)

### Phase 2: Configuration (2-3 hrs)
**Risk**: LOW | **Changes**: Add targets to project.json files

**Steps**:
1. Add build, test, lint targets to `apps/website/project.json`
2. Add build, test, lint targets to `apps/panaversity-fs-py/project.json`
3. Update all plugin `project.json` files
4. Implement Python executor (`tools/executors/python-executor/`)
5. Test each target individually
6. Test `nx affected -t build` detection

**CI Impact**: None (changes isolated to project.json)

### Phase 3: CI/CD Integration (1-2 hrs)
**Risk**: HIGH | **Changes**: Update GitHub Actions workflows

**Steps**:
1. Update `.github/workflows/deploy.yml`
   - Use `nx affected -t build` instead of `npm run build`
   - Update script paths from `panaversity-fs/scripts/` to `tools/scripts/`
2. Update `.github/workflows/sync-content.yml`
   - Use Nx targets instead of direct script calls
3. Test on feature branch with manual workflow trigger
4. Merge to main during low-traffic period
5. Monitor deployment for 1 hour

**CI Impact**: Production deployments affected - requires testing before merge

---

## Nx Commands Reference

### Discovery & Visualization

```bash
# Show all projects
nx show projects

# Show project details
nx show project website --web

# Visualize dependency graph
nx graph

# Show affected projects (since main)
nx show projects --affected
```

### Build & Test

```bash
# Build all projects
nx run-many -t build

# Build affected only (CI-optimized)
nx affected -t build

# Test all
nx run-many -t test

# Test affected
nx affected -t test
```

### Specific Projects

```bash
# Build website
nx build website

# Test Python server
nx test panaversity-fs-py

# Hydrate content from MCP
nx hydrate website

# Deploy to GitHub Pages
nx deploy website
```

---

## Caching Strategy

### Inputs/Outputs Definition

| Project | Inputs | Outputs | Cache Key |
|---------|--------|---------|-----------|
| website | `docusaurusInputs` (docs/, src/, plugins/, package.json) | dist/apps/website, .docusaurus/content | MD5 of input files |
| panaversity-fs-py | `pythonInputs` (pyproject.toml, uv.lock, .py files) | dist/apps/panaversity-fs-py | MD5 of input files |
| plugins | `npmInputs` (package.json, .js, .ts) | dist/libs/{plugin}/ | MD5 of input files |

### Nx Cloud Integration (Phase 4, Optional)

```bash
# Connect workspace to Nx Cloud
npx nx connect

# Benefits:
# - Remote caching (CI builds cache to cloud)
# - Computation sharing (PRs reuse builds from main)
# - Faster feedback loops
```

---

## Key Design Patterns

### 1. Polyglot Monorepo Pattern

**Challenge**: Mixing Node.js + Python in one workspace

**Solution**:
- JavaScript/TypeScript projects use npm + Nx natively
- Python projects run via custom executor
- No cross-language dependencies (clean boundary)
- Orchestration through Nx CLI

**Example**:
```bash
# Both Node.js and Python targets in one command
nx run-many -t test
# Runs: jest for website + pytest for panaversity-fs-py
```

### 2. Plugin Architecture Pattern

**Challenge**: 6 plugins are sub-dependencies of main Docusaurus app

**Solution**:
- Plugins are first-class Nx projects in `libs/`
- Each has own `package.json` and `project.json`
- Main app depends on plugins through npm (not Nx)
- Enables independent versioning, testing, caching

**Dependency Flow**:
```
Docusaurus builds → reads libs/*/package.json
Nx builds plugins → outputs to dist/libs/*/
NPM resolves → finds built plugins
```

### 3. Build Script Sharing Pattern

**Challenge**: Python scripts (hydrate, ingest) referenced by both Node.js and CI

**Solution**:
- Scripts moved to `tools/scripts/`
- Exposed as Nx targets (`nx hydrate website`, `nx ingest website`)
- CI calls Nx targets instead of direct scripts
- Enables caching and dependency tracking

---

## Risk Assessment

### Low Risk (Phases 1-2)

**What can go wrong**: Dependency resolution issues, missing files

**Mitigation**:
- Validate at each step: `nx show projects`, `pnpm install`
- Keep originals for reference during transition
- Test locally before committing

**Likelihood**: LOW | **Impact**: Easily fixable

### High Risk (Phase 3)

**What can go wrong**: CI/CD workflow breaks, production deployment fails

**Mitigation**:
- Test on feature branch with manual workflow trigger
- Get team approval before merging
- Keep original CI code in comments for quick rollback
- Monitor deployment for 1 hour after merge
- Have rollback plan ready

**Likelihood**: MEDIUM | **Impact**: CRITICAL (production)

**Rollback Time**: < 5 minutes (git revert)

---

## Success Metrics

### Phase 1 Success

```bash
✅ nx show projects
   # Returns 8 projects

✅ nx graph --file=graph.json
   # Valid JSON file created

✅ pnpm install
   # Completes with no errors

✅ git status
   # Shows apps/, libs/, tools/, nx.json
```

### Phase 2 Success

```bash
✅ nx build website
   # Produces dist/apps/website

✅ nx test website
   # Runs tests or passes with no tests

✅ nx build panaversity-fs-py
   # Runs uv build successfully

✅ nx affected -t build --base=main
   # Correctly detects changed projects
```

### Phase 3 Success

```bash
✅ GitHub Pages deployment succeeds
   # Website loads at https://panaversity.ai

✅ Book content loads
   # Pages render, no 404 errors

✅ Python backend running
   # MCP server accessible, hydrate works

✅ Zero console errors
   # Browser console clean, no JavaScript errors
```

---

## Next Steps

1. **Review this document** (NX-MONOREPO-ANALYSIS.md)
2. **Read implementation guide**: NX-IMPLEMENTATION-GUIDE.md
3. **Read full specification**: NX-WORKSPACE-DESIGN.md
4. **Create feature branch**: `git checkout -b feat/nx-migration`
5. **Start Phase 1**: Follow step-by-step guide in NX-IMPLEMENTATION-GUIDE.md
6. **Validate Phase 1**: Run validation commands
7. **Get team approval** for Phase 3 (CI/CD changes)
8. **Merge to main** once Phase 1 & 2 complete and tested

---

## References

- **Full Design**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/NX-WORKSPACE-DESIGN.md`
- **Implementation Steps**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/NX-IMPLEMENTATION-GUIDE.md`
- **Migration Status**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/MIGRATION-SUMMARY.md`
- **Nx Documentation**: https://nx.dev/docs
- **pnpm Workspaces**: https://pnpm.io/workspaces

---

**Status**: Analysis Complete, Design Complete, Ready for Phase 1 Implementation
**Last Updated**: 2025-12-15
**Prepared by**: monorepo-agent (Analysis Mode)

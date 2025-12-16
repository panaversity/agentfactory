# panaversity-fs GitHub Workflows & Access Pattern Analysis

**Generated**: 2025-12-15
**Scope**: Analyzing all GitHub workflows and Python project access patterns for Nx monorepo migration
**Current State**: panaversity-fs is partially integrated into Nx (project.json exists) but workflows still use raw paths

---

## Executive Summary

The panaversity-fs Python project is integrated into the Nx monorepo but workflows still reference paths directly rather than using Nx orchestration. All 4 GitHub workflows touch panaversity-fs either directly or indirectly through the book-source deployment pipeline.

**Critical Finding**: The workflows can largely remain unchanged because they operate on relative paths within the monorepo structure. However, we need to:
1. Document all path dependencies (both relative and absolute)
2. Identify workflow trigger patterns that may change with Nx
3. Plan MCP server registration updates
4. Establish CI/CD affected-only testing strategy

---

## 1. GitHub Workflow Analysis

### 1.1 Workflow Touch Points

| Workflow | File | Touches panaversity-fs | When | Impact |
|----------|------|------------------------|------|--------|
| **PR Build Check** | `pr-check.yml` | NO (direct) | PR on `apps/learn-app/**` | Book-source only, doesn't invoke Python |
| **Sync Content** | `sync-content.yml` | YES | Push on `apps/learn-app/docs/**` | Calls `scripts/ingest-book.py` |
| **Deploy** | `deploy.yml` | YES | workflow_run + push | Calls `scripts/hydrate-book.py` |
| **Validate Content** | `validate-content.yml` | NO (direct) | PR on `apps/learn-app/docs/**` | Asset reference validation only |

### 1.2 Detailed Workflow Breakdown

#### Workflow 1: PR Build Check (`pr-check.yml`)

**Triggers**: PR on `apps/learn-app/**` branches
**Current Execution**: Only builds book-source npm package
**panaversity-fs Connection**: None (indirect via deployment, not triggered on PR)

```yaml
trigger: pull_request
  paths:
    - "apps/learn-app/**"
jobs:
  - Setup Node.js
  - Install book-source deps
  - Build book-source
```

**Nx Migration Impact**: LOW
- Can remain unchanged
- Does not depend on panaversity-fs
- Path filter works in Nx monorepo

---

#### Workflow 2: Sync Content (`sync-content.yml`)

**Triggers**: Push on `apps/learn-app/docs/**` or manual workflow_dispatch
**Current Execution**:
1. Checkout repo
2. Setup Python 3.12
3. Install minimal deps: `click, httpx, pydantic, python-dotenv`
4. Get changed files via `git diff`
5. Run `python scripts/ingest-book.py` from `panaversity-fs/` directory
6. Environment variables:
   - `PANAVERSITY_MCP_URL` = `secrets.PANAVERSITY_SERVER_URL`
   - `PANAVERSITY_API_KEY` = `secrets.PANAVERSITY_API_KEY`

**Key Path Dependencies**:
```bash
cd panaversity-fs
python scripts/ingest-book.py \
  --book-id "ai-native-dev" \
  --source-dir "../apps/learn-app/docs"   # RELATIVE path
  --verbose
```

**Current Working Directory**: Root monorepo
**Execution Directory**: `panaversity-fs/`

**Nx Migration Impact**: MEDIUM
- Uses relative paths (`../apps/learn-app/docs`) that will remain valid
- Script relies on `scripts/ingest/cli.py` which uses `sys.path.insert(0, str(scripts_dir.parent))`
- Can keep current path structure as-is
- Consider adding `--project panaversity-fs` for Nx awareness

---

#### Workflow 3: Deploy (`deploy.yml`)

**Triggers**:
- workflow_run (after sync-content success/skip)
- Direct push on `apps/learn-app/**` (excluding docs/ which go via sync)
- Manual workflow_dispatch with optional full_rebuild

**Current Execution** (Python portion):
1. Setup Python 3.12 (conditional: `if PANAVERSITY_PLUGIN_ENABLED`)
2. Install deps: `pip install click httpx pydantic python-dotenv` (working dir: `panaversity-fs/`)
3. Create directories: `.panaversity/`, `build-source/`
4. Run `python scripts/hydrate-book.py`:
   ```bash
   cd panaversity-fs
   python scripts/hydrate-book.py \
     --book-id "ai-native-dev" \
     --output-dir "../build-source" \
     --manifest-file "../.panaversity/manifest.json" \
     [--full-rebuild or incremental via manifest]
   ```
5. Fallback to local docs if hydration fails
6. Setup Node.js and build Docusaurus
7. Deploy to GitHub Pages

**Path Dependencies**:
```
Relative to monorepo root:
  panaversity-fs/scripts/hydrate-book.py
  .panaversity/manifest.json           (cache key)
  build-source/                         (output)
  apps/learn-app/docs/                     (fallback)
  apps/learn-app/build/                    (final artifact)
```

**Secrets Used**:
- `PANAVERSITY_SERVER_URL` (env var: `PANAVERSITY_MCP_URL`)
- `PANAVERSITY_API_KEY` (env var: `PANAVERSITY_API_KEY`)
- `GA4_MEASUREMENT_ID`
- `AUTH_URL`, `OAUTH_CLIENT_ID`, `BASE_URL`

**Environment Variables Passed to Docusaurus**:
```bash
PANAVERSITY_PLUGIN_ENABLED
PANAVERSITY_SERVER_URL
AUTH_URL
OAUTH_CLIENT_ID
BASE_URL
PANAVERSITY_API_KEY
```

**Nx Migration Impact**: HIGH
- Manifest cache uses absolute path key: `panaversity-manifest-` (no project scope)
- Should update to: `panaversity-fs-manifest-` for clarity
- Multi-step orchestration: Python script → Node build → GitHub Pages
- Consider using Nx run-commands with proper working-directory

---

#### Workflow 4: Validate Content (`validate-content.yml`)

**Triggers**: PR on `apps/learn-app/docs/**`
**Current Execution**:
1. Get changed markdown files via git diff
2. Grep for local asset references (`/img/`, `/slides/`)
3. Fail if found (enforce CDN usage)

**panaversity-fs Connection**: None
**Nx Migration Impact**: LOW - Pure content validation, independent of Python project

---

### 1.3 Cross-Project Dependencies

**Book-source → panaversity-fs**:
- Docusaurus config references: `./plugins/docusaurus-panaversityfs-plugin` (Node plugin)
- Plugin communicates with panaversity-fs MCP server (at runtime, not build-time)
- Hydration script called from deploy workflow (direct invocation)

**panaversity-fs → book-source**:
- Scripts/hydrate outputs to `../build-source/` (monorepo-relative)
- Scripts/ingest reads from `../apps/learn-app/docs/` (monorepo-relative)

**No direct npm/Python dependencies** between projects

---

## 2. Python Project Access Patterns

### 2.1 Import Patterns

**Internal Package Structure**:
```python
from panaversity_fs.database import FileJournal, AuditLog, get_session
from panaversity_fs.tools.content import read_content
from panaversity_fs.tools.bulk import get_book_archive
from panaversity_fs.tools.registry import list_books
from panaversity_fs.storage import get_operator
```

**Installation Method**: Direct imports work because:
1. `pyproject.toml` defines `packages = ["src/panaversity_fs"]`
2. Tests use standard `pytest` discovery (no path manipulation needed)

**Scripts Use Path Insertion** (legacy pattern):
```python
# scripts/hydrate-book.py and scripts/ingest-book.py
scripts_dir = Path(__file__).parent
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir.parent))
from scripts.hydrate.cli import main
```

**Concern**: This assumes panaversity-fs is the root working directory or parent is accessible. In Nx context, this could be brittle.

### 2.2 Pyproject.toml Configuration

```toml
[project]
name = "panaversity-fs"
version = "0.1.0"
requires-python = ">=3.13"

[project.optional-dependencies]
dev = [pytest, black, ruff, mypy, ...]

[tool.pytest.ini_options]
testpaths = ["tests"]
python_files = ["test_*.py"]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.hatch.build.targets.wheel]
packages = ["src/panaversity_fs"]
```

**Nx Integration**: Already has `project.json` with targets:
- `install`, `test` (unit/integration/property/performance), `lint`, `format`, `typecheck`, `validate`
- `dev` (FastAPI server), `build`, `ingest`, `hydrate`, `clean`

### 2.3 MCP Server Registration

**Current `.mcp.json`**:
```json
{
  "mcpServers": {
    "nx-mcp": {...},
    "context7": {...},
    "deepwiki": {...},
    "playwright": {...},
    "better-auth": {...},
    "next-devtools": {...}
  }
}
```

**Missing**: panaversity-fs is NOT registered as an MCP server in `.mcp.json`

**Expected Entry** (for future integration):
```json
{
  "panaversity-fs": {
    "type": "stdio",
    "command": "python",
    "args": [
      "-m", "panaversity_fs.mcp"
    ],
    "env": {
      "PYTHONPATH": "${MONOREPO_ROOT}/panaversity-fs/src:${PYTHONPATH}"
    }
  }
}
```

Or using uv:
```json
{
  "panaversity-fs": {
    "type": "stdio",
    "command": "uv",
    "args": [
      "run", "-p", "panaversity-fs",
      "python", "-m", "panaversity_fs.mcp"
    ]
  }
}
```

---

## 3. Path Migration Reference Table

### Current vs. Post-Nx Paths

| Component | Current Path | Nx Path | Status | Notes |
|-----------|--------------|---------|--------|-------|
| **Python source** | `panaversity-fs/src/` | `panaversity-fs/src/` | UNCHANGED | Already relative in monorepo |
| **Tests** | `panaversity-fs/tests/` | `panaversity-fs/tests/` | UNCHANGED | Already relative |
| **Scripts** | `panaversity-fs/scripts/` | `panaversity-fs/scripts/` | UNCHANGED | Already relative |
| **Pyproject** | `panaversity-fs/pyproject.toml` | `panaversity-fs/pyproject.toml` | UNCHANGED | Path-independent |
| **Workflow input** | `apps/learn-app/docs/**` | `apps/learn-app/docs/**` | UNCHANGED | Path filter works in Nx |
| **Hydrate output** | `build-source/` | `build-source/` | UNCHANGED | Relative to monorepo root |
| **Manifest cache** | `.panaversity/manifest.json` | `.panaversity/manifest.json` | UNCHANGED | Root-level dotfile |
| **Fallback docs** | `apps/learn-app/docs/` | `apps/learn-app/docs/` | UNCHANGED | Relative path works |

### Workflow Working Directory Changes

| Workflow Step | Current `cwd` | Nx Target | Recommended `cwd` |
|---------------|---------------|-----------|-------------------|
| Install deps | `panaversity-fs/` | `nx install panaversity-fs` | Use Nx working-directory |
| Ingest script | `panaversity-fs/` → `python scripts/ingest-book.py` | `nx run panaversity-fs:ingest` | Already configured |
| Hydrate script | `panaversity-fs/` → `python scripts/hydrate-book.py` | `nx run panaversity-fs:hydrate` | Already configured |

---

## 4. CI/CD Migration Requirements

### 4.1 Current Workflow Paths Issues

**Issue 1: Hardcoded pip install in workflows**
```bash
# Current (deploy.yml, sync-content.yml)
working-directory: panaversity-fs
run: pip install click httpx pydantic python-dotenv
```

**Problem**: Only installs minimal deps, full pyproject.toml not installed
**Solution**: Use `pip install -e .[dev]` or `uv pip install -e .`

**Recommendation**:
```bash
# Use uv for speed + repeatability
working-directory: panaversity-fs
run: |
  uv pip install -e .
  # Or via Nx:
  # nx install panaversity-fs
```

---

### 4.2 Affected-Only Testing Strategy

**Current**: Workflows trigger based on path filters

**Recommended for Nx**:
```bash
# Check if panaversity-fs was affected
if nx show projects --affected --projects panaversity-fs | grep -q "panaversity-fs"; then
  # Run panaversity-fs tests
  nx affected -t test
else
  # Skip Python tests if only book-source changed
  echo "panaversity-fs not affected, skipping tests"
fi
```

**PR Check Workflow Migration**:
```yaml
# Current: only checks book-source
# Nx: Should also run tests if panaversity-fs changed

- name: Check which projects affected
  id: affected
  run: |
    AFFECTED=$(nx show projects --affected)
    echo "affected=$AFFECTED" >> $GITHUB_OUTPUT

- name: Test affected projects
  if: contains(steps.affected.outputs.affected, 'panaversity-fs')
  run: nx affected -t test --projects=panaversity-fs
```

---

### 4.3 Manifest Cache Scoping

**Current**:
```yaml
- uses: actions/cache@v4
  with:
    path: .panaversity/manifest.json
    key: panaversity-manifest-${{ github.sha }}
    restore-keys: |
      panaversity-manifest-
```

**Problem**: Key doesn't indicate which project owns manifest
**Recommendation**:
```yaml
- uses: actions/cache@v4
  with:
    path: .panaversity/manifest.json
    key: panaversity-fs-manifest-${{ hashFiles('apps/learn-app/docs/**/*') }}
    restore-keys: |
      panaversity-fs-manifest-
```

This ensures cache invalidates when docs actually change, not just on every commit.

---

### 4.4 Secret Management

**Current Secrets Used**:
- `PANAVERSITY_SERVER_URL` (MCP server endpoint)
- `PANAVERSITY_API_KEY` (Authentication)
- `GA4_MEASUREMENT_ID` (Analytics)
- `AUTH_URL`, `OAUTH_CLIENT_ID`, `BASE_URL` (Docusaurus auth)

**Path Dependency**: None - all secrets are environment variables

**Nx Impact**: No changes needed, secrets management unchanged

---

## 5. MCP Server Registration Plan

### 5.1 Current Status

**Not registered** in `.mcp.json`

### 5.2 Registration Requirements

panaversity-fs provides MCP tools for:
- Content ingestion (ingest-book.py)
- Hydration (hydrate-book.py)
- Book registry queries
- Content search and retrieval

### 5.3 Recommended Registration

**Option A: Using Python -m (FastAPI server)**
```json
{
  "panaversity-fs": {
    "type": "stdio",
    "command": "python",
    "args": ["-m", "panaversity_fs.server"],
    "env": {
      "PYTHONPATH": "/path/to/panaversity-fs/src"
    }
  }
}
```

**Option B: Using uv (monorepo-aware)**
```json
{
  "panaversity-fs": {
    "type": "stdio",
    "command": "uv",
    "args": ["run", "-p", "panaversity-fs", "python", "-m", "panaversity_fs.server"]
  }
}
```

**Option C: Using Nx (preferred)**
```json
{
  "panaversity-fs": {
    "type": "stdio",
    "command": "nx",
    "args": ["run", "panaversity-fs:server"]
  }
}
```

### 5.4 Server Entry Point

Requires adding to `panaversity-fs/src/panaversity_fs/server.py` or `__main__.py`:
```python
if __name__ == "__main__":
    # Start MCP server
    # Register tools for ingestion, hydration, etc.
    # Listen on stdio
```

---

## 6. Risk Assessment

### 6.1 High Risk Areas

| Risk | Current Mitigation | Nx Mitigation |
|------|-------------------|---------------|
| **Hardcoded paths in scripts** | Scripts use relative paths from `panaversity-fs/` | Keep relative, document in Nx |
| **sys.path manipulation** | Scripts insert parent dir to path | Update to use proper imports or PYTHONPATH |
| **Manifest cache invalidation** | No content-based hashing | Add `hashFiles()` to cache key |
| **Multi-repo coordination** | Deploy triggers on workflow_run | Nx can detect cross-project changes |

### 6.2 Medium Risk Areas

| Risk | Current State | Nx Impact |
|------|---------------|-----------|
| **Python version mismatch** | Workflows specify 3.12 | Ensure `.python-version` is source of truth |
| **Dependency drift** | pip install hardcoded list | Use pyproject.toml via Nx install target |
| **Windows/macOS CI** | Only ubuntu-latest | Nx doesn't change this |

### 6.3 Low Risk Areas

| Risk | Current State | Nx Impact |
|------|---------------|-----------|
| **Node.js build** | Separate from Python | Nx can orchestrate but independent |
| **Secrets management** | Already robust | No changes needed |
| **GitHub Pages deployment** | Standard workflow | No changes needed |

---

## 7. Recommended Migration Approach

### Phase 1: Minimal Changes (Week 1)

1. **Update workflow pip install** to use full pyproject.toml
   ```bash
   pip install -e panaversity-fs[dev]
   ```

2. **Add manifest cache hashing**
   ```yaml
   key: panaversity-fs-manifest-${{ hashFiles('apps/learn-app/docs/**/*') }}
   ```

3. **Document path conventions** in README

### Phase 2: Nx Integration (Week 2)

1. **Update workflows to use Nx commands**:
   ```bash
   nx run panaversity-fs:hydrate
   nx affected -t test
   ```

2. **Add panaversity-fs to .mcp.json**:
   ```json
   {
     "panaversity-fs": {
       "type": "stdio",
       "command": "uv",
       "args": ["run", "-p", "panaversity-fs", "python", "-m", "panaversity_fs.server"]
     }
   }
   ```

3. **Create MCP server entry point** if not exists

### Phase 3: Optimization (Week 3)

1. **Implement affected-only testing** in PR checks
2. **Add cross-project dependency tracking** in Nx
3. **Profile and cache Python builds** in CI

---

## 8. Implementation Checklist

### 8.1 Workflow Updates

- [ ] Update `sync-content.yml`:
  - Use `pip install -e panaversity-fs[dev]` instead of hardcoded list
  - Consider using `nx run panaversity-fs:ingest` instead of direct invocation
  - Add Nx affected detection

- [ ] Update `deploy.yml`:
  - Use `pip install -e panaversity-fs[dev]`
  - Consider using `nx run panaversity-fs:hydrate` instead of direct invocation
  - Add manifest cache key hashing
  - Add panaversity-fs affected detection

- [ ] Update `pr-check.yml`:
  - Add conditional Python test step if panaversity-fs affected
  - Reference: `nx show projects --affected`

- [ ] Keep `validate-content.yml` unchanged (no Python dependency)

### 8.2 Configuration Updates

- [ ] Update `panaversity-fs/project.json`:
  - Verify all targets are properly configured
  - Add any missing Nx-specific options

- [ ] Create `.mcp.json` entry for panaversity-fs:
  - Add server registration (uv or Python)
  - Document required environment variables

- [ ] Update `.python-version` handling:
  - Ensure GitHub Actions uses same Python as local
  - Reference: `actions/setup-python@v5` with version from file

### 8.3 Documentation

- [ ] Add `PANAVERSITY-FS-WORKFLOW-GUIDE.md`:
  - How to run workflows locally
  - How to test hydration/ingestion
  - Troubleshooting guide

- [ ] Update `panaversity-fs/README.md`:
  - Document path assumptions
  - Add Nx command examples

- [ ] Update root README:
  - Document monorepo structure
  - Note on panaversity-fs being conditional dependency

---

## 9. Quick Reference: File Changes Needed

### Files Requiring Updates

```
.github/workflows/
  ├── sync-content.yml          # Update pip install, add Nx awareness
  ├── deploy.yml                # Update pip install, add cache hashing, Nx awareness
  ├── pr-check.yml              # Add conditional Python testing
  └── validate-content.yml      # No changes needed

.mcp.json                         # Add panaversity-fs registration

panaversity-fs/
  ├── project.json              # Verify Nx targets (already good)
  ├── pyproject.toml            # Add [project.scripts] entry if MCP server
  └── README.md                 # Add Nx integration docs

Root README.md                    # Document monorepo structure

docs/                            # Create new guide for workflows
  └── PANAVERSITY-FS-WORKFLOW-GUIDE.md
```

### Files NOT Requiring Changes

- `panaversity-fs/scripts/hydrate-book.py` (relative paths work fine)
- `panaversity-fs/scripts/ingest-book.py` (relative paths work fine)
- `panaversity-fs/pyproject.toml` (no path dependencies)
- `apps/learn-app/**` (independent of panaversity-fs paths)

---

## 10. Validation Checklist

Before considering this analysis complete:

- [ ] All GitHub workflow files reviewed (4 files)
- [ ] All path dependencies documented (working dirs, relative paths)
- [ ] MCP server registration requirements identified
- [ ] Risk assessment completed
- [ ] Implementation steps are actionable
- [ ] No contradictions with existing Nx configuration
- [ ] Cross-project dependencies mapped
- [ ] Secrets management reviewed

---

## Summary: Key Findings

1. **Path Structure**: panaversity-fs uses relative paths that work in monorepo - NO major changes needed
2. **Workflows**: 2 of 4 workflows touch Python project (sync-content, deploy)
3. **Integration**: Already has project.json with Nx targets configured
4. **Missing**: MCP server registration and py script improvements
5. **Opportunity**: Can implement affected-only testing to speed up CI
6. **Risk**: Low - relative paths are monorepo-safe

**Next Action**: Implement Phase 1 (minimal changes) to improve robustness, then Phase 2 to unlock full Nx benefits.

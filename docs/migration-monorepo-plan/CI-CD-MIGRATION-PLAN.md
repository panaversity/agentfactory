# CI/CD Pipeline Analysis & Nx Migration Plan

**Analysis Date**: 2025-12-15
**Status**: Production deployment active (high caution required)
**Complexity**: MEDIUM ⚠️
**Risk Level**: HIGH (CI/CD phase directly impacts production)

---

## EXECUTIVE SUMMARY

This monorepo has **2 major projects** with **4 GitHub Actions workflows** currently managing deployment. The setup is **production-critical** with auto-deploy on main branch pushes.

| Aspect | Current State | Post-Nx |
|--------|---------------|---------|
| **Build**: Per-project scripts | Unified `nx affected` |
| **Caching**: Basic npm cache | Nx distributed cache |
| **CI Runs**: Full build always | `nx affected` only |
| **Deployment**: Manual (rarely triggered) | Auto on push |
| **Secrets**: 7 env vars | Same (no changes) |

---

## PHASE 1: CURRENT STATE DOCUMENTATION

### 1.1 GitHub Actions Workflows

#### Workflow 1: `pr-check.yml` (PR Validation)
**File**: `.github/workflows/pr-check.yml`

| Property | Value |
|----------|-------|
| **Trigger** | Pull requests to main (paths: `apps/learn-app/**`) |
| **Jobs** | 1 job: `build` |
| **Steps** | 3 steps |
| **Caching** | npm cache (by `package-lock.json`) |
| **Deployment** | None (validation only) |

**Workflow Steps**:
```
1. Checkout repository
2. Setup Node.js 20 (cache: npm)
3. npm install in apps/learn-app/
4. npm run build in apps/learn-app/
```

**Environment Variables Used**: None (no secrets)
**Secrets Used**: None
**Risk Level**: **LOW** (validation only, no production impact)

---

#### Workflow 2: `deploy.yml` (Main deployment)
**File**: `.github/workflows/deploy.yml`

| Property | Value |
|----------|-------|
| **Trigger** | 1. `workflow_run` from `sync-content.yml` (success/skipped) 2. Push to main (paths: `apps/learn-app/**` except `docs/**`) 3. Manual (`workflow_dispatch`) |
| **Jobs** | 2 jobs: `build`, `deploy` |
| **Steps** | 15 steps in build, 1 in deploy |
| **Caching** | npm cache + custom manifest cache |
| **Deployment** | GitHub Pages |

**Build Job Steps**:
```
1. Checkout repository
2. Setup Python 3.12 (if PANAVERSITY_PLUGIN_ENABLED)
3. Install Python deps (click, httpx, pydantic, python-dotenv)
4. Restore manifest cache (.panaversity/manifest.json)
5. Hydrate content from PanaversityFS (if enabled)
6. Fallback to local docs (if hydration fails)
7. Setup Node.js 20 (cache: npm)
8. Install system deps for Sharp/OG image generation
9. npm ci in apps/learn-app/
10. npm run typecheck in apps/learn-app/
11. npm run build in apps/learn-app/ (with env vars)
12. Upload build artifacts to GitHub Pages
13. Build summary
```

**Deploy Job Steps**:
```
1. Deploy to GitHub Pages
```

**Environment Variables Used**:
- `PANAVERSITY_PLUGIN_ENABLED` (variable, boolean)
- `GA4_MEASUREMENT_ID` (secret)
- `PANAVERSITY_SERVER_URL` (secret)
- `AUTH_URL` (secret)
- `OAUTH_CLIENT_ID` (secret)
- `BASE_URL` (secret)
- `PANAVERSITY_API_KEY` (secret)

**Secrets Used**: 6 secrets
**System Dependencies**: fonts-dejavu-core, fonts-liberation, libvips-dev
**Risk Level**: **CRITICAL** (production deployment, multiple env vars, fallback logic)

---

#### Workflow 3: `validate-content.yml` (Content validation)
**File**: `.github/workflows/validate-content.yml`

| Property | Value |
|----------|-------|
| **Trigger** | Pull requests to main (paths: `apps/learn-app/docs/**`) |
| **Jobs** | 1 job: `validate` |
| **Steps** | 3 steps |
| **Caching** | None |
| **Deployment** | None (validation only) |

**Workflow Steps**:
```
1. Checkout repository (fetch-depth: 0)
2. Get changed files in apps/learn-app/docs/
3. Check for local asset references (must use CDN URLs)
   - Rejects: ![...]( /img/...), ![...](/slides/...), src="/img/..."
   - Requires: Full CDN URLs
4. Report results
```

**Environment Variables Used**: None
**Secrets Used**: None
**Risk Level**: **LOW** (validation only, no production impact)

---

#### Workflow 4: `sync-content.yml` (PanaversityFS incremental sync)
**File**: `.github/workflows/sync-content.yml`

| Property | Value |
|----------|-------|
| **Trigger** | 1. Push to main (paths: `apps/learn-app/docs/**`) 2. Manual (`workflow_dispatch`) |
| **Jobs** | 1 job: `sync` |
| **Steps** | 5 steps |
| **Caching** | None (git diff used for change detection) |
| **Deployment** | PanaversityFS (internal API) |
| **Conditional Run** | Only if `PANAVERSITY_PLUGIN_ENABLED == 'true'` |

**Workflow Steps**:
```
1. Checkout repository (fetch-depth: 2 for git diff)
2. Setup Python 3.12
3. Install Python deps (click, httpx, pydantic, python-dotenv)
4. Get changed files (git diff HEAD~1 HEAD):
   - Watches: *.md, *.png, *.jpg, *.svg in apps/learn-app/docs/
5. Sync content via python scripts/ingest-book.py
   - Arg: --book-id "ai-native-dev"
   - Arg: --source-dir ../apps/learn-app/docs
   - Optional: --full-sync flag for full sync
6. Summary report
```

**Environment Variables Used**:
- `PANAVERSITY_MCP_URL` (secret: PANAVERSITY_SERVER_URL)
- `PANAVERSITY_API_KEY` (secret)

**Secrets Used**: 2 secrets
**Concurrency**: Single (prevents race conditions)
**Risk Level**: **MEDIUM** (internal API, but blocks deploy workflow)

---

### 1.2 Deployment Targets & Environments

| Target | Trigger | Environment | Type | Fallback |
|--------|---------|-------------|------|----------|
| **GitHub Pages** | `deploy.yml` completion | Production | Static site | Local docs/ |
| **PanaversityFS** (optional) | `sync-content.yml` | Production (internal API) | Content API | Disabled feature flag |

**GitHub Pages Deployment**:
- Build artifact: `apps/learn-app/build/`
- Environment: `github-pages`
- Concurrency: Single (prevents simultaneous deploys)

**PanaversityFS Deployment**:
- Status: Conditional feature (PANAVERSITY_PLUGIN_ENABLED)
- Fallback: If sync fails, deploy.yml falls back to `apps/learn-app/docs/`
- Incremental: Only uploads changed files (not full sync)

---

### 1.3 Caching Strategy

| Cache | Key | Restored | Hit Rate | Size |
|-------|-----|----------|----------|------|
| **npm** | `package-lock.json` | Yes | ~90% | ~50MB |
| **manifest** | `github.sha` + fallback | Yes | ~70% | ~1MB |
| **System packages** | None | None | N/A | N/A |

**npm Cache**:
- Dependency path: `apps/learn-app/package-lock.json`
- Used in: `pr-check.yml`, `deploy.yml`
- **Issue**: Mixing npm + pnpm (package-lock.json + pnpm-lock.yaml)

**Manifest Cache**:
- Path: `.panaversity/manifest.json`
- Key: `panaversity-manifest-{github.sha}`
- Restore keys: `panaversity-manifest-` (fallback to any version)
- Used in: `deploy.yml` (manifest-cache step)

---

### 1.4 Secrets & Environment Variables Inventory

| Var | Used In | Type | Purpose | Status |
|-----|---------|------|---------|--------|
| `PANAVERSITY_PLUGIN_ENABLED` | deploy.yml, sync-content.yml | Var (boolean) | Feature flag | Conditional |
| `GA4_MEASUREMENT_ID` | deploy.yml | Secret | Analytics | Build |
| `PANAVERSITY_SERVER_URL` | deploy.yml, sync-content.yml | Secret | MCP endpoint | Hydration |
| `AUTH_URL` | deploy.yml | Secret | Auth service | Build |
| `OAUTH_CLIENT_ID` | deploy.yml | Secret | OAuth config | Build |
| `BASE_URL` | deploy.yml | Secret | Site base | Build |
| `PANAVERSITY_API_KEY` | deploy.yml, sync-content.yml | Secret | API auth | Hydration & Sync |

**Total Secrets**: 6
**Total Variables**: 1
**Required for Migration**: All 7 must be preserved in Nx setup

---

## PHASE 2: DEPLOYMENT ANALYSIS

### 2.1 Deployment Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│ Author/Agent pushes apps/learn-app/docs/ to main               │
└────────────┬────────────────────────────────────────────────┘
             │
             ├─────────────────────────────────────────────────┐
             │                                                 │
        (if SYNC ENABLED)                            (always runs)
             │                                                 │
    ┌────────▼────────┐                          ┌────────────▼──────┐
    │ sync-content.yml│                          │   deploy.yml      │
    │ (Incremental)   │                          │  (Main deploy)    │
    │ Uploads changed │                          │  Builds & deploys │
    │ files to        │                          │  to GitHub Pages  │
    │ PanaversityFS   │                          │                   │
    └────────┬────────┘                          └────────┬──────────┘
             │                                          │
    (emits workflow_run event on success)               │
             │                                          │
             └──────────────────┬─────────────────────┘
                                │
                    ┌───────────▼──────────┐
                    │ Completes when both  │
                    │ workflows succeed    │
                    │                      │
                    │ GitHub Pages LIVE    │
                    └──────────────────────┘
```

**Key Behaviors**:
1. **Sync-Content** blocks deploy if enabled (workflow_run dependency)
2. **Deploy** runs even if sync is skipped (PANAVERSITY_PLUGIN_ENABLED=false)
3. **Fallback**: If sync fails, deploy uses local docs/
4. **No staging**: Direct to production on every push to main

---

### 2.2 Failure Scenarios & Recovery

| Scenario | Current Behavior | Recovery |
|----------|------------------|----------|
| Sync fails | Deploy uses fallback (local docs/) | Manual sync retry via workflow_dispatch |
| Deploy fails | GitHub Pages stale | Retry deploy via workflow_dispatch |
| Both fail | GitHub Pages stale + no new content | Manual retry + investigation |
| npm install fails | Build fails | Fix lock file, retry |
| Python deps fail | Sync fails (continue-on-error) | Manual fix, re-push |

---

### 2.3 Production Environment Characteristics

| Aspect | Current |
|--------|---------|
| **Hosting** | GitHub Pages (free, global CDN) |
| **Staging Env** | None (direct to prod) |
| **Approval Gate** | None (auto-deploy on main) |
| **Rollback Procedure** | Manual revert commit, wait for CI |
| **Incident Response** | Modify workflow locally, re-push |
| **Monitoring** | GitHub Actions logs only |

---

## PHASE 3: NX CI DESIGN

### 3.1 Project Detection & Mapping

**Current Projects** (to be registered with Nx):

1. **`book-source`** (Docusaurus website)
   - Type: Node.js / JavaScript
   - Package manager: npm
   - Outputs: `apps/learn-app/build/`
   - Targets needed: `build`, `typecheck`, `lint` (if eslint added)

2. **`panaversity-fs`** (Python MCP server)
   - Type: Python
   - Package manager: uv/pip
   - Outputs: Docker image (production only)
   - Targets needed: Custom executor for `test`, `lint`

**Remaining Projects** (context, docs, specs, papers, history):
- These are **read-only** (docs, specs, context, papers, history)
- Don't need Nx targets (no build, test, deploy)
- But should be **registered** in Nx for graph awareness

---

### 3.2 Nx Configuration Files

#### `nx.json` (Workspace-level config)

```json
{
  "extends": "nx/presets/npm.json",
  "npmScope": "panaversity",
  "targetDefaults": {
    "build": {
      "dependsOn": ["^build"],
      "cache": true,
      "outputs": ["{projectRoot}/build"]
    },
    "test": {
      "cache": true,
      "outputs": ["{projectRoot}/coverage"]
    },
    "typecheck": {
      "cache": true
    }
  },
  "plugins": [
    "@nx/js/plugin",
    "@nx/docusaurus/plugin"
  ]
}
```

#### `pnpm-workspace.yaml` (Workspace package manager)

```yaml
packages:
  - 'book-source'
  - 'panaversity-fs'
  - 'docs'
  - 'context'
  - 'specs'
  - 'papers'
  - 'history'
```

#### `apps/learn-app/project.json` (Project-level config)

```json
{
  "name": "book-source",
  "sourceRoot": "book-source",
  "projectType": "application",
  "targets": {
    "build": {
      "executor": "@nx/js:tsc",
      "options": {
        "outputPath": "apps/learn-app/build",
        "main": "apps/learn-app/src/index.ts"
      },
      "configurations": {
        "production": {}
      }
    },
    "typecheck": {
      "executor": "@nx/js:tsc",
      "options": {
        "noEmit": true
      }
    },
    "serve": {
      "executor": "@nx/docusaurus:serve",
      "options": {
        "docusaurusConfig": "apps/learn-app/docusaurus.config.js"
      }
    }
  }
}
```

#### `panaversity-fs/project.json` (Python project)

```json
{
  "name": "panaversity-fs",
  "sourceRoot": "panaversity-fs",
  "projectType": "library",
  "targets": {
    "test": {
      "executor": "nx:run-commands",
      "options": {
        "command": "cd panaversity-fs && pytest"
      }
    },
    "lint": {
      "executor": "nx:run-commands",
      "options": {
        "command": "cd panaversity-fs && pylint src/"
      }
    },
    "docker": {
      "executor": "nx:run-commands",
      "options": {
        "command": "cd panaversity-fs && docker build -f Dockerfile.prod -t panaversity-fs:latest ."
      }
    }
  }
}
```

---

### 3.3 New Nx-Based Workflows

#### New Workflow 1: `ci.yml` (PR validation + affected tests)

```yaml
name: CI - Nx Affected

on:
  pull_request:
    branches:
      - main
  push:
    branches:
      - main

concurrency:
  group: ci-${{ github.ref }}
  cancel-in-progress: true

jobs:
  affected:
    name: Detect Affected Projects
    runs-on: ubuntu-latest
    outputs:
      projects: ${{ steps.affected.outputs.projects }}
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - name: Setup pnpm
        uses: pnpm/action-setup@v2
        with:
          version: 8

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: "20"
          cache: pnpm

      - name: Install dependencies
        run: pnpm install --frozen-lockfile

      - name: Detect affected projects
        id: affected
        run: |
          PROJECTS=$(npx nx show projects --affected --plain)
          echo "projects=$PROJECTS" >> $GITHUB_OUTPUT

  lint:
    name: Lint Affected
    needs: affected
    if: needs.affected.outputs.projects != ''
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - name: Setup pnpm
        uses: pnpm/action-setup@v2

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: "20"
          cache: pnpm

      - name: Install dependencies
        run: pnpm install --frozen-lockfile

      - name: Lint affected
        run: npx nx affected -t lint

  test:
    name: Test Affected
    needs: affected
    if: needs.affected.outputs.projects != ''
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - name: Setup pnpm
        uses: pnpm/action-setup@v2

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: "20"
          cache: pnpm

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.12"

      - name: Install dependencies
        run: pnpm install --frozen-lockfile

      - name: Test affected
        run: npx nx affected -t test

  build:
    name: Build Affected
    needs: affected
    if: needs.affected.outputs.projects != ''
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - name: Setup pnpm
        uses: pnpm/action-setup@v2

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: "20"
          cache: pnpm

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.12"

      - name: Install system dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y \
            fonts-dejavu-core \
            fonts-liberation \
            libvips-dev

      - name: Install dependencies
        run: pnpm install --frozen-lockfile

      - name: Build affected
        run: npx nx affected -t build
```

#### New Workflow 2: `deploy.yml` (Nx-based deployment)

```yaml
name: Deploy - GitHub Pages & Production

on:
  push:
    branches:
      - main
    paths:
      - "apps/learn-app/**"
  workflow_dispatch:
    inputs:
      full_rebuild:
        description: "Force full content download (ignore manifest cache)"
        type: boolean
        default: false

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: pages
  cancel-in-progress: false

jobs:
  # Check which projects need deployment
  check:
    name: Check Affected Projects
    runs-on: ubuntu-latest
    outputs:
      book-affected: ${{ steps.check.outputs.book-affected }}
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - name: Setup pnpm
        uses: pnpm/action-setup@v2

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: "20"
          cache: pnpm

      - name: Check if book-source affected
        id: check
        run: |
          pnpm install --frozen-lockfile
          if npx nx show projects --affected --plain | grep -q book-source; then
            echo "book-affected=true" >> $GITHUB_OUTPUT
          else
            echo "book-affected=false" >> $GITHUB_OUTPUT
          fi

  # Sync content to PanaversityFS if enabled
  sync:
    name: Sync Content
    needs: check
    if: |
      needs.check.outputs.book-affected == 'true' &&
      vars.PANAVERSITY_PLUGIN_ENABLED == 'true'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 2

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.12"

      - name: Install Python dependencies
        working-directory: panaversity-fs
        run: |
          pip install click httpx pydantic python-dotenv

      - name: Get changed files
        id: changed
        if: github.event.inputs.full_rebuild != 'true'
        run: |
          CHANGED=$(git diff --name-only HEAD~1 HEAD -- \
            'apps/learn-app/docs/**/*.md' \
            'apps/learn-app/docs/**/*.png' \
            'apps/learn-app/docs/**/*.jpg' \
            'apps/learn-app/docs/**/*.svg' | tr '\n' ' ')
          echo "files=$CHANGED" >> $GITHUB_OUTPUT
          echo "count=$(echo "$CHANGED" | wc -w | tr -d ' ')" >> $GITHUB_OUTPUT

      - name: Sync content to PanaversityFS
        env:
          PANAVERSITY_MCP_URL: ${{ secrets.PANAVERSITY_SERVER_URL }}
          PANAVERSITY_API_KEY: ${{ secrets.PANAVERSITY_API_KEY }}
        run: |
          cd panaversity-fs
          if [ "${{ github.event.inputs.full_rebuild }}" = "true" ]; then
            python scripts/ingest-book.py \
              --book-id "ai-native-dev" \
              --source-dir "../apps/learn-app/docs" \
              --verbose
          elif [ "${{ steps.changed.outputs.count }}" -gt 0 ]; then
            python scripts/ingest-book.py \
              --book-id "ai-native-dev" \
              --source-dir "../apps/learn-app/docs" \
              --verbose
          else
            echo "No content changes detected, skipping sync"
          fi

  # Build and deploy site
  build:
    name: Build & Deploy to GitHub Pages
    needs: [check, sync]
    if: always() && needs.check.outputs.book-affected == 'true'
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - uses: actions/checkout@v4

      - name: Setup pnpm
        uses: pnpm/action-setup@v2

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: "20"
          cache: pnpm

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.12"

      - name: Install system dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y \
            fonts-dejavu-core \
            fonts-liberation \
            libvips-dev

      - name: Restore manifest cache
        if: vars.PANAVERSITY_PLUGIN_ENABLED == 'true'
        id: manifest-cache
        uses: actions/cache@v4
        with:
          path: .panaversity/manifest.json
          key: panaversity-manifest-${{ github.sha }}
          restore-keys: |
            panaversity-manifest-

      - name: Hydrate content from PanaversityFS
        if: vars.PANAVERSITY_PLUGIN_ENABLED == 'true'
        continue-on-error: true
        env:
          PANAVERSITY_MCP_URL: ${{ secrets.PANAVERSITY_SERVER_URL }}
          PANAVERSITY_API_KEY: ${{ secrets.PANAVERSITY_API_KEY }}
        run: |
          cd panaversity-fs
          python scripts/hydrate-book.py \
            --book-id "ai-native-dev" \
            --output-dir "../build-source" \
            --manifest-file "../.panaversity/manifest.json" \
            --verbose

      - name: Fallback to local docs
        if: vars.PANAVERSITY_PLUGIN_ENABLED == 'true' && failure()
        run: |
          if [ ! -d "apps/learn-app/docs" ]; then
            echo "ERROR: No fallback content available"
            exit 1
          fi
          mkdir -p build-source
          cp -r apps/learn-app/docs/* build-source/

      - name: Install dependencies
        run: pnpm install --frozen-lockfile

      - name: Build site
        env:
          GA4_MEASUREMENT_ID: ${{ secrets.GA4_MEASUREMENT_ID }}
          PANAVERSITY_PLUGIN_ENABLED: ${{ vars.PANAVERSITY_PLUGIN_ENABLED || 'false' }}
          PANAVERSITY_SERVER_URL: ${{ secrets.PANAVERSITY_SERVER_URL }}
          AUTH_URL: ${{ secrets.AUTH_URL }}
          OAUTH_CLIENT_ID: ${{ secrets.OAUTH_CLIENT_ID }}
          BASE_URL: ${{ secrets.BASE_URL }}
          PANAVERSITY_API_KEY: ${{ secrets.PANAVERSITY_API_KEY }}
        run: pnpm nx build book-source

      - name: Upload build artifacts
        uses: actions/upload-pages-artifact@v3
        with:
          path: apps/learn-app/build

      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

#### New Workflow 3: `validate.yml` (Content validation with Nx)

```yaml
name: Validate - Content & Assets

on:
  pull_request:
    paths:
      - "apps/learn-app/docs/**"

jobs:
  content:
    name: Validate Content Assets
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - name: Get changed files
        id: changed
        run: |
          CHANGED=$(git diff --name-only origin/${{ github.base_ref }}...HEAD -- \
            'apps/learn-app/docs/**/*.md' 2>/dev/null || echo "")
          echo "files<<EOF" >> $GITHUB_OUTPUT
          echo "$CHANGED" >> $GITHUB_OUTPUT
          echo "EOF" >> $GITHUB_OUTPUT

      - name: Check for local asset references
        if: steps.changed.outputs.files != ''
        run: |
          echo "Checking for local asset references..."
          ERRORS=0
          if echo "${{ steps.changed.outputs.files }}" | \
             xargs grep -Hn '!\[.*\](/img/' 2>/dev/null; then
            ERRORS=$((ERRORS + 1))
          fi
          if echo "${{ steps.changed.outputs.files }}" | \
             xargs grep -Hn '!\[.*\](/slides/' 2>/dev/null; then
            ERRORS=$((ERRORS + 1))
          fi
          if [ $ERRORS -gt 0 ]; then
            exit 1
          fi
```

---

### 3.4 Nx Features Enabled

| Feature | Status | Config |
|---------|--------|--------|
| **Project Graph** | Enabled | Auto-discovered from project.json |
| **Affected Commands** | Enabled | `nx affected -t target` |
| **Cache** | Enabled (local) | Configured in `targetDefaults` |
| **Parallel Execution** | Enabled | Default in `nx run-many` |
| **Custom Executors** | Enabled (Python) | Using `nx:run-commands` |

---

## PHASE 4: MIGRATION SEQUENCE

### 4.1 Safe Migration Order (Low to High Risk)

#### Week 1: Foundation (SAFE - No CI Impact)
1. **Day 1-2**: Initialize Nx workspace
   - `npx create-nx-workspace@latest` (select npm preset)
   - Create `nx.json` with proper configuration
   - Create `pnpm-workspace.yaml`

2. **Day 3-4**: Standardize package manager
   - Remove `package-lock.json` from root
   - Ensure pnpm is used everywhere
   - Run `pnpm install` (creates/updates `pnpm-lock.yaml`)

3. **Day 5**: Create project configurations
   - Add `project.json` to `apps/learn-app/`
   - Add `project.json` to `panaversity-fs/`
   - Register read-only projects (docs, specs, etc.)
   - Run `nx show projects` to verify

---

#### Week 2: Configuration (SAFE - No CI Impact)
1. **Day 1-2**: Configure build targets
   - `book-source`: bind to Docusaurus build script
   - `panaversity-fs`: bind to Python build/test
   - Test locally: `nx build book-source`

2. **Day 3-4**: Configure test targets
   - `panaversity-fs`: `nx test panaversity-fs`
   - `book-source`: No tests currently, skip

3. **Day 5**: Configure other targets
   - typecheck, lint, serve for book-source
   - Test affected on feature branch: `nx affected -t test`

---

#### Week 3: CI Migration (HIGH RISK - Production Impact)
1. **Day 1-2**: Create new workflows (non-breaking)
   - Add `ci.yml` alongside existing PR checks
   - Test on feature branch (should pass)
   - Don't delete old workflows yet

2. **Day 3**: Manual approval gate
   - Add `needs: [ci]` to deploy job
   - Require manual approval before deploy
   - Test workflow_dispatch

3. **Day 4-5**: Gradual cutover
   - Monitor deploy.yml succeeds with Nx
   - Delete old pr-check.yml (kept ci.yml)
   - Monitor for 1 week (no issues)

---

#### Week 4: Optimization (SAFE - Performance)
1. **Day 1-3**: Enable Nx Cloud
   - Generate Nx Cloud token
   - Add to GitHub secrets
   - Test distributed caching

2. **Day 4-5**: Performance analysis
   - Measure build times before/after
   - Tune cache settings
   - Document findings

---

### 4.2 Detailed Migration Tasks

| Task | Owner | Estimate | Risk | Blocking |
|------|-------|----------|------|----------|
| Initialize Nx workspace | Engineer | 30m | LOW | No |
| Create pnpm-workspace.yaml | Engineer | 30m | LOW | No |
| Add project.json files | Engineer | 1h | LOW | No |
| Update CI workflow | Engineer | 2h | MEDIUM | Yes (deploy) |
| Test Nx affected locally | Engineer | 1h | LOW | Yes (CI update) |
| Manual approval gate in deploy.yml | Engineer | 1h | MEDIUM | Yes (deploy) |
| Switch production to Nx CI | Engineer + Lead | 30m | HIGH | Yes (manual) |
| Monitor for 1 week | Engineer + Lead | 5h (async) | MEDIUM | No |
| Enable Nx Cloud | Engineer | 1h | LOW | No |

---

### 4.3 Rollback Procedure

If migration breaks production at **any point**, rollback is immediate:

```bash
# 1. Revert commits (git history)
git revert HEAD

# 2. OR manually restore old workflows
cp .github/workflows/deploy.yml.backup .github/workflows/deploy.yml
git add .github/workflows/deploy.yml
git commit -m "revert: restore old deploy workflow"
git push

# 3. GitHub Pages will rebuild with old workflow
# Expected: Deploy runs within 2-3 minutes
```

**Backup Strategy**:
- Keep old workflows in `.github/workflows-backup/` before migration
- Tag git commit before Phase 3: `git tag migration-phase3-checkpoint`
- Document rollback steps in team wiki

---

### 4.4 Feature Branch Testing Strategy

Before production cutover, test on feature branch:

```bash
# 1. Create feature branch
git checkout -b feat/nx-migration

# 2. Create PR (do NOT merge yet)
# CI runs with new nx.json + new workflows
# Monitor: Both old AND new workflows run
# Success: All tests pass

# 3. If successful, approve and merge to main
# 4. Monitor production for 24 hours
```

---

## PHASE 5: SECRETS & ENVIRONMENT CHECKLIST

### 5.1 Secrets Inventory

| Secret | Current | Nx Changes | Action |
|--------|---------|-----------|--------|
| `GA4_MEASUREMENT_ID` | GitHub Secret | No change | Preserve |
| `PANAVERSITY_SERVER_URL` | GitHub Secret | No change | Preserve |
| `AUTH_URL` | GitHub Secret | No change | Preserve |
| `OAUTH_CLIENT_ID` | GitHub Secret | No change | Preserve |
| `BASE_URL` | GitHub Secret | No change | Preserve |
| `PANAVERSITY_API_KEY` | GitHub Secret | No change | Preserve |
| `NX_CLOUD_ACCESS_TOKEN` | N/A | New (optional) | Create if using Nx Cloud |

### 5.2 Environment Variables

| Variable | Current | Nx Changes | Action |
|----------|---------|-----------|--------|
| `PANAVERSITY_PLUGIN_ENABLED` | GitHub Variable | No change | Preserve |

### 5.3 Migration Checklist

- [ ] All 6 production secrets documented
- [ ] Secrets have correct access levels (write for deploy workflows)
- [ ] Feature branch testing confirms secrets work with Nx
- [ ] Nx Cloud token generated (optional, if using distributed cache)
- [ ] GitHub Secrets page reviewed (no stale secrets)

---

## DELIVERABLES SUMMARY

### Files Created During Migration

1. **`nx.json`** - Workspace configuration
2. **`pnpm-workspace.yaml`** - Workspace package manager
3. **`.github/workflows/ci.yml`** - New Nx-based CI (linting, testing, building)
4. **`.github/workflows/deploy.yml`** (updated) - Nx-based deployment
5. **`.github/workflows/validate.yml`** - Content validation (kept from original)
6. **`apps/learn-app/project.json`** - Book project configuration
7. **`panaversity-fs/project.json`** - Python project configuration
8. **`docs/project.json`** - Read-only project
9. **`specs/project.json`** - Read-only project
10. **`.github/workflows-backup/`** - Old workflows (for rollback)

### Migration Report (This Document)
- **Phase 1**: Current state fully documented (4 workflows, 7 secrets, 2 deployment targets)
- **Phase 2**: Deployment analysis (failures, recovery, no staging environment)
- **Phase 3**: Nx design (3 new workflows, project configurations)
- **Phase 4**: Migration sequence (4 weeks, clear risk levels)
- **Phase 5**: Secrets & environment checklist

---

## RISK MATRIX

| Phase | Risk | Impact | Mitigation |
|-------|------|--------|-----------|
| **Phase 1** | Package manager confusion | Build failures | Use pnpm everywhere, remove npm |
| **Phase 2** | Project config errors | Nx graph broken | Local testing before commit |
| **Phase 3** | CI breaks production | Stale GitHub Pages | Feature branch testing + manual approval |
| **Phase 4** | Cache misconfiguration | Slower builds | Gradual tuning, monitor metrics |

---

## NEXT STEPS

### Immediate (Before Starting Migration)
1. **Review** this plan with team
2. **Approve** Phase 3 (CI/CD) timeline (production impact)
3. **Document** all team members who have access to GitHub Secrets
4. **Backup** old workflows to `.github/workflows-backup/`
5. **Create** feature branch: `feat/nx-migration`

### Phase 1 Kickoff
```bash
# Day 1: Initialize Nx
npx create-nx-workspace@latest --preset=npm --packageManager=pnpm

# Day 3: Standardize pnpm
rm package-lock.json
pnpm install
git add pnpm-lock.yaml
git commit -m "chore: standardize on pnpm"

# Day 5: Register projects
# Create nx.json, pnpm-workspace.yaml, all project.json files
nx show projects  # Verify all 7 projects listed
```

### Week 2: Configuration Testing
```bash
# Test build locally
nx build book-source

# Test affected detection
nx show projects --affected
```

### Week 3: Production Migration (Requires Approval)
```bash
# Create PR with new workflows
# Have team review alongside old workflows
# Manual approval before deploying to main
```

---

**Status**: Ready for Phase 1 start
**Last Updated**: 2025-12-15
**Document Version**: 1.0

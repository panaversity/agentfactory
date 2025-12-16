# Implementation Plan: Nx Monorepo Migration

**Branch**: `034-nx-monorepo-migration` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/034-nx-monorepo-migration/spec.md`

## Summary

Migrate the repository from a multi-project structure to an Nx-orchestrated monorepo containing 2 apps (Docusaurus website, Python MCP server) and 6 Docusaurus plugin libraries. The migration leverages Nx's official MCP server for AI-first development, implements affected-only CI/CD builds, and uses the built-in `nx:run-commands` executor with Makefile delegation to maintain existing Python tooling. The approach prioritizes zero disruption to book writers through symlink strategy and backward-compatible path references, while achieving 50% CI time reduction through intelligent caching and affected detection.

**Technical Approach**: Polyglot monorepo using Nx 20+ with pnpm workspaces, `nx:run-commands` executor delegating to existing Makefile targets (no custom executor needed), 4-phase migration with rollback checkpoints, and parallel CI validation before production cutover.

## Technical Context

**Language/Version**: TypeScript 5.6.2 (Docusaurus), Python 3.13 (MCP server), Node.js 20+
**Primary Dependencies**:

- Nx 20.0+ with @nx/js, @nx/node, @nx/workspace
- Docusaurus 3.9.2 with 6 custom plugins
- pnpm 9.12+ (workspace package manager)
- Python: uv (package manager), FastAPI, pytest, ruff, mypy
  **Storage**:
- Neon Postgres (production data)
- File system (book content during development)
- GitHub Pages (static site deployment)
  **Testing**:
- TypeScript: Jest (if added), TypeScript compiler (typecheck target)
- Python: pytest with 301 tests (unit, integration, property, performance, e2e)
- CI: Nx affected detection for test execution
  **Target Platform**:
- Website: GitHub Pages (Cloudflare CDN)
- Backend: Docker containers (Cloud Run deployment)
- CI: GitHub Actions (Ubuntu runners)
  **Project Type**: Web monorepo with polyglot support (Node.js + Python)
  **Performance Goals**:
- New developer setup: < 15 minutes
- CI affected builds: 50% reduction (full build baseline: ~15 min)
- Cache hit rate: > 80% for unchanged projects
  **Constraints**:
- Preserve all Git history (no force pushes)
- Zero book writer workflow disruption
- Maximum 2 hours planned downtime for cutover
- Python tooling (uv, pyproject.toml) must remain unchanged
- Docusaurus build output compatible with GitHub Pages
  **Scale/Scope**:
- 8 Nx projects (2 apps, 6 libs)
- 61+ path references in `.claude/` configuration
- 4 GitHub Actions workflows to migrate
- 301 Python tests + TypeScript build validation

## Constitution Check

_GATE: Must pass before Phase 0 research. Re-check after Phase 1 design._

**Gate 1: Specification Primacy**

- ✅ Spec defines intent before implementation
- ✅ 5 user stories with testable acceptance criteria
- ✅ 17 functional requirements (FR-001 to FR-017)
- ✅ 8 measurable success criteria

**Gate 2: Progressive Complexity**

- ✅ Phased approach (0-4) with clear checkpoints
- ✅ Each phase has go/no-go decision gate
- ✅ Rollback procedure documented for Phase 3

**Gate 3: Factual Accuracy**

- ✅ ADR-0020 provides decision rationale (Nx vs Bazel)
- ✅ Analysis documents cite Nx official docs, MCP specification
- ✅ Python integration verified via existing Makefile targets
- ✅ 14 gaps identified in MIGRATION-GAPS-ANALYSIS.md (to be addressed)
- ✅ **Nx patterns validated via `nx-mcp` server** (official `nx_docs` tool queries):
  - `nx:run-commands` executor with `cwd` option (no custom executor needed)
  - `"cache": true` for caching targets
  - `--base=origin/main --head=HEAD` for affected detection in CI
  - `fetch-depth: 0` required for Git history in GitHub Actions

**Gate 4: Coherent Structure**

- ✅ 5 analysis documents provide comprehensive context
- ✅ Migration follows dependency order (config → structure → CI → validation)
- ✅ Path references cataloged (61+ locations)

**Gate 5: Intelligence Accumulation**

- ✅ Learnings from Bazel evaluation inform Nx choice
- ✅ CI/CD patterns reusable across future projects
- ✅ Custom executor pattern applicable to other polyglot needs

**Gate 6: Anti-Convergence**

- ✅ Migration preserves existing workflows (not forcing new patterns)
- ✅ Symlink strategy maintains backward compatibility
- ✅ Developer choice: Use `nx` commands OR original `make` targets

**Gate 7: Minimal Content**

- ✅ Plan focused on technical execution, not tutorial content
- ✅ Each phase maps to specific success criteria
- ✅ No speculative features beyond spec requirements

**Violations**: None requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/034-nx-monorepo-migration/
├── spec.md              # Feature specification (input)
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0 output (technology decisions)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Analysis Documents (completed)

```text
docs/migration-monorepo-plan/
├── NX-WORKSPACE-DESIGN.md          # Target structure, config files, project templates
├── CI-CD-MIGRATION-PLAN.md         # Workflow migration, secrets, deployment flow
├── PYTHON-NX-INTEGRATION-SPEC.md   # Custom executor, Makefile wrapper strategy
├── BOOK-WRITER-WORKFLOW-IMPACT.md  # 61+ path references, symlink strategy
└── MIGRATION-GAPS-ANALYSIS.md      # 14 gaps, 16 new docs needed (P0-P2 prioritized)
```

### Source Code (repository root)

**Target Structure** (post-migration):

```text
storage/
├── nx.json                      # Workspace configuration
├── pnpm-workspace.yaml          # Workspace packages definition
├── .nxignore                    # Paths excluded from Nx
├── package.json                 # Root dependencies (Nx, TypeScript, pnpm)
│
├── apps/                        # Deployable applications
│   ├── learn-app/               # Docusaurus "breathing book" (moved from book-source/)
│   │   ├── project.json         # Nx targets: build, serve, hydrate, deploy
│   │   ├── package.json
│   │   ├── docs/
│   │   ├── src/
│   │   ├── static/
│   │   └── docusaurus.config.ts
│   │
│   └── panaversity-fs-py/       # Python MCP server (moved from panaversity-fs/)
│       ├── project.json         # Nx targets via Makefile wrapper
│       ├── pyproject.toml       # Python dependencies (unchanged)
│       ├── uv.lock              # Locked Python deps
│       ├── Makefile             # Build orchestration (unchanged)
│       ├── src/
│       └── tests/
│
├── libs/                        # Shared libraries (namespaced by domain)
│   ├── docusaurus/              # Docusaurus plugins
│   │   ├── remark-interactive-python/
│   │   │   ├── project.json
│   │   │   ├── package.json
│   │   │   └── index.js
│   │   ├── remark-content-enhancements/
│   │   ├── plugin-og-image/
│   │   ├── plugin-structured-data/
│   │   ├── summaries-plugin/
│   │   └── panaversityfs-plugin/
│   ├── auth/                    # Future: SSO, session libs
│   └── shared/                  # Cross-app utilities
│
├── packages/                    # Future: publishable packages (npm/PyPI)
│   └── (empty - add when needed)
│
├── tools/                       # Build tools & scripts
│   └── scripts/
│       ├── hydrate-book.py      # Moved from panaversity-fs/scripts/
│       ├── ingest-book.py
│       └── project.json
│
├── .claude/                     # Claude Code configuration (path updates)
├── .github/workflows/           # CI/CD (update for Nx affected)
├── specs/                       # Feature specifications
├── docs/                        # Architecture docs
├── history/                     # PHRs, ADRs
└── [root files: README.md, CLAUDE.md, etc.]
```

**Structure Decision**: Option 2 variant (Web application with polyglot backend).

This structure:

1. **Separates deployable apps** (`learn-app`, `panaversity-fs-py`) from reusable libs (6 plugins)
2. **Maintains Python isolation**: `panaversity-fs-py/` keeps existing `pyproject.toml`, `uv.lock`, `Makefile`
3. **Extracts shared plugins**: Previously nested in `book-source/plugins/` → now first-class libs
4. **Uses built-in Nx executor**: `nx:run-commands` with `cwd` option bridges Nx and Python builds (no custom executor)
5. **Preserves git history**: Using `git mv` commands (no filter-branch)

**Migration from current**:

```text
Current → Target
book-source/ → apps/learn-app/
panaversity-fs/ → apps/panaversity-fs-py/
book-source/plugins/remark-interactive-python → libs/docusaurus/remark-interactive-python/
[+ 5 more plugins extracted]
panaversity-fs/scripts/ → tools/scripts/
```

## Complexity Tracking

No constitutional violations requiring justification. The migration introduces complexity (8 projects, custom executor) but this is necessary to achieve the spec's requirements (affected detection, AI integration, polyglot support) and is justified by analysis documents.

## Component Architecture

### Component 1: Nx Workspace Configuration

**Purpose**: Define workspace-level settings, caching strategy, and project discovery.

**Key Files**:

- `nx.json` - Workspace configuration, named inputs (npmInputs, pythonInputs, docusaurusInputs), target defaults
- `pnpm-workspace.yaml` - Package manager workspace definition (apps/_, libs/\*\*/_, packages/_, tools/_)
- `.nxignore` - Exclude non-project paths (docs/, specs/, history/, .claude/)

**Dependencies**: None (foundation component)

**Inputs**: Workspace metadata (project directories, package.json files)

**Outputs**: Nx project graph, cache configuration

**Interface**: CLI commands (`nx show projects`, `nx graph`, `nx affected`)

**Test Strategy**:

- Unit: N/A (configuration)
- Integration: Verify `nx show projects` lists 8 projects
- E2E: Run `nx graph` and validate all relationships

### Component 2: Website App (Docusaurus)

**Purpose**: Build and serve the AI-Native Software Development book website.

**Key Files**:

- `apps/learn-app/project.json` - Nx targets (build, serve, hydrate, deploy, typecheck, lint)
- `apps/learn-app/package.json` - Dependencies (Docusaurus 3.9.2, 6 plugins)
- `apps/learn-app/docusaurus.config.ts` - Docusaurus configuration

**Dependencies**:

- 6 plugin libs (`remark-interactive-python`, etc.)
- `tools/scripts/hydrate-book.py` (hydrate target)

**Inputs**:

- `apps/learn-app/docs/**/*.md` (book content)
- `apps/learn-app/static/**/*` (images, slides)
- `apps/learn-app/src/**/*` (React components)

**Outputs**: `dist/apps/learn-app/` (static HTML site)

**Interface**:

- CLI: `nx serve learn-app`, `nx build learn-app`
- Deploy: GitHub Pages via GitHub Actions

**Test Strategy**:

- Unit: N/A (static site generator)
- Integration: Build completes without errors, output directory contains HTML
- E2E: Deploy to test environment, verify pages load

### Component 3: PanaversityFS Python App

**Purpose**: MCP server for agent-native multi-book storage.

**Key Files**:

- `apps/panaversity-fs-py/project.json` - Nx targets using `nx:run-commands` executor
- `apps/panaversity-fs-py/Makefile` - Build orchestration (test, lint, format, build)
- `apps/panaversity-fs-py/pyproject.toml` - Python dependencies

**Dependencies**: None (isolated Python project)

**Inputs**:

- `apps/panaversity-fs-py/src/**/*.py` (source code)
- `apps/panaversity-fs-py/pyproject.toml` (dependencies)

**Outputs**: Docker image `panaversity-fs:latest`

**Interface**:

- CLI: `nx test panaversity-fs-py`, `nx run panaversity-fs-py:serve`
- Makefile (direct): `cd apps/panaversity-fs-py && make test`

**Nx Configuration** (per official docs):

```json
{
  "targets": {
    "test": {
      "executor": "nx:run-commands",
      "options": { "cwd": "apps/panaversity-fs-py", "command": "make test" },
      "cache": true
    }
  }
}
```

**Test Strategy**:

- Unit: pytest (301 tests, 7 categories)
- Integration: End-to-end book workflow
- E2E: Docker container runs, MCP endpoints respond

### Component 4: Plugin Libraries (6 libs)

**Purpose**: Docusaurus plugins for Python interactivity, OG images, structured data, summaries, content enhancements, FS integration.

**Key Files** (each lib):

- `libs/{plugin-name}/project.json` - Nx targets (build, test, lint)
- `libs/{plugin-name}/package.json` - Plugin dependencies
- `libs/{plugin-name}/index.js` - Plugin entry point

**Dependencies**: Docusaurus peer dependencies (provided by learn-app)

**Inputs**: Plugin source code

**Outputs**: `dist/libs/{plugin-name}/` (built plugin)

**Interface**: Imported by `apps/learn-app/docusaurus.config.ts`

**Test Strategy**:

- Unit: Jest tests (if added)
- Integration: Website build includes plugin functionality
- E2E: Verify plugin features work on deployed site

### Component 5: Python Build Integration (nx:run-commands)

**Purpose**: Bridge Nx and Python builds via built-in `nx:run-commands` executor with Makefile delegation.

**Key Files**:

- `apps/panaversity-fs-py/project.json` - Nx targets using run-commands executor
- `apps/panaversity-fs-py/Makefile` - Existing build orchestration (unchanged)

**Dependencies**:

- Nx built-in `nx:run-commands` executor (no custom executor needed)
- Python Makefile targets

**Configuration** (per Nx official docs):

```json
{
  "targets": {
    "test": {
      "executor": "nx:run-commands",
      "options": {
        "cwd": "apps/panaversity-fs-py",
        "command": "make test"
      },
      "cache": true,
      "inputs": [
        "{projectRoot}/**/*.py",
        "{projectRoot}/pyproject.toml",
        "{projectRoot}/uv.lock"
      ],
      "outputs": []
    }
  }
}
```

**Outputs**: Exit code (0 = success)

**Interface**:

- Invoked by Nx: `nx test panaversity-fs-py`
- Direct fallback: `cd apps/panaversity-fs-py && make test`

**Test Strategy**:

- Unit: N/A (uses built-in executor)
- Integration: Run `nx test panaversity-fs-py`, verify pytest runs
- E2E: Full Python CI workflow via Nx

### Component 6: CI/CD Workflows

**Purpose**: Automated testing, building, and deployment using Nx affected detection.

**Key Files**:

- `.github/workflows/ci.yml` - PR validation (affected tests, affected builds)
- `.github/workflows/deploy.yml` - Production deployment (GitHub Pages)
- `.github/workflows/validate.yml` - Content validation (asset references)

**Dependencies**:

- Nx CLI (npx nx)
- GitHub Secrets (7 variables: GA4_MEASUREMENT_ID, PANAVERSITY_SERVER_URL, etc.)

**Inputs**: Git diff (base branch vs HEAD)

**Outputs**:

- Test results, build artifacts
- Deployed GitHub Pages site

**Interface**:

- Triggered by: PR creation, push to main
- Uses: `nx affected -t test`, `nx build learn-app`

**Test Strategy**:

- Unit: N/A (workflow YAML)
- Integration: Test workflow on feature branch
- E2E: Parallel old + new workflows, compare results

### Component 7: Book Writer Path References

**Purpose**: Maintain backward compatibility for Claude Code skills, commands, agents referencing book content paths.

**Key Files**:

- `.claude/**/*.md` (61+ references to `book-source/`)
- CLAUDE.md (context-gathering protocol, lines 54, 60)
- `lesson-template.md`, `file-organization.md` (output styles)

**Dependencies**: Symlink strategy (`book-source → apps/learn-app`)

**Inputs**: File paths in skill/command documentation

**Outputs**: N/A (configuration)

**Interface**: Claude Code tools (Read, Glob, Edit)

**Test Strategy**:

- Unit: N/A (documentation)
- Integration: Verify `Read apps/learn-app/docs/chapter-index.md` resolves correctly
- E2E: Book writer creates lesson via skill, verifies file appears in correct location

## Implementation Sequence

### Phase 0: Preparation (Pre-Migration Setup)

**Goal**: Create Nx configuration without moving any files. Validate Nx recognizes all projects.

**Duration**: 2-3 days

**Steps**:

1. **Create root configuration files** (Day 1):

   - Write `nx.json` with workspace settings, named inputs, target defaults (per official docs):
     ```json
     {
       "targetDefaults": {
         "build": {
           "cache": true,
           "inputs": ["{projectRoot}/**/*", "!{projectRoot}/**/*.md"],
           "outputs": ["{workspaceRoot}/dist/{projectName}"]
         },
         "test": {
           "cache": true
         }
       }
     }
     ```
   - Write `pnpm-workspace.yaml` defining `apps/*`, `libs/**/*`, `tools/*`
   - Write `.nxignore` excluding non-project paths
   - Update root `package.json` with Nx dependencies, pnpm scripts

2. **Create project.json files** (Day 1-2):

   - `apps/learn-app/project.json` (targets: build, serve, hydrate, deploy, typecheck, lint)
   - `apps/panaversity-fs-py/project.json` using `nx:run-commands` executor (per official Nx docs):
     ```json
     {
       "name": "panaversity-fs-py",
       "targets": {
         "test": {
           "executor": "nx:run-commands",
           "options": {
             "cwd": "apps/panaversity-fs-py",
             "command": "make test"
           },
           "cache": true,
           "inputs": ["{projectRoot}/**/*.py", "{projectRoot}/pyproject.toml"]
         },
         "lint": {
           "executor": "nx:run-commands",
           "options": {
             "cwd": "apps/panaversity-fs-py",
             "command": "make lint"
           }
         },
         "serve": {
           "executor": "nx:run-commands",
           "options": {
             "cwd": "apps/panaversity-fs-py",
             "command": "make serve"
           }
         }
       }
     }
     ```
   - 6x `libs/docusaurus/{plugin}/project.json` (targets: build, lint)
   - `tools/scripts/project.json`

3. **Validation** (Day 2):
   - Run `nx show projects` → should list 8 projects
   - Run `nx graph` → should show dependency relationships
   - Run `nx reset && pnpm install` → no errors
   - Test individual targets (dry-run, no file moves yet):
     - `nx build learn-app` → fails (paths wrong, expected)
     - `nx test panaversity-fs-py` → fails (paths wrong, expected)

**Success Criteria**:

- Nx recognizes all 8 projects
- Dependency graph shows learn-app depends on 6 plugins
- No pnpm installation errors
- Ready for Phase 1 (structure migration)

**Rollback**: Delete configuration files (nx.json, pnpm-workspace.yaml, project.json files). No production impact.

**Decision Gate**: Go/No-Go based on:

- ✅ All 8 projects visible in `nx show projects`
- ✅ No pnpm dependency conflicts
- ✅ `nx:run-commands` targets configured correctly

### Phase 1: Structure Migration (Directory Reorganization)

**Goal**: Move `book-source/` and `panaversity-fs/` to `apps/`, extract plugins to `libs/`, update path references. Create symlink for backward compatibility.

**Duration**: 3-4 days

**Steps**:

1. **Move apps** (Day 1):

   ```bash
   git mv book-source apps/learn-app
   git mv panaversity-fs apps/panaversity-fs-py
   ```

2. **Extract plugins to libs/** (Day 1-2):

   ```bash
   mkdir -p libs
   git mv apps/learn-app/plugins/remark-interactive-python libs/docusaurus/
   git mv apps/learn-app/plugins/remark-content-enhancements libs/docusaurus/
   git mv apps/learn-app/plugins/docusaurus-plugin-og-image libs/docusaurus/plugin-og-image
   git mv apps/learn-app/plugins/docusaurus-plugin-structured-data libs/docusaurus/plugin-structured-data
   git mv apps/learn-app/plugins/docusaurus-summaries-plugin libs/docusaurus/summaries-plugin
   git mv apps/learn-app/plugins/docusaurus-panaversityfs-plugin libs/docusaurus/panaversityfs-plugin
   ```

3. **Move shared scripts** (Day 2):

   ```bash
   mkdir -p tools/scripts
   git mv apps/panaversity-fs-py/scripts/hydrate-book.py tools/scripts/
   git mv apps/panaversity-fs-py/scripts/ingest-book.py tools/scripts/
   ```

4. **Create backward-compatible symlink** (Day 2):

   ```bash
   ln -s apps/learn-app book-source
   ```

5. **Update Claude Code path references** (Day 3):

   - Option A: Keep symlink, update only critical paths (CLAUDE.md, lesson-template.md)
   - Option B: Batch-update all 61+ references to `apps/learn-app/`
   - **Recommended**: Option A (minimal disruption)

   Critical files to update:

   - `CLAUDE.md` lines 54, 60 (context-gathering protocol)
   - `.claude/output-styles/structural/lesson-template.md`
   - `.claude/output-styles/structural/file-organization.md`

6. **Update build references** (Day 3):

   - `apps/learn-app/package.json`: Update plugin import paths (`../../../libs/docusaurus/remark-interactive-python`)
   - `apps/learn-app/docusaurus.config.ts`: Update plugin paths
   - CI scripts: Update references from `panaversity-fs/scripts/` to `tools/scripts/`

7. **Validation** (Day 4):
   - Run `nx show projects` → all 8 projects still listed
   - Run `nx build learn-app` → builds successfully
   - Run `nx test panaversity-fs-py` → tests pass (301 tests)
   - Verify symlink: `ls -la book-source` → points to `apps/learn-app`
   - Verify book writer workflow: Create test lesson, confirm file appears in `apps/learn-app/docs/`

**Success Criteria**:

- All git history preserved (verify `git log --follow apps/learn-app/docs/chapter-index.md`)
- Website builds without errors
- Python tests pass
- Symlink resolves correctly
- Book writer can create lesson via Claude Code skill

**Rollback**: `git revert <commit>`, restore original directory structure. Estimated 30 minutes.

**Decision Gate**: Go/No-Go based on:

- ✅ Website build succeeds
- ✅ Python tests pass
- ✅ No broken symlinks
- ✅ Book writer workflow tested successfully

### Phase 2: CI/CD Migration (Parallel Workflows)

**Goal**: Update GitHub Actions workflows to use `nx affected` for test/build selection. Run old and new workflows in parallel for validation.

**Duration**: 3-4 days

**Steps**:

1. **Create new Nx-based workflows** (Day 1-2):

   - `.github/workflows/ci.yml`: PR validation (per official Nx CI docs):

     ```yaml
     name: CI
     on:
       push:
         branches: [main]
       pull_request:

     permissions:
       actions: read
       contents: read

     jobs:
       main:
         runs-on: ubuntu-latest
         steps:
           - uses: actions/checkout@v4
             with:
               filter: tree:0
               fetch-depth: 0 # Required for nx affected
           - uses: actions/setup-node@v4
             with:
               node-version: 20
           - uses: pnpm/action-setup@v2
             with:
               version: 9
           - run: pnpm install --frozen-lockfile
           - run: npx nx affected -t lint test build --base=origin/main --head=HEAD
     ```

   - Update `.github/workflows/deploy.yml`:
     - Add affected detection: `npx nx show projects --affected --base=origin/main~1 --head=origin/main | grep -q learn-app`
     - Update build command: `pnpm nx build learn-app` (instead of `cd book-source && npm run build`)
     - Update script paths: `tools/scripts/hydrate-book.py` (instead of `panaversity-fs/scripts/...`)
   - Keep `.github/workflows/validate.yml` mostly unchanged (content validation)

2. **Configure parallel execution** (Day 2):

   - Copy old `deploy.yml` to `.github/workflows-backup/deploy-old.yml`
   - Temporarily run both old and new workflows (dual validation)
   - Add manual approval gate before production deployment

3. **Test on feature branch** (Day 3):

   - Create PR with Python-only changes → verify Nx affected skips learn-app build
   - Create PR with docs-only changes → verify Nx affected triggers learn-app build
   - Create PR with plugin changes → verify both learn-app and plugin libs build
   - Verify all secrets still accessible in new workflow

4. **Monitor parallel runs** (Day 3-4):
   - Compare build times: old vs new workflow
   - Verify both produce identical artifacts (GitHub Pages build output)
   - Check for issues with Nx caching, affected detection, environment variables

**Success Criteria**:

- Nx affected correctly identifies changed projects
- New workflow produces identical GitHub Pages artifacts as old workflow
- Build times show improvement (or baseline established for Phase 4)
- All 7 secrets/variables accessible in new workflow

**Rollback**: Disable new `ci.yml` and `deploy.yml`, restore old workflows. Estimated 15 minutes.

**Decision Gate**: Go/No-Go based on:

- ✅ Parallel workflows both succeed on 3+ test PRs
- ✅ Affected detection works correctly
- ✅ No secret access issues
- ✅ Manual approval checkpoint before next phase

### Phase 3: Production Cutover (Go-Live)

**Goal**: Switch production CI/CD to Nx workflows exclusively. Monitor for 48 hours. Remove parallel old workflows.

**Duration**: 1-2 days (plus 48h monitoring)

**Steps**:

1. **Pre-cutover checkpoint** (Day 1 morning):

   - Create git tag: `git tag migration-phase3-checkpoint`
   - Backup old workflows: Ensure `.github/workflows-backup/` contains copies
   - Notify team: "Phase 3 cutover starting, expect CI changes"
   - Document rollback procedure (exact git commands, workflow restoration steps)

2. **Production cutover** (Day 1 afternoon, low-traffic time):

   - Merge Phase 2 PR (new workflows active, old workflows still present)
   - Disable old workflows (rename to `.github/workflows-old/*.yml.disabled`)
   - First production deploy uses Nx workflow
   - Monitor GitHub Actions logs, GitHub Pages deployment

3. **Validation** (Day 1, immediate):

   - Website loads: https://panaversity.ai
   - Book content present (spot-check 5 chapters)
   - No 404 errors in browser console
   - Analytics tracking works (GA4)
   - Backend API reachable (if deployed)

4. **48-hour monitoring period** (Day 2-3):

   - Monitor every deployment (push to main triggers deploy)
   - Track build times vs baseline
   - Watch for affected detection failures (tests skipped incorrectly)
   - Check for environment variable issues
   - Monitor team feedback (developers blocked?)

5. **Finalization** (Day 3, if no issues):
   - Remove old workflow files (delete `.github/workflows-old/`)
   - Update documentation: README.md, CONTRIBUTING.md reference Nx commands
   - Remove transition symlink `book-source → apps/learn-app` (if safe, or keep permanently)

**Success Criteria**:

- GitHub Pages deploys successfully on first try
- No production incidents during 48h window
- Zero book writer workflow disruptions reported
- All metrics stable (traffic, performance, error rates)

**Rollback**: If issues arise:

1. `git revert HEAD` (revert Phase 3 merge commit)
2. Restore old workflows: `git mv .github/workflows-old/*.yml.disabled .github/workflows/`
3. Push to main, wait for old workflow to deploy
4. Estimated rollback time: 30 minutes
5. Maximum downtime: 2 hours (as per spec constraint C-004)

**Decision Gate**: Go/No-Go based on:

- ✅ GitHub Pages deployment succeeds
- ✅ Website loads correctly (functional test)
- ✅ No CI failures on main branch for 48 hours
- ✅ Team reports no workflow disruptions

### Phase 4: Optimization (Post-Migration Tuning)

**Goal**: Fine-tune caching, evaluate Nx Cloud for remote caching, measure performance improvements, document learnings.

**Duration**: 3-5 days (optional continuous improvement)

**Steps**:

1. **Cache optimization** (Day 1-2):

   - Review Nx cache hit rates: `nx reset && pnpm nx run-many -t build` (measure cache performance)
   - Tune cache inputs/outputs in `nx.json`:
     - Ensure `pythonInputs` includes `pyproject.toml`, `uv.lock`
     - Ensure `docusaurusInputs` includes `docs/**`, `static/**`
   - Test cache invalidation: Change Python file → verify only Python tests run

2. **Nx Cloud evaluation** (Day 2-3, optional):

   - Connect to Nx Cloud: `npx nx connect`
   - Configure distributed caching in GitHub Actions
   - Measure CI time improvement with remote cache
   - Decision: Keep if >30% time reduction, otherwise local cache sufficient

3. **Performance benchmarking** (Day 3):

   - Baseline (pre-migration): Record build/test times from old CI logs
   - Post-migration: Measure same operations with Nx
   - Target: 50% CI time reduction (spec SC-003)
   - Calculate cache hit rate (target: >80%, spec SC-008)

4. **Documentation updates** (Day 4):

   - Update README.md: New monorepo structure, Nx commands
   - Update CONTRIBUTING.md: PR workflow with `nx affected`
   - Create DEVELOPER-SETUP.md (Gap #19 from MIGRATION-GAPS-ANALYSIS.md)
   - Create ONBOARDING.md (Gap #25)
   - Document learnings in ADR-0021 addendum

5. **Gap closure** (Day 5, based on priority):
   - Create P0 documents from MIGRATION-GAPS-ANALYSIS.md:
     - TESTING-STRATEGY.md (Gap #26)
     - ROLLBACK-TESTING-PROCEDURE.md (Gap #31)
     - INTEGRATION-CHECKLIST.md (Gap #28)

**Success Criteria**:

- Cache hit rate >80% for unchanged projects
- CI time reduced by 50% (or baseline documented if not achieved)
- Developer setup time <15 minutes (spec SC-001)
- All P0 gap documents created
- Team trained on Nx commands

**Rollback**: N/A (optimization phase, no production risk)

**Decision Gate**: Project complete when:

- ✅ All success criteria from spec met (SC-001 to SC-008)
- ✅ No unresolved issues from production monitoring
- ✅ P0 documentation gaps closed
- ✅ Team comfortable with Nx workflows

## Testing Strategy

### Unit Testing

**Website (TypeScript)**:

- No unit tests currently (Docusaurus doesn't require them)
- Post-migration: Optional Jest tests for custom components
- Target: `nx test learn-app` (currently passes with no tests)

**PanaversityFS (Python)**:

- Existing: 301 tests across 7 categories (unit, integration, property, performance, e2e, scripts, edge_cases)
- Target: `nx test panaversity-fs-py` runs pytest
- No changes to test code required
- Verify: All 301 tests pass after migration

**Plugins (6 libs)**:

- No unit tests currently
- Post-migration: Optional Jest tests per plugin
- Target: `nx test {plugin-name}` (currently passes with no tests)

### Integration Testing

**Cross-Project Integration**:

- Website imports 6 plugin libs → verify imports resolve correctly
- Hydrate target calls `tools/scripts/hydrate-book.py` → verify script runs
- CI workflow uses affected detection → verify correct projects detected

**Test Cases**:

1. Change only Python code → `nx affected -t test` runs only Python tests
2. Change only Docusaurus content → `nx affected -t build` builds only learn-app
3. Change plugin lib → `nx affected -t build` builds plugin + learn-app
4. Change shared script → `nx affected -t test` runs both learn-app and Python

**Execution**: Manual testing on feature branch during Phase 2

### End-to-End Testing

**Book Writer Workflow**:

1. Book writer invokes context-gathering protocol (CLAUDE.md)
2. Read `apps/learn-app/docs/chapter-index.md` → resolves via symlink
3. Create new lesson in `apps/learn-app/docs/test-part/test-chapter/lesson.md`
4. Verify file appears in `apps/learn-app/docs/test-part/test-chapter/lesson.md`
5. Run `nx build learn-app` → lesson included in build
6. Deploy to GitHub Pages → lesson visible on site

**CI/CD Pipeline**:

1. Create PR with Python changes
2. GitHub Actions triggers `nx affected -t test`
3. Only Python tests run (learn-app build skipped)
4. Merge PR to main
5. GitHub Actions triggers `nx build learn-app` (for deployment)
6. GitHub Pages updates successfully

**Execution**: Manual testing during Phase 1 (book writer) and Phase 2-3 (CI/CD)

### Performance Testing

**Metrics to Measure**:

- New developer setup time: < 15 minutes (spec SC-001)
- CI build time for affected projects: 50% reduction target (spec SC-003)
- Cache hit rate: > 80% target (spec SC-008)
- Full monorepo build time: Baseline vs optimized
- Affected detection accuracy: 100% (spec SC-002)

**Measurement Strategy**:

- Phase 0: Baseline current setup time, CI times
- Phase 2: Measure Nx affected times on feature branch
- Phase 3: Measure production CI times over 48h
- Phase 4: Final benchmarking with optimization

**Tools**:

- GitHub Actions logs (build duration)
- Nx CLI output (`nx affected --base=main --head=HEAD`)
- Manual stopwatch for developer setup

### Contract Testing

**Python MCP Server** (apps/panaversity-fs-py):

- Existing: Property tests verify R1-R7 requirements (schema compliance, path validation, audit chain)
- No changes required
- Verify: `nx test panaversity-fs-py` runs property tests successfully

**Docusaurus Plugins** (6 libs):

- No explicit contracts currently
- Post-migration: Consider adding interface tests if plugins are published independently

**CI/CD Contracts**:

- Old workflow → New workflow: Same GitHub Pages artifact produced
- Test: Compare `book-source/build/` (old) vs `apps/learn-app/build/` (new)
- Validation: Hash comparison, spot-check 10 HTML files

## Deployment Strategy

### Environments

**Development** (Local):

- Developers run `pnpm install && pnpm nx serve learn-app` from workspace root
- Hot reload works via Docusaurus dev server
- Python backend: `pnpm nx serve panaversity-fs-py` (runs uvicorn --reload)

**Staging** (Feature Branch):

- CI runs on feature branches
- Affected detection tested on PRs
- Manual approval before Phase 3 production cutover

**Production** (Main Branch):

- GitHub Actions auto-deploys on push to main
- Target: GitHub Pages (website) + Cloud Run (backend, if deployed)
- Deployment frequency: Every commit to main (continuous deployment)

### Deployment Steps

**Website (GitHub Pages)**:

1. Trigger: Push to main, paths: `apps/learn-app/**`
2. GitHub Actions runs: `pnpm nx build learn-app`
3. Output: `apps/learn-app/build/` uploaded to GitHub Pages
4. Deploy: GitHub Pages deployment action
5. Verification: Website loads at https://panaversity.ai

**Backend (Cloud Run)** (if deployed):

1. Trigger: Push to main, paths: `apps/panaversity-fs-py/**`
2. GitHub Actions runs: `docker build -f apps/panaversity-fs-py/Dockerfile.prod`
3. Push image to registry
4. Deploy to Cloud Run
5. Verification: Health check endpoint responds

### Rollback Procedure

**Phase 3 Rollback** (if production issues):

1. Identify issue (GitHub Pages down, build failure, CI breaking)
2. Communicate: Notify team, create incident ticket
3. Execute rollback:
   ```bash
   git revert <phase3-merge-commit>
   git push origin main
   # Wait for old workflow to deploy (2-3 minutes)
   ```
4. Restore old workflows (if revert didn't restore):
   ```bash
   git checkout migration-phase3-checkpoint -- .github/workflows/
   git commit -m "revert: restore pre-migration workflows"
   git push origin main
   ```
5. Validation: GitHub Pages loads, CI builds succeed
6. Post-incident: Document root cause, update plan, retry Phase 3 when fixed

**Maximum Downtime**: 2 hours (as per spec constraint C-004)

**Rollback Testing**: Before Phase 3, test rollback procedure on feature branch (verify old workflows redeploy successfully)

### Blue-Green Deployment

**Not Applicable** for Phase 3 cutover (GitHub Pages doesn't support blue-green natively).

**Alternative Strategy**: Parallel workflows (Phase 2) serve as validation before cutover.

**Future Enhancement**: If backend deployed to Cloud Run, consider blue-green deployment with traffic splitting.

### Monitoring

**During Migration** (Phase 2-3):

- GitHub Actions logs: Build success rate, duration
- GitHub Pages: Deployment status, uptime
- Manual spot-checks: Website loads, content accessible
- Team feedback: Slack channel for issue reporting

**Post-Migration** (Phase 4):

- Nx cache hit rate: `nx reset && nx run-many -t build` (observe cache usage)
- CI build times: Track via GitHub Actions duration metrics
- Developer experience: Survey team (setup time, workflow satisfaction)
- Incident rate: Monitor for build failures, deployment issues

**Alerting**: GitHub Actions email notifications (on workflow failure)

## Dependencies

**External Dependencies**:

- Node.js 20+ installed on all developer machines and CI runners
- pnpm 9.12+ installed globally or via `corepack enable`
- Python 3.13 + uv (for Python project development)
- Git with symlink support (macOS/Linux native, Windows requires WSL or Git Bash)
- GitHub Secrets: 7 variables (GA4_MEASUREMENT_ID, PANAVERSITY_SERVER_URL, etc.) configured

**Internal Dependencies** (must complete before this migration):

- ✅ ADR-0020 approved (Nx as monorepo standard)
- ✅ Analysis documents completed (5 docs in `docs/migration-monorepo-plan/`)
- ✅ Team alignment on migration timeline
- ✅ Backup workflows stored (`.github/workflows-backup/`)

**Blocking Dependencies** (must resolve during migration):

- Phase 1 blocked by Phase 0 (Nx configuration must exist before directory moves)
- Phase 3 blocked by Phase 2 (must validate parallel workflows before production cutover)
- Phase 4 unblocked (can start immediately after Phase 3 monitoring period)

## Risks and Mitigations

### Risk 1: Book Writer Workflows Break (HIGH IMPACT)

**Likelihood**: Medium
**Impact**: High (blocks all content authoring)

**Mitigation**:

- Strategy 1: Symlink `book-source → apps/learn-app` maintains backward compatibility
- Strategy 2: Update critical path references in CLAUDE.md, lesson-template.md (61+ references cataloged in BOOK-WRITER-WORKFLOW-IMPACT.md)
- Strategy 3: Test book writer workflow in Phase 1 before Phase 2 (create test lesson, verify paths)

**Contingency**: If workflows break despite symlink, revert Phase 1 (git revert directory moves). Estimated rollback time: 30 minutes.

### Risk 2: CI/CD Breaks Production Deployment (CRITICAL IMPACT)

**Likelihood**: Medium
**Impact**: Critical (no deployments possible)

**Mitigation**:

- Strategy 1: Parallel workflows (Phase 2) validate new CI before cutover
- Strategy 2: Manual approval gate before Phase 3 production cutover
- Strategy 3: Rollback procedure tested on feature branch (dry-run)
- Strategy 4: 48-hour monitoring period after cutover

**Contingency**: Immediate rollback to old workflows (documented in Deployment Strategy). Maximum downtime: 2 hours (spec constraint C-004).

### Risk 3: Python Build Integration Fails Edge Cases (MEDIUM IMPACT)

**Likelihood**: Low
**Impact**: Medium (Python tests fail, development blocked)

**Mitigation**:

- Strategy 1: Using official `nx:run-commands` executor (no custom code, battle-tested)
- Strategy 2: Makefile wrapper preserves existing Python tooling (zero Python code changes)
- Strategy 3: Test all Python targets in Phase 0-1 (test, lint, format, build)
- Strategy 4: Fallback: Developers can run `make test` directly (bypassing Nx)

**Contingency**: Simplify `nx:run-commands` to call pytest directly (no Makefile). Estimated fix time: 1 hour.

### Risk 4: Path References Missed (LOW IMPACT)

**Likelihood**: Medium
**Impact**: Low (specific skill breaks, but others work)

**Mitigation**:

- Strategy 1: Comprehensive search completed (61+ references cataloged in BOOK-WRITER-WORKFLOW-IMPACT.md)
- Strategy 2: Symlink strategy makes most paths work without updates
- Strategy 3: Incremental discovery: Fix paths as they're reported

**Contingency**: Update paths on-demand as issues reported. No rollback required (isolated failures).

### Risk 5: Nx Daemon Memory Issues on CI (LOW IMPACT)

**Likelihood**: Low
**Impact**: Low (slower builds, no functional breakage)

**Mitigation**:

- Strategy 1: Configure Nx daemon timeout in `nx.json`
- Strategy 2: Fallback: `NX_DAEMON=false` environment variable in GitHub Actions
- Strategy 3: Monitor CI runner memory usage during Phase 2

**Contingency**: Disable Nx daemon in CI (builds still work, just slower). No rollback required.

## Success Criteria

**Phase 0 Success**:

- ✅ Nx recognizes all 8 projects (`nx show projects`)
- ✅ Dependency graph shows correct relationships (`nx graph`)
- ✅ No pnpm installation errors

**Phase 1 Success**:

- ✅ Website build succeeds (`nx build learn-app`)
- ✅ Python tests pass (301 tests via `nx test panaversity-fs-py`)
- ✅ Symlink resolves correctly
- ✅ Book writer workflow tested (create lesson, verify location)

**Phase 2 Success**:

- ✅ Parallel workflows both succeed on 3+ test PRs
- ✅ Affected detection works correctly (Python change → skips learn-app)
- ✅ No secret access issues in new workflow

**Phase 3 Success**:

- ✅ GitHub Pages deployment succeeds on first try
- ✅ No production incidents during 48h monitoring
- ✅ Zero book writer workflow disruptions reported

**Phase 4 Success**:

- ✅ Cache hit rate >80% (spec SC-008)
- ✅ CI time reduced by 50% (spec SC-003) or baseline documented
- ✅ Developer setup <15 minutes (spec SC-001)
- ✅ All P0 gap documents created

**Overall Success** (from spec.md):

- SC-001: New developer setup in under 15 minutes ✅
- SC-002: `nx affected` correctly identifies changed projects with 100% accuracy ✅
- SC-003: CI pipeline time for single-project changes reduced by 50% ✅
- SC-004: Zero book writer workflow disruptions reported in first 2 weeks ✅
- SC-005: All 8 projects visible and correctly connected in `nx graph` output ✅
- SC-006: Rollback can be executed in under 30 minutes with documented procedure ✅
- SC-007: Python tests pass via `nx run panaversity-fs-py:test` with same results ✅
- SC-008: Cache hit rate above 80% for unchanged projects in CI ✅

## Timeline

**Total Duration**: 11-14 days (plus 48h monitoring)

**Phase 0**: 2-3 days (Preparation)
**Phase 1**: 3-4 days (Structure Migration)
**Phase 2**: 3-4 days (CI/CD Migration with parallel validation)
**Phase 3**: 1-2 days + 48h monitoring (Production Cutover)
**Phase 4**: 3-5 days (Optimization, optional continuous improvement)

**Critical Path**: Phase 0 → Phase 1 → Phase 2 → Phase 3 (blocking sequence)
**Parallelization**: Phase 4 can overlap with Phase 3 monitoring period

**Milestones**:

- Day 3: Phase 0 complete (Nx configuration validated)
- Day 7: Phase 1 complete (Structure migrated, book writer workflow tested)
- Day 11: Phase 2 complete (Parallel workflows validated)
- Day 14: Phase 3 complete (Production cutover, monitoring complete)
- Day 19: Phase 4 complete (Optimization and gap closure)

**Assumptions**:

- No major blockers discovered during execution
- Team availability for Phase 3 production cutover (requires low-traffic time)
- Rollback not needed (adds 2-3 days if executed)

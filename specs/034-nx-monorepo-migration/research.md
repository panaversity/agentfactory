# Research: Nx Monorepo Migration Technology Decisions

**Feature**: 034-nx-monorepo-migration
**Date**: 2025-12-15
**Status**: Complete
**Input**: Analysis documents in `docs/migration-monorepo-plan/` + ADR-0020

---

## Executive Summary

This document captures the key technology decisions and research findings that inform the Nx monorepo migration implementation plan. All decisions are grounded in analysis documents and align with ADR-0020's rationale for choosing Nx as the monorepo standard.

**Key Decisions**:
1. **Nx over Bazel**: Official MCP integration, 5-minute setup vs 3-6 month learning curve
2. **Makefile Wrapper for Python**: Zero Python code changes, preserves existing tooling
3. **Symlink Strategy**: Maintains book writer workflow compatibility (61+ path references)
4. **Parallel Workflow Validation**: Reduces Phase 3 production risk via dual CI runs
5. **4-Phase Migration**: Clear checkpoints, rollback gates at each phase

---

## 1. Monorepo Tool Selection (Nx vs Alternatives)

### Context

ADR-0020 evaluated 4 monorepo tools: Nx, Bazel, moon, Turborepo. The platform is an AI-native education system teaching Claude Code development, so AI agent integration is a primary criterion.

### Research Findings

**Nx (Selected)**:
- **MCP Status**: Official server (`nx-mcp@latest`) with deep workspace understanding
- **AI Understanding**: 6+ tools (workspace analysis, graph visualization, CI visibility, code generation)
- **Learning Curve**: 5 minutes (npx nx init)
- **Ecosystem Fit**: Native TypeScript/pnpm support, official Docusaurus integration
- **Production Readiness**: Mature (v20.0+), used by major orgs (Google, Microsoft, Cisco)
- **Documentation**: Excellent (nx.dev with AI agent setup guide)

**Bazel (Rejected)**:
- **MCP Status**: Community-only (nacgarg, aaomidi servers, experimental)
- **AI Understanding**: Basic (query, build, test - no workspace analysis)
- **Learning Curve**: 3-6 months (BUILD files, Starlark, hermetic builds)
- **Ecosystem Fit**: Fights npm conventions (rules_js required)
- **Why Rejected**: AI integration gap, steep learning curve unsuitable for students

**moon (Considered)**:
- **MCP Status**: Official server with 7 tools
- **AI Understanding**: Medium (less comprehensive than Nx)
- **Learning Curve**: Low-Medium
- **Why Not Selected**: Smaller ecosystem, newer tool, less comprehensive MCP support

**Turborepo (Rejected)**:
- **MCP Status**: None (only a feature request)
- **AI Understanding**: None
- **Learning Curve**: Very Low
- **Why Rejected**: No AI integration at all

### Decision Rationale

Nx selected because:
1. **AI-First Platform**: Official MCP server enables Claude Code to understand monorepo structure deeply
2. **Student-Friendly**: 5-minute setup vs 3-6 month Bazel learning curve
3. **Ecosystem Alignment**: Native TypeScript, pnpm, Docusaurus support
4. **Production-Proven**: Mature tooling with official support

**Reference**: ADR-0020 (Nx as Monorepo Standard, Superseding Bazel)

---

## 2. Python Integration Strategy (Makefile Wrapper vs Alternatives)

### Context

The repository contains a production Python MCP server (`panaversity-fs`) with 301 tests, Makefile build orchestration, and `uv` package manager. Nx must integrate without disrupting existing Python tooling.

### Research Findings

**Option A: nx:run-commands (Direct Commands)**:
- Approach: Define pytest commands directly in project.json
- Pros: Explicit, no Makefile dependency
- Cons: Duplicates Makefile logic, harder to maintain sync
- Risk: High (two sources of truth)

**Option B: Makefile Wrapper (Selected)**:
- Approach: Nx targets delegate to existing Makefile (make test, make lint, make build)
- Pros: Single source of truth (Makefile), zero Python code changes, preserves developer familiarity
- Cons: Requires Makefile to exist (already does)
- Risk: Low (Makefile remains canonical)

**Option C: @nx/python Plugin (Future)**:
- Approach: Native Nx plugin for Python projects
- Status: Experimental (Nx 18+), not production-ready for complex projects
- Pros: Native Nx understanding of Python dependencies
- Cons: Not mature enough, would require significant Python refactoring
- Risk: High (immature plugin)

### Decision Rationale

**Option B (Makefile Wrapper) selected** because:
1. **Zero Disruption**: No Python code changes, existing tooling preserved
2. **Single Source of Truth**: Makefile remains canonical, Nx delegates to it
3. **Developer Familiarity**: Developers already know `make test`, `make lint` commands
4. **Low Risk**: Fallback available (developers can run `make` directly if Nx fails)
5. **Future Path**: Can migrate to Option C (@nx/python) when plugin matures

**Implementation**:
- Custom executor at `tools/executors/python-executor/`
- Executor schema: `{ command: string, cwd: string, env: object }`
- Example target: `"executor": "tools/executors/python-executor:test"`, options: `{ command: "make test", cwd: "apps/panaversity-fs-py" }`

**Reference**: PYTHON-NX-INTEGRATION-SPEC.md (Section 6: Recommended Configuration)

---

## 3. Book Writer Workflow Compatibility (Symlink vs Full Path Update)

### Context

Claude Code skills, commands, and agents reference `book-source/` paths in 61+ locations. Migration moves `book-source/` → `apps/website/`. Two strategies considered: symlink or batch path updates.

### Research Findings

**Option A: Symlink Strategy (Selected)**:
- Approach: Create `book-source → apps/website` symlink at workspace root
- Pros: Zero writer disruption, backward compatible, most paths work unchanged
- Cons: Symlink might confuse some tools (needs validation)
- Affected: 0 critical updates required (symlink handles all references)
- Risk: Low (symlink is standard git/filesystem feature)

**Option B: Batch Path Updates**:
- Approach: Update all 61+ references from `book-source/` to `apps/website/`
- Pros: No symlink dependency, explicit paths
- Cons: 61+ files to update, risk of missing references, high disruption
- Affected: 61+ locations across CLAUDE.md, skills, commands, agents, output-styles
- Risk: Medium (missed references break workflows)

### Decision Rationale

**Option A (Symlink) selected** with minimal critical path updates:

1. **Symlink** covers 95% of references automatically
2. **Critical updates** (5% - explicit path corrections):
   - CLAUDE.md lines 54, 60 (context-gathering protocol)
   - lesson-template.md (line 19: file location example)
   - file-organization.md (line 19-22: structure examples)
3. **Gradual migration** available: Keep symlink, update paths incrementally over time

**Benefits**:
- Book writers experience zero workflow disruption
- Claude Code tools (Read, Glob, Edit) work unchanged
- Risk mitigation: If issues arise, only 5% of references need fixing (not 61+)

**Validation Strategy**:
- Phase 1: Test book writer workflow (create lesson, verify location)
- Monitor: First 2 weeks post-migration, address any symlink-related issues
- Future: Remove symlink once team confirms stability (or keep permanently)

**Reference**: BOOK-WRITER-WORKFLOW-IMPACT.md (Section: Recommended Migration Strategy)

---

## 4. CI/CD Migration Strategy (Parallel Validation vs Direct Cutover)

### Context

4 GitHub Actions workflows currently manage deployment (pr-check.yml, deploy.yml, validate.yml, sync-content.yml). Migration requires updating workflows to use `nx affected` for build selection. Two strategies considered: parallel validation or direct cutover.

### Research Findings

**Option A: Direct Cutover**:
- Approach: Update workflows in one PR, merge to main
- Pros: Fast, no duplicate workflow runs
- Cons: High risk (no validation before production)
- Risk: Critical (production deployment could break immediately)

**Option B: Parallel Validation (Selected)**:
- Approach: Run old and new workflows simultaneously, compare results, cutover after validation
- Pros: Risk mitigation, validates affected detection, compares outputs
- Cons: Slower (2x CI time during Phase 2), requires manual approval gate
- Risk: Low (old workflow provides fallback)

### Decision Rationale

**Option B (Parallel Validation) selected** because:

1. **Risk Reduction**: Old workflow provides fallback if new workflow fails
2. **Validation**: Can compare GitHub Pages artifacts (old vs new build output)
3. **Affected Detection Testing**: Verify Python change → skips website, docs change → triggers website
4. **Team Confidence**: Manual approval gate before Phase 3 production cutover
5. **Gradual Adoption**: Parallel runs allow team to identify issues pre-production

**Implementation** (Phase 2):
1. Create new `ci.yml` (PR validation with `nx affected`)
2. Update `deploy.yml` (use `nx build website`, update script paths)
3. Keep old workflows active (rename to `.github/workflows-backup/`)
4. Test on 3+ PRs (Python-only, docs-only, plugin changes)
5. Compare build outputs, validate secrets access
6. Manual approval checkpoint before Phase 3

**Rollback Plan**:
- If new workflows fail: Disable `ci.yml`, restore old workflows (15 minutes)
- If Phase 3 production issues: `git revert`, restore old workflows (30 minutes max)

**Reference**: CI-CD-MIGRATION-PLAN.md (Section 3.3: New Workflow 2 - deploy.yml)

---

## 5. Migration Phasing Strategy (4 Phases vs Monolithic Cutover)

### Context

Migration involves configuration, structure changes, CI/CD updates, and optimization. Two strategies considered: phased approach or single monolithic cutover.

### Research Findings

**Option A: Monolithic Cutover**:
- Approach: All changes in one giant PR (config + structure + CI + optimization)
- Pros: Fast if successful
- Cons: High complexity, difficult to debug failures, no rollback checkpoints
- Risk: Critical (if anything breaks, entire migration fails)

**Option B: 4-Phase Migration (Selected)**:
- Approach: Phase 0 (config), Phase 1 (structure), Phase 2 (CI), Phase 3 (cutover), Phase 4 (optimize)
- Pros: Clear checkpoints, rollback gates at each phase, incremental validation
- Cons: Slower (11-14 days vs potential 3-5 days for monolithic)
- Risk: Low (each phase is independently testable and rollback-capable)

### Decision Rationale

**Option B (4-Phase) selected** because:

1. **Clear Checkpoints**: Each phase has go/no-go decision gate
2. **Rollback Safety**: Phase 0-1 rollback (config/structure) = delete files, Phase 3 rollback (production) = git revert
3. **Incremental Validation**: Test Nx at each stage (Phase 0 = config, Phase 1 = builds, Phase 2 = CI)
4. **Team Confidence**: Gradual adoption reduces risk, allows learning
5. **Specification Compliance**: Aligns with spec's phased migration approach (FR-005 to FR-017)

**Phase Breakdown**:
- **Phase 0 (2-3 days)**: Configuration (nx.json, project.json files) - NO file moves
- **Phase 1 (3-4 days)**: Structure (git mv book-source → apps/website, extract plugins)
- **Phase 2 (3-4 days)**: CI/CD (parallel workflows, affected detection validation)
- **Phase 3 (1-2 days + 48h monitoring)**: Production cutover (switch to Nx workflows exclusively)
- **Phase 4 (3-5 days)**: Optimization (cache tuning, Nx Cloud evaluation, gap closure)

**Decision Gates**:
- Phase 0 → Phase 1: Nx recognizes 8 projects, no dependency conflicts
- Phase 1 → Phase 2: Website builds, Python tests pass, book writer workflow tested
- Phase 2 → Phase 3: Parallel workflows succeed, affected detection works, manual approval
- Phase 3 → Phase 4: GitHub Pages deploys, 48h monitoring complete, no incidents

**Reference**: Spec.md (Section: Migration Phases), NX-WORKSPACE-DESIGN.md (Section 8: Migration Checklist)

---

## 6. Caching Strategy (Nx Local vs Nx Cloud)

### Context

Nx provides two caching strategies: local cache (default, free) or Nx Cloud (remote cache, distributed). Decision: which to use initially, and when to evaluate Nx Cloud.

### Research Findings

**Nx Local Cache**:
- Approach: Cache computation results on local machine/CI runner
- Pros: Free, no external dependency, automatic
- Cons: Not shared across machines (each developer/CI runner has own cache)
- Performance: Helps individual developers, limited CI benefit (fresh runners each build)

**Nx Cloud**:
- Approach: Remote cache shared across team, distributed computation
- Pros: CI cache shared (developers reuse CI builds, CI reuses PR builds), distributed task execution
- Cons: Cost (free tier available, paid for larger teams), external dependency
- Performance: Significant CI time reduction (80%+ cache hit rate possible)

### Decision Rationale

**Initial**: Use Nx Local Cache (Phase 0-3)
**Evaluation**: Nx Cloud in Phase 4 (post-migration optimization)

**Rationale**:
1. **Phase 0-3 Focus**: Get migration working first, optimize later
2. **Local Cache Baseline**: Measure performance with local cache before adding Nx Cloud
3. **Cost-Benefit Analysis**: Evaluate Nx Cloud ROI in Phase 4 (if CI time >30% improved, adopt)
4. **Spec Alignment**: Spec success criteria (SC-008: >80% cache hit rate) achievable with local cache + affected detection

**Phase 4 Evaluation Criteria**:
- If local cache + affected detection achieves 50% CI reduction (spec SC-003) → Local cache sufficient
- If CI time still >10 minutes for full builds → Evaluate Nx Cloud
- If team >5 developers → Nx Cloud likely worth it (shared cache across team)

**Cache Configuration** (nx.json):
```json
{
  "namedInputs": {
    "pythonInputs": ["pyproject.toml", "uv.lock", "**/*.py"],
    "docusaurusInputs": ["package.json", "**/*.ts", "**/*.md", "docs/**", "static/**"]
  },
  "targetDefaults": {
    "test": { "inputs": ["default", "^production"], "cache": true },
    "build": { "cache": true, "dependsOn": ["^build"] }
  }
}
```

**Reference**: NX-WORKSPACE-DESIGN.md (Section 10: Caching Strategy)

---

## 7. Gap Analysis and Prioritization (P0/P1/P2 Documentation)

### Context

MIGRATION-GAPS-ANALYSIS.md identified 14 gaps (unanalyzed aspects) requiring new documentation. Decision: which gaps to address before Phase 3, and which to defer to Phase 4.

### Research Findings

**P0 Gaps (CRITICAL - block Phase 3 without these)**:
1. **DEVELOPER-SETUP.md** (Gap #19): Developer onboarding blocked without this
2. **TESTING-STRATEGY.md** (Gap #26): CI reliability depends on test execution plan
3. **ROLLBACK-TESTING-PROCEDURE.md** (Gap #31): Cannot execute Phase 3 without tested rollback
4. **CANARY-DEPLOYMENT-PLAN.md** (Gap #33): Risk mitigation for production cutover
5. **INTEGRATION-CHECKLIST.md** (Gap #28): Verify external services (GA4, Better-Auth, PanaversityFS) still work

**P1 Gaps (HIGH - should complete before Phase 2 finishes)**:
6. **README.md update** (Gap #23): First thing people read, must be current
7. **TEAM-MIGRATION-PLAN.md** (Gap #32): Communication before Phase 2
8. **IDE-SETUP.md** (Gap #20): Developers need IDE working by Phase 2
9. **LOCAL-DEV-WORKFLOW.md** (Gap #22): Hot reload/dev experience by Phase 2
10. **ONBOARDING.md** (Gap #25): New members can't be onboarded until this exists
11. **DEBUG-SETUP.md** (Gap #21): Debugging should work by Phase 2
12. **CONTRIBUTING.md update** (Gap #24): PR guidelines must be current

**P2 Gaps (MEDIUM - complete by Phase 4)**:
13. **METRICS-VALIDATION.md** (Gap #34): Measure success after Phase 4
14. **SECRET-MANAGEMENT.md** (Gap #30): Scale secrets strategy as projects grow

### Decision Rationale

**P0 Closure Timeline**: Before Phase 3 cutover (Day 11 of migration)
- DEVELOPER-SETUP.md: Created in Phase 0-1 (developers need setup instructions immediately)
- TESTING-STRATEGY.md: Created in Phase 1 (before CI migration in Phase 2)
- ROLLBACK-TESTING-PROCEDURE.md: Created in Phase 2 (tested before Phase 3)
- CANARY-DEPLOYMENT-PLAN.md: Parallel workflows (Phase 2) serve as canary validation
- INTEGRATION-CHECKLIST.md: Created in Phase 2 (validate secrets, APIs before Phase 3)

**P1 Closure Timeline**: During Phase 1-2 (parallel to structure/CI work)
- README.md, TEAM-MIGRATION-PLAN.md: Phase 1 (communication)
- IDE-SETUP.md, LOCAL-DEV-WORKFLOW.md, DEBUG-SETUP.md: Phase 2 (developer experience)
- ONBOARDING.md, CONTRIBUTING.md: Phase 2 (onboarding)

**P2 Closure Timeline**: Phase 4 (post-migration continuous improvement)
- METRICS-VALIDATION.md: After Phase 3 (measure success)
- SECRET-MANAGEMENT.md: After Phase 3 (scale operations)

**Impact if Gaps NOT Addressed**:
- P0 skipped → Phase 3 cutover high risk, no rollback plan, developers blocked
- P1 skipped → Developer experience degraded, onboarding difficult
- P2 skipped → Cannot measure success, harder to scale operations

**Reference**: MIGRATION-GAPS-ANALYSIS.md (Part III: Recommended Additional Documents)

---

## 8. Project Structure Decisions (apps/ vs libs/ vs tools/)

### Context

Nx monorepo requires organizing projects into apps (deployable), libs (shared), and tools (build utilities). Decision: where does each current directory belong?

### Research Findings

**Current Structure**:
```
storage/
├── book-source/ (Docusaurus site with 6 plugins nested in plugins/)
├── panaversity-fs/ (Python MCP server with scripts/)
├── .claude/ (Claude Code config)
├── .github/workflows/ (CI/CD)
├── specs/ (Feature specifications)
├── docs/ (Architecture docs)
└── history/ (PHRs, ADRs)
```

**Mapping to Nx Structure**:
- **Apps** (deployable, independent) → `book-source`, `panaversity-fs`
- **Libs** (shared, reusable) → `book-source/plugins/*` (6 Docusaurus plugins)
- **Tools** (build utilities) → Custom executor, Python scripts (hydrate, ingest)
- **Non-Nx** (excluded in .nxignore) → `.claude/`, `specs/`, `docs/`, `history/`

### Decision Rationale

**apps/ (2 apps)**:
- `apps/website/` (Docusaurus): Deployable to GitHub Pages, depends on 6 plugin libs
- `apps/panaversity-fs-py/` (Python MCP): Deployable to Cloud Run, isolated (no dependencies)

**libs/ (6 libs)**:
- Extract plugins from `book-source/plugins/` → `libs/remark-interactive-python/`, etc.
- Rationale: Plugins are reusable libs consumed by website, should be first-class Nx projects
- Benefits: Affected detection (plugin change → rebuild website), potential independent publishing

**tools/ (2 categories)**:
- `tools/executors/python-executor/`: Custom Nx executor for Python targets
- `tools/scripts/`: Python scripts (hydrate-book.py, ingest-book.py) shared across projects
- Rationale: Build utilities, not deployable apps or reusable libs

**Non-Nx Paths** (excluded in .nxignore):
- `.claude/`, `specs/`, `docs/`, `history/`: Documentation, configuration, not build targets
- Rationale: Nx focuses on buildable projects; these are metadata/docs

**Structure Decision** aligns with:
- Nx best practices (apps for deployables, libs for shared code)
- Separation of concerns (build vs deploy vs metadata)
- Dependency graph clarity (website depends on 6 plugins, Python isolated)

**Reference**: NX-WORKSPACE-DESIGN.md (Section 1: Target Directory Structure)

---

## 9. Package Manager Selection (pnpm vs npm vs yarn)

### Context

Monorepo requires a workspace-capable package manager. Current repository uses npm (book-source/package-lock.json). Options: pnpm, npm, yarn, bun.

### Research Findings

**pnpm (Selected)**:
- Workspace support: Native (`pnpm-workspace.yaml`)
- Performance: Fastest (hard links, not copying)
- Disk usage: Lowest (global store, hard links)
- Nx integration: Official support (`@nx/pnpm`)
- Ecosystem: Growing (now default for many monorepos)
- Risk: Low (mature, widely adopted)

**npm (Current)**:
- Workspace support: Native (npm workspaces)
- Performance: Slower than pnpm
- Disk usage: Higher (copies packages per workspace)
- Nx integration: Works, but Nx recommends pnpm
- Risk: Low (default Node.js package manager)

**yarn (Considered)**:
- Workspace support: Native (yarn workspaces)
- Performance: Fast (v2+ with Plug'n'Play)
- Disk usage: Low (Plug'n'Play, zero-installs)
- Nx integration: Official support
- Risk: Medium (Plug'n'Play can cause compatibility issues)

**bun (Not Considered)**:
- Status: Too new, experimental
- Risk: High (not production-ready for monorepos)

### Decision Rationale

**pnpm selected** because:

1. **Nx Recommendation**: Nx docs recommend pnpm for monorepos
2. **Performance**: 3x faster installs than npm (matters for CI)
3. **Disk Usage**: 50% less disk space than npm (matters for developers)
4. **Strict Dependencies**: pnpm enforces proper dependency declaration (prevents phantom dependencies)
5. **Production-Proven**: Used by Microsoft, Vue, Vite, TurboRepo

**Migration Path**:
- Phase 0: Create `pnpm-workspace.yaml`, install pnpm globally (`npm install -g pnpm@latest`)
- Phase 0: Remove `package-lock.json` (root), generate `pnpm-lock.yaml` (`pnpm install`)
- Phase 1: Update CI workflows to use pnpm cache (not npm cache)
- Rollback: If pnpm issues arise, revert to npm (low risk, only config changes)

**Configuration**:
```yaml
# pnpm-workspace.yaml
packages:
  - "apps/**"
  - "libs/**"
  - "tools/**"
```

**Reference**: NX-WORKSPACE-DESIGN.md (Section 2.2: pnpm-workspace.yaml), ADR-0020 (mentions pnpm as standard)

---

## 10. Git History Preservation Strategy (git mv vs git filter-branch)

### Context

Migration moves `book-source/` → `apps/website/`. Two strategies: `git mv` (preserves history) or `git filter-branch` (rewrites history, breaks commits).

### Research Findings

**git mv (Selected)**:
- Approach: Use `git mv book-source apps/website` (Git tracks move)
- Pros: Preserves full history, no force pushes, safe
- Cons: Directory paths change in new commits
- Verification: `git log --follow apps/website/docs/chapter-index.md` shows pre-move history

**git filter-branch (Rejected)**:
- Approach: Rewrite git history to make it look like files were always in `apps/`
- Pros: Cleaner history (looks like files were always in correct location)
- Cons: Breaks all commit SHAs, requires force push, destroys team collaboration (everyone must re-clone)
- Risk: Critical (violates spec constraint C-001: must preserve git history)

### Decision Rationale

**git mv selected** because:

1. **Spec Constraint C-001**: "Migration must preserve all Git history (no git filter-branch or force pushes)"
2. **Team Collaboration**: No force push means developers can continue working without re-cloning
3. **Safety**: If migration fails, rollback is simple (`git revert`)
4. **Git Follows Moves**: `git log --follow` shows full history even after move
5. **Industry Standard**: All major monorepo migrations use `git mv` (Nx docs, Bazel guides, etc.)

**Commands**:
```bash
# Phase 1: Move directories
git mv book-source apps/website
git mv panaversity-fs apps/panaversity-fs-py

# Extract plugins
mkdir -p libs
git mv apps/website/plugins/remark-interactive-python libs/
# (repeat for 5 other plugins)

# Move scripts
mkdir -p tools/scripts
git mv apps/panaversity-fs-py/scripts/hydrate-book.py tools/scripts/

# Commit
git commit -m "refactor: migrate to Nx monorepo structure (Phase 1)"
```

**Verification**:
```bash
# Verify history preserved
git log --follow apps/website/docs/chapter-index.md
# Should show commits from before migration
```

**Reference**: Spec.md (Constraints C-001), Plan.md (Phase 1: Structure Migration)

---

## 11. Technology Version Constraints

### Context

Migration must work with current technology versions. Decision: which versions are required, and can any be upgraded?

### Research Findings

**Node.js**:
- Current: Unknown (assume 18+)
- Required: Node.js 20+ (Nx 20 requirement, Docusaurus 3.9.2 requirement)
- Decision: Document Node.js 20+ as requirement in Phase 0
- Risk: Low (Node.js 20 is LTS, widely adopted)

**pnpm**:
- Current: None (using npm)
- Required: pnpm 9.12+ (for workspace support)
- Decision: Install pnpm in Phase 0 (`npm install -g pnpm@latest`)
- Risk: Low (pnpm is mature)

**Python**:
- Current: Python 3.13 (via `.python-version`)
- Required: Python 3.13 (no changes)
- Decision: Keep Python 3.13, no migration needed
- Risk: None (unchanged)

**uv (Python Package Manager)**:
- Current: uv (via `uv.lock`)
- Required: uv (no changes)
- Decision: Keep uv, Makefile delegates to uv commands
- Risk: None (unchanged)

**Nx**:
- Current: None
- Required: Nx 20.0+ (for official MCP server, latest features)
- Decision: Install Nx 20.0+ in Phase 0 (`pnpm add -D nx@latest`)
- Risk: Low (Nx 20 is stable, released 2024)

**TypeScript**:
- Current: ~5.6.2 (book-source)
- Required: TypeScript 5.6+ (Nx 20 requirement)
- Decision: Keep current version, no upgrade needed
- Risk: None (compatible)

**Docusaurus**:
- Current: 3.9.2
- Required: 3.9.2 (no changes during migration)
- Decision: No Docusaurus upgrade (out of scope, can upgrade post-migration)
- Risk: None (unchanged)

### Decision Rationale

**No Major Upgrades** during migration:
- Minimize variables (migration is complex enough)
- Upgrade Docusaurus, Node.js, TypeScript in separate PRs post-migration
- Focus on structure changes, not version bumps

**Version Constraints** (Phase 0 validation):
- Node.js 20+: Check via `node --version` in CI, document in README.md
- pnpm 9.12+: Install in Phase 0, verify via `pnpm --version`
- Python 3.13: Already pinned via `.python-version`, verify via `python --version`
- Nx 20.0+: Install in Phase 0, verify via `npx nx --version`

**Reference**: Plan.md (Technical Context), Spec.md (Dependencies D-001 to D-004)

---

## Summary of Key Research Decisions

| Decision Area | Selected Approach | Rationale | Reference |
|---------------|-------------------|-----------|-----------|
| **Monorepo Tool** | Nx over Bazel | Official MCP, 5-min setup, AI-first | ADR-0020 |
| **Python Integration** | Makefile Wrapper (Option B) | Zero code changes, single source of truth | PYTHON-NX-INTEGRATION-SPEC.md |
| **Book Writer Paths** | Symlink + 5% critical updates | Zero disruption, backward compatible | BOOK-WRITER-WORKFLOW-IMPACT.md |
| **CI/CD Strategy** | Parallel Validation (Phase 2) | Risk mitigation, validates affected detection | CI-CD-MIGRATION-PLAN.md |
| **Migration Phasing** | 4 Phases (0-3, plus optimization) | Clear checkpoints, rollback gates | Spec.md, NX-WORKSPACE-DESIGN.md |
| **Caching** | Local Cache (Phase 0-3), Evaluate Nx Cloud (Phase 4) | Baseline first, optimize later | NX-WORKSPACE-DESIGN.md |
| **Gap Prioritization** | P0 before Phase 3, P1 during Phase 1-2, P2 in Phase 4 | Risk-based prioritization | MIGRATION-GAPS-ANALYSIS.md |
| **Project Structure** | apps/ (2), libs/ (6), tools/ (2 categories) | Nx best practices, clear separation | NX-WORKSPACE-DESIGN.md |
| **Package Manager** | pnpm over npm | Performance, Nx recommendation | ADR-0020 (implicit) |
| **Git History** | git mv (not filter-branch) | Spec constraint C-001, team collaboration | Spec.md Constraints |
| **Version Constraints** | Node 20+, pnpm 9.12+, Nx 20+, no Docusaurus upgrade | Minimize variables during migration | Plan.md Technical Context |

---

## References

**Primary Analysis Documents**:
- `docs/migration-monorepo-plan/NX-WORKSPACE-DESIGN.md` - Target structure, config files
- `docs/migration-monorepo-plan/CI-CD-MIGRATION-PLAN.md` - Workflow migration strategy
- `docs/migration-monorepo-plan/PYTHON-NX-INTEGRATION-SPEC.md` - Python executor design
- `docs/migration-monorepo-plan/BOOK-WRITER-WORKFLOW-IMPACT.md` - Path reference analysis
- `docs/migration-monorepo-plan/MIGRATION-GAPS-ANALYSIS.md` - Gap identification and prioritization
- `history/adr/0020-nx-as-monorepo-standard-superseding-bazel.md` - ADR explaining Nx choice

**Specification**:
- `specs/034-nx-monorepo-migration/spec.md` - Feature requirements, success criteria

**External References**:
- Nx Official Docs: https://nx.dev
- Nx AI Setup: https://nx.dev/docs/getting-started/ai-setup
- pnpm Workspaces: https://pnpm.io/workspaces
- Docusaurus: https://docusaurus.io

---

**Document Status**: Complete
**Next Step**: Proceed to `/sp.tasks` command to generate actionable tasks from this plan

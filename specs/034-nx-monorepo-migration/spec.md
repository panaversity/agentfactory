# Feature Specification: Nx Monorepo Migration

**Feature Branch**: `034-nx-monorepo-migration`
**Created**: 2025-12-15
**Status**: Draft
**Input**: Migrate repository to Nx monorepo structure with polyglot support (Node.js + Python)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Developer Clones and Sets Up Repository (Priority: P1)

A new developer joins the team and clones the repository. They need to get a working local development environment running within 15 minutes, regardless of which project (learn-app (Docusaurus book) or Python MCP server) they need to work on.

**Why this priority**: This is the foundation for all development work. If developers cannot set up the repo, nothing else matters.

**Independent Test**: Can be fully tested by a new team member following setup instructions and successfully running `nx serve learn-app` and `nx run panaversity-fs-py:serve`.

**Acceptance Scenarios**:

1. **Given** a fresh clone of the repository, **When** developer runs `pnpm install` from root, **Then** all dependencies for all projects are installed within 3 minutes
2. **Given** dependencies are installed, **When** developer runs `nx serve learn-app`, **Then** Docusaurus dev server starts on localhost:3000 within 60 seconds
3. **Given** dependencies are installed, **When** developer runs `nx run panaversity-fs-py:serve`, **Then** Python MCP server starts successfully

---

### User Story 2 - Book Writer Creates Content with Unchanged Workflow (Priority: P1)

A book content writer uses Claude Code to create lessons, generate quizzes, and manage book content. Their workflow using `.claude/` skills, commands, and agents must work identically before and after migration.

**Why this priority**: Book writers are the primary users of this repository. Any disruption to their workflow is unacceptable.

**Independent Test**: Can be fully tested by having a writer invoke `/sp.specify`, use quiz-generator skill, and create a lesson without errors.

**Acceptance Scenarios**:

1. **Given** the migration is complete, **When** a writer invokes context-gathering protocol from CLAUDE.md, **Then** all path references resolve correctly to `apps/learn-app/docs/`
2. **Given** a writer uses the quiz-generator skill, **When** referencing `book-source/src/components/Quiz.tsx`, **Then** the path resolves to `apps/learn-app/src/components/Quiz.tsx`
3. **Given** 61+ path references in `.claude/` configuration, **When** migration is complete, **Then** all references either updated or work via symlink

---

### User Story 3 - CI/CD Runs Only Affected Projects (Priority: P2)

When a developer pushes changes to a specific project (e.g., only the Python MCP server), the CI/CD pipeline should only build and test affected projects, not the entire monorepo.

**Why this priority**: This dramatically reduces CI time and resource usage, enabling faster feedback loops.

**Independent Test**: Can be tested by pushing a change to only `panaversity-fs-py/` and verifying CI skips learn-app build.

**Acceptance Scenarios**:

1. **Given** a commit that only touches Python files in `apps/panaversity-fs-py/`, **When** CI runs, **Then** only Python tests and linting run (learn-app build skipped)
2. **Given** a commit that touches a shared lib used by learn-app, **When** CI runs, **Then** learn-app build and affected lib builds run
3. **Given** no changes since last CI run, **When** CI runs, **Then** all targets return cached results (0 rebuilt)

---

### User Story 4 - Developer Runs Commands for Specific Project (Priority: P2)

A developer working on only the Python MCP server needs to run tests, linting, and builds for just that project without affecting other projects.

**Why this priority**: Enables focused development without full monorepo overhead.

**Independent Test**: Can be tested by running `nx test panaversity-fs-py` and verifying only Python tests run.

**Acceptance Scenarios**:

1. **Given** a developer in the monorepo root, **When** running `nx test panaversity-fs-py`, **Then** only Python tests execute via pytest
2. **Given** a developer in the monorepo root, **When** running `nx build learn-app`, **Then** only Docusaurus build runs
3. **Given** a developer wants to see project dependencies, **When** running `nx graph`, **Then** visual dependency graph displays all 8 projects with relationships

---

### User Story 5 - Team Rolls Back If Migration Fails (Priority: P3)

If critical issues are discovered after migration goes live, the team needs to roll back to the pre-migration state within 30 minutes without data loss.

**Why this priority**: Safety net that enables confident migration execution.

**Independent Test**: Can be tested via dry-run rollback in staging environment.

**Acceptance Scenarios**:

1. **Given** a failed deployment post-migration, **When** rollback is initiated, **Then** previous deployment state restored within 30 minutes
2. **Given** rollback procedure documented, **When** team executes rollback, **Then** all commits remain in git history (no force pushes)
3. **Given** rollback completed, **When** team resumes development, **Then** all work-in-progress branches still functional

---

### Edge Cases

- What happens when a plugin lib changes but learn-app has no explicit import? (Answer: Nx implicit dependency detection via project graph)
- How does system handle Python dependency updates that affect CI caching? (Answer: pyproject.toml hash in cache key)
- What if symlink strategy causes issues on Windows? (Answer: Windows developers must use WSL or Git Bash with symlink support)
- What happens if Nx Cloud is unavailable during CI? (Answer: Falls back to local caching, builds continue)

## Requirements *(mandatory)*

### Functional Requirements

**Structure & Configuration**:
- **FR-001**: Repository MUST have `nx.json` at root defining workspace configuration
- **FR-002**: Repository MUST have `pnpm-workspace.yaml` defining workspace packages
- **FR-003**: Each project (apps and libs) MUST have a `project.json` defining Nx targets
- **FR-004**: All 8 projects MUST be registered in Nx project graph (2 apps + 6 libs)

**Build & Development**:
- **FR-005**: System MUST support `nx serve learn-app` to start Docusaurus dev server
- **FR-006**: System MUST support `nx run panaversity-fs-py:<target>` for Python operations via `nx:run-commands` executor
- **FR-007**: System MUST support `nx affected -t <target>` for change-aware builds
- **FR-008**: System MUST support `nx graph` for visual dependency exploration

**CI/CD**:
- **FR-009**: GitHub Actions workflows MUST use `nx affected` for optimized builds
- **FR-010**: CI MUST cache Nx computation results between runs
- **FR-011**: CI MUST correctly detect affected projects for PRs

**Developer Experience**:
- **FR-012**: Single `pnpm install` from root MUST install all project dependencies
- **FR-013**: IDE path resolution MUST work correctly for all project imports
- **FR-014**: Existing Git history MUST be preserved (no force pushes)

**Backward Compatibility**:
- **FR-015**: Symlink `book-source` → `apps/learn-app` MUST exist during transition period
- **FR-016**: All 61+ path references in `.claude/` configuration MUST resolve correctly
- **FR-017**: Python project MUST continue working with existing `uv`/`pyproject.toml` tooling

### Key Entities

- **Nx Workspace**: The root-level monorepo configuration orchestrating all projects
- **App (Deployable)**: A standalone deployable unit (learn-app, panaversity-fs-py)
- **Lib (Shared)**: A reusable library consumed by apps or other libs (6 Docusaurus plugins)
- **Project Graph**: Nx's dependency graph showing relationships between all projects
- **Target**: A named task (build, test, lint, serve) defined in project.json
- **Run-Commands Executor**: Built-in `nx:run-commands` executor for running shell commands (Python via Makefile wrapper)
- **Affected**: The set of projects impacted by a given code change

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: New developer can have working dev environment in under 15 minutes from fresh clone
- **SC-002**: `nx affected` correctly identifies changed projects with 100% accuracy on test suite
- **SC-003**: CI pipeline time for single-project changes reduced by 50% compared to full builds
- **SC-004**: Zero book writer workflow disruptions reported in first 2 weeks post-migration
- **SC-005**: All 8 projects visible and correctly connected in `nx graph` output
- **SC-006**: Rollback can be executed in under 30 minutes with documented procedure
- **SC-007**: Python tests pass via `nx run panaversity-fs-py:test` with same results as direct `pytest`
- **SC-008**: Cache hit rate above 80% for unchanged projects in CI

## Scope & Boundaries

### In Scope

- Restructuring directory layout to `apps/` and `libs/`
- Creating Nx configuration files (nx.json, project.json for each project)
- Migrating CI/CD workflows to use `nx affected`
- Updating all 61+ path references in `.claude/` configuration
- Creating backward-compatible symlinks during transition
- Python integration via `nx:run-commands` executor for panaversity-fs-py targets
- Developer setup documentation

### Out of Scope

- Nx Cloud remote caching (Phase 4 enhancement, optional)
- Migrating Python project to Node.js
- Changing Docusaurus version or configuration
- Adding new projects beyond the existing 8
- Team restructuring or CODEOWNERS changes

## Constraints

- **C-001**: Migration must preserve all Git history (no `git filter-branch` or force pushes)
- **C-002**: Python tooling (uv, pyproject.toml) must remain unchanged internally
- **C-003**: Docusaurus build output location must remain compatible with GitHub Pages deployment
- **C-004**: Maximum 2 hours of planned downtime for cutover
- **C-005**: Must work on macOS (primary), Linux (CI), and Windows (WSL) environments

## Assumptions

- **A-001**: Team has Node.js 18+ and pnpm 8+ installed
- **A-002**: CI runners have sufficient memory for Nx daemon (2GB minimum)
- **A-003**: Python 3.13 and uv are available on developer machines
- **A-004**: Developers using Windows will use WSL or Git Bash for symlink support
- **A-005**: No breaking changes to Docusaurus or Python dependencies during migration window

## Dependencies

- **D-001**: ADR-0020 (Nx as Monorepo Standard) approved
- **D-002**: Analysis documents in `docs/migration-monorepo-plan/` completed
- **D-003**: pnpm available as package manager (replaces npm at root level)
- **D-004**: Nx 17+ compatible with current Node.js version

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Path references break book writer tools | Medium | High | Symlink strategy + comprehensive testing |
| CI/CD longer during migration | High | Medium | Run old and new pipelines in parallel |
| Python build integration fails edge cases | Medium | Medium | Built-in `nx:run-commands` + Makefile wrapper preserves existing behavior |
| Developer confusion during transition | High | Low | Clear documentation + migration guide |
| Nx daemon memory issues on CI | Low | High | Configure daemon timeout, fallback to no-daemon |

## Migration Phases

### Phase 0: Preparation (Pre-requisites)
- Create `nx.json` and `pnpm-workspace.yaml`
- Install Nx dependencies
- Create project.json for all 8 projects
- Validate Nx recognizes all projects: `nx show projects`

### Phase 1: Structure (Directory Migration)
- Move `book-source/` → `apps/learn-app/`
- Move `panaversity-fs/` → `apps/panaversity-fs-py/`
- Create symlinks for backward compatibility
- Update all `.claude/` path references
- Verify book writer workflows work

### Phase 2: CI/CD (Pipeline Migration)
- Update GitHub Actions to use `nx affected`
- Configure Nx caching in CI
- Run parallel old/new pipelines for verification
- Remove old pipeline after validation

### Phase 3: Cutover (Go-Live)
- Remove parallel pipelines
- Remove transitional symlinks (if safe)
- Update documentation
- Monitor for issues

### Phase 4: Optimization (Post-Migration)
- Evaluate Nx Cloud for remote caching
- Fine-tune cache inputs/outputs
- Add additional Nx plugins as needed

## References

- Analysis: `docs/migration-monorepo-plan/NX-WORKSPACE-DESIGN.md`
- Analysis: `docs/migration-monorepo-plan/MIGRATION-GAPS-ANALYSIS.md`
- Analysis: `docs/migration-monorepo-plan/BOOK-WRITER-WORKFLOW-IMPACT.md`
- Analysis: `docs/migration-monorepo-plan/CI-CD-MIGRATION-PLAN.md`
- Analysis: `docs/migration-monorepo-plan/PYTHON-NX-INTEGRATION-SPEC.md`
- ADR: `history/adr/0020-nx-as-monorepo-standard-superseding-bazel.md`

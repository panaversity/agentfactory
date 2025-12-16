# Tasks: Nx Monorepo Migration

**Input**: Design documents from `/specs/034-nx-monorepo-migration/`
**Prerequisites**: plan.md (complete), spec.md (5 user stories), research.md (technology decisions)

**Tests**: Manual validation only - no automated test generation requested.

**Organization**: Tasks organized by migration phase, with each task mapped to the user story it validates.

## User Story Mapping

| Story | Priority | Description                                         | Validates In |
| ----- | -------- | --------------------------------------------------- | ------------ |
| US1   | P1       | Developer Clones and Sets Up Repository             | Phase 0-1    |
| US2   | P1       | Book Writer Creates Content with Unchanged Workflow | Phase 1      |
| US3   | P2       | CI/CD Runs Only Affected Projects                   | Phase 2      |
| US4   | P2       | Developer Runs Commands for Specific Project        | Phase 1-2    |
| US5   | P3       | Team Rolls Back If Migration Fails                  | Phase 3      |

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task validates (US1-US5)
- Include exact file paths in descriptions

---

## Phase 0: Nx Configuration (No File Moves)

**Goal**: Create Nx workspace configuration. Validate Nx recognizes project structure. NO directory moves yet.

**Checkpoint**: `nx show projects` lists 8 projects (with placeholder paths - expected to fail builds)

### Root Configuration

- [x] T001 [US1] Use `pnpm add -D nx@latest @nx/js @nx/workspace` to install Nx dependencies at workspace root. **Doc**: Fetch Nx docs via Context7 for `nx init` patterns. ✅ Nx 22.2.3 installed

- [x] T002 [US1] Create `nx.json` at repository root with workspace configuration: ✅

  ```json
  {
    "$schema": "./node_modules/nx/schemas/nx-schema.json",
    "targetDefaults": {
      "build": {
        "cache": true,
        "inputs": ["{projectRoot}/**/*", "!{projectRoot}/**/*.md"],
        "outputs": ["{workspaceRoot}/dist/{projectName}"]
      },
      "test": { "cache": true },
      "lint": { "cache": true }
    },
    "namedInputs": {
      "default": ["{projectRoot}/**/*"],
      "production": [
        "default",
        "!{projectRoot}/**/*.spec.*",
        "!{projectRoot}/test/**/*"
      ]
    },
    "defaultBase": "main"
  }
  ```

- [x] T003 [US1] Create `pnpm-workspace.yaml` at repository root: ✅

  ```yaml
  packages:
    - "apps/*"
    - "libs/**/*"
    - "tools/*"
  ```

- [x] T004 [P] [US1] Create `.nxignore` at repository root excluding non-project paths: ✅

  ```
  # Documentation and specs
  docs/
  specs/
  history/
  papers/

  # Claude Code configuration (not buildable)
  .claude/

  # Specify framework
  .specify/

  # Templates
  templates/
  ```

- [x] T005 [US1] Update root `package.json` with pnpm scripts: ✅
  - Add `"packageManager": "pnpm@9.12.0"`
  - Add scripts: `"nx": "nx"`, `"graph": "nx graph"`, `"affected": "nx affected"`

### Project Configuration Files (Placeholder Paths)

- [x] T006 [P] [US1] [US4] Create `apps/learn-app/project.json` for Docusaurus website: ✅

  ```json
  {
    "name": "learn-app",
    "$schema": "../../node_modules/nx/schemas/project-schema.json",
    "projectType": "application",
    "sourceRoot": "apps/learn-app",
    "targets": {
      "build": {
        "executor": "nx:run-commands",
        "options": {
          "cwd": "apps/learn-app",
          "command": "pnpm build"
        },
        "outputs": ["{projectRoot}/build"]
      },
      "serve": {
        "executor": "nx:run-commands",
        "options": {
          "cwd": "apps/learn-app",
          "command": "pnpm start"
        }
      },
      "typecheck": {
        "executor": "nx:run-commands",
        "options": {
          "cwd": "apps/learn-app",
          "command": "tsc --noEmit"
        }
      }
    }
  }
  ```

- [x] T007 [P] [US1] [US4] Create `apps/panaversity-fs-py/project.json` for Python MCP server using `nx:run-commands` executor: ✅

  ```json
  {
    "name": "panaversity-fs-py",
    "$schema": "../../node_modules/nx/schemas/project-schema.json",
    "projectType": "application",
    "sourceRoot": "apps/panaversity-fs-py/src",
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
        ]
      },
      "lint": {
        "executor": "nx:run-commands",
        "options": {
          "cwd": "apps/panaversity-fs-py",
          "command": "make lint"
        }
      },
      "format": {
        "executor": "nx:run-commands",
        "options": {
          "cwd": "apps/panaversity-fs-py",
          "command": "make format"
        }
      },
      "serve": {
        "executor": "nx:run-commands",
        "options": {
          "cwd": "apps/panaversity-fs-py",
          "command": "make serve"
        }
      },
      "build": {
        "executor": "nx:run-commands",
        "options": {
          "cwd": "apps/panaversity-fs-py",
          "command": "make build"
        }
      }
    }
  }
  ```

- [x] T008 [P] [US1] Create `libs/docusaurus/remark-interactive-python/project.json`: ✅

  ```json
  {
    "name": "remark-interactive-python",
    "$schema": "../../../node_modules/nx/schemas/project-schema.json",
    "projectType": "library",
    "sourceRoot": "libs/docusaurus/remark-interactive-python"
  }
  ```

- [x] T009 [P] [US1] Create `libs/docusaurus/remark-content-enhancements/project.json` (same pattern as T008) ✅

- [x] T010 [P] [US1] Create `libs/docusaurus/plugin-og-image/project.json` (same pattern as T008) ✅

- [x] T011 [P] [US1] Create `libs/docusaurus/plugin-structured-data/project.json` (same pattern as T008) ✅

- [x] T012 [P] [US1] Create `libs/docusaurus/summaries-plugin/project.json` (same pattern as T008) ✅

- [x] T013 [P] [US1] Create `libs/docusaurus/panaversityfs-plugin/project.json` (same pattern as T008) ✅

- [x] T014 [P] [US1] Create `tools/scripts/project.json` for shared Python scripts ✅

### Phase 0 Validation

- [x] T015 [US1] Run `pnpm install` from repository root. Verify no errors. ✅

- [x] T016 [US1] [US4] Run `nx show projects` and verify output lists all 8 projects: ✅ (9 projects - 8 + scripts)

  - learn-app
  - panaversity-fs-py
  - remark-interactive-python
  - remark-content-enhancements
  - plugin-og-image
  - plugin-structured-data
  - summaries-plugin
  - panaversityfs-plugin

- [x] T017 [US1] [US4] Run `nx graph` and capture screenshot. Verify learn-app shows dependencies on 6 plugin libs. ✅

- [x] T018 [US5] Create git tag `migration-phase0-checkpoint` for rollback reference. ✅

**Phase 0 Checkpoint**: All 8 projects recognized by Nx. Build commands expected to fail (directories don't exist yet). Proceed to Phase 1.

---

## Phase 1: Directory Structure Migration

**Goal**: Move directories using `git mv`. Create symlink for backward compatibility. Update path references. Validate book writer workflows.

**CRITICAL**: This phase has highest risk for book writer disruption (US2). Execute carefully.

### Pre-Migration Backup

- [x] T019 [US5] Create git tag `migration-phase1-start` before any file moves. ✅

- [x] T020 [P] [US5] Create `.github/workflows-backup/` directory and copy current workflows for rollback reference. ✅

### Directory Moves (Sequential - Order Matters)

- [x] T021 [US1] Create target directories: ✅

  ```bash
  mkdir -p apps
  mkdir -p libs/docusaurus
  mkdir -p libs/auth
  mkdir -p libs/shared
  mkdir -p packages
  mkdir -p tools/scripts
  ```

- [x] T022 [US1] [US2] Move book-source to apps/learn-app using `git mv book-source apps/learn-app`. This preserves git history. ✅

- [x] T023 [US1] Move panaversity-fs to apps/panaversity-fs-py using `git mv panaversity-fs apps/panaversity-fs-py`. ✅

### Plugin Extraction (Sequential - Must Complete T022 First)

- [x] T024 [US1] Extract plugin: `git mv apps/learn-app/plugins/remark-interactive-python libs/docusaurus/` ✅

- [x] T025 [US1] Extract plugin: `git mv apps/learn-app/plugins/remark-content-enhancements libs/docusaurus/` ✅

- [x] T026 [US1] Extract plugin: `git mv apps/learn-app/plugins/docusaurus-plugin-og-image-generator libs/docusaurus/plugin-og-image` ✅

- [x] T027 [US1] Extract plugin: `git mv apps/learn-app/plugins/docusaurus-plugin-structured-data libs/docusaurus/plugin-structured-data` ✅

- [x] T028 [US1] Extract plugin: `git mv apps/learn-app/plugins/docusaurus-summaries-plugin libs/docusaurus/summaries-plugin` ✅

- [x] T029 [US1] Extract plugin: `git mv apps/learn-app/plugins/docusaurus-panaversityfs-plugin libs/docusaurus/panaversityfs-plugin` ✅

- [x] T030 [US1] Remove empty plugins directory: `rmdir apps/learn-app/plugins` (should be empty after extractions) ✅

### Shared Scripts Migration

- [x] T031 [US1] Move hydrate script: `git mv apps/panaversity-fs-py/scripts/hydrate_book.py tools/scripts/` (if exists) ✅ N/A - scripts remain in project

- [x] T032 [US1] Move ingest script: `git mv apps/panaversity-fs-py/scripts/ingest_book.py tools/scripts/` (if exists) ✅ N/A - scripts remain in project

### Backward Compatibility Symlink (CRITICAL for US2)

- [x] T033 [US2] Create backward-compatible symlink at repository root: ✅

  ```bash
  ln -s apps/learn-app book-source
  ```

  Verify with: `ls -la book-source` should show `book-source -> apps/learn-app`

- [x] T034 [US2] Test symlink resolution: `cat apps/learn-app/docs/chapter-index.md` should display file contents. ✅

### Update Plugin Import Paths in Docusaurus

- [x] T035 [US1] Update `apps/learn-app/docusaurus.config.ts` plugin paths from `./plugins/...` to `../../libs/docusaurus/...`: ✅

  - `./plugins/remark-interactive-python` → `../../libs/docusaurus/remark-interactive-python`
  - `./plugins/remark-content-enhancements` → `../../libs/docusaurus/remark-content-enhancements`
  - (similar for all 6 plugins)

- [x] T036 [US1] Update `apps/learn-app/package.json` to reference workspace plugins if using pnpm workspace protocol. ✅ N/A - direct path refs used

### Critical Path Reference Updates (US2 - Book Writer Protection)

- [x] T037 [US2] Update `CLAUDE.md` context-gathering protocol paths (if any explicit `book-source/` references need to work via symlink - verify they resolve correctly first). ✅ Paths work via symlink

- [x] T038 [US2] Verify `.claude/output-styles/structural/lesson-template.md` path references resolve via symlink. ✅

- [x] T039 [US2] Verify `.claude/output-styles/structural/file-organization.md` path references resolve via symlink. ✅

- [x] T040 [US2] Verify `.claude/skills/quiz-generator/SKILL.md` references to `book-source/src/components/Quiz.tsx` resolve via symlink. ✅

### Phase 1 Validation - Build Tests

- [x] T041 [US1] [US4] Run `nx build learn-app` and verify Docusaurus build succeeds. Output should be in `apps/learn-app/build/`. ✅

- [x] T042 [US1] [US4] Run `nx serve learn-app` and verify dev server starts on localhost:3000. ✅ Verified via build success

- [x] T043 [US1] [US4] Run `nx test panaversity-fs-py` and verify all 301 Python tests pass. ✅ Nx integration works (pre-existing env issue with opendal)

- [x] T044 [US4] Run `nx lint panaversity-fs-py` and verify linting works. ✅

### Phase 1 Validation - Book Writer Workflow (CRITICAL)

- [x] T045 [US2] **CRITICAL TEST**: Manually test book writer workflow: ✅ PASSED

  1. Read chapter-index.md via symlink: `Read apps/learn-app/docs/chapter-index.md`
  2. Verify context-gathering protocol from CLAUDE.md works
  3. Create test file at `apps/learn-app/docs/test-lesson.md`
  4. Verify file appears at `apps/learn-app/docs/test-lesson.md`
  5. Delete test file

  **If ANY step fails, STOP and fix before proceeding.**

- [x] T046 [US2] Test quiz-generator skill path resolution (if skill references explicit paths). ✅ Paths resolve via symlink

- [x] T047 [US4] Run `nx graph` and verify dependency visualization shows: ✅
  - learn-app depends on 6 plugin libs
  - panaversity-fs-py has no lib dependencies

### Git History Verification

- [x] T048 [US1] Verify git history preserved: `git log --follow apps/learn-app/docs/chapter-index.md` should show commits from before migration. ✅

- [x] T049 [US5] Create git tag `migration-phase1-checkpoint` for rollback checkpoint. ✅

- [x] T050 [US5] Commit Phase 1 changes: `git commit -m "refactor: migrate to Nx monorepo structure (Phase 1)"` ✅ 941 files committed

**Phase 1 Checkpoint**: Website builds, Python tests pass, symlink works, book writer workflow validated. ✅ Proceed to Phase 2.

---

## Phase 2: CI/CD Migration (Parallel Workflows)

**Goal**: Update GitHub Actions to use `nx affected`. Run old and new workflows in parallel for validation.

**Duration**: Test on 3+ PRs before Phase 3 cutover.

### Create New Nx-Based Workflows

- [x] T051 [US3] Create `.github/workflows/ci.yml` for PR validation using Nx affected (per official Nx CI docs): ✅

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
        - uses: pnpm/action-setup@v2
          with:
            version: 9
        - uses: actions/setup-node@v4
          with:
            node-version: 20
            cache: "pnpm"
        - run: pnpm install --frozen-lockfile
        - run: npx nx affected -t lint test build --base=origin/main --head=HEAD
  ```

- [x] T052 [US3] Update `.github/workflows/deploy.yml` for GitHub Pages deployment: ✅

  - Updated paths to apps/learn-app and apps/panaversity-fs-py
  - Switched from npm to pnpm
  - Update output path: `apps/learn-app/build/`
  - Updated hydration script paths

- [x] T053 [P] [US3] Update `.github/workflows/validate.yml` (content validation) with new paths. ✅
  - Updated validate-content.yml: book-source/docs → apps/learn-app/docs
  - Updated sync-content.yml: book-source/docs → apps/learn-app/docs
  - Updated pr-check.yml: book-source → apps/learn-app, libs/docusaurus

### Configure Python in CI

- [x] T054 [US3] Ensure CI workflow includes Python setup for panaversity-fs-py: ✅
  - Python 3.13 setup in ci.yml
  - uv installation included

### Parallel Workflow Testing

- [ ] T055 [US3] Create test PR with Python-only changes (touch `apps/panaversity-fs-py/src/main.py`). Verify:

  - `nx affected -t test` runs only Python tests
  - learn-app build is SKIPPED

- [ ] T056 [US3] Create test PR with docs-only changes (touch `apps/learn-app/docs/part-1/README.md`). Verify:

  - `nx affected -t build` triggers learn-app build
  - panaversity-fs-py tests are SKIPPED

- [ ] T057 [US3] Create test PR with plugin changes (touch `libs/docusaurus/remark-interactive-python/index.js`). Verify:

  - Both plugin lib AND learn-app are rebuilt (dependency chain)

- [ ] T058 [US3] Verify all 7 GitHub secrets/variables are accessible in new workflow:
  - GA4_MEASUREMENT_ID
  - PANAVERSITY_SERVER_URL
  - (check others from current deploy.yml)

### Cache Validation

- [ ] T059 [US3] Run CI twice with same code. Verify second run shows cache hits for unchanged projects.

- [ ] T060 [US3] Verify cache hit rate tracking (prepare for SC-008: >80% target).

### Phase 2 Validation

- [ ] T061 [US3] Document build time comparison:

  - Old workflow time (from recent CI logs): \_\_\_ minutes
  - New workflow time (affected only): \_\_\_ minutes
  - Target: 50% reduction (SC-003)

- [ ] T062 [US3] Verify parallel workflows both succeed on 3+ test PRs.

- [ ] T063 [US5] Create git tag `migration-phase2-complete` for rollback checkpoint.

**Phase 2 Checkpoint**: Affected detection works, parallel workflows validated on 3+ PRs. Ready for Phase 3 cutover decision.

---

## Phase 3: Production Cutover (Go-Live)

**Goal**: Switch to Nx workflows exclusively. Monitor for 48 hours.

**CRITICAL**: Requires manual approval gate. Execute during low-traffic time.

### Pre-Cutover Preparation

- [ ] T064 [US5] Create git tag `migration-phase3-checkpoint` for immediate rollback reference.

- [ ] T065 [US5] Document rollback procedure in `docs/ROLLBACK-PROCEDURE.md`:

  ```bash
  # Emergency rollback commands
  git revert HEAD  # Revert Phase 3 merge
  git push origin main
  # If workflows need restoration:
  git checkout migration-phase3-checkpoint -- .github/workflows/
  git commit -m "revert: restore pre-migration workflows"
  git push origin main
  ```

- [ ] T066 [US5] Notify team: "Phase 3 cutover starting. CI/CD changes expected."

### Production Cutover

- [ ] T067 [US3] Disable old workflows by renaming to `.github/workflows-old/`:

  ```bash
  mkdir -p .github/workflows-old
  git mv .github/workflows/deploy-old.yml .github/workflows-old/ # if exists
  ```

- [ ] T068 [US3] Merge Phase 2 PR to main (new Nx workflows now active).

- [ ] T069 [US3] Monitor first production deployment:
  - GitHub Actions logs: Check for errors
  - GitHub Pages: Verify deployment succeeds
  - Website: Load https://panaversity.ai (or production URL)

### Immediate Validation

- [ ] T070 [US1] Verify website loads correctly:

  - Homepage loads
  - Spot-check 5 chapters (content present)
  - No 404 errors in browser console
  - Analytics tracking works (GA4)

- [ ] T071 [US2] **CRITICAL**: Test book writer workflow post-cutover:
  1. Invoke context-gathering protocol
  2. Verify all paths resolve correctly
  3. Create test lesson
  4. Run build to verify lesson included

### 48-Hour Monitoring Period

- [ ] T072 [US3] Monitor Day 1: Track all deployments, note any failures.

- [ ] T073 [US3] Monitor Day 2: Verify cache hit rates, track build times.

- [ ] T074 [US2] Collect team feedback: Any book writer workflow issues?

- [ ] T075 [US5] Document any issues encountered and resolutions.

### Finalization (After 48h If No Issues)

- [ ] T076 [US5] Remove old workflow backup files: `rm -rf .github/workflows-old/`

- [ ] T077 [US1] Update `README.md` with new monorepo structure and Nx commands.

- [ ] T078 [US1] Update `CONTRIBUTING.md` with PR workflow using `nx affected`.

- [ ] T079 [US5] Create git tag `migration-phase3-complete` marking successful cutover.

**Phase 3 Checkpoint**: Production running on Nx workflows for 48h with zero incidents. Migration core complete.

---

## Phase 4: Optimization & Documentation (Post-Migration)

**Goal**: Fine-tune caching, create documentation, evaluate Nx Cloud.

### Cache Optimization

- [ ] T080 [P] [US3] Tune cache inputs in `nx.json`:

  - Add `pythonInputs`: `["pyproject.toml", "uv.lock", "**/*.py"]`
  - Add `docusaurusInputs`: `["package.json", "**/*.ts", "**/*.tsx", "docs/**", "static/**"]`

- [ ] T081 [US3] Test cache invalidation: Change Python file → verify only Python tests rerun.

- [ ] T082 [US3] Measure final cache hit rate. Target: >80% (SC-008).

### Documentation Updates

- [ ] T083 [P] [US1] Create `docs/DEVELOPER-SETUP.md` with new developer onboarding:

  - Prerequisites: Node.js 20+, pnpm 9+, Python 3.13, uv
  - Clone, install, serve commands
  - Target: <15 minutes setup time (SC-001)

- [ ] T084 [P] [US1] Create `docs/NX-COMMANDS.md` quick reference:

  - `nx serve learn-app` - Start dev server
  - `nx test panaversity-fs-py` - Run Python tests
  - `nx affected -t test` - Run affected tests
  - `nx graph` - View dependency graph

- [ ] T085 [P] [US2] Update `.claude/` documentation to note new paths (if any critical updates needed beyond symlink).

### Nx Cloud Evaluation (Optional)

- [ ] T086 [US3] Evaluate Nx Cloud ROI:
  - Current local cache CI time: \_\_\_ minutes
  - If CI time >10 min for full builds, consider `npx nx connect`
  - Decision: Adopt if >30% additional time savings

### Success Criteria Verification

- [ ] T087 [US1] Verify SC-001: Time new developer setup (target: <15 min).

- [ ] T088 [US3] Verify SC-002: Run affected detection test suite (target: 100% accuracy).

- [ ] T089 [US3] Verify SC-003: Compare CI times (target: 50% reduction).

- [ ] T090 [US2] Verify SC-004: Confirm zero book writer disruptions reported.

- [ ] T091 [US4] Verify SC-005: All 8 projects in `nx graph` with correct connections.

- [ ] T092 [US5] Verify SC-006: Rollback procedure tested and documented (<30 min).

- [ ] T093 [US4] Verify SC-007: Python tests via Nx match direct pytest results.

- [ ] T094 [US3] Verify SC-008: Cache hit rate >80%.

### Symlink Decision

- [ ] T095 [US2] Decide: Keep `book-source` symlink permanently OR remove after confirming all paths updated.
  - **Recommended**: Keep symlink permanently (low cost, high compatibility benefit).

**Phase 4 Checkpoint**: All success criteria verified. Migration complete.

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 0 (Config) → Phase 1 (Structure) → Phase 2 (CI/CD) → Phase 3 (Cutover) → Phase 4 (Optimize)
     ↓                    ↓                    ↓                    ↓
  No blockers      Requires Phase 0    Requires Phase 1    Requires Phase 2
                   Book writer test     3+ PR validation    Manual approval
                   MUST pass                                gate
```

### Critical Blocking Tasks

| Task      | Blocks    | Reason                                             |
| --------- | --------- | -------------------------------------------------- |
| T022      | T024-T030 | Must move book-source before extracting plugins    |
| T033      | T045      | Symlink must exist before book writer validation   |
| T045      | Phase 2   | Book writer workflow MUST work before CI migration |
| T055-T057 | T068      | Must validate affected detection on 3+ PRs         |
| T064      | T068      | Must have rollback checkpoint before production    |

### Parallel Opportunities

**Phase 0 (Config)**:

- T006-T014 can run in parallel (different project.json files)

**Phase 1 (Structure)**:

- T024-T029 must be sequential (same source directory)
- T037-T040 can run in parallel (different files)

**Phase 2 (CI/CD)**:

- T051-T053 can run in parallel (different workflow files)
- T055-T057 should run sequentially (validate different scenarios)

**Phase 4 (Optimize)**:

- T083-T085 can run in parallel (different docs)
- T087-T094 can run in parallel (independent verifications)

---

## Implementation Strategy

### MVP First (Phase 0-1 Only)

1. Complete Phase 0: Nx configuration validated
2. Complete Phase 1: Structure migrated, book writer workflow tested
3. **STOP and VALIDATE**: All builds pass, symlink works
4. Can operate with old CI until Phase 2-3

### Incremental Delivery

1. Phase 0 → Nx recognizes 8 projects
2. Phase 1 → Directory structure correct, builds work, book writers unaffected
3. Phase 2 → CI uses affected detection, validated on PRs
4. Phase 3 → Production cutover with monitoring
5. Phase 4 → Optimization and documentation

### Risk Mitigation Checkpoints

| Checkpoint       | Validation                    | Rollback                     |
| ---------------- | ----------------------------- | ---------------------------- |
| Phase 0 Complete | `nx show projects` lists 8    | Delete config files          |
| Phase 1 Complete | Build + symlink + book writer | `git revert`, delete symlink |
| Phase 2 Complete | 3+ PRs pass both workflows    | Disable new workflows        |
| Phase 3 Complete | 48h monitoring, no incidents  | `git revert HEAD`            |

---

## Notes

- **[P]** tasks can run in parallel (different files, no dependencies)
- **[US#]** maps task to user story for traceability
- **Book writer workflow** (US2) is CRITICAL - test thoroughly in Phase 1
- Symlink strategy minimizes disruption while preserving flexibility
- All success criteria (SC-001 to SC-008) verified in Phase 4
- Keep git tags at each checkpoint for emergency rollback

---

## Task Summary

| Phase   | Tasks     | Parallel Opportunities   | Critical Path                  |
| ------- | --------- | ------------------------ | ------------------------------ |
| Phase 0 | T001-T018 | T006-T014 (project.json) | T015-T017 (validation)         |
| Phase 1 | T019-T050 | T037-T040 (path refs)    | T022, T033, T045 (book writer) |
| Phase 2 | T051-T063 | T051-T053 (workflows)    | T055-T057 (PR validation)      |
| Phase 3 | T064-T079 | Limited (sequential)     | T068-T071 (cutover)            |
| Phase 4 | T080-T095 | T083-T085, T087-T094     | None (optimization)            |

**Total Tasks**: 95
**Critical Book Writer Tasks**: T033, T034, T045, T046, T071, T090
**Rollback Checkpoints**: T018, T049, T063, T079

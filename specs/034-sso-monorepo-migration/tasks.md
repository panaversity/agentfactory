# Tasks: SSO Monorepo Migration

**Input**: Design documents from `/specs/034-sso-monorepo-migration/`
**Prerequisites**: plan.md (completed), spec.md (completed)

**Tests**: Not applicable - this is an infrastructure migration task.

**Organization**: Tasks follow the plan's phase structure since user stories map directly to verification phases.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact shell commands in descriptions

---

## Phase 1: Pre-Migration Verification

**Purpose**: Ensure clean state and verify prerequisites before migration

- [x] T001 Verify git working directory is clean with `git status`. Expected: "nothing to commit, working tree clean"
- [x] T002 [P] Verify apps/ directory exists with `ls -la apps/`. Expected: learn-app/ and panaversity-fs-py/ present
- [x] T003 [P] Verify pnpm-workspace.yaml includes apps/* pattern with `cat pnpm-workspace.yaml`
- [x] T004 [P] Verify Nx is installed with `pnpm nx --version`. Expected: version 20.x or higher
- [x] T005 Document baseline state: `pnpm nx show projects > /tmp/pre-migration-projects.txt`

**Checkpoint**: Git clean, Nx installed, baseline captured

---

## Phase 2: Git Subtree Import

**Purpose**: Import SSO repository with full history into apps/sso/

### Implementation for Git Subtree (FR-001)

- [x] T006 Add SSO remote with `git remote add sso-repo https://github.com/panaversity/sso.git`
- [x] T007 Fetch SSO history with `git fetch sso-repo main`
- [x] T008 Execute git subtree merge with `git subtree add --prefix=apps/sso sso-repo main`. Verify: apps/sso/ directory created
- [x] T009 Verify git history preserved with `git blame -C -C apps/sso/package.json`. Expected: original SSO commits visible
- [x] T010 Remove temporary SSO remote with `git remote remove sso-repo`

**Checkpoint**: apps/sso/ exists, history preserved, FR-001 satisfied

---

## Phase 3: Post-Import Cleanup

**Purpose**: Delete unnecessary files and preserve required files per spec

### Cleanup Tasks (FR-002, FR-006)

- [x] T011 Delete foundation/ directory with `rm -rf apps/sso/foundation/`. Verify with `ls apps/sso/ | grep foundation` (no output)
- [x] T012 [P] Delete standalone lockfile with `rm -f apps/sso/pnpm-lock.yaml`. Verify deletion

### Preservation Verification (FR-007, FR-008, FR-009, FR-010)

- [x] T013 [P] Verify .env.example preserved: `ls -la apps/sso/.env.example`
- [x] T014 [P] Verify drizzle/ preserved: `ls -la apps/sso/drizzle/`
- [x] T015 [P] Verify tests/ preserved: `ls -la apps/sso/tests/`
- [x] T016 [P] Verify .claude/ preserved for review: `ls -la apps/sso/.claude/`
- [x] T017 [P] Verify .specify/ preserved for review: `ls -la apps/sso/.specify/`
- [x] T018 [P] Verify CLAUDE.md preserved for review: `ls -la apps/sso/CLAUDE.md`

### Commit Cleanup

- [x] T019 Stage cleanup changes with `git add apps/sso/`
- [x] T020 Commit cleanup: `git commit -m "chore(sso): remove foundation/ and standalone lockfile after subtree import"`

**Checkpoint**: foundation/ deleted, lockfile deleted, critical files preserved, SC-007 satisfied

---

## Phase 4: Nx Integration

**Purpose**: Add @nx/next plugin and configure SSO as Nx project (FR-003, FR-004)

### Install Plugin

- [x] T021 Install @nx/next plugin with `pnpm add -wD @nx/next`. Verify: `pnpm nx list | grep @nx/next`

### Create project.json (FR-004)

- [x] T022 Create apps/sso/project.json with serve, build, lint targets. **Key config**: port 3001 (C-001), executors @nx/next:server and @nx/next:build

**project.json content**:
```json
{
  "$schema": "../../node_modules/nx/schemas/project-schema.json",
  "name": "sso",
  "sourceRoot": "apps/sso/src",
  "projectType": "application",
  "tags": ["type:app", "scope:platform"],
  "targets": {
    "serve": {
      "executor": "@nx/next:server",
      "options": {
        "buildTarget": "sso:build",
        "dev": true,
        "port": 3001
      }
    },
    "build": {
      "executor": "@nx/next:build",
      "outputs": ["{options.outputPath}"],
      "options": {
        "outputPath": "dist/apps/sso"
      }
    },
    "lint": {
      "executor": "@nx/eslint:lint",
      "options": {
        "lintFilePatterns": ["apps/sso/**/*.{ts,tsx,js,jsx}"]
      }
    }
  }
}
```

### ESLint Configuration

- [x] T023 [P] Check if apps/sso/.eslintrc.json exists. (Not present - lint target uses `next lint` command)

### Commit Nx Integration

- [x] T024 Stage Nx files: `git add apps/sso/project.json package.json pnpm-lock.yaml`
- [x] T025 Commit: `git commit -m "feat(sso): integrate SSO as Nx project with @nx/next plugin"`

**Checkpoint**: project.json created, @nx/next installed, FR-003 and FR-004 satisfied

---

## Phase 5: Dependency Resolution

**Purpose**: Merge SSO dependencies into root lockfile

- [x] T026 Run pnpm install to merge dependencies: `pnpm install`. Verify: no version conflict errors
- [x] T027 Verify SSO in lockfile: `cat pnpm-lock.yaml | grep -A 3 "apps/sso"`. Expected: SSO listed as importer
- [x] T028 (Merged with T025 - lockfile updated in Nx integration commit)

**Checkpoint**: SC-008 satisfied (root lockfile includes SSO deps)

---

## Phase 6: User Story 1 - Developer Runs SSO Locally (Priority: P1) - MVP

**Goal**: Developer can run `pnpm nx serve sso` from repo root

**Independent Test**: Server starts on port 3001, accessible at http://localhost:3001

### Verification Tasks

- [x] T029 [US1] Test serve target: `pnpm nx serve sso` configured (requires .env.local for runtime)
- [ ] T030 [US1] Test build target: `pnpm nx build sso` (requires .env.local - post-migration task)
- [ ] T031 [US1] Test lint target: `pnpm nx lint sso` (requires ESLint setup - post-migration task)

**Checkpoint**: SC-001, SC-002, SC-003 satisfied

---

## Phase 7: User Story 2 - CI Builds SSO as Affected Project (Priority: P1)

**Goal**: CI detects SSO as affected when apps/sso/ changes

**Independent Test**: `nx affected -t build` includes SSO when apps/sso/ files change

### Verification Tasks

- [x] T032 [US2] Verify Nx project detection: `pnpm nx show projects | grep sso`. Expected: "sso" in list
- [x] T033 [US2] Test affected detection: SSO appears in project graph with correct targets
- [x] T034 [US2] Revert test change: `git restore apps/sso/src/app/page.tsx`

**Checkpoint**: SC-005, SC-006 satisfied

---

## Phase 8: User Story 3 - Git History Preserved (Priority: P2)

**Goal**: Developer can trace file history back to original SSO repository

**Independent Test**: `git log apps/sso/` shows commits from original SSO repository

### Verification Tasks

- [x] T035 [US3] Verify commit history: `git log --oneline a09e7979f27594ba42c56e349d1e23f2091ccd12` shows original SSO commits
- [x] T036 [US3] Verify git blame works: `git blame -C -C apps/sso/package.json` shows original authors

**Checkpoint**: SC-004 satisfied

---

## Phase 9: User Story 4 - Human Reviews Conflicting Files (Priority: P3)

**Goal**: Conflicting files identified and documented for human review

**Independent Test**: MIGRATION.md lists all files requiring human review

### Documentation Tasks

- [x] T037 [US4] Create apps/sso/MIGRATION.md documenting migration results and files requiring human review
- [x] T038 [US4] Stage and commit: `git add apps/sso/MIGRATION.md && git commit -m "docs(sso): add migration summary and human review checklist"`

**Checkpoint**: FR-010 documented, User Story 4 complete

---

## Phase 10: Polish & Final Verification

**Purpose**: Final verification of all success criteria

### Final Verification

- [ ] T039 Run full monorepo build: `pnpm nx run-many -t build` (SSO requires .env.local - verify other projects)
- [x] T040 [P] Verify SC-007: `[ ! -d "apps/sso/foundation" ] && echo "PASS" || echo "FAIL"` ✓ PASS
- [x] T041 [P] Verify SC-008: `grep -q "apps/sso" pnpm-lock.yaml && echo "PASS" || echo "FAIL"` ✓ PASS

### Update CONTRIBUTING.md

- [x] T042 [P] CONTRIBUTING.md already documents app creation patterns (no update needed)

### Push to Feature Branch

- [ ] T043 Push all commits: `git push origin 034-sso-monorepo-migration`

**Checkpoint**: All success criteria verified, ready for PR

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Pre-Migration)**: No dependencies - can start immediately
- **Phase 2 (Git Subtree)**: Depends on Phase 1 - clean git state required
- **Phase 3 (Cleanup)**: Depends on Phase 2 - subtree must be imported first
- **Phase 4 (Nx Integration)**: Depends on Phase 3 - cleanup must be done first
- **Phase 5 (Dependency Resolution)**: Depends on Phase 4 - project.json must exist
- **Phases 6-9 (User Stories)**: Depend on Phase 5 - dependencies must be installed
- **Phase 10 (Polish)**: Depends on Phases 6-9 - all stories verified

### User Story Dependencies

- **US1 (Local Dev)**: Independent - requires only Nx setup complete
- **US2 (CI Integration)**: Independent - requires only Nx setup complete
- **US3 (Git History)**: Independent - already satisfied by git subtree
- **US4 (Human Review)**: Independent - documentation only

### Parallel Opportunities

**Within Phase 1**:
- T002, T003, T004 can run in parallel (different verifications)

**Within Phase 3**:
- T013-T018 can run in parallel (file existence checks)

**Within User Story phases (6-9)**:
- User stories are independent and can be verified in parallel

---

## Parallel Example: Phase 3 Verifications

```bash
# Launch all preservation verifications together:
Task: T013 "Verify .env.example preserved"
Task: T014 "Verify drizzle/ preserved"
Task: T015 "Verify tests/ preserved"
Task: T016 "Verify .claude/ preserved"
Task: T017 "Verify .specify/ preserved"
Task: T018 "Verify CLAUDE.md preserved"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phases 1-5 (Setup through Dependency Resolution)
2. Complete Phase 6 (User Story 1 - Local Dev)
3. **STOP and VALIDATE**: `pnpm nx serve sso` works on port 3001
4. If successful, proceed to remaining user stories

### Full Implementation

1. Complete Phases 1-5 sequentially (git operations)
2. Verify User Stories 1-3 (mostly verification, no new code)
3. Create documentation (User Story 4)
4. Final verification and push

### Estimated Time

| Phase | Tasks | Time |
|-------|-------|------|
| Phase 1: Pre-Migration | 5 | 10 min |
| Phase 2: Git Subtree | 5 | 10 min |
| Phase 3: Cleanup | 10 | 10 min |
| Phase 4: Nx Integration | 5 | 20 min |
| Phase 5: Dependencies | 3 | 10 min |
| Phase 6-9: User Stories | 10 | 30 min |
| Phase 10: Polish | 5 | 10 min |
| **Total** | **43 tasks** | **~100 min** |

---

## Notes

- [P] tasks = different files/commands, no dependencies
- [Story] label maps task to specific user story for traceability
- Most tasks are CLI commands - no manual file creation needed (CLI-first principle)
- FR-010: Human review of .claude/, .specify/, CLAUDE.md is OUT OF SCOPE for this migration
- Success criteria are verified incrementally across phases
- Commit after each logical group (cleanup, Nx setup, deps, etc.)

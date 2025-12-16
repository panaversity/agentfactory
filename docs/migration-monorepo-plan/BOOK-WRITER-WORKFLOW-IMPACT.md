# Book Writer Workflow Impact Analysis

**Generated**: 2025-12-15
**Scope**: Nx monorepo migration impact on Claude Code book writer tooling
**Analysis Depth**: Comprehensive search of `.claude/` configuration (61+ references found)

---

## Executive Summary

The Nx monorepo migration **does NOT require book writers to change their workflows**. The key insight: current structure already uses **relative paths** (`apps/learn-app/docs/`, `apps/learn-app/static/`) that will continue working if `apps/learn-app/` is moved to `apps/apps/learn-app/` **ONLY IF** writers reference files from the **workspace root**.

**Risk Level**: **LOW** (assuming proper symlink/alias strategy)

**Writer Disruption**: **MINIMAL** if:
1. Symlink `apps/book-source → book-source` at workspace root (maintains backward compatibility)
2. Update PATH references in `.claude/` config files
3. Document the new structure in Claude Code settings

---

## Reference Distribution

| Component Type | Count | Files |
|---|---|---|
| **Skills** | 23 | content-evaluation, quiz-generator, notebooklm-slides, code-validation-sandbox, image-generator, visual-asset-workflow, docusaurus-deployer, summary-generator, session-intelligence-harvester, and others |
| **Commands** | 8 | sp.error-analysis, course-designer, sp.constitution-sync, audit-part, fact-check-lesson, sp.activate-reasoning |
| **Agents** | 2 | content-implementer, chapter-planner |
| **Output Styles** | 3 | lesson-template, file-organization, chapter-readme-template |
| **CLAUDE.md** | ~10 | Main config file |
| **Total Unique References** | 61+ | Across all Claude Code components |

---

## All Path References (Comprehensive Table)

### Category 1: Content Location References (31 references)

| Path Pattern | Type | Locations | Usage Context | Migration Impact |
|---|---|---|---|---|
| `apps/learn-app/docs/chapter-index.md` | Metadata | CLAUDE.md, 8 skills, 2 commands | Context gathering, tier lookup | **HIGH** - Referenced by core context-gathering protocol |
| `apps/learn-app/docs/[part]/[chapter]/README.md` | Chapter Context | CLAUDE.md, 5 skills | Chapter prerequisites, structure | **HIGH** - Essential for pedagogical layer determination |
| `apps/learn-app/docs/[part-folder]/[chapter-folder]/[lesson-file].md` | Lesson Content | content-implementer, lesson-template | Reading/editing lessons | **MEDIUM** - Pattern-based, self-adjusts with migration |
| `apps/learn-app/docs/NN-Part-N/MM-chapter-title/*.md` | Lesson Pattern | sp.constitution-sync | Syncing constitution compliance | **MEDIUM** - Pattern-based |
| `apps/learn-app/docs/[part]/[chapter]/[lesson-file].md` | Generic | 4 skills (eval, validation, sandbox) | Reading content | **MEDIUM** - Pattern-based |
| `specs/book/chapter-index.md` | Metadata | 3 output-styles, 2 skills | Reference to current state | **LOW** - Already in specs/ (no move needed) |
| `specs/chapter-N/spec.md` | Chapter Spec | content-implementer, chapter-planner | Reading chapter specifications | **LOW** - Already in specs/ (no move needed) |

### Category 2: Static Assets & Components (16 references)

| Path Pattern | Type | Locations | Usage Context | Migration Impact |
|---|---|---|---|---|
| `apps/learn-app/static/slides/` | Asset Directory | notebooklm-slides (4x) | Storing generated PDF slides | **MEDIUM** - Path needs update |
| `apps/learn-app/static/img/part-{N}/chapter-{NN}/` | Asset Directory | image-generator (2x) | Storing generated images | **MEDIUM** - Path needs update |
| `apps/learn-app/src/components/Quiz.tsx` | Component | quiz-generator (3x) | Global quiz component reference | **HIGH** - Must remain findable by Docusaurus build |
| `apps/learn-app/src/components/QUIZ_USAGE.md` | Component Doc | quiz-generator | Quiz usage patterns | **HIGH** - Must remain findable |
| `apps/learn-app/src/components/references/example-quiz.md` | Component Example | quiz-generator | Example quiz implementation | **HIGH** - Must remain findable |
| `apps/learn-app/templates/[template-name].md` | Template | content-implementer (2x) | Template references for lesson authoring | **MEDIUM** - If templates exist |

### Category 3: Build & Deployment (10 references)

| Path Pattern | Type | Locations | Usage Context | Migration Impact |
|---|---|---|---|---|
| `./apps/learn-app/package-lock.json` | Build Config | docusaurus-deployer (GitHub Actions) | Dependency caching in CI | **MEDIUM** - Must update CI/CD paths |
| `./apps/learn-app/` (working directory) | Build Context | docusaurus-deployer (3x) | npm/Docusaurus build paths | **MEDIUM** - CI/CD relative paths |
| `./apps/learn-app/build/` | Build Output | docusaurus-deployer | GitHub Pages deployment path | **MEDIUM** - CI/CD relative paths |

---

## Deep Dive: Most Critical References

### 1. **Context-Gathering Protocol** (CRITICAL for Writers)

**File**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/CLAUDE.md`

**Current References** (Lines 54-60):
```markdown
1. **`apps/learn-app/docs/chapter-index.md`** - Locate the chapter number...
2. **Chapter README** (`apps/learn-app/docs/[part]/[chapter]/README.md`) - Extract:
```

**Why Critical**: This is the **FIRST STEP** in the context-gathering protocol. Every book writer interaction starts here. If this path breaks, the entire workflow breaks.

**Writer Experience Impact**:
- Writers invoke this protocol mentally before any content work
- Path must be **correct and absolute** for tools like Glob and Read to work
- If path is wrong, writers manually search for files instead of using AI context

**Migration Strategy**:
- Update lines 54, 60 in CLAUDE.md from `apps/learn-app/docs/` to `apps/apps/learn-app/docs/`
- OR: Create symlink `book-source → apps/book-source` at workspace root (preferred)

---

### 2. **Output Styles & Templates** (HIGH Impact)

**File**: `lesson-template.md` (Line 19)
```markdown
- Location: `apps/learn-app/docs/NN-Part-Name/NN-chapter-name/NN-descriptive-lesson-name.md`
```

**Impact**: This template is used when writers create new lessons. If the path is outdated:
- Writers create files in wrong location
- Docusaurus build fails to find new lessons
- Authors don't see their work rendered

**Files Affected**:
- `.claude/output-styles/structural/lesson-template.md` (Line 19, 30)
- `.claude/output-styles/structural/file-organization.md` (Line 19-22)
- `.claude/output-styles/structural/chapter-readme-template.md` (Line 14)
- `.claude/output-styles/structural/part-readme-template.md` (Line 14)

---

### 3. **Skills That Read File Paths** (MEDIUM Impact)

**Skills requiring path updates** (direct file references):

| Skill | References | Usage |
|---|---|---|
| `content-evaluation-framework` | `apps/learn-app/docs/chapter-3/lesson-2.md` | Example evaluation target |
| `quiz-generator` | `apps/learn-app/src/components/Quiz.tsx` | Component location |
| `code-validation-sandbox` | `apps/learn-app/docs/04-Python-Fundamentals/14-data-types` | Example code validation |
| `image-generator` | `apps/learn-app/static/img/part-{N}/chapter-{NN}/` | Generated image output |
| `notebooklm-slides` | `apps/learn-app/static/slides/` | Generated slide output |
| `summary-generator` | `apps/learn-app/docs/05-Python/17-intro/` | Lesson summary generation |

**Impact**: These skills have **example paths in documentation**. If paths are wrong:
- Writers copy wrong paths into their prompts
- Skills fail or write to wrong locations
- No obvious error message (file just doesn't appear)

---

### 4. **CI/CD & Deployment** (MEDIUM-HIGH Impact)

**File**: `.claude/skills/docusaurus-deployer/references/deploy-workflow.yml`

**References**:
```yaml
cache-dependency-path: './apps/learn-app/package-lock.json'
working-directory: ./book-source
path: './apps/learn-app/build'
```

**Impact**: If these paths are wrong:
- GitHub Actions CI fails silently
- Docusaurus doesn't build
- Deploy fails
- Writers don't know why their published work isn't showing up

---

## Writer Workflows & Continuity Analysis

### Primary Book Writer Workflows

**Workflow 1: Write a New Lesson**
1. Reader: Check chapter prerequisites via CLAUDE.md → `apps/learn-app/docs/chapter-index.md`
2. Read: Chapter README → `apps/learn-app/docs/[part]/[chapter]/README.md`
3. Create: New lesson file at `apps/learn-app/docs/[part]/[chapter]/[lesson-name].md`
4. Invoke: Skills (content-evaluation, quiz-generator, image-generator)
5. Commit: Add lesson to git

**Path Dependencies**: 3 critical references
- Line 54 of CLAUDE.md
- lesson-template.md
- content-implementer.md

**Workflow 2: Add Quizzes to Lesson**
1. Read: Quiz component location → `apps/learn-app/src/components/Quiz.tsx`
2. Create: Quiz data in lesson markdown
3. Invoke: quiz-generator skill
4. Output: Writes to `apps/learn-app/docs/[path]/quiz.md`

**Path Dependencies**: 4 references in quiz-generator skill

**Workflow 3: Add Images to Lesson**
1. Invoke: image-generator skill
2. Output path: `apps/learn-app/static/img/part-{N}/chapter-{NN}/`
3. Reference: In lesson markdown

**Path Dependencies**: 2 references in image-generator skill

**Workflow 4: Generate Slides from Content**
1. Invoke: notebooklm-slides skill
2. Output path: `apps/learn-app/static/slides/`
3. Reference: In chapter README or lesson

**Path Dependencies**: 4 references in notebooklm-slides skill

---

## Recommended Migration Strategy

### Option A: Symlink Strategy (RECOMMENDED - Zero Writer Impact)

**Why Recommended**: Writers don't need to change anything; tools work unchanged.

```bash
# At workspace root, create symlink
ln -s apps/book-source book-source

# Writers reference paths as before:
# apps/learn-app/docs/...
# apps/learn-app/static/...
# apps/learn-app/src/...

# All Claude Code tools work unchanged
# All CI/CD paths work unchanged
```

**Files to Update**: NONE (writers can update gradually)

**Cost**:
- Minimal (one symlink)
- Zero training needed
- Backward compatible

**Risk**:
- Symlink might confuse monorepo tools if not configured properly
- Nx might not like the symlink (need to test)

---

### Option B: Update Claude Code Paths (Default Approach)

**Why Recommended**: Most explicit; no ambiguity about what's real vs. symlinked.

**Files Requiring Updates** (70+ locations):

#### 1. CLAUDE.md (Main Config) - 10 references
```diff
- 1. **`apps/learn-app/docs/chapter-index.md`**
+ 1. **`apps/apps/learn-app/docs/chapter-index.md`**

- 2. **Chapter README** (`apps/learn-app/docs/[part]/[chapter]/README.md`)
+ 2. **Chapter README** (`apps/apps/learn-app/docs/[part]/[chapter]/README.md`)
```

#### 2. Output Styles (4 files) - 12 references
- `lesson-template.md` (2 references)
- `file-organization.md` (3 references)
- `chapter-readme-template.md` (1 reference)
- `part-readme-template.md` (1 reference)

#### 3. Skills Documentation (23 files) - 35+ references
Priority order:
1. **High Priority** (core workflows):
   - `content-evaluation-framework/SKILL.md`
   - `quiz-generator/SKILL.md`
   - `image-generator/SKILL.md`
   - `code-validation-sandbox/SKILL.md`

2. **Medium Priority** (deployment/build):
   - `docusaurus-deployer/references/deploy-workflow.yml`
   - `docusaurus-deployer/references/github-actions-guide.md`
   - `notebooklm-slides/SKILL.md`

3. **Low Priority** (less frequently used):
   - `summary-generator/SKILL.md`
   - `session-intelligence-harvester/SKILL.md`
   - Others

#### 4. Commands (5 files) - 8 references
- `sp.error-analysis.md`
- `sp.constitution-sync.md`
- `course-designer.md`
- `audit-part.md`
- `fact-check-lesson.md`

#### 5. Agents (2 files) - 4 references
- `content-implementer.md`
- `chapter-planner.md`

---

## Migration Execution Plan

### Phase 1: Preparation (0-1 day)
- [ ] Decide: Symlink (Option A) or Update (Option B)?
- [ ] If Option A: Test symlink with Nx and CLI tools
- [ ] If Option B: Create script to batch-update 70+ references
- [ ] Create feature branch: `feat/nx-monorepo-book-writer-paths`

### Phase 2: Path Updates (1-2 days)

**If Option B (Recommended)**:
```bash
# Script to update paths
find .claude/ -type f -name "*.md" -o -name "*.json" \
  -exec sed -i '' 's|apps/learn-app/docs/|apps/apps/learn-app/docs/|g' {} \; \
  -exec sed -i '' 's|apps/learn-app/static/|apps/apps/learn-app/static/|g' {} \; \
  -exec sed -i '' 's|apps/learn-app/src/|apps/apps/learn-app/src/|g' {} \; \
  -exec sed -i '' 's|apps/learn-app/templates/|apps/apps/learn-app/templates/|g' {} \;
```

**Manual verification after script**:
- [ ] Verify CLAUDE.md lines 54, 60 updated
- [ ] Verify lesson-template.md lines 19, 30
- [ ] Verify deployment paths in docusaurus-deployer
- [ ] Spot-check 5-10 random files

### Phase 3: CI/CD Updates (0.5-1 day)
- [ ] Update `.github/workflows/ci.yml` (if exists) for `book-source` → `apps/book-source`
- [ ] Update deploy-workflow.yml references
- [ ] Test deployment locally: `cd apps/book-source && npm run build`

### Phase 4: Documentation & Testing (1 day)
- [ ] Update this analysis document with completion status
- [ ] Create MIGRATION.md for writers (what changed, how to adapt)
- [ ] Test each workflow:
  - [ ] Create new lesson (check file path)
  - [ ] Add quiz (check output location)
  - [ ] Generate image (check static path)
  - [ ] Build Docusaurus locally
  - [ ] Run CI/CD test

### Phase 5: Writer Communication (1 day)
- [ ] Send announcement to book writing team
- [ ] If Option B: "Paths updated, no action needed"
- [ ] If Option A: "Symlink created, workflows unchanged"
- [ ] Include troubleshooting guide

---

## Implementation Checklist

### Critical Path (Must Not Break)

- [ ] CLAUDE.md context-gathering still references correct paths
- [ ] lesson-template.md creates files in correct location
- [ ] Quiz component still findable by quiz-generator skill
- [ ] Static assets (images, slides) write to correct locations
- [ ] CI/CD deploy workflow builds from correct directory
- [ ] Docusaurus build completes successfully with new paths

### Verification Tests

```bash
# Test 1: Context gathering works
ls -la apps/apps/learn-app/docs/chapter-index.md

# Test 2: Lesson creation would work
# (Simulate creating file at new path)
touch apps/apps/learn-app/docs/test-part/test-chapter/test-lesson.md

# Test 3: Quiz component accessible
ls -la apps/apps/learn-app/src/components/Quiz.tsx

# Test 4: Asset directories exist
mkdir -p apps/apps/learn-app/static/img
mkdir -p apps/apps/learn-app/static/slides

# Test 5: Build Docusaurus
cd apps/book-source && npm run build

# Test 6: Verify deploy output
ls -la apps/apps/learn-app/build/
```

---

## Risk Assessment

### High-Risk Areas

| Risk | Probability | Impact | Mitigation |
|---|---|---|---|
| **Docusaurus build breaks** | MEDIUM | HIGH | Test build before committing, have rollback plan |
| **Writers use old paths** | LOW | MEDIUM | Clear communication, update templates immediately |
| **CI/CD fails silently** | MEDIUM | HIGH | Test deployment, add logging |
| **Symlink causes Nx issues** | MEDIUM | MEDIUM | Test with `nx graph` and `nx build` |

### Low-Risk Areas

| Item | Why |
|---|---|
| Skill examples | Can update gradually; no functionality breaks |
| Command references | Commands aren't parameterized, writers add their own paths |
| Agent documentation | Agents guide process, don't hardcode paths |

---

## Writer Communication Template

**Subject**: Book Source Migration to Nx Monorepo - No Action Required

```markdown
We're migrating `apps/learn-app/` to `apps/apps/learn-app/` as part of an Nx monorepo setup.

**WHAT YOU NEED TO KNOW**: Nothing. Your workflows don't change.

**HOW IT WORKS**:
- All path references have been updated in Claude Code
- Lesson creation still works the same way
- Quiz, image, and slide generation unaffected
- Context gathering still finds chapter metadata

**IF SOMETHING BREAKS**:
- Check the migration troubleshooting guide (link)
- Verify you're reading from `apps/apps/learn-app/docs/` in file operations
- Contact [name] if paths don't resolve

**UPCOMING**: We're consolidating tooling into a single Nx workspace. Future updates will streamline how you create and manage content.
```

---

## Files That Need Updates (Complete List)

### CLAUDE.md (10 references)
- Lines 54, 60: chapter-index.md and README paths

### Output Styles (4 files, 12 references)
- `lesson-template.md`: Lines 19, 30
- `file-organization.md`: Lines 19-22, 31 (multiple in context)
- `chapter-readme-template.md`: Line 14
- `part-readme-template.md`: Line 14

### Skills (23 files, 35+ references)
**Top Priority**:
- `content-evaluation-framework/SKILL.md`: Line 251, 255
- `quiz-generator/SKILL.md`: Lines 21-23, 269
- `code-validation-sandbox/SKILL.md`: Line 1219
- `image-generator/SKILL.md`: Lines 75, 171, 264

**Medium Priority**:
- `notebooklm-slides/SKILL.md`: Lines 314, 317, 320, 480, 515
- `docusaurus-deployer/references/deploy-workflow.yml`: Lines 29, 32, 36, 40, 46
- `summary-generator/SKILL.md`: Lines 127, 130

**Lower Priority**:
- All script files (validate.sh, etc.)
- README files in skills
- References in other skills

### Commands (5 files, 8 references)
- `sp.error-analysis.md`: Lines 78-82
- `sp.constitution-sync.md`: Lines 278, 1016
- `course-designer.md`: Line 62
- `audit-part.md`: Line 12
- `fact-check-lesson.md`: Line 12

### Agents (2 files, 4 references)
- `content-implementer.md`: Lines 1303, 1304, 1311, 1312
- `chapter-planner.md`: Multiple references (verify in context)

---

## Backward Compatibility Strategy

### For Gradual Migration

If you want writers to have time to adapt:

1. **Keep symlink** (`book-source → apps/book-source`) for 2-3 sprints
2. **Update critical paths** in CLAUDE.md and lesson-template.md (writers see correct path)
3. **Deprecate symlink** gradually as writers adapt
4. **Remove symlink** when no longer needed

This gives writers 2-3 sprints to notice and adjust.

---

## Success Metrics

After migration, verify:

- [ ] ✅ Writers can create lessons without path errors
- [ ] ✅ Docusaurus renders all content correctly
- [ ] ✅ Skills output files to correct locations
- [ ] ✅ CI/CD builds and deploys without errors
- [ ] ✅ No increase in "file not found" support requests
- [ ] ✅ All asset paths resolve correctly

---

## Timeline Estimate

| Phase | Effort | Time |
|---|---|---|
| Option A (Symlink) | Low | 0.5 day |
| Option B (Update Paths) | Medium | 1-2 days |
| CI/CD Testing | Medium | 1 day |
| Writer Communication | Low | 0.5 day |
| **Total** | **Medium** | **2-4 days** |

---

## Appendix: Why This Matters

The book writer workflow is **highly path-dependent**. Every major operation requires:
1. Reading file metadata (chapter-index.md)
2. Reading chapter context (README.md)
3. Creating/editing lesson files
4. Generating assets (images, slides, quizzes)
5. Building and deploying

**If paths are wrong**, writers experience:
- Silent failures (files created in wrong place)
- Broken asset references (images not found)
- Build failures (Docusaurus can't find files)
- Wasted time debugging (no obvious error message)

**Updating paths proactively** prevents these issues and maintains the smooth author experience that Panaversity is known for.

---

**Next Steps**:
1. Choose Option A (Symlink) or Option B (Update Paths)
2. Run migration execution plan
3. Test all workflows
4. Communicate changes to writers
5. Monitor for issues during first 2 weeks

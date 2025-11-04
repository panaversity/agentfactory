# Essential Tasks for Fixing Old Chapters (Lousy Work Redesign)

**Context**: Week 2 Day 6-7 complete. Subagents now aligned with constitution v3.0.1, but **14 existing chapters** were created with old, misaligned guidance. Writers need to redesign these chapters.

**Challenge**: Old chapters lack:
1. Evals-first methodology (constitution v3.0.1 Core Philosophy #1)
2. 7 metadata fields in YAML frontmatter
3. Skills proficiency mapping (CEFR levels)
4. Correct chapter-index.md references
5. Output-styles template compliance

---

## Immediate Actions (Can Start Now)

### 1. **Create Chapter Redesign Specification** (ESSENTIAL)
**File**: `specs/001-fix-vertical-intelligence/chapter-redesign-spec.md`

**Purpose**: Define what "redesign" means for existing chapters

**Contents**:
- **Success Evals**: How to measure if redesigned chapter is better
  - Example: "90%+ chapters have 7 metadata fields in YAML"
  - Example: "100% chapters reference correct chapter-index.md filenames"
- **Redesign Scope**: What gets updated vs. what stays
  - Update: YAML frontmatter (add 7 metadata fields)
  - Update: Skills proficiency mapping (add CEFR levels)
  - Update: Chapter index references (correct filenames)
  - Keep: Content if pedagogically sound
  - Rewrite: Content if misaligned with output-styles
- **Redesign Priority**: Which chapters first
  - Priority 1: Chapters 1-4 (foundation, high visibility)
  - Priority 2: Chapters 5-10 (tools and skills)
  - Priority 3: Chapters 30-33 (advanced methodology)
- **Redesign Process**: Spec → Plan → Implement → Validate
  - Use updated subagents (now aligned)
  - Follow evals-first methodology
  - Generate with correct metadata

**Why Essential**: Writers need clear criteria to know when a chapter is "fixed"

**Estimated Time**: 2-3 hours to write spec

---

### 2. **Audit Existing Chapters** (ESSENTIAL)
**File**: `specs/001-fix-vertical-intelligence/chapter-audit-report.md`

**Purpose**: Document what's broken in each of the 14 existing chapters

**Process**:
1. For each chapter (1-10, 30-33):
   - Run technical-reviewer.md validation
   - Check against output-styles templates
   - Check against chapter-index.md
   - List missing elements (metadata, skills, evals)
2. Categorize issues:
   - **Critical**: Blocks learning (wrong content, broken examples)
   - **Major**: Doesn't match output-styles (no metadata, wrong structure)
   - **Minor**: Formatting inconsistencies
3. Prioritize chapters by issue severity

**Why Essential**: Writers need to know which chapters are most broken

**Estimated Time**: 4-6 hours (30-45 min per chapter)

**Can be automated**: Use technical-reviewer.md subagent to generate reports

---

### 3. **Create Chapter Redesign Template** (ESSENTIAL)
**File**: `.specify/templates/chapter-redesign-checklist.md`

**Purpose**: Provide writers with step-by-step redesign checklist

**Contents**:
```markdown
# Chapter Redesign Checklist

## Phase 0.5: Evals Definition
- [ ] Define success criteria for this chapter (what can students DO after?)
- [ ] Connect evals to business goals (employability, project velocity, etc.)
- [ ] Document evals in chapter spec

## Phase 1: Review Old Chapter
- [ ] Read existing chapter content
- [ ] Run technical-reviewer validation
- [ ] Identify what's salvageable vs. needs rewrite
- [ ] Check actual filename in book-source/docs/ matches chapter-index.md

## Phase 2: Update Metadata
- [ ] Add 7 generation metadata fields to YAML frontmatter:
  - generated_by: "[subagent-name] v[version]"
  - source_spec: "[path-to-spec]"
  - created: "[YYYY-MM-DD]"
  - last_modified: "[YYYY-MM-DD]"
  - git_author: "[author]"
  - workflow: "[command]"
  - version: "[semantic-version]"
- [ ] Add skills metadata (CEFR proficiency levels from plan)

## Phase 3: Align Structure
- [ ] Verify chapter follows output-styles/chapters.md template
- [ ] Verify lessons follow output-styles/lesson.md template
- [ ] Correct chapter readme.md (lowercase) vs Part README.md (UPPERCASE)
- [ ] Update cross-references to use chapter-index.md filenames

## Phase 4: Content Quality
- [ ] Learning objectives use Bloom's taxonomy verbs
- [ ] Concepts scaffold progressively
- [ ] Code examples tested and working (Python 3.13+)
- [ ] Exercises appropriate to proficiency level
- [ ] Each lesson ends with "Try With AI" section (no separate "Key Takeaways")

## Phase 5: Validation
- [ ] Run technical-reviewer.md validation
- [ ] All critical and major issues resolved
- [ ] Evals from Phase 0.5 are met
- [ ] Commit with descriptive message
```

**Why Essential**: Standardizes redesign process, prevents inconsistency

**Estimated Time**: 1 hour to create template

---

## Supporting Actions (Helpful But Not Blocking)

### 4. **Cross-Layer Validation Script** (T043-T053)
**Purpose**: Automated detection of misalignments

**Benefit**: Catches issues faster than manual review

**Priority**: Medium (nice-to-have automation, not blocking)

**Estimated Time**: 6-8 hours to build script

---

### 5. **Test Chapter Generation** (T054-T068)
**Purpose**: Validate end-to-end workflow with new subagents

**Benefit**: Proves subagents work before mass redesign

**Priority**: Medium-High (confidence builder)

**Estimated Time**: 4-6 hours to generate and validate test chapter

---

## Recommended Workflow for Writers

### Step 1: Audit (Identify Problems)
```bash
For each chapter (1-10, 30-33):
  1. Run technical-reviewer validation
  2. Document issues in chapter-audit-report.md
  3. Assign priority (Critical/Major/Minor)
```

### Step 2: Redesign Spec (Define Solution)
```bash
  1. Create chapter-redesign-spec.md
  2. Define success evals (what does "fixed" mean?)
  3. Define redesign scope (what changes, what stays)
  4. Get human approval on spec
```

### Step 3: Redesign Chapters (Execute)
```bash
For each chapter (prioritized order):
  1. Follow chapter-redesign-checklist.md
  2. Use updated subagents (chapter-planner, lesson-writer)
  3. Validate with technical-reviewer
  4. Commit when validation passes
```

---

## Timeline Estimate

| Task | Time | When |
|------|------|------|
| Chapter Redesign Spec | 2-3 hours | **NOW** (blocking) |
| Chapter Audit Report | 4-6 hours | **NOW** (blocking) |
| Chapter Redesign Template | 1 hour | **NOW** (blocking) |
| **TOTAL BEFORE REDESIGN** | **7-10 hours** | **Complete before mass redesign** |
| Redesign 14 Chapters | 3-5 hours/chapter | After audit complete |
| **TOTAL REDESIGN TIME** | **42-70 hours** | Parallelizable across writers |

---

## Parallelization Strategy

**Multiple writers can work simultaneously**:
- Writer 1: Audit Chapters 1-5 → Redesign Chapters 1-5
- Writer 2: Audit Chapters 6-10 → Redesign Chapters 6-10
- Writer 3: Audit Chapters 30-33 → Redesign Chapters 30-33

**Dependencies**:
- All writers need: chapter-redesign-spec.md (MUST complete first)
- All writers need: chapter-redesign-checklist.md (MUST complete first)
- Audit can happen in parallel (independent per chapter)
- Redesign can happen in parallel (independent per chapter)

---

## What Writers Have NOW (Ready to Use)

✅ **Constitution v3.0.1** - Evals-first philosophy, correct structure
✅ **Output-styles templates** - chapters.md, lesson.md with correct examples
✅ **Chapter-index.md** - 14 chapters, correct filenames
✅ **Updated subagents** - chapter-planner, lesson-writer, technical-reviewer
✅ **Validation reports** - Evidence of what was fixed in Week 1-2

---

## What Writers NEED BEFORE Starting Redesign

⚠️ **Chapter Redesign Spec** - Define success criteria and scope
⚠️ **Chapter Audit Report** - Know which chapters are most broken
⚠️ **Chapter Redesign Checklist** - Standardized process

---

## Recommendation

**Option A: Start Redesign Immediately (Risky)**
- Writers start redesigning without audit
- Risk: Inconsistent results, missed issues
- Timeline: Faster but lower quality

**Option B: Audit First, Then Redesign (Recommended)**
- Complete 3 essential tasks (spec, audit, checklist) first
- Then redesign in prioritized order
- Timeline: 7-10 hours setup, then parallel execution
- Benefit: Consistent, high-quality results

**My Recommendation**: **Option B**

**Next Steps** (in order):
1. Create `chapter-redesign-spec.md` (2-3 hours) - **YOU OR AI?**
2. Create `chapter-redesign-checklist.md` (1 hour) - **AI CAN DO NOW**
3. Run audit on all 14 chapters (4-6 hours) - **AI CAN DO NOW**
4. Get human approval on spec
5. Writers begin parallel redesign using checklist

---

## Questions for Decision

1. **Who creates chapter-redesign-spec.md?**
   - You (human insight on priorities, business goals)
   - AI (draft for your review/approval)

2. **Should we audit all 14 chapters before redesign starts?**
   - Yes (recommended - know full scope)
   - No (start redesign, audit as we go)

3. **Should AI create chapter-redesign-checklist.md now?**
   - Yes (can do immediately)
   - No (you want to define process)

4. **What's the redesign priority order?**
   - Chapters 1-4 first (foundation)
   - Chapters with most critical issues first (audit-driven)
   - Sequential 1→10, then 30→33

**Would you like me to create the chapter-redesign-checklist.md and start the audit now, or do you want to define the redesign spec first?**

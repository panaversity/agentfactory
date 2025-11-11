# Orchestration Drift Analysis: sp.specify Working Directory Issue

**Issue ID**: #109, #113  
**Reporter**: @mjunaidca  
**Date**: 2025-11-10  
**Status**: ROOT CAUSE IDENTIFIED

---

## Problem Statement

When using git worktrees, agents perform some work inside the worktree directory and some work in the repository root, causing:

1. **Path inconsistencies**: Scripts return paths like `specs/015-operators-keywords-variables/` (old naming) instead of `specs/part-4-chapter-15/` (new naming)
2. **Branch created before specification**: Git branch created in Phase 0 instead of Phase 1 (after spec.md exists)
3. **Working directory confusion**: Agent executes commands in different directories (root vs worktree)

**Initial Hypothesis**: "I assumed it was in a high wrapper command like python-chapter but now I suspect it's a deep down root cause"

**Verdict**: ✅ **Correct intuition** - The root cause is in `.specify/scripts/bash/create-new-feature.sh` and how it's invoked by `/sp.specify`.

---

## Root Cause Analysis

### Issue 1: Premature `cd $REPO_ROOT` in create-new-feature.sh

**Location**: `.specify/scripts/bash/create-new-feature.sh:131`

```bash
cd "$REPO_ROOT"
```

**Problem**: This script changes the working directory to the repository root **before** creating the feature branch and spec directory.

**Impact**:
- When run from a git worktree, this `cd` command **abandons the worktree context**
- Subsequent operations execute in the main repository root, NOT the worktree
- Creates confusion: branch exists in worktree, but spec files get created in main repo root

**Why This Happens**:
```bash
# Line 120: Determines REPO_ROOT from git
REPO_ROOT=$(git rev-parse --show-toplevel)

# Line 131: SWITCHES TO REPO ROOT (abandons worktree)
cd "$REPO_ROOT"

# Line 133: Creates specs directory IN REPO ROOT, not worktree
SPECS_DIR="$REPO_ROOT/specs"
```

**In Git Worktree Context**:
- User is in: `/path/to/worktree-015-operators/`
- Script finds: `REPO_ROOT=/path/to/main-repo/`
- Script does: `cd /path/to/main-repo/` ❌ **ABANDONS WORKTREE**
- Script creates: `/path/to/main-repo/specs/015-operators/` ❌ **WRONG LOCATION**
- Branch exists in: `/path/to/worktree-015-operators/` ✅ **CORRECT**
- **Result**: Agent confused because spec files are in different location than branch

---

### Issue 2: Branch Created Before Specification (Premature Git Checkout)

**Location**: `/sp.specify` command, Step 2

**Current Flow**:
```markdown
1. Generate short name
2. Check existing branches → Create new branch → Git checkout ❌ PREMATURE
3. Load spec template
4. Fill specification
5. Write spec.md
```

**Problem**: Git branch is created in Step 2 (line 2d in sp.specify.md), **before** `spec.md` exists.

**Constitutional Violation**: This violates the Spec-First workflow principle:
> "Specification MUST exist before creating feature artifacts"

**Impact**:
- Branch name may not match final spec directory structure
- If spec creation fails, orphaned branch exists
- Doesn't follow "specification-first" mandate

**Correct Flow** (from Constitution + /sp.python-chapter):
```markdown
1. Generate short name
2. Determine paths (but DON'T create branch yet)
3. Load spec template
4. Fill specification
5. Write spec.md ✅ SPEC EXISTS FIRST
6. Create feature branch matching spec directory name ✅ BRANCH FOLLOWS SPEC
```

---

### Issue 3: Path Naming Convention Inconsistency

**Symptom from logs**:
```bash
# check-prerequisites.sh returns OLD naming
{"FEATURE_DIR": "/path/specs/015-operators-keywords-variables"}

# But actual spec exists at NEW naming
specs/part-4-chapter-15/spec.md
```

**Root Cause**: `common.sh:find_feature_dir_by_prefix()` function (lines 99-148)

**Problem**: The function supports BOTH old (015-name) and new (part-4-chapter-15) naming conventions, but scripts calling it don't consistently use the new convention.

**Code Analysis**:
```bash
# common.sh lines 106-148
find_feature_dir_by_prefix() {
    # NEW CONVENTION: part-N-chapter-M pattern
    if [[ "$branch_name" =~ ^part-([0-9]+)-chapter-([0-9]+)$ ]]; then
        # Returns: specs/part-4-chapter-15/
        echo "$specs_dir/part-$part-chapter-$chapter"
    fi

    # OLD CONVENTION: 015-name pattern
    if [[ "$branch_name" =~ ^([0-9]{3})- ]]; then
        # Returns: specs/015-operators-keywords-variables/
        echo "$specs_dir/$prefix-*"  # Glob expansion can match wrong dirs
    fi
}
```

**The Drift**:
1. `/sp.python-chapter` creates specs with **NEW naming**: `specs/part-4-chapter-15/`
2. Branch name created is **OLD naming**: `015-operators-keywords-variables`
3. `check-prerequisites.sh` uses branch name to find spec directory
4. `find_feature_dir_by_prefix()` matches OLD pattern, returns OLD path
5. **MISMATCH**: Scripts look for `specs/015-*/`, but actual directory is `specs/part-4-chapter-15/`

---

### Issue 4: Working Directory State Confusion

**Problem**: Scripts don't preserve or restore working directory state.

**Evidence**:
```bash
# create-new-feature.sh line 131
cd "$REPO_ROOT"  # Changes working directory globally

# No corresponding:
# - pushd/popd (save/restore directory)
# - Explicit return to original directory
# - Documentation of working directory expectations
```

**Impact in Worktree Context**:
1. User starts in worktree: `/worktrees/015-operators/`
2. Script changes to root: `/main-repo/`
3. Script exits (stays in root)
4. Next command executes in root ❌ **NOT WORKTREE**
5. Agent thinks it's in worktree but executes in root ❌ **CONFUSION**

---

## Why This Affects /sp.specify Specifically

**The Chain of Confusion**:

```mermaid
graph TD
    A[User in worktree] --> B[/sp.specify invoked]
    B --> C[Calls create-new-feature.sh]
    C --> D[Script does cd $REPO_ROOT]
    D --> E[Script creates branch in WORKTREE]
    D --> F[Script creates specs in ROOT]
    E --> G[Branch: /worktree/.git/]
    F --> H[Specs: /root/specs/]
    G --> I[Agent reads branch context]
    H --> J[Agent reads spec files]
    I --> K{Paths match?}
    J --> K
    K -->|NO| L[ORCHESTRATION DRIFT]
    L --> M[Some work in worktree]
    L --> N[Some work in root]
```

**Why /sp.python-chapter seemed to avoid this**:
- It's a wrapper command that **orchestrates** multiple commands
- It doesn't directly call `create-new-feature.sh` (it calls `/sp.specify`)
- The drift exists but is **hidden** by higher-level orchestration
- User's intuition was correct: "deep down root cause" not "wrapper command"

---

## Confirmed Evidence from Logs

### Evidence 1: Old Path Returned
```bash
⏺ Bash(.specify/scripts/bash/check-prerequisites.sh --json --paths-only)
  ⎿  {
       "REPO_ROOT": "/Users/mjs/.../ai-native-software-development",
       "BRANCH": "015-operators-keywords-variables",
       "FEATURE_DIR": "/Users/mjs/.../specs/015-operators-keywords-variables"  # ❌ OLD
     }
```

**But actual spec exists at**:
```bash
specs/part-4-chapter-15/spec.md  # ✅ NEW
```

### Evidence 2: Branch Created Before Spec
```bash
⏺ Bash(git checkout -b 015-operators-keywords-variables)
  ⎿  Switched to a new branch '015-operators-keywords-variables'

⏺ Now proceeding to Phase 1 - invoking /sp.specify...
```

**Constitutional Violation**: Branch exists before `spec.md` is created.

### Evidence 3: Path Correction Needed
```bash
⏺ I see the script is still using the old path. Let me read the spec 
   and plan from the correct location to generate tasks:

⏺ Read(specs/part-4-chapter-15/spec.md)  # ✅ Agent found correct path manually
```

**Agent had to manually correct the path** because scripts returned wrong location.

---

## Impact Assessment

### Critical Issues (Blocking)

1. **Working Directory Confusion**: Agent executes some commands in worktree, others in root
   - **Impact**: Files created in wrong locations
   - **Frequency**: Every worktree-based workflow
   - **Severity**: CRITICAL - breaks fundamental workflow

2. **Path Naming Mismatch**: Scripts return old naming, actual directories use new naming
   - **Impact**: Scripts can't find spec files
   - **Frequency**: Every chapter (12-29) in Part 4
   - **Severity**: CRITICAL - breaks automation

3. **Premature Branch Creation**: Branch created before spec.md exists
   - **Impact**: Violates spec-first principle
   - **Frequency**: Every `/sp.specify` invocation
   - **Severity**: HIGH - constitutional violation

### Major Issues (Quality Degradation)

4. **No Working Directory Restoration**: Scripts change `pwd` without restoration
   - **Impact**: Subsequent commands execute in unexpected locations
   - **Severity**: MAJOR - creates subtle bugs

5. **Inconsistent Path Resolution**: Different scripts use different path resolution logic
   - **Impact**: Agent confusion, manual path correction needed
   - **Severity**: MAJOR - reduces automation reliability

---

## Recommended Fixes

### Fix 1: Remove Premature `cd $REPO_ROOT` (CRITICAL)

**File**: `.specify/scripts/bash/create-new-feature.sh`  
**Line**: 131

**Current**:
```bash
cd "$REPO_ROOT"

SPECS_DIR="$REPO_ROOT/specs"
```

**Recommended**:
```bash
# DO NOT change working directory - work with absolute paths
# This preserves worktree context and avoids confusion

SPECS_DIR="$REPO_ROOT/specs"  # Already absolute path, no cd needed
```

**Why This Works**:
- All subsequent operations use `$SPECS_DIR` (already absolute path)
- No need to change working directory
- Preserves worktree context if user is in worktree
- Prevents working directory state confusion

---

### Fix 2: Move Branch Creation to AFTER Spec Creation (HIGH)

**File**: `.claude/commands/sp.specify.md`  
**Section**: Step 2 (lines 52-94)

**Current Flow**:
```markdown
1. Generate short name
2. Check existing branches → Create branch → Git checkout ❌ BEFORE SPEC
3. Load template
4. Fill spec
5. Write spec.md
```

**Recommended Flow**:
```markdown
1. Generate short name
2. Check existing branches → Determine next number (but DON'T create branch)
3. Load template
4. Fill spec
5. Write spec.md ✅ SPEC EXISTS FIRST
6. Create branch matching spec directory name ✅ BRANCH FOLLOWS SPEC
```

**Changes Required**:
```markdown
## Step 0: Detect existing feature branch (UNCHANGED)

## Step 1: Generate short name (UNCHANGED)

## Step 2: Determine Paths (CHANGED - delay branch creation)
- Find highest feature number
- Calculate next number
- Set FEATURE_DIR and SPEC_FILE paths
- **DO NOT create git branch yet**

## Step 3-5: Create Specification (UNCHANGED)

## Step 6: Create Feature Branch (NEW - after spec exists)
- Check if on main branch
- If yes: git checkout -b [branch-name]
- If already on feature branch: stay on it
- Branch name matches spec directory name
```

---

### Fix 3: Standardize on New Naming Convention (MAJOR)

**Files Affected**:
- `.specify/scripts/bash/common.sh` (find_feature_dir_by_prefix)
- `.specify/scripts/bash/create-new-feature.sh` (branch naming)
- `.claude/commands/sp.specify.md` (path expectations)

**Strategy**: Deprecate old naming (015-name) in favor of new naming (part-N-chapter-M)

**Phase 1: Update create-new-feature.sh to generate new naming**:
```bash
# For Python chapters (12-29), generate: part-4-chapter-N
# For other features, use: NNN-feature-name

if [[ "$FEATURE_DESCRIPTION" =~ [Cc]hapter[[:space:]]+([0-9]+) ]]; then
    chapter_num="${BASH_REMATCH[1]}"
    if [[ $chapter_num -ge 12 && $chapter_num -le 29 ]]; then
        # Python chapters: part-4-chapter-N
        BRANCH_NAME="part-4-chapter-$chapter_num"
    fi
else
    # Generic features: NNN-feature-name
    BRANCH_NAME="${FEATURE_NUM}-${BRANCH_SUFFIX}"
fi
```

**Phase 2: Update common.sh to prefer new naming**:
```bash
find_feature_dir_by_prefix() {
    # Try NEW convention first (part-N-chapter-M)
    if [[ "$branch_name" =~ ^part-([0-9]+)-chapter-([0-9]+)$ ]]; then
        echo "$specs_dir/part-$part-chapter-$chapter"
        return
    fi

    # Fall back to OLD convention (015-name) with deprecation warning
    if [[ "$branch_name" =~ ^([0-9]{3})- ]]; then
        >&2 echo "⚠️  WARNING: Old naming convention detected: $branch_name"
        >&2 echo "    Consider using new naming: part-N-chapter-M"
        # ... existing logic
    fi
}
```

---

### Fix 4: Preserve Working Directory State (MAJOR)

**File**: `.specify/scripts/bash/create-new-feature.sh`

**Pattern**: Use `pushd/popd` or explicit directory restoration

**Option A: pushd/popd (Recommended)**:
```bash
# Save current directory
pushd "$REPO_ROOT" > /dev/null

# ... do work ...

# Restore original directory
popd > /dev/null
```

**Option B: Absolute Paths Only (Simpler)**:
```bash
# Don't change directory at all - use absolute paths
# Already have $REPO_ROOT, so:

SPECS_DIR="$REPO_ROOT/specs"
FEATURE_DIR="$SPECS_DIR/$BRANCH_NAME"
SPEC_FILE="$FEATURE_DIR/spec.md"

# All paths are absolute, no cd needed
```

**Recommendation**: **Option B** - Simpler and avoids working directory state entirely.

---

## Testing Plan

### Test Case 1: Worktree Context Preservation

**Setup**:
```bash
cd /main-repo
git worktree add /worktrees/test-feature
cd /worktrees/test-feature
```

**Execute**:
```bash
/sp.specify "Test feature description"
```

**Expected**:
- `pwd` remains `/worktrees/test-feature` after script execution ✅
- Spec created at `/main-repo/specs/001-test-feature/spec.md` ✅
- Branch created in worktree context ✅

**Current Behavior** (BEFORE FIX):
- `pwd` changes to `/main-repo` ❌
- Spec created at `/main-repo/specs/001-test-feature/spec.md` ✅
- Branch created but working directory confused ❌

---

### Test Case 2: Python Chapter Naming Convention

**Setup**:
```bash
cd /main-repo
git checkout main
```

**Execute**:
```bash
/sp.python-chapter 15
```

**Expected**:
- Branch name: `part-4-chapter-15` ✅
- Spec directory: `specs/part-4-chapter-15/` ✅
- check-prerequisites.sh returns: `{"FEATURE_DIR": ".../specs/part-4-chapter-15"}` ✅

**Current Behavior** (BEFORE FIX):
- Branch name: `015-operators-keywords-variables` ❌
- Spec directory: `specs/part-4-chapter-15/` ✅ (from /sp.specify logic)
- check-prerequisites.sh returns: `{"FEATURE_DIR": ".../specs/015-operators-..."}` ❌

---

### Test Case 3: Spec-First Branch Creation

**Setup**:
```bash
cd /main-repo
git checkout main
```

**Execute**:
```bash
/sp.specify "New authentication system"
```

**Expected Flow**:
1. Determine paths (no branch creation) ✅
2. Create spec.md ✅
3. Spec exists at `specs/002-authentication-system/spec.md` ✅
4. Create branch `002-authentication-system` AFTER spec exists ✅
5. Branch points to commit with spec.md ✅

**Current Flow** (BEFORE FIX):
1. Create branch `002-authentication-system` BEFORE spec ❌
2. Create spec.md ✅
3. Branch exists but may not include spec.md commit ❌

---

## Summary

### Root Cause Identified ✅

**Primary Issue**: `create-new-feature.sh` executes `cd $REPO_ROOT` (line 131), which:
- Abandons worktree context when run from worktree
- Causes working directory confusion
- Leads to operations in root instead of worktree

**Secondary Issues**:
- Branch created before spec.md exists (violates spec-first principle)
- Path naming mismatch (old vs new convention)
- No working directory state restoration

### User's Intuition Confirmed ✅

> "Initially I assumed it was in a high wrapper command like python-chapter but now I suspect it's a deep down root cause"

**Analysis**: ✅ **CORRECT** - The issue is NOT in `/sp.python-chapter` (wrapper), it's in `create-new-feature.sh` (deep infrastructure script).

### Recommended Priority

1. **Fix 1 (CRITICAL)**: Remove premature `cd $REPO_ROOT` - use absolute paths only
2. **Fix 2 (HIGH)**: Move branch creation to AFTER spec creation (spec-first compliance)
3. **Fix 3 (MAJOR)**: Standardize on new naming convention (part-N-chapter-M)
4. **Fix 4 (MAJOR)**: Document working directory expectations (or eliminate need via absolute paths)

### Estimated Effort

- **Fix 1**: 15 minutes (remove one line, verify absolute paths work)
- **Fix 2**: 30 minutes (restructure sp.specify.md steps 2-6)
- **Fix 3**: 1-2 hours (update multiple scripts, test both conventions)
- **Fix 4**: 10 minutes (documentation only if Fix 1 applied)

**Total**: ~2.5 hours to resolve completely

---

## Next Steps

**Immediate Action**:
1. Apply Fix 1 (remove `cd $REPO_ROOT`)
2. Test with worktree context
3. Verify no regressions

**Follow-up Actions**:
1. Apply Fix 2 (spec-first branch creation)
2. Apply Fix 3 (naming convention standardization)
3. Comprehensive testing with all 3 fixes

**Validation**:
1. Run Test Cases 1-3
2. Execute `/sp.python-chapter 16` end-to-end
3. Verify worktree workflows work correctly

---

**Document Status**: Analysis Complete  
**Next**: User approval to apply fixes

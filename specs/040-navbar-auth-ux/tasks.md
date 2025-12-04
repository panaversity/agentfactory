# Tasks: Navbar Authentication UX Refinement

**Feature**: 040-navbar-auth-ux
**Generated**: 2025-12-04
**Status**: Ready for Implementation

## Task Overview

| # | Task | File | Status |
|---|------|------|--------|
| 1 | Change "Get Started" → "Sign Up" | NavbarAuth/index.tsx | Pending |
| 2 | Add search-to-auth spacing (desktop) | SearchBar/styles.module.css | Pending |
| 3 | Add search-to-auth spacing (mobile) | SearchBar/styles.module.css | Pending |
| 4 | Build verification | - | Pending |
| 5 | Visual testing (light/dark modes) | - | Pending |

## Tasks

### Task 1: Label Change (FR-001)

**Priority**: P1
**File**: `book-source/src/components/NavbarAuth/index.tsx`
**Line**: ~188

**Action**: Change button text from "Get Started" to "Sign Up"

**Acceptance**: Button displays "Sign Up" text

---

### Task 2: Desktop Spacing (FR-004)

**Priority**: P1
**File**: `book-source/src/components/SearchBar/styles.module.css`

**Action**: Add `margin-right: 1.5rem` to `.searchContainer`

**Acceptance**: 24px gap visible between search and auth buttons on desktop

---

### Task 3: Mobile Spacing (FR-005)

**Priority**: P1
**File**: `book-source/src/components/SearchBar/styles.module.css`

**Action**: Add mobile media query with `margin-right: 0.75rem`

**Acceptance**: 12px gap on viewports < 996px

---

### Task 4: Build Verification (NFR-001, NFR-002)

**Priority**: P1
**Command**: `cd book-source && npm run build`

**Acceptance**: 
- Build succeeds with no errors
- Bundle size increase < 1KB

---

### Task 5: Visual Testing (FR-003, FR-006, FR-007, FR-008)

**Priority**: P1
**Method**: Manual inspection

**Checklist**:
- [ ] Sign In appears LEFT of Sign Up
- [ ] Sign Up has primary (filled) styling
- [ ] Sign In has secondary (ghost) styling
- [ ] Light mode: correct hierarchy
- [ ] Dark mode: correct hierarchy
- [ ] Mobile: correct spacing

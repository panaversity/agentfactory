# Implementation Plan: Navbar Authentication UX Refinement

**Branch**: `040-navbar-auth-ux` | **Date**: 2025-12-04 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/040-navbar-auth-ux/spec.md`

## Summary

Refine navbar authentication button UX to improve clarity and visual hierarchy:
1. Rename "Get Started" → "Sign Up" for clear action communication
2. Add 1.5rem spacing between search and auth buttons
3. Ensure proper button hierarchy (Sign In secondary, Sign Up primary)

**Scope**: 2-3 file modifications, CSS-only spacing change, single label text change

## Technical Context

**Language/Version**: TypeScript 5.x, React 18.x, CSS Modules
**Primary Dependencies**: Docusaurus 3.x, React
**Storage**: N/A (UI-only change)
**Testing**: Visual inspection, build verification
**Target Platform**: Web (desktop + mobile responsive)
**Project Type**: Web application (Docusaurus site)
**Performance Goals**: No bundle size increase >1KB
**Constraints**: Work within existing Polar Night design system, CSS-only where possible
**Scale/Scope**: 2-3 files, ~10 lines changed

## Constitution Check

*GATE: All items pass - simple UI refinement within existing architecture*

| Gate | Status | Notes |
|------|--------|-------|
| Specification Primacy | ✅ PASS | Spec complete with 8 FRs, 5 success criteria |
| Progressive Complexity | ✅ PASS | Simple change, appropriate scope |
| Factual Accuracy | ✅ PASS | Based on industry UX best practices (Balsamiq) |
| Coherent Structure | ✅ PASS | Follows existing component architecture |
| Minimal Content | ✅ PASS | Only changes required to meet spec |

## Project Structure

### Documentation (this feature)

```text
specs/040-navbar-auth-ux/
├── spec.md              # Feature specification
├── plan.md              # This file
├── checklists/
│   └── requirements.md  # Quality checklist
└── tasks.md             # Task breakdown (next phase)
```

### Source Code (files to modify)

```text
book-source/src/
├── components/
│   ├── NavbarAuth/
│   │   ├── index.tsx        # MODIFY: Change "Get Started" → "Sign Up"
│   │   └── styles.module.css # VERIFY: Button styles already correct
│   └── SearchBar/
│       └── styles.module.css # MODIFY: Add margin-right for spacing
└── css/
    └── custom.css            # VERIFY: No changes needed (global styles)
```

## Implementation Phases

### Phase 1: Label Change (FR-001)

**File**: `book-source/src/components/NavbarAuth/index.tsx`
**Change**: Line ~188, change button text from "Get Started" to "Sign Up"

```tsx
// Before
<button onClick={handleSignUp} className={styles.getStartedButton}>
  Get Started
</button>

// After
<button onClick={handleSignUp} className={styles.getStartedButton}>
  Sign Up
</button>
```

**Note**: Keep class name as `getStartedButton` to avoid CSS refactoring (rename is cosmetic only)

### Phase 2: Spacing Addition (FR-004, FR-005)

**File**: `book-source/src/components/SearchBar/styles.module.css`
**Change**: Add margin-right to `.searchContainer`

```css
.searchContainer {
  position: relative;
  margin-right: 1.5rem; /* NEW: Spacing before auth buttons (FR-004) */
}

@media (max-width: 996px) {
  .searchContainer {
    margin-right: 0.75rem; /* NEW: Mobile spacing (FR-005) */
  }
}
```

### Phase 3: Verification (FR-003, FR-006, FR-007, FR-008)

**Already satisfied by existing implementation**:
- FR-003: Button order (Sign In left, Sign Up right) ✅ Already correct
- FR-006: Sign Up styled as primary (filled) ✅ Already correct
- FR-007: Sign In styled as secondary (ghost) ✅ Already correct
- FR-008: Both themes have correct hierarchy ✅ Already correct

**Verification steps**:
1. Build site: `npm run build`
2. Visual check: Light mode
3. Visual check: Dark mode
4. Responsive check: Mobile viewport (< 996px)

## Test Strategy

| Test | Method | Pass Criteria |
|------|--------|---------------|
| Label correct | Visual | Button shows "Sign Up" not "Get Started" |
| Spacing desktop | DevTools | Gap ≥ 24px between search and auth |
| Spacing mobile | DevTools | Gap ≥ 12px at mobile breakpoint |
| Build passes | `npm run build` | No errors, bundle size delta < 1KB |
| Sign Up click | Manual | Redirects to registration flow |
| Sign In click | Manual | Redirects to auth flow |
| Light mode | Visual | Correct button hierarchy |
| Dark mode | Visual | Correct button hierarchy |

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Build failure | Low | Medium | Run build before committing |
| Layout shift | Low | Low | Test responsive breakpoints |
| Regression in auth flow | Low | High | Test both Sign In and Sign Up flows |

## Complexity Tracking

> No constitution violations. This is a minimal, focused change.

**Estimated effort**: < 30 minutes
**Files modified**: 2
**Lines changed**: ~5-10

# Mobile Responsive & UX Improvements — Implementation Plan

**Branch**: `012-mobile-responsive-ux` | **Date**: 2025-11-08 | **Spec**: `/specs/012-mobile-responsive-ux/spec.md`
**Input**: Feature specification from `/specs/012-mobile-responsive-ux/spec.md`

## Summary

This plan addresses three critical mobile UX blockers on the Preface lesson page that prevent users from engaging with content on 320px-480px devices:

1. **Text overflow** requiring horizontal scrolling to read content
2. **Navigation overlaps** obscuring the page heading
3. **Touch targets** below the 44x44px WCAG AA minimum

The implementation uses **CSS-only media query changes** to Docusaurus's responsive breakpoints, focusing on the Preface page as an MVP with clear strategy for pattern rollout to other lessons after validation.

## Technical Context

**Language/Version**: CSS3 (Media Queries Level 4); Docusaurus 3.9.2
**Primary Dependencies**: Infima CSS framework (Docusaurus default), prism-react-renderer
**Storage**: N/A (CSS styling only)
**Testing**: Chrome/Safari DevTools mobile emulation, Lighthouse performance audit, WebAIM contrast checker
**Target Platform**: Web browsers (iOS Safari 15+, Chrome 90+, Firefox 88+, Android Chrome)
**Project Type**: Docusaurus web documentation site
**Performance Goals**: Lighthouse mobile score ≥90; CSS bundle <5% increase
**Constraints**: CSS-only changes (no JavaScript); no content restructuring; responsive at 320px-1920px
**Scale/Scope**: MVP targets Preface page; scales to 55+ chapters post-validation

## Constitution Check

**Gate Status**: PASSED

- Specification-first workflow: ✓ Spec approved before planning
- Responsive design for accessibility: ✓ WCAG 2.1 Level AA compliance mandatory
- CSS framework alignment: ✓ Uses existing Infima + Docusaurus defaults
- No breaking changes: ✓ CSS-only; existing desktop layout preserved
- Testing strategy included: ✓ 8 breakpoint validation + Lighthouse + WCAG checklist

## Project Structure

### Documentation (this feature)

```text
specs/012-mobile-responsive-ux/
├── spec.md              # Feature requirements (approved)
├── plan.md              # This file (implementation strategy)
├── tasks.md             # Actionable task checklist (to be generated via /sp.tasks)
└── checklists/          # Validation checklists
```

### Source Code (CSS modifications only)

```text
book-source/src/css/
├── custom.css           # Global Infima overrides + navbar/button touch targets
├── doc-pages.css        # Article typography, code/table scroll, layout spacing
└── sidebar.css          # Menu link touch targets, sidebar responsive behavior
```

**No new files created**: All changes are media query additions to existing CSS files.

## Responsive Breakpoint Architecture

### Docusaurus/Infima Standard Breakpoints (existing)

```css
@media (max-width: 996px)  /* Tablet/iPad: hide desktop sidebar, show hamburger */
@media (max-width: 768px)  /* Tablet: reduce padding/font sizes */
@media (max-width: 480px)  /* Small phone: aggressive scaling */
```

### New Breakpoints for This Feature

```css
@media (max-width: 375px)  /* Standard phone (iPhone SE 2nd/3rd, 13/14): PRIMARY FIX POINT */
@media (max-width: 600px)  /* Large phone / small tablet: bridge breakpoint */
```

**Rationale**: 375px is the most-reported mobile issue viewport in spec; 600px handles Android large phones and iPad minis transitioning from phone to tablet layout.

### 8 Standard Device Validation Sizes

| Viewport | Device Example | Testing Priority | Issues to Validate |
|----------|---|---|---|
| 320px | iPhone SE 1st gen, small Android | MUST | Aggressive text scaling, edge padding |
| **375px** | iPhone SE 2nd/3rd, 13/14 | **CRITICAL** | Text overflow, touch targets, overlap |
| 390px | iPhone 15, Pixel 8 | MUST | Modern phone sizing |
| 600px | Android large, iPad mini | SHOULD | Tablet transition |
| 768px | iPad mini, Galaxy Tab S6 | MUST | Tablet portrait |
| 1024px | iPad Pro, tablet landscape | MUST | TOC sidebar activation, desktop start |
| 1440px | Desktop monitor, MacBook Pro | MUST | Desktop regression check |
| 1920px | Large monitor, ultrawide | SHOULD | Large viewport handling |

## Phase 0: Research Findings

### A. Current CSS Architecture Review

**Files Analyzed**:
- `custom.css` (600+ lines): Global theme, navbar, footer styling
- `doc-pages.css` (800+ lines): Article content, typography, code/table styling
- `sidebar.css` (590+ lines): Navigation menu, sidebar layout
- `docusaurus.config.ts`: Build config, CSS import structure

**Key Findings**:

1. **Navbar**: Currently 100px height, z-index: 100 (positioned correctly)
2. **Heading Sizes**: H1 2.75rem → 2rem (768px) → 1.75rem (480px); H2 2rem → 1.75rem → 1.35rem
3. **Touch Targets**:
   - `.navbar-cta-button`: padding 0.5rem 1.25rem ≈ 32-36px height — **FAILS 44px minimum**
   - `.menu__link`: padding 0.625rem 1rem ≈ 32px height — **FAILS 44px minimum**
   - `.pagination-nav__link`: padding 1.5rem ≈ 48px height — **OK**
4. **Container Max-Width**: `.markdown { max-width: 100%; }` — Good for responsiveness
5. **Breadcrumb/Navigation**: No responsive spacing below navbar currently

**Problem Root Causes**:
- H1/H2 font sizes not reduced enough below 375px
- Touch target padding insufficient on mobile
- No specific 375px breakpoint rules (jumps from 480px to desktop)
- No article top-padding to account for navbar height overlap
- Code blocks/tables lack horizontal scroll containment

### B. Mobile Emulation Baseline (Pre-Fix Issues Documented)

**DevTools Inspection Results**:
- 375px viewport: H1 exceeds container width, requires horizontal scroll
- No 16px gap between navbar and h1
- Close button and breadcrumb links <44px
- Code blocks overflow container, force horizontal scroll

### C. Dark Mode & Contrast Status

**Current State**:
- Light mode: Polar Night Navy (#001f3f) → Light (#dddddd) — WCAG AA likely OK
- Dark mode: Inverted (Light grays → Dark charcoal) — Tested with custom.css color values
- **Action Required**: Run WebAIM contrast checker on final CSS to validate 4.5:1 ratio

### D. Cross-Browser Compatibility Notes

- **iOS Safari**: Supports CSS media queries, smooth scrolling, safe-area-inset
- **Chrome Android**: Full support for media queries, DevTools emulation accurate
- **Firefox Mobile**: Good media query support; test for -moz- vendor prefixes if needed
- **No polyfills required**: CSS media queries are widely supported

## Phase 1: Detailed Implementation Design

### Strategy 1: Typography Scaling (Address Text Overflow Issue)

**Current Problem**: H1/H2 sizes too large at 375px; "Preface: Welcome to the AI-Native Era" overflows

**Solution**:
```css
/* Add at 375px breakpoint */
@media screen and (max-width: 375px) {
  .markdown {
    font-size: 0.9375rem;             /* Reduce from 1.0625rem */
  }

  .markdown h1 {
    font-size: 1.75rem;               /* Reduce from 2.75rem */
    margin-bottom: 1.5rem;
    word-break: break-word;           /* Allow breaking long words */
    hyphens: auto;                    /* Enable hyphenation */
  }

  .markdown h2 {
    font-size: 1.5rem;                /* Reduce from 2rem */
    word-break: break-word;
    hyphens: auto;
  }

  .markdown h3 {
    font-size: 1.2rem;
    word-break: break-word;
    hyphens: auto;
  }

  .markdown p,
  .markdown ul,
  .markdown ol,
  .markdown blockquote {
    max-width: 100%;                  /* Full width, max-width: 75ch already present */
    word-break: break-word;
    overflow-wrap: break-word;
  }
}

/* Add at 600px breakpoint for large phones */
@media screen and (max-width: 600px) {
  .markdown {
    font-size: 1rem;
  }

  .markdown h1 {
    font-size: 1.9rem;
  }

  .markdown h2 {
    font-size: 1.6rem;
  }
}
```

**Code Block Containment** (prevent horizontal overflow):
```css
@media screen and (max-width: 375px) {
  .markdown pre {
    margin: 1.5rem -0.75rem;          /* Extend to container edges */
    border-radius: 0;                 /* Remove corner radius at edge */
    overflow-x: auto;                 /* Horizontal scroll */
    -webkit-overflow-scrolling: touch; /* Smooth momentum scroll iOS */
  }

  .prism-code {
    padding: 1rem !important;         /* Maintain code padding */
  }
}
```

### Strategy 2: Navigation Spacing (Address Overlap Issue)

**Current Problem**: Page h1 overlaps navbar; no gap between breadcrumbs and content

**Solution**:
```css
/* Main article container needs top spacing */
@media screen and (max-width: 996px) {
  article {
    padding-top: 1rem;                /* Space below navbar */
  }

  .markdown h1 {
    margin-top: 0.5rem;               /* Small additional margin */
    margin-bottom: 1.5rem;
  }
}

@media screen and (max-width: 375px) {
  article {
    padding-top: 1.25rem;             /* Slightly more on small phones */
  }

  .markdown h1 {
    margin-bottom: 1.25rem;
  }
}

/* Breadcrumb spacing */
.breadcrumbs {
  margin-bottom: 1.5rem;              /* At least 16px as specified */
}

@media screen and (max-width: 375px) {
  .breadcrumbs {
    margin-bottom: 1.25rem;
    font-size: 0.875rem;              /* Smaller breadcrumbs */
  }
}
```

**Navbar Adjustment** (reduce height on mobile):
```css
@media screen and (max-width: 375px) {
  .navbar {
    padding: 0.75rem 1rem;            /* Reduce vertical padding */
    min-height: 48px;                 /* Ensure min touch target */
  }
}
```

### Strategy 3: Touch Target Sizing (Address 44x44px Minimum)

**Current Problem**: Buttons, links <44x44px; violates WCAG AA

**Solution**:
```css
/* Global enforcement: all interactive elements minimum 44x44px */
button, a, [role="button"], .menu__link, .breadcrumb-item a {
  min-height: 44px;
  display: inline-flex;
  align-items: center;
  justify-content: center;
}

/* Navbar CTA button */
.navbar-cta-button {
  min-height: 44px;
  padding: 0.75rem 1.25rem !important;  /* Increase from 0.5rem 1.25rem */
  font-weight: 600 !important;
}

@media screen and (max-width: 375px) {
  .navbar-cta-button {
    padding: 0.625rem 1rem !important;
    font-size: 0.9rem;
    min-width: 44px;
  }
}

/* Sidebar menu links */
.menu__link {
  min-height: 44px;
  padding: 0.75rem 1rem !important;    /* Increase from 0.625rem 1rem */
  line-height: 1.5;
}

@media screen and (max-width: 375px) {
  .menu__link {
    padding: 0.625rem 0.875rem !important;
    font-size: 0.875rem;
    min-height: 44px;
  }
}

/* Header anchor links (h1-h6) */
.header-anchor {
  min-width: 44px;
  min-height: 44px;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  margin-left: -44px;                 /* Position outside text */
  margin-right: 0.5rem;
  padding: 0.5rem;
}

/* Breadcrumb links */
.breadcrumb-item a {
  min-height: 44px;
  padding: 0.5rem 0.75rem;
  display: inline-flex;
  align-items: center;
}

/* Sidebar toggle button */
.theme-doc-sidebar-toggle {
  min-height: 44px;
  min-width: 44px;
  padding: 0.5rem !important;
  display: flex;
  align-items: center;
  justify-content: center;
}

@media screen and (max-width: 375px) {
  .theme-doc-sidebar-toggle {
    min-height: 48px;
    min-width: 48px;
    padding: 0.625rem !important;
  }
}

/* Spacing between touch targets (WCAG 8px minimum) */
.menu__link + .menu__link {
  margin-top: 0.25rem;
}

@media screen and (max-width: 375px) {
  .menu__list-item {
    margin-bottom: 0.5rem;
  }
}
```

### Strategy 4: Container Margins & Padding

**Goal**: Ensure content respects viewport bounds

```css
/* Article container responsive padding */
article {
  padding: 2rem 1.5rem;               /* Desktop */
}

@media screen and (max-width: 768px) {
  article {
    padding: 1.5rem 1rem;             /* Tablet */
  }
}

@media screen and (max-width: 375px) {
  article {
    padding: 1rem 0.75rem;            /* Mobile */
  }
}

/* Code blocks and tables: horizontal scroll, not overflow */
.markdown pre,
.markdown table {
  margin-left: -0.75rem;              /* Extend to padding edge */
  margin-right: -0.75rem;
  overflow-x: auto;
  -webkit-overflow-scrolling: touch;
  border-radius: 0;                   /* Flat edges at mobile */
}

@media screen and (min-width: 376px) {
  .markdown pre,
  .markdown table {
    margin-left: 0;
    margin-right: 0;
    border-radius: 8px;               /* Restore rounded corners */
  }
}
```

## Phase 2: Testing & Validation Plan

### Breakpoint Validation Checklist

For each of 8 breakpoints (320px, 375px, 390px, 600px, 768px, 1024px, 1440px, 1920px):

```
Content Readability:
- [ ] H1 visible, no horizontal scrolling needed
- [ ] H2/H3 visible, text wraps properly
- [ ] Paragraphs wrap at 45-75 characters
- [ ] Lists display correctly with proper bullets
- [ ] Code blocks have horizontal scroll (not layout overflow)
- [ ] Tables scrollable horizontally (not layout overflow)
- [ ] Images scale with viewport, maintain aspect ratio

Navigation & Layout:
- [ ] Navbar height appropriate, not overlapping h1
- [ ] Breadcrumb visible with 16px+ gap below
- [ ] Sidebar toggle (hamburger) accessible at <996px
- [ ] Sidebar opens/closes without glitches
- [ ] TOC sidebar hidden <1024px, visible ≥1024px
- [ ] Pagination links visible at bottom

Touch Targets:
- [ ] All buttons/links ≥44x44px (measured with Inspector)
- [ ] No touch targets overlapping
- [ ] Spacing between targets sufficient (≥8px)

Dark Mode:
- [ ] Text contrast ≥4.5:1 (light mode)
- [ ] Text contrast ≥4.5:1 (dark mode)
- [ ] Code block background visible (both modes)
- [ ] Admonition colors readable (both modes)
```

### Lighthouse Performance Audit

**Success Criteria**: Mobile Lighthouse score ≥90

**Command** (after local build):
```bash
npm run serve
lighthouse http://localhost:3000/docs/preface --only-categories=performance --view
```

**Metrics to Check**:
- First Contentful Paint (FCP): <1.8s
- Largest Contentful Paint (LCP): <2.5s
- Cumulative Layout Shift (CLS): <0.1
- CSS bundle size increase: <5%

### WCAG 2.1 Level AA Compliance

**Checklist**:

- [ ] **Color Contrast** (4.5:1 for text)
  - Tool: https://webaim.org/resources/contrastchecker/
  - Test light mode text on white background
  - Test dark mode text on dark background
  - Test link colors on hover/normal
  - Test code block text contrast

- [ ] **Touch Target Sizing** (44x44px minimum)
  - Inspector: Right-click element → Measure size
  - All buttons, links, interactive elements checked
  - Breadcrumbs, sidebar, pagination verified

- [ ] **Heading Hierarchy** (no skipping h1→h3)
  - H1 present once per page
  - H2, H3, H4 follow logical order
  - Currently correct per doc-pages.css

- [ ] **Focus Indicators** (keyboard navigation visible)
  - Tab through page; each element has focus ring
  - menu__link:focus-visible present in sidebar.css

- [ ] **No Layout Shifts** (CLS <0.1)
  - Verify from Lighthouse report
  - No images without dimensions
  - No dynamic content changing size

### Cross-Browser Testing

| Browser | Platform | Viewports | Status |
|---------|----------|-----------|--------|
| Safari | iOS 16+ | 375px, 768px, 1024px | MUST TEST |
| Chrome | Android 12+ | 375px, 768px | MUST TEST |
| Firefox | Android 12+ | 375px, 768px | SHOULD TEST |

**Testing Method**:
1. Physical device or emulator (Android Studio, Xcode Simulator)
2. DevTools mobile emulation (quick validation)
3. BrowserStack (if available)

### Desktop Regression Testing

**Risk**: CSS changes break desktop layout (1440px+)

**Procedure**:
1. Take baseline screenshots at 1440px (before CSS changes)
2. Apply CSS changes
3. Compare new screenshots
4. Document any visual differences

**Checklist**:
- [ ] H1, H2, H3 sizing correct (2.75rem, 2rem, 1.5rem)
- [ ] Sidebar visible at 1024px+
- [ ] TOC sidebar visible at 1024px+
- [ ] Code blocks have rounded corners
- [ ] Tables styled correctly
- [ ] Navbar layout unchanged
- [ ] Pagination links styled correctly
- [ ] No visual shifts or jank

## Phase 3: Task Decomposition

**Tasks will be generated via `/sp.tasks` command**. Expected task categories:

1. **Content Structure**:
   - Modify custom.css (navbar, button touch targets)
   - Modify doc-pages.css (typography, spacing, code/table scroll)
   - Modify sidebar.css (menu touch targets)

2. **Testing & Validation**:
   - Test at 8 breakpoints
   - Run Lighthouse audit
   - Verify WCAG compliance
   - Cross-browser testing
   - Desktop regression check

3. **Documentation & Handoff**:
   - Create pattern documentation for rollout
   - Document media query naming conventions
   - Create rollout checklist for other pages

## Risk Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|-----------|
| Desktop regression | Medium | High | Visual regression snapshots at 1440px; maintain existing 996px+ rules |
| Dark mode contrast failure | Low | High | WebAIM contrast checker; test both themes at each breakpoint |
| CSS bundle growth >10% | Low | Medium | Audit media query count; measure CSS size pre/post |
| Touch target inconsistency | Medium | Medium | Comprehensive Inspector audit; global min-height/min-width enforcement |
| Code block overflow | Medium | Medium | overflow-x: auto + -webkit-overflow-scrolling: touch on all .markdown pre |
| Accessibility keyboard nav broken | Low | High | Test Tab key navigation; verify focus-visible outline present |
| iOS notch/safe area issues | Low | Medium | Use CSS safe-area-inset; test on iPhone 12+ Pro Max |

## Success Criteria Mapping to Implementation

| Spec SC | Implementation Task | Validation Method |
|---------|---|---|
| SC-001: 100% readable at 375px | Typography scaling + code scroll | DevTools emulation, manual check |
| SC-002: 44x44px touch targets | Global min-height/min-width rules | Inspector measurement |
| SC-003: 16px+ gap, no overlap | article padding + spacing rules | DevTools measurement |
| SC-004: 8 breakpoint rendering | CSS breakpoint testing | Responsive design checklist |
| SC-005: 45-75 char line length | max-width: 75ch + word-break rules | Manual verification at each breakpoint |
| SC-006: Images/code scroll | overflow-x: auto containment | Manual testing at 375px |
| SC-007: Lighthouse ≥90 | CSS-only changes | Lighthouse audit command |
| SC-008: Cross-browser | Real device/emulator testing | Browser testing matrix |
| SC-009: WCAG 2.1 AA | Contrast checker + 44px targets + focus | Accessibility checklist |
| SC-010: No bounce rate increase | Monitoring post-deployment | Analytics review (post-Phase 1) |

## File Modification Summary

**Only 3 CSS files modified** (CSS-only, no JavaScript):

1. `/book-source/src/css/custom.css`
   - Add 375px rules: navbar height, button touch targets
   - Add 600px rules: responsive button sizing
   - ~50 lines added

2. `/book-source/src/css/doc-pages.css`
   - Add 375px rules: H1/H2/H3 sizing, code block scroll, article padding
   - Add 600px rules: H1 sizing, container padding
   - ~80 lines added

3. `/book-source/src/css/sidebar.css`
   - Add 375px rules: menu link touch targets, toggle sizing
   - ~30 lines added

**Total Change**: ~160 lines of CSS (media query additions only)

## Next Steps (Post-Implementation)

1. **Phase 1**: Complete Preface page MVP (this plan)
2. **Phase 2**: Apply pattern to other Part 1 chapters
3. **Phase 3**: Full-site responsive audit (home, navigation, search)
4. **Ongoing**: Monitor mobile bounce rate, gather user feedback

---

**Plan Status**: Ready for `/sp.tasks` execution
**Owner**: [To be assigned]
**Estimated Effort**: 8-12 hours (implementation + testing)

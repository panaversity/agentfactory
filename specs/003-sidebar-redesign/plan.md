# Sidebar Redesign Implementation Plan

**Feature**: sidebar-redesign
**Spec Version**: 1.0.0
**Plan Version**: 1.0.0
**Created**: 2025-11-29

---

## 1. Architecture Overview

This is a **CSS-only redesign** of the Docusaurus sidebar. No new React components are needed.

```
┌─────────────────────────────────────────────────────────────┐
│                     File Structure                          │
├─────────────────────────────────────────────────────────────┤
│  src/css/                                                   │
│  ├── custom.css         [MODIFY: Add sidebar design tokens] │
│  └── sidebar.css        [REWRITE: New IFK-aligned styles]   │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Implementation Phases

### Phase 1: Design Token Extension

**Goal**: Add sidebar-specific design tokens to custom.css

**Tasks**:
1. Add sidebar color variables under `:root`
2. Add sidebar animation keyframes
3. Ensure dark/light mode variants

**Files**: `src/css/custom.css`

### Phase 2: Sidebar Container Redesign

**Goal**: Restyle the main sidebar container

**Tasks**:
1. Replace background with IFK dark gradient
2. Add subtle border glow effect
3. Implement backdrop blur for depth
4. Add optional scan line animation

**CSS Selectors**: `.theme-doc-sidebar-container`, `.sidebar`

### Phase 3: Category/Module Headers

**Goal**: Make module headers visually distinctive

**Tasks**:
1. Bold uppercase typography with IBM Plex Mono
2. Subtle glow effect on active categories
3. Smooth expand/collapse animation
4. Remove default arrow, add custom indicator

**CSS Selectors**: `.menu__link--sublist`, `.menu__list-item--collapsed`

### Phase 4: Menu Item Styling

**Goal**: Style individual navigation items

**Tasks**:
1. Base item styling with IFK colors
2. Hover state with subtle lift + background
3. Active state with cyan left border accent
4. Focus-visible for accessibility

**CSS Selectors**: `.menu__link`, `.menu__link:hover`, `.menu__link--active`

### Phase 5: Progress Indicators (Visual Only)

**Goal**: Add visual patterns for future progress tracking

**Tasks**:
1. Add dot placeholder styles (can be used with future classes)
2. Style for completed, in-progress, not-started states
3. Position indicators to left of menu items

**CSS Classes**: `.sidebar-progress-*` (new utility classes)

### Phase 6: Responsive & Accessibility

**Goal**: Ensure mobile usability and accessibility

**Tasks**:
1. Mobile-specific sizing (44px touch targets)
2. Tablet breakpoint adjustments
3. Reduced motion support
4. Focus states for keyboard navigation

**Media Queries**: `@media (max-width: 996px)`, `@media (prefers-reduced-motion)`

### Phase 7: Light Mode Adaptation

**Goal**: Ensure light mode works correctly

**Tasks**:
1. Override dark tokens for light theme
2. Maintain contrast ratios
3. Test readability

**Selectors**: `[data-theme='light'] .theme-doc-sidebar-container`

---

## 3. CSS Architecture

```css
/* ============================================================================
   SIDEBAR STYLING - Industrial-Kinetic Futurism
   ============================================================================ */

/* === SECTION 1: Design Tokens === */
:root {
  --ifk-sidebar-*: ...;
}

/* === SECTION 2: Container === */
.theme-doc-sidebar-container { ... }

/* === SECTION 3: Categories === */
.menu__link--sublist { ... }

/* === SECTION 4: Items === */
.menu__link { ... }
.menu__link:hover { ... }
.menu__link--active { ... }

/* === SECTION 5: Progress Indicators === */
.sidebar-progress-complete { ... }
.sidebar-progress-current { ... }
.sidebar-progress-pending { ... }

/* === SECTION 6: Animations === */
@keyframes sidebarGlow { ... }
@keyframes sidebarScanLine { ... }

/* === SECTION 7: Responsive === */
@media (max-width: 996px) { ... }
@media (prefers-reduced-motion: reduce) { ... }

/* === SECTION 8: Light Mode === */
[data-theme='light'] { ... }
```

---

## 4. Quality Checklist

Before marking complete:

- [ ] All IFK design tokens used consistently
- [ ] Dark mode renders correctly
- [ ] Light mode renders correctly
- [ ] Mobile layout works (375px-996px)
- [ ] Touch targets ≥44px
- [ ] Keyboard navigation works
- [ ] Focus states visible
- [ ] Reduced motion respected
- [ ] Build passes (`npm run build`)
- [ ] No console errors

---

## 5. Estimated Scope

| Phase | Complexity | Lines of CSS |
|-------|------------|--------------|
| Phase 1: Tokens | Low | ~30 |
| Phase 2: Container | Medium | ~50 |
| Phase 3: Categories | Medium | ~60 |
| Phase 4: Items | Medium | ~80 |
| Phase 5: Progress | Low | ~40 |
| Phase 6: Responsive | Medium | ~50 |
| Phase 7: Light Mode | Low | ~30 |
| **Total** | | **~340 lines** |

---

## 6. Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| Docusaurus class changes | High | Pin version, use stable selectors |
| CSS specificity conflicts | Medium | Use appropriate specificity, `!important` sparingly |
| Performance regression | Low | CSS-only, no JS animations |
| Accessibility regression | Medium | Test with screen reader, keyboard |

---

## 7. Dependencies

- Docusaurus 3.x theme structure
- Existing IFK design tokens
- IBM Plex font family (already loaded)
- No new npm packages required

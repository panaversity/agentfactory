# Sidebar Redesign Specification

**Feature**: Learning Platform Sidebar Redesign
**Version**: 1.0.0
**Status**: Draft
**Created**: 2025-11-29
**Stakeholder**: Students (learning content delivery)

---

## 1. Intent

### Problem Statement

The current sidebar provides basic navigation but lacks the distinctive, engaging design quality that matches the homepage's Industrial-Kinetic Futurism aesthetic. Readers navigating the course need a sidebar that:

1. **Enhances learning context** - Shows module progression and current position
2. **Matches platform identity** - Aligns with IFK design system (cyan/amber accents, dark industrial theme)
3. **Improves usability** - Clear hierarchy, easy navigation, progress indicators
4. **Creates memorable experience** - Distinctive enough to be remembered as part of the RoboLearn brand

### Solution Overview

Redesign the Docusaurus sidebar to integrate with the Industrial-Kinetic Futurism design system, adding:
- Module-aware navigation with visual hierarchy
- Progress indicators showing lesson completion
- Ambient animations matching the platform aesthetic
- Responsive design for mobile learning

---

## 2. Evals (Success Criteria)

| ID | Criterion | Measurement | Target |
|----|-----------|-------------|--------|
| E1 | Visual consistency with homepage | Design review against IFK design system | 100% alignment |
| E2 | Module hierarchy clarity | Users can identify current module/lesson in <2 seconds | Pass |
| E3 | Build success | `npm run build` completes without errors | Pass |
| E4 | Mobile usability | Touch targets ≥44px, navigation works on 375px width | Pass |
| E5 | Performance | No layout shift, CSS-only animations | CLS < 0.1 |
| E6 | Dark/Light mode support | Both modes render correctly | Pass |

---

## 3. Design Requirements

### Visual Design

**Aesthetic Direction**: Industrial-Kinetic Futurism
- Dark backgrounds with subtle depth (--ifk-bg-primary: #141418)
- Cyan accent for active/hover states (--ifk-cyan: #00d4ff)
- Amber accent for progress/completion (--ifk-amber: #ff9500)
- Typography: IBM Plex Mono for labels, IBM Plex Sans for content

**Key Visual Elements**:
1. **Module Headers**: Bold, uppercase, with subtle glow effect
2. **Active Item**: Cyan left border with background highlight
3. **Progress Dots**: Small amber indicators for completed lessons
4. **Hover States**: Subtle transform + glow transition
5. **Scan Line Animation**: Optional ambient animation (respects reduced-motion)

### Information Architecture

```
ROBOLEARN PLATFORM [header]
├── Module 1: ROS 2 [expandable category]
│   ├── ● Introduction [completed - amber dot]
│   ├── ◐ Nodes & Topics [in-progress - half amber]
│   └── ○ Services [not started - outline]
├── Module 2: Gazebo [expandable]
├── Module 3: Isaac [expandable]
└── Module 4: VLA [expandable]
```

### Interaction Design

1. **Category Expansion**: Smooth collapse/expand with height animation
2. **Active State**: Clear visual distinction with cyan accent
3. **Hover Effects**: Subtle lift (translateY) + background highlight
4. **Touch Targets**: Minimum 44px height on mobile
5. **Keyboard Navigation**: Full focus-visible support

---

## 4. Technical Requirements

### Constraints

- **Framework**: Docusaurus 3.x with Infima CSS framework
- **Styling**: CSS modules + global CSS (no runtime JS for styling)
- **Accessibility**: WCAG 2.1 AA compliance
- **Performance**: No JavaScript animations (CSS-only)
- **File Structure**: Modify existing `sidebar.css`, extend design tokens in `custom.css`

### Non-Goals

- NOT implementing actual progress tracking backend
- NOT changing sidebar structure/configuration (sidebars.ts)
- NOT adding new React components (CSS-only redesign)
- NOT implementing authentication-gated features

### Dependencies

- Existing IFK design tokens in custom.css
- Docusaurus theme classes (menu__link, menu__list-item, etc.)
- IBM Plex font family (already loaded)

---

## 5. Acceptance Tests

| Test | Steps | Expected Result |
|------|-------|-----------------|
| AT1 | Open site in Chrome, navigate to any lesson | Sidebar shows with IFK styling, cyan accents |
| AT2 | Click module header | Smooth expand/collapse animation |
| AT3 | Hover over menu item | Subtle transform + glow effect |
| AT4 | Toggle dark/light mode | Both modes display correctly |
| AT5 | View on 375px width | Responsive layout, touch targets 44px+ |
| AT6 | Run `npm run build` | Build succeeds, no CSS errors |
| AT7 | Tab through sidebar items | Focus states visible and logical |

---

## 6. Implementation Notes

### Files to Modify

1. `src/css/sidebar.css` - Main sidebar styling (replace/enhance)
2. `src/css/custom.css` - Add new IFK design tokens if needed

### Design Token Extensions

```css
/* Sidebar-specific IFK tokens */
--ifk-sidebar-bg: rgba(20, 20, 24, 0.98);
--ifk-sidebar-border: rgba(0, 212, 255, 0.1);
--ifk-sidebar-item-active: rgba(0, 212, 255, 0.15);
--ifk-sidebar-item-hover: rgba(0, 212, 255, 0.08);
--ifk-sidebar-progress: var(--ifk-amber);
--ifk-sidebar-completed: var(--ifk-amber-glow);
```

### CSS Structure

```css
/* 1. Container styling */
.theme-doc-sidebar-container { ... }

/* 2. Module/category headers */
.menu__link--sublist { ... }

/* 3. Individual items */
.menu__link { ... }

/* 4. Active states */
.menu__link--active { ... }

/* 5. Animations */
@keyframes sidebarScanLine { ... }

/* 6. Responsive */
@media (max-width: 996px) { ... }
```

---

## 7. Out of Scope

- Backend progress tracking system
- User authentication integration
- Sidebar search functionality
- Bookmark/favorites feature
- Custom sidebar React components

---

## 8. References

- IFK Design System: `src/css/custom.css` lines 14-235
- Current Sidebar: `src/css/sidebar.css`
- Homepage Design: `src/components/HomepageComponents/HeroSection/`
- Docusaurus Sidebar Theme: https://docusaurus.io/docs/sidebar

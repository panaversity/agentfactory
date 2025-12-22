# AI-Native Documentation — Complete Design System

**Project:** AI Native Software Development Book  
**URL:** https://ai-native.panaversity.org  
**Version:** 2.0  
**Last Updated:** December 2025

---

## Table of Contents

1. [Design Philosophy](#1-design-philosophy)
2. [Brand Identity](#2-brand-identity)
3. [Color System](#3-color-system)
4. [Typography](#4-typography)
5. [Spacing & Layout](#5-spacing--layout)
6. [Border Radius](#6-border-radius)
7. [Iconography](#7-iconography)
8. [Landing Page](#8-landing-page)
9. [Documentation Pages](#9-documentation-pages)
10. [Navigation Components](#10-navigation-components)
11. [Content Components](#11-content-components)
12. [Interactive Components](#12-interactive-components)
13. [Responsive Behavior](#13-responsive-behavior)
14. [Motion & Animation](#14-motion--animation)
15. [Accessibility](#15-accessibility)
16. [Do's and Don'ts](#16-dos-and-donts)
17. [Quick Reference](#17-quick-reference)

---

## 1. Design Philosophy

### Core Principle
**"Sharp, Technical, Content-First"**

A developer-focused documentation experience that prioritizes readability and lets technical content shine. The UI should feel invisible — professional, structured, and distraction-free.

### Design Pillars

| Pillar | Description |
|--------|-------------|
| **Content-first** | UI fades into background; content commands attention |
| **Technical precision** | Sharp edges reflect the precision of code |
| **No decoration** | Every pixel serves a purpose |
| **Scannability** | Easy navigation through dense technical material |
| **Accessibility** | High contrast, readable at all sizes, keyboard navigable |

### Reference Aesthetic

These sites exemplify the visual direction:

- **Stripe Documentation** — stripe.com/docs
- **Vercel Docs** — vercel.com/docs
- **Tailwind CSS Docs** — tailwindcss.com/docs
- **Supabase Docs** — supabase.com/docs
- **Linear App** — linear.app

---

## 2. Brand Identity

### Brand Color

| Property | Value |
|----------|-------|
| **Primary Brand Color** | Deep Teal |
| **Hex** | `#0d9488` |
| **RGB** | 13, 148, 136 |
| **HSL** | 174°, 84%, 32% |

### Brand Color Variations

| Variant | Hex | Usage |
|---------|-----|-------|
| **Teal 950 (Darkest)** | `#042f2e` | Dark mode backgrounds, deep accents |
| **Teal 900** | `#134e4a` | Dark mode hover states |
| **Teal 800** | `#115e59` | Dark mode secondary |
| **Teal 700** | `#0f766e` | Hover states |
| **Teal 600 (Primary)** | `#0d9488` | Primary actions, links, accents |
| **Teal 500** | `#14b8a6` | Light mode highlights |
| **Teal 400** | `#2dd4bf` | Dark mode primary |
| **Teal 300** | `#5eead4` | Light accents |
| **Teal 200** | `#99f6e4` | Subtle backgrounds |
| **Teal 100** | `#ccfbf1` | Very light tints |
| **Teal 50** | `#f0fdfa` | Lightest background tint |

### Logo

| Property | Specification |
|----------|---------------|
| **Style** | Text-based wordmark or simple icon + text |
| **Text** | "AI Native" |
| **Font** | Inter Bold or custom wordmark |
| **Color** | Foreground color (adapts to light/dark mode) |
| **Icon (optional)** | Simple, geometric, relates to AI/code/agents |

---

## 3. Color System

### Light Mode Palette

| Token | Hex | RGB | Usage |
|-------|-----|-----|-------|
| **Background** | `#ffffff` | 255, 255, 255 | Page background |
| **Foreground** | `#09090b` | 9, 9, 11 | Primary text |
| **Muted** | `#f4f4f5` | 244, 244, 245 | Secondary backgrounds, inline code |
| **Muted Foreground** | `#71717a` | 113, 113, 122 | Secondary text, captions |
| **Border** | `#e4e4e7` | 228, 228, 231 | Borders, dividers |
| **Primary** | `#0d9488` | 13, 148, 136 | Links, accents, CTAs |
| **Primary Foreground** | `#ffffff` | 255, 255, 255 | Text on primary color |

### Dark Mode Palette

| Token | Hex | RGB | Usage |
|-------|-----|-----|-------|
| **Background** | `#09090b` | 9, 9, 11 | Page background |
| **Foreground** | `#fafafa` | 250, 250, 250 | Primary text |
| **Muted** | `#27272a` | 39, 39, 42 | Secondary backgrounds, inline code |
| **Muted Foreground** | `#a1a1aa` | 161, 161, 170 | Secondary text, captions |
| **Border** | `#27272a` | 39, 39, 42 | Borders, dividers |
| **Primary** | `#2dd4bf` | 45, 212, 191 | Links, accents, CTAs |
| **Primary Foreground** | `#042f2e` | 4, 47, 46 | Text on primary color |

### Semantic Colors

| Token | Light Mode | Dark Mode | Usage |
|-------|------------|-----------|-------|
| **Info** | `#0ea5e9` (Sky 500) | `#38bdf8` (Sky 400) | Information callouts |
| **Success/Tip** | `#22c55e` (Green 500) | `#4ade80` (Green 400) | Success states, tips |
| **Warning** | `#f59e0b` (Amber 500) | `#fbbf24` (Amber 400) | Warning callouts |
| **Danger** | `#ef4444` (Red 500) | `#f87171` (Red 400) | Error states, danger callouts |

### Code Block Colors

| Element | Light Mode | Dark Mode |
|---------|------------|-----------|
| **Background** | `#18181b` (Zinc 900) | `#18181b` (Zinc 900) |
| **Text** | `#fafafa` (Zinc 50) | `#fafafa` (Zinc 50) |
| **Border** | `#27272a` (Zinc 800) | `#27272a` (Zinc 800) |
| **Line highlight** | `#27272a` (Zinc 800) | `#3f3f46` (Zinc 700) |

---

## 4. Typography

### Font Stack

| Type | Font Family | Fallbacks |
|------|-------------|-----------|
| **Sans (Primary)** | Inter | -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif |
| **Mono (Code)** | JetBrains Mono | Fira Code, SF Mono, Consolas, monospace |

### Type Scale

| Element | Size | Weight | Line Height | Letter Spacing |
|---------|------|--------|-------------|----------------|
| **Display** | 48px / 3rem | 700 (Bold) | 1.1 | -0.02em |
| **H1** | 36px / 2.25rem | 700 (Bold) | 1.2 | -0.02em |
| **H2** | 30px / 1.875rem | 600 (Semibold) | 1.3 | -0.02em |
| **H3** | 24px / 1.5rem | 600 (Semibold) | 1.4 | -0.02em |
| **H4** | 20px / 1.25rem | 600 (Semibold) | 1.4 | -0.02em |
| **Body** | 16px / 1rem | 400 (Regular) | 1.75 | 0 |
| **Body Small** | 14px / 0.875rem | 400 (Regular) | 1.5 | 0 |
| **Caption** | 12px / 0.75rem | 400 (Regular) | 1.5 | 0 |
| **Code** | 14px / 0.875rem | 400 (Regular) | 1.6 | 0 |

### Heading Specifications

| Element | Top Margin | Bottom Margin | Additional |
|---------|------------|---------------|------------|
| **H1** | 32px | 16px | — |
| **H2** | 48px | 16px | Border bottom (1px), 8px padding bottom |
| **H3** | 32px | 12px | — |
| **H4** | 24px | 8px | — |

---

## 5. Spacing & Layout

### Base Unit
**4px** — All spacing should be multiples of 4px

### Spacing Scale

| Token | Value | Usage |
|-------|-------|-------|
| **4px** | 0.25rem | Tight gaps, icon padding |
| **8px** | 0.5rem | Small gaps, inline spacing |
| **12px** | 0.75rem | Compact padding |
| **16px** | 1rem | Standard padding, gaps |
| **24px** | 1.5rem | Section gaps, list margins |
| **32px** | 2rem | Major section breaks |
| **48px** | 3rem | Page sections |
| **64px** | 4rem | Large section dividers |

### Content Layout

| Property | Value |
|----------|-------|
| **Max content width** | 768px (prose/reading width) |
| **Max page width** | 1440px |
| **Sidebar width** | 280px |
| **Table of Contents width** | 240px |
| **Content horizontal padding** | 24px (mobile), 48px (desktop) |

### Page Structure (Documentation)

```
┌──────────────────────────────────────────────────────────────────────────┐
│                              NAVBAR (64px)                               │
├───────────────┬──────────────────────────────────────┬───────────────────┤
│               │                                      │                   │
│   SIDEBAR     │            MAIN CONTENT              │   TABLE OF        │
│   (280px)     │            (max 768px)               │   CONTENTS        │
│               │                                      │   (240px)         │
│   - Parts     │   Breadcrumbs                        │                   │
│   - Chapters  │   Page Title                         │   On this page    │
│               │   Content...                         │   - Section 1     │
│               │                                      │   - Section 2     │
│               │   Pagination                         │   - Section 3     │
│               │                                      │                   │
├───────────────┴──────────────────────────────────────┴───────────────────┤
│                              FOOTER                                      │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 6. Border Radius

### Global Setting: **0px (None)**

All components use sharp corners. This is critical to the design aesthetic.

| Component | Radius |
|-----------|--------|
| Buttons | 0px |
| Cards | 0px |
| Inputs | 0px |
| Code blocks | 0px |
| Inline code | 0px |
| Modals/Dialogs | 0px |
| Dropdowns | 0px |
| Tooltips | 0px |
| Callouts | 0px |
| Images | 0px |
| Tabs | 0px |

**Exception:** Avatar images may use full circle (50% radius) if needed.

---

## 7. Iconography

### Icon Library: Lucide

| Property | Value |
|----------|-------|
| **Library** | Lucide Icons (lucide.dev) |
| **Style** | Outline/stroke only |
| **Stroke width** | 2px (default) |
| **Color** | Inherit from text color |

### Icon Sizes

| Size | Dimensions | Usage |
|------|------------|-------|
| **Small** | 16 × 16px | Inline text, compact UI |
| **Medium** | 20 × 20px | Buttons, navigation items |
| **Large** | 24 × 24px | Feature icons, headers |

### Common Icons

| Category | Icons |
|----------|-------|
| **Navigation** | ChevronRight, ChevronLeft, ChevronDown, ChevronUp, Menu, X, ArrowLeft, ArrowRight |
| **Actions** | Search, Copy, Check, ExternalLink, Download, Share, Edit |
| **Code** | Code, Terminal, FileCode, Braces, Play |
| **Files** | File, FileText, Folder, FolderOpen, Book, BookOpen |
| **Feedback** | Info, AlertCircle, AlertTriangle, CheckCircle, Lightbulb |
| **Theme** | Sun, Moon |
| **Social** | Github, Twitter, Linkedin, Youtube |

### Icon Guidelines

- ✅ Use outline/stroke style only
- ✅ Single color (inherits text color)
- ✅ Consistent sizing within context
- ❌ No filled icons
- ❌ No multi-color icons
- ❌ No decorative icons without function
- ❌ No mixing with other icon libraries

---

## 8. Landing Page

> **Note:** Landing page updates are deferred to Phase 2. Focus on documentation content first.

---

## 9. Documentation Pages

### Documentation Navbar

| Element | Specification |
|---------|---------------|
| **Height** | 64px |
| **Position** | Fixed/sticky at top |
| **Background** | Solid background with subtle bottom border |
| **Logo** | Left aligned, clickable to landing page |
| **Search** | Center or right, shows keyboard shortcut (⌘K) |
| **Actions** | GitHub icon, Theme toggle |

### Sidebar

| Property | Value |
|----------|-------|
| **Width** | 280px |
| **Position** | Fixed left (desktop), Drawer (mobile) |
| **Part label** | 11px, uppercase, letter-spacing 0.05em, muted color |
| **Part title** | 14px, semibold |
| **Chapter** | 14px, regular weight, 16px left indent |
| **Active state** | Bold text, 2px left border in primary color OR background tint |
| **Hover state** | Subtle background tint |
| **Collapsible** | Yes — parts expand/collapse with chevron icon |
| **Scroll** | Independent scroll within sidebar |

### Table of Contents (Right Side)

| Property | Value |
|----------|-------|
| **Width** | 240px |
| **Position** | Fixed right (desktop only) |
| **Visibility** | Hidden on screens < 1280px |
| **Label** | "On this page" — small, semibold |
| **Items** | 14px, regular weight |
| **Active state** | 2px left border in primary color, primary text |
| **Hover state** | Primary color text |
| **Behavior** | Highlights current section on scroll |

### Breadcrumbs

| Property | Value |
|----------|-------|
| **Position** | Above page title |
| **Separator** | `›` (or `/`) |
| **Font size** | 14px |
| **Color** | Muted foreground |
| **Links** | Each level clickable, hover shows primary color |

### Pagination (Previous / Next)

| Property | Value |
|----------|-------|
| **Position** | Bottom of content, above footer |
| **Layout** | Two equal-width cards side by side |
| **Border** | 1px solid border |
| **Padding** | 16px |
| **Direction label** | Small, muted ("← Previous" / "Next →") |
| **Title** | Regular weight, foreground color |
| **Hover** | Subtle background tint, border color change to primary |
| **Radius** | 0px (sharp corners) |

---

## 10. Navigation Components

### Search Modal

| Property | Value |
|----------|-------|
| **Trigger** | Click search bar OR keyboard shortcut (⌘K / Ctrl+K) |
| **Position** | Centered modal with backdrop |
| **Width** | 600px max |
| **Border radius** | 0px |
| **Keyboard** | Arrow keys navigate, Enter selects, Escape closes |
| **Highlight** | Search term highlighted in results |

### Mobile Menu (Hamburger)

| Property | Value |
|----------|-------|
| **Trigger** | Hamburger icon (Menu) in navbar |
| **Style** | Full-height drawer from left |
| **Content** | Full sidebar navigation |
| **Close** | X button or tap outside |
| **Backdrop** | Semi-transparent dark overlay |

### Theme Toggle

| Property | Value |
|----------|-------|
| **Icons** | Sun (light mode), Moon (dark mode) |
| **Size** | 20px icon in 40px touch target |
| **Behavior** | Toggles between light/dark mode |
| **Persistence** | Remember user preference |

---

## 11. Content Components

### Paragraphs

| Property | Value |
|----------|-------|
| **Font size** | 16px |
| **Line height** | 1.75 (28px) |
| **Margin** | 24px between paragraphs |
| **Max width** | 768px (natural reading width) |

### Links

| Property | Value |
|----------|-------|
| **Color** | Primary (teal) |
| **Font weight** | 500 (medium) |
| **Decoration** | Underline, 4px offset |
| **Hover** | 80% opacity |

### Lists

**Unordered List:**

| Property | Value |
|----------|-------|
| **Marker** | Disc (•) |
| **Left indent** | 24px |
| **Vertical margin** | 24px |
| **Item spacing** | 8px |

**Ordered List:**

| Property | Value |
|----------|-------|
| **Marker** | Decimal (1. 2. 3.) |
| **Left indent** | 24px |
| **Vertical margin** | 24px |
| **Item spacing** | 8px |

### Code Blocks

| Property | Value |
|----------|-------|
| **Background** | Zinc 900 (`#18181b`) |
| **Text color** | Zinc 50 (`#fafafa`) |
| **Border** | 1px solid Zinc 800 |
| **Border radius** | 0px |
| **Padding** | 16px |
| **Margin** | 24px top, 16px bottom |
| **Font** | JetBrains Mono, 14px |
| **Line height** | 1.6 |
| **Overflow** | Horizontal scroll |

**Code Block Features:**

| Feature | Specification |
|---------|---------------|
| **Language label** | Top left, small badge (e.g., "typescript") |
| **File name** | Top left, replaces language label when present |
| **Copy button** | Top right, shows "Copied!" feedback for 2s |
| **Line numbers** | Optional, muted color, right-aligned |
| **Line highlighting** | Subtle background tint on specified lines |
| **Syntax highlighting** | Use `github-dark` or `one-dark-pro` theme |

### Inline Code

| Property | Value |
|----------|-------|
| **Background** | Muted |
| **Border radius** | 0px |
| **Padding** | 2px vertical, 6px horizontal |
| **Font** | JetBrains Mono, 14px |
| **Color** | Foreground (not primary) |

### Callouts / Admonitions

| Type | Icon | Border Color | Background Tint |
|------|------|--------------|-----------------|
| **Note / Info** | Info | Sky 500 | Sky 50 (light) / Sky 950 (dark) |
| **Tip** | Lightbulb | Green 500 | Green 50 (light) / Green 950 (dark) |
| **Warning** | AlertTriangle | Amber 500 | Amber 50 (light) / Amber 950 (dark) |
| **Danger** | AlertCircle | Red 500 | Red 50 (light) / Red 950 (dark) |

| Property | Value |
|----------|-------|
| **Border** | 4px left border in type color |
| **Border radius** | 0px |
| **Padding** | 16px |
| **Margin** | 24px vertical |
| **Label** | Uppercase, small, semibold, type color |
| **Content** | Regular body text |

### Tables

| Property | Value |
|----------|-------|
| **Border** | 1px solid border on all cells |
| **Header background** | Muted |
| **Header font weight** | Bold (700) |
| **Cell padding** | 12px vertical, 16px horizontal |
| **Alternating rows** | Even rows have muted background |
| **Overflow** | Horizontal scroll wrapper on mobile |

### Blockquotes

| Property | Value |
|----------|-------|
| **Border** | 2px left border in primary color |
| **Padding** | 24px left |
| **Font style** | Italic |
| **Color** | Muted foreground |
| **Margin** | 24px vertical |

### Images

| Property | Value |
|----------|-------|
| **Max width** | 100% of content area |
| **Border** | 1px solid border |
| **Border radius** | 0px |
| **Margin** | 32px vertical |
| **Caption** | Centered, small text, muted color, 8px below image |

### Horizontal Rule

| Property | Value |
|----------|-------|
| **Style** | 1px solid border |
| **Color** | Border color |
| **Margin** | 48px vertical |

---

## 12. Interactive Components

### Buttons

**Primary Button:**

| Property | Value |
|----------|-------|
| **Background** | Primary (teal) |
| **Text** | Primary foreground (white/dark) |
| **Border** | None |
| **Padding** | 12px vertical, 24px horizontal |
| **Font weight** | 500 (medium) |
| **Hover** | 90% opacity |

**Secondary Button:**

| Property | Value |
|----------|-------|
| **Background** | Transparent |
| **Text** | Foreground |
| **Border** | 1px solid border |
| **Padding** | 12px vertical, 24px horizontal |
| **Hover** | Muted background |

**Ghost Button:**

| Property | Value |
|----------|-------|
| **Background** | Transparent |
| **Text** | Foreground |
| **Border** | None |
| **Padding** | 12px vertical, 16px horizontal |
| **Hover** | Muted background |

### Inputs

| Property | Value |
|----------|-------|
| **Background** | Background |
| **Border** | 1px solid border |
| **Border radius** | 0px |
| **Padding** | 12px |
| **Focus** | Primary color border, subtle ring |

---

## 13. Responsive Behavior

### Breakpoints

| Name | Width | Description |
|------|-------|-------------|
| **Mobile** | < 640px | Single column, drawer navigation |
| **Tablet** | 640px – 1023px | Sidebar visible, no ToC |
| **Desktop** | 1024px – 1279px | Sidebar + content, no ToC |
| **Large Desktop** | ≥ 1280px | Full layout with ToC |

### Layout Changes

| Element | Mobile | Tablet | Desktop | Large Desktop |
|---------|--------|--------|---------|---------------|
| **Sidebar** | Drawer | Fixed | Fixed | Fixed |
| **Table of Contents** | Hidden | Hidden | Hidden | Fixed |
| **Content width** | 100% | 100% | 100% | Max 768px |
| **Navbar search** | Icon only | Full | Full | Full |

---

## 14. Motion & Animation

### Principles

- **Minimal animation** — Documentation should feel instant and responsive
- **Functional motion only** — Animation serves feedback, not decoration
- **Fast durations** — Keep users focused on content

### Transition Defaults

| Property | Value |
|----------|-------|
| **Duration** | 150ms |
| **Easing** | ease-out |
| **Properties** | color, background-color, border-color, opacity |

---

## 15. Accessibility

### Color Contrast

| Requirement | Minimum Ratio |
|-------------|---------------|
| **Body text** | 4.5:1 (WCAG AA) |
| **Large text (18px+)** | 3:1 (WCAG AA) |
| **UI components** | 3:1 (WCAG AA) |

### Focus States

| Property | Value |
|----------|-------|
| **Style** | 2px outline with 2px offset |
| **Color** | Primary color |
| **Visibility** | Only on keyboard focus (`:focus-visible`) |

### Keyboard Navigation

| Requirement | Implementation |
|-------------|----------------|
| **All interactive elements** | Focusable via Tab key |
| **Tab order** | Logical left-to-right, top-to-bottom |
| **Skip link** | "Skip to content" link for screen readers |
| **Search** | Keyboard shortcut (⌘K / Ctrl+K) |
| **Modal escape** | ESC key closes modals |

---

## 16. Do's and Don'ts

### ✅ Do

- Use sharp 0px corners everywhere
- Use Inter for all body/UI text
- Use JetBrains Mono for code
- Use Lucide icons consistently
- Keep UI subtle and minimal
- Maintain high contrast for code
- Use consistent spacing (4px base)
- Let content breathe
- Use the teal accent sparingly
- Provide clear visual hierarchy

### ❌ Don't

- No rounded corners (`border-radius`)
- No decorative elements (gradients, patterns)
- No drop shadows
- No colored/filled icons
- No mixing icon libraries
- No soft/playful fonts
- No bold/heavy accents
- No excessive animation
- No small text for body (min 16px)
- No low contrast combinations

---

## 17. Quick Reference

### Configuration Summary

| Property | Value |
|----------|-------|
| **Style** | Sharp, Boxy |
| **Primary Color** | Deep Teal (`#0d9488` light / `#2dd4bf` dark) |
| **Font (Body)** | Inter |
| **Font (Code)** | JetBrains Mono |
| **Icons** | Lucide (outline only) |
| **Border Radius** | 0px everywhere |
| **Base Spacing** | 4px |
| **Content Width** | 768px max |

### Color Quick Reference

| Token | Light | Dark |
|-------|-------|------|
| **Background** | `#ffffff` | `#09090b` |
| **Foreground** | `#09090b` | `#fafafa` |
| **Muted** | `#f4f4f5` | `#27272a` |
| **Border** | `#e4e4e7` | `#27272a` |
| **Primary** | `#0d9488` | `#2dd4bf` |
| **Code BG** | `#18181b` | `#18181b` |

### One-Liner Summary

> **"A Stripe/Vercel-style documentation site — deep teal accent, sharp corners, Inter + JetBrains Mono fonts, Lucide icons, dark code blocks, and a content-first minimal UI."**

---

**End of Design System Specification**

*Document Version 2.0 — December 2025*

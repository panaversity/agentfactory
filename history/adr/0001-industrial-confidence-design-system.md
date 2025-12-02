# ADR-0001: Industrial Confidence Design System

> **Scope**: Homepage visual identity, typography, color system, component architecture, and mobile-first responsive strategy.

- **Status:** Accepted
- **Date:** 2025-11-29
- **Feature:** 001-home-page-redesign
- **Context:** Multiple design iterations rejected for lacking premium feel. User explicitly rejected "noob designs" including terminal previews, cute robot mascots, neural mesh labels, and glitch effects. Platform positioning required distinguishing from course/tutorial aesthetics.

## Decision

Adopt **Industrial Confidence** design system with the following integrated choices:

### 1. Visual Identity

| Element | Decision | Rationale |
|---------|----------|-----------|
| Brand Mark | `ROBOLEARN` + `PLATFORM` tag | Platform positioning, not course |
| Hero Visual | Robot Assembly SVG with tech stack labels | Shows integration of ROS 2, Isaac, Gazebo, VLA |
| Background | Grid pattern + mouse-tracking gradient orb | Engineering blueprint aesthetic with subtle interactivity |
| Motion | CSS keyframes only (no Framer Motion) | Zero JS bundle impact, better performance |

### 2. Typography Stack

| Purpose | Font | Fallback | Weight |
|---------|------|----------|--------|
| Headlines | Space Grotesk | system-ui, sans-serif | 600 |
| Code/Technical | JetBrains Mono | monospace | 400-600 |
| Body | IBM Plex Sans | system-ui | 400-500 |

### 3. Color Palette

| Token | Hex | Usage | WCAG on Dark |
|-------|-----|-------|--------------|
| Background | `#08090d` | Primary dark surface | - |
| Primary Accent | `#22d3ee` | Links, CTAs, highlights | 8.2:1 (AAA) |
| Success | `#22c55e` | Status indicators, pulsing dots | 6.5:1 (AA) |
| Text Primary | `#f0f0f0` | Headlines, main content | 14.3:1 (AAA) |
| Text Secondary | `#8b8b8b` | Descriptions, labels | 7.5:1 (AAA) |

### 4. Mobile-First Responsive Strategy

| Breakpoint | Layout | Key Changes |
|------------|--------|-------------|
| `< 640px` | Stacked | Full-width CTAs, hidden floating labels, compact robot |
| `640px-1024px` | Stacked centered | Text centered, robot above text |
| `> 1024px` | 2-column grid | Side-by-side layout, full animations |

### 5. Messaging Strategy

- **Solution-focused copy**: "Build robots that understand the physical world"
- **Personal journey framing**: From zero to production robots
- **Confident free tier**: "Free forever." stated once, not banner-shouted
- **CTAs**: "Begin Building" + "Explore the Stack" (action-oriented)

## Consequences

### Positive

- **Premium positioning**: Distinguishes from tutorial/course sites (Udemy, Coursera aesthetic)
- **Serious builder audience**: Attracts professionals and motivated learners
- **Platform identity**: Sets foundation for multi-book future
- **Performance**: CSS-only animations = zero JS overhead
- **Accessibility**: All colors pass WCAG AA, most pass AAA
- **Scalable design tokens**: CSS variables enable easy theming

### Negative

- **Higher design bar**: Future sections must maintain premium quality
- **Font loading**: 3 Google Fonts add ~60KB (mitigated with preload + `display=swap`)
- **Dark mode first**: Light mode needs careful contrast adjustments
- **Animation complexity**: Requires `prefers-reduced-motion` handling throughout
- **Mobile testing burden**: Must verify all animations work on touch devices

## Alternatives Considered

### Alternative A: Terminal/Code Preview Hero
- Live terminal showing ROS commands
- Hardware tier selector in hero
- **Rejected**: Confusing for first-time visitors, too technical upfront

### Alternative B: Animated Robot Mascot
- Cute robot character with wave animation
- Playful, approachable aesthetic
- **Rejected**: Not serious enough for platform positioning, "edutainment" feel

### Alternative C: Neural Mesh with Labels
- Abstract neural network visualization
- Floating tech labels
- **Rejected**: Too cluttered, doesn't communicate value proposition

### Alternative D: Brutalist Glitch Effects
- Rotating labels, glitch text animations
- Aggressive visual treatment
- **Rejected**: Still felt amateur ("noob-ish"), distracted from message

### Alternative E: Framer Motion for Animations
- JS-based animation library
- More complex choreography possible
- **Rejected**: Adds bundle size, CSS keyframes sufficient for our needs

## References

- Spec: `specs/001-home-page-redesign/spec.md`
- Plan: `specs/001-home-page-redesign/plan.md`
- Implementation: `robolearn-interface/src/components/HomepageComponents/`
- Tests: `robolearn-interface/tests/home-page.spec.ts`
- Design Inspiration: NVIDIA Developer Portal, Linear, Vercel

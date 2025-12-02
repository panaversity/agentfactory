# Feature Specification: Home Page Redesign

**Feature Branch**: `001-home-page-redesign`
**Created**: 2025-11-29
**Status**: Completed
**Input**: User description: "Redesign RoboLearn home page with Industrial-Kinetic Futurism aesthetic - dark industrial theme, kinetic cyan accents, animated robot joint motifs, blueprint-mode aesthetics"

## Sprint History

### Sprint 1: Initial Iterations (Rejected)
Multiple design iterations were rejected for lacking design thinking:
- Terminal/code preview hero - too technical for first-time visitors
- Animated robot mascot - too cutesy, not serious enough
- Neural mesh with labels - too cluttered
- Brutalist glitch effects - still felt amateur

### Sprint 2: Industrial Confidence (Accepted)
Used `frontend-design` skill to achieve premium platform aesthetic.
See: `adr-001-industrial-confidence-design.md` for full decision record.

**Key Design Shifts**:
1. **Platform, not Course** - Removed all "course" language, positioned as platform
2. **Solution-focused Copy** - "Build Machines That Free Up Your Time" not feature lists
3. **Industrial Confidence** - NVIDIA dev portal meets premium SaaS aesthetic
4. **Robot Assembly SVG** - Tech stack visualization (ROS 2, Isaac, Gazebo, VLA as robot parts)

### Sprint 3: Refinements & Infrastructure (Completed)
Post-implementation refinements based on testing:

**UI/UX Fixes**:
1. **"Explore the Stack" button** - Fixed to scroll to `#hardware-heading` section
2. **Dark theme default** - Changed `colorMode.defaultMode` to `'dark'`
3. **Labs/Chat nav items** - Dimmed with "Coming Soon" tooltip (not yet implemented)

**Infrastructure Changes**:
1. **Folder rename**: `book-source` → `robolearn-interface` (clearer naming)
2. **Local dev URL**: Changed from `/robolearn/` to `/` for localhost
3. **Tests moved**: From root `/tests/` to `robolearn-interface/tests/`
4. **Playwright config**: Moved into `robolearn-interface/`

**Documentation**:
- Created ADR-001 for Industrial Confidence design decision
- Added folder naming consistency rules to orchestrator

## Success Evals (Defined First)

**Platform-Focused Metrics**:
- **Eval-001**: Home page achieves Lighthouse performance score 90+ (automated test)
- **Eval-002**: Home page achieves Lighthouse accessibility score 95+ (automated test)
- **Eval-003**: Design review confirms industrial aesthetic implemented (5/5 visual motifs present: dark bg, cyan accents, grid animation, JetBrains Mono, blueprint styling)
- **Eval-004**: All 25 functional requirements (FR-001 through FR-025) pass acceptance tests
- **Eval-005**: Mobile responsive test passes on 375px, 768px, 1440px viewports

## Overview

Redesign the RoboLearn landing page to establish a distinctive visual identity that embodies the "Physical AI & Humanoid Robotics" theme. The design moves away from generic template aesthetics toward a cohesive "Industrial-Kinetic Futurism" visual language that communicates cutting-edge robotics education.

### Design Direction (Final: Industrial Confidence)

- **Theme**: Deep space dark (#08090d) - premium, serious platform
- **Primary Color**: Electric cyan (#22d3ee) - accent and interactive elements
- **Success Color**: Green (#22c55e) - active states, pulsing indicators
- **Typography**: Space Grotesk (headlines) + JetBrains Mono (code/tech) - industrial premium feel
- **Motion**: Staggered fade-in animations, pulsing dots, mouse-tracking gradient orb
- **Visual Motifs**: Robot assembly SVG with tech stack labels, grid pattern background, floating labels
- **Messaging**: Solution-focused copy, personal journey framing, confident "Free forever" statement

## User Scenarios & Testing *(mandatory)*

### User Story 1 - First-Time Visitor Landing (Priority: P1)

A prospective student lands on the RoboLearn home page for the first time and should immediately understand what the platform offers, feel the "Physical AI" theme, and be motivated to start learning.

**Why this priority**: First impressions determine conversion. The hero section is the most critical element for user retention.

**Independent Test**: Can be fully tested by loading the home page and verifying the hero section displays correctly with all animations, communicates the value proposition, and provides clear CTA.

**Acceptance Scenarios**:

1. **Given** a user opens the home page, **When** the page loads, **Then** they see an animated hero with book cover, title, technology badges, and primary CTA within 2 seconds
2. **Given** a user views the hero section, **When** they read the content, **Then** they understand this is a Physical AI & Robotics learning platform
3. **Given** a user wants to start learning, **When** they click "Start Learning", **Then** they are navigated to the first lesson

---

### User Story 2 - Course Curriculum Discovery (Priority: P1)

A visitor scrolls to explore the curriculum structure and understands the learning path from ROS 2 fundamentals to autonomous humanoids.

**Why this priority**: Curriculum clarity is essential for enrollment decisions. Users need to see the full journey.

**Independent Test**: Can be tested by scrolling to the modules section and verifying all 4 modules display with correct information and visual hierarchy.

**Acceptance Scenarios**:

1. **Given** a user scrolls past the hero, **When** they reach the modules section, **Then** they see all 4 modules displayed with titles, descriptions, topics, and week indicators
2. **Given** a user hovers on a module card, **When** the interaction occurs, **Then** the card provides visual feedback (glow, expansion, or highlight)
3. **Given** modules 3 and 4 are advanced, **When** displayed, **Then** they have visual distinction (badges: "AI-Powered", "Capstone")

---

### User Story 3 - Hardware Requirements Assessment (Priority: P2)

A prospective student evaluates hardware requirements to determine if they can participate in the course.

**Why this priority**: Hardware is a key enrollment barrier. Clear tier options reduce friction and anxiety.

**Independent Test**: Can be tested by viewing the hardware section and verifying all 3 tiers display with costs and recommendations.

**Acceptance Scenarios**:

1. **Given** a user reaches the hardware section, **When** they view the options, **Then** they see 3 clear tiers (Workstation, Cloud+Edge, Simulation Only) with cost estimates
2. **Given** a user is budget-conscious, **When** they see the "RECOMMENDED" badge on Cloud+Edge, **Then** they understand the balanced approach
3. **Given** a user has no hardware, **When** they see Simulation Only option, **Then** they understand they can still complete the course

---

### User Story 4 - Mobile Responsive Experience (Priority: P2)

A user accesses the home page from a mobile device and has a fully functional, visually appealing experience.

**Why this priority**: Significant traffic comes from mobile. Responsive design is essential for accessibility.

**Independent Test**: Can be tested by viewing the page on mobile viewport (375px width) and verifying layout, readability, and touch interactions.

**Acceptance Scenarios**:

1. **Given** a user opens the page on mobile, **When** the page loads, **Then** all content is readable without horizontal scrolling
2. **Given** a user on mobile views the modules, **When** displayed, **Then** modules stack vertically and remain interactive
3. **Given** a user on mobile, **When** they tap CTAs, **Then** buttons have adequate touch targets (min 44px)

---

### User Story 5 - Dark/Light Mode Compatibility (Priority: P3)

The page respects system color preferences while maintaining the industrial aesthetic.

**Why this priority**: Accessibility and user preference. Lower priority as dark mode is the primary design.

**Independent Test**: Can be tested by toggling system dark/light mode and verifying the page adapts appropriately.

**Acceptance Scenarios**:

1. **Given** a user has dark mode enabled, **When** they view the page, **Then** the full industrial-kinetic theme displays as designed
2. **Given** a user has light mode enabled, **When** they view the page, **Then** the page provides a light variant that maintains brand identity

---

### Edge Cases

- What happens when images fail to load? Fallback to styled placeholders with alt text
- How does the page handle slow connections? Progressive loading with skeleton states
- What happens when JavaScript is disabled? Core content remains accessible (graceful degradation)
- How does the page appear in reduced motion mode? Animations are disabled per user preference

## Requirements *(mandatory)*

### Functional Requirements

#### Hero Section
- **FR-001**: Hero MUST display Robot Assembly SVG visualization with animated tech stack labels (ROS 2, Isaac, Gazebo, VLA)
- **FR-002**: Hero MUST display brand mark "ROBOLEARN PLATFORM" with headline "Build robots that understand the physical world"
- **FR-003**: Hero MUST display tech stack pills (ROS 2, Isaac Sim, Gazebo, VLA Models) with pulsing dot indicators
- **FR-004**: Hero MUST provide primary CTA "Begin Building" linking to `/docs/preface-agent-native`
- **FR-005**: Hero MUST provide secondary CTA "Explore the Stack" scrolling to `#hardware-heading`
- **FR-006**: Hero MUST have animated grid pattern background with mouse-tracking gradient orb

#### Modules Section
- **FR-007**: Page MUST display all 4 course modules with visual hierarchy
- **FR-008**: Each module MUST show: icon, module number, title, description, topic list, week range
- **FR-009**: Modules 3 and 4 MUST have distinctive styling with badges ("AI-Powered", "Capstone")
- **FR-010**: Modules MUST have hover interaction with visual feedback

#### Features Section
- **FR-011**: Page MUST display 6 key features of the platform
- **FR-012**: Features MUST use custom icons (not emoji) that match industrial aesthetic
- **FR-013**: Two features MUST be marked as "Core" with distinctive styling

#### Hardware Section
- **FR-014**: Page MUST display 3 hardware tier options
- **FR-015**: Each tier MUST show: name, subtitle, description, cost estimate
- **FR-016**: Cloud+Edge tier MUST have "RECOMMENDED" badge

#### CTA Section
- **FR-017**: Page MUST end with compelling CTA section
- **FR-018**: CTA MUST include prominent "Begin Learning" button

#### Visual & Motion Requirements
- **FR-019**: Page MUST use dark industrial color scheme with CSS variables
- **FR-020**: Page MUST use Space Grotesk for headlines, JetBrains Mono for technical/code text, and IBM Plex Sans for body text
- **FR-021**: Page MUST implement staggered page load animation sequence
- **FR-022**: Page MUST implement scroll-triggered section reveals
- **FR-023**: Page MUST respect `prefers-reduced-motion` user preference

#### Responsive Requirements
- **FR-024**: Page MUST be fully responsive from 320px to 2560px viewport widths
- **FR-025**: Page MUST maintain touch-friendly targets (min 44px) on mobile

### Non-Functional Requirements

- **NFR-001**: Page MUST achieve Lighthouse performance score >= 90
- **NFR-002**: Page MUST achieve Lighthouse accessibility score >= 95
- **NFR-003**: Page MUST load critical content within 2 seconds on 3G connection
- **NFR-004**: All text MUST meet WCAG 2.1 AA contrast requirements

### Assumptions

- Docusaurus 3.x framework with React 19/TypeScript is the implementation platform
- CSS Modules approach maintained for styling isolation
- Google Fonts loaded via CDN with preload/preconnect optimization
- CSS-only animations using keyframes (no Framer Motion dependency)
- Dark theme is default (`colorMode.defaultMode: 'dark'`)
- Font loading strategy: Space Grotesk/JetBrains Mono/IBM Plex with system fallbacks; fonts preloaded via `<link rel="preload">` in document head; font-display: swap for progressive enhancement

### Non-Goals

- Server-side functionality changes (auth, RAG, backend)
- Navigation or footer redesign (focus is on main content sections)
- New sections or content structure changes
- Internationalization/translation features
- Interactive elements beyond hover states and scroll animations

## Key Entities

- **Hero Section**: Primary visual impact area with book cover, title, CTAs
- **Module Card**: Reusable component displaying course module information
- **Feature Card**: Reusable component displaying platform feature
- **Hardware Tier Card**: Reusable component displaying hardware option
- **CTA Block**: Final conversion section with action button

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Design review confirms: dark industrial background (#08090d), electric cyan accents (#22d3ee), Space Grotesk + JetBrains Mono typography, animated grid background with gradient orb, all 5 sections implemented per FR requirements
- **SC-002**: All 5 main sections (Hero, Modules, Features, Hardware, CTA) render correctly on desktop and mobile
- **SC-003**: Page achieves Lighthouse performance score of 90+
- **SC-004**: Page achieves Lighthouse accessibility score of 95+
- **SC-005**: All animations complete smoothly at 60fps on mid-range devices (tested on iPhone SE 2020 equivalent using Chrome DevTools 4x CPU throttling)
- **SC-006**: Page load time is under 3 seconds on broadband, under 5 seconds on 3G
- **SC-007**: Color contrast ratios meet WCAG 2.1 AA standards (4.5:1 for normal text, 3:1 for large text) - validated using WebAIM Contrast Checker + axe DevTools
- **SC-008**: All interactive elements have visible focus states for keyboard navigation
- **SC-009**: Page clearly communicates "Physical AI & Robotics Education" value proposition within first viewport
- **SC-010**: Design uses custom components (not stock Docusaurus): hero book tilt, animated grid background, blueprint aesthetics, kinetic motion patterns - validated via code review
- **SC-011**: Visual audit confirms industrial motifs present: grid patterns in 2+ sections, geometric shapes/angles (not rounded cards), monospace typography for technical elements, dark palette (background luminosity < 20%)

## Dependencies

- Book cover image asset (`/img/book-cover-page.png`) - used in OG meta, not hero
- Google Fonts: Space Grotesk, JetBrains Mono, IBM Plex Sans, IBM Plex Mono
- Docusaurus Layout and theme components
- Existing routing structure (`/docs/preface-agent-native`)
- Project folder: `robolearn-interface/` (renamed from `book-source`)
- Local dev URL: `http://localhost:3000/` (production: `https://mjunaidca.github.io/robolearn/`)

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Font loading delays | Visual flash | Use font-display: swap, provide fallback fonts |
| Animation performance | Janky experience | Test on low-end devices, use GPU-accelerated properties |
| Dark theme accessibility | Poor readability | Verify contrast ratios, test with accessibility tools |
| Complex CSS maintenance | Technical debt | Use CSS variables consistently, document design tokens |

# PHR-0005: Industrial Confidence Hero Redesign

**ID**: PHR-0005
**Title**: Industrial Confidence Hero Redesign with Frontend-Design Skill
**Stage**: misc
**Feature**: home-page-redesign
**Date**: 2025-11-29
**Status**: completed

---

## Prompt

User requested redesign using frontend-design skill after multiple iterations produced "weird noob designs with no sense of design thinking and entrepreneurship." Key requirements:
- Remember the goal of PLATFORM (not course)
- Don't use "course" word anywhere
- Make it personalized - solutions, not features
- People need solutions to make life easier or free up more time

Previous iterations failed with:
1. Terminal preview + hardware tier selector (confusing for visitors)
2. Cutesy animated robot mascot (not serious enough)
3. Neural mesh with labels (too cluttered)
4. Glitch effects and brutalist labels (still felt noob-ish)

---

## Response Summary

### Design Philosophy: Industrial Confidence
Premium, engineering-focused platform aesthetic inspired by NVIDIA developer portal meets premium SaaS. Key principles:
- **Serious platform for serious builders** - not edutainment
- **Confident value prop** - "Free forever" stated once, confidently
- **Show outcomes** - Robot assembly visualization showing the actual tech stack
- **No desperate badges** - no screaming FREE, no cutesy mascots

### Implementation

**Hero Section:**
- Brand mark: `ROBOLEARN` + `PLATFORM` tag
- Headline: "Build robots that **understand** the physical world."
- Robot Assembly SVG: ROS 2 (head), Isaac (left arm), Gazebo (right arm), VLA Models (legs) with animated data flow particles
- CTAs: "Begin Building" + "Explore the Stack"
- Tech stack pills with green pulsing dots
- Mouse-following gradient orb for subtle interactivity

**Typography:**
- Headlines: Space Grotesk (industrial, premium SaaS)
- Code/Tech: JetBrains Mono (engineering credibility)

**Color Palette:**
- Background: `#08090d` (deep space)
- Accent: `#22d3ee` (electric cyan)
- Success: `#22c55e` (green for active states)

**Section Copy Updates:**
- Modules: "Your Journey" → "From Zero to Building Robots That Think"
- Features: "Why This Matters" → "Build Machines That Free Up Your Time"
- Hardware: "Start Where You Are" → "No Expensive Hardware Required"

### Tests
All 27 Playwright tests passing after updates.

---

## Lessons Learned

1. **Multiple iterations are normal** - Design requires exploration
2. **Frontend-design skill activates reasoning** - Clear persona/questions/principles structure
3. **Avoid AI slop** - No purple gradients, no Inter font, no cookie-cutter components
4. **Solution-focused copy** - People don't want features, they want outcomes
5. **Platform ≠ Course** - Language matters for positioning

---

## Files Modified

- `robolearn-interface/src/components/HomepageComponents/HeroSection/index.tsx` - Complete rewrite
- `robolearn-interface/src/components/HomepageComponents/HeroSection/styles.module.css` - Industrial styling
- `robolearn-interface/src/pages/index.tsx` - Section headers updated
- `tests/home-page.spec.ts` - Tests updated for new design

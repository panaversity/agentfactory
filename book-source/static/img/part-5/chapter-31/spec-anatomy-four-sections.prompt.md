# Image Generation Prompt: Specification Anatomy (Four Sections)

**Filename**: `spec-anatomy-four-sections.png`
**Chapter**: 31 (Spec-Driven Development Fundamentals)
**Lesson**: 03 (Anatomy of a Specification)
**Proficiency**: B1 (Intermediate)
**Date Created**: 2025-11-23
**Generator**: Gemini Nano Banana Pro (gemini.google.com)

---

## THE STORY

A developer opens a blank specification template and freezes. "What do I write? Where do I start?" Four sections provide structure—like a blueprint has standard parts (foundation, walls, roof, systems). Once you see the pattern, specs write themselves.

**Emotional Intent**: Clarity through structure. Chaos → organization. "Oh, it's just four questions to answer."

**Visual Metaphor**: Blueprint or building layers. Each section builds on previous. Foundation (Intent) → Structure (Success Criteria) → Constraints (Limits) → Boundary (Non-Goals).

**Key Insight**: Good specs have predictable structure. Master the template, master the method.

---

## CONDENSED CREATIVE BRIEF

Four-section hierarchical diagram showing specification anatomy with clear relationships and purpose.

**Visual Structure**: Vertical stack or pyramid showing how sections relate.

**Section 1 - INTENT (Foundation)**:
- Position: Bottom/foundation layer
- Icon: Lightbulb or "Why?" symbol
- Label: "1. Intent—WHY this feature exists"
- Description: "Problem statement, user need, business value"
- Example snippet: "Users waste time managing config files manually..."
- Color: Blue (#2563eb) - foundation, reasoning
- Size: Widest layer (everything builds on this)

**Section 2 - SUCCESS CRITERIA (Structure)**:
- Position: Second layer, built on Intent
- Icon: Checkmark or target
- Label: "2. Success Criteria—WHAT defines done"
- Description: "Testable, measurable, specific outcomes"
- Example snippet: "✓ API response <200ms\n✓ Handles 10K configs\n✓ Zero data loss"
- Color: Green (#10b981) - validation, goals
- Size: Medium-wide (builds on intent)

**Section 3 - CONSTRAINTS (Boundaries)**:
- Position: Third layer, limits Section 2
- Icon: Warning triangle or fence
- Label: "3. Constraints—Non-negotiable limits"
- Description: "Technical bounds, policies, resources"
- Example snippet: "Must use existing auth\nNo new dependencies\nCompletes in 2 sprints"
- Color: Orange (#f59e0b) - caution, limits
- Size: Medium-narrow (constrains criteria)

**Section 4 - NON-GOALS (Scope Boundary)**:
- Position: Top layer, defines boundary
- Icon: ⛔ or "Not included"
- Label: "4. Non-Goals—What we explicitly WON'T do"
- Description: "Scope prevention, future-proofing"
- Example snippet: "⛔ No GUI (CLI only)\n⛔ No cloud sync\n⛔ No auth system"
- Color: Red (#dc2626) - stop, boundary
- Size: Narrowest (tight scope)

**Relationship Arrows**:
- Intent → Success Criteria: "Informs"
- Success Criteria ← Constraints: "Limited by"
- Non-Goals: "Protects from scope creep"

**Bottom Annotation**: "Four sections, logical flow. Learn the structure, write specs fast."

---

## PEDAGOGICAL REASONING

**Why this visual works**:
1. **Template mastery** - Students memorize 4 sections, never face blank page again
2. **Logical relationships** - Shows how sections interact (not isolated)
3. **Concrete examples** - Each section has real snippet (not abstract)
4. **Size teaches importance** - Intent is widest (most foundational)
5. **Color coding** - Visual mnemonic (blue=why, green=success, orange=limits, red=stop)

**Misconception prevented**: "Specs are freeform essays." Visual proves specs have structure.

**Proficiency alignment (B1)**: 8 elements, clear hierarchy, memorable pattern.

---

## COLOR SEMANTICS

- **Blue** (#2563eb): Intent section - foundation, reasoning (why)
- **Green** (#10b981): Success Criteria - goals, validation (what)
- **Orange** (#f59e0b): Constraints - caution, limits (boundaries)
- **Red** (#dc2626): Non-Goals - stop, scope boundary (protection)
- **Gray** (#6b7280): Relationship arrows, scaffolding

---

## TYPOGRAPHY HIERARCHY

- **Largest**: Section numbers and labels ("1. Intent", "2. Success Criteria")
- **Medium**: Descriptions (one-line purposes)
- **Smallest**: Example snippets (concrete content)

---

## SUCCESS CRITERIA

**Student can grasp in <8 seconds**:
- Four sections in every spec
- Clear progression: why → what → limits → boundaries
- Each section has distinct purpose
- Template provides structure

**Measurement**: Student should say "I can use this template right now."

---

## Alt Text

Hierarchical diagram showing four specification sections stacked vertically: Intent (why—blue foundation), Success Criteria (what—green structure), Constraints (limits—orange boundaries), Non-Goals (scope protection—red boundary). Arrows show logical relationships between sections.

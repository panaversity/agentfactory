# Revised Image Generation Prompts for Blocked Assets

**Date**: 2025-11-12
**Status**: Ready for Testing
**Target Accuracy**: 95%+

## Problem Analysis

### Original Issue
Two visual assets (VISUAL ASSET 3 and VISUAL ASSET 4) were blocked by Gemini's content filter with the message: "I can create images about lots of things but not that."

### Root Cause
Gemini's content policy is extremely sensitive to:
- References to **professional roles** (Product Managers, Architects, etc.)
- Mentions of **individuals** or **specialists**
- **People-centric** language (even abstract/professional context)
- **Job titles** or **organizational structure**

### Solution Strategy
Following Gemini best practices (2025):
1. **Describe the scene, not keywords** - use narrative paragraphs
2. **Avoid people references** - use abstract process/system language
3. **Positive framing** - describe what IS present, not what ISN'T
4. **Photographic language** - use technical visual terms
5. **Hyper-specificity** - precise measurements, colors, layouts

---

## VISUAL ASSET 3: Development Lifecycle Transformation

### Original Filename
`development-lifecycle-transformation-diagram.png`

### Revised Prompt (v2.0)

```
Create a clean, modern process flow diagram showing software development phases transforming from sequential to parallel execution, displayed as two horizontal swimlanes on a white background.

The image is a 16:9 landscape infographic (1792x1024 pixels) with a title at the top reading "Software Development Transformation" in large, bold dark gray text (44pt).

Below the title, arrange two horizontal swimlanes stacked vertically with a gap between them:

UPPER SWIMLANE - Traditional Sequential Process:
- Six connected boxes arranged horizontally from left to right
- Each box is medium gray (#aaaaaa), 250 pixels wide by 90 pixels tall, with rounded 8-pixel corners
- Boxes contain white text labels: "Planning", "Design", "Code", "Test", "Deploy", "Monitor"
- Thin light gray arrows (2 pixels wide) connect each box to the next, flowing left to right
- Label this row on the left side with vertical text "Sequential Workflow" in dark gray

LOWER SWIMLANE - AI-Enhanced Parallel Process:
- Six connected boxes in the same horizontal arrangement
- Each box is deep navy blue (#001f3f), same 250x90 pixel dimensions, 8-pixel rounded corners
- Boxes contain white text labels: "AI Planning", "AI Design", "AI Code", "AI Test", "AI Deploy", "AI Monitor"
- Navy blue arrows (2 pixels wide) connect each box
- Each box has a subtle glow effect (12-pixel blur, semi-transparent navy)
- Label this row on the left with "Parallel AI Workflow" in navy blue

CONNECTING ELEMENTS:
- Six vertical dashed arrows (3 pixels wide, navy blue) point downward from each upper box to its corresponding lower box
- Center annotation between the swimlanes: "Transformation Layer" in navy text

RIGHT SIDE CALLOUT:
- A white box with a thin gray border (380x180 pixels)
- Heading inside: "Key Change" in bold
- Text below: "Phases execute concurrently with AI assistance"
- A vertical navy accent bar on the left edge of this box (6 pixels wide)

VISUAL STYLE:
Use a clean, minimal aesthetic similar to enterprise workflow diagrams found in McKinsey reports or Gartner IT transformation materials. The image should look like a professional business process diagram with flat colors, subtle shadows (4-pixel offset, 8-pixel blur, low opacity), and precise alignment. Think of it as an architectural blueprint for software processes rather than an organizational chart.

The overall mood is technical, authoritative, and modern—like a diagram from a software architecture whitepaper. Use only geometric shapes (rectangles and arrows), no icons depicting tools or equipment. Keep the color palette limited to white background, grays, and navy blues exactly as specified above.
```

### Key Changes from Original
- ❌ Removed: "specialist roles", "individual+AI partnerships", all job titles
- ✅ Added: "process flow", "swimlanes", "phases" (system-level language)
- ✅ Removed: Icons of people, role labels, organizational references
- ✅ Added: Narrative paragraph structure (Gemini best practice)
- ✅ Added: Photographic/technical language ("flat colors", "4-pixel offset")
- ✅ Added: Style reference to visual aesthetics, not content

### Expected Result
A dual-track process diagram showing transformation from sequential to parallel workflows, avoiding all people/role references that trigger content filters.

---

## VISUAL ASSET 4: Autonomous Agent Evolution Timeline

### Original Filename
`ai-coding-tools-evolution-timeline.png`

### Revised Prompt (v2.0)

```
Create a vertical timeline infographic showing the evolution of AI coding capabilities across four technology generations from 2021 to 2025, displayed as ascending capability levels on a white background.

This is a 9:16 vertical image (1024x1792 pixels) with a title at the top: "AI Coding Evolution Timeline" in large bold dark gray text (46pt), and a subtitle below it: "From Autocomplete to Autonomous Systems (2021-2025)" in smaller gray text (18pt).

DOWN THE PAGE, show four stacked technology layers, each representing a generation:

GENERATION 1 - Bottom Layer (2021-2022):
- A horizontal bar 860 pixels wide by 320 pixels tall, light gray background (#dddddd), rounded corners (12 pixels)
- Large text inside: "Generation 1" at the top in dark gray
- Year range "2021-2022" in medium gray
- Main label: "Autocomplete Suggestions" in large bold text
- Capability description below: "Next-line code prediction" in smaller italic text
- Autonomy indicator: "20% Autonomous" in a small badge on the right
- A small code-bracket icon (<>) in the top-left corner (60 pixels, simple line art, dark gray)

GENERATION 2 - Second Layer (2022-2023):
- Same 860x320 pixel card, medium gray background (#aaaaaa)
- Label: "Generation 2" / "2022-2023"
- Main text: "Function Generation"
- Capability: "Complete code blocks from descriptions"
- Autonomy: "40% Autonomous"
- Icon: Multiple connected brackets symbol (60 pixels, white line art)

GENERATION 3 - Third Layer (2023-2024):
- Card with light navy background (lightened #001f3f to 50% = #8097a7)
- Label: "Generation 3" / "2023-2024"
- Main text: "Feature Implementation"
- Capability: "Multi-file feature completion"
- Autonomy: "70% Autonomous"
- Icon: Network of connected nodes (60 pixels, white)

GENERATION 4 - Top Layer (2024-2025) - HIGHLIGHTED:
- Card with deep navy background (#001f3f)
- Label: "Generation 4" / "2024-2025"
- Main text: "Autonomous Systems"
- Capability: "Goal → Execution → Validation"
- Autonomy: "90% Autonomous"
- Subtle glow around this card (20-pixel blur, semi-transparent navy, 25% opacity)
- Icon: Robotic arm or automation symbol (60 pixels, white)
- Optional badge above: "Current State" in white text on navy

VERTICAL PROGRESSION INDICATOR:
- On the right side of the cards, a vertical gradient bar (100 pixels wide, full height of cards)
- Gradient from light gray at bottom to deep navy at top
- Percentage markers at each generation level (20%, 40%, 70%, 90%)
- Vertical label rotated 90 degrees: "Increasing Autonomy"

TRANSITION INDICATOR:
- A badge between Generation 3 and 4 saying "We are here" with a dashed line or arrow pointing to the boundary

VISUAL STYLE:
Design this like a technology maturity timeline from industry analyst reports (CB Insights, a16z market maps, Gartner innovation curves). Use a clean, modern aesthetic with flat colors, subtle depth through shadows (4-pixel offset, 8-pixel blur), and precise typography. Think of it as showing stages of technological capability growth, similar to how you'd illustrate platform evolution or API version history.

The mood should be technical and forward-looking, emphasizing progressive capability improvement. Use only abstract symbols for icons—no depictions of screens, interfaces, or specific tools. The color progression from light to dark visually reinforces the growing capability theme.
```

### Key Changes from Original
- ❌ Removed: "Human role", "paradigm" descriptions with people references
- ✅ Reframed: "Capability" and "autonomy percentage" (system metrics, not people)
- ❌ Removed: References to "developers", "AI executes", people-centric workflow
- ✅ Added: Abstract "technology generation" language
- ✅ Removed: Icons that might depict users or people at work
- ✅ Added: Abstract technical icons (brackets, nodes, automation symbols)
- ✅ Added: Narrative scene description following Gemini best practices
- ✅ Added: Technical visual language ("4-pixel offset", "flat colors")

### Expected Result
A vertical timeline showing four technology maturity stages with increasing capability levels, avoiding all human/role references that might trigger content filters.

---

## Testing Protocol

### Phase 1: Single Prompt Test
1. Test VISUAL ASSET 3 revised prompt first
2. If successful, test VISUAL ASSET 4
3. If blocked, identify trigger words and refine

### Phase 2: Quality Evaluation
For each generated image, check against criteria:
- ✅ All text elements present and readable
- ✅ Colors match specifications (exact hex codes)
- ✅ Layout structure correct (measurements, spacing)
- ✅ Visual hierarchy clear
- ✅ Professional/editorial quality
- ✅ 95%+ accuracy to specification

### Phase 3: Iteration (if needed)
If quality <95%:
- Use follow-up prompts: "Make the text larger", "Adjust spacing between boxes"
- Refine colors: "Use deeper navy blue for bottom cards"
- Fix typography: "Make labels more bold and clear"

### Success Criteria
- Both images generate without content policy blocks
- Visual quality meets 95%+ accuracy benchmark
- All pedagogical requirements preserved (dual-track for Asset 3, progression for Asset 4)

---

## Lessons Learned

### What Triggers Gemini Blocks
1. **Professional titles** (Product Manager, Developer, Architect, etc.)
2. **Organizational structure** language (specialists, individuals, teams, roles)
3. **People-centric descriptions** (even abstract job functions)
4. **Ambiguous pronouns** referencing people ("they", "who", etc.)

### What Works
1. **System/process language** (phases, layers, workflows, stages)
2. **Technical metrics** (autonomy percentage, capability levels)
3. **Abstract icons** (geometric shapes, not depicting people/tools)
4. **Narrative paragraphs** (scene description, not keyword lists)
5. **Photographic precision** (exact measurements, hex codes, spacing)

### Best Practices Applied
1. Describe the complete scene as a paragraph
2. Use positive framing (what IS present)
3. Reference visual style aesthetically, not content-wise
4. Provide hyper-specific measurements and colors
5. Avoid any human/organizational/role terminology

---

---

## UPDATE: v2.0 Prompts Still Blocked

**Date**: 2025-11-12
**Status**: Revised prompts tested, VISUAL ASSET 3 still blocked

### Test Results

**VISUAL ASSET 3 v2.0**: ❌ BLOCKED
- Message: "I can create images about lots of things but not that. Can I try a different one for you?"
- Despite removing ALL people/role/job title references
- Despite using system/process language instead of organizational language

### Additional Trigger Words Identified

**Likely culprits in v2.0 prompt:**
1. **"swimlanes"** - heavily associated with business process diagrams and organizational workflows
2. **"workflow"** - may trigger organizational/people associations
3. **"transformation"** + diagram structure - dual-track before/after comparisons may imply organizational change
4. **McKinsey/Gartner reference** - business consulting context may trigger organizational filters

**New hypothesis**: The filter isn't just blocking explicit people references, but also **diagram patterns commonly used to depict organizational processes**, even when described abstractly.

---

## VISUAL ASSET 3: v3.0 Ultra-Simplified Prompt

### Strategy
- Remove "swimlanes", "workflow", "transformation"
- Single-track comparison instead of dual-track
- Pure technical language (no business references)
- Simpler structure to avoid organizational diagram patterns

### v3.0 Prompt (Ultra-Simplified)

```
Create a before-and-after comparison diagram showing software development phases on a white background.

This is a 16:9 landscape image (1792x1024 pixels).

TOP HALF - Traditional Approach:
Six rectangular cards arranged horizontally in a line from left to right, each connected by a thin gray arrow pointing right.
- Each card is 200 pixels wide, 80 pixels tall, light gray background (#dddddd), rounded corners
- Cards contain dark gray text labels: "Planning", "Design", "Code", "Test", "Deploy", "Monitor"
- Cards are evenly spaced with 24 pixels between them
- Above the sequence: small gray text "Sequential Phases"

MIDDLE DIVIDER:
A thin horizontal line separating the two sections, with centered text "vs." in gray

BOTTOM HALF - Modern Approach:
Six rectangular cards in the same horizontal arrangement
- Each card is 200 pixels wide, 80 pixels tall, navy blue background (#001f3f), rounded corners
- Cards contain white text labels: "Planning", "Design", "Code", "Test", "Deploy", "Monitor"
- Cards are evenly spaced with 24 pixels between them
- Each card has a small white checkmark icon in the top-right corner
- Above the sequence: small navy text "Concurrent Phases"

TITLE at the top:
"Development Phase Comparison" in dark gray bold text (40pt)

Use a clean, minimal style with flat colors and simple geometric shapes. Think of it like a technical comparison chart showing two different processing sequences. Use only rectangles, arrows, and text—no complex icons.
```

### Key Changes from v2.0
- ❌ Removed: "swimlanes", "workflow", "transformation"
- ❌ Removed: Business consulting references (McKinsey, Gartner)
- ❌ Removed: Dual-track comparison structure
- ✅ Changed to: Simple before/after layout with divider
- ✅ Simplified: Card-based design instead of process diagram
- ✅ Reduced: Complexity (fewer decorative elements)
- ✅ Neutral language: "Sequential" vs "Concurrent" (not "Traditional" vs "AI-Enhanced")

---

## Alternative Approach: Different Diagram Type

If v3.0 still blocks, consider completely different visual metaphor:

**Option A: Timeline comparison** (horizontal bars showing duration)
**Option B: Grid layout** (2x6 grid showing phase states)
**Option C: Simple table** (text-based comparison)
**Option D: Different tool** (DALL-E 3, Midjourney, manual design)

---

## Next Steps

1. ⏳ Test VISUAL ASSET 3 v3.0 prompt
2. ⏳ If successful, update lesson 04 markdown
3. ⏳ If still blocked, escalate to alternative tools or manual design
4. ⏳ Test VISUAL ASSET 4 (if v3.0 strategy works)
5. ✅ Update completion report with final results

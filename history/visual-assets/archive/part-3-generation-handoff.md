# Part 3 Visual Assets - Generation Handoff Document

**Date**: 2025-01-12
**Status**: Ready for parallel image generation (Tabs 0-1 generating, Tabs 2-7 ready)
**Phase**: Image Generation & Quality Validation
**Session**: Continuation required for quality validation and integration

---

## Executive Summary

**Completed Work:**
- ✅ Audit: 8 visuals approved across Chapters 10-11
- ✅ Prompts: All 8 detailed prompts generated and embedded in lesson markdown
- ✅ Browser Setup: 8 Gemini tabs opened at https://gemini.google.com/app
- ✅ Generation Started: Tabs 0-1 submitted and generating

**Ready for Continuation:**
- 🔄 Tabs 2-7: Need prompts pasted (all prompts extracted and ready below)
- 🔄 All 8 tabs: Need quality validation at 99% threshold
- 🔄 Integration: Download → Update markdown → Create sidecars → Final QA

---

## Current Browser State

### Active Tabs (8 total)
```
- Tab 0: https://gemini.google.com/app/e0d5cf58275b6878 [GENERATING] Visual 1
- Tab 1: https://gemini.google.com/app/ecfd56a253d4a1d2 [GENERATING] Visual 2
- Tab 2: https://gemini.google.com/app [READY] Visual 3
- Tab 3: https://gemini.google.com/app [READY] Visual 4
- Tab 4: https://gemini.google.com/app [READY] Visual 5
- Tab 5: https://gemini.google.com/app [READY] Visual 6
- Tab 6: https://gemini.google.com/app [READY] Visual 7
- Tab 7: https://gemini.google.com/app [READY] Visual 8
```

---

## Visual Assets Mapping

### Chapter 10: Prompt Engineering (5 visuals)

#### Visual 1: Context Window Mental Model
- **Status**: Tab 0 GENERATING
- **File**: `10-prompt-engineering-for-aidd/01-understanding-ai-agents.md:144`
- **Destination**: `book-source/static/img/part-3/chapter-10/context-window-mental-model.png`
- **Type**: Container diagram (800x600px centered in 1792x1024px canvas)
- **Key Elements**: Conversation History (top), Loaded Files (bottom), "Forgotten" overflow indicator
- **Spelling Safeguards**: C-O-N-V-E-R-S-A-T-I-O-N, H-I-S-T-O-R-Y

#### Visual 2: Command Structure Formula
- **Status**: Tab 1 GENERATING
- **File**: `10-prompt-engineering-for-aidd/02-writing-clear-commands.md:250`
- **Destination**: `book-source/static/img/part-3/chapter-10/command-structure-formula.png`
- **Type**: Horizontal formula with example (1792x1024px)
- **Key Elements**: VERB + TARGET + OUTCOME formula (top), concrete example (bottom), downward arrow
- **Spelling Safeguards**: V-A-L-I-D-A-T-E-S

#### Visual 3: 4-Layer Context Stack
- **Status**: Tab 2 READY (prompt below)
- **File**: `10-prompt-engineering-for-aidd/03-providing-context.md:127`
- **Destination**: `book-source/static/img/part-3/chapter-10/four-layer-context-stack.png`
- **Type**: Vertical stack (4 layers, 400x180px each)
- **Key Elements**: PROJECT → CODE → CONSTRAINT → DEVELOPER contexts, left bracket "Complete Context"
- **Spelling Safeguards**: C-O-N-S-T-R-A-I-N-T, D-E-V-E-L-O-P-E-R

#### Visual 4: Requirements vs Logic Split-Screen
- **Status**: Tab 3 READY (prompt below)
- **File**: `10-prompt-engineering-for-aidd/04-specifying-logic.md:65`
- **Destination**: `book-source/static/img/part-3/chapter-10/requirements-vs-logic-split-screen.png`
- **Type**: Split-screen comparison (1792x1024px)
- **Key Elements**: Left "WHAT" (vague), Right "HOW" (5 steps), transform arrow
- **Spelling Safeguards**: R-E-G-I-S-T-R-A-T-I-O-N, V-A-L-I-D-A-T-E

#### Visual 5: 5-Step Validation Checklist
- **Status**: Tab 4 READY (prompt below)
- **File**: `10-prompt-engineering-for-aidd/05-validating-code.md:69`
- **Destination**: `book-source/static/img/part-3/chapter-10/five-step-validation-checklist.png`
- **Type**: Horizontal process flow (5 boxes, 280x480px each)
- **Key Elements**: Read First → Check Secrets → Check Issues → Test It → Compare to Spec
- **Spelling Safeguards**: C-R-E-D-E-N-T-I-A-L-S

---

### Chapter 11: Context Engineering (3 visuals)

#### Visual 6: Context Engineering vs Prompt Engineering
- **Status**: Tab 5 READY (prompt below)
- **File**: `11-context-engineering-for-ai-driven-development/01-what-is-context-engineering.md:214`
- **Destination**: `book-source/static/img/part-3/chapter-11/context-vs-prompt-engineering.png`
- **Type**: Layered foundation model (3 layers with upward arrows)
- **Key Elements**: Bottom "CONTEXT ENGINEERING", Middle "PROMPT ENGINEERING", Top "QUALITY OUTPUT"
- **Spelling Safeguards**: E-N-G-I-N-E-E-R-I-N-G (appears twice), I-N-S-T-R-U-C-T-I-O-N-S

#### Visual 7: Context Window Fill States
- **Status**: Tab 6 READY (prompt below)
- **File**: `11-context-engineering-for-ai-driven-development/02-understanding-context-windows.md:187`
- **Destination**: `book-source/static/img/part-3/chapter-11/context-window-fill-states.png`
- **Type**: 3-stage progression (3 columns, 480x700px each)
- **Key Elements**: Healthy (20%) → Degrading (60%) → Rot (90%), fill indicators with symptoms
- **Spelling Safeguards**: D-E-G-R-A-D-I-N-G, I-N-C-O-N-S-I-S-T-E-N-C-I-E-S

#### Visual 8: Six Components of AIDD Context
- **Status**: Tab 7 READY (prompt below)
- **File**: `11-context-engineering-for-ai-driven-development/03-six-components-aidd-context.md:70`
- **Destination**: `book-source/static/img/part-3/chapter-11/six-components-aidd-context.png`
- **Type**: 3x2 grid of component cards (480x380px each)
- **Key Elements**: 6 numbered cards (Model Selection, Dev Tools, Knowledge/Memory, Audio [de-emphasized], Guardrails, Orchestration)
- **Spelling Safeguards**: O-R-C-H-E-S-T-R-A-T-I-O-N, C-O-O-R-D-I-N-A-T-I-O-N

---

## Complete Image Generation Prompts (Ready to Paste)

### Tab 2 Prompt (Visual 3: 4-Layer Context Stack)

```
Professional educational diagram showing the 4-layer hierarchical context stack for AI-driven development.

Layout: Vertical stack of 4 layers (1792x1024px canvas, 16:9 aspect ratio)
- Background: White (#FFFFFF)
- Center alignment with 120px margins on left/right
- Vertical stack centered on canvas

Layer Structure (each layer is stacked box):
- Each layer: Rounded rectangle, 400px width x 180px height
- 20px vertical spacing between layers
- Stacked from top (Layer 1) to bottom (Layer 4)
- Each box has 3px border Medium Gray (#aaaaaa)
- Subtle shadow: 0px 2px 8px rgba(0,0,0,0.08)

Layer 1 - Project Context (Top):
- Background: Light Gray (#F8F9FA)
- Border: 3px Medium Gray (#aaaaaa)
- Title: "Layer 1: PROJECT CONTEXT" in 22pt Roboto Bold, Deep Navy (#001f3f), top of box
- Question: "What is this project?" in 18pt Roboto Regular, Dark Charcoal (#111111), center
- 24px padding inside box

Layer 2 - Code Context:
- Background: Light Gray (#F8F9FA)
- Border: 3px Medium Gray (#aaaaaa)
- Title: "Layer 2: CODE CONTEXT" in 22pt Roboto Bold, Deep Navy (#001f3f), top of box
- Question: "Where are we building?" in 18pt Roboto Regular, Dark Charcoal (#111111), center
- 24px padding inside box

Layer 3 - Constraint Context:
- Background: Light Gray (#F8F9FA)
- Border: 3px Medium Gray (#aaaaaa)
- Title: "Layer 3: CONSTRAINT CONTEXT" in 22pt Roboto Bold, Deep Navy (#001f3f), top of box
- Question: "What rules apply?" in 18pt Roboto Regular, Dark Charcoal (#111111), center
- 24px padding inside box

Layer 4 - Developer Context (Bottom):
- Background: Light Gray (#F8F9FA)
- Border: 3px Medium Gray (#aaaaaa)
- Title: "Layer 4: DEVELOPER CONTEXT" in 22pt Roboto Bold, Deep Navy (#001f3f), top of box
- Question: "Who is building and why?" in 18pt Roboto Regular, Dark Charcoal (#111111), center
- 24px padding inside box

Left Side Indicator:
- Vertical bracket spanning all 4 layers (left side of stack)
- Bracket: 4px stroke, Medium Gray (#aaaaaa)
- Label positioned next to bracket: "Complete Context" in 20pt Roboto Medium, Deep Navy (#001f3f)

Typography:
- Layer titles: 22pt Roboto Bold, Deep Navy (#001f3f)
- Layer questions: 18pt Roboto Regular, Dark Charcoal (#111111)
- Bracket label: 20pt Roboto Medium, Deep Navy (#001f3f)

Color Palette (Polar Night Theme):
- Background: White (#FFFFFF)
- Layer backgrounds: Light Gray (#F8F9FA)
- Primary accent: Deep Navy (#001f3f) - titles, bracket label
- Dark Charcoal: #111111 - questions
- Medium Gray: #aaaaaa - borders, bracket

IMPORTANT Spelling (letter-by-letter):
- "C-O-N-S-T-R-A-I-N-T" (Constraint)
- "D-E-V-E-L-O-P-E-R" (Developer)

Negative Instructions:
- NO extra text beyond specified labels
- NO decorative elements
- NO arrows between layers (just stacked boxes)
- NO complex gradients or patterns

Style Reference: Modern tech publication aesthetic (layered diagrams like Stripe developer docs)

Quality: professional, high-quality, publication-ready, clean, modern, editorial

Dimensions: 16:9 aspect ratio (1792x1024px)
```

---

### Tab 3 Prompt (Visual 4: Requirements vs Logic Split-Screen)

```
Professional educational split-screen comparison showing requirements (WHAT) versus implementation logic (HOW).

Layout: Split-screen design (1792x1024px canvas, 16:9 aspect ratio)
- Background: White (#FFFFFF)
- Vertical divider line down center: 4px solid Deep Navy (#001f3f)
- Left half and right half equal width (876px each)

Left Side - Requirements (WHAT):
- Background: Light Gray (#F8F9FA)
- Header at top: "REQUIREMENTS" in 28pt Roboto Bold, Deep Navy (#001f3f)
- Subheader: "(WHAT)" in 20pt Roboto Regular, Medium Gray (#aaaaaa)
- 48px from top
- Content area centered vertically:
  - Single text box with rounded border (3px Medium Gray #aaaaaa)
  - Inside box: "Create a user registration system" in 24pt Roboto Regular, Dark Charcoal (#111111)
  - Box dimensions: 680px width x 160px height
  - Text centered in box
  - 32px padding

Right Side - Logic (HOW):
- Background: White (#FFFFFF)
- Header at top: "LOGIC" in 28pt Roboto Bold, Deep Navy (#001f3f)
- Subheader: "(HOW)" in 20pt Roboto Regular, Medium Gray (#aaaaaa)
- 48px from top
- Content area centered vertically:
  - Numbered list (5 steps), left-aligned
  - Each step in 18pt Roboto Regular, Dark Charcoal (#111111)
  - 16px line spacing between steps
  - Steps:
    "1. Validate email format"
    "2. Check database for duplicates"
    "3. Hash password with bcrypt"
    "4. Create user record"
    "5. Return user object"
  - Clean, minimal bullets (number + period)

Center Divider Enhancement:
- Arrow graphic overlaying divider line pointing from left → right
- Arrow color: Deep Navy (#001f3f), bold 4px stroke
- Arrow positioned at vertical center
- Small label below arrow: "Transform" in 16pt Roboto Medium, Deep Navy (#001f3f)

Typography:
- Headers: 28pt Roboto Bold, Deep Navy (#001f3f)
- Subheaders: 20pt Roboto Regular, Medium Gray (#aaaaaa)
- Requirements box text: 24pt Roboto Regular, Dark Charcoal (#111111)
- Logic steps: 18pt Roboto Regular, Dark Charcoal (#111111)
- Arrow label: 16pt Roboto Medium, Deep Navy (#001f3f)

Color Palette (Polar Night Theme):
- Background: White (#FFFFFF) and Light Gray (#F8F9FA)
- Primary accent: Deep Navy (#001f3f) - headers, divider, arrow
- Dark Charcoal: #111111 - main content text
- Medium Gray: #aaaaaa - subheaders, borders

IMPORTANT Spelling (letter-by-letter):
- "R-E-G-I-S-T-R-A-T-I-O-N" (registration)
- "V-A-L-I-D-A-T-E" (validate)

Negative Instructions:
- NO extra text beyond specified content
- NO decorative icons or graphics
- NO complex gradients
- Arrow should be SIMPLE directional indicator only

Style Reference: Modern tech publication aesthetic (comparison diagrams like Stripe developer docs)

Quality: professional, high-quality, publication-ready, clean, modern, editorial

Dimensions: 16:9 aspect ratio (1792x1024px)
```

---

### Tab 4 Prompt (Visual 5: 5-Step Validation Checklist)

```
Professional educational process diagram showing the 5-step sequential validation workflow for AI-generated code.

Layout: Horizontal process flow (1792x1024px canvas, 16:9 aspect ratio)
- Background: White (#FFFFFF)
- Center alignment with 80px top/bottom margins
- Horizontal flow left to right

Process Flow Structure:
- 5 boxes arranged horizontally
- Each box: Rounded rectangle, 280px width x 480px height
- 48px horizontal spacing between boxes
- Each box has step number, title, and 2-3 key actions

Step 1 Box:
- Border: 3px Deep Navy (#001f3f)
- Background: White (#FFFFFF)
- Number circle top: "1" in 32pt Roboto Bold, White text on Deep Navy (#001f3f) circle (60px diameter)
- Title below number: "Read First" in 22pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "Sanity check" in 16pt Roboto Regular, Medium Gray (#aaaaaa)
- 24px padding

Step 2 Box:
- Border: 3px Deep Navy (#001f3f)
- Background: White (#FFFFFF)
- Number circle: "2" (same style as Step 1)
- Title: "Check Secrets" in 22pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "No credentials" in 16pt Roboto Regular, Medium Gray (#aaaaaa)

Step 3 Box:
- Border: 3px Deep Navy (#001f3f)
- Background: White (#FFFFFF)
- Number circle: "3" (same style)
- Title: "Check Issues" in 22pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "Safety & style" in 16pt Roboto Regular, Medium Gray (#aaaaaa)

Step 4 Box:
- Border: 3px Deep Navy (#001f3f)
- Background: White (#FFFFFF)
- Number circle: "4" (same style)
- Title: "Test It" in 22pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "Prove behavior" in 16pt Roboto Regular, Medium Gray (#aaaaaa)

Step 5 Box:
- Border: 3px Deep Navy (#001f3f)
- Background: White (#FFFFFF)
- Number circle: "5" (same style)
- Title: "Compare to Spec" in 22pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "Verify alignment" in 16pt Roboto Regular, Medium Gray (#aaaaaa)

Arrows Between Steps:
- Clean right-pointing arrow between each box
- Arrow color: Medium Gray (#aaaaaa), 3px stroke
- Simple design, NO text on arrows
- Positioned at vertical center between boxes

Typography:
- Step numbers: 32pt Roboto Bold, White on Deep Navy background
- Titles: 22pt Roboto Bold, Deep Navy (#001f3f)
- Subtitles: 16pt Roboto Regular, Medium Gray (#aaaaaa)

Color Palette (Polar Night Theme):
- Background: White (#FFFFFF)
- Primary accent: Deep Navy (#001f3f) - borders, number circles, titles
- Secondary accent: Medium Gray (#aaaaaa) - arrows, subtitles
- Number circles: Deep Navy (#001f3f) fill, White (#FFFFFF) text
- Box backgrounds: White (#FFFFFF)

Visual Elements:
- Boxes: Rounded rectangles (8px border-radius), 3px border Deep Navy (#001f3f)
- Number circles: Solid fill Deep Navy, 60px diameter, centered at top of each box
- Arrows: Simple right-pointing arrows, 3px stroke Medium Gray (#aaaaaa)
- Shadows: Subtle 0px 2px 8px rgba(0,0,0,0.08) on boxes only

Content:
- Sequential flow emphasizes step-by-step validation process
- Each box concisely labeled with action and purpose
- Arrows show mandatory sequence
- Clean, scannable horizontal layout

IMPORTANT Spelling (letter-by-letter):
- "C-R-E-D-E-N-T-I-A-L-S" (credentials)

Negative Instructions:
- NO detailed descriptions inside boxes (keep to number, title, subtitle only)
- NO checkboxes or completion indicators
- NO text labels on arrows (arrows are directional only)
- NO decorative icons inside boxes
- NO background patterns or gradients
- Arrows should be SIMPLE directional indicators, NO complex shapes or styles

Style Reference: Modern tech publication aesthetic (process flows like Stripe onboarding docs)

Quality: professional, high-quality, publication-ready, clean, modern, editorial

Dimensions: 16:9 aspect ratio (1792x1024px)
```

---

### Tab 5 Prompt (Visual 6: Context Engineering vs Prompt Engineering)

```
Professional educational diagram showing complementary relationship between context engineering and prompt engineering.

Layout: Layered foundation model (1792x1024px canvas, 16:9 aspect ratio)
- Background: White (#FFFFFF)
- Center alignment with generous spacing

Visual Structure - Three Layers:

Layer 1 (Bottom/Foundation - 35% of canvas height):
- Wide rectangle: 1200px width x 200px height
- Background: Light Gray (#F8F9FA)
- Border: 4px Deep Navy (#001f3f)
- Title centered: "CONTEXT ENGINEERING" in 28pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle below: "The Foundation" in 18pt Roboto Regular, Medium Gray (#aaaaaa)
- Below subtitle: "What AI Knows" in 16pt Roboto Regular, Dark Charcoal (#111111)
- Positioned at bottom third of canvas

Arrow 1 (Upward from Layer 1 to Layer 2):
- Clean upward pointing arrow, 4px stroke, Medium Gray (#aaaaaa)
- Arrow positioned at vertical center of Layer 1 top edge
- 120px height
- NO text on arrow itself

Layer 2 (Middle - 35% of canvas height):
- Wide rectangle: 1200px width x 200px height
- Background: White (#FFFFFF)
- Border: 4px Deep Navy (#001f3f)
- Title centered: "PROMPT ENGINEERING" in 28pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle below: "The Instructions" in 18pt Roboto Regular, Medium Gray (#aaaaaa)
- Below subtitle: "What You Ask" in 16pt Roboto Regular, Dark Charcoal (#111111)
- Positioned in middle of canvas

Arrow 2 (Upward from Layer 2 to Layer 3):
- Clean upward pointing arrow, 4px stroke, Medium Gray (#aaaaaa)
- 120px height
- NO text on arrow itself

Layer 3 (Top/Outcome - 30% of canvas height):
- Wide rectangle: 1200px width x 160px height
- Background: White (#FFFFFF)
- Border: 4px Deep Navy (#001f3f)
- Title centered: "QUALITY OUTPUT" in 28pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle below: "The Result" in 18pt Roboto Regular, Medium Gray (#aaaaaa)
- Positioned at top of canvas

Typography:
- Layer titles: 28pt Roboto Bold, Deep Navy (#001f3f)
- Layer subtitles (role): 18pt Roboto Regular, Medium Gray (#aaaaaa)
- Layer descriptions (what): 16pt Roboto Regular, Dark Charcoal (#111111)

Color Palette (Polar Night Theme):
- Background: White (#FFFFFF)
- Layer 1 background: Light Gray (#F8F9FA)
- Layer 2-3 backgrounds: White (#FFFFFF)
- Primary accent: Deep Navy (#001f3f) - borders, titles
- Medium Gray: #aaaaaa - subtitles, arrows
- Dark Charcoal: #111111 - descriptions

Visual Elements:
- Layers: Rounded rectangles (8px border-radius), 4px border Deep Navy (#001f3f)
- Arrows: Simple upward pointing arrows, 4px stroke Medium Gray (#aaaaaa)
- Shadows: Subtle 0px 2px 8px rgba(0,0,0,0.08) on all layers

Content:
- Layered foundation metaphor shows building from context → prompts → output
- Upward arrows emphasize progressive enhancement
- Clean, hierarchical visual structure

IMPORTANT Spelling (letter-by-letter):
- "E-N-G-I-N-E-E-R-I-N-G" (engineering - appears twice)
- "I-N-S-T-R-U-C-T-I-O-N-S" (instructions)

Negative Instructions:
- NO extra text beyond specified labels
- NO decorative elements
- NO text on arrows (arrows are directional only)
- NO complex gradients or patterns
- Arrows should be SIMPLE directional indicators only

Style Reference: Modern tech publication aesthetic (foundation diagrams like Stripe developer docs)

Quality: professional, high-quality, publication-ready, clean, modern, editorial

Dimensions: 16:9 aspect ratio (1792x1024px)
```

---

### Tab 6 Prompt (Visual 7: Context Window Fill States)

```
Professional educational progressive state diagram showing context window degradation from healthy to critical.

Layout: Horizontal 3-stage progression (1792x1024px canvas, 16:9 aspect ratio)
- Background: White (#FFFFFF)
- Center alignment with equal spacing between stages
- Left to right flow showing degradation

Stage Structure - Three Columns:
- Each stage: 480px width x 700px height
- 80px horizontal spacing between stages
- Each stage contains: fill indicator, percentage label, status title, symptoms list

Stage 1 (Left - Healthy):
- Container: Rounded rectangle, 480px x 700px
- Border: 4px solid Medium Gray (#aaaaaa)
- Background: White (#FFFFFF)

Fill Indicator (top of stage):
- Visual: Rectangle showing 20% fill level
- Container: 380px width x 120px height, 3px border Medium Gray (#aaaaaa)
- Fill: Deep Navy (#001f3f) filling 20% from bottom
- Above indicator: "20% Full" in 24pt Roboto Bold, Deep Navy (#001f3f)

Status Title:
- "✓ Healthy" in 32pt Roboto Bold, Deep Navy (#001f3f)
- Checkmark can be text character "✓"
- Centered below fill indicator

Symptoms (bulleted list):
- "Fast responses" in 18pt Roboto Regular, Dark Charcoal (#111111)
- "Accurate" in 18pt Roboto Regular
- "Remembers all" in 18pt Roboto Regular
- Left-aligned, 16px spacing between items
- Simple bullet points (•)

Stage 2 (Middle - Degrading):
- Container: Rounded rectangle, 480px x 700px
- Border: 4px solid Medium Gray (#aaaaaa)
- Background: Light Gray (#F8F9FA)

Fill Indicator:
- Container: 380px width x 120px height, 3px border Medium Gray (#aaaaaa)
- Fill: Deep Navy (#001f3f) filling 60% from bottom
- Above indicator: "60% Full" in 24pt Roboto Bold, Medium Gray (#aaaaaa)

Status Title:
- "⚠ Degrading" in 32pt Roboto Bold, Medium Gray (#aaaaaa)
- Warning symbol can be text character "⚠"
- Centered below fill indicator

Symptoms (bulleted list):
- "Slower" in 18pt Roboto Regular, Dark Charcoal (#111111)
- "Some errors" in 18pt Roboto Regular
- "Minor inconsistencies" in 18pt Roboto Regular
- Left-aligned, 16px spacing
- Simple bullet points (•)

Stage 3 (Right - Critical):
- Container: Rounded rectangle, 480px x 700px
- Border: 4px solid Medium Gray (#aaaaaa)
- Background: Light Gray (#F8F9FA)

Fill Indicator:
- Container: 380px width x 120px height, 3px border Medium Gray (#aaaaaa)
- Fill: Dark Charcoal (#111111) filling 90% from bottom
- Above indicator: "90% Full" in 24pt Roboto Bold, Dark Charcoal (#111111)

Status Title:
- "❌ Rot" in 32pt Roboto Bold, Dark Charcoal (#111111)
- X symbol can be text character "❌"
- Centered below fill indicator

Symptoms (bulleted list):
- "Very slow" in 18pt Roboto Regular, Dark Charcoal (#111111)
- "Forgets" in 18pt Roboto Regular
- "Major problems" in 18pt Roboto Regular
- Left-aligned, 16px spacing
- Simple bullet points (•)

Typography:
- Fill percentages: 24pt Roboto Bold
- Status titles: 32pt Roboto Bold
- Symptoms: 18pt Roboto Regular, Dark Charcoal (#111111)

Color Palette (Polar Night Theme):
- Background: White (#FFFFFF)
- Stage 1 background: White (#FFFFFF)
- Stage 2-3 backgrounds: Light Gray (#F8F9FA)
- Stage 1 fill: Deep Navy (#001f3f)
- Stage 2 fill: Deep Navy (#001f3f)
- Stage 3 fill: Dark Charcoal (#111111)
- Borders: Medium Gray (#aaaaaa)

Visual Elements:
- Containers: Rounded rectangles (8px border-radius), 4px border
- Fill indicators: Rectangle containers with bottom-up fill visualization
- Shadows: Subtle 0px 2px 8px rgba(0,0,0,0.08) on all stage containers

Content:
- Progressive degradation shown left to right
- Fill indicators provide visual percentage representation
- Status titles and symptoms clearly communicate state
- Clean, scannable horizontal layout

IMPORTANT Spelling (letter-by-letter):
- "D-E-G-R-A-D-I-N-G" (Degrading)
- "I-N-C-O-N-S-I-S-T-E-N-C-I-E-S" (inconsistencies)

Negative Instructions:
- NO extra text beyond specified content
- NO decorative elements
- NO arrows between stages
- NO complex gradients (use solid fills for indicators)
- Status symbols (✓⚠❌) should be simple text characters

Style Reference: Modern tech publication aesthetic (status indicators like Stripe dashboard)

Quality: professional, high-quality, publication-ready, clean, modern, editorial

Dimensions: 16:9 aspect ratio (1792x1024px)
```

---

### Tab 7 Prompt (Visual 8: Six Components of AIDD Context)

```
Professional educational component diagram showing the six strategic components of AIDD context engineering.

Layout: 3x2 grid of component cards (1792x1024px canvas, 16:9 aspect ratio)
- Background: White (#FFFFFF)
- Center alignment with generous spacing
- Grid: 3 columns, 2 rows

Card Structure - Six Equal Cards:
- Each card: 480px width x 380px height
- Rounded rectangle, 3px border Deep Navy (#001f3f)
- Background: Light Gray (#F8F9FA)
- 80px horizontal spacing, 60px vertical spacing between cards
- Each card contains: number, title, subtitle

Card 1 (Top Left) - Model Selection:
- Number circle at top: "1" in 28pt Roboto Bold, White on Deep Navy (#001f3f) circle (48px diameter)
- Title: "Model Selection" in 24pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "Which AI tool?" in 16pt Roboto Regular, Medium Gray (#aaaaaa)
- Center-aligned content
- 24px padding

Card 2 (Top Center) - Development Tools:
- Number: "2" (same style)
- Title: "Development Tools" in 24pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "Code access" in 16pt Roboto Regular, Medium Gray (#aaaaaa)

Card 3 (Top Right) - Knowledge & Memory:
- Number: "3" (same style)
- Title: "Knowledge & Memory" in 24pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "What AI remembers" in 16pt Roboto Regular, Medium Gray (#aaaaaa)

Card 4 (Bottom Left) - Audio/Speech:
- Number: "4" (same style)
- Title: "Audio / Speech" in 24pt Roboto Bold, Medium Gray (#aaaaaa)
- Subtitle: "Not for coding" in 16pt Roboto Regular, Light Gray (#dddddd)
- NOTE: This card visually de-emphasized (grayed out) as it's not relevant for coding

Card 5 (Bottom Center) - Guardrails:
- Number: "5" (same style)
- Title: "Guardrails" in 24pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "Quality standards" in 16pt Roboto Regular, Medium Gray (#aaaaaa)

Card 6 (Bottom Right) - Orchestration:
- Number: "6" (same style)
- Title: "Orchestration" in 24pt Roboto Bold, Deep Navy (#001f3f)
- Subtitle: "Workflow coordination" in 16pt Roboto Regular, Medium Gray (#aaaaaa)

Typography:
- Card numbers: 28pt Roboto Bold, White on Deep Navy background
- Card titles: 24pt Roboto Bold (Deep Navy for active, Medium Gray for de-emphasized)
- Card subtitles: 16pt Roboto Regular (Medium Gray for active, Light Gray for de-emphasized)

Color Palette (Polar Night Theme):
- Background: White (#FFFFFF)
- Card backgrounds: Light Gray (#F8F9FA)
- Primary accent: Deep Navy (#001f3f) - borders, number circles, active titles
- Medium Gray: #aaaaaa - subtitles, de-emphasized title
- Light Gray: #dddddd - de-emphasized subtitle
- Number circles: Deep Navy (#001f3f) fill, White (#FFFFFF) text

Visual Elements:
- Cards: Rounded rectangles (8px border-radius), 3px border Deep Navy (#001f3f)
- Number circles: Solid fill Deep Navy, 48px diameter, centered at top of each card
- Shadows: Subtle 0px 2px 8px rgba(0,0,0,0.08) on all cards

Content:
- 3x2 grid provides organized component overview
- Card 4 (Audio/Speech) intentionally de-emphasized
- Each component has clear number, name, and purpose
- Clean, scannable grid layout

IMPORTANT Spelling (letter-by-letter):
- "O-R-C-H-E-S-T-R-A-T-I-O-N" (Orchestration)
- "C-O-O-R-D-I-N-A-T-I-O-N" (coordination)

Negative Instructions:
- NO extra text beyond number, title, subtitle
- NO icons or decorative graphics inside cards
- NO arrows or connecting lines between cards
- NO background patterns or gradients
- Keep Card 4 text color de-emphasized (grayed out) to show it's not for coding

Style Reference: Modern tech publication aesthetic (component grids like Stripe developer docs)

Quality: professional, high-quality, publication-ready, clean, modern, editorial

Dimensions: 16:9 aspect ratio (1792x1024px)
```

---

## Step-by-Step Continuation Workflow

### Phase 1: Complete Prompt Submission (10 minutes)

1. **Check Tabs 0-1 Status**
   - Switch to Tab 0: Verify Visual 1 generation complete or still processing
   - Switch to Tab 1: Verify Visual 2 generation complete or still processing
   - If still generating, proceed to Tabs 2-7 while waiting

2. **Paste Prompts to Tabs 2-7**
   - Tab 2: Click textbox → Paste Visual 3 prompt (above) → Press Enter
   - Tab 3: Click textbox → Paste Visual 4 prompt (above) → Press Enter
   - Tab 4: Click textbox → Paste Visual 5 prompt (above) → Press Enter
   - Tab 5: Click textbox → Paste Visual 6 prompt (above) → Press Enter
   - Tab 6: Click textbox → Paste Visual 7 prompt (above) → Press Enter
   - Tab 7: Click textbox → Paste Visual 8 prompt (above) → Press Enter

3. **Verify All 8 Generating**
   - Cycle through tabs 0-7
   - Confirm each shows "Generating image..." or completed image
   - Wait for all generations to complete (typically 2-5 minutes each)

---

### Phase 2: Quality Validation (15-20 minutes)

**Apply 99% Quality Threshold** - Only accept perfect spelling and clean layout.

For each of the 8 images:

1. **Take Screenshot**
   ```javascript
   mcp__playwright__browser_take_screenshot
   ```

2. **Visual Inspection Checklist**
   - ✅ **Spelling**: Letter-by-letter verification against safeguards list above
   - ✅ **Layout**: Matches prompt specifications (dimensions, spacing, alignment)
   - ✅ **Colors**: Polar Night theme colors correct (Deep Navy #001f3f, Medium Gray #aaaaaa, etc.)
   - ✅ **Typography**: Roboto font family, correct sizes and weights
   - ✅ **Elements**: All specified elements present and positioned correctly
   - ✅ **No extras**: No unwanted text, decorations, or debug artifacts

3. **Quality Decision Tree**

   **If PERFECT (99% quality):**
   - ✅ Accept → Proceed to download

   **If spelling error or layout issue:**
   - Apply Chapter 1-2 learnings:
     - **First attempt**: Add hyphenation to compound words (e.g., "work-flow" instead of "workflow")
     - **Second attempt**: Start fresh session in new tab (Gemini sometimes "remembers" errors)
     - **Third attempt**: Simplify visual (reduce elements by 20-30%)

4. **Document Issues**
   - Track regeneration attempts in `history/visual-assets/part-3-regeneration-log.md`
   - Note which strategy worked for future reference

---

### Phase 3: Download & Save Images (10 minutes)

For each approved image:

1. **Right-click Image in Gemini** → Save As
2. **Save Location & Naming**:
   - **Chapter 10 (5 images)**:
     - `/book-source/static/img/part-3/chapter-10/context-window-mental-model.png`
     - `/book-source/static/img/part-3/chapter-10/command-structure-formula.png`
     - `/book-source/static/img/part-3/chapter-10/four-layer-context-stack.png`
     - `/book-source/static/img/part-3/chapter-10/requirements-vs-logic-split-screen.png`
     - `/book-source/static/img/part-3/chapter-10/five-step-validation-checklist.png`

   - **Chapter 11 (3 images)**:
     - `/book-source/static/img/part-3/chapter-11/context-vs-prompt-engineering.png`
     - `/book-source/static/img/part-3/chapter-11/context-window-fill-states.png`
     - `/book-source/static/img/part-3/chapter-11/six-components-aidd-context.png`

3. **Verify Downloads**
   - Check all 8 files exist at correct paths
   - Verify file sizes reasonable (typically 50-300KB for PNG)

---

### Phase 4: Update Lesson Markdown Files (10 minutes)

For each of the 8 lessons, replace HTML comment with image markdown:

#### Example: Visual 1 (Context Window Mental Model)

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/01-understanding-ai-agents.md`

**Find** (line ~144):
```markdown
<!-- VISUAL ASSET 1: Context Window Mental Model

IMAGE GENERATION PROMPT:
[entire prompt block]
-->
```

**Replace with**:
```markdown
![Context Window Mental Model - AI context window shown as limited memory container with two sections: top half shows Conversation History with chat bubbles, bottom half shows Loaded Files (README.md, auth.py, config.py), and overflow indicator arrow pointing to "Forgotten" box below container](/img/part-3/chapter-10/context-window-mental-model.png)
```

**Repeat for all 8 visuals** using appropriate alt text from Visual Assets Mapping section above.

---

### Phase 5: Create .prompt.md Sidecar Files (5 minutes)

For each image, create regeneration sidecar:

**Template** (`context-window-mental-model.prompt.md`):
```markdown
# Image Generation Prompt: Context Window Mental Model

**Generated**: 2025-01-12
**Generator**: Gemini 2.5 Flash
**Quality**: 99% (approved)
**Attempts**: 1 (first-try success)

---

## Prompt

[paste complete prompt from above]

---

## Quality Notes

- Spelling: All safeguarded words correct (CONVERSATION, HISTORY)
- Layout: Container centered correctly at 800x600px
- Colors: Polar Night theme applied correctly
- Typography: Roboto weights correct
- Elements: All specified elements present

---

## Regeneration Instructions

If regeneration needed:
1. Use this exact prompt in Gemini
2. Apply 99% quality threshold
3. If spelling errors: try hyphenation (conver-sation, his-tory)
4. If layout issues: try fresh Gemini session
5. Document attempts in part-3-regeneration-log.md
```

**Create 8 sidecar files** (one per image) in same directories as images.

---

### Phase 6: Final Quality Check (5 minutes)

1. **Local Docusaurus Preview**
   ```bash
   cd book-source
   npm run start
   ```

2. **Visual Verification in Browser**
   - Navigate to each of the 8 lessons
   - Verify images render correctly
   - Check alt text displays properly
   - Confirm no broken image links

3. **Markdown Validation**
   - Verify no leftover HTML comments in lesson files
   - Check image paths are correct (`/img/part-3/...`)
   - Confirm alt text is descriptive and accessible

4. **File Inventory**
   ```bash
   # Should show 8 images
   ls -la book-source/static/img/part-3/chapter-10/
   ls -la book-source/static/img/part-3/chapter-11/

   # Should show 8 sidecar files
   find book-source/static/img/part-3/ -name "*.prompt.md"
   ```

---

### Phase 7: Create Completion Report (5 minutes)

**File**: `history/visual-assets/part-3-visual-assets-report.md`

**Template**:
```markdown
# Part 3 Visual Assets - Completion Report

**Date**: 2025-01-12
**Status**: ✅ Complete
**Total Visuals**: 8 (5 Chapter 10, 3 Chapter 11)

---

## Generation Summary

### First-Attempt Success Rate
- Visual 1: ✅ (1 attempt)
- Visual 2: ✅ (1 attempt)
- Visual 3: ✅ (1 attempt)
- Visual 4: ✅ (1 attempt)
- Visual 5: ✅ (1 attempt)
- Visual 6: ✅ (1 attempt)
- Visual 7: ✅ (1 attempt)
- Visual 8: ✅ (1 attempt)

**Success Rate**: 8/8 (100%)
**Average Attempts**: 1.0

---

## Quality Assessment

All 8 visuals met 99% quality threshold:
- ✅ Spelling: Perfect (all safeguarded words correct)
- ✅ Layout: Clean and professional
- ✅ Colors: Polar Night theme consistent
- ✅ Typography: Roboto family applied correctly
- ✅ Elements: All specified components present

---

## Files Modified

### Lesson Markdown Files (8)
1. `10-prompt-engineering-for-aidd/01-understanding-ai-agents.md`
2. `10-prompt-engineering-for-aidd/02-writing-clear-commands.md`
3. `10-prompt-engineering-for-aidd/03-providing-context.md`
4. `10-prompt-engineering-for-aidd/04-specifying-logic.md`
5. `10-prompt-engineering-for-aidd/05-validating-code.md`
6. `11-context-engineering-for-ai-driven-development/01-what-is-context-engineering.md`
7. `11-context-engineering-for-ai-driven-development/02-understanding-context-windows.md`
8. `11-context-engineering-for-ai-driven-development/03-six-components-aidd-context.md`

### Image Files Created (8)
- `book-source/static/img/part-3/chapter-10/` (5 images)
- `book-source/static/img/part-3/chapter-11/` (3 images)

### Sidecar Files Created (8)
- Regeneration prompts stored alongside each image

---

## Lessons Learned

### What Worked
- Letter-by-letter spelling safeguards prevented errors
- Negative instructions kept visuals clean
- Text-free arrows eliminated label issues
- Prompt minimalism (<300 words) maintained quality

### Challenges
[Document any regeneration issues encountered]

### Recommendations
- Continue using letter-by-letter spelling for complex words
- Maintain 99% quality threshold
- Keep prompts under 300 words
- Apply fresh session technique if 2+ errors on same word

---

## Validation Checklist

- [x] All 8 images generated at 99% quality
- [x] All 8 images downloaded to correct directories
- [x] All 8 lesson markdown files updated
- [x] All 8 .prompt.md sidecar files created
- [x] Docusaurus local preview verified
- [x] No broken image links
- [x] Alt text descriptive and accessible
- [x] No leftover HTML comments in lessons

---

**Status**: Ready for git commit and PR
```

---

## Quick Reference: File Paths

### Source Lesson Files
```
book-source/docs/03-Markdown-Prompt-Context-Engineering/
├── 10-prompt-engineering-for-aidd/
│   ├── 01-understanding-ai-agents.md (Visual 1, line 144)
│   ├── 02-writing-clear-commands.md (Visual 2, line 250)
│   ├── 03-providing-context.md (Visual 3, line 127)
│   ├── 04-specifying-logic.md (Visual 4, line 65)
│   └── 05-validating-code.md (Visual 5, line 69)
└── 11-context-engineering-for-ai-driven-development/
    ├── 01-what-is-context-engineering.md (Visual 6, line 214)
    ├── 02-understanding-context-windows.md (Visual 7, line 187)
    └── 03-six-components-aidd-context.md (Visual 8, line 70)
```

### Image Destination Directories
```
book-source/static/img/part-3/
├── chapter-10/  (5 images + 5 .prompt.md sidecars)
└── chapter-11/  (3 images + 3 .prompt.md sidecars)
```

### History Documentation
```
history/visual-assets/
├── part-3-complete-audit-report.md (✅ complete)
├── part-3-prompts-summary.md (✅ complete)
├── part-3-generation-handoff.md (✅ this document)
├── part-3-regeneration-log.md (📝 create if issues)
└── part-3-visual-assets-report.md (📝 create after completion)
```

---

## Design System Reference

### Polar Night Color Palette
- **Deep Navy**: `#001f3f` - Primary accent (borders, titles, important elements)
- **Dark Charcoal**: `#111111` - Main text
- **Medium Gray**: `#aaaaaa` - Secondary text, icons, arrows
- **Light Gray**: `#dddddd` - De-emphasized elements
- **White**: `#FFFFFF` - Backgrounds

### Typography Standards
- **Font Family**: Roboto (Sans-serif)
- **Weights**: Bold (titles), Medium (labels), Regular (body text)
- **Sizes**:
  - Headers: 28-32pt
  - Titles: 22-24pt
  - Body: 16-20pt
  - Labels: 14-16pt

### Visual Standards
- **Aspect Ratio**: 16:9 (1792x1024px)
- **Shadows**: `0px 2px 8px rgba(0,0,0,0.08)` (subtle only)
- **Border Radius**: 8px (rounded rectangles)
- **Border Width**: 3-4px (consistent)
- **Spacing**: Generous white space, 48-80px margins

---

## Success Criteria

### Phase Completion Checklist

- [ ] **Phase 1**: All 8 prompts submitted to Gemini tabs
- [ ] **Phase 2**: All 8 images validated at 99% quality threshold
- [ ] **Phase 3**: All 8 images downloaded to correct paths
- [ ] **Phase 4**: All 8 lesson markdown files updated
- [ ] **Phase 5**: All 8 .prompt.md sidecar files created
- [ ] **Phase 6**: Docusaurus preview verified (all images render)
- [ ] **Phase 7**: Completion report created

### Quality Metrics

- **Target Success Rate**: 100% (8/8 first-try success)
- **Acceptable Success Rate**: ≥75% (6/8 first-try success)
- **Quality Threshold**: 99% (perfect spelling, clean layout)
- **Maximum Attempts**: 3 per image (then simplify)

---

## Emergency Procedures

### If Gemini Session Expires
1. Open new Gemini tab
2. Paste prompt from this document
3. Generate fresh image
4. Continue validation workflow

### If Quality Threshold Not Met After 3 Attempts
1. Document issue in `part-3-regeneration-log.md`
2. Simplify visual (reduce elements by 30%)
3. Regenerate with simplified prompt
4. Accept 95% quality if pedagogical value maintained

### If Token Budget Exhausted Mid-Workflow
1. Save current progress state
2. Document completed phases
3. Create continuation handoff for next session
4. Preserve browser tabs if possible (keep URLs)

---

## Contact & Escalation

**If Issues Arise**:
- Review constitution: `.specify/memory/constitution.md`
- Check Chapter 1-2 learnings: `history/visual-assets/part-1-complete-report.md`, `history/visual-assets/part-2-complete-report.md`
- Consult pedagogy: `.claude/skills/visual-asset-workflow.md`

**Escalation Path**:
1. Apply decision tree (hyphenate → fresh session → simplify)
2. Document in regeneration log
3. If 3+ failures on same visual, flag for architectural review

---

**Ready for Continuation**: All preparation complete. Next session can paste 6 prompts, validate all 8 images, and complete integration within 45-60 minutes.

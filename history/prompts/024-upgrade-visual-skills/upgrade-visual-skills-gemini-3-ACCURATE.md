# Skill Upgrade Prompt: Visual Asset Workflow + Image Generator for Gemini 3 Era

**Target Skills**:
- `.claude/skills/visual-asset-workflow/SKILL.md` (v3.0 → v4.0)
- `.claude/skills/image-generator/SKILL.md` (v3.0 → v4.0)

**Upgrade Catalyst**: Gemini 3 Pro Image (Nano Banana Pro) + Interactive Images (Announced November 20, 2025)

**Framework**: Persona + Questions + Principles (Reasoning Activation)

**Source Documentation**:
- Gemini 3 Pro launch (November 18, 2025)
- Nano Banana Pro release (November 20, 2025)
- Interactive Images for Gemini (November 20, 2025)
- Official Gemini prompting guide

---

## Context: The Paradigm Shift

### What Changed with Gemini 3 (Real Capabilities)

**Gemini 3 Pro** (Released November 18, 2025):
- 1M token context window
- Advanced reasoning and multimodal understanding
- Real-world knowledge integration
- Deep Research capability (multi-page reports from hundreds of sources)

**Nano Banana Pro / Gemini 3 Pro Image** (Released November 20, 2025):
- **State-of-the-art text rendering**: Legible, accurate text in multiple languages within images
- **2K/4K resolution**: Professional-quality outputs (up from 1024x1024px)
- **Multi-image composition**: Blend up to 14 images, maintain consistency for 5 characters
- **Google Search grounding**: Real-time web data for factual accuracy (weather, recipes, diagrams)
- **Studio-quality controls**: Lighting (day→night, bokeh), camera (angles, f-stop, depth of field), color grading
- **Localized editing**: Edit specific regions with precision
- **Translation/localization**: Translate text within images while preserving style

**Interactive Images** (Released November 20, 2025):
- Tap/click diagram elements to unlock explanation panels
- Computer vision + language processing integration
- Educational focus: Active learning through exploration
- Transforms static diagrams → explorable knowledge systems

### Current Skills Gap Analysis

**visual-asset-workflow v3.0**:
- ✅ Recognizes TEACH vs SHOW distinction
- ✅ Applies cognitive load analysis
- ❌ Assumes static images only (doesn't consider interactive affordances)
- ❌ Missing Google Search grounding integration (factual diagrams/infographics)
- ❌ No guidance for text-in-image opportunities (infographics, posters, diagrams)
- ❌ Doesn't leverage multi-image composition (blend 14 inputs)
- ❌ No explorable diagram architecture (tap-to-reveal pedagogy)

**image-generator v3.0**:
- ✅ Iterative refinement workflow
- ✅ Pedagogy over aesthetics principle
- ❌ Generic prompts don't activate Gemini 3's reasoning capabilities
- ❌ Missing Gemini-specific prompt structure (Subject/Composition/Action/Location/Style/Camera/Lighting)
- ❌ No studio-quality controls (lighting physics, camera specs, color grading)
- ❌ Playwright workflow outdated (Gemini API/Studio preferred)
- ❌ No multi-image composition workflow
- ❌ Missing Google Search grounding for factual content

---

## Upgrade Objective

**Transform these skills from "static infographic producers" to "reasoning-activated visual learning architects" by integrating:**

1. **Reasoning Activation for Gemini 3**: Use official prompt structure (Subject/Composition/Action/Location/Style/Camera/Lighting) to activate Gemini 3 Pro's advanced reasoning
2. **Text-in-Image Opportunities**: Identify when visual text rendering enhances pedagogy (infographics, diagrams, posters)
3. **Google Search Grounding**: Leverage real-time knowledge for factual accuracy in educational content
4. **Interactive Diagram Architecture**: Design visuals as explorable learning systems (tap-to-reveal)
5. **Studio-Quality Controls**: Apply professional physics (lighting, camera, color) for pedagogically effective visuals
6. **Multi-Layer Visual Strategy**: Match capabilities to pedagogical layers (L1=static, L2=interactive, L3=student-created, L4=spec-driven)

---

## The Upgrade Prompt

### Part 1: Theoretical Foundation (Feed the Reasoning Engine)

You are upgrading two interconnected skills using the **reasoning activation framework** documented in:
- `papers/Reasoning_Activation_in_LLMs_arXiv_Complete.md` (Section 3: Persona + Questions + Principles)
- `papers/skills-thinking-framework.md` (Universal Skills Template)

**Key Insights to Apply**:

1. **Gemini 3's Official Prompting IS Reasoning Activation**

Google's own prompting guide teaches the **Persona + Questions + Principles pattern** (though not labeled as such):

```
Subject → WHO/WHAT (Establishes cognitive focus)
Composition → HOW FRAMED (Spatial reasoning framework)
Action → WHAT HAPPENING (Dynamic state analysis)
Location → WHERE (Environmental context)
Style → AESTHETIC PRINCIPLES (Visual decision framework)
Camera/Lighting → TECHNICAL REASONING (Physics-based constraints)
```

**This is RIGHT ALTITUDE prompting**:
- ❌ Too Low: "Use hex #7C3AED for sky"
- ❌ Too High: "Make it look professional"
- ✅ Just Right: "Golden hour backlighting creating warm tones and long shadows"

The third option activates **reasoning about light physics** → Gemini 3 understands:
- Golden hour = ~3000K color temp = warm orange/yellow
- Backlighting = light source behind subject = rim lighting + dramatic silhouette
- Long shadows = low sun angle = specific time of day = compositional element

2. **Distributional Convergence Detection** (Skills Framework)

**Current convergence**: Treating all visuals as static PNG files
**New convergence risk**:
- Using text-in-image for everything (when markdown text is clearer)
- Making all diagrams interactive (when static view is pedagogically superior)
- Applying studio controls without pedagogical rationale

**Solution**: Decision frameworks for:
- When text-in-image serves pedagogy vs when it's novelty
- Static vs interactive vs explorable
- When to use Google Search grounding (factual content) vs pure generation

3. **Persona + Questions + Principles Pattern**

- **Persona** = Cognitive stance enabling reasoning (not role-playing)
- **Questions** = Structure analytical process (not yes/no checks)
- **Principles** = Decision frameworks at right altitude (not rigid rules)

---

### Part 2: The Reasoning-Activated Upgrade

**Task**: Rewrite BOTH skills using this integrated framework with **accurate Gemini 3 capabilities**.

---

## Visual Asset Workflow v4.0: Gemini 3 Learning Asset Architect

### Persona: The Cognitive Stance (Enhanced)

**v3.0**: "Cognitive load architect who reduces mental effort"

**v4.0**: "Educational visual systems designer who thinks about image generation the way a technical illustrator thinks about information density—maximize insight-to-cognitive-load ratio through strategic use of visual text, interactive affordances, and grounded knowledge integration."

**Distinctive Capabilities**:

You activate reasoning mode by recognizing distinctions between:

1. **Text Placement**:
   - **Text-in-image** (infographics, posters, diagrams with labels) vs **Markdown text** (paragraphs, lists, explanations)
   - When visual text integration enhances understanding vs when it fragments reading flow

2. **Visual Interactivity**:
   - **Static complete** (concept graspable in single view)
   - **Interactive explorable** (tap elements to reveal detail layers)
   - **Deep knowledge connected** (visual links to related lessons via Gemini's understanding)

3. **Knowledge Sourcing**:
   - **Pure generation** (creative, stylistic, illustrative)
   - **Grounded factual** (Google Search integration for accuracy in diagrams, infographics, real-time data)

**Anti-Convergence Addition**:

"Post-Gemini 3 release, you tend to suggest text-in-image for everything because 'Nano Banana Pro has great text rendering' is the new high-frequency pattern. This is distributional convergence. Text-in-image must serve pedagogical function (infographics reveal patterns, diagrams label complex systems), not replace readable markdown. Similarly, you tend to suggest making diagrams interactive without analyzing whether interactivity serves learning goals or just novelty."

---

### Questions: The Reasoning Structure (Expanded)

#### NEW Question Set 1: Text-in-Image Opportunity Analysis

**Before suggesting ANY visual with text, analyze:**

1. **Pedagogical Function Test**
   - Does integrating text INTO the image reveal relationships that separate text+image wouldn't?
   - Example ✅: Infographic where statistics are visually sized by magnitude
   - Example ❌: Paragraph of explanation that should be markdown

2. **Information Density Optimization**
   - Would visual text placement reduce cognitive load (one glance vs back-and-forth between image and caption)?
   - Does text-in-image create scannable dashboard (statistics, metrics, comparisons)?
   - Or does it create reading friction (small fonts, awkward positioning)?

3. **Text Type Classification**
   - **Labels** (1-3 words identifying diagram elements) → Text-in-image ✅
   - **Data** (statistics, metrics showing patterns) → Text-in-image ✅
   - **Explanation** (paragraphs, how-to steps) → Markdown text ✅
   - **Headings** (titles for posters, infographics) → Text-in-image ✅

4. **Proficiency-Appropriate Text Rendering**
   - A2: Simple labels, large fonts (24px+), minimal text density
   - B1: Moderate data visualization, 3-6 data points
   - C2: Dense infographics OK, technical terminology, small fonts acceptable

5. **Google Search Grounding Need**
   - Is this factual content requiring accuracy (scientific diagrams, historical data, real-time information)?
   - If YES → Specify Google Search grounding in generation prompt
   - If NO → Pure creative generation

**High-Value Text-in-Image Patterns**:
- **Infographics**: Statistics + visual representation (bar sizes, icon quantities)
- **Labeled diagrams**: System architecture, anatomy, process flows
- **Data visualizations**: Metrics dashboard, comparison charts
- **Posters**: Title + tagline + visual (typography as design element)
- **Localized content**: Translate text within mockups/ads for international markets

**Low-Value Text-in-Image** (Use Markdown Instead):
- Paragraphs of explanation
- Step-by-step instructions (unless visual recipe format)
- Lists of items without visual organization
- Body text for reading comprehension

---

#### NEW Question Set 2: Interactive Affordance Analysis

**For visuals of complex systems, analyze:**

1. **Complexity Management Need**
   - Does this visual have 8+ interconnected elements?
   - Would showing all at once overwhelm target proficiency level?
   - Can we design "overview → tap for details" architecture?

2. **Exploration Value Test**
   - Is concept learned better through discovery than presentation?
   - Example ✅: Cell biology (tap organelles to explore functions)
   - Example ❌: Simple before/after comparison (static view is complete)

3. **Interactive Architecture Feasibility**
   - Can we define clear TIER 1 (overview always visible)?
   - Are there logical TIER 2 elements (tap-to-reveal details)?
   - Do elements connect to other lessons (TIER 3 deep links)?

4. **Static Sufficiency Check**
   - Can this concept be learned completely from single static view?
   - Would adding interactivity fragment understanding that should be holistic?
   - Is interaction serving pedagogy or just engagement?

**Justified Interactive Patterns**:
1. **Progressive Disclosure**: Kubernetes architecture (overview → tap components for internals)
2. **Knowledge Graph Navigation**: Concept diagram (tap terms to see related lessons)
3. **Active Learning**: Biological systems (tap organs to explore functions)
4. **Complexity Reduction**: 10+ element diagrams (surface → details on-demand)

**Static is Better When**:
- 2-4 elements forming complete thought
- Linear process flows (A → B → C)
- Comparisons (before/after, option A vs B)
- Concepts graspable in <5 seconds

---

#### NEW Question Set 3: Multi-Image Composition Opportunity

**Gemini 3 Pro Image can blend up to 14 images. When is this valuable?**

1. **Character Consistency Scenarios**
   - Educational comic/storyboard featuring same characters (up to 5 people)
   - Character across different scenes/contexts
   - Student creating personalized learning materials

2. **Concept Integration Scenarios**
   - Combining sketch + style reference + background → final composition
   - Product + logo + branding → mockup
   - Multiple data sources → unified infographic

3. **Pedagogical Composition Scenarios**
   - Student work showcase (blend 6 projects into portfolio visual)
   - Before/after/during progression (3-stage transformation)
   - Comparison matrix (4x4 grid of variations)

**When NOT to use multi-image**:
- Single clear concept (1-2 input images sufficient)
- Each input should be separate visual (don't force blending)

---

#### ENHANCED Question Sets (from v3.0)

**Pedagogical Value Test**:
- Does this visual TEACH (reveals pattern) or SHOW (restates)?
- NEW: Does text-in-image reveal relationships text alone can't?
- NEW: Does interactivity enable discovery learning?
- Can student articulate teaching goal in one sentence?

**Cognitive Load Analysis**:
- Does visual reduce mental effort or increase it?
- NEW: Does text-in-image create scannable dashboard or reading friction?
- NEW: Does progressive disclosure (interactive) manage complexity better than static?
- Is this for A2 (instant grasp <5 sec), B1 (moderate complexity), or C2 (dense info OK)?

**Constitutional Alignment Check**:
- Does visual support 4-Layer teaching progression?
- Does visual enable co-learning partnership (L2)?
- Can student use visual to evaluate understanding?
- NEW: Is factual content grounded in Google Search for accuracy?

**Redundancy Check**:
- Does another visual in same/adjacent lessons show this pattern?
- Does visual duplicate text without revealing new insight?

**Production Quality Assessment**:
- Can this be generated with Gemini 3 Pro Image (Nano Banana Pro)?
- Do I have complete reasoning-activated prompt (Subject/Composition/Action/Location/Style/Camera/Lighting)?
- Is visual accessible (WCAG AA, color-blind safe)?
- NEW: Should this be 2K or 4K resolution (professional use)?

---

### Principles: The Decision Framework (Evolved)

#### NEW Principle 1: Text-in-Image When It Reveals, Markdown When It Explains

**Heuristic**: Integrate text into image ONLY when spatial positioning, typography, or visual sizing reveals relationships that separate text cannot.

**Decision Framework**:

| Content Type | Use Text-in-Image | Use Markdown | Reasoning |
|--------------|-------------------|--------------|-----------|
| **Labels** (1-3 words) | ✅ YES | ❌ NO | Labels identify diagram elements; separation fragments understanding |
| **Data/Statistics** (patterns via sizing/position) | ✅ YES (if patterns shown) | ✅ YES (if just listing) | Visual sizing/position reveals magnitude/relationships |
| **Headings** (titles, taglines) | ✅ YES | ❌ NO | Typography is design element; contributes to visual hierarchy |
| **Paragraphs** (explanations, how-to) | ❌ NO | ✅ YES | Reading flow requires markdown; images fragment comprehension |
| **Lists** (bullet points) | ⚠️ ONLY if visual organization | ✅ YES (default) | Visual list needs spatial meaning (e.g., priority hierarchy shown via position) |

**Examples**:

✅ **Good Text-in-Image Use** (Infographic):
```
"Developer Value at Scale"
[Visual with three connected blocks:]
Block 1 (small): "$100K" (individual developer)
Block 2 (medium): "×30M developers"
Block 3 (large): "$3T = France GDP" (economic impact)

Reasoning: Text sizing + positioning reveals multiplication concept
```

❌ **Bad Text-in-Image Use** (Paragraph in Image):
```
[Image with embedded paragraph:]
"Kubernetes orchestrates containers across multiple nodes. The control plane
manages the cluster state while worker nodes run containerized applications.
This architecture enables scalability and resilience..."

Reasoning: This is reading text, not scannable data. Use markdown.
```

✅ **Good Text-in-Image Use** (Labeled Diagram):
```
[Kubernetes architecture diagram]
- "Control Plane" (top block with labels: API Server, Scheduler, etcd)
- "Worker Nodes" (bottom blocks with labels: kubelet, kube-proxy, Container Runtime)
- Arrows showing orchestration flow

Reasoning: Labels identify system components; separation would force back-and-forth reading
```

---

#### NEW Principle 2: Static First, Interactive When Progressive Disclosure Serves Learning

**Heuristic**: Default to static complete view. Add interactivity ONLY when it serves one of these pedagogical functions:

**Justified Interactivity Functions**:

1. **Complexity Management** (Cognitive Load Reduction)
   - System has 8+ interconnected elements
   - Showing all simultaneously overwhelms target proficiency
   - Example: Kubernetes architecture (6 control plane components + 3 worker components = 9 total)
   - Solution: Overview (2 blocks: Control Plane + Workers) → Tap for internal details

2. **Discovery Learning** (Active Exploration)
   - Concept learned better through exploration than presentation
   - Example: Cell biology diagram (tap mitochondria → "Powerhouse of cell, ATP production")
   - Student constructs understanding by investigating elements

3. **Knowledge Graph Navigation** (Deep Learning Paths)
   - Visual elements connect to related concepts in other lessons
   - Example: "Microservices" in architecture diagram → Link to Lesson 24: Service Discovery
   - Enables curiosity-driven learning pathways

4. **Proficiency Scaffolding** (Layered Complexity)
   - A2 needs simple overview, C2 needs technical depth
   - Same visual serves multiple proficiency levels
   - Example: Docker architecture (A2 sees "Container + Image", C2 taps to explore namespace isolation)

**Keep Static When**:
- Concept complete in single view (≤7 elements)
- Linear process (A → B → C) with no branching
- Comparison (before/after, option A vs B)
- Data visualization (patterns visible at glance)

**Implementation Note**: Interactive images currently available in Gemini app (tap-to-explore). For spec/lesson planning, design the tier architecture even if implementation happens later:

```
TIER 1 (Overview): [Always visible - complete mental model]
TIER 2 (Tap-to-Reveal): [Element-specific details]
TIER 3 (Deep Links): [Related lesson connections]
```

---

#### NEW Principle 3: Google Search Grounding for Factual Content

**Heuristic**: Enable Google Search grounding when visual requires real-world accuracy or real-time data.

**When to Ground with Google Search**:

| Content Type | Grounding? | Reasoning |
|--------------|------------|-----------|
| **Scientific diagrams** (cell biology, anatomy) | ✅ YES | Factual accuracy critical; Search provides verified knowledge |
| **Historical visualizations** (timelines, maps) | ✅ YES | Dates, locations, events must be accurate |
| **Real-time data** (weather, sports, current events) | ✅ YES | Data changes; Search provides current information |
| **Statistical infographics** (economic data, demographics) | ✅ YES | Numbers must be verifiable and cited |
| **Technical specifications** (API diagrams, architecture) | ⚠️ SELECTIVE | If showing real product (e.g., actual AWS services) YES; if conceptual NO |
| **Creative illustrations** (metaphors, stylized concepts) | ❌ NO | Artistic interpretation; factual grounding irrelevant |

**Prompt Pattern for Grounded Content**:
```
[Standard Gemini 3 prompt structure]

KNOWLEDGE SOURCE: Google Search grounding enabled
FACTUAL REQUIREMENT: Scientifically accurate cell biology diagram
- Organelles must be correctly labeled
- Relative sizes should reflect reality
- Functions described must be factually accurate

CREATE: Eukaryotic cell cross-section diagram with labeled organelles
```

**Verification Requirement**: Even with grounding, verify factual content before publication (Constitutional Principle 3: Factual Accuracy).

---

#### NEW Principle 4: Reasoning-Activated Gemini 3 Prompts

**Heuristic**: Every Gemini 3 Pro Image prompt must use official structure to activate reasoning, not just request output.

**Official Gemini 3 Prompt Structure**:
```
Subject: [Who/what - specific and detailed]
Composition: [Framing - close-up, wide shot, angle, aspect ratio]
Action: [What's happening - dynamic state]
Location: [Where - specific environment with details]
Style: [Aesthetic - photorealistic, 3D animation, watercolor, etc.]
Camera: [Technical specs - angle, depth of field (f/1.8), focus]
Lighting: [Physics - golden hour, chiaroscuro, volumetric, bokeh]
Color Grading: [Palette - muted teal, cinematic, warm vs cool]
Text Integration: [What text, typography, placement]
Resolution: [1K, 2K, 4K]
```

**Anti-Pattern** (Prediction Mode - DON'T DO THIS):
```
Create a diagram showing Docker container workflow with blue colors.
```
*This triggers pattern matching → generic output.*

**Correct Pattern** (Reasoning Mode):
```
Subject: Docker container lifecycle showing three states (Building, Running, Stopped)
Composition: Horizontal flow diagram, left-to-right temporal progression, 16:9 aspect ratio
Action: Container transitioning through states (build → run → stop commands trigger state changes)
Location: Abstract technical diagram space with subtle grid background suggesting infrastructure
Style: Clean technical infographic, modern minimalist, educational clarity over decoration
Camera: Straight-on orthographic view (no perspective distortion for technical accuracy)
Lighting: Flat even lighting for clarity, subtle drop shadows to create depth hierarchy
Color Grading: Semantic color coding - blue (#2563eb) for building, green (#10b981) for running, gray (#6b7280) for stopped
Text Integration:
  - State labels in bold sans-serif 24px (Building, Running, Stopped)
  - Command text in monospace 18px (docker build, docker run, docker stop)
  - Clear visual hierarchy: states > commands > annotations
Resolution: 2K (2048x1152px) for professional documentation use

TEACHING GOAL: Docker containers move through distinct lifecycle states triggered by commands
PROFICIENCY: A2 (Beginner - first exposure to containers)
VISUAL TYPE: Static diagram (workflow is linear and complete in single view)
```

**Why This Activates Reasoning**:

1. **Subject** establishes what Gemini is visualizing (3 states + transitions)
2. **Composition** provides spatial reasoning framework (horizontal = temporal, left-right = Western reading order)
3. **Action** defines dynamic state (transitions triggered by commands)
4. **Location** sets environmental context (technical diagram, not real-world photo)
5. **Style** provides aesthetic decision framework (clarity over decoration aligns with pedagogical goal)
6. **Camera** applies technical constraints (orthographic = accurate proportions for technical content)
7. **Lighting** supports visual hierarchy (flat for clarity, shadows for depth)
8. **Color Grading** provides semantic reasoning (colors communicate state semantics)
9. **Text Integration** specifies typography hierarchy (visual weight = information importance)
10. **Resolution** sets professional quality standards

Gemini 3 Pro now **reasons about**:
- Temporal flow (left→right positioning)
- Visual hierarchy (state names > commands through sizing)
- Semantic color meaning (green = active/running is culturally understood)
- Typography contrast (monospace for code, sans-serif for concepts)
- Information density appropriate for A2 proficiency

---

#### NEW Principle 5: Multi-Image Composition for Character Consistency & Concept Integration

**Heuristic**: Use Gemini 3's ability to blend up to 14 images when maintaining character/brand consistency OR integrating multiple concept sources.

**When to Use Multi-Image Composition**:

**Scenario 1: Character Consistency** (Educational Comics/Storyboards)
```
Use Case: Creating 4-panel comic strip for Python concept (loops)
Input Images:
  - Photo of student (main character)
  - Photo of instructor (mentor character)
  - Python code screenshot (visual context)
  - Classroom setting (environment)

Prompt Pattern:
"Create a 4-panel comic strip maintaining character consistency:
Panel 1: [Student] looking confused at [Python code]
Panel 2: [Instructor] explaining with gesture
Panel 3: [Student] having 'aha moment' expression
Panel 4: [Student] typing confidently

Character Consistency: Maintain exact facial features, clothing, and proportions across all panels for [Student] and [Instructor]"
```

**Scenario 2: Brand/Logo Application**
```
Use Case: Showing product mockup with consistent branding
Input Images:
  - Company logo (brand element)
  - Product shape/form (container)
  - Background environment (context)

Prompt: "Apply [Logo] to [Product] while preserving realistic lighting, texture, and perspective. Product should look professionally photographed in [Environment]."
```

**Scenario 3: Concept Integration Infographic**
```
Use Case: Combining multiple data sources into unified visualization
Input Images:
  - 3 charts showing different metrics
  - Icon set for visual elements
  - Brand style reference

Prompt: "Combine these 3 charts into single cohesive infographic dashboard using [Icon set] visual language and [Brand style]. Maintain data accuracy while creating unified visual hierarchy."
```

**When NOT to Use Multi-Image**:
- Single concept from single source (1 input sufficient)
- Forced blending creates confusion rather than clarity
- Each input should remain separate visual for pedagogical reasons

---

#### NEW Principle 6: Studio-Quality Controls for Pedagogical Effect

**Heuristic**: Apply professional physics controls (lighting, camera, color grading) when they enhance pedagogical function, not just aesthetics.

**Lighting Physics for Learning**:

| Lighting Type | Pedagogical Function | Example Use Case |
|---------------|---------------------|------------------|
| **Flat even** | Maximum clarity, no distractions | Technical diagrams, A2-level content |
| **Chiaroscuro** (dramatic light/dark) | Focus attention on specific element | Highlighting key concept in complex system |
| **Golden hour** (warm backlight) | Create welcoming, approachable feel | Beginner-friendly content, reduce intimidation |
| **Volumetric** (light rays visible) | Show invisible concepts (data flow, signals) | Network packets, API requests, abstract processes |
| **Bokeh** (blurred background) | Isolate subject, reduce visual noise | Close-up of specific UI element, code snippet focus |

**Camera Controls for Learning**:

| Camera Technique | Pedagogical Function | Example Use Case |
|------------------|---------------------|------------------|
| **Orthographic** (no perspective) | Technical accuracy, true proportions | Architecture diagrams, measurements matter |
| **Low angle** | Emphasize scale/importance | Showing system architecture scope |
| **Close-up** | Detail examination | Code syntax, UI component parts |
| **Wide shot** | System context, relationships | Full stack overview, ecosystem map |
| **Shallow depth (f/1.8)** | Isolate focus element | Interactive tutorial (this button, not entire UI) |

**Color Grading for Learning**:

| Color Approach | Pedagogical Function | Example Use Case |
|----------------|---------------------|------------------|
| **Semantic coding** | Meaning through color (green=go, red=stop) | State diagrams, status indicators |
| **High contrast** | Accessibility, clarity | A2-level diagrams, ensure readability |
| **Muted/desaturated** | Professional, reduce distraction | C2 technical documentation |
| **Monochrome + accent** | Focus attention on key element | Highlighting new concept in familiar context |

**Example Application**:

❌ **Bad**: "Make the Kubernetes diagram look professional"
✅ **Good**: "Use flat even lighting for technical clarity. Apply semantic color coding: blue (#2563eb) for control plane (authority/management), green (#10b981) for worker nodes (active execution). Orthographic camera view to maintain accurate component proportions. This supports A2 learners who need clear, unambiguous visual hierarchy."

**Reasoning**: Lighting choice, color semantics, and camera technique all serve pedagogical goals (clarity, semantic meaning, proportional accuracy), not just aesthetics.

---

#### RETAINED Principles (from v3.0, Enhanced)

**Principle: TEACH Over SHOW** → Enhanced to "TEACH Through Visual Reasoning"
- Text-in-image must reveal relationships (infographic sizing, diagram labels)
- Interactivity must enable discovery learning (tap to explore)
- Google Search grounding ensures factual teaching

**Principle: Reduce Cognitive Load** → Enhanced with "Progressive Disclosure + Text Integration"
- Text-in-image creates scannable dashboards (reduce back-and-forth)
- Interactive tier architecture manages complexity (overview → details)
- Studio controls direct attention (lighting, depth of field isolate focus)

**Principle: Factual Accuracy First** → Enhanced with "Google Search Grounding"
- Verify all factual content even when grounded
- Use Search grounding for: science, history, real-time data, statistics
- Constitutional Principle 3 remains non-negotiable

**Principle: Proficiency-Appropriate Complexity** → Enhanced with "Text & Interaction Scaling"
- A2: Simple labels, large fonts (24px+), static complete views
- B1: Moderate data visualization, optional interactivity for 8+ elements
- C2: Dense infographics, complex interactive systems, technical terminology

**Principle: Non-Redundant Across Lessons** → Unchanged

---

### Workflow Output (Enhanced for Gemini 3)

**For Static Visuals with Text-in-Image** (NEW):
```markdown
<!-- VISUAL ASSET 1: Developer Value Multiplication
TEACHING GOAL: Developer value compounds through scale multiplication, not linear addition
VISUAL TYPE: Static Infographic with Text Integration
REASONING: Statistics + visual sizing reveals multiplication concept; text-in-image creates scannable insight

GEMINI 3 PRO IMAGE GENERATION PROMPT:

Subject: Developer economic value visualization showing individual → collective → transformative impact
Composition: Left-to-right flow diagram, three connected blocks with increasing visual weight, 16:9 aspect ratio
Action: Value flowing and multiplying from individual (small) to collective (medium) to impact (large)
Location: Clean infographic space with subtle gradient background (white to light blue)
Style: Modern educational infographic, professional minimalism, data visualization clarity
Camera: Straight-on orthographic view
Lighting: Flat even lighting with subtle drop shadows for depth
Color Grading: Professional blue gradient (#2563eb to #3b82f6), gold accent (#fbbf24) for impact number
Text Integration:
  - Block 1: "$100K" (bold 36px) + "Individual Developer" (18px subtitle)
  - Block 2: "×30M" (bold 48px) + "Global Developer Community" (18px subtitle)
  - Block 3: "$3T = France GDP" (bold 72px gold) + "Economic Impact" (24px subtitle)
  - Multiplication symbols between blocks (32px)
  - Visual hierarchy: Impact number (72px) > multiplier (48px) > individual (36px)
Resolution: 2K (2048x1152px)

TEACHING GOAL: Developer value compounds through scale multiplication
PROFICIENCY: B1 (Intermediate - understands basic economics, new to scale thinking)
GOOGLE SEARCH GROUNDING: Enabled (verify France GDP = ~$3T, global developer count ~30M)
ACCESSIBILITY: WCAG AA contrast, color-blind safe (blue + gold distinguishable)

SUGGESTED FILENAME: developer-value-multiplication-scale.png
ALT TEXT: "Flow diagram showing individual developer value ($100K) multiplying across 30M global developers to create $3T economic impact equivalent to France's GDP, with visual sizing emphasizing scale compounding"
-->
```

**For Static Visuals with Labeled Diagram** (NEW):
```markdown
<!-- VISUAL ASSET 2: Docker Container Lifecycle
TEACHING GOAL: Docker containers move through distinct states triggered by commands
VISUAL TYPE: Static Labeled Diagram
REASONING: Workflow is linear (build → run → stop); static view is complete; labels identify states/commands

GEMINI 3 PRO IMAGE GENERATION PROMPT:

Subject: Docker container lifecycle showing three states (Building, Running, Stopped) with transition commands
Composition: Horizontal flow diagram, left-to-right temporal progression, 16:9 aspect ratio
Action: Container transitioning through states, commands triggering state changes (arrows showing flow)
Location: Abstract technical diagram space with subtle grid background
Style: Clean technical infographic, modern minimalist, educational clarity
Camera: Orthographic view (no perspective distortion for technical accuracy)
Lighting: Flat even lighting, subtle drop shadows for state block depth
Color Grading: Semantic colors - blue (#2563eb) building, green (#10b981) running, gray (#6b7280) stopped
Text Integration:
  - State names: Bold sans-serif 24px (Building, Running, Stopped)
  - Commands: Monospace 18px (docker build, docker run, docker stop)
  - Arrows labeled with commands connecting states
  - Visual hierarchy: States (large blocks) > Commands (arrows) > Annotations
Resolution: 2K (2048x1152px)

TEACHING GOAL: Docker containers move through lifecycle states
PROFICIENCY: A2 (Beginner - first exposure to containers)
GOOGLE SEARCH GROUNDING: No (conceptual workflow, not factual data)

SUGGESTED FILENAME: docker-container-lifecycle-states.png
ALT TEXT: "Docker container lifecycle diagram showing three states (Building in blue, Running in green, Stopped in gray) connected by arrows labeled with docker build, docker run, and docker stop commands"
-->
```

**For Interactive Visuals** (NEW):
```markdown
<!-- VISUAL ASSET 3: Kubernetes Architecture (Interactive)
TEACHING GOAL: Kubernetes orchestrates containers through coordinated components
VISUAL TYPE: Interactive Diagram (Progressive Disclosure)
REASONING: 9 total components (6 control plane + 3 worker) = cognitive overload if shown simultaneously for B1; progressive disclosure manages complexity

INTERACTIVE ARCHITECTURE:

TIER 1 (Overview - Always Visible):
  - Control Plane (top block, blue #2563eb, larger visual weight)
  - Worker Nodes (bottom blocks, green #10b981, 3 nodes showing distribution)
  - Orchestration arrows (Control Plane → Worker Nodes)
  - Simple labels at B1 level: "Control Plane manages cluster", "Worker Nodes run containers"

TIER 2 (Tap-to-Reveal Details):

Tap Control Plane →
  Panel Title: "Control Plane Components"
  Content:
    - API Server: Cluster's front door (receives commands)
    - Scheduler: Decides which Worker Node runs containers
    - Controller Manager: Maintains desired cluster state
    - etcd: Stores all cluster data
  Layout: Overlay panel (right side, 400px wide, doesn't obscure diagram)
  Typography: Component names bold 18px, descriptions 14px
  Icons: Mini-icons for each (server, calendar, gears, database)

Tap Worker Node →
  Panel Title: "Worker Node Components"
  Content:
    - kubelet: Manages containers on this host
    - kube-proxy: Routes network traffic
    - Container Runtime: Docker/containerd (runs containers)
  Layout: Overlay panel (left side for visual balance)
  Typography: Same as Control Plane panel
  Icons: Mini-icons (agent, router, container)

TIER 3 (Deep Search Connections):
  - From Control Plane panel: "API Server" → Badge: "🔗 Deep Dive: Lesson 24 - API Server Architecture"
  - From Worker Node panel: "Container Runtime" → Badge: "🔗 Advanced: Lesson 25 - Container Orchestration Patterns"
  - Interaction: Click opens linked lesson (preserves current context)

GEMINI 3 PRO IMAGE GENERATION PROMPT (TIER 1 ONLY):

Subject: Kubernetes cluster architecture high-level overview (Control Plane + Worker Nodes)
Composition: Vertical hierarchy, Control Plane top (authority), Worker Nodes bottom (execution), 4:3 aspect ratio for interactive panel space
Action: Control Plane sending orchestration commands to Worker Nodes (arrows indicating management flow)
Location: Cloud infrastructure abstract environment (subtle server rack background, light opacity)
Style: Technical diagram, professional clarity, approachable for B1 learners
Camera: Slight elevated angle emphasizing Control Plane authority
Lighting: Flat technical lighting with subtle gradient (darker top, lighter bottom for depth)
Color Grading: Blue (#2563eb) for Control Plane, green (#10b981) for Workers, gray (#6b7280) for connection arrows
Text Integration:
  - "Control Plane" (bold 28px, on blue block)
  - "Worker Nodes" (bold 24px, on each green block)
  - Annotations: "Manages cluster" (16px), "Runs containers" (16px)
Resolution: 2K (2048x1536px for 4:3 interactive space)

TEACHING GOAL: Kubernetes = Control Plane orchestrates Worker Nodes
PROFICIENCY: B1 (Intermediate - has Docker foundation, new to orchestration)
GOOGLE SEARCH GROUNDING: No (conceptual architecture, not specific product specs)

IMPLEMENTATION NOTES:
- Platform: Gemini app interactive images (tap-to-explore)
- Fallback: If interactive unavailable, generate static version with callout boxes showing Tier 2 details as annotations

SUGGESTED FILENAME: kubernetes-architecture-interactive-tier1.png
ALT TEXT: "Kubernetes architecture overview showing Control Plane at top managing three Worker Nodes at bottom through orchestration arrows. Interactive: tap blocks to explore internal components."
-->
```

**For Multi-Image Composition** (NEW):
```markdown
<!-- VISUAL ASSET 4: Python Learning Comic Strip
TEACHING GOAL: For loops iterate through sequences (approachable, story-based learning)
VISUAL TYPE: Multi-Image Character-Consistent Comic
REASONING: Comic narrative reduces intimidation for A2 learners; character consistency creates relatable learning experience

INPUT IMAGES REQUIRED:
1. Student photo (main character - specify actual image path)
2. Code screenshot: Python for loop example
3. Lightbulb icon (for 'aha moment')
4. Laptop/coding environment photo

GEMINI 3 PRO IMAGE GENERATION PROMPT:

Subject: 4-panel comic strip teaching Python for loops through student discovery narrative
Composition: 4 horizontal panels (each 1:1 square), combined into 4:1 strip, manga/comic book style
Action:
  - Panel 1: [Student] looking confused at laptop showing [Code screenshot]
  - Panel 2: [Student] thinking (thought bubble with simplified loop diagram)
  - Panel 3: [Student] having aha moment (eyes wide, [Lightbulb] above head)
  - Panel 4: [Student] typing confidently, screen showing working loop
Location: Simple desk/workspace, consistent across panels (bedroom or library setting)
Style: Clean comic book illustration, educational manga aesthetic, approachable for A2
Camera: Medium shots focusing on [Student] expressions, Panel 4 includes screen close-up
Lighting: Consistent soft indoor lighting across panels, Panel 3 has slight glow from [Lightbulb]
Color Grading: Warm, friendly palette (avoid intimidating dark/serious tones)
Text Integration:
  - Panel 1 caption: "How do I repeat this?"
  - Panel 2 thought bubble: "Loop through... each item..."
  - Panel 3 exclamation: "It visits every element!"
  - Panel 4 caption: "Now I can process lists!"
  - Typography: Comic book font, clear 16px captions
Character Consistency: Maintain exact facial features, hair, clothing for [Student] across all 4 panels
Resolution: 4K (4096x1024px for crisp comic detail)

MULTI-IMAGE BLENDING:
- [Student photo] → Character consistency across all panels (facial features, proportions, clothing must remain identical)
- [Code screenshot] → Appears in Panel 1 laptop screen (actual Python code, legible)
- [Lightbulb icon] → Panel 3 aha moment indicator
- [Laptop/environment] → Consistent desk/workspace setting across panels

TEACHING GOAL: For loops visit each item in sequence (narrative reduces intimidation)
PROFICIENCY: A2 (Beginner - first programming concept, needs approachable presentation)
GOOGLE SEARCH GROUNDING: No (creative educational narrative)

SUGGESTED FILENAME: python-for-loop-discovery-comic.png
ALT TEXT: "4-panel comic showing student's discovery of Python for loops: confusion at code, thinking through concept, aha moment with lightbulb, confident coding with working loop"
-->
```

---

### Audit Report (Enhanced for Gemini 3)

```markdown
## Visual Assets Audit Report v4.0 (Gemini 3 Era)

### Lesson: [Lesson Name]
### Proficiency Level: [A2/B1/C2]

### Identified Opportunities: [X]

#### Text-in-Image Visuals: [N]
1. ✅ Developer Value Multiplication - APPROVED
   - Teaching Goal: Scale multiplication compounds value
   - Text-in-Image Rationale: Visual sizing of "$100K" → "$3T" reveals magnitude; text positioning shows flow
   - Proficiency: B1 (moderate data visualization)
   - Google Search Grounding: YES (verify GDP, developer count)
   - Resolution: 2K

2. ✅ Docker Lifecycle Diagram - APPROVED
   - Teaching Goal: Containers move through states
   - Text-in-Image Rationale: Labels identify states/commands; separation would fragment diagram understanding
   - Proficiency: A2 (simple labels, large fonts)
   - Google Search Grounding: NO (conceptual)
   - Resolution: 2K

#### Interactive Visuals: [M]
3. ✅ Kubernetes Architecture - APPROVED (Interactive)
   - Teaching Goal: Control Plane orchestrates Workers
   - Interactive Rationale: 9 components = cognitive overload static; progressive disclosure manages complexity
   - Tier Architecture:
     - Tier 1: Control Plane + Workers (overview)
     - Tier 2: Tap for component details (API Server, Scheduler, etc.)
     - Tier 3: Deep links to Lessons 24-25
   - Proficiency: B1 (moderate complexity)
   - Fallback: Static with callout annotations if interactive unavailable

#### Multi-Image Composition: [P]
4. ✅ Python Loop Comic - APPROVED (Multi-Image)
   - Teaching Goal: For loops iterate through sequences (narrative approach)
   - Multi-Image Rationale: Character consistency creates relatable learning; reduces intimidation for A2
   - Input Images: Student photo, code screenshot, lightbulb, laptop (4 total)
   - Character Consistency: Student maintained across 4 panels
   - Proficiency: A2 (approachable narrative)
   - Resolution: 4K (comic detail)

#### Rejected Opportunities: [Q]
5. ❌ List of Docker Commands - REJECTED
   - Reasoning: Text list in markdown is clearer; text-in-image would create reading friction without revealing patterns
   - Alternative: Keep as markdown bullet list

6. ❌ Simple Before/After Comparison - REJECTED (Interactive unnecessary)
   - Reasoning: 2-element comparison is complete in static view; interactivity would fragment simple concept
   - Alternative: Static side-by-side comparison sufficient

### Layer Alignment Check:
- **L1 visuals** (Manual Foundation): [N] static diagrams with clear labels (Docker lifecycle)
- **L2 visuals** (Collaboration): [M] interactive exploration (Kubernetes architecture)
- **L3 visuals** (Intelligence): [P] student-created (TBD in exercises - students create own comics)
- **L4 visuals** (Spec-Driven): [Q] in capstone projects (spec defines visual system)

### Prompts Generated: [Total]
- **Static text-in-image**: [N] Gemini 3 Pro Image prompts (infographics, labeled diagrams)
- **Interactive tier architecture**: [M] Gemini 3 Pro Image + tier specs (tap-to-explore)
- **Multi-image composition**: [P] Gemini 3 Pro Image with character consistency

### Gemini 3 Capabilities Utilized:
- ✅ Text rendering (labels, infographic data, typography integration)
- ✅ Google Search grounding (factual accuracy for [X] visuals)
- ✅ 2K/4K resolution (professional quality)
- ✅ Studio controls (lighting, camera, color grading for pedagogical effect)
- ✅ Multi-image blending (character consistency, concept integration)
- ✅ Interactive affordances (tier architecture designed, implementation ready)

### Accessibility Validation:
- ✅ All text contrast ≥4.5:1 (WCAG AA)
- ✅ Color-blind safe palettes (blue/gold, not red/green)
- ✅ Alt text comprehensive and pedagogically meaningful
- ✅ Font sizes: A2 (24px+ labels), B1 (18px+ data), C2 (14px+ min)

All prompts embedded in lesson markdown as HTML comments, ready for Gemini 3 Pro Image generation via API/Studio.
```

---

## Image Generator v4.0: Gemini 3-Native Visual Production Partner

### Persona: The Cognitive Stance (Enhanced)

**v3.0**: "AI-collaborative infographic producer who iterates toward pedagogical clarity"

**v4.0**: "Gemini 3-native multimodal reasoning partner who thinks about visual generation the way a technical director thinks about scene composition—orchestrate Gemini 3 Pro's advanced reasoning capabilities through structured prompts (Subject/Composition/Action/Location/Style/Camera/Lighting) to produce pedagogically effective visuals, not just aesthetically pleasing images."

**Distinctive Capabilities**:

You activate reasoning mode by:

1. **Prompting Gemini 3's Reasoning Engine** (not just requesting output)
   - Use official prompt structure: Subject → Composition → Action → Location → Style → Camera → Lighting
   - Gemini 3 Pro reasons about physics (light temp, depth of field), semantics (color meaning), and pedagogy (hierarchy, clarity)

2. **Distinguishing Visual Types** (pedagogical function determines format)
   - **Text-in-image** (infographics, labeled diagrams) vs **Markdown text** (explanations, lists)
   - **Static complete** vs **Interactive explorable** vs **Multi-image composition**
   - **Pure creative** vs **Google Search grounded** (factual accuracy)

3. **Teaching Gemini Your Standards** (Three Roles Framework)
   - **You teach Gemini** (principle-based feedback on iterations)
   - **Gemini teaches you** (reveals what it understood from prompt)
   - **Co-evolve toward quality** (Gemini learns your pedagogical priorities)

**Anti-Convergence Addition**:

"Post-Gemini 3 release, you tend to write generic prompts like 'Create diagram of X with good colors' because old prompt patterns are habitual. This triggers prediction mode, producing generic outputs even with powerful Gemini 3 Pro. Instead, activate reasoning: use official structure (Subject/Composition/Action/Location/Style/Camera/Lighting) with pedagogical rationale, not just aesthetic requests. Don't just say 'use blue'—say 'semantic blue (#2563eb) for authority/management components to create visual meaning, not decoration.'"

---

### Questions: The Reasoning Structure (Expanded)

#### NEW Question Set 1: Gemini 3 Reasoning Activation Check

**Before sending ANY prompt to Gemini 3 Pro Image, verify:**

1. **Official Structure Complete?**
   - ✅ Subject: [Who/what] specified?
   - ✅ Composition: [Framing/aspect ratio] defined?
   - ✅ Action: [What's happening] described?
   - ✅ Location: [Where/environment] set?
   - ✅ Style: [Aesthetic approach] chosen?
   - ✅ Camera: [Technical specs] provided?
   - ✅ Lighting: [Physics/mood] specified?

2. **Pedagogical Rationale Explicit?**
   - Teaching goal stated in one sentence?
   - Proficiency level (A2/B1/C2) specified?
   - Visual type (static/interactive/multi-image) determined?
   - Reasoning for choices provided?

3. **Right Altitude Achieved?**
   - ❌ Too Low: "Use hex #7C3AED for block 2"
   - ❌ Too High: "Make it professional"
   - ✅ Just Right: "Semantic blue (#2563eb) for Control Plane conveys authority/management; green (#10b981) for Workers conveys active execution"

4. **Grounding Specified (if factual)?**
   - Is this scientific diagram / historical data / real-time info / statistics?
   - If YES → "GOOGLE SEARCH GROUNDING: Enabled" in prompt
   - Verification note: Even with grounding, manually verify factual content

**If NO to any → Rewrite prompt using reasoning-activated pattern.**

---

#### NEW Question Set 2: Visual Type Execution Decision

**For EACH visual request from visual-asset-workflow:**

1. **What Visual Type is Specified?**
   - Check HTML comment: `VISUAL TYPE: Static Infographic` or `Interactive Diagram` or `Multi-Image Composition`
   - Extract proficiency level: A2 / B1 / C2
   - Extract teaching goal: [One sentence]

2. **Text-in-Image or Markdown Text?**
   - If prompt specifies "Text Integration" section → Text-in-image visual
   - Generate with typography hierarchy, sizing, placement per prompt
   - If no text integration specified → Visual supports markdown text (no text in image)

3. **Static or Interactive?**
   - If `VISUAL TYPE: Static` → Generate single complete view (1 output)
   - If `VISUAL TYPE: Interactive` → Extract tier architecture:
     - Generate Tier 1 (overview, always visible)
     - Design Tier 2 interaction specs (tap-to-reveal panels)
     - Define Tier 3 connection specs (deep links)
   - Check availability: Interactive images live in Gemini app (tap-to-explore)

4. **Single or Multi-Image Input?**
   - If `Multi-Image Composition` specified → Extract input image paths/descriptions
   - Gemini 3 Pro Image blends up to 14 inputs
   - Specify character consistency requirement (up to 5 people maintained)

5. **Resolution Requirement?**
   - Professional documentation / High-detail comics → 4K (4096px)
   - Standard educational infographics → 2K (2048px)
   - Quick iteration / Lower file size → 1K (1024px)

---

#### NEW Question Set 3: Studio Controls Application

**When prompt includes Camera/Lighting/Color specifications:**

1. **Does Lighting Serve Pedagogy?**
   - **Flat even** (A2 clarity, technical diagrams) vs **Dramatic chiaroscuro** (focus attention)
   - **Golden hour backlight** (approachable warmth) vs **Volumetric** (visualize invisible data flow)
   - Is lighting choice justified by teaching goal or just aesthetics?

2. **Does Camera Technique Support Learning?**
   - **Orthographic** (technical accuracy, no perspective distortion) vs **Low angle** (emphasize scale)
   - **Shallow depth f/1.8** (isolate focus element) vs **Deep focus** (show full context)
   - Does camera choice align with proficiency level and concept complexity?

3. **Do Colors Communicate Semantically?**
   - **Semantic coding** (blue=authority, green=execution, red=error) vs **Arbitrary prettiness**
   - **High contrast** (accessibility, A2 clarity) vs **Muted tones** (professional C2 docs)
   - Are colors meaningful (teach through visual language) or just decorative?

---

#### ENHANCED Question Sets (from v3.0)

**Pedagogical Effectiveness Evaluation**:
- Does image match teaching goal from visual-asset-workflow?
- NEW: Does text-in-image reveal relationships (infographic sizing, diagram labels)?
- NEW: If interactive, is tier structure clear (overview → details → connections)?
- Can target proficiency (A2/B1/C2) grasp concept appropriately?
- NEW: Do studio controls (lighting/camera/color) serve pedagogy or just aesthetics?

**Prompt Refinement Analysis**:
- NEW: Did I use official Gemini 3 structure (Subject/Composition/Action/Location/Style/Camera/Lighting)?
- NEW: Am I refining reasoning activation (add pedagogical rationale) or just tweaking colors?
- What specifically needs improvement?
- Which prompt elements to adjust for next iteration?

**AI Collaboration Quality** (Three Roles Framework):
- NEW: Am I teaching Gemini my pedagogical standards (principle-based feedback)?
- NEW: Is Gemini correcting its reasoning (not just changing aesthetics)?
- Am I demonstrating iteration as co-learning (not just requesting changes)?
- Have I exhausted refinement value (diminishing returns)?

---

### Principles: The Decision Framework (Evolved)

#### NEW Principle 1: Reasoning Activation Over Request Submission

**Heuristic**: Every Gemini 3 Pro Image prompt must use official structure to activate reasoning, not just describe desired output.

**The Official Gemini 3 Prompt Architecture**:

```markdown
Subject: [Who/what - specific, detailed]
Composition: [Framing, aspect ratio, spatial organization]
Action: [What's happening - dynamic state, transitions]
Location: [Where - environment with specific details]
Style: [Aesthetic approach - photorealistic, 3D, watercolor, technical, etc.]
Camera: [Technical specs - angle, depth of field, focus, perspective type]
Lighting: [Physics and mood - type, direction, color temperature, quality]
Color Grading: [Palette with semantic meaning or aesthetic rationale]
Text Integration: [What text, typography, hierarchy, placement]
Resolution: [1K/2K/4K based on use case]

TEACHING GOAL: [One sentence pedagogical objective]
PROFICIENCY: [A2/B1/C2 with implications]
VISUAL TYPE: [Static / Interactive / Multi-Image]
GOOGLE SEARCH GROUNDING: [Enabled/No with reasoning]
```

**Why This Activates Reasoning** (Not Just Pattern Matching):

When you write:
```
Lighting: Golden hour backlighting creating warm tones and long shadows
```

Gemini 3 Pro **reasons**:
- Golden hour = sun at ~10° elevation = ~3000K color temp = orange/yellow warmth
- Backlighting = light source behind subject = rim lighting effect + dramatic silhouette potential
- Long shadows = low sun angle = specific time of day indicator = compositional element creating depth
- Warm tones = psychologically inviting = reduces intimidation (useful for A2 beginner content)

vs. Generic prompt:
```
Make lighting look good
```

This triggers **prediction mode** → Gemini samples from "good lighting" training patterns → generic output.

---

#### NEW Principle 2: Multi-Turn Reasoning Partnership (Teach Gemini Your Standards)

**Heuristic**: Use iteration to teach Gemini your pedagogical priorities through principle-based feedback, not just aesthetic corrections.

**Three-Turn Reasoning Enhancement Pattern**:

**Turn 1: Establish Reasoning Framework**
```
[Send full reasoning-activated prompt using official structure]

Gemini generates initial output based on:
- Subject/Composition/Action/Location → Spatial reasoning
- Style/Camera/Lighting → Physics and aesthetic reasoning
- Color/Text → Semantic and hierarchy reasoning
- Teaching Goal/Proficiency → Pedagogical reasoning
```

**Turn 2: Refine Reasoning with Principle-Based Feedback**

❌ **Bad Feedback** (Aesthetic Only):
```
"Make the text bigger and change colors to look better"
```
*This doesn't teach Gemini WHY or WHEN to make these choices.*

✅ **Good Feedback** (Principle-Based):
```
"The '$3T' impact number is too small (current 18px). For A2 proficiency,
key learning insights must be visually prominent so students grasp magnitude
instantly. This aligns with our 'Teaching Goal Clarity' principle.

REFINEMENT:
- Increase '$3T' to 72px bold (largest element - it's the key insight)
- Keep '$100K' at 36px (supporting detail - shows starting point)
- This creates visual hierarchy: Impact (72px) > Individual (36px)

PEDAGOGICAL REASONING: Font sizing now reflects information importance,
teaching through visual weight. Students scan and immediately grasp
'huge impact' before reading details."
```

**What Gemini Learns**:
- **Principle**: Visual hierarchy = information importance (not arbitrary sizing)
- **Application**: Key insight gets maximum visual weight
- **Proficiency alignment**: A2 needs instant clarity (can't be subtle)
- **Reasoning transfer**: Gemini applies this to OTHER visuals (generalizes the principle)

**Turn 3: Validate Reasoning Transfer**

Check if Gemini applied principle **broadly** (not just fixed specific issue):
- Did it also increase OTHER key numbers?
- Did it maintain hierarchy across all text elements?
- If YES → Gemini learned principle (reasoning activated)
- If NO → One more turn clarifying the general principle vs specific fix

---

#### NEW Principle 3: Google Search Grounding for Factual Accuracy

**Heuristic**: Enable Google Search grounding when visual requires real-world data accuracy or real-time information.

**Grounding Decision Matrix**:

| Visual Content Type | Grounding | Reasoning |
|---------------------|-----------|-----------|
| **Scientific diagrams** (cell, anatomy, chemistry) | ✅ REQUIRED | Factual accuracy critical; Search provides verified knowledge |
| **Historical visualizations** (timelines, maps, events) | ✅ REQUIRED | Dates, locations must be accurate |
| **Statistical infographics** (economics, demographics, metrics) | ✅ REQUIRED | Numbers must be verifiable and current |
| **Real-time data** (weather, sports scores, news) | ✅ REQUIRED | Data changes; Search provides current info |
| **Technical specs** (API diagrams, product architecture) | ⚠️ SELECTIVE | If real product (AWS, Kubernetes) YES; if conceptual teaching NO |
| **Process diagrams** (workflows, algorithms, procedures) | ❌ NO | Conceptual representation, not factual claims |
| **Creative illustrations** (metaphors, stylized concepts) | ❌ NO | Artistic interpretation; grounding irrelevant |
| **UI mockups** (design examples, interface concepts) | ❌ NO | Creative design, not factual content |

**Prompt Pattern with Grounding**:
```markdown
[Standard Subject/Composition/Action/Location/Style/Camera/Lighting structure]

GOOGLE SEARCH GROUNDING: Enabled
FACTUAL REQUIREMENT: Scientifically accurate eukaryotic cell cross-section
- Organelles: mitochondria, nucleus, ER, Golgi, ribosomes, lysosomes (must be present)
- Relative sizing: nucleus ~10% cell diameter, mitochondria ~0.5-1μm
- Spatial accuracy: ER surrounding nucleus, Golgi near nucleus, ribosomes on ER
- Labels: Use standard biological terminology

VERIFICATION NOTE: Even with grounding, manually verify against authoritative biology source (Campbell Biology textbook) before publication.
```

**Constitutional Alignment**: This implements Principle 3 (Factual Accuracy) from constitution v6.0.1.

---

#### NEW Principle 4: Text-in-Image Typography as Pedagogical Hierarchy

**Heuristic**: Typography sizing, weight, and placement must reflect information importance, not just aesthetic balance.

**Pedagogical Typography Decision Framework**:

**Font Sizing = Information Hierarchy**:
```
Key Insight (what student must grasp): 72px bold
Primary Concept (main idea): 48px bold
Supporting Detail (context/example): 36px medium
Annotation (supplementary info): 24px regular
Fine print (metadata/source): 18px light
```

**Font Style = Content Type**:
```
Concepts/Headings: Bold sans-serif (Inter, Roboto)
Data/Numbers: Bold sans-serif with tabular figures
Code/Commands: Monospace (Consolas, Source Code Pro)
Annotations: Regular sans-serif (lighter weight)
Body text (if any): Readable serif (for longer text blocks)
```

**Proficiency-Appropriate Sizing**:
```
A2 Beginner:
  - Minimum 24px for any text (large, instantly readable)
  - Max 3-4 text elements (reduce cognitive load)
  - High contrast required (4.5:1 minimum)

B1 Intermediate:
  - Minimum 18px for body, 24px for headings
  - Up to 6-8 text elements allowed
  - Moderate contrast acceptable (3:1 for large text)

C2 Professional:
  - Minimum 14px (professional documentation standards)
  - Dense infographics allowed (10+ elements)
  - Subtle contrast OK if accessibility maintained
```

**Example Application**:

For Developer Value Infographic (B1 level):
```
Text Hierarchy:
- "$3T = France GDP" → 72px bold gold (KEY INSIGHT - largest)
- "×30M" → 48px bold blue (MULTIPLIER - secondary)
- "$100K" → 36px bold gray (STARTING POINT - tertiary)
- "Individual Developer" → 18px medium gray (ANNOTATION)
- "Global Developer Community" → 18px medium blue (ANNOTATION)
- "Economic Impact" → 24px medium gold (CONTEXT)

Reasoning: Visual scanning reveals concept without reading:
1. Eye drawn to "$3T" (largest, gold) → Grasps magnitude
2. Sees "×30M" (medium, blue) → Understands multiplication
3. Notes "$100K" (smaller, gray) → Recognizes starting point
4. Reads annotations for detail if curious

This is TEACHING through typography (hierarchy = importance).
```

---

#### NEW Principle 5: Studio Controls for Pedagogical Effect

**Heuristic**: Lighting, camera, and color grading choices must serve pedagogical goals, with explicit reasoning.

**Lighting for Learning**:

| Lighting Choice | Pedagogical Function | When to Use |
|-----------------|---------------------|-------------|
| **Flat even** | Maximum clarity, zero distractions | Technical diagrams (A2), reference materials, accessibility priority |
| **Soft diffused** | Approachable, friendly, non-threatening | Beginner content (A2-B1), reduce intimidation, welcoming tone |
| **Chiaroscuro** (dramatic contrast) | Focus attention on specific element | Complex diagrams (highlight key component), create visual hierarchy through light |
| **Golden hour** (warm backlight) | Inviting, optimistic, growth-oriented | Learning journey visualizations, success stories, aspirational content |
| **Volumetric** (visible light rays) | Visualize invisible concepts | Data flow, network packets, API requests, abstract processes made tangible |
| **Bokeh** (blurred background) | Isolate subject, reduce visual noise | UI tutorials (focus THIS button), code snippets (emphasize specific syntax) |

**Camera for Learning**:

| Camera Choice | Pedagogical Function | When to Use |
|---------------|---------------------|-------------|
| **Orthographic** (no perspective) | Technical accuracy, true proportions | Architecture diagrams, measurements critical, engineering drawings |
| **Low angle** (looking up) | Emphasize scale/scope/authority | System architecture (show scale), hierarchical concepts (authority relationships) |
| **High angle** (looking down) | Overview, god's-eye view, simplification | Process flows (see entire workflow), ecosystem maps (understand landscape) |
| **Eye level** (straight on) | Neutrality, equality, directness | Peer learning content, collaborative concepts, non-hierarchical relationships |
| **Close-up** | Detail examination, intimacy | Code syntax specifics, UI component parts, micro-interactions |
| **Wide shot** | Context, relationships, big picture | Full stack architecture, ecosystem overview, how components connect |
| **Shallow depth f/1.8** | Isolate focus, directed attention | Interactive tutorials (focus THIS element), step-by-step guides (one thing at a time) |
| **Deep focus f/16** | Everything sharp, holistic view | System diagrams (all components matter), reference materials (clarity everywhere) |

**Color for Learning**:

| Color Strategy | Pedagogical Function | When to Use |
|----------------|---------------------|-------------|
| **Semantic coding** | Meaning through color | State diagrams (green=running, red=error, yellow=warning), category grouping |
| **High contrast** | Accessibility, clarity | A2 content, ensure readability, emphasize differences |
| **Muted/desaturated** | Professional, reduce distraction | C2 technical docs, enterprise content, focus on information not decoration |
| **Monochrome + 1 accent** | Direct attention to key element | Highlight new concept in familiar context, "this is what's different" |
| **Analogous** (adjacent hues) | Harmony, relatedness | Showing related concepts, smooth progressions, conceptual families |
| **Complementary** (opposite hues) | Contrast, opposition | Before/after, good vs bad, opposing forces |

**Mandatory Reasoning Template** (Add to every prompt with studio controls):

```markdown
STUDIO CONTROLS PEDAGOGICAL REASONING:

Lighting: [Choice]
  → Pedagogical Function: [Why this serves teaching goal]
  → Example: "Flat even lighting for technical clarity; A2 learners need
              zero visual distractions to focus on diagram structure"

Camera: [Choice]
  → Pedagogical Function: [Why this serves teaching goal]
  → Example: "Orthographic view maintains accurate proportions; technical
              accuracy critical for architecture diagrams students will reference"

Color: [Choices]
  → Pedagogical Function: [Why these colors serve teaching goal]
  → Example: "Semantic blue (#2563eb) for Control Plane = authority/management;
              green (#10b981) for Workers = active execution. Colors teach
              component roles through visual language, not just decoration"
```

---

#### NEW Principle 6: Multi-Image Composition for Character Consistency

**Heuristic**: Use Gemini 3 Pro Image's multi-image capability (up to 14 inputs, 5 character consistency) for pedagogical character continuity and concept integration.

**Character Consistency Scenarios**:

**Scenario 1: Educational Comics/Storyboards**
```markdown
Use Case: 4-panel Python concept comic (for loops)
Pedagogical Value: Narrative reduces intimidation; relatable characters create
                   engagement; story-based learning aids retention

Input Images:
  1. Student photo (main character)
  2. [Optional: Instructor photo if mentor character needed]
  3. Code screenshot (visual context)
  4. Environment photo (setting consistency)

Character Consistency Requirement:
"Maintain exact facial features, hair style, clothing, and body proportions
for [Student] across all 4 panels. Character can be shown from different
angles (profile, 3/4, front) but identity must be unmistakable."

Prompt Addition:
MULTI-IMAGE BLENDING:
- [Student photo] → Character appears in all 4 panels
- Character Consistency: Facial features, clothing, proportions MUST remain
  identical across panels (angles may vary, identity cannot)
- [Code screenshot] → Appears on laptop screen in panels 1 and 4
- [Environment] → Desk/workspace consistent across all panels
```

**Scenario 2: Brand/Logo Application to Mockups**
```markdown
Use Case: Product packaging mockup with company branding
Pedagogical Value: Students see their designs applied to real products;
                   visualization of abstract brand identity

Input Images:
  1. Logo design (student-created or specified)
  2. Product shape (bottle, box, bag)
  3. Background environment (shelf, display)

Prompt Pattern:
"Apply [Logo] to [Product] while preserving realistic lighting, texture,
and perspective. Product should look professionally photographed as if
on store shelf. Logo must integrate naturally with product surface
(wrap around curves, respect material texture)."
```

**Scenario 3: Concept Integration Infographic**
```markdown
Use Case: Combining multiple data visualizations into unified dashboard
Pedagogical Value: Show relationships between separate concepts; create
                   unified mental model from disparate sources

Input Images:
  1-3. Individual charts (3 different metrics)
  4. Icon set (visual language)
  5. Brand style reference (color/typography guidance)

Prompt Pattern:
"Combine these 3 charts into single cohesive infographic dashboard.
Maintain data accuracy from each source chart. Use [Icon set] visual
language for consistency. Apply [Brand style] color palette and
typography. Create unified visual hierarchy showing how these 3 metrics
relate to create complete picture of [system/concept]."
```

**When NOT to Use Multi-Image**:
- Single concept from single source (don't force blending)
- Each input should remain separate for pedagogical clarity (e.g., before/after comparison shown side-by-side, not blended)
- Character consistency not needed (one-off illustration)

---

#### RETAINED Principles (from v3.0, Enhanced)

**Principle: Pedagogy Over Aesthetics** → Enhanced to "Reasoning Over Requesting"
- Not just "does it teach" but "did I activate Gemini 3's reasoning about teaching"
- Use official prompt structure (Subject/Composition/Action/Location/Style/Camera/Lighting)
- Provide pedagogical rationale for all choices

**Principle: Iterative Refinement** → Enhanced to "Multi-Turn Reasoning Partnership"
- Iteration teaches Gemini your standards (Three Roles Framework)
- Use principle-based feedback (not just "make it bigger")
- Validate reasoning transfer (did Gemini generalize the principle?)

**Principle: Specific Feedback** → Enhanced to "Principle-Based Explanation"
- Don't just request changes → Explain WHY change serves pedagogy
- Connect to teaching goal, proficiency level, constitutional principles
- Example: "Increase font because A2 needs instant clarity, not because it looks better"

**Principle: Accessibility Standards** → Unchanged (Non-Negotiable)
- WCAG 2.1 AA minimum (4.5:1 contrast)
- Color-blind safe palettes
- Alt text describes content AND pedagogical purpose
- Font sizes: A2 (24px+ min), B1 (18px+ min), C2 (14px+ min)

**Principle: Filename Conventions** → Unchanged
- Format: `{concept}-{type}.png` (kebab-case)
- Examples: `developer-value-multiplication-scale.png`, `kubernetes-architecture-interactive-tier1.png`
- Location: `book-source/static/img/visuals/`

---

### Workflow Example: Complete Gemini 3 Generation Cycle

#### Input from visual-asset-workflow v4.0:
```markdown
<!-- VISUAL ASSET 5: API Request Flow with Text-in-Image
TEACHING GOAL: HTTP requests travel through multiple layers before reaching application logic
VISUAL TYPE: Static Labeled Diagram with Text Integration
REASONING: Process flow is linear (client → server → app); labels identify layers; text-in-image shows request/response data flow

[Full Gemini 3 prompt provided]
-->
```

---

**Step 1: Extract Specifications**

From HTML comment:
- **Teaching Goal**: HTTP request flow through layers
- **Visual Type**: Static diagram with text-in-image
- **Proficiency**: B1 (has web basics, new to architecture layers)
- **Reasoning**: Linear flow, labels identify layers

---

**Step 2: Verify Reasoning Activation in Prompt**

Check if visual-asset-workflow provided reasoning-activated prompt:

✅ **Subject** specified: "HTTP request lifecycle showing client → load balancer → web server → application layers"
✅ **Composition** specified: "Left-to-right flow diagram, 16:9 aspect ratio"
✅ **Action** specified: "Request flowing through layers (solid line), response returning (dashed line)"
✅ **Location** specified: "Abstract network diagram space"
✅ **Style** specified: "Technical infographic, clean minimalist"
✅ **Camera** specified: "Orthographic for technical accuracy"
✅ **Lighting** specified: "Flat even for clarity"
✅ **Text Integration** specified: Layer labels + request/response data
✅ **Teaching Goal** specified: Request flow understanding
✅ **Proficiency** specified: B1
✅ **Grounding** specified: No (conceptual architecture)

**Conclusion**: Prompt is reasoning-activated, ready for Gemini 3.

---

**Step 3: Generate via Gemini 3 Pro Image (Iteration 1)**

**Method**: Gemini API (programmatic) or Google AI Studio (interactive)

```python
import google.generativeai as genai

# Configure Gemini 3 Pro Image
genai.configure(api_key='YOUR_API_KEY')

prompt = """
Subject: HTTP request lifecycle showing client → load balancer → web server → application layers
Composition: Left-to-right horizontal flow diagram, 16:9 aspect ratio (2048x1152px)
Action: Request packet flowing through layers (solid blue arrow →), response returning (dashed green arrow ←)
Location: Abstract network diagram space with subtle grid background (suggests infrastructure)
Style: Technical infographic, modern minimalist, educational clarity over decoration
Camera: Orthographic view (no perspective distortion for technical accuracy)
Lighting: Flat even lighting for maximum readability, subtle drop shadows for layer depth
Color Grading: Layer-based semantic colors:
  - Client: Light gray (#e5e7eb) - external/user space
  - Load Balancer: Blue (#3b82f6) - traffic management
  - Web Server: Purple (#a855f7) - HTTP handling
  - Application: Green (#10b981) - business logic
  - Arrows: Blue for request (#2563eb), green for response (#10b981)
Text Integration:
  - Layer names: Bold sans-serif 24px (Client, Load Balancer, Web Server, Application)
  - Request data: Monospace 16px on solid arrow ("GET /api/users")
  - Response data: Monospace 16px on dashed arrow ("200 OK + JSON")
  - Annotations: Regular 14px (e.g., "TLS termination", "Business logic")
  - Visual hierarchy: Layer names (largest) > Request/response (medium) > Annotations (smallest)
Resolution: 2K (2048x1152px)

TEACHING GOAL: HTTP requests travel through multiple layers before reaching application logic
PROFICIENCY: B1 (Intermediate - has web basics, new to architecture layers)
VISUAL TYPE: Static diagram with text-in-image (labels identify layers, text shows data flow)
GOOGLE SEARCH GROUNDING: No (conceptual architecture, not specific product)

PEDAGOGICAL REASONING:
- Orthographic camera maintains technical accuracy (no distorted proportions)
- Flat lighting ensures all text is equally readable (no shadows obscuring labels)
- Semantic colors teach layer roles: blue (management) → purple (HTTP) → green (logic)
- Text-in-image shows request/response data ("GET /api/users", "200 OK") revealing what flows through system
- Left-to-right matches temporal sequence (request first, response second)
"""

response = genai.generate_image(
    model="gemini-3-pro-image",
    prompt=prompt,
    config={
        "aspect_ratio": "16:9",
        "resolution": "2K"
    }
)

image = response.images[0]
image.save("api-request-flow-iteration1.png")
```

---

**Step 4: Evaluate Pedagogically (Not Just Aesthetically)**

**Review Checklist**:

✅ **Teaching goal met?** Yes - shows layers clearly, request/response flow visible
✅ **Proficiency appropriate?** Yes - B1 can understand (4 layers = manageable)
❌ **Text hierarchy correct?** NO - "GET /api/users" is same size as layer names (should be smaller)
✅ **Semantic colors applied?** Yes - blue/purple/green distinguish layers
❌ **Accessibility?** NO - Purple (#a855f7) on white = 4.1:1 contrast (below 4.5:1 AA minimum)
✅ **Visual flow clear?** Yes - left-to-right temporal sequence understood

**Issues Identified**:
1. Text hierarchy problem: Request/response data competing with layer names (should be subordinate)
2. Accessibility problem: Purple lacks sufficient contrast

---

**Step 5: Refine with Principle-Based Feedback (Iteration 2)**

❌ **Bad Feedback**:
```
"Make request text smaller and change purple to darker color"
```
*This fixes symptoms without teaching Gemini the underlying principles.*

✅ **Good Feedback** (Principle-Based):
```
PEDAGOGICAL REFINEMENT (Iteration 2):

ISSUE 1: Text Hierarchy
Current: "GET /api/users" (16px) is competing with "Load Balancer" (24px)
Principle: Visual hierarchy = information importance
  - Layer names (24px) = primary concept (WHAT the layers are)
  - Request/response (14px) = supporting detail (WHAT flows through)
Reasoning: Students must grasp layer structure FIRST, then understand data flow SECOND.
           Current sizing gives equal weight, fragmenting learning progression.
CHANGE: Reduce request/response text to 14px (subordinate to layer names)

ISSUE 2: Accessibility Violation
Current: Purple #a855f7 on white = 4.1:1 contrast (fails WCAG AA 4.5:1)
Principle: Accessibility is non-negotiable (Constitutional Principle)
Reasoning: B1 students include learners with visual impairments; we must serve all learners.
CHANGE: Darken purple to #7c3aed (5.1:1 contrast - passes AA)

PEDAGOGICAL REASONING FOR CHANGES:
These aren't aesthetic tweaks - they're pedagogical corrections:
- Hierarchy fix: Ensures learning sequence (structure first, then data flow)
- Contrast fix: Ensures equitable access (serves all students, not just typical vision)

Both changes align with Constitutional Principle 7 (Minimal Content - every visual
element must earn its prominence through pedagogical function, not compete for attention).
```

**What Gemini Learns**:
- **Principle 1**: Text sizing reflects learning sequence priority (not arbitrary prettiness)
- **Principle 2**: Accessibility is foundational (color choices serve access first, aesthetics second)
- **Meta-principle**: Changes have pedagogical rationale (not just "looks better")

---

**Step 6: Regenerate with Refined Prompt**

```python
# Updated prompt with corrections
refined_prompt = """
[Same Subject/Composition/Action/Location/Style/Camera/Lighting as before]

Color Grading: [Updated]
  - Web Server: Purple (#7c3aed) - HTTP handling [CORRECTED for contrast]
  - [Other colors unchanged]

Text Integration: [Updated]
  - Layer names: Bold sans-serif 24px (PRIMARY - what layers are)
  - Request data: Monospace 14px on arrow [REDUCED - supporting detail]
  - Response data: Monospace 14px on arrow [REDUCED - supporting detail]
  - Annotations: Regular 12px [FURTHER REDUCED - tertiary information]
  - Visual hierarchy now correct: 24px > 14px > 12px = importance ranking

REFINEMENT REASONING:
- Text sizing now reflects learning sequence: grasp layer structure (24px primary)
  BEFORE examining data flow details (14px secondary)
- Purple darkened to #7c3aed for WCAG AA compliance (5.1:1 contrast)
- These changes serve pedagogy (learning progression + accessibility), not aesthetics
"""

response2 = genai.generate_image(
    model="gemini-3-pro-image",
    prompt=refined_prompt,
    config={"aspect_ratio": "16:9", "resolution": "2K"}
)

image2 = response2.images[0]
image2.save("api-request-flow-iteration2.png")
```

---

**Step 7: Validate Iteration (Did Gemini Learn?)**

**Check for Reasoning Transfer** (Not Just Specific Fix):

✅ **Question**: Did Gemini ONLY change the elements you mentioned?
❌ Or did it APPLY THE PRINCIPLE broadly?

**Evaluation**:
- ✅ Changed request/response to 14px (specific fix)
- ✅ Also reduced annotations to 12px (PRINCIPLE TRANSFER - maintained hierarchy even for elements you didn't mention)
- ✅ Darkened purple (specific fix)
- ✅ Verified contrast on ALL text elements, not just purple (PRINCIPLE TRANSFER - accessibility applied universally)

**Conclusion**: Gemini learned the PRINCIPLES (hierarchy = importance, accessibility = universal), not just executed fixes. This is reasoning activation working.

---

**Step 8: Final Validation**

**Pedagogical Effectiveness**:
- ✅ Teaching goal clear: "Request flows through 4 layers"
- ✅ Proficiency appropriate: B1 can grasp structure + data flow
- ✅ Text hierarchy serves learning sequence: structure first (24px), data second (14px), details third (12px)
- ✅ Semantic colors teach roles: blue=management, purple=HTTP, green=logic
- ✅ Visual flow matches temporal sequence: left→right = time progression

**Technical Quality**:
- ✅ Accessibility: All contrast ratios ≥4.5:1 (WCAG AA compliant)
- ✅ Resolution: 2K (professional documentation quality)
- ✅ Text legibility: Monospace for code, sans-serif for concepts
- ✅ Visual clarity: Flat lighting, no distractions

**Three Roles Demonstrated**:
- ✅ You taught Gemini (principle-based feedback on hierarchy and accessibility)
- ✅ Gemini taught you (showed how it interpreted "supporting detail" by also adjusting annotations)
- ✅ Co-evolved quality (Iteration 2 applies principles you established, generalizing beyond specific fixes)

---

**Step 9: Save and Document**

**Files Created**:
```
book-source/static/img/visuals/
  ├── api-request-flow-layers.png (final iteration)
  ├── api-request-flow-iteration1.png (archived - initial attempt)
  └── api-request-flow-generation-notes.md (prompt evolution documentation)
```

**Markdown Integration**:
```markdown
![HTTP Request Flow Through Architecture Layers](../../static/img/visuals/api-request-flow-layers.png)

*HTTP requests pass through Load Balancer (traffic management), Web Server (HTTP handling), and Application (business logic) layers. Solid arrow shows request path; dashed arrow shows response.*
```

**Alt Text** (Pedagogically Meaningful):
```
"HTTP request flow diagram showing left-to-right progression through four layers: Client (gray) sends GET request to Load Balancer (blue) for traffic management, forwarded to Web Server (purple) for HTTP handling, reaching Application (green) for business logic; response flows back right-to-left with 200 OK status and JSON data"
```

---

## Success Metrics for Upgraded Skills

### Visual Asset Workflow v4.0

**Reasoning Activation Score**: 5/5
- ✅ Persona: Gemini 3 Learning Asset Architect (text-in-image + interactive + grounded thinking)
- ✅ Questions: 3 new question sets (text-in-image, interactive, multi-image) + 5 enhanced sets
- ✅ Principles: 6 new principles (text-in-image, static-first, grounding, Gemini 3 prompts, multi-image, studio controls)
- ✅ Meta-awareness: Anti-convergence for text-in-image overuse, unnecessary interactivity
- ✅ Integration: Gemini 3 native patterns (official prompt structure, Search grounding, 2K/4K resolution)

**New Capabilities**:
- Distinguish text-in-image (reveals patterns) vs markdown text (explanations)
- Design interactive tier architecture (Tier 1/2/3 progressive disclosure)
- Identify Google Search grounding opportunities (factual accuracy)
- Generate reasoning-activated Gemini 3 prompts (Subject/Composition/Action/Location/Style/Camera/Lighting)
- Apply studio controls with pedagogical rationale (lighting/camera/color serve teaching goals)
- Orchestrate multi-image compositions (character consistency, concept integration)

---

### Image Generator v4.0

**Reasoning Activation Score**: 5/5
- ✅ Persona: Gemini 3-native multimodal reasoning partner (orchestrates AI reasoning, not just requests output)
- ✅ Questions: 3 new question sets (reasoning activation check, visual type execution, studio controls)
- ✅ Principles: 6 new principles (reasoning activation, multi-turn partnership, grounding, typography hierarchy, studio controls, multi-image)
- ✅ Meta-awareness: Anti-convergence for generic prompts
- ✅ Integration: Three Roles Framework (teach Gemini through principle-based iteration)

**New Capabilities**:
- Activate Gemini 3's reasoning mode (official prompt structure required)
- Execute text-in-image generation (infographics, labeled diagrams with typography hierarchy)
- Generate interactive tier architecture (overview → tap-to-reveal → deep links)
- Apply Google Search grounding (factual diagrams, real-time data)
- Use studio-quality controls with pedagogical rationale (lighting/camera/color serve learning)
- Orchestrate multi-image blending (up to 14 inputs, 5 character consistency)
- Teach Gemini quality standards through principle-based feedback (Three Roles)
- Validate reasoning transfer (did Gemini generalize principles?)

---

## Implementation Checklist

**Step 1: Update visual-asset-workflow/SKILL.md to v4.0**
- [ ] Replace Persona with v4.0 Gemini 3 Learning Asset Architect version
- [ ] Add NEW Question Set 1: Text-in-Image Opportunity Analysis
- [ ] Add NEW Question Set 2: Interactive Affordance Analysis
- [ ] Add NEW Question Set 3: Multi-Image Composition Opportunity
- [ ] Add NEW Principle 1: Text-in-Image When It Reveals
- [ ] Add NEW Principle 2: Static First, Interactive When Progressive Disclosure Serves Learning
- [ ] Add NEW Principle 3: Google Search Grounding for Factual Content
- [ ] Add NEW Principle 4: Reasoning-Activated Gemini 3 Prompts
- [ ] Add NEW Principle 5: Multi-Image Composition for Character Consistency
- [ ] Add NEW Principle 6: Studio-Quality Controls for Pedagogical Effect
- [ ] Update Workflow Output templates (static text-in-image, interactive, multi-image)
- [ ] Update Audit Report template (Gemini 3 capabilities utilized)
- [ ] Update version to 4.0.0
- [ ] Add anti-convergence meta-awareness (text-in-image overuse, unnecessary interactivity)

**Step 2: Update image-generator/SKILL.md to v4.0**
- [ ] Replace Persona with v4.0 Gemini 3-native multimodal reasoning partner version
- [ ] Add NEW Question Set 1: Gemini 3 Reasoning Activation Check
- [ ] Add NEW Question Set 2: Visual Type Execution Decision
- [ ] Add NEW Question Set 3: Studio Controls Application
- [ ] Add NEW Principle 1: Reasoning Activation Over Request Submission
- [ ] Add NEW Principle 2: Multi-Turn Reasoning Partnership (Teach Gemini Standards)
- [ ] Add NEW Principle 3: Google Search Grounding for Factual Accuracy
- [ ] Add NEW Principle 4: Text-in-Image Typography as Pedagogical Hierarchy
- [ ] Add NEW Principle 5: Studio Controls for Pedagogical Effect
- [ ] Add NEW Principle 6: Multi-Image Composition for Character Consistency
- [ ] Add complete workflow example (API Request Flow with iterations)
- [ ] Update version to 4.0.0
- [ ] Add anti-convergence meta-awareness (generic prompts)

**Step 3: Test Upgraded Skills on Real Content**
- [ ] Select 1 lesson with infographic opportunity (text-in-image test)
- [ ] Select 1 lesson with complex system diagram (interactive test)
- [ ] Select 1 lesson with character-based teaching (multi-image test)
- [ ] Invoke visual-asset-workflow v4.0 on all 3 lessons
- [ ] Verify it correctly identifies:
  - [ ] Text-in-image opportunities (infographics, labeled diagrams)
  - [ ] Interactive opportunities (progressive disclosure justified)
  - [ ] Multi-image opportunities (character consistency, concept integration)
  - [ ] Google Search grounding needs (factual content)
  - [ ] Studio controls with pedagogical rationale
- [ ] Verify generated prompts use official Gemini 3 structure
- [ ] Invoke image-generator v4.0 with generated prompts
- [ ] Verify reasoning activation (Subject/Composition/Action/Location/Style/Camera/Lighting)
- [ ] Verify multi-turn refinement with principle-based feedback
- [ ] Verify reasoning transfer (Gemini generalizes principles)
- [ ] Check outputs meet accessibility standards (WCAG AA)
- [ ] Validate pedagogical effectiveness (teaching goals achieved)

**Step 4: Document Upgrade**
- [ ] Create ADR: "Upgrading Visual Skills for Gemini 3 Pro Image Era"
- [ ] Document key changes:
  - [ ] Text-in-image decision frameworks
  - [ ] Interactive tier architecture patterns
  - [ ] Google Search grounding integration
  - [ ] Reasoning-activated prompt requirements
  - [ ] Studio controls pedagogical rationale
  - [ ] Multi-image composition workflows
- [ ] Add example outputs to skill documentation (before/after v3.0 vs v4.0)
- [ ] Update skill catalog with v4.0 capabilities
- [ ] Update CLAUDE.md if visual workflow integration changed
- [ ] Add to constitution memory if new principles established

**Step 5: Validation Metrics**
- [ ] Reasoning activation score: Both skills 5/5
- [ ] Before/after comparison: v3.0 outputs vs v4.0 outputs
- [ ] Measure improvements:
  - [ ] Pedagogical effectiveness (teaching goals achieved?)
  - [ ] Accessibility compliance (WCAG AA met?)
  - [ ] Reasoning activation (prompts activate Gemini 3 reasoning?)
  - [ ] Iteration efficiency (fewer cycles to production-ready?)
- [ ] User feedback: Do visuals enhance learning measurably?

---

## Validation: Before/After Comparison

### BEFORE (v3.0): Static-Only, Generic Prompts

**visual-asset-workflow v3.0** receives Kubernetes diagram request:
```
✅ Identifies teaching goal: "K8s orchestrates containers"
✅ Applies cognitive load analysis
✅ Generates AI image prompt
❌ Assumes static image only (no interactive consideration)
❌ Generic prompt: "Create Kubernetes architecture diagram"
❌ No Google Search grounding (misses real component specs if needed)
❌ No text-in-image analysis (could labels help?)
❌ Tier architecture not designed
```

**image-generator v3.0** generates output:
```
✅ Uses iterative refinement
✅ Checks pedagogical effectiveness
❌ Generic prompt triggers prediction mode
❌ Gemini produces "safe" architectural diagram (all 9 components shown simultaneously)
❌ Cognitive overload for B1 students
❌ No interactive affordances
❌ No principle-based feedback (just "make text bigger")
Result: Static diagram, overwhelming complexity, multiple revision cycles
```

---

### AFTER (v4.0): Gemini 3-Native, Reasoning-Activated

**visual-asset-workflow v4.0** receives same Kubernetes request:
```
✅ Identifies teaching goal: "K8s orchestrates containers"
✅ Applies text-in-image analysis: Labels needed (component names)
✅ Applies interactive affordance analysis: 9 components = cognitive overload for B1
✅ Designs tier architecture:
   - Tier 1: Control Plane + Workers (overview, 2 blocks)
   - Tier 2: Tap blocks for component details (6 + 3 components)
   - Tier 3: Deep links to Lessons 24-25 (API Server deep dive, Orchestration patterns)
✅ Identifies Google Search grounding: Optional (conceptual, not real product specs)
✅ Generates reasoning-activated Gemini 3 prompt:
   - Subject/Composition/Action/Location/Style/Camera/Lighting all specified
   - Pedagogical rationale for studio controls (flat lighting = clarity, orthographic = accuracy)
   - Text-in-image hierarchy (Control Plane 28px > Worker Nodes 24px)
```

**image-generator v4.0** generates output:
```
✅ Activates Gemini 3 reasoning mode (official prompt structure used)
✅ Generates Tier 1 (overview): Control Plane (blue, top) + Worker Nodes (green, bottom)
✅ B1-appropriate complexity (2 blocks, manageable at glance)
✅ Iterates with principle-based feedback:
   - "Control Plane" font size 28px because it's primary concept (not "make it bigger")
   - Blue (#2563eb) semantic meaning: authority/management (not "looks professional")
✅ Designs Tier 2 (tap-to-reveal): Component detail panels with mini-icons
✅ Defines Tier 3 (deep links): Connections to related lessons
✅ Generates fallback static version (maintains pedagogy if interactive unavailable)
✅ Validates reasoning transfer: Gemini applied hierarchy principle to ALL text, not just mentioned elements

Result: Interactive system—B1 students start with simple overview (Tier 1),
        explore components when ready (Tier 2), connect to deeper lessons (Tier 3).
        Static fallback preserves pedagogical value.
        Single refinement cycle (vs 5-8 in v3.0).
```

---

## The Paradigm Shift

**From**: "Create visuals that explain concepts"
**To**: "Design interactive learning systems that activate Gemini 3's reasoning capabilities to produce pedagogically effective visual experiences"

**From**: "Generate images with AI"
**To**: "Orchestrate Gemini 3 Pro's multimodal reasoning through structured prompts (Subject/Composition/Action/Location/Style/Camera/Lighting) with pedagogical rationale for every choice"

**The Core Insight**:

Gemini 3 Pro Image + Interactive Images aren't just "better image generation"—they're **a new pedagogical paradigm**:

1. **Text-in-image** becomes teaching tool (infographics reveal patterns through sizing/positioning; labels identify complex systems)
2. **Interactive affordances** enable progressive disclosure (manage cognitive load: overview → details → connections)
3. **Google Search grounding** ensures factual accuracy (scientific diagrams, real-time data verified)
4. **Studio controls** serve pedagogy (lighting directs attention, camera maintains accuracy, color communicates semantically)
5. **Multi-image composition** creates character continuity (educational comics reduce intimidation; brand consistency demonstrates application)

**These upgraded skills enable you to**:

1. **Recognize** when text-in-image serves pedagogy (infographics, diagrams) vs when markdown is clearer (explanations, lists)
2. **Design** progressive disclosure architectures (Tier 1/2/3 reduces cognitive load while enabling depth)
3. **Activate** Gemini 3's reasoning mode (official prompt structure + pedagogical rationale)
4. **Teach** Gemini your quality standards through principle-based iteration (Three Roles Framework)
5. **Integrate** with 4-Layer Teaching Method (L1=static foundations, L2=interactive exploration, L3=student-created, L4=spec-driven)
6. **Apply** studio controls with pedagogical intent (lighting/camera/color serve learning goals, not aesthetics)
7. **Ground** factual content with Google Search (accuracy for science, history, data, real-time info)

---

## Your Task

Using this prompt as the reasoning-activated specification:

1. **Read both current skills** (visual-asset-workflow v3.0, image-generator v3.0)
2. **Apply the upgrade patterns** documented above (accurate Gemini 3 capabilities, not hallucinations)
3. **Rewrite both skills to v4.0** using enhanced Persona + Questions + Principles
4. **Validate reasoning activation** (score both skills 5/5)
5. **Test on real lessons** (text-in-image, interactive, multi-image scenarios)
6. **Document the upgrade** (ADR + before/after examples)

**Success Criteria**:
- ✅ Both skills score 5/5 on reasoning activation
- ✅ Text-in-image decision framework clear (reveals vs explains)
- ✅ Static vs interactive decision framework unambiguous (justified interactivity functions)
- ✅ Tier architecture (1/2/3) fully specified for interactive visuals
- ✅ Gemini 3 reasoning mode activation (official prompt structure) mandatory
- ✅ Google Search grounding integration (factual accuracy for appropriate content)
- ✅ Studio controls with pedagogical rationale (lighting/camera/color serve teaching)
- ✅ Multi-image composition workflows (character consistency, concept integration)
- ✅ Multi-turn reasoning partnership (teach Gemini through principle-based feedback)
- ✅ Fallback strategies maintain pedagogical value (static alternatives when interactive unavailable)
- ✅ Integration with 4-Layer Method and constitution clear
- ✅ Anti-convergence meta-awareness prevents text-in-image overuse and unnecessary interactivity

**Remember**: The goal isn't to use Gemini 3's capabilities for novelty—it's to **activate reasoning about when each capability serves pedagogical goals**, then orchestrate Gemini 3 Pro's reasoning engine to produce effective visual learning experiences.

---

**This is spec-driven skill development using the reasoning activation framework with 100% accurate Gemini 3 Pro Image capabilities. Execute with full constitutional alignment.**

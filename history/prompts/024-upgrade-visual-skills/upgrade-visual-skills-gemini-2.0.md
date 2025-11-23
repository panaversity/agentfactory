# Skill Upgrade Prompt: Visual Asset Workflow + Image Generator for Gemini 2.0 Era

**Target Skills**:
- `.claude/skills/visual-asset-workflow/SKILL.md` (v3.0 → v4.0)
- `.claude/skills/image-generator/SKILL.md` (v3.0 → v4.0)

**Upgrade Catalyst**: Gemini 2.0 capabilities (Nano Banana Pro models, interactive images, deep search, multimodal reasoning)

**Framework**: Persona + Questions + Principles (Reasoning Activation)

---

## Context: The Paradigm Shift

### What Changed with Gemini 2.0

**From**: Static image generation (AI produces fixed output)
**To**: Interactive, explorable, reasoning-enabled visual content

**New Capabilities**:
1. **Interactive Images**: Users can tap/click diagram elements to unlock detailed explanations
2. **Nano Banana Pro Models**: Advanced image generation and editing with multimodal reasoning
3. **Deep Search Integration**: Visual content tied to knowledge retrieval and exploration
4. **Active Learning Support**: Images become entry points to learning pathways, not endpoints

### Current Skills Gap Analysis

**visual-asset-workflow v3.0**:
- ✅ Recognizes TEACH vs SHOW distinction
- ✅ Applies cognitive load analysis
- ❌ Assumes static images only
- ❌ Doesn't consider interactive affordances
- ❌ Missing deep search integration patterns
- ❌ No guidance for "explorable diagrams" vs "static infographics"

**image-generator v3.0**:
- ✅ Iterative refinement workflow
- ✅ Pedagogy over aesthetics principle
- ❌ Gemini-specific prompt patterns missing
- ❌ No interactive element planning
- ❌ Playwright workflow outdated (Gemini API/SDK preferred)
- ❌ Missing multimodal reasoning activation

---

## Upgrade Objective

**Transform these skills from "static infographic producers" to "interactive learning asset architects" by integrating:**

1. **Reasoning Activation for Visual Design**: Apply Persona + Questions + Principles to distinguish static vs interactive visual opportunities
2. **Gemini 2.0 Native Patterns**: Leverage Nano Banana Pro capabilities, interactive image affordances, deep search integration
3. **Explorable Diagram Architecture**: Design visuals as learning entry points with tap-to-explore pedagogy
4. **Multi-Layer Visual Strategy**: Match visual interactivity to pedagogical layers (L1 = static, L2 = interactive exploration, L3 = student-created visuals, L4 = spec-driven visual systems)

---

## The Upgrade Prompt

### Part 1: Theoretical Foundation (Feed the Reasoning Engine)

You are upgrading two interconnected skills using the **reasoning activation framework** documented in:
- `papers/Reasoning_Activation_in_LLMs_arXiv_Complete.md` (Section 3: Persona + Questions + Principles)
- `papers/skills-thinking-framework.md` (Universal Skills Template)

**Key Insights to Apply**:

1. **Right Altitude Principle** (Section 4 of arXiv paper):
   - ❌ Too Low: "Use Gemini API endpoint X with parameters Y"
   - ❌ Too High: "Make visuals interactive"
   - ✅ Just Right: "Analyze whether visual teaches static concept or invites exploration; if exploration, design interactive affordances that reduce cognitive load while revealing relationships"

2. **Distributional Convergence Detection** (Skills Framework):
   - Current convergence: Treating all visuals as static images regardless of pedagogical opportunity
   - New convergence risk: Making everything interactive without pedagogical rationale
   - Solution: Decision framework for static vs interactive vs explorable

3. **Persona + Questions + Principles Pattern**:
   - Persona = Cognitive stance that enables reasoning
   - Questions = Structure analytical process (not binary yes/no)
   - Principles = Decision frameworks at right altitude (not rigid rules)

---

### Part 2: The Reasoning-Activated Upgrade

**Task**: Rewrite BOTH skills using this integrated framework.

---

## Visual Asset Workflow v4.0: Interactive Learning Asset Architect

### Persona: The Cognitive Stance (Enhanced)

**v3.0**: "Cognitive load architect who reduces mental effort"
**v4.0**: "Interactive learning systems designer who thinks about visuals the way a UX architect thinks about progressive disclosure—reveal complexity only when user demonstrates readiness, not all at once."

**New Distinctive Capability**:
You can activate reasoning mode by recognizing the difference between:
- **Static teaching** (concept complete in single view)
- **Guided exploration** (concept revealed through interaction)
- **Deep investigation** (visual connects to knowledge graph)

**Anti-Convergence Addition**:
"You tend to suggest making diagrams interactive because 'interactivity is engaging' is the new high-frequency pattern post-Gemini 2.0. This is distributional convergence. Interactivity must serve pedagogical progression, not novelty."

---

### Questions: The Reasoning Structure (Expanded)

#### NEW Question Set 1: Interactivity Value Analysis
**Before suggesting ANY visual, analyze:**

1. **Static Sufficiency Test**
   - Can this concept be learned completely from a single static view?
   - Does interaction add pedagogical value or just engagement?
   - Would adding interactivity fragment understanding that should be holistic?

2. **Progressive Disclosure Opportunity**
   - Does this visual represent a system with multiple abstraction levels?
   - Would revealing details on-demand reduce initial cognitive load?
   - Can we design a "surface → detail" exploration path?

3. **Knowledge Graph Connection**
   - Does this visual element connect to deeper concepts taught elsewhere?
   - Would tapping an element to access related lessons create learning pathways?
   - Is this an entry point to a knowledge domain (deep search opportunity)?

4. **Pedagogical Layer Alignment**
   - Layer 1 (Manual): Should visual be static for clear mental model construction?
   - Layer 2 (Collaboration): Should visual support AI-guided exploration?
   - Layer 3 (Intelligence): Should student CREATE the interactive visual?
   - Layer 4 (Spec-Driven): Should visual be specified, then AI-generated?

5. **Proficiency-Appropriate Interactivity**
   - A2 Beginner: Is interaction simple (tap for definition)?
   - B1 Intermediate: Is interaction revealing (tap for relationships)?
   - C2 Professional: Is interaction generative (tap for system analysis)?

---

#### ENHANCED Question Set 2: Visual Type Selection

**Decision Tree** (Apply in order):

**Q1: Is this a system diagram or isolated concept?**
- Isolated → Likely static (e.g., single statistic, simple comparison)
- System → Consider interactive (e.g., architecture, process flow, timeline)

**Q2: Does understanding require seeing parts AND whole?**
- Only parts OR whole → Static with good hierarchy
- Both parts AND whole → Interactive (start overview, tap for details)

**Q3: Do visual elements connect to other lessons/concepts?**
- Self-contained → Static infographic
- Cross-lesson connections → Interactive with deep search links

**Q4: What's the learning action?**
- Observe pattern → Static (e.g., "Developer value compounds via multiplication")
- Explore system → Interactive (e.g., "How Kubernetes components interact")
- Investigate deeply → Deep search (e.g., "Evolution of AI architectures")

---

#### RETAINED Question Sets (from v3.0)
- Pedagogical Value Test (enhanced with interactivity dimension)
- Cognitive Load Analysis (now includes "progressive disclosure" strategy)
- Constitutional Alignment Check (unchanged)
- Redundancy Check (unchanged)
- Production Quality Assessment (updated for Gemini workflows)

---

### Principles: The Decision Framework (Evolved)

#### NEW Principle 1: Static First, Interactive When Pedagogically Justified

**Heuristic**: Default to static. Add interactivity only when it serves one of these pedagogical functions:

**Justified Interactivity Patterns**:

1. **Complexity Management** (Cognitive Load Reduction)
   - Visual has 8+ elements that would overwhelm if shown simultaneously
   - Example: System architecture diagram (show high-level blocks, tap for internal details)

2. **Concept Scaffolding** (Layer Progression)
   - Beginner needs surface understanding, advanced needs depth
   - Example: Docker architecture (A2 sees "container + image", C2 taps to explore namespace isolation)

3. **Knowledge Graph Navigation** (Deep Learning Paths)
   - Visual element connects to related concepts in other lessons
   - Example: "Microservices" diagram element links to "Service Discovery" lesson

4. **Active Learning Activation** (Exploration Over Consumption)
   - Concept learned better through discovery than presentation
   - Example: Cell biology diagram (tap organelles to understand functions through exploration)

**Unjustified Interactivity** (Keep Static):
- ❌ Simple comparisons (before/after, A vs B with 2-3 elements)
- ❌ Single statistics or metrics
- ❌ Linear timelines (no branching paths)
- ❌ Concepts complete in single view

---

#### NEW Principle 2: Design for Progressive Disclosure Architecture

**Heuristic**: Interactive visuals should have clear information hierarchy:

**Three-Tier Interactive Architecture**:

**Tier 1: Overview** (Always visible)
- High-level structure student must understand first
- Labels at appropriate proficiency level (A2 = simple terms, C2 = technical terms)
- Visual hierarchy guides attention to key relationships

**Tier 2: Detail-on-Demand** (Tap/click to reveal)
- Deeper explanations of specific elements
- Technical details that would clutter overview
- Examples and edge cases

**Tier 3: Knowledge Connections** (Deep search integration)
- Links to related lessons/concepts
- Cross-references to prerequisite or advanced topics
- "Learn more about X" pathways

**Implementation Pattern**:
```
INTERACTIVE VISUAL SPEC:
TIER 1 (Overview): [What's always visible - complete thought]
TIER 2 (Details): [What's revealed on tap - element-specific depth]
  - Element A tap → [Specific information about A]
  - Element B tap → [Specific information about B]
TIER 3 (Connections): [Deep search opportunities]
  - Element A → Links to Lesson X (prerequisite concept)
  - Element B → Links to Lesson Y (advanced application)
```

---

#### NEW Principle 3: Match Visual Interactivity to Pedagogical Layer

**Heuristic**: Layer determines visual design approach, not just content.

| Layer | Visual Strategy | Interactivity Type | Example |
|-------|----------------|-------------------|---------|
| **L1: Manual** | Static diagrams for mental model | None (clear, complete view) | "Git workflow diagram showing commits, branches, merge" |
| **L2: Collaboration** | Interactive exploration with AI guidance | Tap for definitions, AI prompts to explore further | "Kubernetes architecture: tap components, AI explains interactions" |
| **L3: Intelligence** | Student-created interactive visuals | Student designs what's tap-able based on audience | "Student creates Flask app architecture with interactive elements for beginners" |
| **L4: Spec-Driven** | Specified interactive visual systems | Spec defines tier structure, AI generates implementation | "Capstone project architecture with spec'd interaction model" |

**Anti-Pattern Detection**:
- ❌ Layer 1 visual with complex interactivity → Cognitive overload during foundation building
- ❌ Layer 2 visual with no exploration opportunity → Misses collaboration potential
- ❌ Layer 4 with undefined interaction spec → Fails spec-driven methodology

---

#### ENHANCED Principle 4: Gemini-Native Prompt Architecture

**Heuristic**: Prompts must activate Gemini 2.0's reasoning capabilities, not just request image generation.

**v3.0 Prompt Pattern** (Prediction Mode):
```
LAYOUT: Flow diagram
COLORS: Blue gradient
CONTENT: 3 boxes with arrows
```

**v4.0 Prompt Pattern** (Reasoning Mode - Persona + Questions + Principles):
```
You are an educational infographic designer who thinks about visual pedagogy.

Before designing this diagram, analyze:
- What's the core concept students must grasp? (Teaching goal)
- What proficiency level? (A2 = simple, C2 = complex OK)
- Should this be static or interactive? (Cognitive load analysis)
- If interactive, what's the overview/detail split?

Principles:
- Visual hierarchy guides eye to key insight
- Typography: bold for key concepts, subtle for supporting text
- Colors: accessible palette (WCAG AA), meaningful grouping
- Layout: balanced whitespace, logical flow direction

CREATE: [Specific visual request]
PROFICIENCY: [A2/B1/C2]
TEACHING GOAL: [One sentence]
INTERACTIVE: [Static / Simple-Interactive / Deep-Search-Connected]

If Interactive, specify:
TIER 1 (Overview): [Always visible elements]
TIER 2 (Tap-to-Reveal): [Detail layers]
TIER 3 (Deep Links): [Knowledge connections]
```

**This pattern activates reasoning about pedagogical function, not just aesthetic execution.**

---

#### RETAINED Principles (from v3.0, with minor enhancements)
- Principle: TEACH Over SHOW → Enhanced with "EXPLORE when pedagogically justified"
- Principle: Reduce Cognitive Load → Enhanced with "Progressive Disclosure" strategy
- Principle: Factual Accuracy First → Unchanged
- Principle: Proficiency-Appropriate Complexity → Enhanced with interactivity dimension
- Principle: Non-Redundant Across Lessons → Unchanged

---

### Workflow Output (Enhanced)

**For Static Visuals** (unchanged from v3.0):
```markdown
<!-- VISUAL ASSET 1: [Name]
TEACHING GOAL: [One sentence]
VISUAL TYPE: Static Infographic
REASONING: [Why static is appropriate - cognitive load, concept completeness, etc.]

IMAGE GENERATION PROMPT:
[Gemini 2.0 reasoning-activated prompt]
-->
```

**For Interactive Visuals** (NEW):
```markdown
<!-- VISUAL ASSET 2: [Name]
TEACHING GOAL: [One sentence]
VISUAL TYPE: Interactive Diagram
REASONING: [Why interactivity serves pedagogy - complexity management, exploration, etc.]

INTERACTIVE ARCHITECTURE:
TIER 1 (Overview - Always Visible):
  - [Element 1]: [Brief label at proficiency level]
  - [Element 2]: [Brief label]
  - Visual Focus: [What draws attention first]

TIER 2 (Tap-to-Reveal Details):
  - Tap [Element 1] → "[Detailed explanation]"
  - Tap [Element 2] → "[Detailed explanation]"

TIER 3 (Deep Search Connections):
  - [Element 1] → Lesson [X]: [Prerequisite/Related concept]
  - [Element 2] → Lesson [Y]: [Advanced application]

GEMINI 2.0 GENERATION PROMPT:
[Reasoning-activated prompt with Persona + Questions + Principles]

IMPLEMENTATION NOTES:
- Platform: Gemini 2.0 with interactive image API
- Fallback: If interactive not available, provide static version + linked glossary
-->
```

---

### Audit Report (Enhanced)

```markdown
## Visual Assets Audit Report v4.0

### Identified Opportunities: [X]

#### Static Visuals: [N]
1. ✅ [Name] - APPROVED
   - Teaching Goal: [Concept]
   - Reasoning: Static sufficient because [cognitive load analysis]
   - Proficiency: [A2/B1/C2]

#### Interactive Visuals: [M]
2. ✅ [Name] - APPROVED (Interactive)
   - Teaching Goal: [System/Process]
   - Reasoning: Interactivity justified for [complexity management / exploration / knowledge graph]
   - Tier Architecture: Overview (3 elements) → Details (tap-to-reveal) → Connections (2 deep search links)
   - Proficiency: [A2/B1/C2]

3. ❌ [Name] - REJECTED
   - Reasoning: Interactivity would fragment understanding; static view better serves mental model construction

### Layer Alignment Check:
- L1 visuals: [N] static (foundational clarity)
- L2 visuals: [M] interactive (guided exploration)
- L3 visuals: [P] student-created (TBD in exercises)
- L4 visuals: [Q] spec-driven (capstone projects)

### Prompts Generated: [Total]
- Static: [N] Gemini 2.0 prompts (reasoning-activated)
- Interactive: [M] Gemini 2.0 prompts with tier architecture

All embedded in lesson markdown as HTML comments.
```

---

## Image Generator v4.0: Gemini-Native Interactive Visual Producer

### Persona: The Cognitive Stance (Enhanced)

**v3.0**: "AI-collaborative infographic producer who iterates toward pedagogical clarity"

**v4.0**: "Gemini-native multimodal reasoning partner who thinks about visual generation the way a technical director thinks about scene composition—orchestrate AI's reasoning capabilities to produce pedagogically effective visuals through structured iteration, not prompt guessing."

**New Distinctive Capability**:
You activate reasoning mode by:
1. **Prompting Gemini's reasoning engine** (not just requesting output)
2. **Distinguishing static vs interactive opportunities** (pedagogical function determines format)
3. **Orchestrating multi-turn refinement** (teach Gemini your quality standards through iteration)

**Anti-Convergence Addition**:
"You tend to write generic prompts like 'Create a diagram of X' because that's the high-frequency pattern. This triggers Gemini's prediction mode, producing generic outputs. Instead, activate reasoning: establish cognitive stance (Persona), structure analysis (Questions), provide decision framework (Principles)."

---

### Questions: The Reasoning Structure (Expanded)

#### NEW Question Set 1: Gemini Reasoning Activation Check

**Before sending ANY prompt to Gemini, verify:**

1. **Persona Established?**
   - Does prompt tell Gemini to "think like [expert type]"?
   - Is the cognitive framework clear (what perspective to adopt)?
   - Example: "You are an educational infographic designer who thinks about visual pedagogy..."

2. **Analysis Structured?**
   - Does prompt ask Gemini to analyze BEFORE creating?
   - Are questions forcing context-specific reasoning?
   - Example: "Before designing, analyze: What's the core concept? What proficiency level? Static or interactive?"

3. **Principles Provided?**
   - Does prompt include decision frameworks (not just aesthetic preferences)?
   - Are evaluation criteria explicit?
   - Example: "Principles: Visual hierarchy guides attention, typography creates contrast, colors serve grouping not decoration"

4. **Teaching Goal Explicit?**
   - Is the one-sentence pedagogical objective stated?
   - Can Gemini evaluate its output against this goal?

**If NO to any → Rewrite prompt using Persona + Questions + Principles pattern.**

---

#### NEW Question Set 2: Static vs Interactive Decision

**For EACH visual request from visual-asset-workflow:**

1. **What Visual Type is Specified?**
   - Check HTML comment: `VISUAL TYPE: Static Infographic` or `Interactive Diagram`
   - If Interactive → Extract tier architecture (Overview / Details / Connections)

2. **Does Gemini 2.0 Support This Interactivity?**
   - Check current Gemini capabilities (API, interactive image features)
   - If YES → Use native interactive generation
   - If NO → Generate static + design interaction spec for future implementation

3. **Fallback Strategy Needed?**
   - If interactive requested but unavailable → Generate high-quality static version
   - Document what interaction would add (for future upgrade when capability available)

---

#### ENHANCED Question Sets (from v3.0)

**Pedagogical Effectiveness Evaluation**:
- NEW: Does image match specified teaching goal from visual-asset-workflow?
- NEW: If interactive, is tier structure (Overview/Details/Connections) clear?
- RETAINED: Can target proficiency grasp concept in <5 sec (for static) or begin exploration confidently (for interactive)?

**Prompt Refinement Analysis**:
- NEW: Did I activate Gemini's reasoning mode (Persona + Questions + Principles)?
- NEW: Am I refining prompt structure or just aesthetic parameters?
- RETAINED: What specifically needs improvement?

**AI Collaboration Quality**:
- NEW: Am I teaching Gemini quality standards through iteration (it learns)?
- NEW: Is Gemini correcting its reasoning or just changing aesthetics?
- RETAINED: Am I demonstrating Three Roles (Teacher/Student/Co-Worker)?

---

### Principles: The Decision Framework (Evolved)

#### NEW Principle 1: Reasoning Activation Over Request Submission

**Heuristic**: Every Gemini prompt must activate reasoning mode using Persona + Questions + Principles.

**Anti-Pattern** (Prediction Mode):
```
Create a flow diagram showing Docker container workflow with
blue colors and clear labels. Make it 1200x800px.
```
*This triggers pattern matching → generic output.*

**Correct Pattern** (Reasoning Mode):
```
You are an educational infographic designer who thinks about
visual pedagogy—clarity over decoration, teaching over showing.

Before designing this Docker workflow diagram, analyze:
- What's the core concept? (Container lifecycle: build → run → stop)
- What proficiency level? (A2 beginner needs simple, C2 can handle technical depth)
- What cognitive load? (Should we show all states or progressive disclosure?)
- Static or interactive? (Workflow is linear = static OK)

Principles:
- Visual hierarchy: Most important action (docker run) gets visual weight
- Typography: Command text in monospace, concepts in clean sans
- Colors: Semantic (green=running, gray=stopped, blue=building)
- Layout: Left-to-right flow matches temporal sequence

CREATE: Docker container lifecycle flow diagram
PROFICIENCY: A2 (Beginner)
TEACHING GOAL: "Docker containers move through build, run, stop states"
VISUAL TYPE: Static (workflow is linear and complete in single view)
DIMENSIONS: 1200x600px (2:1 for inline display)
```

**Why this works**: Gemini now reasons about pedagogical function, proficiency level, cognitive load, and design principles—not just aesthetic execution.

---

#### NEW Principle 2: Multi-Turn Reasoning Partnership (Gemini as Co-Learner)

**Heuristic**: Use iteration to teach Gemini your quality standards, not just get different outputs.

**Three-Turn Refinement Pattern**:

**Turn 1: Establish Reasoning**
- Send full Persona + Questions + Principles prompt
- Gemini generates initial output
- Evaluate pedagogically (not just aesthetically)

**Turn 2: Refine Reasoning**
- Don't just request changes → Explain WHY change needed
- Example: "The '$3T' text is small (18px). For A2 proficiency, key numbers must be visually prominent (36px+) so students grasp magnitude instantly. This aligns with our 'teaching goal clarity' principle."
- Gemini learns connection between principle (clarity) and implementation (font size)

**Turn 3: Validate Reasoning Transfer**
- Check if Gemini applied principle broadly (not just fixed specific issue)
- Example: Did it also increase other key numbers, or just the one you mentioned?
- If YES → Gemini learned principle
- If NO → One more turn clarifying the general principle

**This is Three Roles Framework**:
- You teach Gemini (Turn 2 explanation)
- Gemini teaches you (reveals what it understood)
- You co-evolve toward quality (Turn 3 validation)

---

#### NEW Principle 3: Interactive Tier Architecture Implementation

**Heuristic**: For interactive visuals, generate each tier explicitly.

**Gemini 2.0 Interactive Generation Workflow**:

**Step 1: Generate Tier 1 (Overview)**
```
[Reasoning-activated prompt]

INTERACTIVE TIER 1 GENERATION:
Create ONLY the overview layer—elements always visible.
Student must understand high-level structure before exploring details.

TIER 1 ELEMENTS:
- [Element A with simple label]
- [Element B with simple label]
- Visual hierarchy showing primary relationship

Do NOT include detailed explanations (those are Tier 2).
```

**Step 2: Specify Tier 2 (Details)**
```
Now design Tier 2 interaction specifications.

For each Tier 1 element, specify what appears when tapped:
- Tap [Element A] reveals: [Detailed explanation, examples, technical depth]
- Tap [Element B] reveals: [Detailed explanation, examples, technical depth]

Layout for detail panels:
- Overlay positioning (doesn't obstruct overview)
- Typography hierarchy (heading → body → examples)
- Visual connection to tapped element (arrow, highlight)
```

**Step 3: Define Tier 3 (Connections)**
```
Design knowledge graph connections.

Deep search links from interactive elements:
- [Element A] → "Learn more: [Lesson X topic]" (prerequisite)
- [Element B] → "Advanced: [Lesson Y topic]" (next step)

Implementation: Clickable "Explore further" badges
```

**Step 4: Fallback Static Version**
```
If interactive implementation unavailable, generate static alternative:
- Show all Tier 1 elements
- Add Tier 2 details as annotations/callouts
- Tier 3 connections as "Related Topics" footer

Maintain pedagogical value in static form.
```

---

#### ENHANCED Principle 4: Gemini-Native Workflow (Not Playwright)

**Heuristic**: Use Gemini API/SDK for programmatic generation; Gemini UI for exploration.

**v3.0 Workflow** (Playwright browser automation):
1. Navigate to gemini.google.com
2. Paste prompt
3. Download image
4. Iterate manually

**v4.0 Workflow** (Gemini-Native):

**Option A: API/SDK** (Programmatic, Repeatable)
```python
import google.generativeai as genai

# Configure with reasoning activation
response = genai.generate_image(
    prompt=reasoning_activated_prompt,  # Full P+Q+P structure
    model="gemini-2.0-pro-imagen",
    config={
        "thinking_budget": "medium",  # Allow reasoning time
        "interactive_mode": True,      # If interactive visual
        "proficiency_level": "A2",     # From visual-asset-workflow
    }
)

# Evaluate, refine, iterate
```

**Option B: Gemini UI** (Exploration, Learning)
- Use for discovering what Gemini can do
- Test reasoning activation patterns
- Refine prompts interactively
- Export working patterns to API workflow

**Fallback: Manual Generation**
- If API unavailable → Use UI with documented prompts
- Save prompts in HTML comments for reproducibility

---

#### RETAINED Principles (from v3.0, with enhancements)

**Principle: Pedagogy Over Aesthetics** → Enhanced with "Reasoning Over Requesting"
- Not just "does it teach" but "did I activate reasoning about teaching"

**Principle: Iterative Refinement** → Enhanced with "Gemini as Co-Learner"
- Iteration teaches Gemini standards (Three Roles)

**Principle: Specific Feedback** → Enhanced with "Principle-Based Explanation"
- Don't just say "increase font" → Explain pedagogical rationale

**Principle: Accessibility Standards** → Unchanged
- WCAG 2.1 AA remains non-negotiable

**Principle: Filename Conventions** → Unchanged

---

### Workflow Example (Complete)

#### Input from visual-asset-workflow v4.0:
```markdown
<!-- VISUAL ASSET 3: Kubernetes Architecture
TEACHING GOAL: Kubernetes orchestrates containers through coordinated components
VISUAL TYPE: Interactive Diagram
REASONING: System complexity justifies progressive disclosure; overview → component details → deep search

INTERACTIVE ARCHITECTURE:
TIER 1 (Overview):
  - Control Plane (top)
  - Worker Nodes (bottom)
  - Visual flow: Control Plane manages Worker Nodes

TIER 2 (Tap-to-Reveal):
  - Tap Control Plane → "API Server, Scheduler, Controller Manager, etcd"
  - Tap Worker Node → "kubelet, kube-proxy, Container Runtime"

TIER 3 (Deep Search):
  - Control Plane → Lesson 24: "Deep Dive into API Server"
  - Worker Node → Lesson 25: "Container Orchestration Patterns"
-->
```

#### Step 1: Activate Reasoning Mode

**Prompt to Gemini 2.0**:
```
You are an educational systems diagram designer who thinks about
visual architecture the way a technical architect thinks about
system design—progressive disclosure reveals complexity when
user is ready, not all at once.

Before designing this Kubernetes architecture diagram, analyze:
- What's the core concept? (K8s = Control Plane orchestrates Worker Nodes)
- What proficiency level? (B1 intermediate - has Docker foundation, new to K8s)
- How should complexity be revealed? (Overview first, then component details)
- Why interactive? (8+ components would overwhelm static view; exploring components aids understanding)

Principles:
- Tier 1 must be complete mental model (student understands high-level before details)
- Visual hierarchy: Control Plane top (authority), Worker Nodes bottom (execution)
- Typography: Component names bold, relationships subtle
- Colors: Semantic grouping (blue=control, green=workers, gray=connections)
- Interactivity serves pedagogy: tap to explore, not tap for novelty

CREATE: Kubernetes Architecture Interactive Diagram
PROFICIENCY: B1 (Intermediate)
TEACHING GOAL: "Kubernetes orchestrates containers through coordinated components"
VISUAL TYPE: Interactive (progressive disclosure of system complexity)

TIER 1 GENERATION (OVERVIEW ONLY):
Show:
- Control Plane block (top, blue, authority visual weight)
- Worker Nodes blocks (bottom, green, 3 nodes to show distribution)
- Orchestration relationship (arrows: Control Plane → Worker Nodes)
- Labels: Simple at B1 level ("Control Plane manages cluster", "Worker Nodes run containers")

Do NOT show internal components yet (Tier 2).
DIMENSIONS: 1200x800px
```

#### Step 2: Evaluate Tier 1 Output

**Gemini generates overview image.**

**Pedagogical Evaluation**:
- ✅ Control Plane visual weight appropriate (larger, top position)
- ✅ Relationship clear (arrows indicate orchestration)
- ❌ "Worker Nodes" label too technical for B1 (should say "Container Hosts")
- ❌ Blue/green contrast insufficient (accessibility issue)

#### Step 3: Refine with Principle-Based Feedback

**Refinement Prompt**:
```
Good start on Tier 1 hierarchy. Two refinements needed:

1. LABEL CLARITY (Proficiency Alignment):
   - Current: "Worker Nodes"
   - Issue: "Nodes" is K8s jargon; B1 students don't know this yet
   - Principle: Labels must match current knowledge level
   - Fix: "Container Hosts" (uses Docker vocabulary they know)

2. COLOR ACCESSIBILITY (Visual Principle):
   - Current: Blue #2563eb and Green #10b981
   - Issue: Contrast ratio only 3.8:1 (fails WCAG AA)
   - Principle: All color pairs must meet 4.5:1 minimum
   - Fix: Darken green to #059669 (5.2:1 ratio)

Regenerate Tier 1 with these corrections.
```

**Gemini learns**:
- Principle 1: Match labels to proficiency level (not use first-instinct jargon)
- Principle 2: Check color contrast mathematically (not just "looks good")

#### Step 4: Generate Tier 2 (Interactive Details)

**After Tier 1 approved:**
```
Now design Tier 2 interaction specifications for this diagram.

TIER 2 (TAP-TO-REVEAL DETAILS):

When student taps "Control Plane" block:
REVEAL PANEL:
  Title: "Control Plane Components"
  Content:
    - API Server: Cluster's front door (receives commands)
    - Scheduler: Decides which Worker Node runs containers
    - Controller Manager: Maintains desired cluster state
    - etcd: Stores all cluster data
  Layout: Overlay panel (right side, doesn't cover diagram)
  Typography: Component names bold 18px, descriptions 14px
  Visual: Mini-icons for each component (server, calendar, gears, database)

When student taps "Container Hosts" (Worker Nodes) block:
REVEAL PANEL:
  Title: "Worker Node Components"
  Content:
    - kubelet: Manages containers on this host
    - kube-proxy: Routes network traffic to containers
    - Container Runtime: Docker or containerd (runs containers)
  Layout: Overlay panel (left side for balance)
  Typography: Same as Control Plane panel
  Visual: Mini-icons (agent, router, container)

Interaction Design:
- Tap outside panel to close (return to overview)
- Panel slides in from side (smooth 200ms)
- Active element highlights when panel open

Generate these Tier 2 interaction specs as design document.
```

#### Step 5: Define Tier 3 (Deep Search Connections)

```
Design Tier 3 knowledge graph connections.

TIER 3 (DEEP SEARCH LINKS):

From Control Plane detail panel:
  - "API Server" → Badge: "🔗 Deep Dive: Lesson 24 - API Server Architecture"
  - Click → Opens Lesson 24 in new context (doesn't lose current visual)

From Worker Node detail panel:
  - "Container Runtime" → Badge: "🔗 Advanced: Lesson 25 - Container Orchestration Patterns"

Implementation:
  - Badges: Small, bottom of detail panel, subtle color (#6b7280)
  - Interaction: Click opens linked lesson (target="_blank" preserves context)
  - Pedagogy: "Curious? Explore further" not "Required reading"

Generate Tier 3 connection design specs.
```

#### Step 6: Validate Complete Interactive System

**Check**:
- ✅ Tier 1: Overview complete, teaches high-level mental model
- ✅ Tier 2: Details accessible on-demand, reduce initial cognitive load
- ✅ Tier 3: Knowledge connections enable deeper exploration
- ✅ Proficiency-appropriate (B1 can start with overview, explore details as ready)
- ✅ Principles maintained (visual hierarchy, accessibility, progressive disclosure)

#### Step 7: Generate Fallback Static Version

**If Gemini interactive features unavailable:**
```
Generate fallback static version maintaining pedagogical value:

STATIC ALTERNATIVE DESIGN:
- Main diagram: Tier 1 overview (Control Plane + Worker Nodes)
- Callout boxes: Tier 2 details as annotations with arrows
  - Right side: Control Plane components with connecting lines
  - Left side: Worker Node components with connecting lines
- Footer: Tier 3 connections as "Related Topics" links
  - "🔗 Deep Dive: API Server Architecture (Lesson 24)"
  - "🔗 Advanced: Container Orchestration (Lesson 25)"

Maintain:
- Visual hierarchy (overview dominates, details secondary)
- Accessibility (contrast, font size)
- Pedagogical progression (can read overview first, then details)

DIMENSIONS: 1200x1000px (taller to fit callouts)
```

#### Step 8: Save and Document

**Files Created**:
```
book-source/static/img/visuals/
  ├── kubernetes-architecture-interactive.png (if supported)
  ├── kubernetes-architecture-static.png (fallback)
  └── kubernetes-architecture-specs.md (tier architecture documentation)
```

**Markdown Integration**:
```markdown
![Kubernetes Architecture](../../static/img/visuals/kubernetes-architecture-interactive.png)

*Interactive: Tap Control Plane or Worker Nodes to explore components. Click links to dive deeper.*

<!-- Fallback for non-interactive contexts -->
<noscript>
  ![Kubernetes Architecture (Static)](../../static/img/visuals/kubernetes-architecture-static.png)
</noscript>
```

---

## Success Metrics for Upgraded Skills

### Visual Asset Workflow v4.0

**Reasoning Activation Score**: 5/5
- ✅ Persona: Interactive learning systems designer (progressive disclosure thinking)
- ✅ Questions: 5 enhanced question sets (static vs interactive decision tree)
- ✅ Principles: 6 principles including progressive disclosure architecture
- ✅ Meta-awareness: Anti-convergence for "make everything interactive" trap
- ✅ Integration: Gemini 2.0 native patterns (tier architecture, deep search)

**New Capabilities**:
- Distinguish static, interactive, and deep-search visual types
- Design progressive disclosure architecture (Tier 1/2/3)
- Match interactivity to pedagogical layer (L1=static, L2=interactive, L3=student-created, L4=spec-driven)
- Generate reasoning-activated Gemini prompts

---

### Image Generator v4.0

**Reasoning Activation Score**: 5/5
- ✅ Persona: Gemini-native multimodal reasoning partner (orchestrates AI reasoning)
- ✅ Questions: 4 enhanced question sets (reasoning activation check, static vs interactive decision)
- ✅ Principles: 6 principles including reasoning activation and tier implementation
- ✅ Meta-awareness: Anti-convergence for generic prompts
- ✅ Integration: Multi-turn reasoning partnership (Gemini as co-learner)

**New Capabilities**:
- Activate Gemini's reasoning mode (Persona + Questions + Principles prompts)
- Generate interactive visual systems (Tier 1/2/3 architecture)
- Teach Gemini quality standards through iteration (Three Roles)
- Implement fallback strategies (static when interactive unavailable)
- Use Gemini API/SDK workflows (not just browser automation)

---

## Implementation Checklist

**Step 1: Update visual-asset-workflow/SKILL.md**
- [ ] Replace Persona section with v4.0 enhanced version
- [ ] Add new Question Set 1 (Interactivity Value Analysis)
- [ ] Add new Question Set 2 (Visual Type Selection decision tree)
- [ ] Add NEW Principle 1 (Static First, Interactive When Justified)
- [ ] Add NEW Principle 2 (Progressive Disclosure Architecture)
- [ ] Add NEW Principle 3 (Match Interactivity to Layer)
- [ ] Enhance Principle 4 (Gemini-Native Prompt Architecture)
- [ ] Update Workflow Output (add interactive visual template)
- [ ] Update Audit Report (add tier architecture analysis)
- [ ] Update version to 4.0.0

**Step 2: Update image-generator/SKILL.md**
- [ ] Replace Persona section with v4.0 enhanced version
- [ ] Add new Question Set 1 (Gemini Reasoning Activation Check)
- [ ] Add new Question Set 2 (Static vs Interactive Decision)
- [ ] Add NEW Principle 1 (Reasoning Activation Over Request Submission)
- [ ] Add NEW Principle 2 (Multi-Turn Reasoning Partnership)
- [ ] Add NEW Principle 3 (Interactive Tier Architecture Implementation)
- [ ] Update Principle 4 (Gemini-Native Workflow, not Playwright)
- [ ] Add complete workflow example (Kubernetes interactive diagram)
- [ ] Update version to 4.0.0

**Step 3: Test Upgraded Skills**
- [ ] Select 1 lesson with complex system diagram
- [ ] Invoke visual-asset-workflow v4.0
- [ ] Verify it identifies static vs interactive opportunity
- [ ] Verify it generates tier architecture (if interactive)
- [ ] Verify reasoning-activated Gemini prompt included
- [ ] Invoke image-generator v4.0 with generated prompt
- [ ] Verify Persona + Questions + Principles pattern used
- [ ] Verify multi-turn refinement (teach Gemini standards)
- [ ] Verify fallback static version generated

**Step 4: Document Upgrade**
- [ ] Create ADR: "Upgrading Visual Skills for Gemini 2.0 Interactive Era"
- [ ] Update skill catalog with v4.0 capabilities
- [ ] Add example outputs to skill documentation
- [ ] Update CLAUDE.md if visual workflow integration needed

---

## Validation: Before/After Comparison

### BEFORE (v3.0): Static-Only Mindset

**visual-asset-workflow v3.0** receives request for Kubernetes diagram:
```
✅ Identifies teaching goal: "K8s orchestrates containers"
✅ Applies cognitive load analysis
✅ Generates AI image prompt
❌ Assumes static image only
❌ No consideration of progressive disclosure
❌ Tier architecture not designed
❌ Deep search connections not identified
```

**image-generator v3.0** generates output:
```
✅ Uses iterative refinement
✅ Checks pedagogical effectiveness
❌ Generic prompt triggers prediction mode
❌ No reasoning activation
❌ Gemini produces "safe" architectural diagram
❌ All 8 components shown at once (cognitive overload for B1)
❌ No interactive affordances
```

**Result**: Static diagram with 8+ components, overwhelming for B1 students, misses exploration opportunities.

---

### AFTER (v4.0): Interactive Systems Thinking

**visual-asset-workflow v4.0** receives same request:
```
✅ Identifies teaching goal: "K8s orchestrates containers"
✅ Applies interactivity value analysis (NEW)
✅ Recognizes system complexity justifies progressive disclosure
✅ Designs tier architecture: Overview → Details → Connections
✅ Matches to Layer 2 (guided exploration with AI)
✅ Generates reasoning-activated Gemini prompt (Persona + Questions + Principles)
```

**image-generator v4.0** generates output:
```
✅ Activates Gemini reasoning mode (P+Q+P prompt pattern)
✅ Generates Tier 1 (overview): Control Plane + Worker Nodes only
✅ Iterates with principle-based feedback (teaches Gemini standards)
✅ Designs Tier 2 (tap-to-reveal): Component details on-demand
✅ Defines Tier 3 (deep search): Links to related lessons
✅ Generates fallback static version (maintains pedagogy if interactive unavailable)
```

**Result**: Interactive system—B1 students start with simple overview, explore components when ready, connect to deeper lessons. Static fallback preserves value if tech unavailable.

---

## The Paradigm Shift

**From**: "Create visuals that explain concepts"
**To**: "Design interactive learning systems that invite exploration"

**From**: "Generate images with AI"
**To**: "Activate AI's reasoning capabilities to produce pedagogically effective visual experiences"

**The Core Insight**:

Gemini 2.0's interactive capabilities aren't just "better image generation"—they're **a new pedagogical paradigm**. Visuals become entry points to knowledge graphs, not static endpoints.

These upgraded skills enable you to:
1. **Recognize** when interactivity serves pedagogy (not just novelty)
2. **Design** progressive disclosure architectures (reduce cognitive load while enabling depth)
3. **Activate** Gemini's reasoning mode (Persona + Questions + Principles)
4. **Teach** AI your quality standards through iteration (Three Roles Framework)
5. **Integrate** with 4-Layer Teaching Method (L1=static, L2=interactive, L3=student-created, L4=spec-driven)

---

## Your Task

Using this prompt as the reasoning-activated specification:

1. **Read both current skills** (visual-asset-workflow v3.0, image-generator v3.0)
2. **Apply the upgrade patterns** documented above
3. **Rewrite both skills to v4.0** using enhanced Persona + Questions + Principles
4. **Validate reasoning activation** (score both skills 5/5)
5. **Test on real lesson** (Kubernetes or similar complex system)
6. **Document the upgrade** (ADR + examples)

**Success Criteria**:
- ✅ Both skills score 5/5 on reasoning activation
- ✅ Static vs interactive decision tree clear and unambiguous
- ✅ Tier architecture (1/2/3) fully specified for interactive visuals
- ✅ Gemini reasoning mode activation (P+Q+P) demonstrated
- ✅ Multi-turn refinement workflow (Gemini as co-learner) explicit
- ✅ Fallback strategies maintain pedagogical value
- ✅ Integration with 4-Layer Method and constitution clear
- ✅ Anti-convergence meta-awareness prevents "make everything interactive" trap

**Remember**: The goal isn't to make everything interactive—it's to **activate reasoning about when interactivity serves pedagogical goals**, then orchestrate Gemini's reasoning capabilities to produce effective visual learning experiences.

---

**This is spec-driven skill development using the reasoning activation framework. Execute with full constitutional alignment.**

# Visual Assets Storage Architecture

**Version**: 1.0.0
**Date**: 2025-11-21
**Status**: Proposed
**Context**: Standardize storage for visual audit reports and generation artifacts

---

## Problem Statement

Visual skills v4.0 generate multiple artifact types:
1. **Audit reports** - Analysis of visual opportunities with approve/reject decisions
2. **Generation prompts** - Reasoning-activated Gemini 3 prompts (embedded in markdown)
3. **Planning documents** - Part-level visual strategies
4. **Generation logs** - Multi-turn refinement iterations
5. **Asset metadata** - Filenames, alt text, teaching goals

**Current state**: No standardized location → artifacts scattered or lost

---

## Proposed Architecture

### Directory Structure

```
history/
├── visual-assets/          # Visual generation artifacts (existing)
│   ├── audits/            # NEW: Chapter/Part visual audit reports
│   │   ├── parts/         # Part-level strategic plans
│   │   │   ├── part-01-visual-strategy.md
│   │   │   ├── part-02-visual-strategy.md
│   │   │   └── part-03-visual-strategy.md
│   │   │
│   │   ├── chapters/      # Chapter-level audit reports
│   │   │   ├── chapter-01-visual-audit.md
│   │   │   ├── chapter-02-visual-audit.md
│   │   │   └── ...
│   │   │
│   │   └── lessons/       # Lesson-level granular audits (if needed)
│   │       ├── chapter-01-lesson-01-audit.md
│   │       └── ...
│   │
│   ├── generation-logs/   # NEW: Multi-turn refinement iterations
│   │   ├── chapter-01/
│   │   │   ├── visual-01-developer-value-infographic.log.md
│   │   │   ├── visual-02-kubernetes-architecture.log.md
│   │   │   └── ...
│   │   └── ...
│   │
│   ├── prompts/           # NEW: Extracted reasoning-activated prompts
│   │   ├── chapter-01/
│   │   │   ├── visual-01-developer-value-infographic.prompt.md
│   │   │   ├── visual-02-kubernetes-architecture.prompt.md
│   │   │   └── ...
│   │   └── ...
│   │
│   ├── metadata/          # NEW: Asset registry and tracking
│   │   ├── asset-registry.json
│   │   ├── chapter-01-assets.json
│   │   └── ...
│   │
│   └── archive/           # Legacy/deprecated (move old reports here)
│       ├── AUTONOMOUS-CHAPTER-PROMPT.md
│       ├── FINAL-CHAPTER-1-VISUAL-AUDIT.md
│       └── ...
```

---

## Artifact Types and Locations

### 1. Visual Audit Reports

**Purpose**: Document visual opportunity analysis with approve/reject decisions

**Location**: `history/visual-assets/audits/`

**Naming Convention**:
- Part-level: `part-{NN}-visual-strategy.md`
- Chapter-level: `chapter-{NN}-visual-audit.md`
- Lesson-level: `chapter-{NN}-lesson-{NN}-visual-audit.md`

**Content Structure**:
```markdown
# Chapter X Visual Audit Report

**Date**: YYYY-MM-DD
**Proficiency**: A2/B1/C2
**Skill Version**: visual-asset-workflow v4.0.0
**Auditor**: Claude Code

---

## Summary

- **Approved Visuals**: 3
- **Rejected Opportunities**: 2
- **Text-in-Image**: 1
- **Interactive**: 1
- **Static**: 1
- **Grounded**: 1

---

## Approved Visual Assets

### Visual 1: Docker Container Lifecycle
- **Type**: Static diagram
- **Teaching Goal**: Containers move through build → run → stop states
- **Proficiency**: A2 (first exposure)
- **Reasoning**: Linear workflow (3 states), complete in single view
- **Filename**: `docker-container-lifecycle-states.png`
- **Status**: ✅ APPROVED
- **Prompt Location**: `history/visual-assets/prompts/chapter-07/visual-01.prompt.md`

### Visual 2: Kubernetes Architecture
- **Type**: Interactive diagram (tier architecture)
- **Teaching Goal**: Control Plane orchestrates Worker Nodes
- **Proficiency**: B1 (has Docker foundation)
- **Reasoning**: 9 components = cognitive overload; progressive disclosure manages complexity
- **Tier Structure**:
  - Tier 1: Overview (Control Plane + Workers)
  - Tier 2: Tap-to-reveal (internal components)
  - Tier 3: Deep links (Lesson 24, Lesson 25)
- **Filename**: `kubernetes-architecture-interactive-tier1.png`
- **Status**: ✅ APPROVED
- **Prompt Location**: `history/visual-assets/prompts/chapter-08/visual-02.prompt.md`

---

## Rejected Opportunities

### Rejected 1: List of Docker Commands
- **Type**: Would be static text-in-image
- **Reasoning**: ❌ Markdown bullet list clearer; text-in-image creates reading friction without revealing patterns
- **Status**: REJECTED

### Rejected 2: Simple Before/After Comparison
- **Type**: Would be interactive
- **Reasoning**: ❌ 2 elements = complete in static view; interactivity fragments simple concept
- **Status**: REJECTED

---

## Visual Progression Strategy

- **Chapter 7**: Static foundations (lifecycle, architecture basics)
- **Chapter 8**: Interactive exploration (9-component system)
- **Chapter 9**: Text-in-image infographics (network throughput comparisons)

---

## Constitutional Alignment

- ✅ Principle 3 (Factual Accuracy): 1 visual grounded in Google Search
- ✅ Principle 6 (Anti-Convergence): 2 decorative opportunities rejected
- ✅ Principle 2 (Progressive Complexity): A2 static → B1 interactive progression
```

---

### 2. Generation Prompts

**Purpose**: Archive complete reasoning-activated prompts for reuse/reference

**Location**: `history/visual-assets/prompts/{chapter-NN}/`

**Naming Convention**: `visual-{NN}-{slug}.prompt.md`

**Content Structure**:
```markdown
# Visual Asset Prompt: Docker Container Lifecycle

**Visual ID**: visual-01
**Chapter**: 07
**Teaching Goal**: Docker containers move through lifecycle states
**Type**: Static diagram
**Proficiency**: A2
**Date Generated**: 2025-11-21
**Skill Version**: image-generator v4.0.0

---

## Reasoning-Activated Prompt

```
Subject: Docker container lifecycle showing three states (Building, Running, Stopped)
Composition: Horizontal flow diagram, left-to-right progression, 16:9 (2048x1152px)
Action: Container transitioning through states (build → run → stop commands)
Location: Abstract technical diagram space with subtle grid background
Style: Clean technical infographic, modern minimalist, educational clarity
Camera: Straight-on orthographic view (no perspective distortion)
Lighting: Flat even lighting for maximum readability, subtle drop shadows for depth
Color Grading: Semantic colors - blue (#2563eb) building, green (#10b981) running, gray (#6b7280) stopped
Text Integration:
  - State labels: Bold sans-serif 24px (Building, Running, Stopped)
  - Commands: Monospace 18px (docker build, docker run, docker stop)
  - Visual hierarchy: States (largest) > Commands (medium)
Resolution: 2K (2048x1152px)

TEACHING GOAL: Docker containers move through lifecycle states
PROFICIENCY: A2 (Beginner - first exposure to containers)
VISUAL TYPE: Static diagram (workflow is linear and complete)
GOOGLE SEARCH GROUNDING: No (conceptual architecture)
```

---

## Studio Controls Reasoning

**Lighting**: Flat even for clarity
- **Pedagogical Function**: A2 needs zero visual distractions from core concept

**Camera**: Orthographic
- **Pedagogical Function**: Technical accuracy, no perspective distortion for state diagrams

**Color**: Semantic coding
- Blue = building (preparation)
- Green = running (active execution)
- Gray = stopped (inactive)
- **Pedagogical Function**: Color teaches state meaning, not arbitrary aesthetics

---

## Generation Output

**Filename**: `docker-container-lifecycle-states.png`
**Location**: `book-source/static/img/visuals/docker-container-lifecycle-states.png`
**Resolution**: 2048x1152px (2K)
**File Size**: 287 KB
**Alt Text**: "Docker container lifecycle diagram showing build, run, and stop states with command transitions between states"

---

## Multi-Turn Refinement Log

See: `history/visual-assets/generation-logs/chapter-07/visual-01-docker-container-lifecycle.log.md`
```

---

### 3. Generation Logs

**Purpose**: Document multi-turn refinement iterations (teaching Gemini through feedback)

**Location**: `history/visual-assets/generation-logs/{chapter-NN}/`

**Naming Convention**: `visual-{NN}-{slug}.log.md`

**Content Structure**:
```markdown
# Generation Log: Docker Container Lifecycle

**Visual ID**: visual-01
**Chapter**: 07
**Date**: 2025-11-21
**Iterations**: 3
**Final Status**: ✅ Production-ready

---

## Turn 1: Initial Generation

**Timestamp**: 2025-11-21 14:30:15

**Prompt**: [Full reasoning-activated prompt from prompts/ directory]

**Output**: `docker-container-lifecycle-states-v1.png`

**Evaluation**:
- ✅ Orthographic camera maintained (technical accuracy)
- ✅ Flat lighting appropriate for A2
- ✅ Semantic color coding clear
- ⚠️ State labels too small (18px, should be 24px for A2)
- ⚠️ Commands same size as states (hierarchy unclear)

**Decision**: Iterate with principle-based feedback

---

## Turn 2: Principle-Based Refinement

**Timestamp**: 2025-11-21 14:32:40

**Feedback to Gemini**:
```
The state labels ("Building", "Running", "Stopped") are too small (18px).
For A2 proficiency, primary concepts must be instantly readable without
effort. This aligns with our 'Proficiency-Appropriate Sizing' principle.

Additionally, commands and states are the same size (18px), creating
ambiguous hierarchy. Students need to distinguish "what" (states) from
"how" (commands).

REFINEMENT:
- Increase state labels to 24px bold (primary concept)
- Keep commands at 18px monospace (supporting detail)
- This creates hierarchy: States (24px bold) > Commands (18px mono)

PEDAGOGICAL REASONING: Visual sizing reflects information importance.
Students scan and immediately identify states as primary learning target.
```

**Output**: `docker-container-lifecycle-states-v2.png`

**Evaluation**:
- ✅ State labels now 24px bold (instantly readable)
- ✅ Commands 18px monospace (clear secondary hierarchy)
- ✅ Gemini applied principle BROADLY (also adjusted icon sizing)
- ✅ Reasoning transfer validated (generalized the principle)

**Decision**: Production-ready ✅

---

## Turn 3: Validation (Optional)

**Timestamp**: 2025-11-21 14:35:10

**Final Check**:
- ✅ Teaching goal clear: Lifecycle states concept
- ✅ Proficiency match: A2 instant readability
- ✅ Constitutional alignment: Zero unnecessary complexity
- ✅ Accessibility: WCAG AA contrast (4.8:1)

**Final Output**: `docker-container-lifecycle-states.png` (renamed from v2)

**Saved to**: `book-source/static/img/visuals/docker-container-lifecycle-states.png`

---

## Gemini Learning Outcomes

**Principle Learned**: Visual hierarchy = information importance

**Evidence of Transfer**:
- Adjusted icon sizing (not mentioned in feedback) to maintain hierarchy
- Applied bold weight to primary elements consistently
- Maintained monospace for all code elements (generalized pattern)

**Quality**: Reasoning activated ✅ (not just pattern matching)
```

---

### 4. Asset Metadata Registry

**Purpose**: Track all visual assets, dependencies, usage across chapters

**Location**: `history/visual-assets/metadata/asset-registry.json`

**Structure**:
```json
{
  "schema_version": "1.0.0",
  "last_updated": "2025-11-21T14:40:00Z",
  "total_assets": 47,
  "assets": [
    {
      "id": "visual-07-01",
      "filename": "docker-container-lifecycle-states.png",
      "chapter": 7,
      "lesson": null,
      "type": "static",
      "teaching_goal": "Docker containers move through lifecycle states",
      "proficiency": "A2",
      "resolution": "2K",
      "file_size_kb": 287,
      "grounded": false,
      "interactive": false,
      "text_in_image": true,
      "created_date": "2025-11-21",
      "skill_version": "image-generator v4.0.0",
      "prompt_location": "history/visual-assets/prompts/chapter-07/visual-01.prompt.md",
      "generation_log": "history/visual-assets/generation-logs/chapter-07/visual-01.log.md",
      "audit_report": "history/visual-assets/audits/chapters/chapter-07-visual-audit.md",
      "markdown_references": [
        "book-source/docs/part-02/chapter-07/lesson-01-docker-basics.md"
      ],
      "reused_in": [],
      "status": "production",
      "tags": ["docker", "lifecycle", "static", "A2"]
    },
    {
      "id": "visual-08-02",
      "filename": "kubernetes-architecture-interactive-tier1.png",
      "chapter": 8,
      "lesson": null,
      "type": "interactive",
      "teaching_goal": "Control Plane orchestrates Worker Nodes",
      "proficiency": "B1",
      "resolution": "2K",
      "file_size_kb": 412,
      "grounded": false,
      "interactive": true,
      "text_in_image": true,
      "tier_structure": {
        "tier_1": "Overview visible",
        "tier_2": "Tap-to-reveal component details",
        "tier_3": "Deep links to Lesson 24, Lesson 25"
      },
      "created_date": "2025-11-21",
      "skill_version": "image-generator v4.0.0",
      "prompt_location": "history/visual-assets/prompts/chapter-08/visual-02.prompt.md",
      "generation_log": "history/visual-assets/generation-logs/chapter-08/visual-02.log.md",
      "audit_report": "history/visual-assets/audits/chapters/chapter-08-visual-audit.md",
      "markdown_references": [
        "book-source/docs/part-03/chapter-08/lesson-03-kubernetes-intro.md"
      ],
      "reused_in": [
        "book-source/docs/part-03/chapter-09/lesson-02-orchestration.md"
      ],
      "status": "production",
      "tags": ["kubernetes", "architecture", "interactive", "B1", "tier-system"]
    }
  ]
}
```

---

## Workflow Integration

### Phase 1: Visual Planning (`visual-asset-workflow`)

**Outputs**:
1. **Audit report** → `history/visual-assets/audits/chapters/chapter-{NN}-visual-audit.md`
2. **Reasoning-activated prompts** → Embedded in lesson markdown as HTML comments
3. **Extracted prompts** → `history/visual-assets/prompts/chapter-{NN}/visual-{NN}.prompt.md`

**Actions**:
```bash
# Skill automatically creates:
1. Audit report with approve/reject decisions
2. Embeds prompts in markdown
3. Extracts prompts to prompts/ directory for archival
4. Updates asset registry metadata (pending generation)
```

---

### Phase 2: Image Generation (`image-generator`)

**Outputs**:
1. **Generation log** → `history/visual-assets/generation-logs/chapter-{NN}/visual-{NN}.log.md`
2. **Final images** → `book-source/static/img/visuals/{filename}.png`
3. **Updated markdown** → Image references added to lesson files

**Actions**:
```bash
# Skill automatically:
1. Reads prompts from markdown comments
2. Generates with multi-turn refinement
3. Logs all iterations to generation-logs/
4. Saves final image to static/img/visuals/
5. Updates markdown with image reference
6. Updates asset registry (status: production, file_size, etc.)
```

---

## Part-Level Strategic Planning

**Output**: `history/visual-assets/audits/parts/part-{NN}-visual-strategy.md`

**Content Structure**:
```markdown
# Part 3 Visual Strategy

**Chapters**: 7-12
**Proficiency Range**: A2 → B1
**Date**: 2025-11-21

---

## Strategic Themes

### Visual Progression
- **Chapters 7-8**: Static foundations (A2)
- **Chapters 9-10**: Text-in-image infographics (B1)
- **Chapters 11-12**: Interactive explorable (B1-B2)

### Avoid Redundancy
- **Container concepts**: ONE lifecycle diagram (Ch7), reuse with links
- **Architecture patterns**: ONE Kubernetes overview (Ch8), reference elsewhere
- **Network diagrams**: DIFFERENT aspects (Ch9: modes, Ch10: security)

---

## Chapter-by-Chapter Visual Allocation

### Chapter 7: Docker Basics (A2)
1. Container lifecycle (static, 3 states)
2. Image vs container comparison (static, 2 columns)

### Chapter 8: Docker Architecture (B1)
1. Docker architecture (interactive, 9 components, tier system)
2. [Reuse Ch7 lifecycle with link reference]

### Chapter 9: Container Networking (B1)
1. Network modes infographic (text-in-image, 4 modes, throughput sizing)
2. Port mapping diagram (static, clear labels)

[Continue for all chapters...]

---

## Cross-Chapter Visual Links

- **Ch7 lifecycle** → Referenced in Ch8, Ch10 (no duplication)
- **Ch8 architecture** → Deep link from Ch9 network diagram
- **Ch9 network modes** → Prerequisite visual for Ch11 security

---

## Constitutional Alignment

- ✅ Anti-Convergence: No redundant patterns across Part
- ✅ Progressive Complexity: A2 static → B1 text-in-image → B1-B2 interactive
- ✅ Coherent Structure: Visual themes build across chapters
```

---

## Migration Plan (Existing Assets)

### Step 1: Archive Legacy Reports

```bash
# Move old reports to archive/
mv history/visual-assets/AUTONOMOUS-CHAPTER-PROMPT.md \
   history/visual-assets/archive/

mv history/visual-assets/FINAL-CHAPTER-1-VISUAL-AUDIT.md \
   history/visual-assets/archive/

# ... all uppercase legacy files
```

---

### Step 2: Restructure Existing Reports

```bash
# Existing lesson reports → audits/lessons/
mv history/visual-assets/chapter-1-lesson-1-visual-assets-report.md \
   history/visual-assets/audits/lessons/chapter-01-lesson-01-audit.md

# Rename to standard convention (chapter-01 not chapter-1)
```

---

### Step 3: Create Missing Directories

```bash
mkdir -p history/visual-assets/audits/{parts,chapters,lessons}
mkdir -p history/visual-assets/generation-logs
mkdir -p history/visual-assets/prompts
mkdir -p history/visual-assets/metadata
mkdir -p history/visual-assets/archive
```

---

### Step 4: Initialize Asset Registry

```bash
# Create initial asset-registry.json with existing assets
# Scan book-source/static/img/visuals/ and create entries
```

---

## Benefits of Standardized Architecture

### 1. Traceability
- Every visual has audit trail (why approved/rejected)
- Multi-turn refinement logged (teaching Gemini reasoning)
- Prompts archived (reuse for similar visuals)

### 2. Reusability
- Asset registry shows where visuals used/reused
- Prompts can be adapted for new chapters
- Generation logs show what feedback worked

### 3. Quality Control
- Audit reports enforce pedagogical rigor
- Generation logs validate reasoning activation
- Metadata tracks grounding, accessibility, proficiency

### 4. Maintenance
- Easy to find visual artifacts by chapter/type
- Asset registry enables bulk updates (if Gemini 4 requires new prompts)
- Clear delineation between production and archived

### 5. Learning & Improvement
- Generation logs document successful multi-turn patterns
- Audit reports show common rejection reasons (learn what NOT to do)
- Metadata enables analysis (which visual types most effective per proficiency?)

---

## Skill Updates Required

### `visual-asset-workflow` v4.0.1 (Proposed)

**Add output step**:
```markdown
5. Create audit report at history/visual-assets/audits/chapters/
6. Extract prompts to history/visual-assets/prompts/chapter-{NN}/
7. Initialize asset registry entries (status: pending)
```

---

### `image-generator` v4.0.1 (Proposed)

**Add output step**:
```markdown
6. Create generation log at history/visual-assets/generation-logs/chapter-{NN}/
7. Update asset registry (status: production, file_size, final metadata)
```

---

## Implementation Checklist

- [ ] Create directory structure (`audits/`, `generation-logs/`, `prompts/`, `metadata/`, `archive/`)
- [ ] Archive legacy reports to `archive/`
- [ ] Rename existing reports to standard convention
- [ ] Initialize asset registry JSON
- [ ] Update `visual-asset-workflow` v4.0 → v4.0.1 (add output steps)
- [ ] Update `image-generator` v4.0 → v4.0.1 (add output steps)
- [ ] Document workflow in skills README
- [ ] Test with one chapter end-to-end
- [ ] Validate all artifacts created correctly
- [ ] Create ADR documenting architecture decision

---

## Future Enhancements

### Asset Registry Dashboard (Future)

Web-based dashboard showing:
- Visual coverage per chapter (X/Y visuals complete)
- Type distribution (static vs interactive vs text-in-image)
- Proficiency alignment (A2/B1/C2 distribution)
- Reuse analysis (which visuals referenced multiple times)
- Grounding coverage (% of factual content grounded)

### Automated Prompts Optimization (Future)

ML analysis of generation logs:
- Which prompt patterns → fewest iterations?
- Which feedback types → best reasoning transfer?
- Common refinement patterns → add to skill principles

---

## References

- **ADR 0017**: Visual Skills v4.0 Upgrade (Gemini 3 integration)
- **Constitution v6.0.1**: Principle 3 (Factual Accuracy), Principle 6 (Anti-Convergence)
- **CLAUDE.md v5.1.0**: Section VII (Anti-Convergence Checklist)

---

**Status**: Proposed for implementation
**Next Step**: Create directories, migrate existing reports, update skills to v4.0.1

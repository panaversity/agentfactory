# Visual Assets Repository

**Version**: 1.0.0 (Gemini 3 Era)
**Last Updated**: 2025-11-21
**Skills**: visual-asset-workflow v4.0, image-generator v4.0

---

## Directory Structure

```
visual-assets/
├── audits/                    # Visual opportunity analysis & decisions
│   ├── parts/                # Part-level strategic planning
│   ├── chapters/             # Chapter-level audit reports
│   └── lessons/              # Lesson-level granular audits
│
├── generation-logs/          # Multi-turn refinement iterations
│   └── chapter-{NN}/        # Organized by chapter
│
├── prompts/                  # Reasoning-activated Gemini 3 prompts
│   └── chapter-{NN}/        # Extracted from markdown, archived for reuse
│
├── metadata/                 # Asset tracking & registry
│   └── asset-registry.json  # Master index of all visual assets
│
└── archive/                  # Legacy reports (pre-v4.0 structure)
```

---

## Naming Conventions

### Audit Reports

- **Part-level**: `part-{NN}-visual-strategy.md` (e.g., `part-03-visual-strategy.md`)
- **Chapter-level**: `chapter-{NN}-visual-audit.md` (e.g., `chapter-07-visual-audit.md`)
- **Lesson-level**: `chapter-{NN}-lesson-{NN}-audit.md` (e.g., `chapter-07-lesson-01-audit.md`)

### Generation Logs

- **Format**: `visual-{NN}-{slug}.log.md`
- **Example**: `visual-01-docker-container-lifecycle.log.md`
- **Location**: `generation-logs/chapter-{NN}/`

### Prompts

- **Format**: `visual-{NN}-{slug}.prompt.md`
- **Example**: `visual-01-docker-container-lifecycle.prompt.md`
- **Location**: `prompts/chapter-{NN}/`

### Visual Assets (actual images)

- **Format**: `{concept}-{type}.png` (kebab-case)
- **Examples**:
  - `docker-container-lifecycle-states.png`
  - `kubernetes-architecture-interactive-tier1.png`
  - `developer-value-multiplication-scale.png`
- **Location**: `book-source/static/img/visuals/`

---

## Workflow

### Phase 1: Visual Planning (`visual-asset-workflow`)

**Input**: Chapter/lesson markdown content
**Process**: Analyze for visual opportunities, apply decision frameworks
**Outputs**:

1. **Audit report** → `audits/chapters/chapter-{NN}-visual-audit.md`
2. **Embedded prompts** → HTML comments in lesson markdown
3. **Extracted prompts** → `prompts/chapter-{NN}/visual-{NN}.prompt.md`
4. **Registry entries** → Updates `metadata/asset-registry.json` (status: pending)

**Invocation**:

```
Analyze Chapter 7 for visual opportunities using visual-asset-workflow
```

---

### Phase 2: Image Generation (`image-generator`)

**Input**: Reasoning-activated prompts from Phase 1
**Process**: Generate with Gemini 3 Pro Image, multi-turn refinement
**Outputs**:

1. **Generation log** → `generation-logs/chapter-{NN}/visual-{NN}.log.md`
2. **Final images** → `book-source/static/img/visuals/{filename}.png`
3. **Updated markdown** → Image references embedded
4. **Registry update** → Status changed to "production", metadata added

**Invocation**:

```
Generate approved visuals for Chapter 7 using image-generator
```

---

### Single-Phase Workflow (Combined)

**Invocation**:

```
Plan and generate all visual assets for Chapter 7
```

Both skills run sequentially: analysis → generation → all artifacts created

---

## Asset Registry

**Location**: `metadata/asset-registry.json`

**Purpose**: Master index tracking all visual assets across the book

**Schema**:

```json
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
    "apps/learn-app/docs/part-02/chapter-07/lesson-01-docker-basics.md"
  ],
  "reused_in": [],
  "status": "production",
  "tags": ["docker", "lifecycle", "static", "A2"]
}
```

---

## Migrated Content

### Archive Directory

Contains legacy reports from pre-v4.0 structure:

- `AUTONOMOUS-CHAPTER-PROMPT.md`
- `FINAL-CHAPTER-1-VISUAL-AUDIT.md`
- `IMAGE-GENERATION-LESSONS-LEARNED.md`
- `chapter-1-visual-assets-report.md` (original)
- `part-3-complete-audit-report.md` (original)
- ... and other historical reports

These files are preserved for reference but superseded by the new standardized structure.

### Reorganized Reports

**Migrated on**: 2025-11-21

**From** (flat structure):

```
history/visual-assets/
├── chapter-1-lesson-1-visual-assets-report.md
├── chapter-2-audit-report.md
├── part-3-complete-audit-report.md
└── ...
```

**To** (organized structure):

```
history/visual-assets/
├── audits/
│   ├── lessons/chapter-01-lesson-01-audit.md
│   ├── chapters/chapter-02-visual-audit.md
│   └── parts/part-03-visual-strategy.md
└── archive/
    └── [originals preserved]
```

---

## Benefits of This Structure

### 1. Traceability

- Every visual has complete audit trail (why approved/rejected)
- Multi-turn refinement logged (principle-based feedback)
- Prompts archived for reuse/adaptation

### 2. Reusability

- Asset registry shows usage across chapters
- Prompts can be adapted for similar visuals
- Generation logs document successful patterns

### 3. Quality Control

- Audit reports enforce pedagogical rigor (no decorative filler)
- Generation logs validate reasoning activation (5/5 scores)
- Metadata tracks grounding, accessibility, proficiency alignment

### 4. Maintenance

- Easy to find artifacts by chapter/type
- Asset registry enables bulk updates (e.g., Gemini 4 migration)
- Clear delineation between production and archived

### 5. Learning & Improvement

- Generation logs show successful multi-turn patterns
- Audit reports reveal common rejection reasons
- Metadata enables analysis (which visual types most effective per proficiency?)

---

## Future Enhancements

### Asset Registry Dashboard

- Visual coverage per chapter (X/Y complete)
- Type distribution charts (static vs interactive)
- Proficiency alignment analysis
- Reuse patterns identification

### Automated Prompts Optimization

- ML analysis of generation logs
- Pattern extraction (fewest iterations)
- Feedback effectiveness scoring
- Add successful patterns to skill principles

---

## References

- **Architecture Document**: `docs/visual-assets-storage-architecture.md`
- **ADR 0017**: Visual Skills Gemini 3 Upgrade
- **Skills Documentation**: `.claude/skills/visual-asset-workflow/`, `.claude/skills/image-generator/`
- **Constitution**: `.specify/memory/constitution.md` (Principle 3: Factual Accuracy, Principle 6: Anti-Convergence)

---

## Quick Start

**For new chapters**:

```bash
# Two-phase workflow (recommended)
1. "Analyze Chapter X for visual opportunities"
2. [Review audit report in audits/chapters/]
3. "Generate approved visuals for Chapter X"

# Single-phase workflow (faster)
"Plan and generate all visuals for Chapter X"
```

**For existing chapters**:

```bash
"Audit and regenerate visuals for Chapter X with Gemini 3 quality"
```

**For part-level planning**:

```bash
"Plan visual strategy for Part 3 (Chapters 7-12)"
```

---

**Version**: 1.0.0
**Status**: Active
**Last Migration**: 2025-11-21 (flat → organized structure)

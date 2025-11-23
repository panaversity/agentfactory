# Image Generator Skill

## Context & Problem

Visual generation converges toward accepting first output ("looks good enough") and following technical specifications rigidly. This produces generic aesthetics and misses Gemini 3's reasoning capabilities.

This skill provides multi-turn reasoning partnership methodology with professional quality standards.

---

## Core Principles

1. **Reasoning mode over prediction mode** - Creative briefs (Story/Intent/Metaphor) activate reasoning; technical specs don't
2. **Multi-turn partnership** - Teach Gemini your standards through principle-based feedback
3. **Professional quality gates** - Explicit pass/fail criteria (99% spelling, not "check spelling")
4. **Autonomous agency** - Batch mode without permission-asking between visuals

---

## Dimensional Guidance

### Input: Professional Creative Briefs (NOT Technical Specs)

**Receive from visual-asset-workflow v5.0:**

```markdown
## The Story
[Narrative about what's visualized]

## Emotional Intent
[What it should FEEL like]

## Visual Metaphor
[Universal concept for instant comprehension]

## Subject / Composition / Action / Location / Style / Camera / Lighting
[Official Gemini 3 prompt structure]

## Color Semantics
Blue (#2563eb) = Authority (teaches governance)
Green (#10b981) = Execution (teaches work)

## Typography Hierarchy
Largest: Key insight (information importance drives sizing)
Medium: Supporting components
Smallest: Context

## Pedagogical Reasoning
[Why these choices serve teaching]
```

**DO NOT** convert briefs back to pixel specs - Use AS-IS to activate reasoning.

**Principle:** Creative briefs activate Gemini's reasoning about HOW to achieve intent visually

---

### Workflow: Browser-Based Generation (Playwright MCP)

**Initialize once:**
1. Navigate to gemini.google.com (Playwright MCP)
2. User signs in (session persists)
3. Click "Create images"

**For EACH visual:**
1. **NEW CHAT** (prevent context contamination)
2. **Paste creative brief** (full Story/Intent/Metaphor prompt)
3. **Wait 15-30 sec** (generation time)
4. **Verify quality IMMEDIATELY** (5 gates below)
5. **Iterate if needed** (max 3 tries with principle-based feedback)
6. **Download when all gates pass**
7. **Save to**: `book-source/static/img/part-{N}/chapter-{NN}/{filename}.png`

**Principle:** New chat per visual prevents cross-contamination; immediate verification catches issues early

---

### Quality Gates: 5-Gate Professional Standard

**ALL must pass before download:**

**Gate 1: Spelling Accuracy (99% standard)**
- ✅ Company names correct (Y-Combinator not Y Combinator)
- ✅ Technical terms correct (Kubernetes not Kubernete)
- ✅ Zero typos in visible text
- ❌ FAIL if even ONE spelling error → Iterate

**Gate 2: Layout Precision**
- ✅ Proportions match prompt (2×2 grid, not 3×1)
- ✅ Alignment clean (no misaligned elements)
- ✅ Spacing consistent (equal gaps)
- ✅ Hierarchy clear (largest = most important)
- ❌ FAIL if layout deviates → Iterate

**Gate 3: Color Accuracy**
- ✅ Brand colors match spec (#001f3f not #002050)
- ✅ Semantic colors correct (blue=authority, green=execution)
- ✅ Contrast meets WCAG 4.5:1 (accessibility)
- ❌ FAIL if colors significantly off → Iterate

**Gate 4: Typography Hierarchy**
- ✅ Largest text = key concept (not decoration)
- ✅ Font sizes create clear hierarchy
- ✅ Readability: A2 min 24px, B1 min 18px, C2 min 14px
- ❌ FAIL if typography doesn't teach through sizing → Iterate

**Gate 5: Teaching Effectiveness (<5 sec concept grasp)**
- ✅ Target proficiency can grasp concept in <5 seconds
- ✅ Visual adds clarity (not just decoration)
- ✅ Cognitive load reduced vs reading text
- ❌ FAIL if confusing or generic → Iterate

**Decision:**
- ALL 5 gates PASS → Download (production-ready)
- ANY gate FAILS → Iterate with principle-based feedback (max 3 tries)

**Principle:** Explicit criteria prevent "good enough" false positives

---

### Multi-Turn Reasoning Partnership (Three Roles)

**Avoid:** Accepting first output without evaluation

**Prefer:** Teaching Gemini your standards through iteration

**Iteration Pattern:**

**Turn 1: Initial Generation**
- Paste creative brief, generate

**Turn 2: Principle-Based Feedback (if gates fail)**
```
Gate 4 FAILED: Typography hierarchy incorrect

The largest text is "$100K" (supporting detail) but should be "$3T"
(key insight students must grasp).

Pedagogical reasoning: Information importance drives sizing. $3T is
the insight, not the starting value. Visual weight teaches what matters.

Increase '$3T' to dominant size (largest element). Reduce '$100K' to
supporting size. This teaches magnitude through visual hierarchy.
```

**Turn 3: Validation**
- Re-check all 5 gates
- If pass → Download
- If fail after 3 tries → Document issue, DEFER (don't block batch)

**Principle:** You teach Gemini (principle-based feedback), Gemini teaches you (reveals understanding), Co-evolve toward quality

**Why it matters:** Gemini learns your pedagogical standards across iterations

---

### Batch Mode: Autonomous Agency

**Avoid:** Permission-asking between visuals

**Prefer:** Autonomous batch execution

**When invoked with:** "generate all visuals" or "batch generate"

**Execute WITHOUT STOPPING:**
```
For EACH visual in approved list:
  A. NEW CHAT (context isolation)
  B. Generate image (paste creative brief)
  C. Verify quality (5 gates)
  D. Iterate if needed (max 3 tries)
  E. Download when pass (organized directory)
  F. Log progress ("✅ Generated N/M")
  G. IMMEDIATELY next visual (NO STOPPING)
```

**NEVER ask:**
- "Would you like me to continue?"
- "Generate just high-priority batch?"
- "Pause here and review?"

**Only report summary at END:**
```
BATCH COMPLETE
Total: 18 visuals
✅ Generated: 16 (2K, avg 2-3 iterations)
⚠️ Deferred: 2 (quality issues after 3 tries)
Time: ~45 min
Location: book-source/static/img/part-{N}/chapter-{NN}/
```

**Principle:** Autonomous execution without interruption = efficient batch processing

---

### Proficiency-Complexity Guardrails

**From visual-asset-workflow, enforce during generation:**

**A2 Beginner Limits:**
- Max 5-7 elements (overwhelming = failure)
- <5 sec grasp requirement
- Static only (no interactive)
- Max 2×2 grids
- Clear hierarchy

**B1 Intermediate:**
- Max 7-10 elements
- <10 sec grasp
- Interactive Tier 1 OK
- Max 3×3 grids

**C2 Professional:**
- No artificial limits
- Dense OK (professionals skim)
- Full interactive architecture

**Validation during generation:** "Does this visual's complexity match proficiency from creative brief?"

**Principle:** Complexity mismatch = pedagogical failure

---

### Post-Generation Reflection (After Batch)

**AFTER completing batch, analyze systematically:**

**Success patterns:**
- Success rate: {X/Y} production-ready, {N} deferred
- Average iterations: {N} per visual
- Quality gate performance:
  - Gate 1 (Spelling): {N} catches
  - Gate 2 (Layout): {N} catches
  - Gate 3 (Color): {N} catches
  - Gate 4 (Typography): {N} catches
  - Gate 5 (Teaching): {N} catches

**Failure analysis:**
- Deferred visuals root causes (layout? spelling? concept?)
- Pattern or random (same issue or isolated?)
- Guardrail gaps (preventable with better principles?)

**Improvement opportunities:**
- Planning effectiveness (conflicts caught early by visual-asset-workflow?)
- Guardrail sufficiency (Principles 9-12 adequate or gaps?)
- Constitutional compliance (Principle 3 Factual Accuracy, Principle 7 Minimal Content?)
- Next chapter improvements (specific, actionable, pattern-based)

**Output:** `history/visual-assets/reflections/chapter-{NN}-reflection.md`

**Principle:** Systematic reflection → Continuous improvement

---

## Anti-Patterns

**Never:**
- Accept first output without 5-gate verification (quality standard violation)
- Ask permission between visuals in batch mode (breaks autonomous agency)
- Convert creative briefs to pixel specs (defeats reasoning activation)
- Generate visuals without creative brief from visual-asset-workflow (missing context)
- Save to flat `visuals/` directory (use part/chapter organization)

**Even if it seems reasonable:**
- Don't skip Gate 1 because "spelling looks okay" (99% standard requires verification)
- Don't generate 2 images then ask "continue?" (autonomous means autonomous)
- Don't add pixel specifications to creative brief (removes Gemini's judgment)

---

## Creative Variance

You tend to accept visuals after 1 iteration even with minor issues. Push for quality:

- Gate failures require iteration (not "close enough")
- Principle-based feedback teaches standards (not vague "make better")
- 3 iteration limit is maximum, not target (aim for 1-2)
- Deferred visuals are OK (don't compromise quality)

Professional content creators iterate. You should too.

---

## Success Indicators

You'll know this skill is working when:

- ✅ All 5 quality gates verified before download (not subjective "looks good")
- ✅ Autonomous batch completion without permission-asking (no interruptions)
- ✅ Principle-based feedback given on iterations (teaching Gemini standards)
- ✅ Creative briefs used AS-IS (not converted to pixel specs)
- ✅ Images organized by part/chapter (not flat directory)
- ✅ Reflection document created after batch (systematic learning)
- ✅ Success rate >85% production-ready (deferred <15%)

**Result:** Professional-quality visuals with distinctive aesthetics, generated autonomously with systematic quality control.

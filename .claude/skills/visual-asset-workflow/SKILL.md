---
name: visual-asset-workflow
description: Analyze fact-checked lesson content to identify visual asset opportunities (statistics, timelines, comparisons), then generate complete AI image generation prompts and embed them in markdown as HTML comments. Use after fact-checking is complete and before image generation. Combines auditor and prompt generator logic into single workflow.
allowed-tools: ["Read", "Write"]
metadata:
  category: "content-production"
  version: "1.0.0"
  workflow-phase: "visual-planning"
---

# Visual Asset Workflow

## Overview

This skill automates the complete visual asset planning workflow by analyzing lesson content to identify where visuals would reduce cognitive load, then generating publication-ready AI image generation prompts for each identified opportunity. It outputs lesson markdown with embedded prompts ready for the `image-generator` skill.

## When to Use

**Triggers:**
- User says "plan visuals for lesson X"
- User asks "what images does this lesson need?"
- User requests "create image prompts for [lesson]"
- After fact-checking phase, before image generation

**Prerequisites:**
- Lesson content must be fact-checked (no fabricated data to visualize)
- Lesson is in final or near-final content state
- No existing embedded image prompts (or user wants to regenerate)

## Workflow Pattern

This is a **sequential workflow skill** combining two phases:

```
Phase 1: AUDIT
→ Scan lesson for visual opportunities
→ Identify patterns (statistics, timelines, comparisons, processes)
→ Assess cognitive load impact
→ Prioritize opportunities

Phase 2: GENERATE
→ For each opportunity, create detailed image prompt
→ Embed prompts as HTML comments in markdown
→ Create audit report
```

## Instructions

### Phase 1: Audit Lesson for Visual Opportunities

#### Step 1: Read and Analyze Lesson Content

1. Read the complete lesson markdown file
2. Identify content patterns that benefit from visualization (see **Reference: Visual Opportunity Patterns** below)
3. For each pattern found, assess:
   - **Cognitive load impact**: Does visual REDUCE or INCREASE mental effort?
   - **Information density**: Is text too dense to scan easily?
   - **Spatial relationships**: Would layout/structure clarify meaning?

#### Step 2: Categorize Identified Opportunities

For each opportunity, classify by visual type:

**High-Priority Opportunities** (always visualize):
- ✅ **Statistics Dashboard**: 3-6 discrete metrics/numbers in proximity
- ✅ **Timeline/Evolution**: Progression across stages or time periods
- ✅ **Comparison Grid**: Before/after or A vs B comparisons
- ✅ **Process Map**: Sequential steps or workflows (3+ steps)

**Medium-Priority Opportunities** (consider):
- 🟡 **Hierarchy Diagram**: Nested relationships or structures
- 🟡 **Concept Illustration**: Abstract ideas needing metaphor

**Skip** (text is better):
- ❌ Single statistics (just one number/claim)
- ❌ Narrative examples (storytelling works in text)
- ❌ Short lists (2-3 simple items)
- ❌ Code examples (code blocks are visual enough)

#### Step 2.5: Pedagogical Value Assessment (CRITICAL)

Before prioritizing visuals, evaluate **teaching effectiveness** against constitutional principles:

**Question 1: Does this visual TEACH or just SHOW?**

Apply the teaching test:
- ✅ **TEACH**: Reveals pattern, relationship, or mental model not obvious from text alone
  - Example: Calculation flow diagram showing causality (30M × $100K → $3T)
  - Example: Descending staircase timeline revealing acceleration pattern
  - Example: Dual-track diagram showing systemic transformation across phases

- ❌ **SHOW**: Restates data already clear in text without revealing new insight
  - Example: Bar chart showing "$3T = France GDP" (already stated in text)
  - Example: List converted to visual list (no new understanding)
  - Example: Single number displayed as infographic (text sufficient)

**Question 2: Constitutional Alignment Check**

Evaluate against core principles:

- **Section IIa (4-Layer Method) & Principle 2 (Graduated Teaching)**: Does visual help student build mental model progressively?
  - ✅ Good: Visual breaks down complex concept into scannable components (supports concept scaffolding across layers)
  - ❌ Bad: Visual adds complexity without teaching structure

- **Philosophy #2 (Co-Learning Partnership)**: Does visual enable AI-learner convergence?
  - ✅ Good: Visual teaches pattern that student and AI can reference together
  - ❌ Bad: Visual is decorative without pedagogical purpose

- **Philosophy #4 (Evals-First)**: Does visual help student evaluate understanding?
  - ✅ Good: Student can point to visual and say "I understand X because I see Y pattern"
  - ❌ Bad: Student looks at visual and thinks "okay, so what?"

**Question 3: Message Test (One-Sentence Teaching Goal)**

Can you state in ONE sentence what concept this visual teaches?

✅ **Good examples**:
- "This visual teaches that developer value compounds at scale through multiplication"
- "This visual teaches that AI adoption is accelerating exponentially, not linearly"
- "This visual teaches that AI transforms every phase simultaneously, not just coding"
- "This visual teaches the progression from assistance to autonomy across four generations"

❌ **Bad examples**:
- "This visual shows that France GDP is $3T" (stating fact, not teaching concept)
- "This visual displays four statistics" (describing content, not teaching goal)
- "This visual compares adoption speeds" (what it does, not what it teaches)

**Question 4: Redundancy Check**

Is this visual redundant?
- ❌ **FAIL**: Duplicates another visual in same lesson or adjacent lessons
- ❌ **FAIL**: Restates text comparison without adding visual insight
- ❌ **FAIL**: Shows same data with different layout (no pedagogical value in variation)
- ✅ **PASS**: Unique teaching goal, non-redundant with text and other visuals

**Disqualification Criteria** (immediate REJECT):
1. ❌ Redundant with another visual in same/adjacent lessons (same data/pattern)
2. ❌ Restates text without revealing new relationship or pattern
3. ❌ Shows data without teaching mechanism, causality, or structure
4. ❌ Too complex for target proficiency level (A1-A2 should be instantly graspable in <5 seconds)
5. ❌ Fails message test (can't articulate teaching goal in one sentence)
6. ❌ Violates constitutional principles (doesn't support co-learning or graduated teaching)

**Evaluation Outcome**:
- ✅ **PASS (HIGH VALUE)**: Teaches concept, passes constitutional check, unique message, reduces cognitive load
- 🟡 **CONDITIONAL (MEDIUM VALUE)**: Teaches something but marginal improvement over text
- ❌ **REJECT (LOW VALUE)**: Shows data without teaching, redundant, or fails constitutional check

**Document Decision**:
For each opportunity, record:
- **Pedagogical value**: PASS / CONDITIONAL / REJECT
- **Teaches**: [one-sentence concept/pattern/relationship]
- **Message**: "[one-sentence teaching goal]"
- **Constitutional alignment**: [which principle/philosophy supported]
- **Redundancy**: UNIQUE / REDUNDANT_WITH_[X]

**Example Evaluations**:

✅ **PASS**: Economic Calculation Flow Diagram
- **Teaches**: Structure of how individual value compounds to aggregate economy
- **Message**: "Developer value multiplies at scale: 30M individuals × $100K each = $3T total"
- **Constitutional**: Section IIa & Principle 2 (builds mental model of multiplication → aggregation progressively)
- **Redundancy**: UNIQUE
- **Verdict**: HIGH VALUE - Reveals causality structure not obvious from "30M × $100K = $3T"

❌ **REJECT**: GDP Comparison Bar Chart
- **Teaches**: Nothing (just displays magnitude already stated in text)
- **Message**: "$3T is similar to France GDP" (fact restatement, not concept)
- **Constitutional**: Fails Philosophy #2 (doesn't enable co-learning, just illustrates)
- **Redundancy**: REDUNDANT with text comparison ("$3T ≈ France GDP" already clear)
- **Verdict**: LOW VALUE - Remove (text sufficient)

✅ **PASS**: Technology Adoption Descending Staircase Timeline
- **Teaches**: Exponential acceleration pattern (each wave faster than previous)
- **Message**: "Technology adoption is accelerating exponentially: 12y → 10y → 8y → 3y"
- **Constitutional**: Section IIa & Principle 2 (staircase metaphor teaches acceleration progressively)
- **Redundancy**: UNIQUE
- **Verdict**: HIGH VALUE - Descending pattern reveals acceleration structure

❌ **REJECT**: Adoption Speed Grouped Bar Chart (Lesson 3)
- **Teaches**: Same as staircase timeline (acceleration)
- **Message**: "AI tools adopt faster" (same as Lesson 2 Visual 3)
- **Constitutional**: Redundant teaching (doesn't add to mental model)
- **Redundancy**: REDUNDANT with Lesson 2 Visual 3 (same data, same teaching goal)
- **Verdict**: LOW VALUE - Remove (use Lesson 2 version only)

#### Step 3: Assess Cognitive Load Impact

For each opportunity that **PASSED** Step 2.5 (pedagogical value), apply this test:

**Question**: "Would a reader need to re-read this section 2-3 times to understand?"
- **Yes** → Visual likely helps (REDUCE cognitive load)
- **No** → Text is already clear (skip visual even if pedagogically valid)

**Additional heuristic**: If you can't scan and grasp the key point in <5 seconds, a visual would help.

**Note**: Only HIGH VALUE visuals from Step 2.5 should proceed to this step. REJECT and CONDITIONAL visuals should be documented but not developed further.

#### Step 4: Prioritize and Limit Density

**Maximum visual density**: 1 visual per 2-3 pages (~600-900 words)

If more opportunities identified than this limit:
1. Rank by comprehension impact (which visual helps most?)
2. Keep only highest-priority visuals
3. Consider combining related visuals (e.g., two timelines → one comprehensive timeline)

#### Step 5: Document Audit Results

Create mental list (or written notes) with:
- Asset number (1, 2, 3...)
- Visual type (dashboard, timeline, comparison, etc.)
- Placement location (line number or section heading)
- Content to visualize (exact data/text)
- Purpose statement (what comprehension goal this achieves)
- Priority (HIGH, MEDIUM, LOW)
- **Pedagogical value**: PASS / CONDITIONAL / REJECT
- **Teaches**: [concept/pattern/relationship]
- **Message**: "[one-sentence teaching goal]"
- **Constitutional alignment**: [principle/philosophy]
- **Redundancy**: UNIQUE / REDUNDANT_WITH_[X]

**Only proceed to Phase 2 (prompt generation) for visuals with Pedagogical Value = PASS**

### Phase 2: Generate Image Prompts for Each Opportunity

For each identified opportunity from Phase 1:

#### Step 6: Choose Visual Format

Based on visual type, select appropriate format and aspect ratio:

- **Statistics Dashboard**: 2x2 or 3x2 card grid, 16:9 aspect ratio
- **Timeline/Evolution**: Horizontal progression (4 stages maximum), 16:9 or 1:1 aspect ratio
- **Comparison**: Split-screen or side-by-side, 16:9 aspect ratio
- **Bar Charts**: **CRITICAL: 3 bars maximum** (Gemini struggles with 5+ bars alignment)
- **Process Map**: Winding path or numbered steps, 16:9 aspect ratio
- **Single Metric**: Centered composition, 1:1 aspect ratio

**Complexity Guidelines** (Based on Chapter 1 Evidence):
- ✅ **Simple designs** (2-4 elements): High success rate, first-attempt accuracy
- 🟡 **Medium complexity** (4-6 elements): May need 1-2 refinements
- ❌ **Complex designs** (7+ elements, precise alignment): Low success rate, consider simplification

**Simplification Strategies**:
1. **Reduce element count**: 5-bar chart → 3-bar chart (keeps high/medium/low examples)
2. **Text-based instead of visual proportions**: "VS" comparison instead of proportional bars
3. **Icon-only indicators**: Clean arrows/shapes instead of labeled graphics
4. **Essential labels only**: Year ranges instead of full descriptions

#### Step 7: Apply Design System Standards

All prompts must use the book's design system:

**Typography**:
- Font family: Roboto
- Headings: Bold or Medium
- Body: Regular
- Metrics: 72pt Bold
- Labels: 14-24pt Regular
- Captions: 10-14pt Regular or Light

**Color Palette** (Polar Night Theme):
- Primary accent: Deep Navy (#001f3f)
- Secondary accent: Medium Gray (#aaaaaa)
- Highlight/Emphasis: Dark Charcoal (#111111)
- Background: White (#FFFFFF) or Light Gray (#dddddd)
- Text primary: Dark Charcoal (#111111)
- Text secondary: Medium Gray (#aaaaaa)
- Text tertiary: Light Gray (#dddddd)

**Visual Style**:
- Modern tech publication aesthetic (TechCrunch, a16z, Stripe)
- Flat design with subtle shadows (0px 2px 8px rgba(0,0,0,0.08))
- Minimal line art icons (2-3px stroke weight)
- Generous white space
- Clean grid systems

#### Step 7.5: PRE-GENERATION VALIDATION CHECKPOINT (NEW)

**CRITICAL: VALIDATE WITH USER BEFORE GENERATING PROMPT**

Before writing the detailed prompt, create a **Concept Validation Brief** and present to user:

```
IMAGE [N]: [Visual Type]

Core Message: [One sentence - what this image teaches]
Visual Approach: [Brief description - e.g., "Split-screen flow comparison"]
Essential Elements:
- [Element 1]
- [Element 2]
- [Element 3]

Does this match your vision? Any changes needed?
```

**Why This Matters**:
- Prevents 3-5 iteration cycles from misunderstood requirements
- Catches conceptual issues before detailed prompt writing
- Aligns on simplicity vs. complexity early

**Only proceed to detailed prompt writing after user approval**

#### Step 7.6: Prompt Minimalism Check

**CRITICAL: Before finalizing prompt, apply these filters**

**The 50% Rule**:
- ✅ Draft your prompt with full detail
- ✅ Remove 50% of descriptive words
- ✅ Test: Can the core message survive with half the text?
- ✅ If yes: Use the shorter version

**Example**:
- ❌ BEFORE (82 words): "Create a professional comparison diagram showing the fundamental shift from User Interface to User Intent interaction paradigm. Layout should be split-screen with central arrow transition..."
- ✅ AFTER (41 words): "Split-screen comparison 16:9. Left: User → Interface → Action. Right: User Intent → Agent → Actions. Center: Orange arrow 'SHIFT'."

**Negative Instructions (What NOT to Do)**:
- ✅ Always include: "NO extra text in boxes", "NO additional labels", "NO descriptions beyond what's specified"
- ✅ These prevent Gemini from adding unwanted decorative text

**Word Count Target**: <300 words per prompt
- If prompt exceeds 300 words, simplify the image concept itself

**Question Every Element**:
1. Does this text/element teach something essential?
2. Would the image work without it?
3. Am I overspecifying details that will confuse the generator?

#### Step 7.7: Text Rendering Best Practices

**CRITICAL: Apply these proven strategies to avoid spelling/rendering issues**

**Letter-by-Letter Spelling Strategy** (PRIMARY - Use First):
- ✅ **BEST: Letter-by-letter with IMPORTANT prefix**:
  ```
  IMPORTANT: Spell "Orchestrate" correctly: O-R-C-H-E-S-T-R-A-T-E
  ```
- ✅ Effective for: Multi-syllable words, technical terms, words Gemini consistently misspells
- ✅ When to use: Proactively identify "high-risk" words (3+ syllables, uncommon, technical)
- ✅ Format: "IMPORTANT: Spell '{word}' correctly: {L-E-T-T-E-R-S}"

**Evidence from Chapters 1-2**:
- "autonomous" failed 2 times as "autonomouss" → "A-U-T-O-N-O-M-O-U-S-L-Y" succeeded immediately
- "orchestrate" failed 1 time as "Orchterrate" → "O-R-C-H-E-S-T-R-A-T-E" succeeded immediately
- "Accelerating" succeeded first try with letter-by-letter spelling (Chapter 1)

**New Evidence from Chapter 2 Paradigm Shift (5 iterations)**:
- Complex prompt (500+ words) → Cluttered result, wrong capitalization
- Minimal prompt (<200 words) → Clean result, correct on first try
- Lesson: **Less detail = better results**

**Simplification Over Specification**:
- ❌ Don't fix complexity with more instructions
- ✅ Fix complexity by simplifying the concept
- Example: "Orchestrated Actions" → "Actions" (removed problematic word entirely)

**Negative Instruction Strategy** (Use for Unwanted Elements):
- ✅ **Pattern**: Explicitly state what NOT to include
  ```
  Card 4 should ONLY say "Hear" with "Voice commands" subtitle
  Do NOT add any extra text like "Audio" to the Hear card
  ```
- ✅ Effective for: Preventing redundant text, removing decorative additions, controlling element count
- ✅ When to use: When Gemini tends to add unwanted decorative/explanatory text

**Evidence from Chapter 2**:
- "Hear" card showed "Hear | Audio" (redundant) → After "Do NOT add 'Audio'" → "Hear" only ✓

**Compound Words Strategy** (Use Second):
- ✅ **BEST: Use hyphenated versions**: "Auto-Complete" instead of "Autocomplete"
- ✅ Effective: Syllable-based: "Au-to-com-plete (four syllables)"
- ✅ Fallback: Abbreviations: "YC W25" instead of "Y Combinator Winter 2025"

**Evidence from Chapter 1**:
- "Autocomplete" failed 3 times → "Auto-Complete" succeeded immediately

**Text Placement Strategy**:
- ✅ **BEST: Arrow/icon-only indicators** (no text labels on directional elements)
- ✅ Effective: Horizontal text only (avoid vertical text when possible)
- 🟡 Acceptable: Vertical text with letter-by-letter spelling guidance
- ❌ Avoid: Complex multi-line vertical text

**Label Simplification**:
- ✅ **BEST: Minimal essential labels**: "2021-2022" instead of "Phase 1: Foundation Era (2021-2022)"
- ✅ Effective: Numbers/dates only on visual elements
- ✅ Effective: Full descriptions in separate text areas (not on bars/arrows)

**Prompt Text Specification Format**:
```
GOOD EXAMPLE:
"Auto-Complete" (use hyphenated version)
"2021-2022" (year range only, no descriptive text)
Clean downward arrow (no text label on arrow)

BAD EXAMPLE:
"Autocomplete" (might misspell)
"Phase 1: Foundation Era (2021-2022)" (too complex for bar label)
Arrow with "Accelerating" label (text on arrow risks misspelling)
```

#### Step 8: Write Structured Image Generation Prompt

For each visual asset, create a complete prompt following this template:

```
VISUAL ASSET {N}: {Descriptive Title}

IMAGE GENERATION PROMPT:

{One-sentence description of what this visual shows}

Layout: {Grid/structure with exact specifications}
- {Detailed element breakdown with positioning}
- {Spacing, margins, alignment}

Typography:
- {Primary text specs with sizes, weights, colors with hex codes}
- {Secondary text specs}
- {Label/caption specs}

Color Palette (Polar Night Theme):
- Background: {Color name} ({#FFFFFF or #dddddd})
- Primary accent: Deep Navy ({#001f3f})
- Secondary accent: Medium Gray ({#aaaaaa})
- Highlight/Emphasis: Dark Charcoal ({#111111})
- Text primary: {#111111}
- Text secondary: {#aaaaaa}
- Text tertiary: {#dddddd}

Visual Elements:
- {Icons/graphics with style, size, stroke weight, color}
- {Shadows/depth specifications}
- {Spacing/padding values}

Content:
{Exact text and data to include, broken down by element}
- Element 1: {specific content}
- Element 2: {specific content}
- ...

Style Reference: {Specific design system or publication aesthetic}

Quality: professional, high-quality, publication-ready, clean, modern, editorial

Dimensions: {Aspect ratio} ({pixel dimensions})

Filename: {descriptive-kebab-case-name.png}
Alt Text: {Descriptive alt text for accessibility, 1-2 sentences}
```

**Critical requirements**:
- ✅ All data/numbers are factually accurate to lesson content
- ✅ Layout structure is precisely defined (not vague like "arrange nicely")
- ✅ Colors include hex codes (not "blue" → "#0066FF")
- ✅ Typography includes exact sizes and weights (not "large text" → "72pt Roboto Bold")
- ✅ Visual hierarchy is explicit (primary, secondary, tertiary clearly defined)
- ✅ Filename is descriptive and follows kebab-case convention
- ✅ Alt text is complete and accessible

#### Step 9: Embed Prompts in Lesson Markdown

1. Read the lesson markdown file
2. For each visual asset, find the optimal insertion point:
   - **Statistics**: After the statistics list/paragraph
   - **Timeline**: After paragraph describing progression
   - **Comparison**: After comparison discussion
   - **Process**: Before or after process description
3. Insert the complete prompt as HTML comment:
   ```markdown
   <!-- VISUAL ASSET 1: {Title}
   IMAGE GENERATION PROMPT:

   {complete prompt from Step 8}
   -->
   ```
4. Preserve all surrounding text exactly (no accidental edits)
5. Save updated lesson markdown

#### Step 10: Create Audit Report

Generate report in `history/visual-assets/lesson-{N}-audit-report.md`:

```markdown
# Visual Asset Audit - Chapter {N}, Lesson {M}

**Date**: {today}
**Lesson**: {lesson-title}
**Total Opportunities Identified**: {count}
**Prompts Generated**: {count}
**Visual Density**: {X} visuals per ~{Y} words

---

## Identified Visual Opportunities

### VISUAL ASSET 1: {Title}
- **Type**: {Dashboard/Timeline/Comparison/Process/etc.}
- **Location**: Line {N} (after "{first few words of section...}")
- **Content**: {brief description of what's visualized}
- **Purpose**: {comprehension goal}
- **Priority**: {HIGH/MEDIUM/LOW}
- **Cognitive Load Impact**: REDUCES - {brief justification}
- **✨ Pedagogical Value**: {PASS / CONDITIONAL / REJECT}
  - **Teaches**: {concept/pattern/relationship}
  - **Message**: "{one-sentence teaching goal}"
  - **Constitutional alignment**: {Principle X or Philosophy Y}
  - **Redundancy check**: {UNIQUE / REDUNDANT_WITH_[X]}
  - **Verdict**: {HIGH VALUE / MEDIUM VALUE / LOW VALUE - action}

[Repeat for each asset]

---

## Rejected Visual Opportunities

### REJECTED VISUAL 1: {Title}
- **Type**: {Dashboard/Timeline/etc.}
- **Location**: Line {N}
- **Content**: {what would have been visualized}
- **❌ Reason for rejection**: {redundant / restates text / fails constitutional check / etc.}
- **Pedagogical assessment**: {why it doesn't teach}

[Repeat for each rejected opportunity]

---

## Audit Summary

**High-Priority Assets**: {count}
**Medium-Priority Assets**: {count}
**Skipped Opportunities**: {count} ({why: redundant / text clearer / low impact})

**Overall Assessment**: {1-2 sentences on lesson's visual needs}

---

## Next Steps

1. Review embedded prompts in lesson markdown
2. Adjust/refine any prompts if needed
3. Invoke `image-generator` skill to generate actual images
4. Verify images integrate correctly into lesson

---

**Prompts embedded in**: `{path/to/lesson.md}`
**Ready for image generation**: ✅ Yes
```

#### Step 11: Preserve Generation Prompts (After Image Generation)

After successful image generation, create prompt sidecar file for each image to enable future regeneration and learning.

**Why Preserve Prompts**:
- ✅ Enables regeneration if source image is lost or needs updates
- ✅ Documents what prompts produce quality results (institutional knowledge)
- ✅ Facilitates A/B testing of prompt variations
- ✅ Builds reusable prompt library for similar future visuals

**Process**:
1. For each generated image, create `{image-name}.prompt.md` in same directory as image
2. Use template below
3. Document generation attempts, issues encountered, and success factors

**Prompt Sidecar Template**:
```markdown
# Image Generation Prompt: {Image Title}

**Image File**: `{filename}.png`
**Generated**: {YYYY-MM-DD}
**Generator**: Gemini 2.0 Flash (gemini.google.com)
**Attempts**: {N} (final successful attempt)
**Quality**: {100% perfect / 99% with minor fixes / 95% acceptable}

---

## Final Successful Prompt

{FULL IMAGE GENERATION PROMPT EXACTLY AS SUBMITTED TO GEMINI}

---

## Generation Notes

**Attempts Log**:
- **Attempt 1**: {description of result - success or issue}
- **Attempt 2**: {how prompt was adjusted, if needed}
- **Final**: {what succeeded}

**Issues Encountered**:
- {e.g., "Spelling: 'Orchestrate' misspelled as 'Orchterrate'"}
- {e.g., "Redundant text: 'Audio' added to 'Hear' card"}

**Solutions Applied**:
- {e.g., "Letter-by-letter spelling: O-R-C-H-E-S-T-R-A-T-E"}
- {e.g., "Negative instruction: 'Do NOT add any extra text like Audio'"}

**Key Success Factors**:
- {strategies that made this prompt succeed}
- {why this approach worked}

**Reusable Patterns**:
- {patterns from this prompt to apply to future similar visuals}
- {design choices that worked well}

---

## Image Metadata

**Alt Text**: {Accessibility description from original prompt}

**Pedagogical Value**:
- **Teaches**: {one-sentence concept/pattern taught by this visual}
- **Message**: "{one-sentence teaching goal}"
- **Constitutional Alignment**: {Principle X or Philosophy Y}
- **Cognitive Load Impact**: {REDUCES/NEUTRAL} - {justification}

**Design Specs**:
- **Aspect Ratio**: {16:9 / 1:1 / etc.}
- **Dimensions**: {1792x1024px / etc.}
- **Color Palette**: Polar Night Theme (Deep Navy #001f3f, Medium Gray #aaaaaa)
- **Typography**: Roboto family
```

**File Location**: `book-source/static/img/{part}/{chapter}/{image-name}.prompt.md`

**Example**: `book-source/static/img/part-1/chapter-2/five-powers-ai-agents.prompt.md`

**Automation Note**: This can be done manually first, then automated with a Python script to extract prompts from lesson markdown HTML comments.

### Phase 3: Quality Validation

Before finalizing, verify:

- [ ] All prompts are embedded as HTML comments (not visible in rendered markdown)
- [ ] Each prompt includes all required sections (Layout, Typography, Colors, Content, etc.)
- [ ] All data/numbers match lesson content exactly (no fabrication)
- [ ] Filenames are unique and descriptive
- [ ] Alt text is complete and accessible
- [ ] Visual density is appropriate (not too many visuals)
- [ ] Audit report created in `history/visual-assets/`

## Reference: Visual Opportunity Patterns

### Pattern 1: Multiple Statistics (High Priority)

**Identify when**:
- 3+ numbers/percentages in close proximity
- Statistics presented as bullet list
- Dense numerical data in paragraph

**Example**:
```
- 84% of developers using AI tools
- $500M ARR in 2 months
- 75% daily usage
- 7.5% improvement
```

**Visual solution**: Statistics dashboard (2x2 or 3x2 card grid)

---

### Pattern 2: Temporal Progression (High Priority)

**Identify when**:
- Text describes stages, eras, or evolution
- Before/after comparison across time
- Sequential progression mentioned

**Example**:
```
Previous transitions took 10-15 years.
Current AI revolution happening in months.
```

**Visual solution**: Timeline or horizontal bar chart

---

### Pattern 3: A vs B Comparison (High Priority)

**Identify when**:
- Explicit comparison (traditional vs new, old vs modern)
- Contrasting approaches or methodologies
- Table comparing two options

**Example**:
```
Traditional: 80% coding, 20% design
AI-era: 20% specification, 80% oversight
```

**Visual solution**: Split-screen comparison or side-by-side grid

---

### Pattern 4: Sequential Process (Medium Priority)

**Identify when**:
- Numbered steps (3+ steps)
- Workflow or procedure described
- Journey or roadmap mentioned

**Example**:
```
1. Specification → 2. AI generation → 3. Validation → 4. Ship
```

**Visual solution**: Process map with arrows or journey path

---

### Pattern 5: Nested Hierarchy (Medium Priority)

**Identify when**:
- Parent-child relationships
- Organizational structure
- Category breakdowns

**Visual solution**: Tree diagram or nested boxes

## Examples

### Example 1: Statistics Dashboard

**Input**: Lesson with 4 statistics in bullet list

**Output**:
```markdown
The numbers tell the story:
- 84% of developers using AI tools
- $500M Claude Code ARR
- 75% professionals using daily
- 7.5% better documentation

<!-- VISUAL ASSET 1: AI Adoption Statistics Dashboard

IMAGE GENERATION PROMPT:

Professional statistics dashboard showing AI development tool adoption metrics.

Layout: 2x2 grid of metric cards, 1792x1024px (16:9 aspect ratio).
- Overall background: Light Gray (#F8F9FA)
- Grid: 24px gaps between cards, 48px margin from edges
- Each card: 832x448px with 32px internal padding, 8px border-radius

[... complete detailed prompt ...]

Filename: ai-adoption-statistics-2025.png
Alt Text: Dashboard showing four key AI adoption statistics...
-->
```

### Example 2: Timeline

**Input**: Lesson describing historical vs current adoption speeds

**Output**: Complete timeline prompt embedded at optimal location

### Example 3: Skip Low-Priority

**Input**: Single statistic "84% of developers"

**Action**: Skip (one number doesn't need visualization, text emphasis sufficient)

**Rationale**: Low cognitive load, text is clear, visual would be redundant

## Lessons Learned from Chapters 1-2 (Visual Asset Sessions)

This section documents key insights from Chapters 1-2 visual asset generation that should inform prompt creation.

### Chapter 2 New Discoveries (Added 2025-01-12)

#### Lesson 6: Letter-by-Letter Spelling Is PRIMARY Strategy

**Finding**: Explicit letter-by-letter spelling with "IMPORTANT:" prefix eliminates spelling errors more reliably than hyphenation alone.

**Application to Prompt Writing**:
- **Proactively identify "high-risk" words**: 3+ syllables, technical terms, uncommon words
- **Add letter-by-letter spelling BEFORE first attempt**: Don't wait for errors
- **Format**: `IMPORTANT: Spell "Orchestrate" correctly: O-R-C-H-E-S-T-R-A-T-E`

**Evidence from Chapter 2**:
```
❌ WITHOUT: "autonomous" → misspelled as "autonomouss" (2 attempts failed)
✅ WITH LETTER-BY-LETTER: "A-U-T-O-N-O-M-O-U-S-L-Y" → succeeded first try

❌ WITHOUT: "orchestrate" → misspelled as "Orchterrate" (1 attempt failed)
✅ WITH LETTER-BY-LETTER: "O-R-C-H-E-S-T-R-A-T-E" → succeeded first try
```

**Use This Strategy First** (before hyphenation fallback).

#### Lesson 7: Negative Instructions Prevent Unwanted Elements

**Finding**: Explicit "Do NOT" instructions effectively prevent Gemini from adding decorative/redundant text.

**Application to Prompt Writing**:
- When Gemini tends to add extra text (e.g., "Hear | Audio" when you want just "Hear")
- Use combination of positive ("should ONLY say X") + negative ("Do NOT add Y")
- Be specific about what to exclude

**Evidence from Chapter 2**:
```
❌ WITHOUT: "Hear" card → "Hear | Audio" (redundant text added)
✅ WITH NEGATIVE:
   "Card 4 should ONLY say 'Hear' with 'Voice commands' subtitle
    Do NOT add any extra text like 'Audio' to the Hear card"
   → "Hear" only ✓
```

#### Lesson 8: Product Specificity for Emerging Tools

**Finding**: Gemini defaults to well-known products; emerging/niche tools need explicit listing.

**Application to Prompt Writing**:
- For product comparisons: List ALL products explicitly, including emerging ones
- For multi-interface products: Specify which interface (e.g., "Gemini CLI" not "Gemini")
- Don't assume Gemini knows latest/niche tools

**Evidence from Chapter 2**:
```
❌ GENERIC: "Modern AI tools" → Shows only Claude, GPT-4
✅ SPECIFIC: "Claude Code CLI, Gemini CLI, Devin, Zed" → All listed correctly
```

### Chapter 1 Foundational Lessons (Validated in Chapter 2)

### Lesson 1: Simplicity Beats Precision

**Finding**: Gemini handles simple designs (3 elements) far better than complex precise layouts (5+ elements)

**Application to Prompt Writing**:
- When planning bar charts: **Default to 3 bars maximum**
- Select representative examples (high/medium/low) rather than comprehensive datasets
- Favor conceptual clarity over data completeness

**Example**:
```
❌ COMPLEX: 5-bar adoption timeline (Historical/Cloud/Mobile/AI/Future)
   Result: Duplicates, 7+ bars, misalignment (6 failed attempts)

✅ SIMPLE: 3-bar adoption timeline (Historical/Mobile/AI)
   Result: Perfect alignment, correct count (first attempt)
```

### Lesson 2: Hyphenate Technical Terms Proactively

**Finding**: Compound words consistently misspell unless hyphenated

**Application to Prompt Writing**:
- Scan content for compound words: "Autocomplete", "Combinator", "Codebase", etc.
- **Proactively specify hyphenated versions in prompts**: "Auto-Complete", "Code-Base"
- Don't wait for misspellings to occur - prevent them at prompt-writing stage

**High-Risk Words** (add to prompts with hyphens):
- Technical terms: Auto-Complete, Code-Base, Full-Stack, Open-Source
- Company names: Y-Combinator (or use abbreviation "YC")
- Product names: Check actual branding, default to hyphenated

### Lesson 3: Arrow/Icon-Only Indicators Eliminate Text Errors

**Finding**: Text on visual elements (arrows, icons, shapes) frequently misspells; removing text solves issue entirely

**Application to Prompt Writing**:
- **Default to unlabeled indicators**: Clean arrows, clean icons, clean shapes
- Use directionality/position to convey meaning instead of text labels
- Place descriptive text in separate areas (titles, captions), not ON visual elements

**Example**:
```
❌ WITH TEXT: Downward arrow labeled "Accelerating"
   Risk: "Accelrartion" misspelling

✅ TEXT-FREE: Clean downward arrow
   Context: Title "Technology Adoption Speed: Accelerating" provides meaning
   Result: No spelling risk, cleaner design
```

### Lesson 4: Progressive Cleanup After Initial Success

**Finding**: First successful generation often has unnecessary clutter; simplify in follow-up iteration

**Application to Prompt Writing**:
- **Plan for two-stage prompts**: Initial (full detail) → Cleanup (essential only)
- Specify "Version 2" cleanup instructions in prompt notes
- Anticipate that descriptive labels can often be removed after verifying core accuracy

**Example**:
```
Stage 1: Generate timeline with full descriptions
  "Auto-Complete: Line-by-line suggestions (2021-2022)"

Stage 2: Simplify to essential labels only
  "Auto-Complete" and "2021-2022" (separate, minimal)
```

### Lesson 5: Redundancy Check Is Critical

**Finding**: Multiple lessons in Chapter 1 initially had redundant visuals teaching same concept

**Application to Prompt Writing**:
- **Before writing prompt**: Check adjacent lessons for similar visuals
- Ask: "Does another visual already teach this pattern/concept?"
- If redundant: Skip visual OR combine into single comprehensive visual

**Example from Chapter 1**:
```
Lesson 2: Technology adoption acceleration timeline (3 bars)
Lesson 4: [Planned] Adoption speed comparison chart
→ REDUNDANT: Same teaching goal (acceleration pattern)
→ ACTION: Use Lesson 2 visual only, skip Lesson 4 version
```

### Prompt Writing Checklist

Before finalizing prompts for a lesson, verify:

- [ ] **Complexity check**: No charts with 5+ similar elements (simplify to 3 max)
- [ ] **Letter-by-letter spelling**: Challenging words (3+ syllables, technical) spelled out with "IMPORTANT:" prefix
- [ ] **Negative instructions**: Explicit "Do NOT" statements for unwanted elements (if applicable)
- [ ] **Text rendering**: All compound words hyphenated ("Auto-Complete")
- [ ] **Visual indicators**: Arrows/icons unlabeled (text in titles/captions instead)
- [ ] **Label simplification**: Only essential text on visual elements (dates, numbers)
- [ ] **Product specificity**: Emerging/niche tools listed explicitly with full names (e.g., "Gemini CLI" not "Gemini")
- [ ] **Redundancy check**: No duplicate teaching goals with other lesson visuals
- [ ] **Pedagogical value**: Teaches concept/pattern (doesn't just show data)
- [ ] **Teaching goal**: Can state in one sentence what visual teaches
- [ ] **Constitutional alignment**: Supports co-learning, graduated teaching, or evals-first
- [ ] **Prompt preservation plan**: Will create `.prompt.md` sidecar file after generation

## Progressive Disclosure

- **Metadata** (~100 words): Combines auditor + generator into single workflow
- **SKILL.md** (~2k words): Complete audit → generate → embed workflow
- **No bundled resources**: All logic self-contained in SKILL.md

---

**Ready to plan visual assets.** Invoke with: "Plan visuals for lesson {N}" or "Create image prompts for {lesson-file.md}"

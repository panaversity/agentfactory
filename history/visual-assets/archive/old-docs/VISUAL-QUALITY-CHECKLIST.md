# Visual Quality Checklist

**Project**: TutorsGPT Educational Content
**Version**: 1.0 (Visual Skills Framework v5.1.0)
**Last Updated**: 2025-11-22

---

## Purpose

This checklist ensures all educational visuals meet quality standards across five dimensions:
1. Spelling Accuracy
2. Layout Precision
3. Color Accuracy
4. Typography Hierarchy
5. Teaching Effectiveness

Use this checklist during visual generation, review, and before embedding.

---

## How to Use This Checklist

### During Generation (Pre-Review)
- ✅ Review creative brief against checklist BEFORE sending to Gemini
- ✅ Ensure brief specifies all required elements
- ✅ Confirm complexity tier appropriate for proficiency level

### During Visual Review (Post-Generation)
- ✅ Evaluate generated visual against ALL criteria
- ✅ Document specific failures with correction prompts
- ✅ Regenerate if any gate fails critically
- ✅ Archive quality notes for future reference

### Before Embedding (Final Check)
- ✅ Confirm filename matches convention
- ✅ Verify file saved in correct directory
- ✅ Check alt text matches visual content
- ✅ Validate pedagogical placement location

---

## Quality Gate 1: Spelling Accuracy

**Standard**: 99%+ correct spelling across all text elements

### Checklist Items

#### Text Element Review
- [ ] **Headings**: No typos in section/zone titles
- [ ] **Labels**: All command names, file names, technical terms spelled correctly
- [ ] **Annotations**: Explanatory text free of spelling errors
- [ ] **Legend/Key**: Terminology consistent and correct

#### Technical Accuracy
- [ ] **Command syntax**: Exact spelling (e.g., `git commit`, not `git comit`)
- [ ] **File extensions**: Correct case and spelling (`.md`, `.py`, `.png`)
- [ ] **Technical terms**: Industry-standard spelling (e.g., "repository", not "repositry")
- [ ] **Abbreviations**: Standard abbreviations used (e.g., "API", not "Api")

#### Consistency Check
- [ ] **Terminology**: Same term used consistently (don't mix "folder" and "directory")
- [ ] **Capitalization**: Consistent across similar elements
- [ ] **Hyphenation**: Consistent compound word treatment

### Common Errors to Watch For
- ❌ "recieve" → ✅ "receive"
- ❌ "seperator" → ✅ "separator"
- ❌ "occured" → ✅ "occurred"
- ❌ "Environement" → ✅ "Environment"
- ❌ "dependancy" → ✅ "dependency"

### Validation Method
```
Manual Review:
1. Read all visible text slowly
2. Verify technical terms against documentation
3. Check abbreviations against style guide
4. Confirm consistency across entire visual

Automated Check (if available):
1. Screenshot → OCR → spell check
2. Review flagged words individually
3. Verify false positives (technical terms)
```

### Failure Threshold
- **Critical**: >2 errors → Regenerate immediately
- **Minor**: 1-2 errors → Correct if possible, regenerate if quick
- **Pass**: 0 errors ✅

---

## Quality Gate 2: Layout Precision

**Standard**: Aligned, balanced, appropriate whitespace

### Checklist Items

#### Alignment
- [ ] **Vertical alignment**: Elements stack along common axis
- [ ] **Horizontal alignment**: Elements align left/center/right consistently
- [ ] **Grid alignment**: Elements appear to follow invisible grid
- [ ] **Connector alignment**: Arrows/lines start and end precisely at element edges

#### Balance
- [ ] **Visual weight**: No heavy clustering on one side
- [ ] **Symmetry**: Intentional symmetry/asymmetry (not accidental)
- [ ] **Distribution**: Elements evenly distributed across canvas
- [ ] **Negative space**: Whitespace balanced around elements

#### Spacing
- [ ] **Margins**: Adequate space from canvas edges (minimum 20px)
- [ ] **Padding**: Space within boxes/containers consistent
- [ ] **Between elements**: Consistent gaps (e.g., all 40px apart)
- [ ] **Line spacing**: Text lines not cramped or too sparse

#### Sizing
- [ ] **Element proportions**: Boxes/shapes appropriately sized for content
- [ ] **Relative sizing**: Hierarchy clear through size differences
- [ ] **Consistency**: Similar elements same size (all boxes same height)
- [ ] **No overflow**: Text fits within containers without clipping

### Layout Patterns by Visual Type

**Workflow Diagrams**:
- [ ] Left-to-right or top-to-bottom flow
- [ ] Consistent arrow spacing
- [ ] Aligned start/end points
- [ ] Step boxes same height

**Comparison Matrices**:
- [ ] Grid alignment perfect
- [ ] Column widths consistent
- [ ] Row heights uniform
- [ ] Header row clearly distinguished

**Hierarchy Trees**:
- [ ] Parent-child relationships clear through positioning
- [ ] Sibling nodes aligned horizontally
- [ ] Vertical spacing consistent per level
- [ ] Balanced branching

**Architecture Diagrams**:
- [ ] Layer stacking clear
- [ ] Component boundaries distinct
- [ ] Data flow arrows precise
- [ ] Legend positioned consistently

### Validation Method
```
Visual Grid Check:
1. Overlay imaginary grid (8-10 column)
2. Verify elements snap to grid lines
3. Check spacing between elements equal
4. Confirm margins symmetric

Balance Test:
1. Divide visual into quadrants
2. Assess visual weight per quadrant
3. Verify intentional distribution
4. Check for accidental clustering

Ruler Test (if critical):
1. Measure element spacing with tool
2. Verify consistency (±5px acceptable)
3. Check alignment precision
```

### Failure Threshold
- **Critical**: Misaligned by >20px, unbalanced composition → Regenerate
- **Minor**: Slight spacing inconsistencies (<10px) → Accept if otherwise excellent
- **Pass**: Aligned, balanced, well-spaced ✅

---

## Quality Gate 3: Color Accuracy

**Standard**: Semantic colors match intent

### Checklist Items

#### Semantic Color Usage
- [ ] **Red (#EF4444 or similar)**: Friction/problems/old way/errors
- [ ] **Green (#10B981 or similar)**: Success/solutions/new way/correct state
- [ ] **Blue (#3B82F6 or similar)**: Foundation/stability/core concepts/neutral info
- [ ] **Yellow/Orange (#F59E0B or similar)**: Caution/attention/transitions/intermediate state

#### Color Consistency
- [ ] **Same meaning = same color**: All "success" elements use same green
- [ ] **No conflicting meanings**: Red never used for success
- [ ] **Across visual**: Color semantics consistent within single visual
- [ ] **Across project**: Color semantics consistent with other visuals in set

#### Accessibility
- [ ] **Contrast ratio**: Text on background meets WCAG AA (4.5:1 minimum)
- [ ] **Color blindness**: Information not conveyed by color alone
- [ ] **Saturation**: Colors not overly bright or dull
- [ ] **Background**: Appropriate contrast with canvas (white or light gray)

#### Visual Clarity
- [ ] **Distinguishable**: Colors clearly different from each other
- [ ] **Not overwhelming**: Maximum 4-5 colors (excluding black/white/gray)
- [ ] **Purposeful**: Every color has semantic meaning (not decorative)
- [ ] **Professional**: Color palette cohesive and polished

### Semantic Color Guide

**RED (Friction/Problems)**:
- Old/deprecated methods
- Error states
- Bottlenecks in workflows
- Security risks
- Conflicts/collisions
- Anti-patterns

**GREEN (Success/Solutions)**:
- New/recommended methods
- Successful states
- Smooth workflows
- Best practices
- Resolved states
- Achievements

**BLUE (Foundation/Core)**:
- Stable components
- Core concepts
- Neutral information
- System architecture
- Unchanging elements
- Trustworthy sources

**YELLOW/ORANGE (Transition/Caution)**:
- Intermediate states
- Review stages
- Pending actions
- Warning states
- Things to notice
- Decision points

**GRAY (Supporting/Inactive)**:
- Disabled states
- Supporting text
- Backgrounds
- Borders
- Inactive elements
- Secondary information

### Validation Method
```
Color Checker:
1. Screenshot visual
2. Use color picker tool
3. Verify hex codes match semantic intent:
   - Red family: #DC2626 to #EF4444
   - Green family: #059669 to #10B981
   - Blue family: #2563EB to #3B82F6
   - Yellow family: #D97706 to #F59E0B

Semantic Audit:
1. List all colored elements
2. State intended meaning
3. Verify color matches meaning
4. Check for conflicts

Accessibility Test:
1. Use contrast checker (webaim.org/resources/contrastchecker/)
2. Input foreground/background colors
3. Verify AA compliance (4.5:1 minimum)
4. Test with color blindness simulator
```

### Failure Threshold
- **Critical**: Wrong semantic color (red for success) → Regenerate immediately
- **Minor**: Slightly off shade but correct family → Accept if excellent otherwise
- **Pass**: Colors match intent, accessible, consistent ✅

---

## Quality Gate 4: Typography Hierarchy

**Standard**: Clear information hierarchy through typography

### Checklist Items

#### Font Hierarchy
- [ ] **Headings**: Bold, larger size (18pt+)
- [ ] **Subheadings**: Medium weight, medium size (14-16pt)
- [ ] **Body text**: Regular weight, readable size (12-14pt)
- [ ] **Annotations**: Regular or light, smaller size (10-12pt)
- [ ] **Code/Commands**: Monospace font, appropriate size (12-14pt)

#### Size Differentiation
- [ ] **Clear steps**: Each hierarchy level visually distinct
- [ ] **Proportional**: Size differences proportional to importance
- [ ] **Readable**: Minimum 10pt for smallest text
- [ ] **Maximum**: Headings not overwhelming (max 24pt)

#### Font Treatment
- [ ] **Weight variation**: Bold for emphasis, regular for body
- [ ] **Style variation**: Italic for secondary info (if used)
- [ ] **Case variation**: ALL CAPS for major headings only (if used)
- [ ] **Font consistency**: Maximum 2 font families (sans-serif + monospace)

#### Legibility
- [ ] **Letter spacing**: Not too tight (especially in headings)
- [ ] **Line length**: Labels not excessively long
- [ ] **Text on background**: High contrast, easy to read
- [ ] **Anti-aliasing**: Text appears smooth, not pixelated

#### Code/Command Typography
- [ ] **Monospace font**: Commands/code in monospace (e.g., Courier, Consolas)
- [ ] **Distinct from labels**: Clearly different from regular text
- [ ] **Syntax formatting**: Proper spacing, no word wrap
- [ ] **Background treatment**: Often on subtle background for distinction

### Typography Patterns by Element Type

**Zone/Section Headings**:
- Font: Sans-serif, Bold
- Size: 18pt
- Color: Dark (black or near-black)
- Case: Title Case

**Command Labels**:
- Font: Monospace
- Size: 14pt
- Color: Semantic (often blue or black)
- Case: As-written (e.g., `git commit`)

**Annotations/Explanations**:
- Font: Sans-serif, Regular
- Size: 12pt
- Color: Medium gray or semantic
- Case: Sentence case

**File/Path Names**:
- Font: Monospace
- Size: 12-14pt
- Color: Black or dark gray
- Case: As-written

### Validation Method
```
Hierarchy Test:
1. Squint at visual (blur text)
2. Verify heading sizes visually distinct
3. Confirm hierarchy clear without reading
4. Check most important info most prominent

Legibility Test:
1. View at 100% zoom
2. Read all text easily?
3. No squinting required?
4. Text sharp (not blurry)?

Font Audit:
1. List all font faces used
2. Maximum 2 families? (sans-serif + monospace)
3. List all font sizes
4. Clear hierarchy (18pt > 14pt > 12pt)?
```

### Failure Threshold
- **Critical**: Illegible text (<10pt), no hierarchy, wrong fonts → Regenerate
- **Minor**: Slight size inconsistencies, acceptable legibility → Accept
- **Pass**: Clear hierarchy, legible, appropriate fonts ✅

---

## Quality Gate 5: Teaching Effectiveness

**Standard**: Clarifies concept, not decoration

### Checklist Items

#### Pedagogical Value
- [ ] **Enhances understanding**: Concept clearer WITH visual than without
- [ ] **Not decorative**: Every element serves learning purpose
- [ ] **Appropriate complexity**: Matches proficiency tier (A1/A2/B1)
- [ ] **Cognitive load**: Number of elements appropriate for tier
- [ ] **Focus**: No distracting elements

#### Conceptual Clarity
- [ ] **Mental model**: Provides clear mental model of concept
- [ ] **Relationships**: Shows connections between elements clearly
- [ ] **Process flow**: Workflows/sequences obvious (left→right, top→bottom)
- [ ] **Distinctions**: Differences highlighted appropriately

#### Learning Support
- [ ] **Scaffolding**: Supports learning at current proficiency level
- [ ] **Self-explanatory**: Can understand without extensive legend
- [ ] **Memorable**: Visual metaphor aids retention
- [ ] **Transferable**: Pattern applies to similar concepts

#### Proficiency Alignment

**A1 (Beginner: 1-4 elements)**:
- [ ] Extreme simplicity
- [ ] Single concept focus
- [ ] Heavy labeling
- [ ] Familiar metaphors
- [ ] No assumed knowledge

**A2 (Intermediate: 5-7 elements)**:
- [ ] Moderate complexity
- [ ] Related concepts connected
- [ ] Clear labeling
- [ ] Some assumed knowledge
- [ ] Appropriate scaffolding

**B1 (Advanced: 7-10 elements)**:
- [ ] Realistic complexity
- [ ] Production-relevant
- [ ] Minimal labeling needed
- [ ] Assumes foundation knowledge
- [ ] Professional context

#### Avoidance of Anti-Patterns
- [ ] **Not a screenshot**: Custom-designed for teaching (not raw screenshot)
- [ ] **Not clipart**: Professional illustration (not generic clipart)
- [ ] **Not chart junk**: No unnecessary decorative elements
- [ ] **Not overcrowded**: Whitespace and breathing room present
- [ ] **Not ambiguous**: Single clear interpretation (not confusing)

### Teaching Effectiveness Evaluation

**Ask These Questions**:
1. **Does this visual make the concept easier to understand?**
   - YES → Continue
   - NO → What's missing? Regenerate with correction

2. **Can a student explain the concept using ONLY this visual?**
   - YES → Excellent teaching effectiveness
   - MOSTLY → Good, may need minor text support
   - NO → Visual insufficient, regenerate

3. **What would happen if we removed this visual from the lesson?**
   - Concept significantly harder to understand → Essential visual ✅
   - Slightly harder → Helpful visual ✅
   - No impact → Decorative visual ❌ (reconsider need)

4. **Does the complexity match the student's current knowledge level?**
   - YES → Appropriately scaffolded ✅
   - TOO SIMPLE → May bore advanced students
   - TOO COMPLEX → May overwhelm beginners ❌

5. **Does every element serve a learning purpose?**
   - YES → Efficient visual ✅
   - MOSTLY → Could trim 1-2 elements
   - NO → Contains decoration ❌ (simplify)

### Validation Method
```
Concept Test:
1. Show visual to someone unfamiliar with concept
2. Ask them to explain what they see
3. Can they derive the concept from visual alone?
4. Are they forming correct mental model?

Complexity Audit:
1. Count visual elements
2. Compare to proficiency tier limits:
   - A1: 1-4 elements
   - A2: 5-7 elements
   - B1: 7-10 elements
3. Verify no extraneous elements
4. Check cognitive load appropriate

Element Purpose Check:
1. List every visual element
2. State learning purpose for each
3. If no purpose → Mark for removal
4. Regenerate if >2 purposeless elements

Before/After Comparison:
1. Read lesson WITHOUT visual
2. Rate understanding difficulty (1-10)
3. Review lesson WITH visual
4. Rate understanding difficulty again
5. Visual should reduce difficulty by 2+ points
```

### Failure Threshold
- **Critical**: Decorative only, wrong complexity tier, confusing → Regenerate
- **Minor**: Could be slightly simpler but still effective → Accept
- **Pass**: Clearly teaches concept, appropriate complexity, purposeful ✅

---

## Combined Quality Assessment

### Overall Quality Score

**Rate Each Gate** (1-5 scale):
- 5 = Exceeds standards
- 4 = Meets standards fully
- 3 = Meets standards minimally
- 2 = Below standards (minor issues)
- 1 = Fails standards (major issues)

**Example Assessment**:
```
Visual: V18 - Git Three-Stage Workflow

Gate 1 - Spelling Accuracy:       5/5 ✅ (No errors detected)
Gate 2 - Layout Precision:        4/5 ✅ (Slight spacing variance, acceptable)
Gate 3 - Color Accuracy:          5/5 ✅ (Perfect semantic colors)
Gate 4 - Typography Hierarchy:    5/5 ✅ (Clear hierarchy, legible)
Gate 5 - Teaching Effectiveness:  5/5 ✅ (Excellent mental model)

Overall Score: 24/25 (96%) ✅ APPROVED
```

### Approval Criteria

**APPROVE** (Embed visual):
- Overall score ≥ 20/25 (80%)
- No gate scores below 3
- No critical failures

**CONDITIONAL APPROVE** (Accept with notes):
- Overall score 18-19/25 (72-76%)
- One gate score of 2, rest 3+
- Minor issues documented for future improvement

**REJECT** (Regenerate visual):
- Overall score < 18/25 (<72%)
- Any gate score of 1
- Any critical failures

---

## Quick Reference Checklist

### Pre-Generation Checklist (Creative Brief Review)

```
□ Proficiency tier identified (A1/A2/B1)
□ Element count appropriate for tier
□ Semantic colors specified
□ Typography hierarchy defined
□ Pedagogical purpose stated
□ Visual type appropriate (workflow, comparison, etc.)
□ Metaphor/mental model clear
□ All technical terms spelled correctly
```

### Post-Generation Checklist (Visual Review)

```
□ Spelling: All text error-free
□ Layout: Aligned, balanced, spaced
□ Colors: Semantic accuracy, accessible
□ Typography: Clear hierarchy, legible
□ Teaching: Clarifies concept effectively
□ Filename: Matches convention
□ Directory: Saved in correct location
□ Overall: Score ≥ 20/25
```

### Pre-Embedding Checklist (Final Validation)

```
□ Visual approved (score ≥ 20/25)
□ File location verified
□ Filename correct
□ Alt text written (50-100 words)
□ Placement location identified
□ Pedagogical timing appropriate
□ Markdown reference prepared
□ Ready to embed ✅
```

---

## Appendix A: Common Visual Issues & Fixes

### Issue: Text Too Small

**Symptom**: Labels < 10pt, difficult to read

**Fix**:
- Increase minimum font size to 12pt
- Reduce number of labels if space constrained
- Simplify visual to allow larger text

**Example Prompt**:
```
Regenerate with larger text. Minimum font size 12pt for all labels.
Reduce number of elements if needed to accommodate larger text.
```

### Issue: Too Many Colors

**Symptom**: 6+ colors, overwhelming, no clear semantics

**Fix**:
- Limit to 4 colors + black/white/gray
- Assign semantic meaning to each color
- Remove decorative colors

**Example Prompt**:
```
Regenerate using only 4 colors:
- Red (#EF4444): Problems/old way
- Green (#10B981): Solutions/new way
- Blue (#3B82F6): Core concepts
- Gray (#6B7280): Supporting elements
```

### Issue: Poor Alignment

**Symptom**: Elements appear randomly placed, no grid

**Fix**:
- Specify alignment explicitly
- Request grid-based layout
- Define spacing rules

**Example Prompt**:
```
Regenerate with precise alignment:
- All boxes align vertically on left edge
- Consistent 40px spacing between elements
- Arrows start/end exactly at box edges
- Use invisible grid for perfect alignment
```

### Issue: No Visual Hierarchy

**Symptom**: All text same size, no emphasis

**Fix**:
- Define size hierarchy explicitly
- Specify font weights
- Clarify information priority

**Example Prompt**:
```
Regenerate with clear typography hierarchy:
- Zone headings: Bold, 18pt
- Command labels: Monospace, 14pt
- Annotations: Regular, 12pt
Ensure headings most prominent, commands distinct, annotations supporting.
```

### Issue: Overcomplicated (Too Many Elements)

**Symptom**: A1 visual with 10 elements (cognitive overload)

**Fix**:
- Count elements in brief
- Remove non-essential elements
- Simplify to tier limit

**Example Prompt**:
```
This visual has 10 elements but targets A1 proficiency (limit: 4 elements).
Regenerate with only these 4 essential elements:
1. Working Directory box
2. Staging Area box
3. Repository box
4. Command flow arrow
Remove all file icons, state labels, and extra annotations.
```

---

## Appendix B: Visual Type Templates

### Workflow Diagram Template

**Structure**:
```
[Step 1] → [Step 2] → [Step 3]
```

**Quality Criteria**:
- [ ] Left-to-right or top-to-bottom flow
- [ ] Arrows clearly show direction
- [ ] Steps visually distinct (boxes, rounded rectangles)
- [ ] Labels concise (2-4 words per step)
- [ ] Color: Blue for stable steps, yellow for transition, green for success

### Comparison Matrix Template

**Structure**:
```
         | Option A | Option B | Option C
Feature 1|    ✓     |    ✗     |    ✓
Feature 2|    ✗     |    ✓     |    ✓
```

**Quality Criteria**:
- [ ] Perfect grid alignment
- [ ] Column widths equal
- [ ] Row heights equal
- [ ] Header row visually distinct (bold, colored background)
- [ ] Checkmarks/crosses clear

### Hierarchy Tree Template

**Structure**:
```
       [Parent]
      /    |    \
  [Child] [Child] [Child]
```

**Quality Criteria**:
- [ ] Parent centered above children
- [ ] Children aligned horizontally
- [ ] Connecting lines precise
- [ ] Levels clearly separated
- [ ] Balanced distribution

### Architecture Diagram Template

**Structure**:
```
[Layer 3: Top]
[Layer 2: Middle]
[Layer 1: Foundation]
```

**Quality Criteria**:
- [ ] Layers stacked clearly
- [ ] Vertical alignment consistent
- [ ] Arrows show data flow
- [ ] Components within layers aligned
- [ ] Foundation layer visually stable (blue)

---

**Checklist Version**: 1.0
**Framework**: Visual Skills Framework v5.1.0
**Last Updated**: 2025-11-22
**Status**: Production-Ready ✅

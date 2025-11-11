---
name: technical-clarity
description: |
  Reviews technical explanations for jargon, readability, completeness, and accessibility for target
  learners. Activate when authors need feedback on clarity of technical writing, want to identify
  unclear passages or gatekeeping language, assess readability metrics, or improve accessibility
  of explanations. Analyzes text for jargon density, missing context, assumed knowledge, sentence
  complexity, and provides specific improvement suggestions. Use when reviewing tutorials, documentation,
  book chapters, or any instructional technical content for Python or programming concepts.
---

## Purpose

The technical-clarity skill helps authors review technical explanations for clarity, accessibility, and completeness. This skill identifies jargon, gatekeeping language, readability issues, missing context, and provides actionable suggestions to improve instructional content for target audiences.

## When to Activate

Use this skill when:
- Authors need feedback on technical explanation clarity
- Reviewing draft tutorials, documentation, or book chapters
- Identifying jargon that needs definition
- Detecting gatekeeping language ("obviously", "simply", etc.)
- Assessing readability level vs. target audience
- Checking for completeness (missing prerequisites, context, examples)
- Improving accessibility of technical content
- Preparing content for publication

## Inputs

Required:
- **Text to review**: The technical explanation or document
- **Target audience**: beginner | intermediate | advanced

Optional:
- **Specific concerns**: Areas author wants feedback on
- **Document type**: tutorial | reference | example | explanation
- **Context**: Where content will be used (book, course, documentation)

## Process

### Step 1: Understand Review Scope

Clarify:
- What concept is being explained
- Who is the target reader
- What specific clarity concerns exist
- Type of review needed (quick scan vs. comprehensive)

### Step 2: Load Readability Metrics Reference

Read readability metrics for quantitative assessment:

```bash
Read reference/readability-metrics.md
```

Key considerations:
- Flesch-Kincaid grade level target
- Sentence length guidelines
- Vocabulary complexity
- Jargon density thresholds

### Step 3: Perform Initial Readability Analysis

Use shared readability analyzer:

```bash
python .claude/skills/_shared/readability-analyzer.py explanation.md
```

The script will provide:
- Flesch-Kincaid grade level
- Average sentence length
- Vocabulary complexity metrics
- Comparison to target audience level

Review output:
- **Beginner target**: Grade level 6-8 ideal
- **Intermediate target**: Grade level 9-12 ideal
- **Advanced target**: Grade level 13+ acceptable

### Step 4: Check for Gatekeeping Language

Load gatekeeping language reference:

```bash
Read reference/gatekeeping-language.md
```

Search for dismissive/exclusive terms:
- **Minimizers**: "obviously", "clearly", "simply", "just", "trivially"
- **Assumptive**: "of course", "everyone knows", "naturally"
- **Ableist**: "crazy", "insane", "dumb", "lame"
- **Gendered**: Generic "he/his" (use "they/their")

Flag each instance with:
- Location (paragraph/line)
- Type of gatekeeping
- Suggested replacement

### Step 5: Identify Jargon and Technical Terms

Scan for technical vocabulary:
- Highlight all domain-specific terms
- Check if each term is defined or already familiar
- Calculate jargon density (terms per paragraph)

**Thresholds**:
- **Beginner**: Max 2-3 undefined terms per paragraph
- **Intermediate**: Max 4-5 undefined terms
- **Advanced**: More flexible, but still define uncommon terms

### Step 6: Assess Completeness

Run completeness check:

```bash
python .claude/skills/technical-clarity/scripts/check-completeness.py explanation.md
```

The script checks for:
- Prerequisites stated
- Code examples provided
- Terms defined
- Context/motivation (why it matters)
- Expected outputs shown
- Error cases mentioned
- Document structure (headings)

Review output for missing critical elements.

### Step 7: Load Clarity Checklist

Perform systematic clarity audit:

```bash
Read reference/clarity-checklist.md
```

Check all dimensions:
1. **Jargon**: Defined or familiar?
2. **Completeness**: All information provided?
3. **Assumptions**: Prior knowledge explicit?
4. **Context**: "Why" explained, not just "what"?
5. **Examples**: Concrete demonstrations?
6. **Readability**: Sentence length, structure appropriate?
7. **Structure**: Logical flow, clear sections?
8. **Error Prevention**: Common mistakes mentioned?
9. **Verification**: Can learner test understanding?

### Step 8: Evaluate Analogies

If analogies are used, load analogy patterns reference:

```bash
Read reference/analogy-patterns.md
```

Check each analogy:
- Maps to genuinely familiar experience?
- Highlights key shared properties?
- Explains where analogy breaks down?
- Culturally accessible?
- Doesn't introduce new confusion?

### Step 9: Compile Clarity Report

Create structured report following template:

```bash
Read templates/clarity-report-template.yml
```

Include:
- **Overall clarity score** (1-5)
- **Readability metrics** (grade level, sentence length)
- **Jargon analysis** (terms found, definition status)
- **Gatekeeping language** instances
- **Completeness analysis** (missing elements)
- **Specific clarity issues** with locations
- **Strengths** (positive aspects)
- **Recommendations** (immediate/important/enhancement)

### Step 10: Prioritize Recommendations

Categorize improvements:

**Critical** (must fix):
- Gatekeeping language
- Missing prerequisites for complex concepts
- Undefined jargon in beginner content
- Grade level 5+ levels above target

**Important** (should fix):
- Missing code examples
- No context/motivation
- Very long sentences (>30 words)
- Poor structure/organization

**Enhancement** (nice to have):
- Additional examples
- Better analogies
- Improved transitions

## Output Format

Provide clarity report as structured markdown:

```markdown
# Clarity Report: [Document Title]

**Target Audience**: [beginner/intermediate/advanced]
**Overall Clarity Score**: [X/5]
**Date**: [YYYY-MM-DD]

## Summary

[2-3 sentence overview of clarity assessment]

## Readability Metrics

- **Flesch-Kincaid Grade Level**: [X.X]
- **Target Level**: [Y]
- **Assessment**: [on-target / too-complex / too-simple]
- **Average Sentence Length**: [X words]
- **Recommendation**: [if adjustment needed]

## Jargon Analysis

**Jargon Density**: [low / medium / high]
**Undefined Terms**: [count]

| Term | Defined? | Location | Recommendation |
|------|----------|----------|----------------|
| [term] | No | Line X | Define on first use |
| [term] | Yes | N/A | ✓ Clear |

## Gatekeeping Language

[If none: "No gatekeeping language detected ✓"]

[If found:]
| Phrase | Location | Type | Replace With |
|--------|----------|------|--------------|
| "obviously" | Para 2 | Minimizer | [Explain reasoning] |
| "simply add" | Para 5 | Minimizer | "Add..." [with steps] |

## Completeness Issues

**Completeness Score**: [X/100]

### Missing Elements

1. **Prerequisites** [Severity: moderate]
   - **Issue**: Assumed knowledge not stated
   - **Fix**: Add "Prerequisites: understanding of functions and lists"

2. **Code Examples** [Severity: critical]
   - **Issue**: No working examples provided
   - **Fix**: Add 2-3 runnable code demonstrations

[Continue for all missing elements...]

## Specific Clarity Issues

### Issue 1: Vague Language
- **Location**: Paragraph 3
- **Problem**: "Do the necessary steps" is unclear
- **Fix**: List specific steps: 1) ..., 2) ..., 3) ...

### Issue 2: Missing Context
- **Location**: Paragraph 7
- **Problem**: Introduces decorators without explaining why
- **Fix**: Add: "Decorators are useful because..."

[Continue for all issues...]

## Strengths

- ✓ Clear code examples with comments
- ✓ Good use of analogies
- ✓ Logical structure with helpful headings

## Recommendations

### Immediate (Critical)

1. Define technical terms on first use (lines X, Y, Z)
2. Remove gatekeeping language ("obviously", "simply")
3. State prerequisites explicitly at beginning

### Important (Should Address)

1. Add code examples for each major concept
2. Explain "why" not just "what" for key concepts
3. Break up long paragraphs (>150 words)

### Enhancements (Optional)

1. Include common mistakes section
2. Provide checkpoint questions for self-assessment

## Accessibility Score

**Score**: [X/5]
**Assessment**: [Explanation of score and key improvements for accessibility]
```

## Examples

### Example 1: Review Tutorial for Beginners

**Input**: "Review this Python functions tutorial for beginners"

**Process**:
1. Run readability analysis (Grade level: 14 - too high!)
2. Identify jargon (8 undefined terms)
3. Find gatekeeping ("obviously", "just", "simply" used 6 times)
4. Check completeness (missing prerequisites, no examples)
5. Compile report with prioritized fixes

**Output**: Clarity report showing grade level too high, too much jargon, gatekeeping language, and missing examples. Specific line numbers and replacements provided.

---

### Example 2: Quick Jargon Scan

**Input**: "Just check for undefined jargon in this explanation"

**Process**:
1. Scan for technical terms
2. Check definition status
3. Flag undefined terms
4. Suggest where to add definitions

**Output**: List of jargon with definition status and recommendations

---

### Example 3: Gatekeeping Audit

**Input**: "Does this contain gatekeeping language?"

**Process**:
1. Load gatekeeping reference
2. Search for minimizers, assumptive phrases, ableist terms
3. Flag each instance with location and replacement

**Output**: Gatekeeping audit with specific instances and suggested replacements

## Common Patterns

### Pattern 1: Beginner Content Review

Focus on:
- Grade level (should be 6-8)
- Jargon definitions
- Gatekeeping language
- Complete examples
- Prerequisites stated

### Pattern 2: Technical Accuracy vs. Clarity

Sometimes technically precise language is less clear. Balance:
- Technical correctness
- Learner comprehension
- Progressive terminology (simple first, precise later)

### Pattern 3: Cultural Accessibility

Check analogies and references:
- Are they universally understandable?
- Do they assume specific cultural knowledge?
- Consider international audience

## Validation Checklist

Before finalizing report:
- [ ] Readability metrics calculated and assessed
- [ ] All jargon identified and definition status checked
- [ ] Gatekeeping language flagged with replacements
- [ ] Completeness issues identified
- [ ] Specific issues include locations and fixes
- [ ] Recommendations prioritized (critical/important/enhancement)
- [ ] Overall clarity score assigned (1-5)
- [ ] Report is actionable (author knows what to do)

## Acceptance Checks

- [ ] SpecRef present (what spec/section this content maps to)
- [ ] Gatekeeping auto-fix suggestions provided for each flagged instance
- [ ] Readability target band appropriate to audience (Beginner: 6–8, Interm: 9–12, Adv: 13+)
- [ ] Spec alignment: each major paragraph maps to a spec point or is marked rationale/context

## References

Supporting documentation (loaded as needed):
- `reference/readability-metrics.md` - Flesch-Kincaid, sentence length, vocabulary
- `reference/gatekeeping-language.md` - Minimizers, assumptive phrases to avoid
- `reference/analogy-patterns.md` - Effective analogy structure
- `reference/clarity-checklist.md` - Systematic clarity review criteria

## Error Handling

If analysis fails:
1. Report specific issues (e.g., "File too short to analyze", "No text content found")
2. Suggest minimum requirements (e.g., "Need at least 50 words for meaningful analysis")
3. Halt and require user intervention (hard failure mode)

Clarity feedback must be specific, actionable, and prioritized—not vague criticism.

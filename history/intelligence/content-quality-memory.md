# Content Quality Memory - Educational Intelligence Learnings

**Purpose**: Persistent memory of quality issues and prevention patterns from audits.
**Scope**: Applies to ALL educational content (any subject, any part, any proficiency level).
**Version**: 1.0.0
**Last Updated**: 2025-11-18 (Part 4 Audit)

---

## Anti-Patterns: What NOT to Do

### ❌ Anti-Pattern 1: Exposing Pedagogical Scaffolding

**Violation**: Students see the framework instead of experiencing the learning.

**Example from Part 4 Audit** (Python, but pattern applies to ANY subject):
```markdown
<!-- WRONG: Exposes Three Roles framework -->
### Part 2: AI as Teacher (Teaching You the Pattern)
**AI's Role**: Explain asyncio patterns...
**Your Role**: Listen and learn...

<!-- CORRECT: Students experience roles invisibly -->
### Understanding Async Patterns
> **💬 AI Colearning Prompt**: "Explain asyncio patterns with examples..."
```

**Why it happens**:
- Agent thinks being explicit helps students understand
- Pattern-matching from pre-constitutional content
- Not asking: "Am I exposing scaffolding?"

**Prevention (works for ANY subject)**:
- Headers describe ACTIVITIES, not roles: "Exploring", "Understanding", "Building"
- Prompts guide action without naming framework: "Ask your AI..." not "AI as Teacher will..."
- Students EXPERIENCE bidirectional learning, never see labels

**Validation**: Grep for role labels → MUST return 0
```bash
grep -i "Part [0-9]:.*as Teacher\|as Student\|as Scientist\|Your Role:\|AI's Role:" [content]
# Expected: 0 matches
```

---

### ❌ Anti-Pattern 2: Unverified Learning Claims

**Violation**: Content makes claims without showing evidence.

**Example from Part 4 Audit** (code, but applies to ANY knowledge domain):
```markdown
<!-- WRONG: No proof this works -->
Use this algorithm to sort efficiently:
```python
def quicksort(arr): ...
```

<!-- CORRECT: Evidence provided -->
Use this algorithm to sort efficiently:
```python
def quicksort(arr): ...
```

**Output:**
```
[1, 2, 3, 4, 5]  # Sorted in 0.002s
```
```

**Generalized to ANY subject**:
- Teaching history? Show primary source citations
- Teaching design? Show before/after examples with measurements
- Teaching math? Show worked problems with verification

**Why it happens**:
- Agent focuses on explanation, not proof
- "Create content" interpreted as "write explanations" not "provide evidence"
- No self-check: "Can I verify this claim?"

**Prevention**:
- Constitution Principle 3: **Verification Over Assumption**
- Every factual claim → cite source or show test
- Every code example → show execution output
- Every design principle → demonstrate with real artifact

**Validation**: Check claims have evidence
```bash
# For code lessons:
grep -A 5 '```python' [content] | grep "**Output:**"

# For any lesson:
grep -i "studies show\|research indicates\|evidence suggests" [content]
# If found → must have citation nearby
```

---

### ❌ Anti-Pattern 3: Cognitive Overload at Lesson End

**Violation**: Adding "helpful" content after the lesson's natural conclusion.

**Example from Part 4 Audit**:
```markdown
<!-- WRONG: Adds cognitive load after conclusion -->
## Try With AI
[Student explores on their own]

---

## Summary: What You Learned
[Repeats what they just did]

## What's Next
[Preview of next lesson]

## Red Flags to Watch
[Troubleshooting list]

<!-- CORRECT: Minimal sufficient ending -->
## Try With AI
[Exploration prompts, including troubleshooting if needed]

---
```

**Why it happens**:
- Agent thinks "more content = more helpful"
- Traditional textbook pattern (summary, review, preview)
- Not applying Constitution Principle 7: **Minimal Sufficient Content**

**Prevention (applies to ANY subject)**:
- Lesson ends with student action (Try With AI, Explore, Practice)
- No "summary" (students just learned it, don't repeat)
- No "what's next" (breaks immersion, creates anxiety)
- Troubleshooting? Integrate into action section

**Validation**: Check lesson structure
```bash
# Find last ## heading before end of file
tail -20 [content] | grep -E "^## " | tail -1
# Expected: "## Try With AI" or "## Practice" or "## Explore"

# Check nothing comes after
grep -A 20 "^## Try With AI" [content] | grep -E "^## "
# Expected: Only "## Try With AI", then "---", then EOF
```

---

### ❌ Anti-Pattern 4: Template Compliance Without Understanding

**Violation**: Following template literally without grasping pedagogical intent.

**Example**:
- Template says "end with Try With AI"
- Agent adds it, but ALSO adds "Summary" section
- Technically has "Try With AI", but violates spirit (minimal content)

**Why it happens**:
- Pattern-matching on structure keywords
- Not asking: "What is the INTENT of this rule?"
- Checklist mentality vs. reasoning mentality

**Prevention**:
- Don't just match patterns → understand WHY
- "End with Try With AI" means "student's LAST action is exploration"
- Ask: "Does this serve learning, or is it just habit?"

---

## Successful Patterns: What TO Do

### ✅ Pattern 1: Activity-Focused Pedagogy

**Principle**: Frame content as activities students DO, not roles they play.

**Examples** (work for ANY subject):
```markdown
<!-- Teaching programming -->
### Building Your First Function
[Students construct, not told about construction]

<!-- Teaching history -->
### Analyzing Primary Sources
[Students examine documents, not told about examination]

<!-- Teaching design -->
### Prototyping Your Interface
[Students create, not told about creation]
```

**Why it works**:
- Students engage directly with content
- Framework (Three Roles) happens naturally
- No meta-cognitive overhead

---

### ✅ Pattern 2: Evidence-First Content

**Principle**: Show → Explain, not Explain → (Maybe Show)

**Template**:
```markdown
## [Concept Name]

**What it does**: [Brief description]

**Example** (showing it works):
[Artifact, code, document, calculation]

**Evidence**:
[Output, result, citation, measurement]

**Why it matters**: [Context and application]

**Try it yourself**: [Action prompt for AI collaboration]
```

**Why it works**:
- Concrete before abstract (better retention)
- Claims are verifiable
- Constitutional Principle 3 compliance

---

### ✅ Pattern 3: Minimal Sufficient Structure

**Principle**: Only sections that serve learning objectives.

**Standard Lesson Structure**:
```markdown
# Lesson Title

[Frontmatter metadata]

## Introduction (1-2 paragraphs)
[Why this matters, what you'll learn]

## [Learning Objective 1]
[Explanation, examples, evidence]

## [Learning Objective 2]
[Explanation, examples, evidence]

## [Learning Objective 3]
[Explanation, examples, evidence]

## Try With AI
[Action prompts for exploration]

---
```

**What's ABSENT** (deliberately):
- ❌ Summary (just repeat what was said)
- ❌ Key Takeaways (already in learning objectives)
- ❌ What's Next (breaks immersion)
- ❌ Common Mistakes (integrate into Try With AI instead)

**Why it works**:
- Respects student's cognitive load
- Every section maps to learning objective
- Constitutional Principle 7 compliance

---

## Pre-Generation Checklist

Before generating ANY educational content, ask these questions:

### 1. Framework Visibility Check
**Question**: "Am I exposing pedagogical scaffolding to students?"

**Red Flags**:
- Using role labels: "AI as Teacher", "Student as Scientist"
- Explaining the framework: "Now we'll do bidirectional learning"
- Meta-commentary: "This section teaches you to teach AI"

**Fix**: Use activity headers ("Understanding...", "Building..."), not role labels.

---

### 2. Evidence Check
**Question**: "Can I prove the claims I'm making?"

**Red Flags**:
- Code without output
- "Studies show..." without citation
- "This is best practice" without demonstration

**Fix**: Add **Output:**, citations, or worked examples showing proof.

---

### 3. Structure Check
**Question**: "Does lesson end with student action ONLY?"

**Red Flags**:
- Multiple ## sections after Try With AI
- Summary/Key Takeaways at end
- "What's Next" or "Coming Up"

**Fix**: Move/remove sections, or integrate into Try With AI if essential.

---

### 4. Complexity Check
**Question**: "Does cognitive load match proficiency tier?"

**Tier Guidelines**:
- **A1-A2 (Beginner)**: 5-7 concepts, heavy scaffolding, 2 options max
- **B1-B2 (Intermediate)**: 7-10 concepts, moderate scaffolding, 3-4 options
- **C1-C2 (Advanced)**: No artificial limits, realistic complexity

**Red Flags**:
- A2 lesson with 15 concepts
- C2 lesson with toy examples
- Missing proficiency tier in metadata

**Fix**: Reduce/expand concepts to match tier, add/remove scaffolding.

---

## Post-Generation Validation

After drafting content, run these checks:

```bash
# 1. Meta-commentary check (MUST be 0)
grep -i "Part [0-9]:\|AI as\|Student as\|Your Role:\|AI's Role:" [draft-file]

# 2. Evidence check (code lessons)
python_blocks=$(grep -c '```python' [draft-file])
output_blocks=$(grep -c '\*\*Output:\*\*' [draft-file])
# output_blocks should be >= 70% of python_blocks

# 3. Structure check (MUST be Try With AI or equivalent)
last_section=$(tail -30 [draft-file] | grep -E '^## ' | tail -1)
# Should be "## Try With AI" or "## Practice" or "## Explore"

# 4. Metadata check
grep -q "^proficiency_level:" [draft-file]  # Required
grep -q "^cefr_level:" [draft-file] && echo "DEPRECATED FORMAT"  # Forbidden
```

**If ANY check fails → Self-correct before delivery.**

---

## Subject-Agnostic Validation Script

Use this template for validating ANY educational content:

```bash
#!/bin/bash
# Constitutional validation for educational content

FILE="$1"

echo "=== Constitutional Validation ==="
echo "File: $FILE"
echo ""

# Check 1: Meta-commentary
META_COUNT=$(grep -ic "Part [0-9]:\|AI as\|Student as\|Your Role:\|AI's Role:" "$FILE")
if [ "$META_COUNT" -gt 0 ]; then
    echo "❌ FAIL: Meta-commentary found ($META_COUNT instances)"
    grep -in "Part [0-9]:\|AI as\|Student as" "$FILE"
else
    echo "✅ PASS: No meta-commentary"
fi

# Check 2: Lesson ending
LAST_SECTION=$(tail -30 "$FILE" | grep -E '^## ' | tail -1)
if [[ "$LAST_SECTION" =~ "Try With AI"|"Practice"|"Explore" ]]; then
    # Check nothing after
    AFTER_COUNT=$(grep -A 10 "$LAST_SECTION" "$FILE" | grep -c '^## ')
    if [ "$AFTER_COUNT" -gt 1 ]; then
        echo "❌ FAIL: Sections after final activity section"
    else
        echo "✅ PASS: Lesson ends correctly"
    fi
else
    echo "❌ FAIL: Lesson doesn't end with activity section"
    echo "   Found: $LAST_SECTION"
fi

# Check 3: Metadata format
if grep -q "^proficiency_level:" "$FILE"; then
    echo "✅ PASS: Using proficiency_level"
elif grep -q "^cefr_level:" "$FILE"; then
    echo "❌ FAIL: Using deprecated cefr_level"
else
    echo "⚠️  WARN: No proficiency metadata"
fi

echo ""
echo "=== Validation Complete ==="
```

---

## Learning from Part 4 Audit (2025-11-18)

**Context**: First comprehensive constitutional audit of completed content.

**Violations Found**:
- 13 lessons: Meta-commentary exposing framework
- 70%+ lessons: Missing test evidence
- 7 lessons: Non-compliant endings
- 5 lessons: Deprecated metadata

**Root Causes** (generalizable):
1. **Agent prompts emphasized "create content" not "verify content"**
   - Fix: Add verification as generation requirement

2. **Template said "end with X" but didn't say "ONLY X"**
   - Fix: Explicit structure constraints in templates

3. **No self-validation checkpoint before delivery**
   - Fix: Mandatory post-generation grep checks

4. **Pattern-matching from pre-constitutional content**
   - Fix: This memory file as reference

**Prevention**: All future content MUST reference this file before generation.

---

## Agent Prompt Improvements

### For content-implementer

Add to beginning of prompt:

```markdown
## MANDATORY: Pre-Generation Constitutional Check

**Step 1**: Read `.specify/memory/content-quality-memory.md`
- Review anti-patterns (what NOT to do)
- Review successful patterns (what TO do)
- Reference validation checklist

**Step 2**: Before generating, ask yourself:
1. "Am I exposing pedagogical scaffolding?" → Use activity headers, not role labels
2. "Can I prove my claims?" → Add evidence (output, citations, examples)
3. "Does lesson end with student action ONLY?" → No summary/what's next sections
4. "Does complexity match proficiency tier?" → Check A2/B1/C2 guidelines

**Step 3**: After generating, run validation script
- Meta-commentary check → 0 instances
- Evidence check → code has output
- Structure check → ends with Try With AI
- Metadata check → uses proficiency_level

**Only deliver content after ALL checks pass.**
```

---

## Version History

**v1.0.0** (2025-11-18):
- Initial creation from Part 4 audit learnings
- Anti-patterns: Scaffolding exposure, unverified claims, cognitive overload, template compliance
- Successful patterns: Activity-focused, evidence-first, minimal structure
- Validation: Pre/post-generation checklists
- Subject-agnostic (applies to ANY educational content)

**Next Update**: After Part 5+ audits, incorporate new learnings.

---

**Usage**: All content-generating agents MUST reference this file before creating educational materials.
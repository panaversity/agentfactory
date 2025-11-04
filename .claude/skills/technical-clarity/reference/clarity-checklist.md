# Technical Clarity Checklist

## Overview

Systematic criteria for evaluating technical explanations for clarity, completeness, and accessibility.

## Clarity Dimensions

### 1. Jargon Identification

**Check**: Are technical terms defined or already familiar?

**Red Flags**:
- More than 2-3 undefined terms in a paragraph
- Acronyms without expansion
- Domain-specific vocabulary without context

**Fix**:
```
❌ "Use a REPL for interactive development."

✅ "Use a REPL (Read-Eval-Print Loop), an interactive programming
environment, for testing code snippets."
```

### 2. Completeness

**Check**: Is all necessary information provided?

**Common Gaps**:
- Missing prerequisites
- Undefined parameters
- Unspecified expected behavior
- Missing error conditions

**Example**:
```
❌ "Call the function with appropriate arguments."
(What are appropriate arguments?)

✅ "Call calculate_average(numbers) with a list of numbers as the argument."
```

### 3. Assumption Detection

**Check**: What prior knowledge is assumed?

**Test**: Can a beginner follow without external research?

**Fix**: State prerequisites explicitly
```
✅ "Prerequisites: Understanding of functions and lists."
✅ "If you're unfamiliar with X, review [link/section] first."
```

### 4. Context Provision

**Check**: Is the "why" explained, not just "what"?

**Example**:
```
❌ "Use list comprehensions."
(Why should I?)

✅ "Use list comprehensions because they're more concise and often faster
than explicit loops for simple transformations."
```

### 5. Examples and Counter-Examples

**Check**: Are concrete examples provided?

**Pattern**:
- General concept
- Specific example
- Counter-example (what it's NOT)

```
✅ "Mutable objects can be changed after creation. For example, lists are
mutable—you can append items. In contrast, strings are immutable—changing
them creates a new string."
```

### 6. Readability

**Check Metrics**:
- [ ] Grade level appropriate for audience
- [ ] Average sentence length <20 words (beginners)
- [ ] Active voice preferred
- [ ] Clear topic sentences

### 7. Structure and Flow

**Check**:
- [ ] Logical organization (simple → complex)
- [ ] Clear section headings
- [ ] Smooth transitions between ideas
- [ ] No sudden topic jumps

### 8. Error Prevention

**Check**:
- [ ] Common mistakes mentioned
- [ ] Edge cases discussed
- [ ] Pitfalls warned against

**Example**:
```
✅ "Common mistake: Modifying a list while iterating over it can skip
elements. Instead, create a new list or iterate over a copy."
```

### 10. Verification Opportunities

**Check**:
- [ ] Learner can test understanding
- [ ] Expected outputs shown
- [ ] Checkpoints for self-assessment

## Systematic Review Process

### Pass 1: Jargon Audit
Highlight every technical term. Is it defined or familiar?

### Pass 2: Assumption Audit
List every assumed prerequisite. Are they explicitly stated?

### Pass 3: Completeness Audit
Can learner complete task with only information provided?

### Pass 4: Readability Audit
Check sentence length, vocabulary complexity, structure.

### Pass 5: Gatekeeping Audit
Search for "obviously", "simply", "just", "clearly", etc.

## Quick Fixes

### Too Dense
**Problem**: Paragraph introduces 5+ new concepts

**Fix**: Break into multiple paragraphs, each with 1-2 concepts

### Too Vague
**Problem**: "Do the necessary steps"

**Fix**: List specific steps numbered 1, 2, 3

### Too Abstract
**Problem**: Only general descriptions

**Fix**: Add concrete code examples

### Missing Context
**Problem**: Jumps into details without overview

**Fix**: Add introductory paragraph explaining purpose

## Clarity Scoring

Rate each dimension 1-5:
1. Major clarity issues
2. Significant problems
3. Acceptable with minor issues
4. Good clarity
5. Excellent clarity

**Overall Target**: Average ≥4 for instructional content

## Red Flags

Immediate attention needed if:
- [ ] Uses gatekeeping language (obviously, simply, etc.)
- [ ] Defines no technical terms
- [ ] No code examples for programming concept
- [ ] Grade level 5+ levels above target audience
- [ ] No explanation of "why", only "what"

## Further Reading

- "Docs for Developers" (Blegen & Sloyer)
- Google Developer Documentation Style Guide
- Microsoft Writing Style Guide

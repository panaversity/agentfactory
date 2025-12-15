---
description: Systematically fact-check educational lesson content for accuracy, removing hallucinations and verifying all claims against authoritative sources. Applies Part 1 fact-checking protocol.
argument-hint: <lesson-file.md>
---

# /fact-check-lesson

## Purpose
Verify factual accuracy of lesson content by identifying and correcting fabricated examples, unverifiable statistics, outdated data, and misleading claims. Ensures all numbers, dates, names, and sources are verifiable against authoritative sources.

## Contract
**Inputs:** `$1` — Path to lesson markdown file (e.g., `apps/learn-app/docs/.../01-lesson.md`)
**Outputs:** `STATUS=<VERIFIED|CORRECTED|FAIL> ISSUES=<count> CRITICAL=<count>`

## Instructions

Follow the systematic 4-phase fact-checking protocol from `.specify/prompts/part-1-fact-checking-protocol.md`:

### Phase 1: Identification (Scan for Red Flags)

1. **Read the complete lesson file**
2. **Scan for high-risk claim categories:**
   - Personal stories with names (e.g., "Sarah Chen built...")
   - Specific statistics (percentages, dollar amounts, user counts)
   - Company data (revenue, employee counts, growth metrics)
   - Dates and timelines (product launches, events, surveys)
   - Quotes attributed to named individuals
   - Historical comparisons and precedents

3. **Flag each claim for verification:**
   - Mark fabricated-looking stories (generic names, too-perfect scenarios)
   - Mark statistics without clear attribution
   - Mark dates that seem approximate ("early 2025" vs "March 2025")
   - Mark quotes without verifiable sources

### Phase 2: Verification (Check Against Sources)

4. **Establish source hierarchy:**
   - **PRIMARY** (use these): Official company announcements, press releases, verified surveys (Stack Overflow, DORA), government data, academic research
   - **SECONDARY** (verify carefully): TechCrunch, Bloomberg, WSJ, Reuters with named sources
   - **TERTIARY** (avoid): Blog posts, Medium articles, Reddit, unattributed claims

5. **For each flagged claim:**
   - Search for authoritative source (PRIMARY preferred)
   - Verify exact numbers match (not "approximately")
   - Verify dates are precise (not estimates)
   - Verify names and attributions are correct
   - Document source with URL or citation

6. **Common verification patterns:**
   - Statistics: Check original survey/report, verify methodology
   - Company data: Check official announcements, SEC filings, press releases
   - Dates: Cross-reference multiple sources, prefer official sources
   - Quotes: Verify exact wording, check context

### Phase 3: Correction (Fix Issues)

7. **For fabricated content (fictional characters, made-up stories):**
   - **REMOVE entirely** if no verified replacement exists
   - **REPLACE with verified example** if authoritative alternative found
   - Document what was removed and why

8. **For incorrect statistics:**
   - **UPDATE with correct numbers** from verified source
   - **ADD source attribution** inline (e.g., "Stack Overflow 2025 Developer Survey")
   - If stat is unverifiable: REMOVE

9. **For outdated data:**
   - **UPDATE to 2025 data** where available (e.g., Stack Overflow 2024 → 2025)
   - If 2025 data unavailable: Keep 2024 but note "as of 2024"
   - Document what was updated

10. **For misleading claims:**
    - **ADD nuance/context** (e.g., "solo developer" → "small team at larger company")
    - **CLARIFY attribution** (e.g., "Claude Code" → "Claude Code (Anthropic product)")
    - **FIX oversimplifications** that could mislead readers

### Phase 4: Reporting (Document Changes)

11. **Create fact-check report** in `history/fact-checking/lesson-{N}-fact-check-report.md`:

```markdown
# Fact-Check Report - Chapter {N}, Lesson {M}

**Date**: {today}
**Lesson**: {lesson-title}
**Status**: {VERIFIED / CORRECTED}
**Total Issues**: {count}
**Critical Issues**: {count}

---

## Issues Found and Corrected

### CRITICAL ISSUE 1: {Title}
- **Location**: Line {N}
- **Original claim**: "{exact text}"
- **Problem**: {Fabricated / Unverifiable / Outdated / Misleading}
- **Correction**: {What was changed}
- **Source**: {Verified source with URL}
- **Impact**: {Why this mattered}

### MAJOR ISSUE 2: {Title}
[Same format]

### MINOR ISSUE 3: {Title}
[Same format]

---

## Verification Summary

**Claims Verified**: {count}
**Claims Corrected**: {count}
**Claims Removed**: {count}
**Sources Added**: {count}

**Source Quality**:
- Primary sources: {count}
- Secondary sources: {count}
- Tertiary sources: {count} (flagged for review)

---

## Recommendations

{Any suggestions for future lessons, patterns to watch, or systematic issues}

---

**Lesson status**: ✅ Fact-checked and corrected
**Ready for**: Visual asset planning (next workflow phase)
```

12. **Update lesson markdown file** with all corrections applied

13. **Output final status:**
    - Print summary: `STATUS=CORRECTED ISSUES={total} CRITICAL={count}`
    - List all changes made
    - Confirm report created

## Verification Checklist

Before marking lesson as fact-checked, verify:

- [ ] All personal stories with names are verified OR removed
- [ ] All statistics have authoritative sources
- [ ] All dates are precise and verified
- [ ] All company data is from official sources
- [ ] All quotes are attributed with context
- [ ] No "approximately" or "around" for verifiable numbers
- [ ] Survey data is from 2025 where available (not 2024)
- [ ] No fictional characters presented as real people
- [ ] No misleading attributions (solo dev vs company product)
- [ ] Fact-check report created in history/

## Examples

### Example 1: Fabricated Character

**Original** (Line 58):
```markdown
Sarah Chen, a Seattle developer, built a $50M company in 6 months...
```

**Problem**: Fictional character presented as real person

**Correction**:
```markdown
In March 2025, Y Combinator managing partner Jared Friedman revealed that 25% of startups in their Winter 2025 batch had codebases that were approximately 95% AI-generated...
```

**Source**: TechCrunch, March 6, 2025 (verified)

---

### Example 2: Outdated Statistic

**Original**:
```markdown
76% of developers use AI tools (Stack Overflow 2024)
```

**Problem**: 2024 data exists, but 2025 survey is available

**Correction**:
```markdown
84% of developers are using or plan to use AI coding tools, with 51% using them daily (Stack Overflow 2025 Developer Survey)
```

**Source**: Stack Overflow 2025 Developer Survey

---

### Example 3: Misleading Attribution

**Original**:
```markdown
A solo developer generated $500M in revenue in 2 months with Claude Code.
```

**Problem**: Claude Code is Anthropic's product (100+ employees), not a solo dev company

**Correction**:
```markdown
Claude Code (Anthropic's product) reached $500 million annualized revenue within two months of launch, making it one of the fastest-growing products in startup history.
```

**Source**: Multiple tech news sources, Anthropic announcements

---

## Common Patterns to Watch

**Red Flag Patterns**:
- Generic Western names (Sarah, John, Michael) with tech success stories
- Round numbers without sources ($1M, $10M, 100K users)
- "Recent study shows..." without naming the study
- "Experts say..." without naming experts
- Statistics from "2024" when 2025 data exists

**Green Flag Patterns**:
- Named individuals with verifiable positions (Jared Friedman, YC Managing Partner)
- Official company announcements (press releases, blog posts)
- Specific survey names with years (Stack Overflow 2025, DORA 2024)
- Precise dates (March 2025, September 4, 2025)
- Multiple corroborating sources

## Error Handling

**If lesson file not found**:
```
STATUS=FAIL ERROR="Lesson file not found: {path}"
```

**If unable to verify critical claim**:
- Flag in report as "NEEDS MANUAL REVIEW"
- Do NOT remove if it's central to lesson thesis
- Document why verification failed
- Suggest alternative sources to check

**If too many critical issues (>10)**:
```
STATUS=FAIL ERROR="Too many critical issues ({count}). Lesson may need major revision."
```

## Constraints

- Prioritize accuracy over narrative flow (remove compelling-but-false stories)
- Use PRIMARY sources wherever possible
- When in doubt, REMOVE rather than keep unverifiable claims
- Never guess or approximate verifiable data
- Document ALL changes in fact-check report

---

**Reference Protocol**: `.specify/prompts/part-1-fact-checking-protocol.md`

**After completion**: Lesson is ready for visual asset workflow (`visual-asset-workflow` skill)

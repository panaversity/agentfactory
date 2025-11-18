# Constitution Updated: v6.0.1 — Meta-Commentary Prohibition

**Date**: 2025-11-18
**Amendment Type**: PATCH (Constitutional enhancement, not breaking change)
**Trigger**: Chapter 9 redesign exposed Three Roles scaffolding through meta-commentary

---

## Problem Identified

During Chapter 9 implementation, Lessons 2 and 3 violated constitutional Student-Facing Language Protocol by exposing pedagogical scaffolding through meta-commentary:

**Violations**:
- "What to notice: AI is teaching you patterns"
- "AI learned from you: [bullet list]"
- "AI adapted to your domain requirements"
- "This is AI as Co-Worker: Convergence through iteration"

**Impact**: Students saw internal framework labels instead of experiencing collaboration naturally, breaking immersion and adding unnecessary cognitive load.

---

## Constitutional Amendment Added

### Location
`.specify/memory/constitution.md`
- **Section**: II. Student-Facing Language Protocol
- **New Subsection**: "CRITICAL: Meta-Commentary Prohibition (Scaffolding Exposure)"
- **Lines Added**: ~80 lines of comprehensive guidance

### What Was Added

**1. Forbidden Meta-Commentary Patterns** (Explicit Examples)

❌ "What to notice" explanations
❌ Explicit framework explanations ("This is AI as Teacher")
❌ "AI learned/knows" meta-commentary
❌ Passive Q&A with explanatory commentary

**2. Correct Pattern Template** (5-Part Active Collaboration)

```markdown
## Try With AI: [Challenge Name]

**Part 1: Initial Request**
Ask AI: "[specific prompt]"

**Part 2: Critical Evaluation**
Review AI's response. Ask yourself:
- Does this match my requirements?
- Which suggestions add unnecessary complexity?

**Part 3: Constraint Teaching**
Tell AI your constraints: "[specific constraints]"

**Part 4: Refinement**
Ask AI to validate: "[validation prompt]"

**Part 5: Final Check**
Compare original to final:
- What improved?
- What did you reject?
- Is result better? Why?
```

**3. Narrative Example Guidance**

❌ WRONG: "What AI learned from you: [bullets]"
✅ CORRECT: "What emerged from iteration: [bullets]"

**4. Enhanced Validation Grep Patterns**

```bash
grep -i "What to notice\|What to expect\|AI.*teach\|AI.*learn\|AI as\|AI now knows" [files]
```

**5. Acceptable vs Forbidden Distinction**

- ✅ Acceptable: Section headers ("Part 3: Constraint Teaching")
- ✅ Acceptable: Reflection questions ("What did AI assume?")
- ❌ Forbidden: Meta-commentary ("AI is teaching you")
- ❌ Forbidden: Framework labels ("AI as Teacher")

---

## Version Update

**Previous**: v6.0.0 (MAJOR — Reasoning Activation Redesign)
**Current**: v6.0.1 (PATCH — Meta-Commentary Prohibition)

**Changelog Entry**:
```
v6.0.1 (PATCH — Meta-Commentary Prohibition) — 2025-11-18
Rationale: Prevent scaffolding exposure in "Try With AI" sections following Chapter 9 violation
WHAT CHANGED:
- Added comprehensive "Meta-Commentary Prohibition" section to Student-Facing Language Protocol
- Explicit forbidden patterns: "What to notice", "AI is teaching you", "AI learned from you"
- Added correct pattern template for "Try With AI" sections (5-part active collaboration)
- Enhanced validation grep patterns to catch meta-commentary
WHAT'S NEW:
- Complete "Try With AI" template using action prompts + self-reflection (no framework labels)
- Distinction between acceptable activity names vs forbidden meta-commentary
- Narrative example guidance: "What emerged" (CORRECT) vs "What AI learned" (WRONG)
MIGRATION IMPACT: PATCH
- Content-implementer must use new "Try With AI" template
- All existing lessons must be validated for meta-commentary patterns
- Validation-auditor checklist updated with meta-commentary grep checks
SIZE: ~1180 lines (+80 lines for meta-commentary section)
Trigger: Chapter 9 redesign exposed Three Roles framework through "What to notice" commentary
```

---

## Impact on Agents

### content-implementer
**MUST NOW**:
- Use 5-part active collaboration template for ALL "Try With AI" sections
- Replace "What to notice" with self-reflection questions
- Focus narrative examples on "what changed" not "who taught whom"
- Run meta-commentary grep validation before completion

### validation-auditor
**MUST NOW**:
- Add meta-commentary grep check to validation workflow
- Flag any "What to notice", "AI is teaching", "AI learned" patterns
- Validate "Try With AI" sections use action prompts (not passive Q&A)
- Ensure narrative examples avoid framework exposition

### chapter-planner
**MUST NOW**:
- Plan "Try With AI" sections using 5-part structure
- Specify reflection questions (not meta-commentary) in lesson outlines
- Document Three Roles through activities (not labels) in planning docs

---

## Migration Path for Existing Content

**Immediate Action Required**:
1. ✅ Chapter 9: FIXED (Lessons 2 and 3 corrected)
2. ⏳ Chapters 1-8: VALIDATE for meta-commentary patterns
3. ⏳ Chapters 10-11: VALIDATE before publication
4. ⏳ Part 4 (Python chapters): VALIDATE during creation

**Validation Command**:
```bash
grep -rn -i "What to notice\|What to expect\|AI.*teach\|AI.*learn\|AI as Teacher\|AI as Student\|AI as Co-Worker\|AI now knows\|AI adapted" book-source/docs/**/*.md
```

**Fix Pattern**:
- Remove "What to notice" commentary
- Replace with self-reflection questions
- Change "AI learned X" to "X emerged from iteration"
- Rewrite passive Q&A as 5-part active collaboration

---

## Validation Checklist (All Future Lessons)

Before publishing ANY lesson with "Try With AI" section:

- [ ] No "What to notice" meta-commentary
- [ ] No "What to expect" scaffolding
- [ ] No "AI is teaching you" phrases
- [ ] No "AI learned from you" phrases
- [ ] No "AI as Teacher/Student/Co-Worker" labels
- [ ] Uses action prompts (not passive Q&A)
- [ ] Uses self-reflection questions (not meta-explanations)
- [ ] Narrative examples focus on "what emerged" not "who taught"
- [ ] Run grep validation: `grep -i "What to notice\|AI.*teach\|AI.*learn" [file]`
- [ ] Zero matches for forbidden patterns

---

## Key Takeaway

**Before Amendment**:
- Constitution prohibited framework labels ("Stage 2", "Three Roles")
- Did NOT explicitly prohibit meta-commentary ("What to notice: AI is teaching you")
- content-implementer exposed scaffolding through explanatory commentary

**After Amendment**:
- Explicit prohibition of ALL meta-commentary patterns
- Complete template for correct "Try With AI" structure
- Enhanced grep validation patterns
- Clear examples of forbidden vs acceptable language

**Result**: Future content will EXPERIENCE Three Roles collaboration without seeing framework labels OR meta-commentary explaining the framework.

---

## Files Modified

1. `.specify/memory/constitution.md`
   - Version: 6.0.0 → 6.0.1
   - Added: "Meta-Commentary Prohibition" subsection (~80 lines)
   - Updated: Changelog with v6.0.1 entry
   - Updated: Last Amended date to 2025-11-18

2. `specs/034-chapter-9-markdown-redesign/SCAFFOLDING-FIX-REPORT.md`
   - Documents the specific violations found
   - Shows before/after examples
   - Provides pattern for future content

3. `specs/034-chapter-9-markdown-redesign/CONSTITUTION-UPDATE.md` (this file)
   - Comprehensive amendment documentation
   - Migration guidance for existing content
   - Validation checklist for future content

---

**Status**: ✅ Constitution amended, Chapter 9 fixed, validation enhanced
**Next Step**: Validate Chapters 1-11 for meta-commentary violations

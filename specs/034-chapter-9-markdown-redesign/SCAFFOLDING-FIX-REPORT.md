# Scaffolding Exposure Fix Report

**Issue**: Constitutional Violation - Three Roles Framework Scaffolding Exposed to Students
**Date**: 2025-11-18
**Status**: ✅ FIXED

---

## Problem Identified

Lessons 2 and 3 contained explicit Three Roles framework labels in student-facing content, violating constitutional requirement that students should EXPERIENCE collaboration without seeing internal scaffolding.

### Violations Found

**Lesson 2 (02-lists-for-requirements.md)**:
- ❌ "This is AI teaching you patterns"
- ❌ "You taught it your MVP scope. AI learned from you"
- ❌ "This is AI as co-worker: Convergence through iteration"
- ❌ "What AI learned from you: [bullet list]"

**Lesson 3 (03-code-blocks-for-specifications.md)**:
- ❌ "AI is teaching you specification thoroughness"
- ❌ "AI adapted to your MVP scope. You taught it your constraints. AI learned your priorities"
- ❌ "What AI learned from you: [bullet list]"
- ❌ "AI now knows the precise output format"

---

## Correct Pattern (Per User Guidance)

**❌ WRONG (exposing scaffolding)**:
```markdown
**What to notice**: AI might suggest requirements you didn't think of.
This is AI teaching you patterns.
```

**✅ CORRECT (effects without labels)**:
```markdown
**Part 2: Critical Evaluation**

Review AI's response. Ask yourself:
- Does this match my MVP scope?
- Are there features too complex for v1?
- What assumptions did AI make about my project?
```

**Key Principle**: Students EXPERIENCE bidirectional learning through action prompts and reflection questions, NOT through explicit framework explanations.

---

## Fixes Applied

### Lesson 2: Try With AI Section

**Before**:
- Passive Q&A with scaffolding commentary
- "What to notice" meta-explanations
- Explicit "AI teaching you", "You taught AI" labels

**After**:
- Active collaboration with 5-part structure
- Self-reflection questions
- Validation prompts without framework labels

**Structure**:
1. **Part 1**: Initial Structure Request (student asks AI)
2. **Part 2**: Critical Evaluation (student evaluates AI output)
3. **Part 3**: Constraint Teaching (student provides feedback to AI)
4. **Part 4**: Edge Case Discovery (student prompts AI to think deeper)
5. **Part 5**: Validation (student compares before/after)

### Lesson 2: Narrative Examples

**Before**:
```markdown
**What AI learned from you:**
- MVP constraint (simple scope)
- Desktop-first approach (no mobile yet)

AI now knows your priorities and won't suggest complex features.
```

**After**:
```markdown
**What emerged from your feedback:**
- MVP constraint clarified (simple scope only)
- Desktop-first approach specified
- Future features explicitly deferred
```

### Lesson 3: Try With AI Section

**Before**:
- Meta-commentary explaining Three Roles
- "AI is teaching you...", "You taught AI..." phrases
- "What to notice" scaffolding

**After**:
- 5-part active collaboration structure
- Critical review prompts
- Clarity validation questions

**Structure**:
1. **Part 1**: Initial Precision Request
2. **Part 2**: Critical Review (student evaluates suggestions)
3. **Part 3**: Constraint Communication (student teaches AI requirements)
4. **Part 4**: Clarity Validation (student tests implementability)
5. **Part 5**: Final Check (student compares before/after)

### Lesson 3: Narrative Examples

**Before**:
```markdown
**What AI learned from you**:
- Users care about TODAY's tasks
- Overdue tasks need highlighting

AI adapted to your domain requirements.
```

**After**:
```markdown
**What changed based on your feedback**:
- Focus shifted to TODAY's tasks
- Overdue tasks now highlighted
- Daily breakdown emphasized
```

---

## Validation Results

### Grep Check for Scaffolding Patterns

**Command**:
```bash
grep -in "AI.*teach\|AI.*learn\|teach.*AI\|AI as\|role 1\|role 2\|role 3\|What to notice\|What to expect" 02-lists-for-requirements.md 03-code-blocks-for-specifications.md
```

**Results**: ✅ PASS
- No "AI as Teacher/Student/Co-Worker" labels found
- No "What to notice" meta-commentary found
- No "AI is teaching you" phrases found
- No "AI learned from you" phrases found

**Acceptable matches**:
- "Constraint Teaching" - section header describing activity (not framework label)
- "What did AI assume" - reflection question (not scaffolding explanation)

### Constitutional Compliance

- ✅ **Students experience collaboration** through prompts and reflection
- ✅ **No framework labels** visible in student-facing text
- ✅ **Active prompts** replace passive Q&A
- ✅ **Validation questions** replace meta-commentary
- ✅ **Effects without labels** pattern implemented throughout

---

## Files Modified

1. `/book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/02-lists-for-requirements.md`
   - Rewrote "Try With AI" section (lines 549-607)
   - Removed scaffolding from narrative examples (lines 389-394, 426-430)

2. `/book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/03-code-blocks-for-specifications.md`
   - Rewrote "Try With AI" section (lines 690-746)
   - Removed scaffolding from narrative examples (lines 115-121, 340-344)

---

## Pattern for Future Content

### Use This (Active Collaboration)

```markdown
## Try With AI: [Challenge Name]

**Part 1: Initial Request**
Ask AI: "[specific prompt]"

**Part 2: Critical Evaluation**
Review AI's response. Ask yourself:
- [Evaluation question 1]
- [Evaluation question 2]
- [Evaluation question 3]

**Part 3: Constraint Teaching**
Tell AI your requirements: "[specific constraints]"

**Part 4: Refinement**
Ask AI: "[validation prompt]"

**Part 5: Validation**
Compare before/after:
- [Reflection question 1]
- [Reflection question 2]
- [Outcome question]
```

### NOT This (Scaffolding Exposure)

```markdown
❌ "What to notice: AI is teaching you..."
❌ "This is AI as Teacher/Student/Co-Worker"
❌ "What AI learned from you: ..."
❌ "AI now knows..."
```

---

## Impact

**Before Fix**:
- Students saw explicit Three Roles labels
- Meta-commentary explained the framework
- Passive learning experience ("notice this")

**After Fix**:
- Students experience collaboration naturally
- Self-reflection prompts guide discovery
- Active learning experience ("evaluate, teach, validate")

**Pedagogical Improvement**:
- Students develop critical thinking (evaluating AI output)
- Students practice constraint communication (teaching AI)
- Students validate outcomes (iteration awareness)
- Framework effects achieved WITHOUT exposing scaffolding

---

## Validation Checklist for Future Lessons

Before publishing any lesson with "Try With AI" sections:

- [ ] No "AI as Teacher/Student/Co-Worker" labels
- [ ] No "What to notice" meta-commentary
- [ ] No "AI is teaching you" phrases
- [ ] No "AI learned from you" phrases
- [ ] No "AI now knows" phrases
- [ ] Prompts are action-oriented (not passive)
- [ ] Questions are self-reflection (not explanatory)
- [ ] Validation is through student comparison (not meta-commentary)
- [ ] Run grep check: `grep -i "AI.*teach\|AI.*learn\|AI as\|What to notice" [lesson-file]`

---

**Status**: ✅ Constitutional compliance restored
**Verification**: Grep validation passed
**Lessons Fixed**: 2 (Lesson 2, Lesson 3)
**Pattern Documented**: Yes (for future content creation)

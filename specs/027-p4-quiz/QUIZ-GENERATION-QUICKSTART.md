# Python Chapter Quiz Generation - Quick Start Guide

**Issue**: [#268 - No Quiz at the end of the Python chapters](https://github.com/panaversity/tutorsgpt/issues/268)

**Solution**: Use `quiz-generator` skill with reasoning-activated prompt guidance

---

## Quick Commands

### Generate a quiz for any Python chapter:

```bash
# Step 1: Invoke the quiz-generator skill
/quiz-generator

# OR via Skill tool
skill: quiz-generator

# Step 2: Tell the skill to follow guidance from:
prompts/quiz-generation-python-chapters.md

# Step 3: Specify the chapter
"Generate quiz for Chapter [N]: [Chapter Title]"
```

### After quiz generation, run redistribution:

```bash
python .claude/skills/quiz-generator/scripts/redistribute_answers_v2.py \
  book-source/docs/04-Python-Fundamentals/[chapter-folder]/[NN]_chapter_[NN]_quiz.md \
  [A-H]
```

**Sequence rotation** (to vary answer patterns):
- Chapter 13 → A
- Chapter 14 → B
- Chapter 15 → C
- Chapter 16 → D
- Chapter 17 → E
- Chapter 18 → F
- Chapter 19 → G
- Chapter 20 → H
- Chapter 21 → A (restart rotation)
- (etc.)

---

## What Gets Generated

✅ **50 interactive questions** (not 10-15 static ones)
✅ **Quiz component format** (`<Quiz />` - globally registered)
✅ **Randomized batching** (15-20 questions displayed per session)
✅ **Immediate feedback** (after each question, not at end)
✅ **All options within ±3 words** (prevents length-based guessing)
✅ **Even answer distribution** (12-13 per index 0-3)
✅ **File naming**: `06_chapter_18_quiz.md` (or appropriate lesson number)

---

## Question Distribution (50 questions)

| Category | Count | Purpose |
|----------|-------|---------|
| **By Type** | | |
| Scenario-based | 20 | Real development situations |
| Code review | 15 | Evaluating AI-generated code |
| Concept understanding | 10 | Deep Python semantics |
| AI workflow | 5 | Collaboration competence |
| **By Bloom Level** | | |
| Remember | 10 | Basic recall/recognition |
| Understand | 12 | Explanation/interpretation |
| Apply | 15 | Scenario-based usage |
| Analyze | 8 | Code review/comparison |
| Evaluate | 5 | Quality assessment |
| **By Difficulty** | | |
| Easy | 10 | Foundational concepts |
| Medium | 30 | Core chapter content |
| Hard | 10 | Advanced applications |

---

## Validation Checklist

Before considering quiz complete:

- [ ] Exactly 50 questions generated
- [ ] All options within ±3 words per question (manually verified)
- [ ] All lessons from chapter covered (~8-10 questions per lesson)
- [ ] Quiz component format (NOT static markdown)
- [ ] Explanations reference specific lesson numbers
- [ ] At least 5 questions test AI collaboration workflow
- [ ] Production-relevant scenarios (no toy examples)
- [ ] Answer redistribution script executed successfully
- [ ] Script reports even distribution (12-13 per index)

---

## Example: Chapter 18 (Control Flow and Loops)

```bash
# 1. Invoke skill
/quiz-generator

# 2. Generate quiz following prompt guidance
"Generate quiz for Chapter 18: Control Flow and Loops
Follow guidance from: prompts/quiz-generation-python-chapters.md"

# 3. After generation, run redistribution
python .claude/skills/quiz-generator/scripts/redistribute_answers_v2.py \
  book-source/docs/04-Python-Fundamentals/18-control-flow-loops/06_chapter_18_quiz.md \
  F

# 4. Verify output
✅ Quiz generated successfully with 50 questions
✅ Answer distribution: 13-0, 12-1, 13-2, 12-3
✅ All explanations validated
✅ Ready for student use
```

---

## Files & References

### Main Prompt
- **prompts/quiz-generation-python-chapters.md** (v2.0)
  - Reasoning activation framework
  - Python-specific guidance
  - AI-native learning alignment
  - Chapter-specific strategies

### Skill
- **.claude/skills/quiz-generator/SKILL.md** (v5.0.0)
  - Quiz component requirements
  - 50-question format
  - Answer redistribution
  - Option length validation

### Example Quizzes
- **Part 1**: `book-source/docs/01-Introducing-AI-Driven-Development/*/09_chapter_*_quiz.md`
  - Shows quality standard
  - Question structure examples
  - Explanation format

---

## Troubleshooting

### Issue: Generated quiz has only 10-15 questions
**Fix**: Make sure you invoked `quiz-generator` skill first (not just referencing the prompt directly)

### Issue: Quiz is static markdown, not Quiz component
**Fix**: The quiz-generator skill automatically uses Quiz component - ensure skill was invoked

### Issue: Options have wildly different lengths
**Fix**: Before running redistribution, manually check and fix option lengths (all within ±3 words)

### Issue: Answer distribution is uneven
**Fix**: This is normal before redistribution - the script fixes it automatically

### Issue: Redistribution script fails
**Fix**: Check file path is correct and file uses proper Quiz component format

---

## Batch Generation Strategy

To generate quizzes for all Python chapters (13-30):

1. **Start with early chapters** (13-17) to validate process
2. **Review quality** after first 2-3 chapters
3. **Iterate prompt** if needed based on output quality
4. **Continue systematically** through remaining chapters (18-30)
5. **Vary sequence letters** to avoid answer pattern repetition across chapters

**Estimated time**: ~30-45 minutes per chapter (including validation)

---

## Quality Standards

Questions should:
- ✅ Test understanding, not memorization
- ✅ Use production-relevant scenarios (not toy examples)
- ✅ Have plausible distractors (represent real misconceptions)
- ✅ Provide explanations that teach (even when wrong)
- ✅ Align with AI-native learning philosophy
- ✅ Connect to specific lesson content
- ✅ Activate reasoning mode (not just pattern matching)

Questions should NOT:
- ❌ Test pure syntax recall
- ❌ Use trick questions or gotchas
- ❌ Have obviously wrong options
- ❌ Lack connection to real development
- ❌ Ignore AI collaboration workflow
- ❌ Be answerable by option length patterns

---

## Support

- **Issue**: https://github.com/panaversity/tutorsgpt/issues/268
- **Prompt History**: `history/prompts/general/0160-quiz-generation-python-chapters.general.prompt.md`
- **Constitution**: `.specify/memory/constitution.md`
- **Papers**: `papers/` directory (SDD-RI, Reasoning Activation, Skills Thinking)

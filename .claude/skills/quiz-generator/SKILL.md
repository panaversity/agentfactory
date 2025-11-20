---
name: quiz-generator
description: |
  ALWAYS generate interactive quizzes using the Quiz component (50 comprehensive questions total).
  Generates 50 college-level conceptual questions with immediate feedback per question.
  Quiz component automatically displays 15-20 questions per batch, randomized each retake.
  Features: immediate feedback after each answer (correct option + explanation + why wrong if incorrect),
  automatic batch shuffling on retake, no passing/failing threshold, color-coded feedback,
  theme support. Globally-registered Quiz component handles all UI/UX.
  Follows ##_chapter_##_quiz.md naming convention.
  INCLUDES: Automated answer redistribution with intelligent explanation regeneration (consolidates quiz-answer-redistributor functionality).
version: 5.0.0
allowed-tools: [Bash]
replaces: quiz-answer-redistributor (v3.0.0)
---

# Quiz Generator: College-Level Interactive Chapter Assessments

**Version:** 5.0.0 | **Consolidates:** quiz-answer-redistributor v3.0.0 | **Alignment:** Constitution v3.1.2, Co-Learning Partnership, "Specs Are the New Syntax" | **KEY FEATURES:** 50 total questions, automated answer redistribution with intelligent explanation regeneration, 15-20 displayed per batch, immediate feedback per question, randomized batching on retake, no pass/fail threshold, evals-aligned questions

**Quiz Component Location:** `book-source/src/components/Quiz.tsx` (globally registered—no imports needed)
**Usage Reference:** `book-source/src/components/QUIZ_USAGE.md` (example structure + best practices)
**Example Quiz:** `book-source/src/components/references/example-quiz.md` (full working example with all patterns)

---

## Purpose

Generate high-quality, college-level MCQ quizzes (50 comprehensive questions total) using the globally-registered Quiz component (`<Quiz />`). The component automatically:
- Displays 15-20 random questions per session (shuffled from the 50 total)
- Shows immediate feedback after each answer (correct option + explanation + why wrong if incorrect)
- Shuffles questions differently on each retake (randomized batching from the same 50)
- Provides progress tracking, color-coded feedback, and detailed review page
- NO passing/failing threshold—just tracks score and learning

**Output is ALWAYS a Quiz component with 50 questions, NEVER static markdown quizzes.**

**Core Principles:**
- Test understanding and application, not memorization
- **50 comprehensive questions** ensures thorough chapter coverage
- **Immediate feedback per question** enables real-time learning (not delayed until results page)
- **Randomized batching** (15-20 per session) keeps students engaged across multiple attempts
- Provide meaningful explanations addressing all misconceptions
- **No pass/fail threshold**—focus on learning, not grading

---

## When to Activate

This skill should be used when creating end-of-chapter assessments. **CRITICAL: This skill ALWAYS generates interactive quizzes using the Quiz component—NEVER static markdown quizzes. ALWAYS 50 QUESTIONS TOTAL.**

Activate this skill when:
- Creating end-of-chapter assessments that MUST use the Quiz component
- Need college-level conceptual questions (50 comprehensive questions total)
- Want interactive quiz with immediate feedback per question (not delayed until results)
- Want randomized batch display (15-20 questions per session, different batches on retake)
- Want instant explanations showing correct option AND why wrong answers are incorrect
- Need fully responsive, accessible quiz with light/dark theme support
- Creating MDX-compatible quiz files using `<Quiz />` component

**Trigger phrases:**
- "Create a quiz for Chapter X"
- "Generate a 50-question quiz for Chapter X"
- "Build an interactive quiz with immediate feedback"
- "Create chapter assessment with randomized batching"
- "Generate college-level assessment with 50 questions"

**MANDATORY REQUIREMENTS:**
1. Use Quiz component (NEVER static markdown)
2. Generate 50 total questions (comprehensive chapter coverage)
3. Component displays 15-20 per batch (randomized each retake)
4. Randomize correctOption indices evenly across 0-3
5. Explanations MUST address why wrong answers are incorrect
6. NO passing score threshold—just score tracking
7. Immediate feedback shown after each answer
8. **🚨 CRITICAL: ALL options within ±3 words of each other per question** (prevents test-takers from guessing by selecting longest/shortest option)
9. **🚨 MANDATORY VALIDATION: Manually count words for EVERY option in EVERY question (all 50 questions)** before submitting quiz
10. **🚨 CRITICAL: Verify correct answer is NOT correlated with option length** (longest option must NOT be more frequently correct)

---

## Key Concepts

### College-Level Conceptual Questions

**Shift from Recall to Understanding:**

❌ **Recall (Avoid):**
> "What is a Python list?" → Tests memorization

✅ **Conceptual (Target):**
> "Given this code with list operations, what misconception does this error reveal?" → Tests understanding

**Why College-Level?**
1. Job readiness: Professional developers need to understand WHY, not just WHAT
2. AI-era skills: With AI generating code, understanding concepts matters more than syntax recall
3. Transfer learning: Conceptual knowledge transfers; memorized facts don't
4. Better assessment: Conceptual questions predict programming competence

**Cognitive Level Target:**
- 75%+ questions at Apply level or higher
- Focus on debugging, prediction, analysis (not recall)

---

### Immediate Feedback Learning Model

**Quiz component shows feedback IMMEDIATELY after each answer** (not delayed until results page). This enables:

1. **Real-time learning:** Students learn from mistakes instantly, not after quiz completion
2. **Deeper engagement:** Immediate reinforcement = higher retention
3. **Misconception correction:** Explain WHY their wrong answer was incorrect right away
4. **Better explanations:** Space for 100-200 word explanations per question (comprehensive, not rushed)

**Feedback shows:**
- ✅ **Correct answer highlight** (green background on correct option)
- ✅ **Why your answer was wrong** (if incorrect: "You selected X, but this is incorrect because...")
- ✅ **Correct option named** (The correct answer is: Y)
- ✅ **Full explanation** (80-150 words explaining concept, addressing misconceptions, real-world connection)

**Example feedback structure:**
```
✓ CORRECT!
[Explanation of why this is right...]

vs.

✗ INCORRECT
Why your answer was wrong:
You selected: "append() is immutable"
This is incorrect. The correct answer is: "append() modifies the list in place"

Explanation:
Lists are mutable (changeable) in Python. The append() method modifies the original list
in place and returns None (doesn't return a new list). This is different from strings,
which are immutable. Understanding mutability is crucial for debugging...
```

---

### 50-Question Bank with Randomized Batching

**Why 50 questions?**
- **Comprehensive coverage:** 50 questions ensure all chapter concepts are thoroughly tested
- **Spaced repetition:** Student can take quiz multiple times, seeing different questions each time
- **Long-term retention:** 50 unique questions = better recall than 15-20 repeated questions

**How batching works:**
1. You provide all 50 questions to the Quiz component
2. Component shuffles all 50 on mount
3. Component displays questions 1-15 (or 1-20) for user to answer
4. User completes batch, sees results page
5. User clicks "Try Another Batch" → Component reshuffles all 50
6. Component displays a NEW random batch of 15-20 (completely different questions)

**Example flow:**
```
Session 1: User sees questions [3, 47, 12, 28, 5, 11, ...] (15-20 questions)
Completes, sees results: 14/18 correct

User clicks "Try Another Batch" →

Session 2: User sees questions [42, 8, 31, 1, 19, 35, ...] (15-20 DIFFERENT questions)
Takes quiz again: 16/19 correct

Session 3: User clicks "Try Another Batch" →

Component shows completely DIFFERENT shuffle: [25, 9, 48, 2, 18, ...]
```

---

### Option Length Validation (CRITICAL - Test Validity)

**THE PROBLEM:** Options of unequal length allow test-takers to guess correctly without reading questions. Example:
- ❌ "Yes" (2 words) vs. "Organizational capabilities create guardrails preventing failures" (6 words) → Students select longest without reading
- ❌ Longest option is correct 80% of the time → Test measures reading strategy, not understanding

**THE SOLUTION:** **ALL options within ±3 words of each other** across all 50 questions.

**Validation Procedure (MANDATORY):**

1. **Count words for EVERY option in EVERY question** (all 50 questions):
   ```
   Question: "Why is X important?"
   A: "It improves performance quickly" (4 words)
   B: "It simplifies code structure" (4 words)
   C: "It prevents common bugs" (4 words)
   D: "It helps developers work together" (5 words) ✓ All within ±3 range (4-5 is acceptable)
   ```

2. **Flag any question failing the ±3 word rule:**
   ```
   ❌ FAIL:
   A: "Yes" (2 words)
   B: "The framework processes requests asynchronously in a single event loop without blocking" (12 words)
   → Difference: 10 words → FAIL (>3 word difference)

   ✅ PASS:
   A: "AI amplifies existing practices" (4 words)
   B: "AI fixes all problems" (4 words)
   C: "AI changes developer roles" (4 words)
   D: "AI prevents errors completely" (4 words)
   → All within 4-4 range → PASS
   ```

3. **Verify length distribution** (no correlation with correctness):
   - Count how many correct answers are in longest option group
   - Count how many correct answers are in shortest option group
   - Should be roughly equal (not "longest is always correct")

4. **Document validation in handoff:**
   ```
   Option Length Validation Complete:
   ✓ All 50 questions checked
   ✓ All options within ±3 word range
   ✓ No length-correctness correlation
   ✓ Longest option correct in 6 questions
   ✓ Shortest option correct in 7 questions
   ✓ Middle-length correct in 12 questions
   ```

**Why ±3 words matters:**
- 4-5 words: Similar cognitive load, hard to distinguish by scanning
- 10+ word difference: Readers start pattern-matching (always pick longest/shortest)
- ±3 rule: Prevents strategic test-taking without reading

📖 **Reference:** [option-length-validation.md](./references/option-length-validation.md) for detailed examples and verification scripts

---

### Answer Randomization with Quiz Component

**Requirements:**
- Correct answers distributed across 0-3 indices (not a/b/c/d)
- For 50 questions: ~12-13 per index (25% each)
- No obvious patterns
- Maximum 2 consecutive same answers

**Quiz Component Format:**
```javascript
{
  question: "Your question?",
  options: ["Option A", "Option B", "Option C", "Option D"],
  correctOption: 2,  // Index 0-3, NOT 1-4!
  explanation: "Why this is correct AND why other options are wrong..."
}
```

**Workflow: Generate → Redistribute → Validate**

LLMs are excellent at content creation but struggle with strict procedural constraints like answer distribution. Therefore, this skill uses a **two-step process**:

**Step 1: Generate Quiz (Content Creation)**
1. Write all 50 questions first (focus on quality, don't worry about correctOption distribution)
2. Use natural answer placement (what makes pedagogical sense)

**Step 2: Automated Redistribution (Procedural Validation)**

After quiz generation, automatically fix answer distribution bias using the bundled Python script:

```bash
python .claude/skills/quiz-generator/scripts/redistribute_answers_v2.py <quiz_file_path> <sequence_letter>
```

**Example:**
```bash
python .claude/skills/quiz-generator/scripts/redistribute_answers_v2.py \
  book-source/docs/04-Python-Fundamentals/14-data-types/quiz.md A
```

**Available Sequences (A-H):**
- **Sequence A**: 13 Index-0, 13 Index-1, 12 Index-2, 12 Index-3
- **Sequence B**: 12 Index-0, 13 Index-1, 13 Index-2, 12 Index-3
- **Sequence C**: 12 Index-0, 12 Index-1, 13 Index-2, 13 Index-3
- **Sequence D**: 13 Index-0, 12 Index-1, 12 Index-2, 13 Index-3
- **Sequence E**: 12 Index-0, 13 Index-1, 12 Index-2, 13 Index-3
- **Sequence F**: 13 Index-0, 12 Index-1, 13 Index-2, 12 Index-3
- **Sequence G**: 12 Index-0, 13 Index-1, 13 Index-2, 12 Index-3 (alternative)
- **Sequence H**: 13 Index-0, 12 Index-1, 12 Index-2, 13 Index-3 (alternative)

**What the Redistributor Does:**
1. Parses quiz questions from MDX
2. Swaps option positions to match the chosen sequence
3. **Intelligently regenerates explanations** to reference new answer positions
4. Validates ALL explanations ensure correctOption and explanation match
5. Reports comprehensive statistics

**Critical Features:**
- **Smart explanation updates**: Only updates explanations that explicitly reference option letters (e.g., "Option B is correct")
- **Preserves context-based explanations**: Leaves explanations untouched if they don't reference specific options
- **Validation**: Checks every question to ensure explanations match the correct answer
- **No annotation markers**: Keeps explanations clean for students (no "[Updated to reflect...]" markers)

**Output Report Example:**
```
Successfully re-distributed quiz using Sequence C

Execution Summary:
* Questions Processed: 50
* Options Swapped: 18
* Explanations Updated: 18
* Validation: ALL CHECKS PASSED

Final Distribution:
* Index 0: 12
* Index 1: 12
* Index 2: 13
* Index 3: 13

All 50 explanations verified. Each explanation correctly references the corresponding correct answer option.
```

📖 **Reference:** [answer-distribution.md](./references/answer-distribution.md) for verification methods

---

### Explanation Quality (Critical for Immediate Feedback)

**Quiz component shows explanations immediately after each answer.** Comprehensive explanations enable deeper learning:

**Good Explanations (100-150 words):**
1. **Explain WHY correct** (2-3 sentences)
2. **Address why EACH distractor is wrong** (1-2 sentences each × 3 distractors = 6-8 sentences)
3. **Add context/examples** (real-world connection, misconception clarification)

**Example:**
```javascript
explanation: "Lists are mutable (changeable) in Python, so append() modifies the original
list in place. This is different from strings, which are immutable—you can't change them
after creation. The extend() method adds multiple items (like appending a list), but
append() adds a single item. Insert() requires both value and position. Understanding
mutability is crucial for debugging variable scope issues and understanding function
side effects. When a function calls append() on a list parameter, it modifies the
original list outside the function—a common source of bugs."
```

---

### Source Field (Lesson Attribution)

**The `source` field links each question to the specific lesson it addresses.** This helps students know which lesson material to review.

**Format:** `"Lesson N: [Lesson Title]"`

**Source Extraction:**
- Extract lesson number from lesson file name (01, 02, 03, etc.)
- Extract lesson title from lesson .md file's title metadata (the `title:` field in YAML frontmatter)
- Quiz is already chapter-specific, so chapter info is redundant

**Examples:**
```javascript
source: "Lesson 1: Understanding Mutability"
source: "Lesson 3: Scope and Closures"
source: "Lesson 2: Unit Test Design"
```

**Display in Feedback:**
- Appears in the feedback section after explanation
- Format: "Source: Lesson 1: Understanding Mutability"
- Italic styling with subtle border separator
- Helps students quickly navigate to the relevant lesson if they want to review

---

## Quiz Architecture

### Fixed Constraints (Non-Negotiable)

```yaml
question_count: 50  # Comprehensive bank (50 total questions)
questions_per_batch: 15-20  # Questions displayed per session (component shuffles)
options_per_question: 4  # Always exactly 4 options
question_format: multiple_choice  # Only MCQ
correct_answer_distribution: random_equal  # indices 0-3 equally distributed (~12-13 per index)
feedback_timing: immediate  # Shown after each answer (not delayed)
passing_score: NONE  # No pass/fail threshold—just score tracking
file_naming: ##_chapter_##_quiz.md  # e.g., 05_chapter_02_quiz.md
output_format: Markdown with Quiz component  # <Quiz {...} questions={[...50 questions...]} />
component_globally_registered: true  # No imports needed
```

**CRITICAL ANTI-PATTERNS:**
- ❌ **Fewer than 50 questions** (comprehensive coverage requires full 50)
- ❌ **Pass/fail threshold** (remove passingScore prop entirely)
- ❌ Using array indices 1-4 (use 0-3!)
- ❌ Questions that test recall instead of understanding
- ❌ Explanations NOT addressing why EACH distractor is wrong
- ❌ No randomization of correctOption values across 50 questions
- ❌ Incomplete explanations (must be 100-150 words addressing all options)
- ❌ File naming like `##_quiz.md` (use `##_chapter_##_quiz.md`)

---

### File Structure (Quiz Component Format)

```markdown
---
sidebar_position: X  # e.g., 05 (lesson count + 1)
title: "Chapter X: [Topic] Quiz"
---

# Chapter X: [Topic] Quiz

Brief introduction (1-2 sentences describing what students will assess).

<Quiz
  title="Chapter X: [Topic] Assessment"
  questions={[
    {
      question: "Question 1 text here? (Conceptual, not recall)",
      options: [
        "Option A (specific text for this concept)",
        "Option B (specific text for this concept)",
        "Option C (specific text for this concept) ← CORRECT",
        "Option D (specific text for this concept)"
      ],
      correctOption: 2,  // Index 0-3 (NOT 1-4!)
      explanation: "COMPREHENSIVE explanation (100-150 words): Explain why C is correct (2-3 sentences).
        Then address why each distractor is wrong: Why A is wrong (1-2 sentences). Why B is wrong (1-2 sentences).
        Why D is wrong (1-2 sentences). Real-world connection or misconception clarification (1-2 sentences).
        Total should be 100-150 words addressing all four options.",
      source: "Lesson 1: Understanding Mutability"
    },
    {
      question: "Question 2 text?",
      options: [
        "Option A (distinct alternative misconception)",
        "Option B (distinct alternative misconception)",
        "Option C (distinct alternative misconception)",
        "Option D (correct answer) ← CORRECT"
      ],
      correctOption: 3,
      explanation: "Full explanation addressing why D is correct and why A, B, C are incorrect...",
      source: "Lesson 2: Reference vs. Value"
    },
    // ... 48 more questions (total: 50 questions)
    // Quiz component will shuffle and display 15-20 per session
  ]}
  questionsPerBatch={18}  // Optional: customize questions per session (default: 15)
/>
```

**Key Requirements (CRITICAL):**
- ✅ Exactly 50 questions total (comprehensive bank for repeated practice)
- ✅ Exactly 4 options per question (no more, no less)
- ✅ `correctOption` uses 0-3 index (NOT 1-4!)
- ✅ Explanations address why correct AND why EACH distractor is wrong (100-150 words)
- ✅ **`source` field REQUIRED for all questions** (format: "Lesson N: [Lesson Title]")
- ✅ Randomized correctOption distribution (~12-13 per index across all 50)
- ✅ No imports needed for `<Quiz />` (globally registered component)
- ✅ NO `passingScore` prop (removed—no pass/fail threshold)
- ✅ Optional `questionsPerBatch` prop (default: 15, can be 15-20)

📖 **Reference:** [file-naming.md](./references/file-naming.md) for naming conventions | [example-quiz.md](./references/example-quiz.md) for complete working example

---

## The Generation Process (Overview)

```
Chapter Content → Analyze Concepts → Generate 50 Questions →
Design Distractors → Randomize Answers → Write Explanations →
Format Quiz Component → Validate → ##_chapter_##_quiz.md
```

### 7-Stage Process Summary

1. **Analyze Chapter Structure:** Identify all core concepts and learning objectives (comprehensive, not highlights)
2. **Generate 50 Conceptual Questions:** Write 50 understanding-focused questions covering all chapter material
3. **Design Meaningful Distractors:** Create 3 distractors per question testing specific misconceptions
4. **Randomize Correct Answers:** Shuffle correctOption indices (0-3) across all 50 questions; verify ~12-13 per index
5. **Write Comprehensive Explanations:** 100-150 words per question explaining why correct AND why each distractor is wrong
6. **Format Quiz Component:** Use `<Quiz {...} questions={[...50 questions...]} />` with all 50 in questions array
7. **Validate Distribution:** Verify 50 questions, equal-length explanation clarity, randomized correctOption spread

📖 **Reference:** [generation-process.md](./references/generation-process.md) for detailed stage-by-stage workflow

---

## Quality Standards

### Content Quality
- ✅ **50 questions total** (comprehensive bank for spaced repetition)
- ✅ College-level conceptual (75%+ Apply or higher)
- ✅ Realistic scenarios (debugging, prediction, analysis, design decisions)
- ✅ No obvious recall questions ("What is...?")
- ✅ Covers ALL major topics (not just highlights)

### Immediate Feedback Quality
- ✅ Explanations show immediately after each answer (not delayed)
- ✅ Clear "why you were wrong" section for incorrect answers
- ✅ Correct option visually highlighted (green background)
- ✅ Incorrect option visually marked (red background) if selected

### Answer Randomization Quality (CRITICAL)
- ✅ Correct answers evenly distributed across all 50 (indices 0-3: ~12-13 each)
- ✅ No 3+ consecutive same correctOption values
- ✅ No obvious patterns (not 0,1,2,3,0,1,2,3...)
- ✅ All 4 index values (0,1,2,3) represented

### Explanation Quality
- ✅ **100-150 words each** (space for comprehensive explanations)
- ✅ Explain WHY correct (2-3 sentences)
- ✅ Address why EACH distractor is wrong (1-2 sentences each × 3 = 6-8 sentences)
- ✅ Real-world connection or misconception clarification (1-2 sentences)
- ✅ Addresses all four options explicitly

### Option Length Quality (CRITICAL - Prevents Cheating)
- ✅ **ALL options within ±3 words of each other** (e.g., 8, 9, 7, 10 words = PASS; 5, 6, 18, 7 = FAIL)
- ✅ **MANUALLY COUNT WORDS for every option in every question** (spot-check every question)
- ✅ Longest option is NOT always/usually correct (verify distribution)
- ✅ Shortest option is NOT always/usually correct (verify distribution)
- ✅ Correct answer randomly distributed across all lengths (not correlated)
- ✅ All 4 options are distinct and different
- ✅ No grammatical or spelling errors

### Component Format Quality
- ✅ Proper JSX syntax in markdown file
- ✅ Exactly 50 questions in questions array
- ✅ Exactly 4 options per question (no more, no less)
- ✅ correctOption uses 0-3 indices (NOT 1-4!)
- ✅ File named: `##_chapter_##_quiz.md`
- ✅ No imports needed (Quiz globally registered)
- ✅ **NO passingScore property** (removed entirely)
- ✅ Optional `questionsPerBatch` prop for customization

📖 **Reference:** [quality-checklist.md](./references/quality-checklist.md) for complete validation criteria

---

## Common Pitfalls (Top 7)

1. **Fewer than 50 Questions (CRITICAL):** Only generating 15-20 questions
   - ❌ "Only generated 30 questions" → Doesn't provide comprehensive coverage
   - ✅ Generate all 50 questions → Component handles batching automatically
   - Fix: **Always write 50 questions. Component displays 15-20 per session automatically.**

2. **Index Confusion (CRITICAL):** Using correctOption: 1-4 instead of 0-3
   - ❌ `correctOption: 4` → References non-existent 5th option
   - ✅ `correctOption: 3` → Correct (last option, 4th item)
   - Fix: Always use 0-3 indices

3. **Missing Source Field (CRITICAL):** Not including `source` field for questions
   - ❌ No `source` field → Students don't know which lesson the question addresses
   - ✅ All 50 questions have `source: "Lesson N: [Lesson Title]"`
   - Fix: Extract lesson number and title from lesson .md file metadata and populate source field for every question

4. **Including Passing Score:** Adding `passingScore` prop
   - ❌ `<Quiz ... passingScore={70} />` → No pass/fail in new version
   - ✅ `<Quiz ... />` → Just score tracking, no threshold
   - Fix: Remove passingScore entirely—focus on learning, not grading

5. **Testing Recall:** "What is X?" questions → Memorization
   - ❌ "What is a Python list?"
   - ✅ "Which operation modifies a list in-place and what is its return value?"
   - Fix: Focus on Apply/Analyze/Evaluate levels

6. **Weak Distractors or Incomplete Explanations:** Not addressing why each option is right/wrong
   - ❌ "Lists are mutable." (only explains correct answer)
   - ❌ "Lists are mutable; strings aren't." (misses all 3 distractors)
   - ✅ Full 100-150 word explanation addressing all 4 options explicitly
   - Fix: Write comprehensive explanations explaining why each of the 3 wrong options is incorrect

7. **Answer Patterns:** Obvious distribution patterns in correctOption across 50 questions
   - ❌ First 25 questions all have correctOption 0-1, last 25 all have 2-3
   - ❌ correctOption sequence: 0,1,2,3,0,1,2,3... (repeating pattern)
   - ✅ Evenly distributed: ~12-13 per index, random order
   - Fix: Shuffle correctOption values across all 50; verify equal distribution

8. **Option Length Bias (🚨 CRITICAL - TEST VALIDITY THREAT):** Options of unequal length allow test-takers to achieve 60-70%+ accuracy by selecting longest/shortest option WITHOUT reading questions

   **Impact:** Unequal lengths undermine the entire quiz's validity. Student might appear to understand when they're just following a pattern.

   **Examples:**
   - ❌ **INVALID:** A: "Yes" (2 words), B: "The framework processes requests asynchronously in a single event loop" (11 words), C: "No" (2 words), D: "Maybe" (5 words)
     - Difference: 2 to 11 words = 9-word spread → FAIL
     - Students can select B (longest) and get questions correct without reading

   - ❌ **INVALID:** Longest option is correct in 35 out of 50 questions (70%)
     - This creates systematic bias toward selecting longest option
     - Even if individual questions pass ±3 rule, overall pattern fails

   - ✅ **VALID:** All options 4-5 words: "AI amplifies existing practices" (4 words), "AI fixes broken processes" (4 words), "AI prevents all errors" (4 words), "AI changes developer skill" (5 words)
     - Difference: 4 to 5 words = 1-word spread → PASS
     - Correct answer distributed (longest correct in 2 questions, middle in 3, shortest in 1)

   **Fix (MANDATORY):**
   1. **Count words for ALL 50 questions** (not spot-check, all of them)
   2. **Enforce ±3 word limit strictly** (4, 5, 6, 7 words = PASS; 3, 6, 9 = FAIL)
   3. **Verify no length-correctness correlation** (correct answer sometimes longest, sometimes shortest, mostly middle)
   4. **Document validation results** before handoff

   **Validation Checklist:**
   - [ ] All 50 questions checked for word count
   - [ ] All options within ±3 word range
   - [ ] Longest option correct in ~25% of questions (distributed, not biased)
   - [ ] Shortest option correct in ~25% of questions (distributed, not biased)
   - [ ] No pattern visible (not "always pick longest or shortest")

📖 **Reference:** [pitfalls-and-solutions.md](./references/pitfalls-and-solutions.md) for all common mistakes

---

## File Naming Convention

### Pattern: `##_chapter_##_quiz.md`

Where:
- First `##` = sidebar_position (lesson count + 1)
- Second `##` = chapter number (zero-padded)
- Extension: `.md` (Quiz component is globally registered, no imports needed)

**Examples:**
- Chapter 2 (4 lessons): `05_chapter_02_quiz.md`
- Chapter 5 (6 lessons): `07_chapter_05_quiz.md`
- Chapter 14 (5 lessons): `06_chapter_14_quiz.md`

**Why this naming:**
- Matches lesson naming convention
- Clear chapter identification
- `.md` extension (Quiz component handles JSX rendering in markdown)
- Natural sorting places quiz at chapter end
- File contains 50 questions (component handles batching)

📖 **Reference:** [file-naming.md](./references/file-naming.md) for complete guidance

---

## Integration with Book Workflow

### Related Skills
- **learning-objectives:** Align questions to chapter objectives
- **assessment-builder:** General assessment principles
- **technical-clarity:** Ensure question language is accessible
- **content-evaluation-framework:** Validate quiz quality

### Related Infrastructure
- **Constitution:** Aligns with Principle 1 (AI-First Teaching) and Principle 5 (Progressive Complexity)
- **Chapter Index:** `specs/book/chapter-index.md`

---

## Handoff Criteria

The quiz is ready for human review when:

**Content Complete:**
- [ ] **50 questions generated** (comprehensive chapter coverage with spaced repetition potential)
- [ ] ALL major topics covered (proportional across all lessons)
- [ ] 75%+ at Apply level or higher (conceptual, not recall)
- [ ] Each question tests distinct concept from chapter material
- [ ] No "What is...?" recall questions
- [ ] Realistic scenarios (debugging, analysis, prediction, design decisions)

**Answer Randomization Verified:**
- [ ] correctOption uses 0-3 indices only (NOT 1-4)
- [ ] Correct answers evenly distributed (~12-13 per index across 50 questions)
- [ ] No 3+ consecutive same correctOption values
- [ ] No obvious patterns (0,1,2,3,0,1,2,3... is bad)
- [ ] All 4 indices (0,1,2,3) represented in distribution

**Option Length Validation Verified (🚨 MANDATORY):**
- [ ] **Word count manually verified for EVERY option in EVERY question (all 50 questions)**
- [ ] **ALL options within ±3 words of each other** (e.g., 5, 6, 7, 8 words = PASS; 2, 5, 10, 6 = FAIL)
- [ ] No question has options varying by >3 words
- [ ] Correct answer NOT correlated with longest option (verified distribution)
- [ ] Correct answer NOT correlated with shortest option (verified distribution)
- [ ] Longest option correct in ~20% of questions (distributed, not biased)
- [ ] Shortest option correct in ~20% of questions (distributed, not biased)

**Explanation Quality Verified (CRITICAL for Immediate Feedback):**
- [ ] All explanations 100-150 words (comprehensive, not rushed)
- [ ] Each explanation explains WHY correct (2-3 sentences)
- [ ] Each explanation addresses WHY EACH of the 3 distractors is wrong (1-2 sentences each)
- [ ] Each explanation includes real-world connection or misconception clarification (1-2 sentences)
- [ ] Distractors test specific misconceptions (not joke answers)
- [ ] Spot-check: 5-10 explanations address ALL 4 options explicitly

**Quiz Component Format Valid:**
- [ ] Valid JSX syntax in markdown file (proper braces, quotes, arrays)
- [ ] Exactly 50 questions in questions array (NOT fewer)
- [ ] Exactly 4 options per question (no more, no less)
- [ ] correctOption values use 0-3 indices (spot-checked accuracy)
- [ ] Questions array contains all 50 questions with complete data
- [ ] **`source` field present for ALL 50 questions** (format: "Lesson N: [Lesson Title]")
- [ ] **NO passingScore property** (removed entirely)
- [ ] Optional `questionsPerBatch={18}` (or omitted to use default 15)
- [ ] Quiz component used with correct props: title, questions, questionsPerBatch (optional)
- [ ] No imports needed - `<Quiz />` is globally registered
- [ ] File named: `##_chapter_##_quiz.md` (correct numbering)
- [ ] File uses `.md` extension
- [ ] YAML frontmatter correct: sidebar_position, title
- [ ] Saved to correct chapter directory

**Human Review Checklist:**
- [ ] Spot-check 10-15 questions for: misconception testing, conceptual rigor, explanation completeness
- [ ] Verify each explanation addresses all 4 options (not just the correct one)
- [ ] Confirm technical accuracy of all explanations against chapter content
- [ ] Test Quiz component rendering in Docusaurus (actual interactive test)
- [ ] Verify navigation works (Back/Next, dots, submit, results, retake)
- [ ] Check immediate feedback displays correctly after each answer
- [ ] Verify correct option is highlighted (green) when feedback shows
- [ ] Verify incorrect option is marked (red) when feedback shows
- [ ] Check "why your answer was wrong" section displays for incorrect answers
- [ ] Verify all explanations are readable and clear (no syntax errors)
- [ ] Approve for deployment

📖 **Reference:** [quality-checklist.md](./references/quality-checklist.md) for complete validation

---

## Bundled Resources

This skill includes detailed reference documentation:

- **[generation-process.md](./references/generation-process.md)** - Complete 7-stage workflow for Quiz component generation with examples
- **[answer-distribution.md](./references/answer-distribution.md)** - Randomization strategies and verification methods for correctOption indices
- **[file-naming.md](./references/file-naming.md)** - Naming conventions with examples
- **[pitfalls-and-solutions.md](./references/pitfalls-and-solutions.md)** - Common mistakes and fixes for Quiz component quizzes
- **[quality-checklist.md](./references/quality-checklist.md)** - Complete validation checklist for Quiz component format

Use `Read` tool to access references as needed during quiz generation.

---

**Quiz Generator v4.0.0 ALWAYS creates interactive assessments using the globally-registered Quiz component with 50 COMPREHENSIVE QUESTIONS. NEVER creates static markdown quizzes or fewer than 50 questions. Component automatically displays 15-20 random questions per batch, shuffled differently on each retake. Features immediate feedback per question (correct option + explanation + why wrong if incorrect), no passing/failing threshold (just score tracking), progress tracking, answer validation, color-coded feedback, retake button, and full theme support.

Every quiz MUST:
- Use <Quiz /> component with 50 questions
- Have randomized correctOption indices (0-3) evenly distributed
- Include 100-150 word explanations addressing all 4 options
- Include source field for all questions (format: "Lesson N: [Lesson Title]")
- **ENFORCE STRICT OPTION LENGTH VALIDATION: ALL options within ±3 words of each other for ALL 50 questions** (manually verified, not assumed)
- **VERIFY no correlation between option length and correctness** (longest option NOT biased toward being correct)
- NO passingScore prop
- NO passing/failing threshold**
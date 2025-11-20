# Python Chapter Quiz Generation Prompt
## Reasoning-Activated Assessment Design

**Version**: 2.0
**Date**: 2025-11-19
**Purpose**: Generate high-quality, reasoning-activated quizzes for Python Fundamentals chapters (13-30)
**Based on**: SDD-RI Whitepaper, Reasoning Activation paper, Skills Thinking Framework
**Skill**: Uses `quiz-generator` skill (v5.0.0) for interactive Quiz component generation

---

## 🚨 CRITICAL: Skill Integration

**This prompt MUST be used with the `quiz-generator` skill.**

**To generate a quiz for any Python chapter**:
```
/quiz-generator [or invoke skill: quiz-generator]

Then reference this prompt: prompts/quiz-generation-python-chapters.md
```

**The quiz-generator skill handles**:
- ✅ Interactive Quiz component generation (NOT static markdown)
- ✅ 50 total questions (comprehensive coverage)
- ✅ Randomized batching (15-20 questions per session)
- ✅ Immediate feedback after each answer
- ✅ Answer redistribution (evenly across indices 0-3)
- ✅ Option length validation (all within ±3 words)
- ✅ Automated explanation regeneration
- ✅ File naming convention: `##_chapter_##_quiz.md`

**This prompt provides**:
- 🎯 Reasoning activation framework (Persona + Questions + Principles)
- 🎯 Python-specific question design guidance
- 🎯 AI-native learning alignment
- 🎯 Chapter-specific content strategies
- 🎯 Quality validation criteria

---

## Context & Problem

**Issue**: Python chapters (Part 4, Chapters 13-30) lack end-of-chapter quizzes, unlike Part 1 chapters which have comprehensive assessments.

**Why This Matters**:
- Quizzes validate understanding before students progress
- Assessments reinforce AI-native learning principles
- Testing prevents false confidence from passive reading
- Questions should activate reasoning, not test memorization

**Reference Example**: Part 1 quizzes (e.g., `01-ai-development-revolution/09_chapter_01_quiz.md`) demonstrate the quality standard we're targeting.

---

## Your Role & Cognitive Stance

**You are NOT**: A generic question generator creating surface-level multiple choice tests

**You ARE**: An assessment architect who designs diagnostic instruments that:
- Reveal genuine understanding vs. surface memorization
- Test application ability, not fact recall
- Validate AI-native workflow competence
- Identify misconceptions that block progression

**Activate Reasoning By Asking**:

### 1. Layer Recognition
- **What layer does this chapter teach?**
  - L1 (Manual Foundation): First exposure to Python concepts (variables, loops, conditionals)
  - L2 (AI Collaboration): Using AI to write/refactor Python code
  - L3 (Intelligence Design): Creating reusable Python patterns/skills
  - L4 (Spec-Driven): Orchestrating Python projects with specifications

### 2. Proficiency Tier Assessment
- **What's the target complexity for this chapter?**
  - Check chapter number in Part 4 (13-30)
  - Early chapters (13-16): A2-B1 (beginner-intermediate)
  - Mid chapters (17-23): B1-B2 (intermediate)
  - Advanced chapters (24-30): B2-C1 (intermediate-advanced)

### 3. Concept vs. Workflow Testing
- **What should I test?**
  - **Concept mastery**: Understanding Python semantics (what `for` loops do)
  - **Workflow competence**: Describing intent to AI effectively
  - **Pattern recognition**: Identifying when to use specific structures
  - **Edge case awareness**: Understanding failure modes and boundaries

---

## Core Principles (Your Decision Frameworks)

### Principle 1: Test Understanding, Not Memorization

**Generic Convergence** (What to AVOID):
```
❌ "What is the syntax for a for loop?"
❌ "Which keyword breaks out of a loop?"
❌ "How many parameters does range() accept?"
```

**Reasoning Activation** (What to CREATE):
```
✅ "Why would you use enumerate() instead of range(len(list))?"
   → Tests understanding of Pythonic patterns vs. mechanics

✅ "When reviewing AI-generated code, what loop structure issue
   should you check first?"
   → Tests AI collaboration workflow awareness

✅ "Given this scenario [specific context], which data structure
   should you choose and why?"
   → Tests decision-making ability, not syntax recall
```

### Principle 2: Align with AI-Native Learning Philosophy

**Remember**: Students learn Python WITH AI collaboration, not BEFORE it.

**Questions Should Validate**:
- Can student describe their intent clearly to AI?
- Can student evaluate AI-generated Python code for correctness?
- Does student understand when to use AI vs. manual coding?
- Can student identify which Python patterns solve specific problems?

**Question Types That Match This Philosophy**:
1. **Scenario-based**: "You need to process 1M records. How would you describe this to AI?"
2. **Code review**: "AI generated this code [snippet]. What's the issue?"
3. **Pattern selection**: "Which Python structure best handles [specific use case]?"
4. **Intent articulation**: "How would you prompt AI to implement [requirement]?"

### Principle 3: Progressive Complexity (Bloom's Taxonomy)

**Map Questions to Cognitive Levels**:

| Bloom Level | Question Type | Example |
|-------------|--------------|---------|
| **Remember** (20%) | Definition/recognition | "What does `break` do in a loop?" |
| **Understand** (25%) | Explanation/interpretation | "Why does this code produce infinite loop?" |
| **Apply** (30%) | Scenario-based usage | "Which loop type handles unknown iteration counts?" |
| **Analyze** (15%) | Code review/comparison | "Compare these two approaches—which is better and why?" |
| **Evaluate** (10%) | Quality assessment | "Review this AI-generated code. What should change?" |

**Chapter-Level Distribution**:
- **Early chapters (13-17)**: Heavier on Remember/Understand (60%)
- **Mid chapters (18-24)**: Balanced Apply/Analyze (50%)
- **Advanced chapters (25-30)**: Heavier on Analyze/Evaluate (40%)

### Principle 4: Use Production-Relevant Context

**Avoid**: Toy examples disconnected from real development
```
❌ "Write a program to check if a number is even"
❌ "Create a list of your favorite fruits"
```

**Prefer**: Scenarios students will encounter in AI-native development
```
✅ "You're processing API responses with variable structure.
   Which Python pattern handles missing keys safely?"

✅ "Your AI agent needs to retry failed operations.
   What loop control mechanism ensures maximum 3 attempts?"

✅ "When validating user input with Pydantic, which data structure
   stores unique email addresses most efficiently?"
```

### Principle 5: Provide Explanations That Teach

**Each wrong answer should reveal a specific misconception.**

**Structure for Explanations**:
1. **State the correct answer explicitly**
2. **Explain WHY it's correct (reasoning)**
3. **Connect to chapter content (source)**
4. **Address common misconceptions** (what students might think)
5. **Link to AI-native workflow** (how AI collaboration applies)

**Example**:
```javascript
explanation: "The correct answer is option 2 (use enumerate() instead of
range(len())). This is more Pythonic because enumerate() directly provides
both index and value in each iteration, making code more readable and less
error-prone. While range(len(items)) works, it requires manual indexing
(items[i]), creating opportunities for off-by-one errors. When working with
AI, describing 'iterate with index and value' will prompt AI to suggest
enumerate(), not manual indexing. This pattern appears in Lesson 3:
Iteration Patterns.",
source: "Lesson 3: Pythonic Iteration Patterns"
```

---

## Quiz Structure Template

### File Naming & Location
```
book-source/docs/04-Python-Fundamentals/[chapter-number]-[chapter-slug]/
  └── [NN]_chapter_[chapter-number]_quiz.md
```

**Example**:
```
book-source/docs/04-Python-Fundamentals/18-control-flow-loops/
  └── 06_chapter_18_quiz.md
```

### Frontmatter Format
```yaml
---
sidebar_position: [last lesson number + 1]
title: "Chapter [N]: [Chapter Title] Quiz"
---
```

### Quiz Component Structure

**IMPORTANT**: The quiz-generator skill requires **50 questions total** (not 10-15).

```markdown
# Chapter [N]: [Chapter Title] Quiz

Test your understanding of [1-sentence chapter summary focused on learning outcomes].

<Quiz
  title="Chapter [N]: [Chapter Title] Assessment"
  questions={[
    {
      question: "[Clear, specific question testing understanding]",
      options: [
        "[Option A - all within ±3 words]",
        "[Option B - all within ±3 words]",
        "[Option C - all within ±3 words]",
        "[Option D - all within ±3 words]"
      ],
      correctOption: [0-3, zero-indexed],
      explanation: "[Comprehensive explanation following Principle 5]",
      source: "Lesson [N]: [Lesson Title]"
    },
    // ... 50 QUESTIONS TOTAL (required by quiz-generator skill)
  ]}
/>
```

**Key Requirements from quiz-generator skill**:
1. **Exactly 50 questions** (component displays 15-20 per batch, randomized)
2. **All options within ±3 words** of each other per question (prevents length-based guessing)
3. **correctOption** is 0-indexed (0, 1, 2, or 3)
4. **Immediate feedback** - explanations should address WHY wrong answers are incorrect
5. **Answer redistribution** - After generation, run:
   ```bash
   python .claude/skills/quiz-generator/scripts/redistribute_answers_v2.py <quiz_file> <sequence_letter>
   ```

---

## Question Generation Workflow

### Phase 1: Chapter Analysis (BEFORE writing questions)

**Read and Extract**:
1. **Chapter README.md**: What are the learning objectives?
2. **All lesson files**: What concepts are taught in what sequence?
3. **Code examples**: What patterns are demonstrated?
4. **Exercises**: What skills do students practice?

**Map Concepts to Testable Understanding**:
- **List core concepts**: (e.g., for loops, while loops, break, continue, nested loops)
- **Identify key decision points**: When would you use X vs. Y?
- **Note common mistakes**: What errors do students make?
- **Find AI collaboration opportunities**: Where does AI help in this chapter?

### Phase 2: Question Design (Activate Reasoning)

**For Each Question, Ask**:

1. **What misconception does this test?**
   - "Students think X, but actually Y"
   - Design distractors that match the misconception

2. **What cognitive level does this target?**
   - Remember/Understand/Apply/Analyze/Evaluate
   - Ensure distribution matches proficiency tier

3. **Does this connect to real development?**
   - Would a professional encounter this scenario?
   - Can student use AI to solve it?

4. **Are distractors plausible?**
   - Each wrong answer should be tempting
   - Each should represent a specific misunderstanding
   - Avoid obviously wrong options

5. **Does explanation teach?**
   - Student should learn from WRONG answers
   - Explanation should reference specific lesson content
   - Should reinforce AI-native workflow principles

### Phase 3: Question Distribution

**Ensure Balance Across 50 Questions**:

| Dimension | Target Distribution (out of 50 questions) |
|-----------|------------------------------------------|
| **Bloom Levels** | Remember (10 q), Understand (12 q), Apply (15 q), Analyze (8 q), Evaluate (5 q) |
| **Content Coverage** | All lessons represented (~8-10 questions per lesson for 5-lesson chapter) |
| **Question Types** | Scenario (20 q), Code review (15 q), Concept (10 q), Workflow (5 q) |
| **Difficulty** | Easy (10 q), Medium (30 q), Hard (10 q) |
| **Option Length** | All options within ±3 words per question (CRITICAL - validates test integrity) |

**Total Questions Per Chapter**: **ALWAYS 50 questions** (required by quiz-generator skill)
- Component displays 15-20 questions per batch (randomized)
- Students can retake with different question batches
- Comprehensive chapter coverage across all 50 questions

### Phase 4: Validation Checklist

**Before Finalizing, Verify**:

- [ ] **Exactly 50 questions total** (required by quiz-generator skill)
- [ ] All learning objectives from README are tested
- [ ] Question distribution matches target counts (10/12/15/8/5 for Bloom levels)
- [ ] Every question has 4 options (no 3-option questions)
- [ ] **All options within ±3 words per question** (CRITICAL - count manually!)
- [ ] Explanations reference specific lesson numbers
- [ ] No pure memorization questions (all require reasoning)
- [ ] Distractors are plausible (not obviously wrong)
- [ ] Correct answers are definitively correct (no ambiguity)
- [ ] At least 5 questions test AI collaboration workflow
- [ ] Production-relevant context (no toy examples)
- [ ] Language matches chapter proficiency tier (A2/B1/B2/C1)
- [ ] **After generation, run answer redistribution script** with chosen sequence (A-H)

---

## Example Questions by Type

### Type 1: Scenario-Based Application (40% of quiz)

**Pattern**: Present real development scenario, test decision-making

```javascript
{
  question: "You're building an API client that processes paginated responses. Each page might have 0-100 items, and you don't know total pages in advance. Which loop structure handles this best?",
  options: [
    "for loop with range(1000) and break when empty page found",
    "while loop checking if page has items, requesting next page",
    "do-while loop (not available in Python, use workaround)",
    "Recursive function calling itself until no more pages"
  ],
  correctOption: 1,
  explanation: "Option 2 (while loop) is correct because it naturally handles unknown iteration counts. You check the condition (page has items) before each iteration and continue until false. Option 1 works but uses arbitrary large number (1000), which is fragile. Option 3 mentions do-while, which Python doesn't have. Option 4 (recursion) would work but risks stack overflow with many pages and is less idiomatic Python. When describing this to AI, you'd say 'loop while pages remain' or 'continue until empty page,' which naturally maps to while loop structure. This pattern is demonstrated in Lesson 3: Loop Selection Strategies.",
  source: "Lesson 3: Loop Selection Strategies"
}
```

### Type 2: Code Review/AI Evaluation (30% of quiz)

**Pattern**: Show code snippet (possibly AI-generated), identify issue

```javascript
{
  question: "You asked AI to 'process all user records.' It generated this code:\n```python\nfor i in range(len(users)):\n    process(users[i])\n```\nWhat improvement should you request?",
  options: [
    "Add type hints to the process() function",
    "Use 'for user in users:' instead of indexing",
    "Convert to while loop for better performance",
    "Add error handling around process() call"
  ],
  correctOption: 1,
  explanation: "Option 2 is correct. The AI used C-style iteration (indexing) when Python prefers direct iteration. You should request: 'Iterate directly over users collection without indexing.' While option 4 (error handling) might be needed depending on requirements, the immediate code smell is unpythonic iteration. Option 1 (type hints) is good practice but doesn't address the iteration pattern. Option 3 (while loop) would make code worse—for loops are correct here, just poorly written. This teaches you to recognize when AI produces technically correct but non-idiomatic Python, covered in Lesson 4: Pythonic Patterns.",
  source: "Lesson 4: Pythonic Iteration Patterns"
}
```

### Type 3: Concept Understanding (20% of quiz)

**Pattern**: Test deep understanding of Python semantics

```javascript
{
  question: "What is the key difference between 'break' and 'return' when used inside a loop within a function?",
  options: [
    "break exits the loop only; return exits the entire function",
    "return is faster because it skips cleanup code",
    "break can be used in any loop; return only in for loops",
    "They are functionally identical in Python 3.10+"
  ],
  correctOption: 0,
  explanation: "Option 1 is correct. 'break' exits only the innermost loop and continues executing code after that loop. 'return' exits the entire function immediately, returning control to the caller. This matters when you have code after the loop—break will execute it, return won't. Option 2 is false (no performance difference in this context). Option 3 is backwards (return works anywhere, break only in loops). Option 4 is false (they've always been different). Understanding this distinction helps you describe intent to AI: 'stop this loop but continue function' vs. 'exit function immediately.' Covered in Lesson 4: Loop Control.",
  source: "Lesson 4: Controlling Loop Execution"
}
```

### Type 4: AI Workflow Integration (10% of quiz)

**Pattern**: Test AI collaboration competence specifically

```javascript
{
  question: "When asking AI to implement nested loops, which prompt structure produces better results?",
  options: [
    "Write nested for loops to process matrix",
    "Iterate over 2D array: for each row, process each column element",
    "Use two for loops, one inside the other",
    "Create nested iteration structure for matrix operations"
  ],
  correctOption: 1,
  explanation: "Option 2 is best because it describes INTENT and STRUCTURE without prescribing exact syntax. It tells AI what you're iterating over (2D array), the hierarchy (rows then columns), and the action (process elements). Option 1 prescribes structure ('for loops') without context. Option 3 describes mechanics without purpose. Option 4 uses vague terminology ('structure,' 'operations'). Good AI prompts answer: WHAT are you iterating? WHY (in what order)? WHAT happens each iteration? This principle applies to all AI code generation: describe desired outcome and constraints, not implementation syntax. Covered in Lesson 5: Effective AI Collaboration for Control Flow.",
  source: "Lesson 5: Describing Control Flow Intent"
}
```

---

## Anti-Patterns to Avoid

### ❌ Memorization Questions
```javascript
// DON'T: Pure syntax recall
{
  question: "What is the syntax for a while loop in Python?",
  options: [
    "while (condition) { }",
    "while condition:",
    "loop while condition:",
    "while condition do:"
  ],
  correctOption: 1
}
```
**Why wrong**: Tests memory, not understanding. Students can look up syntax.

### ❌ Trivial Questions
```javascript
// DON'T: Overly simple
{
  question: "Can you use break in a for loop?",
  options: ["Yes", "No", "Only in Python 3.10+", "Only with else clause"],
  correctOption: 0
}
```
**Why wrong**: Too easy, doesn't test meaningful understanding.

### ❌ Ambiguous Questions
```javascript
// DON'T: Multiple valid interpretations
{
  question: "What's the best way to iterate over a list?",
  options: [
    "for item in list:",
    "for i in range(len(list)):",
    "while i < len(list):",
    "list.forEach()"
  ],
  correctOption: 0
}
```
**Why wrong**: "Best" is context-dependent. Need specific scenario.

### ❌ Trick Questions
```javascript
// DON'T: Gotchas without learning value
{
  question: "What does this print? [tricky edge case]",
  options: ["...", "...", "...", "..."],
  correctOption: 2
}
```
**Why wrong**: Tests knowledge of obscure edge cases, not practical understanding.

---

## Chapter-Specific Guidance

### For Chapter 13 (UV Package Manager)
- Focus on: Tool selection, dependency management, virtual environments
- AI workflow: Describing project requirements, reviewing generated configs
- Scenarios: Multi-developer projects, conflicting dependencies, reproducible builds

### For Chapters 14-17 (Python Basics)
- Focus on: Data types, operators, variables, type hints
- AI workflow: Prompting for type-safe code, reviewing generated type annotations
- Scenarios: API data processing, configuration validation, data transformation

### For Chapter 18 (Control Flow & Loops)
- Focus on: Conditionals, pattern matching, loop types, control mechanisms
- AI workflow: Describing iteration intent, evaluating loop efficiency
- Scenarios: Input validation, batch processing, retry logic, data filtering

### For Chapters 19-20 (Data Structures)
- Focus on: Lists, tuples, dicts, sets, performance characteristics
- AI workflow: Choosing correct structure for use case, optimization
- Scenarios: Caching, deduplication, ordered processing, key-value lookups

### For Chapters 21-23 (Functions & I/O)
- Focus on: Function design, exception handling, file operations
- AI workflow: Describing function contracts, error handling strategies
- Scenarios: API integrations, file processing, error recovery, logging

### For Chapters 24 (Math & DateTime)
- Focus on: Numeric operations, date handling, timezone awareness
- AI workflow: Prompting for correct datetime operations, avoiding pitfalls
- Scenarios: Scheduling, calculations, timestamp handling, date arithmetic

### For Chapters 25-27 (OOP & Advanced)
- Focus on: Classes, inheritance, metaclasses, dataclasses, Pydantic
- AI workflow: Describing class relationships, validating generated architectures
- Scenarios: Domain modeling, validation logic, data serialization, extensibility

### For Chapters 28-30 (Advanced Python)
- Focus on: Generics, async/await, CPython internals, GIL
- AI workflow: Articulating concurrency requirements, performance optimization
- Scenarios: Concurrent I/O, type-safe collections, performance tuning, async APIs

---

## Final Execution Workflow

**Step 1: Invoke quiz-generator Skill**
```
/quiz-generator
[or use Skill tool to invoke quiz-generator]
```

**Step 2: Reference This Prompt**
Tell the skill to follow guidance from: `prompts/quiz-generation-python-chapters.md`

**Step 3: Chapter Analysis** (show your reasoning):
- What are this chapter's learning objectives?
- What layer does it teach (L1/L2/L3/L4)?
- What proficiency tier (A2/B1/B2/C1)?
- What are the key concepts to test?
- What misconceptions do students have here?
- How many lessons in this chapter? (determines question distribution)

**Step 4: Generate 50 Questions** following:
- Quiz component structure (not static markdown)
- **50 questions total** (not 10-15)
- Question type distribution: Scenario (20), Code review (15), Concept (10), Workflow (5)
- Bloom level distribution: Remember (10), Understand (12), Apply (15), Analyze (8), Evaluate (5)
- **All options within ±3 words per question**
- All anti-patterns avoided
- All validation criteria met

**Step 5: Validation Checklist**
- [ ] Run through all validation criteria
- [ ] Manually count option lengths for all 50 questions
- [ ] Verify all lessons covered
- [ ] Confirm explanations teach, not just state answers
- [ ] Ensure production-relevant scenarios used

**Step 6: Answer Redistribution** (automated)
```bash
python .claude/skills/quiz-generator/scripts/redistribute_answers_v2.py \
  book-source/docs/04-Python-Fundamentals/[chapter-folder]/[NN]_chapter_[NN]_quiz.md \
  [A-H sequence letter]
```

**Choose sequence letter** to vary across chapters:
- Chapter 13: Sequence A
- Chapter 14: Sequence B
- Chapter 15: Sequence C
- Chapter 16: Sequence D
- Chapter 17: Sequence E
- Chapter 18: Sequence F
- (etc., rotating through A-H)

**Step 7: Final Verification**
Script will report:
- Answer distribution (should be ~12-13 per index)
- Explanation validation (all match correct answers)
- Any issues requiring manual review

---

## Success Criteria

**You've succeeded when**:
✅ **Exactly 50 questions generated** (quiz-generator requirement)
✅ **All options within ±3 words per question** (test validity)
✅ Every question tests understanding, not memorization
✅ Scenarios connect to real AI-native development
✅ Distractors represent plausible misconceptions
✅ Explanations teach concepts even when answering wrong
✅ Distribution matches targets (20/15/10/5 question types, 10/12/15/8/5 Bloom levels)
✅ AI collaboration workflow is tested explicitly (5 questions minimum)
✅ All chapter lessons are represented in questions (~8-10 per lesson)
✅ Language matches chapter proficiency tier
✅ Questions activate reasoning mode (not prediction mode)
✅ **Answer redistribution script executed successfully**
✅ Quiz uses Quiz component (NOT static markdown)
✅ Quiz could be reused across cohorts without modification

---

## References

### Papers & Frameworks
- **SDD-RI Whitepaper**: `papers/SDD-RI Whitepaper.md` - Specification-driven development with reusable intelligence
- **Reasoning Activation Paper**: `papers/Reasoning_Activation_in_LLMs_arXiv_Complete.md` - Persona + Questions + Principles framework
- **Skills Thinking Framework**: `papers/skills-thinking-framework.md` - Identifying convergence, mapping to implementation
- **Constitution**: `.specify/memory/constitution.md` - Three Roles, pedagogical principles
- **4-Layer Teaching Method**: L1 (Manual) → L2 (Collaboration) → L3 (Intelligence) → L4 (Spec-Driven)

### Skills & Components
- **quiz-generator Skill**: `.claude/skills/quiz-generator/SKILL.md` (v5.0.0)
- **Quiz Component**: `book-source/src/components/Quiz.tsx` (globally registered)
- **Quiz Usage Guide**: `book-source/src/components/QUIZ_USAGE.md`
- **Example Quiz**: `book-source/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/09_chapter_01_quiz.md`

### Scripts
- **Answer Redistribution**: `.claude/skills/quiz-generator/scripts/redistribute_answers_v2.py`
- **Option Length Validation**: `.claude/skills/quiz-generator/references/option-length-validation.md`

---

## Usage Example

**User says**: "Generate quiz for Chapter 18: Control Flow and Loops"

**You respond**:

"Invoking quiz-generator skill for Chapter 18: Control Flow and Loops..."

**[After skill activation, following prompts/quiz-generation-python-chapters.md]**

**Chapter Analysis**:
- **Learning Objectives**: Conditionals (if/elif/else), pattern matching (match-case), loops (for/while), control flow (break/continue), nested structures
- **Layer**: L1 (Manual Foundation) + early L2 (AI Collaboration for code generation)
- **Proficiency**: B1 (intermediate - Chapter 18 is mid-range in Part 4)
- **Lessons**: 5 lessons (01-05)
- **Key Concepts**: Conditional logic, loop selection, iteration patterns, loop control, nested structures
- **Common Misconceptions**: Thinking for loops require range(), confusing break/continue/return, unnecessary nesting, using wrong loop type for scenario

**Question Distribution Plan (50 questions)**:
- **By Type**: Scenario (20), Code review (15), Concept (10), Workflow (5)
- **By Bloom**: Remember (10), Understand (12), Apply (15), Analyze (8), Evaluate (5)
- **By Lesson**: ~10 questions per lesson (covering all 5 lessons)
- **By Difficulty**: Easy (10), Medium (30), Hard (10)

**Specific Coverage**:
- Lesson 1 (Conditionals): 10 questions on if/elif/else logic
- Lesson 2 (Pattern Matching): 10 questions on match-case
- Lesson 3 (Loops): 10 questions on for/while selection
- Lesson 4 (Loop Control): 10 questions on break/continue/else
- Lesson 5 (Nested): 10 questions on nested structures

**Now generating 50 questions with:**
- All options within ±3 words per question
- Quiz component format (not static markdown)
- Immediate feedback explanations
- Production-relevant scenarios
- AI collaboration workflow integration

[Then generate the complete 50-question quiz following all templates and principles]

**After generation**:
```bash
python .claude/skills/quiz-generator/scripts/redistribute_answers_v2.py \
  book-source/docs/04-Python-Fundamentals/18-control-flow-loops/06_chapter_18_quiz.md \
  F
```

✅ Quiz generated successfully with 50 questions
✅ Answer distribution validated (13-0, 12-1, 13-2, 12-3)
✅ All explanations updated and verified
✅ Ready for student use"

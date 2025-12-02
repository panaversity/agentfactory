# Quiz Component - Usage Guide (v4.0)

## ✅ No Imports Needed!

The Quiz component is **globally registered**. Just use `<Quiz />` directly in your MDX files.

## 🚀 Basic Usage

```mdx
---
title: Chapter 1 Quiz
---

# Chapter 1 Quiz

<Quiz
  title="Python Basics Assessment"
  questions={[
    {
      question: "When you modify a list passed to a function, what happens to the original list?",
      options: [
        "The original list remains unchanged",
        "Lists cannot be passed to functions"
        "The original list is deleted",
        "The original list is also modified",
        
      ],
      correctOption: 3,
      explanation: "The original list is modified because lists are mutable and Python passes them by reference. When you modify the list inside the function, you're modifying the same object that exists outside the function. This is a key concept in understanding Python's object model and how mutable vs. immutable types behave in function calls.",
      source: "Lesson 1: Understanding Mutability"
    },
    {
      question: "What is the primary purpose of using a default parameter in a function definition?",
      options: [
        "To make the function run faster",
        "To prevent the parameter from being modified",
        "To allow callers to omit the parameter when calling the function",
        "To create a copy of the parameter instead of using the original"
      ],
      correctOption: 2,
      explanation: "Default parameters allow you to call a function without providing that argument. If the argument is omitted, the default value is used. This is useful for making functions more flexible and easier to call. For example, a function can have sensible defaults that work for most cases, while still allowing callers to override them when needed.",
      source: "Lesson 2: Function Parameters and Scope"
    }
  ]}
/>
```

## 📋 Props

### Required
- **`questions`** - Array of 50 question objects (enables proper batching and spaced repetition)

### Optional
- **`title`** - Quiz title (default: "Quiz")
- **`questionsPerBatch`** - Questions to display per session (default: 15, range: 15-20)

**Note:** `passingScore` prop has been removed in v4.0. Quizzes focus on learning through immediate feedback rather than pass/fail grading.

## 📝 Question Format

Each question must have:

```python
{
  question: "Your question text?",
  options: ["Option A", "Option B", "Option C", "Option D"],  // Exactly 4 options
  correctOption: 2,  // Index 0-3 of correct answer
  explanation: "Detailed explanation (100-150 words) addressing why correct and why each distractor is wrong",
  source: "Lesson N: [Lesson Title]"  // Required in v4.0
}
```

**Important:**
- ✅ Must have exactly **4 options**
- ✅ `correctOption` must be **0, 1, 2, or 3** (not 1-4!)
- ✅ All options must be **within ±3 words of each other** (prevents length-based guessing)
- ✅ `explanation` required (100-150 words, addresses all 4 options)
- ✅ `source` required (format: "Lesson N: [Lesson Title]")

## 💡 Complete Example

```md
---
sidebar_position: 6
title: Chapter 3 Quiz
---

# Chapter 3: Functions - Assessment

Test your understanding of Python functions through this comprehensive quiz. You'll see 15-20 questions per session, with a new random selection each time you retake it.

<Quiz
  title="Functions and Scope Assessment"
  questions={[
    {
      question: "What keyword is used to define a function in Python?",
      options: [
        "function keyword defines a block",
        "func keyword introduces code",
        "def keyword starts a function",
        "define keyword creates scope"
      ],
      correctOption: 2,
      explanation: "Python uses the 'def' keyword to define functions. This distinguishes it from languages like JavaScript that use 'function'. The def keyword followed by a function name and parentheses introduces a new function scope. Other languages have different keywords: C uses no keyword (just type), JavaScript uses 'function', etc. Understanding Python's syntax is crucial for writing correct code.",
      source: "Lesson 1: Function Basics"
    },
    {
      question: "What does a function return if no return statement is specified?",
      options: [
        "None is returned automatically",
        "0 is returned by default",
        "Empty string is returned",
        "undefined is returned always"
      ],
      correctOption: 0,
      explanation: "Functions without an explicit return statement automatically return None. This is Python's way of representing 'no value'. If you try to use the returned value in calculations, you'll get an error because None is not a number. This differs from languages like JavaScript which return undefined. None is a special singleton object in Python used throughout the language.",
      source: "Lesson 1: Function Basics"
    },
    {
      question: "Which is the correct syntax for a function with default parameters?",
      options: [
        "def greet(name = 'World'):",
        "def greet(name: 'World'):",
        "def greet(name -> 'World'):",
        "def greet(name := 'World'):"
      ],
      correctOption: 0,
      explanation: "Default parameters use the equals sign: parameter = default_value. The equals sign in the function definition context means 'use this value if no argument is provided'. The other options represent different Python concepts: the colon is for type hints, the arrow is for return type hints, and := is the walrus operator. Mixing these up will cause syntax errors.",
      source: "Lesson 2: Function Parameters"
    },
    {
      question: "What is a lambda function?",
      options: [
        "A named function inside another",
        "An anonymous single-expression function",
        "A function with multiple return statements",
        "A function that processes lists"
      ],
      correctOption: 1,
      explanation: "Lambda functions are anonymous (unnamed) functions limited to a single expression. They're useful for short operations you don't want to formally define. The syntax is lambda arguments: expression. For example: lambda x: x * 2. Lambda functions return the expression value automatically. They can't contain multiple statements or a full suite of code like regular functions.",
      source: "Lesson 3: Advanced Functions"
    },
    {
      question: "Can a function modify a list passed as an argument?",
      options: [
        "No, arguments are never modified",
        "Only if you use 'global'"
        "Only with special syntax",
        "Yes, because lists are mutable",
      ],
      correctOption: 3,
      explanation: "Lists are mutable objects, so functions can modify them directly. When you pass a list to a function, you're passing a reference to the same object. Changes made to the list inside the function affect the original. This is different from immutable types like integers and strings, which can't be modified in place. Understanding mutability is crucial for writing correct Python code.",
      source: "Lesson 4: Mutability in Functions"
    },
    {
      question: "What is the scope of a variable defined inside a function?",
      options: [
        "Global scope for all code",
        "Local scope limited to function",
        "Module scope for imports",
        "Class scope if nested"
      ],
      correctOption: 1,
      explanation: "Variables defined inside a function have local scope and only exist during function execution. After the function returns, they're deleted. This prevents naming conflicts and keeps namespaces organized. If you need to access a global variable inside a function, use the global keyword. Nested functions create their own local scope, but can access enclosing scopes with the nonlocal keyword.",
      source: "Lesson 2: Function Parameters and Scope"
    },
    // ... 44 more questions to reach 50 total ...
  ]}
/>
```

## 🎯 How the v4.0 Batching Works

### Immediate Feedback Model
- When you select an answer, you **immediately see if you're correct**
- Color-coded feedback: green for correct, red for incorrect
- **"Why your answer was wrong"** section explains the misconception
- **Comprehensive explanation** (100-150 words) teaches the concept
- **Source attribution** shows which lesson covers this topic
- Click **Next** to move to the next question

### 50-Question Bank with Batching
- Quiz starts with a **random selection of 15-20 questions** from the 50-question bank
- Each time you click **"Try Another Batch"** after completing, you see a **different random selection**
- This enables **spaced repetition**: learn the same concepts multiple times with different questions
- After 3 retakes, you'll have seen 45-60 different questions, reinforcing learning

### Why 50 Questions?
1. **Comprehensive coverage** - Every concept thoroughly tested
2. **Spaced repetition** - Multiple exposures improve retention
3. **Meaningful batching** - With 15-20 per batch, each retake shows mostly new content
4. **No repeated monotony** - Students see different angles on the same concepts

## 🎨 Features (Built-in)

Your quiz automatically includes:

- ✅ **Immediate feedback** - See if correct/incorrect right away
- ✅ **Explanation section** - Comprehensive teaching after each answer
- ✅ **Source attribution** - Know which lesson covers each topic
- ✅ **Progress bar** showing completion
- ✅ **Back/Next buttons** for navigation
- ✅ **Question dots** - click to jump to answered questions
- ✅ **Answer counter** - shows X/Y answered
- ✅ **Validation** - can't submit until all answered
- ✅ **Results page** with score and full review
- ✅ **Try Another Batch button** - get new random selection
- ✅ **Light/Dark theme** support
- ✅ **Fully responsive** (mobile, tablet, desktop)

## 🎨 Colors

Automatically matches your site theme:

- **Primary:** Navy blue (`#004d99`)
- **Success:** Green (correct answers)
- **Error:** Red (incorrect answers)
- **Auto theme switching** between light/dark mode

## ❓ Common Questions

### Q: Do I need to import the Quiz component?
**A:** No! It's globally registered. Just use `<Quiz />` directly.

### Q: Why 50 questions? Can I use fewer?
**A:** v4.0 requires 50 questions to enable proper batching and spaced repetition. With fewer questions (e.g., 25), students see most/all content on first attempt with no novelty on retake. 50 questions ensures meaningful batching: a 15-20 question batch covers only 30-40% of content, so retakes show mostly new material.

### Q: Can I have multiple quizzes on one page?
**A:** Yes! Add multiple `<Quiz />` components with different questions.

### Q: Why must I have exactly 4 options?
**A:** The component is designed for 4 options (A, B, C, D). This is the standard for multiple-choice assessments and matches college/university standards.

### Q: Why must options be within ±3 words?
**A:** This prevents "length-based guessing" where students pick the longest option. Equal-length options force students to read carefully and understand, not guess by visual cues.

### Q: What if I use the wrong index for correctOption?
**A:** Remember: arrays start at 0!
- First option = 0
- Second option = 1
- Third option = 2
- Fourth option = 3

### Q: Why is explanation 100-150 words?
**A:** The immediate feedback model shows explanations right after students answer. Longer explanations (100-150 words) provide comprehensive teaching while still being scannable. This teaches through the mistake, which has higher learning impact than showing answers at the end.

### Q: What is the source field?
**A:** The `source` field indicates which lesson covers the concept being tested. Format: `"Lesson N: [Lesson Title]"`. This helps students trace back to the relevant lesson if they need review.

### Q: Can I customize the colors?
**A:** Yes! Edit `src/components/quiz/Quiz.module.css` to change styles.

## ✅ Quick Checklist

When creating a quiz (v4.0):

- [ ] Exactly **50 questions** total
- [ ] Exactly 4 options per question
- [ ] All options **within ±3 words** each other
- [ ] `correctOption` is 0, 1, 2, or 3
- [ ] Explanations **100-150 words each**
- [ ] Explanations address **why correct AND why each distractor is wrong**
- [ ] ALL questions have `source` field ("Lesson N: Title")
- [ ] Title is descriptive
- [ ] Questions are clear and conceptual (not recall)
- [ ] 75%+ questions at Apply/Analyze level (Bloom's)
- [ ] All options are plausible distractors
- [ ] Answer indices randomized (~12-13 per index)
- [ ] No 3+ consecutive same answer indices
- [ ] Longest option ≠ always correct

## 🚀 That's It!

No imports, no configuration. Just copy the example above, create your 50 questions, and you're done! 🎉

**For detailed guidance:** See [SKILL.md](../../.claude/skills/quiz-generator/SKILL.md) for the complete quiz generation workflow.

---
title: "Writing Clear Commands - Your First Code Generation"
chapter: 9
lesson: 2
part: 3
duration_minutes: 40

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Writing Clear AI Commands"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student writes 3+ prompts using strong technical verbs that produce usable AI responses"

  - name: "Iterating with AI Feedback"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student refines initial prompt based on AI response to improve output quality"

learning_objectives:
  - objective: "Use strong technical action verbs (Create, Debug, Refactor) to guide AI"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Prompt writing exercises with verification through AI tool"

  - objective: "Transform vague prompts into specific, actionable commands"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Rewrite exercise with before/after comparison"

  - objective: "Generate working code using Claude Code or Gemini CLI"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Hands-on code generation with executed output validation"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (technical action verbs, vague vs. specific, command structure, testing prompts, iterative refinement) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Students write prompts for more complex features; combine multiple verbs in single prompt; explore how different verbs affect AI output"
  remedial_for_struggling: "Use simplified verb list (just Create, Debug, Refactor first); provide more examples of vague vs. strong prompts; scaffold Exercise 3 with template"
---

## Your AI Orchestrator Moment

You're about to cross a threshold. In Lesson 1, you learned *how* AI agents think. In this lesson, you'll make your first real command and watch an AI build code for you. This isn't theoretical—by the end of this lesson, you'll have written actual working code with an AI partner.

Thousands of developers build production systems this way every day. They don't memorize Python syntax or JavaScript templates. They learn to think strategically, express intent clearly, and validate what AI produces. That's what you're learning right now.

In 40 minutes, you'll understand why a single word can mean the difference between AI generating exactly what you need and AI generating generic boilerplate you'll spend an hour debugging.

---

## WHAT: Technical Action Verbs - The Vocabulary of AI Direction

**Definition**: Technical action verbs are precise, unambiguous words that tell AI agents exactly what to do. Instead of vague language like "help me" or "make something," strong verbs communicate intent with laser focus.

Think of it like giving instructions to a contractor:
- **Weak**: "Help me with a roof" → Contractor asks 20 clarifying questions
- **Strong**: "Build a pitched roof with 8-inch trusses and architectural shingles" → Contractor knows exactly what to build

AI agents work the same way. Strong verbs eliminate confusion.

### The 8 Core Developer Verbs

These eight verbs appear constantly in software development work. Master them, and you'll guide AI effectively across 90% of development tasks.

#### 1. **Create** — Build new features or components from scratch

**Definition**: Construct something entirely new that doesn't exist yet.

**When to use**: Building new functions, designing new API endpoints, writing new classes, implementing new features

**Example prompt**: "Create a Python function that validates email addresses using regex"

**What you get**: AI generates complete, working code for a new function

**Red flags avoided**: Generic tutorials (vague), unclear scope (missed requirements)

---

#### 2. **Debug** — Identify and fix specific issues in existing code

**Definition**: Find problems in code that isn't working and provide solutions.

**When to use**: Code crashes, produces wrong output, throws errors, behaves unexpectedly

**Example prompt**: "Debug the login function that throws a 'KeyError: username' exception when users submit the form"

**What you get**: AI analyzes the error, explains the root cause, and provides a fix

**Why this matters**: Debugging with AI is faster than debugging alone. You provide the error message and context; AI finds the issue.

---

#### 3. **Refactor** — Restructure existing code without changing its behavior

**Definition**: Improve code quality, readability, or maintainability while keeping functionality identical.

**When to use**: Code works but is hard to read, has duplicate logic, needs better structure, is slower than it should be

**Example prompt**: "Refactor the calculateTotal function to eliminate duplicate loops and improve readability"

**What you get**: AI restructures code to be cleaner, faster, or easier to understand—but still does the same thing

**The difference**: "Refactor" is NOT the same as "fix a bug." It's about improving code that already works.

---

#### 4. **Optimize** — Improve performance, memory usage, or efficiency

**Definition**: Make code run faster, use less memory, or be more resource-efficient.

**When to use**: Code is slow, uses too much memory, makes unnecessary API calls, has bottlenecks

**Example prompt**: "Optimize the data processing function to use vectorized NumPy operations instead of loops"

**What you get**: AI redesigns code to be more efficient—often 10x-100x faster

---

#### 5. **Generate** — Produce boilerplate, tests, or documentation

**Definition**: Create repetitive content that follows patterns (tests, docs, configuration files, templates).

**When to use**: Writing test cases, creating documentation, generating configuration files, building templates

**Example prompt**: "Generate unit tests for the calculateTotal function covering edge cases like negative numbers and zero"

**What you get**: AI creates comprehensive test code following testing best practices

---

#### 6. **Analyze** — Review code for issues, patterns, problems, or improvements

**Definition**: Examine code and identify patterns, problems, security issues, or architectural concerns.

**When to use**: Code review, security audit, understanding complex code, identifying patterns

**Example prompt**: "Analyze this authentication function for security vulnerabilities"

**What you get**: AI reviews code and reports issues, suggests fixes, explains risks

**Note**: Analyze is diagnostic (find issues). Debug is corrective (fix issues). Both are useful at different times.

---

#### 7. **Migrate** — Convert code between languages, frameworks, or versions

**Definition**: Transform code from one technology to another while preserving functionality.

**When to use**: Upgrading to new versions, switching frameworks, translating between languages

**Example prompt**: "Migrate this Python 2 code to Python 3.11 syntax"

**What you get**: AI updates code to work with new versions, frameworks, or languages

---

#### 8. **Integrate** — Connect different systems, APIs, or components

**Definition**: Make separate pieces of code work together seamlessly.

**When to use**: Connecting to external APIs, combining components, adding to existing systems

**Example prompt**: "Integrate the payment processing module with the checkout workflow"

**What you get**: AI connects components together with proper error handling and data flow

---

## Before and After: Vague vs. Strong Commands

To cement this, let's contrast weak and strong prompts side by side.

| **Context** | **Weak Prompt** | **Strong Prompt** | **What Happens** |
|---|---|---|---|
| **Create a function** | "Help me with a function" | "Create a Python function named `sum_positive_numbers` that takes a list of integers and returns the sum of only positive values" | Weak: AI asks 5 clarifying questions. Strong: AI generates exactly what you need on first try |
| **Debug code** | "My code is broken" | "Debug the `read_csv` function that throws 'UnicodeDecodeError: utf-8' when processing CSV files with special characters" | Weak: AI needs more context. Strong: AI identifies the encoding issue and fixes it immediately |
| **Improve code** | "Make this better" | "Refactor the `process_orders` function to eliminate nested loops and improve readability without changing behavior" | Weak: AI guesses what "better" means. Strong: AI knows exactly what you want |
| **Optimize performance** | "This is slow" | "Optimize the `search_products` function to use indexed database queries instead of scanning all 100,000 rows" | Weak: AI tries random improvements. Strong: AI targets the specific bottleneck |

Notice the pattern: **vague prompts require AI to guess; strong prompts require AI to execute.**

---

## WHY: Why Strong Commands Matter (The Business Case)

Here's the real cost: **Time and frustration.**

### The Numbers

Research from AI development teams shows:
- **Vague prompts**: 30% success rate on first try → Average 50 minutes to get working code
- **Strong prompts**: 70% success rate on first try → Average 5 minutes to get working code

That's a **10x productivity increase** from one word change.

### Why Vague Prompts Fail

When you ask AI "Help me with authentication," the AI doesn't know:
- What language you're using (Python? JavaScript? Go?)
- What framework (FastAPI? Express? Flask?)
- What authentication method (JWT? OAuth2? Sessions?)
- What level of security you need
- How it integrates with your project

So AI picks ONE interpretation and hopes it's right. Usually it's wrong. Then you spend 45 minutes explaining what you really wanted.

### Why Strong Prompts Work

When you ask AI "Create a Python function that validates JWT tokens using the PyJWT library," AI immediately knows:
- You want Python (not JavaScript)
- You want a specific library (PyJWT)
- You want JWT validation (specific authentication method)
- You want a complete function (not a tutorial)

No guessing. No back-and-forth. Working code in minutes.

### Time Isn't the Only Benefit

Strong prompts also:
- **Reduce frustration**: You get what you asked for, not something you have to debug
- **Improve code quality**: AI can optimize for specific frameworks and patterns
- **Build confidence**: Quick wins show you that "I can direct AI to build things"
- **Scale your skills**: As tasks get complex, strong commands are the difference between working and failing

This lesson is about mastering that one difference.

---

## HOW: Command Structure — The Formula

Strong commands follow a simple, repeatable structure:

```
[VERB] + [TARGET] + [DESIRED OUTCOME]
```

Let's break this down with real examples:

### Pattern: [Verb] + [Target] + [Desired Outcome]

#### Example 1: Creating Something
```
CREATE a Python function
TARGET: that calculates the Fibonacci sequence
OUTCOME: up to n terms with type hints
```

**Full prompt**: "Create a Python function that calculates the Fibonacci sequence up to n terms with type hints"

---

#### Example 2: Debugging Something
```
DEBUG the user registration endpoint
TARGET: that returns 500 errors
OUTCOME: when email already exists
```

**Full prompt**: "Debug the user registration endpoint that returns 500 errors when the email already exists in the database"

---

#### Example 3: Refactoring Something
```
REFACTOR the authentication module
TARGET: currently using session-based auth
OUTCOME: to use JWT tokens instead
```

**Full prompt**: "Refactor the authentication module to use JWT tokens instead of sessions while maintaining backward compatibility"

---

#### Example 4: Optimizing Something
```
OPTIMIZE the database query
TARGET: that loads all user records
OUTCOME: to use pagination and indexing
```

**Full prompt**: "Optimize the database query that loads all user records to use pagination and database indexing for faster performance"

---

#### Example 5: Analyzing Something
```
ANALYZE the login function
TARGET: for security issues
OUTCOME: like SQL injection and password storage
```

**Full prompt**: "Analyze the login function for security vulnerabilities including SQL injection, weak password hashing, and insecure token storage"

---

### Why This Structure Works

The structure works because it gives AI:
1. **Action** (Verb): What operation to perform
2. **Scope** (Target): What piece you're talking about
3. **Success criteria** (Outcome): How you'll know if it's right

AI fills in all the technical details because you've eliminated ambiguity.

---

## PRACTICE: Three Exercises (20 minutes total)

### Exercise 1: Identify Strong Commands (5 minutes)

**Task**: Read these five prompts. Identify which uses the strongest command.

1. "I need something for user authentication"
2. "Help me make a login system"
3. "Create a user authentication function with email validation and bcrypt password hashing"
4. "Can you do authentication?"
5. "Fix my login code"

**Success criteria**: Correctly identify the strongest prompt + explain why

<details>
<summary>Answer (Click to reveal)</summary>

**Correct answer: Prompt #3**

Why it's strongest:
- Uses clear verb: "Create"
- Specifies target: "user authentication function"
- Specifies outcome: "email validation and bcrypt password hashing"
- Eliminates ambiguity: AI knows exactly what to build

Why others are weak:
- #1: No verb, vague ("something")
- #2: Weak verb ("help"), vague scope ("system")
- #4: Not a command, asks permission
- #5: Wrong verb (Fix assumes there's existing code)

</details>

---

### Exercise 2: Rewrite Vague Prompts to Strong Commands (10 minutes)

**Task**: Transform these vague prompts into strong commands using the [Verb] + [Target] + [Outcome] structure.

**Prompt 1**: "Help me with a function"

<details>
<summary>Example answer (Click to reveal)</summary>

**Strong rewrite**: "Create a Python function that calculates the area of a rectangle given length and width parameters, with type hints and a docstring"

Why this is better:
- Verb: Create (clear action)
- Target: Python function (specific scope)
- Outcome: Calculate area of rectangle with specific requirements (what success looks like)

</details>

**Prompt 2**: "Make my code better"

<details>
<summary>Example answer (Click to reveal)</summary>

**Strong rewrite**: "Refactor the calculateTotal function to reduce cyclomatic complexity, eliminate duplicate code, and improve readability without changing its behavior"

Why this is better:
- Verb: Refactor (not "better," specific action)
- Target: calculateTotal function (exactly which code)
- Outcome: Specific improvements (complexity, duplication, readability)

</details>

**Prompt 3**: "Something is broken"

<details>
<summary>Example answer (Click to reveal)</summary>

**Strong rewrite**: "Debug the database connection function that throws 'connection refused' errors when connecting to PostgreSQL on port 5432"

Why this is better:
- Verb: Debug (diagnostic action)
- Target: database connection function (exact component)
- Outcome: Fix specific error (connection refused, specific context)

</details>

---

### Exercise 3: Write Your Own Prompt and Test with AI (5 minutes)

**This is your first code generation. Your moment.**

**Task**:
1. Choose a simple task you'd like to build (examples below)
2. Write a prompt using the [Verb] + [Target] + [Outcome] structure
3. Test it with Claude Code or Gemini CLI
4. Observe: Does AI understand your intent? Is the output usable?

**Starter ideas** (pick one):
- Create a function that checks if a number is prime
- Create a function that reverses a string without built-in methods
- Create a function that counts how many times each word appears in text
- Create a function that converts temperatures from Celsius to Fahrenheit
- Debug a function that should return the maximum value in a list but returns the minimum instead

**Your prompt structure**:
```
[Verb] a [Language] [Type] that [Desired outcome]
```

**Examples**:
- "Create a Python function that checks if a number is prime and returns True or False"
- "Create a JavaScript function that reverses a string without using built-in reverse methods"
- "Create a Python function that counts word frequency in a text and returns a dictionary"

**Success criteria**:
- Your prompt follows [Verb] + [Target] + [Outcome] structure ✓
- AI understands your intent ✓
- Output is functional (even if not perfect) ✓
- You can read and understand what the AI generated ✓

**Important**: This is your FIRST code generation. The code doesn't need to be perfect. If there are small issues, that's completely normal—Lesson 5 will teach you how to refine and validate. For now, celebrate that **you can direct an AI to generate code.**

---

## Try With AI: Testing Vague vs. Strong Side-by-Side

This is where the lesson comes alive. You'll see the difference firsthand.

### Setup

Open your AI tool (choose one):
- **Claude Code** (if you have it set up): Recommended for this lesson
- **Gemini CLI** (if you prefer): Works exactly the same way
- **ChatGPT web** (free fallback): Open https://chatgpt.com in your browser

If you've already set up an AI companion tool from previous lessons, feel free to use it. The principles work with any AI agent.

---

### Test 1: Vague Command (Observe the confusion)

**Copy and paste this prompt exactly**:

```
Help me with Python code
```

**Observe**:
- Does AI understand what you want? Probably not.
- Does it ask clarifying questions? Most likely.
- Could you copy-paste the output and use it immediately? Doubtful.

**Note the feeling**: This is what happens without strong commands. AI is intelligent but literal. Vagueness forces it to guess.

---

### Test 2: Strong Command (Observe the precision)

**Copy and paste this prompt exactly**:

```
Create a Python function named add_numbers that takes two integer parameters and returns their sum. Include type hints and a docstring explaining what the function does.
```

**Observe**:
- Does AI understand what you want? Exactly.
- Does it ask clarifying questions? No—it gets to work.
- Can you copy-paste the output immediately? Yes, probably.

**Note the difference**: Same AI agent. Same capability. But one prompt gets immediate, usable results. The other wastes time.

---

### Test 3: Execute Your Own Prompt (Your first milestone)

**Copy your prompt from Exercise 3 above** (the one you wrote yourself).

**Paste it into your AI tool** and run it.

**Observe**:
- Did the AI understand your intent?
- Is the generated code something you could use?
- Are there parts you don't understand? (That's okay—ask AI to explain)
- Does the output have the structure you expected?

**Reflection Questions**:

- **Which prompt saved more time?** Vague or strong?
- **Why did the strong command produce better results?**
- **Can you identify the verb, target, and outcome in your Exercise 3 prompt?**
- **What would you change about your prompt to make it even stronger?**

---



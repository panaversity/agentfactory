---
title: "Build Your First Spec: Basic Calculator"
chapter: 30
lesson: 4
duration: "75-90 minutes"
skills:
  - name: "Specification Writing"
    proficiency: "B1"
    category: "Technical"
  - name: "Requirements Clarity"
    proficiency: "B1"
    category: "Soft"
  - name: "User Story Writing"
    proficiency: "A2"
    category: "Soft"
  - name: "Acceptance Criteria Definition"
    proficiency: "B1"
    category: "Technical"
learning_objectives:
  - "Write user stories that express user intent without prescribing implementation (A2)"
  - "Define clear acceptance criteria that AI agents can execute (B1)"
  - "Specify edge cases and error handling for all four arithmetic operations (B1)"
  - "Experience the complete spec-first workflow across a real feature (B1)"
---

# Build Your First Spec: Basic Calculator

You now understand what SDD is. But understanding is not the same as doing. Before we explore opinionated frameworks, **let's get to the core of specification-first thinking**.

**This lesson**: You and your AI companion will collaborate to write a complete specification for a **basic calculator with all four operations** (add, subtract, multiply, divide)—no framework, no templates, just pure spec-first thinking building a real feature.

---

### How the Collaboration Works

Throughout this lesson, you'll see prompts like this:

```
Tell your companion:
[prompt text here]
```

When you see these:
1. **Copy the prompt** and paste it into your AI tool
2. **Read the AI's response** carefully
3. **Copy useful parts** into your `spec.md` file
4. **Ask follow-up questions** if anything is unclear
5. **Refine through dialogue** until the spec is clear

**You're not just reading this chapter—you're actively building a spec with AI.**

---

## The Project: Basic Calculator Module

You're going to specify a **calculator module** with four core operations. This is perfect for learning because:

- ✅ **Everyone knows what calculators do** (no domain expertise)
- ✅ **Simple enough to complete in one lesson** (4 operations)
- ✅ **Complex enough to reveal real challenges**:
  - Type handling (integers vs floats)
  - Edge cases (division by zero, negatives, zero)
  - Error handling (invalid inputs)
  - Precision issues (0.1 + 0.2)
- ✅ **Progressive complexity**: Addition is simple, division forces you to handle errors

**The insight**: A complete feature specification is more than one function—it's user stories, acceptance criteria, edge cases, and error handling working together.

## The Spec-First Workflow

Before writing any code, we'll follow this cycle:

```
1. USER STORIES  → Express what users want to do
2. ACCEPTANCE    → Define "done" criteria
3. EDGE CASES    → Think through what can go wrong
4. GENERATE      → Let AI implement from spec
5. VALIDATE      → Test if it matches intent
6. REFINE        → Update spec based on gaps
```

---

## User Stories (The "Why")

Every specification starts with **user stories**—what do users want to accomplish?

Tell your companion:

```
I'm building a basic calculator module with four operations:
addition, subtraction, multiplication, and division.

Before writing any code, help me write user stories for this calculator.
Think about:
- Who is the user? (another developer using this module)
- What do they want to do?
- Why do they need it?

Write user stories in the format:
"As a [user], I want to [action], so that [benefit]"
```

Your companion might provide:

> **User Story 1: Addition**
> As a developer, I want to add two numbers together, so that I can calculate sums in my application.
>
> **User Story 2: Subtraction**  
> As a developer, I want to subtract one number from another, so that I can calculate differences.
>
> **User Story 3: Multiplication**
> As a developer, I want to multiply two numbers, so that I can calculate products.
>
> **User Story 4: Division**
> As a developer, I want to divide one number by another, so that I can calculate quotients.
>
> **User Story 5: Error Handling**
> As a developer, I want clear error messages for invalid operations (like division by zero), so that I can handle edge cases gracefully.

**Good start!** User stories capture _intent_ without prescribing _implementation_.

## Acceptance Criteria (The "What")

For each user story, we need **acceptance criteria**—specific conditions that must be true for the story to be "done."

Ask your companion:

```
For each user story, define acceptance criteria with specific examples.

Format:
- GIVEN [initial context]
- WHEN [action taken]
- THEN [expected outcome]

Include:
- Happy path scenarios
- Edge cases (zero, negatives, large numbers)
- Type handling (integers, floats, mixed)
- Error cases (division by zero, invalid types)

Use Python 3.12+ type hints in function signatures.
```

Your companion creates detailed acceptance criteria.

**ITERATION EXAMPLE**: Your AI might ask:

> "For acceptance criteria, should I include scenarios for:
> - Mixed types (int + float)?
> - Very large numbers (beyond float precision)?
> - Negative numbers in all operations?
> - What about the subtract(a, b) order—does subtract(5, 3) = 2 or -2?"

**You respond** (refining the spec):

```
Yes, include all those scenarios. For subtract(a, b), the result is a - b, so subtract(5, 3) = 2.
Also add a scenario for 0.1 + 0.2 to document IEEE 754 precision behavior.
```

**AI refines** acceptance criteria based on your clarifications.

**See the iteration?** AI asks questions → You clarify → Spec improves. This happens throughout the process.

#### 💬 AI Colearning Prompt
> "Why do AI agents ask clarifying questions during spec-writing instead of just making assumptions? How does this question-asking improve the final specification quality?"

## Edge Cases & Design Decisions (The "Gotchas")

Acceptance criteria covered **expected behavior**. Edge cases reveal **boundary conditions** where specs must be most precise.

Ask your companion:

```
Analyze the calculator operations for edge cases and design decisions:

1. Floating point precision (0.1 + 0.2 ≠ 0.3)
2. Division by zero (error or special value?)
3. Type preservation rules (when int vs float?)
4. Zero behavior across operations (identity? absorbing?)
5. Negative number handling
6. Very large number limits

For each, specify EXACT behavior and reasoning.
```

Your companion identifies critical edge cases.

---

## Reflection: What Did Edge Cases Teach Us?

Edge cases aren't afterthoughts—they're where **specifications earn their value**:

1. **Floating point precision:** Forced us to document tolerance expectations
2. **Division by zero:** Made us choose explicit error handling
3. **Type rules:** Required clear policy on int/float preservation
4. **Zero behavior:** Revealed operation-specific special cases
5. **Negative signs:** Made mathematical rules explicit
6. **Large numbers:** Exposed difference between int and float limits

**Key insight:**

> "The happy path is obvious. Edge cases are where you prove you've thought deeply about your specification." —SDD Practitioner

When writing any spec, ask:

- ✅ What can go wrong?
- ✅ What are the mathematical/computational limits?
- ✅ What assumptions am I making?
- ✅ How will users expect errors to surface?

#### 🎓 Expert Insight
> In AI-native development, edge cases aren't bugs waiting to happen—they're design decisions waiting to be made. When you specify "division by zero raises ZeroDivisionError" instead of leaving it ambiguous, you're not being pedantic. You're preventing your AI from making a choice you'll regret at 2 AM when production breaks.

## The Complete Calculator Specification

Now let's compile user stories, acceptance criteria, and edge cases into one complete spec document.

```
Now create a calc/spec.md file and document the complete specification there. Also initialize a repo and commit it to github.

**What makes this a good specification?**

1. **User-Centric:** Starts with user stories (why features exist)
2. **Type-Explicit:** Clear signatures with Python 3.12+ union types
3. **Edge-Case Complete:** Documents all "gotcha" behaviors
4. **Testable:** Concrete test scenarios, not prose descriptions
5. **Scoped:** Explicitly states what's out of scope
6. **Unambiguous:** No room for interpretation (e.g., "division always returns float")
```

## Quality Iteration (Validate and Refine)

You've drafted a spec collaboratively. Now comes the critical step: **validating your spec against quality standards** before you ask AI to implement.

Remember the quality checklist from Lesson 3? Use it now.

### Step 1: Apply the Quality Checklist

Review your calculator spec against these criteria:

```
✅ Clear Intent?
   - Does the spec explain what the calculator is for?
   - Do user stories explain "why," not just "what"?

✅ Testable Success Criteria?
   - Can you write tests that prove success?
   - Are acceptance criteria specific (examples, not vague descriptions)?

✅ Explicit Constraints?
   - What types can operations accept? (int, float, both?)
   - What about very large numbers? Negative numbers?
   - What operations return float vs int?

✅ Edge Cases Covered?
   - What about division by zero? 0 + 0? 0.1 + 0.2?
   - How do you handle type preservation across operations?
   - What happens with invalid inputs?

✅ No Implementation Details?
   - Does your spec avoid prescribing HOW to build it?
   - Could a different AI implement it and still be correct?
```

### Step 2: Identify Vagueness (AI as Co-Reviewer)

Ask your companion:

```
Review my calculator spec for ambiguities:

[Paste your current spec.md here]

Which parts are unclear?
- Operations: Do you know exactly which operations to implement?
- Input handling: Do you know what types are acceptable?
- Output: Do you know what type each operation returns?
- Error handling: Do you know exactly what to do with division by zero?
- Edge cases: Are all boundary conditions explicit?

For each ambiguity, explain what assumptions you would make if it's not specified.
```

Your companion identifies gaps (like "spec doesn't say if 1 + 1.0 returns int or float").

### Step 3: Refine (You as Teacher)

For each gap your companion identified, refine your spec with explicit answers:

**Before (vague)**:
```
Operations: Addition, subtraction, multiplication, division
Error handling: Handle errors gracefully
```

**After (explicit)**:
```
Operations:
- add(a, b): Returns int | float (preserves input types)
- subtract(a, b): Returns int | float (preserves input types)
- multiply(a, b): Returns int | float (preserves input types)
- divide(a, b): Always returns float (consistency over type preservation)

Error handling:
- Division by zero: Raises ZeroDivisionError with message "Cannot divide by zero"
- Invalid types: Type hints expected; runtime validation not required
```

### Step 4: Validate Together (Convergence)

Ask your companion:

```
I've updated my spec. Before you implement, tell me:
- Can you understand each operation's exact behavior?
- Are there any remaining ambiguities?
- What assumptions are you NOT making because the spec is explicit?

[Paste refined spec]
```

**This convergence loop is crucial:** AI validates your spec clarity without implementing yet. Both of you improve the specification together.

**Key insight:** A good spec should make AI's implementation obvious and correct, not require the AI to guess or ask questions during coding.

---

## Test Your Specification (Co-Learning in Action)

Now comes the crucial moment. Ask your companion:

```

Based on the complete calculator specification, implement all four functions in a file called `calculator.py`.

Include:

- Type hints as specified
- Docstrings with examples
- Error handling for division by zero
- No additional features beyond the spec

````

**Watch what happens.** Your companion will exhibit one of three behaviors:

### ✅ Scenario 1: Clear Spec → Clean Implementation

Your companion generates complete, correct implementations
**Success!** The spec was clear enough that AI:

- ✅ Used correct type hints (`int | float`)
- ✅ Returned `float` for division (even when exact)
- ✅ Handled division by zero with explicit check
- ✅ Wrote helpful docstrings with examples

---

### ❌ Scenario 2: Spec Had Gaps → AI Asks Questions

Your companion generates the code but asks clarification:

> "The specification doesn't mention runtime type validation. Should I add checks like:
>
> ```python
> if not isinstance(a, (int, float)) or not isinstance(b, (int, float)):
>     raise TypeError('Arguments must be int or float')
> ```
>
> Or rely on Python's type hints and let mypy/Pylance catch type errors at development time?"

**Good!** The spec had a gap. The AI is asking **before** making assumptions. You refine:

```
Update spec "Out of Scope" section:
- "Input validation beyond type hints: We rely on Python's static type checking (mypy, Pylance). No runtime `isinstance()` checks needed."
```

Then ask AI to regenerate. Now it knows to trust the type system.

---

### ⚠️ Scenario 3: Spec Was Ambiguous → AI Makes Wrong Assumptions


**Problems:**

1. `add()` converts `int` results to `float` (spec said preserve `int`)
2. `divide()` returns `0.0` for division by zero (spec said raise error)

**You refine the spec:**

```
Update "Type Handling" section with example:
- ❌ DON'T: return float(a + b)  # Loses int type
- ✅ DO: return a + b  # Python preserves types naturally

Update "Division By Zero" section:
- Must raise ZeroDivisionError with message "Cannot divide by zero"
- Do NOT return 0, None, inf, or any other value
```

Then ask AI to regenerate. Now it implements correctly.

---

## Validate and Iterate

Now let's validate the implementation against our specification using all the test scenarios. You can carefully review the code and tests generated.

---

### The Specification-Development Feedback Loop

This is **AI-native specification development** in action:

1. **You collaborated with AI to draft the specification** (user stories, edge cases, acceptance criteria)
2. **AI generated code from your spec** (showing its understanding)
3. **You validated output** (testing if spec was clear)
4. **Gaps revealed** (Scenario 2: AI asked questions; Scenario 3: AI made wrong assumptions)
5. **You refined spec with AI's help** (learning from failures)
6. **AI regenerated** (both improved together)

**Key insight:** Specifications improve through iteration. When AI generated wrong code (Scenario 3), it wasn't AI's fault—it was an **ambiguous specification**. The failure taught you where precision was needed. Professional specs are written iteratively, using AI as a co-reasoning partner to catch gaps early.

#### 💬 AI Colearning Prompt
> "Reflect on the calculator spec we just built together. Identify one moment where (1) AI taught me something I didn't consider, (2) I taught AI a constraint it didn't assume, and (3) we converged on a better spec through iteration. How is this different from 'prompting AI to write code'?"

---

### Reflection: The Co-Learning Pattern

Notice what just happened across all steps:

**Quality Iteration (Parts 1-4a):**
- You drafted spec with AI (user stories, acceptance criteria, edge cases)
- AI identified ambiguities: "Does 1 + 1.0 return int or float?"
- You taught AI: "Division always returns float for consistency"
- Together, you refined spec before any code was written
- **This is convergence at the specification level—neither perfect alone, both better together**

**AI as Teacher** (Scenario 2):
- AI suggested runtime type validation pattern you hadn't considered
- AI asked "type hints or isinstance checks?" revealing a specification gap
- You learned from AI's question (type checking strategies)

**You as Teacher** (Scenario 3):
- AI assumed division by zero should return 0 (wrong)
- You corrected: "Must raise ZeroDivisionError" (domain knowledge)
- AI learned your constraint and regenerated correctly

**Convergence Together** (All scenarios):
- Draft spec → Quality check → Identify gaps → Refine with AI → Code generation → Validation → Remaining gaps revealed → Spec refinement → Regeneration
- Neither human nor AI could have written the perfect spec alone
- Iteration produced production-ready specification

This is **bidirectional learning**—both human and AI improve through collaboration. You bring domain knowledge and intent; AI brings systematic thinking and edge case discovery. Together, you create specifications that neither could write independently.

**The quality iteration step (Part 4a) is where this truly shines:** You didn't just generate code and hope for the best. You validated your spec against quality criteria first. AI helped identify vagueness. You taught it your constraints. Only then did you ask for code.

**This pattern is the foundation of AI-native software development.**

---

## What You Just Learned (By Doing)

### ✅ User Stories Come Before Technical Specs

You started with **why** (user needs), not **how** (implementation). This forced you to think from the user's perspective before diving into types and edge cases.

**Traditional approach:**

1. Jump to code → "Let's implement add(), subtract..."
2. Discover requirements during debugging

**Spec-first approach:**

1. Define user value → acceptance criteria → edge cases → spec
2. AI implements, tests validate

### ✅ Progressive Complexity Reveals Spec Requirements

- **Addition:** Simple, commutative, straightforward
- **Subtraction:** Order matters, negative results
- **Multiplication:** Zero is special (absorbing element)
- **Division:** Requires error handling (zero check), always returns float

Each operation taught you something about **specification decisions**. Division forced you to think about error handling in ways addition didn't.

### ✅ Edge Cases ARE the Specification

The happy path (2 + 3 = 5) is obvious. The **real specification** is:

- What happens with 0.1 + 0.2? (floating point precision)
- What happens with divide(5, 0)? (error handling)
- What type is divide(10, 2)? (design decision: always float)

**Specs make implicit assumptions explicit.**

### ✅ Test Scenarios ARE Executable Contracts

Your test cases weren't just tests—they **defined** what "correct" means:

```python
assert divide(10, 2) == 5.0  # Not 5! This is a contract.
```

The spec isn't separate from tests; **tests ARE the specification** in executable form.

### ✅ Co-Learning Through Validation

When AI generated wrong code (Scenario 3), you learned your spec was ambiguous. When AI asked questions (Scenario 2), you learned your spec had gaps. **This is how specs improve**—through iteration, not perfection on first try.

**The AI-native approach**: You used AI to help write the specification itself (user stories, edge cases, acceptance criteria), not just implement it. This catches problems earlier and produces clearer specs.

### ✅ Specification is a Design Activity

You made **design decisions**:

- Division always returns float (even when exact) → consistency over type preservation
- Division by zero raises exception (not None, not 0.0) → explicit over silent
- Accept IEEE 754 float precision (not Decimal) → simplicity over exactness

**Specifications force you to think through design before coding.**

#### 🤝 Practice Exercise

> **Ask your AI**: "I want to add a modulo operation (remainder after division) to the calculator. Help me write the complete specification: user story, acceptance criteria, edge cases, and test scenarios. What design decisions do I need to make?"

**Expected Outcome**: Your AI will ask about edge cases like modulo by zero, negative numbers, float vs int behavior, and help you make explicit design decisions before implementing.

---

## Extension Challenges

You've learned SDD with a complete 4-operation calculator. Now extend your skills:

### Challenge 1: Add Exponentiation

**Your Prompt:**

```
Using the specification structure from the calculator (user stories, acceptance criteria, edge cases, tests), help me add a power(base, exponent) function.

Think about:
- What's the user story? (Why do users need exponentiation?)
- What about power(2, 0)? power(0, 0)? power(-2, 0.5)?
- Should exponent be int only, or int | float?
- What about very large results (2^1000)?
- Error cases: negative base with fractional exponent?

Write complete specification before implementing.
```

This teaches you how **new features extend existing specs** while maintaining consistency.

---

### Challenge 2: Build a CLI Calculator

**Your Prompt:**

```
Create a CLI wrapper around the calculator module:

User story:
As a user, I want a command-line interface, so that I can perform calculations interactively.

Acceptance criteria:
- Read operation and two numbers from command line
- Support: add, subtract, multiply, divide
- Display result or error message
- Handle invalid inputs gracefully

Example usage:
$ python calc_cli.py add 5 3
Result: 8

$ python calc_cli.py divide 10 0
Error: Cannot divide by zero

Write specification first (user stories, acceptance criteria, edge cases).
Then generate implementation.
```

This teaches you how **specs work across layers** (business logic vs UI).

---

### Challenge 3: Map to Test-Driven Development Stages
Map your specification to TDD stages:

**Your Prompt:**

```
Compare specification-driven development (what we just did) with test-driven development:

SDD Process:
1. User stories (intent)
2. Acceptance criteria (testable requirements)
3. Edge cases (design decisions)
4. Complete spec (contract)
5. AI generates implementation
6. Validate against tests

TDD Process:
1. Write test
2. Run test (it fails—red)
3. Write minimal code to pass (green)
4. Refactor (improve)
5. Repeat

Questions:
- When would you use SDD vs TDD?
- Can they work together?
- What if you wrote specs FIRST, then used TDD to implement them?
```

This teaches you **when different methodologies apply** in AI-native development.

---

**You have mastered the core skill:**

> **Translating user intent → acceptance criteria → edge cases → complete specification → validated implementation**

**You're now thinking specification-first.** 🎯

This isn't just theory—you built a **complete, tested, production-ready calculator module** by specifying first, coding second.

**That's the power of Specification-Driven Development.**

---

## Try With AI

You've built a complete calculator specification collaboratively. Now deepen your spec-writing mastery:

**🔍 Explore Advanced Edge Cases:**
> "What edge cases exist for exponentiation that don't exist for basic arithmetic? Think about 0^0, negative bases with fractional exponents, and very large results. Help me specify these correctly."

**🎯 Practice Spec Iteration:**
> "Review my calculator specification [paste your spec]. What ambiguities remain? What edge cases did I miss? What design decisions are implicit that should be explicit?"

**🧪 Test Spec Completeness:**
> "Generate code from my calculator spec, then intentionally introduce an ambiguity (like not specifying division by zero behavior). Show me what goes wrong when the spec has gaps."

**🚀 Apply to Real Projects:**
> "I need to build [describe your actual feature]. Walk me through writing a complete specification using the calculator example as a template: user stories → acceptance criteria → edge cases → complete spec."

---

---
title: "Prompts as Specifications"
chapter: 10
lesson: 1
part: 3
duration_minutes: 30

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Understanding Prompts as Specifications"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student explains prompt engineering as specification skill using Jake Heller's framework"

learning_objectives:
  - objective: "Explain prompt engineering as 'specification skill' for AI execution"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Comparison exercise identifying specification vs vague prompts"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (prompts as specs, WHAT vs HOW, spec-first, iteration 60%→97%, quality=output) at A2 limit ✓"
---

# Prompts as Specifications

You're about to learn something that separates AI-native developers from those who struggle with AI tools: **prompts are not commands—they're specifications**.

This distinction matters because specifications define **what should be built**, while commands simply tell tools **how to execute**. When you master prompt engineering as a specification skill, you're not just "using AI"—you're **directing intelligent systems** to implement your vision with the same precision a product manager uses to guide engineering teams.

By the end of this lesson, you'll understand why Jake Heller—founder of Casetext, which sold to Thomson Reuters for $650 million—spent weeks refining prompts to transform **60% accuracy into 97% accuracy**. You'll see why prompt quality determines output quality, and you'll begin thinking like a specification writer rather than a command typer.

---

## The Casetext Story: When Prompts Became Product

In 2022, Jake Heller's company Casetext paused their $20M/year legal software business to build something revolutionary: CoCounsel, an AI legal assistant powered by GPT-4.

They faced a critical question: **How do you get AI to produce legal work that meets professional standards?**

The answer wasn't "train a better model" or "add more features." The answer was **prompt engineering as rigorous specification writing**.

Here's what Jake Heller discovered:

> "Spend weeks tweaking prompts to get from 60% accuracy to 97%+. Most people quit too early."
>
> — Jake Heller, Founder of Casetext (Y Combinator talk, 2024) [[20:03]](https://www.youtube.com/watch?v=l0h3nAW13ao&t=1203s)

**What this means**: A legal AI that's correct 60% of the time is worthless (you can't trust it). A legal AI that's correct 97% of the time becomes a $650 million product. The difference? **Weeks of iterative prompt refinement**.

This is prompt engineering at the professional level: **specification writing that produces reliable, production-quality outputs**.

---

## What vs How: The Specification Mindset

Traditional programming taught developers to think in terms of **HOW**:

- "HOW do I loop through this array?"
- "HOW do I validate this email format?"
- "HOW do I handle errors in this function?"

AI-native development requires thinking in terms of **WHAT**:

- "WHAT should this validation accomplish?" → AI determines HOW
- "WHAT constitutes a valid email for our system?" → AI implements it
- "WHAT error states exist and what should happen for each?" → AI handles them

**Here's the shift**:

**Traditional Programmer**:
```
I need to write a loop that iterates through user records,
validates each email using regex, and collects invalid entries
into an array for logging.
```
*Focus: HOW to implement (loop structure, regex pattern, array manipulation)*

**AI-Native Developer**:
```
WHAT: Validate all user emails in our database
SUCCESS CRITERIA: Identify invalid emails and return list
CONSTRAINTS: Use our project's email validation rules
FORMAT: Return array of {userId, email, invalidReason}
```
*Focus: WHAT to achieve (AI figures out HOW to implement)*

**This is specification thinking**: Define the target, not the path to the target.

---

## The Specification-First Principle

Jake Heller's team at Casetext didn't start by prompting AI to "write legal research code." They started by asking:

> "Define 'what good looks like' for every step. Create objective tests."
>
> — Jake Heller [[15:27-18:16]](https://www.youtube.com/watch?v=l0h3nAW13ao&t=927s)

**Specification-first thinking** means:
1. Define success criteria BEFORE writing prompts
2. Identify what "excellent output" looks like
3. Create measurable quality thresholds
4. THEN write prompts targeting those specifications

**Example**:

**Vague Approach** (no specification):
> "Help me with a file backup script"

*AI has no idea what "help" means, what constitutes "backup," or what success looks like.*

**Specification-First Approach**:
```markdown
WHAT: Automated backup script for project files
SUCCESS CRITERIA:
  - Copies all .md files from /docs to /backups
  - Creates timestamped backup folders (backup_YYYY-MM-DD)
  - Logs success/failure for each file
  - Returns count of backed-up files

CONSTRAINTS:
  - Must preserve directory structure
  - Must skip hidden files (.git, .DS_Store)
  - Must handle permission errors gracefully

VALIDATION:
  - Test: Backup folder created with correct timestamp
  - Test: All eligible files copied
  - Test: Log file contains accurate count
```

*AI now has a clear specification to implement against.*

**The difference**: First approach generates guesswork. Second approach generates production-quality code.

---

## Prompts = Specifications for AI Execution

Think of AI as a highly skilled contractor. You wouldn't hire a contractor and say:

> "Build me something nice for my house."

Instead, you'd provide **blueprints** (specifications) showing:
- WHAT you want built (2-story addition)
- WHERE it connects (west side of house)
- WHAT materials to use (matching existing brick)
- WHAT building codes apply (residential zoning)
- WHAT the finished product looks like (architectural rendering)

**Prompts work exactly the same way.**

Good prompts are **blueprints for AI execution**:
- WHAT the AI should produce
- WHAT constraints apply
- WHAT success looks like
- WHAT quality standards to meet

**Poor prompts** leave AI guessing:
- "Make this better" → Better how? By what criteria?
- "Fix the bug" → Which bug? What's the expected behavior?
- "Create documentation" → For whom? What format? What depth?

**Excellent prompts** eliminate ambiguity:
- "Refactor this function to reduce cyclomatic complexity below 10 while preserving all existing test coverage"
- "Debug the authentication timeout occurring after 15 minutes of inactivity by checking token refresh logic"
- "Create user-facing API documentation for endpoints in `/routes/api`, targeting developers with basic REST knowledge, including curl examples for each endpoint"

---

## The 60% → 97% Iteration Framework

Here's what Jake Heller learned building CoCounsel:

**Iteration 1** (First attempt): 60% accuracy
- Vague prompt: "Analyze this legal document"
- AI misses key legal concepts
- Output unusable in professional context

**Iterations 2-5** (Refinement): 70% → 80% accuracy
- Specify legal standards to apply
- Define document sections to analyze
- Add examples of "good analysis"

**Iterations 6-10** (Precision): 85% → 90% accuracy
- Add edge case handling
- Refine success criteria
- Test against professional benchmarks

**Iterations 11-20** (Production quality): 93% → 97% accuracy
- Eliminate remaining failure modes
- Add validation steps
- Incorporate feedback from actual lawyers

**Key insight**: "Most people quit too early" [Jake Heller, 20:03]

The first prompt gives you **60% results**. Professional results require **weeks of iteration**. This is not "quick AI hacks"—this is **rigorous engineering discipline applied to prompts**.

---

## Garbage In, Garbage Out: Prompt Quality = Output Quality

The oldest rule in computing applies to AI:

**Garbage prompts → Garbage outputs**
**Excellent prompts → Excellent outputs**

Let's see this in action with five real examples.

### Example 1: Bug Fix Request

**❌ Vague Prompt** (Specification-free):
```
"Fix the login bug"
```

**Why this fails**:
- Which login bug? (There might be multiple)
- What's the current behavior? (Unknown)
- What's the expected behavior? (Unspecified)
- How do we know it's fixed? (No validation criteria)

**AI's response**: Guesswork. AI might fix the wrong bug, or "fix" something that wasn't broken.

---

**✅ Specification Prompt**:
```
WHAT: Fix authentication timeout bug
CURRENT BEHAVIOR: Users logged out after 15 minutes idle time
EXPECTED BEHAVIOR: Users stay logged in for 24 hours or until explicit logout
ROOT CAUSE ANALYSIS NEEDED: Check token expiration settings in auth/config
SUCCESS CRITERIA:
  - Token lifetime extended to 24 hours
  - Existing sessions preserved across app restart
  - Logout still works correctly
TESTING: Verify user can idle for 20 minutes without logout
```

**AI's response**: Targeted fix addressing exact issue with validation built in.

---

### Example 2: Code Refactoring

**❌ Vague Prompt**:
```
"Make this code better"
```

**Why this fails**:
- "Better" by what measure? (Speed? Readability? Memory?)
- What can change? (Structure? Algorithm? Both?)
- What must stay the same? (API? Behavior?)

---

**✅ Specification Prompt**:
```
WHAT: Refactor calculateShippingCost function
GOAL: Improve readability without changing behavior
CONSTRAINTS:
  - MUST preserve function signature (input/output unchanged)
  - MUST pass all existing tests
  - MUST eliminate nested if statements (reduce to max 2 levels)
IMPROVEMENTS SOUGHT:
  - Extract complex conditionals into named functions
  - Add explanatory comments for business logic
  - Use early returns to reduce nesting
SUCCESS CRITERIA:
  - Cyclomatic complexity < 8
  - All tests pass
  - Code review approval from peer
```

---

### Example 3: Documentation Request

**❌ Vague Prompt**:
```
"Write documentation"
```

**Why this fails**:
- Documentation for whom? (Developers? End users? Admins?)
- What depth? (Quick reference? Tutorial? API spec?)
- What format? (Markdown? JSDoc? README?)

---

**✅ Specification Prompt**:
```
WHAT: API documentation for user authentication endpoints
AUDIENCE: Frontend developers integrating our auth service
FORMAT: Markdown with code examples
SECTIONS REQUIRED:
  1. Endpoint list (/login, /logout, /refresh)
  2. Request/response schemas (JSON)
  3. Authentication flow diagram
  4. Error codes and meanings
  5. Example curl commands for each endpoint
TONE: Technical but accessible (assume REST API knowledge)
SUCCESS CRITERIA:
  - Developer can integrate auth without asking questions
  - All examples are copy-pasteable and working
```

---

### Example 4: New Feature Implementation

**❌ Vague Prompt**:
```
"Add search functionality"
```

**Why this fails**:
- Search what? (Users? Products? Documents?)
- How sophisticated? (Exact match? Fuzzy? Full-text?)
- Where does it go? (UI? API? Both?)

---

**✅ Specification Prompt**:
```
WHAT: Add product search to e-commerce dashboard
SCOPE:
  - Search products by name, SKU, or category
  - Display results in existing product table UI
  - Highlight search terms in results
BEHAVIOR:
  - Real-time search (update as user types)
  - Minimum 3 characters to trigger search
  - Case-insensitive matching
  - Show "No results found" message when appropriate
TECHNICAL CONSTRAINTS:
  - Use existing /api/products endpoint with query params
  - No database schema changes
  - Match current design system styles
SUCCESS CRITERIA:
  - Search returns results in \<500ms
  - Results accurately match search query
  - UI feels responsive (no lag during typing)
```

---

### Example 5: Testing Requirements

**❌ Vague Prompt**:
```
"Write tests"
```

**Why this fails**:
- Tests for what? (Unit? Integration? E2E?)
- What coverage? (100%? Critical paths only?)
- What framework? (Jest? pytest? RSpec?)

---

**✅ Specification Prompt**:
```
WHAT: Unit tests for EmailValidator class
FRAMEWORK: pytest
COVERAGE TARGET: 100% of EmailValidator methods
TEST CASES REQUIRED:
  Happy Path:
    - Valid email formats (user@domain.com)
    - Valid subdomains (user@mail.domain.com)
  Edge Cases:
    - Email with + symbol (user+tag@domain.com)
    - Uppercase letters (User@Domain.COM)
  Error Cases:
    - Missing @ symbol
    - Missing domain
    - Special characters not allowed
ASSERTIONS:
  - Valid emails return True
  - Invalid emails return False
  - Invalid emails raise ValueError with descriptive message
SUCCESS CRITERIA:
  - All tests pass
  - pytest coverage report shows 100% for EmailValidator
```

---

## Prompt Quality Analysis Exercise

Now it's your turn. Below are **five prompts**. Your task: Identify which are **specification-quality** (clear intent, success criteria, constraints) and which are **vague** (missing critical information).

For each prompt, ask yourself:
1. Is the WHAT clear? (What should AI produce?)
2. Are success criteria defined? (How do we know it's good?)
3. Are constraints specified? (What must/must not happen?)

**Prompt A**:
```
"Create a Git commit message"
```

**Prompt B**:
```
WHAT: Generate conventional commit message for authentication changes
FORMAT: <type>(<scope>): <description>
CHANGES MADE:
  - Added JWT token refresh endpoint
  - Updated token expiration to 24 hours
  - Fixed logout race condition
RULES:
  - Type: "feat" (new feature added)
  - Scope: "auth"
  - Description: Under 50 characters, imperative mood
  - Body: List each change with "- " bullets
SUCCESS CRITERIA:
  - Message follows conventional commits standard
  - All changes documented
  - Readable by teammate in 10 seconds
```

**Prompt C**:
```
"Help me optimize this code"
```

**Prompt D**:
```
WHAT: Optimize database query in getUserOrders function
PERFORMANCE ISSUE: Query takes 3.2s with 1000+ orders
GOAL: Reduce query time to \<500ms
APPROACH:
  - Add index on orders.userId
  - Paginate results (50 orders per page)
  - Use SELECT only needed columns (not SELECT *)
CONSTRAINTS:
  - MUST preserve sort order (newest first)
  - MUST work with existing database schema
VALIDATION:
  - Test with 5000 orders in test DB
  - Measure query time with EXPLAIN ANALYZE
  - Confirm results identical to original query
```

**Prompt E**:
```
"Explain how this Bash script works"
```

---

**Analysis** (Do this on paper or in a markdown file BEFORE looking ahead):

For each prompt (A-E), write:
- **Specification-quality?** Yes/No
- **What's clear?** (If anything)
- **What's missing?** (What would AI have to guess?)

---

**Answers**:

- **Prompt A**: ❌ Vague (No context about what commit, what changes, what format)
- **Prompt B**: ✅ Specification (Clear format, changes listed, rules specified, success criteria defined)
- **Prompt C**: ❌ Vague (What code? Optimize for what? Speed? Memory? Readability?)
- **Prompt D**: ✅ Specification (Performance goal quantified, approach suggested, constraints clear, validation defined)
- **Prompt E**: ❌ Vague (Which script? What level of detail? Explain to whom?)

**Key pattern**: Specification prompts answer **"What should the output be?"** Vague prompts leave AI guessing.

---

## What You've Learned

By analyzing these examples, you've built the foundational understanding of prompt engineering as **specification writing**:

1. **Prompts are specifications** (not commands) — They define WHAT to build, not HOW
2. **Specification-first thinking** — Define success criteria before prompting
3. **Jake Heller's iteration framework** — 60% → 97% accuracy requires weeks of refinement
4. **Garbage in, garbage out** — Prompt quality determines output quality
5. **WHAT vs HOW** — AI-native developers specify outcomes, AI determines implementation

In the next lesson, you'll learn the **anatomy of effective prompts**: the structural components (Intent → Constraints → Success Criteria) that transform vague requests into precise specifications.

For now, practice the specification mindset:
- Before prompting AI, ask: "Have I defined what 'good' looks like?"
- When AI output disappoints, ask: "Was my prompt a specification or a vague request?"
- Remember Jake Heller's lesson: "Most people quit too early." Iteration is the path from 60% to 97%.

**You're not learning to "use AI faster." You're learning to think like a product manager who writes specifications that any implementation team—human or AI—can execute flawlessly.**

This is the foundation of AI-native development.

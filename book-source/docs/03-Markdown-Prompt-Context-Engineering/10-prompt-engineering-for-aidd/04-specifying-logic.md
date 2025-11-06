---
title: "Specifying Logic - Implementation Steps for Real Features"
chapter: 10
lesson: 4
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Defining Implementation Logic"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student writes 5-8 step implementation plan that AI follows to build complex feature, demonstrating ability to think architecturally about how to build (not just what to build)"

  - name: "Algorithmic Thinking for AI Direction"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student explains why numbered steps produce better AI code than vague requirements and contrasts requirements (WHAT) with implementation logic (HOW)"

learning_objectives:
  - objective: "Write 5-8 numbered implementation steps to guide AI through complex features"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Exercise 2: Students write 8 implementation steps for to-do API endpoint; AI follows steps in generated code"

  - objective: "Prevent AI from guessing architecture by providing explicit logic flow"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Comparison exercise: vague vs. logic-specified prompts showing architectural differences in AI responses"

  - objective: "Apply logic specification to real-world development tasks"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Exercise 3: Add implementation logic to previous lesson's contextual prompt; validate AI follows logic"

cognitive_load:
  new_concepts: 2
  assessment: "2 new concepts (implementation logic vs. vague requirements, numbered step-by-step instructions) - transition to B1 with synthesis of prior elements. Leverages concepts from L1-L3 without introducing new cognitive burden. Appropriate for A2→B1 transition."

differentiation:
  extension_for_advanced: "For B1+ students: Include design patterns (Strategy pattern, Factory pattern) in logic steps; algorithmic specifications (LRU cache with TTL, graph traversal algorithms); distributed systems logic (eventual consistency, microservice choreography)"
  remedial_for_struggling: "For A2 students needing reinforcement: Provide starter templates with blanks to fill; simplified examples with 5 steps instead of 8; visual flowcharts before text-based steps; guided prompting where AI shows step-by-step thinking before implementation"
---

# Lesson 4: Specifying Logic - Implementation Steps for Real Features

You've mastered three critical skills: **clear commands**, **rich context**, and **architectural thinking**. Now comes the breakthrough moment that separates casual AI users from true software architects.

In the previous lessons, you learned to tell AI *what* you want. Now you'll learn to tell AI *how* to build it—and that distinction changes everything.

When you specify not just requirements but explicit implementation logic, something remarkable happens: **AI stops guessing and starts following your design**. The code that emerges isn't a generic template you'll spend hours adapting. It's your architecture, implemented exactly as you envisioned it.

This lesson teaches you to think like a software architect working with a builder: you don't just describe the house you want (requirements); you provide the construction blueprints with numbered steps (logic). In 50 minutes, you'll see how this transforms AI-generated code from "good enough" to "exactly what I designed."

---

## WHAT: Logic Specification vs. Vague Requirements

**Definition**: Implementation logic is the explicit, step-by-step *how* you want a feature built—not just the *what*. It's numbered instructions that guide AI through your architectural decisions.

Think of the difference this way:

**Vague Requirement** (what many developers write):
> "Create a user registration system"

**Implementation Logic** (what architects write):
> "Implement user registration following these steps:
> 1. Validate email format using regex pattern
> 2. Check if email already exists in database
> 3. Hash password using bcrypt (salt rounds = 12)
> 4. Create user record in database
> 5. Return user object without password field"

Same feature. Completely different outcomes.

### Why This Matters: Requirements vs. Logic

Consider what happens with a vague requirement. When you tell an AI agent "create a user registration system," the AI must make dozens of architectural decisions *without you*:

- How should passwords be hashed? (bcrypt? argon2? plain hash?)
- What validation rules apply? (regex for email? length requirements for password?)
- How should duplicate users be handled? (error message? redirect?)
- What fields get returned? (password included? created_at timestamp?)
- How is error handling structured? (try/except? validation decorator?)

Even an excellent AI agent will make *reasonable* guesses. But reasonable guesses aren't your design.

When you specify implementation logic, you eliminate the guessing. You become the architect, and AI becomes the builder following your exact blueprints.

**Professional Pattern**: "You design the solution architecture. AI implements it faithfully."

---

## WHY: How Logic Prevents AI from Guessing

### Without Logic: Generic Boilerplate

```plaintext
Prompt: "Create a payment processing function"

AI's Internal Thought Process:
"Hmm... payment processing... The developer didn't specify HOW.
I'll use common patterns: maybe call a payment gateway, handle success/failure...
I'll include error handling... I'll add some logging..."

Result: Generic, one-size-fits-all code that requires hours of adaptation
```

### With Logic: Tailored Implementation

```plaintext
Prompt: "Create payment processing:
1. Validate card details (card number, CVV, expiry)
2. Call Stripe API with validated card
3. Log transaction to database
4. Return result with transaction ID"

AI's Internal Thought Process:
"Clear steps. Validate → Call Stripe → Log → Return.
I'll implement exactly these steps in this order.
I'll use the Stripe API as specified.
No guessing about payment gateways or logging structure."

Result: Code that implements your exact design
```

### Time Savings

Studies with professional development teams show:

- **Without logic**: 1 prompt → AI generates code → 30-60 min debugging/adapting → working
- **With logic**: 1 prompt → AI generates code → 2-5 min validation → working

The logic specification forces you to think architecturally upfront, which means the generated code matches your mental model from the start.

---

## HOW: Writing Implementation Steps

Implementation logic comes in several forms, from simple (basic features) to advanced (algorithms, design patterns, error handling). Let's explore each.

### Form 1: Basic Feature Logic (5-8 Steps)

For straightforward features, write 5-8 numbered steps describing the implementation flow.

**Example: User Registration Endpoint**

```
Implement a user registration endpoint with these steps:

1. Accept POST request with email and password in request body
2. Validate email format using regex pattern (standard email validation)
3. Query database to check if email already exists
4. If email exists, return 409 Conflict error with message "Email already registered"
5. Hash password using bcrypt with salt rounds = 12
6. Create new user record in database with email and hashed password
7. Return 201 Created with user object (ID, email, created_at timestamp)
8. Include proper error handling for database failures (500 Internal Server Error)
```

Notice the specificity:
- Not "validate email" but "using regex pattern"
- Not "hash password" but "bcrypt with salt rounds = 12"
- Not "return user" but "return user object without password field"
- Not "handle errors" but "return 500 for database failures"

When AI sees this logic, there's no room for architectural guessing. AI implements exactly what you specified.

**Example: To-Do List API - Create Task Endpoint**

```
Implement the "create task" endpoint following this logic:

1. Accept POST request with title and description in request body
2. Validate that title is not empty (required field)
3. Validate that title is less than 100 characters
4. Set task status to "pending" by default
5. Set created_at timestamp to current time
6. Generate unique task ID (use UUID)
7. Store task in database
8. Return 201 Created with complete task object (all fields)
```

### Form 2: Advanced Logic with Design Patterns

For more complex features, specify the design pattern or architectural approach.

**Example: Implementing Payment Processing with Strategy Pattern**

```
Refactor payment processing using the Strategy pattern:

1. Create abstract PaymentStrategy base class with process_payment() method
2. Implement concrete strategies:
   - CreditCardPayment: validates card details, calls Stripe API
   - PayPalPayment: handles PayPal authentication and transfer
   - CryptoPayment: validates wallet address, integrates with blockchain API
3. Create PaymentContext class that accepts any strategy in constructor
4. PaymentContext.process_payment() delegates to the strategy's process_payment()
5. Each strategy returns uniform PaymentResult object (status, transaction_id, timestamp)
6. Client code selects strategy based on payment method (dependency injection)
7. All strategies include error handling and logging
```

This tells AI not just *what* to build but *which architectural pattern* to use and *why* (extensibility, loose coupling, easy to add new payment methods).

### Form 3: Algorithmic Logic

For features requiring specific algorithms, specify the algorithm by name or behavior.

**Example: Implementing a Cache with LRU Eviction**

```
Implement a caching layer with these specifications:

Algorithm: Least Recently Used (LRU) cache
- Max capacity: 1000 items
- Cache key: MD5 hash of request URL + query parameters
- Cache value: API response data
- Time-to-live (TTL): 5 minutes

Implementation steps:
1. On request received, compute cache key (MD5 of URL + params)
2. Check if key exists in cache and is not expired
3. If cache hit: return cached response immediately (no API call)
4. If cache miss or expired: call external API
5. Store result in cache with timestamp
6. If cache is full (1000+ items), evict least recently used item
7. Return API response
8. Handle API failures gracefully: log error but don't break request flow
```

This ensures AI implements LRU specifically (not some other cache strategy) with your exact performance characteristics (1000 items, 5-min TTL).

### Form 4: Error Handling and Edge Cases

For safety-critical features, specify error handling explicitly.

**Example: Database Transaction with Rollback**

```
Implement user account creation with transactional safety:

1. Begin database transaction
2. Create user record with email and hashed password
3. Create default user settings record
4. Create welcome email entry in notification queue
5. Commit transaction
6. If any step fails:
   - Catch exception
   - Rollback all database changes
   - Log error with details
   - Return 500 error to client with generic message
7. Return 201 with created user (do not include password)
8. Do not create user if transaction fails (atomic operation)
```

The key: Error handling isn't an afterthought. It's part of the explicit logic flow.

---

## PRACTICE: Three Progressive Exercises

### Exercise 1: Recognize Logic vs. Requirements (5 minutes)

Read these three prompts. For each, identify whether it specifies **logic** (step-by-step HOW) or **requirements** (just WHAT).

**Prompt A**:
> "Create a REST API endpoint that retrieves user data from the database"

**Prompt B**:
> "Create a GET `/users/{user_id}` endpoint: 1) Query database for user by ID, 2) If found, return user data with 200 status, 3) If not found, return 404 error, 4) Handle database connection errors with 500 status"

**Prompt C**:
> "Implement user authentication"

Which prompts specify **logic** (step-by-step) vs. just **requirements** (what to build)?

<details>
<summary>Answer</summary>

- **Prompt A**: Requirements only (WHAT - retrieve user data, but not HOW)
- **Prompt B**: Implements logic (STEP-BY-STEP - query → check → return → error handling)
- **Prompt C**: Requirements only (WHAT - but missing all HOW details)

**Key insight**: Prompt B is actionable. AI reads the numbered steps and implements exactly that. Prompts A and C leave room for AI to guess architecture.

</details>

---

### Exercise 2: Write Implementation Steps for a Real Feature (15 minutes)

**Scenario**: You're building a to-do list API. A user has requested a "create new task" feature.

**Your task**: Write 8 implementation steps that guide AI through building this endpoint.

**Starting point** (requirements only):
> "Create an endpoint that allows users to create new tasks"

**Your job**: Convert this into 8 numbered implementation steps. Consider:
- What data comes in? (request body)
- What validation is needed?
- What happens in the database?
- What gets returned?
- How are errors handled?

Write your 8 steps below:

```
Implement the "create task" endpoint with these steps:

1. [Your step]
2. [Your step]
3. [Your step]
...
8. [Your step]
```

**After writing your steps**, you'll test them in the "Try With AI" section below. The goal is to see whether AI follows your exact logic when building the endpoint.

<details>
<summary>Example Answer (for comparison)</summary>

```
Implement the "create task" endpoint with these steps:

1. Accept POST request with title, description, and priority in request body
2. Validate that title is provided and not empty (required)
3. Validate that title is less than 100 characters
4. Validate that priority is one of: low, medium, high
5. Set status to "pending" by default
6. Set created_at timestamp to current UTC time
7. Insert task record into database with all fields
8. Return 201 Created with complete task object (ID, title, description, priority, status, created_at, updated_at)
```

**Evaluation**: Did you include validation? database operation? return format? error handling? If yes to 3+ of these, you've captured the core logic.

</details>

---

### Exercise 3: Add Logic to an Existing Prompt (5 minutes)

Recall Lesson 3's contextual prompt. You built a prompt with:
- Clear command (verb + target)
- Project context (tech stack, current file)
- Four-layer context (project, code, constraints, developer)

**Your task**: Take that prompt and add 5-8 implementation steps.

**Example evolution**:

**Before** (from Lesson 3):
```
Create a user authentication function for a FastAPI project.

Context:
- Project: User management API
- Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL
- Experience: Intermediate Python, learning FastAPI
```

**After** (adding logic):
```
Create a user authentication function for a FastAPI project.

Context:
- Project: User management API
- Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL
- Experience: Intermediate Python, learning FastAPI

Implementation steps:
1. Accept username and password in request body
2. Query database for user by username
3. If user not found, return 401 Unauthorized
4. If user found, compare provided password with stored hash using bcrypt
5. If password incorrect, return 401 Unauthorized
6. If password correct, generate JWT token (expiry: 24 hours)
7. Return 200 OK with token and user ID
8. Log authentication attempt (success or failure) with timestamp
```

Notice the power: The same feature, but now with explicit architectural decisions baked in.

**Your action**: Try this with your own prompt from Lesson 3. Add 5-8 implementation steps. In "Try With AI," you'll test whether the added logic improves AI's output.

---

## Try With AI: Testing Logic vs. Vague Prompts

In this section, you'll experience the difference logic makes by testing three variations of the same request.

### Setup

You'll need:
- **Claude Code** (browser-based, easiest entry point) OR
- **Gemini CLI** (if you've set up the CLI from Lesson 2)
- If neither is accessible, **ChatGPT web** works as a fallback

### Test 1: Vague Prompt (No Logic)

Copy and paste this into your AI tool:

```
Create a function to process payments
```

**Observe**: What does AI ask? What assumptions does it make? Does it ask clarifying questions or just generate generic code?

**Expected outcome**: AI will either:
- Ask you 5+ clarifying questions (good collaboration) OR
- Generate generic payment code that requires lots of adaptation

---

### Test 2: Logic-Specified Prompt

Copy and paste this into your AI tool:

```
Create a payment processing function following these implementation steps:

1. Validate card details (check card number length, CVV, expiry date)
2. Call Stripe API with validated card information
3. If Stripe returns success, create transaction record in database with status="completed"
4. If Stripe returns error, create transaction record in database with status="failed" and store error message
5. In both cases (success or failure), log transaction to audit trail with timestamp and user ID
6. Return a PaymentResult object containing: status (success/failure), transaction_id, error_message (if failed), amount, timestamp
7. Handle network timeout errors: retry once after 2 seconds
8. If retry fails, return failure status without raising exception
```

**Observe**: Does AI follow the numbered steps? Does it implement the Stripe integration, transaction logging, and error handling as specified? Does the code match your architectural decisions?

**Expected outcome**: AI generates code that:
- Includes validation step
- Calls Stripe API
- Creates transaction records with specified statuses
- Returns PaymentResult with exact fields
- Handles retries as specified

**Comparison note**: This is likely more complete and production-ready than Test 1, with fewer iterations needed to reach working code.

---

### Test 3: Your Own Implementation Steps (From Exercise 2)

Using the 8 steps you wrote in **Exercise 2** (the to-do API endpoint), copy your prompt into your AI tool:

```
Implement the "create task" endpoint with these steps:

[Your 8 steps from Exercise 2]
```

**Observe**:
- Does AI generate code that follows your steps in order?
- Does the return format match step 8?
- Does it implement validation from your steps?
- Compare this to what AI would generate without your steps

**Reflection questions**:
1. How did specifying logic change the AI response compared to Test 1?
2. Which parts of your steps did AI follow exactly?
3. Where, if anywhere, did AI deviate from your steps?
4. If there were deviations, would you refine the steps to be more explicit next time?



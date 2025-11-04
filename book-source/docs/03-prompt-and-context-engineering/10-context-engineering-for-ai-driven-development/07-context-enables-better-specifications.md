---
title: "Context Enables Better Specifications"
sidebar_position: 7
description: "Learn why rich context is essential for writing clear specifications in AI-native development"
keywords: [context engineering, specification writing, AI-native development, planning-first, AIDD]
---

# Context Enables Better Specifications

## Introduction: The Missing Link

So far in this chapter, you've learned:
- ✓ What context engineering is (Lesson 1)
- ✓ How context windows work and degrade (Lesson 2)
- ✓ How to load context progressively (Lesson 3)
- ✓ How to compress and isolate contexts (Lesson 4)

**But why does any of this matter?**

Here's the crucial insight you need for AI-native development:

> **Context engineering isn't just about managing AI tools better. It's the foundation for writing clear specifications—and specifications are the foundation for everything you build.**

This lesson reveals the hidden connection between context and specifications.

---

## The Core Problem: Poor Context → Poor Specs → Broken Code

Let's compare two developers working on the same task.

### Scenario: Add Password Reset Feature

**Task:** "Add password reset functionality to our authentication system."

---

### Developer A: No Context (The Common Mistake)

**Developer A's Approach:**

```bash
# Immediately jumps to asking AI
claude-code prompt "Add password reset functionality to the auth system"
```

**AI's Response:**
```
Sure! I'll add password reset. Here's a basic implementation...

[Generates generic code]
```

**What Developer A gets:**
- Generic password reset code
- No integration with existing auth patterns
- Doesn't know about existing token system
- Uses different coding style than rest of project
- Missing error handling patterns
- Doesn't follow security requirements

**Developer A then spends 2 hours:**
- Debugging integration issues
- Rewriting to match existing patterns
- Adding missing security checks
- Fixing broken tests
- Asking AI "why isn't this working?"

**Total time: 3 hours of frustration**

---

### Developer B: Rich Context First (The Right Way)

**Developer B's Approach:**

```bash
# Step 1: Load context BEFORE asking
claude-code prompt "I need to add password reset. Before we start, help me understand the existing auth system.

Read and summarize:
1. src/auth/authentication.py (existing auth patterns)
2. src/services/token_service.py (how tokens work now)
3. src/models/user.py (user model structure)
4. tests/test_auth.py (testing patterns we follow)

Summarize:
- What authentication approach we use
- How tokens are currently generated/validated
- What error handling patterns we follow
- What testing patterns we follow"
```

**AI's Response:**
```
Based on the code:

Authentication Approach:
- JWT-based with access + refresh tokens
- Using redis for token blacklist
- Access: 15 min, Refresh: 7 days

Token Generation:
- Using secrets.token_urlsafe(32)
- Stored in redis with expiry
- Async/await pattern throughout

Error Handling:
- Custom exceptions (AuthenticationError, ValidationError)
- Raised with specific codes
- Converted to HTTP exceptions in API layer

Testing Patterns:
- Pytest with async fixtures
- Mock redis and database
- AAA pattern (Arrange-Act-Assert)
```

**Now Developer B asks:**

```bash
claude-code prompt "Perfect! Now let's design the password reset feature:

Requirements:
- Follow existing JWT pattern
- Use same token generation approach
- Follow same error handling
- Follow same testing patterns
- Security: Token expiry 1 hour, rate limit 5/hour

Create a specification for this feature."
```

**AI generates a detailed spec** that:
- ✓ Matches existing patterns
- ✓ Uses established token approach
- ✓ Follows project conventions
- ✓ Includes proper error handling
- ✓ Has comprehensive tests

**Developer B then:**
```bash
claude-code prompt "Implement the specification you just wrote."
```

**What Developer B gets:**
- Code that integrates perfectly
- Follows all existing patterns
- Matches security requirements
- Tests pass immediately
- No debugging needed

**Total time: 45 minutes of smooth development**

---

## Understanding the Difference

Let's break down what happened:

### Developer A's Path (No Context)

```
Vague Understanding
        ↓
    Vague Request
        ↓
   Generic Response
        ↓
    Doesn't Fit
        ↓
  Hours of Fixing
```

**Why it failed:**
- AI had no context about existing system
- Generated generic solution
- Solution didn't fit project patterns
- Integration broke things
- Lots of rework needed

### Developer B's Path (Rich Context)

```
  Load Context
        ↓
Deep Understanding
        ↓
  Clear Request
        ↓
 Tailored Response
        ↓
Works Immediately
```

**Why it succeeded:**
- AI understood existing system deeply
- Generated solution matching patterns
- Solution integrated smoothly
- Everything worked first try
- Minimal rework

---

## The Key Insight: Context → Thinking → Specification

Here's the pattern you MUST understand for AI-native development:

### Without Context

```
❌ Poor Context
    ↓
❌ Vague Thinking
    ↓
❌ Bad Specification
    ↓
❌ Broken Code
```

**Example:**

**Poor Context:** "There's an auth system somewhere..."

**Vague Thinking:** "I need password reset... I think?"

**Bad Specification:** 
```
Add password reset
[That's it. No details.]
```

**Broken Code:** Generic implementation that doesn't fit

---

### With Context

```
✅ Rich Context
    ↓
✅ Clear Thinking
    ↓
✅ Precise Specification
    ↓
✅ Working Code
```

**Example:**

**Rich Context:** "JWT-based auth, redis token blacklist, specific patterns..."

**Clear Thinking:** "Password reset should use same token approach, follow same patterns, integrate with redis..."

**Precise Specification:**
```markdown
# Password Reset Feature Specification

## Integration Points
- Use existing token_service.py patterns
- Store reset tokens in redis with 1-hour expiry
- Follow authentication.py error handling

## Token Format
- Generate with secrets.token_urlsafe(32)
- Key format: "reset_token:{token}"
- Value: user_id
- Expiry: 3600 seconds

## API Endpoints
POST /auth/request-reset
POST /auth/reset-password

[... detailed spec ...]
```

**Working Code:** Integrates perfectly, tests pass

---

## WHAT: Context-First Specification Workflow

**Context-First Specification** is the practice of loading all relevant context BEFORE writing a specification or asking AI to build anything.

**The Analogy:**

**Writing an Essay:**

**Wrong Way:**
- Start writing without reading the book
- Make stuff up as you go
- Essay is vague and incorrect

**Right Way:**
- Read the relevant chapters first
- Understand the topic deeply
- Then write a clear, accurate essay

**Context-first workflow applies this to development.**

---

## WHY: Why Context Must Come First

### Reason 1: You Can't Specify What You Don't Understand

**Think about it:**
- Can you write clear requirements without understanding the system?
- Can you make good decisions without knowing existing patterns?
- Can you design integration without knowing what you're integrating with?

**Answer: No.**

**Specifications require understanding. Understanding requires context.**

---

### Reason 2: AI Reflects Your Understanding

**Critical insight about AI:**

The AI doesn't have independent knowledge of your project. It only knows:
1. What you tell it
2. What code you load
3. What documentation you share

**If you have poor understanding (from poor context):**
- You ask vague questions
- AI gives vague answers
- You write vague specifications
- Code doesn't work

**If you have deep understanding (from rich context):**
- You ask specific questions
- AI gives specific answers
- You write precise specifications
- Code works

**The AI is your co-thinker, not your replacement. If your thinking is vague, the AI's output will be vague too.**

---

### Reason 3: Specifications Are Commitments

**What is a specification?**

A specification is a **commitment** to:
- What you're building
- How it will work
- How it integrates
- What patterns it follows

**You can't commit to something you don't understand.**

**Example of a bad commitment:**
```
"Add authentication"
[No details = No real commitment]
```

**Example of a good commitment:**
```markdown
# Authentication Specification

Approach: JWT with access + refresh tokens
Token Storage: Redis for blacklist
Access Token: 15 minutes
Refresh Token: 7 days
Endpoints: /auth/login, /auth/logout, /auth/refresh
Error Handling: Custom exceptions with codes
Testing: Pytest async fixtures, AAA pattern

[This is a real commitment you can build from]
```

**Rich context enables real commitments.**

---

## HOW: The Context-First Workflow

Here's the step-by-step process you'll use for **every** feature from now on.

### Step 1: Identify What Context You Need

**Before starting, ask:**
- What existing code will this feature interact with?
- What patterns does this project follow?
- What decisions have already been made?
- What constraints exist?

**Template Questions:**
```
For this feature, I need to understand:

1. **Integration Points**: What existing code will I touch?
2. **Patterns**: What conventions does this project follow?
3. **Constraints**: What rules or limitations exist?
4. **Dependencies**: What other features does this rely on?
```

**Example (Password Reset Feature):**
```
For password reset, I need to understand:

1. Integration Points:
   - Existing authentication system
   - Current token generation/validation
   - Email sending service
   - User model structure

2. Patterns:
   - How we handle authentication
   - Error handling conventions
   - API endpoint design
   - Testing approaches

3. Constraints:
   - Security requirements
   - Rate limiting rules
   - Token expiry policies

4. Dependencies:
   - Email service (do we have one?)
   - Redis (is it already set up?)
```

---

### Step 2: Load Context Progressively

**Use Lesson 3's progressive loading strategy!**

```bash
# Phase 1: Overview
claude-code prompt "Give me a high-level overview of how authentication currently works in this project. Don't read all files yet, just analyze the directory structure and key file names."

# Phase 2: Key Files
claude-code prompt "Now read these specific files and summarize the patterns:
1. src/auth/authentication.py
2. src/services/token_service.py  
3. src/models/user.py"

# Phase 3: Deep Dive (if needed)
claude-code prompt "I need to understand the token validation logic in detail. Read the validate_token method and explain the flow."
```

**Result:** You now have deep understanding without overloading context.

---

### Step 3: Validate Your Understanding

**Before writing the specification, test your understanding:**

```bash
claude-code prompt "Based on what you've learned, answer these questions:

1. What authentication approach does this project use?
2. How are tokens currently generated and validated?
3. What error handling patterns should I follow?
4. What testing patterns should I follow?
5. Are there any security requirements I need to follow?

If you're unsure about any of these, tell me what additional context you need."
```

**This checkpoint ensures:**
- You (and AI) actually understand the system
- No critical gaps in knowledge
- Ready to write a clear specification

---

### Step 4: Write the Specification

**Now, with rich context, you can write a clear specification:**

```bash
claude-code prompt "Based on the context we've loaded, create a detailed specification for the password reset feature.

Include:
- How it integrates with existing auth system
- What patterns it follows from the codebase
- Token generation approach (matching current approach)
- Security requirements (token expiry, rate limiting)
- Error handling (following existing patterns)
- Testing approach (following existing patterns)
- API endpoints design

Format as a markdown specification document."
```

**What you get:**
- Specification deeply aligned with existing system
- Follows established patterns
- Accounts for real constraints
- Ready to implement without surprises

---

### Step 5: Implement from Specification

**Finally, implement:**

```bash
claude-code prompt "Implement the specification you just created. Follow all patterns and conventions we identified."
```

**Because you did Steps 1-4:**
- Implementation matches reality
- Code integrates smoothly
- Tests pass
- No surprises or rework

---

## The Context Files Pattern

**Professional tip:** Create context summary files you can reuse.

### PROJECT_CONTEXT.md

Create a file that captures key project patterns:

```markdown
# Project Context: Authentication System

## Architecture Overview
- Monolithic FastAPI application
- PostgreSQL database (asyncpg)
- Redis for caching and token management
- SendGrid for emails

## Authentication Approach
- JWT-based (access + refresh tokens)
- Access token: 15 minutes
- Refresh token: 7 days
- Redis token blacklist for revoked tokens

## Code Patterns

### Service Layer Pattern
```python
class ServiceName:
    def __init__(self, db: Database, redis: Redis):
        self.db = db
        self.redis = redis
    
    async def method(self, param: Type) -> ReturnType:
        """Google-style docstrings."""
        # Implementation
```

### Error Handling
- Custom exceptions: AuthenticationError, ValidationError
- Raised with specific error codes
- API layer converts to HTTPException

### Testing Pattern
- Pytest with async fixtures
- Mock external dependencies
- AAA pattern (Arrange-Act-Assert)

[... more patterns ...]
```

**Usage:**

```bash
# For any new feature
claude-code prompt "Read PROJECT_CONTEXT.md to understand project patterns.

Then create a specification for [new feature] following these patterns."
```

**Benefits:**
- ✓ Reusable context across all features
- ✓ Consistent patterns throughout project
- ✓ Faster onboarding for new developers
- ✓ Single source of truth for conventions

---

## Validation Checkpoint

### ✅ Signs You Have Sufficient Context

Before writing a specification, check:
- [ ] I understand how this feature integrates with existing code
- [ ] I know what patterns and conventions to follow
- [ ] I understand any constraints or requirements
- [ ] I can explain the feature to someone else clearly
- [ ] AI can answer specific questions about the system

If **all** are true → Ready to write specification!

### ❌ Signs You Need More Context

Red flags that you should load more context:
- ❌ Specification is vague ("Add auth" with no details)
- ❌ Not sure how feature integrates
- ❌ Don't know what patterns to follow
- ❌ AI's responses are generic or off-base
- ❌ You're making assumptions about the system

If **any** are true → Load more context before proceeding!

---

## The Thinking Shift

**Old way of thinking (code-first):**
```
"I need to add a feature"
    ↓
"I'll just start coding"
    ↓
[Problems emerge during implementation]
```

**New way of thinking (context-first):**
```
"I need to add a feature"
    ↓
"First, what context do I need?"
    ↓
"Now I understand - let's specify it clearly"
    ↓
"Now implementation is straightforward"
```

**This shift is the essence of AI-native development.**

---

## Try With AI

**Tool:** Claude Code

Practice the context-first mindset.

### Prompt 1: Analyze Context Needs

```bash
claude-code prompt "I need to add a 'like' feature to a social media application. Users should be able to like posts and see how many likes a post has.

Before I write a specification, what context do I need to understand?

Give me:
1. List of integration points I need to understand
2. Patterns I should research
3. Constraints I should consider
4. Dependencies I should check

Format as a checklist I can use to gather context."
```

**Expected Outcome:**
- List of questions about existing post system
- Database structure questions
- Questions about user authentication
- API design patterns to research
- Real-time update considerations

**Reflection:** Did you think of all these context needs before? This is why context-first thinking matters.

---

### Prompt 2: Context Before Specification

```bash
claude-code prompt "Here's a specification someone wrote:

'Add user profiles. Users should have profiles.'

This specification has poor context. What's missing? What context should have been loaded before writing this specification?

List:
1. What questions should have been asked
2. What code should have been reviewed
3. What patterns should have been researched
4. How this specification should be improved"
```

**Expected Outcome:**
- List of missing details (what fields? how to access? privacy settings?)
- Questions about existing user system
- Integration points that weren't considered
- Improved specification outline

**Practice:** This shows you how to think critically about specification quality.

---

### Prompt 3: Validate Specification Readiness

```bash
claude-code prompt "I'm about to write a specification for adding payment processing to an e-commerce site. I've loaded this context:

- How products are currently stored in database
- How shopping cart works
- How user accounts are structured

Am I ready to write the specification? What else do I need to understand? Create a readiness checklist."
```

**Expected Outcome:**
- Missing context identified (payment gateway choice, security requirements, refund policy, etc.)
- Checklist of additional context to load
- Warning that current context is incomplete

**Key insight:** Even when you think you have enough context, there's often more to consider!


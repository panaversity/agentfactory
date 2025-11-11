---
title: "Vague Code and the AI Partner Problem"
chapter: 30
lesson: 1
duration: "1-1.5 hours"
skills:
  - name: "Problem Identification"
    proficiency: "A2"
    category: "Conceptual"
  - name: "AI Communication Clarity"
    proficiency: "B1"
    category: "Soft"
  - name: "Specification Understanding"
    proficiency: "A1"
    category: "Conceptual"
learning_objectives:
  - "Identify vagueness in requirements and its impact on AI-generated code (A2)"
  - "Recognize the gap between conversational intent and machine-executable instructions (B1)"
  - "Understand the core problem that Specification-Driven Development solves (A2)"
---

# Vague Code and the AI Partner Problem

As coding agents have grown more powerful, a pattern has emerged: you describe your goal, get a block of code back, and often it looks right, but doesn't quite work. This "vibe-coding" approach can be great for quick prototypes, but less reliable when building serious, mission-critical applications or working with existing codebases.

## Vibe Coding: Again what is the problem!

You're about to discover why your AI coding companion sometimes builds something that looks right but doesn't quite work.

Here's the pattern you've probably experienced:

You describe a feature to your AI companion. "Build me a login system," you say. Your companion generates code. You run it. It works. Users can create accounts and log in.

But then you try to reset a password—nothing. You try to recover a forgotten username—nothing. You ask your companion, "Where's the password reset?"

Your companion responds calmly: "You didn't ask for it."

You're frustrated. You assumed a login system *obviously* includes password reset. You didn't spell it out because it seemed implied. But your companion has no ability to infer what you meant. It can only work from what you explicitly stated.

This frustration—this gap between "what I described" and "what I wanted"—is the root of every failed AI coding session.

**This lesson explains why this happens and how Specification-Driven Development solved this problem.**

---

## The Problem with Vibe Coding

The emergence of powerful AI coding agents has highlighted a critical problem in how we communicate intent to machines.

**Vibe coding** is developing software by intuition or conversational suggestion. You describe your goal loosely, get code back, and hope it matches what you envisioned. It's called "vibe" coding because you're coding by feel, not by precision.

Here's the pattern:

1. **You describe a feature** (loosely, conversationally)
   - "Build me a login system"
   - "Add a search feature to my blog"
   - "Create a payment processor"

2. **Your AI companion generates code** (following the pattern it recognizes)
   - The code is syntactically correct
   - The code implements what you said literally
   - The code *looks* complete
   - Made 50+ implicit decisions without your input.

3. **You run it and discover** the code doesn't do what you *meant*
   - Missing error handling
   - No password reset
   - No input validation
   - No rate limiting on login attempts
   - No encryption for sensitive data

4. **You iterate** (in frustration)
   - "Add password reset"
   - "Handle invalid input"
   - "Add rate limiting"
   - Each iteration: one more fix, one more conversation cycle

Vibe coding can be great for rapid prototypes—throw something together quickly, see if the concept works. But it's **fragile** when building production systems, integrating with existing code, or handling sensitive data.

### Why This Happens

The problem isn't that AI coding agents are poor programmers. They're actually remarkably good.

The problem is how we're using them.

We treat AI agents like search engines:
- Search engines: "Find me pictures of cats" → Get pictures of cats (good enough)
- Coding: "Build me a login system" → Get login code (but missing 30% of real requirements)

But AI coding agents aren't search engines. **They're more like literal-minded pair programmers.**

Pair programmers need clarity. They thrive on:
- **Explicit requirements** (not implied assumptions)
- **Structured context** (not loose descriptions)
- **Clear constraints** (not open-ended possibilities)

Without these, even brilliant pair programmers can only infer intent from patterns they've seen before. And when your system is unique—when it has special requirements, edge cases, or domain-specific rules—patterns from general code examples won't suffice.

---

## Let's Experience It: Build a Login System (The Wrong Way)

Let me show you what happens with vague specifications.

### Step 1: Give Your Companion a Vague Prompt

Open your AI companion (Claude Code, ChatGPT, Gemini CLI, whatever you use). Paste this prompt:

```
Build me a login system. Users need to be able to create accounts
and log in with a username and password. Make it work with Python.
```

That's it. That's what most people naturally say when they want software built. Natural language, conversational, loose.

### Step 2: See What It Generates

Your companion will generate something like this:
```
create_account("alice", "password123")
print(login("alice", "password123"))  # True
print(login("alice", "wrongpassword"))  # False
```

### Step 3: Read It and Ask Questions

The code works. Users can create accounts and log in. Your vague prompt has been satisfied.

But now start asking:

**"Where's the password reset?"**

Your companion: "updated with a password reset functionality."

**"Where's account recovery if someone forgets their username?"**

Your companion: "You didn't mention that."

**"Where's email verification?"**

Your companion: "I will add now."

**"What if someone tries to log in 100 times with wrong passwords? Is there rate limiting?"**

Your companion: "I will add now."

**"Are passwords salted? This SHA-256 without salt is vulnerable to rainbow table attacks."**

Your companion: "Rainbow tables weren't mentioned in your request."

---

## The Frustration Moment

Here's where most developers get frustrated. They think:

> "I shouldn't have to spell out every detail. These things are obvious for a login system!"

But your companion's perspective is different:

> "I can only implement what you told me to implement. I don't know what's 'obvious' for your specific use case. Maybe you're building a toy demo where security doesn't matter. Maybe you're building a banking system where it's critical. I can't assume."

**Your companion is right.**

The problem isn't with the AI. The problem is with the communication. You provided 30% of the information needed to build a *good* login system. You provided 100% of the information needed to build a *minimal* login system.

---

## The Cost of Vagueness

Let's calculate what vague specification costs:

**Initial prompt** (your vague description):
- Time: 5 minutes
- Information density: 30%
- Code generated: ✅ Works, but incomplete

**First iteration** (you discover missing password reset):
- Time: 30 minutes (recognize problem, ask for fix, test it)
- Added code: 20 lines
- Information density: Now 40%

**Second iteration** (you discover missing rate limiting):
- Time: 30 minutes
- Added code: 15 lines
- Information density: Now 50%

**Third iteration** (you discover security vulnerability):
- Time: 2 hours (redesign, test, verify fix)
- Changed code: 10 lines modified
- Information density: Now 65%

**N iterations later...**

**Total time: 10-20 hours for a feature that could have been built right in 4-6 hours if the specification was clear upfront.**

---

## The Solution: Collaborate on Clear Specifications

Now imagine a different approach—one where you and your AI companion work together to build a clear specification BEFORE generating any code.

### Step 1: Draft Specification With Your Companion

Instead of writing specs alone, you collaborate with your AI companion. You start with intent and let AI help you think through the details.

Tell your companion:

```
Help me write a specification for a login system. I want users to create accounts
and authenticate securely, with account recovery support.

What should I specify to make this complete? What questions should I answer upfront
to avoid implementation surprises?
```

Your companion will help you identify:
- Missing requirements (What about email verification? Rate limiting? Session management?)
- Security considerations (Password hashing? Token expiration?)
- Edge cases (What if user tries to reset password for non-existent email?)

Through this dialogue, you arrive at a complete specification like:

```
## Login System Specification

### Intent
Allow users to create accounts and securely authenticate. Support account recovery.

### Inputs
- create_account(email, password, full_name)
- login(email, password)
- request_password_reset(email) -> reset_token
- reset_password(reset_token, new_password)

### Outputs
- create_account: success (boolean) + user_id
- login: success (boolean) + session_token (valid for 24 hours)
- password_reset: success (boolean)

### Functional Requirements
- Passwords must be at least 12 characters
- Accounts use email (unique per account) not username
- Email verification required before login enabled
- Password reset link valid for 1 hour
- Rate limiting: max 5 login attempts per IP per minute
- Failed login attempt logged

### Non-Functional Requirements
- Authentication must use bcrypt (cost factor 12)
- All passwords encrypted at rest
- Rate limiting for security, not just cosmetic
- Password reset must email user (not SMS)

### Test Scenarios
Given: email="alice@example.com", password="SecurePassword123!"
When: create_account(email, password, "Alice")
Then: success=True, user created

Given: valid login attempt
When: 6 failed attempts in 60 seconds
Then: Further login attempts blocked for 5 minutes
```

### Step 2: Generate Code From the Collaborative Spec

Now that you and your companion have built a clear specification together, ask for implementation:

```
Based on the login system specification we just created, generate the Python code
with all the requirements we discussed.
```

**Watch what happens:**

Your companion:
- ✅ Understands email requirement (not username)
- ✅ Implements bcrypt hashing (not just SHA-256)
- ✅ Adds rate limiting logic
- ✅ Adds email verification workflow
- ✅ Implements reset tokens with expiration
- ✅ Generates code that's production-ready, not toy code

### Step 3: Compare

The first code (from vague prompt): needs 10+ iterations
The second code (from clear spec): works correctly on first try

---

## The Aha Moment

Here's what you're realizing:

- The old way: Write specs alone (hard, tedious, easy to miss things) → Give to AI → Get code
- The new way: Collaborate with AI to write specs (AI asks questions you didn't think of) → Spec is complete → Generate code that works first time

**Key insight**: AI helps you write BETTER specifications by:
- Asking clarifying questions you didn't consider
- Identifying edge cases and security issues
- Suggesting standard patterns and best practices
- Catching ambiguities before they become bugs

This isn't a flaw in AI coding agents. This is how communication works:

- Clear instructions → correct understanding
- Vague instructions → guessing + iteration

**And specifications become clear through collaborative dialogue, not solo effort.**

---

## Why This Matters

You've experienced vague spec → mediocre code → iteration cycles.

The insight is:

> **Specification quality determines implementation quality.**

This is true whether your implementation partner is an AI agent or a human colleague. The better your specification, the better the implementation.

And when you're working with AI agents—which can't read minds, can't infer context, can't guess at unstated requirements—**specification becomes even more critical.**

This is the foundation of professional software development. And in the age of AI agents, it's no longer optional.

**It's the way work gets done.**

---

<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />


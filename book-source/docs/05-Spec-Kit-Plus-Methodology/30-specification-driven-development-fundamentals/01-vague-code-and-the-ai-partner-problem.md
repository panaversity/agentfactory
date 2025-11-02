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

**This lesson explains why this happens and introduces the solution: Specification-Driven Development.**

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

### What Actually Happens

When you say to your companion:

> "Build me a login system"

Your companion:
1. Recognizes the pattern: "user authentication"
2. Searches its knowledge: "what are common login implementations?"
3. Generates code that satisfies the literal request: "create accounts, log in with username/password"
4. Returns the result

Your companion has:
- ✅ Generated syntactically correct code
- ✅ Implemented the core pattern
- ❌ Made 50+ implicit decisions without your input (password hashing algorithm, session length, whether email is required, rate limiting, password reset flow, account recovery, etc.)

Each of those decisions is a choice that *you* should have made, but you left implicit.

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

## The Solution: Be Explicit

Now imagine instead:

### Step 1: Write a Detailed Specification

Before asking your companion to code anything, you write:

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

### Step 2: Share with Your Companion

Now paste this specification to your companion and ask for the code.

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

**Clarity prevents miscommunication.**

When you're explicit about requirements, your AI partner understands what to build. When you're vague, it guesses. And when it guesses, it misses the 70% of requirements you thought were "obvious."

This isn't a flaw in AI coding agents. This is how communication works:

- Clear instructions → correct understanding
- Vague instructions → guessing + iteration

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
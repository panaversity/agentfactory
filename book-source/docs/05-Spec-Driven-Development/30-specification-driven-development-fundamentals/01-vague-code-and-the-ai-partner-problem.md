---
title: "Vague Code and the AI Partner Problem"
chapter: 30
lesson: 1
duration: "60-90 minutes"
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

#### 💬 AI Colearning Prompt
> "Why can't AI coding agents infer what I mean when I say 'build a login system'? What's different about how AI understands requirements compared to human developers?"

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

#### 🎓 Expert Insight
> In AI-native development, you don't iterate your way to clarity—you collaborate upfront to achieve clarity. Vague specs aren't "good enough to start"; they're expensive technical debt disguised as rapid prototyping. The time you save skipping specification work gets multiplied 5-10x in iteration cycles.

---

## The Solution: Collaborate on Clear Specifications

Now imagine a different approach—one where you and your AI companion work together to build a clear specification BEFORE generating any code.

This isn't just "AI answers your questions." This is **bidirectional learning**—both you and AI improve through collaboration.

### Step 1: AI Teaches You (Missing Requirements)

Instead of writing specs alone, you start with intent and let AI help you think through the details.

Tell your companion:

```
Help me write a specification for a login system. I want users to create accounts
and authenticate securely, with account recovery support.

What should I specify to make this complete? What questions should I answer upfront
to avoid implementation surprises?
```

**Your companion responds** (AI as Teacher):

> "To make this specification complete, you should answer:
>
> 1. **Email verification**: Should users verify email before login is enabled?
> 2. **Rate limiting**: How many failed login attempts before blocking?
> 3. **Session management**: How long should sessions last? Refresh tokens?
> 4. **Password hashing**: Which algorithm? (bcrypt, argon2, scrypt?)
> 5. **Password reset**: Email or SMS? Token expiration time?
> 6. **Edge cases**: What if user tries to reset password for non-existent email?"

**What just happened?** AI taught you 6 specification questions you didn't think to ask.

### Step 1.5: You Teach AI (Your Constraints)

Now you refine AI's generic suggestions with YOUR specific constraints.

You respond:

```
Good questions. Here are my constraints:

1. Email verification: YES, required before login
2. Rate limiting: 5 attempts per IP per minute (we're a healthcare app, security is critical)
3. Session management: 24-hour sessions, no refresh (users log in daily anyway)
4. Password hashing: bcrypt cost 12 (our security policy mandates this)
5. Password reset: EMAIL only (no SMS, we don't collect phone numbers)
6. Password reset for non-existent email: Return "success" anyway (don't leak which emails exist)
```

**Your companion responds** (AI as Student):

> "Got it. A few clarifications based on your healthcare context:
>
> - Should failed login attempts be logged for audit? (HIPAA compliance)
> - Do passwords need minimum complexity rules? (12 chars minimum, special chars?)
> - Should we implement 2FA given the security requirements?
> - What about session timeout after inactivity?"

**What just happened?** AI learned your domain (healthcare) and adapted its suggestions. You taught AI constraints it didn't know.

### Step 2: Converge Together (Iterative Refinement)

Now watch the convergence loop—both you and AI refine the spec through dialogue.

**You respond to AI's HIPAA question:**

```
Yes, log all failed attempts with timestamp, IP, and attempted email (for audit trail).
Password complexity: minimum 12 chars, at least one uppercase, one number, one special char.
2FA: Not in v1 (out of scope), but design API to support it later.
Session timeout: 15 minutes of inactivity.
```

**Your companion responds** (AI as Co-Worker):

> "Perfect. I've incorporated those constraints. One edge case I noticed: what happens if someone requests password reset multiple times before the first token expires? Should we:
>
> A. Invalidate previous tokens (only latest works)
> B. Allow multiple valid tokens simultaneously
> C. Rate limit reset requests (e.g., max 3 per hour)"

**You respond:**

```
Option A—invalidate previous tokens. We don't want multiple valid reset links floating around.
Also add rate limiting: max 3 reset requests per email per hour.
```

**Your companion:** will generate the final spec.

**What just happened?** Through 3 rounds of dialogue:
1. **AI taught you** (suggested questions you didn't think of)
2. **You taught AI** (provided domain-specific constraints)
3. **You both converged** (refined edge cases together)

This is **co-learning**—neither you nor AI could have written this spec alone. You brought domain knowledge (healthcare context, security requirements). AI brought systematic thinking (edge cases, best practices). Together, you created a production-ready specification.

#### 💬 AI Colearning Prompt
> "Explain why the specification we just built collaboratively is better than what either a human or AI could write alone. What did each partner contribute that the other couldn't?"

---

## Generate Code From the Collaborative Spec

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

### Compare the Results

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

#### 🤝 Practice Exercise

> **Ask your AI**: "Help me write a specification for a user registration system with email verification. What questions should I answer upfront to avoid implementation surprises? What edge cases should I consider?"

**Expected Outcome**: Your AI will ask clarifying questions (password requirements? rate limiting? email service? token expiration?) that reveal gaps in your initial thinking. This dialogue produces a complete specification collaboratively.

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

## Try With AI

Ready to experience the difference between vague and clear specifications? Here are four prompts to explore:

**🔍 Explore the Problem:**
> "Show me two versions of a 'build a search feature' prompt: one vague (like I'd naturally write) and one clear (with all details specified). Then explain what gaps the vague version has."

**🎯 Practice Collaborative Specification:**
> "I want to build a password reset feature. Ask me clarifying questions to help write a complete specification. Don't let me skip important details like security, rate limiting, or error handling."

**🧪 Test Your Understanding:**
> "Generate code from this vague prompt: 'Build a file upload system.' Then show me what's missing or assumed. What should the spec have included?"

**🚀 Apply to Your Work:**
> "I'm building [describe your actual project feature]. Help me identify what's vague in my description and what details I need to specify before asking you to generate code."

---
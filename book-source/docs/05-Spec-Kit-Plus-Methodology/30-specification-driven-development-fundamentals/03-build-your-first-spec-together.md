---
title: "Build Your First Spec Together With Your Companion"
chapter: 30
lesson: 3
duration: "2.5-3 hours"
skills:
  - name: "Specification Writing"
    proficiency: "B1"
    category: "Technical"
  - name: "Requirements Clarity"
    proficiency: "B1"
    category: "Soft"
  - name: "Acceptance Criteria Definition"
    proficiency: "A2"
    category: "Technical"
learning_objectives:
  - "Write a complete specification with Intent, Inputs/Outputs, and Functional Requirements (B1)"
  - "Define acceptance criteria that an AI agent can execute (B1)"
  - "Collaborate with an AI companion to iteratively refine a specification (B1)"
---

# Build Your First Spec Together With Your Companion

You now understand what SDD is. But understanding is not the same as doing.

**This lesson**: You and your companion will build an actual specification together. You'll discover what makes specs work by constructing one.

---

## The Project: Password Reset System

You're going to spec out a **password reset system** for a real application.

This is complex enough to show structure, but simple enough to complete in one lesson.

### Step 1: Start Vague (Like Real Work)

Tell your companion:

```
I need to add a password reset feature to my app. Users should be able
to reset their password if they forget it. Can you help me write a
complete specification for this?

Let's start by clarifying what needs to be included in a good spec.
```

Your companion will likely respond with something like:

> "A good spec includes:
> - Intent (what problem are we solving?)
> - Inputs and Outputs (what data flows in and out?)
> - Functional Requirements (what must it do?)
> - Non-Functional Requirements (how fast? how secure?)
> - Test Scenarios (concrete examples)
> - Principles (design rules)"

### Step 2: Build Each Component Together

Now iterate with your companion. For each component:

#### **Intent**

Ask:

```
Help me write the Intent section. What problem does a password reset
system solve? What are the risks we need to prevent?
```

Your companion might respond:

> "Intent: Allow users who forgot their password to regain access to
> their account securely. Prevent account takeover through password
> reset exploitation (e.g., attackers shouldn't be able to reset
> someone else's password)."

Say: "Good. Let me add one more thing: support email-based reset and
security questions as backup."

Iterate until you're happy.

#### **Inputs and Outputs**

Ask:

```
What are the API functions this system needs? What data goes in?
What comes back out?
```

Your companion might suggest:

```
- POST /password-reset/request (email) → { success: bool, message: string }
- POST /password-reset/confirm (token, new_password) → { success: bool }
- GET /password-reset/validate (token) → { valid: bool, expires_at: timestamp }
```

Refine it:

```
You: "Does the request endpoint need to return the reset token? That seems
like a security risk."

Companion: "Good catch. Reset tokens should only be sent via email, never
returned in API response. Let me revise..."
```

#### **Functional Requirements**

Ask:

```
What are the rules this system MUST follow? Think about timing, limits,
security requirements.
```

Your companion might list:

```
1. Reset links valid for 60 minutes
2. Maximum 5 reset requests per email per hour
3. New password minimum 12 characters
4. Expired links return an error, not silently fail
5. All reset attempts logged
```

Challenge one:

```
You: "Why 60 minutes? That seems arbitrary."

Companion: "Longer is more convenient (user might be sleeping), shorter
is more secure (reduces window for attack). Industry standard is 15-60
minutes. Your choice."

You: "Let's go with 30 minutes to balance both."
```

#### **Non-Functional Requirements**

Ask:

```
How secure does this need to be? How fast? What if services fail?
```

Your companion might respond:

```
- Security: Passwords hashed with bcrypt (cost 12), never logged
- Performance: Reset request <500ms, validation <100ms
- Reliability: System works even if email service is temporarily down
- Scalability: Support 10,000 concurrent users
```

Push back:

```
You: "What if the email service is down for an hour? Should users be
blocked completely?"

Companion: "Good question. Options: (1) Block them, (2) Queue the email
for retry, (3) Let them set password without email confirmation. Which
makes sense for your app?"

You: "Let's queue for retry with exponential backoff."
```

#### **Test Scenarios**

Ask:

```
Give me 5 concrete test scenarios. Specific inputs, expected outputs.
```

Your companion might provide:

```
Scenario 1: Happy path
- User requests reset for email@example.com
- Email sent within 30 seconds
- User clicks link
- User enters new password
- Password changed, email confirmed

Scenario 2: Expired link
- User requests reset
- Waits 31+ minutes
- User tries reset link
- Error returned: "Link expired"

Scenario 3: Rate limiting
- User requests reset 5 times in 60 minutes
- 6th request returns error: "Too many attempts"
- After 60 minutes, reset available again

Scenario 4: Invalid password
- User has valid reset token
- User enters password: "weak"
- Error returned: "Password must be 12+ characters"
- Password not changed
```

Refine:

```
You: "What happens if someone uses the same reset link twice? Should the
second attempt work or fail?"

Companion: "Good question. Security best practice: tokens should be
single-use. Once a reset is completed, that token becomes invalid."
```

#### **Contextual Principles**

Ask:

```
What are the design philosophies that should guide implementation?
```

Your companion might suggest:

```
1. Security First: When security conflicts with convenience, choose security
2. Immutable Audit Trail: All password reset attempts logged
3. Fail Securely: If something goes wrong, deny access (don't silently bypass)
```

Add your own:

```
You: "Add one more: Never let a user discover if an email is registered.
Return same response whether email exists or not (prevents user enumeration)."
```

---

## Step 3: Review Your Complete Spec

Now look at what you've built together:

```
# Password Reset System Specification

## Intent
Allow users who forgot their password to regain access securely.
Prevent account takeover through password reset exploitation.

## Inputs and Outputs
- POST /password-reset/request
  Input: { email: string }
  Output: { success: bool, message: "Check your email" }

- POST /password-reset/confirm
  Input: { token: string, new_password: string }
  Output: { success: bool, message: string }

## Functional Requirements
1. Reset links valid for 30 minutes
2. Maximum 5 reset requests per email per hour
3. New password minimum 12 characters
4. Tokens single-use (can't be reused after reset)
5. All reset attempts logged (email, timestamp, success/fail)
6. Email never reveals if account exists

## Non-Functional Requirements
- Password hashing: bcrypt (cost 12+)
- API response: <500ms
- Scalability: 10,000 concurrent users
- Reliability: Queued email retry (exponential backoff)

## Test Scenarios
[5 concrete scenarios from above]

## Principles
1. Security > Convenience
2. Immutable audit trails
3. Fail securely, don't bypass
4. Never enumerate users
```

### Step 4: Test Your Spec (With Your Companion)

Now ask your companion:

```
Based on this spec, can you generate working code for the password
reset system?
```

Watch what happens. Your companion will either:
- ✅ Generate correct code (spec was complete)
- ❌ Ask clarifying questions (spec had gaps)
- ⚠️ Generate code with issues (spec was ambiguous)

If there are gaps, **go back and refine the spec.** Don't ask your companion to guess.

This is the fundamental insight: **Clear specs prevent iteration. Vague specs require iteration.**

---

## What You Just Learned (By Doing)

### ✅ Spec Structure
You experienced all six components of a good spec. You understand WHY each matters.

### ✅ Iteration Happens at Spec Level
When you refined "60 minutes" to "30 minutes," or "block users" to "queue emails," you were iterating on the SPEC, not on code.

This is powerful: **Change the spec once, regenerate all code.** Much faster than rewriting code.

### ✅ Specs Prevent Guessing
Without your password-reset-twice question, your companion would have guessed: "probably single-use." With your spec-driven mindset, you ASKED the question upfront.

### ✅ Security is Explicit
You didn't assume "password reset is obviously secure." You specified every security requirement explicitly. This prevents dangerous assumptions.

### ✅ Collaboration Works
You and your companion built this together. Companion suggested structure; you asked questions; companion refined; you pushed back. This is how professional specs are written.

---

## Your Reflection

**Questions for your journal:**

1. **How was this different from your normal development?**
   - Traditional: "Build password reset." Companion generates code. You iterate 5 times.
   - Spec-driven: "Let me spec this out." Both think clearly. Generate code. Done.

2. **What surprised you?**
   - The number of decisions a spec forces you to make upfront?
   - How many edge cases emerged through spec discussion?
   - How much clearer the final requirements were?

3. **Where in your own work would this have helped?**
   - A feature that took too long because of miscommunication?
   - A project where you built the wrong thing?
   - Working with someone (human or AI) who misunderstood what you wanted?

---

## Key Insight

**Specifications are not bureaucracy. They're clarity.**

They're the difference between:
- ❌ "Build password reset" (hope you guess right)
- ✅ "Here's exactly what password reset should do, including all edge cases" (now you understand)

---
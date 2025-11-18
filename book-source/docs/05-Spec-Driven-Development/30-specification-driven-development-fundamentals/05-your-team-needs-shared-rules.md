---
title: "Your Team Needs Shared Rules: Memory Banks and Constitutions"
chapter: 30
lesson: 5
duration: "45-60 minutes"
skills:
  - name: "Team Governance"
    proficiency: "B1"
    category: "Conceptual"
  - name: "Constitution Writing"
    proficiency: "A2"
    category: "Technical"
  - name: "Scaling Specifications"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Understand the problem of specification consistency across teams (B1)"
  - "Define a Constitution (Memory Bank) that applies globally to all specifications (B1)"
  - "Apply constitutional rules to ensure uniform implementation across team members (B1)"
---

# Your Team Needs Shared Rules: Memory Banks and Constitutions

## The Problem You Haven't Faced Yet

Imagine you're on a team of 5 developers. Each person writes their own password reset feature for different projects.

Developer A uses bcrypt (secure)
Developer B uses MD5 (insecure)
Developer C doesn't hash at all (catastrophic)

Each developer has a spec for their password reset system. Each spec says "use secure hashing." But they interpreted it differently.

**Result**: Your system is inconsistent and insecure.

---

## The Solution: Shared Rules That Apply to Everything

Instead of assuming everyone knows "password reset should be secure," you write rules that apply to EVERY feature.

These rules have names: **Memory Banks** (Kiro framework) or **Constitutions** (Spec-Kit framework).

**They're basically: the rules that govern your entire system.**

### How to Scale Specs Across Teams

**The Challenge**: You're on a team of 5 developers. Each person writes different features. How do you ensure everyone follows the same security, architecture, and quality standards without constant meetings?

**The Solution**: Create a **Memory Bank** (Kiro framework) or **Constitution** (Spec-Kit framework). This document lists rules that apply to **every feature**:

- ALL passwords use bcrypt
- ALL APIs are rate-limited
- ALL code has 80%+ test coverage
- ALL data is encrypted in transit

Every developer reads this before writing code. Every AI agent follows these rules. **Consistency becomes automatic.**

---

### Constitutions Enforce Quality at Scale

Beyond consistency, Constitutions ensure **every spec meets minimum quality standards**.

**Without Constitution**: Each developer decides what "good spec" means
- Some specs define edge cases, others don't
- Some specify error handling, others assume happy path
- Quality varies across team (inconsistent, unpredictable)

**With Constitution**: Global quality rules apply to ALL specs
- "All API specs MUST define error response format"
- "All authentication specs MUST specify token expiry and rotation policy"
- "All data validation specs MUST list edge cases and rejection responses"
- Quality becomes predictable and measurable

---

## What Goes in a Memory Bank / Constitution?

### 1. Product Vision

```
We're building a healthcare scheduling platform. Our core promise:
"Scheduling in under 30 seconds. No phone calls, no back-and-forth."
```

**Why**: Developers know what problem they're solving. Decisions align with vision.

### 2. Architecture Patterns

```
- All endpoints follow FastAPI patterns
- All services use repository pattern for data
- All databases accessed through SQLAlchemy ORM
- All errors follow standard error response format
```

**Why**: New developers don't reinvent wheels. Consistency.

### 3. Technology Stack

```
Backend: Python 3.13+, FastAPI
Database: PostgreSQL (primary), Redis (cache)
Testing: Pytest
Deployment: Docker + Kubernetes
```

**Why**: Developers know what tools they're using. No tool debates.

### 4. Security Rules (Non-Negotiable)

```
- ALL user data encrypted at rest (AES-256)
- ALL data in transit over TLS 1.3+
- ALL passwords hashed with bcrypt (cost 12+)
- NEVER log passwords, tokens, or sensitive data
- ALL endpoints require authentication (JWT)
```

**Why**: Security is default, not an afterthought. No vulnerable implementations.

### 5. Quality Standards

```
CODE QUALITY:
- Minimum test coverage: 80% per file
- All functions have docstrings
- All code formatted with Black (automatic)
- Type hints on all functions (mypy strict mode)

SPECIFICATION QUALITY (prevents quality gaps):
- All API specs MUST define error response format (not just happy path)
- All authentication specs MUST specify token expiry and rotation
- All data validation specs MUST document edge cases and rejections
- All feature specs MUST list non-functional requirements (performance, security)
```

**Why**: Quality is measurable at both spec AND code level. Bugs prevented at spec time, not caught in testing.

### 6. Common Patterns and Anti-Patterns

```
DO THIS: Use service + repository pattern
  service calls → repository calls → database

DON'T DO THIS: Database calls scattered through endpoints
  (makes code hard to maintain)
```

**Why**: Developers learn patterns by example.

---

## How It Works in Practice

### Scenario: Developer writes password reset feature

**Step 1: Read the Constitution**

Developer reads:

```
- Passwords MUST use bcrypt (cost 12+)
- NEVER log sensitive data
- ALL endpoints require rate limiting
- ALL code must have 80%+ tests
```

**Step 2: Write spec aligned with Constitution**

Developer writes password reset spec:

```
## Non-Functional Requirements
- Password hashing: bcrypt cost 12+ (per Constitution)
- Rate limiting: 5 attempts per hour (per Constitution)
- No logging of tokens (per Constitution)
- Test coverage: 80%+ (per Constitution)
```

**Step 3: Generate code**

Code is generated. It automatically:

- ✅ Uses bcrypt (no option to use MD5)
- ✅ Implements rate limiting (required by Constitution)
- ✅ Never logs tokens (Constitution rule enforced)
- ✅ Has tests for 80%+ coverage (Constitutional requirement)

**Step 4: Code review**

Reviewer checks: "Does code follow Constitution?"
Always yes, because Constitution was enforced in step 1-3.

---

### Encoding Lessons Learned: Bug → Rule

But Constitutions aren't written once and then forgotten. They evolve as your team learns.

**Pattern**: Production bug discovered → Root cause: vague specification → Add constitutional rule → Future specs prevent the same bug

**Example: The Forgotten Token Expiry**

```
INCIDENT: Leaked password reset token doesn't expire
  Users found: Reset tokens from weeks ago still work (critical security gap)

ROOT CAUSE: Original spec didn't specify token lifetime
  Spec said: "Create password reset token"
  Never said: "Token expires after X minutes"

CONSEQUENCE: Generated code has no expiry logic. Tests don't check expiry.

CONSTITUTIONAL FIX:
+ Add to Constitution, under Authentication:
  "All temporary access tokens (password reset, email verification, etc.)
   MUST specify time-based expiry. Default: 30 minutes unless otherwise justified."

RESULT: Future password reset specs automatically include token expiry.
         Generated code includes expiry logic.
         Team learns from mistake: quality rule prevents recurrence.
```

This is **organizational learning**. Your team encounters a gap once, encodes it as a rule, and prevents that gap from happening again in 50 other features across 10 teams.

---

## The Power at Scale

**Without Constitution:**

- 5 developers
- Each makes security decisions independently
- Some use bcrypt, some use MD5, some use nothing
- Security is chaotic
- Code review has to check everything

**With Constitution:**

- 5 developers
- Constitution says "bcrypt always"
- All developers implement bcrypt (no debate)
- Security is consistent
- Code review can focus on logic, not security basics

Now imagine 50 developers, or 500 developers. Constitution doesn't scale linearly. It scales exponentially: **More developers → more need for shared rules.**

#### 💬 AI Colearning Prompt
> "Why do teams with Constitutions (shared rules) scale better than teams without them? What breaks down in large teams when everyone makes independent security or architecture decisions?"

---

## The Key Insight

**Specifications are feature-specific. Constitutions are system-wide.**

- **Spec**: "How should password reset work?"
- **Constitution**: "How should ALL code handle security, quality, testing?"

Specs drive individual features. Constitutions ensure consistency across ALL features.

---


## Beyond Constitution: Capturing the Journey and Decisions

A Constitution defines **what rules we follow**. But as your team develops software, two other critical questions emerge:

1. **How did we learn what works?** (The journey of discovery)
2. **Why did we choose this approach?** (The rationale behind mutable decisions)

Two additional artifacts address these questions: **Prompt History Records (PHR)** and **Architectural Decision Records (ADR)**.

---

### Prompt History Records (PHR): Capturing the Journey

**The Problem**: When AI-generated code fails, how do you debug it? When a spec works perfectly, how do you learn why it succeeded? Without a record of your AI interactions, knowledge is lost.

**The Solution**: PHR = Structured log of all AI interactions during development.

#### Why PHRs Matter

**1. Debugging**: When generated code doesn't work as expected

```
Problem: Password reset emails not sending
↓
Check PHR: What did we ask AI to generate?
↓
PHR shows: "Generate password reset with email notification"
↓
Insight: We never specified SMTP configuration in the spec!
↓
Solution: Update spec with email configuration details
```

**2. Learning**: Patterns emerge showing which prompts produce better results

```
PHR Analysis after 3 months:
- Prompts with explicit error handling: 95% success rate
- Prompts without error handling: 60% success rate
→ Team learns: Always specify error handling in specs
```

**3. Collaboration**: Team members understand reasoning path, not just final code

```
New developer joins team:
"Why did we use JWT instead of sessions for password reset?"
↓
Check PHR from that feature:
↓
Shows full discussion: Security Subagent recommended JWT,
team discussed trade-offs, chose JWT for stateless scaling
↓
New developer understands context without meeting
```
---

### Architectural Decision Records (ADR): Documenting Mutable Decisions

**Constitution = Immutable**: "ALL passwords use bcrypt" (never changes)  
**ADR = Mutable**: "For this feature, we chose JWT over sessions because..." (might change)

#### Understanding the Difference

**Constitution**: System-wide principles that rarely change

- "All passwords use bcrypt cost 12+"
- "Test coverage must exceed 80%"
- "All data encrypted in transit"

**ADR**: Feature-specific decisions that might evolve

- "Use JWT for password reset tokens" (might switch to sessions later)
- "30-minute token expiry" (might adjust based on user feedback)
- "Email-only reset" (might add SMS backup in future)

#### When to Write an ADR

Write an ADR for:

- ✅ Significant architecture decisions (database choice, auth pattern, API design)
- ✅ Trade-offs between competing approaches (performance vs simplicity)
- ✅ Deviations from obvious/standard patterns (why we didn't use the common approach)
- ✅ Decisions that future developers will question ("Why did they choose this?")

Don't write an ADR for:

- ❌ Decisions covered by Constitution (those are already documented)
- ❌ Trivial implementation details (variable naming, minor refactoring)
- ❌ Temporary workarounds (document in code comments instead)

## The Power at Scale Revisited

**Without SDD**:

- 5 developers make independent decisions
- Some use JWT, some use sessions, some use both
- No one knows why decisions were made
- New developers ask same questions repeatedly
- AI agents make inconsistent choices

**With SDD and an Opinionated Tools:**

- Constitution: "Auth pattern is JWT" (everyone follows)
- ADR: "Why JWT? Because..." (everyone understands)
- Spec: "This feature uses JWT per ADR-001" (consistent implementation)
- PHR: "AI generated this based on ADR-001" (traceable)
- New developers: Read ADRs, understand context, continue pattern

**Result**: Consistency emerges not just from rules (Constitution), but from shared understanding (ADRs) and institutional learning (PHRs).

#### 🎓 Expert Insight
> In AI-native development, your Constitution isn't just documentation—it's executable memory. Every AI agent reads it before generating code. Every human reads it before writing specs. The Constitution becomes your organization's shared brain, preventing the same security mistake from happening in 50 different features across 10 different teams.

---

## Your Reflection

**Questions:**

1. **What rules would YOUR Constitution include?**

   - Security rules for your domain?
   - Architecture patterns you want everyone to follow?
   - Quality standards?
   - Technology choices?

2. **How would a Constitution change your team's work?**

   - Less debate about "should we use bcrypt or MD5?" (Constitution says bcrypt)
   - Faster code review (Constitution compliance checked automatically)
   - More consistent codebase
   - Easier onboarding (new devs read Constitution, understand rules)

3. **Where do specs and Constitution meet?**
   - Specs implement the Constitution
   - Constitution enforces quality across all specs
   - No conflict: they work together

#### 🤝 Practice Exercise: Encode Quality Rules

> **Ask your AI**: "Help me draft a Constitution for a [describe your domain: e-commerce, fintech, healthcare, etc.] application. What security rules should apply to every feature? What architecture patterns should be mandatory? **What quality gaps has our team experienced repeatedly?** What specification quality rules would prevent those gaps?"

**Two-part exercise**:

1. **Domain-Specific Rules**: Ask AI to suggest security (PCI-DSS for payments, HIPAA for healthcare, GDPR for EU data) and architecture rules.

2. **Quality-at-Scale Rules**: Ask AI to help you identify and encode specification quality standards:
   - "What should every API spec specify?" (error handling, status codes, rate limits)
   - "What should every authentication spec require?" (token expiry, refresh rotation, session management)
   - "What should every data validation spec include?" (edge cases, rejection responses, format validation)

**Expected Outcome**: Constitution that prevents common quality gaps before they become production bugs. Your team learns from past mistakes by encoding them as rules.

---

**Professional teams don't debate fundamentals every project.**

They write down the rules (Constitution / Memory Bank). Everyone follows them. Consistency emerges.

This is how teams scale without chaos.

---

## What Happens Next: From Patterns to Reusable Intelligence

You now understand how Constitutions enforce quality across team specs. But you're noticing something:

**The same patterns repeat**:
- Every API spec defines endpoints, error handling, authentication
- Every authentication spec specifies tokens, expiry, rotation
- Every data validation spec documents edge cases, rejections
- Every feature spec includes the same quality checklists

**Question**: If the same patterns show up in 30 different specs, could those patterns become **reusable**?

**Lesson 6 introduces Reusable Intelligence (RI)**—where specification patterns become Skills and Subagents that your team and AI agents use to write better specs faster.

Constitution + Reusable Intelligence = Teams that scale without losing quality.

---

## Try With AI

Ready to apply constitutional thinking to your work? Explore these prompts:

**🔍 Explore Constitution Design:**
> "What's the difference between a rule that belongs in a Constitution (applies to everything) vs. a rule that belongs in an ADR (applies to one decision)? Give me 5 examples of each for a web application."

**🎯 Practice Drafting Rules:**
> "I'm building a social media platform. Help me draft constitutional rules for: (1) Data privacy, (2) Content moderation, (3) API rate limiting, (4) Testing standards. What should be mandatory across ALL features?"

**🧪 Test Constitutional Enforcement:**
> "Here's a feature spec for user authentication [paste or describe]. Does it comply with this Constitution rule: 'ALL passwords use bcrypt cost 12+'? If not, what's missing?"

**🚀 Apply to Your Organization:**
> "My team of [X people] is building [describe project]. We keep having debates about [describe recurring issue]. Should this be a Constitutional rule or handled case-by-case? Help me decide."

---
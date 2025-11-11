---
title: "Your Team Needs Shared Rules: Memory Banks and Constitutions"
chapter: 30
lesson: 4
duration: "2-2.5 hours"
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
- Minimum test coverage: 80% per file
- All functions have docstrings
- All code formatted with Black (automatic)
- Type hints on all functions (mypy strict mode)
```

**Why**: Quality is measurable. CI enforces it.

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

---

## Ask Your Companion: Build a Constitution for Your System

Tell your companion:

```
Help me draft a Constitution for a healthcare scheduling app. What rules
should apply to every feature? Think: security, architecture, quality,
compliance.
```

Your companion will help you draft.

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

---

**Professional teams don't debate fundamentals every project.**

They write down the rules (Constitution / Memory Bank). Everyone follows them. Consistency emerges.

This is how teams scale without chaos.

---

<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />


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

### Ask Your Companion: How Do I Scale Specs?

Tell your companion:

```
I'm on a team of 5 developers. Each person is writing different features.
How do I ensure everyone follows the same security, architecture, and
quality standards without constant meetings?

What if I created a document that says "these rules apply to ALL code"?
How would that work?
```

Your companion will explain the concept:

> "Create a Memory Bank (or Constitution). It lists rules that apply to
> every feature:
>
> - ALL passwords use bcrypt
> - ALL APIs are rate-limited
> - ALL code has 80%+ test coverage
> - ALL data is encrypted in transit
>
> Every developer reads this before writing code. Every AI agent follows
> these rules. Consistency is automatic."

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

Your companion will help you draft:

```
# Our Constitution

## Product Principles
- Security first: Privacy is non-negotiable
- Fast: Scheduling in <30 seconds
- Accessible: Works for non-tech-savvy patients

## Security (Non-Negotiable)
- ALL patient data is PHI (Protected Health Information)
- ALL PHI encrypted at rest and in transit
- NEVER log PHI or appointment details
- Passwords: bcrypt cost 12+
- Rate limiting: standard on all endpoints

## Architecture
- Service + Repository pattern always
- Circuit breakers on external APIs
- Events for cross-service communication

## Quality
- Test coverage: 80%+ minimum
- Type hints on all functions
- Docstrings on all public functions
- Code formatted with Black (automatic)

## Compliance
- HIPAA compliance required
- Audit logging for all patient access
- Data retention: 3 years minimum
```

Refine it with your companion:

```
You: "Should we require 80% coverage for tests, or is 70% enough?"

Companion: "That depends on risk tolerance. Healthcare is high-risk.
80%+ catches more bugs. 70% might miss critical paths."

You: "Let's go with 80%. Healthcare is not the place to cut corners."
```

---

## The Key Insight

**Specifications are feature-specific. Constitutions are system-wide.**

- **Spec**: "How should password reset work?"
- **Constitution**: "How should ALL code handle security, quality, testing?"

Specs drive individual features. Constitutions ensure consistency across ALL features.

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

## Key Takeaway

**Professional teams don't debate fundamentals every project.**

They write down the rules (Constitution / Memory Bank). Everyone follows them. Consistency emerges.

This is how teams scale without chaos.

---

## Next Steps

You now understand:
- ✅ What SDD is (Lesson 2)
- ✅ How to build a spec (Lesson 3)
- ✅ How to scale specs (Lesson 4: Constitutions)
- Still to learn:
  - Why SDD emerged now (Lesson 5: History and AI)
  - Which tools to use (Lesson 6: Kiro, Spec-Kit, Tessel)
  - Where it's going (Lesson 7: Spec-as-source, MDD lessons)
  - Your commitment (Lesson 8: Manifesto)

---

**Next Lesson**: Why did SDD become standard NOW? Explore the history and the AI moment.

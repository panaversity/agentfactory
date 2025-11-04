---
title: "How Agents Stay Current: Context Architecture for Living Specs"
chapter: 32
lesson: 6
duration: "3-3.5 hours"
skills:
  - name: "Context Architecture"
    proficiency: "B1"
    category: "Technical"
  - name: "Knowledge Management"
    proficiency: "B1"
    category: "Conceptual"
  - name: "API Integration Strategies"
    proficiency: "B1"
    category: "Technical"
learning_objectives:
  - "Understand how knowledge cutoffs impact AI agent specifications (B1)"
  - "Compare three context strategies: Context7, Tessel Registry, and hybrid approaches (B1)"
  - "Design context architecture decisions appropriate for specific domains (B1)"
---

# How Agents Stay Current — Context Architecture for Living Specs

## The Problem No One Talks About

You've learned to write clear specifications. Your AI agents (Claude Code, Gemini CLI) generate code from those specs. Everything works.

**Until three weeks later.**

```
September 1: You spec "Integrate Stripe API"
    ↓
Claude Code generates code using its knowledge (trained Sept 2024)
    ↓
Code works perfectly
    ↓
September 28: Stripe releases API v2025 with new endpoints
    ↓
Your spec is still valid, but Claude's knowledge is stale
    ↓
Code references deprecated endpoints
    ↓
Integration breaks in production
    ↓
Rework needed
```

**The Core Problem**: Agents have knowledge cutoffs. APIs don't. This gap creates a fundamental mismatch.

**The Question**: How do you architect AI agents so they stay current with evolving APIs and libraries?

---

## Part 1: Why This Matters (And Why It's a Specification Problem)

### The Knowledge Cutoff Reality

Ask your companion:

```
Why do AI agents have knowledge cutoffs?
What's the difference between my training knowledge and current API reality?
Can specs help bridge this gap?
```

Your companion explains:

> "AI models are trained on data up to a specific date (knowledge cutoff).
> Claude's knowledge: September 2024
> ChatGPT's knowledge: April 2024
> These are frozen snapshots.
>
> Meanwhile, libraries and APIs evolve continuously:
> - Stripe releases new endpoints monthly
> - OpenAI changes their API every quarter
> - FastAPI adds features every release
> - AWS adds new services weekly
>
> Your spec says 'use the latest Stripe API.' But 'latest' and 'my training data'
> are DIFFERENT THINGS.
>
> Traditional solution: Hope the API hasn't changed since your training data.
> Better solution: Explicitly tell agents where to find current information."

### This IS a Specification Decision

Here's the key insight: **Choosing how agents access current knowledge is a SPECIFICATION decision.**

When you write a spec, you're not just specifying WHAT to build. You're also specifying:
- WHO will build it (team roles)
- HOW it will be built (architecture patterns)
- HOW it will be MAINTAINED (keeping it current)

The third part—keeping current—is about **context architecture**: Where will agents get current knowledge?

---

## Part 2: Three Strategies for Current Knowledge

### Strategy 1: Context7 (Live Documentation as an MCP Server)

**What is Context7?**

Context7 is an MCP (Model Context Protocol) server that provides current library documentation to AI agents.

**How it works**:

```
You: "Spec: Integrate Stripe API (latest)"
        ↓
Claude Code receives spec
        ↓
Spec includes: "Use Context7 MCP for current Stripe docs"
        ↓
Claude Code connects to Context7 MCP server
        ↓
Context7 fetches CURRENT Stripe API docs (real-time)
        ↓
Claude Code generates code against ACTUAL current API
        ↓
Code is correct on day 1, week 1, month 1
```

**Real example spec**:

```markdown
# Stripe Payment Integration Specification

## Intent
Enable one-time and recurring payments through Stripe API

## Context Source
Use Context7 MCP server for current Stripe API documentation
- Endpoint: context7.anthropic.com (or your instance)
- Library: stripe-python
- Auto-refresh: Daily

## Functional Requirements
1. Accept payment via Stripe API (consult Context7 for current endpoints)
2. Handle webhook callbacks (use current Stripe webhook format from Context7)
3. Validate payment status (use current status enum from Context7)
4. Log transaction (PII must match current Stripe privacy spec)

## Implementation Note
Agent should:
1. Fetch current Stripe API spec from Context7
2. Confirm endpoints exist and are not deprecated
3. Use current parameter names and formats
4. Handle current error codes and status values
```

**When to use Context7**:
- ✅ Production integrations with external APIs
- ✅ APIs that evolve frequently (Stripe, OpenAI, AWS)
- ✅ You want guarantee of current knowledge
- ✅ Code must work immediately after generation
- ❌ Offline agents (no network access)
- ❌ Highly proprietary/internal APIs

**Hands-on Exercise**:

Ask your companion:

```
Help me write a spec for Stripe integration that uses Context7.
Show me how the spec references the current API docs.
What does Claude Code do differently when it sees Context7 reference?
```

**Deliverable**: A complete Stripe integration spec that explicitly uses Context7

---

### Strategy 2: Tessel Registry (Spec-Driven Library Discovery)

**Remember Tessel?** (From Chapter 30, Lesson 7)

Tessel is the spec-as-source framework. When libraries publish Tessel-compatible specs, those specs become **machine-readable contracts**.

**How it works**:

```
Library publishes Tessel spec (e.g., FastAPI)
        ↓
Tessel Registry stores all FastAPI versions' specs
        ↓
You write spec: "Use FastAPI (latest from Tessel Registry)"
        ↓
Code generator queries Tessel Registry
        ↓
Retrieves FastAPI's spec for current version
        ↓
Generates code that matches spec exactly
        ↓
Code is guaranteed correct (spec is source of truth, not knowledge cutoff)
```

**Real example spec**:

```markdown
# FastAPI Web Service Specification

## Intent
Build REST API endpoints for order management

## Library Specifications
Use Tessel Registry for library specifications:
- FastAPI: tessel://python/fastapi/latest
- Pydantic: tessel://python/pydantic/latest
- SQLAlchemy: tessel://python/sqlalchemy/latest

## Functional Requirements
(Implementation will conform exactly to Tessel specs)

1. Create endpoint /orders (POST)
   - Validate using Tessel Pydantic schema format
   - Respond with Tessel FastAPI response format

2. Query endpoint /orders/{id} (GET)
   - Return data matching Tessel schema
   - Use Tessel-defined error responses
```

**When to use Tessel Registry**:
- ✅ Library publishes Tessel specs (increasingly common)
- ✅ Safety-critical systems (medical, finance, aerospace)
- ✅ You need mathematical correctness guarantee
- ✅ Regulated domains with audit requirements
- ❌ Library doesn't publish Tessel specs yet
- ❌ Rapid iteration/experimental work
- ❌ Proprietary/internal libraries

**The Promise of Tessel**:
If all libraries publish Tessel specs, then agents can generate code that is CORRECT by definition, not correct-by-chance.

**Hands-on Exercise**:

Ask your companion:

```
What would a Tessel spec for FastAPI look like?
How is it different from regular documentation?
If FastAPI published Tessel specs, what could agents do differently?
```

**Reflection**: Is Tessel the future, or too ambitious?

---

### Strategy 3: Hybrid Approaches (When Context7 and Tessel Aren't Enough)

**Real world**: Not all libraries have Context7 access or Tessel specs. What then?

#### Option A: Embedded Current Documentation

Store the current API documentation directly in your system prompt or spec.

**Approach**:
```
Spec includes full API documentation as part of the spec itself.
Agent doesn't fetch anything—docs are embedded.
```

**Pros**:
- Simple, no external dependencies
- Works offline
- Guaranteed consistency (docs and spec match)

**Cons**:
- Limited by context window size
- Manual updates required
- Can't cover complex APIs

**When to use**:
- Small, stable APIs
- Internal/proprietary APIs
- Offline environments
- Simple integrations

#### Option B: RAG (Retrieval-Augmented Generation)

Your agent has access to a vector database of API documentation. When building, it retrieves relevant docs.

**Approach**:
```
1. Store API docs in vector database
2. Agent embeds query: "How do I authenticate in Stripe?"
3. Vector DB retrieves matching doc sections
4. Agent uses retrieved docs for code generation
```

**Pros**:
- Scales to large, complex APIs
- Relevant docs retrieved automatically
- You control the knowledge source

**Cons**:
- Requires infrastructure (vector DB)
- Retrieval quality depends on embedding model
- Adds latency to code generation

**When to use**:
- Complex APIs with hundreds of endpoints
- You want semantic search (not keyword match)
- You have infrastructure to maintain

#### Option C: Agent Memory and Continuous Learning

For long-running agents, maintain a memory of API changes they've encountered.

**Approach**:
```
Agent encounters: "This endpoint no longer exists"
    ↓
Agent logs: "Stripe endpoint /v1/charges deprecated, use /v1/payment_intents"
    ↓
Next time: Agent checks memory before generating code
    ↓
Result: Agent learns and adapts over time
```

**Pros**:
- Agents get smarter over time
- Discovers real-world issues
- Builds institutional knowledge

**Cons**:
- Requires agent architecture
- Memory can become stale or incorrect
- Not suitable for one-off specs

**When to use**:
- Long-running agents (chatbots, continuous services)
- You have time for learning period
- Want agent to discover issues in real time

#### Option D: Version Pinning

Specify exact library versions in your spec. Fetch docs for THAT version.

**Approach**:
```
Spec: "Use Stripe API v2024-04-10"
    ↓
Agent fetches docs for that specific version
    ↓
Code matches that version exactly
    ↓
When you're ready, update spec to v2024-06-15
    ↓
Regenerate code for new version
```

**Pros**:
- Explicit, reproducible
- Easy to control upgrade timeline
- Good for critical systems

**Cons**:
- Requires spec updates
- Manual upgrade process
- Can fall behind current versions

**When to use**:
- You want control over when to upgrade
- Regulated domains with change control
- Microservices with coordinated versioning

---

## Part 3: Decision Framework

### The Matrix: Choose Your Strategy

| Scenario | Context7 | Tessel | Embedded | RAG | Memory | Version Pin |
|----------|----------|--------|----------|-----|--------|-------------|
| Production SaaS integration | ✅ | ⚠️ | ❌ | ⚠️ | ❌ | ⚠️ |
| Safety-critical system | ⚠️ | ✅ | ⚠️ | ❌ | ❌ | ✅ |
| Startup (rapid changes) | ✅ | ❌ | ⚠️ | ✅ | ✅ | ❌ |
| Internal API only | ❌ | ❌ | ✅ | ✅ | ✅ | ✅ |
| Complex external API | ⚠️ | ⚠️ | ❌ | ✅ | ⚠️ | ⚠️ |
| Offline agents | ❌ | ❌ | ✅ | ❌ | ✅ | ✅ |

### Questions to Answer

1. **How often does the API change?**
   - Daily/Weekly → Context7 (need constant updates)
   - Monthly → Tessel or Version Pin (periodic review)
   - Rarely → Embedded (simple, manual)

2. **How critical is correctness?**
   - Mission-critical → Tessel (spec is truth) or Embedded (verified manually)
   - Important → Context7 (trusted source) or RAG (semantic search)
   - Nice-to-have → Version Pin or Memory (learning acceptable)

3. **Is the library Tessel-enabled?**
   - Yes → Consider Tessel (if safety matters)
   - No → Context7 (if frequent changes) or Embedded (if simple)

4. **Do you have offline requirements?**
   - Yes → Embedded, Memory, or Version Pin
   - No → Context7, Tessel, or RAG

5. **What's your infrastructure appetite?**
   - Minimal → Context7 (MCP) or Embedded
   - Moderate → RAG or Version Pin
   - Extensive → Full agent with Memory

---

## Part 4: Real-World Example

### The Scenario

You're building a SaaS app that processes payments. You need Stripe integration that:
- Works correctly from day 1
- Stays current as Stripe evolves
- Can be understood by a new developer in 6 months
- Must be compliant with PCI requirements

### The Decision

**Your analysis**:
- Stripe changes API quarterly ✅ → Need frequent updates
- Payment processing is critical ✅ → Need high confidence
- Stripe doesn't publish Tessel specs yet ❌ → Can't use Tessel
- You have cloud infrastructure ✅ → Can use MCP

**Decision**: **Use Context7 with fallback to Version Pin**

### The Specification

```markdown
# Stripe Payment Processing Specification

## Intent
Process one-time and recurring payments through Stripe API

## Context Architecture
### Primary: Context7
- Use Context7 MCP server for Stripe API documentation
- Reference: stripe-python, latest version
- Refresh frequency: Daily
- Endpoint: your-context7-instance.example.com

### Fallback: Version Pin
- If Context7 unavailable, use Stripe API version: v2025-01-01
- Update version pin quarterly after testing
- Document any breaking changes in changelog

## Implementation Pattern
Agent should:
1. Fetch current Stripe API spec from Context7
2. Verify that chosen endpoints exist (not deprecated)
3. Use current parameter names and formats
4. Handle current error codes
5. Test against live Stripe sandbox API before production

## Acceptance Criteria
1. ✅ Code generated includes Context7 reference comment
2. ✅ Code uses currently valid Stripe endpoints (verified against Context7)
3. ✅ Error handling matches current Stripe error responses
4. ✅ PCI compliance checklist completed
5. ✅ Works immediately in production without changes
```

### The Outcome

```
Week 1: Claude Code fetches current Stripe docs from Context7
        Generates payment code using ACTUAL current API
        ✅ Works first time

Week 8: Stripe releases new version
        Context7 automatically updates
        When you regenerate from same spec, gets new version
        ✅ Stays current automatically

Month 6: New developer reads spec
         Sees "uses Context7"
         Understands: This code is always current
         ✅ Clarity for future developers
```

---

## Part 5: Choosing Your Architecture

### Exercise: Design Your Context Architecture

Choose a real API you want to integrate. Then answer:

```
1. API: ________________
2. How often does it change? ________________
3. How critical is correctness? ________________
4. Is it Tessel-enabled? ________________
5. Do you have offline needs? ________________
6. What's your infrastructure appetite? ________________

Based on answers, my context strategy is: ________________

Rationale:
```

Work with your companion if you get stuck. They can help you reason through the tradeoffs.

### Your Commitment

Before moving to the capstone, write this down:

```
For my capstone project, I will use this context architecture:

[ ] Context7 (live docs via MCP)
[ ] Tessel Registry (spec-driven discovery)
[ ] Embedded documentation (in spec itself)
[ ] RAG system (vector search)
[ ] Agent memory (continuous learning)
[ ] Version pinning (explicit control)

My reasoning: [Why this choice makes sense for my project]

How I'll implement it: [Specific steps]
```

This decision affects your capstone code. Make it deliberately.

---

## Key Insight

**Specifications are not static documents.** They're LIVING artifacts that must stay current with the systems they describe.

This requires **architectural decisions** about how your agents access knowledge.

Choosing the right context architecture is as important as choosing the right technology stack.

---

## Next Steps

You now understand:
- ✅ Why agents have knowledge cutoff problems
- ✅ Why this is a specification decision
- ✅ Five strategies for keeping agents current (Context7, Tessel, Embedded, RAG, Memory, Version Pin)
- ✅ How to choose the right strategy for your context
- ✅ How to commit to a context architecture before coding

**Ready for the capstone?** You have the strategic foundation to design systems where agents stay current with specifications.

---

**Next**: Capstone Project Part 1 — Decompose Your Spec With Context Architecture In Mind

---
title: "SDD In the Wild: Real Companies"
chapter: 32
lesson: 5
duration: "2 hours"
skills:
  - name: "Case Study Analysis"
    proficiency: "B1"
    category: "Soft"
  - name: "Real-World SDD Patterns"
    proficiency: "B1"
    category: "Conceptual"
  - name: "Implementation Insights"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Analyze real-world examples of SDD adoption in industry (B1)"
  - "Extract patterns and lessons from successful SDD implementations (B1)"
  - "Understand both benefits and challenges in production SDD environments (B1)"
---

# SDD In the Wild: Real Companies

## What You're About to Learn

In Lessons 1-4, you learned specification-driven development theory and scaled it from teams to enterprises. Now you're going to study *real companies* and see how they actually use specifications.

You'll analyze how Amazon, Stripe, Google, and others enable thousands of engineers to work without chaos. You'll discover the patterns they use. And you'll extract lessons you can apply in your own work.

The lesson is not "copy what Amazon does." It's "understand *why* companies at scale use specifications, and apply that principle to your context."

---

## Part 1: Amazon Web Services (AWS) – Scale Through APIs

### The Problem Amazon Faced (2000s)

Amazon had hundreds of engineers. Each team built features. Features needed to work together. But without clear specs:
- Team A builds user service with user data format X
- Team B builds order service expecting user data format Y
- They don't match
- Integration fails
- Rework happens
- Shipping is delayed

### How Amazon Solved It

In the mid-2000s, Amazon leadership made a decision: **Every team publishes API specifications.** Not optional. Non-negotiable.

**The Rule**: If your team's code isn't callable through a published API spec, it doesn't go to production.

### What Amazon Specifications Look Like

**Specification for: User Service API**

```
Version: v2.1
Owner: Platform User Team
Status: Stable (changed 3 months ago)

Endpoints:
  `GET /users/{user_id}`
    Returns: User object (see schema below)
    Errors: 404 Not Found, 401 Unauthorized
    Rate Limit: 1000 req/min per auth key
    SLA: 99.99% availability, p99 latency <100ms

  POST /users
    Input: User creation object (see schema)
    Returns: User object (see schema)
    Errors: 400 Bad Request, 409 Conflict
    Rate Limit: 100 req/min per auth key

User Schema:
  user_id: string (UUID)
  email: string (valid email format)
  created_at: ISO 8601 timestamp
  updated_at: ISO 8601 timestamp
  ... (20 more fields)

Versioning:
  - Current: v2.1
  - Sunset: v1.0 (deprecated 2 years ago)
  - Roadmap: v3.0 (planned for next quarter)
```

### What This Enabled

- **5000+ engineers** can work independently
- **Thousands of services** depend on User Service
- **Zero coordination meetings** required (except quarterly architecture reviews)
- **Independent scaling**: User Service can scale without affecting other teams
- **Independent deployment**: User Service can deploy multiple times per day
- **Ecosystem**: Third-party companies build on AWS APIs because specs are so clear

### The Business Impact

Because AWS specs are clear and stable:
- Developers can learn AWS in days (clear docs from clear specs)
- Third-party companies build integrations (because specs are unambiguous)
- Customers trust AWS (specs define reliability guarantees)
- AWS became a $60+ billion business (in part, because specifications enable scale)

---

## Part 2: Stripe – Scale Through Payment Specs

### The Problem Stripe Solved

Stripe enables thousands of companies to accept payments. Every company integrates differently. Without clear specs, integration would be chaos.

### Stripe's Specification Strategy

Stripe publishes **incredibly detailed API specifications**:

**Specification for: Create a Charge**

```
POST https://api.stripe.com/v1/charges

Request Parameters:
  amount: integer (in cents, required)
    - Minimum: 50 (50 cents)
    - Maximum: 999999999
  currency: string (required)
    - Example: "usd", "eur", "gbp"
  source: string (required if customer not provided)
    - Can be: card token, card ID, bank account, Apple Pay token
  customer: string (optional)
    - Existing customer ID
  description: string (optional)
  metadata: object (optional)
    - Custom key-value pairs

Response:
  {
    "id": "ch_1234567890",
    "object": "charge",
    "amount": 1000,
    "currency": "usd",
    "created": 1234567890,
    "status": "succeeded" | "failed" | "pending"
    ... (30 more fields)
  }

Errors:
  400 Bad Request: Invalid parameters
  402 Payment Required: Card declined
  401 Unauthorized: Invalid API key
  429 Too Many Requests: Rate limited

Examples (Code in Python, Ruby, JavaScript):
  [Stripe provides runnable examples in 10+ languages]
```

### What This Enabled

- **100,000+ companies** integrate with Stripe
- **SDK support**: Python, Ruby, JavaScript, Go, Java, C#, PHP, etc.
- **Ecosystem**: 1000s of payment integrations built on Stripe API
- **Clear expectations**: Developers know exactly what to expect
- **Rapid integration**: Companies can integrate in days, not weeks

---

## Part 3: Google Cloud – Specifications as Organizational Structure

### Google's Approach

Google runs **the largest cloud infrastructure in the world**. Thousands of engineers across hundreds of teams. How do they prevent chaos?

**They use API-first architecture**: Every component publishes an API specification. Every team uses others' APIs. Services coordinate through specs.

### Example: Google Cloud Dataflow Specification

**Service**: Google Cloud Dataflow (data processing service)

**Key Specifications**:
1. **Pipeline API**: How to define data processing pipelines
2. **Worker API**: How to write transformation functions
3. **Monitoring API**: How to observe pipeline execution
4. **Cost API**: How to estimate and control costs

Each specification is detailed, documented, with SDKs in multiple languages.

### What This Enables

- **Independent scaling**: Each service scales independently
- **Independent innovation**: Teams innovate without affecting others
- **Ecosystem**: 3rd party tools build on Google Cloud APIs
- **Enterprise adoption**: Fortune 500 companies trust Google Cloud (because specs are clear)

---

## Part 4: GitHub – Open Source Specifications

### GitHub's Open Approach

GitHub publishes all API specifications **publicly**. Anyone can read them.

**GitHub API Specification** (publicly available):

```
GET /repos/{owner}/{repo}/issues

Query Parameters:
  state: "open" | "closed" | "all"
  labels: comma-separated list of label names
  sort: "created" | "updated" | "comments"
  direction: "asc" | "desc"
  per_page: 1-100
  page: page number

Response:
  [
    {
      "id": 1,
      "url": "https://api.github.com/repos/...",
      "title": "Issue title",
      "body": "Issue description",
      "state": "open" | "closed",
      ... (20 more fields)
    }
  ]

Rate Limit: 60 requests per hour (unauthenticated), 5000 per hour (authenticated)
```

### What This Enabled

- **10,000+ integrations** built on GitHub API
- **Developer tools**: IDEs, CI/CD systems, analytics tools all integrate
- **Open source community**: Anyone can build on GitHub
- **Business model**: GitHub became a $13 billion acquisition (in part, because specs are public)

---

## Part 5: What These Companies Have in Common

Let's extract the patterns:

### Pattern 1: Specifications are Products

At Amazon, Stripe, Google, GitHub:
- API specification is as important as code
- Specs are documented in detail
- Specs are stable (versions are maintained)
- Breaking changes are rare and announced
- Backwards compatibility is assumed

**Why?** Because other teams and external companies depend on specs. If specs change unexpectedly, thousands of users break.

### Pattern 2: Specifications Enable Ecosystems

When specs are clear and stable:
- Third-party companies build integrations
- User communities form
- Network effects multiply value
- Business model expands

**Example**: Stripe publishes clear specs → 3rd party payment processors build on Stripe → Customers get more value → More companies use Stripe → Stripe grows.

### Pattern 3: Specifications Reduce Communication

At scale, meetings don't work. Specs replace meetings:
- Amazon: No need to ask the User Service team "What's your data format?" Read the spec.
- Stripe: No need to ask "How do I create a charge?" Read the spec.
- Google: No need to ask "What's the Dataflow pipeline format?" Read the spec.

### Pattern 4: Specifications Enable Parallelization

Because specs are clear:
- 5000+ Amazon engineers can work independently
- 1000s of Stripe integrations can happen in parallel
- 1000s of Google Cloud services can evolve independently

### Pattern 5: Specification Quality Equals Business Quality

Companies with clear specs:
- Ship faster
- Have fewer bugs
- Enable partners
- Build thriving ecosystems
- Achieve organizational scale

---

## Part 6: Extract Lessons for Your Work

Now let's extract specific lessons you can apply.

Ask your companion:

> **Prompt**: "I've studied how Amazon, Stripe, Google, and GitHub use specifications. Now I'm building [your project]. Extract 5 lessons from these companies that apply to my context. For each lesson, show me: (1) What the big company does, (2) Why they do it, (3) How I can apply it to my team size and scope."

Your companion will help you extract lessons like:

**Lesson 1: Make Specs Your Product**
- Amazon: API spec is as important as code
- Why: Because thousands of teams depend on it
- Your application: If other teams will use your service, invest in clear specs

**Lesson 2: Versioning Matters**
- Stripe: Maintains API versions; can deprecate old versions
- Why: Because breaking changes destroy integrations
- Your application: If your specs will be used long-term, version them

**Lesson 3: Specs Enable Partners**
- GitHub: Public specs enable 10,000+ integrations
- Why: Because clear specs remove friction
- Your application: If you want others to build on your work, publish clear specs

**Lesson 4: Specs Replace Meetings**
- Google: No need to ask "How does this service work?" Read the spec.
- Why: Because meetings don't scale
- Your application: Invest in clear specs upfront; save meeting time later

**Lesson 5: Document with Examples**
- Stripe: Shows examples in 10+ languages
- Why: Because examples remove ambiguity
- Your application: When you write specs, include concrete examples

---

## Part 7: Analyze a Real Company's Specs

Let's do a hands-on analysis. Pick a real company's API and study their specs.

### Step 1: Choose a Company

Pick one:
- Stripe (stripe.com/docs/api)
- GitHub (docs.github.com/en/rest)
- AWS (docs.aws.amazon.com)
- Google Cloud (cloud.google.com/docs)
- Twilio (twilio.com/docs/usage/api)

### Step 2: Find a Specific API Endpoint

Pick one API endpoint. Example: "Create a charge" (Stripe) or "Create an issue" (GitHub).

### Step 3: Analyze the Specification

For that endpoint, answer:
1. **What's specified clearly?** (exact input format, exact output format, error cases)
2. **What's left to interpretation?** (Are there ambiguities?)
3. **What examples are provided?** (How many languages? How detailed?)
4. **What's the commitment?** (SLA, rate limits, versioning)
5. **How would you feel** if you had to integrate with this API? Easy or hard?

### Step 4: Compare With Your Specs

Ask your companion:

> **Prompt**: "I analyzed [Company]'s API specification for [endpoint]. Here's what I noticed: [your analysis]. Now compare it with the specifications I've written: [paste your feature spec]. What's my spec missing? How could I improve my spec using [Company]'s approach?"

---

## Part 8: Identify Your Company Type

Different companies at different scales use specs differently.

Ask your companion:

> **Prompt**: "I'm going to describe my company/team: [describe your context]. Based on that, which pattern from Amazon, Stripe, Google, GitHub best applies? Should my specs be like [Company]'s specs? Why? What should I adopt from their approach?"

### Company Types and Their Spec Focus

**Type 1: Internal Tools Team (5-20 people)**
- Specs: Focused on internal clarity
- Pattern: Amazon's internal standards
- Goal: Enable independent feature development

**Type 2: SaaS Platform (50-200 people)**
- Specs: APIs are core product feature
- Pattern: Stripe's public API approach
- Goal: Enable customer integrations

**Type 3: Enterprise/Cloud Platform (200+ people)**
- Specs: Complex ecosystem
- Pattern: Google Cloud's service specification approach
- Goal: Enable organizational scale and partner ecosystem

**Type 4: Open Source Project (contributor community)**
- Specs: Publicly documented
- Pattern: GitHub's open spec approach
- Goal: Enable community contributions

---

## Part 9: Reflection

Answer these questions:

1. **What surprised you** about how real companies use specs?

2. **Which company's approach** is closest to what you want to build? Why?

3. **What's the biggest gap** between their specs and yours? How will you close it?

4. **How would your work change** if you treated your specifications like a product (the way Stripe does)?

5. **What ecosystem** could you enable with clear specifications? Who would build on your specs?

---

## Try With AI

Deepen your learning by analyzing a real company's approach.

**Setup**: Use your AI companion to study and analyze real specifications.

**Prompt Set**:

1. **Study in depth:**
   > "Show me a detailed specification from [Company]. Walk me through: (1) What's specified, (2) How detailed, (3) What examples are provided, (4) What's the user's experience when reading this spec?"

2. **Compare approaches:**
   > "Compare specifications from [Company A] and [Company B] for similar features. Show me differences in: clarity, detail, examples, commitment (SLA), versioning. Which is better and why?"

3. **Analyze for your context:**
   > "I'm building [your project] for [your audience]. Should I follow [Company]'s specification approach or [Other Company]'s approach? Design my spec strategy based on the best example."

4. **Challenge the pattern:**
   > "Real companies use detailed specifications, but what are the downsides? When do specs become too detailed? When do they slow you down instead of speed you up?"

**Expected Outcomes**:
- You understand how real companies use specifications
- You can identify what makes specifications good (clarity, examples, commitment)
- You can design specifications for your specific context
- You recognize that specification-driven development is how professional companies scale
- You're prepared to apply these patterns in Parts 6-13 (where APIs, agent specs, monitoring specs become central)

**Perspective**: The companies you studied (Amazon, Stripe, Google) didn't invent specification-driven development. They adopted it because it was the only way to scale. You're learning the same approach. In a few years, if you work at a scaling company, you'll be the one designing specifications that 1000s of people depend on.

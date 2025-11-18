---
title: Lesson 3 - Progressive Loading Strategy
sidebar_position: 3
chapter: 11
lesson: 3
learning_objectives:
  - Apply three-phase progressive loading (Foundation → Current → On-Demand) to new codebase
  - Collaborate with AI to discover optimal loading patterns for project-specific constraints
  - Iterate with AI to refine loading strategies based on context budget and task requirements
estimated_time: 60 minutes
proficiency_level: B1
generated_by: content-implementer v1.0.0
source_spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
created: 2025-01-18
version: 1.0.0
---

# Lesson 3: Progressive Loading Strategy

## The Problem You're About to Solve

You're inheriting a large FastAPI project. It has 60 files. You need to add a new endpoint for user authentication. But loading all 60 files at once will fill your context window to 90%+ before you even start implementing.

What do you load first?

Lessons 1-2 taught you to recognize degradation when it arrives. This lesson teaches you to **prevent degradation before it starts** through strategic, progressive loading of context.

This lesson brings AI into the conversation. You'll discover that context loading isn't something AI does *to* you—it's something you **collaborate on together**. AI suggests patterns, you correct assumptions based on your project's reality, and together you converge on a loading strategy that fits your specific codebase.

## The Three Phases of Progressive Loading

Before diving into a real example, meet the three phases. Each answers a different question:

### Phase 1: Foundation (Session Start)

**Question**: What's the minimum context needed to understand the project structure and start work?

**Answer**: Load approximately 5-10 files covering:
- Project structure and README
- Core configuration files (requirements.txt, settings, environment templates)
- Data models (schema definitions, core entity definitions)
- API entry points (main application file, router setup)

**Why this matters**: Foundation context gives AI a mental model of your codebase without overwhelming context. AI understands "this is a FastAPI app, uses PostgreSQL, has these core entities."

**Size**: Typically 3,000-5,000 tokens. At 200K context window, this is 1.5-2.5% of your budget.

### Phase 2: Current Work (Task-Specific)

**Question**: What files are directly related to the feature I'm implementing RIGHT NOW?

**Answer**: Load only files you're actively modifying or that your feature depends on directly. For an authentication endpoint, that might be:
- Authentication models (user schema, auth flow definitions)
- Authentication routes (where your new endpoint goes)
- Authentication utilities (token generation, hashing functions)
- Database session/connection utilities
- Tests for authentication (to understand testing patterns)

**Why this matters**: Current work phase keeps focus sharp. AI knows "we're building authentication, these are the relevant files" without loading unrelated payment processing or reporting modules.

**Size**: Typically 4,000-8,000 tokens. Total after Phase 1+2 is ~7,000-13,000 tokens (3.5-6.5% of 200K window).

### Phase 3: On-Demand (Just-In-Time)

**Question**: What does AI need WHILE IMPLEMENTING that we didn't anticipate?

**Answer**: During implementation, AI might say: "I need to see how database sessions are initialized" or "What's the email notification system look like?" You fetch just that file in response to AI's request.

**Why this matters**: On-demand loading lets you add context only when necessary, keeping window utilization low throughout the session.

**Size**: Variable, typically 1,000-3,000 tokens per request, added as needed.

## Exercise 1: Discover Progressive Loading Through Practice

Before I explain the framework, let's experience why it matters.

**Scenario**: You have a real FastAPI project with 60 files. You're going to implement an authentication endpoint. Below is what FULL loading would look like, and what PROGRESSIVE loading would look like.

**Option A: Load Everything (Naive Approach)**

```
project/
├── main.py
├── config.py
├── requirements.txt
├── models/
│   ├── __init__.py
│   ├── user.py
│   ├── product.py
│   ├── order.py
│   ├── payment.py
│   ├── review.py
│   └── inventory.py
├── routes/
│   ├── __init__.py
│   ├── auth.py
│   ├── users.py
│   ├── products.py
│   ├── orders.py
│   ├── payments.py
│   ├── reviews.py
│   └── inventory.py
├── schemas/
│   ├── user.py
│   ├── product.py
│   ├── order.py
│   └── payment.py
├── services/
│   ├── auth_service.py
│   ├── user_service.py
│   ├── product_service.py
│   ├── order_service.py
│   ├── payment_service.py
│   └── notification_service.py
├── utils/
│   ├── database.py
│   ├── security.py
│   ├── email.py
│   ├── logging.py
│   └── validators.py
├── middleware/
│   ├── auth.py
│   ├── logging.py
│   ├── cors.py
│   └── error_handling.py
├── tests/
│   ├── test_auth.py
│   ├── test_users.py
│   ├── test_products.py
│   ├── test_orders.py
│   └── test_payments.py
└── migrations/
    └── versions/
        ├── 001_create_users_table.py
        ├── 002_add_auth_fields.py
        ├── 003_create_products_table.py
        └── 004_create_orders_table.py
```

**Loading all 60 files at start**: ~50,000 tokens. You're at 25% context utilization before writing a single line of code. You have 75% remaining budget for conversation, code generation, and responses. This works—but it's wasteful.

**Option B: Progressive Loading**

**Phase 1 (Foundation)**: Load 5 files:
- main.py (500 tokens)
- config.py (200 tokens)
- models/user.py (400 tokens)
- routes/auth.py (300 tokens)
- utils/security.py (400 tokens)

**Total Phase 1**: ~1,800 tokens (0.9% of window)

**You tell AI**: "I'm implementing OAuth2 support in the authentication system."

**Phase 2 (Current Work)**: Based on your task, load additional files:
- schemas/user.py (300 tokens)
- services/auth_service.py (600 tokens)
- tests/test_auth.py (400 tokens)
- utils/email.py (300 tokens)

**Total Phase 1+2**: ~3,400 tokens (1.7% of window)

**You have 98.3% context window remaining** while AI fully understands the project and your current task.

**During implementation**:
- AI: "I need to see how the database session is created."
- You: Load utils/database.py (300 tokens)
- Total after Phase 3: ~3,700 tokens (1.85% of window)

This is the efficiency of progressive loading.

## Collaborative Loading Strategy in Action

AI isn't a passive tool that loads files on command. You collaborate on loading strategy through back-and-forth dialogue.

Watch this happen in a real conversation.

## Session Transcript: Discovering the Optimal Loading Pattern

This is a complete session showing how collaborative refinement works.

### Suggesting a Loading Pattern

You start the session and describe your task:

```
You: I have a FastAPI project with 60 files. I need to add OAuth2
authentication. I've loaded main.py and models/user.py so far.
What should I load next to set up OAuth2 effectively?

---

Claude: Good starting point. For OAuth2 implementation, I recommend
loading context in this pattern:

FOUNDATION PHASE (what you've started):
- ✅ main.py (project structure)
- ✅ models/user.py (user entity definition)

CURRENT WORK PHASE (add these next):
- config.py (OAuth2 settings, provider keys)
- routes/auth.py (where OAuth2 routes will live)
- services/auth_service.py (authentication logic)
- utils/security.py (token generation, validation)
- tests/test_auth.py (understand existing auth tests)

Why this order? OAuth2 has dependencies:
1. Main.py tells me how the app is structured
2. User.py defines the user entity
3. Config.py has OAuth2 provider credentials and settings
4. Routes/auth.py is where the new endpoint lives
5. Services/auth_service.py contains the business logic you'll expand
6. Utils/security.py might already have token utilities you can reuse
7. Tests show you the testing patterns for auth endpoints

This approach keeps your context focused on OAuth2 concern while
preventing surprises from hidden dependencies.

What's in config.py? Does it already have OAuth2 settings?
```

**What emerged**: A structured approach to file loading based on dependency analysis—structure first, then entities, configuration, routes, business logic, utilities, and finally tests. This specific order prevents "hidden dependency" surprises.

The AI suggested a pattern you might not have discovered independently through trial and error.

---

### Refining Based on Project Reality

Now you describe your project's reality:

```
You: Good suggestion. But our config.py is actually generated from
environment variables—we don't hardcode OAuth2 settings there. Instead,
we have a separate services/oauth_providers.py file that handles all
provider configuration (Google, GitHub, Microsoft). Should I load that
instead of config.py?

---

Claude: Ah, that's important context I didn't know. You've designed
config for environment-driven setup, which is good security practice.
Let me revise:

REVISED CURRENT WORK PHASE:
- services/oauth_providers.py (provider configuration) ← Load THIS instead of config.py
- routes/auth.py
- services/auth_service.py
- utils/security.py
- tests/test_auth.py

Your approach is actually better than what I suggested. Loading
oauth_providers.py directly shows me provider handling patterns without
me guessing at config.py structure.

QUESTION: Do you have a models/oauth_session.py or similar for storing
OAuth tokens? Or does that logic live in auth_service.py?
```

**What happened**: Your project-specific feedback refined the recommendation. You corrected the initial assumption about config.py structure, explaining that OAuth provider configuration lives in a separate service file. What started as a generic suggestion became tailored to your architecture.

---

### Iterative Refinement: Balancing Context Budget

```
You: We store OAuth tokens in models/oauth_token.py. But I'm worried—
loading all these files will blow up my context window. We're already at
maybe 5,000 tokens. If I add 7 more files, I'll hit 15,000 tokens just
for setup. That's 7.5% of my budget. It feels like too much upfront.

---

Claude: You're right to be cautious. Let me reconsider. The risk:
loading too much upfront leaves you limited budget for actual
implementation and responses.

What if we do a TWO-TIER approach:

TIER 1 (LOAD NOW - absolute minimum to understand OAuth2 scope):
- services/oauth_providers.py
- routes/auth.py
- models/oauth_token.py
- utils/security.py
Total: ~2,500 tokens

TIER 2 (LOAD LATER - when you need them):
- services/auth_service.py (load when implementing provider callback)
- tests/test_auth.py (load when writing tests)
- models/user.py (you already have this)

This keeps Phase 2 to ~2,500 tokens, leaving 96.75% context window
for actual coding and problem-solving. We load Tier 2 files ONLY when
implementation tells us we need them.

Does this feel more balanced?
```

**What happened**: You raised a constraint (context budget concern). The AI revised the strategy to address your specific limitation. Neither of you had this exact two-tier approach initially—it **emerged from the dialogue**.

The collaboration produced a solution that balanced completeness with efficiency.

---

### Further Iteration: Dependency Discovery

```
You: I like the two-tier idea. But actually, looking at my routes/auth.py,
the callback handler calls services/auth_service.py directly. So auth_service
is TIER 1, not TIER 2. And I realize tests/test_auth.py imports oauth_providers,
so there's a dependency. If I don't load the test file, I might not understand
the OAuth2 flow that the tests expect.

You know what, let me just load the whole thing upfront:
- services/oauth_providers.py
- routes/auth.py
- models/oauth_token.py
- utils/security.py
- services/auth_service.py
- tests/test_auth.py

Is there anything else critical I'm missing?

---

Claude: Good catch on those dependencies. You know your codebase better
than I do. Let me verify: does anything ELSE depend on these files?

What about:
- models/user.py (already loaded, good)
- middleware/auth.py (OAuth2 middleware? or is that separate?)
- services/notification_service.py (does OAuth callback trigger emails?)

I want to make sure we're not missing transitive dependencies.

If middleware/auth.py is different from routes/auth.py and handles
JWT validation, we might need that too.

You: Actually, middleware/auth.py does the JWT validation, but that's
separate from OAuth setup. It's not on the critical path for building
the OAuth endpoint. Skip it for now.

Claude: Perfect. Your final list is complete then:
- services/oauth_providers.py
- routes/auth.py
- models/oauth_token.py
- utils/security.py
- services/auth_service.py
- tests/test_auth.py

This is ~5,000 tokens total. You're at 2.5% context utilization with
97.5% budget for implementation. Good place to start. When AI needs
additional context during coding, we load it on-demand.

What's the first thing you want to implement?
```

**What happened**: Through continued back-and-forth, you discovered additional dependencies (auth_service.py called from routes, tests importing oauth_providers). The AI asked clarifying questions about middleware and notifications. Together, you eliminated unnecessary files. The loading strategy became sharp and specific to YOUR project, not generic.

Neither of you had this final strategy at the beginning. It **emerged from iterative dialogue**.

---

## Formalizing the Pattern: Foundation → Current → On-Demand

The session above happened naturally. Now let's formalize what we observed into a decision framework you can apply to any project.

### Foundation Phase Decision Criteria

**Load these files at session start:**

1. **Project structure/README** (understanding the domain)
   - Example: README.md, architecture.md, or project-overview.md
   - Size: 500-1000 tokens

2. **Main application entry point** (how the app starts)
   - Example: FastAPI → main.py or app.py
   - Example: Django → manage.py or wsgi.py
   - Size: 300-800 tokens

3. **Core configuration** (non-secret settings)
   - Example: database URL pattern, logging config, feature flags
   - Size: 200-400 tokens

4. **Primary data models** (core entities)
   - Example: User, Product, Order—NOT every model
   - Size: 400-1000 tokens

5. **API structure** (how routes are organized)
   - Example: FastAPI → router setup or endpoint organization
   - Size: 200-400 tokens

**Total Foundation Phase**: 1,600-3,600 tokens (0.8-1.8% of 200K window)

**Decision rule**: If a file is essential for understanding "what is this project and how does it work?", load it in Foundation.

### Current Work Phase Decision Criteria

**Load these files when starting on a specific feature:**

1. **Routes for your feature** (where you're implementing)
   - Example: For auth feature → routes/auth.py
   - Size: 300-1000 tokens

2. **Data models for your feature** (entities you'll work with)
   - Example: For auth → models/user.py, models/oauth_token.py
   - Size: 300-800 tokens

3. **Business logic services** (the core implementation)
   - Example: For auth → services/auth_service.py, services/oauth_providers.py
   - Size: 400-1200 tokens

4. **Utilities your feature uses** (helpers, security, validation)
   - Example: For auth → utils/security.py, utils/email.py
   - Size: 300-800 tokens

5. **Tests for your feature** (understand testing patterns)
   - Example: For auth → tests/test_auth.py
   - Size: 300-600 tokens

**Total Current Work Phase**: 1,600-4,400 tokens
**Total After Phase 1+2**: 3,200-8,000 tokens (1.6-4% of 200K window)

**Decision rule**: If a file is directly related to the feature you're implementing RIGHT NOW, load it in Current Work. If it's "might be useful eventually," defer to On-Demand.

### On-Demand Phase Decision Criteria

**Load these files WHEN AI ASKS for them during implementation:**

**Examples of AI requests that trigger On-Demand loading:**

- "I need to see how you're querying the database. Load utils/database.py."
- "What's your email notification system? Load services/notification_service.py."
- "Show me an example of a similar route for pattern reference. Load routes/users.py."
- "I need to understand your database schema. Load migrations/versions/..."

**Size per request**: 300-1500 tokens, added as needed

**Decision rule**: Load it when AI says "I need to see X" OR when you realize "AI should know how X works to implement Y correctly."

## Exercise 2: Applying Progressive Loading to a Real Scenario

You have a real FastAPI project with these files. Your task: Add a payment webhook handler. Decide which files to load in each phase.

**Files in the project**:
- main.py (200 lines)
- config.py (150 lines)
- models/payment.py, models/user.py, models/order.py
- routes/payments.py, routes/users.py, routes/orders.py
- services/payment_service.py, services/order_service.py, services/user_service.py
- utils/security.py, utils/database.py, utils/email.py
- middleware/logging.py, middleware/auth.py
- tests/test_payments.py, tests/test_webhooks.py
- migrations/versions/... (5 migration files)

**Your task**: Add a webhook handler in routes/payments.py that receives Stripe webhook events and updates payment status.

<details>
<summary>Show my suggested loading strategy</summary>

**Foundation Phase** (what the project IS):
- main.py (how app is structured)
- models/payment.py (what a payment looks like)

**Current Work Phase** (what you're building):
- routes/payments.py (where webhook handler goes)
- services/payment_service.py (payment business logic)
- utils/security.py (webhook signature verification)
- tests/test_webhooks.py (understand webhook testing patterns)

**Total Phase 1+2**: ~3,500 tokens

**NOT loading**:
- models/user.py, models/order.py (not directly needed for webhook)
- routes/users.py, routes/orders.py (not directly needed)
- services/user_service.py, services/order_service.py (not directly needed)
- middleware files (webhook doesn't go through middleware)
- migrations (nice to know, but not needed to implement webhook)

**On-Demand** (ask for when needed):
- utils/database.py (if payment_service doesn't show how to query DB)
- utils/email.py (if need to send notifications on payment updates)
- config.py (if webhook signature secret is in config)

</details>

## How the Collaboration Works

The OAuth2 session above demonstrates several key interaction patterns:
- AI suggests an initial approach based on common practices
- You provide project-specific context that refines the suggestion
- You raise constraints (like context budget concerns)
- AI asks clarifying questions about dependencies
- Together you iterate toward a solution tailored to your codebase

This happens when you **ask good questions**, **share domain knowledge**, and **iterate together**.

### Making Collaboration Effective

**Get initial suggestions**: Ask "What's the recommended pattern here?" to get AI's baseline recommendation.

**Provide project context**: Share specifics: "Actually, our setup is different..." so AI can adapt.

**Raise concerns**: Voice constraints ("context budget!"), ask clarifying questions ("what about transitive dependencies?"), iterate on solutions.

## Common Mistakes in Progressive Loading

### Mistake 1: Loading Too Much Upfront

**Wrong**:
```
"I'll load everything so I don't have to ask later"
```

**Why it fails**: You waste 20-30% of context window on files you won't need. Later, when degradation appears, you're at 85%+ utilization with no room to load additional context.

**Right**:
```
Load 2-5 files for foundation, 5-8 for current work, add on-demand.
Total Phase 1+2: 2-5% of window. 95% remaining for implementation.
```

### Mistake 2: Loading Too Little, Forcing Constant On-Demand Requests

**Wrong**:
```
"I'll load just main.py and one model. I'll ask for everything else."
```

**Why it fails**: You spend half the session saying "load X, load Y, load Z." Context window fragments. AI loses focus jumping between files.

**Right**:
```
Think ahead: "For auth feature, I'll need these 6 files for comprehensive understanding."
Load once. Work for 30+ minutes without interruption.
```

### Mistake 3: Not Communicating Context Constraints to AI

**Wrong**:
```
You: "Load all the files you think are relevant for authentication."
Claude: [Loads 30 files, uses 60% of context window upfront]
```

**Why it fails**: AI doesn't know your context budget. It optimizes for "completeness" not "efficiency."

**Right**:
```
You: "I have a 200K context window. I want to stay under 5%
utilization for foundation + current work. What files are
essential for OAuth2?"
Claude: [Suggests focused file list that fits your constraint]
```

## Try With AI

You've now learned progressive loading strategy and seen Three Roles in action. Apply this immediately through AI collaboration.

### Setup

Open Claude Code (or your preferred AI tool) with a real project you work on (or use an open-source project if you don't have one). You're going to implement a feature collaboratively with AI, managing context through progressive loading.

### Three-Part Collaboration

**Part 1: Propose a Feature**

Load only the project README or structure, then say:

```
I'm thinking about implementing [feature name] in this project.
I have about 200K context window and want to stay under 5%
for planning. What files should I load?
```

**Observe**: AI suggests a loading pattern based on common dependency flows.

**Part 2: Correct and Refine**

After AI suggests files, say:

```
Actually, we structure things differently. We have [your-reality].
Does that change your suggestion?
```

**Observe**: AI adapts its recommendation to your project's architecture.

**Part 3: Iterate**

As you load files and start implementing, raise concerns:

```
I'm worried we're loading too much. Context is approaching 10%.
Should I pause and load on-demand instead of preloading [file]?
```

**Observe**: Through iteration, you arrive at a better strategy together.

### Measure Success

When done, you should:
- ✅ Have a clear loading strategy specific to YOUR project
- ✅ Know what files are Foundation vs Current vs On-Demand
- ✅ Have experienced collaborative refinement through iterative dialogue
- ✅ Have avoided degradation by staying under 70% context utilization during implementation

### Safety Note

When loading files, be mindful of **sensitive data** (API keys, secrets, PII). If your project has .env files, security credentials, or personal data, discuss with AI how to load structure without exposing secrets. Safe patterns: load code structure + tests, avoid raw credential files.


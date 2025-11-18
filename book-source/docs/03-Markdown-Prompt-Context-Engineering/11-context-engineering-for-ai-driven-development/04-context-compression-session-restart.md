---
title: Lesson 4 - Context Compression and Session Restart
sidebar_position: 4
chapter: 11
lesson: 4
learning_objectives:
  - Create structured checkpoint summaries capturing architectural decisions, progress, and next steps
  - Measure context reclamation after session restart (50%+ recovery target)
  - Collaborate with AI to develop effective checkpoint structures tailored to project needs
  - Iterate with AI to optimize checkpoint density and information preservation
estimated_time: 75 minutes
proficiency_level: B1
generated_by: content-implementer v1.0.0
source_spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
created: 2025-01-18
version: 1.0.0
---

# Lesson 4: Context Compression and Session Restart

## The Problem You're About to Solve

You've been working for two hours straight. You're implementing a payment processing feature for an e-commerce platform. You've:

- Set up Stripe webhook integration
- Built the payment processing service
- Implemented idempotency keys to prevent duplicate charges
- Written database migrations for payment history tracking
- Added unit tests for edge cases

Your context window is at 85% utilization. You have 30K tokens remaining out of 200K.

And then it happens.

You ask Claude for help with the final webhook validation step, and Claude says: "For webhook validation, you'll want to implement signature verification using..." and starts suggesting a pattern that you already discussed and **rejected** two hours ago.

Claude forgot.

The context window is full. Lessons 1-2 taught you to recognize this degradation. Lesson 3 taught you to prevent it through strategic loading. But here you are, two hours into focused work, and degradation has arrived anyway.

This lesson teaches you to **recover from degradation** through compression: creating a checkpoint summary that preserves essential knowledge, restarting the session with that checkpoint, and reclaiming context space to continue working.

## Understanding Checkpoints: Context Preservation Strategy

Before diving into a real example, let's understand what a checkpoint actually does.

### What Is a Checkpoint?

A **checkpoint** is a structured summary of your session state—the decisions you've made, the work you've completed, and what comes next. It's not just a transcript. It's a carefully designed document that extracts the signal (important decisions) from the noise (implementation details, failed attempts, tangents).

### The Checkpoint Structure

An effective checkpoint has three core sections:

**Section 1: Architectural Decisions Made**
- Which major decisions shaped the work?
- Why were those decisions made?
- What constraints or requirements drove them?

Example:
```
ARCHITECTURAL DECISIONS:
1. Stripe Webhook Signature Verification (REJECTED):
   - Initial approach: Using raw signature verification library
   - Decision: Use Stripe's official webhook validation library
   - Why: Official library handles edge cases (clock skew, deprecations)
   - Impact: Prevents common webhook validation bugs

2. Idempotency Key Strategy (ADOPTED):
   - Decision: Store idempotency keys in payment_idempotency table
   - Why: Allows duplicate request detection without modifying payment records
   - Constraint: Keys expire after 24 hours
   - Impact: Prevents duplicate charges from network retries
```

**Section 2: Progress Completed**
- What tasks finished?
- What's the current state of the codebase?
- Where did you leave off?

Example:
```
PROGRESS COMPLETED:
- ✅ Stripe API client initialization (services/stripe_client.py)
- ✅ Webhook endpoint created (routes/webhooks.py) with signature validation
- ✅ Idempotency key table schema (migrations/payment_idempotency.py)
- ✅ Payment processing service core logic (services/payment_service.py)
- ✅ Database transaction handling for atomicity
- ✅ Unit tests for happy path and edge cases (tests/test_webhooks.py)
- ⏳ IN PROGRESS: Webhook recovery (retry logic for failed webhook deliveries)
```

**Section 3: Next Steps**
- What's immediately next?
- What are the follow-up tasks?
- Any blockers or dependencies?

Example:
```
NEXT STEPS:
1. (HIGH PRIORITY) Implement webhook retry logic:
   - Stripe sends failed webhooks up to 3 times
   - Store failed webhook attempts with timestamps
   - Implement exponential backoff for retries
   - Add monitoring/alerting for permanently failed webhooks

2. (MEDIUM) Handle refund webhooks:
   - Process refund events from Stripe
   - Update payment status to "refunded"
   - Trigger customer notification emails

3. (MEDIUM) Add webhook dead-letter queue:
   - Webhooks failing after 3 retries go to DLQ
   - Manual review process for DLQ items

BLOCKERS: None currently
```

### Why This Structure Matters

The checkpoint extracts **signal**—decisions and progress—while filtering out **noise** from your session:

- Noise: "I tried approach A, didn't work. Tried approach B, still broken. Finally tried approach C which worked." (implementation details)
- Signal: "Decision: Use approach C because it handles edge case E." (decision and rationale)

The smaller, cleaner checkpoint becomes your new session context. Claude reads it and continues working with full understanding of your decisions, without the 50K tokens of conversation history.

## Exercise 1: Experience Degradation and Recovery

Let's experience checkpoint compression through a real scenario.

### Scenario Setup

You've been implementing payment processing for 2 hours. Below is a **simulation** of what your session might look like near the end (degraded state), and then the checkpoint you'll create to recover.

### Simulation: Session at 85% Utilization (Degraded)

Imagine this is your session context after 2 hours:

```
[Earlier conversation history: 100K tokens worth of:]
- Initial prompt about payment processing requirements
- Back-and-forth on Stripe webhook signature validation
- Discussion of idempotency patterns
- Code generation and refinement rounds
- Testing strategy discussions
- Database migration design
- Edge case handling
[Many implementation details, failed attempts, refinements]

[Recent message]
You: Claude, for the webhook recovery logic, do we need to implement retry queues?

Claude: Great question. For webhook recovery, I'd recommend a simple pattern:
when a webhook fails, store it in a retry_queue table and process it
later using a background job. You could use Celery for async processing...

[But Claude is repeating earlier discussion and missing the context]
```

At this point, Claude has:
- ✅ Seen 85% of context window (170K of 200K)
- ❌ Lost some nuance of earlier decisions
- ❌ Is repeating suggestions from earlier in conversation
- ❌ Is forgetting that you discussed AND REJECTED retry_queue approach

This is the compression moment.

### Creating Your Checkpoint

You stop active implementation and create a checkpoint:

```markdown
# Session Checkpoint: Stripe Payment Processing Integration

## Architectural Decisions Made

### 1. Stripe Webhook Signature Verification
- **Pattern Evaluated**: Using Stripe's signature verification library
- **Decision**: ADOPTED - Use stripe.Webhook.construct_event()
- **Rationale**: Official library handles timestamp validation, header parsing, edge cases
- **Constraint**: Webhook secret key must be loaded from environment
- **Implementation**: validation in routes/webhooks.py validates all incoming webhooks before processing

### 2. Idempotency Key Strategy
- **Pattern Evaluated**: Store idempotency keys in separate table vs embedding in payment records
- **Decision**: ADOPTED - Separate payment_idempotency table
- **Rationale**: Allows detecting duplicate requests without modifying core payment record. Enables tracking retry attempts separately.
- **Constraint**: Keys expire after 24 hours (configurable)
- **Implementation**: Check idempotency_key in table, if exists return previous result, if new store and process

### 3. Retry Queue Pattern
- **Pattern Evaluated**: Celery async queue for failed webhooks vs in-database retry table with background service
- **Decision**: REJECTED Celery, ADOPTED in-database retry table with cron job
- **Rationale**: Simpler initial implementation. Failed webhooks stored in webhook_failures table with attempt count and next_retry_time. Cron job processes retries. Reduces external dependencies for MVP.
- **Constraint**: Cron runs every 5 minutes. Max 3 retry attempts per webhook.
- **Implementation**: See services/webhook_retry_handler.py (in progress)

## Progress Completed

### Code Artifacts
- ✅ services/stripe_client.py - Stripe API client initialization with API key from environment
- ✅ routes/webhooks.py - POST /webhooks/stripe endpoint with signature validation
- ✅ migrations/001_create_payments_table.py - Payment table schema
- ✅ migrations/002_create_idempotency_keys_table.py - Idempotency table schema
- ✅ services/payment_service.py - Core payment processing (charge, refund methods)
- ✅ services/stripe_webhook_handler.py - Webhook event router (charge.succeeded, charge.failed, etc.)
- ✅ tests/test_webhook_signature.py - Signature validation tests
- ✅ tests/test_payment_idempotency.py - Duplicate detection tests
- ✅ tests/test_webhook_handler.py - Event routing tests

### What Works Now
- Webhooks from Stripe properly validated with signature verification
- Duplicate webhook events detected via idempotency key lookup
- Payment records created with status tracking (pending, completed, failed, refunded)
- Basic transaction logging for audit trail

### In Progress
- Webhook retry logic (currently at 40% completion)
  - webhook_failures table schema complete
  - webhook_retry_handler.py service skeleton written
  - Cron scheduling setup started (not yet tested)

## Next Steps (Immediate Priority Order)

### 1. Complete Webhook Retry Logic (2-3 hours estimated)
- Finish webhook_retry_handler.py to implement:
  - Poll webhook_failures table for entries where next_retry_time <= now()
  - Attempt webhook reprocessing up to 3 times
  - Log attempt count and errors
  - Move to permanent_failures table after 3 failed attempts
- Test with manual webhook triggering (using Stripe CLI test events)

### 2. Webhook Dead-Letter Queue (1-2 hours estimated)
- Create permanent_failures table for webhooks failing 3+ times
- Implement monitoring query: SELECT * FROM permanent_failures WHERE created_at > NOW() - INTERVAL 24h
- Document manual remediation process for DevOps team

### 3. Refund Webhook Handling (1 hour estimated)
- Add handler for charge.refunded events
- Update payment status to "refunded" in database
- Trigger customer notification email via services/notification_service.py

### 4. Testing in Staging (before production)
- Use Stripe test mode webhook events
- Verify retry logic with manual Stripe CLI webhook triggers
- Load test with rapid webhook deliveries

## Context Notes for Next Session

**Codebase Structure**:
- services/ - Business logic (stripe_client, payment_service, webhook handlers)
- routes/ - API endpoints (webhooks.py is main webhook entry point)
- migrations/ - Database schemas (numbered sequentially, always run migrations before testing)
- tests/ - Unit tests (use pytest, test file names match source module names)

**Configuration**:
- Stripe API key: Read from STRIPE_API_KEY environment variable
- Webhook secret: Read from STRIPE_WEBHOOK_SECRET environment variable
- Database: PostgreSQL, connection via DATABASE_URL environment variable

**Testing Pattern**:
- Unit tests in tests/ directory
- Use pytest fixtures for Stripe mock objects (stripe.Webhook.construct_event is mocked)
- Test both success and failure paths for webhook handlers

**Key Modules to Know**:
- services/stripe_client.py - All Stripe API interactions flow through here
- routes/webhooks.py - Webhook validation and routing (entry point for all webhooks)
- services/stripe_webhook_handler.py - Event-specific handlers

Token Budget: 450 tokens (within 500-token target)
```

This checkpoint is **450 tokens**. Compare that to your **170K token session context**. You've compressed 2 hours of work into a few hundred tokens while preserving all the critical decisions.

Now comes the powerful part: **session restart**.

## Session Restart: Recovery in Action

You close your Claude Code session. You open a **new** Claude Code session. Your first message is the checkpoint you just created:

```
[Paste checkpoint as session opening message]

Continue with: I was working on webhook retry logic.
The service skeleton is in services/webhook_retry_handler.py
but it's incomplete. Let me share that file...

[Paste webhook_retry_handler.py - partially written code]

I'll load the current state. Then help me implement
the retry polling logic that checks webhook_failures
table every 5 minutes.
```

What happens:

1. **Claude reads the checkpoint** - Understanding your project state, all decisions made, and constraints
2. **You provide the current code** - Claude sees webhook_retry_handler.py (partially written) and understands exactly what to implement
3. **Context window resets** - You've freed up 140K+ tokens that were consumed by 2-hour conversation history
4. **Work continues seamlessly** - Claude implements retry logic aware of all your architectural decisions

**Context reclamation achieved**: You went from 85% (170K) utilization to ~20% (40K) utilization with fresh session.

You've reclaimed **65%** of your context budget.

## Exercise 2: Collaborating on Checkpoint Creation

Now let's see how checkpoint creation benefits from collaboration with AI.

### Discovering Checkpoint Structure

**Scenario**: Session is degrading. You ask Claude for help structuring a checkpoint.

```
You: My session is running out of context. I need to create
a checkpoint summary before restarting. What should go in it?

---

Claude: Great instinct to checkpoint. Here's a pattern that works well:

A CHECKPOINT HAS THREE CORE SECTIONS:

1. **Architectural Decisions** - Major decisions made, rationale, constraints
   - Why this decision matters: Future code must respect these constraints
   - Example: "Rejected Celery queue pattern, using cron-based retry instead (constraint: 5-min intervals)"

2. **Progress Completed** - What's done, what's in progress, code artifacts created
   - Why this matters: Shows current state of codebase, prevents redundant work
   - Example: "✅ Payment service created, ⏳ Webhook retry handler (40% complete)"

3. **Next Steps** - Ordered by priority, with estimates and dependencies
   - Why this matters: Prevents restart confusion about what to do next
   - Example: "1. Complete webhook retry logic (2-3h), 2. Test with Stripe CLI (1h)"

**TOKEN BUDGET**: 500 tokens maximum
- Keeps checkpoint lightweight
- Preserves only high-signal content
- Leaves room for code context in new session

**Why this structure?** Your checkpoint is like a commit message for your session.
Good commit messages are specific, structured, and future-proof.
```

**What emerged**: A structured approach to checkpoint creation with three key sections (decisions, progress, next steps), explicit token budget discipline (500 tokens maximum), and clear reasoning for why each section matters for future session continuity.

The AI suggested a pattern that helps extract signal from noise in your session history.

---

### Refining with Project Specifics

**Scenario**: You start drafting your checkpoint, then realize your generic structure misses important context.

```
Claude (generic checkpoint): Here's a checkpoint template:

ARCHITECTURAL DECISIONS:
- Webhook signature validation using Stripe library
- Idempotency key storage pattern

PROGRESS:
- Payment service created
- Webhook handler completed
- 8 tests passing

NEXT STEPS:
- Implement webhook retries
- Handle refunds
- Add monitoring

---

You: Wait, that's too vague. "Payment service created" doesn't
tell me enough about what's ACTUALLY in that service. And
"webhook retries" is too broad—I need to remember I specifically
REJECTED the Celery queue pattern and decided on cron-based retries
with constraints (5-minute intervals, max 3 attempts). That decision
took me 30 minutes to reach. I need to record WHY I decided that.

Also, in the next session, I might forget that webhook_failures
goes to the database schema, not Celery. I need to be specific
about the table name and its purpose.

---

Claude: You're absolutely right. The generic template loses critical
context. Let me revise to capture those specifics:

ARCHITECTURAL DECISIONS:
- **Webhook Signature Verification**
  Decision: Use stripe.Webhook.construct_event() official library
  Why: Handles edge cases (timestamp validation, header parsing)

- **Retry Queue Pattern**
  Evaluated: Celery async queue
  Decision: REJECTED - Instead use in-database retry table with cron job
  Why: Simpler MVP, fewer external dependencies
  Constraint: Cron runs every 5 minutes, max 3 attempts per webhook

PROGRESS:
- ✅ services/stripe_client.py - API client
- ✅ routes/webhooks.py - Endpoint with signature validation
- ✅ services/stripe_webhook_handler.py - Event routing by type
- ⏳ services/webhook_retry_handler.py - 40% complete (retry polling)

This captures the DECISION RATIONALE and specific CONSTRAINTS
that matter when you resume work.
```

**What happened**: Your feedback about specificity prompted a revision. You explained what was missing (the WHY behind decisions, the specific constraints like 5-minute cron intervals, the rejection of Celery). The checkpoint evolved from generic template to project-specific documentation that captures what you actually need to remember.

---

### Iterative Optimization: Balancing Detail and Brevity

**Scenario**: Working together to optimize token budget vs information density.

```
You: Your revised checkpoint is better, but now it's 800 tokens.
That defeats the purpose—I'm trying to save context space.
Can we compress it while keeping the important parts?

---

Claude: Good catch. Let me trim non-essentials:

CHECKPOINT v1 (Your attempt): 450 tokens (too sparse, missing decision rationale)
CHECKPOINT v2 (My suggestion): 800 tokens (too verbose, loses budget efficiency)
CHECKPOINT v3 (Converged): 500 tokens

What did we drop from v2?
- Removed: Verbose explanations of WHY each decision (students can infer)
- Kept: Decision name, specific pattern chosen, constraint if critical
- Result: 50% token reduction, 95% information retention

For example, instead of:
"Webhook signature verification using Stripe's official library
is important because it handles edge cases like timestamp validation
and HTTP header parsing, which custom implementations often miss"

We write:
"Decision: stripe.Webhook.construct_event() official library
Why: Handles edge cases (timestamp, header parsing)
Constraint: Webhook secret from environment"

CHECKPOINT v3 DRAFT:
[500 tokens with optimized sections]

What's the first thing you'll do in the restarted session?

You: I'll paste this checkpoint, then load the partially-written
webhook_retry_handler.py file and ask you to complete the retry
polling logic.

Claude: Perfect. That means our checkpoint needs to reference
webhook_retry_handler.py specifically—current state, line numbers,
what's complete vs incomplete:

"⏳ services/webhook_retry_handler.py - 40% complete
  - ✅ Function signatures defined
  - ✅ Database query scaffolding
  - ❌ Retry attempt loop (needs implementation)
  - ❌ Error handling for failed webhooks"

This helps me understand exactly what to pick up on in the next session.

---

[After 3-4 iterations]

FINAL CHECKPOINT: 480 tokens
- Sufficient detail for continuation
- Efficient use of token budget
- Captures all critical decisions
- Points to specific code state (line counts, completion %)
```

**What happened**: Through multiple iterations, you and the AI converged on an optimal checkpoint. You raised the token budget concern, the AI suggested compression techniques, and together you identified what could be trimmed (verbose explanations) versus what must be preserved (decision names, constraints, specific file states).

The result was:
- **Detailed enough** to preserve critical context
- **Concise enough** to reclaim budget
- **Specific enough** to guide implementation in next session

Neither of you had this exact balance initially. It **emerged from iterative refinement**.

---

## Formalizing the Pattern: Checkpoint Structure and Metrics

The session above happened naturally. Now let's formalize the checkpoint pattern and metrics you can apply.

### Checkpoint Template (Production-Ready)

```markdown
# Session Checkpoint: [Project Name]

## Architectural Decisions Made

### Decision 1: [Category/Component]
- **Pattern Evaluated**: [What approaches you considered]
- **Decision**: [Which pattern you chose]
- **Rationale**: [Why this decision was made]
- **Constraint**: [Critical limitations or requirements]
- **Implementation Location**: [Key file or module]

### Decision 2: [Category/Component]
- **Pattern Evaluated**: ...
- **Decision**: ...

[Repeat for each major decision, prioritized by impact]

## Progress Completed

### Artifacts
- ✅ [File/Module] - [Brief purpose, e.g., "API client initialization"]
- ✅ [File/Module] - [Brief purpose]
- ⏳ [File/Module] - [Brief purpose, % completion if in progress]
- ⏳ [File/Module] - [Brief purpose, specific gap/what needs completion]

### Testing Status
- [Number] unit tests passing
- [Coverage percentage] code coverage
- [Any test failures or gaps noted]

## Next Steps (Ordered by Priority)

### 1. [High-priority task]
- **Subtasks**: [2-3 specific actions]
- **Estimated time**: [Hours]
- **Dependencies**: [Any other tasks this depends on]
- **Success criteria**: [How you'll know this is done]

### 2. [Medium-priority task]
- [Same structure]

### 3. [Lower-priority task]
- [Same structure]

**Blockers**: [Any obstacles preventing progress]
**Critical Context**: [Anything the next session absolutely must know]

---
Token Count: [Actual count, target 400-500]
```

### Metrics: Measuring Checkpoint Effectiveness

A good checkpoint achieves these metrics:

**Compression Ratio**: How much context did you reclaim?
- Calculation: (Old session token count - New session token count) / Old session token count × 100%
- Target: 50%+ recovery (e.g., from 170K to 85K utilization)
- Example: 2-hour session = 150K tokens. Checkpoint = 500 tokens. Recovery: (150K - 500) / 150K = 99.67% freed

**Information Density**: Did you preserve critical context in small size?
- Measurement: In new session, can you continue work without ramp-up or re-explanation?
- Target: Zero re-explanation needed (Claude reads checkpoint and immediately understands constraints, decisions, code state)

**Token Efficiency**: Did you stay within budget?
- Target: Checkpoint + code files loaded in new session ≤ 70% of window
- Example: Checkpoint (500) + Code files (8,000) + Prompt (500) = 9,000 tokens = 4.5% utilization. Goal achieved.

**Continuation Quality**: Does work in new session respect checkpoint decisions?
- Measurement: Does Claude's code in new session follow the patterns you decided on?
- Target: 100% - Claude should never contradict architectural decisions from checkpoint

## Exercise 3: Design a Checkpoint for Your Project

Now it's your turn. Using a real or imagined project, create a checkpoint summary following the template above.

**Scenario**: You've been working for 90+ minutes on a feature. Context is at 85%. Create a checkpoint that would let you restart effectively.

**Your task**:
1. Choose a real project you work on, OR use the Stripe payment processing example from earlier
2. Identify 3-5 major architectural decisions you've made (or would make)
3. List progress artifacts (files created, tests passing, what's incomplete)
4. Define next steps in priority order
5. Write the checkpoint using the template
6. Count tokens (target: 400-500)
7. Review: Could someone read this and understand your project's current state and constraints?

---

## Try With AI

**Setup**: Open Claude Code or your preferred AI development tool. We'll practice checkpoint creation with AI collaboration.

**Scenario**: You've been working on authentication feature implementation for 2+ hours. You're at 85% context utilization. Time to create a checkpoint and restart.

**Prompt Set**:

**Prompt 1: Request Checkpoint Guidance**:
```
I've been working on authentication implementation for our FastAPI
application. My context window is getting full (85% utilization).
I need to create a checkpoint summary before restarting the session.
What's a good structure for a checkpoint that will help me continue
work efficiently in a new session?
```

Listen for: AI suggests checkpoint structure with reasoning for each section.

**Prompt 2: Provide Project Specifics**:
```
Good structure. Here's my situation specifically:

DECISIONS MADE:
1. JWT-based authentication (stateless)
2. Refresh token rotation every 7 days
3. Session blacklist for logout (stored in Redis)

PROGRESS:
- Created auth models (user, token, session)
- Implemented login/logout routes
- Added JWT token generation and validation

INCOMPLETE:
- Refresh token rotation logic
- Session blacklist queries for revoked tokens

Help me refine this into a proper checkpoint. What's missing
that you'd need to understand to continue this work?
```

Listen for: AI asks clarifying questions about your project and adapts suggestions based on details.

**Prompt 3: Optimize for Efficiency**:
```
AI is asking for details on my refresh token rotation. This is what
I decided:

"Refresh tokens stored in postgres (not Redis) with columns:
token_value, user_id, issued_at, expires_at, rotated_token_value,
rotation_timestamp. Each refresh generates new token with previous
token value stored as rotated_token_value, preventing use of old token."

My checkpoint is getting long though (already 650 tokens). How do I
capture this decision concisely without losing the important details?
```

Listen for: AI helps you balance detail vs brevity to create an efficient yet complete checkpoint.

**Expected Outcomes**:
- Prompt 1: Discover effective checkpoint structure patterns
- Prompt 2: Tailor checkpoint to your project's specific needs
- Prompt 3: Optimize checkpoint for both completeness and token efficiency

**Safety Note**: Checkpoints are most effective when they're honest about incomplete work and known issues—don't hide "abandoned attempts" or "bugs to fix" because that information helps AI avoid the same mistakes.

---

**Version 1.0.0**
**Next Lesson**: Lesson 5 - Context Isolation for Parallel Tasks (identifying when to separate vs combine contexts)

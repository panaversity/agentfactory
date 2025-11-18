---
title: Lesson 8 - Hands-On Debugging and Optimization
sidebar_position: 8
chapter: 11
lesson: 8
learning_objectives:
  - Diagnose failing context scenarios (degradation, pollution, saturation, persistence)
  - Apply appropriate remediation strategies from Lessons 1-7
  - Measure optimization results and validate improvements
  - Demonstrate AI collaboration in complex troubleshooting
estimated_time: 90 minutes
proficiency_level: B1
generated_by: content-implementer v1.0.0
source_spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
created: 2025-01-18
version: 1.0.0
---

# Lesson 8: Hands-On Debugging and Optimization

## The Problem You're About to Solve

Over the last 7 lessons, you've learned context engineering techniques:
- Recognize degradation symptoms (Lesson 2)
- Apply progressive loading (Lesson 3)
- Create checkpoints and compress context (Lesson 4)
- Isolate parallel tasks (Lesson 5)
- Build persistent memory files (Lesson 6)
- Select the right tool (Lesson 7)

But **reading about techniques** and **executing them under pressure** are different skills.

This lesson puts you in real failure scenarios. You'll **diagnose problems, apply solutions, measure results** — like you would in actual development work.

This is Layer 2 Validation: **Applying all learned patterns to integrated scenarios**.

## Scenario 1: Degradation Crisis

### The Situation

You're 3 hours into a feature implementation. The session started fresh with 200K tokens available. You've:

- Loaded project files (30K tokens)
- Implemented core logic (40K tokens)
- Added tests (35K tokens)
- Fixed bugs (30K tokens)
- **Current state**: 135K tokens used (67% utilization)

You're about to implement the final piece: error handling and edge cases.

You ask Claude to "implement comprehensive error handling for all edge cases."

Claude responds with code that looks right, but you notice something odd:

```python
def process_payment(amount, customer_id):
    """Process payment with error handling."""

    # Validate inputs
    if amount <= 0:
        raise ValueError("Amount must be positive")

    # Check customer exists
    customer = get_customer(customer_id)
    if not customer:
        raise ValueError("Customer not found")

    # Process with payment gateway
    result = stripe.charge.create(
        amount=amount,
        currency='usd',
        customer_id=customer_id,
        description=f"Payment for {customer_id}",
        # PROBLEM: This contradicts earlier decision
        test_mode=True  # Never use test mode in production!
    )

    return result
```

But you remember clearly: **Earlier in this session, you decided AGAINST test mode in production.** You have an environment-based approach:

```python
# From 2 hours ago (early in session):
stripe.api_key = os.getenv('STRIPE_SECRET')
test_mode = os.getenv('ENVIRONMENT') == 'test'
```

Claude **forgot** this decision despite it being in the current session. It's suggesting a pattern contradictory to your earlier decision.

**Why?** Degradation. Your context is 67% full. Claude is starting to lose memory of earlier decisions.

### Diagnosis Exercise

Before looking at the solution, **diagnose the problem**:

1. **What's the symptom?** (Look at the evidence)
   - Claude forgot earlier decision
   - Contradictory code suggestions
   - Pattern regression

2. **Which lessons does this relate to?**
   - Lesson 1: Context window mechanics
   - Lesson 2: Degradation symptoms (forgotten earlier decisions is Symptom #3)

3. **What's the root cause?**
   - Session at 67% utilization
   - Earlier decisions fading from AI's available context
   - Approaching degradation threshold

4. **What strategy should fix this?**
   - Lesson 4: Context Compression with checkpoint

### Remediation: Create Checkpoint

**Step 1: Identify what to checkpoint**

What important decisions might be at risk of being forgotten?

```markdown
# Session Checkpoint

## Architectural Decisions Made
1. Environment-based configuration (not test mode in production)
   - STRIPE_SECRET from .env
   - test_mode = (ENVIRONMENT == 'test')
   - Pattern: All external service calls check environment first

2. Authentication approach: JWT with 24-hour expiry
   - Tokens signed with HS256
   - Refresh tokens stored in Redis
   - No bcrypt here (Argon2 for password hashing only)

3. Database approach: SQLAlchemy query API (not ORM)
   - All queries explicit
   - No lazy loading
   - Migrations via Alembic

## Progress So Far
- ✅ Core payment logic implemented
- ✅ Database models created
- ✅ API endpoints scaffolded
- ✅ Tests for payment flow passing
- ✅ Error handling partially implemented

## Next Steps
- [ ] Complete error handling (edge cases)
- [ ] Add logging for all payment events
- [ ] Integration tests with Stripe
- [ ] Final deployment checklist
```

**Step 2: Create new session with checkpoint**

Close current session. Save checkpoint to `CHECKPOINT.md`.

Start new session with:

```
Here's my project context from memory files:

[CLAUDE.md content]
[architecture.md content]

And here's my session checkpoint from earlier work:

[CHECKPOINT.md content]

Now continue: Complete error handling implementation, remembering:
- Environment-based config pattern (NOT test_mode hardcoded)
- JWT approach with 24h expiry
- SQLAlchemy query API
```

**Step 3: Verify checkpoint worked**

Claude's next response:

```python
def process_payment(amount, customer_id):
    """Process payment with error handling."""

    # Validate inputs
    if amount <= 0:
        raise ValueError("Amount must be positive")

    # Check customer exists
    customer = get_customer(customer_id)
    if not customer:
        raise ValueError("Customer not found")

    # Process with payment gateway
    test_mode = os.getenv('ENVIRONMENT') == 'test'
    result = stripe.charge.create(
        amount=amount,
        currency='usd',
        customer_id=customer_id,
        description=f"Payment for {customer_id}",
        test_mode=test_mode  # Uses environment variable, not hardcoded!
    )

    return result
```

**Checkpoint worked!** Claude recovered the environment-based config pattern and applied it correctly.

### Metrics Validation

**Before compression**:
- Context utilization: 67%
- Code quality: Degraded (contradictory patterns)
- Session duration: 3 hours

**After compression + restart**:
- Context utilization: 25% (after loading checkpoint)
- Code quality: Restored (correct patterns)
- Session duration: 3+ hours (continued)

**Result**: Reclaimed 42% of context by creating checkpoint. Prevented degradation from getting worse.

---

## Scenario 2: Context Pollution

### The Situation

You're working on a fintech application. Two parallel tasks in the same session:

**Task A** (started first): Implement authentication
- JWT tokens, password hashing, login/logout
- 15 files loaded, 40K tokens

**Task B** (added midway): Fix critical bug in payments
- Stripe webhook handling, signature verification, transaction rollback
- Added 12 more files, now 85K tokens

Now you're implementing webhook signature verification for Stripe:

```python
def verify_webhook_signature(payload, signature, webhook_secret):
    """Verify Stripe webhook signature."""

    # Construct signed content
    signed_content = payload + webhook_secret

    # Generate HMAC
    expected_signature = hmac.new(
        webhook_secret.encode(),
        signed_content.encode(),
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(expected_signature, signature)
```

But this is **wrong for Stripe**. Stripe's webhook signature verification is completely different:

```python
# CORRECT Stripe approach:
# 1. Reconstruct: timestamp + '.' + json_payload
# 2. HMAC with webhook secret
# 3. Compare with signature header

def verify_stripe_webhook(request_body, signature_header, endpoint_secret):
    """Correct Stripe webhook verification."""

    timestamp = request_body.split(',')[0].split('=')[1]
    signed_content = f'{timestamp}.{request_body}'

    expected_sig = hmac.new(
        endpoint_secret.encode(),
        signed_content.encode(),
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(expected_sig, signature_header)
```

Why did Claude generate wrong code?

**Context pollution**: Both authentication and payment are in the same session. Claude mixed the authentication pattern (general HMAC verification) with the payment pattern (Stripe-specific signature format).

### Diagnosis Exercise

1. **What's the symptom?**
   - Wrong webhook verification approach
   - Mixed authentication patterns into payment domain

2. **Which lessons does this relate to?**
   - Lesson 5: Context pollution and isolation
   - Symptoms: AI suggesting patterns from one domain in another domain

3. **What's the root cause?**
   - Both auth and payment context loaded together
   - AI can't distinguish which pattern applies where
   - Domain boundaries unclear in single session

4. **What strategy should fix this?**
   - Lesson 5: Context Isolation
   - Create separate session for payment task

### Remediation: Isolate Context

**Step 1: Save authentication session state**

You've completed authentication work. Save it:

```
Session A (Authentication):
- Completed: Login, logout, token refresh
- Status: Ready to merge

Save state: Save all auth-related code, tests, implementation notes
```

**Step 2: Start isolated payment session**

Close Session A. Start Session B with only payment context:

```
# Session B: Payment Processing (Isolated)

## Project Context
[Load CLAUDE.md and architecture.md - same as Session A]

## Session-Specific Focus
This session is FOCUSED on payment processing only.
Do NOT apply authentication patterns here.

## Payment Task
Implement webhook signature verification for Stripe webhooks.

Key constraint: Stripe uses HMAC with specific format:
- Reconstruct: timestamp + '.' + json_payload
- HMAC the reconstructed content
- Compare with signature header

Do NOT use generic HMAC pattern from authentication.
This is Stripe-specific.
```

**Step 3: Verify isolation worked**

Claude's next response uses correct Stripe-specific verification:

```python
def verify_stripe_webhook(request_body, signature_header, endpoint_secret):
    """Correct Stripe webhook verification."""

    timestamp = request_body.split(',')[0].split('=')[1]
    signed_content = f'{timestamp}.{request_body}'

    expected_sig = hmac.new(
        endpoint_secret.encode(),
        signed_content.encode(),
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(expected_sig, signature_header)
```

**Isolation worked!** Removing authentication context from the payment session prevented pattern cross-contamination.

### Metrics Validation

**Before isolation** (mixed session):
- Files in context: 27 (15 auth + 12 payment)
- Tokens used: 85K
- Code quality: Incorrect (wrong pattern)

**After isolation** (separated sessions):
- Session A: 15 files, 40K tokens, correct auth code
- Session B: 12 files, 35K tokens, correct payment code
- Quality: Both sessions correct

**Result**: By isolating contexts, each session focused on correct patterns for its domain.

---

## Scenario 3: Saturation Problem

### The Situation

Your project is complex. You've loaded:

- Core models (15 files, 20K tokens)
- Database layer (8 files, 15K tokens)
- API routes (12 files, 25K tokens)
- Tests (10 files, 18K tokens)
- **Total**: 45 files, 78K tokens (39% utilization)

You're about to implement a new critical feature: real-time notifications. This requires:

- WebSocket setup (3 new files)
- Redis integration (4 new files)
- Event publishing system (5 new files)
- Notification templates (8 new files)
- Tests for notifications (6 new files)

That's 26 MORE files, ~45K tokens.

**Total would be**: 71 files, 123K tokens (61% utilization)

But you realize: **You still need to implement the feature logic, write tests, fix bugs, and handle edge cases.** Those will add another 40K tokens.

**Final projection**: 163K tokens (81% utilization) — approaching degradation.

And you haven't started on **two other features** scheduled for this sprint.

### Diagnosis Exercise

1. **What's the problem?**
   - Context budget running out
   - Can't fit all notification feature + previous work in one session
   - But notification feature is critical

2. **Which lessons apply?**
   - Lesson 3: Progressive Loading (load only what's needed)
   - Lesson 4: Compression (checkpoint current work to free space)

3. **What's the root cause?**
   - Loaded too much up front (45 files)
   - No prioritization of what's essential

4. **What strategy should fix this?**
   - Lesson 3: Progressive Loading — load Foundation phase only, add Current phase dynamically

### Remediation: Progressive Loading

**Step 1: Identify Foundation vs Current**

**Foundation phase** (always needed):
- Core models (entities, schemas) — 10 essential files, 10K tokens
- Main API setup (FastAPI app, config) — 3 files, 5K tokens
- **Total Foundation**: 13 files, 15K tokens

**Current work** (notification feature):
- WebSocket setup — 3 files, 8K tokens
- Event system — 5 files, 12K tokens
- Notification logic — 6 files, 10K tokens
- Tests for notifications — 6 files, 12K tokens
- **Total Current**: 20 files, 42K tokens

**On-demand** (fetch only if needed):
- Specific route handlers (200+ routes)
- Specific test utilities
- Legacy modules not related to notification feature

**Step 2: Implement Progressive Loading**

Start new session:

```
# Notification Feature Development — Progressive Loading

## Foundation Phase (Always Loaded)
[Load 13 files: core models, API setup]
- Core models (10 files, 10K)
- API config (3 files, 5K)
Total: 15K tokens

## Current Work Phase (Notification Feature)
[Load 20 files for this feature]
- WebSocket setup (3 files)
- Event system (5 files)
- Notification logic (6 files)
- Tests (6 files)
Total: 42K tokens

## Running Tally
Foundation + Current = 57K tokens (28% utilization)

## Implementation Task
Build real-time notification feature with this loaded context.
If you need files not yet loaded, ask for them and I'll fetch on-demand.
```

**Step 3: Implement feature**

As you build, if Claude needs additional files:

```
Claude: I need to understand how user sessions work to send notifications
to the right users. Can you load app/services/session.py?

You: [Load session.py — 3K tokens]

Running tally: 60K tokens (30% utilization)
```

**Step 4: Validate remaining budget**

After implementing and testing notification feature:

```
Final tally: 110K tokens (55% utilization)

Remaining budget: 90K tokens (45%)
Still room for:
- Two more moderate features
- Bug fixes
- Final optimization
```

### Metrics Validation

**Without progressive loading** (naive approach):
- Would load: 71 files, 123K tokens
- Utilization: 61%
- Risk: Can't complete feature without degradation

**With progressive loading**:
- Load: 13 foundation + 20 current = 33 files, 57K tokens
- Utilization: 28%
- Fetch: 3 more files as needed, reaching 60K tokens (30%)
- Final: 110K tokens (55%) after feature complete
- Budget remaining: 90K tokens for more work

**Result**: Progressive loading kept context utilization low, completed feature with budget to spare.

---

## Scenario 4: Persistence Failure

### The Situation

It's Wednesday morning. You start a new Claude Code session to continue the feature you worked on Monday.

You ask: "Let me continue implementing the reporting dashboard. What was our approach for caching report results?"

Claude: "I don't have context about your reporting dashboard approach. What caching strategy did you choose?"

**Wait, what?** This should be your second day of development. Claude should remember Monday's work.

But this is a **new session**. Claude has no memory of Monday unless you loaded your memory files.

You left them in your project:
- CLAUDE.md (project conventions)
- architecture.md (system design)
- decisions.md (architectural decisions)

But you didn't load them into this new session.

**This is persistence failure**: You created memory files (Lesson 6) but didn't implement the persistence strategy correctly.

### Diagnosis Exercise

1. **What's the problem?**
   - New session, no context about Monday's work
   - Memory files exist but weren't loaded
   - Lost continuity

2. **Which lessons apply?**
   - Lesson 6: Memory Files and Persistent Intelligence
   - Lesson 8 (this lesson): Validation of techniques

3. **What's the root cause?**
   - Memory files created but persistence strategy not implemented
   - Session start didn't load memory files
   - No automation for loading

4. **What strategy should fix this?**
   - Lesson 6: Implement read-on-session-start workflow

### Remediation: Implement Persistence Strategy

**Step 1: Load memory files at session start**

When starting new session:

```
I'm continuing work on the reporting dashboard feature.

Here's my project context from memory files:

[Load and paste CLAUDE.md]

[Load and paste architecture.md]

[Load and paste decisions.md]

Now, what was our approach for caching report results?
```

**Step 2: Claude recovers context**

Claude now responds with full context:

```
Based on your memory files, I see that you:
- Chose Redis for caching report results
- Set cache expiry to 1 hour (from decisions.md ADR-005)
- Store cached results keyed by report_id + filters

On Monday you started implementing the caching layer.
Let me continue from there. What's the current state?
```

**Step 3: Update memory files after session**

After Wednesday's work, update decisions.md:

```markdown
## ADR-006: Report Result Caching Strategy

**Decision**: Cache report results in Redis with 1-hour expiry.

**Context**:
- Reports are expensive (multiple joins, aggregations)
- Users frequently run same report with same filters
- Need fast result retrieval for dashboard

**Rationale**:
- Redis provides fast in-memory caching
- 1-hour expiry balances freshness vs reuse
- Keyed by report_id + filter hash for accuracy

**Implementation** (Wednesday):
- Created CacheKey generator from report_id + filters
- Implemented cache_report decorator for report endpoints
- Added cache invalidation on new data

**Consequences**:
- Need to monitor cache hit rate
- Memory usage increases with cached reports (acceptable for MVP)
- Must handle cache misses gracefully
```

### Metrics Validation

**Without persistence strategy** (naive):
- Monday: Full context, build feature
- Tuesday: No context, restart from zero
- Productivity: 50% (lost Monday's context)

**With persistence strategy**:
- Monday: Build feature, update memory files
- Tuesday morning: Load memory files, recover Monday's context
- Tuesday: Continue seamlessly from Monday's stopping point
- Productivity: 90%+ (maintained continuity)

**Result**: Persistence strategy prevented context loss across sessions. Continuous development across multiple days without re-explanation.

---

## Integration Exercise: Combine All Strategies

Now you'll encounter a **complex scenario requiring multiple strategies from Lessons 1-7**.

### The Complex Scenario

It's a 5-day sprint. Your project:
- 100 files, 150K lines
- 3 parallel features (authentication, payments, notifications)
- Multiple developers contributing to different areas

Day 1 morning: You start fresh. **Which strategies apply?**

1. **Tool selection** (Lesson 7): 100 files is large. Use Claude Code with progressive loading or Gemini CLI for exploration?
   - **Your decision**: Start with Gemini CLI (2M context) to understand full architecture
   - Duration: 30 minutes
   - Output: Full architectural understanding

2. **Transition to Claude Code**: Switch to Claude Code for implementation (better IDE, deep reasoning)

3. **Memory files** (Lesson 6): Create CLAUDE.md, architecture.md, decisions.md based on Gemini's analysis
   - Capture patterns, conventions, key decisions
   - These become persistent context

4. **Progressive loading** (Lesson 3): For each feature, load Foundation + Current phase only
   - Foundation: Core models, config
   - Current: Task-specific files

5. **Checkpoints** (Lesson 4): At end of each day, create checkpoint summarizing progress and decisions

6. **Isolation** (Lesson 5): Each of 3 features in separate sessions to prevent pollution

7. **Persistence** (Lesson 6): Load memory files at each day's start

**5-Day Workflow**:

**Day 1**:
- Gemini CLI: Understand architecture (30 min)
- Create memory files based on understanding (1 hour)
- Claude Code: Start authentication feature (2 hours)
- Checkpoint: Save progress and decisions

**Day 2**:
- Load memory files
- Start new Claude Code session for authentication continuation
- Progressive loading: Foundation + auth work from Day 1
- Complete authentication feature
- Checkpoint: Update memory files with new decisions

**Days 3-4**: Repeat for payment and notification features

**Day 5**:
- Testing and validation across all features
- Memory files document all decisions
- Handoff ready

### Try With AI

**Setup**: Choose a real scenario or use provided example.

**Prompt Set**:

**Prompt 1: Diagnose the Problem**
```
I'm experiencing this problem in my AI development session:
[Describe your symptom: degradation, pollution, saturation, or persistence]

- Current context utilization: [X]%
- Task: [What are you building?]
- Session duration: [How long?]

Based on Lessons 1-7, what's the root cause?
What symptom from Lesson 2 matches this?
```

**Prompt 2: Apply Remediation**
```
I've diagnosed the problem as [degradation/pollution/saturation/persistence].

Based on Lesson [4/5/3/6], what's the remedy?

Walk me through the steps to fix this:
1. [What to do first]
2. [What to do next]
3. [How to validate it worked]
```

**Prompt 3: Multi-Strategy Approach**
```
I have a 5-day project with 3 parallel features and 100 files.

Design a complete strategy using techniques from Lessons 1-7:
- Which tool to start with? (Lesson 7)
- How to use memory files? (Lesson 6)
- How to apply progressive loading? (Lesson 3)
- When to create checkpoints? (Lesson 4)
- How to prevent pollution between features? (Lesson 5)
- How to persist context across days? (Lesson 6)

Create a detailed 5-day workflow.
```

**Expected Outcomes**:
- Prompt 1: Clear diagnosis with Lesson reference
- Prompt 2: Step-by-step remediation plan
- Prompt 3: Complete integrated workflow across 5 days

**Safety Note**: When sharing context with AI (memory files, transcripts), ensure no secrets or credentials are included. Use placeholders like "API_KEY_FROM_ENV" instead of actual keys.


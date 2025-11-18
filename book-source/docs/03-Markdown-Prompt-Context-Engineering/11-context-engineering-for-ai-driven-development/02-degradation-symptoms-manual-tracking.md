---
title: Lesson 2 - Degradation Symptoms and Manual Tracking
sidebar_position: 2
chapter: 11
lesson: 2
learning_objectives:
  - Recognize 5+ degradation symptoms in failing AI sessions
  - Manually track context utilization over session lifetime
  - Articulate when compression vs isolation vs restart is needed based on symptoms
estimated_time: 50 minutes
proficiency_level: B1
generated_by: content-implementer v1.0.0
source_spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
created: 2025-01-18
version: 1.0.0
---

# Lesson 2: Degradation Symptoms and Manual Tracking

## The Feeling When Context Window Fills

You're deep in a Claude Code session. You've been working for 90 minutes. The code is getting built. But something feels off.

The suggestions stop being specific to your project. The AI repeats advice from 30 minutes ago. You mention a design decision from early in the conversation, and the AI responds as if you never said it. Output starts coming slower. Generic suggestions replace the creative problem-solving you had earlier.

You're not experiencing a different AI model. You're experiencing **context degradation**—the AI running out of working memory.

Lesson 1 taught you to observe context window filling. This lesson teaches you to **name what you observe**, **track it systematically**, and **decide what to do about it**.

## The Five Degradation Symptoms

Context degradation doesn't announce itself with a red warning. It creeps in gradually through observable symptoms. Here are the five most common:

### Symptom 1: Repetitive Suggestions

The AI suggests the same solution multiple times across the session.

**What it looks like:**

```
[00:15] You: I need to handle authentication. What's the recommended approach?

Claude: I recommend JWT tokens with refresh token rotation. Here's the pattern:
1. Generate access token (short-lived, 15 minutes)
2. Generate refresh token (long-lived, 7 days)
3. Client stores both, uses access token for requests
4. When access token expires, client sends refresh token to get new access token
5. If refresh token is compromised, require re-login
```

Later in same session...

```
[01:20] You: Now I need to handle token expiration. How should I approach this?

Claude: Great question. For token expiration, I recommend JWT tokens with
refresh token rotation. Here's the pattern:
1. Generate access token (short-lived, 15 minutes)
2. Generate refresh token (long-lived, 7 days)
3. Client stores both, uses access token for requests...
[EXACT SAME EXPLANATION, NEARLY WORD-FOR-WORD]
```

**Why it happens:** Context is filling up. The AI no longer "remembers" it already explained this—so conversation history is still there (technically), but the model isn't effectively referencing it. It falls back to generating a plausible-sounding answer, which happens to be what it said before.

**How to recognize it:** You're thinking "didn't we already discuss this?" The explanation is correct but identical—same examples, same order of points, same phrasing.

---

### Symptom 2: Forgotten Context

The AI contradicts or ignores earlier decisions you made.

**What it looks like:**

```
[00:30] You: Our database is PostgreSQL. I'm using SQLAlchemy with async
support. We'll use alembic for migrations.

Claude: Good choices. PostgreSQL with SQLAlchemy is solid. Let me help
you with the schema design...

[Later, 80 minutes in...]

[01:50] You: I need to add a new field to the users table. Should I use
alembic or direct SQL?

Claude: For schema changes, you have a few options. You could write raw SQL
scripts to update the database. You could use SQLAlchemy's column
add operations directly in Python. Or you could use raw SQL in a
management script...

[ALEMBIC—THE TOOL YOU SPECIFIED AT 00:30—IS COMPLETELY ABSENT]
```

**Why it happens:** Conversation history exists (it's in tokens), but the model isn't effectively using it. New context is crowding out old context in the model's attention calculation. Earlier decisions get sidelined.

**How to recognize it:** You have to re-explain something you clearly stated. The AI's suggestion ignores decisions you made. You think "we already decided this"—and you did, but the AI doesn't remember.

---

### Symptom 3: Performance Degradation

Responses get noticeably slower to generate.

**What it looks like:**

```
[00:10] You: Write a login function.
Claude: [Response arrives in 2 seconds. Thoughtful, project-specific code.]

[01:50] (same session, 80+ minutes later) You: Write a logout function.
Claude: [Response takes 12 seconds. Noticeably slower.]
[Generic code, less specific to project patterns]
```

**Why it happens:** Transformer architecture (how LLMs work) requires computing attention over ALL previous tokens. As context grows, this becomes more expensive. The model slows down. Token generation rate drops.

**How to recognize it:** You notice response latency increasing. Messages that took 2-3 seconds now take 10+ seconds. It feels like the AI is "thinking harder."

---

### Symptom 4: Contradictory Advice

The AI suggests contradictory approaches in the same session.

**What it looks like:**

```
[00:45] You: Should we validate user input on the client or server?

Claude: Always validate on the server. Client-side validation improves UX,
but a malicious user can bypass it. Server validation is the security
boundary. Always validate on the server first.

[Later, 75 minutes in...]

[01:55] You: One more question on validation—what's the best practice?

Claude: The most efficient approach is to validate on the client side
and let client validation prevent unnecessary server requests. Here's
why client validation is the best practice...

[COMPLETELY REVERSED EARLIER POSITION. Client is NOT "best practice"
for security. But context degradation made the AI forget its own
earlier reasoning.]
```

**Why it happens:** The model is operating with limited context understanding. It generates a plausible-sounding answer without checking what it said before. Contradictions reveal broken reasoning chains.

**How to recognize it:** You catch a direct contradiction. AI suggests opposite approaches at different points. You think "did the AI just reverse its advice?"—yes, it did, because context degradation broke its ability to maintain consistent reasoning.

---

### Symptom 5: Context Reference Loss

The AI can't find or reference information you provided earlier.

**What it looks like:**

```
[00:20] You: Here's our API response format:
{
  "status": "success",
  "data": {...},
  "errors": [...]
}

Claude: Clear format. I'll follow this pattern for all API responses.

[Later, 80+ minutes in...]

[01:55] You: I need to create an error response for password validation failure.

Claude: Here's how to return an error:
return {"error": "Password validation failed"}

[COMPLETELY IGNORED THE RESPONSE FORMAT YOU ESTABLISHED. Should be:
return {"status": "error", "data": null, "errors": ["Password validation failed"]}
]
```

**Why it happens:** The information is technically in context (in earlier tokens), but the model isn't retrieving it effectively. It falls back to generic patterns.

**How to recognize it:** You have to re-explain project conventions. The AI's output doesn't follow patterns you established. You think "we agreed on this format"—and you did, but the AI can't retrieve it.

---

## Exercise 1: Symptom Identification

Read the 90-minute session transcript below. For each degradation symptom that appears, note:
1. The symptom type (1-5 from above)
2. When it appears (timestamp)
3. What specifically triggered your recognition

**90-Minute Claude Code Session: Building Payment Processing**

```
[00:05] You: I'm building payment processing for an e-commerce platform.
Stack: Python FastAPI backend, PostgreSQL database, Stripe API. Start by
loading the project structure and models/payment.py.

Claude: Loaded. I see a Payment model with fields for amount, currency,
user_id, created_at. Good start. To integrate Stripe, you'll need:
1. Stripe API keys in environment variables
2. Webhook handling for payment events
3. Payment status tracking (pending → completed → failed)
4. Error handling for declined cards and network failures

What would you like to implement first?

[00:20] You: Let's start with the Stripe integration. Create routes/stripe.py
with a create_payment endpoint.

Claude: I'll create a POST /payments endpoint that:
1. Takes amount and customer email from request
2. Creates Stripe charge using stripe.Charge.create()
3. Returns payment_id for tracking
4. Stores payment record in database with status "pending"
[Provides complete route code]

Good practice: always validate amount > 0 and handle stripe.error.CardError
for declined cards. The payment record status progresses: pending →
completed (when webhook received) → settled (funds received).

[00:35] You: Good. Now add webhook handling. When Stripe sends charge.completed
events, we need to update payment status in database.

Claude: Perfect. Create a POST /webhooks/stripe endpoint that:
1. Verifies webhook signature (stripe.Webhook.construct_event)
2. Checks event type (charge.completed, charge.failed)
3. Updates Payment record status based on event
4. Logs event for audit trail
[Provides webhook code]

This ensures your payment status stays in sync with Stripe's records.

[00:50] You: Now I need to handle failures. When a customer's card is declined,
we should retry once, then mark payment as failed.

Claude: For retry logic, I recommend:
1. First attempt: try stripe.Charge.create()
2. If CardError caught: wait 5 seconds, retry once
3. If second attempt fails: mark payment as failed in database
4. Send email to customer explaining failure
[Provides retry code with exponential backoff]

Save all payment attempts to audit log so you can debug failures.

[01:10] You: Good. Now let's add logging. I want to track every payment
operation. Create a logging setup.

Claude: I recommend structured logging using Python's logging module:
1. Create logs/payments.log file
2. Log every payment creation: timestamp, amount, user_id
3. Log every status change: pending → completed → failed
4. Log webhook events received
5. For errors, log full exception traceback
[Provides logging configuration]

This way you can audit payment history and debug issues.

[01:25] You: Add email notifications. When payment completes, send customer
an email receipt. When payment fails, send a failure notice.

Claude: For email, use the existing email service in your project. I saw
utils/email.py earlier. Send transactional emails:
1. On payment success: send receipt with amount, date, order details
2. On payment failure: send notice with retry instructions
3. Always use templated emails (not plain text) for professionalism
[Provides email sending code]

[01:40] You: Now I need to handle refunds. A customer should be able to
request a refund within 30 days.

Claude: For refund handling:
1. Add refund_requested_at field to Payment model
2. Create POST /payments/{payment_id}/refund endpoint
3. Validate: refund requested within 30 days of payment
4. If valid, call stripe.Refund.create(charge_id)
5. Update Payment status to "refunded"
6. Send email confirmation to customer
[Provides refund logic]

Only allow refunds within your business window (30 days is common).

[01:55] You: Good. Now, how should I structure the payment validation?
Should I validate in the route handler or in a separate validation layer?

Claude: For payment validation, I recommend validating in the route handler
directly. Here's why:
- Route handlers can directly access request data
- Validation is simple: just check amount > 0
- No need for complex validation logic
- Keep it simple in the handler function
[Provides inline validation example]

This is the most straightforward approach for payment processing.

[02:10] You: Hmm, actually that seems odd. Earlier I thought you recommended
a validation layer pattern. Let me search back...

Claude: Yes, you're right. Actually, I recommend creating a separate
validation layer. Here's why:
- Separates concerns (route logic vs validation)
- Reusable across multiple endpoints
- Easier to test validation independently
- Cleaner route handlers
[Provides validation layer pattern]

This is the more professional approach.

[02:25] You: Wait, these contradict. Which is it? Inline validation or
separate layer?

Claude: Good question. Both approaches work. Route handler validation is
simple but less scalable. Validation layers are better for larger
applications. For payment processing specifically...
[Provides generic response without clear recommendation]

The choice depends on your project scale. For a simple project, inline
is fine. For enterprise, use validation layers. You'll need to decide
based on your needs.

[02:40] You: Let me ask specifically about card validation. We need to
validate card details before sending to Stripe. What's the pattern?

Claude: For card validation, you have several options:
1. Validate using Stripe's client library
2. Validate using regex patterns
3. Use a validation library
4. Store raw card numbers in database
[NO MENTION OF PCI COMPLIANCE—CRITICAL SECURITY REQUIREMENT]

Choose whichever works for your workflow.

[02:55] You: Wait, should we store raw card data? That seems wrong.

Claude: Actually, no. Never store raw card numbers in your database.
That violates PCI compliance requirements. Always use Stripe's
tokenization:
1. Client sends card details to Stripe frontend (no server touch)
2. Stripe returns secure token
3. Server stores token, not card data
[Provides corrected guidance]

This is the secure pattern.

[03:05] You: We covered PCI compliance at 00:20 in our initial requirements,
and it's documented in security.md in the project. Why did you suggest
storing raw card data?

Claude: You're absolutely right. I apologize. PCI compliance is critical
for payment systems. The secure pattern is Stripe tokenization only.
Never store raw card numbers.
[Generic apology, slower response]

Let me refocus: for card validation, use Stripe's client-side token
generation and never store raw card data on your server.

[03:20] You: I'm going to wrap up here. This session has been productive
for the first 90 minutes, but the last 30 minutes have been degraded.
I'm noticing you're repeating advice, forgetting earlier decisions,
and contradicting yourself. I think the context window is filling up.

Claude: You're right. This is a good observation. As the session gets
longer and context accumulates, AI quality can degrade. I've been less
effective in the last 30 minutes:
- Contradicted my validation advice (inline vs layer)
- Momentarily forgot about PCI compliance
- Repeated concepts from earlier
- Responses have been slower and less specific

This is exactly what you learned in Lesson 1: context degradation.
Starting fresh in a new session will give you better focus. Well done
identifying the symptoms.
```

**Analysis Questions:**

1. **Repetitive suggestions**: Find at least one instance where Claude repeats advice. When does it occur?

2. **Forgotten context**: Identify when Claude forgets about PCI compliance and stripe tokenization. What does it suggest instead?

3. **Performance degradation**: Note the timestamps. When do responses start getting slower or more generic?

4. **Contradictory advice**: Find the validation advice contradiction. What makes it contradictory?

5. **Context reference loss**: When does Claude fail to reference something established earlier in the session?

<details>
<summary>Click to reveal analysis</summary>

**Symptom 1 - Repetitive Suggestions** (00:35 and beyond)
- The webhook handling pattern is explained multiple times with nearly identical structure
- Logging recommendations repeat the same points (timestamp, status changes, exceptions)

**Symptom 2 - Forgotten Context** (02:40)
- Claude suggests "Store raw card numbers in database"—completely contradicts PCI compliance requirement mentioned implicitly at 00:05 (Stripe API integration) and stored in security.md
- At 02:55, Claude corrects itself, but the fact it suggested raw card storage shows forgotten context

**Symptom 3 - Performance Degradation** (02:10 onwards)
- Early responses: specific code examples, clear patterns, confident recommendations
- Late responses (post 02:10): slower, more generic ("depends on your needs"), less specific
- You can see Claude taking longer to generate responses

**Symptom 4 - Contradictory Advice** (01:55 vs 02:10)
- At 01:55: "Validate in route handler, simpler approach"
- At 02:10: "Actually, validation layer is better"
- Both are valid patterns, but the same assistant suggested opposite approaches in same session
- Worse: Claude initially defended inline validation before flip-flopping

**Symptom 5 - Context Reference Loss** (03:05)
- You reference PCI compliance from "security.md in the project" mentioned early
- Claude doesn't reference this earlier decision when asked about card validation
- Has to be reminded: "We covered PCI compliance at 00:20"

**Timeline of Degradation:**
- 00:05-01:55: Excellent, specific advice, clear patterns, consistent
- 02:10: First contradictions appear (validation approach flip-flop)
- 02:40: Serious error (raw card data suggestion) shows context failure
- 02:55-03:05: Slow responses, need for correction, acknowledgment of degradation
- 03:20: Explicit recognition that last 30 minutes were degraded

**Lesson**: You don't need automated metrics to spot degradation. **Observation is enough**: repetition, contradictions, forgotten decisions, slower responses, generic advice. These are your warning signals.

</details>

## Framework: When to Compress vs Isolate vs Restart

Now that you can spot degradation symptoms, the question becomes: **what do you do about it?**

There are three mitigation strategies. Choosing the right one depends on **why** your context window is full and **what you're trying to accomplish next**.

### Decision Framework

```
SITUATION: Context window approaching saturation (80%+ utilization)
│
├─ QUESTION 1: Are you continuing the SAME TASK?
│  │
│  ├─ YES (same feature, same codebase, same goal)
│  │  └─ COMPRESS (Checkpoint + Restart)
│  │      Save progress summary, restart session with compressed context
│  │      Good for: Long feature implementation (3+ hours), staying focused
│  │
│  └─ NO (switching tasks or goals)
│     └─ ISOLATE (Separate Session)
│         Create independent session for new task
│         Good for: Context pollution, task switching, clarity
│
├─ QUESTION 2: Is the current session still productive?
│  │
│  ├─ YES (suggestions are specific, AI understands project)
│  │  └─ Keep working, monitor for degradation
│  │
│  └─ NO (repetition, contradictions, generic suggestions)
│     └─ RESTART (Fresh session)
│         Start completely fresh if compression isn't feasible
│         Good for: Severe degradation, recovery impossible
```

### Strategy 1: Compress (Checkpoint + Restart)

**Use when**: You're continuing the SAME TASK and want to maintain context continuity.

**How it works**:

1. **Create checkpoint** (500-1000 tokens) summarizing:
   - Architectural decisions made
   - Progress completed so far
   - Next steps to implement
   - Key patterns/patterns established

2. **Restart session** with checkpoint as context instead of full history

3. **Reclaim space** (often recovers 50-70% of context window)

**Example checkpoint** (for the payment processing session above):

```markdown
## Payment Processing Checkpoint (Completed after 90 minutes)

### Architecture Decisions
- Stack: FastAPI + PostgreSQL + Stripe API
- Authentication: JWT tokens required for payment endpoints
- PCI Compliance: Never store raw card data, use Stripe tokenization only
- Payment Status Flow: pending → completed → failed → refunded
- Error Handling: Catch stripe.error.CardError for card declines

### Progress Completed
- Stripe integration routes created (POST /payments)
- Webhook handler implemented (POST /webhooks/stripe)
- Retry logic for card declines (1 retry with 5s delay)
- Structured logging setup (logs/payments.log)
- Email notifications (receipt on success, failure notice)
- Refund logic (30-day window, stripe.Refund.create)

### Next Steps
1. Add payment filtering/search endpoints (GET /payments, GET /payments/{id})
2. Implement subscription logic (recurring payments)
3. Add payment analytics dashboard
4. Performance optimization (caching payment checks)

### Established Patterns
- Always validate input: amount > 0, valid currency
- Always check request authentication (JWT token required)
- Always handle CardError and NetworkError exceptions
- Use templated emails for transactional messages
- Log every state change for audit trail

### Files Modified
- routes/stripe.py: Payment endpoints + webhook
- models/payment.py: Payment model structure
- utils/email.py: Transactional email sending
- utils/logging.py: Structured logging config
- security.md: PCI compliance notes
```

**When to compress**:
- You're 90+ minutes into focused work on ONE feature
- Context window is 75%+ utilized
- AI suggestions are degrading
- You plan to continue feature development

**Recovery time**: 5 minutes (write checkpoint, paste into new session)

---

### Strategy 2: Isolate (Separate Session)

**Use when**: You're switching to a DIFFERENT TASK and want to avoid context pollution.

**How it works**:

1. **Finish current task** (or pause at natural breakpoint)
2. **Start NEW SESSION** for different task
3. **Keep old session alive** for returning to later if needed
4. **Benefit**: Each task has clean context, no interference

**Example isolation scenario**:

```
Session 1 (Payment Processing):
- 90 minutes of payment logic work
- Context at 85% utilization
- Feature nearing completion

Interrupt: Urgent bug in authentication module discovered

Option A (POLLUTION): Add bug fix to same session
- Problem: Auth bug context mixes with payment context
- AI gets confused about scope: "Should I fix bug or finish payments?"
- Context window explodes to 95%
- Both features suffer

Option B (ISOLATION): Create separate session
- Session 1: Continue payment processing (isolated)
- Session 2: Fix auth bug (isolated, fresh context)
- Each session 40% utilized, clear focus
- Merge bug fix back into main codebase when ready
```

**When to isolate**:
- You're switching to unrelated task
- Both tasks are important (can't abandon first task)
- Context pollution risk is high
- You want clear focus on each task

**Cost**: Lose some project context in Session 2 (have to reload project structure), but much cleaner than pollution.

---

### Strategy 3: Restart (Fresh Session)

**Use when**: Degradation is severe and compression isn't feasible.

**How it works**:

1. **Recognize degradation is too severe** for recovery (extreme contradictions, repeated errors)
2. **Start completely fresh session**
3. **Load only essential context** (project structure, current progress)
4. **Abandon conversation history** (too corrupted to reference)

**When to restart**:
- Compression attempted but still degraded
- AI is generating incorrect code consistently
- Recovery time > benefit time
- Only other option is abandoning task

---

## Exercise 2: Decision Framework Practice

For each scenario below, decide: **Compress, Isolate, or Restart?**

**Scenario 1**: You've been implementing authentication for 2 hours. Context is 80% full. Suggestions are becoming generic but not contradictory. You plan to add OAuth2 in the next 30 minutes to finish the feature.

**Your decision**: _______________
**Reasoning**: _________________

<details>
<summary>Reveal answer</summary>

**Decision**: COMPRESS

**Reasoning**:
- Same task (authentication feature)
- Planning to continue immediately (OAuth2 is part of same feature)
- Context is 80% (compression point)
- Suggestions are degrading but not severely
- Recovery time (5 min) &lt;&lt; benefit time (30+ more minutes of work)
- Checkpoint captures architectural decisions made so far
- New session can continue with clear context

</details>

**Scenario 2**: You're building an API (60 min, 65% context). Your team reports a critical database bug in production that needs immediate investigation. The bug is unrelated to the API feature.

**Your decision**: _______________
**Reasoning**: _________________

<details>
<summary>Reveal answer</summary>

**Decision**: ISOLATE

**Reasoning**:
- Different task (bug fix vs feature development)
- API feature is not urgent (can pause)
- Database bug is isolated problem (separate context)
- Session 1 at 65% can tolerate more work on API
- Session 2 starts fresh for bug investigation
- Prevents context pollution (API decisions don't interfere with bug diagnosis)
- Later: merge bug fix back, return to API feature in Session 1

</details>

**Scenario 3**: You're 100+ minutes into a session. Suggestions are contradictory (AI suggested opposite approaches 30 min apart). Responses are very slow (10+ sec per message). Attempted one compression checkpoint, but degradation continued in new session.

**Your decision**: _______________
**Reasoning**: _________________

<details>
<summary>Reveal answer</summary>

**Decision**: RESTART

**Reasoning**:
- Severe degradation: contradictions, slow responses
- Compression attempted but failed (still degraded after checkpoint)
- Context is too corrupted for recovery (can't trust AI's references)
- Recovery time (compression + restarting) > benefit time
- Only option: start completely fresh
- Load project structure + recent progress only
- Accept loss of conversation history (too corrupted to reference)
- Better to start clean than continue in degraded session

</details>

## Manual Tracking Template

Before moving to Lesson 3 (where you'll use AI to manage context), practice **manual tracking** of context degradation symptoms.

Use this template during your next extended Claude Code session (60+ minutes):

```markdown
# Session Tracking Log

## Session Meta
- **Start time**: [timestamp]
- **Task**: [what you're building]
- **Expected duration**: [estimated time]

## Symptom Timeline

| Time | Symptom Type | Evidence | Action Taken |
|---|---|---|---|
| 00:30 | (none yet) | Suggestions specific, responsive | Continue |
| 01:15 | (none yet) | Still focused and relevant | Continue |
| 01:45 | Repetition | AI repeated database indexing advice | Compress? Isolate? Restart? |
| 02:10 | Forgotten context | AI didn't reference error handling pattern from 00:45 | ↓ |
| 02:25 | Contradictory | Suggested different validation approach than 01:00 | ↓ |
| 02:40 | Slow response | 15+ seconds to generate code suggestion | TRIGGER COMPRESSION |

## Context Utilization Estimate

- Started at: ~5% (fresh session)
- Ended at: ~85% (estimated)
- Degradation threshold crossed at: ~2:10 (about 65-70%)

## Decision Made
- **Compress**: YES
- **Checkpoint created**: YES (500 token summary)
- **New session started**: YES

## Lessons Learned
- [What you learned about your context management]
```

## Try With AI

You now understand degradation symptoms and three mitigation strategies. Validate this through self-check exercises:

**Exercise 1: Symptom Recognition**

Read the session snippet below and identify ALL degradation symptoms present:

```
[01:30] You: We're building a recommendation engine. Load models/recommendation.py
and routes/api.py.

Claude: Loaded. I see you're using collaborative filtering with cosine similarity.
Good approach. I recommend pre-computing similarity matrices to speed up
recommendations. This is the most efficient pattern.

[Later, after 60+ more messages...]

[02:50] You: We need to handle cold start problem—new users with no history.

Claude: For new users, I recommend using random recommendations until you have
enough user history. Also, pre-compute the similarity matrices as I mentioned
before. [RECOMMENDATION 1: Matrix pre-computation repeated]

You: We already pre-computed the matrices. But I'm asking specifically about
the cold start problem.

Claude: Right, for cold start, use random recommendations. But also, you could
compute similarity on-the-fly for every request instead of pre-computing.
[CONTRADICTS EARLIER: Said pre-computation was best, now suggests on-the-fly]

You: We were going to use matrix pre-computation, then switched to on-the-fly?
What's the actual recommendation?

Claude: Both approaches work depending on your use case. Random recommendations
work well for new users...
[Generic response, doesn't address contradiction]
```

**Identify**:
1. Repetitive suggestion (what was repeated?): ___________
2. Contradictory advice (what contradicts?): ___________
3. Forgotten context (what's not referenced?): ___________

<details>
<summary>Reveal answers</summary>

1. **Repetitive**: "Pre-compute similarity matrices" is mentioned at 01:30 and 02:50 with same reasoning both times.

2. **Contradictory**: At 01:30, Claude says pre-computation is "most efficient pattern" and "best approach." At 02:50, suggests on-the-fly computation (opposite). Both are valid strategies, but contradicting within same session shows degradation.

3. **Forgotten context**: Claude doesn't reference the architectural decision made at 01:30. Should say "remember we decided to pre-compute matrices for efficiency?"—instead re-suggests alternatives.

</details>

**Exercise 2: Real Session Tracking**

In your next Claude Code session lasting 60+ minutes:
1. Note timestamp when you first notice ANY degradation symptom
2. Describe the specific symptom
3. Estimate context utilization at that point
4. Decide: compress, isolate, or restart?

Write this down—you'll need this observation skill throughout your career.

**Exercise 3: Self-Check Quiz**

Answer without looking at the lesson:

1. Name the 5 degradation symptoms:
   - ________
   - ________
   - ________
   - ________
   - ________

2. When should you COMPRESS (checkpoint + restart)?
   ________________________________________________________

3. When should you ISOLATE (separate session)?
   ________________________________________________________

4. When should you RESTART (fresh session)?
   ________________________________________________________

<details>
<summary>Reveal answers</summary>

1. **Five symptoms**:
   - Repetitive suggestions
   - Forgotten context
   - Performance degradation (slower responses)
   - Contradictory advice
   - Context reference loss

2. **Compress when**: Continuing SAME TASK, context 75%+ utilized, suggestions degrading but still useful. Saves 5 minutes, gains 30+ more minutes of productive work.

3. **Isolate when**: Switching to DIFFERENT TASK, both important, want to avoid context pollution. Creates parallel sessions for unrelated work.

4. **Restart when**: Severe degradation (contradictions, slow responses), compression failed, can't trust AI's references. Start fresh with essential context only.

</details>

---

## Foundation Complete

You've now learned to:
- ✅ Observe context windows filling
- ✅ Estimate token usage
- ✅ Identify degradation symptoms (5 types)
- ✅ Decide when to compress/isolate/restart
- ✅ Track degradation manually

**Manual foundation is complete.** You can diagnose context issues independently.

In Lesson 3, you'll collaborate with AI on managing context—discovering how AI can suggest strategies, how you can teach AI your constraints, and how you converge together on optimal approaches.

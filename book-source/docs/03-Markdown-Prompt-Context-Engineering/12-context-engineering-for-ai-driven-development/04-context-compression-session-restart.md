---
title: "Context Compression and Session Restart"
description: "Extract session intelligence into checkpoints and restart with clean context while preserving decisions"
sidebar_label: "Compression & Session Restart"
sidebar_position: 4
chapter: 11
lesson: 4
duration_minutes: 45
proficiency: "B1"
concepts: 5

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Context Compression and Checkpoint Creation"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student creates checkpoints under 600 tokens preserving essential session intelligence"

learning_objectives:
  - objective: "Extract essential decisions from sessions into concise checkpoints"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Checkpoint document created from sample session transcript"

  - objective: "Restart sessions with clean context using checkpoint restoration"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Session restart demonstrating preserved intelligence"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (compression, checkpointing, session restart, intelligence preservation, noise elimination) within B1 limit of 7-10 ✓"

differentiation:
  extension_for_advanced: "Research automatic summarization techniques; compare checkpoint compression to RAG (Retrieval-Augmented Generation); design hierarchical checkpoint systems for multi-day projects"
  remedial_for_struggling: "Focus on single compression: Take 10-minute conversation → Identify 3 key decisions → Write 3-sentence summary before learning full checkpoint methodology"
---

# Context Compression and Session Restart

In Lesson 3, you learned to design progressive loading strategies that prevent context degradation through intelligent Foundation/Current/On-Demand allocation. You discovered how to balance preloading efficiency with context waste, adapting strategies to project constraints.

But even with optimal loading, long sessions eventually approach context limits. You might be deep into valuable work—design decisions made, patterns established, progress documented—when you hit 85% utilization.

**The question**: How do you preserve this intelligence while reclaiming context space?

In this lesson, you'll learn **context compression**: extracting key decisions from a full session, consolidating them into a concise checkpoint, and restarting with a clean context window. Through collaborative iteration with AI, you'll discover what's essential to preserve versus what can be safely discarded, creating checkpoints that restore context effectively in under 600 tokens.

## The Context Saturation Problem

Imagine you're 90 minutes into a development session:

```
Current session state (at 85% utilization):
- Foundation loaded: 25K tokens (CLAUDE.md, architecture.md, decisions.md)
- Current files: 50K tokens (3 API files, 2 test files)
- Conversation: 95K tokens (32 exchanges over 90 minutes)
- Total: 170K / 200K = 85% utilization

Work completed:
- Designed authentication flow (password + JWT)
- Decided on bcrypt hashing with 12 rounds
- Implemented rate limiting (5 attempts per hour)
- Discussed refresh token rotation strategy
- Created 3 test cases covering happy paths and errors

Remaining work:
- Implement token blacklist for logout
- Add email verification flow
- Write integration tests
- Document API endpoints

Problem: You're at 85% with 60+ minutes of work remaining.
```

You have three options:

**Option 1: Continue anyway** (risky)
- Context will degrade further
- AI will forget earlier decisions
- Quality drops, errors increase

**Option 2: Restart from scratch** (wasteful)
- Lose all session intelligence
- Re-explain decisions
- Rebuild context

**Option 3: Create checkpoint and restart** (optimal)
- Compress session into summary
- Restart with clean context
- Restore key intelligence in under 600 tokens

This is where **checkpoint compression** becomes essential.

## The Checkpoint Structure

A checkpoint is a compressed summary of your session that preserves critical intelligence while discarding conversational noise.

### What Goes in a Checkpoint

**Section 1: Architectural Decisions Made**
- Key choices that constrain future work
- Why each decision was made (reasoning, not just outcome)
- Alternatives considered and rejected

**Section 2: Progress Summary**
- What was accomplished (features completed, files modified)
- Current working state (what's functional, what's partially done)

**Section 3: Next Steps**
- Immediate tasks for next session
- Blockers or unknowns to address
- Dependencies or prerequisites

**Section 4: Context to Preserve**
- Patterns discovered during session (not obvious from code)
- Constraints that shouldn't be forgotten
- Design principles guiding implementation

**Target size**: under 600 tokens (leaves 199.4K for fresh work)

### What DOESN'T Go in a Checkpoint

**Conversational details** (wasteful):
- "At 10:15 AM, you asked..." → Discard timestamps
- "I suggested X, then you said Y..." → Discard dialogue

**Obvious information** (redundant):
- "We're building an authentication system" → If already in project docs
- "Python uses indentation for blocks" → Common knowledge

**Exploratory dead-ends** (not actionable):
- "We considered OAuth but rejected it" → If not influencing future work
- "Briefly discussed microservices" → If not part of current scope

**Principle**: Checkpoint preserves **what you need to know** to resume effectively, not a complete transcript.

## Creating a Checkpoint: Collaborative Extraction

You're 90 minutes into the authentication session described above. Context is at 85%. Time to compress.

### Requesting Checkpoint Creation

**Your request:**
"We've been working for 90 minutes and we're at 85% context utilization. I need to create a checkpoint so we can restart with clean context. Help me extract the key decisions and progress we've made."

**AI's initial checkpoint draft:**

```markdown
# CHECKPOINT — Authentication Session Ending 2025-11-18 11:30 AM

## Architectural Decisions Made:
1. **Authentication Method**: Password + JWT tokens
   - Why: Balance security with stateless API design
   - Rejected: OAuth (too complex for MVP), session cookies (stateless preference)

2. **Password Security**: Bcrypt hashing, 12 rounds
   - Why: Industry standard, sufficient security for our threat model
   - Considered: Argon2 (better but overkill for current scale)

3. **Rate Limiting**: 5 login attempts per hour per IP
   - Why: Prevent brute force without blocking legitimate retries
   - Implementation: In-memory store (acceptable for single-server MVP)

4. **Token Strategy**: 15-minute access tokens, 7-day refresh tokens with rotation
   - Why: Short-lived access limits exposure, refresh enables seamless UX
   - Rotation: Invalidate old refresh token when issuing new one

5. **Database Schema**: users table with fields: id, email, password_hash, created_at, refresh_token_hash
   - Why: Minimal schema for MVP, can extend later

## Progress Summary:
- Implemented password authentication endpoint (POST /auth/login)
- Implemented JWT generation and validation utilities
- Implemented bcrypt hashing with configuration
- Created rate limiting middleware
- Wrote 3 test cases: valid login, invalid password, rate limit enforcement
- All tests passing

## Next Steps:
1. Implement token blacklist for logout (use Redis or in-memory for MVP)
2. Add email verification flow (separate from core auth)
3. Write integration tests (end-to-end login/logout/refresh)
4. Document API endpoints in OpenAPI spec

## Context to Preserve:
- Rate limiting implementation uses IP-based tracking; may need user-based tracking later if multiple users share IPs (corporate networks)
- Refresh token rotation means old tokens become invalid; clients must handle 401 and re-authenticate
- Password reset flow intentionally deferred to v2 (out of scope for MVP)

## Files Modified:
- api/auth.py: Added login endpoint, JWT utilities
- api/middleware.py: Added rate limiting
- tests/test_auth.py: Added 3 test cases
- models/user.py: Updated schema
```

### Discovering What's Missing

**Your response:**
"This is comprehensive, but it's 720 tokens—too large. Our target is under 600 tokens. What can we compress without losing essential context?"

**AI's analysis:**
"Let's compress by removing redundancy and condensing explanations:

**What we can remove**:
- Rejected alternatives (OAuth, Argon2) — interesting but not actionable
- Implementation details already in code (JWT generation utilities, bcrypt configuration)
- Obvious next steps (write tests → that's standard workflow)
- Timestamp precision (11:30 AM → just date matters)

**What we must keep**:
- Core architectural decisions (Password+JWT, rate limiting strategy)
- Non-obvious constraints (IP-based rate limiting limitation, refresh token rotation behavior)
- Deferred items (password reset to v2)
- Current working state (what's functional)

Let me compress to under 600 tokens."

### Iterating Toward Concise Checkpoint

**AI's compressed checkpoint:**

```markdown
# CHECKPOINT — Auth Session 2025-11-18

## Key Decisions:
1. Auth: Password + JWT (15min access, 7d refresh with rotation)
2. Security: Bcrypt 12 rounds, rate limit 5/hour per IP
3. Schema: users(id, email, password_hash, refresh_token_hash)

## Completed:
- POST /auth/login endpoint functional
- Rate limiting middleware implemented
- 3 tests passing (valid login, invalid pw, rate limit)

## Next:
1. Token blacklist for logout
2. Email verification flow
3. Integration tests
4. API docs

## Critical Context:
- Rate limit is IP-based; may need user-based if shared IP issues arise
- Refresh rotation invalidates old tokens; clients must handle 401
- Password reset deferred to v2 (out of MVP scope)

## Files: api/auth.py, api/middleware.py, tests/test_auth.py, models/user.py
```

**Token count**: 487 tokens (~30% reduction from 720)

### What Emerged from Compression

Through two rounds of iteration, you arrived at a checkpoint that:
- Started with AI's comprehensive draft (preserved everything)
- Identified compression targets through your feedback (what's essential vs redundant)
- Converged on a concise format that balances preservation with efficiency

The final checkpoint captures **what you need to know** to resume work effectively, not every conversational detail.

## Session Restart Workflow

Now that you have a checkpoint, here's how to restart with clean context:

### Step 1: End Current Session

**Current session**: 85% utilization, 170K tokens used, degradation symptoms likely

**Action**: Save checkpoint to CHECKPOINT.md in project root

### Step 2: Start New Session

**New session**: 0% utilization, 200K tokens available, fresh context window

**Initial prompt** (load checkpoint):
```
I'm resuming work on authentication from a previous session. Here's my checkpoint:

[Paste full CHECKPOINT.md content]

I'm ready to continue with the next tasks. What should we tackle first?
```

**Checkpoint loaded**: ~500 tokens (0.25% utilization)

**Context reclaimed**: 170K → 0.5K tokens = 169.5K tokens freed (99.7% recovery)

### Step 3: Verify Context Restoration

**AI's response:**
"Based on your checkpoint, I understand:

- You've implemented password + JWT authentication with bcrypt and rate limiting
- Tests are passing for core login functionality
- Next priorities: token blacklist, email verification, integration tests, docs

Let's start with the token blacklist for logout. Since this is MVP, would you prefer an in-memory implementation or Redis?"

**Validation**: AI understood session state, remembered architectural decisions, ready to proceed.

### When to Create Checkpoints

Use this decision framework:

```
Checkpoint Trigger Decision:

IF utilization > 80% AND session_duration > 60 minutes:
  → CREATE checkpoint NOW (degradation imminent)

ELSE IF utilization > 70% AND remaining_work > 30 minutes:
  → PLAN checkpoint soon (degradation likely before completion)

ELSE IF utilization < 70%:
  → CONTINUE (sufficient capacity remains)

ELSE IF degradation_symptoms_detected (from Lesson 2):
  → CREATE checkpoint regardless of utilization (quality matters more)
```

**Principle**: Create checkpoints **before** degradation becomes severe, not after.

## Practice Exercises

### Exercise 1: Extract Checkpoint from Transcript

You're 2 hours into a session. Here's a summary of the conversation:

```
Session: API Documentation Generator

Discussion points:
- Decided to use OpenAPI spec as input format
- Chose markdown as output format (one file per endpoint)
- Discussed authentication documentation (decided to extract from OpenAPI security schemes)
- Explored folder structure options (settled on grouping by resource: users/, products/, orders/)
- Implemented OpenAPI parser that extracts endpoints, parameters, responses
- Created markdown template with sections: Description, Parameters, Responses, Examples
- Wrote 2 test cases (basic endpoint, endpoint with authentication)
- Discovered edge case: Some OpenAPI specs have incomplete descriptions
- Decided to flag incomplete descriptions rather than generating placeholder text
- Discussed but didn't implement: Code example generation (deferred to v2)
- Current state: Parser works, template renders, 2 tests passing
- Remaining: Handle edge cases, add more test coverage, support query parameters

Files modified: parser.py, template.py, tests/test_parser.py
Context utilization: 88%
```

**Your assignment**:
1. Create a checkpoint under 600 tokens that preserves essential decisions
2. Identify what to KEEP vs what to DISCARD from the above
3. Structure using: Decisions, Progress, Next Steps, Critical Context
4. Validate token count (estimate: ~5 tokens per word)

### Exercise 2: Compress an Oversized Checkpoint

A colleague created this checkpoint (892 tokens):

```markdown
# CHECKPOINT — Database Migration Session

## Background:
We started this session at 9:00 AM. The goal was to migrate our database from SQLite to PostgreSQL because SQLite doesn't handle concurrent writes well and we're expecting increased traffic.

## Decisions Made:
1. **Database Choice**: PostgreSQL instead of MySQL
   - Reasoning: Better JSON support, more robust transaction handling, team familiarity
   - Alternatives: MySQL (considered but team less experienced), MongoDB (wrong fit for relational data), SQLite (current, insufficient for scale)
   - Research: Consulted PostgreSQL docs, Stack Overflow, internal team survey

2. **Migration Strategy**: Blue-green deployment
   - Reasoning: Zero downtime requirement for production
   - Alternatives: Maintenance window (unacceptable to business), gradual rollout (too complex)
   - Timeline: 2-week implementation

3. **Schema Changes**: Added indexes on frequently-queried columns
   - Columns: user_id, created_at, status
   - Reasoning: Query performance optimization based on production logs
   - Expected improvement: 10x faster queries

4. **Connection Pooling**: Using pgbouncer with 20 connections
   - Reasoning: Handles connection overhead efficiently
   - Configuration: Transaction pooling mode

## What We Implemented:
- Migration scripts for schema conversion
- Connection pooling configuration
- Test environment with PostgreSQL
- Updated database connection code in main app
- Modified 5 query functions to use PostgreSQL syntax

## Testing Status:
- Unit tests: 15/15 passing
- Integration tests: 8/10 passing (2 failures related to timestamp timezone handling)

## What's Left:
- Fix 2 failing timezone tests
- Set up production PostgreSQL instance
- Create backup strategy
- Write rollback procedure
- Update deployment documentation
- Train team on PostgreSQL administration
- Monitor performance in staging for 1 week before production

## Edge Cases Discovered:
- Timezone handling differs between SQLite and PostgreSQL
- Some implicit type conversions from SQLite don't work in PostgreSQL
- Need to handle connection pool exhaustion gracefully

## Files Modified:
- db/connection.py
- db/queries/users.py
- db/queries/orders.py
- db/queries/products.py
- tests/test_db.py
- config/database.yml

## Conversation Highlights:
- Discussed transaction isolation levels
- Explored indexing strategies
- Debated connection pool size
- Reviewed PostgreSQL best practices
```

**Your assignment**:
1. Identify redundant content (what's obvious or unnecessary)
2. Compress to under 600 tokens while preserving critical decisions
3. Remove conversation details that don't impact future work
4. Justify each deletion

### Exercise 3: Design Checkpoint for Multi-Session Project

You're working on a complex project that will take 5 sessions to complete:

**Project**: E-commerce platform (authentication, product catalog, shopping cart, checkout, admin dashboard)

**Session 1**: Authentication (completed, checkpoint created)
**Session 2**: Product catalog (in progress, 75% context utilization)

**Your assignment**:
1. What decisions from Session 1 need to be in Session 2's checkpoint?
2. What can be safely omitted from Session 2's checkpoint (already captured in Session 1)?
3. Design a checkpoint strategy that accumulates intelligence across 5 sessions without redundancy

## Try With AI

Now practice creating checkpoints collaboratively with Claude Code.

### Setup
Open Claude Code. Use a long development session (real or simulated) that's approaching 80%+ utilization.

### Prompt Set

**Prompt 1 — Request Checkpoint Extraction:**
```
We've been working for [N] minutes and context utilization is at [X%]. I need to create a checkpoint to preserve our progress before restarting with clean context.

Help me extract:
1. Key architectural decisions we made (with reasoning)
2. What we completed successfully
3. What's left to do
4. Any critical context that shouldn't be forgotten

Target: under 600 tokens while preserving everything essential to resume work.
```

**Prompt 2 — Identify Compression Targets:**
```
This checkpoint draft is [N] tokens—too large. Our target is under 600 tokens.

Review the checkpoint and identify:
- What's redundant (obvious from code or standard practice)?
- What's conversational noise (dialogue, timestamps)?
- What's essential (decisions, non-obvious constraints, current state)?

Help me compress without losing critical context.
```

**Prompt 3 — Validate Compression:**
```
Here's the compressed checkpoint (now [N] tokens):
[Paste compressed checkpoint]

Validate:
- Is anything critical missing that would block work resumption?
- Are decisions preserved with enough context to understand WHY?
- Are next steps clear and actionable?
- Could someone else resume this work from this checkpoint?
```

**Prompt 4 — Test Session Restart:**
```
[Start NEW Claude Code session]

I'm resuming work from a previous session. Here's my checkpoint:
[Paste checkpoint]

What's our current state, and what should we work on next?
```

### Expected Outcomes

Through collaborative checkpoint creation, you should:
- Extract essential decisions from conversational noise
- Compress session intelligence into under 600 tokens
- Discover what's truly critical versus what's redundant
- Successfully restore context in a new session with minimal token cost

**Safety Note**: If restarting with a checkpoint feels like starting from scratch (AI doesn't understand context), your checkpoint was too compressed. Include more detail about decisions and constraints.


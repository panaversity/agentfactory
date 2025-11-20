---
title: "Hands-On Debugging and Optimization"
description: "Apply all context engineering techniques to diagnose and fix real degradation scenarios"
sidebar_label: "Hands-On Debugging & Optimization"
sidebar_position: 8
chapter: 11
lesson: 8
duration_minutes: 90
proficiency: "B1"
concepts: 4

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Context Engineering Integration and Debugging"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student diagnoses context degradation and applies appropriate remediation from Lessons 1-7"

learning_objectives:
  - objective: "Diagnose context degradation from conversation symptoms using Lesson 2 patterns"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Root cause analysis from degraded session transcripts"

  - objective: "Apply accumulated techniques (progressive loading, compression, isolation, memory files, tool selection) to recover degraded sessions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Remediation plan execution with validation metrics"

  - objective: "Optimize context strategy under production constraints"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Context strategy optimization for real development scenarios"

cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (systematic diagnosis, technique integration, validation metrics, optimization under constraints) within B1 limit of 7-10 ✓"

differentiation:
  extension_for_advanced: "Analyze multiple simultaneous degradation patterns; design automated monitoring and remediation systems; create context engineering playbook for team use"
  remedial_for_struggling: "Focus on single scenario: Identify one symptom → Match to one technique → Apply → Validate, before attempting multiple simultaneous issues"
---

# Hands-On Debugging and Optimization

## The Problem You're About to Solve

Over the last 7 lessons, you've learned context engineering techniques:
- Recognize degradation symptoms (Lesson 2)
- Apply progressive loading (Lesson 3)
- Create checkpoints and compress context (Lesson 4)
- Isolate parallel tasks (Lesson 5)
- Build persistent memory files (Lesson 6)
- Select the right tool (Lesson 7)

But **reading about techniques** and **executing them under pressure** are different skills.

This lesson puts you in real failure scenarios. You'll **diagnose problems, apply solutions, measure results**—like you would in actual development work.

This is Layer 2 Validation: **Applying all learned patterns to integrated scenarios**.

## Diagnostic Process: Your Debugging Framework

Before diving into scenarios, understand the **systematic diagnostic workflow** you'll apply:

**Step 1: Observe Symptoms**
- What behaviors signal a problem?
- Which Lesson 2 symptom appears? (Repetitive, forgotten, generic, etc.)
- When did symptoms start? (Check session timeline)

**Step 2: Identify Root Cause**
- Symptom → Root cause mapping (from Lesson 2)
- Which context engineering principle violated?
- Utilization level? Session duration? Task structure?

**Step 3: Select Remediation Strategy**
- Which lesson's technique applies?
- Lesson 3 (loading), 4 (compression), 5 (isolation), 6 (persistence), 7 (tool selection)?
- What's the validation criterion?

**Step 4: Execute and Validate**
- Apply selected strategy
- Measure: Did symptoms disappear?
- Metrics: Utilization, response quality, continuity?

Now you'll apply this to 4 real scenarios.

---

## Scenario 1: High Utilization Crisis (85%, 90-Minute Session)

### The Situation

You're 90 minutes into feature development. Session note shows:

```markdown
# Development Session — 2025-11-18

## Task: Implement Payment Processing

### Context Loaded:
- CLAUDE.md (project patterns)
- architecture.md (system design)
- decisions.md (past decisions)
- payment-routes.md (API structure)
- stripe-integration.md (external service)
- error-handling.md (patterns)
- test-suite.md (testing approach)

### Progress:
- Hour 1: Core payment logic designed
- Hour 1.5: Error handling added
- Hour 2: Edge cases being addressed

### Token Estimation:
- Context loaded: ~30K tokens
- Conversation so far: ~140K tokens
- Total utilization: 170K / 200K = 85%

### Observations:
- AI responses getting shorter
- AI suggestions becoming generic ("use standard patterns")
- AI asking me to re-explain decisions I stated 45 minutes ago
```

### Conversation Transcript (Degradation Evidence)

```
[You, 10 minutes ago]:
"For error handling, we decided to use environment-based config.
Test mode should come from .env, NOT hardcoded."

[Claude, now]:
"Here's error handling approach:
- Use try/catch blocks
- Log errors to console
- Return generic error messages

For external service calls, enable test mode for safety:
test_mode = True  # Prevents accidental production charges"
```

**Problem**: Claude suggested `test_mode = True` (hardcoded) despite your clear decision 10 minutes ago: "test mode from .env, NOT hardcoded."

**This is degradation**. Claude forgot your earlier constraint.

### Diagnosis Exercise

Before reading the solution, **diagnose this yourself**:

1. **What's the symptom?**
   - (Check Lesson 2 degradation checklist)
   - Claude forgot earlier decision
   - Responses are generic
   - Suggestions contradict earlier constraints

2. **Which lessons apply?**
   - Lesson 1: Context window mechanics (85% is high)
   - Lesson 2: Degradation symptom #2 (forgotten patterns)

3. **What's the root cause?**
   - Session at 85% utilization (nearing limit)
   - 90-minute duration (long session)
   - Earlier decisions fading from working memory

4. **What strategy should fix this?**
   - Lesson 4: Context Compression with checkpoint

### Remediation: Create Checkpoint

**Step 1: Extract Key Decisions**

Create CHECKPOINT.md:

```markdown
# Session Checkpoint — 2025-11-18

## Architectural Decisions Made

### Decision 1: Environment-Based Configuration
- Pattern: All external service calls check environment first
- Implementation: test_mode = read from .env file
- Constraint: NEVER hardcode test_mode
- Rationale: Production safety; environment isolation

### Decision 2: Error Handling Strategy
- Pattern: Try/except at system boundaries only
- Implementation: Validate inputs BEFORE processing
- Logging: Log with context (not just message)
- Constraint: No silent failures

### Decision 3: Payment Flow Architecture
- Pattern: Validate → Process → Confirm → Log
- Integration: Stripe API with idempotency keys
- Constraint: All payments must be idempotent

## Progress Summary
- ✅ Core payment logic completed
- ✅ Error handling patterns established
- ✅ Environment-based config implemented
- ⏳ Edge case handling 70% complete

## Next Steps
1. Complete edge case handling (timeout, network failure, invalid input)
2. Add comprehensive logging
3. Write integration tests with Stripe
4. Deploy to staging for validation

## Context to Preserve
- Environment-based config is CRITICAL pattern
- Never hardcode test_mode or API keys
- Always use idempotency keys for external services
```

**Step 2: Restart Session with Checkpoint**

Close current session (85% full). Start new session:

```
## Session Restart

I'm continuing payment feature development.

Here's my project context:

[Load CLAUDE.md]
[Load architecture.md]

And here's my checkpoint from earlier session:

[Paste CHECKPOINT.md]

Now continue: Complete edge case handling, remembering:
- Environment-based config pattern (test_mode from .env)
- Error handling at system boundaries
- Idempotency for all payments
```

**Step 3: Verify Checkpoint Worked**

Claude's next response:

```
Based on your checkpoint, I see:
- Environment-based config is established (test_mode from .env)
- Edge cases to handle: timeout, network failure, invalid input
- Pattern: Validate → Process → Confirm → Log

For edge case handling, following your environment-based pattern:

Timeout handling:
- Read timeout value from .env (DEFAULT_TIMEOUT=30)
- Use configurable retry strategy
- Log timeout events with request context
```

**Checkpoint worked!** Claude recovered the environment-based pattern and applied it correctly.

### Metrics Validation

**Before compression**:
- Utilization: 85% (170K/200K)
- Symptoms: Forgotten decisions, generic responses
- Session: 90 minutes, degrading

**After compression + restart**:
- Utilization: 20% (40K/200K) — checkpoint + context
- Symptoms: None — correct patterns remembered
- Session: Continued seamlessly

**Result**: Reclaimed 65% of context. Recovered correct patterns. Prevented further degradation.

---

## Scenario 2: Context Pollution (Mixed Tasks, Wrong Patterns Applied)

### The Situation

Session note shows mixed tasks:

```markdown
# Development Session — 2025-11-18

## Tasks in Progress:
1. Implement user authentication (JWT, password hashing)
2. Fix critical payment bug (webhook signature verification)

## Context Loaded:
- auth-routes.md (authentication)
- jwt-implementation.md (token handling)
- password-hashing.md (security patterns)
- payment-routes.md (payment processing)
- stripe-webhooks.md (webhook handling)
- error-handling.md (general patterns)

## Token Estimation: ~85K tokens (42% utilization)
```

### Conversation Transcript (Pollution Evidence)

```
[You]:
"Implement webhook signature verification for Stripe payments."

[Claude]:
"For webhook signature verification, here's the approach:

1. Reconstruct signed content: payload + secret
2. Generate HMAC: hash(payload + secret)
3. Compare with signature header

This follows the general pattern from your password hashing approach:
- Construct input
- Hash with secret
- Compare securely"
```

**Problem**: Claude applied **authentication patterns** (general HMAC from password hashing) to **payment domain** (Stripe-specific webhook verification).

Stripe webhooks use completely different format:
- Reconstruct: `timestamp + '.' + json_payload`
- HMAC the reconstructed string
- Compare with signature from header

**This is context pollution**. Mixed authentication and payment context in same session → AI confused which pattern applies where.

### Diagnosis Exercise

1. **What's the symptom?**
   - Wrong pattern applied to payment domain
   - Authentication HMAC used where Stripe-specific format needed
   - Domain boundaries unclear

2. **Which lessons apply?**
   - Lesson 5: Context Isolation and Parallel Tasks
   - Symptom: Pattern cross-contamination

3. **What's the root cause?**
   - Both auth and payment loaded together (85K tokens)
   - AI can't distinguish domain boundaries
   - Similar concepts (HMAC, hashing) but different implementations

4. **What strategy should fix this?**
   - Lesson 5: Context Isolation — separate sessions for unrelated tasks

### Remediation: Isolate Tasks

**Step 1: Complete Authentication Work**

Save authentication session state:

```markdown
# Authentication Session — Complete

## Status: ✅ Ready to Merge
- JWT implementation complete
- Password hashing implemented (Argon2)
- Login/logout routes tested

## Decisions Made:
- JWT with 24-hour expiry
- Argon2 for password hashing
- Refresh tokens in Redis

## Files Modified:
- auth-routes.md
- jwt-implementation.md
- password-hashing.md
```

**Step 2: Start Isolated Payment Session**

Close Session A. Start Session B with **payment context only**:

```markdown
# Payment Processing Session (Isolated)

## Project Context:
[Load CLAUDE.md]
[Load architecture.md]

## Session-Specific Focus:
This session is FOCUSED on payment processing ONLY.

DO NOT apply authentication patterns here.
Authentication uses general HMAC.
Payment webhooks use Stripe-specific format.

## Payment Task:
Implement webhook signature verification for Stripe.

Key constraint: Stripe uses specific format:
1. Extract timestamp from webhook
2. Reconstruct: timestamp + '.' + json_payload
3. HMAC the reconstructed content
4. Compare with signature in header

This is Stripe-specific. NOT general HMAC from authentication.
```

**Step 3: Verify Isolation Worked**

Claude's response:

```
For Stripe webhook verification, following Stripe's specification:

1. Extract timestamp from signature header
2. Reconstruct signed content: "timestamp.payload"
3. Generate expected signature: HMAC(reconstructed_content, webhook_secret)
4. Compare expected vs provided signature (constant-time comparison)

This follows Stripe's documented approach for webhook security.
```

**Isolation worked!** Removing authentication context → Claude applied correct Stripe-specific pattern.

### Metrics Validation

**Before isolation** (mixed session):
- Tasks: 2 (auth + payment)
- Files: 6 (3 auth + 3 payment)
- Utilization: 85K tokens (42%)
- Quality: Incorrect (wrong pattern)

**After isolation** (separated sessions):
- Session A: Auth only, 3 files, 40K tokens, ✅ correct
- Session B: Payment only, 3 files, 35K tokens, ✅ correct
- Quality: Both correct

**Result**: Isolation prevented pattern cross-contamination. Each session focused on correct domain patterns.

---

## Scenario 3: Lost Intelligence (New Session, No Memory Files)

### The Situation

It's Wednesday morning. You're continuing work from Monday.

```markdown
# Development Session — 2025-11-20 (Wednesday)

## Task: Continue reporting dashboard feature

## Context Loaded:
[Empty — just started session]

## Conversation:
You: "Let me continue the reporting dashboard.
What was our caching strategy for report results?"

Claude: "I don't have context about your reporting dashboard project.
What caching approach did you choose?"
```

**Problem**: This is your **second day** of development. Claude should remember Monday's decisions.

But this is a **new session**. Claude has no memory of Monday's work.

Your memory files exist:
- CLAUDE.md (project conventions)
- architecture.md (system design)
- decisions.md (architectural decisions from Monday)

**But you didn't load them.**

**This is persistence failure**: Memory files created (Lesson 6) but not loaded at session start.

### Diagnosis Exercise

1. **What's the problem?**
   - New session, no context from previous work
   - Memory files exist but not used
   - Lost continuity across days

2. **Which lessons apply?**
   - Lesson 6: Memory Files and Persistent Intelligence
   - Symptom: No cross-session persistence

3. **What's the root cause?**
   - Memory files created but persistence strategy not executed
   - Session start didn't load memory files
   - No workflow automation

4. **What strategy should fix this?**
   - Lesson 6: Load memory files at session start

### Remediation: Implement Persistence Strategy

**Step 1: Load Memory Files at Session Start**

Restart session:

```markdown
# Development Session — 2025-11-20 (Wednesday, Restarted)

## Project Context from Memory Files:

### CLAUDE.md (Project Patterns):
```
# Project Conventions

## Caching Strategy
- Use Redis for all report caching
- Cache expiry: 1 hour (balance freshness vs performance)
- Cache keys: report_id + filter_hash

## Naming Conventions
- Variables: camelCase
- Functions: snake_case
- Constants: UPPER_SNAKE_CASE

## Error Handling
- Validate inputs at API boundary
- Log with structured context
- Return user-friendly messages
```

### architecture.md (System Design):
```
# System Architecture

## Components:
- API Layer: FastAPI endpoints
- Business Logic: Report generation engine
- Caching Layer: Redis (1-hour expiry)
- Database: PostgreSQL with read replicas

## Data Flow:
1. Request → API validates
2. Check cache (Redis)
3. If miss → Generate report (database query)
4. Store in cache
5. Return result
```

### decisions.md (Past Decisions):
```
## ADR-005: Report Caching Strategy (Monday)

**Decision**: Cache report results in Redis with 1-hour expiry.

**Context**: Reports are expensive (joins, aggregations).
Users frequently run same report with same filters.

**Rationale**: Redis provides fast caching.
1-hour expiry balances freshness vs reuse.

**Implementation Notes**: Cache keyed by report_id + filter_hash.
```

Now continue: Implement report caching decorator.
```

**Step 2: Claude Recovers Context**

Claude responds with full context:

```
Based on your memory files, I see:

From decisions.md (ADR-005):
- Redis caching with 1-hour expiry
- Cache keys: report_id + filter_hash
- Rationale: Expensive reports, frequent reuse

From architecture.md:
- Caching layer sits between API and database
- Cache-first strategy (check before generating)

Let me continue from Monday's stopping point.
What's the current implementation status?
```

**Persistence worked!** Memory files recovered Monday's context seamlessly.

**Step 3: Update Memory Files After Wednesday's Work**

After Wednesday's session, update decisions.md:

```markdown
## ADR-006: Cache Invalidation Strategy (Wednesday)

**Decision**: Invalidate cached reports when underlying data changes.

**Context**: Reports cached for 1 hour, but data updates may occur
within that window. Stale data is worse than slow reports.

**Implementation** (Wednesday):
- Added event listeners for data changes
- Invalidate cache on: new data insert, update, delete
- Invalidate specific reports (not full cache flush)

**Consequence**: Cache hit rate may decrease, but accuracy guaranteed.
```

### Metrics Validation

**Without persistence strategy** (naive):
- Monday: Full context, feature progress
- Tuesday: No context, restart from zero
- Productivity: 50% (Monday's context lost)

**With persistence strategy**:
- Monday: Build feature, update memory files
- Tuesday: Load memory files, recover Monday's context
- Productivity: 90%+ (seamless continuity)

**Result**: Persistence strategy prevented context loss. Continuous development across multiple days without re-explanation.

---

## Scenario 4: Saturation Problem (High Utilization Before Starting Work)

### The Situation

Session note shows:

```markdown
# Development Session — 2025-11-18

## Task: Implement real-time notifications feature

## Context Loaded (Initial):
- Core models (15 files)
- Database layer (8 files)
- API routes (12 files)
- Tests (10 files)
- **Total**: 45 files, ~78K tokens (39% utilization)

## Problem:
Notification feature requires:
- WebSocket setup (3 files, ~8K tokens)
- Redis integration (4 files, ~10K tokens)
- Event system (5 files, ~12K tokens)
- Templates (8 files, ~10K tokens)
- Tests (6 files, ~12K tokens)

**If I load all 26 files**: 123K tokens (61% utilization)

**But feature implementation will add**:
- Conversation: ~30K tokens
- Iterations: ~20K tokens
- Debugging: ~10K tokens

**Projected final**: 183K tokens (91% utilization) — severe degradation risk!

**Plus**: Two MORE features scheduled this sprint.
```

**Problem**: Loading everything upfront → no budget left for actual work.

### Diagnosis Exercise

1. **What's the problem?**
   - Context budget exhausted before feature started
   - Can't fit notification feature + previous work
   - But notification is critical

2. **Which lessons apply?**
   - Lesson 3: Progressive Loading (load only what's needed)
   - Lesson 4: Compression (checkpoint to free space)

3. **What's the root cause?**
   - Loaded 45 files upfront (no prioritization)
   - No distinction between Foundation vs Current

4. **What strategy should fix this?**
   - Lesson 3: Progressive Loading — Foundation/Current/On-Demand

### Remediation: Apply Progressive Loading

**Step 1: Identify Foundation vs Current**

```markdown
## Progressive Loading Strategy for Notification Feature

### Foundation Phase (Always Loaded):
- Core models (entities, schemas) — 10 essential files
- API setup (FastAPI app, config) — 3 files
- **Total Foundation**: 13 files, ~15K tokens (7% utilization)

### Current Work Phase (Notification Feature):
- WebSocket setup — 3 files, ~8K tokens
- Event system — 5 files, ~12K tokens
- Notification logic — 6 files, ~10K tokens
- Tests — 6 files, ~12K tokens
- **Total Current**: 20 files, ~42K tokens (21% utilization)

### On-Demand (Fetch if Needed):
- Specific route handlers (load only if requested)
- Specific test utilities
- Legacy modules unrelated to notifications

### Running Tally:
Foundation (15K) + Current (42K) = 57K tokens (28% utilization)
Remaining budget: 143K tokens (72%)
```

**Step 2: Start Session with Progressive Loading**

```markdown
# Notification Feature Development

## Foundation Phase (Loaded):
[Load 13 files: core models, API setup]
Total: 15K tokens

## Current Work Phase (Loaded):
[Load 20 files for notification feature]
Total: 42K tokens

## Running Total: 57K tokens (28% utilization)

## Implementation Task:
Build real-time notification feature with loaded context.

If you need files NOT yet loaded, ASK and I'll fetch on-demand.
```

**Step 3: Implement with On-Demand Fetching**

```
[During implementation]:

Claude: "I need to understand user session management to route
notifications correctly. Can you load app/services/session.py?"

You: [Load session.py — 3K tokens]

Running tally: 60K tokens (30% utilization)
```

**Step 4: Validate Budget After Feature Complete**

```markdown
## Final Session State

### Context Used:
- Foundation: 15K tokens
- Current (notification): 42K tokens
- On-demand (session handling): 3K tokens
- Conversation + iterations: 50K tokens
- **Total**: 110K tokens (55% utilization)

### Remaining Budget: 90K tokens (45%)

### Still Room For:
- Two more moderate features
- Bug fixes
- Final optimization
```

### Metrics Validation

**Without progressive loading** (naive approach):
- Would load: 45 existing + 26 new = 71 files, ~123K tokens
- Utilization: 61% BEFORE starting work
- Risk: Degradation before feature complete

**With progressive loading**:
- Load: 13 foundation + 20 current = 33 files, 57K tokens
- Utilization: 28% when starting work
- Final: 110K tokens (55%) after feature complete
- Remaining: 90K tokens for more work

**Result**: Progressive loading kept utilization low. Feature completed with budget to spare.

---

## Integration Exercise: Multi-Strategy 5-Day Sprint

Now combine ALL strategies (Lessons 1-7) for a complex project.

### The Complex Scenario

**Project**: 100 files, 150K lines
**Sprint**: 5 days
**Features**: 3 parallel (authentication, payments, notifications)
**Team**: Multiple developers contributing

### Day 1: Setup and Foundation

**Strategy**: Tool selection (Lesson 7) + Memory files (Lesson 6)

```markdown
## Day 1 Workflow

### Morning: Architecture Understanding (Gemini CLI)
- Use Gemini (2M context) to understand full architecture
- Duration: 30 minutes
- Output: Comprehensive architectural understanding

### Afternoon: Memory Files Creation
- Create CLAUDE.md (patterns discovered)
- Create architecture.md (system structure)
- Create decisions.md (initial decisions)
- Store in project root

### Evening: Authentication Feature (Claude Code)
- Switch to Claude Code (better IDE, deep reasoning)
- Progressive loading: Foundation + auth files only
- Checkpoint at end of day
```

### Day 2: Authentication Continuation

**Strategy**: Persistence (Lesson 6) + Compression (Lesson 4)

```markdown
## Day 2 Workflow

### Morning: Load Memory Files
- Start Claude Code session
- Load CLAUDE.md, architecture.md, decisions.md
- Load Day 1 checkpoint

### Work: Continue Authentication
- Progressive loading: Foundation + auth work from Day 1
- Utilization target: under 70%

### Evening: Checkpoint
- Update decisions.md with new decisions
- Create Day 2 checkpoint
- Status: Authentication 80% complete
```

### Day 3: Payments Feature (Isolated)

**Strategy**: Isolation (Lesson 5) + Progressive Loading (Lesson 3)

```markdown
## Day 3 Workflow

### Morning: New Isolated Session
- Load memory files
- Start NEW session for payments (isolated from auth)
- Progressive loading: Foundation + payment files only

### Rationale for Isolation:
- Similarity score: Auth vs Payment = 40% (under 50% threshold)
- Different domain (security vs transactions)
- Prevent pattern cross-contamination

### Work: Payments Implementation
- Utilization target: under 70%

### Evening: Checkpoint
- Update memory files with payment decisions
- Create Day 3 checkpoint
```

### Day 4: Notifications Feature (Isolated)

**Strategy**: Same as Day 3 (Isolation + Progressive Loading)

### Day 5: Integration and Testing

**Strategy**: All techniques combined

```markdown
## Day 5 Workflow

### Morning: Load All Context
- Memory files (accumulated decisions from Days 1-4)
- Checkpoints from all 3 features
- Integration testing scope

### Work: Integration Testing
- Test authentication + payments integration
- Test authentication + notifications integration
- Test full system flow

### Validation:
- All features work independently (isolation prevented pollution)
- All features work together (memory files maintained continuity)
- Context strategy prevented degradation across 5 days
```

### 5-Day Metrics

**Productivity**:
- Day 1: Architecture understanding + setup (foundation)
- Day 2: Auth 80% complete
- Day 3: Payments 80% complete
- Day 4: Notifications 80% complete
- Day 5: Integration complete, all features tested

**Context Health**:
- No degradation incidents (checkpoints at under 80%)
- No pollution incidents (isolated sessions)
- No persistence failures (memory files loaded daily)

**Result**: 5-day sprint completed with 3 features, zero context-related failures.

---

## Try With AI

Ready to diagnose and fix context problems using integrated strategies from Lessons 1-7?

**🔍 Explore Integrated Diagnosis:**
> "Walk me through diagnosing these 4 scenarios: (1) Session at 85% utilization with AI repeating suggestions, (2) Mixed authentication and payment tasks with wrong patterns applied, (3) New session has no memory of yesterday's work, (4) Loading all files upfront hits 75% before coding starts. For each: identify symptom type (degradation/pollution/persistence/saturation), map to Lesson concepts, recommend remediation strategy."

**🎯 Practice Systematic Remediation:**
> "I'm experiencing [describe actual symptom]. Use the diagnostic framework: (1) Identify symptom from Lesson 2 checklist, (2) Determine root cause, (3) Map to appropriate Lesson strategy, (4) Design remediation with validation checkpoints. Walk me through step-by-step, explaining WHY each step matters."

**🧪 Test Strategy Effectiveness:**
> "Simulate 'before and after' metrics for context compression: Before: 85% utilization, forgotten decisions, 2-hour session. Apply Lesson 4 checkpoint strategy. Calculate: new utilization percentage, budget reclaimed, symptom resolution. Validate whether remediation was effective using quantitative metrics."

**🚀 Design Complete Multi-Day Workflow:**
> "Design a 5-day sprint strategy for [my actual project] with [X] files and 3 parallel features. Orchestrate all techniques: Day 1 (tool selection + memory files), Day 2 (persistence + compression), Days 3-4 (isolation + progressive loading), Day 5 (integration). Create daily workflow checklist with: session start (what to load), work strategy (utilization target), session end (checkpoint protocol), remediation triggers (if degradation appears)."

---

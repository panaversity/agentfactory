---
title: "Lesson 9: Validation, Pitfalls & Best Practices"
sidebar_position: 9
description: "Learn common mistakes, validation strategies, metrics, and best practices for context engineering mastery"
keywords: [context engineering, common mistakes, validation, metrics, best practices, AIDD, real-world examples]
---

# Validation, Pitfalls & Best Practices

## Introduction: Learning from Common Mistakes

You've learned all the core strategies:
- ✓ Progressive loading (Lesson 3)
- ✓ Context compression (Lesson 4)
- ✓ Context isolation (Lesson 4)
- ✓ Context-first workflow (Lesson 5)

But knowing the strategies isn't enough. **Most beginners make the same mistakes** when applying these strategies—mistakes that waste time and produce poor results.

This final lesson teaches you:
1. **What mistakes to avoid** (so you don't waste time learning the hard way)
2. **How to validate** your context practices (so you know you're doing it right)
3. **How to build good habits** (so context engineering becomes natural)

Let's learn from others' mistakes instead of making them yourself!

---

## Mistake #1: Loading All Files Upfront

### The Mistake

**What it looks like:**

```bash
# Wrong approach - loading everything at once
claude-code prompt "Read all files in the project and tell me how everything works"

# or

claude-code prompt "Read:
- src/auth/authentication.py
- src/auth/authorization.py
- src/auth/permissions.py
- src/services/user_service.py
- src/services/email_service.py
- src/services/notification_service.py
- src/models/user.py
- src/models/profile.py
- src/models/session.py
- [... 20 more files ...]

Now help me add a password reset feature."
```

**What happens:**
- Context window: 85% full immediately
- AI overwhelmed with information
- Can't distinguish what's relevant
- Slow responses
- Generic, unfocused answers

---

### Why This Is Wrong

**The problem:** You're treating context like a dump truck—load everything in and hope AI figures it out.

**But context windows are not:**
- ❌ Storage (they're working memory)
- ❌ Bigger is better (more information ≠ better understanding)
- ❌ Permanent (they degrade)

**The fallacy:** "If AI has all the code, it will give better answers."

**The reality:** Too much context makes AI **less** effective, not more effective.

**The analogy:**

Imagine trying to find a recipe in a cookbook:

**Wrong way:** Read the entire cookbook cover-to-cover before cooking  
**Right way:** Look up the specific recipe you need

Loading everything upfront is like reading the whole cookbook when you just want to make pancakes.

---

### How to Avoid This Mistake

**✅ Use progressive loading (Lesson 3):**

```bash
# Phase 1: Overview only
claude-code prompt "Analyze the directory structure and tell me where authentication code is located. Don't read files yet."

# Phase 2: Specific files
claude-code prompt "Now read just these 3 files:
- src/auth/authentication.py
- src/services/token_service.py
- src/models/user.py

Summarize the authentication approach."

# Phase 3: Deep dive on relevant parts
claude-code prompt "I need to understand token validation. Read the validate_token method and explain it."
```

**Benefits:**
- Context usage: 20% (not 85%)
- AI focused on relevant information
- Fast, specific responses
- Room to explore more as needed

**Key principle:** Load context **as needed**, not all at once.

---

## Mistake #2: Never Restarting Sessions

### The Mistake

**What it looks like:**

```bash
# Monday morning - start session
claude-code prompt "Let's build feature A"
# ... work for 2 hours ...

# Monday afternoon - same session
claude-code prompt "Now let's build feature B"  
# ... work for 2 hours ...

# Tuesday - same session continuing
claude-code prompt "Let's add feature C"
# ... work for 2 hours ...

# Wednesday - session still going
claude-code prompt "Why isn't this working?"
# AI confused, giving inconsistent answers
```

**What happens:**
- Context: 95% full, severely degraded
- AI mixing information from Monday and Wednesday
- Responses slow and unreliable
- Errors and confusion increasing
- Frustration mounting

---

### Why This Is Wrong

**The problem:** You're trying to run a marathon in a single sprint.

**Context windows aren't designed for infinite sessions:**
- They have **size limits** (fill up)
- They suffer **context rot** (quality degrades over time)
- They accumulate **noise** (dead ends, mistakes, corrections)

**The fallacy:** "If I restart, I lose all my progress."

**The reality:** You've already lost progress—the degraded context is making everything harder. Restarting with a checkpoint **recovers** progress.

**The analogy:**

**Session like a whiteboard:**

**First hour:** Clean whiteboard, clear thinking  
**After 3 hours:** Whiteboard covered with notes, hard to find space, messy  
**After 6 hours:** Can't read anything, erasing and rewriting on top of old stuff

**Solution:** Clean the whiteboard and write a fresh summary.

---

### How to Avoid This Mistake

**✅ Use context compression (Lesson 4):**

**Compress every 10-15 interactions or 90 minutes:**

```bash
# After 90 minutes working on feature A
claude-code prompt "Create a checkpoint summary of what we've built. Include:
- Features completed
- Decisions made
- Code patterns established
- Current status and next steps

Save as SESSION_CHECKPOINT.md"

# Start fresh session
# [New terminal or explicit break]

# Resume with checkpoint
claude-code prompt "Read SESSION_CHECKPOINT.md. Let's continue with feature B based on what we've built."
```

**Signs you should restart:**
- Every 90-120 minutes of work
- After completing major milestone
- When you notice AI confusion
- Before switching to significantly different task
- At end of work day (to resume tomorrow)

**Key principle:** Proactive compression beats reactive debugging.

---

## Mistake #3: Assuming AI Remembers Everything

### The Mistake

**What it looks like:**

```bash
# Early in session
claude-code prompt "We're using JWT tokens with 15-minute expiry"
# ... AI acknowledges ...

# 20 messages later
claude-code prompt "Add refresh token endpoint"

# AI responds with:
"Sure! I'll use session-based authentication..."

# You: "Wait, what?! We're using JWT!"
```

**Or:**

```bash
# You reference earlier decision
claude-code prompt "Use the authentication pattern we discussed"

# AI: "I don't recall discussing an authentication pattern. Could you remind me?"

# You: "We just talked about this 10 messages ago!"
```

**What happens:**
- AI "forgets" earlier decisions
- Gives inconsistent recommendations
- You waste time re-explaining things
- Frustration grows

---

### Why This Is Wrong

**The problem:** You're confusing AI context with human long-term memory.

**AI context is:**
- ✓ Working memory (short-term)
- ✓ Everything in current session
- ✓ Degrades over time

**AI context is NOT:**
- ❌ Long-term memory (doesn't persist between sessions)
- ❌ Perfect recall (older info degrades)
- ❌ Guaranteed to remember everything

**The fallacy:** "If I said it once, AI remembers forever."

**The reality:** Context degrades. Important decisions fade. Older messages have less influence.

**The analogy:**

**Human conversation:**
- Start of conversation: You remember everything clearly
- After 2 hours: Details from beginning getting fuzzy
- After 4 hours: Forgot exact wording of earlier points

**AI context works the same way—recent information is clearer than old information.**

---

### How to Avoid This Mistake

**✅ Document critical decisions explicitly:**

**Option A: Create decision log in project**

```bash
# After making key decisions
claude-code prompt "Create DECISIONS.md documenting:
1. Authentication: JWT with 15-min access, 7-day refresh
2. Token storage: Redis blacklist
3. Error handling: Custom exceptions with codes
4. [other decisions...]"

# Later, when needed
claude-code prompt "Read DECISIONS.md. Now implement refresh token endpoint following these decisions."
```

**Option B: Re-state critical context when needed**

```bash
# When resuming or after long discussion
claude-code prompt "Reminder of key decisions:
- JWT authentication (not sessions)
- 15-minute access tokens
- Redis for token blacklist

Now implement refresh endpoint with these constraints."
```

**Option C: Use checkpoints (Lesson 4)**

SESSION_CHECKPOINT.md captures decisions automatically.

**Key principle:** Critical decisions should be **documented**, not just **mentioned**.

---

## Mistake #4: Not Documenting Context Decisions

### The Mistake

**What it looks like:**

**Day 1:**
```bash
claude-code prompt "Should we use JWT or sessions?"
AI: "JWT is better for your use case because..."
You: "Great, let's use JWT"
# ... build feature ...
```

**Day 7 (new session):**
```bash
claude-code prompt "Add login endpoint"
AI: "I'll use session-based authentication"
You: "Wait... why not JWT?"
AI: "I don't have context on previous decisions. Both are valid."
You: "Ugh, we decided this last week!"
```

**What happens:**
- Re-making decisions already made
- Inconsistencies across features
- Wasted time re-explaining rationale
- Team confusion (if multiple developers)

---

### Why This Is Wrong

**The problem:** Context exists only **during** a session. When you start a fresh session, **all context is gone** unless explicitly loaded.

**This means:**
- Decisions don't persist automatically
- Rationale is lost
- Future sessions start from zero

**The fallacy:** "I'll remember why we made that decision."

**The reality:** You won't. And AI definitely won't.

**Future you (or your teammates) will ask:**
- Why did we choose JWT?
- Why Redis and not database?
- Why 15 minutes and not 60?
- What alternatives did we consider?

**Without documentation, these answers are lost.**

---

### How to Avoid This Mistake

**✅ Create architectural decision records:**

**DECISIONS.md (or ADRs for larger projects)**

```markdown
# Architecture Decisions

## Authentication Approach

**Decision:** JWT tokens (not sessions)

**Date:** 2025-01-15

**Context:**
- Building API for mobile and web clients
- Need stateless authentication for scalability
- Planning microservices architecture in future

**Considered Alternatives:**
1. Session-based auth
   - Pro: Easier to revoke
   - Con: Requires sticky sessions / shared session store
   - Con: Doesn't scale well for microservices

2. OAuth 2.0
   - Pro: Industry standard
   - Con: Overkill for our current needs
   - Con: Complex to implement

**Decision:** JWT with access + refresh tokens

**Rationale:**
- Stateless (scales horizontally)
- Works across multiple services
- Can implement revocation with blacklist
- Industry standard for APIs

**Consequences:**
- Must implement token refresh flow
- Need Redis for token blacklist
- Access tokens short-lived (15 min)
- Refresh tokens long-lived (7 days)

---

## Token Storage

**Decision:** Redis for token blacklist

**Rationale:** Fast lookups, built-in expiry, already using for caching

[... more decisions ...]
```

**Benefits:**
- Future sessions can load this context
- Teammates understand decisions
- Rationale preserved
- Can revisit if needs change

**Key principle:** **Architecture decisions are context**—document them like code.

---

## Mistake #5: Mixing Unrelated Contexts

### The Mistake

**What it looks like:**

```bash
# Working on authentication feature
claude-code prompt "Build JWT authentication"
# ... discussion about auth ...

# Suddenly switch topics
claude-code prompt "Also, help me fix this CSS bug where the button is misaligned"
# ... CSS discussion ...

# Then back to auth
claude-code prompt "Now add password reset to the auth system"

# AI responds mixing auth and CSS concepts
AI: "For password reset styling, we should use JWT tokens in the CSS class names..."
# [Confused response - mixing contexts]
```

**What happens:**
- AI confuses which topic you're discussing
- Responses mix concepts from different domains
- Lower quality answers for everything
- Hard to follow conversation history
- Context fills up faster

---

### Why This Is Wrong

**The problem:** You're asking one "brain" to think about completely unrelated topics simultaneously.

**Context windows are like focus:**
- When focused on one topic → Deep, quality thinking
- When jumping between topics → Shallow, confused thinking

**The fallacy:** "AI is powerful enough to handle multiple topics."

**The reality:** AI (like humans) works better with focused context.

**The analogy:**

**Multi-tasking:**
- Trying to write a report while debugging code while designing a logo
- Each task gets partial attention
- Quality suffers on all tasks
- Switching has overhead

**Better:**
- Write report (fully focused)
- Then debug code (fully focused)
- Then design logo (fully focused)

**Each task gets full attention and better results.**

---

### How to Avoid This Mistake

**✅ Use context isolation (Lesson 4):**

**Separate sessions for separate concerns:**

```bash
# Terminal 1: Authentication feature
claude-code prompt "Working on authentication module. Building JWT token system."
# ... stay focused on auth ...

# Terminal 2: CSS bug
claude-code prompt "Working on CSS styling bug. Button alignment issue on profile page."
# ... stay focused on styling ...

# Terminal 3: Database optimization
claude-code prompt "Working on database query optimization. Slow queries in analytics dashboard."
# ... stay focused on database ...
```

**Benefits:**
- Each session has clear, focused context
- AI provides relevant, on-topic responses
- Easy to resume any specific task
- Conversation history makes sense

**Rule of thumb:** If two tasks are unrelated, use separate contexts.

**Key principle:** One context, one concern.

---

## Validation Strategy: The Context Health Check

Now that you know what **not** to do, how do you validate that you're doing it **right**?

Use this checklist regularly:

### The 5-Point Context Health Check

Run this check **before starting work** and **every 30 minutes** during work:

#### 1. Context Fill Level

**Question:** How full is my context?

**Check:**
```bash
claude-code prompt "Estimate how much of the context window is currently in use as a percentage."
```

**Healthy:** 20-40%  
**Warning:** 50-70%  
**Critical:** 70%+

**Action if critical:** Compress and restart (Mistake #2 prevention)

---

#### 2. Context Relevance

**Question:** Is everything in context relevant to current task?

**Check:** Mentally review last 5-10 messages. Are they all about the same topic?

**Healthy:** All messages related to current task  
**Warning:** 1-2 unrelated messages  
**Critical:** Multiple unrelated topics mixed

**Action if critical:** Isolate context (Mistake #5 prevention)

---

#### 3. Context Recency

**Question:** How long has this session been running?

**Check:** Look at timestamp of first message.

**Healthy:** < 60 minutes  
**Warning:** 60-120 minutes  
**Critical:** 120+ minutes

**Action if critical:** Compress and restart (Mistake #2 prevention)

---

#### 4. Context Clarity

**Question:** Can AI recall key decisions?

**Check:**
```bash
claude-code prompt "What are the 3 most important decisions or patterns we've established so far in this session?"
```

**Healthy:** AI accurately recalls key points  
**Warning:** AI recalls but vaguely  
**Critical:** AI can't recall or gets it wrong

**Action if critical:** Document decisions (Mistake #3, #4 prevention)

---

#### 5. Context Loading Strategy

**Question:** Did I load context progressively?

**Check:** Review your first prompts. Did you:
- ✓ Start with overview?
- ✓ Load specific files only?
- ✓ Load more as needed?

**Healthy:** Followed progressive loading  
**Warning:** Loaded more than needed  
**Critical:** Loaded everything upfront

**Action if critical:** Restart with progressive approach (Mistake #1 prevention)

---

### The Weekly Context Review

**Every week, reflect on your context practices:**

```markdown
## Weekly Context Engineering Review

### What Worked Well This Week?
- [ ] Using progressive loading consistently
- [ ] Compressing sessions proactively
- [ ] Documenting key decisions
- [ ] Maintaining focused contexts

### What Mistakes Did I Make?
- [ ] Loaded too much context upfront (Mistake #1)
- [ ] Sessions ran too long without compression (Mistake #2)
- [ ] Assumed AI remembered everything (Mistake #3)
- [ ] Didn't document decisions (Mistake #4)
- [ ] Mixed unrelated contexts (Mistake #5)

### What Will I Improve Next Week?
[Specific action items...]
```

**This reflection builds good habits over time.**

---

## Building Good Context Habits

**Good context engineering isn't about perfection—it's about consistent habits.**

### Habit 1: Start Every Session Right

**Template for starting any development session:**

```bash
# 1. Set clear focus
claude-code prompt "Today's task: [specific goal]. This session will focus only on this task."

# 2. Load context progressively
claude-code prompt "Before we start, give me high-level overview of [relevant area] without reading all files."

# 3. Validate understanding
claude-code prompt "Based on that overview, what files should we read to understand [specific aspect]?"

# 4. Set checkpoint reminder
# (Note to self: Compress after 90 minutes or 15 interactions)
```

**This template prevents Mistakes #1 and #5.**

---

### Habit 2: Checkpoint Regularly

**Every 90 minutes or 15 interactions:**

```bash
# Create checkpoint
claude-code prompt "Checkpoint time. Summarize:
- What we've built
- Decisions made
- Current status
- Next steps

Save as SESSION_CHECKPOINT_[DATE].md"

# Start fresh session
[New terminal or break]

# Resume with checkpoint
claude-code prompt "Read SESSION_CHECKPOINT_[DATE].md. Let's continue."
```

**This template prevents Mistake #2.**

---

### Habit 3: Document as You Go

**After making any architectural decision:**

```bash
claude-code prompt "We just decided to use [X] instead of [Y]. Add this to DECISIONS.md:

Decision: [X]
Rationale: [Why?]
Alternatives considered: [Y, Z]
Trade-offs: [Pros/Cons]
Date: [Today]"
```

**This template prevents Mistakes #3 and #4.**

---

## Try With AI

**Tool:** ChatGPT Web

Practice recognizing and correcting mistakes.

### Prompt 1: Recognize Mistakes
Copy this into ChatGPT:

```
Here's how a developer started their session:

"Read all 47 files in my project and then help me add a small bug fix to one function."

What mistakes is this developer making? Reference the 5 common mistakes from context engineering:
1. Loading all files upfront
2. Never restarting sessions
3. Assuming AI remembers everything
4. Not documenting decisions
5. Mixing unrelated contexts

Which mistakes apply here? How should they have approached this instead?
```

**What to expect:**
- Identification of Mistake #1 (loading everything)
- Explanation of why this approach is inefficient
- Better approach: progressive loading to find the bug
- Estimate of context waste (90%+ filled for a small bug fix)

**Practice:** Can you spot context engineering mistakes in your own workflow?

---

### Prompt 2: Practice Context Health Check
Copy this into ChatGPT:

```
I've been working in a coding session for 3 hours. I've built 4 different features. I haven't restarted the session. The AI is starting to give inconsistent answers and mixing up which feature I'm working on.

Run the 5-point context health check on this scenario:
1. Context fill level
2. Context relevance
3. Context recency
4. Context clarity
5. Context loading strategy

For each point, assess: Healthy / Warning / Critical
Then tell me what actions I should take immediately.
```

**What to expect:**
- Health check assessment (likely all Critical)
- Identification of multiple mistakes (#2, #5, possibly others)
- Action plan: compress, isolate, restart
- Explanation of why current approach is failing

**Reflection:** This is what good validation looks like—catching problems before they cause major issues.

---

### Prompt 3: Create Your Personal Context Checklist
Copy this into ChatGPT:

```
Based on the 5 common mistakes and the context health check, create a personalized checklist I can use every time I start a development session.

Include:
- Pre-session setup (before I start)
- During session (what to check regularly)
- End session (how to wrap up properly)

Make it practical and concise—something I can realistically follow every day.
```

**What to expect:**
- Customized checklist template
- Practical, actionable items
- Integration of all 5 mistake prevention strategies
- Realistic timing (not overwhelming)

**Action:** Save this checklist! Use it for the next week and refine it based on what works for you.

---

### Safety Note

**Why practicing mistake recognition matters:**

Even with a small tool like ChatGPT:
- You'll make these mistakes (everyone does!)
- Recognizing them quickly is the key skill
- The faster you catch mistakes, the less time wasted

**These validation habits transfer to any AI tool:**
- Claude Code (larger context, but same principles)
- Gemini CLI (named sessions, but same mistakes possible)
- Any future AI coding tool

**Building good habits now saves hours of frustration later.**

---

## Chapter Summary: Context Engineering Mastery

Congratulations! You've completed all 6 lessons on context engineering. Let's review what you've mastered:

### Lesson 1: What Is Context Engineering?
- ✅ Context is "everything AI knows right now"
- ✅ Context engineering is deliberate management of that information
- ✅ Karpathy Principle: "Prompt is program, context is RAM, AI is CPU"

### Lesson 2: Understanding Context Windows
- ✅ Context windows are limited working memory
- ✅ Context rot happens as windows fill up
- ✅ Recognition of degradation signs (slower, inconsistent, repetitive)

### Lesson 3: Progressive Context Loading
- ✅ Three-phase approach: Overview → Focus → Deep Dive
- ✅ Load only what's needed, when it's needed
- ✅ Context budget management (stay under 40%)

### Lesson 4: Context Compression & Isolation
- ✅ Compression: Summarize and restart with checkpoint
- ✅ Isolation: Separate sessions for separate concerns
- ✅ Strategy selection: When to use which technique

### Lesson 5: Context Enables Better Specifications
- ✅ Poor context → bad specs → broken code
- ✅ Rich context → clear specs → working code
- ✅ Context-first workflow before specification writing

### Lesson 6: Common Mistakes & Validation (This Lesson)
- ✅ 5 mistakes to avoid (loading all, never restarting, assuming memory, not documenting, mixing contexts)
- ✅ 5-point context health check
- ✅ Habits for consistent good practice

---

## Your Context Engineering Skillset

You now have **complete beginner-to-intermediate context engineering skills**:

**Foundation (A1):**
- ✓ Understand what context is
- ✓ Recognize context window limitations
- ✓ Identify context rot symptoms

**Application (A2/B1):**
- ✓ Apply progressive loading strategically
- ✓ Use compression to maintain long sessions
- ✓ Isolate contexts for focused work
- ✓ Follow context-first workflow before specifications

**Analysis & Evaluation (B1):**
- ✓ Diagnose context problems
- ✓ Choose appropriate strategy for situation
- ✓ Validate context health proactively
- ✓ Recognize and correct mistakes quickly

**These are professional-grade skills.** Experienced AI-native developers use these exact techniques every day.

---

## What's Next?

**In Part 4 (Terminal & Development Environment):**
You'll apply these context engineering skills while learning terminal commands, because:
- Loading project context requires understanding file structure
- Context isolation uses terminal sessions
- Compression checkpoints are saved as files

**In Part 5 (Specification-Driven Development):**
You'll use Lesson 5's context-first workflow intensively:
- Every specification starts with context loading
- Context engineering enables clear requirements
- Context compression preserves specification decisions

**In Part 10+ (Production Deployment):**
Context engineering becomes critical for:
- Complex infrastructure specifications
- Multi-service architecture context management
- Debugging production systems with rich context

**Context engineering is the foundation for everything else you'll learn in this book.**

---

## Measuring Context Engineering Effectiveness

Now you know the strategies and mistakes. But **how do you know if you're doing it right?**

**Use these 5 metrics to measure your context engineering skills.**

---

### Metric 1: First-Attempt Accuracy

**What it measures:** How often AI generates correct code/solution on first try (without multiple iterations).

**How to measure:**
- Track your last 10 AI development sessions
- Count: How many times did AI produce working code on first attempt?
- Formula: (Correct First-Attempts / Total Attempts) × 100

**Targets:**
- **Beginner:** 40-50% (acceptable, still learning)
- **Intermediate:** 60-70% (good context management)
- **Advanced:** 75-85% (professional-level context engineering)

**Example Tracking:**

```markdown
## My First-Attempt Accuracy Log

| Session | Task | First Attempt? | Notes |
|---------|------|---------------|-------|
| 1 | User auth | ✅ Yes | Good context loading |
| 2 | Password reset | ❌ No (3 tries) | Forgot to load email templates |
| 3 | JWT refresh | ✅ Yes | Loaded token handler |
| 4 | Logout endpoint | ✅ Yes | Good session context |
| 5 | User profile | ❌ No (2 tries) | Missing schema context |

**Current Score:** 3/5 = 60% (Good!)
```

**How to improve:**
- **Below 50%:** Review progressive loading (Lesson 4)
- **50-65%:** Practice context curation (Lesson 6, Strategy 4)
- **65%+:** Refine advanced strategies (Lesson 6)

---

### Metric 2: Context Relevance Score

**What it measures:** What percentage of loaded context was actually used in the solution?

**How to measure:**
1. After AI completes a task, ask: "Which files you read were most relevant to solving this?"
2. Count: (Relevant Files / Total Files Loaded) × 100

**Targets:**
- **Poor:** <50% (too much unnecessary context)
- **Good:** 60-75% (reasonable targeting)
- **Excellent:** 80%+ (precise context curation)

**Example Check:**

```bash
# After task completion
claude-code prompt "You just implemented the password reset feature.

Of the files you read, which were:
1. Essential (couldn't complete without)
2. Helpful (provided useful context)
3. Unused (didn't reference)

List each file in one of these categories."
```

**AI response:**
```
Essential (5 files):
- src/auth/password_reset.py (main implementation)
- src/services/email_service.py (send reset email)
- src/models/user.py (user lookup)
- src/schemas/auth_schema.py (request validation)
- tests/test_auth.py (test patterns)

Helpful (2 files):
- src/auth/authentication.py (auth patterns reference)
- src/utils/token_generator.py (token generation pattern)

Unused (3 files):
- src/auth/authorization.py (not needed for this task)
- src/auth/permissions.py (not needed for this task)
- src/models/profile.py (not relevant)
```

**Score:** 7 essential+helpful / 10 total = 70% (Good!)

**How to improve:**
- **Below 50%:** You're loading too much; use explicit file lists (Lesson 6, Strategy 4)
- **50-70%:** Review context curation techniques
- **70%+:** Excellent targeting; maintain this practice

---

### Metric 3: Session Productivity

**What it measures:** How many features/tasks completed per AI session before needing context refresh.

**How to measure:**
- Track completed tasks before context degrades
- Count restarts due to context issues

**Targets:**
- **Beginner:** 1-2 features per session (frequent restarts)
- **Intermediate:** 3-5 features per session
- **Advanced:** 5-8 features per session (with checkpointing)

**Example Session Log:**

```markdown
## Session Productivity - Week of Jan 15

**Session 1: User Management** (45 minutes)
- ✅ User registration
- ✅ Email verification
- ✅ Password reset
- Context refresh needed (Context rot detected)

**Tasks completed:** 3 features
**Session quality:** Good (intermediate level)

**Session 2: API Endpoints** (1 hour)
- ✅ GET /users endpoint
- ✅ POST /users endpoint
- ✅ PATCH /users/:id endpoint
- ✅ DELETE /users/:id endpoint
- ✅ GET /users/:id/posts endpoint
- No context refresh needed (used checkpointing)

**Tasks completed:** 5 features
**Session quality:** Excellent (advanced level)
```

**How to improve:**
- **Below 3 features:** Practice progressive loading and context compression (Lessons 4-5)
- **3-5 features:** Add checkpointing between features
- **5+ features:** You're doing great; document your process

---

### Metric 4: Consistency Across Sessions

**What it measures:** How well architectural decisions persist across multiple sessions (no contradictions or reinvention).

**How to measure:**
- After 3-5 sessions, ask AI to review for consistency
- Count contradictions or pattern breaks

**Targets:**
- **Poor:** 5+ inconsistencies (no memory files)
- **Good:** 2-3 inconsistencies (some memory files)
- **Excellent:** 0-1 inconsistencies (comprehensive memory files + checkpointing)

**Example Consistency Check:**

```bash
# After 5 sessions building blog API
gemini chat --session audit "Read all code we've written in src/api/ (20 files created this week).

Check for inconsistencies:
1. **Authentication patterns:** Do all endpoints handle auth the same way?
2. **Error handling:** Do all routes use same error format?
3. **Validation:** Do all endpoints validate consistently?
4. **Response format:** Do all endpoints return same structure?
5. **Service patterns:** Do all services follow same dependency injection?

List any inconsistencies found."
```

**AI response:**
```
Inconsistencies Found: 2

1. **Error Handling:** 
   - Files 1-10: Return {error: "message"}
   - Files 11-20: Return {status: "error", message: "..."}
   → Should standardize on one format

2. **Authentication:**
   - Most endpoints use @require_auth decorator
   - POST /posts/bulk uses manual token checking
   → Should use decorator consistently
```

**Score:** 2 inconsistencies = Good (but needs refinement)

**How to improve:**
- **5+ inconsistencies:** Create PATTERNS.md and DECISIONS.md (Lesson 6, Strategy 5)
- **2-4 inconsistencies:** Review memory files before each session
- **0-1 inconsistencies:** Excellent; you're using structured note-taking effectively

---

### Metric 5: Debug Efficiency

**What it measures:** How often you can debug issues without having to load additional context.

**How to measure:**
- Track debugging sessions
- Count how many required additional context loading

**Targets:**
- **Beginner:** 50% (half of debug sessions need more context)
- **Intermediate:** 70% (most debugs work with initial context)
- **Advanced:** 80-90% (rare context additions needed)

**Example Debug Log:**

```markdown
## Debug Efficiency - Week of Jan 15

| Bug | Initial Context Sufficient? | Additional Context Needed? |
|-----|---------------------------|---------------------------|
| JWT expiry issue | ✅ Yes | None (had jwt_handler.py loaded) |
| Email not sending | ❌ No | Needed email_service.py config |
| User not found error | ✅ Yes | Had user_service.py + repository |
| Password validation | ✅ Yes | Had validators.py |
| Login rate limiting | ❌ No | Needed rate_limit_middleware.py |

**Score:** 3 sufficient / 5 bugs = 60% (Intermediate level)
```

**How to improve:**
- **Below 60%:** Load related files proactively (e.g., when loading service, load its dependencies)
- **60-80%:** Practice layered file access (Lesson 6, Strategy 4, Technique 3)
- **80%+:** Excellent context anticipation; maintain this

---

## AIDD-Specific Context Engineering Pitfalls

Beyond the 5 common mistakes, here are **pitfalls specific to AI-Driven Development** workflows.

### Pitfall 1: Context Overload at Session Start

**What it looks like:**
```bash
# Session 1: Feature planning
claude-code prompt "Read:
- ARCHITECTURE.md
- All 30 API route files
- All 15 service files
- All 20 model files
- All tests

Plan the new notifications feature."
```

**The problem:** Planning requires **high-level context** (architecture, patterns), not **implementation details** (every single file).

**The fix:**
```bash
# Better approach
claude-code prompt "Read only:
- ARCHITECTURE.md (system overview)
- DECISIONS.md (past decisions)
- PATTERNS.md (code patterns)

Based on these, plan the notifications feature architecture at a high level."

# Then in separate implementation sessions, load specific files
```

**Key principle:** Match context depth to task abstraction level.

---

### Pitfall 2: Losing Context Between Sessions

**What it looks like:**

**Day 1:**
```bash
claude-code prompt "Implement user authentication with JWT tokens, 15-minute expiry, refresh tokens with 7-day expiry."
# AI implements it
```

**Day 2 (new session):**
```bash
claude-code prompt "Implement logout endpoint."
# AI asks: "What authentication system are you using? How are tokens structured?"
```

**The problem:** AI has no memory of yesterday's decisions.

**The fix:** Use memory files
```bash
# Day 1: Document decision
claude-code prompt "Create/update DECISIONS.md:

## Decision: JWT Authentication
- Access tokens: 15-minute expiry
- Refresh tokens: 7-day expiry
- Storage: Redis for revocation support"

# Day 2: Load decisions
claude-code prompt "Read DECISIONS.md, then implement logout endpoint following those token decisions."
```

---

### Pitfall 3: Mixing Contexts Without Boundaries

**What it looks like:**

```bash
# Single session trying to do everything
claude-code prompt "Read the entire codebase.

Now:
1. Design new payment system architecture
2. Implement PaymentService
3. Create tests
4. Write API documentation
5. Design frontend components"
```

**The problem:**
- Backend and frontend contexts mixed
- Architecture and implementation mixed
- Strategic and tactical concerns mixed
- Context window overloaded with conflicting concerns

**The fix:** Context isolation
```bash
# Session 1: Backend architecture (isolated context)
claude-code prompt "Read only backend docs. Design payment system architecture."

# Session 2: Backend implementation (isolated context)
claude-code prompt "Read backend architecture + backend patterns. Implement PaymentService."

# Session 3: Testing (isolated context)
claude-code prompt "Read PaymentService implementation + test patterns. Create tests."

# Session 4: Frontend (completely separate context)
gemini chat --session frontend "Read frontend docs + API contracts. Design payment UI components."
```

---

### Pitfall 4: Not Maintaining Architectural Memory

**What it looks like:**

Feature implementation drifts from original architecture over time because decisions aren't documented.

**Week 1:** "We'll use Service Layer pattern with dependency injection"  
**Week 3:** Some files use Service Layer, others access database directly  
**Week 5:** Complete inconsistency, technical debt accumulating

**The fix:** PATTERNS.md + regular consistency checks
```bash
# At start of project
claude-code prompt "Create PATTERNS.md documenting:
- Service Layer pattern (with example)
- Repository pattern (with example)
- Dependency injection approach"

# Every 10-15 files created
gemini chat --session audit "Read all services (src/services/*.py).

Check: Do all services follow the pattern in PATTERNS.md?
List any that don't and suggest fixes."
```

---

### Pitfall 5: Ignoring Context Budget

**What it looks like:**

```bash
# Continually adding files without removing
claude-code prompt "Also read src/payments/gateway.py"
# Later
claude-code prompt "Also read src/payments/processor.py"
# Later
claude-code prompt "Also read src/payments/webhook.py"
# Later (context window 95% full)
claude-code prompt "Implement refund handling"
# AI response is generic and low-quality
```

**The problem:** Context window fills incrementally; quality degrades without you noticing.

**The fix:** Context budget management
```bash
# Before adding files, check budget
claude-code prompt "Before we continue, context health check:
1. Current context usage estimate?
2. Which files in context are still relevant for upcoming tasks?
3. Which files can we drop to make room?

Then load src/payments/refund.py for implementing refund handling."
```

---

## Context Engineering Checklist for AIDD

Use this checklist for every AI development session.

### Before Starting (Pre-Session Setup)

**Context Preparation:**
- [ ] Read relevant specs/requirements first (understand WHAT to build)
- [ ] Identify which memory files exist (DECISIONS.md, PATTERNS.md, TODO.md, GOTCHAS.md)
- [ ] Plan which files to load (progressive, not all at once)
- [ ] Choose appropriate tool (Claude Code for implementation, Gemini CLI for analysis)

---

### Session Initialization (First 5 Minutes)

**Context Loading:**
- [ ] Load memory files first (DECISIONS.md, PATTERNS.md)
- [ ] Load architecture files (high-level context)
- [ ] Load only essential implementation files (3-10 files max to start)
- [ ] Explicitly state don't load other files without asking

**Example Start:**
```bash
claude-code prompt "Read in this order:
1. DECISIONS.md (past decisions)
2. PATTERNS.md (code patterns to follow)
3. src/services/user_service.py (example service)

Don't read other files yet. After reading, confirm you understand the patterns, then I'll tell you the task."
```

---

### During Development (Every 30-45 Minutes)

**Context Health Monitoring:**
- [ ] Run the 5-point context health check (from earlier in this lesson)
- [ ] Ask AI: "Do you need any additional context? What files would help?"
- [ ] Remove irrelevant files from context if window filling up
- [ ] Create checkpoint if making good progress (capture decisions)

**Example Health Check:**
```bash
claude-code prompt "Context health check:
1. Response speed: Fast or slow?
2. Relevant files: Which loaded files are you actually using?
3. Missing context: Any files you wish you had but don't?
4. Current window usage: Estimate percentage
5. Degradation signs: Are answers getting generic?"
```

---

### Session Checkpointing (After Each Feature)

**Capture Progress:**
- [ ] Update DECISIONS.md (any architectural choices made)
- [ ] Update TODO.md (what's done, what's next)
- [ ] Update GOTCHAS.md (any tricky bugs discovered)
- [ ] Document context state (which files are loaded, what's understood)

**Example Checkpoint:**
```bash
claude-code prompt "We just completed user authentication feature.

Update memory files:
1. DECISIONS.md: Add JWT token decision (15-min expiry, refresh tokens)
2. TODO.md: Mark 'User authentication' as done, next is 'Password reset'
3. Create checkpoint summary: What did we build? What context is currently loaded? What's the plan for next feature?"
```

---

### Post-Session Cleanup (Last 5 Minutes)

**Knowledge Preservation:**
- [ ] Finalize all memory file updates
- [ ] Document any unresolved issues in GOTCHAS.md
- [ ] Update TODO.md with clear next steps
- [ ] Note which files will be relevant for next session

**Example Cleanup:**
```bash
claude-code prompt "Session ending. Prepare for next session:

1. Update DECISIONS.md with all architecture choices made today
2. Update TODO.md with:
   - [x] Completed: User auth, password reset
   - [ ] Next session: Email verification
3. Create NEXT_SESSION.md:
   - What we accomplished today
   - What files are relevant for email verification
   - Context to load next time
   - Current state of codebase"
```

---

## Real-World Example: Blog API with Context Engineering

Let's walk through building a complete Blog API over 6 sessions using ALL context engineering strategies.

---

### Session 1: Project Initialization + Architecture (45 minutes)

**Goal:** Set up project structure and design architecture

```bash
# Minimal context - just architecture
claude-code prompt "New project: Blog API with these features:
- User authentication (JWT)
- Create/read/update/delete blog posts
- Comments on posts
- Tags for posts
- User profiles

Design:
1. Project structure (folders/files)
2. Database schema
3. API endpoints structure
4. Service layer architecture

Output as ARCHITECTURE.md and create initial project files."
```

**Outcome:**
- ARCHITECTURE.md created
- Initial project structure
- Database models planned
- API design complete

**Context snapshot at end:**
- Files loaded: 0 implementation files (pure design)
- Memory files created: ARCHITECTURE.md
- Next session prep: Will need PATTERNS.md for code patterns

**Checkpoint:**
```bash
claude-code prompt "Create NEXT_SESSION.md:

## Session 1 Summary
**Completed:** Architecture design, project setup
**Next:** Implement core User authentication

## Context for Next Session
**Load these files:**
- ARCHITECTURE.md
- Create PATTERNS.md (service layer pattern)
- src/models/user.py (after creating it)

**Don't load yet:** Posts, comments (not relevant for auth)"
```

---

### Session 2: Core Feature - User Authentication (1 hour)

**Pre-session:** Read NEXT_SESSION.md from Session 1

```bash
# Progressive loading
claude-code prompt "Read:
- ARCHITECTURE.md (architecture decisions)

Create PATTERNS.md documenting:
- Service Layer pattern (dependency injection)
- Repository pattern
- Route handler pattern

Show examples for each pattern."

# After PATTERNS.md created
claude-code prompt "Now implement User Authentication following PATTERNS.md:

Create:
- src/models/user.py (User model)
- src/repositories/user_repository.py (data access)
- src/services/auth_service.py (business logic)
- src/api/routes/auth.py (endpoints: /register, /login)
- src/schemas/auth_schema.py (request/response)
- tests/test_auth.py (unit + integration tests)

Follow patterns exactly as documented in PATTERNS.md."
```

**Outcome:**
- 6 files created
- All following consistent pattern
- Tests included

**Context snapshot at end:**
- Files loaded: 6 (user auth related)
- Memory files: ARCHITECTURE.md, PATTERNS.md
- Context usage: ~30%

**Checkpoint:**
```bash
claude-code prompt "Session 2 complete. Update:

## DECISIONS.md (create):
**JWT Authentication**
- Access tokens: 15-minute expiry
- Refresh tokens: 7-day expiry, stored in Redis
- Passwords: bcrypt hashing

## TODO.md (create):
- [x] Session 1: Architecture
- [x] Session 2: User authentication
- [ ] Session 3: Blog posts CRUD
- [ ] Session 4: Comments feature
- [ ] Session 5: Tags feature
- [ ] Session 6: User profiles + testing

## NEXT_SESSION.md:
**Load for Session 3:**
- ARCHITECTURE.md, PATTERNS.md, DECISIONS.md
- src/services/auth_service.py (reference for similar pattern)
- Don't load: tests, routes (not needed for posts service design)"
```

---

### Session 3: Related Feature - Blog Posts (1 hour)

```bash
# Load architecture + one example
claude-code prompt "Read:
- ARCHITECTURE.md
- PATTERNS.md
- DECISIONS.md
- src/services/auth_service.py (example of our service pattern)

Implement Blog Posts CRUD following the EXACT same pattern:
- src/models/post.py
- src/repositories/post_repository.py
- src/services/post_service.py
- src/api/routes/posts.py (endpoints: GET/POST/PATCH/DELETE /posts)
- src/schemas/post_schema.py
- tests/test_posts.py

Follow auth_service pattern exactly - same dependency injection, same error handling, same structure."
```

**Outcome:**
- Posts feature complete
- Consistent with auth feature (same patterns)
- Tests included

**Checkpoint:**
```bash
claude-code prompt "Update TODO.md:
- [x] Session 3: Blog posts CRUD

Create consistency check before continuing:

Read:
- src/services/auth_service.py
- src/services/post_service.py

Compare: Are both services following the SAME pattern consistently?
List any differences that shouldn't exist."
```

---

### Session 4: Complex Feature - Comments (Nested Structure) (1.5 hours)

**Challenge:** Comments can reply to other comments (nested structure)

```bash
# More context needed for complex feature
claude-code prompt "Read:
- ARCHITECTURE.md (for comments design)
- PATTERNS.md (service patterns)
- src/models/post.py (to understand post relationship)
- src/services/post_service.py (example service)

Implement Comments with nesting support:
- src/models/comment.py (parent_id for nesting)
- src/repositories/comment_repository.py (handle nesting queries)
- src/services/comment_service.py (nested comment logic)
- src/api/routes/comments.py
- src/schemas/comment_schema.py (nested response format)
- tests/test_comments.py (test nesting edge cases)

Challenges to handle:
- Prevent infinite nesting depth (max 5 levels)
- Efficient nested comment retrieval
- Delete cascades (deleting comment deletes replies)"
```

**Outcome:**
- Complex feature complete
- Nesting logic handled
- Edge cases tested

**Checkpoint:**
```bash
claude-code prompt "Comments were complex. Document learnings:

## GOTCHAS.md (create):
**Issue:** Nested Comments Query Performance
- **Problem:** Recursive queries for deep nesting are slow
- **Solution:** Limit nesting to 5 levels, use CTEs (Common Table Expressions)
- **Code:** src/repositories/comment_repository.py, line 45

Update TODO.md:
- [x] Session 4: Comments with nesting"
```

---

### Session 5: Feature Enhancement - Tags (45 minutes)

```bash
# Simpler feature after complex one
claude-code prompt "Read:
- PATTERNS.md (service pattern)
- src/models/post.py (to add tags relationship)
- src/services/post_service.py (to add tag operations)

Implement Tags feature:
- src/models/tag.py
- Update src/models/post.py (many-to-many relationship with tags)
- src/services/tag_service.py (tag CRUD + tag search)
- src/api/routes/tags.py (GET /tags, POST /tags)
- Update src/api/routes/posts.py (add tag filtering: GET /posts?tag=python)
- tests/test_tags.py"
```

**Outcome:**
- Tags feature complete
- Integrated with posts

**Checkpoint:**
```bash
claude-code prompt "Update TODO.md:
- [x] Session 5: Tags feature
- [ ] Session 6: Integration testing + documentation

Almost done! Prepare for final session."
```

---

### Session 6: Integration + Documentation (1 hour)

```bash
# Large context needed for integration tests
gemini chat --session integration "Read all implementation files:
- src/models/*.py
- src/services/*.py
- src/api/routes/*.py

Create comprehensive integration test suite:
1. Full user journey: Register → Login → Create post → Add comment → Add tags
2. Permission testing: Users can only edit own posts
3. Edge cases: Nested comments, duplicate tags, etc.

Output: tests/integration/test_full_workflow.py"

# Then documentation with Claude
claude-code prompt "Read:
- ARCHITECTURE.md
- All API routes (src/api/routes/*.py)
- All schemas (src/schemas/*.py)

Generate:
1. API_DOCS.md (all endpoints with examples)
2. README.md (project overview, setup instructions)
3. DEPLOYMENT.md (how to deploy)"
```

**Outcome:**
- Integration tests complete
- Documentation complete
- Project ready for production

**Final session summary:**
```bash
claude-code prompt "Project complete! Create PROJECT_SUMMARY.md:

## Blog API - Project Complete

**6 Sessions:** Architecture → Auth → Posts → Comments → Tags → Testing/Docs
**Total Files:** 45 files
**Lines of Code:** ~3,500 lines
**Test Coverage:** 92%

**Key Decisions:** (from DECISIONS.md)
- JWT auth with Redis session revocation
- Service Layer + Repository pattern
- Nested comments (max 5 levels)

**Memory Files Used:**
- ARCHITECTURE.md (overall design)
- PATTERNS.md (code consistency)
- DECISIONS.md (architectural choices)
- GOTCHAS.md (bugs/learnings)
- TODO.md (progress tracking)

**Context Engineering Strategies Used:**
✅ Progressive loading (each session)
✅ Context isolation (separate sessions for features)
✅ Memory files (persistent decisions)
✅ Example-driven context (PATTERNS.md)
✅ Just-in-time fetching (loaded files as needed)
✅ Context checkpointing (after each feature)

**Next Steps:** Deploy to production!"
```

---

## Practice Exercises from Real-World Scenarios

Apply context engineering to these exercises.

### Exercise 1: Context Budget Analysis

```bash
# With Claude Code
claude-code prompt "I have a debugging task: Users report login fails after 3 attempts.

I want to load these files:
- src/api/routes/auth.py (login endpoint)
- src/services/auth_service.py (auth logic)
- src/models/user.py (user model)
- src/repositories/user_repository.py (data access)
- src/middleware/rate_limiter.py (might be blocking?)
- src/utils/password.py (password validation)
- src/services/email_service.py (might send lockout email?)
- tests/test_auth.py (see test patterns)

Analyze this list:
1. Which files are **essential** for debugging?
2. Which files are **helpful but optional**?
3. Which files are **probably unnecessary**?
4. What's your recommended loading order?

Give reasoning for each classification."
```

---

### Exercise 2: Memory File Creation

```bash
# With Gemini CLI
gemini chat "I'm starting a new e-commerce project.

Create initial memory files:

1. **ARCHITECTURE.md:** Design high-level architecture for:
   - Products catalog
   - Shopping cart
   - Checkout/payment
   - Order management
   - User accounts

2. **PATTERNS.md:** Document code patterns we'll follow:
   - Service Layer pattern (with example)
   - Repository pattern (with example)
   - API route pattern (with example)

3. **DECISIONS.md:** Document key architectural decisions:
   - Database choice (PostgreSQL vs MongoDB)
   - Payment gateway (Stripe vs PayPal)
   - Session management (Redis vs JWT)

4. **TODO.md:** Create initial roadmap (10 major features/milestones)

Output all four files with realistic content."
```

---

### Exercise 3: Multi-Session Continuity

```bash
# With Claude Code
claude-code prompt "Simulation: I worked on a notification system yesterday (Session 1).

Session 1 summary:
- Implemented email notifications (src/services/notification_service.py)
- Used Sendgrid for email sending
- Stored notification preferences in src/models/notification_preferences.py

Today (Session 2), I want to add:
- SMS notifications
- Push notifications (mobile app)
- In-app notifications

Design the handoff:
1. What memory files should have been created in Session 1?
2. What should those files contain?
3. How would Session 2 start (what context to load)?
4. How do we ensure consistency between email/SMS/push implementations?"
```

---

### Exercise 4: Context Compression Practice

```bash
# With Claude Code
claude-code prompt "I have a context-heavy session with 25 files loaded:

Essential files (still need):
- src/services/payment_service.py
- src/services/order_service.py
- tests/test_payments.py

Possibly useful:
- src/models/order.py
- src/models/payment.py
- src/schemas/payment_schema.py

No longer relevant (was for earlier tasks):
- src/services/user_service.py
- src/services/email_service.py
- src/api/routes/users.py
- [... 15 more files ...]

Show me a context compression script:
1. Create CHECKPOINT.md with summary of what's been accomplished
2. Identify minimum files needed for current task (refund processing)
3. Create restart command with only essential context"
```

---

### Exercise 5: Tool Comparison for Large Refactoring

```bash
# With Gemini CLI
gemini chat "Scenario: I need to refactor 50 route files to add consistent error handling.

Current state: Each route has different error handling (inconsistent).
Goal: Standardize to a single error handling pattern.

Compare approaches:
1. **Claude Code approach:** How would you tackle this with 200K context?
2. **Gemini CLI approach:** How would you use 1M context differently?
3. **Hybrid approach:** Use both tools in sequence?

Design the workflow for each approach with specific commands."
```


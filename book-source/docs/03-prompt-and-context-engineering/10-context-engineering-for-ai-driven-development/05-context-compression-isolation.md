---
title: "Lesson 5: Context Compression & Isolation"
chapter: 10
lesson: 5
duration_minutes: 22
sidebar_position: 5
description: "Learn to compress context when sessions get long and isolate contexts for different tasks"
keywords: [context compression, context isolation, session management, checkpoints, AIDD strategies]

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Applying Context Compression"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can compress long AI sessions using summarization and checkpoints to reclaim context window space while preserving essential information"

  - name: "Implementing Context Isolation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create separate AI sessions for unrelated tasks to prevent context pollution and maintain focus"

  - name: "Analyzing Context Management Strategies"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate when to use compression (long session), isolation (multiple tasks), or fresh start (clean slate) based on scenario analysis"

learning_objectives:
  - objective: "Apply context compression to manage long sessions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student performs session compression by creating summary checkpoint and continuing with reduced context"

  - objective: "Implement context isolation for multiple concurrent tasks"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates separate AI sessions for 2+ unrelated tasks with appropriate context boundaries"

  - objective: "Analyze when to use compression vs isolation vs starting fresh"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Given 5 scenarios, student selects correct strategy with justification for each"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (Context Compression, Checkpoints, Summarization, Context Isolation, Session Separation, Strategy Selection) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Implement automated compression triggers (detect when context >80% full); design multi-session orchestration for complex projects with dependency management"
  remedial_for_struggling: "Practice compression only first (skip isolation initially); use provided templates for compression prompts; start with 1-hour sessions before attempting longer"
---

# Context Compression & Isolation

## Introduction: When Progressive Loading Isn't Enough

In Lesson 3, you learned progressive loading—the best way to **start** a session. But what happens when:

- Your session has been going for 2+ hours?
- You've completed several features and context is filling up?
- You need to work on multiple unrelated tasks?

This is where you need two more strategies:
1. **Context Compression** - for long sessions
2. **Context Isolation** - for separate concerns

---

## Strategy 1: Context Compression

### The Problem: Long Sessions Fill Context

**Scenario:** You've been working for 90 minutes using perfect progressive loading.

**Current State:**
- ✓ You've built 3 features
- ✓ Made 5 architectural decisions
- ✓ Fixed 2 bugs
- ✓ Had 25 back-and-forth messages with AI

**Context Window:** 70% full

**Warning Signs Starting:**
- ⚠ AI taking longer to respond
- ⚠ Occasional need to repeat information
- ⚠ Minor inconsistencies appearing

**The Question:** You want to keep working, but context is degrading. Do you:
- A) Stop working (lose momentum)
- B) Continue until it breaks (poor quality work)
- C) Something smarter?

**Answer: C) Compress your context!**

---

### WHAT: Context Compression Explained

**Context Compression** is the process of summarizing your session's progress, saving it as a checkpoint, and starting a fresh session with just the essential summary.

**The Analogy:**

Think of your notes while studying:

**Before Compression:**
- 30 pages of messy notes
- Highlighting, scribbles, scratch work
- Some mistakes you corrected
- Hard to find important points

**After Compression:**
- 2 pages of clean, organized summary
- Just the key insights
- Decisions clearly stated
- Easy to reference

**Context compression does this for your AI session.**

---

### WHY: Why Compression Works

**What Happens During Compression:**

**Old Context (70% full):**
```
- Message 1: Load project
- Message 2: Discussion about approach
- Message 3: Try implementation A
- Message 4: Fix bug in approach A
- Message 5: Actually approach B is better
- Message 6: Implement approach B
- Messages 7-25: [More back-and-forth...]
```
**Problems:**
- Contains dead ends and mistakes
- Includes detailed discussions no longer relevant
- Takes up tons of space
- AI has to process all of it

**New Context After Compression (15% full):**
```
CHECKPOINT SUMMARY:

## What We Built
- Feature A: User authentication (JWT tokens)
- Feature B: Profile management (CRUD endpoints)
- Feature C: Password reset flow

## Key Decisions
- Using JWT (not sessions) for auth
- Token expiry: 15 min access, 7 day refresh
- Storing sessions in Redis

## Next Steps
- Add email verification
- Implement 2FA
```
**Benefits:**
- No dead ends or mistakes
- Just the final decisions
- Much less space
- AI processes it instantly

**Result:** You've compressed 70% context down to 15%, leaving 85% free for more work!

---

### HOW: Implementing Context Compression

Let's walk through the complete process.

#### Step 1: Recognize the Trigger

**When to Compress:**
- Every 10-15 interactions (messages back-and-forth)
- After completing a major milestone
- When you notice context degradation signs
- Before ending a session (to resume later)

**Trigger Checklist:**
- [ ] Been working 1+ hours?
- [ ] More than 15 messages exchanged?
- [ ] Completed 2-3 features?
- [ ] AI responding slower?
- [ ] About to stop but want to resume later?

If **any** of these are true → Time to compress!

---

#### Step 2: Create the Compression Summary

**Template Prompt for Claude Code:**

```bash
claude-code prompt "Create a compression checkpoint of our session. 
Summarize in this format:

# SESSION_CHECKPOINT.md

## What We've Built
[List completed features with key details]

## Architecture Decisions Made
[Key technical decisions with reasoning]

## Code Patterns Established
[Patterns to follow in future code]

## Current Status
[What works, what's in progress, what's next]

## Known Issues / TODOs
[Outstanding tasks and known bugs]

Keep it under 500 words. Be specific enough that I (or an AI) can 
resume work from this summary."
```

**Example Output:**

```markdown
# SESSION_CHECKPOINT.md

## What We've Built

### 1. User Authentication System
- JWT-based authentication (access + refresh tokens)
- Login/logout endpoints in src/api/routes/auth.py
- Token validation middleware
- Access token: 15 min, Refresh token: 7 days

### 2. Profile Management
- CRUD operations for user profiles
- Profile model extends User model
- Fields: bio, avatar_url, social_links
- Endpoints: /profile (GET, PUT)

### 3. Password Reset Flow
- Secure token generation (secrets.token_urlsafe)
- Email integration (SendGrid)
- Token expiry: 1 hour
- Rate limiting: 5 requests/hour per email

## Architecture Decisions Made

**Decision: JWT over Sessions**
- Reason: Stateless, scalable for microservices
- Tradeoff: Can't revoke tokens (solved with short expiry)

**Decision: Redis for Token Blacklist**
- Reason: Fast lookups for revoked tokens
- Alternative considered: Database (too slow)

**Decision: Async/Await Throughout**
- Reason: Better performance for I/O operations
- Pattern: All services and routes are async

## Code Patterns Established

**Service Pattern:**
```python
class ServiceName:
    def __init__(self, db: Database, redis: Redis):
        self.db = db
        self.redis = redis
    
    async def method(self, param: Type) -> ReturnType:
        """Docstring in Google style."""
        # Implementation
```

**Error Handling:**
- Custom exceptions (AuthenticationError, ValidationError)
- Raised with specific error codes
- Converted to HTTP exceptions in API layer

**Testing Pattern:**
- Pytest with async fixtures
- Mock external dependencies
- AAA pattern (Arrange, Act, Assert)

## Current Status

✅ **Complete:**
- User auth endpoints working
- Profile CRUD working
- Password reset working
- Unit tests: 85% coverage

🚧 **In Progress:**
- Email verification (started, not complete)

📋 **Next Steps:**
1. Complete email verification
2. Add 2FA (time-based tokens)
3. Implement session management dashboard

## Known Issues / TODOs

⚠ **Bug:** Reset tokens not properly cleaned up after use (low priority)
📝 **TODO:** Add rate limiting to all auth endpoints
📝 **TODO:** Write integration tests for full auth flow
📝 **TODO:** Document API endpoints in OpenAPI spec

---
Last Updated: 2025-11-03
Session Duration: 90 minutes
Messages: 23
```

---

#### Step 3: Start Fresh Session with Summary

**New Session (Day 2 or After Break):**

```bash
# New terminal session or next day
claude-code prompt "I'm resuming work on the auth project. 
Read SESSION_CHECKPOINT.md to understand what we've built so far.

Then let's continue with the next task: implementing email verification."
```

**What Happens:**
- AI reads the checkpoint (small, focused summary)
- Understands all decisions and patterns instantly
- Knows current status and next steps
- Context usage: 15% (vs 70% if you'd continued old session!)

**Result:** You've "transported" your project context into a fresh session!

---

### Compression Best Practices

**✅ DO:**
- Compress every 10-15 interactions
- Be specific in summaries (not vague)
- Include code pattern examples
- Note TODOs and known issues
- Save checkpoint to a file (SESSION_CHECKPOINT.md)

**❌ DON'T:**
- Wait until context is 95% full (too late!)
- Make summaries too vague ("built some auth stuff")
- Forget to include decisions and reasoning
- Lose the checkpoint file (save it!)
- Include obsolete or incorrect information

---

## Strategy 2: Context Isolation

### The Problem: Multiple Concurrent Tasks

**Scenario:** In one day, you need to:
1. Build a new user authentication feature
2. Fix a bug in the payment processing
3. Update documentation
4. Review and improve database indexes

**If you do all of this in ONE session:**
- Context gets mixed up
- AI confuses which task you're on
- Code from Task 1 leaks into Task 2
- Different concerns blend together
- Lower quality results for everything

**Better Approach:** Keep contexts isolated.

---

### WHAT: Context Isolation Explained

**Context Isolation** is the practice of using separate "context environments" for different tasks, so information doesn't mix or interfere.

**The Analogy:**

**Without Isolation (One Messy Desk):**
- Math homework, history essay, art project all spread out
- Papers getting mixed up
- Can't focus on one thing
- Constantly context-switching

**With Isolation (Separate Work Areas):**
- Math at dining table
- History essay in bedroom
- Art project in studio
- Each area has only relevant materials
- Easy to focus

**Context isolation gives each task its own "clean desk."**

---

### WHY: Why Isolation Works

**Benefits of Isolated Contexts:**

**1. Prevents Cross-Contamination**
```bash
# BAD: Mixed context
Session 1: Working on auth AND payment bug AND database
AI: "Should I use JWT tokens for the database query?" 
     (confused! mixing auth and database context)

# GOOD: Isolated contexts
Session A: Auth only → AI focused on auth patterns
Session B: Payment bug only → AI focused on payment logic
Session C: Database only → AI focused on database optimization
```

**2. Maintains Focus**
- Each session has clear goal
- No distractions from other tasks
- AI provides more relevant suggestions

**3. Clearer Conversation History**
- Easy to review what happened for specific task
- No need to scroll through unrelated discussions
- Better for future reference

---

### HOW: Implementing Context Isolation

You have three options:

---

#### Option A: Separate Terminal Sessions (Simplest)

**How:** Open different terminal windows or tabs for each task.

```bash
# Terminal 1 (Task A: Auth Feature)
claude-code prompt "Working on authentication module"
# ... all auth work here ...

# Terminal 2 (Task B: Payment Bug)
claude-code prompt "Working on payment processing bug"
# ... all payment work here ...

# Terminal 3 (Task C: Documentation)
claude-code prompt "Updating project documentation"
# ... all doc work here ...
```

**Pros:**
- ✓ Physically separate sessions
- ✓ Easy to see which task you're on
- ✓ Can switch between tasks easily

**Cons:**
- ✗ Need to manage multiple windows
- ✗ Easy to forget which window is which

**Best for:** Different tasks you'll switch between during the day.

---

#### Option B: Explicit Context Switch Markers

**How:** Use clear markers in same session to "reset" context mentally.

```bash
# Working on Task A
claude-code prompt "Build auth feature..."
# ... work ...

# Explicit switch
claude-code prompt "=== CONTEXT SWITCH ===

Forget everything about authentication. 

NEW TASK: Fix payment processing bug.
Context for this task:
- Bug: Payment fails when amount > $1000
- File: src/services/payment_service.py
- Errors occur in Stripe API call

Ready to debug this issue."
```

**Pros:**
- ✓ One session (less window management)
- ✓ Clear separation in conversation

**Cons:**
- ✗ Old context still taking up space
- ✗ Relies on AI respecting "forget" instruction
- ✗ Less reliable than physical separation

**Best for:** Quick task switches, short tasks.

---

#### Option C: Named Sessions (Gemini CLI)

**How:** Gemini CLI supports named sessions.

```bash
# Create isolated sessions
gemini chat --session auth "Working on authentication module"
gemini chat --session payment-bug "Working on payment processing bug"
gemini chat --session database-optimization "Working on database indexes"

# Switch between them
gemini chat --session auth "Continue auth work..."
gemini chat --session payment-bug "Continue payment debugging..."
```

**Pros:**
- ✓ True session isolation
- ✓ Easy to resume any session
- ✓ Sessions persist

**Cons:**
- ✗ Specific to Gemini CLI
- ✗ Requires remembering session names

**Best for:** Multi-day projects with different workstreams.

---

### When to Use Each Isolation Method

| Scenario | Recommended Method |
|----------|-------------------|
| **Different features, same day** | Separate terminal sessions (Option A) |
| **Quick task switch** | Context switch marker (Option B) |
| **Multi-day parallel work** | Named sessions (Option C) |
| **Debugging vs building** | Separate sessions (A or C) |
| **Different tech stacks in same project** | Separate sessions (A or C) |

---

## Choosing the Right Strategy

You now have three strategies. When do you use each?

### Decision Matrix

| Situation | Strategy | Why |
|-----------|----------|-----|
| **Starting new project** | Progressive Loading | Load only what you need as you go |
| **Session going 1+ hour** | Compression | Prevent context rot in long sessions |
| **Multiple unrelated tasks** | Isolation | Keep contexts focused and separate |
| **Resuming work next day** | Compression (checkpoint) | Reload context from summary |
| **Need to switch tasks mid-session** | Isolation | Don't mix concerns |
| **Context 70%+ full** | Compression + Fresh Start | Reset before it breaks |

### Combining Strategies

**Often you'll use multiple strategies together!**

**Example Workflow:**

```
Day 1 Morning:
├─ Progressive Loading → Start feature A
├─ [Work for 90 min, context 60% full]
└─ Compression → Create checkpoint

Day 1 Afternoon:
├─ Reload checkpoint → Resume feature A
├─ Isolation → New session for bug fix (Task B)
└─ Compression → Save checkpoint for both tasks

Day 2:
├─ Reload checkpoints → Resume both features
└─ Progressive Loading → Start new feature C (separate session)
```

**All three strategies working together!**

---

## Validation Checkpoints

### ✅ Signs You're Using Compression Correctly

After compression:
- [ ] New session starts with 10-20% context (not 70%+)
- [ ] Checkpoint captures all key decisions
- [ ] AI can resume work without confusion
- [ ] You have clear record of what was built

### ✅ Signs You're Using Isolation Correctly

With isolated contexts:
- [ ] Each task has its own clear focus
- [ ] AI doesn't mix concepts from different tasks
- [ ] Easy to switch between tasks
- [ ] Conversation history for each task is clean

### ❌ Signs You Need to Adjust

**Red Flags:**
- ❌ Context still 70%+ after compression (summary too detailed)
- ❌ AI confuses which task you're working on (poor isolation)
- ❌ Checkpoint too vague to resume work from
- ❌ Working on 3 tasks in one session and AI is confused

---

## Try With AI

**Tool:** ChatGPT Web

Practice thinking about compression and isolation.

### Prompt 1: When to Compress
Copy this into ChatGPT:

```
I've been working with Claude Code for an hour on a Python project. 
We've built 2 features and had about 20 back-and-forth messages. 
I want to keep working, but I'm noticing the AI is taking longer 
to respond.

Should I:
A) Just keep going
B) Start a completely fresh session
C) Do something else?

What's the best practice here?
```

**What to expect:**
- ChatGPT should recommend compression (option C)
- Explanation of why continuing causes problems
- Guidance on creating a checkpoint summary

**Check:** Do you understand when compression is the right choice?

---

### Prompt 2: Practicing Compression
Copy this into ChatGPT:

```
I'm about to compress my context. Here's what I've done in this session:
- Built a user authentication system using JWT
- Decided to use Redis for token blacklisting
- Created login, logout, and refresh endpoints
- Started but didn't finish email verification

Create a checkpoint summary following best practices. Include:
- What we built
- Key decisions
- Current status
- Next steps

Make it concise but specific enough to resume work.
```

**What to expect:**
- Structured checkpoint summary
- Specific details about what was built
- Clear decisions documented
- Next steps identified

**Practice:** Save this! You've just practiced creating a real checkpoint.

---

### Prompt 3: Understanding Isolation
Copy this into ChatGPT:

```
In one day, I need to:
1. Build a new payment integration feature
2. Fix a critical security bug in authentication
3. Update documentation

Should I do all of this in one AI session or use separate sessions? 
Why? What are the benefits of each approach?
```

**What to expect:**
- Recommendation for separate sessions (isolation)
- Explanation of why mixing contexts causes problems
- Benefits: focus, no cross-contamination, clearer history

**Reflection:** Can you think of tasks that would be OK in one session vs need isolation?


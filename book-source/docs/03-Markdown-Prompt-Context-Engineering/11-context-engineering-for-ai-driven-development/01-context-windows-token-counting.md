---
title: Lesson 1 - Context Windows and Token Counting
sidebar_position: 1
chapter: 11
lesson: 1
learning_objectives:
  - Manually observe context window filling without relying on automated metrics
  - Identify the degradation threshold where context window approaches saturation
  - Distinguish between different types of content in context window (code, conversation, system prompts)
estimated_time: 45 minutes
proficiency_level: B1
generated_by: content-implementer v1.0.0
source_spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
created: 2025-01-18
version: 1.0.0
---

# Lesson 1: Context Windows and Token Counting

## Your Context Window Is Your Working Memory

Imagine you're writing code with a colleague looking over your shoulder. Your colleague can hold the entire project structure in their mind, remember every function you've written, track conversation history, and synthesize it all into helpful suggestions. But there's a limit to how much they can hold in working memory at once.

AI models work the same way—except their "working memory" is called the context window, and it's measured in tokens (atomic units of text). When you load files, write prompts, and execute commands, you're filling that working memory. And just like a human colleague gets confused when given too much information at once, AI models degrade when their context window fills beyond capacity.

Andrej Karpathy, from the Y Combinator AI School, crystallizes this principle perfectly:

> "The LLM is the CPU, performing the core computation. The context window is the RAM, holding the working memory for a task. Context engineering is the delicate art and science of filling the context window with just the right information for the next step."

This lesson teaches you to observe and measure your context window—not to understand LLM internals, but to recognize when it's getting full and when degradation is about to strike.

## Exercise 1: Estimate Token Count by Hand

Before learning frameworks, try something: estimate how many tokens are in the transcript below. Don't overthink it. Your job is to get a rough sense of scale.

**Sample Project Transcript** (Claude Code session, beginning):

```
User: I have a Python FastAPI authentication system.
Let me load the project structure:

```
project/
├── main.py (150 lines)
├── models/
│   ├── user.py (45 lines)
│   └── auth.py (60 lines)
├── routes/
│   ├── auth.py (120 lines)
│   └── users.py (85 lines)
└── utils/
    ├── security.py (75 lines)
    └── database.py (95 lines)
```

User: Start by loading main.py and models/user.py. I want to understand the current
authentication flow and add OAuth2 support.

Claude: I can see the project structure. Let me start with the foundation:
- main.py sets up FastAPI application
- models/user.py defines user schema
This gives me project context. What's in main.py?

User: Here's main.py:

```python
from fastapi import FastAPI
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from routes import auth, users

app = FastAPI(
    title="Auth System",
    version="1.0.0"
)

# Database setup
DATABASE_URL = "postgresql://localhost/auth_db"
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(bind=engine)

@app.get("/health")
def health_check():
    return {"status": "healthy"}

@app.on_event("startup")
async def startup():
    # Create tables
    pass

@app.on_event("shutdown")
async def shutdown():
    # Close connections
    pass

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

Claude: Good. Standard FastAPI setup with SQLAlchemy. I see the database URL is
hardcoded—we'll need to move that to environment variables before OAuth2 integration.

What's in models/user.py?

User: Here's models/user.py:

```python
from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.ext.declarative import declarative_base
from pydantic import BaseModel
import datetime

Base = declarative_base()

class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True)
    email = Column(String, unique=True, index=True)
    password_hash = Column(String)
    created_at = Column(DateTime, default=datetime.datetime.utcnow)

    class Config:
        orm_mode = True

class UserCreate(BaseModel):
    email: str
    password: str

class UserResponse(BaseModel):
    id: int
    email: str
    created_at: datetime.datetime
```

Claude: I understand. Current auth is password-based with password hashing.
To add OAuth2, we need to:
1. Keep password auth as fallback
2. Add OAuth2 provider support (Google, GitHub)
3. Add provider_id field to User model
4. Create OAuth2 routes

Should I check authentication routes now, or do you want to discuss the OAuth2 strategy first?
```

**Your Task**:

1. Count the approximate tokens in this transcript using this rule of thumb: **roughly 1.3 words = 1 token** (for English text)
   - Count words in the transcript above
   - Divide by 1.3 to estimate tokens

2. Write down your estimate: **___ tokens**

3. In the box below, describe what YOU think is taking up the most space in this conversation so far:
   - AI explanations?
   - Code blocks?
   - Project structure diagrams?
   - Conversation metadata?

Don't worry about being exact. The point is building intuition for scale.

<details>
<summary>Click to reveal actual token count</summary>

The transcript above is approximately **1,200-1,350 tokens**.

Here's the breakdown:
- Project structure diagram: ~150 tokens
- User messages + Claude responses: ~400 tokens
- main.py code block: ~400 tokens
- models/user.py code block: ~350 tokens
- Whitespace and formatting: ~100 tokens

**Key insight**: Code is information-dense. That 40-line code sample took ~350 tokens, but the same *meaning* could be explained in 50 words (65 tokens). Context engineering is about choosing which form (code vs explanation) makes sense for your task.

</details>

## What Is a Token?

Now that you've estimated tokens, let's define them precisely—not as LLM internals, but as a practical unit of context.

A **token** is an atomic unit of text that an AI model processes. Think of it like a word, but not exactly:

- **English words** average about 1.3 words per token (words split into subwords)
- **Code** is more token-dense because special characters (`:`, `{`, `}`, `.`, etc.) are separate tokens
- **Whitespace** and formatting characters count as tokens

**Why this matters for context engineering**: If your context window is 200,000 tokens (Claude 3.5 Sonnet standard), and you load a 500-page document, you've used up a significant chunk. If you load 5 large Python files, you're approaching the midpoint. Understanding token scale helps you make decisions about what to load.

## Types of Content in Your Context Window

Your context isn't just code. Every piece of information takes up tokens:

1. **System prompt** (how you're asking AI to behave)
   - "You are a Python expert..."
   - Example: 200-500 tokens

2. **Conversation history** (everything you and AI have said)
   - Accumulates as session continues
   - Example: 1,000+ tokens after extended conversation

3. **Loaded files** (code, documentation, specs)
   - Project structure, model files, configurations
   - Example: 5 files × 400 tokens average = 2,000 tokens

4. **AI responses** (suggestions, generated code, explanations)
   - Output counts toward context for next message
   - Example: Complex code suggestion = 500 tokens

5. **Tool outputs** (error messages, command results)
   - git status, test output, terminal logs
   - Example: Full test suite run = 1,000+ tokens

A typical 2-hour Claude Code session might look like:

| Content Type | Estimated Tokens | % of Context |
|---|---|---|
| System prompt | 300 | 1% |
| Initial project files (5) | 2,000 | 10% |
| Conversation (100 messages) | 10,000 | 50% |
| Generated code outputs | 5,000 | 25% |
| Tool outputs (git, tests) | 2,700 | 14% |
| **Total** | **20,000** | **100%** |

If you have a 200K context window, this session uses only 10% of your budget. But scale this to a full day of work, and you can see how context fills up.

## Tracking Utilization: Your First Measurement Tool

**Context utilization** is simply: what percentage of your context window are you using?

Formula:
```
Utilization % = (Tokens Used ÷ Context Window Size) × 100
```

**Example calculations**:

**Scenario A: Fresh Claude Code session**
- Context window: 200,000 tokens (Claude 3.5 Sonnet)
- Used so far: 5,000 tokens (2 files loaded, brief conversation)
- Utilization: (5,000 ÷ 200,000) × 100 = **2.5%** ✅ Very comfortable

**Scenario B: Mid-session with extended conversation**
- Context window: 200,000 tokens
- Used: 130,000 tokens (100+ message conversation, 10 files, lots of code)
- Utilization: (130,000 ÷ 200,000) × 100 = **65%** ✅ Still healthy

**Scenario C: Long session getting crowded**
- Context window: 200,000 tokens
- Used: 160,000 tokens (90+ minute session, conversation fills up)
- Utilization: (160,000 ÷ 200,000) × 100 = **80%** ⚠️ Approaching danger zone

**Why track this?** Because at 80%+ utilization, AI quality degrades. Suggestions become repetitive. The model forgets earlier architectural decisions. Contradictions appear.

## Exercise 2: Spot the Saturation Point

Read the conversation transcript below. It shows a 2-hour Claude Code session where the developer is implementing authentication. Your job: **identify the message where context saturation starts affecting AI quality**.

You're looking for:
- Repetitive suggestions (AI repeats earlier advice)
- Forgotten context (AI contradicts earlier statements)
- Delayed responses (takes longer to generate)
- Generic suggestions (loses project-specific understanding)

**Session Transcript (Minutes 0-120)**:

```
[00:05] User: I'm building an auth module for FastAPI. My project uses
PostgreSQL and Pydantic for validation. Load project structure +
models/auth.py to start.

[00:10] Claude: Loaded. I see your User model uses bcrypt for password
hashing. Good security practice. I recommend adding JWT token support
with 24-hour expiry for stateless auth. Here's a token generation
function...

[00:25] User: Great. Now I need route handlers for login, register,
refresh token. Add routes/auth.py.

[00:30] Claude: Perfect. I'll create login (username/password),
register (email/password), and refresh_token routes. All following
security best practices: password validation, rate limiting on login
attempts...

[00:45] User: This looks good. Now add a middleware to protect routes
that need authentication.

[00:50] Claude: [Provides middleware code with JWT verification] Good
practice to include the @require_auth decorator for protecting routes.

[01:15] User: Now I need to integrate this into my main application.
Load main.py.

[01:20] Claude: I see main.py has routes for /users, /auth, /health.
I'll add the auth middleware to the FastAPI app configuration.

[01:30] User: Perfect. Now I need email verification on registration.
When user registers, send them a verification email.

[01:35] Claude: For email verification, add a new field to User model:
email_verified (boolean), then create routes/email.py for sending
verification tokens.

[01:50] User: Good. Now add a verify_email route handler.

[01:55] Claude: [Provides verify_email route] This takes a token from
email link, validates it, marks user as verified in database.

[02:15] User: I need password reset now. User forgets password, we
send reset email with token.

[02:20] Claude: For password reset, I recommend JWT tokens with short
expiry (15 minutes). Create a reset_password route that accepts token +
new_password...

[02:35] User: Good. Now I want to add OAuth2 support. Load
routes/oauth2.py.

[02:40] Claude: I see OAuth2 routes exist. For OAuth2, I recommend
creating a password-based auth system first. Add password validation
and... [NOTICE: This contradicts our approach from 00:10-00:50 where
we already built password + JWT system]

[02:55] Claude: Wait, I also notice I'm suggesting adding bcrypt
password hashing again. Didn't we add that earlier?

[03:10] User: Yes, we already have bcrypt, JWT, middleware, email
verification. Why are you repeating?

[03:15] Claude: [Generic response] You're right, let me review the
conversation history... [Long pause, slower response time] I see JWT
was implemented at 00:30. My apologies for the redundancy.

[03:30] User: I'm also concerned about security. Should we use OAuth2
refresh token rotation?

[03:35] Claude: OAuth2 is a good security practice. I recommend using
JWT tokens... [NOTICE: Ignoring your question about refresh token
ROTATION, instead explaining OAuth2 basics already covered]

[03:50] User: I'm going to start a new session. This one is getting
unusable.
```

**What happened?**

At **02:40** (minute 160 of a 200K context window), the AI started degrading:
- **02:40**: Contradiction (suggesting password auth when already built)
- **02:55**: Repetition (suggesting bcrypt hashing again)
- **03:15**: Slowness (long pause, generic response, context overhead)
- **03:35**: Context reference loss (forgets we discussed refresh token rotation)

This is the saturation point. Not a sudden cliff—gradual degradation over 50 minutes as context fills from 60% to 85%+ utilization.

## The Degradation Threshold

You can't measure your exact context utilization (Claude Code doesn't display live token counts), but you can **observe** when degradation starts through:

1. **Response quality drops** (generic instead of specific suggestions)
2. **Repetition increases** (AI suggests same solution twice)
3. **Contradictions appear** (AI contradicts earlier statements)
4. **Forgotten context** (AI doesn't reference earlier decisions)
5. **Response latency increases** (noticeably slower generations)

The **degradation threshold** is typically **80%+ context utilization**, but you discover it through observation, not metrics.

## Comparing Models: Context Window Specifications

Different AI models have different context window capacities. If you're designing your context strategy, knowing these matters:

| Model | Standard Context | Extended Context | Use Case |
|---|---|---|---|
| **Claude 3.5 Sonnet** | 200K tokens | 1M tokens (tier 4+) | Focused development, deep reasoning on focused codebases |
| **Gemini 1.5 Pro** | 128K tokens | 2M tokens | Large codebase exploration, pattern analysis across files |

**Context window growth over time** (2024-2025):
- 2024: Standard windows 8K-32K tokens
- 2025: Standard windows 128K-200K tokens, extended up to 2M tokens available
- Trend: Access to larger context windows increasing, but cognitive load still applies (even with 2M tokens, quality degrades at high utilization)

**Implication for context engineering**: More tokens are available, but your job isn't "load everything"—it's "load JUST THE RIGHT AMOUNT." A 2M token window is worthless if you load files randomly. Context engineering is about discipline.

## Try With AI

You've now observed context windows, estimated tokens, and spotted saturation symptoms. Before moving to Lesson 2 (mitigation strategies), validate your understanding through these self-check exercises.

**Exercise 1: Estimation Accuracy**

Given these scenarios, estimate tokens (using 1.3 words per token):

1. A Pydantic model with 25 lines of code (plus docstrings): ___ tokens
2. A typical 80-message conversation between you and AI: ___ tokens
3. A 500-line Python file with comments and docstrings: ___ tokens

<details>
<summary>Reveal estimates</summary>

Typical estimates:
1. **25-line Pydantic model**: 200-300 tokens (code is token-dense)
2. **80-message conversation**: 3,000-5,000 tokens (average 40-60 tokens per message)
3. **500-line Python file**: 2,500-3,500 tokens

Your estimates don't need to be exact. The point is recognizing **code takes more tokens than you'd guess from line count** and **conversations accumulate quickly**.

</details>

**Exercise 2: Identify Content Types**

In your next Claude Code session, pause after 15 minutes and mentally inventory what's in your context:

- How many files have you loaded? (estimate tokens)
- How many messages exchanged? (estimate tokens)
- Any command outputs? (estimate tokens)
- What percentage of your 200K window is used?

Write this down. You'll use similar observations in Lesson 2.

**Exercise 3: Spot the Saturation Point**

When you next work in Claude Code for 60+ minutes on a real task, watch for:
- Do AI suggestions become more generic around 90 minutes?
- Do contradictions appear?
- Do responses get noticeably slower?
- Record when you first notice degradation (rough time elapsed)

This observation skill is your foundation for Lesson 2.
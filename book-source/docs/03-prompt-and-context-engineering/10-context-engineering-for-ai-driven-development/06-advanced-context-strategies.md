---
title: "Advanced Context Strategies"
chapter: 10
lesson: 6
duration_minutes: 28
sidebar_position: 6
description: "Master advanced techniques for context management including file selection, memory files, example-driven patterns, multi-agent architecture, and just-in-time fetching"
keywords: [advanced context strategies, memory files, example-driven context, multi-agent, just-in-time loading, AIDD]

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Context Curation Through File Selection"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can explicitly specify which files AI should read (using paths/patterns) to control context precisely"

  - name: "Structured Note-Taking with Memory Files"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create and maintain memory files (decisions.md, patterns.md) to persist knowledge across sessions"

  - name: "Example-Driven Context Engineering"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can provide code examples to teach AI project patterns instead of writing lengthy descriptions"

  - name: "Designing Multi-Agent Workflows"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can decompose complex tasks into specialized agent responsibilities with appropriate context boundaries"

  - name: "Just-In-Time Context Fetching"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can configure AI tools to request additional context as needed rather than loading everything upfront"

learning_objectives:
  - objective: "Apply context curation through explicit file selection"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student provides explicit file paths/patterns to AI tool to control context load"

  - objective: "Implement structured note-taking with memory files"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates memory files documenting decisions, patterns, standards for project"

  - objective: "Use example-driven context engineering effectively"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student provides 2-3 code examples to teach AI instead of lengthy descriptions"

  - objective: "Design multi-agent workflows for complex features"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student breaks down complex task into 3+ specialized agents with context boundaries"

  - objective: "Apply just-in-time context fetching strategies"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student demonstrates on-demand context requests during development session"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Context Curation, Memory Files, Example-Driven Context, Multi-Agent Architecture, Just-In-Time Fetching) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Build automated memory file system with git hooks; implement multi-agent orchestration with Claude Projects or custom tooling; research agentic frameworks"
  remedial_for_struggling: "Master context curation and example-driven first (skip multi-agent initially); use templates for memory files; practice with provided examples before creating own"
---

# Advanced Context Strategies

## Introduction: Beyond the Basics

You've learned the foundational strategies:
- ✓ Progressive loading (Lesson 4)
- ✓ Context compression (Lesson 5)
- ✓ Context isolation (Lesson 5)

But professional AI-native developers use **five more advanced strategies** for complex real-world scenarios.

**This lesson teaches:**
1. Context Curation (control exactly what AI reads)
2. Structured Note-Taking (persist decisions across sessions)
3. Example-Driven Context (teach through code examples)
4. Multi-Agent Architecture (specialized AI agents for different tasks)
5. Just-In-Time Fetching (let AI request context as needed)

These strategies separate beginners from professionals.

---

## Strategy 4: Context Curation Through File Selection

### The Problem

When you ask AI to "fix the authentication bug," it might:
- Read ALL files in `src/auth/` (30+ files)
- Load database models it doesn't need
- Fill 60% of context with irrelevant code

**Result:** Context waste, slower responses, less accurate fixes.

---

### The Solution: Explicit File Targeting

**Context Curation** = You explicitly control which files AI reads.

**The Analogy:**

**Doctor with test results:**
- ❌ **Bad:** "Here are all 50 medical tests from the past year"
- ✅ **Good:** "Here's the X-ray from this week showing the fracture"

**Same with AI:**
- ❌ **Bad:** "Read everything in src/auth/ and fix the bug"
- ✅ **Good:** "Read ONLY src/auth/jwt_handler.py and fix the token expiration bug"

---

### Technique 1: Explicit File Lists

**Example with Claude Code:**

```bash
# Instead of vague request
claude-code prompt "Fix the authentication bug"
# AI might read 20+ files

# Explicit file targeting
claude-code prompt "Fix the JWT token expiration bug.

Only read these files:
- src/auth/jwt_handler.py
- src/auth/middleware.py
- tests/test_jwt.py

Don't read other files unless you explain why you need them."
```

**Benefits:**
- Context usage: 15% instead of 60%
- AI focused on relevant code
- Faster, more accurate fixes

---

### Technique 2: Context Budget Management

For complex tasks requiring many files, negotiate with AI first.

**Example with Gemini CLI:**

```bash
gemini chat "I need to refactor authentication to support multiple identity providers (Google, GitHub, email/password).

This is complex and will touch many files. Let's manage context budget:

Step 1: Tell me which files you need to read
Step 2: I'll confirm which are most critical
Step 3: You read only approved files
Step 4: If you need more, ask before reading

Start by telling me your file reading plan."
```

**AI might respond:**
```
I need to understand:
1. Current auth: src/auth/authentication.py, src/auth/jwt_handler.py
2. User model: src/models/user.py
3. Existing providers: Check if src/auth/providers/ exists
4. Testing patterns: tests/test_auth.py

Should I read these 4-5 files first?
```

**You approve:** "Yes, read those. Wait for my approval before reading more."

**Benefits:**
- AI reads minimum necessary files
- You maintain control over context usage
- Can course-correct if AI wants too much

---

### Technique 3: Layered File Access

Load context in layers, from abstract to specific.

**Example with Claude Code:**

```bash
# Layer 1: Interfaces only
claude-code prompt "Read only these interface files to understand architecture:
- src/services/base_service.py (abstract base)
- src/repositories/base_repository.py (abstract base)

Don't read implementations yet."

# Layer 2: Specific implementation
claude-code prompt "Now read the User implementation specifically:
- src/services/user_service.py
- src/repositories/user_repository.py"

# Layer 3: Related utilities (if needed)
claude-code prompt "If needed, read src/utils/validation.py to understand validation patterns"
```

**Benefits:**
- AI understands architecture before details
- Can stop early if architecture is wrong
- Builds context progressively

---

## Strategy 5: Structured Note-Taking (Agentic Memory)

### The Problem

You work on a feature for 3 days. Important decisions made:
- Day 1: "We'll use JWT tokens with 15-minute expiry"
- Day 2: "We'll store sessions in Redis for revocation"
- Day 3: New session, AI has NO MEMORY of days 1-2!

**Result:** You re-explain everything, waste time, AI makes inconsistent choices.

---

### The Solution: Memory Files

**Structured Note-Taking** = Create files that persist decisions outside AI conversation memory.

**The Analogy:**

**Meeting notes:**
- ❌ **Bad:** Remember everything from memory (forget key details)
- ✅ **Good:** Take structured notes during meeting, reference later

**Same with AI development:**
- ❌ **Bad:** Rely on AI conversation memory (degrades and resets)
- ✅ **Good:** Create memory files AI can read in future sessions

---

### Memory File 1: DECISIONS.md

**Purpose:** Document architectural decisions with reasoning.

**Example with Claude Code:**

```bash
# During implementation
claude-code prompt "We just decided to use Redis for session storage instead of database sessions.

Create/update DECISIONS.md with this entry:

## Decision: Redis for Session Storage
**Date:** [today]
**Context:** Need fast session lookup for API authentication
**Decision:** Use Redis instead of database sessions
**Alternatives Considered:**
- Database sessions (too slow for high-traffic API)
- JWT-only (can't revoke tokens server-side)
**Consequences:**
- Positive: Fast, scalable, easy revocation
- Negative: Additional infrastructure, memory usage
**Implementation:**
- TTL: 24 hours
- Key format: session:{user_id}:{token_id}"
```

**Next session (tomorrow):**
```bash
# Load decisions
claude-code prompt "Read DECISIONS.md to understand our architecture decisions.

Now implement the logout endpoint following those decisions."
```

**Benefits:**
- Decisions persist across sessions
- Future developers (or future you) understand WHY
- AI generates consistent code

---

### Memory File 2: PATTERNS.md

**Purpose:** Document code patterns to follow.

**Example with Gemini CLI:**

```bash
# After establishing service pattern
gemini chat "Create PATTERNS.md documenting our Service Layer Pattern.

Include this example:

\`\`\`python
class UserService:
    '''Service for user operations.'''
    
    def __init__(self, db: Database, cache: Cache):
        self.db = db
        self.cache = cache
    
    async def get_user(self, user_id: int) -> UserSchema:
        '''Get user by ID with caching.'''
        # Try cache first
        cached = await self.cache.get(f'user:{user_id}')
        if cached:
            return UserSchema.parse_raw(cached)
        
        # Fetch from database
        user = await self.db.query(User).get(user_id)
        if not user:
            raise HTTPException(404, 'User not found')
        
        # Cache result
        schema = UserSchema.from_orm(user)
        await self.cache.set(f'user:{user_id}', schema.json(), ttl=3600)
        return schema
\`\`\`

Document:
- Constructor pattern (dependency injection)
- Async methods throughout
- Caching strategy
- Error handling approach
- Type hints and docstrings"
```

**Next feature:**
```bash
gemini chat "Read PATTERNS.md.

Now implement ProductService following the EXACT same pattern."
```

---

### Memory File 3: TODO.md

**Purpose:** Track outstanding tasks.

**Example with Claude Code:**

```bash
# Update as you work
claude-code prompt "Update TODO.md:

## In Progress
- [x] User authentication (JWT)
- [x] User profile CRUD
- [ ] Password reset (50% done - email integration pending)

## Next Up
1. Email verification
2. Two-factor authentication
3. Session management dashboard

## Backlog
- OAuth social login
- API rate limiting
- Admin panel"
```

**Next session:**
```bash
claude-code prompt "Read TODO.md. What's next?"
# AI: "Password reset is 50% done with email integration pending"
```

---

### Memory File 4: GOTCHAS.md

**Purpose:** Document tricky bugs and solutions.

**Example with Gemini CLI:**

```bash
gemini chat "Add to GOTCHAS.md:

## Issue: Async Context Manager Leaks
**Problem:** Database connections not closing in async context managers
**Symptom:** 'Too many connections' error after 100 requests
**Root Cause:** Forgot `await` in `__aexit__`
**Solution:** Always use `async with` AND `await` in exit
**Code Fix:**
\`\`\`python
async def __aexit__(self, *args):
    await self.connection.close()  # MUST await!
\`\`\`
**Affected Files:** src/database/session.py"
```

**When debugging similar issue:**
```bash
gemini chat "Read GOTCHAS.md. I'm getting 'Too many connections' error."
# AI immediately finds the async context manager solution
```

---

## Strategy 6: Example-Driven Context Engineering

### The Problem

You tell AI: "Use our error handling pattern."

AI's question: "What pattern? Can you describe it?"

You try: "We wrap routes in try-catch with specific exceptions..."

AI generates generic error handling that doesn't match your pattern.

---

### The Solution: Show, Don't Tell

**Example-Driven Context** = Provide concrete code examples instead of verbal descriptions.

**The Analogy:**

**Teaching a recipe:**
- ❌ **Bad:** "Fold the eggs gently with a spatula"
- ✅ **Good:** [Show video of proper folding technique]

**Same with AI:**
- ❌ **Bad:** "Use dependency injection in services"
- ✅ **Good:** [Show actual service code with dependency injection]

---

### Technique 1: Pattern Examples

**Example with Claude Code:**

```bash
claude-code prompt "Here are three examples of our service classes:

**Example 1: src/services/user_service.py**
\`\`\`python
class UserService:
    def __init__(self, db: Database, cache: Cache):
        self.db = db
        self.cache = cache
    
    async def create_user(self, data: UserCreate) -> User:
        # Implementation
\`\`\`

**Example 2: src/services/product_service.py**
\`\`\`python
class ProductService:
    def __init__(self, db: Database, cache: Cache):
        self.db = db
        self.cache = cache
    
    async def create_product(self, data: ProductCreate) -> Product:
        # Implementation
\`\`\`

**Example 3: src/services/order_service.py**
\`\`\`python
class OrderService:
    def __init__(self, db: Database, cache: Cache, payment: PaymentService):
        self.db = db
        self.cache = cache
        self.payment = payment
    
    async def create_order(self, data: OrderCreate) -> Order:
        # Implementation
\`\`\`

Study these three examples. Then implement PaymentService following this EXACT pattern:
- Same constructor signature style
- Same dependency injection
- Same async methods
- Same naming conventions"
```

**Benefits:**
- AI sees EXACTLY what you want
- No ambiguity
- Generated code matches perfectly

---

### Technique 2: Error Handling Examples

**Example with Gemini CLI:**

```bash
gemini chat "Here's our error handling pattern for API routes:

\`\`\`python
@router.post('/users')
async def create_user(data: UserCreate):
    try:
        user = await user_service.create_user(data)
        return {'status': 'success', 'data': user}
    except EmailAlreadyExistsError as e:
        raise HTTPException(400, str(e))
    except ValidationError as e:
        raise HTTPException(422, str(e))
    except Exception as e:
        logger.error(f'Unexpected error: {e}')
        raise HTTPException(500, 'Internal server error')
\`\`\`

Apply this EXACT pattern to the new payment endpoint:
- Same try/except structure
- Same custom exception handling
- Same generic exception catch-all
- Same logging format"
```

---

### Technique 3: Test Pattern Examples

**Example with Claude Code:**

```bash
claude-code prompt "Here's our test structure:

\`\`\`python
class TestUserService:
    @pytest.fixture
    async def user_service(self, mock_db):
        return UserService(mock_db, mock_cache)
    
    @pytest.mark.asyncio
    async def test_create_user_success(self, user_service):
        '''Test successful user creation.'''
        data = UserCreate(email='test@example.com')
        result = await user_service.create_user(data)
        
        assert result.email == 'test@example.com'
        assert result.id is not None
    
    @pytest.mark.asyncio
    async def test_create_user_duplicate_email(self, user_service):
        '''Test user creation with duplicate email.'''
        with pytest.raises(EmailAlreadyExistsError):
            await user_service.create_user(UserCreate(email='exists@example.com'))
\`\`\`

Create tests for ProductService using this EXACT structure."
```

**Key Principle:** One clear example is worth a thousand words of explanation.

---

## Strategy 7: Multi-Agent Context Architecture

### The Problem

You're building a complex feature that needs:
1. Architecture design
2. Backend implementation
3. Testing strategy
4. API documentation

Doing all in ONE AI session = mixed context, lower quality for each concern.

---

### The Solution: Specialized Agents

**Multi-Agent Architecture** = Use separate AI "agents" (sessions) with specialized contexts for different concerns.

**The Analogy:**

**Building a house:**
- Architect designs blueprint
- Contractor builds structure
- Electrician handles wiring
- Inspector validates quality
- Each specialist focuses on their expertise

**Same with AI:**
- Architecture Agent designs system
- Implementation Agent builds code
- Testing Agent creates tests
- Documentation Agent writes docs

---

### Agent 1: Architecture Agent

**Example with Claude Code:**

```bash
# Session 1: Architecture
claude-code prompt "You are a software architect.

Based on these requirements:
- Users can create posts
- Posts can have comments
- Comments can be nested (replies)
- Users can like posts and comments

Design:
1. Database schema with relationships
2. API endpoints structure
3. Service layer components
4. Data flow diagram

Don't implement - just design. Output as ARCHITECTURE.md"
```

**Output:** Complete architecture document.

---

### Agent 2: Implementation Agent

**Example with Gemini CLI:**

```bash
# Session 2: Implementation (separate session)
gemini chat "You are a backend developer.

Read ARCHITECTURE.md (the design from architect).

Implement Component 1: Post Management Service

Requirements:
- Follow the architecture exactly
- Use patterns from PATTERNS.md
- Ask questions if anything is unclear

Don't implement comments yet - just posts."
```

---

### Agent 3: Testing Agent

**Example with Claude Code:**

```bash
# Session 3: Testing (separate session)
claude-code prompt "You are a QA engineer.

Read:
- src/services/post_service.py (implementation)
- tests/test_user_service.py (test patterns)

Create comprehensive test suite for PostService:
1. Unit tests for each method
2. Integration tests for full flows
3. Edge case tests
4. Permission/security tests

Aim for 90%+ coverage."
```

---

### Agent 4: Documentation Agent

**Example with Gemini CLI:**

```bash
# Session 4: Documentation (separate session)
gemini chat "You are a technical writer.

Read:
- ARCHITECTURE.md (design)
- src/api/routes/posts.py (implementation)
- src/schemas/post_schema.py (API contracts)

Create:
1. API endpoint documentation
2. Request/response examples
3. Error codes and messages
4. Integration guide for frontend developers"
```

---

### Benefits of Multi-Agent Approach

✅ **Specialized Expertise:** Each agent focused on one concern  
✅ **Cleaner Context:** No mixing of architecture and implementation  
✅ **Better Quality:** Testing agent doesn't see implementation details (tests behavior, not implementation)  
✅ **Parallel Work:** Can delegate different agents to different developers

---

## Strategy 8: Just-In-Time Context Fetching

### The Problem

**Too Little Context:** AI doesn't have enough information → Multiple back-and-forth rounds  
**Too Much Context:** You pre-load everything → Context window fills, performance degrades

---

### The Solution: Lazy Loading with AI Questions

**Just-In-Time Fetching** = Let AI tell you what context it needs, then provide only that.

**The Analogy:**

**Research project:**
- ❌ **Bad:** Check out ALL library books on the topic (overwhelmed)
- ✅ **Good:** Browse catalog, request specific books as needed

**Same with AI:**
- ❌ **Bad:** Load all 50 files upfront
- ✅ **Good:** AI asks for specific files as it needs them

---

### Technique 1: AI-Driven Context Discovery

**Example with Claude Code:**

```bash
# Start minimal
claude-code prompt "I need to add social login (Google and GitHub) to our authentication system.

Don't start implementing yet. First, tell me:
1. What files do you need to read to understand current authentication?
2. What files do you need to understand OAuth integration?
3. What files do you need to see testing patterns?

List the files, and I'll approve which to read."
```

**AI responds:**
```
I need to see:

1. Current auth:
   - src/auth/authentication.py (main auth logic)
   - src/auth/jwt_handler.py (token management)
   - src/models/user.py (user structure)

2. OAuth:
   - Check if src/auth/oauth/ exists
   - requirements.txt (existing OAuth libraries)

3. Testing:
   - tests/test_auth.py (test patterns)

Should I read these 5-6 files?
```

**You approve:** "Yes, read those. Wait before reading more."

---

### Technique 2: Progressive Disclosure

**Example with Gemini CLI:**

```bash
# Start with AI asking questions
gemini chat "I want to implement a recommendation engine for products.

Start by asking me questions about:
1. What data we have available
2. What the recommendation logic should be
3. What constraints exist

Don't assume anything - ask first."
```

**AI asks:**
```
Questions:
1. Do we track user purchase history? User browsing history?
2. Should recommendations be personalized or general "trending" items?
3. Performance requirements? Real-time or batch processing?
4. Should I read product and order models first?
```

**You answer, then:** "Yes, read src/models/product.py and src/models/order.py"

**AI requests more:** "I also need to see how we're currently querying orders. Can I read src/repositories/order_repository.py?"

**You approve:** "Yes."

---

### Technique 3: Conditional Context Loading

**Example with Claude Code:**

```bash
claude-code prompt "Implement a shopping cart feature.

Read files conditionally:
1. Start by reading src/models/product.py
2. IF we have existing cart logic, read that
3. IF we have session management, read that
4. IF we have Redis, read cache patterns

Tell me what you found at each step before reading more."
```

**AI responds step-by-step:**
```
Step 1: Read src/models/product.py ✓
- Product has id, name, price, inventory_count

Step 2: Checking for existing cart logic...
- Found src/models/cart.py (incomplete implementation)
- Should I read it?

[You: Yes]

Step 3: Checking for session management...
- Found src/services/session_service.py
- Sessions stored in Redis
- Should I read it?

[You: Yes]

Ready to implement with this context.
```

---

## Putting It All Together: Real-World Scenario

Let's use ALL five advanced strategies for one complex task.

### Scenario: Add Multi-Tenant Support to Existing App

---

### Step 1: Context Curation (Strategy 4)

```bash
# Explicit file list
claude-code prompt "We're adding multi-tenant support (each customer is isolated).

Only read these critical files:
- src/models/user.py (to understand current user model)
- src/database/connection.py (to understand DB setup)
- src/api/middleware/auth.py (to understand current auth)

Total: 3 files. Don't read more without asking."
```

---

### Step 2: Load Memory Files (Strategy 5)

```bash
claude-code prompt "Before designing, read:
- DECISIONS.md (past architecture decisions)
- PATTERNS.md (code patterns we follow)

Then propose multi-tenancy approach."
```

---

### Step 3: Example-Driven Context (Strategy 6)

```bash
claude-code prompt "Here's our current User model:

\`\`\`python
class User:
    id: int
    email: str
    created_at: datetime
\`\`\`

Design Tenant model following the SAME style."
```

---

### Step 4: Multi-Agent Architecture (Strategy 7)

```bash
# Agent 1: Architecture (Claude)
claude-code prompt "Design multi-tenancy architecture"

# Agent 2: Implementation (Gemini with large context)
gemini chat --session tenant "Implement tenant isolation across all models"

# Agent 3: Testing (Claude)
claude-code prompt "Create tenant isolation tests"
```

---

### Step 5: Just-In-Time Fetching (Strategy 8)

```bash
# Let AI discover needed files
claude-code prompt "Implement tenant middleware.

Ask me for files as you need them. Don't assume."
```

**AI:** "I need src/api/middleware/auth.py to understand current middleware pattern"

**You:** "Read it"

**AI:** "Now I need to see how we access database - which file has our DB session?"

**You:** "src/database/session.py"

---

## Validation Checkpoint

### ✅ Can You Apply These Strategies?

**Scenario:** You're adding a payment processing feature.

**Which strategies would you use?**

1. **Context Curation** ✓ - Read only payment-related files, not entire codebase
2. **Memory Files** ✓ - Document decision to use Stripe vs PayPal in DECISIONS.md
3. **Example-Driven** ✓ - Show AI existing service patterns before implementing PaymentService
4. **Multi-Agent** ✓ - Separate agents for: implementation, security testing, PCI compliance docs
5. **Just-In-Time** ✓ - Let AI ask for Stripe SDK documentation when needed

**All five strategies working together!**

---

## Try With AI

Practice using advanced strategies.

### Exercise 1: Context Curation

```bash
# With Claude Code
claude-code prompt "I have a bug: users can see other users' private posts.

Don't read all files. Tell me which 3-5 files are most critical to read for debugging this permission issue.

Explain your reasoning for each file."
```

**Expected:** AI identifies permission-checking files, not unrelated code.

---

### Exercise 2: Memory Files

```bash
# With Gemini CLI
gemini chat "Create a DECISIONS.md file for a hypothetical blog project.

Include 3 architectural decisions:
1. Database choice
2. Authentication approach
3. File storage solution

Use the template from this lesson with Context, Decision, Alternatives, Consequences."
```

**Expected:** Properly formatted ADR document.

---

### Exercise 3: Multi-Agent Workflow

```bash
# With Claude Code
claude-code prompt "I'm building a comment system. Describe how I would use four specialized agents:

1. Architecture Agent - what does it do?
2. Implementation Agent - what does it do?
3. Testing Agent - what does it do?
4. Documentation Agent - what does it do?

Explain what each agent reads and produces."
```

**Expected:** Clear workflow with isolated contexts for each concern.




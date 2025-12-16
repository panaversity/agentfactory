---
title: Cursor AI Features and Workflows
chapter: 8
lesson: 5
learning_objectives:
  - Apply multi-turn conversations to solve iterative coding problems
  - Evaluate Cursor's diff editor for code review and modification
  - Compose Claude Rules for consistent AI collaboration patterns
  - Analyze context window management strategies
  - Design IDE workflows that leverage AI strengths
estimated_time: 75 minutes
skills:
  cursor-pro:
    proficiency: B1
  ai-collaboration:
    proficiency: B1
  prompt-composition:
    proficiency: B1
  workflow-design:
    proficiency: B1
generated_by: content-implementer v1.0.0
source_spec: specs/028-chapter-8-ai-native-ides/spec.md
created: 2025-11-20
last_modified: 2025-11-20
git_author: Claude Code
workflow: /sp.implement
version: 1.0.0
---

# Cursor AI Features and Workflows

You've learned the fundamentals of AI-native development and explored Cursor's interface. Now comes the most powerful aspect of AI-native IDEs: mastering the workflows that turn AI into a collaborative partner rather than a code-generation tool. In this lesson, you'll discover how to use Cursor's most advanced features—multi-turn conversations, diff editing, and context management—to solve real problems the way production teams do.

The shift from "ask AI to build something" to "iterate with AI toward better solutions" is where development speed genuinely accelerates. You'll see this through concrete examples where constraints refine outputs, feedback guides iteration, and conversation history captures decision-making patterns you can reuse.

## Understanding Multi-Turn Conversations

When you first used ChatGPT or Claude in a browser, you likely noticed something: asking a follow-up question refers back to previous messages. "You" means the assistant remembers what came before. This continuity is fundamental to AI collaboration, and Cursor brings this power into your development environment.

Multi-turn conversations in Cursor work like this: each message you send adds to a conversation history. The AI reads all previous messages (up to context limits) and understands the narrative of what you're building. This means you don't re-explain the entire problem each time—you refine, adjust, question, or extend previous outputs.

**Why this matters for development**: In traditional development, you might write a function once, then refactor it based on testing. With AI collaboration, you write the first draft TOGETHER with the AI through conversation. You catch issues earlier. You explore alternatives cheaply (in words, not debugging time). You document your decision-making process as you go.

### Conversation State Management

Every message in Cursor's chat interface affects what the AI "knows" about your project. This knowledge state is called context. Context includes:

- **Explicit context**: Files you've explicitly added with @filename
- **Implicit context**: The file currently open in the editor (Cursor automatically includes this)
- **Conversation history**: All previous messages in this chat thread
- **Model knowledge**: The AI's training data (knowledge of Python, web frameworks, etc.)

The AI combines these to generate relevant responses. If you change what files are in context, or add new messages with different constraints, the AI adapts accordingly.

**Example**: You start a chat asking for user authentication. Conversation goes like this:

```
Message 1 (You): "Create a user authentication system"
[AI generates basic password authentication]

Message 2 (You): "Good start. For MVP, we need this simpler—
just password hashing, no OAuth yet"
[AI refines: removes OAuth, keeps bcrypt hashing]

Message 3 (You): "We're using SQLite for MVP storage"
[AI adjusts database interactions for SQLite]

Message 4 (You): "Add rate limiting for login attempts"
[AI extends: adds rate limiting, still keeps previous changes]
```

Each message updated the context. By message 4, the AI knows: MVP scope, SQLite choice, authentication pattern preference, and is now adding rate limiting. The final code incorporates all decisions made through conversation.

This is fundamentally different from asking for "user authentication" once and getting a generic solution.

### Managing Conversation Threads

Cursor lets you create multiple chat threads within a single project. Each thread maintains its own conversation history and context window.

**Thread use cases**:

1. **Parallel exploration**: Thread A explores authentication, Thread B explores database schema simultaneously
2. **Problem isolation**: Use separate threads for different features (auth, payment, notifications)
3. **Conversation length management**: When one thread gets long, start a new thread (prevents context window overflow)
4. **Referencing decisions**: Look back at Thread A when making decisions in Thread B

**Workflow example**:
- **Thread 1: Authentication Design** - Develops login/registration flow
- **Thread 2: Database Schema** - Designs user table, password storage
- **Thread 3: Integration** - Connects authentication to database, validates decisions from threads 1-2

This threaded approach mirrors how teams work: parallel work streams that occasionally reference each other.

### Building Context Intentionally

You don't passively accept whatever context Cursor provides. You actively shape it. This is a critical skill.

**Intentional context management**:

1. **Add relevant files**: Use @filename to add files the AI should consider
   - @models.py (if discussing data structures)
   - @config.py (if discussing configuration)
   - But NOT @unused_legacy_code.py (noise)

2. **Reference conversation history**: "Earlier we decided on SQLite, remember?"
   - Forces AI to look back
   - Keeps decisions consistent

3. **Clarify constraints explicitly**: "For this iteration, focus only on the login flow, not password reset"
   - Prevents scope creep
   - Helps AI generate focused code

4. **Ask the AI to summarize context**: "What do you understand about our architecture so far?"
   - Validates that context is accurate
   - Catches misunderstandings early

**Example of intentional context**:

```
You (Message 5): "We now need API endpoints for authentication.
Remember our constraints from earlier:
- SQLite database (@models.py shows current schema)
- Rate limiting on login (implemented in auth.py)
- JWT tokens for stateless auth
Focus on the POST /login endpoint first"
```

This message explicitly references:
- Previous context ("Remember our constraints")
- Specific files (SQLite, current schema)
- Completed work (rate limiting, JWT implementation)
- Scoped focus ("POST /login endpoint first")

The AI now has clear parameters. The response will be more focused and consistent with previous decisions.

## Mastering the Diff Editor

The diff editor is where you stop being a code-generation consumer and become a code-review collaborator. Instead of accepting or rejecting AI-generated code wholesale, you examine the changes line-by-line, understand the reasoning, and make surgical edits.

### Understanding Diff Display

When the AI generates code changes, Cursor shows them in diff format:
- **Red lines**: Code being removed
- **Green lines**: Code being added
- **Unchanged context**: Gray background showing surrounding code

The diff view lets you see WHAT changed and WHERE, making it easy to verify that changes match your request.

**Example diff** (password hashing function):

```diff
def hash_password(password: str) -> str:
-    """Store password securely."""
-    return bcrypt.hashpw(password.encode(), bcrypt.gensalt()).decode()
+    """Hash password with bcrypt and return as string.
+
+    Args:
+        password: Plain text password from user
+
+    Returns:
+        Hashed password as string (safe for database storage)
+    """
+    salt = bcrypt.gensalt(rounds=12)
+    hashed = bcrypt.hashpw(password.encode(), salt)
+    return hashed.decode('utf-8')
```

What changed:
- ✅ Added detailed docstring (explains intent and examples)
- ✅ Explicit salt generation with 12 rounds (security improvement)
- ✅ Explicit encoding to UTF-8 (handles edge cases)
- ❌ Function logic stayed the same (already correct)

By examining the diff, you verify the changes are appropriate before accepting them.

### Accept, Reject, or Modify

The diff editor gives you three options:

**1. Accept (checkmark button)**
- Apply all changes from this diff
- Use when changes are correct and complete

**2. Reject (X button)**
- Discard all changes
- Use when approach is wrong or incomplete
- You can request a different approach in the chat

**3. Modify (edit directly in the diff view)**
- Accept most changes, but tweak specific lines
- Use when changes are 80% right but need refinement
- This is the most powerful option for collaboration

**Example modification workflow**:

AI generates:
```python
def verify_password(password: str, hashed: str) -> bool:
    return bcrypt.checkpw(password.encode(), hashed.encode())
```

You examine the diff and notice: `hashed.encode()` is wrong. The hashed password is ALREADY a string (from hash_password), not bytes. You modify directly:

```python
def verify_password(password: str, hashed: str) -> bool:
    return bcrypt.checkpw(password.encode(), hashed)  # Remove .encode() on hashed
```

Then accept. You've just fixed a bug that AI made, without rejecting the entire solution. This is collaborative code review.

### Using Diffs for Code Review

The diff editor is also your code review tool. When reviewing AI-generated code:

1. **Read the diff top-to-bottom**: What's being added/removed?
2. **Check for bugs**: Logic errors, missing error handling, security issues?
3. **Verify style consistency**: Does code match your project's conventions?
4. **Examine imports and dependencies**: Did the AI add necessary imports?
5. **Test mentally**: Does the change do what was requested?

**Code review checklist for diffs**:
- [ ] Changes match the request?
- [ ] No new bugs introduced?
- [ ] Error handling present (try/except, None checks)?
- [ ] Type hints correct?
- [ ] Docstrings accurate?
- [ ] Imports added correctly?
- [ ] No unused code?
- [ ] Performance reasonable (no obvious inefficiencies)?

If all checks pass → Accept. If some fail → Modify or Reject and clarify the problem in chat.

## Designing Effective Prompts

A prompt is your request to the AI. Its quality directly affects the response quality. Poor prompts generate poor code; clear prompts generate useful code.

### Specification-First Prompts

Before asking for implementation, specify WHAT you want, not just HOW to build it.

**Poor prompt** (implementation-focused):
```
"Create a function to validate email"
```
→ The AI guesses: regex validation? Sending verification emails? Checking format only?

**Effective prompt** (specification-focused):
```
"Create an email validation function that:
- Checks for valid email format (RFC 5322 simplified)
- Returns True/False
- Handles edge cases (empty string, None)
- Does NOT make external requests (local validation only)
- Should be suitable for form field validation

What would be a good approach?"
```

The second prompt specifies intent, constraints, and non-goals. The AI response addresses all of them.

### Constraint-Based Refinement

Constraints are your power tool for guiding AI toward production solutions.

**Example constraint refinement**:

```
Message 1: "Create user registration endpoint"
[AI generates basic Flask endpoint]

Message 2: "Good start. Let's add constraints:
- Maximum 1 registration per IP address per hour
- Password must be 8+ characters with uppercase
- Email must be verified before activation
Does this change the approach?"
```

The constraints reframe the problem. Rate limiting, validation requirements, and email verification are now part of the design, not afterthoughts.

### Context Priming

Before asking for code, prime the AI with context about your project.

**Priming technique**:
```
"I'm building a Flask API with these patterns:
- @auth_required decorator for protected routes
- SQLAlchemy ORM for database
- Pydantic models for validation
- Return {"status": "success", "data": {...}} format

Now, create a user registration endpoint following these patterns"
```

By establishing patterns first, the AI generates code that matches your existing codebase. You don't get generic solutions—you get project-specific solutions.

### Asking for Iterations, Not Perfection

AI rarely generates perfect code on first try. This isn't a limitation; it's the collaboration model.

**Effective iteration prompts**:

```
Message 1: "Create authentication service"
[AI generates basic implementation]

Message 2: "This is good. Now let's make the password
validation stricter. Current rules:
- 8+ characters, 1 uppercase, 1 number

New rules:
- 12+ characters, 1 uppercase, 1 number, 1 special char
Please update the validation and add tests"
```

Each message asks for incremental improvement, not a complete rewrite.

### Using Claude Rules for Consistency

Claude Rules are instructions you write once and apply to all conversations with Claude in Cursor. They encode your project patterns, team conventions, and collaboration preferences.

**Example Claude Rules** (saved in `.cursor/rules`):

```markdown
# Project Coding Standards

## Python Style
- Use type hints for all function parameters and returns
- Docstrings follow Google style
- Variable names are descriptive (user_id not uid)
- Maximum line length: 100 characters

## Error Handling
- All external API calls wrapped in try/except
- Database queries check for None/empty results
- Form validation returns {'error': 'message'} on failure

## Testing
- All functions have corresponding tests
- Test names describe what's being tested (test_valid_email not test_1)
- Use pytest fixtures for common test setup

## API Responses
- All endpoints return {"status": "success/error", "data": {...}, "message": "..."}
- HTTP status codes are appropriate (201 for created, 400 for bad input)
- All responses include timestamp

## Security
- All passwords hashed with bcrypt (minimum 12 rounds)
- SQL queries use parameterized queries (no f-strings)
- Rate limiting on all public endpoints
```

When the AI sees these rules, it generates code matching your standards automatically. You don't have to repeat "add type hints" in every prompt.

### Example: Building an Authentication Service Through Conversation

Let's see a full conversation demonstrating multi-turn development:

**Message 1 (Specification)**:
```
I'm building user authentication for a Flask app. Here's what I need:

Requirements:
- User registration (email + password)
- User login (returns JWT token)
- Password must be 12+ characters (1 uppercase, 1 number, 1 special char)
- Rate limiting: Max 5 login attempts per hour per IP
- Passwords stored with bcrypt (12 rounds)

Database: SQLite with SQLAlchemy
Auth: JWT tokens with 24-hour expiry

Can you create the core authentication service?
```

**AI Response**: [Generates auth.py with User model, registration function, login function, JWT token generation]

**Message 2 (Constraint Refinement)**:
```
Good! Before implementing, let me add one constraint:
- Registered users should receive a verification email
- Login should fail if email not verified
- Unverified accounts expire after 24 hours

Does this change the database schema?
```

**AI Response**: [Updates User model to add verified, email_token, email_token_expiry columns]

**Message 3 (Security Review)**:
```
I see the password storage uses bcrypt.gensalt(). What rounds
are you using by default? The requirement is 12 rounds minimum.
Also, is the JWT secret stored securely (not hardcoded)?
```

**AI Response**: [Reviews code, confirms gensalt() defaults to 12 rounds, updates JWT secret to read from environment variable]

**Message 4 (Integration)**:
```
Now let's create the Flask routes:
- POST /register (email, password)
- POST /login (email, password) → returns JWT token
- POST /verify-email (token) → marks email as verified
- GET /me (requires auth) → returns current user

Follow the response format we discussed in Claude Rules
```

**AI Response**: [Generates Flask routes with @auth_required decorator, proper error handling, consistent response format]

Through four messages, the authentication service evolved from initial concept to production-ready implementation. Each message refined based on constraints, security review, and integration needs. This is collaborative development.

## Context Window Management

Context window is the AI's "working memory"—the total amount of conversation it can read and understand. Cursor uses Claude 3.5 Sonnet by default (200K context window), which is large, but not infinite. Large projects with long conversations can hit limits.

### Understanding Context Consumption

Context is consumed by:
- **Files you add**: @models.py adds the entire file to context
- **Conversation history**: Every message you and the AI exchange
- **Editor content**: The currently open file is auto-included
- **Model knowledge**: Training data (doesn't count toward limits, but affects quality)

A typical conversation:
```
You: "Create function" (50 tokens) → AI: [500-token response]
You: Follow-up (100 tokens) → AI: [800-token response]
Total context used: ~1,450 tokens from this conversation alone
```

Add a 5KB file (@models.py): ~1,200 tokens. Your 2-message conversation is now 2,650 tokens of context.

### Optimizing Context Usage

**Strategy 1: Avoid adding unnecessary files**
- Only add files the AI actually needs
- ❌ Don't add entire project at once
- ✅ Add specific files: @auth.py, @models.py, @config.py

**Strategy 2: Start new threads for long conversations**
- After 50+ messages, conversation context might be substantial
- Start a new thread to "reset" and avoid context bloat
- Reference previous thread: "In Thread 1, we decided on SQLite storage"

**Strategy 3: Summarize before long discussions**
- "Here's what we've decided so far: [bullets]"
- The AI reads this summary instead of re-reading all previous messages
- Saves context tokens

**Strategy 4: Remove context you don't need**
- You can "uncheck" files in Cursor's context panel
- Removes them from the conversation without losing chat history
- Useful when a file was needed for one discussion but not the next

**Example context optimization**:

```
Conversation started: Context includes auth.py, models.py, config.py

After 10 messages discussing authentication...
→ Models.py no longer needed, uncheck it
→ Context reduced, freed up for new additions

Next discussion (database schema)...
→ Check models.py again (needed for this discussion)
→ Uncheck auth.py (not needed for schema work)
```

### Context Window Limits and Recovery

If you hit the context limit:
- The AI will tell you explicitly: "I'm reaching my context limit..."
- Common solutions:
  1. Start a new thread (simplest)
  2. Remove non-essential files from context
  3. Ask the AI for a summary of decisions made so far
  4. Continue in new thread with summary as reference

This isn't a disaster—it's a natural workflow point. Long projects benefit from checkpoint conversations anyway.

## Exploring the Tab Context Window

Beyond the chat interface, Cursor includes a "Tab Context" feature that shows you exactly what the AI can "see" in your project. This transparency is powerful for understanding how the AI is context-aware.

### What Tab Context Shows

When you open a file and interact with the AI, Cursor shows you:
- Which files are included in the current context
- Size of context (tokens, approximate)
- Which parts of files are visible
- What the AI "knows" about your project

This is your debugging tool: if the AI gives a response that ignores something important, you can check Tab Context to see if that file was actually included.

**Example scenario**:

You ask: "Should I change the User model?"
AI responds: "I don't see a User model yet..."

You check Tab Context and realize: @models.py wasn't included. Add it to context, ask again. The AI now sees the model and gives relevant feedback.

## Exercises: Mastering Cursor Workflows

These exercises teach you through doing—you'll build small features while practicing multi-turn conversation, diff review, and prompt crafting.

### Exercise 1: Multi-Turn Authentication (Beginner)

**Objective**: Use multi-turn conversation to iteratively build an authentication function.

**Setup**:
1. Create a new file: `auth.py`
2. Open Cursor's chat
3. Add this context to your prompt:
   ```
   Project: User authentication for Flask app
   Database: SQLite with SQLAlchemy
   Constraints:
   - Passwords must be 10+ characters
   - Use bcrypt for hashing
   - Return success/failure as boolean
   ```

**Task**:

Message 1: "Create a password hashing function that takes a plain text password and returns a hashed version suitable for database storage."

[Examine AI response, accept if good, or request changes]

Message 2: "Now create a password verification function that compares a plain text password against the hashed version stored in the database."

[Review the diff: Did the AI use bcrypt.checkpw correctly?]

Message 3: "Add a constraint: The hashing function must use bcrypt with 12 rounds. Is your current implementation meeting this requirement?"

[This asks the AI to audit its own work—good validation practice]

**Deliverable**: auth.py with two functions (hash_password, verify_password) that work together.

### Exercise 2: Diff Review and Modification (Intermediate)

**Objective**: Practice reviewing diffs and making targeted edits.

**Setup**:
1. Create: `models.py` with a simple User class
2. Prompt the AI: "I have a User model. Please add email validation to ensure emails are unique in the database."

**Task**:

When the AI generates changes:
1. **Don't accept immediately**. Review the diff line-by-line:
   - What's being added?
   - Does it use SQLAlchemy uniqueness constraints correctly?
   - Are there any queries that might be slow?

2. **Make a modification**: Even if the code looks good, practice editing one line in the diff:
   - Change a variable name for clarity
   - Add a comment explaining a complex line
   - Adjust an error message for consistency

3. **Accept the modified diff**

**Deliverable**: Modified diff showing both AI-generated code and your improvements.

### Exercise 3: Constraint-Based Refinement (Intermediate)

**Objective**: Use constraints to guide AI toward a production-quality solution.

**Setup**:
1. Prompt: "Create a simple login endpoint for Flask"
2. AI generates a basic endpoint

**Task**:

**Message 1**: "This endpoint needs three new constraints:
- Rate limiting: Max 5 login attempts per IP per hour
- Password requirements: 12+ chars, 1 uppercase, 1 number
- Response format: Always return `{"status": "success"|"error", "data": {...}, "message": "..."}`

Does this require changes to the endpoint?"

[The AI explains what needs to change]

**Message 2**: "Good. Before implementing, let me add one more: failed login attempts should be logged. Does that affect the architecture?"

[The AI considers the implication]

**Message 3**: "Now implement the updated endpoint with all constraints."

[Review the final code]

**Deliverable**: Complete login endpoint meeting all specified constraints.

### Exercise 4: Claude Rules Application (Intermediate)

**Objective**: Create and apply Claude Rules for consistent code generation.

**Setup**:
1. Create `.cursor/rules.md` in your project
2. Add these rules:
   ```markdown
   # Project Standards

   ## Python
   - Type hints required for all functions
   - Google-style docstrings
   - Maximum line length: 88 characters (Black formatter)

   ## Error Handling
   - All external calls wrapped in try/except
   - Specific exception types (not generic Exception)
   - Return {"error": "message"} on failure

   ## Testing
   - All functions have tests
   - Test names are descriptive
   - Use pytest fixtures
   ```

3. In Cursor settings, set `.cursor/rules.md` as the rules file

**Task**:

Request a feature WITHOUT mentioning your standards:
```
"Create a function that fetches user data from an external API"
```

Compare the AI response with your rules. Does it follow them?

Request again with this message:
```
"Create a function that fetches user data from an external API.
Make sure it follows the project standards in .cursor/rules.md"
```

Compare the two responses. The second should better match your rules.

**Deliverable**: Side-by-side comparison showing how rules improved code quality.

### Exercise 5: Context Window Optimization (Advanced)

**Objective**: Manage context in a moderately complex project.

**Setup**:
1. Create files: auth.py, models.py, routes.py, config.py (4 files total)
2. Start a multi-message conversation about authentication:
   - Message 1: "Review my auth.py. What security improvements would you suggest?"
   - Message 2: Follow-up refinements

**Task**:

1. **Track context**: Notice which files are included in Tab Context
2. **Optimize mid-conversation**:
   - After message 2, remove models.py from context (not needed for auth discussion)
   - Note the context reduction
3. **Switch focus**:
   - Message 3: Start discussing database schema
   - Check: Do you need to re-add models.py? Should you remove auth.py?
4. **Summarize**:
   - Ask the AI: "Summarize the key decisions we've made about authentication so far"
   - Start a new thread with this summary as reference

**Deliverable**: Screenshot showing context management and thread switching.

### Exercise 6: Specification Prompts vs. Implementation Prompts (Intermediate)

**Objective**: Learn the difference between asking "what" vs. "how".

**Setup**:
Create a test file for each prompt type and record the results.

**Task**:

**Scenario A** (implementation prompt - weaker):
```
Prompt: "Create a user registration function"
[Observe: Generic response, misses some requirements]
```

**Scenario B** (specification prompt - stronger):
```
Prompt: "Create a user registration function with these requirements:
- Input: email, password (string)
- Output: {'status': 'success/error', 'user_id': int or null}
- Validation: Email format RFC 5322, password 12+ chars
- Security: Bcrypt hashing, rate limiting 5 reqs/hour/IP
- Storage: SQLite via SQLAlchemy
- Non-goals: Email verification (separate feature)

What would be a good approach, considering these requirements?"
```

[Observe: More focused response, addresses constraints]

**Compare**: Which prompt got you closer to production code?

**Deliverable**: Both prompts, responses, and analysis of differences.

### Exercise 7: Error Detection in Diffs (Advanced)

**Objective**: Develop your code review skill by finding bugs in AI-generated code.

**Setup**:
Ask for a function with a subtle requirement:
```
"Create a function to verify passwords. It should:
- Take a password (string) and hashed password (string) from database
- Use bcrypt.checkpw() to verify
- Return True if match, False otherwise
- Handle exceptions gracefully"
```

**Task**:

1. **Generate the code**: Let AI create the function
2. **Review the diff carefully**: Look for this common bug:
   - AI might do: `bcrypt.checkpw(password.encode(), hashed.encode())`
   - The bug: `hashed` is already a string from the database, shouldn't be encoded twice
3. **Fix the bug**: Use the diff modification feature to correct it
4. **Explain to the AI**: "I found an issue. The hashed password is already stored as a string, so encoding it a second time causes a failure. Here's the fix:"

**Deliverable**: Before/after diffs showing the bug and your correction.

### Exercise 8: Building a Mini-Project Through Conversation (Advanced)

**Objective**: Complete a small feature entirely through multi-turn conversation.

**Project**: Simple TODO API with authentication

**Requirements**:
- User registration/login
- Create/read/update/delete TODOs
- TODOs belong to users (privacy)
- Password validation, rate limiting

**Task** (Follow this conversation flow):

**Message 1**: "I'm building a TODO API. Here are the core requirements: [list above]"
→ AI suggests architecture (models, endpoints)

**Message 2**: "Let's start with the User model and authentication. Here are our patterns: [add Claude Rules]"
→ AI generates auth-related models

**Message 3**: "Now authentication routes: POST /register, POST /login"
→ AI generates Flask routes

**Message 4**: "Now the TODO model and routes. TODOs should only be visible to their owner."
→ AI generates privacy-aware TODO model

**Message 5**: "Create tests for the authentication routes"
→ AI generates pytest-based tests

**Deliverable**: Complete API implementation (models.py, routes.py, tests.py) built collaboratively through conversation.

### Mini-Project: Authentication + API Rate Limiting (Capstone)

**Objective**: Integrate everything you've learned into a production-quality feature.

**Requirements**:
```
Build a user authentication system with these constraints:

User Registration:
- Email must be valid format
- Password 12+ chars (1 uppercase, 1 number, 1 special)
- Rate limit: 5 registrations per IP per day
- Return: {"status": "success", "user_id": int} or error

User Login:
- Valid credentials required
- Rate limit: 5 failed attempts per hour → account lockout
- Returns JWT token (24-hour expiry)
- Return: {"status": "success", "token": string} or error

Constraints:
- Use bcrypt (12 rounds minimum)
- SQLite database
- Follow project standards (Claude Rules)
- All functions tested with pytest
```

**Deliverables**:
1. **auth.py** - Authentication logic (functions, no routes)
2. **models.py** - User model with SQLAlchemy
3. **routes.py** - Flask routes for registration, login
4. **test_auth.py** - Comprehensive tests
5. **IMPLEMENTATION_LOG.md** - Document your multi-turn conversation:
   - What constraints changed the architecture?
   - What security issues did you discover?
   - What iterations refined the solution?

**Process**:
1. Start with specification message (all requirements above)
2. Have the AI suggest the approach
3. Iterate 4-5 times, adding constraints/refinements
4. Build tests as you go
5. Document the conversation in IMPLEMENTATION_LOG.md

**Success Criteria**:
- All endpoints working and tested
- Password hashing uses bcrypt with 12+ rounds
- Rate limiting actually prevents abuse
- Code follows your Claude Rules
- Conversation demonstrates iterative refinement (not one-shot generation)

## Orchestrating Complex Features With Cursor

Large features often require multiple components: models, API routes, tests, frontend integration. Cursor's multi-threaded chat helps you orchestrate this complexity.

### Thread-Based Orchestration

Instead of one massive conversation, use separate threads for different concerns:

**Example: Building a payment feature**

- **Thread 1: Data Model Design** - Design Subscription, Payment, Billing models
- **Thread 2: Payment Processing** - Integrate Stripe API, handle webhooks
- **Thread 3: API Routes** - Implement endpoints for subscription management
- **Thread 4: Frontend Integration** - Connect React components to API
- **Thread 5: Testing** - Comprehensive test coverage

Each thread focuses on one concern. When you need to reference decisions from another thread, you do so explicitly:

```
Thread 3 (Routes):
"In Thread 1, we designed the Subscription model with status: active/cancelled/expired.
When building the GET /subscriptions endpoint, should we filter by status?
If so, what's the most common use case?"
```

This threaded approach prevents any single conversation from becoming unwieldy.

### Keeping Decisions Visible

When a conversation makes architectural decisions, capture them immediately:

```
Thread 1 concludes with:
"DECISION: Store subscription state as Enum(active, cancelled, expired) on Subscription model"
```

Copy this to a DECISIONS.md file at the project root. Future conversations reference this file:

```
@DECISIONS.md

"Before designing the API routes, here are the architectural decisions
from Thread 1..."
```

This ensures new threads see previous decisions and maintain consistency.

## Debugging With the AI

Cursor is particularly powerful for debugging: the AI can see your error message, code, and context simultaneously.

### Effective Debugging Workflow

**Step 1: Paste the error**
```
Chat: "I'm getting this error:

TypeError: 'NoneType' object is not subscriptable
File "auth.py", line 45, in verify_token
    payload = jwt.decode(token, SECRET_KEY, algorithms=['HS256'])

Line 45 is: payload = jwt.decode(...)
But I'm not sure why it would be None"
```

**Step 2: Add relevant code**
```
@auth.py

"Can you see why verify_token is failing?"
```

**Step 3: Provide context**
```
"The token is generated in login() on line 30.
The error happens when I try to use it in verify_token()."
```

**Step 4: AI suggests the issue**
The AI examines the code, sees the problem (token might be None if login fails), and suggests a fix.

**Step 5: Review and apply**
You review the diff, verify it addresses the root cause, accept.

## Maintaining Code Quality Through AI Partnership

Just because AI generated code doesn't mean you stop thinking. Maintain quality by:

1. **Code reviewing every diff** - Don't auto-accept
2. **Testing early** - Run code immediately, catch issues
3. **Questioning assumptions** - If something seems off, ask the AI to explain
4. **Referencing standards** - Keep Claude Rules visible and enforced
5. **Documenting decisions** - Track architectural choices as you make them

## Try With AI

**Setup**: Open Cursor and create a new project (or use an existing one).

**Prompt Set 1 (Building Specification)**:

```
I'm building a user authentication system. Here are my constraints:
- Flask API (not frontend)
- Password validation: 12+ characters, 1 uppercase, 1 digit
- Use bcrypt for storage
- JWT tokens for authentication (24-hour expiry)
- Rate limiting: Max 5 login attempts per hour per IP

What would be your suggested approach? Should I start with models, routes, or helper functions?
```

Expected outcome: AI suggests a logical architecture (models → helpers → routes). The response should reference your specific constraints.

**Prompt Set 2 (Iterative Refinement)**:

```
Good approach. Before we implement, one addition:
- Registered users need email verification
- Unverified accounts should expire after 48 hours

Does this require changes to the User model schema?
```

Expected outcome: AI acknowledges the new constraint, explains schema changes needed (new columns like email_verified, verification_token, etc.).

**Prompt Set 3 (Specification to Implementation)**:

```
Now let's implement. Create the User model with SQLAlchemy that supports:
- Email, password_hash, email_verified, verification_token, verification_expiry
- Add validation: Ensure email is unique, password hash uses bcrypt

Then create helper functions: hash_password(), verify_password(), generate_verification_token()
```

Expected outcome: Complete, production-quality code with type hints and docstrings.

**Prompt Set 4 (Code Review)**:

When the AI provides code:
1. Don't accept immediately
2. Review the diff carefully
3. Examine the bcrypt usage: Is it using 12 rounds?
4. Check the datetime handling: Is verification_expiry calculated correctly?
5. Look for security: Are SQL queries parameterized?

Use this review to ask:
```
I see you're using bcrypt.gensalt() without specifying rounds.
What's the default? The requirement is 12 rounds minimum.
```

Expected outcome: AI clarifies (default is 12), or you request an adjustment.

**Safety Note**: When building authentication, test password hashing locally before deploying. Weak hashing is a security vulnerability that AI sometimes misses.

**Optional Stretch**: Add rate limiting logic to your authentication system. Ask the AI: "How should I store failed login attempts to implement rate limiting? In-memory cache, Redis, or database?"

---

**Next lesson**: You'll learn AI-native debugging techniques and how to use Cursor to find and fix bugs faster than traditional debugging workflows.

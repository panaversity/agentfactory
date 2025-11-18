---
title: "Question-Driven Development"
chapter: 10
lesson: 5
part: 3
duration_minutes: 35

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Question-Driven Development"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student prompts AI to ask clarifying questions before implementation"

learning_objectives:
  - objective: "Prompt AI to identify specification gaps through clarifying questions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Generate requirement questions for documentation task"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (QDD, clarifying questions, specification gaps, assumption discovery, requirement elicitation, iterative refinement) at B1 threshold ✓"
---

# Question-Driven Development

Here's a hidden AI capability most developers never use: **asking AI to ask YOU questions**.

Most developers prompt like this:

> "Create documentation for the authentication API."

AI generates something. It's generic. It doesn't match your project. You spend 20 minutes refining it.

**The problem**: AI made assumptions about what you needed. Those assumptions were wrong.

**The fix**: Prompt AI to **ask you clarifying questions BEFORE implementing**.

This lesson teaches Question-Driven Development (QDD): a technique where you explicitly request AI to identify specification gaps and ask questions that expose your hidden requirements.

By the end of this lesson, you'll transform AI from "assumption maker" into "requirements analyst" who helps YOU discover what you actually need before any code gets written.

---

## The Hidden Cost of Assumptions

When you give AI an underspecified prompt, it makes assumptions to fill gaps:

**Your prompt**:
```
"Create documentation for the authentication API."
```

**AI's silent assumptions**:
- Audience = developers (not end users, not admins)
- Format = markdown (not OpenAPI, not JSDoc)
- Depth = complete reference (not quick start, not tutorial)
- Auth type = JWT (not OAuth, not session-based)
- Examples = curl commands (not JavaScript, not Python)

**Each assumption has ~60% chance of being wrong.**

If 5 assumptions, probability all correct = 0.6^5 = **7.8% chance AI guesses everything right**.

**Result**: You spend 10-15 iterations correcting assumptions AI should have asked about.

---

## Question-Driven Development (QDD)

**Core principle**: Before AI implements, AI asks questions that expose specification gaps.

**QDD workflow**:

```
1. You: Give initial prompt (intentionally underspecified)
2. AI: Identifies gaps, asks 5-8 clarifying questions
3. You: Answer questions (reveals your hidden requirements)
4. AI: Generates specification based on your answers
5. You: Validate specification, make corrections
6. AI: Implements against validated specification
```

**Benefit**: Assumptions made EXPLICIT through questions. Correct first implementation dramatically more likely.

---

## QDD Prompt Structure

Here's the meta-prompt that activates Question-Driven Development:

```markdown
I need [general description of task].

Before implementing, ask me 5-8 clarifying questions about:
- Audience and use case
- Technical constraints
- Output format and structure
- Success criteria
- What I don't need (non-goals)

Based on my answers, create a specification document, then implement.
```

**Key components**:

1. **"Before implementing"** → Prevents AI from jumping to code
2. **"5-8 clarifying questions"** → Specific count prevents endless questions
3. **Question categories** → Guides AI toward useful questions (not trivial ones)
4. **"Create specification"** → Ensures structured requirements capture
5. **"Then implement"** → Workflow includes implementation after clarification

---

## Example: Documentation Generation with QDD

Let's see QDD in action for API documentation.

### Step 1: Initial Prompt (Intentionally Underspecified)

```markdown
I need documentation for our authentication API endpoints.

Before implementing, ask me 5-8 clarifying questions about:
- Audience and use case
- Technical constraints
- Output format and structure
- Success criteria
- What I don't need (non-goals)

Based on my answers, create a specification document, then generate the documentation.
```

### Step 2: AI Asks Clarifying Questions

AI responds with:

```markdown
Before I create the documentation, I need to understand your requirements better:

**Audience & Use Case:**
1. Who is the primary audience? (frontend developers, backend developers, QA engineers, external API consumers?)
2. What's their assumed knowledge level? (familiar with REST APIs? with your auth system specifically?)

**Technical Details:**
3. What authentication method(s) does your API use? (JWT, OAuth 2.0, API keys, session-based?)
4. Which endpoints should be documented? (login, logout, refresh, register, password reset - all or subset?)

**Format & Structure:**
5. What format do you prefer? (Markdown, OpenAPI/Swagger, JSDoc, HTML?)
6. Do you need interactive examples? (code snippets, curl commands, Postman collection?)

**Success Criteria:**
7. What should a developer be able to do after reading this documentation without asking questions?

**Non-Goals:**
8. What should I NOT include? (implementation details, security architecture, rate limiting, error handling specifics?)
```

**Notice**:
- Questions are **specific** (not "what do you want?")
- Questions are **categorized** (matches your prompt structure)
- Questions **expose assumptions** AI would have made (JWT vs OAuth, markdown vs OpenAPI)

### Step 3: You Answer Questions

You respond:

```markdown
1. Primary audience: Frontend developers integrating our auth into React apps
2. Knowledge level: Familiar with REST APIs, but first time using our system
3. Authentication method: JWT with refresh tokens
4. Endpoints: /login, /logout, /refresh (NOT register/password reset - separate docs)
5. Format: Markdown with JavaScript fetch() examples
6. Interactive examples: Yes - copy-pasteable fetch() code for each endpoint
7. Success criteria: Developer can integrate auth without reading our backend code
8. Non-goals: Don't document error handling (separate guide), don't explain JWT internals
```

### Step 4: AI Creates Specification

Based on your answers, AI generates:

```markdown
SPECIFICATION: Authentication API Documentation

WHAT: Developer guide for integrating JWT authentication into React apps

AUDIENCE:
  - Frontend developers (React)
  - Familiar with REST APIs
  - First-time users of our auth system

SCOPE:
  - Endpoints: /login, /logout, /refresh
  - Excluded: registration, password reset (separate docs)

FORMAT:
  - Markdown document
  - JavaScript fetch() examples for each endpoint
  - Copy-pasteable code snippets

STRUCTURE:
  1. Quick Start (authentication flow overview)
  2. Endpoint Reference
     - POST /login (email + password → access token + refresh token)
     - POST /logout (invalidate tokens)
     - POST /refresh (refresh token → new access token)
  3. Common Integration Patterns
     - Storing tokens in React
     - Attaching tokens to requests
     - Handling token expiration

SUCCESS CRITERIA:
  - Developer can integrate auth without reading backend code
  - All code examples are copy-pasteable and working
  - Authentication flow clear from examples

NON-GOALS:
  - Error handling details (separate guide)
  - JWT internals explanation
  - Security best practices (separate guide)
```

**What just happened**:
- Your answers → specific requirements
- AI transformed answers → structured specification
- Specification ready for validation BEFORE implementation

### Step 5: You Validate Specification

You review and correct:

```markdown
Specification looks good. One correction:

STRUCTURE section - add:
  4. Token Refresh Strategy (when to call /refresh, how to handle expired tokens)

Otherwise approved. Proceed with documentation.
```

### Step 6: AI Implements Against Validated Specification

Now AI generates documentation matching your exact requirements:

```markdown
# Authentication API Guide

Quick integration guide for adding JWT authentication to your React application.

## Authentication Flow

1. User logs in → Receive access token (15 min expiry) + refresh token (7 day expiry)
2. Include access token in API requests
3. When access token expires → Use refresh token to get new access token
4. When refresh token expires → User must log in again

---

## API Endpoints

### POST /login

**Purpose**: Authenticate user and receive tokens

**Request**:
```javascript
const response = await fetch('https://api.example.com/auth/login', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    email: 'user@example.com',
    password: 'userpassword'
  })
});

const data = await response.json();
// data = { accessToken: "...", refreshToken: "...", expiresIn: 900 }
```

**Response**:
- `accessToken` (string): JWT token for API requests (15 min expiry)
- `refreshToken` (string): Token for refreshing access token (7 day expiry)
- `expiresIn` (number): Seconds until access token expires

**Storage**:
```javascript
localStorage.setItem('accessToken', data.accessToken);
localStorage.setItem('refreshToken', data.refreshToken);
```

---

### POST /logout

**Purpose**: Invalidate user's tokens

**Request**:
```javascript
const response = await fetch('https://api.example.com/auth/logout', {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
  }
});

// Clear local storage
localStorage.removeItem('accessToken');
localStorage.removeItem('refreshToken');
```

---

### POST /refresh

**Purpose**: Get new access token using refresh token

**Request**:
```javascript
const response = await fetch('https://api.example.com/auth/refresh', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    refreshToken: localStorage.getItem('refreshToken')
  })
});

const data = await response.json();
// data = { accessToken: "...", expiresIn: 900 }

// Update stored access token
localStorage.setItem('accessToken', data.accessToken);
```

---

## Common Integration Patterns

### Attaching Tokens to Requests

```javascript
async function authenticatedFetch(url, options = {}) {
  const token = localStorage.getItem('accessToken');

  const response = await fetch(url, {
    ...options,
    headers: {
      ...options.headers,
      'Authorization': `Bearer ${token}`
    }
  });

  return response;
}

// Usage
const data = await authenticatedFetch('https://api.example.com/user/profile');
```

### Token Refresh Strategy

**When to refresh**:
- Proactive: Refresh when access token has \<2 minutes remaining
- Reactive: Refresh when API returns 401 Unauthorized

**Proactive approach**:
```javascript
function isTokenExpiringSoon() {
  const expiresIn = parseInt(localStorage.getItem('tokenExpiresIn'));
  return expiresIn < 120; // Less than 2 minutes
}

// Before important requests
if (isTokenExpiringSoon()) {
  await refreshAccessToken();
}
```

**Reactive approach**:
```javascript
async function authenticatedFetch(url, options = {}) {
  let response = await fetch(url, {
    ...options,
    headers: {
      ...options.headers,
      'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
    }
  });

  // Token expired - refresh and retry
  if (response.status === 401) {
    await refreshAccessToken();
    response = await fetch(url, options); // Retry with new token
  }

  return response;
}
```
```

**Result**: Documentation matches your exact requirements. First implementation ~95% correct because specification captured all requirements upfront.

---

## The Three Roles in QDD

Notice how Question-Driven Development demonstrates all three AI collaboration roles:

### Role 1: AI as Teacher

AI teaches you **what questions to consider**:

> "What authentication method(s) does your API use? (JWT, OAuth 2.0, API keys, session-based?)"

**You learn**: "Oh, I need to specify auth method—AI can't assume JWT."

### Role 2: AI as Student

AI learns **your project-specific requirements**:

> You: "Audience is React developers, first-time users of our system"
>
> AI: *Adjusts documentation depth, includes React-specific storage examples*

**AI adapts**: Documentation level and code examples match your context.

### Role 3: AI as Co-Worker

Neither you nor AI has complete picture initially:
- **You know**: What exists (endpoints, auth method)
- **AI knows**: What needs documenting (structure, examples developers expect)

**Together**: Through questions → answers → specification, you converge on complete requirements.

**Result**: Better documentation than either could produce alone.

---

## QDD for Different Development Tasks

QDD works for any underspecified task. Here are meta-prompts for common scenarios.

### QDD for Bug Fixes

```markdown
I need help debugging [brief description of issue].

Before suggesting solutions, ask me 5-8 diagnostic questions about:
- Exact error messages or symptoms
- When the problem occurs (reproducibility)
- Recent changes to the system
- What I've already tried
- Expected vs actual behavior

Based on my answers, create a diagnostic hypothesis, then suggest fixes.
```

**Example questions AI might ask**:
1. What's the exact error message? (full stack trace if available)
2. Does this happen consistently or intermittently?
3. What was the last working version?
4. Which components/files were changed recently?
5. Have you checked logs? What do they show?
6. Can you reproduce in development environment?

---

### QDD for Code Refactoring

```markdown
I need to refactor [component/function/module name].

Before proposing changes, ask me 5-8 questions about:
- What problems exist with current implementation
- What must stay the same (API, behavior, tests)
- What can change (structure, algorithm, dependencies)
- Performance or maintainability goals
- Constraints (time, backwards compatibility, team knowledge)

Based on my answers, create a refactoring specification, then propose changes.
```

**Example questions AI might ask**:
1. What specific problems make this code difficult to maintain?
2. Must the function signature stay the same? (breaking changes allowed?)
3. Are existing tests comprehensive? (can we trust them as regression suite?)
4. Performance-critical code? (can we prioritize readability over speed?)
5. What patterns does your team prefer? (classes vs functions, etc.)

---

### QDD for New Feature Implementation

```markdown
I need to implement [feature description].

Before designing the solution, ask me 5-8 questions about:
- User workflow and use cases
- Technical constraints (existing architecture, dependencies)
- Success criteria and acceptance tests
- Performance/scale requirements
- What's out of scope for this iteration

Based on my answers, create a feature specification, then propose implementation.
```

**Example questions AI might ask**:
1. Who are the users? What problem does this solve for them?
2. How does this fit into existing user workflows?
3. What existing systems/components does this interact with?
4. What's the expected usage volume? (1 user? 1000? 1 million?)
5. What's the MVP? (minimum viable version for first iteration)
6. What can wait for v2? (features to explicitly defer)

---

## Common QDD Mistakes

### Mistake 1: Asking AI to Ask Too Many Questions

❌ **Wrong**:
```markdown
Ask me questions about everything that might be relevant.
```

*AI asks 20+ questions. You get exhausted and give shallow answers.*

✅ **Right**:
```markdown
Ask me 5-8 clarifying questions about [specific categories].
```

*Focused questions → thoughtful answers → better requirements.*

---

### Mistake 2: Not Providing Question Categories

❌ **Wrong**:
```markdown
Ask me clarifying questions before implementing.
```

*AI asks obvious or trivial questions ("What language?" when obvious from context).*

✅ **Right**:
```markdown
Ask me clarifying questions about:
- Audience and use case
- Technical constraints
- Output format
- Success criteria
- Non-goals
```

*Structured categories → relevant questions → useful answers.*

---

### Mistake 3: Skipping Specification Validation

❌ **Wrong workflow**:
```
AI asks questions → You answer → AI implements immediately
```

*If AI misinterpreted your answers, you discover problems AFTER implementation.*

✅ **Right workflow**:
```
AI asks questions → You answer → AI creates specification → You validate → AI implements
```

*Specification checkpoint catches misinterpretations BEFORE wasted implementation effort.*

---

### Mistake 4: Answering Questions Too Vaguely

**AI asks**:
> "What's the primary audience for this documentation?"

❌ **Vague answer**:
> "Developers"

*Too broad. AI still has to assume (frontend? backend? junior? senior?).*

✅ **Specific answer**:
> "Frontend developers with React experience, integrating our API for the first time. They understand REST but don't know our authentication system."

*Specific audience → AI can match documentation depth and examples appropriately.*

---

## QDD Meta-Prompt Template

Use this template to activate Question-Driven Development for any task:

```markdown
I need [one-sentence description of task].

Before [implementing/designing/creating], ask me 5-8 clarifying questions about:
- [Question category 1]
- [Question category 2]
- [Question category 3]
- [Question category 4]
- [Question category 5]

Based on my answers, create a [specification/plan/design document], then [implement/build/generate].
```

**Fill in**:
- **Task description**: What you're building
- **Question categories**: What aspects AI should clarify (audience, constraints, format, success criteria, non-goals)
- **Output type**: specification, plan, design document
- **Final action**: implement, build, generate

---

## Practice Exercise: Bash Script Requirements

**Scenario**: You need a script that monitors log files for errors.

**Your Task**: Write a QDD meta-prompt that gets AI to ask YOU the right questions.

**Fill in this template**:

```markdown
I need a Bash script that monitors log files for errors.

Before implementing, ask me 5-8 clarifying questions about:
- [Category 1]
- [Category 2]
- [Category 3]
- [Category 4]
- [Category 5]

Based on my answers, create a specification document, then implement the script.
```

**What categories should AI ask about?** (Spend 3 minutes thinking)

---

**Solution** (Compare yours):

```markdown
I need a Bash script that monitors log files for errors.

Before implementing, ask me 5-8 clarifying questions about:
- Which log files to monitor (paths, file patterns)
- What constitutes an "error" (error keywords, patterns, severity levels)
- What action to take when errors found (alert, log, execute command)
- How often to check (continuous, interval-based, one-time scan)
- Output format and notifications (stdout, file, email, Slack)

Based on my answers, create a specification document, then implement the script.
```

**Did your categories cover**:
- ✅ Input (which logs?)
- ✅ Detection criteria (what's an error?)
- ✅ Action (what to do when found?)
- ✅ Timing (when to check?)
- ✅ Output (where to report?)

If yes → your QDD prompt will elicit useful requirements.

---

## What You've Learned

You've learned Question-Driven Development: how to transform AI from "assumption maker" into "requirements analyst."

**Core concepts**:

1. **QDD workflow** — AI asks questions → You answer → AI creates spec → You validate → AI implements
2. **Meta-prompt structure** — "Before implementing, ask 5-8 questions about [categories]"
3. **Question categories** — Audience, constraints, format, success criteria, non-goals
4. **Specification checkpoint** — Validate AI's understanding BEFORE implementation
5. **Three Roles in QDD** — AI teaches (questions you didn't consider), AI learns (your answers), AI co-creates (converges on complete spec)

**Key principle**: Spend 5 minutes answering AI's questions to save 30 minutes correcting AI's assumptions.

In the next lesson, you'll learn how to create **reusable prompt templates** for tasks you perform repeatedly—capturing question-driven workflows into templates you can invoke instantly.

---

## Try With AI

**Your Task**: Use QDD to generate a Git workflow guide for your team.

**Part 1: Write the QDD Meta-Prompt** (3 minutes)

```markdown
I need documentation explaining our Git workflow for new team members.

Before creating the guide, ask me 5-8 clarifying questions about:
- [Fill in question categories]

Based on my answers, create a specification document, then generate the guide.
```

**Part 2: Paste into AI and Answer Questions** (5 minutes)

Open your AI coding assistant and paste your QDD prompt. When AI asks questions, answer with specifics:
- Who's your audience? (junior devs? all team members? specific tech stack?)
- What's your workflow? (Git Flow? GitHub Flow? trunk-based?)
- What tools? (GitHub? GitLab? Bitbucket?)
- What should guide cover? (branching? PRs? code review? deployment?)

**Part 3: Validate the Specification** (2 minutes)

AI will create a specification. Review it:
- Does it capture your workflow accurately?
- Are there gaps? (things you forgot to mention in answers)
- Are there assumptions AI made? (mark these for correction)

**Part 4: Request Implementation** (2 minutes)

Once specification validated, tell AI: "Specification approved. Generate the guide."

**Part 5: Reflection** (3 minutes)

Compare:
- **Without QDD**: How many iterations would generic "create Git workflow guide" take? (probably 5-8)
- **With QDD**: How many iterations did this take? (specification caught issues upfront, likely 1-2)

**Key learning**: Questions surface hidden requirements. Answering questions is faster than correcting assumptions after implementation.

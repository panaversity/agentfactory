---
title: "Degradation Symptoms and Manual Tracking"
description: "Recognize seven degradation symptoms and diagnose context issues through pattern recognition"
sidebar_label: "Degradation Symptoms & Tracking"
sidebar_position: 2
chapter: 11
lesson: 2
duration_minutes: 45
proficiency: "A2"
concepts: 7

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Context Degradation Recognition"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies degradation symptoms from conversation transcripts"

learning_objectives:
  - objective: "Recognize seven distinct symptoms of context degradation"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Symptom identification from conversation examples"

  - objective: "Diagnose degradation patterns from session transcripts"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Comparison of healthy vs degraded session analysis"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (seven degradation symptoms) at A2 limit of 5-7 ✓"

differentiation:
  extension_for_advanced: "Study attention mechanism degradation in transformer models; analyze correlation between token position and recall accuracy; research context window optimization techniques"
  remedial_for_struggling: "Focus on recognizing one symptom (repetitive suggestions) through 3 examples before learning other symptoms"
---

# Degradation Symptoms and Manual Tracking

In Lesson 1, you learned to track context utilization manually using session notes and token estimation. You identified warning thresholds (green, yellow, red zones) and practiced calculating utilization percentages.

But knowing your context is at 80% doesn't tell you **whether it matters yet**. The real question is: **Has quality degraded?**

In this lesson, you'll learn to recognize the seven distinct symptoms of context degradation, practice diagnosing them from conversation transcripts, and decide when to take action. You'll build pattern recognition through compare-and-contrast examples: healthy sessions vs degraded sessions side-by-side.

## The Seven Degradation Symptoms

Context degradation doesn't announce itself with error messages. It manifests through subtle changes in AI behavior that compound over time. Here are the seven symptoms you'll learn to recognize:

### Symptom 1: Repetitive Suggestions

**Definition**: AI suggests the same solution multiple times without acknowledging it already explained it.

**Example** (from same session):

```
[00:15] YOU: How should I structure error handling for API responses?

AI: I recommend a consistent error response format:
- "status": "error"
- "message": User-friendly error description
- "code": Machine-readable error code (ERR_VALIDATION, ERR_AUTH, etc.)
- "details": Optional object with field-specific errors

This makes error handling predictable for API consumers.

[Later, 01:30]

YOU: What about handling validation errors specifically?

AI: For validation errors, I recommend a consistent response format:
- "status": "error"
- "message": User-friendly description
- "code": Error code like ERR_VALIDATION
- "details": Field-specific errors

This pattern ensures API consumers can handle errors predictably.
```

**Observation**: The AI repeated nearly the same explanation word-for-word. It didn't reference the earlier discussion or build on it—just regenerated similar content.

**Why it happens**: Context window saturation causes the model to lose track of what it already explained. It falls back to generating plausible answers that happen to duplicate earlier responses.

---

### Symptom 2: Forgotten Patterns

**Definition**: AI ignores or contradicts project-specific decisions you established earlier in the session.

**Example**:

```
[00:20] YOU: Our API uses camelCase for JSON keys. All responses should follow this convention.

AI: Understood. I'll use camelCase consistently: userId, createdAt, errorMessage.

[Later, 02:00]

YOU: Create an error response for invalid email format.

AI: Here's the error response:

{
  "error_message": "Invalid email format",
  "field_name": "email",
  "validation_failed": true
}
```

**Observation**: The AI used snake_case (error_message, field_name, validation_failed) despite agreeing to camelCase at 00:20.

**Why it happens**: Earlier decisions get evicted from effective context as the window fills. The AI defaults to generic patterns rather than project-specific conventions.

---

### Symptom 3: Performance Degradation

**Definition**: Responses take noticeably longer to generate and become shorter or less detailed.

**Example**:

```
[00:10] YOU: Explain how JWT tokens work for authentication.

AI: [Responds in 2 seconds with 300-word explanation including:
- Token structure (header, payload, signature)
- How servers validate tokens without database lookup
- Security considerations
- Example use case]

[Later, 02:30]

YOU: Explain how refresh tokens work.

AI: [Responds in 12 seconds with 100-word explanation:
- Refresh tokens are long-lived
- Used to get new access tokens
- Stored securely]
```

**Observation**: Response latency tripled (2s → 12s) and output quality dropped (detailed → generic, 300 words → 100 words).

**Why it happens**: Computing attention over large context windows is expensive. As context grows, generation slows down and quality decreases.

---

### Symptom 4: Generic Responses

**Definition**: AI shifts from project-specific advice to generic best practices.

**Example**:

```
[00:30] YOU: How should I handle database connection pooling for our PostgreSQL setup with SQLAlchemy async?

AI: For your async SQLAlchemy setup with PostgreSQL, use asyncpg driver with connection pooling:
- Set pool_size=10 for moderate traffic
- Set max_overflow=5 for burst handling
- Use pool_pre_ping=True to detect stale connections
- Configure pool_recycle=3600 for long-running connections

This matches your async architecture from earlier.

[Later, 02:40]

YOU: How should I handle connection errors?

AI: For database connection errors, follow best practices:
- Implement retry logic
- Use exponential backoff
- Log errors properly
- Monitor connection health

Choose an approach that fits your architecture.
```

**Observation**: Early response referenced specific project details (async SQLAlchemy, PostgreSQL, asyncpg). Late response gave generic advice without project context.

**Why it happens**: Project-specific context gets compressed or forgotten. AI falls back to generic knowledge.

---

### Symptom 5: Lost Context References

**Definition**: AI can't retrieve information you provided earlier, even when explicitly asked.

**Example**:

```
[00:25] YOU: Our database schema uses these tables: users, sessions, api_keys, audit_logs. The audit_logs table tracks all user actions.

AI: Clear schema. I see audit_logs captures user activity. I'll reference this for logging recommendations.

[Later, 02:15]

YOU: Where should I log user password changes?

AI: For password change logging, you could:
- Create a new logging table
- Use application logs
- Store in users table with timestamp

What approach fits your database design?
```

**Observation**: You established audit_logs at 00:25. AI should have said "Log password changes to audit_logs table." Instead, it suggested creating a new table.

**Why it happens**: Information technically exists in context, but the model isn't effectively retrieving it.

---

### Symptom 6: Contradictory Advice

**Definition**: AI suggests opposite approaches at different points in the same session.

**Example**:

```
[00:50] YOU: Should we validate user input on client-side or server-side?

AI: Always validate on the server. Client-side validation improves UX but can be bypassed. Server validation is the security boundary—never trust client input.

[Later, 02:20]

YOU: What's the best practice for validation?

AI: Client-side validation is the recommended approach. It provides immediate feedback and reduces unnecessary server requests. Validate on the client for better user experience.
```

**Observation**: AI explicitly contradicted itself. Server validation (00:50) is security-critical. Client validation (02:20) isn't a replacement—it's a UX enhancement.

**Why it happens**: Model generates plausible answers without checking consistency with earlier statements. Degradation breaks reasoning chains.

---

### Symptom 7: Confusion About Task Scope

**Definition**: AI loses track of what you're trying to accomplish and asks for clarification on things already established.

**Example**:

```
[00:15] YOU: I'm building a markdown documentation generator. It reads markdown files from docs/ folder and generates HTML pages.

AI: Clear scope. Markdown → HTML generator. I'll help implement the parser and HTML output.

[Later, 02:50]

YOU: Now add support for linking between pages.

AI: For page linking, I need to understand: Are you building a static site generator or a dynamic web app? What format are the input files?
```

**Observation**: You specified the scope at 00:15 (markdown files → HTML, static generator). AI asked questions already answered.

**Why it happens**: Task context gets evicted. AI doesn't remember the overall goal and asks for re-clarification.

---

## Compare and Contrast: Healthy vs Degraded Sessions

The best way to build pattern recognition is to see healthy and degraded sessions side-by-side. Below are two sessions working on the same task—one at 30% context utilization (healthy), one at 85% utilization (degraded).

### Healthy Session (30% Context Utilization)

```markdown
YOU: I'm working on user authentication. We use JWT tokens with 15-minute expiry. I need to add refresh token support.

AI: For refresh token support with your existing JWT setup:

1. Generate two tokens on login:
   - Access token: 15 min expiry (current implementation)
   - Refresh token: 7 days expiry (new)

2. Create POST /auth/refresh endpoint:
   - Accepts refresh token
   - Validates token signature
   - Issues new access + refresh tokens
   - Rotates refresh token (invalidates old one)

3. Update login response to include both tokens

This follows your 15-minute access token pattern while adding long-lived refresh capability.

---

YOU: Good. How should I store refresh tokens?

AI: For your refresh token storage, I recommend:

**Server-side**:
- Store in database (users_sessions table recommended)
- Include fields: token_hash, user_id, expires_at, created_at
- Hash the token (don't store plaintext)
- Index on token_hash for fast lookups

**Why not store client-side only**: If token is compromised, you need server-side ability to revoke it.

This gives you revocation control while maintaining your JWT stateless architecture for access tokens.

---

YOU: Should I rotate the refresh token on every refresh?

AI: Yes, rotate refresh tokens on every use. Here's why this matters for your implementation:

**Security benefit**: If refresh token is stolen, attacker has limited window before legitimate user refreshes (invalidating stolen token).

**Implementation**: When POST /auth/refresh is called:
1. Validate current refresh token
2. Generate NEW refresh token
3. Invalidate old refresh token in database
4. Return new access + refresh token pair

This is called "refresh token rotation"—industry best practice for OAuth 2.0 refresh flows.
```

**Healthy session indicators**:
- ✅ AI references earlier decisions ("your 15-minute access token pattern")
- ✅ Responses are specific to the project (users_sessions table, stateless JWT architecture)
- ✅ AI builds on previous responses (rotation concept introduced logically)
- ✅ No repetition, no contradictions, no forgotten context
- ✅ Response quality is consistent throughout

---

### Degraded Session (85% Context Utilization)

```markdown
YOU: I'm working on user authentication. We use JWT tokens with 15-minute expiry. I need to add refresh token support.

AI: For authentication with JWT tokens:

**Token Types**:
- Access tokens: Short-lived (typically 15 minutes)
- Refresh tokens: Long-lived (7-30 days)

**Implementation Steps**:
1. Generate both tokens on login
2. Access token used for API requests
3. Refresh token used to get new access tokens
4. Store tokens securely

This is standard JWT authentication pattern.

---

YOU: How should I store refresh tokens?

AI: For token storage, consider these approaches:

**Option 1: Database Storage**
- Store tokens in database
- Allows revocation
- Requires database lookup

**Option 2: Client-Side Only**
- Store in localStorage or cookie
- No server storage needed
- Simpler implementation

**Option 3: Hybrid Approach**
- Combination of both

Choose based on your security requirements.

---

YOU: Earlier we discussed using JWT tokens with 15-minute expiry. Which storage approach fits that pattern?

AI: For JWT tokens with 15-minute expiry, I recommend using refresh tokens:

**Token Generation**:
1. Generate access token (15 min expiry)
2. Generate refresh token (7 day expiry)
3. Return both to client
4. Client uses access token for requests
5. When access token expires, use refresh token

This follows OAuth 2.0 best practices.

[NOTICE: AI repeated the original implementation explanation almost word-for-word. It didn't answer the storage question.]

---

YOU: You didn't answer my storage question. Should I store refresh tokens in a database?

AI: Yes, store refresh tokens in a database. Here's why:

**Security**: Database storage allows token revocation
**Implementation**: Create tokens table with fields for token value, user ID, expiration
**Lookup**: Query database on refresh requests

This is the secure approach.

[NOTICE: Generic response. Doesn't reference "users_sessions table" or "hash the token" like healthy session did.]

---

YOU: Should I rotate refresh tokens on every use?

AI: Token rotation depends on your security requirements:

**Option A: Rotate on every use**
- More secure
- Limits attack window
- Requires database updates

**Option B: Fixed refresh tokens**
- Simpler implementation
- Longer validity period
- Less database traffic

Both approaches are valid. Consider your security vs simplicity tradeoff.

[NOTICE: Generic advice. Healthy session confidently recommended rotation with security reasoning. Degraded session gave "it depends" answer without project-specific guidance.]
```

**Degraded session indicators**:
- ❌ Symptom 1 (Repetition): AI repeated JWT explanation almost identically
- ❌ Symptom 4 (Generic): "Choose based on your requirements" instead of specific recommendation
- ❌ Symptom 5 (Lost Context): Didn't reference the 15-minute pattern from earlier when asked about storage
- ❌ Symptom 7 (Confusion): Failed to directly answer storage question, forcing re-asking
- ❌ Response quality degraded from specific → generic over session

---

## Practice: Symptom Identification Checklist

Use this checklist during your next extended session (60+ minutes) to track degradation manually:

```markdown
## Context Degradation Checklist

**Session Info**:
- Start time: [timestamp]
- Task: [what you're building]
- Estimated duration: [time]

**Symptom Tracking** (mark when observed):

| Time | Symptom Type | Evidence | Action |
|------|-------------|----------|--------|
|      | [ ] Repetitive Suggestions | AI repeated ___ without acknowledgment |  |
|      | [ ] Forgotten Patterns | AI ignored ___ decision from earlier |  |
|      | [ ] Performance Degradation | Response time: ___ seconds (baseline: ___) |  |
|      | [ ] Generic Responses | AI gave generic advice instead of project-specific |  |
|      | [ ] Lost Context References | AI couldn't retrieve ___ from earlier |  |
|      | [ ] Contradictory Advice | AI suggested opposite approach to ___ |  |
|      | [ ] Confusion About Task | AI asked for clarification on ___ already stated |  |

**Degradation Severity**:
- [ ] MILD (1-2 symptoms, work continues effectively)
- [ ] MODERATE (3-4 symptoms, quality declining)
- [ ] SEVERE (5+ symptoms, create checkpoint NOW)

**Action Decision**:
- [ ] Continue (still in green zone)
- [ ] Compress (create checkpoint, restart session)
- [ ] Isolate (start separate session for new task)
- [ ] Restart (degradation too severe, start fresh)
```

---

## Exercise: Diagnose This Session

Read the conversation below and identify ALL degradation symptoms present:

```
[00:10] YOU: I'm building an API documentation generator. It reads OpenAPI specs and generates markdown documentation.

AI: Clear goal. OpenAPI → markdown generator. I'll help structure the parser and documentation templates.

[00:30] YOU: Start with parsing the OpenAPI spec. Read the YAML file and extract endpoint information.

AI: For OpenAPI parsing, extract these elements:
- Paths (API endpoints)
- Methods (GET, POST, PUT, DELETE)
- Parameters (query, path, body)
- Responses (status codes, schemas)

Here's the parsing structure: [provides detailed breakdown]

[00:50] YOU: Good. Now generate markdown for each endpoint.

AI: For markdown generation, create this template:

## [METHOD] /api/path
Description: [endpoint description]
Parameters: [parameter list]
Responses: [response documentation]

This follows standard API documentation format.

[01:20] YOU: Add support for authentication schemes (API keys, OAuth, Bearer tokens).

AI: For authentication documentation:
- API Key: Document header name and format
- OAuth 2.0: Document authorization flow
- Bearer Token: Document token format

Include authentication requirements for each endpoint.

[02:10] YOU: How should I organize the generated documentation? Should each endpoint be a separate file?

AI: For documentation organization, you have several options:
1. Single file (all endpoints in one document)
2. Multiple files (one per endpoint)
3. Grouped files (endpoints by resource)
4. Custom structure

What's your project's documentation structure preference?

[NOTICE: You already stated at 00:10 it's a documentation generator. This asks for clarification on something you're building.]

[02:30] YOU: Generate one markdown file per API resource (users, products, orders).

AI: For markdown generation, create this template for each resource:

## [METHOD] /api/path
Description: [endpoint description]
Parameters: [parameter list]
Responses: [response documentation]

This follows standard API documentation format.

[NOTICE: This is nearly identical to the response at 00:50. AI repeated itself.]

[02:50] YOU: Should I include code examples in the documentation?

AI: Code examples improve API documentation. Consider adding:
- Request examples
- Response examples
- Error handling examples

This helps developers understand API usage.

[NOTICE: Generic advice. Doesn't reference anything specific about OpenAPI specs or the generator being built.]

[03:05] YOU: We already discussed OpenAPI at 00:10. Does OpenAPI spec include example requests?

AI: Yes, OpenAPI specifications can include example requests and responses. You can extract examples from the spec and include them in generated documentation.

For your documentation generator, parse the "examples" field from OpenAPI spec.

[NOTICE: Should have referenced this at 02:50 when asked about code examples. Only remembered after being reminded.]
```

**Your Task**:
1. Identify each degradation symptom present
2. Note the timestamp when it appeared
3. Classify severity (mild, moderate, severe)
4. Recommend action (continue, compress, isolate, restart)

<details>
<summary>Click to reveal analysis</summary>

**Symptoms Identified**:

1. **Symptom 7 - Confusion About Task** (02:10)
   - AI asked "What's your project's documentation structure preference?"
   - Project scope already established at 00:10 (building a generator)

2. **Symptom 1 - Repetitive Suggestions** (02:30)
   - AI repeated the markdown template explanation from 00:50 word-for-word
   - No acknowledgment of prior explanation

3. **Symptom 4 - Generic Responses** (02:50)
   - "Code examples improve API documentation" - generic advice
   - Didn't reference OpenAPI spec capabilities established earlier

4. **Symptom 5 - Lost Context References** (03:05)
   - Had to be reminded about OpenAPI spec from 00:10
   - Should have referenced OpenAPI examples capability at 02:50

**Severity**: MODERATE (4 symptoms, quality declining)

**Recommendation**: COMPRESS
- Symptoms present but not yet severe
- Task is continuous (same documentation generator)
- Creating checkpoint would preserve progress while resetting context
- Recovery time (5 min) less than degradation cost

</details>

---

## Try With AI

Now that you can recognize degradation symptoms manually, practice diagnosing them with AI assistance.

### Setup
Open Claude Code with these exercises ready.

### Prompt Set

**Prompt 1 — Symptom Recognition Training:**
```
Explain the seven context degradation symptoms (repetitive suggestions, forgotten patterns, performance degradation, generic responses, lost context references, contradictory advice, task confusion) with examples.

For each symptom:
- Why it happens when context fills
- How to recognize it as a human observer
- What distinguishes it from other symptoms
```

**Prompt 2 — Analyze This Transcript:**
```
[Paste the OpenAPI documentation generator conversation from Exercise above]

Identify all degradation symptoms present in this conversation. For each:
- Symptom type
- Timestamp when it appeared
- Evidence from the transcript
- Severity assessment
```

**Prompt 3 — Compare Healthy vs Degraded:**
```
Create two side-by-side conversations about the same task (building a REST API):
- Conversation A: 30% context utilization (healthy session)
- Conversation B: 85% context utilization (degraded session)

Show how the same questions get different quality responses. Highlight specific degradation symptoms in Conversation B.
```

**Prompt 4 — Decision Framework Practice:**
```
I'm in a 2-hour session showing these symptoms:
- AI repeated database indexing advice from 45 minutes ago
- AI forgot we're using PostgreSQL (stated at session start)
- Responses taking 10+ seconds (baseline was 2-3 seconds)
- AI suggested contradictory approaches to error handling

Diagnose:
- Which symptoms are present?
- Estimated context utilization?
- Should I compress, isolate, or restart?
- What action do you recommend and why?
```

### Expected Outcomes

Claude Code should:
- Accurately identify each symptom from transcripts
- Explain why each symptom indicates context degradation
- Compare healthy vs degraded sessions with specific differences
- Recommend appropriate action (compress/isolate/restart) based on severity

**Safety Note**: Trust your observations. If you see repetition or contradictions, degradation is present even if utilization seems low.

---

**Up Next**: In Lesson 3, you'll shift from manual observation to collaborative context management—using progressive loading strategies with AI to prevent degradation before it happens.

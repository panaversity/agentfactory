# Implementation Plan: Chapter 11 Context Engineering Redesign

**Feature ID**: 002-011-chapter-11-redesign-fix
**Status**: Ready for Implementation
**Created**: 2025-11-18
**Version**: 1.0.0
**Plan Scope**: 9 Lessons + 2 Skills + README

---

## I. Overview & Strategic Context

### Chapter Identity
- **Title**: Context Engineering for AI-Driven Development
- **Part**: 3 (Markdown, Prompt & Context Engineering — NO programming)
- **Proficiency Tier**: B1 (Intermediate) — 7-10 concepts per lesson
- **Pedagogical Approach**: Systems thinking (architecture, decision frameworks, optimization)
- **Prerequisite**: Chapter 10 (Three Roles framework + AI collaboration)

### Critical Constraints (Part 3 Compliance)
**ZERO code examples allowed in Lessons 1-8:**
- No Python, JavaScript, TypeScript, SQL, or any programming syntax
- No FastAPI, SQLAlchemy, authentication libraries, or frameworks
- No pseudo-code that resembles real programming
- Only Markdown files, Claude Code prompts, plain text examples, decision frameworks

**Lesson 9 (Capstone Spec) exception:**
- Can reference components in plain English: "Context Monitor component"
- Can use structured English algorithms: "IF utilization > 80% THEN create checkpoint"
- Still ZERO runnable code (specification-only)

### Teaching Modality Differentiation
- **Chapter 10** (previous): Conversational pedagogy — "What to SAY to AI" (role-playing scenarios)
- **Chapter 11** (this): Systems thinking pedagogy — "What your AI KNOWS" (architecture, decision frameworks)
- **Distinction**: Not repeating Chapter 10's teaching pattern (anti-convergence requirement)

---

## II. Lesson-by-Lesson Implementation Plan

### LESSON 1: Context Windows and Token Counting (Layer 1: Manual Foundation)

**Pedagogical Role**: Foundation establishment — build mental model of context as working memory

**Learning Objectives** (CEFR B1):
- Recognize what "context" means in AI context (not semantic context, but working memory size)
- Estimate token usage manually using rules of thumb
- Identify when context is approaching limits through observable behaviors
- Track context loading decisions in session notes

**Layer Architecture**: Layer 1 (Manual Foundation)
- NO AI assistance in exercises
- Students practice writing session notes manually
- Students estimate tokens using simple rules (1 token ≈ 1 word, ~1.2x multiplier)

**Concepts Introduced** (Count: 8 — within B1 limit of 7-10):
1. Context window definition (working memory of AI tool)
2. Token counting basics (word-based estimation)
3. Context window sizes (Claude Sonnet 4.5: 200K, extended: 1M; Gemini: 2M)
4. Utilization percentage calculation
5. Session notes structure (Date, Task, Context Loaded, Progress, Token Estimation)
6. Degradation early signals (AI takes longer, responses more generic)
7. Research source: Google PDF Sessions Architecture
8. Observable behaviors vs token counts (correlation)

**Content Structure**:

**Part A: What is a Context Window?**
- Analogy: RAM in computing (fixed capacity, shared resource, diminishing performance as full)
- Real numbers: Claude Sonnet 4.5 = 200K tokens standard, 1M extended; Gemini 1.5 Pro = 2M
- NOT unlimited; NOT infinitely scalable

**Part B: Token Estimation (Manual Practice)**
- Rule of thumb: ~1 token per word (conservative estimate)
- Extended conversations: Multiply word count × 1.2 for token overhead
- Example: 1,000-word conversation ≈ 1,200 tokens

**Part C: Session Note Format (Hands-On)**
```markdown
# Development Session — 2025-11-18

## Task: [Clear task description]

## Context Loaded:
- [File 1 — estimated tokens]
- [File 2 — estimated tokens]
- Total loaded: ~X tokens

## Progress:
1. [Action taken]
2. [Response]
3. [Observation]

## Token Estimation:
- Context loaded: ~X tokens
- Conversation so far: ~Y tokens
- Total utilization: ~Z / 200K = A%

## Observations:
- [When did responses feel different?]
- [When did AI stop remembering earlier points?]
```

**Practice Exercises** (Manual, no AI):
1. Write 3 session notes for different projects (estimate tokens for each)
2. Compare two projects: Which has higher token load? Why?
3. Identify "red flags" in sample transcript (where AI might be approaching limits)

**Research Integration**:
- **Source**: Google PDF (Context Engineering: Sessions & Memory)
- **Concept**: Sessions architecture — chronological history vs working memory state
- **Application**: Students understand session notes as "working memory snapshots"

**Cognitive Load Validation**: 8 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Student can write session note with accurate token estimation (within 20% of actual)
- Student can identify 3+ signals that context is approaching limits
- Zero programming code in lesson content

**Try With AI** (Closure):
```markdown
## Try With AI: Verify Your Token Estimates

Take one of the session notes you wrote manually. Copy it to Claude Code:

Ask: "I estimated this session used ~X tokens. Looking at the word count and conversation complexity, is my estimate reasonable? What would be more accurate?"

Compare your manual estimate to AI's assessment. What factors did you miss?
```

---

### LESSON 2: Degradation Symptoms and Manual Tracking (Layer 1: Manual Foundation)

**Pedagogical Role**: Pattern recognition in manual observation — build diagnostic capabilities

**Learning Objectives** (CEFR B1):
- Recognize context degradation symptoms from conversation transcripts
- Create degradation checklists for systematic diagnosis
- Distinguish between AI limitations and context degradation
- Understand root causes of each symptom

**Layer Architecture**: Layer 1 (Manual Foundation)
- NO AI assistance in exercises
- Students manually review transcripts and identify symptoms
- Build diagnostic intuition before applying AI collaboration

**Concepts Introduced** (Count: 9 — within B1 limit):
1. Repetitive suggestions (symptom: AI suggests same solution 2+ times)
2. Forgotten patterns (symptom: AI ignores rules stated earlier in session)
3. Performance degradation (symptom: AI takes longer to respond, response length decreases)
4. Generic responses (symptom: AI provides vague answer instead of specific to project)
5. Lost context (symptom: AI asks for info already provided earlier)
6. Compare-and-contrast pedagogy (healthy vs degraded side-by-side)
7. Degradation checklist structure
8. Root cause analysis (which symptom maps to which cause)
9. Recovery decision (when to restart session vs continue)

**Content Structure**:

**Part A: The Five Degradation Symptoms (Conceptual)**

**Symptom 1: Repetitive Suggestions**
- Pattern: "Here's how to handle errors" → [5 minutes later] → "You could try error handling"
- Indicator: AI suggests identical approach without acknowledgment of prior suggestion
- Cause: Context window saturation — AI repeats patterns rather than advancing

**Symptom 2: Forgotten Patterns**
- Pattern: "We use camelCase for variables" → [20 minutes later] → "I'd recommend snake_case"
- Indicator: AI violates established pattern explicitly stated in session
- Cause: Pattern buried in context window, retrieved pattern conflicts with oldest context

**Symptom 3: Performance Degradation**
- Pattern: Responses shift from 500 tokens → 200 tokens; latency increases
- Indicator: AI response quality/length decreases; thinking time increases
- Cause: Working memory saturation — AI allocates context differently

**Symptom 4: Generic Responses**
- Pattern: "How do I handle authentication?" → "Use industry-standard practices" (vague)
- Indicator: Loss of project-specific context; recommendations become generic
- Cause: Context window full; specific project details evicted to make room for conversation

**Symptom 5: Lost Context**
- Pattern: "I explained our database schema here" → [later] → "Can you describe your data model?"
- Indicator: AI requests information already provided, needs re-explanation
- Cause: Context window eviction; earlier messages compressed or discarded

**Part B: Compare-and-Contrast Examples (Healthy vs Degraded)**

**Example 1: Healthy Session (20-30% Utilization)**
```
USER: "How do we handle errors in our validation?"
AI: "Looking at your project patterns, I see three approaches that fit your architecture:
1. Exception handling (matches your existing patterns)
2. Result types (new but maintainable)
3. Validation pipeline (least disruptive)

Given your constraints, Approach 1 aligns best."

[5 minutes, 8 exchanges later]

USER: "Now for network errors?"
AI: "Consistent with the error handling pattern we established (the exception approach), here's how I'd extend it for network failures:
- Inherit from your base error class
- Maintain consistency with validation error structure
- Add retry logic specific to network context"

OBSERVATION: AI remembers established pattern and extends it. Specific to project.
```

**Example 2: Degraded Session (70%+ Utilization)**
```
USER: "How do we handle errors in our validation?"
AI: [Detailed response, 800 tokens]

[15 minutes, 12 exchanges later]

USER: "Now for network errors?"
AI: "There are many ways to handle errors. You could use:
1. Try-catch blocks
2. Error codes
3. Logging systems"

OBSERVATION: Generic advice. No memory of validation pattern discussed 15 minutes ago.
Lost project specificity. Generic approaches suggested.
```

**Part C: Degradation Checklist**

```markdown
## Degradation Diagnosis Checklist

**For each conversation, mark which symptoms present:**

- [ ] Repetitive suggestions (same solution suggested multiple times)
- [ ] Forgotten patterns (AI ignores established rules/approaches)
- [ ] Performance degradation (responses shorter, latency higher)
- [ ] Generic responses (no project-specific context in AI output)
- [ ] Lost context (AI asks for re-explanation of prior info)

**If 2+ symptoms present**: Session has degraded → Consider restart

**Severity Scoring**:
- 1 symptom: Minor (can continue short-term)
- 2 symptoms: Moderate (continue carefully, plan restart)
- 3+ symptoms: Severe (restart recommended)
```

**Practice Exercises** (Manual, no AI):
1. Read 5 provided transcripts; mark degradation symptoms on checklist
2. For each transcript, identify which symptom appeared first
3. Compare two transcripts side-by-side: Which shows degradation? Why?
4. Create degradation checklist for your own work pattern

**Research Integration**:
- **Source**: GitHub Spec (CoLearning Agentic AI)
- **Concept**: Compare-and-contrast pedagogy (healthy vs degraded shown side-by-side)
- **Application**: Students develop pattern recognition through contrast

**Cognitive Load Validation**: 9 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Student identifies 4/5 degradation symptoms correctly in sample transcript
- Student creates accurate degradation checklist for their own conversation
- Zero programming code in lesson

**Try With AI** (Closure):
```markdown
## Try With AI: Analyze Your Recent Sessions

Provide Claude Code with a transcript from a recent long development session:

Ask: "Review this session transcript for degradation symptoms.
Mark which symptoms appear and approximately when each appeared.
What's your assessment: does this session need a restart?"

Compare AI's degradation analysis to your own from the checklist exercise.
```

---

### LESSON 3: Progressive Loading Strategy (Layer 2: AI Collaboration, Three Roles)

**Pedagogical Role**: Collaborative strategy design — AI as Teacher/Student/Co-Worker

**Learning Objectives** (CEFR B1):
- Design loading strategy using Foundation → Current → On-Demand pattern
- Distinguish between always-loaded, task-relevant, and on-demand context
- Evaluate loading decisions under different project constraints
- Demonstrate bidirectional learning (AI teaches pattern, student teaches constraints)

**Layer Architecture**: Layer 2 (AI Collaboration with Three Roles)
- **AI as Teacher**: Suggests Foundation → Current → On-Demand pattern (student didn't know)
- **AI as Student**: Learns which files are "Foundation" for project (student knows better)
- **AI as Co-Worker**: Iterates on loading strategy (2+ rounds) toward optimal approach

**Concepts Introduced** (Count: 8 — within B1 limit):
1. Foundation context (core patterns, always loaded)
2. Current context (task-relevant, loaded per task)
3. On-Demand context (fetch as needed)
4. Smallest set of high-signal tokens principle (Anthropic research)
5. Context allocation percentages (Foundation: 10-15%, Current: 20-30%, Reserve: 30%)
6. Loading decision criteria
7. Trade-offs: Preload (faster) vs On-Demand (less waste)
8. Iteration toward optimal strategy

**Content Structure**:

**Part A: The Three-Tier Loading Model (Conceptual)**

**Tier 1: Foundation (Always Loaded)**
- What: Core project patterns that apply to ALL tasks
- Examples:
  - Project conventions (CLAUDE.md)
  - System architecture (architecture.md)
  - Key decisions (decisions.md)
- Size target: 10-15% of context window
- Rationale: Foundation is overhead; small foundation leaves room for task work

**Tier 2: Current (Loaded for This Task)**
- What: Task-relevant files needed for CURRENT work only
- Examples:
  - For "fix authentication bug": auth.md + bug-report.md
  - For "add new API endpoint": api-structure.md + endpoint-spec.md
- Size target: 20-30% of context window
- Rationale: Minimize current context; on-demand fetches additional if needed

**Tier 3: On-Demand (Fetch As Needed)**
- What: Reference files only requested when AI explicitly asks
- Examples:
  - "Can I see the database schema?"
  - "I need your testing patterns"
- Reserve: 30% of context window for on-demand fetching
- Rationale: Avoid preloading everything; fetch only when helpful

**Part B: Three Roles Demonstration (Collaborative Iteration)**

**Round 1: AI as Teacher (Suggests Pattern)**

```markdown
STUDENT: "I'm working on a documentation project with 50 files.
How do I load context efficiently without overwhelming my AI assistant?"

AI as Teacher (suggests pattern):
"I'd recommend a three-tier loading strategy:

1. Foundation (always): Core doc structure, templates, style guide (~15% context)
2. Current (task-specific): Docs for TODAY'S section (~25% context)
3. On-Demand (fetch if needed): Reference docs fetched when I ask (~30% reserved)

This keeps your context utilization controlled while maintaining flexibility."

OBSERVATION: AI teaches student the pattern (student didn't know this approach)
```

**Round 2: AI as Student (Learns Project Constraints)**

```markdown
STUDENT: "That makes sense, but I have a constraint: our style guide changes
frequently. Should it be Foundation or Current?"

AI as Student (learns constraint):
"Good point—frequently-changing Foundation becomes stale.
I should treat your style guide as Current (task-specific) and fetch
latest-version on session start instead of hard-coding it into Foundation."

OBSERVATION: AI learns student's constraint; adapts strategy accordingly
```

**Round 3: AI as Co-Worker (Iterates Toward Optimal)**

```markdown
STUDENT: "Let's test this. What if Foundation is just project-identity
(name, purpose), Current is section-specific docs, and On-Demand is
reference library?"

AI as Co-Worker:
"Even better. That separation creates:
- Smaller Foundation (faster to load)
- Focused Current (minimal waste)
- Clear On-Demand boundary (when to ask for reference)

Let's validate: What happens when you need cross-section consistency?"

[Iteration continues until they converge on strategy]

OBSERVATION: Neither student nor AI had perfect strategy initially.
Through iteration, they converge on better solution.
```

**Part C: Loading Strategy Template (Artifact)**

```markdown
## Context Loading Strategy for [Project Name]

**Foundation (Always Loaded):**
1. [File 1] — [Why always needed] — Est: X tokens
2. [File 2] — [Why always needed] — Est: Y tokens
Total Foundation: ~Z tokens (~10-15% of 200K window)

**Current (Task-Specific):**
1. [File A] — [When needed for tasks] — Est: X tokens
2. [File B] — [When needed for tasks] — Est: Y tokens
Total Current: ~Z tokens (~20-30% of 200K window)

**On-Demand (Reference Library):**
- [Reference category 1] — [What AI might ask for]
- [Reference category 2] — [What AI might ask for]
Reserved: ~60K tokens (~30% of 200K window)

**Decision Logic:**
- Session start: Load Foundation + Current for today's task
- Mid-session: If AI asks "Can I see...?" → Fetch from On-Demand
- Session end: Update Foundation/Current for next session
```

**Practice Exercises** (With AI Collaboration):

1. **Collaborative Exercise**: Work with AI to design loading strategy for your current project
   - You describe project structure
   - AI suggests Foundation/Current/On-Demand split
   - You refine based on constraints
   - You iterate until both satisfied

2. **Evaluation Exercise**: Given 3 project descriptions, evaluate loading strategies
   - Which has Foundation too large?
   - Which has Current too scattered?
   - Which maximizes On-Demand reserve?

3. **Scaling Exercise**: How would strategy change if project doubled in size?

**Research Integration**:
- **Source**: Anthropic Article (Context Engineering for AI Agents)
- **Concept**: "Smallest set of high-signal tokens" principle
- **Application**: Students learn to curate Foundation (highest signal only)

**Cognitive Load Validation**: 8 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Student designs loading strategy with Foundation/Current/On-Demand breakdown
- Strategy allocation totals < 100% of context window
- Student can justify each file placement
- Three Roles demonstrated (AI teaches, student teaches, convergence shown)

**Try With AI** (Closure):
```markdown
## Try With AI: Validate Your Loading Strategy

Describe your project structure to Claude Code:

Part 1 - Initial Proposal:
Ask: "Here's my project structure.
Suggest a Foundation/Current/On-Demand loading strategy."

Part 2 - Constraint Teaching:
Review AI's suggestion. What constraint did AI miss about your project?
Tell AI: "[Your constraint that changes the strategy]"

Part 3 - Iteration:
Ask AI: "How does [constraint] change the loading strategy?"

Part 4 - Validation:
Ask: "Is this strategy stable if I add 10 new features?"

Part 5 - Reflection:
Compare your final strategy to your initial thinking.
What did you learn through iteration?
```

---

### LESSON 4: Context Compression and Session Restart (Layer 2: AI Collaboration, Three Roles)

**Pedagogical Role**: Compression technique with collaborative iteration

**Learning Objectives** (CEFR B1):
- Create checkpoint summaries that compress sessions into reusable context
- Identify what to preserve vs what to discard in checkpoints
- Execute session restart with checkpoint recovery
- Recognize trade-offs: Restart cost vs context reclamation benefit

**Layer Architecture**: Layer 2 (AI Collaboration with Three Roles)
- **AI as Teacher**: Suggests checkpoint structure and extraction strategy
- **AI as Student**: Learns what's important to preserve for student's project
- **AI as Co-Worker**: Iterates on checkpoint conciseness vs completeness

**Concepts Introduced** (Count: 8 — within B1 limit):
1. Checkpoint definition (compressed session summary)
2. Extraction process (identify key decisions/discoveries)
3. Consolidation process (compress into <600 tokens)
4. Checkpoint structure (decisions, progress, next steps, patterns)
5. Restart trigger decision (IF utilization > 80% AND duration > 60min THEN checkpoint)
6. Recovery procedure (load checkpoint at session start)
7. Extraction + Consolidation pipeline (Anthropic research)
8. Trade-off analysis (compression cost vs context space gained)

**Content Structure**:

**Part A: Checkpoint Structure (Conceptual)**

**What is a Checkpoint?**
- Compressed summary of session progress
- Created when context nearing limits (80%+ utilization)
- Loaded at start of next session to recover context
- Target size: <600 tokens (leaves 199.4K for new work)

**Checkpoint Template**:
```markdown
# CHECKPOINT — Session Ending [DATE]

## Architectural Decisions Made:
1. [Decision 1] — [Why this choice] — [Alternatives considered]
2. [Decision 2] — [Why this choice] — [Alternatives considered]
3. [Decision 3] — [Why this choice] — [Alternatives considered]

## Progress Summary:
- [What was accomplished in this session]
- [Current working state]

## Next Steps:
1. [What to do in next session]
2. [Blockers/unknowns to address]

## Context to Preserve:
- [Pattern discovered that applies to future work]
- [Constraint that shouldn't be forgotten]
- [Design principle that guides implementation]

## Files Modified:
- [File 1] — [What changed]
- [File 2] — [What changed]
```

**Part B: Three Roles Demonstration (Collaborative Extraction)**

**Round 1: AI as Teacher (Suggests Extraction Strategy)**

```markdown
STUDENT: "After 90 minutes, my session is at 87% context utilization.
I've made progress but still have work to do. How do I create a checkpoint?"

AI as Teacher:
"When context reaches 80%+ with incomplete work, create a checkpoint:

1. Extract architectural decisions made
2. Compress progress into bullet points
3. Capture patterns discovered
4. Note constraints/blockers
5. Write targeted next steps

This gives you 200K fresh context with key context recovered."

OBSERVATION: AI teaches extraction strategy
```

**Round 2: AI as Student (Learns What to Preserve)**

```markdown
STUDENT: "My decisions were: (1) database schema, (2) API structure, (3) error handling.
My patterns: camelCase naming, validation-before-processing. My blocker: performance testing."

AI as Student:
"I see your pattern now. For your type of project, decisions about data structure
matter most. Naming conventions matter for consistency. Blockers like performance
are critical to surface at session start.

For checkpoints, I should prioritize: Schema decisions > Patterns > Blockers."

OBSERVATION: AI learns what this student values in checkpoints
```

**Round 3: AI as Co-Worker (Iterates on Compression)**

```markdown
STUDENT: "Can we make it more concise? My last checkpoint was 800 tokens—wasteful."

AI as Co-Worker:
"Let's compress. Instead of full decision rationale, use:
- Decision: [one line]
- Why: [constraint that drove it]

Drop alternatives unless it's a hard-fought choice.

Let me compress your session: [Shows 450-token checkpoint]"

STUDENT: "Perfect. That's usable without waste."

OBSERVATION: Iteration converges on 450-token checkpoint that preserved everything needed
```

**Part C: Session Restart Procedure (Algorithm)**

```markdown
## Session Restart Workflow

**Trigger Decision:**
IF utilization > 80% AND session_duration > 60_minutes
THEN create checkpoint ELSE continue current session

**Checkpoint Creation:**
1. Extract 3-5 key architectural decisions made this session
2. Write one-line progress summary ("What got done?")
3. Identify 2-3 patterns discovered that apply to future work
4. Note any blockers or unknowns
5. Preview next session's tasks

**Compression Target:** <600 tokens

**Validation Check:**
"If I load this checkpoint and started fresh, could I resume effectively?"
IF yes → Checkpoint is complete
IF no → Add missing context

**Next Session Start:**
1. Load Foundation context (CLAUDE.md, architecture.md, decisions.md)
2. Load Current context (files for today's task)
3. Paste checkpoint (provides session history)
4. Ask AI: "Here's my checkpoint. Ready to resume?"
```

**Practice Exercises** (With AI Collaboration):

1. **Extraction Exercise**: Given a long conversation transcript
   - Identify 3-5 key decisions
   - Summarize progress in 3-4 bullets
   - List 2-3 patterns discovered
   - Create checkpoint

2. **Compression Challenge**: Compress your latest checkpoint further
   - Current: 700 tokens
   - Target: <600 tokens
   - Which details can be removed without losing critical context?

3. **Recovery Test**: Create checkpoint, restart session, validate recovery
   - Write checkpoint at 85% utilization
   - Start new session, load checkpoint
   - Ask AI: "What's my current status?"
   - Compare AI's understanding to actual progress

**Research Integration**:
- **Source**: Anthropic Article (Memory Generation Pipeline)
- **Concept**: Extraction + Consolidation (pull key facts, compress into reusable knowledge)
- **Application**: Students learn systematic compression for efficient context management

**Cognitive Load Validation**: 8 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Student creates checkpoint < 600 tokens from 90-minute session
- Checkpoint includes decisions, progress, patterns, blockers, next steps
- Student can explain what was extracted and why
- Three Roles demonstrated (AI teaches structure, student teaches priorities, iteration on compression)

**Try With AI** (Closure):
```markdown
## Try With AI: Create and Test a Checkpoint

Use your most recent development session:

Part 1 - Extraction:
Ask Claude Code: "Here's a transcript from my session that reached 85% context
utilization. Help me extract: (1) 3-5 key decisions, (2) progress summary,
(3) patterns discovered, (4) blockers to address."

Part 2 - Compression:
Take AI's output and compress to <600 tokens. What can you remove without
losing critical context?

Part 3 - Validation:
Create a NEW session. Paste the checkpoint as context. Ask:
"Based on this checkpoint, what's my current status? What should I do next?"

Does AI's understanding match your actual progress?
```

---

### LESSON 5: Context Isolation and Parallel Tasks (Layer 2: AI Collaboration, Three Roles)

**Pedagogical Role**: Decision framework for session management and task boundaries

**Learning Objectives** (CEFR B1):
- Evaluate task similarity and determine isolation needs
- Implement task isolation decision framework with scoring
- Recognize when separate sessions prevent pattern cross-contamination
- Orchestrate multi-session workflows

**Layer Architecture**: Layer 2 (AI Collaboration with Three Roles)
- **AI as Teacher**: Suggests similarity scoring framework (student didn't know)
- **AI as Student**: Learns student's task dependencies (student knows which tasks relate)
- **AI as Co-Worker**: Iterates on isolation decision for ambiguous cases

**Concepts Introduced** (Count: 9 — within B1 limit):
1. Task similarity definition (common domain, dependencies, patterns)
2. Similarity scoring framework (30 points per criterion)
3. Isolation decision threshold (< 50% similarity = separate session)
4. Pattern cross-contamination risk (wrong patterns applied to unrelated task)
5. Multi-session workflow architecture
6. Session independence benefits (smaller context windows, focused reasoning)
7. Context switching cost vs isolation benefit trade-off
8. Parallel task coordination
9. Research source: GitHub Spec (multi-session workflows)

**Content Structure**:

**Part A: Task Similarity Scoring (Conceptual Framework)**

**When are tasks similar?**
- Same business domain (authentication, payment, logging)
- Same data models (users, products, orders)
- Same external service (AWS, GitHub, Stripe)
- Same code routes/modules
- Shared test suite

**Similarity Scoring System** (Add points if true):
```markdown
Business Domain Similarity: +30 points
- Same feature area? (Yes = +30, No = 0)
- Example: "Add user registration" and "Add user login" = +30 (same domain: auth)

Data Model Overlap: +20 points
- Use same database models? (Yes = +20, No = 0)
- Example: Both tasks involve "users" table = +20

External Service Dependency: +20 points
- Use same external API? (Yes = +20, No = 0)
- Example: Both tasks call AWS S3 = +20

Code Route/Module Overlap: +15 points
- Modify same code module? (Yes = +15, No = 0)
- Example: Both tasks modify /auth/ directory = +15

Shared Test Suite: +15 points
- Run same tests? (Yes = +15, No = 0)
- Example: Both tasks affect integration tests = +15

---
TOTAL SIMILARITY: [Sum of matched criteria]

Interpretation:
- Score < 50: Separate sessions (isolated tasks)
- Score 50-70: Can combine, but watch for pattern conflicts
- Score > 70: Same session (tightly related tasks)
```

**Part B: Three Roles Demonstration (Collaborative Evaluation)**

**Round 1: AI as Teacher (Suggests Scoring Framework)**

```markdown
STUDENT: "I have two tasks: (1) Add user authentication, (2) Build search feature.
Should I work on both in same session or separate?"

AI as Teacher:
"I'd use a similarity score to decide. Rate these dimensions:
- Business domain: +30 if same
- Data models: +20 if same
- External services: +20 if same
- Code modules: +15 if same
- Test suite: +15 if same

If total < 50: separate sessions. If > 70: same session."

OBSERVATION: AI teaches scoring framework student can reuse
```

**Round 2: AI as Student (Learns Task Characteristics)**

```markdown
STUDENT: "Auth uses user table, search uses document table.
Auth needs password validation, search needs ranking algorithm.
They don't share code, tests, or external services."

AI as Student:
"I understand now. Your tasks are completely independent:
- Data: Different models (user vs document)
- Code: Different modules (auth vs search)
- Tests: Separate test suites
- External: Auth needs crypto service, search needs vector service

Similarity score: 0. Separate sessions strongly recommended."

OBSERVATION: AI learns why these specific tasks are unrelated
```

**Round 3: AI as Co-Worker (Iterates on Ambiguous Case)**

```markdown
STUDENT: "What about adding payment and refund features? Both use Stripe API,
but refund's logic is complex and separate from payment flow."

AI as Co-Worker:
"Let's score:
- Domain: +30 (both payment-related)
- Data: +20 (both payment records)
- External: +20 (both Stripe)
- Code: 0 (refund is separate module)
- Tests: 0 (separate test suites)
Total: 70 — borderline.

Option A: Same session if complexity is low
Option B: Separate sessions to keep payment logic focused

Which feels right for your codebase complexity?"

STUDENT: "Separate—payment is too sensitive to mix with refund logic."

AI: "Then split. Here's why that's smart: payment needs deep focus,
refund needs to be isolated to prevent introducing payment bugs."

OBSERVATION: Iteration on borderline case—neither alone had the answer
```

**Part C: Multi-Session Workflow Architecture (Pattern)**

```markdown
## Multi-Session Workflow for [Project]

**Session 1: Authentication**
- Focus: User registration, login, session management
- Context: Foundation + auth-specific files
- Decision: Work until 85% utilization or task complete

**Session 2: Search Feature**
- Focus: Document indexing, query ranking, results
- Context: Foundation + search-specific files
- Decision: Independent from auth—no pattern cross-contamination

**Session 3: Integration**
- Focus: Tie auth and search together in UI
- Context: Foundation + auth-summary + search-summary
- Notes: By now, both are stable; integration uses captured patterns

**Coordination Checkpoints:**
- End of Session 1: Checkpoint captures auth decisions
- End of Session 2: Checkpoint captures search decisions
- Start of Session 3: Load both checkpoints + new integration context
```

**Practice Exercises** (With AI Collaboration):

1. **Scoring Exercise**: Evaluate 5 task pairs with similarity framework
   - Task pair 1: User profile + admin dashboard
   - Task pair 2: Payment processing + invoice generation
   - Task pair 3: Email notifications + SMS alerts
   - For each, calculate score and recommend isolation

2. **Decision Making**: 3 ambiguous cases
   - Determine which need separate sessions
   - Justify using scoring framework
   - Design multi-session workflow if needed

3. **Workflow Design**: Plan 3-session project
   - What tasks happen in each session?
   - What checkpoints are created?
   - What context is shared between sessions?

**Research Integration**:
- **Source**: GitHub Spec (Multi-Session Workflows)
- **Concept**: Pattern cross-contamination (wrong patterns applied to unrelated task)
- **Application**: Students understand isolation as risk mitigation

**Cognitive Load Validation**: 9 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Student scores 4/5 task pairs correctly
- Student can explain why separate sessions prevent cross-contamination
- Student designs multi-session workflow for sample project
- Three Roles demonstrated (AI teaches scoring, student teaches task relationships, iteration on ambiguous cases)

**Try With AI** (Closure):
```markdown
## Try With AI: Evaluate Your Current Project

Describe your current development project to Claude Code:

Part 1 - Task Inventory:
Ask: "I have these tasks: [list 3-5 current/planned tasks].
Score the similarity between each pair using the scoring framework."

Part 2 - Isolation Decisions:
For each task pair that scored < 50:
Ask: "Why should [Task A] and [Task B] be in separate sessions?"

Part 3 - Workflow Design:
Ask: "Design a multi-session workflow for my project.
What happens in each session? What context is shared?"

Part 4 - Validation:
Ask: "What patterns could cross-contaminate if I combined [Task A] and [Task B]?"
```

---

### LESSON 6: Memory Files and Persistent Intelligence (Layer 3: Intelligence Design)

**Pedagogical Role**: Create reusable skill encapsulating memory management

**Learning Objectives** (CEFR B1):
- Design memory file architecture (CLAUDE.md, architecture.md, decisions.md)
- Implement update strategy that preserves intelligence across sessions
- Recognize what belongs in each memory file based on persistence value
- Create reusable skill for memory file management

**Layer Architecture**: Layer 3 (Intelligence Design)
- Students create reusable **SKILL**: `memory-file-architecture`
- Skill encodes decision framework for which memory file stores what
- Reusable across projects; applies Persona + Questions + Principles pattern

**Concepts Introduced** (Count: 8 — within B1 limit):
1. Three memory files (CLAUDE.md, architecture.md, decisions.md)
2. Persistence strategy (what survives session to session)
3. Update triggers (when to append/update)
4. Memory retrieval (how to use memory at session start)
5. Memory generation pipeline (extraction → consolidation → storage)
6. Project-specific vs universal memory
7. Skill design framework (Persona + Questions + Principles)
8. Memory skill reusability across projects

**Content Structure**:

**Part A: The Three Memory Files (Conceptual Foundation)**

**File 1: CLAUDE.md (Project Patterns)**
- **Purpose**: Store discovered patterns, preferences, conventions
- **Content**:
  - Coding conventions (naming, formatting, structure)
  - Common patterns discovered (error handling, validation, testing)
  - AI collaboration preferences (what you like from AI, what you don't)
  - Tool preferences (Claude Code settings, context strategies)
  - Project idioms (unique to this project)
- **Persistence**: Session-to-session; update when new pattern discovered
- **Size**: Grows with project; no hard limit
- **Example**:
  ```markdown
  # CLAUDE.md — Project Patterns for [Project Name]

  ## Naming Conventions
  - Variables: camelCase
  - Constants: UPPER_SNAKE_CASE
  - Classes: PascalCase

  ## Pattern: Error Handling
  - Always validate before processing
  - Use try/except at system boundary
  - Log errors with context, not just message

  ## AI Collaboration Preferences
  - I prefer complete solutions over scaffolding
  - Explain tradeoffs explicitly
  - Ask clarifying questions before suggesting
  ```

**File 2: architecture.md (System Design)**
- **Purpose**: Document how system is organized
- **Content**:
  - Component structure (modules, services, boundaries)
  - Key dependencies (what component depends on what)
  - Design constraints (scalability, performance, compliance)
  - Data flow (how information moves through system)
  - Integration points (how components connect)
- **Persistence**: Updates when architecture changes; relatively stable
- **Size**: 500-2000 tokens typically
- **Example**:
  ```markdown
  # architecture.md — System Design

  ## Components
  - API Layer: Handles HTTP requests
  - Business Logic: Core domain operations
  - Data Layer: Manages persistence

  ## Dependencies
  API Layer → Business Logic → Data Layer (one-directional)

  ## Design Constraints
  - Requests must complete in <500ms
  - Data must be consistent (not eventually consistent)
  - All external calls have timeout
  ```

**File 3: decisions.md (Architectural Decision Records)**
- **Purpose**: Record WHAT was decided and WHY
- **Content**: Chronological list of architectural decisions
  - Date of decision
  - Decision made
  - Why this choice (reasoning)
  - Alternatives considered (and why rejected)
  - Impact (what changes as a result)
- **Persistence**: Append-only; never delete (audit trail)
- **Size**: Grows chronologically; one entry per major decision
- **Example**:
  ```markdown
  # decisions.md — Architectural Decisions

  ## 2025-11-18: Use camelCase for variable names
  - **Decision**: Adopt camelCase consistently across codebase
  - **Why**: Matches JavaScript conventions; improves readability in IDE
  - **Alternatives**: snake_case (Python tradition), kebab-case (URL conventions)
  - **Impact**: Codebase now has consistent naming; new developers see pattern immediately

  ## 2025-11-17: Separate authentication into module
  - **Decision**: Extract auth logic to separate module
  - **Why**: Reusable across multiple services; easier to test; clearer boundary
  - **Alternatives**: Keep in main module (simpler initially, but less maintainable)
  - **Impact**: API module can now be used without auth dependencies
  ```

**Part B: Memory File Update Strategy**

```markdown
## Memory File Update Workflow

### Session Start:
1. Load CLAUDE.md (project patterns)
2. Load architecture.md (system structure)
3. Skim decisions.md (understand journey)
4. Inject into session context

### During Session:
- Discover new pattern? Note it (append to CLAUDE.md later)
- Architectural change? Note it (update architecture.md later)
- Make decision? Note it (add to decisions.md)

### Session End (if significant work):
1. **Append to CLAUDE.md**: Any patterns discovered this session
2. **Update architecture.md**: If architecture changed
3. **Append to decisions.md**: All major decisions made this session
   - Include date, decision, why, alternatives, impact

### Update Validation:
- CLAUDE.md is concise, actionable, pattern-focused
- architecture.md is current and reflects actual system
- decisions.md is complete audit trail (never overwrite)
```

**Part C: Skill Design — Memory-File-Architecture (Reusable Component)**

```markdown
# Skill: Memory-File-Architecture

## Persona
"Think like a knowledge management architect optimizing for persistence,
retrieval, and accumulation across multiple sessions."

## Questions (Analysis Framework)
1. **Persistence Value**: Will this information be useful in future sessions?
   - If yes → Store in memory file
   - If no → Document in session notes only

2. **Retrieval Frequency**: How often will this be accessed?
   - Pattern (frequent) → CLAUDE.md
   - Structure (less frequent) → architecture.md
   - Decisions (reference) → decisions.md

3. **Mutation Rate**: How often does this change?
   - Changes frequently → CLAUDE.md (append-only new patterns)
   - Changes occasionally → architecture.md (update when structure shifts)
   - Never changes → decisions.md (append-only, never rewrite)

4. **Discoverability**: How easy is it to find what you need?
   - Patterns grouped by category (validation, error handling, naming)
   - Architecture organized by component (API, Logic, Data)
   - Decisions chronological with clear headers

## Principles
- **Persistence First**: If it's valuable once, it's valuable forever
- **Append-Only**: Never delete decisions; add new patterns; update structure
- **Injection Strategy**: All three files loaded at session start
- **Minimal Overhead**: Updates take <5 minutes; not a documentation burden
```

**Practice Exercises** (Skill Creation + Application):

1. **Skill Creation Exercise**: Design memory-file-architecture skill for your project
   - Create CLAUDE.md with current patterns
   - Create architecture.md with system structure
   - Create decisions.md with 2-3 major decisions to date
   - Document update strategy

2. **Application Exercise**: Use your memory files in a new session
   - Load all three files
   - Start work on new task
   - At session end, update files with new patterns/decisions
   - Validate files are still useful for next session

3. **Skill Reuse Exercise**: Apply memory-file-architecture skill to different project
   - Use same file structure, but adapt content for new project
   - Verify skill transfers without modification

**Research Integration**:
- **Source**: Anthropic Article + Google PDF (Memory Generation Pipeline)
- **Concept**: Extraction → Consolidation → Storage → Retrieval
- **Application**: Students design memory system that persists intelligence

**Cognitive Load Validation**: 8 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Student creates CLAUDE.md, architecture.md, decisions.md for project
- Files follow structure from lesson
- Skill documentation includes Persona + Questions + Principles
- Student applies skill to update memory files after a work session
- Memory files are discoverable and actionable for next session

**Try With AI** (Closure):
```markdown
## Try With AI: Design Your Memory File System

Use Claude Code to build your memory files:

Part 1 - Create CLAUDE.md:
Ask: "I'm starting a new project. Help me identify 5-7 patterns
I should establish in CLAUDE.md for consistency. What patterns matter most?"

Part 2 - Create architecture.md:
Ask: "Here's my project structure [describe].
Help me design architecture.md with clear components and dependencies."

Part 3 - Create decisions.md:
Ask: "I've made these architectural decisions so far: [list].
Help me format them in decisions.md with Date, Decision, Why, Alternatives, Impact."

Part 4 - Update Strategy:
Ask: "When should I update each file? What triggers an update to CLAUDE.md
vs architecture.md vs decisions.md?"

Save all three files to your project. Load them at start of next session.
```

---

### LESSON 7: Tool Selection Framework (Layer 3: Intelligence Design)

**Pedagogical Role**: Create reusable skill for AI tool selection decision-making

**Learning Objectives** (CEFR B1):
- Evaluate project characteristics against tool constraints
- Select appropriate tool (Claude Code vs Gemini CLI) based on context requirements
- Understand trade-offs (deep reasoning vs broad exploration)
- Create reusable skill for tool selection across projects

**Layer Architecture**: Layer 3 (Intelligence Design)
- Students create reusable **SKILL**: `tool-selection-framework`
- Skill provides decision logic for Claude Code (200K context, deep reasoning) vs Gemini (2M context, exploration)
- Framework applies to current tools; extensible for future tools

**Concepts Introduced** (Count: 9 — within B1 limit):
1. Tool capabilities comparison (context window, reasoning depth, cost)
2. Codebase size assessment (metric for context needs)
3. Task complexity evaluation (reasoning depth required)
4. Multi-phase task decomposition (explore → implement)
5. Cost considerations (token budget constraints)
6. Switching strategy (when to use multiple tools in sequence)
7. Skill design for decision frameworks
8. Extensibility (how to adapt framework for new tools)
9. Research source: Spec FR-003 (guardrails and decision criteria)

**Content Structure**:

**Part A: Tool Comparison Matrix (Conceptual Foundation)**

```markdown
| Factor | Claude Code | Gemini CLI |
|--------|------------|------------|
| Context Window | 200K standard | 2M standard |
| Best For | Deep reasoning, focused work | Broad exploration, large codebases |
| Reasoning Depth | Excellent (latest models) | Good (sufficient for most tasks) |
| Cost | Lower per session | Higher (larger context) |
| Latency | Faster | Potentially slower (larger context) |
| Specialization | AI-native development tasks | General-purpose exploration |
```

**Tool Selection Decision Framework**:

**Claude Code is optimal when:**
- Codebase < 50K lines
- Task requires deep architectural reasoning
- Context budget is limited (cost-sensitive)
- Work is focused (one feature, one bug, one refactor)
- You want rapid iteration

**Gemini CLI is optimal when:**
- Codebase 50K+ lines
- Task is broad exploration (understanding unknown system)
- Task requires pattern analysis across large codebase
- Context budget is large (exploration prioritized over cost)
- Multi-phase: explore broadly, THEN use Claude Code to implement focused changes

**Switching Strategy (Multi-Phase):**
```
Phase 1 (Gemini): Broad exploration
- "Understand architecture of 200K-line codebase"
- "What patterns are used for error handling?"
- Create summary document

Phase 2 (Claude Code): Focused implementation
- Load summary from Gemini
- Implement specific feature
- Deep reasoning on edge cases
```

**Part B: Decision Framework Algorithm**

```markdown
## Tool Selection Decision Tree

**START: Assess Codebase Size**

IF codebase < 50K lines:
  → Claude Code is sufficient
  → Use Claude Code for all work
  → [Skip to END]

ELSE IF codebase 50K-500K lines:
  → Size is manageable for Claude Code, but consider task type

  IF task type is "exploration" (understand unknown system):
    → Use Gemini first (broad understanding)
    → Then Claude Code (focused implementation)
    → [Go to Phase 2 below]

  ELSE IF task type is "focused" (implement one feature):
    → Use Claude Code directly
    → Load relevant architecture files
    → [Skip to END]

ELSE IF codebase > 500K lines:
  → Claude Code context insufficient
  → Phase 1: Use Gemini for exploration
  → Phase 2: Use Claude Code for implementation
  → [Go to Phase 2 below]

**PHASE 2: Multi-Tool Workflow**

Step 1: Gemini Session
- Load 500K-line codebase
- Explore: "What's the architecture?"
- Generate summary document (~5K tokens)
- Create checkpoint

Step 2: Claude Code Session
- Load Gemini summary (5K tokens)
- Load Foundation/Current context (20K tokens)
- Implement feature with deep reasoning (175K available)

**END: Execute with selected tool**
```

**Part C: Skill Design — Tool-Selection-Framework (Reusable Component)**

```markdown
# Skill: Tool-Selection-Framework

## Persona
"Think like a resource optimization engineer allocating specialized tools
to tasks where each tool excels."

## Questions (Analysis Framework)
1. **Codebase Size**: How many lines of code?
   - < 50K → Claude Code
   - 50K-500K → Depends on task type
   - > 500K → Gemini for exploration, Claude Code for implementation

2. **Task Complexity**: What's the nature of this task?
   - Exploration (understand unknown) → Gemini
   - Implementation (focused feature) → Claude Code
   - Refactor (system-wide changes) → Gemini first, Claude Code second

3. **Context Efficiency**: How much context will this need?
   - Tight budget → Claude Code (cheaper, faster)
   - Liberal budget → Gemini (broader understanding)

4. **Time Sensitivity**: Is this time-critical?
   - Yes → Claude Code (faster)
   - No → Gemini (thorough understanding)

5. **Expertise Available**: Do you understand the codebase?
   - Yes → Claude Code (focused work)
   - No → Gemini (learning curve)

## Principles
- **Right Tool for Job**: Don't use Gemini when Claude Code suffices (waste of tokens)
- **Specialization Value**: Each tool has optimal context window for its task type
- **Phased Approach**: Exploration + Implementation as two-phase workflow
- **Cost Awareness**: Token budget influences tool selection for large codebases
- **Future Extensibility**: Framework applies when new tools available (GPT-4, Opus, etc.)
```

**Practice Exercises** (Skill Creation + Application):

1. **Framework Application**: Evaluate 5 scenarios
   - Scenario A: 30K-line project, add new feature
   - Scenario B: 200K-line legacy system, understand architecture
   - Scenario C: 500K-line platform, implement high-priority bug fix
   - Scenario D: Unknown 100K-line codebase, establish patterns
   - Scenario E: 2M-line system, replace authentication
   - For each: Recommend tool, justify decision

2. **Multi-Phase Exercise**: Design Gemini → Claude Code workflow
   - Use Gemini for broad exploration of large system
   - Capture findings in summary document
   - Switch to Claude Code with summary for focused implementation

3. **Skill Creation**: Document tool-selection-framework skill
   - Persona statement
   - Questions (analysis framework)
   - Principles (decision boundaries)
   - Examples (common scenarios)

**Research Integration**:
- **Source**: Spec FR-003 (Guardrails and Decision Criteria)
- **Concept**: Explicit decision rules (codebase size, reasoning depth, complexity)
- **Application**: Students develop decision framework reusable across projects

**Cognitive Load Validation**: 9 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Student recommends correct tool for 4/5 scenarios
- Student can explain reasoning using decision framework
- Student designs multi-phase workflow (Gemini → Claude Code) for large project
- Skill includes Persona + Questions + Principles

**Try With AI** (Closure):
```markdown
## Try With AI: Evaluate Your Tool Choices

Describe your current project to Claude Code:

Part 1 - Tool Selection:
Ask: "My project is [size] lines of code.
My current task is [describe task type].
Should I use Claude Code or Gemini CLI? Explain the tradeoff."

Part 2 - Decision Framework:
Ask: "Using the tool-selection framework, evaluate this scenario:
[describe different scenario]. What tool would you recommend?"

Part 3 - Multi-Phase Design:
If your codebase is > 100K lines, ask:
"Design a Gemini → Claude Code workflow for [specific task].
What would I explore in Gemini? What would I implement in Claude Code?"

Part 4 - Validation:
Ask: "Are there cases where I'd want to use BOTH tools? When and why?"
```

---

### LESSON 8: Hands-On Debugging and Optimization (Layer 2: Validation & Integration)

**Pedagogical Role**: Integrated practice applying Lessons 1-7 to real degradation scenarios

**Learning Objectives** (CEFR B1):
- Diagnose context degradation from symptoms
- Apply accumulated techniques to recover degraded session
- Optimize context strategy under different constraints
- Validate integrated understanding of all prior lessons

**Layer Architecture**: Layer 2 → 3 (Bridge)
- Primarily Layer 2 (validation practice)
- Applies accumulated techniques from Lessons 1-7
- Prepares for Layer 4 (capstone integration)

**Concepts Introduced** (Count: 8 — within B1 limit):
1. Integrated diagnosis (symptom → root cause → solution)
2. Recovery procedures (session restart, compression, loading strategy adjustment)
3. Optimization under constraints (cost limits, time limits, token budgets)
4. Production considerations (testing context strategies, security)
5. Fallback strategies (what to do when degradation can't be prevented)
6. Multi-session recovery (checkpoint + restart + memory files)
7. Continuous improvement (iterate on context strategy)
8. Debugging workflow (observe → hypothesis → test → validate)

**Content Structure**:

**Part A: Integrated Debugging Process**

```markdown
## Debugging Context Degradation: Complete Workflow

**STEP 1: Observe Symptoms (Lesson 2 concepts)**
- Read conversation transcript
- Mark degradation symptoms present
- Estimate when degradation started

**STEP 2: Diagnose Root Cause**
- Symptom: Repetitive suggestions
  → Root cause: Context window saturation (Lesson 1 understanding)

- Symptom: Forgotten patterns
  → Root cause: Foundation context evicted; current context too large (Lesson 3 understanding)

- Symptom: Generic responses
  → Root cause: Project-specific context lost due to utilization (Lesson 1-3 understanding)

**STEP 3: Evaluate Context Strategy (Lesson 3, 5 concepts)**
- Current loading strategy too aggressive?
- Current task isolation insufficient?
- Foundation too large or too small?

**STEP 4: Design Recovery (Lesson 4 concepts)**
- Create checkpoint from degraded session
- Identify what to preserve (decisions, patterns, blockers)
- Plan restart with improved strategy

**STEP 5: Implement Optimization (Lessons 3, 5, 6 concepts)**
- Adjust Foundation/Current/On-Demand allocation
- Consider task isolation for future sessions
- Update memory files with lessons learned

**STEP 6: Validate (Lesson 8 continuous improvement)**
- Test improved strategy on similar task
- Measure: Is degradation prevented? Is utilization < 70%?
- Adjust if necessary; iterate
```

**Part B: Hands-On Scenarios (4 Degradation Cases)**

**Scenario 1: Context Saturation in Long Session**

*Situation*: 2-hour session on API design. Started at 0% utilization. Now at 87%.
- Symptom: Responses are shorter; AI suggests generic patterns
- Earlier: AI had specific architectural insights
- Now: AI gives boilerplate advice

*Your Task*: Diagnose and recover
1. Apply Lesson 1 concepts: Estimate when utilization crossed 70%
2. Apply Lesson 2 concepts: Identify specific degradation symptoms
3. Apply Lesson 3 concepts: Was your Foundation/Current/On-Demand strategy appropriate?
4. Apply Lesson 4 concepts: Create checkpoint, plan restart
5. Design improved strategy for next session

*Success Criteria*: Recovery plan prevents re-degradation in similar future sessions

---

**Scenario 2: Pattern Cross-Contamination Across Tasks**

*Situation*: Started authentication feature (used snake_case).
Mid-session, switched to API design (team uses camelCase).
AI now suggests snake_case for API (WRONG—pattern contamination).

*Your Task*: Prevent cross-contamination
1. Apply Lesson 5 concepts: Were these tasks sufficiently different for isolation?
2. Apply Lesson 6 concepts: Did CLAUDE.md capture naming conventions? Did you load it?
3. Diagnosis: Could separate sessions have prevented this?
4. Design task isolation strategy for future

*Success Criteria*: Explain how isolation framework (Lesson 5) would prevent this error

---

**Scenario 3: Unexpected Context Eviction**

*Situation*: Loaded files X, Y, Z as Foundation.
After 30 minutes, AI asks "What's your project structure?" (File Z was about architecture).
AI should have remembered; didn't.

*Your Task*: Understand eviction and prevent it
1. Apply Lesson 1 concepts: Was Foundation too large? (Context eviction typically happens >70% util)
2. Apply Lesson 3 concepts: Evaluate Foundation/Current split—was Foundation appropriate?
3. Apply Lesson 6 concepts: Should key info be in architecture.md instead?
4. Design Foundation/Current strategy that prevents eviction

*Success Criteria*: Propose Foundation structure that keeps critical info accessible

---

**Scenario 4: Multi-Session Coordination Failure**

*Situation*: Session 1 created checkpoint. Session 2 loaded checkpoint + new context.
But AI in Session 2 asks about decisions from Session 1 (should have been in checkpoint).
Checkpoint was insufficient.

*Your Task*: Improve checkpoint quality
1. Apply Lesson 4 concepts: What should checkpoint have captured?
2. Apply Lesson 6 concepts: Where should this have gone in memory files instead?
3. Re-design checkpoint with better extraction/consolidation
4. Plan memory file updates that prevent this next time

*Success Criteria*: Redesigned checkpoint/memory strategy ensures decisions persist correctly

**Part C: Optimization Under Constraints**

```markdown
## Constraint-Based Optimization Exercises

**Constraint 1: Cost Limit ($5/month token budget)**
- Challenge: Large codebase (200K lines) with tight budget
- Apply Tool Selection (Lesson 7): When to use Claude Code vs Gemini?
- Solution: Use Gemini sparingly (exploration only); Claude Code for implementation
- Optimize: Progressive loading (Lesson 3) reduces context waste

**Constraint 2: Time Limit (15 minutes for AI collaboration)**
- Challenge: Deep issue requiring investigation; limited time
- Apply Priority: What information MUST load in first 3 minutes?
- Apply Loading Strategy (Lesson 3): Foundation only; On-Demand for follow-ups
- Result: Get 80% of value in 15 minutes; continue in next session

**Constraint 3: Token Limit (150K context available — half of normal)**
- Challenge: Half the normal context window
- Apply Compression (Lesson 4): More frequent checkpoints, more aggressive summarization
- Apply Isolation (Lesson 5): Split tasks earlier; smaller sessions
- Apply Tool Selection (Lesson 7): Claude Code not suitable; use Gemini for this size
- Result: Adjust workflow for constrained environment
```

**Practice Exercises** (Hands-On Application):

1. **Case Study Analysis**: For each of 4 scenarios above
   - Diagnose the root cause
   - Apply relevant lessons (1-7)
   - Design recovery + prevention

2. **Integrated Debugging Workflow**: Real project session
   - Work with AI until reaching 75%+ utilization
   - Diagnose degradation symptoms
   - Create checkpoint, restart, implement recovery
   - Validate: Does recovery prevent re-degradation?

3. **Constraint-Based Optimization**: Solve 3 challenges
   - Design strategy for $5/month budget
   - Design strategy for 15-minute time window
   - Design strategy for 150K token limit
   - Justify choices using frameworks from Lessons 1-7

**Research Integration**:
- **Source**: Google PDF (Production Considerations)
- **Concept**: Testing context strategies before production, security considerations
- **Application**: Students think operationally about context management

**Cognitive Load Validation**: 8 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Student correctly diagnoses 3/4 scenarios
- Student applies relevant lesson concepts to each scenario
- Student creates recovery plan using multiple techniques
- Student optimizes context strategy under given constraints
- Student demonstrates integrated understanding of Lessons 1-7

**Try With AI** (Closure):
```markdown
## Try With AI: Simulate a Degraded Session and Recover

Part 1 - Simulate Degradation:
Provide Claude Code with a transcript from a real session (yours or sample).
Ask: "Analyze this transcript.
At what point did context degradation become evident?
What symptoms appeared?"

Part 2 - Root Cause Analysis:
Ask: "What caused this degradation?
Was it Foundation too large? Current context misaligned?
Poor task isolation?"

Part 3 - Recovery Design:
Ask: "Create a checkpoint from this session.
What are the 3-5 key pieces of information to preserve?"

Part 4 - Prevention Strategy:
Ask: "Design a context strategy that would have prevented this degradation.
Use Foundation/Current/On-Demand allocation. Justify each decision."

Part 5 - Validation:
Ask: "If I applied your prevention strategy to a similar future session,
would degradation be prevented?"
```

---

### LESSON 9: Capstone — Spec-Driven Orchestration (Layer 4: Specification-Only)

**Pedagogical Role**: Integrate all accumulated techniques in specification-driven project

**Learning Objectives** (CEFR B1):
- Write complete system specification without implementation
- Orchestrate accumulated techniques (loading, compression, isolation, memory files, tool selection)
- Design context-aware development strategy for complex project
- Demonstrate Layer 4 methodology (spec-first, not code-first)

**Layer Architecture**: Layer 4 (Spec-Driven Integration)
- **Specification FIRST** (no code until spec complete)
- Orchestrate techniques from Lessons 1-8
- Compose techniques into coherent strategy

**Concepts Introduced** (Count: 8 — within B1 limit):
1. Specification structure (Intent, Success Criteria, Requirements, Architecture, Algorithms, Non-Goals)
2. Context architecture section (how context will be managed)
3. Session strategy (when to split, restart, compress)
4. Memory file design (what persists across sessions)
5. Tool selection strategy (Claude Code vs Gemini)
6. Multi-session workflow (phase breakdown)
7. Risk mitigation (how to prevent degradation)
8. Validation criteria (how to measure success)

**Content Structure**:

**Part A: Capstone Project Description**

**The Challenge**: Design a specification for a **Context-Aware Development Tool**

This tool helps developers manage context efficiently across long-term projects.

**Scope** (what the tool does):
- Tracks context utilization in real-time
- Suggests when to create checkpoints
- Recommends when to restart sessions
- Manages memory files (CLAUDE.md, architecture.md, decisions.md)
- Monitors degradation symptoms
- Provides reports on context efficiency

**What you're NOT doing**:
- Building/implementing the tool
- Writing code for any component
- Creating UI mockups
- Selecting specific technologies

**What you ARE doing**:
- Writing a complete specification that WOULD drive implementation
- Designing context management strategy for a tool of this scope
- Orchestrating all techniques from Lessons 1-8
- Making architectural decisions about how the system manages its OWN context

**Part B: Specification Template**

```markdown
# Specification: Context-Aware Development Tool

## 1. Intent

**Vision**: A tool that helps developers engineer context windows like DevOps engineers manage infrastructure—systematically, measurably, continuously.

**Problem Statement**:
Currently, developers manage context manually. They watch for degradation symptoms, create checkpoints ad-hoc, restart sessions when frustrated. This is error-prone and wasteful.

**Solution Approach**:
Automate context management decisions while keeping developer in control. Provide visibility into context utilization. Recommend actions (checkpoint, restart, isolation) before degradation is severe.

**Developer Benefit**:
- 50% fewer context-related bugs (forgotten patterns, cross-contamination)
- 30% more productive sessions (better loading strategies)
- Confidence in multi-session workflows

## 2. Success Criteria (Measurable, Falsifiable)

**SC-001**: Degradation Detection
- Tool correctly identifies degradation symptoms in conversation transcripts
- Measurement: 95%+ accuracy on provided test transcripts

**SC-002**: Checkpoint Quality
- Tool-generated checkpoints preserve critical context
- Measurement: Session restart with checkpoint achieves 90%+ restoration of prior context

**SC-003**: Session Optimization
- Recommended loading strategies reduce context waste
- Measurement: Utilization stays < 70% under recommended strategy

**SC-004**: Memory File Management
- Tool updates CLAUDE.md/architecture.md/decisions.md correctly
- Measurement: Memory files are usable for session recovery

**SC-005**: Multi-Session Coordination
- Task isolation recommendations prevent cross-contamination
- Measurement: Pattern consistency maintained across isolated sessions

## 3. Functional Requirements

### FR-001: Context Monitoring Component
- Tracks tokens used in current session (estimate or measure)
- Calculates utilization percentage
- Triggers alerts at 70%, 80%, 90%
- Provides utilization timeline (chart of usage over time)

### FR-002: Degradation Detection Component
- Scans conversation for 5 degradation symptoms
- Assigns severity score (1-5)
- Recommends action (continue, checkpoint, restart)
- Explains why degradation detected

### FR-003: Checkpoint Management Component
- Extracts key decisions from session
- Consolidates progress summary
- Identifies patterns to preserve
- Creates <600-token checkpoint

### FR-004: Session Strategy Recommender
- Evaluates task similarity (Lesson 5 framework)
- Recommends isolation (new session) or continuation
- Explains reasoning

### FR-005: Memory File Manager
- Reads/updates CLAUDE.md (patterns)
- Reads/updates architecture.md (structure)
- Reads/updates decisions.md (decisions)
- Validates files are coherent and up-to-date

### FR-006: Tool Selection Advisor
- Evaluates codebase size
- Assesses task complexity
- Recommends Claude Code or Gemini
- Suggests multi-phase approach if needed

### FR-007: Multi-Session Orchestrator
- Chains sessions with checkpoint → recovery
- Coordinates tool switching (Claude Code ↔ Gemini)
- Tracks progress across sessions
- Reports context efficiency metrics

## 4. Architecture

### System Components

**Monitor**: Observes context utilization in real-time
- Input: Session transcript (incremental)
- Output: Utilization %, degradation symptoms
- Updates: Continuous (every AI response)

**Analyzer**: Diagnoses context health
- Input: Utilization %, symptoms, session history
- Output: Severity assessment, recommended actions
- Updates: Triggered by alert thresholds

**Recommender**: Suggests optimizations
- Input: Project structure, task type, constraints
- Output: Loading strategy, task isolation, checkpoints
- Updates: At session start, mid-session if degradation detected

**Memory Manager**: Persists intelligence
- Input: Decisions, patterns, architecture from session
- Output: Updated CLAUDE.md, architecture.md, decisions.md
- Updates: At session end or on demand

**Orchestrator**: Coordinates multi-session workflow
- Input: Task queue, tool availability, context constraints
- Output: Session plan (which task → which tool → which session)
- Updates: As tasks complete, constraints change

### Data Flow

```
Session Start
  ├─→ Load Foundation context
  ├─→ Load Current context
  ├─→ Load checkpoint (if resuming)
  └─→ Monitor begins tracking

During Session
  ├─→ Monitor tracks utilization
  ├─→ Analyzer detects symptoms
  ├─→ Recommender suggests checkpoint/restart
  └─→ Developer acts (or continues)

Session End
  ├─→ Create checkpoint
  ├─→ Update Memory files
  └─→ Orchestrator schedules next session
```

### Technology Stack (Conceptual)

- **Data Storage**: Markdown files (CLAUDE.md, architecture.md, decisions.md)
- **Session Interface**: Claude Code (tool students know)
- **Analysis**: Patterns library (string matching for symptoms)
- **Orchestration**: Workflow engine (state machine for multi-session)

## 5. Algorithms (Plain English)

### Algorithm 1: Degradation Detection
```
FOR EACH response in conversation:
  Check for symptom: Repetitive suggestions
    IF suggestion matches earlier suggestion → +1 point
  Check for symptom: Forgotten patterns
    IF response violates earlier constraint → +1 point
  Check for symptom: Generic response
    IF response lacks project-specific context → +1 point
  Check for symptom: Performance degradation
    IF response shorter or slower than baseline → +1 point
  Check for symptom: Lost context
    IF response asks for earlier-provided information → +1 point

Degradation Score = Points / 5 (0-1 range)

IF score > 0.3 AND utilization > 70%:
  → Alert: "Context degradation detected"
  → Recommend: Checkpoint or restart
ELSE IF score > 0.1:
  → Alert: "Monitor for degradation"
  → Recommend: Prepare checkpoint
```

### Algorithm 2: Session Isolation Decision
```
GIVEN: Task A (current), Task B (proposed new task)

FOR EACH criterion:
  Score += 30 if same business domain
  Score += 20 if same data models
  Score += 20 if same external services
  Score += 15 if same code modules
  Score += 15 if shared test suite

Total Similarity Score = Sum of matched criteria

IF Score < 50:
  → Recommend: New session (isolated)
ELSE IF Score 50-70:
  → Recommend: Same session, monitor for contamination
ELSE:
  → Recommend: Same session (closely related)
```

### Algorithm 3: Progressive Loading Strategy
```
GIVEN: Project files, task description

Classify files:
  Foundation = files needed for ALL tasks
    Target: 10-15% of context (20K tokens)
  Current = files needed for THIS task
    Target: 20-30% of context (40K-60K tokens)
  On-Demand = reference files (fetch if requested)
    Target: Reserve 30% of context (60K tokens)

Validation:
  IF Foundation + Current + Reserve > 100%:
    → Reduce Foundation (only essentials)
    → Reduce Current (defer to On-Demand)
  ELSE:
    → Strategy is valid
```

## 6. Non-Goals

- **NG-001**: Real-time token measurement
  - Tool estimates tokens; doesn't measure actual count
  - Estimation is sufficient for decision-making

- **NG-002**: Implementing components
  - This spec defines the WHAT, not the HOW
  - Implementation is future work

- **NG-003**: UI/UX design
  - Tool is specification-only
  - UI would be designed after specification approved

- **NG-004**: Tool-specific features
  - No Claude Code plugins, custom tools, or extensions
  - Uses standard Claude Code capabilities only

- **NG-005**: Automated code generation
  - Tool recommends, doesn't generate
  - Developer makes final decisions

## 7. Acceptance Tests

### Test-001: Degradation Detection Accuracy
**Given**: Sample transcripts (5 healthy, 5 degraded)
**When**: Tool analyzes transcripts
**Then**: Correctly identifies 4/5 degraded sessions, 4/5 healthy sessions

### Test-002: Checkpoint Quality
**Given**: Degraded session transcript
**When**: Tool extracts checkpoint
**Then**: Checkpoint < 600 tokens AND preserves key decisions

### Test-003: Loading Strategy Optimization
**Given**: Project with 50 files
**When**: Tool recommends Foundation/Current/On-Demand split
**Then**: Split is justified AND utilization stays < 70%

### Test-004: Task Isolation Recommendation
**Given**: Two tasks with varying similarity scores
**When**: Tool evaluates isolation need
**Then**: Recommendation prevents pattern cross-contamination

### Test-005: Memory File Consistency
**Given**: Updated CLAUDE.md, architecture.md, decisions.md
**When**: Tool validates files
**Then**: Files are consistent AND usable for next session

## 8. Implementation Plan

### Phase 1: Degradation Detection (Week 1)
- Implement symptom detection algorithm
- Test against sample transcripts
- Validate accuracy > 90%

### Phase 2: Context Monitoring (Week 2)
- Implement utilization tracking
- Implement alert system (70%, 80%, 90%)
- Test against real sessions

### Phase 3: Checkpoint Management (Week 2-3)
- Implement extraction algorithm
- Implement consolidation algorithm
- Validate checkpoint quality

### Phase 4: Memory File Manager (Week 3)
- Implement CLAUDE.md/architecture.md/decisions.md updates
- Implement validation (consistency checks)
- Test with real projects

### Phase 5: Session Orchestrator (Week 4)
- Implement task isolation decision framework
- Implement loading strategy recommender
- Implement tool selection advisor
- Implement multi-session workflow

### Phase 6: Testing & Validation (Week 4-5)
- Acceptance tests for all components
- Integration testing
- Real-world beta testing

## 9. Success Metrics

**Development Productivity**:
- Developers spend <10% of session managing context (automated)
- Developers focus 90%+ on actual work

**Context Efficiency**:
- Average utilization: < 70% (prevents degradation)
- Checkpoints preserve 90%+ of critical context
- Session restarts take < 2 minutes

**Error Reduction**:
- Context-related bugs decrease 50%
- Pattern cross-contamination prevented 95%+
- Forgotten context incidents decrease 80%+

## 10. Constraints & Assumptions

**Constraints**:
- Specification-only; no code implementation
- Works with Claude Code (200K context limit)
- Memory files are Markdown (human-readable)

**Assumptions**:
- Developers will use memory file system
- Developers will accept tool recommendations
- Projects have clear task boundaries (enables isolation)

---

## Capstone Completion Checklist

- [ ] Specification has Intent section with clear problem/solution
- [ ] Success Criteria are measurable and falsifiable (not vague)
- [ ] Functional Requirements cover all major components
- [ ] Architecture section explains how components interact
- [ ] Algorithms are in plain English (no code, but precise)
- [ ] Non-Goals explicitly define scope boundaries
- [ ] Acceptance Tests validate specification
- [ ] Implementation Plan shows phased approach
- [ ] Success Metrics define how to measure value
- [ ] Specification could drive implementation without re-interpretation
```

**Part C: Capstone Submission Guidelines**

```markdown
## Capstone Submission Requirements

**Length**: 3-5 pages (1500-2500 words)
- Don't pad with unnecessary sections
- Every section serves a purpose
- Quality over length

**Format**: Markdown file named `capstone-spec.md`

**Evaluation Criteria**:
1. **Specification Completeness** (40%):
   - Intent is clear (problem, solution, benefit)
   - Requirements cover all major functionality
   - Architecture shows component interactions
   - Algorithms are precise (plain English)

2. **Context Engineering Application** (30%):
   - Specification incorporates lessons from Chapters 1-8
   - Context management strategy is explicit
   - Multi-session workflow is designed
   - Memory persistence strategy is detailed

3. **Implementation Feasibility** (20%):
   - Specification could drive implementation
   - Acceptance tests validate claims
   - No ambiguous requirements
   - Non-goals clarify scope

4. **Personal Insights** (10%):
   - Reflection on how Lessons 1-8 apply
   - Own examples from your projects
   - Clarity of your thinking about context engineering

**What NOT to include**:
- Implementation code (zero code)
- UI mockups or diagrams
- Technology stack recommendations
- Budget or resource estimates
- Management timelines
```

**Practice Exercise** (Specification Writing):

1. **Outline Phase** (30 minutes)
   - Brainstorm: What are the tool's major components?
   - Organize: How do they interact?
   - Sequence: What's the build order?

2. **Draft Phase** (2 hours)
   - Write Intent (problem, solution, benefit)
   - Write Success Criteria (3-5, each measurable)
   - Write Functional Requirements (main components)
   - Sketch Architecture (components + data flow)

3. **Algorithm Phase** (1 hour)
   - Translate 3 key algorithms to plain English
   - Validate they could be implemented (not pseudo-code, but precise)

4. **Polish Phase** (1 hour)
   - Add Non-Goals (clarify scope)
   - Add Acceptance Tests (validate specification)
   - Proofread for clarity

5. **Validation Phase** (30 minutes)
   - Would someone else understand what to build?
   - Are requirements ambiguous?
   - Can you justify every design decision?

**Research Integration**:
- All lessons 1-8 integrated into capstone project
- Orchestration of accumulated techniques
- Demonstration of Layer 4 (spec-driven) thinking

**Cognitive Load Validation**: 8 concepts ✓ (within B1 limit of 7-10)

**Acceptance Test**:
- Specification 3-5 pages (not padded)
- Intent, Success Criteria, Requirements, Architecture all present
- Zero code; algorithms in plain English
- Acceptance tests validate specification
- Student can defend design decisions
- Specification could drive implementation without reinterpretation

**Try With AI** (Closure):
```markdown
## Try With AI: Validate Your Capstone Specification

Use Claude Code to review your draft specification:

Part 1 - Completeness Check:
Ask: "Review this specification.
Does it have: Intent, Success Criteria, Functional Requirements,
Architecture, Algorithms, Non-Goals, Acceptance Tests?
What's missing?"

Part 2 - Ambiguity Detection:
Ask: "Identify any ambiguous requirements or unclear specifications.
Which sections would confuse an implementer?"

Part 3 - Implementation Feasibility:
Ask: "Could an engineer build this from your specification without
asking you clarifying questions? What's still unclear?"

Part 4 - Design Validation:
Ask: "Do my design decisions make sense? Are there obvious alternatives
I should have considered?"

Part 5 - Self-Reflection:
Ask: "What did you learn about context engineering by writing this specification?"

Revise based on feedback. Iterate until specification is clear and complete.
```

---

## III. Skill Proficiency Mapping

### CEFR B1 Proficiency Alignment

**B1 Definition**: Intermediate independent user
- Can accomplish routine tasks in familiar contexts
- Can describe experiences, opinions, with reasonable accuracy
- Can handle social communication adequately
- Can reason about systems and trade-offs

**Chapter 11 B1 Application**:
- Students can estimate context utilization (routine task in coding)
- Students can describe context degradation symptoms (communicate observation)
- Students can evaluate trade-offs (Claude Code vs Gemini; isolation vs performance)
- Students can reason about system architecture (context management strategy)

### Bloom's Taxonomy Alignment

| Lesson | Bloom's Level | Activity |
|--------|--------------|----------|
| **L1** | Remember, Understand | Define context, token, degradation |
| **L2** | Understand, Analyze | Recognize symptoms, analyze patterns |
| **L3** | Analyze, Evaluate | Design loading strategy, evaluate trade-offs |
| **L4** | Analyze, Create | Design checkpoint structure, create from transcript |
| **L5** | Evaluate, Create | Score similarity, design multi-session workflow |
| **L6** | Create, Understand | Design memory files, create for project |
| **L7** | Analyze, Evaluate | Select tools, justify decisions |
| **L8** | Analyze, Evaluate | Diagnose scenarios, design recovery |
| **L9** | Create, Evaluate | Write specification, orchestrate techniques |

### DigComp (Digital Competence Framework) Alignment

| Competence | Lessons | Application |
|------------|---------|-------------|
| **Problem Solving** | 1, 8 | Diagnose context degradation; design recovery |
| **Information Management** | 3, 6 | Organize context (Foundation/Current/On-Demand); persist memory |
| **Communication & Collaboration** | 2, 3, 4, 5, 7 | Explain decisions; iterate with AI; coordinate multi-session |
| **Digital Content Creation** | 4, 6, 9 | Create checkpoints, memory files, specifications |
| **Safety & Security** | 8 | Protect sensitive context; prevent cross-contamination |

---

## IV. Cognitive Load Validation (Per-Lesson)

### Lesson 1: 8 concepts ✓
1. Context window definition
2. Token counting basics
3. Context window sizes
4. Utilization calculation
5. Session notes structure
6. Degradation signals
7. Research integration (Google PDF)
8. Observable behaviors

**Within B1 limit (7-10 concepts)**

### Lesson 2: 9 concepts ✓
1. Repetitive suggestions
2. Forgotten patterns
3. Performance degradation
4. Generic responses
5. Lost context
6. Compare-and-contrast pedagogy
7. Degradation checklist
8. Root cause analysis
9. Recovery decision

**Within B1 limit (7-10 concepts)**

### Lesson 3: 8 concepts ✓
1. Foundation context
2. Current context
3. On-Demand context
4. High-signal principle
5. Allocation percentages
6. Loading criteria
7. Trade-offs (preload vs On-Demand)
8. Iteration toward optimal

**Within B1 limit (7-10 concepts)**

### Lesson 4: 8 concepts ✓
1. Checkpoint definition
2. Extraction process
3. Consolidation process
4. Checkpoint structure
5. Restart trigger
6. Recovery procedure
7. Extraction + Consolidation pipeline
8. Trade-off analysis

**Within B1 limit (7-10 concepts)**

### Lesson 5: 9 concepts ✓
1. Task similarity definition
2. Similarity scoring (framework)
3. Isolation threshold
4. Cross-contamination risk
5. Multi-session workflow
6. Session independence
7. Context switching cost
8. Parallel coordination
9. Research integration (GitHub)

**Within B1 limit (7-10 concepts)**

### Lesson 6: 8 concepts ✓
1. CLAUDE.md (patterns)
2. architecture.md (structure)
3. decisions.md (decisions)
4. Persistence strategy
5. Update triggers
6. Memory retrieval
7. Memory generation pipeline
8. Reusability across projects

**Within B1 limit (7-10 concepts)**

### Lesson 7: 9 concepts ✓
1. Tool capabilities comparison
2. Codebase size assessment
3. Task complexity evaluation
4. Phase decomposition
5. Cost considerations
6. Switching strategy
7. Skill design (decision frameworks)
8. Extensibility
9. Research integration (Spec)

**Within B1 limit (7-10 concepts)**

### Lesson 8: 8 concepts ✓
1. Integrated diagnosis
2. Recovery procedures
3. Optimization under constraints
4. Production considerations
5. Fallback strategies
6. Multi-session recovery
7. Continuous improvement
8. Debugging workflow

**Within B1 limit (7-10 concepts)**

### Lesson 9: 8 concepts ✓
1. Specification structure
2. Context architecture
3. Session strategy
4. Memory file design
5. Tool selection strategy
6. Multi-session workflow
7. Risk mitigation
8. Validation criteria

**Within B1 limit (7-10 concepts)**

---

## V. Example Content Specification (What to Create)

### Markdown Examples (Lessons 1-8)

**Lesson 1**: Session note templates + token estimation worksheet
**Lesson 2**: Healthy vs degraded transcript pairs (side-by-side examples)
**Lesson 3**: Foundation/Current/On-Demand allocation worksheet
**Lesson 4**: Checkpoint extraction & compression examples
**Lesson 5**: Task similarity scoring spreadsheet
**Lesson 6**: CLAUDE.md, architecture.md, decisions.md templates
**Lesson 7**: Tool selection decision tree + scenario worksheet
**Lesson 8**: 4 degradation scenario walkthroughs

### Claude Code Prompts (Try With AI Sections)

**Lesson 1**: "Verify your token estimates"
**Lesson 2**: "Analyze your recent sessions for degradation"
**Lesson 3**: "Validate your loading strategy"
**Lesson 4**: "Create and test a checkpoint"
**Lesson 5**: "Evaluate your task dependencies"
**Lesson 6**: "Design your memory file system"
**Lesson 7**: "Evaluate your tool choices"
**Lesson 8**: "Simulate a degraded session and recover"
**Lesson 9**: "Validate your capstone specification"

### Plain Text Examples (Scenarios)

**Lesson 1**: 3 sample session notes (varying complexity)
**Lesson 2**: 5 conversation transcripts (3 healthy, 2 degraded)
**Lesson 3**: 2 project structures (small, large) with loading strategies
**Lesson 4**: 2 checkpoint examples (good, bad) with explanation
**Lesson 5**: 5 task pair examples with similarity scores
**Lesson 8**: 4 degradation case studies with diagnosis/recovery
**Lesson 9**: Capstone specification template (as shown in Part B)

---

## VI. Research Integration Mapping

### Source 1: Anthropic Article (Context Engineering for AI Agents)

**Concepts to Integrate**:
- Smallest Set of High-Signal Tokens (Lesson 3)
- Extraction (Lesson 4)
- Consolidation (Lesson 4)
- Memory Generation Pipeline (Lesson 6)

**Implementation**:
- Lesson 3: Explain Foundation as "highest-signal core patterns"
- Lesson 4: Use extraction/consolidation terms from research
- Lesson 6: Reference memory generation pipeline concept

**Validation**: Concepts are accurately represented; attributed to Anthropic

---

### Source 2: GitHub Spec (CoLearning Agentic AI)

**Concepts to Integrate**:
- Compare-and-Contrast Pedagogy (Lesson 2)
- Multi-Session Workflows (Lesson 5)
- Guardrails and Decision Criteria (Lesson 7)

**Implementation**:
- Lesson 2: Use healthy vs degraded side-by-side examples
- Lesson 5: Reference multi-session coordination examples
- Lesson 7: Use explicit decision rules (codebase size, reasoning depth)

**Validation**: Concepts are aligned with GitHub spec; attributed correctly

---

### Source 3: Google PDF (Context Engineering: Sessions & Memory, 72 pages)

**Concepts to Integrate**:
- Sessions Architecture (Lessons 1-2)
- Working Memory vs Chronological History (Lesson 1)
- Extraction + Consolidation (Lessons 4, 6)
- Production Considerations (Lesson 8)

**Implementation**:
- Lesson 1: Frame sessions as "working memory + history"
- Lesson 2: Explain context degradation as "working memory saturation"
- Lesson 8: Include production considerations in debugging scenarios

**Validation**: Concepts are accurately explained; attributed to Google

---

## VII. Constitutional Compliance Verification

### Principle 1: Specification Primacy ✓
- Lesson 9 is specification-only (no implementation code)
- Layer 1 establishes specs through session notes
- Layer 3 creates specifications for skills
- Layer 4 writes complete system specification

### Principle 2: Progressive Complexity ✓
- All lessons maintain 7-10 concepts (B1 tier)
- Progression from observation (L1) to specification (L4)
- Concepts build on prior lessons (accumulation)
- Cognitive load managed through careful scope

### Principle 3: Factual Accuracy ✓
- Context window sizes verified (Claude Sonnet, Gemini)
- Research integrated from authoritative sources
- Concepts accurately attributed
- No hallucinations or unverified claims

### Principle 4: Coherent Pedagogical Structure ✓
- Lessons follow Foundation → Application → Integration → Validation → Mastery arc
- Layer progression: Manual (L1-2) → Intelligence (L3) → Spec-Driven (L4)
- Nine lessons justified by concept density
- Structure serves learning, not arbitrary template

### Principle 5: Intelligence Accumulation ✓
- Lessons reference and build on prior lessons
- Lesson 4 extends Lesson 3 (progressive loading)
- Lesson 6 consolidates Lessons 1-5 (memory files)
- Lesson 8 integrates Lessons 1-7 (debugging practice)
- Lesson 9 orchestrates all (capstone)

### Principle 6: Anti-Convergence ✓
- Chapter 10 (conversational) vs Chapter 11 (systems thinking) modality difference
- Teaching varies: walkthroughs, checklists, scenarios, specifications, frameworks
- Examples are production-relevant (memory files, context management), not toy apps
- Methodologies: hands-on discovery, decision frameworks, specification writing

### Principle 7: Minimal Sufficient Content ✓
- Every lesson maps to learning objective
- No filler sections (common mistakes, FAQs)
- No "What's Next" or "Key Takeaways" (violates minimal content)
- Lessons end with "Try With AI" only
- Content length justified by concept density

---

## VIII. Quality Assurance Checklist

### Pre-Implementation Validation
- [ ] Spec.md comprehensive (this document confirms)
- [ ] Chapter prerequisites verified (Chapter 10 completion required)
- [ ] All 9 lessons planned with concept mapping
- [ ] Cognitive load validated per lesson
- [ ] Three Roles demonstrated in Lessons 3-5
- [ ] Layer progression confirmed (L1-2-3-4)
- [ ] Constitutional compliance verified
- [ ] Research integration mapped

### During Implementation
- [ ] Zero code in Lessons 1-8 (markdown/prompts/text only)
- [ ] Lesson 9 spec is specification-only (zero code)
- [ ] All examples use markdown/prompts students know
- [ ] Three Roles visible in L2-3-5 lessons
- [ ] Try With AI sections follow protocol (no meta-commentary)
- [ ] All claims verified with sources
- [ ] Cognitive load checked (count concepts per lesson)

### Post-Implementation Validation
- [ ] Technical review by validation-auditor
- [ ] Factual verification by factual-verifier
- [ ] Content evaluation by content-evaluation-framework
- [ ] All acceptance tests pass
- [ ] Constitutional compliance confirmed

---

## IX. Implementation Timeline & Phase Breakdown

### Phase 0: Planning (CURRENT)
- Duration: 1 session
- Output: This plan.md (approved specification)
- Validation: Plan-architect review

### Phase 1: Tasks Breakdown
- Duration: 1 session
- Output: tasks.md (implementation checklist)
- Validation: Chapter-planner review

### Phase 2: Lesson Implementation (Sessions 1-2)
- Session 1: Lessons 1-2 (Layer 1 foundation)
- Session 2: Lessons 3-5 (Layer 2 collaboration)
- Output: 5 lesson files + README
- Validation: Content-implementer review

### Phase 3: Intelligence Design + Capstone (Session 3)
- Lessons 6-7 (Layer 3 skills)
- Lesson 8 (Layer 2 validation)
- Lesson 9 (Layer 4 capstone)
- Output: 4 lesson files + 2 skills
- Validation: Content-implementer review

### Phase 4: Technical Review (Session 4)
- Validation-auditor technical review
- Factual-verifier accuracy checks
- Content-evaluation-framework scoring
- Issues tracking and fixes
- Output: Validation report + fixes applied

### Phase 5: Publication
- Duration: 1 session
- Output: Chapter 11 ready for production
- Commit to main branch

---

## X. Success Metrics (Chapter Complete)

### Quality Metrics
- ✓ 9 lessons with layer progression (L1-2-3-4)
- ✓ Zero programming code in Lessons 1-8
- ✓ All examples use markdown/prompts/text only
- ✓ 7-10 concepts per lesson (B1 cognitive load)
- ✓ Three Roles demonstrated in Lessons 3-5
- ✓ Constitutional compliance verified
- ✓ Research integrated from all 3 sources

### Pedagogical Metrics
- ✓ Students manually observe context (L1)
- ✓ Students apply AI collaboration (L2)
- ✓ Students create reusable skills (L3)
- ✓ Students write specifications (L4)
- ✓ Lesson progression builds understanding
- ✓ Accumulated intelligence demonstrated in capstone

### Technical Metrics
- ✓ Zero untested claims
- ✓ All context window sizes verified
- ✓ All concepts properly attributed
- ✓ No hallucinations detected
- ✓ Specification quality framework complete

---

## Appendix: File Structure

```
chapter-11-context-engineering/
├── README.md (overview)
├── 01-context-windows-token-counting.md
├── 02-degradation-symptoms-manual-tracking.md
├── 03-progressive-loading-strategy.md
├── 04-context-compression-session-restart.md
├── 05-context-isolation-parallel-tasks.md
├── 06-memory-files-persistent-intelligence.md
├── 07-tool-selection-framework.md
├── 08-hands-on-debugging-optimization.md
├── 09-capstone-spec-driven-orchestration.md
├── skills/
│   ├── memory-file-architecture.md (Lesson 6 skill)
│   └── tool-selection-framework.md (Lesson 7 skill)
└── examples/
    ├── session-notes/ (Lesson 1 templates)
    ├── transcripts/ (Lesson 2 examples)
    ├── loading-strategies/ (Lesson 3 worksheets)
    ├── checkpoints/ (Lesson 4 examples)
    ├── task-scoring/ (Lesson 5 worksheets)
    ├── memory-files/ (Lesson 6 templates)
    ├── tool-selection/ (Lesson 7 decision trees)
    ├── debugging-scenarios/ (Lesson 8 cases)
    └── capstone-template/ (Lesson 9 specification)
```

---

**Plan Status**: Ready for Implementation
**Plan Version**: 1.0.0
**Created**: 2025-11-18
**Next Step**: Generate tasks.md with implementation checklist


# Specification: Chapter 11 Context Engineering Redesign (Part 3 Compliant)

**Feature ID**: 002-011-chapter-11-redesign-fix
**Status**: Draft
**Created**: 2025-11-18
**Version**: 1.0.0

---

## 1. Intent

**CRITICAL CONSTRAINT**: This chapter is in **Part 3: Markdown, Prompt & Context Engineering**. Students have completed Chapters 1-10 and possess ONLY:
- Markdown syntax knowledge (headings, lists, code blocks, links, images)
- Basic prompt engineering skills (Claude Code prompts, clarity, specificity)
- Understanding of AI roles (Teacher, Student, Co-Worker from Chapter 10)

**Students DO NOT have**:
- Python programming knowledge (that's Part 4)
- Any coding/scripting experience
- Database, API, or software architecture knowledge

**What We're Fixing**:

The current Chapter 11 implementation violates Part 3 prerequisites by using:
- Complex Python/FastAPI code examples
- SQLAlchemy database code
- OAuth2 authentication patterns
- Advanced programming concepts

This makes the chapter **completely inaccessible** to the target audience.

**Correct Approach**:

Redesign Chapter 11 to teach context engineering using **ONLY**:
- Markdown-based examples (session notes, project documentation)
- Text-based exercises (writing prompts, analyzing degradation symptoms)
- Claude Code prompts (the only tool students know)
- Plain English explanations (no code required)

**Learning Objective** (Preserved from Original):

By completing this chapter, students will:
- **Manually observe context degradation** — Recognize when AI forgets patterns, repeats suggestions, or drops performance
- **Apply progressive loading strategies** — Use Foundation → Current → On-Demand pattern in prompts
- **Execute compression and isolation** — Create checkpoint summaries, restart sessions, use separate sessions for unrelated tasks
- **Design memory file architecture** — Create CLAUDE.md, architecture.md, decisions.md files for persistent intelligence
- **Select appropriate tools** — Choose Claude Code vs Gemini CLI based on context needs
- **Write context-aware specifications** — Orchestrate all patterns in capstone spec

**Developer Benefit** (Why This Matters):

Students learn to **engineer AI context windows like professionals** using ONLY the tools they already know (Markdown + prompts). This makes them immediately productive without waiting to learn programming.

---

## 2. Success Criteria (Measurable, Falsifiable)

**SC-001**: Zero code examples in Lessons 1-8
- **Measure**: grep for code blocks containing Python, JavaScript, SQL, or programming syntax
- **Pass**: Only markdown examples, prompt templates, or plain text scenarios
- **Fail**: Any code requiring programming knowledge

**SC-002**: All examples use Markdown + Claude Code prompts only
- **Measure**: Review all exercises and scenarios
- **Pass**: Students can complete ALL exercises using only Markdown files and Claude Code
- **Fail**: Any exercise requiring programming, CLI tools (beyond Claude Code), or external services

**SC-003**: Cognitive load within B1 limits (7-10 concepts per lesson)
- **Measure**: Count distinct concepts per lesson
- **Pass**: 7-10 concepts per lesson, cumulative load managed
- **Fail**: >10 concepts or cognitive overload detected

**SC-004**: Hands-on discovery pedagogy in Lessons 1-2 (Layer 1)
- **Measure**: Lessons 1-2 include manual exercises WITHOUT AI assistance
- **Pass**: Students practice recognizing degradation symptoms manually first
- **Fail**: AI used before manual foundation established

**SC-005**: Three Roles framework demonstrated in Lessons 3-5 (Layer 2)
- **Measure**: Each lesson shows AI as Teacher, Student, and Co-Worker
- **Pass**: Complete convergence loop (2+ iterations toward better solution)
- **Fail**: AI presented as passive tool

**SC-006**: Capstone (Lesson 9) is specification-only
- **Measure**: Lesson 9 requires writing complete spec, zero implementation code
- **Pass**: Students write 3-5 page spec with Intent, Success Criteria, Architecture, Algorithms
- **Fail**: Lesson includes implementation code or pseudo-code

**SC-007**: External research accurately integrated
- **Measure**: Compare against Anthropic article, GitHub spec, Google PDF
- **Pass**: Key concepts (extraction, consolidation, memory generation) present and correctly explained
- **Fail**: Hallucinated concepts or misrepresented research

---

## 3. Problem Statement

### Current State (Broken)

**Lesson 1: Context Windows and Token Counting** currently contains:

```python
from fastapi import FastAPI
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from routes import auth, users

app = FastAPI(title="Auth System", version="1.0.0")
DATABASE_URL = "postgresql://localhost/auth_db"
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(bind=engine)
```

**Problems**:
1. Students don't know Python (Part 4 prerequisite)
2. Students don't know FastAPI, SQLAlchemy, PostgreSQL
3. Students don't know OAuth2 or authentication patterns
4. Example violates Layer 1 principle (manual foundation before AI)
5. Cognitive load exceeds B1 tier limits

**Root Cause**: Layer 4 thinking (spec-driven orchestration with code) applied to Layer 1 content (manual foundation).

### Desired State (Fixed)

**Lesson 1: Context Windows and Token Counting** should contain:

**Example 1: Session Note in Markdown**
```markdown
# Development Session - 2025-11-18

## Task: Fix User Authentication Bug

### Context Loaded:
- CLAUDE.md (project patterns)
- auth-routes.md (authentication architecture)
- bug-report-issue-127.md (specific issue)

### Progress:
- Identified root cause (token expiration handling)
- Proposed fix (extend session timeout)
- Claude Code suggested alternative (refresh token pattern)

### Token Estimation:
- Context loaded: ~15K tokens
- Conversation so far: ~8K tokens
- Total utilization: 23K / 200K = 11.5%
```

**Teaching Point**: "Notice how the session note tracks what context is loaded. This helps you recognize when you're approaching context limits."

**No programming required** — students practice writing session notes in Markdown.

---

## 4. Functional Requirements

### FR-001: Lesson Structure (9 Lessons, Layer Progression)

**Lesson 1: Context Windows and Token Counting** (Layer 1: Manual Foundation)
- **NO AI assistance** in exercises
- Students manually write session notes in Markdown
- Students manually estimate token usage using rules of thumb
- Students manually recognize when context is growing
- **Example**: Write 3 session notes, estimate tokens for each (simple counting: ~1 token per word)

**Lesson 2: Degradation Symptoms and Manual Tracking** (Layer 1: Manual Foundation)
- **NO AI assistance** in exercises
- Students manually identify degradation symptoms by reading conversation transcripts
- Students manually create degradation checklists
- **Example**: Given 5 conversation transcripts, mark which show degradation and why

**Lesson 3: Progressive Loading Strategy** (Layer 2: AI Collaboration, Three Roles)
- **AI as Teacher**: Suggests Foundation → Current → On-Demand pattern
- **AI as Student**: Learns project constraints (which files are foundation)
- **AI as Co-Worker**: Iterates toward optimal loading strategy (2+ rounds)
- **Example**: Collaborative prompt session designing progressive loading for documentation project

**Lesson 4: Context Compression and Session Restart** (Layer 2: AI Collaboration, Three Roles)
- **AI as Teacher**: Suggests checkpoint creation
- **AI as Student**: Learns what to preserve in checkpoint
- **AI as Co-Worker**: Iterates toward concise summary (2+ rounds)
- **Example**: Create checkpoint from 80% full session, restart with checkpoint

**Lesson 5: Context Isolation and Parallel Tasks** (Layer 2: AI Collaboration, Three Roles)
- **AI as Teacher**: Suggests when to isolate tasks
- **AI as Student**: Learns task similarity scoring
- **AI as Co-Worker**: Iterates toward isolation decision (2+ rounds)
- **Example**: Evaluate 3 tasks, decide which need separate sessions

**Lesson 6: Memory Files and Persistent Intelligence** (Layer 3: Intelligence Design)
- Create reusable **skill**: `memory-file-architecture`
- Skill uses Persona + Questions + Principles pattern
- **Example**: Design CLAUDE.md, architecture.md, decisions.md for project

**Lesson 7: Tool Selection Framework** (Layer 3: Intelligence Design)
- Create reusable **skill**: `tool-selection-framework`
- Decision logic: Claude Code (deep reasoning) vs Gemini CLI (broad exploration)
- **Example**: Evaluate 5 scenarios, select appropriate tool

**Lesson 8: Hands-On Debugging and Optimization** (Layer 2: Validation)
- Practice diagnosing 4 context degradation scenarios
- Apply accumulated techniques from Lessons 1-7
- **Example**: Fix broken session (high utilization, forgotten patterns, repetitive output)

**Lesson 9: Capstone — Spec-Driven Orchestration** (Layer 4: Spec-Driven Integration)
- Write **complete specification** (NO implementation code)
- Orchestrate ALL techniques from Lessons 1-8
- **Example**: 3-5 page spec for context-aware development tool

### FR-002: Example Content (Markdown + Prompts Only)

**All examples must use**:
- Markdown files (session notes, project documentation, memory files)
- Claude Code prompts (the only tool students know)
- Plain text scenarios (conversation transcripts, degradation symptoms)
- Text-based decision frameworks (checklists, scoring criteria)

**Forbidden in Lessons 1-8**:
- Python, JavaScript, TypeScript, or any programming code
- SQL queries or database schemas
- API definitions (REST, GraphQL)
- Docker, deployment, infrastructure
- Pseudo-code that looks like real code

**Allowed in Lesson 9 (Capstone Spec)**:
- Reference to components in plain English (e.g., "Context Monitor component")
- Algorithm descriptions in structured English (e.g., "IF utilization > 80% AND duration > 60min THEN create checkpoint")
- **NO runnable code** (this is specification-only)

### FR-003: Research Integration from External Sources

**Source 1: Anthropic Article** (Context Engineering for AI Agents)

**Key Concepts to Integrate**:
1. **Smallest Set of High-Signal Tokens** → Teach in Lesson 3 (Progressive Loading)
   - Foundation loading: Core patterns only
   - Current loading: Task-relevant context only
   - On-Demand: Fetch as needed

2. **Extraction and Consolidation** → Teach in Lesson 4 (Compression)
   - Extraction: Identify key decisions from session
   - Consolidation: Summarize into checkpoint (<600 tokens)

3. **Memory Generation Pipeline** → Teach in Lesson 6 (Memory Files)
   - Extract architectural decisions
   - Consolidate into persistent memory
   - Retrieve on session start

**Source 2: GitHub Spec** (CoLearning Agentic AI Specs)

**Key Concepts to Integrate**:
1. **Compare-and-Contrast Pedagogy** → Use in Lesson 2 (Degradation Symptoms)
   - Show healthy session vs degraded session side-by-side
   - Students identify differences manually

2. **Multi-Session Workflows** → Use in Lesson 5 (Isolation)
   - Demonstrate when to start new session vs continue current

3. **Guardrails and Decision Criteria** → Use in Lesson 7 (Tool Selection)
   - Explicit decision rules (codebase size, reasoning depth, complexity)

**Source 3: Google PDF** (Context Engineering: Sessions & Memory, 72 pages)

**Key Concepts to Integrate**:
1. **Sessions Architecture** → Teach in Lessons 1-2
   - Chronological history (events): What happened in order
   - Working memory (state): Current understanding

2. **Memory Generation (Extraction + Consolidation)** → Teach in Lessons 4, 6
   - Extraction: Pull key facts from session
   - Consolidation: Compress into reusable knowledge

3. **Production Considerations** → Teach in Lesson 8 (Debugging)
   - Testing context strategies (before production)
   - Security (don't leak sensitive context)

### FR-004: Pedagogical Constraints (Constitutional Compliance)

**Principle 1: Specification Primacy** (Intent before Implementation)
- Lesson 9 capstone is specification-only
- NO implementation code until specification complete

**Principle 2: Progressive Complexity** (B1 Tier, 7-10 Concepts per Lesson)
- Each lesson: 7-10 distinct concepts
- Cumulative load managed through memory files (persistent context)

**Principle 3: Factual Accuracy** (All Claims Verifiable)
- Context window sizes: Claude Sonnet 4.5 (200K standard, 1M extended), Gemini 1.5 Pro (2M)
- Research citations: Anthropic (2024), Karpathy principle (load minimally), Google Cloud (2024)

**Principle 4: Coherent Structure** (Lessons Build Progressively)
- L1-2: Manual foundation (no AI)
- L3-5: AI collaboration (Three Roles)
- L6-7: Intelligence design (reusable skills)
- L8: Validation (integrated practice)
- L9: Spec-driven capstone

**Principle 5: Intelligence Accumulation** (Lessons Reference Prior Knowledge)
- Lesson 4 references Lesson 3 (progressive loading)
- Lesson 6 references Lessons 3-5 (what to persist in memory)
- Lesson 8 references all Lessons 1-7 (integrated debugging)
- Lesson 9 references all Lessons 1-8 (capstone orchestration)

**Principle 6: Anti-Convergence** (Vary Teaching Modality from Chapter 10)
- Chapter 10: Conversational scenarios (role-playing Teacher/Student/Co-Worker)
- Chapter 11: Systems thinking (architecture, decision frameworks, optimization)
- **Differentiation**: Chapter 10 teaches "what to SAY", Chapter 11 teaches "what AI KNOWS"

**Principle 7: Minimal Content** (Every Section Maps to Learning Objective)
- No filler sections (common mistakes, FAQs, troubleshooting)
- No "What's Next" sections (violates lesson ending protocol)
- Lessons end with "Try With AI" only

---

## 5. System Architecture (Conceptual, NO Implementation)

### Component 1: Session Note Format (Lesson 1)

**Responsibility**: Provide structure for tracking context usage manually

**Inputs**: Student's Markdown editor

**Outputs**: Session note containing:
- Task description
- Context loaded (which files/notes)
- Progress made
- Token estimation (rough count)

**Algorithm** (Plain English):
```
Session Note Structure:
1. Header: Date + Task
2. Context Loaded: List of files/notes loaded
3. Progress: Chronological list of what happened
4. Token Estimation: Count words, multiply by ~1.2 (rough token count)
```

### Component 2: Degradation Symptom Checklist (Lesson 2)

**Responsibility**: Help students recognize context degradation manually

**Inputs**: Conversation transcript (plain text)

**Outputs**: Degradation checklist (symptoms present/absent)

**Algorithm** (Plain English):
```
Degradation Symptoms (Mark Present or Absent):
- [ ] Repetitive suggestions (AI suggests same thing 2+ times)
- [ ] Forgotten patterns (AI forgets rule stated earlier)
- [ ] Performance drop (AI takes longer to respond)
- [ ] Generic responses (AI gives vague answer instead of specific)
- [ ] Lost context (AI asks for information already provided)
```

### Component 3: Progressive Loading Decision Framework (Lesson 3)

**Responsibility**: Guide AI collaboration on what to load when

**Inputs**: Project structure (list of files), current task

**Outputs**: Loading strategy (Foundation, Current, On-Demand tiers)

**Algorithm** (Plain English):
```
Phase 1 (Foundation): Load always
- Project patterns (CLAUDE.md)
- Core architecture (architecture.md)
- Key decisions (decisions.md)
- Target: 10-15% of context window

Phase 2 (Current): Load for this task
- Task-relevant files
- Related documentation
- Target: Additional 20-30% of context window

Phase 3 (On-Demand): Load as needed
- Fetch files when AI requests them
- Reserve: 30% of context window for on-demand fetching
```

### Component 4: Checkpoint Creation Process (Lesson 4)

**Responsibility**: Compress session into reusable summary

**Inputs**: Current session (conversation history), utilization (% of context used)

**Outputs**: CHECKPOINT.md file (<600 tokens)

**Algorithm** (Plain English):
```
Checkpoint Structure:
1. Architectural Decisions Made: [List 3-5 key decisions]
2. Progress Summary: [What was accomplished]
3. Next Steps: [What to do in next session]
4. Context to Preserve: [Patterns discovered]

Creation Trigger:
IF utilization > 80% AND session_duration > 60 minutes
THEN create checkpoint and restart session
```

### Component 5: Task Isolation Decision Framework (Lesson 5)

**Responsibility**: Determine if new task needs separate session

**Inputs**: Current session context, new task description

**Outputs**: Isolation recommendation (Yes/No), similarity score (0-100%)

**Algorithm** (Plain English):
```
Similarity Scoring (Add points if true):
- Same business domain? +30 points
- Same database models? +20 points
- Same external service? +20 points
- Same API routes? +15 points
- Shared test suite? +15 points

Total Score: Sum of matched criteria

Decision:
IF score < 50% THEN recommend separate session
ELSE continue in current session
```

### Component 6: Memory File Architecture (Lesson 6)

**Responsibility**: Persist intelligence across sessions

**Inputs**: Project root directory

**Outputs**: Three memory files (CLAUDE.md, architecture.md, decisions.md)

**Structure**:

**CLAUDE.md** (Project Patterns):
- Coding conventions (if project has code later)
- Common patterns discovered
- AI collaboration preferences

**architecture.md** (System Design):
- Component structure
- Key dependencies
- Design constraints

**decisions.md** (Architectural Decision Records):
- Chronological list of decisions
- Each decision: Date, Decision, Reasoning, Alternatives Considered

**Update Strategy**:
```
Session Start:
- Read all 3 memory files
- Inject into session prompt

Session End:
- Append new decisions to decisions.md
- Update CLAUDE.md if new pattern discovered
- Update architecture.md if structure changed
```

### Component 7: Tool Selection Decision Framework (Lesson 7)

**Responsibility**: Recommend Claude Code vs Gemini CLI

**Inputs**: Task description, codebase size, complexity assessment

**Outputs**: Tool recommendation + reasoning

**Algorithm** (Plain English):
```
Decision Logic:
IF codebase < 50K lines:
  Recommend Claude Code (200K context, deep reasoning)

ELSE IF codebase 50K-500K lines:
  IF task needs deep architectural reasoning:
    Recommend Claude Code
  ELSE IF task needs broad pattern analysis:
    Recommend Gemini CLI (2M context)

ELSE IF codebase > 500K lines:
  Recommend Gemini CLI

IF task is multi-phase (explore + implement):
  Recommend Gemini CLI first (exploration)
  Then Claude Code (focused implementation)
```

---

## 6. Non-Goals (What We Are NOT Doing)

**NG-001**: Teaching programming
- Students learn programming in Part 4 (Python chapters)
- This chapter teaches context engineering using tools students already know

**NG-002**: Implementing the context-aware system
- Lesson 9 capstone is specification-only
- Implementation would require programming (Part 4 skill)

**NG-003**: Teaching advanced AI concepts
- No embeddings, vector databases, or RAG
- Focus on practical context management with Claude Code

**NG-004**: Tool-specific features beyond Claude Code
- No MCP servers, custom tools, or extensions
- Students only know Claude Code from Chapters 1-10

**NG-005**: Code optimization or performance tuning
- Focus on observable behaviors (degradation symptoms)
- Not measuring actual token counts programmatically

---

## 7. Open Questions

**Q1**: Should Lesson 3 introduce Gemini CLI for comparison, or wait until Lesson 7?
- **Option A**: Introduce in Lesson 3 (early awareness of 2M context)
- **Option B**: Wait until Lesson 7 (focused tool selection lesson)
- **Recommendation**: Option B (avoid cognitive overload in L3)

**Q2**: Should Lesson 8 include real conversation transcripts from previous chapters?
- **Option A**: Use synthetic scenarios (designed for teaching)
- **Option B**: Use real transcripts from Chapters 1-10 (authentic examples)
- **Recommendation**: Option B (students recognize familiar patterns)

**Q3**: Should capstone (Lesson 9) spec be prescriptive or open-ended?
- **Option A**: Provide detailed template (scaffolded approach)
- **Option B**: Minimal guidance (test mastery)
- **Recommendation**: Option A (B1 tier needs scaffolding)

---

## 8. Acceptance Tests

### Test-001: No Programming Code in Lessons 1-8

**Given**: All 9 lesson files
**When**: Search for code blocks with programming syntax
**Then**: Zero matches in Lessons 1-8
**Evidence**: `grep -r "^from \|^import \|^def \|^class \|^function \|^const \|^let \|^var " lessons/01-08/*.md` returns empty

### Test-002: All Examples Use Markdown + Prompts

**Given**: All example sections in Lessons 1-8
**When**: Review each example
**Then**: Every example uses ONLY:
- Markdown files (session notes, memory files, project docs)
- Claude Code prompts (conversation starters)
- Plain text (transcripts, checklists, decision frameworks)

### Test-003: Three Roles Framework in Lessons 3-5

**Given**: Lessons 3, 4, 5
**When**: Review AI collaboration sections
**Then**: Each lesson demonstrates:
- AI as Teacher (suggests pattern student didn't know)
- AI as Student (learns project constraint from student)
- AI as Co-Worker (iterates toward solution, 2+ rounds)

### Test-004: Capstone is Specification-Only

**Given**: Lesson 9 (Capstone)
**When**: Review capstone exercise
**Then**:
- Exercise asks for 3-5 page specification
- Zero implementation code required
- Specification includes: Intent, Success Criteria, Functional Requirements, Architecture, Algorithms, Non-Goals

### Test-005: External Research Integrated

**Given**: Lessons 3, 4, 6
**When**: Review content against external sources
**Then**:
- Anthropic concepts (extraction, consolidation) present in Lessons 4, 6
- GitHub spec patterns (compare-and-contrast, multi-session) present in Lessons 2, 5
- Google PDF concepts (sessions architecture, memory generation) present in Lessons 1, 2, 6

### Test-006: B1 Cognitive Load Limits

**Given**: All 9 lessons
**When**: Count distinct concepts per lesson
**Then**: Each lesson has 7-10 concepts maximum

### Test-007: Layer Progression Validated

**Given**: All 9 lessons
**When**: Review pedagogical approach
**Then**:
- Lessons 1-2: Manual foundation (NO AI assistance in exercises)
- Lessons 3-5: AI collaboration (Three Roles demonstrated)
- Lessons 6-7: Intelligence design (reusable skills created)
- Lesson 8: Validation (integrated practice)
- Lesson 9: Spec-driven integration (capstone spec)

---

## 9. Dependencies

**Content Dependencies**:
- Chapter 10 completion (Three Roles framework prerequisite)
- Chapters 1-10 markdown and prompt knowledge (Part 3 prerequisites)

**Resource Dependencies**:
- Anthropic article (context engineering research)
- GitHub spec (pedagogical patterns)
- Google PDF (sessions and memory architecture)

**Technical Dependencies**:
- None (students use only Markdown files + Claude Code)

**Validation Dependencies**:
- validation-auditor agent (technical review)
- factual-verifier agent (research accuracy)
- content-evaluation-framework skill (6-category quality assessment)

---

## 10. Timeline and Milestones

**Phase 0: Specification** (This Document)
- Duration: 1 session
- Output: Complete specification (this file)
- Validation: Spec-architect review

**Phase 1: Planning**
- Duration: 1 session
- Output: plan.md (lesson-by-lesson breakdown)
- Validation: Pedagogical-designer review

**Phase 2: Task Breakdown**
- Duration: 1 session
- Output: tasks.md (implementation checklist)
- Validation: Chapter-planner review

**Phase 3: Implementation**
- Duration: 3-4 sessions
  - Session 1: Lessons 1-2 (Manual Foundation)
  - Session 2: Lessons 3-5 (AI Collaboration)
  - Session 3: Lessons 6-7 (Intelligence Design)
  - Session 4: Lessons 8-9 (Validation + Capstone)
- Output: 9 lesson files + 2 skills + README
- Validation: Content-implementer agents

**Phase 4: Validation and Fixes**
- Duration: 1 session
- Output: Validation reports + fixes applied
- Validation: Validation-auditor + factual-verifier

**Phase 5: Meta-Learning Capture**
- Duration: 0.5 session
- Output: PHR (Prompt History Record)
- Validation: Documentation complete

---

## 11. Risk Assessment

**Risk-001: Content Too Abstract** (HIGH)
- **Description**: Without code examples, students struggle to see concrete applications
- **Mitigation**: Use rich markdown examples (session notes, memory files, conversation transcripts)
- **Trigger**: Student feedback indicates confusion

**Risk-002: Three Roles Demonstration Insufficient** (MEDIUM)
- **Description**: Lessons 3-5 fail to show complete convergence loops
- **Mitigation**: Mandate 2+ iteration rounds in each Three Roles section
- **Trigger**: Test-003 fails during validation

**Risk-003: External Research Misrepresented** (MEDIUM)
- **Description**: Anthropic/Google PDF concepts incorrectly explained
- **Mitigation**: Factual-verifier agent reviews all research citations
- **Trigger**: Verifier flags inaccuracies

**Risk-004: Cognitive Load Exceeds B1 Limits** (LOW)
- **Description**: Lessons introduce >10 concepts
- **Mitigation**: Strict concept counting during implementation
- **Trigger**: Test-006 fails during validation

---

## 12. Revision History

**v1.0.0** (2025-11-18): Initial specification
- Identified Part 3 constraint violation (no programming allowed)
- Redesigned all examples to use Markdown + prompts only
- Integrated Anthropic article, GitHub spec, Google PDF research
- Defined 9-lesson structure with layer progression
- Created acceptance tests for compliance validation

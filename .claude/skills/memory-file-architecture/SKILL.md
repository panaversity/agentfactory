---
name: memory-file-architecture
category: "intelligence-design"
applies_to: ["all-projects"]
required_for: ["content-implementer", "project-memory"]
description: |
  Design and manage persistent memory file architecture (CLAUDE.md, architecture.md, decisions.md)
  that enables intelligence accumulation across AI development sessions. Use this skill when starting
  new projects, migrating existing projects to memory-based workflows, or optimizing context management
  for long-term development. This skill helps extract session knowledge, consolidate into persistent
  memory, and retrieve efficiently at session start, ensuring project patterns and decisions survive
  across session boundaries.
version: "1.0.0"
dependencies: ["constitution:v6.0.1"]
---

# Memory File Architecture

## Purpose

Enable developers to persist project intelligence across AI development sessions through structured memory files. This skill helps:
- Extract valuable knowledge from development sessions (patterns, decisions, constraints)
- Consolidate session knowledge into persistent, reusable memory
- Design memory file architecture that balances completeness with retrieval efficiency
- Update memory files systematically at session boundaries
- Retrieve project context rapidly at session start
- Resolve conflicts when memory files diverge or contradict

## When to Activate

Use this skill when:
- Starting a new multi-session project (establish memory architecture early)
- Migrating existing project to memory-based workflow
- AI sessions repeatedly lose project context (patterns forgotten, decisions re-explained)
- Onboarding new team members or AI sessions to established project
- Project complexity requires systematic knowledge management
- Context window limitations force frequent session restarts
- Students ask about "persistent memory", "project patterns", "cross-session intelligence"

## Persona

"Think like a knowledge management architect optimizing for persistence, retrieval, and accumulation across multiple AI development sessions. Your goal is to ensure that valuable project intelligence—patterns discovered, decisions made, constraints identified—survives session boundaries and remains accessible throughout the project lifecycle."

## Questions (Analysis Framework)

Before designing or updating memory files, analyze through these questions:

### Question 1: Persistence Value
**"Will this information be useful in future sessions?"**
- **If YES**: Store in appropriate memory file (CLAUDE.md, architecture.md, or decisions.md)
- **If NO**: Document in session notes only (ephemeral context, not project-persistent)
- **Examples**:
  - Pattern discovered: "Always validate before processing" → YES (persist in CLAUDE.md)
  - Today's debugging steps: "Tried X, Y, Z" → NO (session-specific, not reusable)
  - Architectural decision: "Using REST over GraphQL" → YES (persist in decisions.md)
  - Temporary workaround: "Skip tests while fixing CI" → NO (temporary state)

### Question 2: Retrieval Frequency
**"How often will this be accessed across sessions?"**
- **Frequent access** → CLAUDE.md (patterns, conventions, AI preferences)
  - Loaded at every session start
  - Compact, actionable content
  - Example: Naming conventions, error handling patterns
- **Occasional access** → architecture.md (system structure, dependencies)
  - Loaded when working on architecture-related tasks
  - Structural understanding, component relationships
  - Example: Component diagram, key dependencies
- **Reference access** → decisions.md (historical decisions, ADRs)
  - Loaded when context about "why" is needed
  - Chronological audit trail
  - Example: "Why did we choose PostgreSQL over MongoDB?"

### Question 3: Mutation Rate
**"How often does this information change?"**
- **Changes frequently** → CLAUDE.md (append new patterns as discovered)
  - Pattern library grows organically
  - New patterns added, rarely removed
  - Example: New validation pattern, new error handling approach
- **Changes occasionally** → architecture.md (update when structure shifts)
  - Updated when components added/removed/restructured
  - Reflects current system state
  - Example: Adding new microservice, splitting monolith
- **Never changes** → decisions.md (append-only, never rewrite)
  - Historical record, immutable
  - New decisions appended chronologically
  - Example: Decision made 6 months ago stays in history

### Question 4: Discoverability
**"Can I find what I need quickly?"**
- **CLAUDE.md**: Patterns grouped by category
  - Categories: Validation, Error Handling, Naming, Testing, AI Collaboration
  - Within category: Sort by importance or frequency
  - Example: "Error Handling > Network Errors > Retry Pattern"
- **architecture.md**: Components organized by layer
  - Layers: Presentation, Business Logic, Data, External
  - Within layer: List components with brief description
  - Example: "Data Layer > PostgreSQL (primary store) > Redis (cache)"
- **decisions.md**: Chronological with clear date headers
  - Format: YYYY-MM-DD: Decision Title
  - Each decision: Date, What, Why, Alternatives, Impact
  - Example: "2025-11-18: Adopt TypeScript for Type Safety"

## Principles

### Principle 1: Persistence First
**"If it's valuable once, it's valuable forever."**

Memory files capture knowledge that transcends individual sessions. Apply this principle:
- **When pattern discovered**: Add to CLAUDE.md immediately (don't wait for "perfect" wording)
- **When decision made**: Record in decisions.md with reasoning (prevent re-debating later)
- **When architecture changes**: Update architecture.md (keep current state accurate)
- **Default to persist**: If unsure whether to store, store it (removal easier than reconstruction)

### Principle 2: Append-Only for Decisions
**"Never delete decisions; add new context if circumstances change."**

decisions.md is an immutable audit trail. Apply this principle:
- **Never rewrite history**: Old decisions stay in file (even if reversed)
- **If decision changes**: Add NEW decision explaining reversal
- **Example**:
  ```markdown
  ## 2025-10-15: Use MongoDB for User Data
  - **Why**: Flexible schema for evolving user model
  - **Impact**: NoSQL expertise required

  ## 2025-11-18: Migrate to PostgreSQL for User Data
  - **Why**: Schema stability achieved; ACID transactions now critical
  - **Reverses**: 2025-10-15 decision
  - **Impact**: Migration effort, but better data integrity
  ```

### Principle 3: Injection Strategy at Session Start
**"All three files loaded at every session start."**

Memory files provide foundation context. Apply this principle:
- **Session start ritual**:
  1. Load CLAUDE.md (project patterns)
  2. Load architecture.md (system structure)
  3. Skim decisions.md (understand journey)
  4. Inject into session prompt
- **Total foundation context**: 10-15% of context window (20K-30K tokens target)
- **If exceeding limits**: Compress CLAUDE.md (consolidate similar patterns), summarize architecture.md (focus on current work area)

### Principle 4: Minimal Update Overhead
**"Updates take <5 minutes; not a documentation burden."**

Memory files must be maintainable. Apply this principle:
- **Update trigger**: End of productive session (significant progress made)
- **Update process**:
  - CLAUDE.md: Add 1-3 new patterns discovered this session
  - architecture.md: Update if structure changed (add/remove components)
  - decisions.md: Add 1-2 ADRs for major decisions
- **Quality over perfection**: Brief notes better than no notes
- **If session unproductive**: No update required (memory files unchanged)

## Memory File Structures

### CLAUDE.md (Project Patterns)

**Purpose**: Store discovered patterns, conventions, preferences

**Structure**:
```markdown
# CLAUDE.md — Project Patterns for [Project Name]

## Naming Conventions
- Variables: camelCase
- Constants: UPPER_SNAKE_CASE
- Classes: PascalCase
- Files: kebab-case.md

## Pattern: Error Handling
- Always validate before processing
- Use try/except at system boundary
- Log errors with context, not just message
- Return meaningful error messages to user

## Pattern: Testing
- Unit tests for pure functions
- Integration tests for API endpoints
- Mock external services in tests
- Minimum 80% coverage for business logic

## AI Collaboration Preferences
- I prefer complete solutions over scaffolding
- Explain tradeoffs explicitly before suggesting
- Ask clarifying questions before implementing
- Use type hints and docstrings in all code
```

**Guidelines**:
- Group patterns by category (Naming, Error Handling, Testing, etc.)
- Each pattern: Brief description + rationale
- Keep patterns actionable (what to DO, not what to avoid)
- Update when new pattern discovered or existing pattern refined

---

### architecture.md (System Design)

**Purpose**: Document how system is organized

**Structure**:
```markdown
# architecture.md — System Design

## Components
- **API Layer**: Handles HTTP requests (FastAPI)
- **Business Logic**: Core domain operations (Python modules)
- **Data Layer**: Manages persistence (PostgreSQL + SQLAlchemy)
- **Cache**: Performance optimization (Redis)

## Dependencies
API Layer → Business Logic → Data Layer (one-directional flow)
API Layer → Cache (read-through caching)

## Design Constraints
- All requests must complete in <500ms
- Data must be consistent (ACID transactions)
- All external calls have 10s timeout
- No direct database access from API Layer

## Key Decisions
- REST over GraphQL (team expertise, simpler for MVP)
- PostgreSQL over MongoDB (data structure stabilized, ACID critical)
- Redis for session storage (speed + expiry)

## Data Flow
1. User request → API Layer (validation)
2. API Layer → Business Logic (processing)
3. Business Logic → Data Layer (persistence)
4. Response path: Data → Logic → API → User
```

**Guidelines**:
- List components with brief responsibility
- Show dependency relationships (who depends on who)
- Capture design constraints (performance, consistency, security)
- Update when architecture changes (add/remove components)

---

### decisions.md (Architectural Decision Records)

**Purpose**: Record WHAT was decided and WHY

**Structure**:
```markdown
# decisions.md — Architectural Decisions

## 2025-11-18: Adopt TypeScript for Type Safety

**Context**: JavaScript codebase growing, type-related bugs increasing

**Decision**: Migrate to TypeScript incrementally (new code TS, old code JS)

**Why**:
- Catch type errors at compile time (prevent runtime bugs)
- Better IDE support (autocomplete, refactoring)
- Team familiar with TypeScript from prior projects

**Alternatives Considered**:
- **JSDoc annotations**: Less invasive, but limited type checking
- **PropTypes (React)**: Only runtime validation, doesn't prevent bugs
- **Flow**: Similar to TS, but smaller community and tooling

**Impact**:
- New files written in .ts/.tsx
- Build process updated (tsc compilation)
- Gradual migration over 3 months
- Expect 30% reduction in type-related bugs

---

## 2025-11-17: Separate Authentication into Module

**Context**: Auth logic scattered across API routes, hard to maintain

**Decision**: Extract authentication into standalone module

**Why**:
- Reusable across multiple services
- Easier to test in isolation
- Clearer separation of concerns
- Simplifies future OAuth integration

**Alternatives Considered**:
- **Keep inline**: Simpler initially, but maintenance nightmare
- **Use third-party service (Auth0)**: Expensive, vendor lock-in

**Impact**:
- API routes now import auth module
- Auth logic centralized in one place
- Can be reused in future microservices
```

**Guidelines**:
- One decision per section (Date + Title)
- Include: Context, Decision, Why, Alternatives, Impact
- Append chronologically (newest at top or bottom, stay consistent)
- Never delete old decisions (audit trail)
- If decision reversed: Add NEW decision explaining why

## Update Workflow

### Session Start (Load Memory)

**Step 1: Load Foundation Context**
```bash
# Load all three memory files at session start
1. Read CLAUDE.md (project patterns)
2. Read architecture.md (system structure)
3. Skim decisions.md (key decisions)
4. Inject into AI session prompt
```

**Prompt Template**:
```markdown
I'm working on [Project Name]. Here's the project context:

[Paste CLAUDE.md]
[Paste architecture.md]
[Paste relevant sections from decisions.md]

Today's task: [Describe current task]
```

**Expected result**: AI understands project patterns, structure, decisions from start

---

### During Session (Note Changes)

**Step 2: Track New Knowledge**

As you work, note:
- **New patterns discovered**: "Learned that validation should happen before parsing"
- **Architectural changes**: "Added Redis cache component"
- **Decisions made**: "Chose JWT over session tokens for API auth"

Keep notes in session document (don't interrupt flow to update memory files)

---

### Session End (Update Memory)

**Step 3: Extract and Consolidate**

**Update CLAUDE.md** (if new patterns discovered):
```markdown
## Pattern: Validation Before Parsing
- Always validate input format before attempting to parse
- Prevents cryptic parsing errors
- Return clear error messages on validation failure
- Discovered: 2025-11-18 (session debugging malformed JSON)
```

**Update architecture.md** (if structure changed):
```markdown
## Components
- **API Layer**: Handles HTTP requests (FastAPI)
- **Business Logic**: Core domain operations (Python modules)
- **Data Layer**: Manages persistence (PostgreSQL + SQLAlchemy)
- **Cache**: Performance optimization (Redis) [ADDED 2025-11-18]
```

**Update decisions.md** (if decisions made):
```markdown
## 2025-11-18: Use JWT for API Authentication

**Context**: API needs stateless authentication for mobile clients

**Decision**: Implement JWT tokens with 24-hour expiry

**Why**:
- Stateless (no server-side session storage)
- Scales horizontally (any server can validate)
- Standard, well-understood approach

**Alternatives**:
- Session cookies: Requires server-side storage, harder to scale
- OAuth2: Too complex for MVP, overkill for internal API

**Impact**:
- API returns JWT on successful login
- Clients send JWT in Authorization header
- Token expiry handled client-side (refresh workflow)
```

**Time estimate**: 5 minutes for typical session

---

## Conflict Resolution

### Conflict 1: Memory File Divergence

**Scenario**: Multiple developers/sessions update memory files simultaneously

**Resolution Strategy**:
1. **Git merge approach**: Treat memory files as code, merge conflicts normally
2. **Append-only sections**: decisions.md rarely conflicts (chronological)
3. **Last-write-wins sections**: architecture.md reflects current state (latest update wins)
4. **Pattern consolidation**: CLAUDE.md may have duplicate patterns (consolidate during review)

**Prevention**:
- Commit memory file updates immediately after session
- Pull latest memory files before session start
- Use feature branches if multiple parallel work streams

---

### Conflict 2: Contradictory Decisions

**Scenario**: Old decision contradicts new decision

**Resolution Strategy**:
1. **Never delete old decision** (audit trail)
2. **Add new decision referencing old**: "Reverses decision from YYYY-MM-DD"
3. **Explain why circumstances changed**: "New requirement makes old approach infeasible"
4. **Update architecture.md to reflect current state** (not historical)
5. **Update CLAUDE.md if pattern changed** (new pattern supersedes old)

**Example**:
```markdown
## 2025-10-15: Use MongoDB for Flexible Schema
[Original decision]

## 2025-11-18: Migrate to PostgreSQL for Data Integrity
**Reverses**: 2025-10-15 decision
**Why**: Schema stabilized, ACID transactions now critical for payments
**Impact**: Migration effort justified by improved reliability
```

---

### Conflict 3: Memory File Too Large

**Scenario**: CLAUDE.md or architecture.md exceeds 10-15% context budget

**Resolution Strategy**:

**For CLAUDE.md (patterns)**:
1. **Consolidate similar patterns**: Merge redundant patterns into one
2. **Archive old patterns**: Move rarely-used patterns to CLAUDE-archive.md
3. **Prioritize by frequency**: Keep most-used patterns in main file
4. **Create pattern categories**: Group related patterns for easier scanning

**For architecture.md (structure)**:
1. **Focus on current work area**: Summarize unrelated components
2. **Create architecture diagrams**: Replace text with visual (link to diagram file)
3. **Split by layer**: Create architecture-api.md, architecture-data.md if needed
4. **Highlight changes**: Use "Last Updated: YYYY-MM-DD" to show what's recent

**For decisions.md (ADRs)**:
1. **Keep all decisions** (audit trail non-negotiable)
2. **Load selectively**: Only load recent decisions (last 6 months) at session start
3. **Archive old decisions**: Move >1-year-old decisions to decisions-archive.md
4. **Summarize if needed**: Create decisions-summary.md with key decisions only

---

## Examples

### Example 1: New Project Setup

**Scenario**: Starting a new Flask API project

**Step 1: Create Initial CLAUDE.md**
```markdown
# CLAUDE.md — Flask API Project

## Naming Conventions
- Routes: snake_case (e.g., /user_profile)
- Functions: snake_case (Python convention)
- Classes: PascalCase

## Pattern: Error Handling
- Use Flask error handlers (@app.errorhandler)
- Return JSON errors with status codes
- Log all 500 errors with stack trace

## AI Collaboration Preferences
- Use type hints for function parameters
- Include docstrings for all public functions
- Prefer explicit over implicit (Zen of Python)
```

**Step 2: Create Initial architecture.md**
```markdown
# architecture.md — Flask API Structure

## Components
- **API Layer**: Flask routes (app/routes/)
- **Business Logic**: Service modules (app/services/)
- **Data Layer**: SQLAlchemy models (app/models/)

## Dependencies
API → Services → Models (layered architecture)

## Design Constraints
- All API responses are JSON
- Authentication via JWT tokens
- Database: PostgreSQL
```

**Step 3: Create Initial decisions.md**
```markdown
# decisions.md — Architectural Decisions

## 2025-11-18: Choose Flask Over FastAPI

**Context**: Need Python web framework for API project

**Decision**: Use Flask (not FastAPI)

**Why**:
- Team has Flask experience (faster development)
- Mature ecosystem (more libraries, tutorials)
- Sufficient for MVP requirements

**Alternatives**:
- FastAPI: Better performance, async support, but team learning curve
- Django: Too heavy for API-only project

**Impact**:
- Flask setup (1 day)
- Standard Flask patterns apply
- May migrate to FastAPI later if performance critical
```

---

### Example 2: Mid-Project Memory Update

**Scenario**: After 2-hour session adding user authentication

**Step 1: Review Session Notes**

Session notes:
- Implemented JWT authentication
- Discovered pattern: validate tokens in decorator
- Added Auth middleware to architecture
- Decided to use bcrypt for password hashing

**Step 2: Update CLAUDE.md**
```markdown
[Existing patterns...]

## Pattern: JWT Authentication
- Generate tokens on successful login
- Validate tokens in @require_auth decorator
- Include user_id in token payload
- Set 24-hour expiry
- Discovered: 2025-11-18 (implementing auth)
```

**Step 3: Update architecture.md**
```markdown
## Components
- **API Layer**: Flask routes (app/routes/)
- **Auth Middleware**: JWT validation (app/middleware/) [ADDED]
- **Business Logic**: Service modules (app/services/)
- **Data Layer**: SQLAlchemy models (app/models/)
```

**Step 4: Update decisions.md**
```markdown
## 2025-11-18: Use Bcrypt for Password Hashing

**Context**: Need secure password storage for user accounts

**Decision**: Use bcrypt library (not SHA256 or plaintext)

**Why**:
- Industry standard for password hashing
- Slow by design (prevents brute force)
- Built-in salt generation

**Alternatives**:
- SHA256: Too fast, vulnerable to rainbow tables
- Plaintext: Unacceptable security risk

**Impact**:
- Bcrypt added to requirements.txt
- Password hashing in user registration flow
- Login validates against bcrypt hash
```

**Time spent**: 5 minutes

---

### Example 3: Onboarding New Team Member

**Scenario**: New developer joins project after 6 months

**Step 1: Load Memory Files**

New developer reads:
1. CLAUDE.md → Understands project patterns (naming, error handling, AI preferences)
2. architecture.md → Understands system structure (components, dependencies, constraints)
3. decisions.md → Understands WHY system designed this way (key decisions, tradeoffs)

**Step 2: Start Session with Memory Context**

Developer starts AI session:
```markdown
I'm joining the [Project Name] project. Here's the project context:

[Paste CLAUDE.md]
[Paste architecture.md]
[Paste recent decisions from decisions.md]

I need to implement feature X. What's the right approach given our patterns?
```

**Result**: AI provides answers consistent with project patterns, no need to re-explain conventions

---

## Output Format

When designing memory files, produce three markdown files:

### CLAUDE.md Template
```markdown
# CLAUDE.md — Project Patterns for [Project Name]

## Naming Conventions
- [Convention 1]
- [Convention 2]

## Pattern: [Category 1]
- [Pattern description]
- [When to use]
- [Example]

## Pattern: [Category 2]
- [Pattern description]

## AI Collaboration Preferences
- [Preference 1]
- [Preference 2]
```

### architecture.md Template
```markdown
# architecture.md — System Design

## Components
- **[Component 1]**: [Responsibility]
- **[Component 2]**: [Responsibility]

## Dependencies
[Show relationships]

## Design Constraints
- [Constraint 1]
- [Constraint 2]
```

### decisions.md Template
```markdown
# decisions.md — Architectural Decisions

## YYYY-MM-DD: [Decision Title]

**Context**: [Why this decision needed]

**Decision**: [What was decided]

**Why**: [Reasoning]

**Alternatives Considered**: [Other options + why rejected]

**Impact**: [What changes as result]
```

---

## Tips for Success

1. **Start early**: Create memory files at project start (not after patterns forgotten)
2. **Update consistently**: Brief updates after each productive session (5 min)
3. **Consolidate periodically**: Review memory files monthly, merge similar patterns
4. **Load at session start**: Always inject memory files into AI context
5. **Append, don't rewrite**: Keep decisions immutable (audit trail)
6. **Prioritize actionability**: Patterns should be "what to DO" (not "what to avoid")
7. **Balance completeness with brevity**: Enough detail to be useful, not overwhelming
8. **Use memory files yourself**: Reference when making decisions (not just for AI)
9. **Commit with code**: Treat memory files as code (version control, review, merge)
10. **Iterate**: Memory file structure evolves as project grows

---

**Ready to design memory file architecture?** Provide:
- Project type and size (new/existing, small/large)
- Current context challenges (patterns forgotten, decisions re-debated)
- Team size (solo developer vs team)
- Session frequency (daily vs weekly)

Or share existing memory files and I'll assess structure, suggest improvements, and help consolidate!

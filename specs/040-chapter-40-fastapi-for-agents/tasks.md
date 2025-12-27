# Tasks: Chapter 40 — FastAPI for Agents

**Version**: 2.0.0
**Status**: Active
**Created**: 2025-12-27
**Input**: spec.md (v3.0.0), plan.md (v2.0.0)
**Related Issues**: #541, #542, #543, #544, #545, #546

---

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

## Path Convention

- **Content**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/`
- **Absolute base**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/`

---

## Phase 1: Renaming Operations

**Purpose**: Renumber existing lessons to accommodate new insertions

- [ ] T40.L03 [P] Rename `02-post-and-pydantic-models.md` → `03-post-and-pydantic-models.md`
  - Update `sidebar_position: 3`
  - Update internal lesson number references
  - Rename corresponding `.summary.md` file
  - Path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/`

- [ ] T40.L04 [P] Rename `03-full-crud-operations.md` → `04-full-crud-operations.md`
  - Update `sidebar_position: 4`
  - Update internal lesson number references
  - Rename corresponding `.summary.md` file
  - Path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/`

- [ ] T40.L05 [P] Rename `04-error-handling.md` → `05-error-handling.md`
  - Update `sidebar_position: 5`
  - Update internal lesson number references
  - Rename corresponding `.summary.md` file
  - Path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/`

- [ ] T40.L11 [P] Rename `06-streaming-with-sse.md` → `11-streaming-with-sse.md`
  - Update `sidebar_position: 11`
  - Update cross-references to other lesson numbers
  - Rename corresponding `.summary.md` file
  - Path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/`

**Checkpoint**: All renaming complete. New lessons can now be created.

---

## Phase 2: Create New Lessons (Issues #544, #542, #541, #543, #546)

**Purpose**: Create 5 new lessons per spec v3.0.0

### T40.L02 — Pytest Fundamentals (Issue #544, L1 Manual First)

- [ ] T40.L02 Create `02-pytest-fundamentals.md` — L1 Manual First
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/02-pytest-fundamentals.md`
    - Writes file directly (returns confirmation only, NOT full content)
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`
  - **VALIDATION**: educational-validator reads file from disk (MUST PASS before marking complete)
  - **SKILLS**: learning-objectives, exercise-designer, fact-check-lesson
  - **Content requirements**:
    - Layer: L1 (Manual) — Students write tests BY HAND first
    - Concepts: pytest test functions, TestClient, assertions, running tests, red-green cycle
    - Dependencies: `pytest httpx`
    - Builds on: Lesson 1 Hello FastAPI endpoint
    - What NOT to cover: fixtures, mocking, async tests, AI-generated tests

---

### T40.L06 — Environment Variables (Issue #542)

- [ ] T40.L06 Create `06-environment-variables.md`
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/06-environment-variables.md`
    - Writes file directly (returns confirmation only, NOT full content)
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`
  - **VALIDATION**: educational-validator reads file from disk (MUST PASS before marking complete)
  - **SKILLS**: learning-objectives, exercise-designer, fact-check-lesson
  - **Content requirements**:
    - Concepts: Why env vars, python-dotenv, .env files, pydantic-settings BaseSettings, .gitignore for secrets
    - Dependencies: `python-dotenv pydantic-settings`
    - Code example: Settings class with app_name, debug, database_url, secret_key
    - **Never commit secrets** message

---

### T40.L07 — SQLModel + Neon Setup (Issue #541, Fast Track)

- [ ] T40.L07 Create `07-sqlmodel-neon-setup.md` — Fast Track
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/07-sqlmodel-neon-setup.md`
    - Writes file directly (returns confirmation only, NOT full content)
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`
  - **VALIDATION**: educational-validator reads file from disk (MUST PASS before marking complete)
  - **SKILLS**: learning-objectives, exercise-designer, fact-check-lesson
  - **DOC**: Fetch SQLModel docs via Context7 for model definition patterns
  - **Content requirements**:
    - Simple setup, NO complexity
    - Concepts: SQLModel basics, Neon account setup (free tier), connection string via env var, create_engine/Session, convert in-memory CRUD to database
    - Dependencies: `sqlmodel psycopg2-binary`
    - What NOT to cover: Repository pattern (L10), Session DI (L10), Alembic (Ch 47), advanced relations
    - Future note: Neon MCP server for Claude as Digital FTE

---

### T40.L08 — JWT Authentication (Issue #543, Tokens Only)

- [ ] T40.L08 Create `08-jwt-authentication.md` — Tokens Only
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/08-jwt-authentication.md`
    - Writes file directly (returns confirmation only, NOT full content)
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`
  - **VALIDATION**: educational-validator reads file from disk (MUST PASS before marking complete)
  - **SKILLS**: learning-objectives, exercise-designer, fact-check-lesson
  - **DOC**: Fetch FastAPI docs via Context7 for OAuth2PasswordBearer patterns
  - **Content requirements**:
    - SEPARATE from password hashing (one concept per lesson)
    - Concepts: Why JWT, pyjwt token creation with expiration, OAuth2PasswordBearer, token verification dependency, protected endpoints with Depends(get_current_user)
    - Dependencies: `pyjwt`
    - Temporary: plain-text password check (EXPLICITLY noted as insecure, fixed in L09)
    - Uses settings.secret_key from L06

---

### T40.L09 — Password Hashing + Rate Limiting (Issue #546)

- [ ] T40.L09 Create `09-password-hashing-rate-limiting.md`
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/09-password-hashing-rate-limiting.md`
    - Writes file directly (returns confirmation only, NOT full content)
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`
  - **VALIDATION**: educational-validator reads file from disk (MUST PASS before marking complete)
  - **SKILLS**: learning-objectives, exercise-designer, fact-check-lesson
  - **Content requirements**:
    - Fixes insecure L08 login
    - Concepts: pwdlib/Argon2 (NOT bcrypt), verify_password function, user signup with hashed password, in-memory rate limiting
    - Dependencies: `"pwdlib[argon2]"`
    - Table: In-memory vs Redis rate limiting trade-offs
    - In-memory rate limiter (NOT Redis — save for production chapter)

**Checkpoint**: All 5 new lessons created. Phase 3 can begin.

---

## Phase 3: Rewrite Existing Lessons

**Purpose**: Modernize legacy content with new patterns from L6-L9

### T40.L10 — Rewrite Dependency Injection (05 → 10)

- [ ] T40.L10 Rewrite `10-dependency-injection.md` — L2 AI Collaboration
  - **SUBAGENT**: content-implementer
    - Source: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/05-dependency-injection.md`
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/10-dependency-injection.md`
    - Writes file directly (returns confirmation only, NOT full content)
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`
  - **VALIDATION**: educational-validator reads file from disk (MUST PASS before marking complete)
  - **SKILLS**: learning-objectives, exercise-designer, ai-collaborate-teaching (L2)
  - **What to KEEP**:
    - Core DI explanation
    - Repository pattern basics
    - Depends() usage
    - Common mistakes section
  - **What to REMOVE/UPDATE**:
    - Remove references to "Lesson 7 agent injection" (now L12)
    - Update to work with SQLModel Session (from L07)
    - Update to use Settings from L06
    - Add Session dependency injection
  - **Layer 2 elements**: Students refactor existing code WITH AI assistance, Three Roles

---

### T40.L12 — Simplify Agent Integration (07 → 12, Issue #545)

- [ ] T40.L12 Rewrite `12-agent-integration.md` — MAJOR SIMPLIFY, L2 AI Collaboration
  - **SUBAGENT**: content-implementer
    - Source: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/07-agent-integration.md`
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/12-agent-integration.md`
    - Writes file directly (returns confirmation only, NOT full content)
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`
  - **VALIDATION**: educational-validator reads file from disk (MUST PASS before marking complete)
  - **SKILLS**: learning-objectives, exercise-designer, ai-collaborate-teaching (L2)
  - **REMOVE** (~400 lines, moved to Capstone L13):
    - TaskManager Triage Agent
    - TaskManager Scheduler Specialist
    - TaskManager Collaboration Specialist
    - Handoff endpoint with complex routing
    - Streaming handoffs section
    - Direct specialist endpoints section
    - Challenge: Add a Third Specialist
  - **KEEP** (core insight):
    - APIs → Functions → Tools pattern
    - Single agent with 2-3 tools
    - SSE streaming endpoint
    - Basic tool call → response flow
  - **Target**: 654 lines → ~250 lines
  - **Learning outcome**: "The REST APIs I built are just functions. Functions become tools. An agent uses tools and streams results."

---

### T40.L13 — Rewrite Capstone (08 → 13)

- [ ] T40.L13 Rewrite `13-capstone-agent-powered-task-service.md` — L4 Spec-Driven, Digital FTE
  - **SUBAGENT**: content-implementer
    - Source: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/08-capstone-agent-powered-task-service.md`
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/13-capstone-agent-powered-task-service.md`
    - Writes file directly (returns confirmation only, NOT full content)
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`
  - **VALIDATION**: educational-validator reads file from disk (MUST PASS before marking complete)
  - **SKILLS**: learning-objectives, exercise-designer, assessment-builder (capstone)
  - **What to KEEP**:
    - Multi-agent architecture (triage + specialists)
    - Streaming handoff visibility
    - Testing checklist
  - **What to ADD** (new patterns from L6-L9):
    - Real Neon PostgreSQL (not in-memory)
    - Environment variables configuration
    - JWT authentication on agent endpoints
    - Password hashing for user signup
    - Rate limiting on expensive endpoints
  - **Layer 4 structure**:
    1. Specification FIRST (spec.md before implementation)
    2. Compose Skills (accumulated patterns from L1-L12)
    3. AI Orchestrates (agent implements spec using composed components)
    4. Validate (test against spec success criteria)
  - **Digital FTE Outcome**: Deployable agent service for monetization

**Checkpoint**: All lessons rewritten with modernized patterns.

---

## Phase 4: Cleanup and Validation

**Purpose**: Remove old files, update README, run validators

- [ ] T40.OLD Remove old lesson files (now renamed/rewritten)
  - Delete: `05-dependency-injection.md` (now at `10-dependency-injection.md`)
  - Delete: `05-dependency-injection.summary.md`
  - Delete: `06-streaming-with-sse.md` (now at `11-streaming-with-sse.md`)
  - Delete: `06-streaming-with-sse.summary.md`
  - Delete: `07-agent-integration.md` (now at `12-agent-integration.md`)
  - Delete: `07-agent-integration.summary.md`
  - Delete: `08-capstone-agent-powered-task-service.md` (now at `13-capstone-agent-powered-task-service.md`)
  - Delete: `08-capstone-agent-powered-task-service.summary.md`
  - Path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/`

- [ ] T40.SUMMARY [P] Generate `.summary.md` files for new lessons
  - Use summary-generator skill
  - Files: L02, L06, L07, L08, L09, L10, L11, L12, L13

- [ ] T40.README Update `README.md` with new 13-lesson structure
  - Path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/README.md`
  - Update lesson list (13 lessons)
  - Update dependencies section
  - Update learning outcomes

- [ ] T40.VALIDATE Run validators (parallel)
  - educational-validator on all 13 lessons
  - fact-check-lesson on new lessons (L02, L06-L09)
  - Build check: `pnpm nx build learn-app`

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Rename) ──> Phase 2 (Create New) ──> Phase 3 (Rewrite) ──> Phase 4 (Validate)
```

### Lesson Dependencies (within chapter)

```
L01 Hello FastAPI
  │
  v
L02 Pytest Fundamentals <── Tests L01 endpoint
  │
  v
L03 POST + Pydantic
  │
  v
L04 Full CRUD
  │
  v
L05 Error Handling
  │
  ├──> L06 Environment Variables (parallel track)
  │       │
  │       v
  │     L07 SQLModel + Neon (requires L06 for env vars)
  │       │
  │       v
  │     L08 JWT Auth (requires L06 for secret_key)
  │       │
  │       v
  │     L09 Password + Rate Limit (requires L08 for auth fix)
  │
  v
L10 Dependency Injection (can reference L07 patterns)
  │
  v
L11 Streaming with SSE
  │
  v
L12 Agent Integration (requires L10, L11)
  │
  v
L13 Capstone (requires ALL lessons L01-L12)
```

### Parallel Opportunities

**Phase 1** (all parallelizable):
```
T40.L03, T40.L04, T40.L05, T40.L11 — different files, no dependencies
```

**Phase 2** (some parallelizable):
```
T40.L02 — independent (tests L01)
T40.L06 — independent (new content)
T40.L07 — depends on L06 (env vars)
T40.L08 — depends on L06 (secret_key)
T40.L09 — depends on L08 (fixes auth)

Parallel: T40.L02, T40.L06
Then: T40.L07, T40.L08
Then: T40.L09
```

**Phase 3** (some parallelizable):
```
T40.L10, T40.L12 — can run in parallel (different lessons)
T40.L13 — depends on all previous (capstone)
```

---

## Implementation Strategy

### Recommended Execution

1. **Phase 1**: Run all rename operations in parallel
2. **Phase 2a**: Create L02, L06 in parallel
3. **Phase 2b**: Create L07, L08 (depend on L06)
4. **Phase 2c**: Create L09 (depends on L08)
5. **Phase 3a**: Rewrite L10, L12 in parallel
6. **Phase 3b**: Rewrite L13 (capstone, depends on all)
7. **Phase 4**: Cleanup, summaries, README, validators

### Total Tasks

- Phase 1: 4 tasks (rename)
- Phase 2: 5 tasks (create new)
- Phase 3: 3 tasks (rewrite)
- Phase 4: 4 tasks (cleanup + validation)
- **Total**: 16 tasks

---

## Issue Traceability

| Issue | Task | Status |
|-------|------|--------|
| #544 | T40.L02 Pytest Fundamentals | Pending |
| #542 | T40.L06 Environment Variables | Pending |
| #541 | T40.L07 SQLModel + Neon Setup | Pending |
| #543 | T40.L08 JWT Authentication | Pending |
| #546 | T40.L09 Password Hashing + Rate Limiting | Pending |
| #545 | T40.L12 Agent Integration (Simplify) | Pending |

---

*Generated by sp.tasks v2.0.0 from plan.md v2.0.0 and spec.md v3.0.0*

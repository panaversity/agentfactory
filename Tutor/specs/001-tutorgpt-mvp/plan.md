# Implementation Plan: TutorGPT MVP - AI Tutor for Docusaurus Book

**Branch**: `001-tutorgpt-mvp` | **Date**: 2025-11-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-tutorgpt-mvp/spec.md`

## Summary

Build an autonomous AI tutor that sits beside students as they read the "AI-Native Software Development" book online. Students can ask questions, highlight confusing text, and receive contextual, personalized explanations powered by multi-level RAG (4 levels) and an autonomous agent system. The system uses **Gemini embeddings** for semantic search, OpenAI Agents SDK for autonomous teaching behavior, and ChatKit for the UI interface embedded in all 107 Docusaurus book pages.

**Core Value**: Instant, context-aware help while reading → reduces frustration, maintains motivation, improves learning outcomes.

**Technical Approach**: FastAPI backend with 4-level RAG retrieval (using **Gemini embeddings**), OpenAI Agents SDK for autonomous decision-making, ChatKit widget integrated into Docusaurus, SQLite for session persistence.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies** (✅ Research Complete):
- **Backend**: FastAPI 0.104+, uvicorn[standard]
- **AI/Agent**: OpenAI Agents SDK (`openai-agents`) - https://openai.github.io/openai-agents-python/
- **Embeddings**: **Google Gemini SDK (`google-genai`) with `gemini-embedding-001`** - NEW SDK (GA Nov 2025)
- **RAG**: **ChromaDB 0.4+ (DIRECT)** - Skipping LangChain for MVP simplicity
- **Frontend**: Docusaurus 3.x, OpenAI ChatKit React (`@openai/chatkit-react` v1.2.0)
- **Database**: aiosqlite (SQLite async)

**Storage**:
- Primary: SQLite 3.x (sessions, messages, profiles)
- Vector: ChromaDB 0.4+ (book content embeddings via **Gemini**)
- Static: Local file system (logs)

**Testing**: pytest, pytest-asyncio, pytest-cov (80% coverage minimum)

**Target Platform**:
- Backend: Linux/Windows server (Python 3.11+)
- Frontend: Modern web browsers (Chrome, Firefox, Safari, Edge - latest 2 versions)

**Project Type**: Web application (backend API + frontend integration)

**Performance Goals**:
- Response time: <3 seconds (95th percentile)
- RAG retrieval: <500ms
- Page load: <2 seconds with ChatKit loaded
- Concurrent users: 100+ during MVP

**Constraints**:
- 4-week timeline (strict)
- Anonymous sessions only (no authentication in MVP)
- 107 book pages must all work identically
- Mobile-responsive (basic - full optimization deferred to Phase 2)

**Scale/Scope**:
- 107 markdown lesson files to embed
- 10,000+ sessions supported
- ~500-1000 chunks per lesson (estimated 50,000-100,000 total chunks)
- Average 5-10 messages per session

---

## TDD Methodology

**Test-Driven Development (TDD) is MANDATORY for this project.**

### TDD Workflow (Red-Green-Refactor)

Every single task follows this cycle:

1. **🔴 RED**: Write test FIRST (test fails - no implementation yet)
2. **🟢 GREEN**: Write MINIMAL code to make test pass
3. **🔵 REFACTOR**: Improve code quality while tests stay green
4. **🔁 REPEAT**: Next feature

### Test Pyramid for TutorGPT

```
           /\
          /  \
         / E2E \          ← Scenario Tests (User Journeys)
        /______\
       /        \
      / Integration\       ← Agent + Tools + RAG working together
     /____________\
    /              \
   /  Unit Tests    \      ← Individual functions, tools, utilities
  /__________________\
```

**Test Distribution** (Target):
- **60%** Unit tests (fast, isolated, specific)
- **30%** Integration tests (agent + tools + services)
- **10%** Scenario/E2E tests (full user journeys)

### Test Types for Agent-First System

#### 1. Unit Tests (60%)
Test individual components in isolation:
- RAG search functions
- Embedding generation
- Database queries
- Text processing utilities
- Session management functions

**Example**:
```python
def test_search_book_content_with_metadata_filter():
    # Given: A query and chapter context
    query = "What is Python?"
    chapter = "04-python"

    # When: Searching with chapter filter
    results = rag_service.search(query, metadata={"chapter": chapter})

    # Then: Results are from correct chapter only
    assert all(r.metadata["chapter"] == chapter for r in results)
    assert len(results) > 0
```

#### 2. Integration Tests (30%)
Test components working together:
- Agent + RAG system
- Agent + Student profile
- ChatKit backend + Agent
- Full question-answer pipeline

**Example**:
```python
async def test_agent_uses_rag_for_book_questions():
    # Given: Agent with RAG tool
    agent = create_tutor_agent()

    # When: Student asks about book content
    response = await agent.answer("What is async programming?")

    # Then: Agent called search_book_content tool
    assert "search_book_content" in response.tools_used
    # And: Response includes book reference
    assert "Chapter" in response.message
```

#### 3. Behavior Tests (20%)
Test agent TEACHING QUALITY (most critical):
- Agent teaches from book (not generic knowledge)
- Agent provides encouraging responses
- Agent adapts to student confusion
- Agent asks clarifying questions when needed
- Agent celebrates milestones

**Example**:
```python
async def test_agent_teaches_with_encouragement():
    # Given: A confused student's question
    question = "I don't understand variables at all"

    # When: Agent responds
    response = await agent.answer(question)

    # Then: Response is encouraging
    assert any(word in response.message.lower()
               for word in ["great question", "don't worry", "let me help"])
    # And: Agent simplifies explanation
    assert response.tool_calls["explain_concept"]["depth"] == "simple"
    # And: Agent offers analogy
    assert response.tool_calls["explain_concept"]["use_analogy"] == True
```

#### 4. Scenario Tests (10%)
Test complete user journeys end-to-end:
- First-time student gets help (US1)
- Student highlights text for explanation (US2)
- Returning student sees history (US3)
- Agent adapts to confusion (US4)

**Example**:
```python
async def test_first_time_student_complete_journey():
    # Setup: New student, Chapter 1 page
    student = create_test_student()
    context = {"chapter": "01-intro", "page": "/docs/01-intro"}

    # Step 1: Student asks question
    q1_response = await agent.answer(
        "What is AI-driven development?",
        context=context
    )

    # Assert: Fast response
    assert q1_response.time_ms < 3000
    # Assert: From book
    assert "search_book_content" in q1_response.tools_used
    # Assert: References current page
    assert "Chapter 1" in q1_response.message

    # Step 2: Student asks follow-up
    q2_response = await agent.answer("Can you give an example?")

    # Assert: Agent remembers context
    assert "provide_code_example" in q2_response.tools_used
```

### Coverage Targets

- **Overall code coverage**: ≥80%
- **Critical agent logic**: 100% (personality, decision-making, tool selection)
- **RAG pipeline**: 100% (search, ranking, metadata filtering)
- **Session management**: 100% (persistence, restoration)
- **Agent behavior**: 100% (teaching quality verified)

### Testing Tools

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=app --cov-report=html

# Run specific test types
pytest tests/unit/           # Unit tests only
pytest tests/integration/     # Integration tests only
pytest tests/behavior/        # Agent behavior tests only
pytest tests/scenarios/       # E2E scenario tests only

# Run tests with agent behavior verbose output
pytest -v tests/behavior/ --log-cli-level=INFO
```

### TDD for Each Phase

Every phase follows **Test → Implement → Verify**:

- **Phase 1 (Setup)**: Create test infrastructure, pytest config
- **Phase 2 (Agent Core)**: Test agent personality, decision-making FIRST
- **Phase 3 (Agent Tools)**: Test each tool individually before integration
- **Phase 4 (Services)**: Test RAG, sessions, embeddings in isolation
- **Phase 5-8 (User Stories)**: Test complete flows (scenario tests)
- **Phase 9 (Polish)**: Load tests, performance tests, edge case tests

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principle Alignment

**✅ I. Ship Fast, Ship Well**
- ONE interface: Book + ChatKit sidebar ✓
- 4-week timeline enforced ✓
- ChatKit provides production UI (no custom build) ✓
- Focus on intelligence (RAG, Agent) not UI ✓

**✅ II. Context is Everything**
- Multi-level RAG (4 levels) planned ✓
- Page context captured ✓
- Highlight detection included ✓
- Session persistence planned ✓

**✅ III. The Agent is Autonomous**
- OpenAI Agents SDK integration planned ✓
- LangChain for RAG orchestration ✓
- GPT-4 for intelligence ✓
- Warm greeting on first visit ✓

**✅ IV. Students Learn Better with Help**
- Highlight → instant explanation ✓
- Ask → contextual answers ✓
- Personalization tracking ✓
- Encouragement built into agent ✓

**✅ V. Personalization Through Observation**
- Behavioral tracking (questions, pace, topics) ✓
- Automatic adaptation ✓
- No surveys/manual config ✓

**✅ VI. Production Quality from Day One**
- <3 sec responses ✓
- All 107 pages ✓
- Accurate (RAG from book) ✓
- Session persistence ✓
- Testing required ✓

### Technology Stack Alignment

**Backend**: FastAPI (from constitution) ✓
**AI Agent**: OpenAI Agents SDK (`openai-agents`) (from constitution) ✓
**Embeddings**: **Gemini `gemini-embedding-001`** via `google-genai` SDK (user-specified) ✓
**RAG**: ChromaDB direct (LangChain skipped for MVP simplicity) ⚠️
**Database**: SQLite (**updated from PostgreSQL** for MVP simplicity - can migrate later) ⚠️
**Frontend**: ChatKit (`@openai/chatkit-react`) + Docusaurus (from constitution) ✓

**Constitution Deviations - JUSTIFIED**:

1. **Database**: Using SQLite instead of PostgreSQL for MVP
   - **Reason**: Simpler deployment, no external DB server needed, sufficient for 10k+ sessions
   - **Migration Path**: Schema designed to be PostgreSQL-compatible for Phase 2 scale-up
   - **Tradeoff**: Limited to single-server deployment in MVP (acceptable for <100 concurrent users)

2. **RAG Framework**: Using ChromaDB directly instead of LangChain
   - **Reason**: LangChain adds complexity for features not needed in MVP
   - **Migration Path**: Custom retrieval logic is simpler and more performant for MVP scale
   - **Tradeoff**: Less abstraction, but more control and fewer dependencies
   - **Future**: Can add LangChain in Phase 2 if advanced RAG patterns needed

### Week-by-Week Alignment

**Week 1-2**: Backend (RAG, Agent, DB) - constitution Week 1-2 ✓
**Week 3**: ChatKit integration - constitution Week 3 ✓
**Week 4**: Polish & deploy - constitution Week 4 ✓

**MVP Scope Alignment**: All "INCLUDED in MVP" features from constitution are in plan ✓

### Gate Status: **PASS** ✅

## Project Structure

### Documentation (this feature)

```text
specs/001-tutorgpt-mvp/
├── spec.md              # Feature specification
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0: Technology research and decisions
├── data-model.md        # Phase 1: Data models and relationships
├── quickstart.md        # Phase 1: Developer onboarding guide
├── contracts/           # Phase 1: API contracts (OpenAPI specs)
│   ├── chat-api.yaml
│   ├── session-api.yaml
│   └── models.yaml
├── checklists/          # Existing: Quality validation
│   └── requirements.md
└── tasks.md             # Phase 2: Generated by /sp.tasks (not by /sp.plan)
```

### Source Code (repository root)

```text
ai-native-software-development/   (Existing repo)
│
├── book-source/                  (Existing Docusaurus site - 107 MD files)
│   ├── docs/                     (DON'T MODIFY - book content)
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatKitWidget/    ← NEW: ChatKit integration
│   │   │       ├── index.tsx     (Main component)
│   │   │       ├── ContextCapture.ts (Page/highlight detection)
│   │   │       ├── api.ts        (Backend communication)
│   │   │       └── styles.module.css
│   │   └── theme/
│   │       └── Root.tsx          ← MODIFY: Add ChatKit to all pages
│   ├── docusaurus.config.ts
│   └── package.json
│
└── Tutor/                        ← NEW: Backend project
    │
    ├── README.md
    ├── .gitignore
    │
    ├── backend/                  (FastAPI application)
    │   │
    │   ├── app/
    │   │   ├── __init__.py
    │   │   ├── main.py           (FastAPI app entry)
    │   │   │
    │   │   ├── api/              (REST endpoints)
    │   │   │   ├── __init__.py
    │   │   │   ├── routes.py     (Router aggregation)
    │   │   │   ├── chat.py       (POST /chat - main interaction)
    │   │   │   ├── session.py    (Session CRUD)
    │   │   │   ├── highlight.py  (POST /highlight)
    │   │   │   └── health.py     (GET /health)
    │   │   │
    │   │   ├── models/           (Pydantic schemas)
    │   │   │   ├── __init__.py
    │   │   │   ├── chat.py       (ChatRequest, ChatResponse)
    │   │   │   ├── session.py    (Session, StudentProfile, Progress)
    │   │   │   ├── rag.py        (RetrievalResult, Chunk)
    │   │   │   └── context.py    (PageContext)
    │   │   │
    │   │   ├── services/         (Business logic)
    │   │   │   ├── __init__.py
    │   │   │   ├── session_manager.py    (Session lifecycle)
    │   │   │   ├── multi_level_rag.py    (4-level retrieval)
    │   │   │   ├── agent_coordinator.py  (OpenAI Agents SDK)
    │   │   │   ├── embedder.py           (Book → Gemini embeddings)
    │   │   │   └── personalizer.py       (Behavioral analysis)
    │   │   │
    │   │   ├── core/             (Core utilities)
    │   │   │   ├── __init__.py
    │   │   │   ├── config.py     (Settings from .env)
    │   │   │   ├── database.py   (SQLite async connection)
    │   │   │   └── logging.py    (Structured logging)
    │   │   │
    │   │   └── utils/            (Helpers)
    │   │       ├── __init__.py
    │   │       ├── text_processing.py
    │   │       └── validators.py
    │   │
    │   ├── data/                 (Runtime data - gitignored)
    │   │   ├── sessions.db       (SQLite database)
    │   │   ├── embeddings/       (ChromaDB persist directory)
    │   │   └── logs/             (Application logs)
    │   │
    │   ├── scripts/              (Utility scripts)
    │   │   ├── embed_book.py     (Generate Gemini embeddings for book)
    │   │   ├── init_db.py        (Initialize SQLite schema)
    │   │   └── test_rag.py       (Manual RAG testing)
    │   │
    │   ├── tests/                (Test suite)
    │   │   ├── __init__.py
    │   │   ├── conftest.py       (Pytest fixtures)
    │   │   ├── test_api/
    │   │   │   ├── test_chat.py
    │   │   │   ├── test_session.py
    │   │   │   └── test_highlight.py
    │   │   ├── test_services/
    │   │   │   ├── test_rag.py
    │   │   │   ├── test_agent.py
    │   │   │   ├── test_embedder.py
    │   │   │   └── test_session_manager.py
    │   │   └── test_utils/
    │   │       └── test_text_processing.py
    │   │
    │   ├── .env.example          (Environment template)
    │   ├── .env                  (Environment vars - gitignored)
    │   ├── pyproject.toml        (Poetry dependencies)
    │   ├── poetry.lock
    │   └── README.md
    │
    └── docs/                     (Project documentation)
        ├── architecture.md       (System design)
        ├── api.md                (API documentation)
        └── deployment.md         (Deployment guide)
```

**Structure Decision**: **Web application** structure selected.

**Rationale**:
- Separate `backend/` (FastAPI) and `book-source/` (Docusaurus + ChatKit integration)
- Backend is independent service (can scale separately)
- Frontend modifies existing Docusaurus site minimally (add ChatKit component only)
- Clear separation of concerns: backend = intelligence, frontend = presentation

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| SQLite instead of PostgreSQL | MVP needs simple deployment with no external DB server. SQLite supports 10k+ sessions and <100 concurrent users sufficiently. | PostgreSQL adds deployment complexity (separate DB server, connection pooling, credentials management) that's unnecessary for MVP scale. Migration path to PostgreSQL exists for Phase 2 when scaling beyond 100 concurrent users. |

---

## Phase 0: Research & Technology Decisions ✅ COMPLETE

**Goal**: Resolve all technical uncertainties and document authoritative implementation patterns.

**Status**: ✅ **COMPLETE** - See `research.md` for full findings

**Key Findings**:
1. **Gemini Embeddings**: Use NEW `google-genai` SDK (NOT deprecated `google-generativeai`)
2. **Agent SDK**: `openai-agents` package with `@function_tool` decorator for tools
3. **ChatKit**: NPM package `@openai/chatkit-react` v1.2.0
4. **LangChain**: **SKIP for MVP** - Direct ChromaDB is simpler
5. **Distance Metric**: Use cosine similarity for Gemini embeddings

### Research Tasks (All Complete)

#### R1: Google Gemini Embeddings API

**Research Objective**: Understand how to use `models/embedding-001` with Python SDK

**Sources**:
- Official: https://ai.google.dev/gemini-api/docs/embeddings
- SDK: https://github.com/google/generative-ai-python
- API Reference: https://ai.google.dev/api/python/google/generativeai

**Questions to Answer**:
1. What is the exact model name for embeddings? (`models/embedding-001` or `embedding-001`?)
2. How to authenticate? (API key setup)
3. What is the Python SDK installation? (`pip install google-generativeai`)
4. How to generate embeddings for text? (Function signatures, parameters)
5. What is the embedding dimension? (768? 1536?)
6. Are there batch processing capabilities?
7. Rate limits and quotas?
8. How to integrate with ChromaDB?
9. Cost considerations?

**Deliverable**: `research.md` section with:
- Installation command
- Authentication pattern
- Code example for generating embeddings
- Embedding dimension
- Integration pattern with ChromaDB

#### R2: OpenAI Agents SDK

**Research Objective**: Understand autonomous agent creation and function calling

**Sources**:
- **PRIMARY**: https://openai.github.io/openai-agents-python/
- GitHub: https://github.com/openai/openai-agents-python
- Examples: Review all examples in repo

**Questions to Answer**:
1. Latest SDK version and installation
2. How to create an autonomous agent?
3. Function calling syntax (tools/functions)
4. State management between turns
5. Streaming responses
6. Error handling patterns
7. Context window management
8. How to inject custom context (RAG results, student profile)
9. Best practices for system prompts
10. How to make agent "think" before responding

**Deliverable**: `research.md` section with:
- Installation command
- Agent creation pattern
- Function/tool definition syntax
- Context injection example
- System prompt best practices

#### R3: OpenAI ChatKit

**Research Objective**: Understand ChatKit CDN integration and customization

**Sources**:
- **PRIMARY**: https://platform.openai.com/docs/guides/custom-chatkit
- CDN URL and versioning
- React integration patterns

**Questions to Answer**:
1. Latest ChatKit CDN URL
2. How to initialize ChatKit in React component?
3. Configuration options (positioning, styling, behavior)
4. How to capture custom context (page, highlighted text)?
5. How to connect to custom backend (not OpenAI)?
6. Event listeners (message sent, received, widget opened/closed)
7. Session persistence in browser
8. Mobile responsiveness
9. Accessibility features
10. Customization limits

**Deliverable**: `research.md` section with:
- CDN URL and version
- React initialization code
- Configuration object structure
- Custom backend integration pattern
- Context capture approach

#### R4: ChromaDB with Custom Embeddings

**Research Objective**: Integrate Gemini embeddings with ChromaDB

**Sources**:
- https://docs.trychroma.com/
- Custom embedding function documentation
- Performance tuning guide

**Questions to Answer**:
1. How to provide custom embeddings (not OpenAI)?
2. ChromaDB collection configuration for Gemini embeddings
3. Metadata filtering capabilities
4. Query performance optimization
5. Persistence configuration
6. Distance metrics (cosine similarity for Gemini?)

**Deliverable**: `research.md` section with:
- ChromaDB setup for Gemini embeddings
- Collection creation code
- Custom embedding function
- Query pattern example

#### R5: LangChain RAG Pipeline

**Research Objective**: Build multi-level RAG with LangChain + Gemini + ChromaDB

**Sources**:
- https://python.langchain.com/docs/get_started/introduction
- RAG patterns documentation
- Custom embeddings integration

**Questions to Answer**:
1. LangChain version compatibility with Gemini
2. How to use custom embeddings (Gemini) in LangChain?
3. Multi-level retrieval patterns
4. Metadata filtering in retrievers
5. Result ranking and fusion
6. Integration with OpenAI Agents SDK

**Deliverable**: `research.md` section with:
- LangChain + Gemini integration pattern
- Multi-level retrieval implementation approach
- Metadata filtering examples

### Research Output Format

**File**: `specs/001-tutorgpt-mvp/research.md`

```markdown
# Technology Research: TutorGPT MVP

**Date**: 2025-11-08
**Purpose**: Document authoritative implementation patterns from official sources

## 1. Google Gemini Embeddings (`models/embedding-001`)

### Official Documentation
- Primary: https://ai.google.dev/gemini-api/docs/embeddings
- SDK: https://github.com/google/generative-ai-python

### Key Findings

**Installation**:
```bash
pip install google-generativeai
```

**Authentication**:
```python
import google.generativeai as genai
genai.configure(api_key="GOOGLE_API_KEY")
```

**Embedding Generation**:
```python
result = genai.embed_content(
    model="models/embedding-001",
    content="Your text here",
    task_type="retrieval_document"  # or "retrieval_query"
)
embedding = result['embedding']  # List[float]
```

**Embedding Dimension**: [TO BE RESEARCHED - typically 768]

**Rate Limits**: [TO BE RESEARCHED]

**Cost**: [TO BE RESEARCHED]

### Integration with ChromaDB

**Decision**: Use custom embedding function
**Pattern**:
```python
class GeminiEmbeddingFunction:
    def __call__(self, texts: List[str]) -> List[List[float]]:
        embeddings = []
        for text in texts:
            result = genai.embed_content(
                model="models/embedding-001",
                content=text,
                task_type="retrieval_document"
            )
            embeddings.append(result['embedding'])
        return embeddings
```

## 2. OpenAI Agents SDK

[TO BE FILLED AFTER RESEARCH]

## 3. OpenAI ChatKit

[TO BE FILLED AFTER RESEARCH]

## 4. ChromaDB Configuration

[TO BE FILLED AFTER RESEARCH]

## 5. LangChain RAG Pipeline

[TO BE FILLED AFTER RESEARCH]

## Summary of Decisions

| Component | Technology | Rationale |
|-----------|------------|-----------|
| Embeddings | Gemini `models/embedding-001` | User-specified, cost-effective, high quality |
| Agent | OpenAI Agents SDK | Autonomous teaching behavior, function calling |
| UI | ChatKit | Production-ready, fast integration |
| RAG | LangChain + ChromaDB | Multi-level retrieval, metadata filtering |
| Database | SQLite | Simple deployment, sufficient for MVP |
```

---

## Phase 1: Design & Contracts ✅ COMPLETE

**Prerequisites**: ✅ `research.md` complete with all findings

**Status**: ✅ **COMPLETE** - All design documents created

**Deliverables Created**:
1. ✅ `data-model.md` - Complete data structures and Pydantic models
2. ✅ `contracts/` - Full API contracts for all endpoints
3. ✅ `quickstart.md` - Developer setup guide with UV package manager

### Deliverables Summary

#### 1. Data Model (`data-model.md`)

**Entities**:

**StudentSession**
- `session_id`: string (primary key, UUID)
- `student_id`: string (anonymous identifier)
- `created_at`: datetime
- `updated_at`: datetime
- `current_chapter`: string (nullable)
- `current_lesson`: string (nullable)
- `chapters_viewed`: JSON array of strings
- `lessons_completed`: JSON array of strings
- `learning_pace`: enum (slow, medium, fast)
- `confused_topics`: JSON array of strings
- `strong_topics`: JSON array of strings
- `questions_asked`: integer
- `highlights_made`: integer
- `total_time_minutes`: integer

**Message**
- `message_id`: string (primary key, UUID)
- `session_id`: string (foreign key → StudentSession)
- `role`: enum (user, assistant)
- `content`: text
- `timestamp`: datetime
- `page_context`: JSON (nullable - page_path, page_title, chapter, lesson, section_id, highlighted_text)

**BookContentChunk** (ChromaDB)
- `chunk_id`: string (primary key)
- `text`: text (full chunk content)
- `embedding`: vector (Gemini embedding, dimension TBD)
- `metadata`: JSON
  - `chapter`: string
  - `lesson`: string
  - `file_path`: string
  - `h1`, `h2`, `h3`: string (heading hierarchy)

**Relationships**:
- StudentSession 1→N Message

**State Transitions**:
- Session: created → active → (updated on each interaction)
- Message: created (immutable)

#### 2. API Contracts (`contracts/`)

**OpenAPI 3.0 Specification**

**File**: `contracts/chat-api.yaml`

```yaml
openapi: 3.0.0
info:
  title: TutorGPT Chat API
  version: 1.0.0
  description: Autonomous AI tutor chat interface

paths:
  /chat:
    post:
      summary: Send message to AI tutor
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ChatRequest'
      responses:
        '200':
          description: Agent response
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ChatResponse'
        '404':
          description: Session not found
        '500':
          description: Server error

components:
  schemas:
    ChatRequest:
      type: object
      required:
        - session_id
        - message
      properties:
        session_id:
          type: string
        message:
          type: string
        page_context:
          $ref: '#/components/schemas/PageContext'

    ChatResponse:
      type: object
      properties:
        message:
          type: string
        sources:
          type: array
          items:
            type: string
        suggested_actions:
          type: array
          items:
            type: object

    PageContext:
      type: object
      properties:
        page_path:
          type: string
        page_title:
          type: string
        chapter:
          type: string
        lesson:
          type: string
        section_id:
          type: string
        highlighted_text:
          type: string
```

**File**: `contracts/session-api.yaml`

```yaml
openapi: 3.0.0
info:
  title: TutorGPT Session API
  version: 1.0.0

paths:
  /session/start:
    post:
      summary: Create new learning session
      parameters:
        - name: student_id
          in: query
          schema:
            type: string
            default: "student_001"
      responses:
        '200':
          description: Session created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Session'

  /session/{session_id}:
    get:
      summary: Get session by ID
      parameters:
        - name: session_id
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Session details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Session'
        '404':
          description: Session not found

components:
  schemas:
    Session:
      type: object
      properties:
        session_id:
          type: string
        student_id:
          type: string
        created_at:
          type: string
          format: date-time
        updated_at:
          type: string
          format: date-time
        profile:
          $ref: '#/components/schemas/StudentProfile'
        progress:
          $ref: '#/components/schemas/Progress'

    StudentProfile:
      type: object
      properties:
        learning_pace:
          type: string
          enum: [slow, medium, fast]
        confused_topics:
          type: array
          items:
            type: string
        strong_topics:
          type: array
          items:
            type: string
        questions_asked:
          type: integer
        highlights_made:
          type: integer
        total_time_minutes:
          type: integer

    Progress:
      type: object
      properties:
        current_chapter:
          type: string
        current_lesson:
          type: string
        chapters_viewed:
          type: array
          items:
            type: string
        lessons_completed:
          type: array
          items:
            type: string
```

#### 3. Quickstart Guide (`quickstart.md`)

**Purpose**: Get developers up and running in <15 minutes

**Contents**:
1. Prerequisites (Python 3.11+, Node 18+, API keys)
2. Backend setup (clone, install, configure .env)
3. Book embedding (run `embed_book.py`)
4. Start backend (`uvicorn app.main:app`)
5. Frontend setup (Docusaurus, add ChatKit)
6. Test flow (create session, send message)
7. Common issues and solutions

---

## Architecture Decisions

### ADR-001: Use Gemini Embeddings Instead of OpenAI

**Context**: Need high-quality embeddings for semantic search in RAG system.

**Decision**: Use Google Gemini `models/embedding-001` for all book content embeddings.

**Rationale**:
- User-specified requirement
- Cost-effective compared to OpenAI embeddings
- High-quality semantic understanding
- Good integration with ChromaDB via custom embedding function

**Consequences**:
- Positive: Lower embedding costs, good quality
- Negative: Additional API dependency (Google AI), need custom ChromaDB integration
- Neutral: Requires research into Gemini embedding API patterns

**Alternatives Considered**:
- OpenAI `text-embedding-3-small`: Higher cost, but simpler integration with OpenAI ecosystem
- Open-source models (sentence-transformers): No API costs, but self-hosting complexity

**Research Update (2025-01-08)**:
- ✅ Confirmed model name: `gemini-embedding-001` (NOT `models/embedding-001`)
- ✅ SDK: Use `google-genai` (NEW SDK, GA Nov 2025) not deprecated `google-generativeai`
- ✅ Dimensions: 768 (recommended) or 3072 (max quality)
- ✅ Task types: `RETRIEVAL_DOCUMENT` for chunks, `RETRIEVAL_QUERY` for searches
- ✅ Batch API: Available for 50% cost reduction

### ADR-002: SQLite for MVP, PostgreSQL for Scale

**Context**: Need persistent storage for sessions and messages.

**Decision**: Use SQLite for MVP, design schema for easy migration to PostgreSQL in Phase 2.

**Rationale**:
- MVP scale (<100 concurrent users, 10k sessions) fits SQLite capabilities
- Zero deployment complexity (no separate DB server)
- Async support via `aiosqlite`
- Easy local development

**Consequences**:
- Positive: Simple deployment, fast local dev, no DB server costs
- Negative: Single-server limit, no horizontal scaling
- Mitigation: Schema designed to be PostgreSQL-compatible

**Migration Strategy**:
- Use standard SQL (avoid SQLite-specific features)
- All timestamps as ISO 8601 strings (portable)
- JSON columns supported by both SQLite and PostgreSQL

### ADR-003: OpenAI Agents SDK for Autonomous Behavior

**Context**: Need autonomous teaching agent that adapts in real-time.

**Decision**: Use OpenAI Agents SDK (https://openai.github.io/openai-agents-python/) for agent orchestration.

**Rationale**:
- Built for autonomous decision-making (not just prompt-response)
- Function calling for future extensibility (quizzes, practice, etc.)
- State management between turns
- Aligns with constitution requirement

**Consequences**:
- Positive: True autonomy, extensible, well-documented
- Negative: Dependency on OpenAI SDK evolution
- Neutral: Requires research into latest SDK patterns

**Research Update (2025-01-08)**:
- ✅ Package: `openai-agents` (install via `uv add openai-agents`)
- ✅ Agent creation: `Agent(name, instructions, tools=[])`
- ✅ Tools: Use `@function_tool` decorator
- ✅ State: SQLiteSession for persistence
- ✅ Streaming: Supported via `Runner.run(..., stream=True)`

### ADR-004: Skip LangChain for MVP

**Context**: Need RAG system for multi-level book content retrieval.

**Decision**: Use ChromaDB directly without LangChain wrapper for MVP.

**Rationale**:
- Simpler architecture with fewer dependencies
- More control over retrieval logic
- Better performance (no abstraction overhead)
- LangChain features (chains, complex orchestration) not needed for MVP
- Can add LangChain in Phase 2 if needed

**Consequences**:
- Positive: Simpler codebase, fewer dependencies, better performance
- Negative: Custom retrieval logic (not reusing LangChain patterns)
- Mitigation: Document retrieval patterns for future LangChain migration

**Implementation**:
- Direct ChromaDB queries with metadata filtering
- Custom multi-level retrieval function in FastAPI
- Gemini embeddings via custom embedding function

### ADR-005: ChatKit with Custom Backend via OpenAI API

**Context**: Need production-ready UI for chat widget embedded in Docusaurus.

**Decision**: Use OpenAI ChatKit with our custom backend via OpenAI Agents API.

**Rationale**:
- ChatKit provides production-ready UI (mobile, accessibility, streaming)
- No need to build custom chat interface (saves 1-2 weeks)
- ChatKit connects to OpenAI Agents API, which calls our tools
- Our tools (RAG search, profile) are implemented in FastAPI
- Best of both worlds: ChatKit UI + our custom backend logic

**Architecture**:
```
ChatKit (Frontend)
  → OpenAI Agents API (creates session with our agent config)
    → Our FastAPI Tools (search_book_content, get_student_profile)
      → ChromaDB, SQLite
```

**Consequences**:
- Positive: Production UI instantly, focus on intelligence not UI
- Negative: Depends on OpenAI ChatKit availability
- Neutral: Domain whitelisting required in OpenAI settings

**Research Update (2025-01-08)**:
- ✅ Package: `@openai/chatkit-react` v1.2.0
- ✅ Integration: React hook `useChatKit` + `<ChatKit />` component
- ✅ Backend: FastAPI creates ChatKit sessions via OpenAI API
- ✅ Context: Inject page/lesson/highlight via session creation

---

## Next Steps After Planning

**Phase 0**: ✅ COMPLETE - `research.md` filled with all official documentation findings
**Phase 1**: ✅ COMPLETE - Design documents created (`data-model.md`, `contracts/`, `quickstart.md`)

**Ready for Phase 2**: Task Generation

### Command to Run Next

```bash
/sp.tasks
```

This will generate testable, dependency-ordered implementation tasks from this plan.

### What Happens Next

1. **Task Generation** (`/sp.tasks`):
   - Generate `tasks.md` with ordered, testable tasks
   - Each task includes acceptance criteria and test cases
   - Tasks are dependency-ordered (DB → Embeddings → RAG → Agent → UI)

2. **Implementation** (`/sp.implement`):
   - Execute tasks in Week 1-4 schedule
   - Build backend (FastAPI, RAG, Agent)
   - Integrate ChatKit frontend
   - Test and deploy

### Phase 2 Preview: Implementation Phases

**Week 1: Foundation & Data Layer**
- SQLite database schema
- ChromaDB setup
- Book content chunking & embedding (Gemini)
- Basic FastAPI structure

**Week 2: RAG & Agent**
- Multi-level RAG retrieval
- OpenAI Agents SDK integration
- Agent tools (search_book_content, get_student_profile)
- API endpoints (ChatKit session, RAG search, profile)

**Week 3: ChatKit Integration**
- Docusaurus ChatKit widget
- Context capture (page, lesson, highlights)
- Session management
- End-to-end testing

**Week 4: Polish & Deploy**
- Performance optimization
- Error handling
- Deployment setup
- Documentation
- User testing

---

**Plan Complete** ✅

This implementation plan provides the architectural foundation for TutorGPT MVP with:
- ✅ **Gemini embeddings** (NEW `google-genai` SDK)
- ✅ **OpenAI Agents SDK** for autonomous teaching
- ✅ **ChatKit** for production UI
- ✅ **Direct ChromaDB** (LangChain skipped for simplicity)
- ✅ **SQLite** for simple deployment
- ✅ **Complete API contracts** and data models
- ✅ **UV package manager** setup guide

**Ready to generate tasks and start implementation!** 🚀

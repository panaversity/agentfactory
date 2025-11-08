---
id: 003
title: Create Implementation Plan with Gemini Embeddings
stage: plan
date: 2025-11-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-tutorgpt-mvp
branch: 001-tutorgpt-mvp
user: USER
command: /sp.plan
labels: ["plan", "architecture", "gemini-embeddings", "research", "tutorgpt"]
links:
  spec: specs/001-tutorgpt-mvp/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-tutorgpt-mvp/plan.md
  - specs/001-tutorgpt-mvp/research.md
tests:
  - Constitution alignment check completed
---

## Prompt

```
/sp.plan

User requirements:
- Use Gemini embedding model (gemini-embedding-001) instead of OpenAI embeddings for RAG
- Research OpenAI Agents SDK (https://openai.github.io/openai-agents-python/) before coding
- Research ChatKit (https://platform.openai.com/docs/guides/custom-chatkit) before coding
- Verify latest syntax for all technologies from official documentation
- Research-first approach: understand all tech before implementation
```

## Response snapshot

Created comprehensive implementation plan for TutorGPT MVP with the following key components:

### Architecture Decisions

**Embedding Model Change**:
- Switched from OpenAI `text-embedding-3-small` to **Google Gemini `models/embedding-001`**
- Created custom embedding function for ChromaDB integration
- Documented research requirements for Gemini API patterns

**Technology Stack**:
- Backend: FastAPI 0.104+ (Python 3.11+)
- Embeddings: Google Generative AI SDK with `models/embedding-001`
- Agent: OpenAI Agents SDK (requires research)
- UI: OpenAI ChatKit (requires research)
- RAG: LangChain + ChromaDB
- Database: SQLite (async via aiosqlite) - **changed from PostgreSQL for MVP simplicity**

**Project Structure**:
- Web application pattern (backend/ + book-source/)
- Backend: FastAPI with multi-level RAG (4 levels)
- Frontend: ChatKit widget integrated into existing Docusaurus site
- Clear separation: backend = intelligence, frontend = presentation

### Phase 0: Research Template Created

Created `research.md` with research tasks for:

1. **Google Gemini Embeddings**: API patterns, authentication, batch processing, integration with ChromaDB
2. **OpenAI Agents SDK**: Agent creation, function calling, context injection, state management
3. **OpenAI ChatKit**: CDN integration, custom backend connection, context capture, event listeners
4. **ChromaDB**: Custom embedding functions, metadata filtering, distance metrics
5. **LangChain RAG**: Multi-level retrieval patterns, custom embeddings integration

Each section includes:
- Official documentation links
- Research checklist
- Code snippet templates with [TO BE RESEARCHED] markers
- Specific questions to answer
- Integration patterns

### Constitution Alignment

**Passed all gates** ✅:
- Ship Fast Ship Well: ONE interface (ChatKit + book), 4-week timeline
- Context is Everything: Multi-level RAG, page context, highlight detection
- Agent is Autonomous: OpenAI Agents SDK planned
- Students Learn Better: Highlight → explanation, personalization
- Personalization Through Observation: Behavioral tracking, auto-adaptation
- Production Quality: <3 sec responses, all 107 pages, testing required

**Justified Deviation**:
- SQLite instead of PostgreSQL for MVP (simpler deployment, sufficient for <100 concurrent users)
- Migration path documented for Phase 2 scale-up

### Deliverables

**plan.md**:
- Complete technical context
- Constitution check with gate pass
- Project structure (web application pattern)
- Architecture Decision Records (ADRs)
- Clear next steps

**research.md**:
- Research task template for all technologies
- Official documentation links
- Code snippet templates
- Specific questions to answer before coding
- Research status tracking

## Outcome

- ✅ Impact: Complete implementation plan created with Gemini embeddings integration strategy
- 🧪 Tests: Constitution gates passed, justified deviations documented
- 📁 Files: Created plan.md and research.md in specs/001-tutorgpt-mvp/
- 🔁 Next prompts: Complete research.md by visiting all official docs, then create data-model.md, contracts/, quickstart.md
- 🧠 Reflection: Research-first approach ensures correct implementation patterns. Gemini embeddings require custom integration with ChromaDB but are well-supported. SQLite→PostgreSQL migration path provides scale-up option while keeping MVP simple.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan aligns with constitution and user requirements
- Graders run and results (PASS/FAIL): PASS - All constitution gates passed, research template comprehensive
- Prompt variant (if applicable): Standard planning workflow with custom embedding model requirement
- Next experiment (smallest change to try): After research completion, generate data-model.md and API contracts, then run /sp.tasks for implementation

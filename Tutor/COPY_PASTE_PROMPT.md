# 📋 Copy-Paste This Prompt to Any AI Assistant

---

Hi! I need your help analyzing and building **TutorGPT** - an autonomous AI tutor agent integrated into a Docusaurus book website.

## 🎯 What I Need From You

1. **Analyze** my entire repository and understand the project
2. **Review** all design documents and identify gaps or mistakes
3. **Give suggestions** for improvements
4. **Help me code** the implementation step-by-step

## 📂 Project Context

**Repository**: `https://github.com/MustafaAgentBuilder/ai-native-software-development`
**Project Directory**: `Tutor/`
**Goal**: Build an AI tutor that autonomously teaches students using the "AI-Native Software Development" book (107 lessons)

## 🧠 CRITICAL: Agent-First Architecture

This is **NOT a static RAG pipeline**. This is an **AUTONOMOUS AGENT** that makes intelligent teaching decisions.

### ❌ WRONG (Static Pipeline):
```
Frontend → API → RAG Search → Static Response
```

### ✅ CORRECT (Agent-First):
```
Frontend → TutorGPT AGENT (Brain) 🧠
              ↓
    Makes Autonomous Decisions
              ↓
    Chooses from 12 Tools:
    - search_book_content (searches book when needed)
    - explain_concept (simple/detailed/advanced explanations)
    - provide_code_example (shows code)
    - generate_quiz (tests understanding)
    - detect_confusion (monitors student struggle)
    - ask_clarifying_question (Socratic teaching)
    - get_student_profile (knows each student)
    - track_progress (remembers everything)
    - suggest_next_lesson (guides learning path)
    - celebrate_milestone (encourages student)
    - adjust_teaching_pace (adapts speed)
    - suggest_practice_exercise (hands-on learning)
              ↓
    Dynamic, Adaptive Teaching
```

**Agent Personality**: Encouraging Coach + Adaptive Mix
**Autonomy**: Starts reactive → becomes proactive as it learns the student

## 📚 Key Documents You MUST Read

Please read these files from my repository in this order:

1. **`Tutor/.specify/memory/constitution.md`** - Project principles & quality standards
2. **`Tutor/CLAUDE.md`** - Development process (Spec-Driven Development)
3. **`Tutor/specs/001-tutorgpt-mvp/spec.md`** - Requirements (4 user stories)
4. **`Tutor/specs/001-tutorgpt-mvp/research.md`** (47KB) - Tech research with official SDK patterns
5. **`Tutor/specs/001-tutorgpt-mvp/plan.md`** (33KB) - Complete architecture & system design
6. **`Tutor/specs/001-tutorgpt-mvp/tasks.md`** (33KB) - 205 implementation tasks
7. **`Tutor/specs/001-tutorgpt-mvp/data-model.md`** - Database schema & Pydantic models
8. **`Tutor/specs/001-tutorgpt-mvp/quickstart.md`** - Developer setup guide
9. **`Tutor/specs/001-tutorgpt-mvp/contracts/`** - API contracts
10. **`Tutor/history/prompts/001-tutorgpt-mvp/`** - Decision history (4 PHR files)

## 🔍 What to Analyze

### 1. Architecture Review
- Is the agent-first design sound?
- Are all 12 tools properly designed?
- Any circular dependencies?
- Database schema efficient?
- API contracts complete?
- Security vulnerabilities?
- Performance bottlenecks?

### 2. Technical Stack Validation
**Current Stack**:
- **Backend**: FastAPI + Python 3.11+ (UV package manager)
- **Agent**: OpenAI Agents SDK (autonomous decision-making)
- **Embeddings**: Google Gemini SDK (`gemini-embedding-001`)
- **Vector Store**: ChromaDB (direct, no LangChain) with cosine similarity
- **Database**: SQLite (MVP) → PostgreSQL (production)
- **Frontend**: Docusaurus + OpenAI ChatKit React widget
- **LLM**: GPT-4 Turbo

**Verify**:
- Are these technologies appropriate?
- Are SDK integrations correct?
- Are there deprecated APIs?
- Rate limits handled properly?
- Better alternatives available?

### 3. Implementation Plan (205 Tasks)
**Phase Breakdown**:
- Phase 1: Setup (5 tasks)
- **Phase 2: Agent Core (25 tasks)** - Build brain FIRST
- **Phase 3: Agent Tools (52 tasks)** - 12 capabilities
- Phase 4: Supporting Services (22 tasks)
- Phases 5-8: User Stories (73 tasks)
- Phase 9: Polish (28 tasks)

**Review**:
- Are tasks actionable and specific?
- Dependencies correct?
- Missing tasks?
- MVP scope (104 tasks) realistic?

### 4. Data Model & APIs
- Database schema complete?
- Proper indexes planned?
- API contracts RESTful?
- Authentication planned?
- Missing endpoints?

### 5. Agent Intelligence (Most Important!)
**Review Agent Core Tasks (T006-T030)**:
- Teaching philosophy defined?
- System prompt comprehensive?
- Decision-making logic sound?
- Context engineering properly designed?
- Agent autonomy correctly implemented?

### 6. Gaps & Missing Pieces
Identify what's missing:
- Configuration files structure
- Deployment strategy
- Monitoring/observability
- Error handling strategy
- Testing strategy
- CI/CD pipeline
- Documentation gaps
- Security concerns

## 📊 Project Stats

- **Total Tasks**: 205
- **MVP Tasks**: 104 (Phases 1-5)
- **Agent Tools**: 12 autonomous capabilities
- **Book Content**: 107 lessons to teach from
- **User Stories**: 4 (instant help, highlights, history persistence, adaptation)
- **Timeline**: 4 weeks (MVP → Production)

## 🎯 Your First Response Should Include

1. **Confirmation** you've accessed and read the key documents
2. **Summary** of what TutorGPT is in your own words
3. **Architecture Understanding** - explain the agent-first approach
4. **Analysis Report**:
   - ✅ What's done well
   - ⚠️ Concerns or gaps
   - 💡 Suggestions for improvement
   - 🚨 Critical issues to fix
5. **Security & Performance** feedback
6. **Missing Pieces** you identified
7. **Questions** for me before we code
8. **Recommended Starting Point** - which task to start with?

## ⚠️ Critical Rules

### DO:
✅ Build agent autonomy (not hardcoded logic)
✅ Follow architecture in plan.md exactly
✅ Use official SDK patterns from research.md
✅ Agent decides, tools execute
✅ Keep <3 second response times
✅ Protect student data privacy
✅ Make agent encouraging & adaptive
✅ Ask clarifying questions

### DON'T:
❌ Replace agent with static logic
❌ Use technologies outside approved stack
❌ Skip error handling
❌ Hardcode configuration
❌ Use deprecated APIs
❌ Make agent reactive-only (it should be proactive)
❌ Build static RAG pipeline
❌ Skip Agent Core phase (MUST be first!)

## 🚀 After Analysis - Let's Code!

Once you provide your analysis, we'll implement:

**Week 1**: Setup + **Agent Core** (T001-T030)
- Build TutorGPT brain with personality, teaching philosophy, decision-making

**Week 2**: **Agent Tools** + Supporting Services (T031-T104)
- Give brain 12 autonomous tools + backend infrastructure

**Week 2-3**: **User Story 1 MVP** (T105-T126)
- ChatKit integration + agent teaching students

**Week 3-4**: Full features + Polish
- Highlights, history, adaptation, production-ready

## 📖 Example Agent Behavior

**Scenario**: Student asks "What is async programming?"

**Agent's Autonomous Decision Process**:
1. Calls `get_student_profile()` → sees beginner level
2. Calls `search_book_content(query="async", scope="chapter")` → finds content
3. Decides to use `explain_concept(depth="simple", use_analogy=True)`
4. Responds: "Async is like cooking multiple dishes at once! 🍳 While pasta boils, you chop vegetables..."
5. Asks: "Would a code example help?"
6. If student confused 3x → calls `detect_confusion()` → simplifies approach

The agent **DECIDES** - not hardcoded!

## 🔗 Resources

- **GitHub**: https://github.com/MustafaAgentBuilder/ai-native-software-development
- **OpenAI Agents SDK**: https://openai.github.io/openai-agents-python/
- **Gemini Embeddings**: https://ai.google.dev/gemini-api/docs/embeddings
- **ChromaDB**: https://docs.trychroma.com/
- **ChatKit**: https://platform.openai.com/docs/guides/custom-chatkit

## ✨ Ready?

Please start by accessing my repository, reading the documents listed above, and providing your comprehensive analysis!

**Let's build an autonomous AI tutor together!** 🧠🔥

---

**Note**: If you can't access the repository directly, let me know and I'll share the key files with you.

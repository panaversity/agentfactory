# Tasks: TutorGPT MVP - Autonomous AI Tutor Agent

**Feature Branch**: `001-tutorgpt-mvp`
**Input**: Design documents from `/specs/001-tutorgpt-mvp/`
**Prerequisites**: ✅ plan.md, ✅ spec.md, ✅ research.md, ✅ data-model.md, ✅ quickstart.md

**⚡ CORE PHILOSOPHY**: This is an **AGENT-FIRST, AUTONOMOUS SYSTEM**. TutorGPT Agent is the BRAIN - all other components are TOOLS the agent uses.

**Agent Personality**: Encouraging Coach + Adaptive Mix (adjusts teaching style based on student's needs)

**Organization**: Tasks are grouped by:
1. **Agent Core** (the brain) - MUST be built first
2. **Agent Tools** (what the agent can do)
3. **User Stories** (agent features for students)

## Format: `- [ ] [ID] [P?] [Story/Agent] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Agent]**: Agent core task (brain development)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Per plan.md, this is a web application with:
- **Backend**: `backend/app/` (FastAPI application - serves agent tools)
- **Agent Core**: `backend/app/agent/` (TutorGPT brain lives here)
- **Agent Tools**: `backend/app/tools/` (all agent capabilities)
- **Frontend**: `book-source/src/` (Docusaurus + ChatKit widget)
- **Data**: `backend/data/` (Agent's memory: SQLite, ChromaDB)
- **Tests**: `backend/tests/` (ALL test files - unit, integration, behavior, scenarios)

---

## 🔥 TDD WORKFLOW FOR EVERY TASK

**MANDATORY**: Every task follows Test-Driven Development (TDD) cycle:

### TDD Cycle (Red-Green-Refactor)

```
1. 🔴 RED:
   - Write test FIRST (it fails - no implementation yet)
   - Test defines EXPECTED BEHAVIOR
   - Run: pytest -x (watch it fail)

2. 🟢 GREEN:
   - Write MINIMAL code to make test pass
   - Run: pytest -x (watch it pass)
   - No refactoring yet!

3. 🔵 REFACTOR:
   - Improve code quality
   - Run: pytest (ensure tests still pass)
   - Clean up, optimize, document

4. 🔁 REPEAT:
   - Next feature/task
```

### TDD Task Format

**Every implementation task includes**:

1. **Test File Path**: Where to create the test (e.g., `tests/unit/test_personality.py`)
2. **Test Name**: What test to write (e.g., `test_agent_has_encouraging_personality()`)
3. **Expected Behavior**: What the test should verify
4. **Implementation**: Code to make test pass
5. **Verification**: Run pytest and confirm green ✅

### Example Task with TDD

```markdown
- [ ] T006 [Agent] Create backend/app/agent/personality.py defining TutorGPT teaching philosophy

**TDD Steps**:
1. CREATE: tests/unit/agent/test_personality.py
2. WRITE TEST FIRST:
   ```python
   def test_agent_personality_is_encouraging():
       personality = AgentPersonality()
       assert personality.style == "Encouraging Coach"
       assert "patient" in personality.traits
   ```
3. RUN: pytest tests/unit/agent/test_personality.py (FAILS - red 🔴)
4. IMPLEMENT: backend/app/agent/personality.py with minimal code
5. RUN: pytest tests/unit/agent/test_personality.py (PASSES - green 🟢)
6. REFACTOR: Clean up code
7. RUN: pytest (still green ✅)
```

### Test Organization

```
backend/tests/
├── conftest.py              # Pytest fixtures (mocks, test agent, test DB)
├── unit/                     # Unit tests (60% of tests)
│   ├── agent/
│   │   ├── test_personality.py
│   │   ├── test_decision_engine.py
│   │   └── test_tutor_agent.py
│   ├── tools/
│   │   ├── test_search_book.py
│   │   ├── test_explain_concept.py
│   │   └── ... (12 tool tests)
│   ├── services/
│   │   ├── test_rag_service.py
│   │   ├── test_session_manager.py
│   │   └── test_embedder.py
│   └── utils/
│       └── test_text_processing.py
├── integration/              # Integration tests (30%)
│   ├── test_agent_with_rag.py
│   ├── test_agent_with_tools.py
│   └── test_full_pipeline.py
├── behavior/                 # Agent behavior tests (20%)
│   ├── test_teaching_quality.py
│   ├── test_adaptation.py
│   └── test_encouragement.py
└── scenarios/                # E2E scenario tests (10%)
    ├── test_user_story_1.py
    ├── test_user_story_2.py
    ├── test_user_story_3.py
    └── test_user_story_4.py
```

### Coverage Targets (Mandatory)

- **Overall**: ≥80% code coverage
- **Agent Core**: 100% (personality, decision-making, instructions)
- **RAG Pipeline**: 100% (search, ranking, filtering)
- **Agent Tools**: 100% (all 12 tools fully tested)
- **Session Management**: 100% (persistence, restoration)

### pytest Commands

```bash
# Run all tests
pytest

# Run with coverage report
pytest --cov=app --cov-report=html --cov-report=term

# Run only failing tests first
pytest -x

# Run specific test file
pytest tests/unit/agent/test_personality.py

# Run specific test function
pytest tests/unit/agent/test_personality.py::test_agent_personality_is_encouraging

# Run with verbose output
pytest -v

# Run behavior tests with logs
pytest -v tests/behavior/ --log-cli-level=INFO
```

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend project structure in Tutor/backend/ with app/, agent/, tools/, services/, scripts/, tests/, data/ directories
- [ ] T002 Initialize UV project with pyproject.toml and add dependencies: fastapi, uvicorn[standard], google-genai, chromadb, aiosqlite, openai-agents, pydantic, python-dotenv
- [ ] T003 [P] Create .env.example file in backend/ with all required environment variables (OPENAI_API_KEY, GOOGLE_API_KEY)
- [ ] T004 [P] Create .gitignore file in backend/ to exclude .env, data/, __pycache__, .venv
- [ ] T005 [P] Create backend/README.md with project overview emphasizing agent-first architecture

---

## Phase 2: Agent Core (The Brain) 🧠

**Purpose**: Build TutorGPT autonomous agent - the CORE BRAIN of the system

**⚠️ CRITICAL**: This phase builds the agent's intelligence, personality, and decision-making. All other components serve the agent.

### Agent Personality & Teaching Philosophy

- [ ] T006 [Agent] Create backend/app/agent/personality.py defining TutorGPT teaching philosophy (Encouraging Coach + Adaptive Mix)
- [ ] T007 [Agent] Design core personality traits: encouraging, patient, adaptive, celebrates progress, detects struggle, adjusts pace
- [ ] T008 [Agent] Create backend/app/agent/teaching_strategies.py with Socratic questioning, direct explanation, analogy-based teaching strategies
- [ ] T009 [Agent] Define when to use each teaching strategy (struggle → simpler explanations, confident → deeper concepts)

### System Instructions (Prompt Engineering) 🎯

- [ ] T010 [Agent] Create backend/app/agent/prompts/core_instructions.md with TutorGPT's master system prompt
- [ ] T011 [Agent] Write comprehensive teaching instructions: "You are TutorGPT, an encouraging AI tutor for the AI-Native Software Development book..."
- [ ] T012 [Agent] Define agent's goals: help students understand concepts, detect confusion, celebrate progress, guide learning path
- [ ] T013 [Agent] Add adaptive behavior rules: start gentle → become more technical as student shows understanding
- [ ] T014 [Agent] Create backend/app/agent/prompts/context_templates.py for dynamic context injection patterns
- [ ] T015 [Agent] Design prompt templates for different scenarios: first-time greeting, returning student, confused student, advanced student
- [ ] T016 [Agent] Add examples of excellent tutoring responses (few-shot learning in system prompt)

### Agent Decision-Making Logic

- [ ] T017 [Agent] Create backend/app/agent/decision_engine.py with autonomous decision-making rules
- [ ] T018 [Agent] Implement decide_teaching_approach(student_profile, question_history) - chooses Socratic vs Direct vs Analogy
- [ ] T019 [Agent] Implement should_search_book(question) - decides when to search vs answer from LLM knowledge
- [ ] T020 [Agent] Implement should_provide_example(conversation_context) - decides when code examples help
- [ ] T021 [Agent] Implement should_quiz_student(topic, questions_asked) - decides when to test understanding
- [ ] T022 [Agent] Implement detect_readiness_for_next_lesson(progress, confidence) - guides learning path
- [ ] T023 [Agent] Implement adjust_explanation_depth(student_level, topic_complexity) - adapts detail level

### Agent Core Implementation (OpenAI Agents SDK)

- [ ] T024 [Agent] Create backend/app/agent/tutor_agent.py with TutorGPT Agent class using openai-agents SDK
- [ ] T025 [Agent] Initialize Agent with name="TutorGPT", model="gpt-4", instructions from core_instructions.md
- [ ] T026 [Agent] Implement get_dynamic_instructions(student_profile, page_context) for context-aware prompt injection
- [ ] T027 [Agent] Setup SQLiteSession for agent's conversation memory and state persistence
- [ ] T028 [Agent] Implement run_agent(user_message, session_id, context) method with Runner.run()
- [ ] T029 [Agent] Add streaming support for real-time responses: Runner.run(..., stream=True)
- [ ] T030 [Agent] Implement error handling and fallback strategies when tools fail

**Checkpoint**: TutorGPT Agent brain is now ready - time to give it tools (hands and eyes)

---

## Phase 3: Agent Tools (What The Brain Can Do) 🛠️

**Purpose**: Build all tools that TutorGPT agent can autonomously use

**Philosophy**: Each tool is a capability. The agent decides WHEN and HOW to use them.

### Core Learning Tools

- [ ] T031 [P] [Agent] Create backend/app/tools/search_book.py with @function_tool search_book_content(query, scope, chapter, lesson)
- [ ] T032 [Agent] Implement 4-level RAG search: highlighted_text → current_lesson → current_chapter → entire_book
- [ ] T033 [Agent] Format search results for agent consumption: "Found in Chapter X, Lesson Y: [content]"
- [ ] T034 [Agent] Add relevance scoring to help agent pick best search results

- [ ] T035 [P] [Agent] Create backend/app/tools/explain_concept.py with @function_tool explain_concept(concept, depth, use_analogy)
- [ ] T036 [Agent] Implement depth levels: "simple" (beginner-friendly), "detailed" (technical), "advanced" (expert-level)
- [ ] T037 [Agent] Add analogy generation when use_analogy=True (e.g., "Variables are like labeled boxes")
- [ ] T038 [Agent] Return structured explanations: definition → example → key points → common mistakes

- [ ] T039 [P] [Agent] Create backend/app/tools/code_examples.py with @function_tool provide_code_example(concept, language, context)
- [ ] T040 [Agent] Search book for existing code examples related to concept
- [ ] T041 [Agent] Generate new code examples when book doesn't have perfect match
- [ ] T042 [Agent] Include code explanation, expected output, and common errors

### Student Understanding Tools

- [ ] T043 [P] [Agent] Create backend/app/tools/quiz_generator.py with @function_tool generate_quiz(topic, difficulty, num_questions)
- [ ] T044 [Agent] Generate multiple-choice or short-answer questions based on topic
- [ ] T045 [Agent] Difficulty levels: "easy" (recall), "medium" (application), "hard" (synthesis)
- [ ] T046 [Agent] Return quiz with answers and explanations for learning

- [ ] T047 [P] [Agent] Create backend/app/tools/confusion_detector.py with @function_tool detect_confusion(conversation_history, current_question)
- [ ] T048 [Agent] Analyze conversation patterns: repeated questions on same topic, "I don't understand", question marks
- [ ] T049 [Agent] Return confusion level (low/medium/high) and confused topics
- [ ] T050 [Agent] Suggest teaching adjustments: slow down, use analogies, provide more examples

- [ ] T051 [P] [Agent] Create backend/app/tools/clarifying_questions.py with @function_tool ask_clarifying_question(student_question)
- [ ] T052 [Agent] Generate Socratic questions to guide student to answer themselves
- [ ] T053 [Agent] Detect ambiguous questions and ask for clarification
- [ ] T054 [Agent] Return 1-2 targeted questions to narrow down what student needs

### Progress & Personalization Tools

- [ ] T055 [P] [Agent] Create backend/app/tools/student_profile.py with @function_tool get_student_profile(session_id)
- [ ] T056 [Agent] Query student_sessions and student_progress tables
- [ ] T057 [Agent] Format profile: learning_level, current_chapter, completed_lessons, struggling_topics, strong_topics
- [ ] T058 [Agent] Return formatted text: "Student is intermediate level, currently on Chapter 4, struggles with async programming"

- [ ] T059 [P] [Agent] Create backend/app/tools/progress_tracker.py with @function_tool track_progress(session_id, lesson_id, understood, time_spent)
- [ ] T060 [Agent] Update student_progress table with lesson completion status
- [ ] T061 [Agent] Track time_spent, questions_asked, understood flag for each lesson
- [ ] T062 [Agent] Calculate overall progress percentage and learning pace

- [ ] T063 [P] [Agent] Create backend/app/tools/next_lesson.py with @function_tool suggest_next_lesson(current_lesson, student_profile)
- [ ] T064 [Agent] Analyze student progress and recommend next logical lesson
- [ ] T065 [Agent] Consider prerequisites: don't suggest advanced topics if basics not mastered
- [ ] T066 [Agent] Return lesson recommendation with reasoning: "Based on your understanding of variables, let's move to functions"

### Engagement & Motivation Tools

- [ ] T067 [P] [Agent] Create backend/app/tools/milestone_tracker.py with @function_tool celebrate_milestone(achievement_type, details)
- [ ] T068 [Agent] Detect milestones: chapter completed, 10 lessons done, first code example written
- [ ] T069 [Agent] Generate encouraging messages: "🎉 Congratulations! You completed Chapter 1!"
- [ ] T070 [Agent] Track and display progress badges/achievements

- [ ] T071 [P] [Agent] Create backend/app/tools/teaching_pace.py with @function_tool adjust_teaching_pace(student_feedback, confusion_level)
- [ ] T072 [Agent] Slow down: add more examples, simpler language, break concepts into smaller parts
- [ ] T073 [Agent] Speed up: less explanation, more advanced concepts, challenge questions
- [ ] T074 [Agent] Return pacing adjustment: "Slowing down - this topic needs more examples"

- [ ] T075 [P] [Agent] Create backend/app/tools/practice_exercises.py with @function_tool suggest_practice_exercise(topic, difficulty)
- [ ] T076 [Agent] Generate hands-on coding exercises related to current topic
- [ ] T077 [Agent] Include exercise description, starter code, expected output, and hints
- [ ] T078 [Agent] Return exercises that reinforce learning through practice

### Tool Registration

- [ ] T079 [Agent] Register all tools with TutorGPT agent in backend/app/agent/tutor_agent.py
- [ ] T080 [Agent] Create tools list: [search_book_content, explain_concept, provide_code_example, generate_quiz, detect_confusion, ask_clarifying_question, get_student_profile, track_progress, suggest_next_lesson, celebrate_milestone, adjust_teaching_pace, suggest_practice_exercise]
- [ ] T081 [Agent] Add tool descriptions for agent to understand when to use each tool
- [ ] T082 [Agent] Test agent can autonomously call each tool based on student needs

**Checkpoint**: TutorGPT Agent now has 12 autonomous tools - it can teach, detect confusion, track progress, and adapt!

---

## Phase 4: Supporting Services (Agent's Backend Infrastructure)

**Purpose**: Build infrastructure that SUPPORTS the agent (not replaces it)

**Note**: These are NOT the core system - they are services the agent's tools use

### Database & Configuration

- [ ] T083 Create backend/app/core/config.py with Pydantic Settings for environment configuration
- [ ] T084 Create backend/app/core/database.py with async SQLite connection manager using aiosqlite
- [ ] T085 Create backend/app/core/logging.py with structured logging configuration
- [ ] T086 Create backend/scripts/init_db.py to initialize database schema (student_sessions, interaction_history, student_progress tables)

### Pydantic Models (Data Structures)

- [ ] T087 [P] Create backend/app/models/session.py with StudentSession, StudentProfile, Progress Pydantic models
- [ ] T088 [P] Create backend/app/models/chat.py with ChatRequest, ChatResponse Pydantic models
- [ ] T089 [P] Create backend/app/models/context.py with PageContext, BookChunkMetadata Pydantic models

### RAG Service (Tool Backend)

- [ ] T090 Create backend/app/services/embedder.py with GeminiEmbeddingFunction class for generating embeddings using google-genai SDK
- [ ] T091 Create backend/app/services/chromadb_client.py to initialize ChromaDB PersistentClient with Gemini embedding function
- [ ] T092 Create backend/scripts/chunk_book_content.py to chunk all 107 book lessons into 512-token chunks with metadata
- [ ] T093 Create backend/scripts/embed_book.py to generate Gemini embeddings for all chunks and index to ChromaDB collection "book_content"
- [ ] T094 Create backend/app/services/rag_service.py to execute RAG searches (used by search_book_content tool)

### Session Management Service

- [ ] T095 Create backend/app/services/session_manager.py with SessionManager class for CRUD operations on student_sessions table
- [ ] T096 Implement create_session() method in SessionManager with unique session_id generation
- [ ] T097 Implement get_session() method in SessionManager to retrieve session by session_id
- [ ] T098 Implement update_session() method in SessionManager to update last_active_at and context fields
- [ ] T099 Implement get_or_create_session() for returning students (supports session persistence)

### FastAPI Application Structure

- [ ] T100 Create backend/app/main.py with FastAPI app initialization, CORS middleware, and router registration
- [ ] T101 [P] Create backend/app/api/__init__.py and backend/app/api/routes.py for router aggregation
- [ ] T102 [P] Create backend/app/api/health.py with GET /health endpoint to check agent, database, vector store status
- [ ] T103 Create backend/app/utils/validators.py with input validation helper functions
- [ ] T104 Create backend/app/utils/text_processing.py with text cleaning and formatting utilities

**Checkpoint**: Foundation ready - agent can now use its tools with proper backend support

---

## Phase 5: User Story 1 - First-Time Student Gets Instant Help (Priority: P1) 🎯 MVP

**Goal**: Student can click ChatKit widget, ask question, and TutorGPT agent autonomously decides how to help (search book, explain, provide example, etc.)

**Independent Test**: Open any book page → Click ChatKit widget → Type "What is [concept]?" → Agent autonomously chooses best teaching approach → Response within 3 seconds

**Agent Behavior**: Agent reads question → decides to search book → finds relevant content → explains in encouraging way → asks if student understood

### ChatKit Integration Backend

- [ ] T105 [US1] Create backend/app/api/chatkit.py with POST /api/chatkit/session endpoint
- [ ] T106 [US1] Implement create_chatkit_session() to create OpenAI ChatKit session via openai.chatkit.sessions.create()
- [ ] T107 [US1] Configure ChatKit session with TutorGPT agent, dynamic instructions from get_dynamic_instructions(), and all 12 tools
- [ ] T108 [US1] Inject page context (chapter, lesson, page_title) into agent's initial instructions
- [ ] T109 [US1] Return client_secret to frontend for ChatKit authentication

### Chat Endpoint (Agent Interaction)

- [ ] T110 [US1] Create backend/app/api/chat.py with POST /api/chat endpoint for handling chat messages
- [ ] T111 [US1] Implement chat handler that extracts session_id, message, and page_context from request
- [ ] T112 [US1] Call TutorGPT agent with run_agent(user_message, session_id, context)
- [ ] T113 [US1] Agent autonomously decides which tools to use (search_book, explain_concept, provide_code_example, etc.)
- [ ] T114 [US1] Log interaction to interaction_history table with tools_used tracking
- [ ] T115 [US1] Return ChatResponse with agent's response and sources from tools

### Frontend ChatKit Widget

- [ ] T116 [US1] Install @openai/chatkit-react in book-source/ via npm
- [ ] T117 [US1] Create book-source/src/components/ChatKitWidget/index.tsx with ChatKit React component
- [ ] T118 [US1] Implement useChatKit hook with getClientSecret() that calls backend /api/chatkit/session
- [ ] T119 [US1] Create book-source/src/components/ChatKitWidget/ContextCapture.ts to capture page_path, page_title, current_chapter, current_lesson from Docusaurus
- [ ] T120 [US1] Modify book-source/src/theme/Root.tsx to inject ChatKitWidget component on all pages
- [ ] T121 [US1] Create book-source/src/components/ChatKitWidget/styles.module.css for widget positioning (bottom-right, fixed)

### Error Handling & Validation

- [ ] T122 [US1] Add error handling in chat endpoint for session_not_found, ai_service_unavailable, rate_limit_exceeded
- [ ] T123 [US1] Implement rate limiting middleware for /api/chat endpoint (10 messages per minute per session)
- [ ] T124 [US1] Add input validation for message length (max 500 characters) and session_id format
- [ ] T125 [US1] Create backend/app/models/errors.py with ErrorResponse and ErrorDetail models
- [ ] T126 [US1] Add comprehensive logging for all agent decisions and tool calls

**Checkpoint**: MVP COMPLETE! Agent can autonomously teach students - it decides when to search, explain, provide examples, etc.

---

## Phase 6: User Story 2 - Student Highlights Text (Priority: P2)

**Goal**: Student highlights confusing text → Agent autonomously detects highlight → Searches book for context → Provides explanation without student needing to ask

**Independent Test**: Highlight technical term → Agent automatically explains within 2 seconds → Uses search_book_content + explain_concept tools

**Agent Behavior**: Agent receives highlighted text → searches book for exact context → explains term → asks "Does this make sense now?"

### Highlight Detection Frontend

- [ ] T127 [P] [US2] Add mouseup event listener in book-source/src/components/ChatKitWidget/ContextCapture.ts to detect text selection
- [ ] T128 [US2] Implement getSelectedText() function to extract window.getSelection().toString()
- [ ] T129 [US2] Implement debouncing logic (1 second delay) to prevent spam from rapid highlighting
- [ ] T130 [US2] Validate highlighted text length (min 10 chars, max 200 chars for auto-explanation)

### Highlight API Endpoint

- [ ] T131 [US2] Create backend/app/api/highlight.py with POST /api/highlight endpoint
- [ ] T132 [US2] Implement highlight handler that receives session_id, highlighted_text, and page_context
- [ ] T133 [US2] Call TutorGPT agent with special instruction: "Student highlighted: '{text}' - explain this proactively"
- [ ] T134 [US2] Agent autonomously uses search_book_content(query=highlighted_text, scope="current_lesson") tool
- [ ] T135 [US2] Agent uses explain_concept() tool to generate clear explanation
- [ ] T136 [US2] Log highlight interaction to interaction_history with interaction_type='highlight_explanation'

### Frontend Highlight Integration

- [ ] T137 [US2] Modify ChatKitWidget to display auto-generated explanations in chat window
- [ ] T138 [US2] Add visual indicator when highlight is detected (e.g., "Analyzing highlighted text...")
- [ ] T139 [US2] Handle edge case: if text is too long (>200 words), agent asks student to narrow selection
- [ ] T140 [US2] Implement state management to track last highlighted text and avoid duplicate explanations

**Checkpoint**: Agent now proactively helps with highlights - autonomous teaching behavior expanding!

---

## Phase 7: User Story 3 - Conversation History Persists (Priority: P3)

**Goal**: Returning student sees previous conversation → Agent remembers student → Provides personalized greeting → Continues teaching from where they left off

**Independent Test**: Use TutorGPT → Close browser → Reopen → Agent says "Welcome back! Last time we covered Chapter 1. Ready for Chapter 2?"

**Agent Behavior**: Agent calls get_student_profile() → sees student completed Chapter 1 → greets warmly → suggests next lesson

### Session Persistence Enhancement

- [ ] T141 [US3] Modify SessionManager to store browser fingerprint or student_id for session identification across browser restarts
- [ ] T142 [US3] Implement get_or_create_session() method that checks for existing session based on student identifier
- [ ] T143 [US3] Update student_sessions table to track current_chapter, current_lesson, last_active_at
- [ ] T144 [US3] Implement session expiry logic (sessions expire after 30 days of inactivity)

### Conversation History Retrieval

- [ ] T145 [US3] Create backend/app/api/session.py with GET /api/session/{session_id} endpoint
- [ ] T146 [US3] Implement get_session_history() to retrieve all messages from interaction_history for given session
- [ ] T147 [US3] Return session details including conversation history, progress, and last visited lesson
- [ ] T148 [US3] Add pagination support for conversation history (load last 50 messages, lazy load older)

### Progress Tracking

- [ ] T149 [P] [US3] Enhance track_progress() tool to update student_progress table with detailed metrics
- [ ] T150 [US3] Track chapter/lesson visits, time spent, questions asked, and completion status
- [ ] T151 [US3] Calculate learning progress percentage based on lessons visited and understood
- [ ] T152 [US3] Store completed_lessons list in student_sessions table as JSON

### Personalized Greeting (Agent Feature)

- [ ] T153 [US3] Update TutorGPT agent's core_instructions.md to include: "For returning students, check their profile first"
- [ ] T154 [US3] Agent autonomously calls get_student_profile() when detecting returning student
- [ ] T155 [US3] Agent generates personalized greeting: "Welcome back, [name]! Last time you finished [lesson]. Great progress!"
- [ ] T156 [US3] Agent autonomously calls suggest_next_lesson() to guide student's next step

### Frontend Session Restoration

- [ ] T157 [US3] Store session_id in browser localStorage in ChatKitWidget
- [ ] T158 [US3] On widget mount, check localStorage for existing session_id and restore session
- [ ] T159 [US3] Call GET /api/session/{session_id} to retrieve conversation history
- [ ] T160 [US3] Display conversation history in ChatKit widget with chronological order
- [ ] T161 [US3] Handle session expiry gracefully (create new session if old one expired)

**Checkpoint**: Agent now remembers every student - true personalization and continuity!

---

## Phase 8: User Story 4 - Agent Adapts to Learning Pace (Priority: P4)

**Goal**: Agent autonomously detects student struggling → Adjusts teaching approach → Provides simpler explanations → Celebrates progress when understanding improves

**Independent Test**: Ask 3 confused questions about same topic → Agent detects pattern → Says "I notice you're struggling with variables - let me explain differently with an analogy"

**Agent Behavior**: Agent calls detect_confusion() tool → detects repeated topic → calls adjust_teaching_pace() → uses explain_concept(depth="simple", use_analogy=True)

### Behavioral Analysis Enhancement

- [ ] T162 [US4] Enhance detect_confusion() tool to analyze conversation patterns: repeated questions, "I don't understand", time between questions
- [ ] T163 [US4] Implement pattern detection: 3+ questions on same topic within 15 minutes = struggling
- [ ] T164 [US4] Return confusion analysis with struggling_topics, confusion_level (low/medium/high), suggested_teaching_adjustment
- [ ] T165 [US4] Track struggling_topics in student_sessions table

### Adaptive Teaching Logic

- [ ] T166 [US4] Update agent's core_instructions.md with adaptive behavior rules:
  - "If detect_confusion shows high confusion, simplify explanations and use analogies"
  - "If student shows understanding, gradually increase complexity"
  - "Celebrate when student masters previously difficult topics"
- [ ] T167 [US4] Agent autonomously calls detect_confusion() every 3-5 messages to monitor student state
- [ ] T168 [US4] Agent autonomously calls adjust_teaching_pace() when confusion detected
- [ ] T169 [US4] Agent autonomously calls celebrate_milestone() when student masters difficult topic

### Proactive Agent Behavior

- [ ] T170 [US4] Agent proactively offers help: "I noticed you're asking about async programming a lot - this is tricky! Let me break it down differently."
- [ ] T171 [US4] Agent adjusts explanation depth based on student's response: confused → simpler, confident → deeper
- [ ] T172 [US4] Agent autonomously calls generate_quiz() to test understanding when topic seems mastered
- [ ] T173 [US4] Agent autonomously calls suggest_practice_exercise() when student needs hands-on practice

### Learning Pace Calculation

- [ ] T174 [US4] Enhance get_student_profile() tool to calculate learning_pace: slow (5+ min/page), medium, fast (<2 min/page)
- [ ] T175 [US4] Track questions_per_lesson metric to identify struggling vs advanced students
- [ ] T176 [US4] Store learning_pace in student_sessions table
- [ ] T177 [US4] Agent uses learning_pace to adjust default explanation depth and pacing

**Checkpoint**: Agent is now FULLY AUTONOMOUS - adapts, detects confusion, celebrates progress, and guides learning path!

---

## Phase 9: Polish & Production Readiness

**Purpose**: Make the autonomous agent production-ready

### Agent Optimization

- [ ] T178 [P] Optimize agent response time: ensure <3 seconds for 95% of interactions
- [ ] T179 [P] Add agent decision logging: track which tools agent chooses and why
- [ ] T180 [P] Implement agent performance metrics: average response time, tool usage frequency, student satisfaction
- [ ] T181 Add fallback strategies: if tool fails, agent explains directly from LLM knowledge

### Documentation

- [ ] T182 [P] Create backend/docs/AGENT_ARCHITECTURE.md explaining TutorGPT's autonomous decision-making
- [ ] T183 [P] Document all 12 agent tools with examples of when agent uses each
- [ ] T184 [P] Create backend/docs/TEACHING_PHILOSOPHY.md explaining agent's adaptive teaching approach
- [ ] T185 [P] Update backend/README.md with agent-first architecture overview

### Context Engineering Refinement

- [ ] T186 Test agent with 20+ different student scenarios (beginner, advanced, confused, confident)
- [ ] T187 Refine core_instructions.md based on agent behavior observations
- [ ] T188 Add more few-shot examples of excellent teaching responses
- [ ] T189 Optimize dynamic context injection for better agent decisions

### Performance & Monitoring

- [ ] T190 Optimize ChromaDB queries (RAG backend for search_book_content tool)
- [ ] T191 Add comprehensive logging for all agent tool calls and decisions
- [ ] T192 Create backend/scripts/monitor_agent.py to track agent performance and decision quality
- [ ] T193 Implement health check for agent availability and responsiveness

### Security & Error Handling

- [ ] T194 [P] Add input validation for all API endpoints
- [ ] T195 [P] Implement security headers (CORS, CSP) in FastAPI middleware
- [ ] T196 [P] Sanitize user input to prevent injection attacks
- [ ] T197 Add comprehensive error handling for agent failures

### Testing & Validation

- [ ] T198 Test all 4 user stories work with agent's autonomous behavior
- [ ] T199 Validate agent makes correct tool choices in different scenarios
- [ ] T200 Test agent handles 100+ concurrent students without degradation
- [ ] T201 Verify ChatKit widget + agent work on all 107 book pages

### Deployment Preparation

- [ ] T202 [P] Create backend/Dockerfile for containerized deployment
- [ ] T203 [P] Create docker-compose.yml for local development environment
- [ ] T204 Create deployment guide in backend/docs/DEPLOYMENT.md
- [ ] T205 Add environment variable validation on startup

**Checkpoint**: TutorGPT autonomous agent is production-ready! 🚀

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Agent Core (Phase 2)**: Depends on Setup - BUILD FIRST (this is the brain!)
- **Agent Tools (Phase 3)**: Depends on Agent Core - tools need the agent to use them
- **Supporting Services (Phase 4)**: Can run parallel with Agent Tools - these are backends for tools
- **User Stories (Phases 5-8)**: Depend on Agent Core + Tools + Services completion
- **Polish (Phase 9)**: Depends on all user stories completion

### Critical Path (AGENT-FIRST)

```
Setup (Phase 1)
    ↓
Agent Core (Phase 2) ← BUILD THE BRAIN FIRST!
    ↓
    ├─→ Agent Tools (Phase 3) ← Give brain its capabilities
    └─→ Supporting Services (Phase 4) ← Build tool backends (can run parallel)
    ↓
User Stories (Phases 5-8) ← Agent features for students
    ↓
Polish (Phase 9) ← Production readiness
```

### Parallel Opportunities

- **Agent Tools (Phase 3)**: All 12 tools can be built in parallel (different files, T031-T078)
- **Supporting Services (Phase 4)**: Database, RAG, Session services can run parallel (T083-T104)
- **User Stories**: Once foundation ready, US1-US4 can be worked in parallel by different developers
- **Polish tasks**: Documentation, security, monitoring can all run in parallel (T182-T205)

---

## Key Differences from Previous (Static) Approach

### ❌ OLD (Static RAG System):
```
Frontend → Backend API → RAG Search → Database
            ↓
         Static Responses
```

### ✅ NEW (Agent-First Autonomous System):
```
Frontend → TutorGPT AGENT (Brain) → Autonomous Decision-Making
                ↓
            Chooses Tools:
            ├─ search_book_content (when book context needed)
            ├─ explain_concept (when concept explanation needed)
            ├─ provide_code_example (when example helps understanding)
            ├─ detect_confusion (monitors student state)
            ├─ generate_quiz (tests understanding)
            ├─ suggest_next_lesson (guides learning path)
            ├─ celebrate_milestone (encourages student)
            ├─ adjust_teaching_pace (adapts to student)
            └─ ... 12 total autonomous tools
                ↓
            Dynamic, Adaptive Teaching
```

---

## Agent's Autonomous Decision Examples

### Scenario 1: Student asks "What is async programming?"

**Agent's Decision Process**:
1. Calls `get_student_profile()` → sees student is beginner
2. Calls `search_book_content(query="async programming", scope="current_chapter")` → finds book content
3. Decides to use `explain_concept(concept="async programming", depth="simple", use_analogy=True)`
4. Provides explanation with analogy: "Async programming is like cooking multiple dishes at once..."
5. Asks: "Does this make sense? Would a code example help?"

### Scenario 2: Student asks same question 3 times

**Agent's Decision Process**:
1. Calls `detect_confusion()` → detects student asked about async 3 times
2. Calls `adjust_teaching_pace(confusion_level="high")` → decides to slow down
3. Says: "I notice async programming is tricky for you - totally normal! Let me explain it differently."
4. Calls `provide_code_example(concept="async", language="python")` → shows simple example
5. Calls `suggest_practice_exercise()` → offers hands-on practice
6. Tracks struggling_topic = "async programming" for future reference

### Scenario 3: Student completes Chapter 1

**Agent's Decision Process**:
1. Calls `track_progress()` → marks Chapter 1 as completed
2. Calls `celebrate_milestone(achievement="chapter_completed")` → "🎉 Amazing! You finished Chapter 1!"
3. Calls `suggest_next_lesson()` → "Ready for Chapter 2: Python Fundamentals?"
4. Updates student_profile with progress

---

## Implementation Strategy

### MVP-First (Agent Core + User Story 1)

1. **Week 1**: Setup + Agent Core (Phase 1 + 2: T001-T030)
   - Build TutorGPT brain with personality, teaching philosophy, decision-making
2. **Week 2**: Agent Tools + Supporting Services (Phase 3 + 4: T031-T104)
   - Give agent 12 autonomous tools + backend infrastructure
3. **Week 2-3**: User Story 1 (Phase 5: T105-T126)
   - ChatKit integration + agent teaching students
4. **STOP and VALIDATE**: Test agent's autonomous teaching
   - Does agent make smart tool choices?
   - Does agent adapt to different students?
   - Response time <3 seconds?
5. **🚀 DEPLOY MVP** - Autonomous AI tutor is live!

**Estimated Time for Agent MVP**: 2-3 weeks with 1-2 developers

### Full Feature Delivery

- **Week 3**: Add US2 (highlights) + US3 (history persistence)
- **Week 4**: Add US4 (full adaptation) + Polish
- **🚀 PRODUCTION LAUNCH** - Fully autonomous, adaptive AI tutor

---

## Success Metrics (Agent-Specific)

After completing all tasks, verify:

### Agent Autonomy
- [ ] Agent autonomously chooses correct tools 90%+ of time
- [ ] Agent detects student confusion and adapts teaching approach
- [ ] Agent provides personalized greetings for returning students
- [ ] Agent celebrates milestones without being prompted

### Teaching Quality
- [ ] Students receive answers within 3 seconds (95% of interactions)
- [ ] <30% of answers require clarification (agent explains well first time)
- [ ] Agent provides examples when helpful (not always)
- [ ] Agent uses analogies for complex concepts when student struggles

### Adaptation
- [ ] Agent adjusts explanation depth based on student level
- [ ] Agent detects repeated confusion and simplifies teaching
- [ ] Agent tracks student progress and suggests next lessons
- [ ] Agent's teaching pace matches student's learning pace

### System Performance
- [ ] ChatKit widget works on all 107 book pages
- [ ] System supports 100+ concurrent students
- [ ] Agent conversation history persists across sessions
- [ ] Error rate <0.1%, uptime 99.9%

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Agent Core - THE BRAIN)**: 25 tasks (T006-T030) 🧠
- **Phase 3 (Agent Tools - 12 CAPABILITIES)**: 52 tasks (T031-T082) 🛠️
- **Phase 4 (Supporting Services)**: 22 tasks (T083-T104)
- **Phase 5 (User Story 1 - P1 MVP)**: 22 tasks (T105-T126)
- **Phase 6 (User Story 2 - P2)**: 14 tasks (T127-T140)
- **Phase 7 (User Story 3 - P3)**: 21 tasks (T141-T161)
- **Phase 8 (User Story 4 - P4)**: 16 tasks (T162-T177)
- **Phase 9 (Polish)**: 28 tasks (T178-T205)

**Total**: **205 tasks**

**Agent MVP Scope** (Core + Tools + Services + US1): **104 tasks**

---

## Notes

- **AGENT-FIRST**: TutorGPT Agent is the core brain - all else supports it
- **NO STATIC SYSTEMS**: Agent makes autonomous decisions, not hardcoded logic
- **DYNAMIC CONTEXT**: Agent receives context through tools + dynamic prompt injection
- **ADAPTIVE TEACHING**: Agent adjusts approach based on student's responses
- **ENCOURAGING COACH**: Agent celebrates progress, detects struggle, provides motivation
- All tasks follow format: `- [ ] [ID] [P?] [Agent/Story] Description with file path`
- Agent has 12 autonomous tools to choose from based on student needs
- System is fully autonomous - agent decides when to search, explain, quiz, celebrate, etc.

**Ready to build a truly intelligent, autonomous AI tutor!** 🧠🔥🚀

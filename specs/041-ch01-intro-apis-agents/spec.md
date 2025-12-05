# Feature Specification: Chapter 1 - Introduction to AI APIs & OpenAI Agents

**Feature Branch**: `041-ch01-intro-apis-agents`
**Created**: 2025-12-05
**Status**: Draft
**Input**: User description: "Chapter 1: Introduction to AI APIs & OpenAI Agents - 4 lessons covering API fundamentals, agent vs LLM distinctions, environment setup, and DocuBot architecture overview"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Complete Foundational API Understanding (Priority: P1)

**Learning Journey**: As a beginner with no AI development experience, I need to understand what an API is using simple analogies and verify that my OpenAI API key works, so that I have confidence my development environment is properly configured before learning complex concepts.

**Why this priority**: This is the absolute foundation. Without understanding APIs and having a working API key, students cannot progress to any subsequent lessons. This is the "hello world" moment that proves their setup works.

**Independent Test**: Can be fully tested by having the student run `test_api.py` and receive a response from OpenAI. Success means they see AI-generated text printed to their terminal, proving API connectivity.

**Acceptance Scenarios**:

1. **Given** a student with an OpenAI API key, **When** they create a `.env` file with `OPENAI_API_KEY=their-key`, **Then** the environment variable is accessible to Python code
2. **Given** a student runs `test_api.py` with valid credentials, **When** the script executes `client.chat.completions.create()`, **Then** they receive a text response from OpenAI within 5 seconds
3. **Given** a student with an invalid API key, **When** they run `test_api.py`, **Then** they see a clear error message indicating authentication failure (not a cryptic stack trace)
4. **Given** a student reads the restaurant analogy, **When** asked to explain APIs in their own words, **Then** they can articulate that APIs are communication bridges between their code and external services

---

### User Story 2 - Distinguish LLM vs Agent Mental Models (Priority: P2)

**Learning Journey**: As a student who has used ChatGPT before, I need to understand how agents differ from simple LLMs, so that I know why we're building DocuBot as an agent rather than just using a chatbot API.

**Why this priority**: This conceptual foundation is critical before writing agent code. Students need to understand the "why" behind agents (memory, tools, actions) to make sense of the architectural decisions in later chapters.

**Independent Test**: Can be fully tested by reviewing the student's comparison table. Success means the table clearly shows that agents have capabilities (memory, tools, actions) that LLMs lack.

**Acceptance Scenarios**:

1. **Given** a student completes the comparison table exercise, **When** they list ChatGPT capabilities, **Then** they identify at least 3 limitations (no memory, no tools, text-only responses)
2. **Given** a student completes the comparison table exercise, **When** they list DocuBot Agent capabilities, **Then** they identify at least 3 advantages (document search, conversation memory, source citations)
3. **Given** a student with the completed comparison table, **When** asked "why does DocuBot need to be an agent?", **Then** they can explain that searching documents and remembering context requires agent capabilities beyond simple text generation
4. **Given** a student reads the "smart person in a room" analogy, **When** asked to explain the difference, **Then** they can articulate that agents have "tools" to interact with the world, not just conversation ability

---

### User Story 3 - Set Up Complete Development Environment (Priority: P1)

**Learning Journey**: As a student new to Python development, I need step-by-step instructions to install UV, create a project folder, configure environment variables, and verify everything works, so that I have a stable foundation for all future coding exercises.

**Why this priority**: Tied with P1 because environment setup is a blocker. Students cannot write code in Chapter 2 if their environment isn't configured. This must work before proceeding.

**Independent Test**: Can be fully tested by checking for: (1) UV installed and accessible via `uv --version`, (2) Project folder with `pyproject.toml` file, (3) `.env` file with API key, (4) Successfully running `uv add openai` without errors.

**Acceptance Scenarios**:

1. **Given** a student follows the UV installation instructions, **When** they run `uv --version` in their terminal, **Then** they see a version number (e.g., "uv 0.4.x") confirming successful installation
2. **Given** a student runs `uv init` in the `docubot` folder, **When** the command completes, **Then** they see `pyproject.toml` and `uv.lock` files created
3. **Given** a student creates a `.env` file with their API key, **When** they run Python code that loads environment variables, **Then** the API key is accessible via `os.getenv("OPENAI_API_KEY")`
4. **Given** a student runs `uv add openai`, **When** the installation completes, **Then** they can import the OpenAI library without errors: `from openai import OpenAI`
5. **Given** a student completes environment setup, **When** they run the `test_api.py` script from Lesson 1.1, **Then** they receive a response from OpenAI, confirming end-to-end setup

---

### User Story 4 - Visualize DocuBot Architecture Roadmap (Priority: P3)

**Learning Journey**: As a student about to embark on a 16-chapter course, I need to see the complete system architecture and understand which chapter builds which component, so that I have a clear mental map and can see how each lesson contributes to the final project.

**Why this priority**: While important for context and motivation, this is a "nice to have" that doesn't block progress. Students can proceed to Chapter 2 without a perfect architecture diagram as long as they understand the basics.

**Independent Test**: Can be fully tested by reviewing the student's `architecture.md` file. Success means the file contains: (1) system component diagram, (2) component descriptions, (3) chapter roadmap showing which chapter builds what.

**Acceptance Scenarios**:

1. **Given** a student creates `architecture.md`, **When** they document the system overview, **Then** they include at least 5 core components: User, Frontend (ChatKit), Backend (FastAPI), Agent (OpenAI SDK), Vector Database
2. **Given** a student creates the component descriptions, **When** they explain each component, **Then** they write 1-2 sentences describing what each component does without implementation details
3. **Given** a student creates the chapter roadmap, **When** they map chapters to components, **Then** they show that Ch 1-3 focus on Agent basics, Ch 4-9 on advanced agent features, Ch 10-11 on RAG pipeline, Ch 12-13 on frontend, Ch 14-16 on production deployment
4. **Given** a student completes the architecture document, **When** asked "what will you build in this course?", **Then** they can describe the end-to-end system: "a RAG chatbot that searches documents and provides cited answers via a web interface"

---

### Edge Cases

- **What happens when a student has network connectivity issues?** Lesson 1.1's API test will fail with a timeout or connection error. Provide troubleshooting steps: check internet connection, verify firewall settings, try alternate DNS.
- **What happens when a student uses an expired or invalid API key?** The API test returns a 401 authentication error. Provide clear instructions: check the API key is correctly copied, verify it's not expired in OpenAI dashboard, ensure no extra spaces in `.env` file.
- **What happens when a student is on Windows and the terminal commands differ?** Provide OS-specific installation instructions for UV (PowerShell vs bash), and note differences in environment variable handling (`.env` file still works cross-platform with `python-dotenv`).
- **What happens when a student's Python version is too old?** UV requires Python 3.8+. Provide installation links for Python 3.11+ (recommended version), and instructions to verify version with `python --version`.
- **What happens when a student cannot install UV due to corporate firewall?** Provide alternative installation via pip: `pip install uv`, and note that while less ideal, it still enables course progression.

## Requirements *(mandatory)*

### Functional Requirements

#### Lesson 1.1 Requirements (API Fundamentals)

- **FR-001**: Lesson content MUST explain what an API is using the restaurant analogy (order → waiter → kitchen → response)
- **FR-002**: Lesson content MUST cover these four key points: APIs connect software, requests/responses work like messaging, API keys authenticate users, JSON is the data format
- **FR-003**: Lesson MUST include "🤖 Apply to DocuBot Project" section with:
  - **Task** subsection: "Test your OpenAI API key with a simple API call. Create test_api.py with just: 1) Import OpenAI, 2) Create client, 3) Make a simple chat completion call, 4) Print the response."
  - **Outcome** subsection: "*Confirmed working API key. You see a response from OpenAI, proving your setup is correct.*"
- **FR-004**: Lesson MUST provide a minimal `test_api.py` starter code (3-5 lines) that students can complete to verify their API key
- **FR-005**: The `test_api.py` script MUST: import OpenAI, create a client, make a chat completion call, print the response text
- **FR-006**: Lesson MUST include 5 progressive hints that guide students toward solution without giving complete answer
- **FR-007**: Starter code MUST be provided after hints using format: `from openai import OpenAI\n\nclient = OpenAI()\n\n# Your code here: make a chat completion call`

#### Lesson 1.2 Requirements (LLM vs Agent Distinction)

- **FR-008**: Lesson content MUST explain the difference between LLMs (text generation only) and agents (LLM + tools + memory + actions)
- **FR-009**: Lesson content MUST use the "smart person in a room" analogy: LLM = talks through slot, Agent = has phone, computer, can leave room
- **FR-010**: Lesson MUST include "🤖 Apply to DocuBot Project" section with:
  - **Task** subsection: "Make a comparison table with two columns: 'What ChatGPT Can Do' vs 'What DocuBot Agent Will Do'. List at least 5 items in each column."
  - **Outcome** subsection: "*A clear comparison showing why DocuBot needs to be an agent (search documents, remember context, format citations) not just a chatbot.*"
- **FR-011**: Comparison table MUST have at least 5 items per column showing concrete capability differences
- **FR-012**: Lesson MUST include hints listing example capabilities for both sides (e.g., ChatGPT: answer general questions; DocuBot: search uploaded documents)
- **FR-013**: Hints MUST provide examples: ChatGPT column ("Answer general questions", "Write text", "Explain concepts") and DocuBot column ("Search my uploaded documents", "Remember our conversation", "Cite which document the answer came from")

#### Lesson 1.3 Requirements (Environment Setup)

- **FR-014**: Lesson MUST provide installation instructions for UV package manager (cross-platform: Mac, Linux, Windows)
- **FR-015**: Installation instructions MUST include OS-specific commands: `curl -LsSf https://astral.sh/uv/install.sh | sh` for Mac/Linux, PowerShell equivalent for Windows
- **FR-016**: Lesson MUST include "🤖 Apply to DocuBot Project" section with:
  - **Task** subsection: "Follow the setup guide to: 1) Install UV, 2) Create docubot project folder, 3) Initialize UV project, 4) Create .env file with your OpenAI API key, 5) Verify everything works with a simple test."
  - **Outcome** subsection: "*A fully configured project folder with UV, virtual environment, and .env file ready.*"
- **FR-017**: `.env` file setup MUST show the exact format: `OPENAI_API_KEY=sk-...` with explanation that this keeps secrets out of code
- **FR-018**: Lesson MUST include verification step: running `uv add openai` successfully and importing the library without errors
- **FR-019**: Hints MUST include commands: "Install UV: curl -LsSf https://astral.sh/uv/install.sh | sh", "Create folder: mkdir docubot && cd docubot", "Initialize: uv init", "Create .env: touch .env then add OPENAI_API_KEY=your-key-here", "Install OpenAI: uv add openai", "Test: run test_api.py from Lesson 1.1"

#### Lesson 1.4 Requirements (Architecture Overview)

- **FR-020**: Lesson MUST explain the 4 main components of DocuBot: Agent (brain), RAG Pipeline (document search), Backend API (FastAPI), Frontend UI (ChatKit)
- **FR-021**: Lesson MUST use the "house blueprint" analogy: architecture diagram is the blueprint before building
- **FR-022**: Lesson MUST include "🤖 Apply to DocuBot Project" section with:
  - **Task** subsection: "Create an architecture.md file in your project that includes: 1) System overview diagram, 2) List of components with brief descriptions, 3) Which chapter builds which component."
  - **Outcome** subsection: "*A documented architecture that serves as your roadmap for the entire course.*"
- **FR-023**: `architecture.md` MUST include 3 sections: System Overview (component diagram), Component Descriptions (what each does), Chapter Roadmap (which chapter builds what)
- **FR-024**: Component diagram MUST show the flow: User → Frontend → Backend → Agent → Vector DB
- **FR-025**: Chapter roadmap MUST map chapters to components: Ch 1-3 (Agent basics), Ch 4-9 (Advanced features), Ch 10-11 (RAG), Ch 12-13 (Frontend), Ch 14-16 (Production)
- **FR-026**: Hints MUST include: "Start with simple box diagram: User → Frontend → Backend → Agent → Vector DB", "Components to list: Agent (brain), Tools (abilities), RAG Pipeline (document search), FastAPI (backend), ChatKit (frontend)", "Use markdown headers: ## System Overview, ## Components, ## Chapter Roadmap", "For roadmap: Ch 1-3: Agent basics, Ch 4-9: Advanced features, Ch 10-11: RAG, Ch 12-13: Frontend, Ch 14-16: Production"

### Key Entities

- **Lesson**: Represents a single learning unit within the chapter
  - Attributes: title, duration (minutes), learning goal, key concepts, analogy, code level, task description, hints, outcome
  - Relationships: belongs to Chapter, has prerequisite knowledge from previous lessons

- **DocuBot Project**: The cumulative course project built across all chapters
  - Attributes: project folder structure, configuration files (.env, pyproject.toml), architecture documentation
  - State after Chapter 1: Folder created, environment configured, architecture documented, no code yet
  - Relationships: progresses through chapters, each chapter adds components

- **API Key**: OpenAI authentication credential
  - Attributes: secret string starting with `sk-`, associated with OpenAI account, has usage limits
  - Storage: `.env` file (never committed to git)
  - Relationships: required by OpenAI client, verified in Lesson 1.1

- **Development Environment**: The complete toolchain setup
  - Components: UV package manager, Python 3.8+, virtual environment, VS Code (recommended editor)
  - Configuration: `.env` for secrets, `pyproject.toml` for dependencies, `uv.lock` for reproducibility
  - Relationships: prerequisite for all coding exercises in subsequent chapters

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can explain what an API is using the restaurant analogy (assessed via comprehension question: "Explain APIs to a friend who doesn't code")
- **SC-002**: 95% of students successfully run `test_api.py` and see an AI response on first attempt (tracked via lesson completion metrics)
- **SC-003**: Students complete the LLM vs Agent comparison table with at least 5 items per column showing correct understanding of capability differences
- **SC-004**: Students successfully install UV and create project structure in under 30 minutes (average time tracked across cohort)
- **SC-005**: 90% of students complete environment setup without instructor intervention (tracked via support ticket volume)
- **SC-006**: Students create `architecture.md` file with all 3 required sections (system overview, components, roadmap) present and accurately documented
- **SC-007**: Chapter completion time falls within 2-3 hour range for 80% of students (tracked via lesson analytics)
- **SC-008**: Zero code written yet in Chapter 1 - students build mental models first (verified via lesson content audit: no Python agent code in Ch 1 lessons)
- **SC-009**: Students can articulate the course end goal: "I'm building DocuBot, a RAG chatbot that searches documents and provides cited answers" (assessed via end-of-chapter reflection prompt)
- **SC-010**: 100% of students have working API key and configured environment before proceeding to Chapter 2 (enforced via automated environment check at chapter boundary)

## Constraints *(mandatory)*

### Technical Constraints

- **TC-001**: Chapter 1 lessons MUST NOT include agent code (no `Agent` class, no `Runner`, no tools) - purely conceptual with minimal API verification code
- **TC-002**: Code examples in Lesson 1.1 MUST be minimal (3-5 lines maximum) to avoid overwhelming beginners
- **TC-003**: All terminal commands MUST be cross-platform compatible (provide OS-specific instructions where necessary)
- **TC-004**: Environment setup MUST use UV package manager (not pip or poetry) to align with course standards
- **TC-005**: API calls in Lesson 1.1 MUST use `gpt-4o-mini` model to minimize costs for students testing their setup

### Pedagogical Constraints

- **PC-001**: Each lesson MUST focus on ONE main concept (Lesson 1.1: APIs, 1.2: Agents, 1.3: Setup, 1.4: Architecture)
- **PC-002**: Cognitive load MUST remain LOW throughout Chapter 1 (pure concepts, analogies, minimal code)
- **PC-003**: Every lesson MUST include a clear analogy (restaurant, smart person in room, chef's station, house blueprint)
- **PC-004**: Lesson 1.2 MUST be code-free to allow mental model formation without syntax distraction
- **PC-005**: Lessons MUST follow Concept → Apply separation: teach the idea first, then apply to DocuBot project
- **PC-006**: Each lesson MUST include 4-6 progressive hints that scaffold learning without giving answers
- **PC-007**: Lesson duration MUST NOT exceed 50 minutes (attention span constraint)

### Content Structure Constraints

- **CSC-001**: Every lesson MUST have these sections in order: Learning Goal, Concept (What You'll Learn), Key Points, Simple Analogy, Code Level, Apply to DocuBot Project, Outcome, Hints
- **CSC-002**: "Apply to DocuBot Project" section MUST have this exact structure:
  - **Task**: Numbered step-by-step instructions describing what to build/create for DocuBot (e.g., "1) Import OpenAI, 2) Create client, 3) Make chat completion call")
  - **Outcome**: Italicized statement of what students achieve (e.g., "*Confirmed working API key. You see a response from OpenAI.*")
  - Both Task and Outcome must be present in EVERY lesson's DocuBot section
- **CSC-003**: Hints MUST be numbered 1-6 and progressively reveal more information (start vague, get specific)
- **CSC-004**: Outcomes MUST state concrete deliverable (e.g., "Confirmed working API key", "Comparison table with 5 items per column")
- **CSC-005**: "Apply to DocuBot Project" sections across all lessons MUST show cumulative progress toward complete DocuBot system (each lesson adds to previous state)
- **CSC-006**: Chapter MUST end with summary recap restating learning objectives and DocuBot project state
- **CSC-007**: Optional starter code MAY be provided after hints using "📝 Starter Code:" heading with code block (for lessons with coding tasks)

### Dependency Constraints

- **DC-001**: Chapter 1 has NO prerequisites (assumes zero programming knowledge)
- **DC-002**: Students MUST complete Lesson 1.1 (API key verification) before Lesson 1.3 (environment setup) to ensure credentials exist
- **DC-003**: Lesson 1.3 MUST be completed before Chapter 2 begins (environment is prerequisite for writing code)
- **DC-004**: Chapter 1 completion is MANDATORY before Chapter 2 (enforced via course platform gating)

## Non-Goals *(mandatory)*

### Explicitly Excluded from Chapter 1

- **NG-001**: Teaching Python syntax (variables, functions, loops) - deferred to Chapter 2+ coding lessons
- **NG-002**: Implementing the Agent class or Runner - covered in Chapter 2
- **NG-003**: Teaching tools, function calling, or agent capabilities - deferred to Chapters 4-9
- **NG-004**: Implementing RAG pipeline or vector database integration - deferred to Chapters 10-11
- **NG-005**: Building frontend UI or API endpoints - deferred to Chapters 12-13
- **NG-006**: Production deployment, monitoring, or scaling - deferred to Chapters 14-16
- **NG-007**: Deep dive into OpenAI API parameters (temperature, max_tokens, etc.) - introduced gradually in later chapters
- **NG-008**: Git version control or collaborative development workflows - assumed prerequisite or separate tutorial
- **NG-009**: Debugging techniques beyond basic print statements - formal debugging covered in Chapter 3
- **NG-010**: Cost optimization or API usage monitoring - covered in production chapters (14-16)

### Scope Boundaries

- **SB-001**: Chapter 1 is ONLY for building mental models and environment setup - not for writing agent logic
- **SB-002**: API key verification code (Lesson 1.1) is the ONLY Python code students write in Chapter 1
- **SB-003**: Lesson 1.2 comparison table is the ONLY formal deliverable documenting conceptual understanding
- **SB-004**: Architecture diagram (Lesson 1.4) is a TEXT document (Markdown), not a visual design tool output (Figma, Miro)

## Assumptions *(mandatory)*

### Student Background Assumptions

- **AS-001**: Students have basic computer literacy (can navigate files/folders, use terminal/command prompt)
- **AS-002**: Students have an OpenAI account with API access and at least $5 credit (or free tier access)
- **AS-003**: Students have internet access to download UV, install packages, and make API calls
- **AS-004**: Students can read and follow English technical documentation (CEFR B1+ level)
- **AS-005**: Students have a code editor installed (VS Code recommended but not required)

### Technical Environment Assumptions

- **AE-001**: Students are on a modern OS: macOS 10.15+, Windows 10+, or Linux (Ubuntu 20.04+)
- **AE-002**: Students have Python 3.8 or higher installed (3.11+ recommended)
- **AE-003**: Students have write permissions in their user directory to create project folders
- **AE-004**: Students' network allows HTTPS requests to OpenAI API (no corporate firewall blocking api.openai.com)
- **AE-005**: Students have at least 1GB free disk space for UV, dependencies, and project files

### Course Design Assumptions

- **AD-001**: Chapter 1 is the FIRST chapter in the course - students have read no prior chapters
- **AD-002**: Course platform provides lesson content rendering (Markdown → HTML) with code syntax highlighting
- **AD-003**: Course platform tracks lesson completion and enforces sequential progression (cannot skip to Chapter 2 without completing Chapter 1)
- **AD-004**: Instructor or teaching assistant is available for troubleshooting environment setup issues (not self-paced asynchronous)
- **AD-005**: Chapter 1 is delivered in a 2-3 hour workshop format (not spread across multiple days)

### Content Format Assumptions

- **AF-001**: Lessons are delivered as Markdown files rendered in a web-based course platform
- **AF-002**: Code blocks use syntax highlighting for Python and bash/shell commands
- **AF-003**: Analogies are presented as blockquotes or highlighted callout boxes for visual emphasis
- **AF-004**: Hints are collapsible/expandable so students can progressively reveal them (UI feature assumption)
- **AF-005**: Lesson outcomes are displayed as checkboxes or completion indicators (platform feature)

## Open Questions *(if any)*

### Clarifications Needed

None - the course specification from `AI-Native-Course-Spec-v2 (2).md` provides complete detail for Chapter 1. All four lessons have clear learning goals, analogies, tasks, hints, and outcomes documented. The chapter structure is well-defined and ready for implementation.

## Dependencies & Related Features

### Prerequisites (Must Complete Before This)

- **None** - Chapter 1 is the course entry point. Students need only basic computer literacy and an OpenAI API key (external setup assumed).

### Downstream Dependencies (Blocks These Features)

- **Chapter 2: Your First Agent** - Cannot proceed until students have working environment setup (Lesson 1.3) and understand API/Agent concepts (Lessons 1.1-1.2)
- **Chapter 3: Debugging Agents** - Requires Chapter 2's agent code as debugging subject
- **All subsequent chapters (4-16)** - Sequential course design means Chapter 1 completion gates all future content

### Related Features (Shared Context)

- **Course Platform Gating** - Chapter 1 completion status must be tracked to enforce sequential progression
- **Environment Verification Tool** - Automated check before Chapter 2 that verifies UV, OpenAI package, and API key are configured
- **Troubleshooting Guide** - Separate resource for common environment setup issues (network, permissions, Python version)

## Acceptance Tests *(mandatory)*

### Test 1: Lesson 1.1 API Verification

**Given** a student with a valid OpenAI API key
**When** they follow Lesson 1.1 instructions to create and run `test_api.py`
**Then**:
- The script executes without errors
- An AI-generated text response prints to the terminal
- The response is relevant to the prompt (e.g., "Hello, I'm testing my API key!" → coherent greeting response)
- Student sees confirmation message: "✓ API key verified successfully"

### Test 2: Lesson 1.2 Conceptual Understanding

**Given** a student completes the LLM vs Agent comparison table
**When** the table is reviewed for correctness
**Then**:
- "ChatGPT" column lists at least 5 capabilities showing limitations (text-only, no memory, no tools)
- "DocuBot Agent" column lists at least 5 capabilities showing agent features (search documents, remember context, cite sources)
- No capabilities are duplicated between columns (shows understanding of distinction)
- Student can verbally explain: "Agents have tools to take actions, LLMs only generate text"

### Test 3: Lesson 1.3 Environment Setup Completeness

**Given** a student follows all environment setup steps
**When** the setup is verified via automated checks
**Then**:
- Running `uv --version` returns a version number (UV installed)
- Running `python --version` returns 3.8+ (compatible Python)
- The `docubot` folder exists with `pyproject.toml` and `uv.lock` files
- The `.env` file exists and contains `OPENAI_API_KEY=sk-...` (API key configured)
- Running `uv add openai` completes successfully (dependency installation works)
- Running `from openai import OpenAI` in Python REPL succeeds (package importable)

### Test 4: Lesson 1.4 Architecture Documentation

**Given** a student creates `architecture.md`
**When** the document is reviewed for completeness
**Then**:
- Section 1 exists: "System Overview" with component flow diagram (text or visual)
- Section 2 exists: "Component Descriptions" with at least 5 components listed (Agent, RAG, Backend, Frontend, Vector DB)
- Section 3 exists: "Chapter Roadmap" mapping chapters to components (e.g., "Ch 1-3: Agent basics")
- Each component has 1-2 sentence description without implementation details (no code, no tech stack specifics)
- Student can verbally explain the end-to-end flow: "User asks question → Frontend → Backend → Agent searches documents → Returns cited answer"

### Test 5: DocuBot Project Section Format Compliance

**Given** all 4 lessons are implemented
**When** lesson content is audited for DocuBot project sections
**Then**:
- EVERY lesson contains "🤖 Apply to DocuBot Project" section
- Each DocuBot section has **Task** subsection with numbered steps (e.g., "1) Do X, 2) Do Y")
- Each DocuBot section has **Outcome** subsection in italics (e.g., "*Student achieves concrete result*")
- Lesson 1.1: Task creates `test_api.py`, Outcome confirms working API key
- Lesson 1.2: Task creates comparison table, Outcome explains why agent is needed
- Lesson 1.3: Task sets up environment with UV, Outcome shows configured project folder
- Lesson 1.4: Task creates `architecture.md`, Outcome documents course roadmap
- All outcomes show cumulative progress toward complete DocuBot system

### Test 6: Chapter 1 Completion Criteria

**Given** a student completes all 4 lessons
**When** chapter completion is assessed
**Then**:
- All 4 lesson outcomes are marked complete (verified API key, comparison table, environment setup, architecture doc)
- Total time spent is 2-3 hours (tracked via platform analytics)
- Student has NOT written any agent code yet (verified via project folder audit - only `test_api.py` exists)
- Student can answer: "What will you build in this course?" with "DocuBot RAG chatbot that searches documents"
- Student scores 80%+ on end-of-chapter quiz covering API concepts, LLM vs Agent distinction, and architecture components

## Additional Context

### Pedagogical Strategy for Chapter 1

This chapter deliberately avoids code to prevent cognitive overload. The restaurant/smart person/chef/house analogies are carefully chosen to map technical concepts to familiar experiences:

- **Restaurant analogy (APIs)**: Maps request/response cycle to ordering food (universal experience)
- **Smart person in room (LLM vs Agent)**: Maps tool access to physical world constraints (intuitive)
- **Chef's station (Environment setup)**: Maps code organization to kitchen prep (process-oriented)
- **House blueprint (Architecture)**: Maps system design to construction planning (big-picture thinking)

These analogies will be referenced in later chapters as anchoring points when introducing complex technical details.

### Cross-Chapter Integration

Chapter 1 sets up state that persists through the entire course:

- **DocuBot project folder**: All code from Chapters 2-16 lives here
- **Environment configuration**: UV and `.env` file used in every subsequent chapter
- **Architecture document**: Referenced in later chapters as components are implemented
- **Mental models**: API/Agent concepts underpin all agent development lessons

This foundational chapter is critical because mistakes here cascade through 15 subsequent chapters. Hence the emphasis on verification (Test 3) and clear outcomes (Test 5).

### Implementation Notes for Course Creators

When implementing this chapter:

1. **Provide pre-configured Docker image** (optional): Students struggling with UV installation can use a containerized environment
2. **Create environment verification script**: Automated check that runs all Test 3 validations and outputs a report
3. **Offer office hours after Chapter 1**: Environment issues are the #1 blocker for beginners - schedule instructor availability
4. **Track completion funnel**: Measure drop-off at each lesson to identify friction points (e.g., if 50% fail at Lesson 1.3, improve setup instructions)

---

**End of Specification**

**Next Steps**: Submit this spec for review via `/sp.clarify` to identify ambiguities, then proceed to `/sp.plan` for implementation planning.

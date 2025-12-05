---
id: 0002
title: ch01-intro-apis-agents-plan
stage: plan
date: 2025-12-06
surface: claude-code-cli
model: claude-sonnet-4-5
feature: 041-ch01-intro-apis-agents
branch: 041-ch01-intro-apis-agents
user: HP
command: /sp.plan 041-ch01-intro-apis-agents
labels: [plan, educational-content, chapter-1, pedagogical-design]
links:
  spec: specs/041-ch01-intro-apis-agents/spec.md
  plan: specs/041-ch01-intro-apis-agents/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/041-ch01-intro-apis-agents/plan.md
    action: created
    lines: 1678
tests: []
---

## Prompt

**User Input**: "yes invoke" (approval to invoke /sp.plan)

**Command Invoked**: `/sp.plan 041-ch01-intro-apis-agents`

**Context**:
- Specification completed and validated (9.8/10 readiness score)
- Chapter 1 structure: 4 lessons, 2-3 hours, LOW cognitive load
- Work type: CONTENT (educational chapter creation)
- Target audience: Complete beginners with zero programming knowledge
- Project: DocuBot RAG chatbot built incrementally across 16 chapters

**Routing Decision**:
- Classified as CONTENT work (spec mentions: lesson, module, learning objectives, pedagogical, students, proficiency)
- Routed to `chapter-planner` subagent for educational content planning
- Provided context: spec path, constitution reference, pedagogical constraints, DocuBot project requirements

**Planning Requirements Specified**:
1. Pedagogical arc planning (Foundation → Application pattern)
2. Layer progression mapping (all lessons are L1 - Manual Foundation)
3. Teaching modality selection (vary across lessons: analogy, hands-on, documentation, visualization)
4. Hardware tier requirements (all Tier 1 - conceptual content, no special hardware)
5. Intelligence creation opportunities (assess if skills/subagents needed)
6. DocuBot project integration (Task + Outcome in every lesson)

## Response snapshot

**chapter-planner agent created comprehensive implementation plan**:

**Plan Structure** (1,678 lines in plan.md):

**I. Chapter Overview & Pedagogical Analysis**
- Chapter positioned as foundational entry point (no prerequisites)
- 4 lessons covering: API fundamentals, LLM vs Agent, environment setup, architecture
- Total duration: 2-3 hours (135-175 minutes)
- Cognitive load: LOW (3-4 concepts per lesson, A1-A2 proficiency)
- Hardware tier: All Tier 1 (cross-platform, no GPU/robot required)

**II. Pedagogical Layer Mapping**
- All 4 lessons use Layer 1 (Manual Foundation)
- Rationale: Students must build mental models before AI collaboration in Chapter 2
- Teaching modalities varied: Analogy-driven (L1.1) → Comparison-driven (L1.2) → Step-by-step hands-on (L1.3) → Visualization (L1.4)
- No agent code allowed (TC-001 constraint enforced)

**III. Lesson-by-Lesson Implementation Plan**

**Lesson 1.1: API Fundamentals (30 min)**
- Learning goal: Understand APIs via restaurant analogy, verify OpenAI API key works
- Key concepts: Request/response cycle, API keys as authentication, JSON data format
- Analogy: Restaurant (you → waiter → kitchen → response)
- DocuBot task: Create test_api.py (3-5 lines) to verify API key
- Expected outcome: "*Confirmed working API key. You see a response from OpenAI.*"
- Hints: 5 progressive (OPENAI_API_KEY in .env → import OpenAI → create client → chat.completions.create → response.choices[0].message.content)
- Word count: ~2000 words
- Time breakdown: Concept (10 min) + DocuBot task (15 min) + Troubleshooting (5 min)

**Lesson 1.2: LLM vs Agent (35 min)**
- Learning goal: Distinguish static LLMs from dynamic agents with tools/memory
- Key concepts: LLM = text generation, Agent = LLM + tools + memory + actions, agent loop
- Analogy: Smart person in room (LLM talks through slot vs Agent has phone/computer)
- DocuBot task: Create comparison table (2 columns, 5+ items each: "What ChatGPT Can Do" vs "What DocuBot Agent Will Do")
- Expected outcome: "*Clear comparison showing why DocuBot needs to be an agent (search documents, remember context, format citations).*"
- Hints: 4 progressive (ChatGPT limitations → DocuBot needs → example capabilities → why agent is necessary)
- Word count: ~1800 words
- Time breakdown: Concept (15 min) + DocuBot task (15 min) + Reflection (5 min)

**Lesson 1.3: Environment Setup (40-45 min)**
- Learning goal: Set up complete development environment (UV, Python, VS Code, .env)
- Key concepts: UV package manager, virtual environments, environment variables, folder structure
- Analogy: Chef's station (organize before cooking)
- DocuBot task: Install UV → Create docubot folder → Initialize UV project → Create .env with API key → Install openai package → Verify
- Expected outcome: "*Fully configured project folder with UV, virtual environment, and .env file ready.*"
- Hints: 6 progressive (UV install command → mkdir docubot → uv init → .env format → uv add openai → test with Lesson 1.1 script)
- Word count: ~2500 words (longest lesson due to cross-platform instructions)
- Time breakdown: Installation (10 min) + Project setup (15 min) + Verification (10 min) + Troubleshooting (10 min)

**Lesson 1.4: Architecture Overview (35 min)**
- Learning goal: Understand complete DocuBot system and chapter roadmap
- Key concepts: 4 main components (Agent, RAG Pipeline, Backend API, Frontend UI), chapter progression
- Analogy: House blueprint (see the plan before building)
- DocuBot task: Create architecture.md with 3 sections (System Overview diagram, Component Descriptions, Chapter Roadmap)
- Expected outcome: "*Documented architecture that serves as your roadmap for the entire course.*"
- Hints: 5 progressive (Simple box diagram → Components to list → Markdown headers → Chapter roadmap → Text-based diagram acceptable)
- Word count: ~2000 words
- Time breakdown: Concept (15 min) + DocuBot task (15 min) + Roadmap reflection (5 min)

**IV. Constitution Compliance Check**
- ✅ PC-001: One concept per lesson (APIs, Agents, Environment, Architecture)
- ✅ PC-002: Cognitive load LOW (3-4 concepts per lesson)
- ✅ PC-003: Clear analogy in every lesson (restaurant, smart person, chef, blueprint)
- ✅ PC-004: Lesson 1.2 is code-free (mental model formation)
- ✅ PC-005: Concept → Apply separation (teach idea, then DocuBot task)
- ✅ PC-006: 4-6 progressive hints per lesson
- ✅ PC-007: Lesson duration ≤50 minutes (30, 35, 45, 35 min)
- ✅ CSC-002: DocuBot section has Task (numbered steps) + Outcome (italicized)
- ✅ CSC-005: Cumulative progress (test_api.py → table → environment → architecture)
- ✅ TC-001: No agent code (only API verification in L1.1)

**V. Testing & Validation Strategy**
- Acceptance Test 1: API key verification (test_api.py executes, AI response received)
- Acceptance Test 2: Conceptual understanding (comparison table with 5+ items per column)
- Acceptance Test 3: Environment setup completeness (uv --version works, pyproject.toml exists, .env configured, openai importable)
- Acceptance Test 4: Architecture documentation (architecture.md has 3 sections, 5+ components listed)
- Acceptance Test 5: DocuBot section format compliance (every lesson has Task + Outcome)
- Acceptance Test 6: Chapter completion (all 4 outcomes achieved, 2-3 hours total, no agent code written yet)

**VI. Risk Mitigation**
- Risk 1: Environment setup failures (Windows line endings, corporate firewalls) → Provide OS-specific instructions + pip fallback for UV
- Risk 2: API key issues (expired, invalid, missing) → Clear error messages + troubleshooting section in L1.1
- Risk 3: Time overrun (students take longer than 50 min per lesson) → Buffer time in estimates, optional "fast track" path
- Risk 4: Cognitive overload (too many concepts) → Limited to 3-4 concepts per lesson, clear analogies, progressive hints

**VII. Implementation Sequence**
1. Create Lesson 1.1 (API Fundamentals) - prerequisite for environment setup
2. Create Lesson 1.3 (Environment Setup) - requires API key from L1.1
3. Create Lesson 1.2 (LLM vs Agent) - can be parallel with L1.3, pure conceptual
4. Create Lesson 1.4 (Architecture Overview) - final lesson, synthesizes all prior context

**VIII. Content Creation Checklist**
- [ ] Lesson 1.1: test_api.py starter code, 5 hints, restaurant analogy, API key troubleshooting
- [ ] Lesson 1.2: Comparison table template, 4 hints, smart person analogy, agent examples
- [ ] Lesson 1.3: UV installation (Mac/Linux/Windows), 6 hints, chef analogy, environment verification script
- [ ] Lesson 1.4: Architecture diagram template, 5 hints, blueprint analogy, chapter roadmap

**Intelligence Creation Assessment**:
- No skills/subagents needed for Chapter 1 (foundational concepts, no recurring patterns)
- Potential skill for Chapter 2+: "environment-setup-helper" (guides students through troubleshooting)
- Chapter 1 focuses on mental model formation, not intelligence accumulation

## Outcome

- ✅ Impact: Created comprehensive 1,678-line implementation plan for Chapter 1 covering all 4 lessons with pedagogical analysis, lesson specifications, testing strategy, risk mitigation, and constitutional compliance. Plan ready for content implementers to create lesson files.
- 🧪 Tests: 6 acceptance tests defined covering API verification, conceptual understanding, environment completeness, architecture documentation, DocuBot format compliance, and chapter completion criteria.
- 📁 Files: 1 file created - `specs/041-ch01-intro-apis-agents/plan.md` (1,678 lines)
- 🔁 Next prompts: `/sp.tasks 041-ch01-intro-apis-agents` to generate actionable task breakdown, or proceed directly to content creation using Section III (Lesson-by-Lesson Implementation Plan) as guide
- 🧠 Reflection: chapter-planner agent successfully applied pedagogical frameworks (4-Layer Method L1, cognitive load A1-A2, teaching modality variation) and enforced all constitutional constraints (no agent code, one concept per lesson, DocuBot project integration). Plan provides complete guidance for content creators including word counts (8,300 total), time breakdowns (2-3 hours), hint frameworks, and troubleshooting strategies. All 26 functional requirements from spec mapped to lesson implementations.

## Evaluation notes (flywheel)

- Failure modes observed: None. chapter-planner correctly identified CONTENT work type, applied pedagogical frameworks, and enforced all specification constraints.
- Graders run and results (PASS/FAIL): Constitutional compliance = PASS (all 7 pedagogical constraints + 7 content structure constraints met), Specification alignment = PASS (all 26 functional requirements addressed), Pedagogical soundness = PASS (cognitive load appropriate for A1-A2, analogies clear, progressive hints structured)
- Prompt variant (if applicable): Standard `/sp.plan` routing to chapter-planner for educational content
- Next experiment (smallest change to try): Consider auto-generating lesson templates with pre-filled sections (Learning Goal, Analogy, DocuBot Task) to reduce content creator burden and ensure consistency

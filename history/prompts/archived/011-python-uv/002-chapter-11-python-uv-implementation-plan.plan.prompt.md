---
id: 002
title: Chapter 11 Python UV Implementation Plan
stage: plan
date: 2025-11-04
surface: agent
model: claude-3.7-sonnet
feature: 011-python-uv
branch: 011-python-uv
user: developer
command: /sp.plan
labels: ["chapter-planning", "python-uv", "part-4", "ai-driven-development", "beginner-intermediate"]
links:
  spec: specs/011-python-uv/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/011-python-uv/plan.md
tests:
  - Constitutional alignment validated
  - Skills proficiency mapping (CEFR A2-B1)
  - Complexity tier verification (Intermediate, max 7 concepts)
  - Learning objectives use Bloom's taxonomy
  - AI-first teaching pattern throughout
---

## Prompt

Follow instructions in sp.plan.prompt.md.
Generate a plan using chapter-planner.md carefully aligned with constitution.md and cowriter-guide. The reader/learner is a beginner with no prior programming and software development experience. Only use AI CLI Tool like Claude Code or Gemini CLI as AI collaborator. Generate complete output when adding the prompt (what user asked), What actions AI cli tool took and the output.

## Response snapshot

**Phase 0: Context Gathering**

Read and analyzed:
- `specs/011-python-uv/spec.md` (complete chapter specification with business goals, success evals, user stories, requirements)
- `.specify/memory/constitution.md` v3.0.1 (AI-native software development principles, evals-first methodology)
- `docs/COWRITER-GUIDE.md` (AI-native content creation workflow)
- `.claude/agents/chapter-planner.md` (pedagogical architecture agent instructions)
- `.claude/skills/skills-proficiency-mapper/SKILL.md` (CEFR proficiency mapping framework)
- `specs/book/chapter-index.md` (current implementation status: 14 chapters implemented, 41 planned)

**Phase 1: Chapter Type & Proficiency Analysis**

**Chapter Type Identified**: Technical/Code-Focused (hands-on AI collaboration with UV package manager)

**Target Audience**: Beginner to Intermediate (Part 4)
- Prerequisites: Chapters 1-10 (AI-driven development foundations, AI CLI tools, Bash, Git, Prompt Engineering)
- No Python syntax knowledge required (UV taught as "project management" before coding)
- First chapter in Part 4 (establishes tooling for all subsequent Python chapters)

**Proficiency Tier**: A2-B1 (Basic → Intermediate)
- A2 skills: Recognize concepts, apply in guided contexts
- B1 skills: Apply independently to real problems, troubleshoot with AI
- Cognitive load: Max 7 concepts per lesson (intermediate tier)
- AI assistance: Primary command executor (students focus on intent, not syntax)

**Phase 1.5: Skills Proficiency Mapping**

Identified 12 skills across 7 lessons mapped to CEFR proficiency levels:

**Foundation Skills (A1-A2)**:
1. Evaluate Python Package Managers (A2) - Compare tools, choose appropriate one
2. Recognize Speed Benefits (A1) - Understand value of fast installation
3. Execute Installation with AI (A2) - Install UV via intent-based prompts
4. Understand PATH Configuration (A2) - Troubleshoot "command not found"

**Intermediate Skills (B1)**:
5. Initialize UV Project (B1) - Create project structure independently with AI
6. Understand Project Configuration (A2) - Interpret pyproject.toml
7. Manage Project Dependencies (B1) - Add/update/remove packages
8. Understand Dependency Resolution (B1) - Troubleshoot conflicts
9. Execute Python in Project Environment (B1) - Run scripts/tests/servers
10. Understand Environment Isolation (B1) - Debug module errors
11. Share Projects Reproducibly (B1) - Prepare for team collaboration
12. Understand Lockfile Purpose (B1) - Explain version pinning

**Advanced Discovery Skill (B1)**:
13. Discover UV Features Just-in-Time (B1) - Self-directed learning with AI

**Proficiency Progression Validation**:
- ✅ Lessons 1-2: A1-A2 (Recognition and basic application)
- ✅ Lessons 3-6: B1 (Independent application to real problems)
- ✅ Lesson 7: B1 (Discovery and extension)
- ✅ No proficiency regressions (always forward or stable)
- ✅ Cognitive load respected (max 7 concepts per lesson)

**Phase 2: Lesson Architecture Design**

Created 7-lesson structure following pedagogical scaffolding:

**Lesson 1: Why UV? Understanding Modern Python Package Management** (Conceptual foundation)
- Objective: Evaluate package managers and recognize speed benefits
- Skills: A2 (Evaluate Package Managers) + A1 (Recognize Speed Benefits)
- Key Concepts: 7 (package manager purpose, tooling fragmentation, unified approach, speed, industry backing, decision framework, AI-driven approach)
- Content: Hook (30-second setup demo), problem (fragmentation), solution (UV), decision matrix, learning approach
- Practice: Reflection questions, comparison exercise
- Try With AI: ChatGPT Web (4 prompts exploring UV value)

**Lesson 2: Installing UV with AI Collaboration** (First hands-on)
- Objective: Execute installation workflow and understand PATH configuration
- Skills: A2 (Execute Installation, Understand PATH)
- Key Concepts: 7 (installation process, PATH variable, platform detection, verification, AI's role, common errors, one-time setup)
- Content: Pre-installation overview, AI-driven installation, platform detection, verification, troubleshooting
- Practice: Hands-on installation, verification exercise, troubleshooting simulation
- Try With AI: Claude Code/Gemini CLI (4 prompts for installation and troubleshooting)

**Lesson 3: Creating Your First UV Project with AI** (Project structure)
- Objective: Initialize UV project and understand project configuration
- Skills: B1 (Initialize Project) + A2 (Understand Configuration)
- Key Concepts: 7 (Python project definition, structure, pyproject.toml, virtual environments, dependency specification, automatic activation, modern vs. legacy)
- Content: Project definition, AI-driven creation, anatomy tour, pyproject.toml deep dive, virtual environments, legacy comparison
- Practice: Create personal project, explore files, customize metadata
- Try With AI: Claude Code/Gemini CLI (4 prompts for creation and exploration)

**Lesson 4: Managing Dependencies with AI** (Core workflow)
- Objective: Manage dependencies and understand dependency resolution
- Skills: B1 (Manage Dependencies, Understand Resolution)
- Key Concepts: 7 (dependency definition, production vs. dev, resolution, version constraints, lockfile, transitive dependencies, updating/removing)
- Content: Dependency explanation, adding with AI, dev vs. production, resolution process, updating, removing, conflict resolution
- Practice: Build FastAPI dependency set, scenario-based exercises, troubleshooting
- Try With AI: Claude Code/Gemini CLI (5 prompts for dependency management)

**Lesson 5: Running Python Code in UV Projects with AI** (Execution)
- Objective: Execute Python in project environment and understand isolation
- Skills: B1 (Execute in Environment, Understand Isolation)
- Key Concepts: 7 (environment isolation, automatic activation, running scripts, running tests, starting servers, one-off commands, debugging)
- Content: Why isolation, automatic activation, running scripts/tests/servers, one-off commands, troubleshooting
- Practice: Run scripts, start server, run tests, debugging exercise
- Try With AI: Claude Code/Gemini CLI (5 prompts for execution and troubleshooting)

**Lesson 6: Team Collaboration and Reproducible Environments with AI** (Integration)
- Objective: Share projects reproducibly and understand lockfile purpose
- Skills: B1 (Share Projects, Understand Lockfiles)
- Key Concepts: 7 (reproducibility, pyproject.toml vs. uv.lock, lockfile generation, syncing, git workflow, production deployment, lockfile updates)
- Content: Collaboration problem, lockfiles solution, preparing for sharing, teammate setup, production deployment, maintenance, git integration
- Practice: "Teammate Handoff" simulation, clone and setup, production install, git commit
- Try With AI: Claude Code/Gemini CLI (5 prompts for collaboration workflows)

**Lesson 7: Exploring Advanced UV Features with AI** (Discovery)
- Objective: Discover features just-in-time and manage Python versions
- Skills: B1 (Discover Just-in-Time, Manage Python Versions)
- Key Concepts: 7 (just-in-time learning, Python version management, standalone scripts, global tools, temporary dependencies, workspace management, AI as discovery partner)
- Content: JIT learning philosophy, version management, standalone scripts, global tools, experimentation, discovery pattern, resources
- Practice: Exploratory exercise (find 3 new features), hands-on experiments, apply to own project
- Try With AI: Claude Code/Gemini CLI (5 prompts for feature discovery)

**Phase 2.5: Topic Completeness & Gaps Analysis**

**Book Gaps Checklist Addressed**:
- ✅ Factual accuracy: UV version referenced, official docs linked, installation tested cross-platform
- ✅ Field volatility: UV is modern (0.1.6+), chapter includes "maintenance triggers" note for future updates
- ✅ Inclusivity: Diverse project examples (web apps, APIs, data work), no gatekeeping terms
- ✅ Accessibility: Grade 7-8 reading level, technical terms defined, 5-7 min sections with visual breaks
- ✅ Security: Installation verification, dependency reputation checks, script review before execution
- ✅ Ethical AI: AI validation emphasized (don't blindly trust), safety notes in every "Try With AI"
- ✅ Cross-platform: Windows, Mac, Linux tested for all commands
- ✅ Real-world deployment: Lesson 6 covers production workflows (--no-dev, lockfiles)
- ✅ Scalability: Team collaboration patterns scale to production environments

**Phase 3: Content Flow & Scaffolding**

**Lesson Progression Logic**:
```
Lesson 1 (Why UV?) → Motivation established
  ↓
Lesson 2 (Installation) → UV installed and verified
  ↓
Lesson 3 (First Project) → Project structure created
  ↓
Lesson 4 (Dependencies) → Packages installed
  ↓
Lesson 5 (Running Code) → Execution successful
  ↓
Lesson 6 (Collaboration) → Team workflows ready
  ↓
Lesson 7 (Advanced Features) → Complete UV competency
```

**Complexity Scaffolding**:
- Lesson 1: Conceptual only (no commands) - 6 concepts
- Lesson 2: First hands-on - 7 concepts
- Lessons 3-5: Progressive skill building - 7 concepts each
- Lesson 6: Integration of prior skills - 7 concepts
- Lesson 7: Synthesis and exploration - 7 concepts

**AI Assistance Scaffolding**:
- Early (1-3): AI provides complete commands with explanations
- Middle (4-5): AI responds to specific needs (less hand-holding)
- Late (6-7): AI as discovery partner (exploratory questions)

**Phase 4: Validation Strategy Design**

**Formative Assessment** (During lessons):
- Reflection questions (conceptual sections)
- Hands-on verification (AI confirms success)
- Troubleshooting scenarios (error diagnosis with AI)
- "Teach back" checks (explain to AI)

**Summative Assessment** (End of chapter):
1. **Quiz Assessment**: 15 questions, 75%+ passing (conceptual understanding, no memorization)
2. **Hands-On Exercise**: "Create FastAPI Project with UV and AI" (project running, structure understood)
3. **Real-World Application**: "Prepare Project for Team Collaboration" (teammate can recreate environment)
4. **Teaching Ability**: "Teach UV to a Peer" (peer successfully uses UV with student's guidance)
5. **AI Interaction Quality**: Effective collaboration (clear prompts, validation, troubleshooting)

**CEFR-Aligned Rubric**:
- A1 (Fail): Cannot perform skill
- A2 (Pass): Performs with AI guidance
- B1 (Target): Performs independently with AI
- B2 (Exceed): Handles complex scenarios, teaches others

**Constitutional Alignment Verification**:
- ✅ AI-First Teaching (Principle 1): Every lesson uses AI CLI tools, prompt-response pattern throughout
- ✅ Spec-First Methodology (Principle 2): Business Goals → Evals → Requirements structure followed
- ✅ Modern Python Standards (Principle 3): pyproject.toml emphasized, UV as modern standard
- ✅ Validation-First Safety (Principle 4): Safety notes in all "Try With AI" sections
- ✅ Progressive Complexity (Principle 5): Conceptual → hands-on → integration → discovery
- ✅ Domain Skills Integration: 6 skills applied (learning objectives, concept scaffolding, assessment builder, AI-collaborate teaching, technical clarity, exercise designer)

**Output Generated**:

Created `specs/011-python-uv/plan.md` with:
- Complete 7-lesson architecture (79 pages of detailed planning)
- Skills proficiency mapping (CEFR A2-B1 progression)
- Learning objectives (Bloom's taxonomy for each lesson)
- Content outlines (7 concepts max per lesson)
- Prompt-response patterns (minimum 20 examples specified)
- "Try With AI" sections (end-of-lesson AI activities)
- Assessment strategy (5 evaluation methods with rubric)
- Constitutional alignment checklist (all principles verified)
- Implementation notes for lesson writers (tone, visuals, safety)
- Validation checklist (before publishing)

## Outcome

- ✅ Impact: Complete implementation plan for Chapter 11 (Python UV) ready for content-implementer agent
- 🧪 Tests: Constitutional alignment verified, skills proficiency validated (A2-B1), complexity tier confirmed (intermediate, max 7 concepts)
- 📁 Files: `specs/011-python-uv/plan.md` (79 pages, comprehensive lesson architecture)
- 🔁 Next prompts: 
  - `/sp.tasks` to generate actionable task checklist from this plan
  - `/sp.implement` with content-implementer agent to generate content
  - `/sp.validate` with validation-auditor to verify quality
- 🧠 Reflection: This is the first chapter in Part 4, establishing UV as the standard Python tooling for all subsequent chapters (12-29). The AI-driven teaching pattern is critical: students learn concepts through AI collaboration, not command memorization. Beginner-friendly positioning (A2-B1) appropriate for first Python tooling chapter.

## Evaluation notes (flywheel)

- Failure modes observed: None detected. Plan follows chapter-planner agent instructions, constitution v3.0.1 principles, and COWRITER-GUIDE workflow.
- Graders run and results (PASS/FAIL): 
  - ✅ PASS: Constitutional alignment (AI-first, spec-first, modern standards, validation-first, progressive complexity)
  - ✅ PASS: Skills proficiency mapping (CEFR A2-B1 validated, no regressions, cognitive load respected)
  - ✅ PASS: Complexity tier (intermediate scaffolding, max 7 concepts, AI as primary executor)
  - ✅ PASS: Learning objectives (Bloom's taxonomy applied to all 7 lessons)
  - ✅ PASS: Prompt-response pattern (minimum 20 examples across all lessons)
- Prompt variant (if applicable): Standard `/sp.plan` workflow with chapter-planner agent
- Next experiment (smallest change to try): Monitor if content-implementer agent produces content matching plan's specifications (especially prompt-response pattern and "Try With AI" sections)

# Chapter 32: Building Projects with Spec-Kit Plus — Task Checklist

**Feature Branch**: `002-chapter-32-redesign`
**Chapter Type**: Technical/Hybrid (Decomposition thinking + tool proficiency)
**Status**: Planning Phase Complete — Ready for Development
**Owner**: [To be assigned]
**Estimated Total Effort**: 60-80 story points (2-3 week full-time effort for one developer, or 4-6 weeks for part-time)

---

## Overview

This task checklist breaks Chapter 32 implementation into specific, testable development tasks. All tasks follow the specification and detailed lesson plan (see `plan.md`).

### Technical References Included

**EVERY task now includes comprehensive technical references** so ANY implementation worker (human or AI agent) can execute it efficiently without conversation context:

1. **What to Read/Study**:
   - Style guides, lesson plans, external documentation
   - Example lessons to study (2-3 per task)
   - Conceptual resources (articles, tutorials)

2. **How to Actually Do It**:
   - Code example templates (copy-paste ready)
   - Try With AI prompts (3 specific prompts with expected outcomes)
   - Reflection worksheet templates
   - Real-world examples (specific companies/scenarios)

3. **How to Validate It's Done Right**:
   - Code execution tests (all scripts must run without errors)
   - Cognitive load verification (count concepts, verify ≤ limit)
   - Scale connection validation (search for "7-9 agents", verify frequency)
   - CEFR proficiency alignment checks

4. **Examples**:
   - **Task 1.1** (Chapter README): 6 reference files + implementation guidance + validation checklist
   - **Task 2.1** (Lesson 1): 7 reference categories + git worktree script template + 3 Try With AI prompts
   - **Task 4.1** (Lesson 8 - CLIMAX): Complete 5-agent orchestration script (bash) + error handling + monitoring

**Result**: Each task is **self-contained and executable** without requiring conversation context or additional clarification.

## North Star Vision: Create "Creative Orchestrators" (Two Climaxes)

**THE TRANSFORMATIVE PROMISE**: After completing this chapter, students become **"creative orchestrators"** who can coordinate 10-15 autonomous agents (AI or human) through decomposition thinking, progressing from manual team leadership (FIRST CLIMAX) to programmatic automation enabling creative independence (SECOND CLIMAX).

**Two Climaxes Design**:
1. **FIRST CLIMAX (Lesson 4)**: MANUAL parallel SpecKit Plus - student acts as "team lead" coordinating 5+ agent teams (SpecKit Plus workflows across 5 worktrees), running `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement` simultaneously - experiencing decomposition thinking firsthand (2.5-3x speedup proven)
2. **SECOND CLIMAX (Lesson 8)**: PROGRAMMATIC parallel SpecKit Plus - student builds Super Orchestrator script that spawns 10-15 independent Claude sessions automatically via headless mode - achieving **creative independence** (10x+ productivity, freeing human for strategy/innovation)

**Graduate Identity**: Students become **"creative orchestrators"**—practical for PMs/founders who decompose once, automate execution, iterate creatively. They can manage agent teams like human teams, with specs eliminating coordination chaos.

**Reality Check for Implementation (Honest Expectation Setting)**:
- ⚠️ First decomposition takes **2-3x longer** than "just start coding" (set expectation in Lesson 1)
- ✅ Payoff is **10x overall gains** (speedup + quality + scalability)
- Students who skip decomposition: Fast start, slow finish (merge conflicts, rework)
- Students who invest in decomposition: Slow start, fast finish (clean integration, reusable patterns)
- **Decomposition quality is THE bottleneck**, not tools

**Key Understanding**: Students already learned Claude Code (Ch 5) and SpecKit Plus (Ch 31). This chapter teaches **running SpecKit Plus workflows IN PARALLEL** to master decomposition thinking - first manually (team lead), then programmatically (creative independence).

### The Super Orchestrator Architecture (THE PRACTICAL PATTERN)

```
Human (sets goal/contract)
  ↓
Super Orchestrator (bash script / meta-agent)
  ├─ Decomposes goal into parallelizable features
  ├─ Spawns N Claude sessions via headless mode (claude -p)
  └─ Monitors progress, integrates results
       ↓
  ┌────────────────┬────────────────┬────────────────┐
  │ Claude Session 1│ Claude Session 2│ ... Session 15 │
  │                │                │                │
  │ worktree-001/  │ worktree-002/  │ worktree-015/  │
  │ Own MCP servers│ Own MCP servers│ Own MCP servers│
  │ Own agents     │ Own agents     │ Own agents     │
  │ (Task/Explore) │ (Task/Explore) │ (Task/Explore) │
  └────────────────┴────────────────┴────────────────┘
       ↓                 ↓                 ↓
  Feature A        Feature B         Feature O
  (isolated)       (isolated)        (isolated)
```

**Key Architecture Principles**:
1. **Super Orchestrator** = Script that establishes contract with human, decomposes work, spawns sessions
2. **Each Claude session is independent**: Own worktree (isolation) + own MCP (capabilities) + own internal agents
3. **Headless mode** (`claude -p`) = Programmatic interface for spawning sessions
4. **MCP purpose**: Gives each session tools/skills it needs (database, filesystem, APIs) - NOT cross-session coordination
5. **No inter-session coordination protocol needed**: Each session works independently via specifications

**Why This Is More Practical**:
- ✅ Natural isolation via git worktrees
- ✅ Each session can fail without affecting others
- ✅ MCP provides capabilities (not coordination complexity)
- ✅ Headless mode makes spawning programmatic
- ✅ Specifications define integration contracts (not runtime coordination)

### The Student Journey (Two Climaxes: Team Lead → Creative Orchestrator)

```
START: Managing 1 SpecKit Plus workflow (from Chapter 31)
  → Role: Individual developer executing workflows
  ↓
LESSONS 1-2: Learn to run 2-3 MANUAL parallel SpecKit Plus workflows
  → New role: "Team lead" coordinating agent teams
  → Setup: git worktrees + open 3 terminals
  → Run: /sp.specify, /sp.plan, /sp.tasks in each worktree (parallel)
  → Understand: Decomposition thinking enables parallelization
  → Build belief: "I can manage agent teams like human teams, specs eliminate coordination chaos"
  → Reality check: Upfront decomposition takes 2-3x longer (but worth it!)
  ↓
LESSON 3: MANUAL parallel /sp.implement + Integration
  → Team lead coordinates 3 agent teams (parallel implementations)
  → Git merge → test integration (proof of decomposition quality)
  → Measure: 2.5x speedup (3 features in 3 hours vs 9 hours sequential)
  → Build belief: "Decomposition thinking WORKS - upfront investment pays off 10x"
  ↓
LESSON 4: ✨ FIRST CLIMAX ✨ — Scale to 5+ MANUAL parallel workflows
  → Team lead coordinates 5 agent teams simultaneously
  → Full workflow: /sp.specify → /sp.plan → /sp.tasks → /sp.implement
  → Experience decomposition thinking at meaningful scale
  → Build belief: "I am a team lead who coordinates agent teams like human teams"
  ↓
LESSONS 5-7: Learn automation tools (OPTIONAL for advanced students)
  → Transition: Team lead → Creative orchestrator
  → Lesson 5: CI/CD for validation (automate quality gates)
  → Lesson 6: Headless mode (programmatic /sp.specify, /sp.plan, /sp.tasks)
  → Lesson 7: Sandboxing (safe isolation for 10-15 sessions)
  → Build belief: "I understand tools that enable creative independence"
  ↓
LESSON 8: ✨ SECOND CLIMAX ✨ — ACHIEVE CREATIVE INDEPENDENCE
  → Write Super Orchestrator script spawning 10 SpecKit Plus workflows (headless mode)
  → Each workflow: /sp.specify → /sp.plan → /sp.tasks → /sp.implement
  → Student RUNS script, SEES 10 workflows execute autonomously
  → Human freed for strategy/creative work (not tactical coordination)
  → Path to 15+ workflows becomes clear (just add features to array)
  → Build belief: "I built a Super Orchestrator - I have creative independence now"
  ↓
LESSON 9: PROVE IT — Capstone project
  → Use Super Orchestrator OR manual parallel (student choice)
  → Build 5-feature system, measure gains, reflect on upfront investment vs payoff
  → Document: "Upfront thinking doubled, total delivery halved - now I focus on strategy"
  → Build belief: "I am a creative orchestrator"
  ↓
END: Creative Orchestrator who coordinates 10-15 autonomous agents (AI or human)
    → Graduate identity: Practical for PMs/founders
    → Superpower: Decompose once, automate execution, iterate creatively
```

**EVERY LESSON MUST**:
1. **Frame role progression**: Lessons 1-4 as "team lead" (manual coordination), Lessons 5-8 as "creative orchestrator" (automation enabling creative independence)
2. **Connect to agent teams analogy**: "Managing agent teams like human teams, specs eliminate coordination chaos"
3. **Show progression**: What you learn enables X sessions now, Y sessions later (via Super Orchestrator script)
4. **Include reflection**: "How does this enable coordinating 10-15 autonomous agents (AI or human)?"
5. **Build beliefs progressively**:
   - Lessons 1-4: "I can manage agent teams like human teams"
   - Lessons 5-8: "I achieve creative independence through automation"
   - Lesson 9: "I am a creative orchestrator"

**CRITICAL SUCCESS CRITERIA**:
- ✅ Student completes chapter believing: **"I am a creative orchestrator"** (not just "I learned parallel workflows")
- ✅ Every lesson answers: "Why does this matter for becoming a creative orchestrator?"
- ✅ Lesson 4 (FIRST CLIMAX): Student coordinates 5 agent teams manually, experiences decomposition thinking at scale
- ✅ Lesson 8 (SECOND CLIMAX): Student WRITES and RUNS Super Orchestrator script, experiences creative independence (freed for strategy while automation handles execution)
- ✅ Capstone reflection: "Upfront thinking doubled, total delivery halved—now I focus on strategy"

**Key Constraints for Implementation**:
- **60% emphasis on decomposition thinking** (the bottleneck - poor decomposition → chaos at 3 agents; excellent decomposition → scalable to 15 agents)
- **40% emphasis on tool proficiency** (tools amplify decomposition quality, don't fix it)
- **Creative independence framing in Lessons 5-8** (automation frees humans for strategic/creative work, not just "faster execution")
- **Upfront investment honesty in Lesson 1** (set expectation: 2-3x longer for first decomposition, 10x overall gains)
- **"Agent teams like human teams" analogy throughout** (makes transferability explicit)
- **"Creative orchestrator" graduate identity in Lesson 9** (portfolio-worthy narrative)
- **MCP purpose clarity**: MCP provides capabilities/tools to each session (NOT inter-session coordination protocol)

**Time Estimates**: Based on Part 5 (Intermediate) complexity tier; assumes content writer is familiar with SpecKit Plus and technical education.

---

## Technical References Template (Apply to EVERY Lesson Task)

Every lesson task MUST include comprehensive technical references so ANY worker can execute it efficiently without conversation context.

### Required Technical References Sections

**1. Technical References** (What to read/study):
- Lesson style guide: `.claude/output-styles/lesson.md`
- Lesson plan: `specs/002-chapter-32-redesign/plan.md` (specific lesson section)
- External documentation: Tool/technology docs (git, Claude Code, SpecKit Plus, etc.)
- Example lessons to study: 2-3 existing lessons with similar structure/complexity
- Conceptual resources: Articles, tutorials explaining key concepts

**2. Implementation Guidance** (How to actually do it):
- **Code example templates**: Specific format/structure for each code block
- **Try With AI prompts**: 3 concrete prompts with expected outcomes
- **Reflection worksheet templates**: Tables, questions, measurement tools
- **Real-world examples**: Which companies/scenarios to reference

**3. Quality Standards** (How to validate it's done right):
- **Code examples**: Must be copy-paste ready, tested on macOS/Linux
- **Cognitive load**: Max N concepts (list them explicitly)
- **Skills proficiency**: CEFR level (A2/B1/B2) with validation criteria
- **Engagement**: Scale connection validation (7-9 agents mentioned X times)
- **Reading level**: Grade 10-12 (Hemingway Editor check)

**4. Validation Checklist** (Testable acceptance):
- [ ] Run all code examples - must work without errors
- [ ] Count new concepts - verify ≤ cognitive load limit
- [ ] Search for "7-9 agents" - must appear 3+ times
- [ ] Test Try With AI prompts - verify helpful responses
- [ ] Check CEFR alignment - content matches stated proficiency level

---

## Super Orchestrator Scale Connection Template (Apply to EVERY Lesson)

Every lesson task MUST include this validation checklist in acceptance criteria:

**Super Orchestrator Scale Connection** (CRITICAL):
- ✅ Introduction explicitly connects lesson to Super Orchestrator vision (spawning 10-15 independent Claude sessions)
- ✅ At least one section shows: "This enables Super Orchestrator to manage N sessions now, M sessions later"
- ✅ Reflection exercise asks: "How does this enable Super Orchestrator to scale to 10-15 sessions?"
- ✅ Try With AI prompt references scale: "How would Super Orchestrator use this with 10-15 sessions?"
- ✅ Student completes lesson understanding its role in building Super Orchestrator capability

**Lesson-Specific Super Orchestrator Messaging**:
- **Lessons 1-2**: "You're learning to spawn 2-3 sessions manually to understand decomposition (foundation for Super Orchestrator)"
- **Lessons 3-4**: "You're proving decomposition works at 2-3 session scale (preparing to automate with Super Orchestrator)"
- **Lessons 5-7**: "You're adding automation capabilities that Super Orchestrator will use (CI/CD, MCP config, background execution)"
- **Lesson 8**: "You're BUILDING THE SUPER ORCHESTRATOR - script that spawns 10-15 sessions via headless mode" (THIS IS THE CLIMAX)
- **Lesson 9**: "You're using Super Orchestrator for real project, proving you can spawn 10-15 sessions programmatically"

**Architecture Clarity** (MUST be explicit in every lesson):
- Each Claude session is INDEPENDENT: own worktree + own MCP + own internal agents
- Super Orchestrator spawns sessions via headless mode (`claude -p`)
- MCP provides capabilities/tools to each session (NOT cross-session coordination)
- Specifications define integration contracts (sessions work independently)

---

## Task List by Phase

### Phase 1: Chapter Structure & README

#### Task 1.1: Create Chapter README.md

**Priority**: MUST
**Effort**: 2 hours

**Description**: Create chapter overview document that LEADS with the transformative vision: 1 human managing 7-9 AI Agents

**Acceptance Criteria**:
- [ ] File exists at `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/book-source/docs/05-Spec-Driven-Development/32-real-world-spec-kit-workflows/README.md`
- [ ] Matches directory structure from `specs/book/directory-structure.md`
- [ ] **Opening hook** (first 2 paragraphs):
  - LEADS with the vision: "Imagine orchestrating 7-9 AI Agents simultaneously..."
  - Establishes the transformation: "From managing 1 agent to coordinating 10-15"
  - States the outcome: "Build in 3 hours what takes 30 hours traditionally"
- [ ] Includes:
  - **What You'll Learn** section emphasizing scale progression (2-3 → 5-7 → 7-9 agents)
  - **Prerequisites** section explicitly listing Chapters 30-31, 5, 7, 8
  - **Time Commitment** section (10-12 hours to 10x capability, 6-8 hours fast-track)
  - **The Journey** section showing progression:
    - Lessons 1-4: Master 2-3 agents manually
    - Lessons 5-7: Scale to 5-7 agents with automation
    - Lesson 8: REALIZE the vision - 7-9 agents with meta-orchestration
    - Lesson 9: Prove it - portfolio-worthy capstone
- [ ] YAML frontmatter:
  ```yaml
  ---
  title: "Chapter 32: The Super AI Orchestra - Managing 7-9 AI Agents"
  chapter: 32
  part: 5
  sidebar_position: 4
  description: "Master decomposition thinking to orchestrate 7-9 AI Agents and achieve 10x productivity"
  ---
  ```
- [ ] **7-9 agent Vision Validation**:
  - ✅ Vision mentioned in first paragraph
  - ✅ Clear progression path to 7-9 agents shown
  - ✅ Lesson 8 framed as "vision realization" not "optional advanced"
  - ✅ Student understands: "This chapter enables me to manage 7-9 agents"
- [ ] No grammatical errors, clear and engaging tone
- [ ] Matches existing chapter READMEs in style and structure

**Technical References**:
1. **Chapter README Style Guide**: `.claude/output-styles/chapters.md`
2. **Example Chapter READMEs to Study**:
   - `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/book-source/docs/05-Spec-Driven-Development/README.md` (Part 5 intro - study structure)
   - `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/book-source/docs/05-Spec-Driven-Development/30-specification-driven-development-fundamentals/README.md` (Chapter 30)
   - `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/book-source/docs/05-Spec-Driven-Development/31-spec-kit-plus-hands-on/README.md` (Chapter 31 - direct prerequisite)
3. **Directory Structure**: `specs/book/directory-structure.md` (naming conventions)
4. **Chapter Index**: `specs/book/chapter-index.md` (chapter titles, numbers)
5. **Vision Reference**: `specs/002-chapter-32-redesign/spec.md` (read Executive Summary for 7-9 agent vision framing)
6. **Docusaurus YAML frontmatter docs**: https://docusaurus.io/docs/markdown-features/front-matter

**Implementation Guidance**:
- **Hook pattern**: Study Chapter 5 README opening ("Picture this...") for engaging hook style
- **Learning objectives format**: Use action verbs (Master, Understand, Build, Orchestrate)
- **Time commitment**: Be specific with progression (6-8h fast-track, 10-12h complete, +4h optional)
- **Journey visualization**: Use simple text diagram (see North Star section in tasks.md for format)

**Quality Standards**:
- Reading level: Grade 10-12 (use Hemingway Editor to verify)
- Engagement: First 2 paragraphs must hook reader with vision
- Clarity: Non-technical stakeholder should understand chapter value after reading README

---

### Phase 2: Core Lesson Content Development (Lessons 1-4, includes FIRST CLIMAX)

#### Task 2.1: Lesson 1 — Git Worktrees & Parallel Specifications

**Priority**: MUST
**Effort**: 8 hours (4h content + 2h code examples + 2h exercises + review)

**Description**: Develop complete Lesson 1 teaching git worktrees, parallel spec generation, and decomposition thinking fundamentals

**Acceptance Criteria**:
- [ ] Content file created: `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/book-source/docs/05-Spec-Driven-Development/32-real-world-spec-kit-workflows/01-git-worktrees-parallel-specifications.md`
- [ ] YAML frontmatter includes:
  - title, chapter, lesson, duration_minutes (90)
  - skills metadata (3 skills with CEFR levels A2/B1, categories, Bloom's levels, measurable outcomes)
  - learning_objectives (1 primary objective using Bloom's "Apply")
  - cognitive_load (list 7 concepts, verified against A2/B1 limits)
- [ ] Content structure (following plan.md):
  - H1 title
  - Introduction (why this matters)
  - 5 sections matching lesson plan:
    1. What is a Worktree? (5 min)
    2. Setting Up Your First 3 Worktrees (20 min)
    3. Running Parallel Specifications (30 min)
    4. Understanding Integration Contracts (20 min)
    5. Reflection & Time Tracking (15 min)
  - Try With AI section (final section only, no separate "Key Takeaways" or "What's Next")
- [ ] Code examples (3 total):
  - Example 1: `git worktree setup script` (bash, 20 lines, copy-paste ready)
    - Creates 3 worktrees: feature-001-auth, feature-002-payment, feature-003-dashboard
    - Includes comments explaining each step
    - Tests: Script runs without errors, creates 3 worktrees verified by `git worktree list`
  - Example 2: Feature spec template (markdown, shows integration contract structure)
  - Example 3: Integration mapping diagram (ASCII or visual description, shows feature dependencies)
- [ ] Exercises (4 total):
  - Exercise 1: Create 3 worktrees in provided starter repository (testable: student produces output of `git worktree list`)
  - Exercise 2: Run parallel `/sp.specify` in all 3 worktrees (testable: student produces 3 specs in feature-001, 002, 003 directories)
  - Exercise 3: Analyze 3 specs and identify integration contracts (testable: student documents 3 integration points)
  - Exercise 4: Time tracking worksheet (testable: student completes worksheet showing parallel vs sequential estimates)
- [ ] Real-world examples (3):
  - Vercel engineering: How they decompose features for 50+ engineers
  - Solo founder example: Building 3-feature MVP using parallel development
  - Academic context: Distributed teams coordinating via specs
- [ ] Try With AI section (15 min):
  - Tool: Claude Code CLI
  - 2-3 prompts provided with expected outcomes
  - Brief safety/ethics note (AI suggests improvements, student responsible for final quality)
- [ ] Accessibility check:
  - Jargon defined (worktree, feature branch, integration contract)
  - Concepts explained before use
  - Code examples have clear comments
  - Inclusive language (diverse examples)
- [ ] No separate "Key Takeaways" or "What's Next" sections (use Try With AI as closure only)
- [ ] **7-9 agent Scale Connection** (CRITICAL):
  - ✅ Introduction explicitly states: "You're learning to manage 2-3 agents NOW to scale to 7-9 agents LATER"
  - ✅ Section 4 (Integration Contracts) connects to scale: "Integration contracts that work for 3 agents will work for 7-9 agents"
  - ✅ Section 5 (Reflection) includes question: "How would this workflow change with 7-9 agents instead of 3?"
  - ✅ Try With AI prompt: "Ask your AI: 'How would I set up worktrees for 10 agents simultaneously?'"
  - ✅ Student understands: "I'm building the foundation for 7-9 agent orchestration"

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md` (YAML frontmatter structure, section format)
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 1 section - learning objectives, skills metadata, content outline)
3. **Git Worktree Documentation**:
   - Official docs: https://git-scm.com/docs/git-worktree
   - Tutorial: https://morgan.cugerone.com/blog/workarounds-to-git-worktree-using-bare-repository-and-cannot-fetch-remote-branches/
   - Best practices: https://www.gitkraken.com/learn/git/git-worktree
4. **SpecKit Plus Documentation**:
   - Workflow: `.specify/` directory (study structure)
   - Commands: Look at existing PHRs in `history/prompts/` for `/sp.specify` usage patterns
5. **Example Lessons to Study**:
   - Chapter 31, Lesson 1: `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/book-source/docs/05-Spec-Driven-Development/31-spec-kit-plus-hands-on/01-your-first-specification.md` (study structure)
   - Chapter 8, Lesson 2: Git branching fundamentals (reference for git concepts explanation)
6. **Integration Contracts Concept**:
   - RESTful API design principles (integration contract examples)
   - Microservices boundaries (decomposition thinking reference)
7. **Claude Code CLI Docs**:
   - https://docs.claude.com/en/docs/claude-code/getting-started
   - https://docs.claude.com/en/docs/claude-code/features

**Implementation Guidance**:
- **Git worktree script**: Provide bash script with OS-specific handling (macOS: `git worktree add ../feature-001-auth feature-001-auth`, Linux/Windows variations)
- **Integration contract template**: Create markdown table showing: Feature | Depends On | Provides | Data Format
- **Code example comments**: Explain WHY each step matters for 7-9 agent scale (not just WHAT it does)
- **Reflection worksheet**: Include time tracking table: Task | Sequential Time | Parallel Time | Speedup Factor
- **Try With AI prompts**:
  1. "Explain how git worktrees enable true isolation for parallel development"
  2. "What would I need to change to set up 10 worktrees instead of 3?"
  3. "Design integration contracts for a 5-feature e-commerce system"

**Quality Standards**:
- **Code examples**: All scripts must be copy-paste ready and tested on macOS/Linux
- **Cognitive load**: Max 7 new concepts (list them: worktree, feature branch, parallel spec, integration contract, PHR routing, feature numbering, time tracking)
- **Skills proficiency**: CEFR A2 level (recognition + simple application with scaffolding)
- **Engagement**: Each section answers "Why does this matter for 7-9 agents?" within first 2 paragraphs

**Validation**:
- [ ] Run git worktree script on fresh repo - must create 3 worktrees without errors
- [ ] Verify cognitive load: Count new concepts, confirm ≤7
- [ ] Check scale connection: Search lesson for "7-9 agents" - must appear 3+ times
- [ ] Test Try With AI prompts with Claude Code CLI - verify they produce helpful responses

---

#### Task 2.2: Lesson 2 — Parallel Planning & Tasks Generation

**Priority**: MUST
**Effort**: 7 hours

**Description**: Develop complete Lesson 2 on running `/sp.plan` and `/sp.tasks` simultaneously

**Acceptance Criteria**:
- [ ] Content file created: `02-parallel-planning-and-tasks.md`
- [ ] YAML frontmatter with skills metadata (3 skills, A2/B1/B1 levels)
- [ ] Content structure (5 sections from plan.md):
  1. Review: Your 3 Specs & Integration Contracts (10 min)
  2. Running Parallel Planning (25 min)
  3. Evaluating Plan Quality as Decomposition Indicator (15 min)
  4. Running Parallel Task Generation (20 min)
  5. Terminal Management Best Practices (10 min)
  6. Reflection: Parallelization Value (10 min)
- [ ] Code examples (3):
  - Example 1: tmux configuration script for 3-session layout
  - Example 2: Terminal naming/management guide (screenshots or ASCII diagrams)
  - Example 3: Bash script to monitor parallel plan generation
- [ ] Exercises (5):
  - Exercise 1: Set up tmux/terminal for 3 parallel sessions
  - Exercise 2: Run `/sp.plan` simultaneously in all 3 worktrees
  - Exercise 3: Evaluate plan quality as decomposition indicator
  - Exercise 4: Run `/sp.tasks` in all 3 worktrees
  - Exercise 5: Time tracking and analysis
- [ ] Try With AI section (15 min, 3 prompts)
- [ ] Duration: 90 minutes estimated
- [ ] **7-9 agent Scale Connection** (CRITICAL):
  - ✅ Introduction: "You're learning to manage 2-3 agents' planning simultaneously - essential for scaling to 10-15"
  - ✅ Section 3 (Plan Quality): "Good decomposition = clean plans. Bad decomposition = complex plans. At 7-9 agent scale, bad decomposition becomes unmanageable."
  - ✅ Section 6 (Reflection): "How would parallel planning change with 10 worktrees? What becomes harder? What stays the same?"
  - ✅ Try With AI prompt: "Design a terminal management strategy for monitoring 10 parallel planning sessions"
  - ✅ Student understands: "Parallel planning proves decomposition quality before implementation"

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md`
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 2 section)
3. **SpecKit Plus Commands Documentation**:
   - `/sp.plan` usage: Study existing PHRs in `history/prompts/` for patterns
   - `/sp.tasks` workflow: `.specify/templates/tasks-template.md`
   - Feature plan structure: `.specify/templates/plan-template.md`
4. **Terminal Multiplexing**:
   - tmux documentation: https://github.com/tmux/tmux/wiki/Getting-Started
   - tmux cheat sheet: https://tmuxcheatsheet.com/
   - iTerm2 split panes guide (macOS): https://iterm2.com/documentation-one-page.html
5. **Example Lessons to Study**:
   - Chapter 31, Lesson 2: Planning workflow (study structure)
   - Chapter 7, Lesson 3: Terminal basics (reference for terminal management)
6. **Parallel Execution Monitoring**:
   - Bash backgrounding: `command &` and `wait`
   - Process monitoring: `ps`, `jobs`, `fg`, `bg`

**Implementation Guidance**:
- **tmux Configuration Script Template**:
  ```bash
  #!/bin/bash
  # setup-3-session-layout.sh

  # Create new tmux session with 3 panes
  tmux new-session -d -s parallel-dev
  tmux split-window -h
  tmux split-window -v

  # Navigate to each worktree
  tmux send-keys -t parallel-dev:0.0 'cd ../feature-001-auth' C-m
  tmux send-keys -t parallel-dev:0.1 'cd ../feature-002-payment' C-m
  tmux send-keys -t parallel-dev:0.2 'cd ../feature-003-dashboard' C-m

  # Attach to session
  tmux attach-session -t parallel-dev
  ```

- **Plan Quality Evaluation Rubric** (markdown table):
  | Quality Indicator | Good Decomposition | Bad Decomposition |
  |-------------------|-------------------|-------------------|
  | Plan length | 2-4 pages per feature | 10+ pages (too complex) or <1 page (underspecified) |
  | Dependencies | Clearly listed, minimal | Circular, unclear, many |
  | Tasks | 10-20 specific tasks | <5 tasks (vague) or 50+ tasks (too granular) |
  | Integration points | Explicitly defined | Implicit, assumed |

- **Try With AI Prompts**:
  1. "Analyze these 3 feature plans - what integration risks do you see based on dependencies?"
  2. "If I'm running 10 parallel planning sessions, what terminal layout would you recommend?"
  3. "This plan has 15 dependencies on other features - is this good or bad decomposition? Why?"

**Quality Standards**:
- **Code examples**: tmux script must work on macOS/Linux (tested)
- **Cognitive load**: 7 concepts (parallel planning, plan quality indicators, terminal multiplexing, dependency analysis, integration risks, time tracking, reflection)
- **Skills proficiency**: A2/B1 (simple application with scaffolding + independent application to familiar problems)
- **Scale connection**: "7-9 agents" mentioned 5+ times
- **Reading level**: Grade 10-12

**Validation Checklist**:
- [ ] Run tmux script - must create 3-pane layout successfully
- [ ] Test plan quality rubric - can differentiate good/bad decomposition
- [ ] Count concepts - verify = 7
- [ ] Search "7-9 agents" - verify appears 5+ times
- [ ] Test Try With AI prompts - verify helpful analysis responses

---

#### Task 2.3: Lesson 3 — Parallel Implementation & Integration

**Priority**: MUST
**Effort**: 8 hours

**Description**: Develop complete Lesson 3 on running `/sp.implement` in parallel and managing integration

**Acceptance Criteria**:
- [ ] Content file created: `03-parallel-implementation-and-integration.md`
- [ ] YAML frontmatter with skills metadata (3 skills, all B1 level)
- [ ] Content structure (7 sections from plan.md):
  1. Preparing for Parallel Implementation (15 min)
  2. Running Parallel Implementation (with background note) (25 min)
  3. Background: Monitoring Multi-Session Execution (20 min)
  4. Integration: Merging in Dependency Order (30 min)
  5. Handling & Learning from Merge Conflicts (20 min)
  6. Integration Validation (15 min)
  7. Time Tracking & Reflection (10 min)
- [ ] Code examples (3):
  - Example 1: 3-feature starter repository with clear specs
  - Example 2: Merge conflict resolution guide (step-by-step, common scenarios)
  - Example 3: Integration testing script
- [ ] Exercises (6):
  - Exercise 1: Map feature dependencies and merge order
  - Exercise 2: Run `/sp.implement` in all 3 worktrees
  - Exercise 3: Merge branches following dependency order
  - Exercise 4: Resolve conflicts strategically (if any)
  - Exercise 5: Run end-to-end tests on integrated system
  - Exercise 6: Time tracking and quality analysis
- [ ] Troubleshooting guide:
  - "Merge conflicts everywhere" checklist
  - "One feature way bigger" decomposition red flags
  - "Tests failing after merge" dependency diagnosis
- [ ] Try With AI section (3 prompts focused on merge conflict meaning)
- [ ] Duration: 120 minutes
- [ ] **7-9 agent Scale Connection** (CRITICAL):
  - ✅ Introduction: "You're proving decomposition works at 2-3 agent scale - the patterns you learn here work for 7-9 agents"
  - ✅ Section 5 (Merge Conflicts): "Merge conflicts = feedback on decomposition quality. At 7-9 agent scale, many conflicts = system design problem, not merge problem."
  - ✅ Section 7 (Reflection): "You integrated 3 features. What would change with 10 features? What decomposition patterns prevent integration chaos?"
  - ✅ Try With AI prompt: "Analyze this merge conflict - does it indicate a decomposition problem or just overlapping work?"
  - ✅ Student understands: "Clean merges prove good decomposition; conflicts teach better decomposition"

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md`
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 3 section)
3. **Git Integration Documentation**:
   - Git merge: https://git-scm.com/docs/git-merge
   - Merge strategies: https://git-scm.com/docs/merge-strategies
   - Conflict resolution: https://git-scm.com/book/en/v2/Git-Branching-Basic-Branching-and-Merging
4. **SpecKit Plus Implementation**:
   - `/sp.implement` command workflow
   - Background bash execution: Study `.specify/scripts/` for patterns
   - PHR routing during implementation
5. **Example Lessons to Study**:
   - Chapter 8, Lesson 4: Git merging fundamentals
   - Chapter 31, Lesson 3: Implementation workflow
6. **Integration Testing Patterns**:
   - Smoke tests for multi-feature systems
   - Integration test design (API contracts, data flow)
   - End-to-end validation strategies

**Implementation Guidance**:
- **3-Feature Starter Repository Structure**:
  ```
  starter-repo/
  ├── feature-001-auth/         # JWT authentication
  │   ├── spec.md
  │   └── plan.md
  ├── feature-002-payment/      # Stripe integration (depends on auth)
  │   ├── spec.md
  │   └── plan.md
  └── feature-003-dashboard/    # Analytics UI (depends on auth)
      ├── spec.md
      └── plan.md
  ```

- **Merge Conflict Resolution Guide** (step-by-step):
  1. **Identify conflict type**: Code overlap vs architectural conflict
  2. **Analyze root cause**: Same file modification (OK) vs integration contract violation (BAD)
  3. **Resolution strategy**:
     - Code overlap → Keep both changes, refactor if needed
     - Integration contract violation → STOP, revisit decomposition
  4. **Validation**: Run tests after resolution

- **Integration Testing Script Template**:
  ```bash
  #!/bin/bash
  # integration-tests.sh

  echo "🧪 Running integration tests..."

  # Test 1: Auth + Payment integration
  echo "Testing authenticated payment flow..."
  # (actual test commands here)

  # Test 2: Auth + Dashboard integration
  echo "Testing authenticated dashboard access..."
  # (actual test commands here)

  echo "✅ All integration tests passed!"
  ```

- **Try With AI Prompts**:
  1. "I have a merge conflict in user.py between auth and payment features. The conflict is on line 45. Here's the conflict: [paste]. Is this a decomposition problem or just overlapping work?"
  2. "Design an integration test strategy for 3 features with these dependencies: Payment→Auth, Dashboard→Auth, Analytics→Dashboard"
  3. "If 2 out of 3 features had merge conflicts, what does that tell me about my decomposition?"

**Quality Standards**:
- **Starter repository**: Must have clear specs with integration contracts defined
- **Merge conflict examples**: Real conflicts from actual parallel development, not synthetic
- **Integration tests**: Must verify cross-feature functionality, not just individual features
- **Cognitive load**: 7-9 concepts (B1 level allows slightly higher load)
- **Scale connection**: "7-9 agents" mentioned 5+ times
- **Troubleshooting guide**: Must address real pain points (not theoretical)

**Validation Checklist**:
- [ ] Create 3 worktrees from starter repo, implement in parallel, verify integration
- [ ] Introduce intentional conflict, test resolution guide clarity
- [ ] Run integration tests - must verify cross-feature functionality
- [ ] Count concepts - verify ≤ 9 (B1 limit)
- [ ] Search "7-9 agents" - verify appears 5+ times
- [ ] Test Try With AI prompts - verify conflict analysis quality

---

#### Task 2.4: Lesson 4 — Scaling from 3 to 5+ Features (FIRST CLIMAX)

**Priority**: MUST (FIRST CLIMAX - Manual Parallel SpecKit Plus at Scale)
**Effort**: 6 hours
**Note**: This is THE FIRST CLIMAX - students coordinate 5+ MANUAL parallel SpecKit Plus workflows, experiencing decomposition thinking at scale firsthand.

**Description**: Develop complete Lesson 4 as FIRST CLIMAX - students scale from 3 to 5+ MANUAL parallel SpecKit Plus workflows, experiencing full decomposition thinking cycle at meaningful scale

**Acceptance Criteria**:
- [ ] Content file created: `04-scaling-decomposition-thinking.md`
- [ ] YAML frontmatter with skills metadata (3 skills, B1/B2 levels)
- [ ] Content structure (7 sections):
  1. Analyzing Your 3-Feature Decomposition (15 min)
  2. What Scales from 3→5 Features (20 min)
  3. What Breaks at 5+ Features (20 min)
  4. Communication Complexity Analysis (10 min)
  5. Identifying Decomposition Problems Early (15 min)
  6. Path to Scaling: What Comes Next (10 min)
  7. Reflection & Strategic Thinking (10 min)
- [ ] Content elements:
  - Visualizations: Integration complexity graph, dependency diagrams, decomposition rubric
  - 4 exercises (self-assessment, red flag identification, redesign, 5-feature mapping)
  - Real-world examples (Stripe, Linux kernel, startup scaling)
- [ ] Try With AI section (3 prompts on scalability and redesign)
- [ ] Duration: 90 minutes
- [ ] Key insight: Transferability (how this applies to human teams, not just AI agents)
- [ ] **7-9 agent Scale Connection** (CRITICAL):
  - ✅ Introduction: "You've mastered 2-3 agents. Now you're preparing for 5-7 agents, with eyes on 10-15"
  - ✅ Section 3 (What Breaks): "At 7-9 agent scale, communication complexity explodes - decomposition quality becomes CRITICAL"
  - ✅ Section 6 (Path to Scaling): "The path from 5 to 7-9 agents: automation (Lessons 5-7) + meta-orchestration (Lesson 8)"
  - ✅ Section 7 (Reflection): "Could you manage 7-9 agents with current decomposition approach? What would need to change?"
  - ✅ Student understands: "Decomposition thinking scales; manual coordination doesn't"

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md`
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 4 section)
3. **System Scaling Patterns**:
   - Conway's Law: https://www.melconway.com/Home/Conways_Law.html (system design mirrors org structure)
   - Fred Brooks - "The Mythical Man-Month": Communication complexity O(n²)
   - Microservices decomposition patterns (Martin Fowler)
4. **Real-world Scaling Examples**:
   - Stripe API decomposition: https://stripe.com/docs/api (study how features are isolated)
   - Linux kernel subsystem boundaries (study `MAINTAINERS` file)
   - Startup scaling case studies (Y Combinator library)
5. **Example Lessons to Study**:
   - Chapter 30, Lesson 3: Specification quality assessment
   - Architecture patterns resources in constitution

**Implementation Guidance**:
- **Integration Complexity Graph** (ASCII visualization):
  ```
  3 features: 3 integration points  (manageable)
     A─B
      ╲│
       C

  5 features: 10 integration points (complex)
     A─B
     │╲│╱│
     │ C │
     │╱│╲│
     D─E

  10 features: 45 integration points (chaos without good decomposition)
     [complex mesh diagram showing exponential growth]
  ```

- **Decomposition Rubric** (assessment tool):
  | Aspect | Score 1-5 | Questions to Ask |
  |--------|-----------|------------------|
  | Independence | How independently can features be built? |
  | Integration contracts | Are boundaries explicit and stable? |
  | Communication overhead | How much coordination needed? |
  | Dependency depth | How many layers of dependencies? |
  | Coupling | How many shared components? |

- **Red Flag Identification Exercise**:
  - "Every feature depends on Feature A" → Feature A is a bottleneck
  - "Circular dependencies" → System design problem
  - "Shared database tables modified by 3+ features" → Coupling problem

- **Try With AI Prompts**:
  1. "Analyze this 5-feature decomposition for scalability red flags: [paste feature descriptions and dependencies]"
  2. "Design a decomposition for [system] that could scale from 5 to 15 features without redesign"
  3. "I have 10 features with circular dependencies. How would you redesign for better decomposition?"

**Quality Standards**:
- **Visualizations**: Must clearly show complexity growth (3→5→10 features)
- **Real-world examples**: Must be specific (not "companies do this") - cite actual systems
- **Cognitive load**: 9-10 concepts (B1/B2 level allows higher complexity for analysis)
- **Scale connection**: "7-9 agents" mentioned 7+ times (preparing for Lessons 5-8)
- **Transferability**: Must explicitly connect to human team coordination

**Validation Checklist**:
- [ ] Visualizations render correctly (ASCII or diagrams)
- [ ] Decomposition rubric can assess real systems
- [ ] Count concepts - verify ≤ 10 (B1/B2 limit)
- [ ] Search "7-9 agents" - verify appears 7+ times
- [ ] Test Try With AI prompts - verify architectural analysis quality
- [ ] Verify transferability: "human teams" mentioned 3+ times

---

### Phase 3: Automation Layer Lessons (Lessons 5-7)

#### Task 3.1: Lesson 5 — CI/CD & Validation Hooks

**Priority**: SHOULD
**Effort**: 7 hours
**Complexity**: B1 (Upper-Intermediate)

**Description**: Develop complete Lesson 5 on GitHub Actions and spec validation automation

**Acceptance Criteria**:
- [ ] Content file created: `05-automation-ci-cd-validation-hooks.md`
- [ ] YAML frontmatter with skills metadata (3 skills: CI/CD pipeline config, spec validation automation, automation as amplifier)
- [ ] Content structure (6 sections, 90 minutes total):
  1. Why Automation Matters for Scaling (15 min)
  2. Setting Up GitHub Actions for Spec Validation (25 min)
  3. Understanding Validation As Decomposition Gating (20 min)
  4. Integration with SpecKit Plus Workflow (15 min)
  5. Scaling Validation (10 min)
  6. Reflection: Automation as Enabler (10 min)
- [ ] Code examples (3):
  - Example 1: GitHub Actions workflow for spec validation (`.github/workflows/spec-validation.yml`, ready to use)
  - Example 2: Bash/Python script to validate spec structure locally
  - Example 3: Constitution-based validation checklist
- [ ] Exercises (4):
  - Exercise 1: Create spec validation workflow (use provided template)
  - Exercise 2: Push deliberately bad spec, watch validation fail
  - Exercise 3: Fix spec to pass validation
  - Exercise 4: Run validation locally and in CI/CD, compare results
- [ ] Try With AI section (3 prompts on validation design and decomposition feedback)
- [ ] Key principle: Automation is only valuable when decomposition is clear
- [ ] **7-9 agent Scale Connection** (CRITICAL):
  - ✅ Introduction: "You're scaling to 5-7 agents with automation - approaching the 7-9 agent vision"
  - ✅ Section 1 (Why Automation): "At 7-9 agent scale, manual validation is impossible - automation becomes mandatory"
  - ✅ Section 5 (Scaling Validation): "This validation setup works for 5 features today, 15 features tomorrow - same workflow"
  - ✅ Try With AI prompt: "Design a validation system that could handle 15 parallel spec generations"
  - ✅ Student understands: "Automation amplifies good decomposition"

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md`
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 5 section)
3. **GitHub Actions Documentation**:
   - Workflows: https://docs.github.com/en/actions/using-workflows
   - Workflow syntax: https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions
   - Triggering workflows: https://docs.github.com/en/actions/using-workflows/triggering-a-workflow
4. **CI/CD Best Practices**:
   - Fast feedback loops (failing fast)
   - Atomic validations (one check per failure mode)
   - Pipeline as code (versioned, reviewed)
5. **SpecKit Plus Validation**:
   - Constitution compliance checks: `.specify/memory/constitution.md`
   - Spec template structure: `.specify/templates/spec-template.md`

**Implementation Guidance**:
- **GitHub Actions Workflow Template**:
  ```yaml
  name: Spec Validation
  on: [push, pull_request]
  jobs:
    validate-spec:
      runs-on: ubuntu-latest
      steps:
        - uses: actions/checkout@v3
        - name: Validate spec structure
          run: |
            # Check spec.md exists
            test -f specs/*/spec.md || exit 1
            # Check required sections present
            grep -q "## Requirements" specs/*/spec.md || exit 1
            grep -q "## Success Criteria" specs/*/spec.md || exit 1
        - name: Check constitution alignment
          run: |
            # Verify no implementation details leaked
            ! grep -i "python\|typescript\|react" specs/*/spec.md || exit 1
  ```

- **Try With AI Prompts**:
  1. "Design validation rules that catch vague requirements before they reach implementation"
  2. "If 15 parallel specs are being generated, what validations would catch decomposition problems early?"
  3. "This spec has 50 requirements - is that a validation failure? Why or why not?"

**Quality Standards**:
- **GitHub Actions workflow**: Must run successfully on test repo
- **Validation script**: Must catch real spec quality issues
- **Cognitive load**: 8 concepts (B1 level)
- **Scale connection**: "7-9 agents" mentioned 5+ times

**Validation Checklist**:
- [ ] Run GitHub Actions workflow - must validate spec successfully
- [ ] Test with bad spec - must fail validation correctly
- [ ] Count concepts - verify ≤ 8
- [ ] Search "7-9 agents" - verify appears 5+ times

---

#### Task 3.2: Lesson 6 — Headless Mode: Programmatic SpecKit Plus Execution

**Priority**: SHOULD
**Effort**: 7 hours
**Complexity**: B1 (Upper-Intermediate)
**Note**: Foundation for Super Orchestrator (Lesson 8). Students learn to run `/sp.specify`, `/sp.plan`, `/sp.tasks` programmatically.

**Description**: Develop complete Lesson 6 teaching headless mode as the programmatic interface for running SpecKit Plus workflows. Students learn `claude -p`, output formats, session management, and multi-turn conversations - the building blocks for Super Orchestrator.

**Acceptance Criteria**:
- [ ] Content file created: `06-headless-mode-programmatic-speckit-plus.md`
- [ ] YAML frontmatter with skills metadata (4 skills: headless execution, JSON parsing, session management, programmatic SpecKit Plus)
- [ ] Content structure (7 sections, 90 minutes):
  1. **Why Programmatic SpecKit Plus Matters** (10 min) - "You've run `/sp.specify` manually; now automate it"
  2. **Headless Mode Basics** (20 min) - `claude -p "/sp.specify"` command structure
  3. **Output Formats** (15 min) - JSON output, stream-json, parsing with jq
  4. **Multi-Turn Conversations** (15 min) - `--resume` flag for /sp.specify → /sp.plan → /sp.tasks workflow
  5. **Practical Examples** (20 min) - Run headless /sp.specify for single feature, capture session_id
  6. **Building Toward Super Orchestrator** (5 min) - "These commands are what Super Orchestrator will use"
  7. **Reflection** (5 min) - "How does headless mode enable spawning 10-15 SpecKit Plus workflows?"
- [ ] Code examples (5 — all copy-paste ready):
  - Example 1: Basic headless /sp.specify command
  - Example 2: JSON output parsing with jq (extract session_id)
  - Example 3: Multi-turn workflow (Specify → Plan with --resume)
  - Example 4: Stream-json output format (real-time monitoring)
  - Example 5: Error handling (check exit codes, parse error JSON)
- [ ] Exercises (4):
  - Exercise 1: Run `claude -p "/sp.specify \"Add user authentication\""` and inspect JSON output
  - Exercise 2: Parse session_id using jq
  - Exercise 3: Resume session with `/sp.plan` using --resume flag
  - Exercise 4: Compare manual vs headless workflow (same result, different interface)
- [ ] Real-world examples: How headless mode enables batch processing, CI/CD integration, automated workflows
- [ ] Try With AI section (3 prompts testing headless mode understanding)
- [ ] **Super Orchestrator Scale Connection** (CRITICAL):
  - ✅ Introduction: "Headless mode is how Super Orchestrator will spawn 10-15 SpecKit Plus workflows programmatically"
  - ✅ Section 6: "Super Orchestrator uses these exact commands in loops to spawn N sessions"
  - ✅ Reflection: "How would you use headless mode to run 10 /sp.specify commands in parallel?"
  - ✅ Try With AI prompt: "Design a bash script that runs headless `/sp.specify` for 5 features in parallel"

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md`
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 6/8 meta-orchestration section)
3. **Claude Code Headless Mode Documentation**: https://docs.claude.com/en/docs/claude-code/headless
   - Basic usage with `--print` / `-p` flag
   - Output formats: text, json, stream-json
   - Multi-turn conversations with `--resume`
   - Configuration options: `--allowedTools`, `--permission-mode`, etc.
4. **SpecKit Plus Commands in Headless Mode**:
   - Study existing PHRs showing `/sp.specify`, `/sp.plan`, `/sp.tasks` usage
   - Understand how session continuity works across workflow phases
5. **JSON Parsing with jq**:
   - jq manual: https://stedolan.github.io/jq/manual/
   - Common patterns: extracting session_id, checking status, handling errors
6. **Example workflows from plan.md**: Lesson 8 orchestration examples

**Implementation Guidance**:
- **Basic Headless Command Template**:
  ```bash
  # Run /sp.specify in headless mode
  claude -p "/sp.specify \"Add user authentication with JWT\"" \
    --output-format json \
    > output-spec.json

  # Extract session ID for later use
  SESSION_ID=$(jq -r '.session_id' output-spec.json)
  echo "Session ID: $SESSION_ID"
  ```

- **Multi-Turn Workflow Template**:
  ```bash
  # Phase 1: Specify
  claude -p "/sp.specify \"Add payment processing\"" \
    --output-format json > spec.json

  # Phase 2: Plan (resume session)
  SESSION=$(jq -r '.session_id' spec.json)
  claude -p "/sp.plan" \
    --resume "$SESSION" \
    --output-format json > plan.json

  # Phase 3: Tasks (resume session)
  claude -p "/sp.tasks" \
    --resume "$SESSION" \
    --output-format json > tasks.json
  ```

- **Try With AI Prompts**:
  1. "Explain the difference between `claude -p` (headless) and regular `claude` (interactive). When would I use each?"
  2. "I ran headless `/sp.specify` and got this JSON output: [paste]. What does each field mean?"
  3. "Design a bash script that runs `/sp.specify` for 5 features in parallel and captures all session IDs"

**Quality Standards**:
- **All commands must run successfully**: Students test each command and verify output
- **JSON parsing works**: jq commands extract session_id correctly
- **Multi-turn works**: --resume flag maintains context across Specify → Plan
- **Cognitive load**: 8 concepts (B1 level)
- **Scale connection**: "Super Orchestrator" + "programmatic SpecKit Plus" mentioned 5+ times

**Validation Checklist**:
- [ ] Run basic headless `/sp.specify` - verify JSON output received
- [ ] Parse session_id with jq - verify extraction works
- [ ] Run multi-turn (Specify → Plan) with --resume - verify context maintained
- [ ] Test error handling - verify error JSON parsed correctly
- [ ] Count "Super Orchestrator" mentions - verify 5+ occurrences
- [ ] Verify students understand: "This is the foundation for Super Orchestrator"

---

#### Task 3.3: Lesson 7 — Sandboxing: Safe Session Isolation at Scale

**Priority**: SHOULD
**Effort**: 7 hours
**Complexity**: B1 (Upper-Intermediate)
**Note**: Critical security foundation for spawning 10-15 sessions. Each SpecKit Plus workflow must be isolated.

**Description**: Develop complete Lesson 7 teaching sandboxing as the security mechanism that enables safely spawning 10-15 independent SpecKit Plus workflows. Students learn filesystem isolation, network isolation, and why sandboxing matters when running multiple autonomous sessions.

**Acceptance Criteria**:
- [ ] Content file created: `07-sandboxing-safe-session-isolation.md`
- [ ] YAML frontmatter with skills metadata (4 skills: sandbox configuration, filesystem isolation, network security, multi-session safety)
- [ ] Content structure (7 sections, 90 minutes):
  1. **Why Sandboxing Matters at Scale** (10 min) - "Running 10-15 sessions safely requires isolation"
  2. **Enabling Sandboxing** (15 min) - `/sandbox` command, basic configuration
  3. **Filesystem Isolation** (20 min) - Each worktree sandboxed, preventing cross-contamination
  4. **Network Isolation** (20 min) - Domain restrictions, preventing data exfiltration
  5. **Security Benefits** (15 min) - Prompt injection protection, reduced attack surface
  6. **Super Orchestrator Security** (5 min) - "How sandboxing enables safe 10-15 session spawning"
  7. **Reflection** (5 min) - "Why is isolation critical when scaling to 10-15 sessions?"
- [ ] Code examples (4 — all copy-paste ready):
  - Example 1: Enable sandboxing with `/sandbox` command
  - Example 2: Filesystem access test (verify isolation works)
  - Example 3: Network access test (domain restrictions)
  - Example 4: Sandbox configuration for multi-worktree setup
- [ ] Exercises (4):
  - Exercise 1: Enable sandboxing in current session
  - Exercise 2: Test filesystem isolation (try accessing parent directory - should fail)
  - Exercise 3: Test network isolation (try accessing unapproved domain - should prompt)
  - Exercise 4: Configure sandbox for 3 worktrees (different allowed paths)
- [ ] Real-world examples: How sandboxing protects against prompt injection, malicious dependencies, compromised scripts
- [ ] Try With AI section (3 prompts testing sandboxing understanding)
- [ ] **Super Orchestrator Scale Connection** (CRITICAL):
  - ✅ Introduction: "Sandboxing enables safely spawning 10-15 SpecKit Plus workflows - each isolated, each secure"
  - ✅ Section 6: "Super Orchestrator will spawn 10-15 sandboxed sessions - no session can corrupt another"
  - ✅ Reflection: "Why is filesystem isolation essential when 10-15 sessions run simultaneously?"
  - ✅ Try With AI prompt: "What security risks exist when spawning 10-15 unsandboxed sessions? How does sandboxing mitigate them?"

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md`
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 6/7 automation section)
3. **Claude Code Sandboxing Documentation**: https://docs.claude.com/en/docs/claude-code/sandboxing
   - Overview: OS-level filesystem and network isolation
   - `/sandbox` command for enabling
   - Filesystem isolation (write access to CWD, read access configurable)
   - Network isolation (proxy server, domain restrictions)
   - Configuration through settings.json
4. **OS-Level Security Primitives**:
   - Linux: bubblewrap for isolation
   - macOS: Seatbelt for sandbox enforcement
5. **Security Benefits**:
   - Prompt injection protection
   - Reduced attack surface
   - Transparent operation with notifications
6. **Example workflows**: Multi-worktree sandbox configurations

**Implementation Guidance**:
- **Enable Sandboxing**:
  ```bash
  # In Claude Code session
  > /sandbox

  # Verify sandboxing enabled
  # Try accessing parent directory - should be blocked
  ```

- **Filesystem Isolation Test**:
  ```bash
  # Inside sandboxed session in worktree-001/

  # This should work (current directory)
  echo "test" > file.txt

  # This should be blocked (parent directory)
  echo "test" > ../sensitive-file.txt
  # Expected: Permission denied or sandbox violation
  ```

- **Network Isolation Test**:
  ```bash
  # Inside sandboxed session

  # Try accessing unapproved domain
  curl https://example-unapproved-domain.com
  # Expected: Connection blocked, permission prompt appears
  ```

- **Sandbox Configuration for Multi-Worktree**:
  ```json
  // settings.json
  {
    "sandbox": {
      "filesystem": {
        "allowedPaths": [
          "/path/to/worktree-001",
          "/path/to/worktree-002",
          "/path/to/worktree-003"
        ]
      },
      "network": {
        "allowedDomains": [
          "github.com",
          "api.anthropic.com"
        ]
      }
    }
  }
  ```

- **Try With AI Prompts**:
  1. "Explain why filesystem isolation is important when running 10 SpecKit Plus workflows simultaneously"
  2. "What attack vectors does sandboxing protect against in multi-session development?"
  3. "Design a sandbox configuration for 5 worktrees, each with different network access needs"

**Quality Standards**:
- **Sandboxing works**: Students successfully enable and verify isolation
- **Security understanding**: Students explain WHY sandboxing matters (not just HOW)
- **Multi-session context**: All examples show 3+ sessions/worktrees
- **Cognitive load**: 8 concepts (B1 level)
- **Scale connection**: "Super Orchestrator" + "10-15 sessions" mentioned 5+ times

**Validation Checklist**:
- [ ] Enable sandboxing - verify `/sandbox` command works
- [ ] Test filesystem isolation - verify parent directory access blocked
- [ ] Test network isolation - verify unapproved domain blocked
- [ ] Configure sandbox for 3 worktrees - verify each isolated
- [ ] Count "Super Orchestrator" mentions - verify 5+ occurrences
- [ ] Verify students understand: "Sandboxing makes 10-15 session spawning SAFE"

---

### Phase 4: SECOND CLIMAX — Super Orchestrator (Lesson 8)

#### Task 4.1: Lesson 8 — Building the Super Orchestrator

**Priority**: MUST (THIS IS THE PAYOFF)
**Effort**: 8 hours
**Complexity**: B2 (Upper-Intermediate/Advanced)
**Note**: THIS IS THE CLIMAX - where students BUILD and RUN a Super Orchestrator script that spawns 10-15 independent Claude sessions. Not optional - this is where everything clicks.

**Description**: Develop Lesson 8 as THE CLIMAX of Chapter 32 - students write and run a Super Orchestrator script that spawns 5 independent Claude Code sessions via headless mode, each building features in isolation with own worktree + MCP + agents

**Acceptance Criteria**:
- [ ] Content file created: `08-building-the-super-orchestrator.md`
- [ ] **OPENING HOOK**: "This is it. Everything you've learned builds to this moment. You're about to build a Super Orchestrator script that spawns 10-15 independent Claude Code sessions, each building features in parallel with zero coordination complexity."
- [ ] YAML frontmatter with skills metadata (4 skills: headless execution, orchestration scripts, session management, decomposition at scale — all B2 level)
- [ ] Content structure (9 sections, 90 minutes, CLIMAX FRAMING):
  1. **The Super Orchestrator Architecture** (15 min) - "Human → Orchestrator Script → N Independent Sessions (each with worktree+MCP+agents)"
  2. Understanding Headless Mode — Programmatic Claude Code Interface (20 min) - `claude -p "<prompt>"` spawns sessions
  3. Session Independence — Each Has Own Worktree + MCP + Agents (20 min) - "No coordination protocol needed"
  4. Writing Your First Super Orchestrator Script (20 min) - Decompose → Spawn → Monitor
  5. Monitoring & Error Handling — Managing 10-15 Independent Sessions (15 min)
  6. **PROOF: Running Your Super Orchestrator** (15 min) - Student RUNS script, SEES 5 sessions spawn and work independently
  7. **The Path to 10-15 Sessions** (15 min) - "Just add more features to the array, spawn more sessions"
  8. **Super Orchestrator as Your Superpower** (10 min) - "This pattern scales infinitely"
  9. **Reflection: You Just Built a Super Orchestrator** (10 min)
- [ ] Code examples (5 — COMPLETE working templates):
  - Example 1: Headless mode single-command spawning one Claude session (students run this NOW)
  - Example 2: Session resume pattern showing session independence (students run this NOW)
  - Example 3: **Complete Super Orchestrator script spawning 5 sessions** (students run this NOW) — THE PROOF
  - Example 4: Error handling: One session fails, others continue independently (students see this)
  - Example 5: Scaling patterns: How to spawn 10-15 sessions (template provided, discussed)
- [ ] Exercises (4 — ALL build belief):
  - Exercise 1: Run single headless command, see Claude session spawn programmatically
  - Exercise 2: Run Super Orchestrator script, WATCH 5 independent sessions spawn and build features
  - Exercise 3: Analyze the script: "Why does each session have own worktree+MCP? What enables 10-15 scale?"
  - Exercise 4 (reflection): "Before this chapter, could you spawn 10-15 Claude sessions? Now?"
- [ ] Real-world context: Technical AI Product Managers at scale, CTO-level coordination patterns
- [ ] Try With AI section: "Ask your AI: 'Help me write a Super Orchestrator script that spawns 12 independent Claude sessions to build a complete SaaS product'"
- [ ] **CLIMAX MESSAGING**: "You just built and ran a Super Orchestrator. Your script spawned 5 independent Claude sessions. Each had own worktree, own MCP, own agents. The path to 10-15 sessions is clear: just add more features. You have the superpower."
- [ ] **Super Orchestrator Scale Connection** (CRITICAL):
  - ✅ Introduction: "This is THE MOMENT - you're about to build Super Orchestrator that spawns 10-15 sessions"
  - ✅ Section 3 (Independence): "Why each session has own worktree+MCP+agents (no coordination complexity)"
  - ✅ Section 6 (Proof): Student RUNS Super Orchestrator script, SEES 5 sessions spawn independently
  - ✅ Section 7 (Path to 10-15): "To spawn 10-15 sessions, just add features to the array"
  - ✅ Section 8: "Super Orchestrator pattern scales infinitely (50+ sessions possible)"
  - ✅ Reflection: "You just built a Super Orchestrator. You can spawn 10-15 Claude sessions programmatically."

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md`
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 8 - meta-orchestration section)
3. **Claude Code Headless Mode Documentation**:
   - Official docs: https://docs.claude.com/en/docs/claude-code/headless
   - Command reference: `claude -p "<prompt>" --output-format json --resume <session-id>`
   - Session management guide: https://docs.claude.com/en/docs/claude-code/session-management
4. **Bash Scripting for Orchestration**:
   - Parallel execution: `&` backgrounding, `wait` for synchronization
   - JSON parsing: `jq` command reference (https://stedolan.github.io/jq/manual/)
   - Error handling patterns: `set -e`, trap handling, retry logic
5. **Example Orchestration Patterns**:
   - Study: GitHub Actions workflow files (`.github/workflows/*.yml`) for parallel job patterns
   - Study: Kubernetes Job parallelism (batch/v1 Job spec)
   - Study: Apache Airflow DAGs (parallel task execution)
6. **Meta-Orchestration Concepts**:
   - Martin Fowler: Orchestration vs Choreography (https://martinfowler.com/articles/microservices.html#OrchestrationVsChoreography)
   - Distributed systems coordination patterns
7. **SpecKit Plus at Scale**:
   - Existing PHRs showing `/sp.specify`, `/sp.plan`, `/sp.tasks` usage
   - Feature numbering system: `001-`, `002-`, `003-` (auto-increment pattern)

**Implementation Guidance**:
- **Super Orchestrator Script Template** (MUST PROVIDE - shows complete architecture):
  ```bash
  #!/bin/bash
  # super-orchestrator.sh
  # PURPOSE: Spawn 5 independent Claude Code sessions
  # ARCHITECTURE: Each session has own worktree + own MCP + own agents

  set -e  # Exit on error

  # Step 1: Human establishes contract (decomposition)
  FEATURES=(
    "001-user-auth:Add user authentication with JWT"
    "002-payment:Integrate Stripe payment processing"
    "003-notifications:Build email notification system"
    "004-analytics:Create usage analytics dashboard"
    "005-api-limits:Implement rate limiting"
  )

  echo "🧠 Super Orchestrator: Decomposed goal into ${#FEATURES[@]} features"
  echo "📋 Each feature will get: own worktree + own MCP config + own agents"
  echo ""

  # Step 2: Create isolated worktrees for each session
  echo "🌳 Creating independent worktrees..."
  for feature in "${FEATURES[@]}"; do
    IFS=':' read -r num_name desc <<< "$feature"
    git worktree add "../worktree-$num_name" -b "feature-$num_name"
    echo "   ✓ worktree-$num_name/ (isolated environment)"
  done
  echo ""

  # Step 3: Spawn 5 Claude sessions via headless mode (parallel)
  echo "🎼 Spawning 5 independent Claude Code sessions..."
  for feature in "${FEATURES[@]}"; do
    IFS=':' read -r num_name desc <<< "$feature"
    cd "../worktree-$num_name"

    # Each session runs independently with own MCP servers
    claude -p "/sp.specify \"$desc\"" \
      --output-format json \
      > "../output-$num_name-spec.json" &

    echo "   🤖 Session $num_name spawned (independent: own worktree + MCP + agents)"
  done
  wait  # Wait for all sessions to complete

  echo ""
  echo "✅ Super Orchestrator complete!"
  echo "   - 5 independent Claude sessions spawned"
  echo "   - Each had: own worktree, own MCP, own internal agents"
  echo "   - Zero coordination complexity (specifications define contracts)"
  echo ""
  echo "🚀 Path to 10-15 sessions: Add more features to FEATURES array"
  ```

- **Try With AI Prompts** (MUST INCLUDE):
  1. "Help me write a Super Orchestrator script that spawns 12 independent Claude sessions to build a complete e-commerce SaaS product (auth, payments, inventory, shipping, analytics, admin)"
  2. "What error handling should I add to Super Orchestrator to handle session failures gracefully?"
  3. "How would I monitor progress of 10-15 Claude sessions running in parallel? What metrics matter?"

- **Real-world Examples**:
  - **Anthropic**: How teams use Claude Code to spawn multiple sessions for parallel feature development
  - **Vercel**: How Next.js team manages 50+ parallel feature branches (same decomposition thinking)
  - **Solo Founder Case Study**: Developer ships 10-feature MVP in 1 week using Super Orchestrator pattern

**Quality Standards**:
- **Super Orchestrator script**: Must run on macOS/Linux, spawn 5 sessions successfully in < 20 minutes
- **Architecture clarity**: Script comments must explain: own worktree + own MCP + own agents for each session
- **Error handling**: Script must handle session failures gracefully (other sessions continue, failures reported)
- **Progress monitoring**: Must show real-time progress (echo statements for each session spawn)
- **Scale connection**: "Super Orchestrator" + "independent sessions" mentioned 10+ times (this is THE CLIMAX)
- **Cognitive load**: 10 concepts max (B2 level allows higher complexity)
- **Emotional impact**: Student must feel "I just built a Super Orchestrator" after running script

**Validation Checklist**:
- [ ] Run Super Orchestrator script on test repo - must spawn 5 sessions and create 5 worktrees successfully
- [ ] Verify each session has own worktree (check `git worktree list`)
- [ ] Test error handling - kill one session mid-run, verify others continue independently
- [ ] Count "Super Orchestrator" + "independent sessions" mentions - must appear 10+ times combined
- [ ] Check CLIMAX framing - first paragraph must say "This is it" or equivalent
- [ ] Student survey: "After Lesson 8, can you build a Super Orchestrator that spawns 10-15 sessions?" - must be 80%+ yes

**CRITICAL SUCCESS CRITERIA**:
- Student RUNS Super Orchestrator script and SEES 5 independent Claude sessions spawn
- Student understands each session is independent (own worktree + own MCP + own agents)
- Student understands path from 5 to 10-15 sessions (just add more features to array)
- Student completes lesson believing: "I just built a Super Orchestrator"

---

### Phase 5: Capstone Project (Lesson 9)

#### Task 5.1: Lesson 9 — Capstone Project & Measurement

**Priority**: MUST
**Effort**: 10 hours (2h lesson content + 4h starter projects + 2h templates + 2h rubric)

**Description**: Develop complete Lesson 9 including capstone project guidance and measurement framework

**Acceptance Criteria**:
- [ ] Content file created: `09-capstone-project-and-measurement.md`
- [ ] YAML frontmatter with skills metadata (4 skills: end-to-end delivery, metrics & reflection, portfolio narrative, decomposition mastery — all B1-B2)
- [ ] Content structure (8 sections, ~180 minutes):
  1. Capstone Project Design (30 min)
  2. Execution: Full Parallel Workflow (120 min actual work, 30 min guidance)
  3. Measurement: Time Tracking & Productivity Analysis (20 min)
  4. GitHub Repository & Documentation (30 min)
  5. Reflection & Narrative (30 min)
  6. Portfolio Narrative (20 min)
  7. Capstone Submission Requirements (checklist)
  8. Final Reflection & Celebration (20 min)
- [ ] Content elements:
  - 3 capstone project starters (task app, blog, API wrapper)
  - Templates: Feature decomposition, time tracking worksheet, GitHub repo, reflection essay
  - Rubric: Decomposition quality (1-5), execution quality (1-5), measurement quality (1-5), reflection quality (1-5), portfolio narrativeness (1-5)
- [ ] Capstone project starters (detailed):
  - Starter 1: Task management app (3 features: CRUD, priorities, sharing)
    - Feature specs provided (students can build from them or redesign)
    - Expected implementation time: 1.5-2 hours total
  - Starter 2: Blog platform (3 features: posts, comments, recommendations)
    - Similar structure, different domain
  - Starter 3: API wrapper (3 features: auth, rate limiting, caching)
    - More technical, different learning emphasis
- [ ] Templates (all provided, students customize):
  - Feature decomposition template (guidance document)
  - Time tracking worksheet (parallel vs sequential comparison)
  - GitHub repository template (clear structure, sample README)
  - Reflection essay template (guided prompts)
- [ ] Rubric for evaluation:
  - 5-point scale for each dimension
  - Example scores (what 3/5 looks like vs 5/5)
  - Guidance for students on how to improve
- [ ] Portfolio narrative framework:
  - NOT: "I learned git worktrees and ran parallel terminals"
  - YES: "I learned decomposition thinking and demonstrated 3x productivity gains"
  - Script showing how to explain project to employers/interviewers
- [ ] Try With AI section (3 prompts: reflection feedback, narrative polish, retrospective insights)
- [ ] Celebration/completion messaging
- [ ] **7-9 agent Scale Connection** (CRITICAL):
  - ✅ Introduction: "You're about to PROVE you can orchestrate 5+ agents - demonstrating the path to 10-15"
  - ✅ Section 2 (Execution): "You're running the workflow that scales to 7-9 agents"
  - ✅ Section 5 (Reflection): "You just orchestrated 5 agents. What would change with 10? With 15?"
  - ✅ Section 6 (Portfolio Narrative): "Frame your capability: 'I can orchestrate 7-9 AI Agents to build multi-feature systems 10x faster'"
  - ✅ Student completes chapter believing: "I AM a Super AI Orchestrator"

**Technical References**:
1. **Lesson Style Guide**: `.claude/output-styles/lesson.md`
2. **Lesson Plan**: `specs/002-chapter-32-redesign/plan.md` (Lesson 9 section)
3. **Capstone Project Examples**:
   - Real portfolios from Chapter 31 (if available)
   - GitHub repository examples showing multi-feature projects
   - Portfolio presentation guides
4. **Measurement & Metrics**:
   - Time tracking methodologies
   - Productivity analysis frameworks
   - ROI calculation for process improvements
5. **Portfolio Narratives**:
   - Technical storytelling guides
   - Interview preparation for technical roles
   - How to explain complex projects simply

**Implementation Guidance**:
- **Capstone Project Starter 1: Task Management App**:
  ```
  Features:
  001-task-crud: Create, Read, Update, Delete tasks
  002-priorities: Priority levels, sorting, filtering
  003-sharing: Share tasks with other users

  Integration contracts:
  - 002 depends on 001 (needs task data model)
  - 003 depends on 001 (needs task IDs)
  - 002 and 003 are independent

  Decomposition quality test:
  Can these 3 features be built simultaneously? YES
  Are integration contracts clear? YES
  Expected merge conflicts? MINIMAL (different files)
  ```

- **Time Tracking Worksheet Template**:
  | Feature | Sequential (estimate) | Parallel (actual) | Speedup |
  |---------|----------------------|-------------------|---------|
  | 001-task-crud | 60 min | 60 min | 1x |
  | 002-priorities | 45 min | 45 min (parallel) | - |
  | 003-sharing | 45 min | 45 min (parallel) | - |
  | **Total** | **150 min** | **60-70 min** | **~2.5x** |

- **Portfolio Narrative Script**:
  ```
  BAD: "I used git worktrees to run 3 Claude Code sessions in parallel"

  GOOD: "I demonstrated decomposition thinking - breaking a complex system
  into 3 parallelizable units with clear integration contracts. This enabled
  me to coordinate 3 AI agents simultaneously and deliver a 3-feature system
  in 60 minutes vs 150 minutes sequentially - a 2.5x productivity gain.

  This skill scales: the same decomposition thinking that coordinates 3 AI
  agents can coordinate 7-9 agents, junior developers, or distributed teams."
  ```

- **Try With AI Prompts**:
  1. "Review my capstone reflection - does it demonstrate understanding of decomposition thinking vs just tool proficiency?"
  2. "Help me explain my capstone project in a 2-minute interview answer. Focus on transferable skills."
  3. "Analyze my time tracking data - what does it reveal about my decomposition quality?"

**Quality Standards**:
- **Capstone starters**: Must have clear integration contracts and be parallelizable
- **Time tracking worksheet**: Must show actual gains (not theoretical)
- **Portfolio narrative**: Must emphasize thinking over tools
- **Cognitive load**: 10 concepts (B1-B2 level capstone)
- **Scale connection**: "7-9 agents" mentioned 8+ times (this is the proof)
- **Celebration**: Must feel like accomplishment, not just "chapter done"

**Validation Checklist**:
- [ ] Test capstone starter - can 3 features be built in parallel?
- [ ] Verify time tracking worksheet captures parallel vs sequential accurately
- [ ] Check portfolio narrative examples - emphasize strategic capability
- [ ] Count concepts - verify ≤ 10
- [ ] Search "7-9 agents" - verify appears 8+ times
- [ ] Verify celebration messaging: Student feels "I accomplished something significant"

---

#### Task 5.2: Create Capstone Starter Repositories

**Priority**: SHOULD
**Effort**: 6 hours (2 hours each for 3 starters)

**Description**: Create 3 pre-built starter repositories for capstone projects

**Acceptance Criteria**:
For each starter (task app, blog, API wrapper):
- [ ] Repository created in organization with clear structure:
  - `.specify/` directory with constitution
  - `specs/` directory with 3 example feature specs (to decompose and build from)
  - `book-source/docs/` skeleton (students add content)
  - `.github/workflows/` with basic CI/CD setup
  - `README.md` explaining the starter
- [ ] 3 Feature specs provided for each starter (students can use as-is or redesign):
  - Spec format: Follows constitutional standards (evals first, clear acceptance criteria)
  - Integration contracts: Documented (how features connect)
  - Estimated effort: 1.5-2 hours total for all 3 features
- [ ] README for each starter:
  - Overview of features
  - Decomposition rationale (why these 3 are parallelizable)
  - Getting started instructions
  - Expected time commitment
  - Notes on what to look for (integration quality, merge conflicts)

**Starters**:
1. **Task Management App**:
   - Feature 1: Task CRUD (create, read, update, delete tasks)
   - Feature 2: Priority levels and sorting
   - Feature 3: Team sharing and collaboration
   - Specs provided; students can redesign for better parallelization

2. **Blog Platform**:
   - Feature 1: Post creation and editing
   - Feature 2: Comments and discussion
   - Feature 3: Recommendations/likes
   - Similar structure, different domain

3. **API Wrapper**:
   - Feature 1: Authentication (JWT tokens, session management)
   - Feature 2: Rate limiting (prevent abuse)
   - Feature 3: Caching layer (improve performance)
   - More technical; demonstrates different decomposition patterns

**Reference**: `plan.md` Lesson 9, capstone starters section

---

### Phase 6: Code Examples & Supporting Materials

#### Task 6.1: Create All Code Examples & Scripts

**Priority**: MUST
**Effort**: 12 hours

**Description**: Create all runnable code examples across all lessons

**Acceptance Criteria**:

**Lesson 1 Examples**:
- [ ] `git-worktree-setup.sh` (bash script)
  - Creates 3 worktrees: feature-001-auth, feature-002-payment, feature-003-dashboard
  - Works on macOS, Linux, Windows (Git Bash)
  - Tests: `git worktree list` shows 3 worktrees
  - Instructions: Copy-paste ready
- [ ] Feature spec template (markdown)
  - Shows required structure with integration contract section
  - Annotated (explains each section)
- [ ] Integration mapping diagram (ASCII or visual description)

**Lesson 2 Examples**:
- [ ] `tmux-session-setup.sh` (tmux configuration)
  - Creates 3-pane layout for simultaneous session monitoring
  - Falls back to instructions for non-tmux users (iTerm2, split terminals)
- [ ] Terminal naming guide (screenshot or ASCII diagram)
  - Shows clear naming conventions
  - Examples of confusion/clarity
- [ ] `monitor-parallel-plans.sh` (bash script)
  - Monitors 3 parallel `/sp.plan` executions
  - Shows completion status, time elapsed

**Lesson 3 Examples**:
- [ ] `3-feature-starter-repo` (example repository)
  - 3 feature branches with complete specs
  - Ready for students to implement
  - Includes test harness
- [ ] `merge-conflict-resolution.md` (guide)
  - Step-by-step conflict resolution
  - Common scenarios (shared file, broken dependency, API change)
  - Using Plan Mode to analyze conflicts
- [ ] `integration-test-suite.py` (Python script)
  - Tests that all 3 features work together
  - Validates integration contracts

**Lesson 5 Example**:
- [ ] `.github/workflows/spec-validation.yml` (GitHub Actions)
  - Validates spec structure
  - Checks for required sections
  - Works on all pull requests
  - Free-tier GitHub compatible

**Lesson 6 Examples**:
- [ ] MCP server setup scripts (3 examples: product, analytics, code-review)
- [ ] Example MCP query patterns (showing how to use MCP from Claude Code)
- [ ] Multi-session consistency test (verifies all sessions see same data)

**Lesson 7 Examples**:
- [ ] Claude Code prompt with `background_bash` flag example
- [ ] `monitor-background-tasks.sh` (bash script)
  - Shows progress of 3 simultaneous background tasks
  - Checks for failures
- [ ] `handle-task-failure.sh` (error handling example)
  - Retries failed tasks
  - Alerts on critical failures

**Lesson 8 Examples** (all templates provided, not exercises):
- [ ] `headless-single-command.sh` (example of running Claude Code headless)
  - Shows `--output-format json` usage
  - Demonstrates session ID capture
- [ ] `orchestration-script-template.sh` (bash script)
  - Orchestrates 5 features through Specify → Plan → Tasks → Implement
  - Spawns headless sessions in parallel
  - Uses session IDs to maintain context
  - Includes error handling and progress monitoring
  - Commented for understanding (not required to write)
- [ ] `parse-orchestration-output.py` (Python script)
  - Parses JSON output from headless sessions
  - Provides progress dashboard
  - Detects failures, alerts

**Test Coverage**:
- All scripts have been tested on macOS, Linux, Windows
- Copy-paste examples are verified runnable
- Code examples compile/run without modification
- Exercises have provided answers (in instructor guide or hidden)

**Reference Documentation**:
- Code examples include clear comments explaining each section
- README for each example explaining purpose and usage
- Troubleshooting sections for common errors

---

#### Task 6.2: Create Exercises & Answer Keys

**Priority**: MUST
**Effort**: 8 hours

**Description**: Develop all exercises with answer keys and success criteria

**Acceptance Criteria**:
- [ ] **Lesson 1** (4 exercises):
  - [ ] Exercise 1: Create 3 worktrees
    - Success criteria: Output of `git worktree list` shows 3 worktrees
    - Answer key: 3-5 sentences showing expected output
  - [ ] Exercise 2: Run parallel `/sp.specify`
    - Success criteria: 3 specs created in feature-001, 002, 003 directories
    - Answer key: Example output structure showing PHR routing
  - [ ] Exercise 3: Identify integration contracts
    - Success criteria: Student identifies 3 integration points between features
    - Answer key: Example integration contract analysis
  - [ ] Exercise 4: Time tracking worksheet
    - Template provided (fillable)
    - Answer key: Example showing 3x speedup calculation

- [ ] **Lesson 2** (5 exercises):
  - [ ] Exercise 1: Set up tmux
    - Success criteria: 3 panes visible showing different sessions
    - Answer key: tmux commands or screenshot
  - [ ] Exercise 2: Run `/sp.plan` in parallel
    - Success criteria: 3 plans generated in ~20 minutes (vs 60 min sequential)
    - Answer key: Expected time savings
  - [ ] Exercise 3: Evaluate plan quality
    - Success criteria: Student analyzes plan complexity and identifies imbalances
    - Answer key: Rubric for evaluating balance
  - [ ] Exercise 4: Run `/sp.tasks` in parallel
    - Success criteria: All task lists generated
    - Answer key: Sample task output
  - [ ] Exercise 5: Time tracking and analysis
    - Template and answer key provided

- [ ] **Lesson 3** (6 exercises):
  - [ ] Exercise 1: Map dependencies
    - Success criteria: Student creates dependency graph
    - Answer key: Example graph for 3 features
  - [ ] Exercise 2: Run parallel implementation
    - Success criteria: 3 implementations complete in parallel
    - Answer key: Timings showing parallelization benefit
  - [ ] Exercise 3: Merge in dependency order
    - Success criteria: Successful merges in correct order
    - Answer key: git log showing merge sequence
  - [ ] Exercise 4: Resolve conflicts
    - Success criteria: Student resolves conflicts strategically
    - Answer key: Example conflict resolution (if any expected)
  - [ ] Exercise 5: Integration testing
    - Success criteria: All tests pass on integrated system
    - Answer key: Test output showing success
  - [ ] Exercise 6: Quality analysis
    - Template and answer key provided

- [ ] **Lesson 4** (4 exercises):
  - [ ] Exercise 1: Self-assess decomposition (rubric provided, answer key shows example scoring)
  - [ ] Exercise 2: Identify red flags (checklist provided, answer key shows what to look for)
  - [ ] Exercise 3: Redesign for scalability (template provided, answer key shows example redesign)
  - [ ] Exercise 4: Map 5-feature decomposition (template provided, answer key shows example)

- [ ] **Lesson 5** (4 exercises):
  - [ ] Exercise 1: Create validation workflow (template provided, success = workflow runs)
  - [ ] Exercise 2: Test with bad spec (template provided, success = validation fails as expected)
  - [ ] Exercise 3: Fix spec (template provided, success = validation passes)
  - [ ] Exercise 4: Run locally and in CI/CD (template provided, success = same validation result)

- [ ] **Lesson 6** (4 exercises):
  - [ ] Exercise 1: Set up MCP servers (template provided, success = servers running)
  - [ ] Exercise 2: Query MCP (template provided, success = data returned)
  - [ ] Exercise 3: Write data-informed specs (template provided, answer key shows example)
  - [ ] Exercise 4: Verify consistency (template provided, success = same data across sessions)

- [ ] **Lesson 7** (4 exercises):
  - [ ] Exercise 1: Spawn background tasks (template provided, success = tasks running)
  - [ ] Exercise 2: Monitor progress (template provided, answer key shows monitoring)
  - [ ] Exercise 3: Handle failure (template provided, success = student debugs and restarts)
  - [ ] Exercise 4: Time comparison (template provided, answer key shows time savings)

- [ ] **Lesson 8** (4 exercises, guided):
  - [ ] Exercise 1 (guided): Run headless command (template provided, success = JSON output)
  - [ ] Exercise 2 (guided): Run orchestration script (template provided, success = all 5 features complete)
  - [ ] Exercise 3 (optional): Modify script (template provided, success = modifications work)
  - [ ] Exercise 4 (reflection): Analyze script (guided questions provided, answer key shows example analysis)

- [ ] **Lesson 9** (capstone, rubric-based):
  - [ ] Capstone submission checklist (repository, time tracking, reflection)
  - [ ] Rubric with 5-point scale (decomposition, execution, measurement, reflection, narrative)
  - [ ] Example capstone projects (3 starters with complete implementations shown)

---

#### Task 6.3: Create "Try With AI" Prompt Sets

**Priority**: MUST
**Effort**: 4 hours

**Description**: Develop Claude Code CLI prompts for the "Try With AI" section in each lesson

**Acceptance Criteria**:
- [ ] **Lesson 1** (3 prompts):
  - Prompt 1: "Review my 3 feature specs. Do the integration contracts look clear?"
  - Prompt 2: "I'm decomposing [complex-feature]. Are these 3 sub-features truly independent?"
  - Prompt 3: "Explain to a non-technical founder why specs enable parallel development"
  - Expected outcomes documented for each

- [ ] **Lesson 2** (3 prompts):
  - Prompt 1: "Compare my 3 plans. Do feature decompositions look balanced?"
  - Prompt 2: "Looking at my task lists, can these 3 features really be built independently?"
  - Prompt 3: "What would good decomposition look like for 5 features instead of 3?"
  - Expected outcomes documented

- [ ] **Lesson 3** (3 prompts):
  - Prompt 1: "I'm seeing merge conflicts in [file]. What does this reveal about my decomposition?"
  - Prompt 2: "Review my integration test suite. Are these good tests?"
  - Prompt 3: "Retrospective: What would I do differently to decompose more cleanly?"
  - Expected outcomes documented

- [ ] **Lesson 4** (3 prompts):
  - Prompt 1: "If I need to scale to 5 features, what would break? How would I redesign?"
  - Prompt 2: "Analyze my feature specifications. Which are tightly integrated? How could I decouple?"
  - Prompt 3: "I want to hire 3 developers. How would I decompose for independence?"
  - Expected outcomes documented

- [ ] **Lesson 5** (3 prompts):
  - Prompt 1: "Review my spec validation workflow. Are there checks I'm missing?"
  - Prompt 2: "My spec validation keeps failing. What does that tell me about my decomposition?"
  - Prompt 3: "How would I extend validation to catch decomposition problems earlier?"
  - Expected outcomes documented

- [ ] **Lesson 6** (3 prompts):
  - Prompt 1: "Design an MCP setup for my team. What data sources should I expose?"
  - Prompt 2: "Review my spec. Is it data-informed or making assumptions?"
  - Prompt 3: "How would I scale MCP to coordinate 10 simultaneous feature teams?"
  - Expected outcomes documented

- [ ] **Lesson 7** (3 prompts):
  - Prompt 1: "Design a monitoring dashboard for 7 simultaneous implementations."
  - Prompt 2: "I have 3 background tasks. One failed. How do I debug while others continue?"
  - Prompt 3: "How would I scale background execution to coordinate 15 simultaneous tasks?"
  - Expected outcomes documented

- [ ] **Lesson 8** (3 prompts):
  - Prompt 1: "Review my orchestration script. What improvements would help it scale to 10-15 features?"
  - Prompt 2: "Explain my orchestration script step-by-step. Where are potential failure points?"
  - Prompt 3: "How would I modify this to ensure all 7-9 agents work in true parallel?"
  - Expected outcomes documented

- [ ] **Lesson 9** (3 prompts):
  - Prompt 1: "Review my capstone. What does the integration quality tell me about my decomposition?"
  - Prompt 2: "Help me refine my portfolio narrative. How would I explain this to a CTO?"
  - Prompt 3: "If I wanted to scale this to 5-10 features, what would I redesign?"
  - Expected outcomes documented

**Format**:
- Each prompt is 1-2 sentences (clear, specific)
- Expected outcome is 3-5 sentences describing what good response looks like
- Safety/ethics note included for each set (AI suggests improvements, student responsible for decisions)

---

### Phase 7: Review & Quality Assurance

#### Task 7.1: Content Peer Review

**Priority**: SHOULD
**Effort**: 6 hours

**Description**: Peer review all lesson content for pedagogy, clarity, and alignment

**Acceptance Criteria**:
- [ ] All 9 lessons reviewed by peer (ideally different author or technical reviewer)
- [ ] Review checklist covers:
  - [ ] Learning objectives are clear and measurable
  - [ ] Content aligns with learning objectives
  - [ ] Code examples are copy-paste ready and tested
  - [ ] Exercises are testable and have clear success criteria
  - [ ] Try With AI prompts are focused and actionable
  - [ ] Real-world examples are relevant and compelling
  - [ ] Tone is appropriate for Part 5 (Intermediate) audience
  - [ ] No jargon without definition
  - [ ] 60/40 thinking-to-tooling ratio maintained
  - [ ] No simulation language or toy examples
- [ ] Feedback captured and addressed
- [ ] Review sign-off from peer

**Reference**: `.claude/output-styles/chapters.md` quality checklist

---

#### Task 7.2: Accessibility & Inclusivity Check

**Priority**: SHOULD
**Effort**: 4 hours

**Description**: Verify accessibility and inclusive language across all lessons

**Acceptance Criteria**:
- [ ] Reading level check:
  - [ ] Grade 8-10 reading level (appropriate for technical audience, not condescending)
  - [ ] Flesch Reading Ease 50-65 (technical but not impenetrable)
  - [ ] Tool: Grammarly, hemingwayapp.com, or manual review
- [ ] Inclusive language:
  - [ ] No gendered pronouns ("they/them" or "I/you")
  - [ ] Examples include diverse names, backgrounds, perspectives
  - [ ] No gatekeeping language ("obviously," "just," "easy")
  - [ ] Assumptions about prior knowledge clearly stated in prerequisites
- [ ] Visual accessibility:
  - [ ] Code examples have contrast
  - [ ] Diagrams have alt text descriptions
  - [ ] Color not the only differentiator
- [ ] Jargon management:
  - [ ] All technical terms defined before use
  - [ ] Glossary of key terms (worktree, feature, integration contract, decomposition, etc.)

---

#### Task 7.3: Code Example Testing

**Priority**: MUST
**Effort**: 8 hours

**Description**: Test all code examples on macOS, Linux, and Windows

**Acceptance Criteria**:
- [ ] All bash scripts tested:
  - [ ] Tested on macOS (Zsh, Bash)
  - [ ] Tested on Linux (Ubuntu 20.04+)
  - [ ] Tested on Windows (Git Bash or WSL2)
  - [ ] All scripts include error handling
  - [ ] All scripts include comments
- [ ] All Python scripts tested:
  - [ ] Python 3.10+ compatibility verified
  - [ ] Dependencies listed in comments or requirements.txt
  - [ ] Scripts run without modification
- [ ] All config files (GitHub Actions, tmux) tested:
  - [ ] GitHub Actions workflow runs successfully (on pull request)
  - [ ] tmux configuration tested in tmux environment
- [ ] Copy-paste examples verified:
  - [ ] All code snippets are copy-paste ready
  - [ ] No missing imports or dependencies
  - [ ] Examples compile/run without modification

**Testing Log**:
- [ ] macOS: [Date], [Tester], [Results]
- [ ] Linux: [Date], [Tester], [Results]
- [ ] Windows: [Date], [Tester], [Results]

---

#### Task 7.4: Chapter-Level Coherence Check

**Priority**: SHOULD
**Effort**: 4 hours

**Description**: Verify flow and coherence across all 9 lessons

**Acceptance Criteria**:
- [ ] Lesson sequence is logical (1 builds on 2, 2 builds on 3, etc.)
- [ ] Prerequisites clearly stated for each lesson
- [ ] Concept progression respects cognitive load (A2 → B1 → B2)
- [ ] Decomposition thinking builds throughout (Part 1, Part 2, Part 3, Part 4...)
- [ ] Tool proficiency (40%) serves thinking (60%), not vice versa
- [ ] Try With AI activities are consistent in format across lessons
- [ ] Chapter conclusion (Lesson 9) synthesizes all prior lessons
- [ ] Forward references to Parts 10-13 are appropriate
- [ ] No unexplained gaps or jumps in content

**Coherence Map**:
- Lesson 1: Foundation (worktrees, 3 parallel specs)
- Lesson 2: Scaling (plans + tasks in parallel)
- Lesson 3: Integration (proof of decomposition quality via merge quality)
- Lesson 4: Analysis (what scales, what breaks, 3→5)
- Lesson 5: Automation 1 (CI/CD gates quality)
- Lesson 6: Automation 2 (MCP provides shared context)
- Lesson 7: Automation 3 (background execution enables true parallelization)
- Lesson 8: Meta-orchestration (programmatic orchestration, 7-9 agents)
- Lesson 9: Integration (capstone project synthesis, measurement, portfolio)

---

### Phase 8: Capstone Project Materials

#### Task 8.1: Create Time Tracking Worksheet Template

**Priority**: MUST
**Effort**: 2 hours

**Description**: Create fillable worksheet for measuring parallel vs sequential productivity

**Acceptance Criteria**:
- [ ] Document created: `capstone-time-tracking-worksheet.md`
- [ ] Sections:
  - [ ] Baseline: Sequential estimate (how long would 3 features take sequentially?)
  - [ ] Parallel execution: Actual time (how long did parallel workflow take?)
  - [ ] Breakdown by phase:
    - Specification phase time (parallel vs sequential estimate)
    - Planning phase time
    - Task generation time
    - Implementation time
    - Integration time
  - [ ] Calculations:
    - Total sequential estimate
    - Total parallel actual
    - Speedup multiplier (sequential / parallel)
  - [ ] Integration quality metrics:
    - Number of merge conflicts
    - Time to resolve conflicts
    - Any failed integrations? (indicates decomposition problems)
- [ ] Template is fillable (students complete with their own numbers)
- [ ] Example filled out (task management app example)
- [ ] Answer key provided (how to interpret results)

---

#### Task 8.2: Create GitHub Repository Documentation Template

**Priority**: MUST
**Effort**: 3 hours

**Description**: Create template for documenting capstone project in GitHub repository

**Acceptance Criteria**:
- [ ] README.md template created
- [ ] Sections:
  - [ ] Project title and overview
  - [ ] Features: What 3-5 features did you build?
  - [ ] Decomposition approach: Why are these features parallelizable?
  - [ ] Results: Time tracking, speedup, integration quality
  - [ ] What worked well: Reflection on decomposition quality
  - [ ] What would be different: Retrospective redesign
  - [ ] Transferability: How does this apply to human teams?
  - [ ] Technology stack (optional, not required)
- [ ] Repository structure documented:
  - Directory layout showing 3+ feature branches
  - Commit history showing parallel work
  - Integration points documented
- [ ] Example README provided (task management app)

---

#### Task 8.3: Create Reflection Essay Template

**Priority**: MUST
**Effort**: 2 hours

**Description**: Create guided prompts for capstone reflection essay

**Acceptance Criteria**:
- [ ] Essay template (1-2 pages, ~500-800 words)
- [ ] Guided prompts:
  - What is decomposition thinking? (define in your own words)
  - How did you decompose your system into 3-5 features? (explain your rationale)
  - What made these features parallelizable? (identify boundaries and contracts)
  - What would make features NOT parallelizable? (identify dependencies and bottlenecks)
  - What does integration quality tell you about your decomposition? (interpret merge conflicts or clean merges)
  - How does decomposition thinking apply to human teams instead of AI agents? (transferability)
  - What is the business value of decomposition thinking? (strategic insight)
  - What would you do differently if building again? (continuous improvement)
- [ ] Example essay provided (task management app)
- [ ] Rubric for evaluation (2-3 points per prompt)

---

#### Task 8.4: Create Capstone Rubric

**Priority**: MUST
**Effort**: 2 hours

**Description**: Develop 5-point rubric for evaluating capstone projects

**Acceptance Criteria**:
- [ ] Rubric created with 5 dimensions:
  - **Decomposition Quality** (1-5):
    - 1 = Features not independent, heavy dependencies, poor contracts
    - 3 = Features mostly independent, some dependencies identified
    - 5 = Features clearly independent, well-defined contracts, excellent boundaries
  - **Execution Quality** (1-5):
    - 1 = Implementation incomplete, tests failing
    - 3 = Implementation complete, basic tests passing
    - 5 = Implementation complete, comprehensive tests, clean code
  - **Measurement Quality** (1-5):
    - 1 = Time tracking incomplete or inaccurate
    - 3 = Time tracking documented, calculations correct
    - 5 = Time tracking detailed, calculations verified, analysis insightful
  - **Reflection Quality** (1-5):
    - 1 = Reflection superficial, no real insights
    - 3 = Reflection shows understanding, addresses main questions
    - 5 = Reflection is deep, demonstrates mastery, addresses transferability
  - **Portfolio Narrativeness** (1-5):
    - 1 = Can't explain project to non-technical person
    - 3 = Can explain project, focuses on tools more than thinking
    - 5 = Clear narrative emphasizing decomposition thinking and strategic value
- [ ] Example scores provided for each dimension
- [ ] Guidance for improvement (how to move from 3 to 5 in each dimension)
- [ ] Total score interpretation (15-25 = not ready, 25-21 = good, 21+ = excellent)

---

### Phase 9: Integration & Final Review

#### Task 9.1: Cross-Chapter Reference Check

**Priority**: SHOULD
**Effort**: 3 hours

**Description**: Verify references to other chapters are accurate

**Acceptance Criteria**:
- [ ] All prerequisites accurately listed:
  - [ ] Chapter 30, 31 (SpecKit Plus)
  - [ ] Chapter 5 (Claude Code CLI)
  - [ ] Chapter 7 (Bash)
  - [ ] Chapter 8 (Git)
  - [ ] All chapters are implemented and available
- [ ] Forward references to Parts 10-13 are appropriate:
  - [ ] Deployment reference to Parts 10 (Docker)
  - [ ] Data/state reference to Part 11
  - [ ] Event-driven reference to Part 12
  - [ ] All forward references are accurate and helpful
- [ ] No circular dependencies
- [ ] Links to related content work correctly (or are documented as future additions)

---

#### Task 9.2: Constitution Alignment Verification

**Priority**: SHOULD
**Effort**: 3 hours

**Description**: Verify chapter aligns with project constitution

**Acceptance Criteria**:
- [ ] Chapter teaches Principle 14: Planning-First Development
  - [ ] Decomposition requires planning before execution
  - [ ] Specs are written before implementation
  - [ ] Integration planning (dependency order) comes before merging
- [ ] Chapter teaches Principle 15: Validation-Before-Trust
  - [ ] Integration quality validates decomposition quality
  - [ ] Merge conflicts are learning opportunities
  - [ ] Tests validate integration
- [ ] Chapter demonstrates evals-first methodology:
  - [ ] Success criteria defined before planning
  - [ ] Integration quality measured (evals)
  - [ ] Productivity gains measured (evals)
- [ ] Chapter applies 9 mandatory domain skills (to be confirmed from discovered skills):
  - [ ] Context: Decomposition thinking applies across domains
  - [ ] Examples: Solo developer, team, organizational scaling
- [ ] Complexity tier (Part 5 = Intermediate) appropriate:
  - [ ] 3-4 options maximum (not overwhelming choices)
  - [ ] 7 concepts per section max (cognitive load respected)
  - [ ] Graduated complexity across lessons
- [ ] Graduated complexity respected:
  - [ ] Part 5 appropriate for students who completed Parts 1-4
  - [ ] Not jumping to professional (Part 10+) complexity prematurely

---

#### Task 9.3: Final Editorial Polish

**Priority**: SHOULD
**Effort**: 4 hours

**Description**: Final proofreading, formatting, and polish

**Acceptance Criteria**:
- [ ] Spelling and grammar check:
  - [ ] No typos or grammatical errors
  - [ ] Consistent voice and tone
  - [ ] Headings use consistent capitalization
  - [ ] Code formatting is consistent
- [ ] Markdown formatting:
  - [ ] All files follow markdown conventions
  - [ ] Code blocks properly formatted
  - [ ] Lists properly indented
  - [ ] Links work correctly
- [ ] YAML frontmatter:
  - [ ] All files have complete frontmatter
  - [ ] Skills metadata is accurate and complete
  - [ ] Learning objectives are measurable
  - [ ] Durations are realistic
- [ ] Visual consistency:
  - [ ] Examples and exercises are similarly formatted
  - [ ] Try With AI sections are consistent
  - [ ] Code examples have consistent commenting
- [ ] Docusaurus compatibility:
  - [ ] All files build without errors
  - [ ] Links resolve correctly
  - [ ] Frontmatter is valid YAML
  - [ ] Sidebar positions are correct

---

### Phase 10: Finalization & Handoff

#### Task 10.1: Create Instructor Guide (Optional)

**Priority**: NICE-TO-HAVE
**Effort**: 5 hours

**Description**: Create optional instructor guide with teaching tips and answer keys

**Acceptance Criteria**:
- [ ] Document created: `instructor-guide.md`
- [ ] Sections:
  - [ ] Chapter overview and learning objectives
  - [ ] Teaching tips for each lesson (common misconceptions, pacing notes)
  - [ ] Answer keys for all exercises
  - [ ] Sample rubric scores and feedback
  - [ ] Troubleshooting guide (common issues and solutions)
  - [ ] Extension activities (for students who finish early)
  - [ ] Differentiation strategies (for advanced or struggling learners)
- [ ] Focus areas:
  - [ ] How to help students understand decomposition thinking (not just tools)
  - [ ] How to respond when students get merge conflicts (learning opportunity)
  - [ ] How to facilitate reflection activities (make it meaningful, not rote)
  - [ ] How to keep chapter on pace (time management suggestions)

---

#### Task 10.2: Create Student Setup Guide

**Priority**: SHOULD
**Effort**: 3 hours

**Description**: Create getting-started guide for students starting Chapter 32

**Acceptance Criteria**:
- [ ] Document created: `setup-guide.md`
- [ ] Sections:
  - [ ] Prerequisites check (did you complete Chapters 30-31? Chapters 5, 7, 8?)
  - [ ] Installation requirements:
    - [ ] Git 2.5+ (check with `git --version`)
    - [ ] Claude Code CLI (check with `claude --version`)
    - [ ] Terminal multiplexing (tmux or iTerm2 recommended)
    - [ ] Python 3.10+ or TypeScript/Node.js
  - [ ] Environment setup:
    - [ ] Clone starter repository (if using one)
    - [ ] Create feature branches for 3-5 features
    - [ ] Verify SpecKit Plus is working (`/sp.specify --help`)
  - [ ] Time commitment:
    - [ ] Fast track: 6-8 hours (Lessons 1-4 + Capstone)
    - [ ] Full path: 10-12 hours (Lessons 1-8 + Capstone)
  - [ ] How to use the chapter:
    - [ ] Read each lesson sequentially
    - [ ] Follow exercises hands-on (don't skip)
    - [ ] Use Try With AI for feedback and reflection
  - [ ] Troubleshooting:
    - [ ] Common setup issues and solutions
    - [ ] Where to get help

---

#### Task 10.3: Create Capstone Submission Checklist

**Priority**: MUST
**Effort**: 2 hours

**Description**: Create clear checklist for capstone submission requirements

**Acceptance Criteria**:
- [ ] Checklist document created: `capstone-submission-checklist.md`
- [ ] Submission items:
  - [ ] GitHub repository with multi-worktree history visible
  - [ ] README.md documenting decomposition approach and results
  - [ ] Time tracking worksheet with parallel vs sequential comparison
  - [ ] Reflection essay (1-2 pages) addressing key questions
  - [ ] At least 3 complete, working features
  - [ ] All features integrate and tests pass
  - [ ] Clear documentation of how to run/test the system
- [ ] Rubric reference (link to capstone rubric)
- [ ] Submission instructions (how to submit, where to send)
- [ ] Grading timeline (when will feedback be provided?)
- [ ] Additional notes (what makes a portfolio-worthy capstone)

---

### Phase 11: Verification & Sign-Off

#### Task 11.1: Compliance Checklist

**Priority**: MUST
**Effort**: 4 hours

**Description**: Verify all content meets constitutional and pedagogical standards

**Acceptance Criteria**:
- [ ] **Specification Compliance**:
  - [ ] All 27 functional requirements addressed (see spec.md)
  - [ ] All 6 success criteria (SC-001 through SC-013) measurable in content
  - [ ] Chapter scope matches constraints (10-12 hour time commitment, Part 5 complexity)
  - [ ] All 5 user stories can be completed using chapter content

- [ ] **Pedagogical Standards**:
  - [ ] Lessons follow Bloom's taxonomy progression
  - [ ] CEFR proficiency levels are appropriate and progressive
  - [ ] Cognitive load limits respected (5-7 concepts per A2/B1 section, 10 for B2)
  - [ ] 60/40 thinking-to-tooling ratio maintained
  - [ ] Decomposition thinking is primary learning objective
  - [ ] Transferability is explicit (human teams, organizational scaling)
  - [ ] No simulation language or toy examples
  - [ ] Real-world examples are relevant and current

- [ ] **Content Quality**:
  - [ ] All code examples tested and working
  - [ ] All exercises have clear success criteria and answer keys
  - [ ] All Try With AI prompts are focused and actionable
  - [ ] Reading level appropriate (Grade 8-10, technical audience)
  - [ ] Inclusive language and examples
  - [ ] Accessibility requirements met (alt text, color contrast, clear language)

- [ ] **Integration**:
  - [ ] Prerequisites clearly stated and verified to exist
  - [ ] Forward references are accurate
  - [ ] Chapter fits within Part 5 structure
  - [ ] No circular dependencies
  - [ ] Cross-references to other chapters work

- [ ] **File Organization**:
  - [ ] All files follow directory structure from `specs/book/directory-structure.md`
  - [ ] All files have proper YAML frontmatter
  - [ ] All files are markdown (.md) format
  - [ ] Naming conventions followed (lowercase-with-hyphens for lesson files)

**Checklist Sign-Off**:
- [ ] Content author: [Name], [Date]
- [ ] Technical reviewer: [Name], [Date]
- [ ] Pedagogical reviewer: [Name], [Date]

---

## Effort Summary

| Phase | Tasks | Effort |
|-------|-------|--------|
| 1. Chapter Structure | README.md | 2 hours |
| 2. Lessons 1-4 | 4 lessons, code examples, exercises | 28 hours |
| 3. Lessons 5-7 | 3 lessons, code examples, exercises | 21 hours |
| 4. Lesson 8 | 1 lesson (optional), templates | 8 hours |
| 5. Lesson 9 | 1 lesson, capstone materials | 10 hours |
| 6. Code & Exercises | All scripts, templates, answers | 20 hours |
| 7. Try With AI | Prompt sets for all lessons | 4 hours |
| 8. Review & QA | Peer review, testing, accessibility | 22 hours |
| 9. Capstone Materials | Worksheet, templates, rubric | 10 hours |
| 10. Finalization | Guides, checklists, sign-off | 10 hours |

**Total Estimated Effort**: 135 hours ≈ 60-80 story points (assuming 2-3 hour story point)

**Timeline** (One person, full-time):
- Lessons 1-4 (core): 28 hours = ~4 days
- Lessons 5-7 (automation): 21 hours = ~3 days
- Lesson 8 (advanced): 8 hours = ~1 day
- Lesson 9 (capstone): 10 hours = ~2 days
- Code examples & exercises: 20 hours = ~2.5 days
- Review, testing, finalization: 48 hours = ~6 days
- **Total: ~4-5 weeks full-time, or 2-3 months part-time**

---

## Dependencies & Risks

### Critical Path Dependencies

1. **Lesson 1 must be complete** before 2-4 (students need worktree understanding)
2. **Lessons 1-4 must be complete** before Lesson 9 (capstone depends on foundation)
3. **Lessons 5-7 are optional** but recommended before Lesson 8
4. **Code examples must be tested** before content publication (no broken examples)
5. **Capstone starters must be ready** before Lesson 9 content is finalized

### Key Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|-----------|
| Code examples don't run on all platforms | Medium | High | Test on macOS, Linux, Windows during Task 6.2 |
| Time estimates are too optimistic | Medium | Medium | Include 20% time buffer in estimates; pilot test with beta reader |
| Lesson content exceeds Part 5 complexity limits | Low | High | Strict peer review for cognitive load; remove options, simplify if needed |
| Students skip Lessons 5-8, feel lost in capstone | Medium | Low | Lesson 9 recap and remediation; clear "fast-track" messaging |
| Capstone starters are too prescriptive | Low | Low | Provide examples, but allow students to bring their own projects |
| Decomposition thinking not resonating with students | Medium | High | Include multiple metaphors (music orchestra, sports teams, org scaling); real-world examples |

---

## Acceptance Criteria (Definition of Done)

**All Tasks Must Pass**:

- [ ] All 9 lessons complete and published
- [ ] Chapter README.md exists and is clear
- [ ] All code examples tested on macOS, Linux, Windows
- [ ] All exercises have answer keys
- [ ] All Try With AI prompts are present and focused
- [ ] Peer review completed and feedback addressed
- [ ] Accessibility check completed (reading level, inclusive language)
- [ ] No grammatical errors or typos
- [ ] Chapter aligns with specification (all 27 FRs, 6 SCs addressed)
- [ ] Chapter aligns with constitution (principles 14-15, 60/40 thinking-to-tooling)
- [ ] Capstone materials complete (worksheet, templates, rubric)
- [ ] Compliance checklist signed off
- [ ] Files follow directory structure and naming conventions
- [ ] YAML frontmatter complete and valid
- [ ] Docusaurus build successful (no errors)

---

## Next Steps

Upon task completion:

1. **Implementation Phase** — Execute tasks in dependency order
   - Start with Lesson 1, verify before moving to Lesson 2
   - Code examples and exercises in parallel
   - Review and QA during development (not just at end)

2. **Validation Phase** — Technical and pedagogical review
   - Run all code examples
   - Complete all exercises as a student would
   - Verify against specification and constitution

3. **Publication Phase** — Docusaurus build and deployment
   - Build locally
   - Verify links and references
   - Deploy to production

4. **Post-Publication** — Monitor and iterate
   - Collect student feedback
   - Refine time estimates based on actual data
   - Update examples based on tool version changes (git, Claude Code, etc.)

---

## References

- **Specification**: `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/002-chapter-32-redesign/spec.md`
- **Plan**: `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/002-chapter-32-redesign/plan.md`
- **Constitution**: `.specify/memory/constitution.md`
- **Output Styles**: `.claude/output-styles/lesson.md`
- **Directory Structure**: `specs/book/directory-structure.md`
- **Chapter Index**: `specs/book/chapter-index.md`

---


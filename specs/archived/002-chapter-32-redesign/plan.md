# Chapter 32: Building Projects with Spec-Kit Plus — Implementation Plan

**Feature Branch**: `002-chapter-32-redesign`
**Chapter Type**: Technical/Hybrid (Code-focused with transferability emphasis)
**Part**: Part 5: Spec-Driven Development
**Status**: Planning Phase Complete — Ready for Implementation

---

## Executive Summary

This implementation plan transforms Chapter 32 from toy simulation to **two practical learning goals** that create "creative orchestrators" who coordinate 10-15 autonomous agents (AI or human) through decomposition thinking.

### Goal 1: Master Decomposition Thinking & Manual Agent Team Management (60% Emphasis)

**What Students Learn**: Break complex problems into parallelizable units, then manage 5+ SpecKit Plus workflows as "agent teams."

**How They Learn**: Act as "team lead"—use 5 terminals/worktrees to orchestrate Claude Code sessions running `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` in parallel.

**Student Belief After Goal 1**: *"I can manage agent teams like human ones, with specs eliminating coordination chaos."* (2.5-3x speedup proven)

### Goal 2: Achieve Creative Independence with Super AI Orchestrator (40% Emphasis, OPTIONAL)

**What Students Learn**: Automate Goal 1's manual orchestration into a programmatic "Super AI Orchestrator" script using headless mode (`claude -p`) and sandboxing.

**How They Learn**: Build bash/Python script that spawns 10-15 independent Claude sessions (each with own worktree + MCP + agents), freeing humans for strategic/creative work.

**Student Belief After Goal 2**: *"I can offload management to automation, scaling effortlessly to 15+ agents."* (10x+ productivity)

**Note**: Goal 2 is **optional for advanced students**. Fast-track: Goal 1 + Capstone = 6-8 hours, core mastery demonstrated.

### Implementation Focus: Creative Independence + Upfront Investment Honesty

**Reality Check for Implementation**:
- First decomposition takes **2-3x longer** than "just start coding" (honest expectation setting in Lesson 1)
- Payoff is **10x overall gains** (speedup + quality + scalability)
- Students who skip decomposition: Fast start, slow finish (merge conflicts, rework)
- Students who invest in decomposition: Slow start, fast finish (clean integration, reusable patterns)

**Graduate Identity**: Students become **"creative orchestrators"**—practical for PMs/founders who decompose once, automate execution, iterate creatively.

**Critical Principle for Lesson Design**: Decomposition quality is **the bottleneck**, not tools. Poor decomposition → chaos at 3 agents. Excellent decomposition → scalable to 15 agents. Tools (headless mode, orchestrator scripts) **amplify** decomposition quality—they don't fix it.

**Two-Climax Learning Progression**:
- **Lessons 1-3**: Foundation - Manual parallel SpecKit Plus workflows (2-3 worktrees, "team lead" role)
- **Lesson 4**: **FIRST CLIMAX** - Scale to 5+ MANUAL parallel workflows (experience decomposition thinking)
- **Lessons 5-7 (OPTIONAL)**: Automation tools - CI/CD, headless mode, sandboxing (enable programmatic scale)
- **Lesson 8**: **SECOND CLIMAX** - Build Super Orchestrator (shift to creative independence)
- **Lesson 9**: Capstone - Demonstrate mastery (manual OR programmatic, reflect on upfront investment vs gains)

**Key Understanding**: Students already learned Claude Code (Ch 5) and SpecKit Plus (Ch 31). This chapter teaches **running SpecKit Plus workflows IN PARALLEL** to master decomposition thinking—the skill enabling 10-15 autonomous workflows.

---

## Lesson Architecture (9 Lessons)

### Lesson 1: Multi-Session Development Fundamentals — Git Worktrees & Parallel Specifications

**Duration**: 1.5 hours
**Target Audience**: Part 5 (Intermediate) students who completed Chapter 31

**Learning Objective**: Students can set up 3 git worktrees and run parallel SpecKit Plus workflows to generate 3 feature specifications without conflicts, understanding WHY parallel specification definition accelerates decomposition thinking

**Skills Taught** (CEFR Proficiency Framework):
- **Git Worktrees Proficiency** — A2 (Elementary) — Technical — Students can create, list, and navigate multiple worktrees; understand worktree isolation mechanism; perform basic worktree operations
- **Parallel Specification Design** — B1 (Upper-Intermediate) — Technical — Students can write specifications for independent features simultaneously; understand integration contracts; coordinate via shared constitution
- **Decomposition Thinking: Part 1** — A2 (Elementary) — Conceptual — Students can recognize what makes features "parallelizable"; understand that clear boundaries enable autonomous work

**Key Concepts** (max 7 for A2/B1):
1. What is a git worktree? (separate working directory, shared git history)
2. Why parallelization matters (3x speedup at 3 features)
3. Isolation benefits (no file conflicts, independent branches)
4. SpecKit Plus workflow in parallel contexts
5. Feature numbering and naming conventions (001-feature-name)
6. Integration contracts (how specs prevent merge chaos)
7. Time tracking mindset (baseline for later comparison)

**Prerequisites**:
- Chapter 31: SpecKit Plus Hands-On (students understand `/sp.specify`, `/sp.plan`, `/sp.tasks`)
- Chapter 8: Git & GitHub (students understand branching, merging)
- Chapter 7: Bash Essentials (terminal comfort)
- Terminal multiplexing tool (tmux, iTerm2 split panes, or multiple windows)

**Content Outline**:

1. **What is a Worktree?** (5 min)
   - Concept: A separate working directory with own file state, shared git history
   - Why it matters: Enables true parallel development without cross-contamination
   - Analogy: Three parallel desks in same library, same books but separate papers

2. **Setting Up Your First 3 Worktrees** (20 min)
   - Step-by-step: `git worktree add ../feature-001-auth main`
   - Verify setup: `git worktree list`, directory structure
   - Open 3 terminal sessions in each worktree
   - Verify isolation: Changes in one worktree don't affect others

3. **Running Parallel Specifications** (30 min)
   - Session 1: Run `/sp.specify` for feature 001-authentication
   - Session 2 (simultaneously): Run `/sp.specify` for feature 002-payment
   - Session 3 (simultaneously): Run `/sp.specify` for feature 003-dashboard
   - Observe: PHRs auto-route to feature-specific directories (history/prompts/001-*, 002-*, 003-*)
   - Observe: Feature numbers auto-increment without conflicts
   - Key insight: Parallelization saved ~20 minutes vs sequential (30 min total vs 90 min sequential)

4. **Understanding Integration Contracts** (20 min)
   - What specs define: Input/output contracts, data formats, API boundaries, dependencies
   - Example: Feature 001-auth spec defines user schema that Feature 002-payment depends on
   - Exercise: Identify integration points between your 3 specs (where do they connect?)
   - Why it matters: Good specs = clean merges; bad specs = integration hell

5. **Reflection & Time Tracking** (15 min)
   - Complete time tracking worksheet: How long did 3 specs take in parallel vs how long sequentially?
   - Reflection: What made these features parallelizable? (independent concerns? clear boundaries?)
   - Journaling: "When parallelization adds value" thought exercise

**Content Elements**:
- **Code examples needed**:
  - Example 1: `git worktree setup script` (bash) — creates 3-worktree structure
  - Example 2: Feature spec template (markdown) — shows integration contract structure
  - Example 3: Integration mapping diagram (visual) — shows feature dependencies

- **Exercises**:
  - Exercise 1: Create 3 worktrees in provided starter repository
  - Exercise 2: Run parallel `/sp.specify` in all 3 worktrees
  - Exercise 3: Analyze your 3 specs and identify integration contracts
  - Exercise 4: Time tracking worksheet (parallel vs sequential estimate)

- **Real-world examples**:
  - Vercel engineering: How they decompose features for 50+ engineers
  - Solo founder example: Building 3-feature MVP using parallel development
  - Academic context: How distributed teams coordinate via specs (not sync meetings)

**Try With AI** (End-of-Lesson Activity) — 15 min
- **Tool**: Claude Code CLI (students' preferred AI companion introduced in earlier chapters)
- **Prompts** (2-3):
  1. "Review my 3 feature specs. Do the integration contracts look clear? Will these features merge cleanly?"
  2. "I'm decomposing [your-complex-feature]. Are these 3 sub-features truly independent, or are there hidden dependencies?"
  3. "Explain to a non-technical founder why defining clear specs enables parallel feature development"
- **Expected Outcome**: Students receive feedback on spec quality and integration clarity; understand that AI validates their decomposition thinking
- **Safety/Ethics Note**: AI suggests spec improvements, but student remains responsible for final spec quality and integration decisions

---

### Lesson 2: Scaling to Full Parallel Workflows — Plan & Tasks in Parallel

**Duration**: 1.5 hours

**Learning Objective**: Students can run `/sp.plan` and `/sp.tasks` simultaneously across 3 worktrees, understanding how shared constitution ensures consistent quality while parallelization saves time

**Skills Taught** (CEFR Proficiency Framework):
- **Multi-Session Project Management** — B1 (Upper-Intermediate) — Technical — Students can monitor 3 simultaneous planning/task-generation sessions; handle state across multiple contexts
- **Decomposition Thinking: Part 2** — B1 (Upper-Intermediate) — Conceptual — Students can evaluate decomposition quality based on plan complexity (simple plans = good decomposition; complex plans = poor decomposition)
- **Constitution as Shared Contract** — A2 (Elementary) — Conceptual — Students understand how shared constitution prevents quality drift across parallel work

**Key Concepts** (max 7 for B1):
1. Why plan and tasks in parallel (shared constitution = confidence in parallel execution)
2. Session management across terminals/windows
3. Monitoring 3 simultaneous plan generations
4. Validating plan quality WITHOUT synchronous meetings
5. Identifying decomposition problems (plans that are too large/complex)
6. Confidence in parallel task execution (shared standards)
7. Time savings calculation (scaling from 2-3x to deeper parallelization)

**Prerequisites**:
- Lesson 1 (3 worktrees set up, parallel specs complete)
- Chapter 31: Understanding `/sp.plan` and `/sp.tasks` workflows

**Content Outline**:

1. **Review: Your 3 Specs & Integration Contracts** (10 min)
   - Recap Lesson 1: What makes these specs parallelizable?
   - Diagram: Show how features connect (data flow, API dependencies)

2. **Running Parallel Planning** (25 min)
   - Session 1: Run `/sp.plan` in feature 001 worktree
   - Session 2 (simultaneously): Run `/sp.plan` in feature 002 worktree
   - Session 3 (simultaneously): Run `/sp.plan` in feature 003 worktree
   - Observe: All plans generated in ~20 minutes (vs 60 minutes sequential)
   - Key insight: Shared constitution ensures plans align even without communication

3. **Evaluating Plan Quality as Decomposition Indicator** (15 min)
   - Compare your 3 plans: Are they roughly equal complexity?
   - Lesson: If one plan is much larger/complex, it may indicate poor decomposition
   - Example: If plan-001 is 30 lessons but plan-002 is 5 lessons, why? Hidden complexity?
   - Principle: Good decomposition → balanced, manageable plans

4. **Running Parallel Task Generation** (20 min)
   - Session 1: Run `/sp.tasks` in feature 001 worktree (building on plan-001)
   - Session 2 (simultaneously): Run `/sp.tasks` in feature 002 worktree
   - Session 3 (simultaneously): Run `/sp.tasks` in feature 003 worktree
   - Verify: All tasks generated in parallel, total time ~20 minutes

5. **Terminal Management Best Practices** (10 min)
   - Tmux windowing: Organize 3 sessions in one terminal
   - iTerm2 split panes: Side-by-side session monitoring
   - Naming conventions: Clear labeling to avoid confusion
   - Screenshot/video walkthrough showing actual setup

6. **Reflection: Parallelization Value** (10 min)
   - Complete time tracking: 3 plans + 3 task sets in parallel vs sequential
   - Document actual time saved (should be ~2x faster: 40 min parallel vs 120 min sequential)
   - Journaling: "When does parallel planning add value?"

**Content Elements**:
- **Code examples**:
  - Example 1: tmux configuration script for 3-session layout
  - Example 2: Terminal naming/management guide
  - Example 3: Bash script to monitor parallel plan generation (checks for completion, aggregates results)

- **Exercises**:
  - Exercise 1: Set up tmux/terminal for 3 parallel sessions
  - Exercise 2: Run `/sp.plan` simultaneously in all 3 worktrees
  - Exercise 3: Evaluate plan quality as decomposition indicator
  - Exercise 4: Run `/sp.tasks` in all 3 worktrees
  - Exercise 5: Time tracking and analysis

- **Diagrams**:
  - Timeline showing sequential vs parallel workflow (visual motivation)
  - Terminal layout diagrams (tmux and iTerm2 examples)

**Try With AI** — 15 min
- **Tool**: Claude Code CLI
- **Prompts**:
  1. "Compare my 3 plans. Do the feature decompositions look balanced? Any red flags?"
  2. "Looking at my task lists, can these 3 features really be built independently, or are there hidden dependencies?"
  3. "What would good decomposition look like if I was building a 5-feature system instead of 3?"
- **Expected Outcome**: AI feedback on balance and independence; students refine understanding of what decomposition quality looks like at plan/task level

---

### Lesson 3: Parallel Implementation & Integration — Hands-On Multi-Session Development

**Duration**: 2 hours

**Learning Objective**: Students implement 3 feature branches in parallel, merge them into main, and understand how integration quality (clean merges vs conflicts) validates decomposition quality

**Skills Taught** (CEFR Proficiency Framework):
- **Parallel Implementation Execution** — B1 (Upper-Intermediate) — Technical — Students can run implementations in parallel; manage state across multiple sessions; handle non-blocking execution
- **Integration & Conflict Resolution** — B1 (Upper-Intermediate) — Technical — Students can merge parallel branches; resolve conflicts strategically; validate integration quality
- **Decomposition Thinking: Part 3** — B1 (Upper-Intermediate) — Conceptual — Students understand concrete proof: good decomposition = clean merges; poor decomposition = integration pain

**Key Concepts** (max 7 for B1):
1. Running `/sp.implement` simultaneously across 3 worktrees
2. Non-blocking execution (ability to monitor progress without waiting)
3. Integration preparation (merging strategy planning)
4. Git merge basics: fast-forward, recursive, conflict resolution
5. What merge conflicts reveal (decomposition problems)
6. "Merge early, merge often" strategy (risk mitigation)
7. Measuring integration quality (number of conflicts, resolution time)

**Prerequisites**:
- Lesson 2 (3 plans + 3 task lists complete)
- Chapter 31: Understanding `/sp.implement` workflow
- Chapter 8: Basic merge conflict resolution

**Content Outline**:

1. **Preparing for Parallel Implementation** (15 min)
   - Merge strategy planning: Order matters (dependencies first)
   - Example: If feature-002-payment depends on feature-001-auth, implement auth first
   - Dependency graph exercise: Map your 3 features, determine merge order
   - Pre-merge checklist: What conditions must be true before merging?

2. **Running Parallel Implementation (2-3 hours actual implementation)** (Background during lesson)
   - Session 1: Run `/sp.implement` for feature 001 (e.g., 30-45 min)
   - Session 2 (simultaneously): Run `/sp.implement` for feature 002 (e.g., 30-45 min)
   - Session 3 (simultaneously): Run `/sp.implement` for feature 003 (e.g., 30-45 min)
   - Total time: ~45 min (vs ~2 hours sequential)
   - Key insight: Same 3 features, 2.5x faster due to parallelization

3. **Background: Monitoring Multi-Session Execution** (20 min)
   - While implementations run: Review code in each worktree
   - Validation activity: Do implementations match specifications?
   - Quality check: Any obvious issues or incomplete work?
   - Logging/journaling: Document progress across sessions

4. **Integration: Merging in Dependency Order** (30 min)
   - Step 1: Merge feature-001 into main
     - Verify tests pass
     - Check integration with existing codebase
   - Step 2: Merge feature-002 into main
     - May need to rebase on updated main
     - Verify no conflicts with feature-001
     - Tests pass
   - Step 3: Merge feature-003 into main
     - Same verification, integration checks

5. **Handling & Learning from Merge Conflicts** (20 min)
   - If clean merges: Celebrate! Document that decomposition was excellent
   - If conflicts arise: Treat as learning opportunity, not failure
     - Analyze what went wrong: Shared file? Broken dependency?
     - Resolve conflict strategically (use Plan Mode if available)
     - Root cause: Was spec clear enough? Was decomposition truly independent?
   - Principle: Merge conflicts teach you about decomposition quality

6. **Integration Validation** (15 min)
   - Final integrated system: All 3 features working together locally
   - End-to-end testing: Does the full system work?
   - Smoke tests: Basic functionality across all 3 features
   - Documentation: Update README to show integrated workflow

7. **Time Tracking & Reflection** (10 min)
   - Complete time tracking: Parallel implementation time vs sequential estimate
   - Document merge conflicts encountered (or lack thereof)
   - Reflection: What made integration smooth/difficult?
   - Journaling: "What did merge conflicts teach me about my decomposition?"

**Content Elements**:
- **Code examples**:
  - Example 1: 3-feature starter repository (with clear specs)
  - Example 2: Merge conflict resolution guide (step-by-step with common scenarios)
  - Example 3: Integration testing script (validates all 3 features work together)

- **Exercises**:
  - Exercise 1: Map feature dependencies and determine merge order
  - Exercise 2: Run `/sp.implement` in all 3 worktrees
  - Exercise 3: Merge branches following dependency order
  - Exercise 4: Resolve conflicts (if any) strategically
  - Exercise 5: Run end-to-end tests on integrated system
  - Exercise 6: Time tracking and quality analysis

- **Troubleshooting Guide**:
  - "Merge conflicts everywhere — what did I do wrong?" checklist
  - "One feature seems way bigger than others" — decomposition red flags
  - "Tests failing after merge" — dependency issue diagnosis

**Try With AI** — 15 min
- **Tool**: Claude Code CLI
- **Prompts**:
  1. "I'm seeing merge conflicts in [file]. What does this conflict reveal about my feature decomposition?"
  2. "Review my integration test suite. Are these good integration tests, or am I missing anything?"
  3. "Retrospective: What would I do differently to decompose these 3 features more cleanly?"
- **Expected Outcome**: Students understand that merge conflicts are feedback on decomposition quality; AI helps them learn from integration challenges

---

### Lesson 4: From 3 to 5+ Features — Scaling Decomposition Thinking

**Duration**: 1.5 hours

**Learning Objective**: Students understand progression from 3 features to 5-7 features, identifying what decomposition patterns scale and what breaks at larger systems

**Skills Taught** (CEFR Proficiency Framework):
- **Scalability Analysis** — B1 (Upper-Intermediate) — Conceptual — Students can analyze system decomposition and predict what will break at larger scale
- **Decomposition Thinking: Part 4 (Scaling)** — B2 (Upper-Intermediate/Advanced) — Conceptual — Students understand that scaling requires discipline: tight specs, clear contracts, excellent boundaries
- **Strategic Architecture Thinking** — B1 (Upper-Intermediate) — Conceptual — Students think about team coordination, delegation, and scale (not just technical code)

**Key Concepts** (max 7 for B1):
1. What scales from 3→5 features (tight specs, clear contracts, good boundaries)
2. What breaks at 5+ features (weak specs, implicit dependencies, poor boundaries)
3. Cross-cutting concerns (shared data, security, logging)
4. Bottleneck analysis (what slows down parallel work?)
5. Communication complexity (N features = N*(N-1)/2 integration points)
6. Shared services pattern (reducing feature independence)
7. Path forward: Automation and meta-orchestration (Lessons 5-8)

**Prerequisites**:
- Lessons 1-3 complete (3-feature parallel workflow mastered)

**Content Outline**:

1. **Analyzing Your 3-Feature Decomposition** (15 min)
   - Retrospective: What worked well? What was hard?
   - Metrics: Parallel time saved, merge conflicts, test failures during integration
   - Quality analysis: Rate your decomposition (1-10: 1=barely parallelizable, 10=perfect isolation)
   - Documentation: Capture what made decomposition succeed/fail

2. **What Scales from 3→5 Features** (20 min)
   - Pattern 1: Tight, clear specs (students who defined good contracts experienced clean integration)
   - Pattern 2: Independent concerns (features don't share code, data, or logic)
   - Pattern 3: Clear APIs (well-defined input/output contracts)
   - Pattern 4: Modular architecture (each feature is self-contained)

3. **What Breaks at 5+ Features** (20 min)
   - Problem 1: Cross-cutting concerns (authentication, logging, monitoring)
     - Solution: Shared services/infrastructure that all features depend on
     - Implication: Shared service = potential bottleneck
   - Problem 2: Implicit dependencies
     - Solution: Explicit integration contracts, API documentation
   - Problem 3: Configuration management
     - Solution: Centralized config, clear environment variables
   - Problem 4: Distributed debugging (harder to find bugs across 5 implementations)
     - Solution: Structured logging, distributed tracing

4. **Communication Complexity Analysis** (10 min)
   - Formula: N features = N*(N-1)/2 potential integration points
   - 3 features = 3 integration points (manageable)
   - 5 features = 10 integration points (getting complex)
   - 10 features = 45 integration points (why automation is necessary)
   - Insight: Specifications are communication mechanism (reduce sync meetings)

5. **Identifying Decomposition Problems Early** (15 min)
   - Red flag 1: One feature is much larger than others (5 hours vs 30 min)
   - Red flag 2: Heavy merge conflicts (indicates poor boundaries)
   - Red flag 3: Circular dependencies (feature A depends on B, B depends on A)
   - Red flag 4: Cross-feature code changes (indicates incomplete separation)
   - Exercise: Analyze your 3-feature system for red flags

6. **Path to Scaling: What Comes Next** (10 min)
   - Manual 3-feature workflow: Works great, proven
   - Scaling to 5-7: Automation (CI/CD, MCP, hooks) reduces friction
   - Scaling to 10-15: Meta-orchestration (headless mode, scripts) enables coordination
   - Next step: Lessons 5-8 teach automation and orchestration

7. **Reflection & Strategic Thinking** (10 min)
   - Journaling: "How would I redesign these 3 features to scale to 10?"
   - Team perspective: "If I had 10 junior developers instead of AI agents, what would I do differently?"
   - Business perspective: "How does better decomposition reduce time-to-market?"

**Content Elements**:
- **Visualizations**:
  - Integration complexity graph (N vs N*(N-1)/2)
  - Feature dependency diagrams
  - Decomposition quality rubric (assess your work)

- **Exercises**:
  - Exercise 1: Self-assess your 3-feature decomposition
  - Exercise 2: Identify red flags in your system
  - Exercise 3: Redesign 3 features for better scalability
  - Exercise 4: Map 5-feature decomposition concept

- **Real-world examples**:
  - Stripe engineering: How they scale to 100+ services (tight specs, clear contracts)
  - Open source examples: Linux kernel organization (decomposition at OS level)
  - Team coordination: How startups scale from solo dev to 10-person team

**Try With AI** — 15 min
- **Tool**: Claude Code CLI
- **Prompts**:
  1. "If I needed to scale this system to 5 features, what would break? What redesigns would make it more scalable?"
  2. "Analyze my feature specifications. Which ones are tightly integrated? How could I decouple them?"
  3. "I want to hire 3 junior developers. How would I decompose this system so they could work independently?"
- **Expected Outcome**: AI feedback on scalability; students understand that decomposition thinking is architecture thinking, transferable beyond AI agents

---

### Lesson 5: Automation Layer 1 — CI/CD & Validation Hooks

**Duration**: 1.5 hours

**Learning Objective**: Students configure GitHub Actions to validate specs automatically and prevent bad decomposition from entering the workflow (demonstrating how automation amplifies decomposition quality)

**Skills Taught** (CEFR Proficiency Framework):
- **CI/CD Pipeline Configuration** — B1 (Upper-Intermediate) — Technical — Students can configure GitHub Actions; understand how automation gates quality
- **Spec Validation Automation** — B1 (Upper-Intermediate) — Technical — Students understand how hooks prevent invalid specs from causing downstream problems
- **Automation as Decomposition Amplifier** — B1 (Upper-Intermediate) — Conceptual — Students understand that automation only works when decomposition is excellent; bad decomposition = automated chaos

**Key Concepts** (max 7 for B1):
1. What CI/CD does (automated validation, gates quality, reduces manual checks)
2. Why spec validation matters (bad spec = bad implementation, bad integration)
3. How hooks work (pre-commit, pre-push validation)
4. Constitution as validation target (what makes a "good" spec?)
5. Failure handling (if spec validation fails, block progress, require fix)
6. Scaling benefit (at 5-7 features, manual validation becomes bottleneck; automation prevents slowdown)
7. Time savings from automation (reduced manual review, faster iteration)

**Prerequisites**:
- Lessons 1-4 complete (students understand decomposition thinking)
- Chapter 31: Understanding constitution and spec standards
- Basic GitHub Actions knowledge (can read workflow files)

**Content Outline**:

1. **Why Automation Matters for Scaling** (15 min)
   - Problem: At 3 features, you can manually check each spec
   - At 5-7 features, manual validation becomes bottleneck
   - Solution: Automate validation, let tools enforce standards
   - Principle: Automation is only valuable when decomposition is clear (garbage in = garbage out)

2. **Setting Up GitHub Actions for Spec Validation** (25 min)
   - Create `.github/workflows/spec-validation.yml` (provided template)
   - Workflow triggers: On pull request, when spec.md changes
   - Validation checks:
     - Check 1: Spec.md follows required structure (has evals, learning objectives, etc.)
     - Check 2: No TBD or TODO in required sections
     - Check 3: Acceptance criteria are measurable
     - Check 4: Learning objectives aligned with CEFR proficiency level
   - Demo: Push a bad spec, watch validation fail, fix spec, validation passes

3. **Understanding Validation As Decomposition Gating** (20 min)
   - Key insight: If spec validation fails, it usually means decomposition was unclear
   - Example: Missing acceptance criteria → feature boundaries not clear → decomposition problem
   - Example: Objectives too vague → feature scope too broad → try to parallelize and fail
   - Lesson: Automation isn't punishment; it's feedback on decomposition quality

4. **Integration with SpecKit Plus Workflow** (15 min)
   - When spec validation passes: Confidence to move to planning
   - When spec validation fails: Fix spec before planning (prevents waste)
   - Time saved: Catching spec problems early prevents downstream rework
   - Example: Bad spec → bad plan → bad tasks → bad implementation (4x waste); good spec → streamlined workflow

5. **Scaling Validation** (10 min)
   - At 3 features: 3 specs to validate (manageable)
   - At 5-7 features: 5-7 specs to validate (manual review slow)
   - At 10-15 features: Automation essential, manual review impossible
   - Lesson: Invest in automation now (Lessons 5-8), pay dividends at scale

6. **Reflection: Automation as Enabler** (10 min)
   - Journaling: "How does automation amplify my decomposition quality?"
   - Analysis: What validation checks would I need for 5 features? 10 features?
   - Strategic thinking: How would I scale validation to 7-9 agents?

**Content Elements**:
- **Code examples**:
  - Example 1: GitHub Actions workflow for spec validation (`.github/workflows/spec-validation.yml`)
  - Example 2: Bash/Python script to validate spec structure locally
  - Example 3: Constitution-based validation checklist (what makes a "good" spec)

- **Exercises**:
  - Exercise 1: Create spec validation workflow (use provided template)
  - Exercise 2: Push a deliberately bad spec, watch validation fail
  - Exercise 3: Fix spec to pass validation
  - Exercise 4: Run validation locally and in CI/CD, compare results

- **Troubleshooting**:
  - "My spec passes locally but fails in CI/CD" (environment differences)
  - "Validation too strict, blocking my workflow" (tuning thresholds)

**Try With AI** — 15 min
- **Tool**: Claude Code CLI
- **Prompts**:
  1. "Review my spec validation workflow. Are there checks I'm missing for good decomposition?"
  2. "My spec validation keeps failing. What does that tell me about my feature decomposition?"
  3. "How would I extend spec validation to catch decomposition problems earlier?"
- **Expected Outcome**: AI feedback on validation quality; students understand automation as decomposition amplifier

---

### Lesson 6: Automation Layer 2 — MCP Servers & Shared Context

**Duration**: 1.5 hours

**Learning Objective**: Students understand how MCP servers provide shared intelligence across parallel sessions, enabling data-informed decomposition and higher-quality specs

**Skills Taught** (CEFR Proficiency Framework):
- **MCP Server Integration** — B1 (Upper-Intermediate) — Technical — Students can connect MCP servers; query shared data
- **Shared Context Across Sessions** — B1 (Upper-Intermediate) — Technical — Students maintain state across multiple parallel sessions
- **Data-Informed Decomposition** — B1 (Upper-Intermediate) — Conceptual — Students understand that specs informed by data (user needs, usage patterns, product insights) produce better decomposition

**Key Concepts** (max 7 for B1):
1. What MCP servers do (provide shared intelligence, avoid duplicated context)
2. Types of MCP servers useful for development (database, monitoring, product analytics)
3. Query patterns (what data informs decomposition decisions?)
4. Context freshness (real production data vs synthetic)
5. Session consistency (all sessions access same shared intelligence)
6. Integration with spec-writing (specs become data-informed, not assumptions)
7. Scaling benefit (at 5-7 features, MCP ensures all specs use same source of truth)

**Prerequisites**:
- Lessons 1-5 complete
- Chapter covering MCP basics (Part 7 introduces MCP)

**Content Outline**:

1. **Why Shared Context Matters in Parallel Development** (15 min)
   - Problem: 3 sessions writing specs independently could make conflicting assumptions
   - Solution: All sessions connected to shared MCP servers (database, analytics, product insights)
   - Benefit: Specs are consistent, data-informed, aligned on real customer needs
   - Scaling: At 5-7 features, shared context becomes essential (prevents conflicting assumptions)

2. **Setting Up MCP Servers for Feature Development** (25 min)
   - MCP 1: Product Database (features, user stories, requirements)
   - MCP 2: Analytics (usage patterns, user pain points)
   - MCP 3: Code Review MCP (linting, style checks, best practices)
   - Demo: Query MCP from one session, verify same data in another session

3. **Writing Data-Informed Specifications** (20 min)
   - Question 1: What are top user pain points? (query analytics MCP)
   - Question 2: Do we have existing requirements? (query product MCP)
   - Question 3: Are there conflicts with existing architecture? (query code review MCP)
   - Principle: Specs informed by shared data = higher quality decomposition
   - Example: Analytics shows payment feature used by 80% of users (high priority), so decompose payment first

4. **Session Consistency & Shared State** (15 min)
   - Challenge: How do 3 sessions writing specs avoid stepping on each other?
   - Solution: MCP as source of truth (all sessions read same data, avoid conflicts)
   - Feature numbering: Even with shared MCP, each session still gets own feature (001, 002, 003)
   - Testing: Edit data in one session, verify other sessions see changes (demonstrates synchronization)

5. **Scaling MCP for 5-7 Features** (10 min)
   - At 5-7 features: Shared context becomes essential (manual meetings are bottleneck)
   - Example: Without MCP, 5 spec-writing sessions would need to sync decisions (hours of meetings)
   - With MCP: All 5 sessions query same data, specs align automatically
   - Time savings: Specifications aligned without synchronous communication

6. **Reflection: Shared Context as Coordination Mechanism** (10 min)
   - Journaling: "How did shared context change how I decomposed features?"
   - Analysis: What decisions in my specs were informed by MCP data?
   - Strategic thinking: "How would I scale MCP context to coordinate 7-9 agents?"

**Content Elements**:
- **Code examples**:
  - Example 1: Setting up 3 MCP servers (product, analytics, code-review)
  - Example 2: Querying MCP from Claude Code session (example prompts)
  - Example 3: MCP configuration for multi-session setup

- **Exercises**:
  - Exercise 1: Set up 3 MCP servers
  - Exercise 2: Query each MCP from a test session
  - Exercise 3: Write specs informed by MCP data
  - Exercise 4: Verify consistency across 3 sessions (all reference same data)

- **Real-world examples**:
  - Anthropic's internal use of MCP for aligned AI development
  - How data-informed specs reduce iteration and rework

**Try With AI** — 15 min
- **Tool**: Claude Code CLI
- **Prompts**:
  1. "I'm setting up MCP servers for my team. What data sources should I expose?"
  2. "Review my spec. Is it data-informed, or making assumptions?"
  3. "How would I scale MCP to coordinate 10 simultaneous feature teams?"
- **Expected Outcome**: Students understand MCP as coordination mechanism, not just tool

---

### Lesson 7: Automation Layer 3 — Background Execution & Progress Monitoring

**Duration**: 1.5 hours

**Learning Objective**: Students use Claude Code background bash execution to run long-running implementations without blocking, understanding how non-blocking execution enables true parallelization

**Skills Taught** (CEFR Proficiency Framework):
- **Background Process Management** — B1 (Upper-Intermediate) — Technical — Students can spawn background tasks; monitor progress; handle async execution
- **Non-Blocking Workflow Execution** — B1 (Upper-Intermediate) — Technical — Students understand async execution patterns and their role in scaling
- **Parallel Work Without Terminal Overhead** — A2 (Elementary) — Technical — Students experience true parallelization (work progresses without blocking on any single session)

**Key Concepts** (max 7 for B1):
1. What background execution does (runs tasks without blocking the session)
2. Why it matters (enables true parallelization; you don't wait for one task to finish before starting next)
3. How to spawn background tasks (Claude Code background bash execution)
4. Monitoring progress (check status, logs, outputs)
5. Error handling (what if background task fails?)
6. Scaling to 5-7 tasks (at 5 concurrent tasks, terminal management becomes difficult; background execution handles it)
7. Time savings (5 implementations in parallel without 5 terminal windows)

**Prerequisites**:
- Lessons 1-6 complete
- Bash Essentials (Chapter 7) understanding

**Content Outline**:

1. **The Problem: Blocking Execution Prevents True Parallelization** (10 min)
   - Current workflow: Start `/sp.implement` in Session 1, wait for completion, start Session 2
   - This is synchronous, defeats parallelization benefit
   - Solution: Background execution (start all 3 implementations, check progress independently)
   - Benefit: True parallelization (3 implementations happening simultaneously, total time = longest implementation)

2. **Setting Up Background Bash Execution** (15 min)
   - Claude Code feature: `background_bash` flag in prompts
   - Example: "Run `make build` in the background and show me when it's done"
   - Returns: Task ID, initial output, ability to check status
   - Demo: Spawn 3 background implementations, check progress across all 3

3. **Monitoring Multiple Background Tasks** (20 min)
   - Command: Check background task status
   - Output: Progress logs, completion status, errors
   - Real-time example: 3 implementations running in background, check progress every 5 minutes
   - Error handling: If one task fails, debug while others continue
   - Benefit: Understand status across all 3 without 3 open terminals

4. **Handling Failures in Background Execution** (15 min)
   - Scenario: Implementation 2 fails, Implementations 1 & 3 succeed
   - Action: Stop implementation 2, investigate error
   - Question: Is this a decomposition problem, or just missing dependency?
   - Lesson: Background execution makes failures visible but doesn't hide decomposition problems

5. **Scaling to 5-7 Background Tasks** (15 min)
   - Current: 3 background tasks (manageable)
   - Next level: 5-7 background tasks (requires structured monitoring)
   - Problem: How do you track 5-7 background tasks without overwhelm?
   - Solution: Structured logging, progress dashboard (Lesson 8 teaches this at scale)

6. **Reflection: Non-Blocking Execution as Enabler** (10 min)
   - Journaling: "How does background execution change my workflow?"
   - Analysis: Time saved by true parallelization (3 tasks in parallel time = ~1/3 sequential time)
   - Strategic thinking: "How would I scale background execution to 10-15 tasks?"

**Content Elements**:
- **Code examples**:
  - Example 1: Claude Code prompt with `background_bash` flag
  - Example 2: Bash script to monitor multiple background tasks
  - Example 3: Error handling script (retry failed tasks, alert on failures)

- **Exercises**:
  - Exercise 1: Spawn 3 background implementations
  - Exercise 2: Monitor progress (check status every few minutes)
  - Exercise 3: Handle a simulated failure (kill one task, investigate, restart)
  - Exercise 4: Time actual execution with background vs without

- **Dashboard/Monitoring**:
  - Simple: Check task IDs and statuses manually
  - Advanced: Create a simple progress file that updates as tasks complete

**Try With AI** — 15 min
- **Tool**: Claude Code CLI
- **Prompts**:
  1. "Design a monitoring dashboard for 7 simultaneous background implementations. What metrics matter?"
  2. "I have 3 background tasks. One failed. How do I debug it while others continue?"
  3. "How would I scale background execution to coordinate 15 simultaneous tasks?"
- **Expected Outcome**: Students understand non-blocking execution as key to true parallelization

---

### Lesson 8: Meta-Orchestration — Headless Mode & Programmatic Coordination (Advanced, Optional)

**Duration**: 1.5 hours
**Complexity Tier**: Advanced (B2/Upper-Intermediate) — OPTIONAL lesson (P2 priority)

**Learning Objective**: Students understand how headless mode enables programmatic orchestration of 7-9 AI Agents without human terminal management, demonstrating the ultimate scaling pathway

**Skills Taught** (CEFR Proficiency Framework):
- **Headless Mode Execution** — B2 (Upper-Intermediate/Advanced) — Technical — Students can invoke Claude Code headless; capture JSON output; maintain session context
- **Orchestration Script Development** — B2 (Upper-Intermediate/Advanced) — Technical — Students can write bash/python scripts that spawn multiple headless sessions programmatically
- **Session Management Across Phases** — B2 (Upper-Intermediate/Advanced) — Technical — Students use `--resume` flag to maintain context across Specify → Plan → Tasks → Implement workflow
- **Decomposition at Scale** — B2 (Upper-Intermediate/Advanced) — Conceptual — Students understand that 7-9 agent orchestration is possible ONLY with excellent decomposition (poor decomposition = coordination chaos)

**Key Concepts** (max 10 for B2):
1. What headless mode is (non-interactive Claude Code execution via CLI)
2. Why it matters (enables programmatic orchestration, batch processing)
3. Output format (JSON structured output for programmatic parsing)
4. Session IDs (maintain context across invocations)
5. `--resume [session-id]` flag (continue workflow in same context)
6. Orchestration script structure (bash/python spawning multiple headless sessions)
7. Progress monitoring (parsing JSON output, detecting failures)
8. Error handling (retry failed features, stop on critical failures)
9. Timeout management (long-running tasks, graceful failure)
10. Proof of concept (orchestration script handling 5-feature workflow end-to-end)

**Prerequisites**:
- Lessons 1-7 complete
- Comfortable reading bash/python scripts (not required to write from scratch)
- Basic understanding of JSON and command-line tools

**Content Outline**:

1. **The Vision: 1 Human + 7-9 AI Agents** (15 min)
   - Lessons 1-3: Manual coordination of 2-3 agents (proven to work, demonstrated 3x speedup)
   - Lessons 4-7: Automation amplified coordination (CI/CD, MCP, background execution)
   - Lesson 8: Meta-orchestration (programmatic coordination of 7-9 agents)
   - Question: What enables this scale? Excellent decomposition. Without it, chaos.

2. **Understanding Headless Mode** (20 min)
   - Concept: Run Claude Code commands without interactive terminal
   - Command: `claude -p "/sp.specify [feature-description]" --output-format json`
   - Output: Structured JSON with:
     - `session_id`: Unique ID for this session (used to resume later)
     - `status`: Success or failure
     - `spec`: Generated specification
   - Demo: Run single headless command, parse JSON output
   - Key insight: JSON output enables programmatic parsing (scripts can read and react to results)

3. **Session Management with `--resume`** (20 min)
   - Problem: How do you continue a workflow (Specify → Plan → Tasks) in headless mode?
   - Solution: Use session ID to resume in same context
   - Example workflow:
     - Command 1: `claude -p "/sp.specify" --output-format json` → Returns session_id
     - Command 2: `claude -p "/sp.plan" --resume [session_id] --output-format json` → Plan uses context from Specify phase
     - Command 3: `claude -p "/sp.tasks" --resume [session_id] --output-format json` → Tasks uses context from Plan phase
   - Benefit: Each phase maintains context, no retyping or re-explaining

4. **Writing an Orchestration Script** (20 min)
   - High-level structure (pseudocode):
     ```bash
     for feature in feature-001 feature-002 feature-003 feature-004 feature-005; do
       # Phase 1: Specify
       SESSION_ID=$(claude -p "/sp.specify [feature-desc]" --output-format json | jq '.session_id')

       # Phase 2: Plan (resume session)
       claude -p "/sp.plan" --resume $SESSION_ID --output-format json

       # Phase 3: Tasks (resume session)
       claude -p "/sp.tasks" --resume $SESSION_ID --output-format json

       # Phase 4: Implement (background execution)
       claude -p "/sp.implement" --resume $SESSION_ID --output-format json > logs/$feature.log &
     done

     # Wait for all implementations to complete
     wait
     ```
   - Key points:
     - Loop spawns headless sessions for each feature in PARALLEL
     - Session ID maintains context across phases
     - Implementations run in background (true parallelization)
     - `wait` command ensures all complete before proceeding to next phase

5. **Monitoring & Error Handling** (15 min)
   - Challenge: How do you know if an implementation failed?
   - Solution: Parse JSON output, check `status` field
   - Error handling example:
     - If status = "error", capture error message, retry or alert
     - If status = "success", proceed to next phase
   - Logging: Capture all outputs to logs/ directory for audit trail
   - Dashboard: Simple progress file showing which features are in which phase

6. **Scaling to 7-9 agents** (15 min)
   - Lessons learned from 5-feature orchestration:
     - What worked well (parallelization benefit, clean integration with good decomposition)
     - What broke (decomposition problems surface earlier at 5 features; 10 features = even more critical)
   - Why 7-9 agents require EXCELLENT decomposition:
     - At 3 features: OK decomposition works
     - At 5 features: Good decomposition required
     - At 10-15 features: EXCELLENT decomposition absolutely required
   - The bottleneck isn't tools; it's decomposition thinking
   - Lesson: Master decomposition (Lessons 1-4), automation amplifies it (Lessons 5-7), orchestration scales it (Lesson 8)

7. **Proof of Concept: End-to-End Orchestration** (15 min)
   - Provided: Complete orchestration script (provided template, not written from scratch)
   - Task: Modify script for your 5-feature system
   - Run: Single command orchestrates Specify → Plan → Tasks → Implement for all 5 features
   - Observe: All 5 features processed in parallel, total time ≈ 1 sequential workflow
   - Reflection: "This is what 1 human + 5 AI agents looks like"

8. **Path to 7-9 agents** (10 min)
   - Current orchestration script: Good enough for 5 features
   - Improvements for 10-15 features:
     - Better error handling (retry logic, circuit breakers)
     - Progress monitoring (dashboard showing which features are in which phase)
     - Resource management (don't spawn too many concurrent processes, system will be overwhelmed)
     - Timeout handling (long-running implementations, graceful failure)
   - Key insight: Orchestration is relatively easy; decomposition quality is the bottleneck
   - Lesson: If you can decompose a system into 10-15 parallelizable units, orchestration = solved problem

9. **Reflection: Meta-Orchestration as Strategic Capability** (10 min)
   - Journaling: "What's the difference between running 3 terminals and running an orchestration script?"
   - Analysis: Time savings, stress reduction, scalability
   - Strategic thinking: "How would I explain this capability to a CTO or VP Engineering?"
   - Transfer: "How does this pattern apply to managing human teams instead of AI agents?"

**Content Elements**:
- **Code examples** (provided templates, not exercises to write from scratch):
  - Example 1: Headless mode single-command example (for experimentation)
  - Example 2: Session resume pattern (showing context preservation)
  - Example 3: Complete orchestration script (5-feature end-to-end workflow) — provided as template
  - Example 4: Error handling and retry logic (defensive programming)
  - Example 5: Progress monitoring script (checking status of 5 parallel tasks)

- **Exercises**:
  - Exercise 1 (guided): Run single headless command, parse JSON output
  - Exercise 2 (guided): Run orchestration script template for your 5-feature system
  - Exercise 3 (optional): Modify orchestration script for your specific needs (add logging, change parallelism)
  - Exercise 4 (reflection): Analyze orchestration script, discuss what would need to change for 7-9 agents

- **Real-world context**:
  - How Anthropic uses similar patterns for internal development
  - How other AI companies coordinate multiple models/agents
  - Academic examples: Distributed computing, parallel processing

**Try With AI** — 15 min
- **Tool**: Claude Code CLI
- **Prompts**:
  1. "Review my orchestration script. What improvements would help it scale to 10-15 features?"
  2. "Explain how my orchestration script works step-by-step. Where are the potential failure points?"
  3. "How would I modify this to ensure all 7-9 agents work in true parallel, without bottlenecks?"
- **Expected Outcome**: Students understand meta-orchestration as strategic capability; recognize decomposition as ultimate bottleneck

---

### Lesson 9: Capstone Project & Measurement — Portfolio-Worthy Decomposition Thinking

**Duration**: 3-4 hours (2-3 hours for 3-feature version, 3-4 hours for 5-feature version)

**Learning Objective**: Students complete a 3-5 feature system using the full parallel workflow (Lessons 1-8), measure actual productivity gains, and document the project as proof of decomposition thinking mastery

**Skills Taught** (CEFR Proficiency Framework):
- **End-to-End Project Delivery** — B1 (Upper-Intermediate) — Technical — Students can execute complete SpecKit Plus workflow from decomposition through integration and deployment
- **Metrics & Reflection** — B1 (Upper-Intermediate) — Conceptual — Students can articulate value of decomposition thinking through concrete metrics (time savings, integration quality, code volume)
- **Portfolio Narrative** — B1 (Upper-Intermediate) — Communication — Students can explain their project to non-technical stakeholders, emphasizing strategic thinking over tool proficiency
- **Decomposition Thinking: Mastery** — B2 (Upper-Intermediate/Advanced) — Conceptual — Students demonstrate comprehensive understanding of decomposition, parallelization, and scaling patterns

**Key Concepts** (max 10 for B2):
1. Decomposition as strategic architecture (not just code organization)
2. Integration quality as proof of decomposition quality
3. Time measurement and productivity gains calculation
4. Portfolio narrative: What story does your project tell?
5. Transferability: How does this apply beyond AI agents?
6. Reflection and learning: What would you do differently?
7. Scaling pathway: From solo + 3 agents to team + 7-9 agents
8. Business value: Why decomposition thinking matters
9. Continuous improvement: How to refine your decomposition approach
10. Documentation and knowledge sharing

**Prerequisites**:
- Lessons 1-8 complete (or Lessons 1-7 for fast-track without meta-orchestration)
- 3-5 small features ready to be decomposed and built

**Content Outline**:

1. **Capstone Project Design** (30 min)
   - Choose system to build: 3-5 features, 2-6 hours implementation time
   - Examples:
     - Task management app (features: task CRUD, priorities, team sharing)
     - Content platform (features: articles, comments, recommendations)
     - API wrapper with caching (features: auth, rate limiting, cache layer)
   - Requirement: Features must be independently buildable, integrate cleanly
   - Key decision: Are these features truly decomposable, or hidden dependencies?
   - Clarification exercise: Define integration contracts BEFORE building

2. **Execution: Full Parallel Workflow** (2-3 hours actual work)
   - Phase 1: Decomposition (15 min)
     - Design: How will you break this system into 3-5 features?
     - Define: What's in each feature? What's the integration contract?
     - Validate: Are features truly independent?
   - Phase 2: Parallel Specifications (15-20 min)
     - Set up worktrees, write specs in parallel
     - Review: Do specs clearly define feature boundaries?
   - Phase 3: Parallel Planning & Tasks (20-30 min)
     - Generate plans and tasks for all features in parallel
     - Analysis: Do plan complexities suggest good decomposition?
   - Phase 4: Parallel Implementation (1-2 hours)
     - Implement all features in parallel (using Lessons 1-8 techniques)
     - Optional: Use Lesson 5-8 automation (CI/CD, MCP, background execution, orchestration)
   - Phase 5: Integration & Testing (30 min)
     - Merge branches in dependency order
     - Test integrated system (all features working together)
     - Document any merge conflicts or integration issues

3. **Measurement: Time Tracking & Productivity Analysis** (20 min)
   - Baseline: How long would 3-5 features take sequentially?
     - Estimate: If each feature takes ~1 hour, sequential = 3-5 hours
   - Actual: How long did parallel workflow take?
     - If 3 features in 1 hour parallel vs 3 hours sequential = 3x speedup
   - Analysis: What drove the speedup?
     - Decomposition quality? Good specs enabled parallelization
     - Terminal efficiency? Parallel work saved context switching
     - Automation? (Lessons 5-8) CI/CD and background execution reduced friction
   - Metrics worksheet:
     - Total time (parallel): [X minutes]
     - Estimated time (sequential): [Y minutes]
     - Speedup multiplier: Y/X (should be 2.5-4x for well-decomposed system)
     - Integration quality: 0 conflicts? 1-2 conflicts? Major conflicts? (Indicator of decomposition quality)

4. **GitHub Repository & Documentation** (30 min)
   - Create repository with clear structure:
     - Branches showing parallel development (feature-001, feature-002, feature-003)
     - Commit history showing independent work in each branch
     - README documenting the multi-session workflow
     - Time tracking worksheet (proof of speedup)
   - Documentation template:
     ```markdown
     # [Project Name] — Decomposition Thinking Case Study

     ## What We Built
     [Overview of 3-5 features]

     ## How We Built It (Decomposition Approach)
     [Feature breakdown, integration contracts, why these are parallelizable]

     ## Results
     - Sequential estimate: [Y] minutes
     - Parallel execution: [X] minutes
     - Speedup: [Y/X]x
     - Integration quality: [# conflicts, time to resolve]

     ## What Worked Well
     [Reflection on decomposition quality, spec clarity, integration]

     ## What We'd Do Differently
     [Redesign for better parallelization if building again]

     ## Transferability
     [How does this approach apply to human team coordination?]
     ```

5. **Reflection & Narrative** (30 min)
   - Core question: What did you learn about decomposition thinking?
   - Write 1-2 page reflection:
     - What made features parallelizable?
     - What would make them NOT parallelizable?
     - How would you explain this to a product manager? An investor? A CTO?
     - How does this thinking apply beyond AI agents? (junior developers? distributed teams? organizational scaling?)
   - Emotional reflection: The "aha moment"
     - Did you experience the insight: "Good decomposition = clean integration"?
     - How did merge conflicts (or lack thereof) validate your decomposition?

6. **Portfolio Narrative** (20 min)
   - Goal: Create a narrative that employers/interviewers care about
   - NOT: "I learned git worktrees and ran parallel terminals"
   - YES: "I learned decomposition thinking — how to break complex systems into parallelizable units with clear contracts, enabling autonomous work and 3x productivity gains"
   - Script for explaining your project:
     - Opening: "I built a [system name] using decomposition thinking"
     - Insight: "The key insight is that good decomposition eliminates coordination overhead. I designed [N] features with clear integration contracts, built them in parallel, and integrated them with [# conflicts]"
     - Impact: "This approach saved [X] hours and demonstrates how 1 person can coordinate 10-15 team members (or AI agents) without synchronous communication"
     - Transfer: "The same decomposition patterns apply to managing junior developers or distributed teams"

7. **Capstone Submission Requirements** (Checklist)
   - Code repository with clear multi-worktree history
   - README documenting decomposition approach and results
   - Time tracking worksheet with actual measurements
   - Reflection essay (1-2 pages, addressing transferability and learning)
   - Demonstration (optional): Walk through project, explain decomposition decisions
   - Portfolio artifact: GitHub repository link + reflection narrative

8. **Final Reflection & Celebration** (20 min)
   - Review your capstone: What's your biggest insight?
   - Journaling: "What did decomposition thinking teach me?"
   - Celebration: You've completed Lessons 1-9 and built a portfolio-worthy project
   - Path forward: How will you apply this in future projects?

**Content Elements**:
- **Capstone project starters** (3 examples):
  - Example 1: Task management app (3 features: CRUD, priorities, sharing)
  - Example 2: Blog platform (3 features: posts, comments, recommendations)
  - Example 3: API wrapper (3 features: auth, rate limiting, caching)

- **Templates**:
  - Feature decomposition template (document your design)
  - Time tracking worksheet (baseline vs parallel, actual measurements)
  - GitHub repository template (clear structure, documentation)
  - Reflection essay template (guided prompts)

- **Rubric for Capstone Evaluation**:
  - Decomposition quality (1-5): Are features truly independent? Clear contracts?
  - Execution quality (1-5): Are all features working? Clean integration?
  - Measurement quality (1-5): Time tracking documented? Calculations correct?
  - Reflection quality (1-5): Genuine insights about decomposition? Transferability addressed?
  - Portfolio narrativeness (1-5): Could you explain this to an employer?

**Try With AI** — 15 min
- **Tool**: Claude Code CLI
- **Prompts**:
  1. "Review my capstone project. What does the integration quality (# of conflicts, merge difficulty) tell me about my decomposition?"
  2. "Help me refine my portfolio narrative. How would I explain this project to a CTO?"
  3. "If I wanted to scale this system to 5-10 features, what would I redesign based on what I learned?"
- **Expected Outcome**: AI feedback on capstone quality; students finalize portfolio narrative; celebration of completing Chapter 32

---

## Scaffolding Strategy

**Complexity Progression** (Part 5 = Intermediate Tier):

- **Lessons 1-4**: Foundation building
  - Lesson 1: Core concept (worktrees) introduced simply
  - Lesson 2: Scaling concept (parallelization at plan/task level)
  - Lesson 3: Integration proof (specs → merged code proves decomposition quality)
  - Lesson 4: Scaling analysis (3→5 features, identifying what breaks)
  - Key: Students experience core learning objective (decomposition thinking) hands-on

- **Lessons 5-7**: Automation amplifies foundation
  - Lesson 5: Validation gates (specs catch decomposition problems)
  - Lesson 6: Shared context (MCP reduces coordination overhead)
  - Lesson 7: Non-blocking execution (background tasks enable true parallelization)
  - Key: Students understand automation as decomposition amplifier, not replacement

- **Lesson 8**: Advanced orchestration (optional)
  - Lesson 8: Meta-orchestration (programmatic coordination of 7-9 agents)
  - Key: Students understand ultimate scaling pathway, but not required to master it

- **Lesson 9**: Integration & reflection
  - Capstone: Apply everything, measure gains, create portfolio artifact
  - Key: Proof of learning through real project delivery

**Cognitive Load Management** (Part 5 = 7 concepts per section):

- Lesson 1: 7 concepts (worktree, isolation, parallelization, specs, numbering, contracts, time tracking)
- Lesson 2: 7 concepts (planning parallelization, plan quality, task generation, terminal management, parallelization value, time savings, reflection)
- Lesson 3: 7 concepts (parallel implementation, non-blocking, merge strategy, conflict resolution, integration validation, time tracking, decomposition feedback)
- Lesson 4: 7 concepts (what scales, what breaks, cross-cutting concerns, bottleneck analysis, communication complexity, shared services, automation)
- Lesson 5: 7 concepts (CI/CD, validation gates, spec validation, decomposition feedback, failure handling, automation scaling, time savings)
- Lesson 6: 7 concepts (MCP servers, shared context, query patterns, session consistency, data-informed specs, scaling, time savings)
- Lesson 7: 7 concepts (background execution, non-blocking tasks, monitoring, error handling, scaling to 5-7 tasks, parallelization benefit, reflection)
- Lesson 8: 10 concepts (headless mode, JSON output, session IDs, `--resume` flag, orchestration scripts, progress monitoring, error handling, scaling to 10-15, bottlenecks, strategic value)
- Lesson 9: 10 concepts (decomposition strategy, integration quality, measurement, portfolio narrative, reflection, transferability, scaling, documentation, rubric, celebration)

**Transferability Explicit** (How decomposition thinking transfers):

- Lessons 1-4 explicitly reference: "How would this work with junior developers instead of AI agents?"
- Lesson 4 dedicated to transferability: Analyzing patterns, scaling insights
- Lesson 9 capstone requires: Reflection on how decomposition applies to human teams
- Throughout: Real-world examples (Stripe, Vercel, startup scaling) show decomposition thinking in practice

---

## Integration Points

**Prerequisites** (Students must complete these first):
- Chapter 30: Understanding Specification-Driven Development
- Chapter 31: SpecKit Plus Hands-On
- Chapter 8: Git & GitHub (branching, merging, basic workflows)
- Chapter 7: Bash Essentials (terminal proficiency)
- Chapter 5: Claude Code CLI (basic features)

**Forward References** (Students can apply Chapter 32 learning to these):
- Part 10: Containerization & Deployment — Apply decomposition thinking to microservices
- Part 11: Data, State, Memory — How do distributed features share state?
- Parts 12-13: Event-Driven Architecture — How do decomposed systems communicate asynchronously?

**Cross-Chapter Alignment**:
- Chapter 32 demonstrates Principle 14 (Planning-First Development) at scale
- Chapter 32 demonstrates Principle 15 (Validation-Before-Trust) via integration quality
- Chapter 32 teaches transferability (Constitution vision: decomposition thinking applies across domains)

---

## Validation Strategy

**How Learners Demonstrate Understanding**:

**Technical Validation** (Can they execute the workflow?):
- Lesson 1: Can set up 3 git worktrees, run parallel specs, verify auto-incrementing numbers
- Lesson 2: Can run plans/tasks in parallel, monitor progress, evaluate plan quality
- Lesson 3: Can implement 3 features in parallel, integrate, resolve conflicts if present
- Lesson 4: Can analyze decomposition quality, identify scaling problems
- Lessons 5-7: Can configure CI/CD, MCP, background execution (if attempting automation)
- Lesson 8: Can run/understand orchestration script (if attempting meta-orchestration)
- Lesson 9: Can deliver working 3-5 feature system with time tracking

**Conceptual Validation** (Do they understand decomposition thinking?):
- Reflection prompts in each lesson: "What made this parallelizable?"
- Merge conflict analysis (Lesson 3): "What does this conflict tell me about my decomposition?"
- Scaling analysis (Lesson 4): "What would break at 5-10 features?"
- Capstone reflection: "Explain decomposition thinking to a non-technical stakeholder"

**Portfolio Validation** (Can they articulate value to employers?):
- GitHub repository with clear multi-worktree history and documentation
- Time tracking worksheet showing actual productivity gains
- Reflection essay addressing transferability and strategic insight
- Portfolio narrative explaining how decomposition thinking is a strategic capability

---

## Skills Proficiency Metadata Summary

### Lesson-by-Lesson Skills Mapping (CEFR Proficiency Levels)

| Lesson | Skill | Level | Category | Bloom's | DigComp |
|--------|-------|-------|----------|---------|---------|
| 1 | Git Worktrees Proficiency | A2 | Technical | Understand | Information |
| 1 | Parallel Specification Design | B1 | Technical | Apply | Communication |
| 1 | Decomposition Thinking: Part 1 | A2 | Conceptual | Understand | Problem-Solving |
| 2 | Multi-Session Project Management | B1 | Technical | Apply | Communication |
| 2 | Decomposition Thinking: Part 2 | B1 | Conceptual | Analyze | Problem-Solving |
| 2 | Constitution as Shared Contract | A2 | Conceptual | Understand | Information |
| 3 | Parallel Implementation Execution | B1 | Technical | Apply | Communication |
| 3 | Integration & Conflict Resolution | B1 | Technical | Apply | Problem-Solving |
| 3 | Decomposition Thinking: Part 3 | B1 | Conceptual | Analyze | Problem-Solving |
| 4 | Scalability Analysis | B1 | Conceptual | Analyze | Problem-Solving |
| 4 | Decomposition Thinking: Part 4 (Scaling) | B2 | Conceptual | Analyze | Problem-Solving |
| 4 | Strategic Architecture Thinking | B1 | Conceptual | Analyze | Problem-Solving |
| 5 | CI/CD Pipeline Configuration | B1 | Technical | Apply | Information |
| 5 | Spec Validation Automation | B1 | Technical | Apply | Safety |
| 5 | Automation as Decomposition Amplifier | B1 | Conceptual | Understand | Problem-Solving |
| 6 | MCP Server Integration | B1 | Technical | Apply | Information |
| 6 | Shared Context Across Sessions | B1 | Technical | Apply | Communication |
| 6 | Data-Informed Decomposition | B1 | Conceptual | Apply | Problem-Solving |
| 7 | Background Process Management | B1 | Technical | Apply | Information |
| 7 | Non-Blocking Workflow Execution | B1 | Technical | Apply | Communication |
| 7 | Parallel Work Without Terminal Overhead | A2 | Technical | Understand | Information |
| 8 | Headless Mode Execution | B2 | Technical | Apply | Information |
| 8 | Orchestration Script Development | B2 | Technical | Apply | Problem-Solving |
| 8 | Session Management Across Phases | B2 | Technical | Apply | Communication |
| 8 | Decomposition at Scale | B2 | Conceptual | Analyze | Problem-Solving |
| 9 | End-to-End Project Delivery | B1 | Technical | Create | Problem-Solving |
| 9 | Metrics & Reflection | B1 | Conceptual | Evaluate | Problem-Solving |
| 9 | Portfolio Narrative | B1 | Communication | Communicate | Communication |
| 9 | Decomposition Thinking: Mastery | B2 | Conceptual | Evaluate | Problem-Solving |

### Proficiency Progression Validation

✅ **A1 → A2 → B1 → B2 Progression Clear**:
- Lessons 1-3: Students recognize and apply decomposition at A2/B1 level (manual coordination)
- Lesson 4: Students analyze decomposition at B1 level (scalability thinking)
- Lessons 5-7: Students understand automation as amplifier at B1 level (automation and orchestration)
- Lesson 8: Students design orchestration scripts at B2 level (advanced meta-orchestration)
- Lesson 9: Students evaluate and refine decomposition at B2 level (mastery)

✅ **Cognitive Load Respects Limits**:
- A2/B1 sections: Max 7 concepts per lesson
- B2 sections (Lessons 8-9): Max 10 concepts per lesson
- Each lesson builds on prior without overwhelming

✅ **Bloom's Taxonomy Alignment**:
- Lessons 1-2: Understand, Apply (foundation)
- Lessons 3-4: Analyze (critical thinking)
- Lessons 5-7: Apply (automation techniques)
- Lesson 8: Apply, Analyze (orchestration)
- Lesson 9: Create, Evaluate (capstone, mastery)

---

## Notes on Chapter Type & Pedagogical Approach

**Chapter Type**: Technical/Hybrid with strong conceptual emphasis (60% decomposition thinking, 40% tool proficiency)

**Why This Structure Works**:
1. **Decomposition first**: Lessons 1-4 teach the thinking pattern, prove it works with 3 features
2. **Automation amplifies**: Lessons 5-7 show how automation scales decomposition patterns
3. **Orchestration demonstrates**: Lesson 8 shows ultimate scaling (7-9 agents possible only with excellent decomposition)
4. **Capstone proves**: Lesson 9 real project demonstrates all learning outcomes simultaneously

**Why Not Just Teach Tools**:
- If Chapter 32 teaches "git worktrees, SpecKit Plus, CI/CD", students miss the core insight
- Core insight: "Good decomposition eliminates coordination overhead"
- Tools (worktrees, SpecKit Plus, CI/CD) are vehicles to experience decomposition patterns
- Tools change; decomposition thinking is evergreen

**Transferability**: Every lesson explicitly connects to human team coordination, organizational scaling, and strategic thinking — not just AI agent orchestration.

---

## Risk Mitigations (From Spec)

- **Risk 1: Students overwhelmed by multiple sessions** → Start with 2 worktrees, expand to 3-5 only after mastery (Lesson 1-2)
- **Risk 2: Merge conflicts discourage students** → Lesson 8 dedicated to conflict resolution; teach "merge early" strategy
- **Risk 3: Students feel capstone incomplete without deployment** → Clearly communicate focus (parallel development, not deployment); reference Parts 10-11 for deployment
- **Risk 4: Complexity exceeds Part 5 intermediate tier** → Strict 3-4 option limit, extensive worked examples, graduated complexity Lessons 1-9
- **Risk 5: Time estimates unrealistic** → Fast track: Lessons 1-4 + Capstone = 6-8 hours; Lessons 5-8 optional; Lesson 8 explicitly marked advanced/optional
- **Risk 6: Meta-orchestration (Lesson 8) too advanced** → Provided as complete template (students run/modify, not write from scratch); understanding > implementation; optional P2 priority

---

## Expected Chapter Outcomes (Success Criteria from Spec)

**Primary Success (60% emphasis): Decomposition Thinking**
- SC-001: 80%+ decompose complex systems into 3-5 independent units (capstone proves this)
- SC-002: Can articulate when parallelization adds value (reflection exercises in each lesson)
- SC-003: 75%+ integrate independently-built features with minimal conflicts (Lesson 3 integration validation)
- SC-004: Can explain decomposition thinking to non-technical stakeholders (Lesson 9 portfolio narrative)
- SC-005: 70%+ understand transferability to junior developers/teams (Lesson 4 + Lesson 9 reflection)
- SC-013: 60%+ understand scaling pathway 2-3 → 5-7 → 7-9 agents (Lessons 4, 8, 9)

**Secondary Success (40% emphasis): Tool Proficiency**
- SC-006: 70%+ achieve 2.5x+ speedup with parallel workflow (time tracking in Lessons 1-3, capstone)
- SC-007: 80%+ successfully use git worktrees, SpecKit Plus, clean merges (Lessons 1-3, 9)
- SC-008: 70%+ understand path to 10x via automation (Lessons 5-7)
- SC-009: 50%+ successfully run orchestration script (Lesson 8, optional)

---

## Next Steps

This plan is complete and ready for implementation phase. The task checklist (`tasks.md`) will break down each lesson into specific development tasks (content writing, code examples, exercises, validations).

**Implementation will follow SpecKit Plus workflow**:
1. Chapter README.md (overview, learning outcomes)
2. Lessons 1-9 (content, code examples, exercises, validations in parallel across multiple worktrees or sequentially)
3. Integration & final review
4. Publication

---


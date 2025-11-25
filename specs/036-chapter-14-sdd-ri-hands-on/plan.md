# Implementation Plan: Chapter 14 — SDD-RI Hands-On with Spec-Kit Plus

**Status**: Ready for Lesson Writer Implementation
**Feature Branch**: `036-chapter-14-sdd-ri-hands-on`
**Chapter Type**: Technical/Project-Based (Intermediate Hands-On Learning)
**Part**: 4 (SDD-RI Fundamentals)
**Proficiency Tier**: A2-B1 (Beginner transitioning to Intermediate)
**Constitutional Version**: 6.0.1
**Created**: 2025-11-25
**Total Estimated Time**: 6-8 hours (including capstone project)

---

## Executive Summary

Chapter 14 transforms SDD-RI theory (Chapter 13) into hands-on practice through a complete, real-world project: building video generation + YouTube upload automation using Spec-Kit Plus workflow. Students progress through four pedagogical layers—Manual Foundation (spec writing, no AI) → AI Collaboration (workflow planning with AI) → Intelligence Design (creating reusable skills) → Spec-Driven Mastery (capstone orchestration)—accumulating reusable intelligence with each lesson.

**Key Pedagogical Distinction**: While Chapter 13 taught theory through problem-discovery (analyzing vague code failures), Chapter 14 uses **project-based hands-on learning** where students execute the complete `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` workflow. The teaching modality varies from Chapter 13 by emphasizing *execution and pattern recognition* rather than *conceptual analysis*.

**Intelligence Accumulation Arc**: Lessons 1-5 execute the workflow for video generation. Lessons 6-7 crystallize patterns into two reusable skills (`generate-video`, `upload-youtube`). Lessons 8-9 compose these skills into the capstone YouTube upload project, demonstrating how accumulated intelligence accelerates iteration.

---

## Pedagogical Architecture: 4-Layer Progression

### Layer 1: Manual Foundation (Lessons 1-2)
**Goal**: Establish mental models for SDD workflow without AI assistance

- **Lesson 1**: Project context + business framing (no AI, pure context-setting)
- **Lesson 2**: Manual specification writing with guided structure (manual effort, AI supports only for validation)

**Cognitive Approach**: Students see the "what" (specification) before the "how" (implementation). AI remains in background, only validating student intent.

### Layer 2: AI Collaboration (Lessons 3-5)
**Goal**: Experience planning and execution with AI as active partner (all three roles invisible but present)

- **Lesson 3**: AI planning phase—students articulate approach, AI suggests refinements (Teacher role)
- **Lesson 4**: Task breakdown with student corrections and refinements (Student role)
- **Lesson 5**: Implementation and iteration toward working video (Co-Worker role—convergence)

**Cognitive Approach**: Students experience AI teaching (suggesting better approaches), being taught (correcting AI output), and co-creating (iterative refinement). Framework labels remain invisible; students experience roles through action narratives.

### Layer 3: Intelligence Design (Lessons 6-7)
**Goal**: Recognize patterns and encapsulate as reusable skills using Persona + Questions + Principles

- **Lesson 6**: Pattern recognition—identify where `generate-video` and `upload-youtube` skills emerged
- **Lesson 7**: P+Q+P skill design—create two reusable skills with reasoning-activated structure

**Cognitive Approach**: Students become designers, not just executers. They analyze what made their workflow successful and abstract it into components reusable across future projects.

### Layer 4: Spec-Driven Mastery (Lessons 8-9)
**Goal**: Orchestrate accumulated intelligence (skills from L3) into new context (YouTube upload)

- **Lesson 8**: YouTube upload specification (spec-first, no code yet)
- **Lesson 9**: Compose L3 skills to execute spec, capstone validation

**Cognitive Approach**: Specification becomes the primary artifact. AI orchestrates using composed skills. Students validate that specifications execute correctly without rework.

---

## Chapter Learning Arc & Lesson Structure

### Arc Overview
```
Foundation (L1-3)     → Application (L4-8)   → Integration (L9)   → Real-World (L10) → Mastery (L11)
Setup + Constitution  → SDD-RI Workflow      → Skill Creation     → Brownfield        → Capstone
Tool Understanding    → Collaborative Exec   → Designing RI       → Adoption          → Orchestrating
```

### Alignment with Existing Chapter Structure

**Key Decision**: Keep existing 10-lesson structure + add Capstone as Lesson 11. This preserves valuable Brownfield content while adding the intelligence accumulation demonstration.

| # | Current Lesson | New Content Strategy |
|---|----------------|---------------------|
| 01 | Foundation (H/V Intelligence) | **KEEP & IMPROVE** — Same concepts, better examples |
| 02 | Installation & Setup | **KEEP & EXTEND** — Add Playwright MCP setup |
| 03 | Constitution Phase | **KEEP & PIVOT** — Same pattern, video project context |
| 04 | Specify Phase | **RECREATE** — Calculator → Video generation spec |
| 05 | Clarify Phase | **KEEP & PIVOT** — Same pattern, video examples |
| 06 | Plan Phase | **RECREATE** — Playwright MCP + Gemini approach |
| 07 | Tasks Phase | **RECREATE** — Video generation tasks |
| 08 | Implement Phase | **RECREATE** — Video generation execution |
| 09 | Designing RI | **KEEP STRUCTURE** — P+Q+P stays, new skill examples |
| 10 | Brownfield Adoption | **KEEP & IMPROVE** — Valuable real-world content |
| 11 | YouTube Capstone | **NEW** — Skills reuse + validation |
| Quiz | Chapter Quiz | **RECREATE** — Video domain questions |

### Total Lesson Count Justification

**Core Concepts Identified** (from spec requirements):
1. Spec-Kit Plus architecture (Horizontal/Vertical Intelligence)
2. Tool installation and configuration
3. Constitution creation (project-wide rules)
4. Specification writing (intent + constraints)
5. Specification refinement (clarify phase)
6. Planning phase decision-making (tech approach selection)
7. Task breakdown and atomization
8. Implementation with checkpoint validation
9. Pattern recognition and P+Q+P skill creation
10. Brownfield adoption (real-world projects)
11. Skill composition and capstone validation

**Complexity Assessment**: Standard-to-Complex (11 concepts across full SDD-RI workflow)

**A2-B1 Tier Progression**:
- Lessons 1-3 (Foundation): A2 level (tool setup, heavy scaffolding)
- Lessons 4-8 (Application): A2-B1 transition (workflow execution, moderate scaffolding)
- Lesson 9 (Integration): B1 level (skill design, student-directed)
- Lesson 10 (Real-World): B1 level (brownfield adoption)
- Lesson 11 (Mastery): B1 level (composition + validation, minimal scaffolding)

**Justified Count**: 11 lessons + quiz (preserves existing valuable content, adds capstone for intelligence accumulation demonstration)

---

## Lesson-by-Lesson Architecture

---

### LESSON 1: Your Video Generation Project (Layer 1: Context & Business Framing)

**Phase**: Foundation
**Layer**: 1 (Manual Foundation)
**CEFR Level**: A2 (Beginner)
**Bloom's Level**: Remember + Understand
**Estimated Time**: 45 minutes

#### Learning Objectives
- Recognize the business context for product demo videos (SaaS onboarding, feature showcase)
- Understand the complete workflow from concept to YouTube publication
- Identify key decisions that will shape specification writing
- Explain why specification clarity matters before implementing

#### New Concepts (Count: 5)
1. **Product Demo Video Purpose** — Showcasing software features for onboarding/marketing
2. **End-to-End Workflow** — Spec → Plan → Tasks → Implementation
3. **Business Success Metrics** — How do we measure if the video accomplishes its goal?
4. **Technical Constraints** — Free tools (Gemini), browser automation (Playwright)
5. **Specification as Blueprint** — Intent, constraints, success criteria defined before execution

#### Cognitive Load Validation
- 5 concepts ≤ A2 limit (7) ✓ Compliant
- No code yet; pure context-setting
- Heavy scaffolding through narrative framing

#### Content Structure

**Section 1: The Scenario**
- Business context: SaaS company needs product demo videos
- Typical workflow: "Show feature X → Explain why it matters → CTA to sign up"
- Why video matters: 45-second video + upload to YouTube = marketing momentum
- Real-world success metric: 100 views in first month

**Section 2: The Workflow You'll Follow**
- Spec: "What's the product? What's the message? Who's the audience?"
- Plan: "How will we use Gemini AI + Playwright to automate video generation?"
- Tasks: "What are atomic steps to execute the plan?"
- Implementation: "Follow tasks, validate output, iterate if needed"
- Parallel: "Create skills that other projects can reuse"

**Section 3: Why Specification Clarity Matters**
- Vague spec → AI generates vague video → rework needed → time lost
- Clear spec → AI generates video matching intent → minimal iteration → success
- Type hints in Python describe intent; specifications describe project intent
- Student mindset: "I write clear intent, AI executes it"

**Section 4: Project Constraints (Establish Boundaries)**
- Must use Gemini.google.com (no API keys, free tier)
- Must use Playwright MCP for automation (no manual UI clicking)
- Must generate video file + upload to YouTube
- Must work on macOS, Linux, Windows

#### Intelligence Accumulation
- This lesson establishes the **business framing** that drives all technical decisions
- Students understand WHY they're learning SDD-RI (to build reusable intelligence for marketing automation)

#### Three Roles Framework (Invisible)
- Students will encounter AI as Teacher (suggesting better spec structures) in later lessons
- Setup here: "Your job is to write clear intent; AI's job is to execute it"

#### Prerequisites
- Completed Chapter 13 (SDD-RI theory)
- Basic familiarity with Spec-Kit Plus concepts

#### Try With AI
**Activity: Specification Brainstorm**
Ask Claude: "I'm building a 45-second product demo video for [your product idea]. What should I include in the specification? What questions should I answer before implementation?"
- Observe: What patterns does AI suggest for spec structure?
- Refine: Which suggestions match your constraints? Which don't?
- Reflect: How does clear specification prevent rework later?

---

### LESSON 2: Write Your First Specification (Layer 1: Manual Foundation)

**Phase**: Foundation
**Layer**: 1 (Manual Foundation)
**CEFR Level**: A2 (Beginner)
**Bloom's Level**: Apply
**Estimated Time**: 90 minutes

#### Learning Objectives
- Write a complete specification following Spec-Kit Plus template (intent, constraints, evals)
- Distinguish between specification (intent) and implementation (approach)
- Apply success criteria thinking (how do we know if the video works?)
- Validate specification completeness using checklist

#### New Concepts (Count: 6)
1. **Specification Template** — Intent, constraints, success evals, non-goals
2. **Intent Clarity** — "What are we building and why?"
3. **Constraints Definition** — "What are the boundaries?"
4. **Success Evals** — "How do we measure success?"
5. **Non-Goals** — "What are we explicitly NOT doing?"
6. **Specification Validation** — Checklist to ensure completeness

#### Cognitive Load Validation
- 6 concepts ≤ A2 limit (7) ✓ Compliant
- Heavy scaffolding through template guidance
- Students write manually with structured prompts

#### Content Structure

**Section 1: The Spec-Kit Plus Template (Walkthrough)**
- Intent: What product are we showcasing? What message? Target audience?
  - Example: "SaaS onboarding video for [Product] → Show feature X → CTA to sign up"
- Constraints: What tools? Time limit? Quality standards?
  - Example: "Use Gemini (free), Playwright automation, 45 seconds, 1080p"
- Success Evals: How do we know the video accomplishes its goal?
  - Example: "Video generates 100+ views in first month", "Product signup page gets 20% clickthrough from video viewers"
- Non-Goals: What are we explicitly NOT doing?
  - Example: "NOT doing advanced animation editing", "NOT using expensive tools"

**Section 2: Writing Your Project's Intent**
- Guided prompts:
  - What specific product feature are you showcasing?
  - Why should someone care about this feature?
  - Who's your target audience (new users? existing customers? partners?)?
  - What's the call-to-action at the end?
- Student writes 4-6 sentence intent paragraph
- Validation: "Is someone unfamiliar with your product clear on what you're building?"

**Section 3: Defining Your Constraints**
- Technical constraints: Gemini (free), Playwright, no API keys
- Duration constraint: 30-60 seconds
- Quality constraints: 1080p or better, clear audio
- Process constraint: Completed by [date]
- Student identifies 4-5 constraints relevant to their project

**Section 4: Success Evals (How Do We Know It Works?)**
- Not "completion" evals ("video generates") but **outcome** evals
- Examples:
  - "Video is downloaded and plays without errors"
  - "Video successfully uploads to YouTube with correct metadata"
  - "Video view count increases by 50+ in first week"
- Student writes 2-3 measurable, observable success evals

**Section 5: Non-Goals (What Are We NOT Doing?)**
- Student explicitly states what they're not tackling:
  - "NOT doing professional color grading"
  - "NOT optimizing for TikTok (YouTube only)"
  - "NOT creating multiple variations"
- This prevents scope creep

**Section 6: Validation Checklist**
- Does intent answer: What? Why? Who?
- Do constraints include: Technical, duration, quality, process?
- Are evals observable and measurable (not subjective)?
- Does specification fit on 1-2 pages (not a novel)?
- Checklist helps ensure specs aren't vague

#### Intelligence Accumulation
- Students learn the discipline of **specification-first thinking**
- This spec becomes the blueprint for all planning + implementation in Lessons 3-5
- Lesson 6 will analyze patterns from this spec → skill extraction

#### Three Roles Framework (Invisible)
- Student role: "I write clear intent"
- AI role (coming in Lesson 3): "I help refine intent + suggest better approaches"
- Framework stays invisible; student just experiences writing with AI feedback

#### Prerequisites
- Lesson 1 (context and business framing)

#### Try With AI
**Activity: Specification Refinement Review**
Ask Claude: "Here's my specification for my product video: [paste your spec]. Is the intent clear? Are the constraints realistic? Do the success evals measure what matters?"
- Observe: What ambiguities does AI highlight?
- Refine: Rewrite 1-2 sentences to clarify intent based on feedback
- Validate: Does the refined spec now pass all checklist items?
- Reflect: How did writing spec-first clarify what you're actually building?

---

### LESSON 3: Plan Your Approach (Layer 2: AI Collaboration)

**Phase**: Application
**Layer**: 2 (AI Collaboration)
**CEFR Level**: A2-B1 (Beginner-Intermediate)
**Bloom's Level**: Apply + Analyze
**Estimated Time**: 90 minutes

#### Learning Objectives
- Execute `/sp.plan` command on your specification
- Understand the technical approach: Gemini AI calls via Playwright browser automation
- Analyze AI's plan against your constraints
- Identify decision points where student judgment improves the plan
- Demonstrate first experience of AI as Teacher (suggesting approaches) and Student (adapting to feedback)

#### New Concepts (Count: 6)
1. **Planning Phase Purpose** — Translating spec into technical approach
2. **Playwright MCP for Browser Automation** — Automating Gemini.google.com interactions
3. **Session Persistence** — Login once, automate multiple actions
4. **Gemini Video Generation Workflow** — Steps to generate video through UI
5. **Error Handling Strategy** — What if video generation fails? Retry logic.
6. **Task Decomposition** — Breaking plan into atomic tasks

#### Cognitive Load Validation
- 6 concepts ≤ B1 limit (10) ✓ Compliant
- AI provides plan framework; student adapts based on constraints
- Moderate scaffolding (student applies judgment)

#### Content Structure

**Section 1: Running `/sp.plan` (Spec-Kit Plus Command)**
- You've written spec.md; now generate plan.md
- Command: `/sp.plan` in Claude Code
- Output: 2-3 page technical approach document
- What to expect: Technical approach, lesson breakdown, tool choices
- Example: "Use Playwright MCP to automate Gemini.google.com → generate video → save locally"

**Section 2: Understanding Playwright MCP for This Project**
- Playwright: Browser automation tool (clicks buttons, fills forms, reads content)
- Why Playwright? Free, open-source, works across platforms
- For this project: Playwright automates Gemini UI (humans can't easily generate videos at scale)
- MCP = Making Claude Protocol (AI access to Playwright)
- Practical setup: "You'll write specs; Playwright executes the clicks/interactions"

**Section 3: Analyzing the AI-Generated Plan**
- AI proposes approach; you evaluate
- Key decisions to check:
  - Does the plan use free tools (Gemini, not paid APIs)?
  - Are steps realistic (can we actually automate Gemini)?
  - Does it handle authentication (session persistence)?
  - What about failure cases (retry logic)?
- Student perspective: "AI suggested a path; I validate if it's correct"

**Section 4: Identifying Gaps in the Plan**
- Common gaps:
  - Missing error handling ("What if Gemini service is down?")
  - Unclear dependencies ("Do we need to log in for each video or once?")
  - Undefined outputs ("Where does the video file go?")
- Your job: Flag these gaps before implementation
- This is **AI as Student**—AI wrote the plan; you're teaching it where it's incomplete

**Section 5: Refining the Plan with Your Constraints**
- Back-and-forth with AI:
  - You: "My Playwright knowledge is limited; how can we simplify?"
  - AI: "Here's a more straightforward approach: [revised steps]"
  - You: "Better. But I also need session persistence to work on Windows—can we verify that?"
  - AI: "Yes, Playwright sessions persist across OS; here's how to set it up..."
- This is **AI as Co-Worker**—iterative refinement toward production approach

#### Intelligence Accumulation
- Plan creates the **technical blueprint** for video generation
- In Lesson 5, you'll execute this plan
- In Lesson 6, you'll recognize that this plan + patterns = the `generate-video` skill

#### Three Roles Framework (Invisible in Student Experience)
- **AI as Teacher**: AI proposes a technical approach student might not have considered (Playwright MCP for browser automation)
- **AI as Student**: Student refines plan where AI missed considerations (error handling, Windows compatibility)
- **AI as Co-Worker**: Iterative back-and-forth refines plan from rough sketch to production-ready (no single perfect plan; convergence through dialogue)

**Framework Language (Student-Facing)**:
- "AI suggests → You evaluate" (not "AI as Teacher")
- "You refine → AI adapts" (not "AI as Student")
- "Together you iterate → Better approach emerges" (not "AI as Co-Worker")

#### Prerequisites
- Lesson 1-2 (specification written and validated)

#### Try With AI
**Activity: Plan Evaluation & Refinement Loop**
1. Run `/sp.plan` using your spec.md
2. Read the generated plan; identify 2-3 risks or gaps
   - "What if Gemini is slow?" / "How do we handle retries?"
   - "How do we verify the video is correct quality?"
3. Ask Claude: "In my plan, I see a gap: [describe gap]. How should we handle this?"
4. Listen: AI explains a refinement or confirms your concern
5. Refine: Update plan.md with improved steps based on feedback
- Observe: How does iteration improve the plan?
- Reflect: What did AI teach you about technical approaches? What did you teach AI about your constraints?

---

### LESSON 4: Break Down Tasks (Layer 2: AI Collaboration)

**Phase**: Application
**Layer**: 2 (AI Collaboration)
**CEFR Level**: B1 (Intermediate)
**Bloom's Level**: Analyze + Apply
**Estimated Time**: 90 minutes

#### Learning Objectives
- Execute `/sp.tasks` command to generate atomic work units
- Understand task dependencies and ordering
- Validate tasks are atomic (small enough to complete in 15-30 minutes)
- Identify checkpoints where implementation validates against spec
- Practice detailed AI feedback (teacher → student → co-worker iteration)

#### New Concepts (Count: 5)
1. **Task Atomicity** — Tasks small enough to complete in one focus session
2. **Dependency Ordering** — Which tasks must complete before others
3. **Acceptance Criteria** — How do we know each task succeeded?
4. **Checkpoint Validation** — Intermediate checks to ensure progress matches spec
5. **Implementation Readiness** — Tasks detailed enough that AI can execute them correctly

#### Cognitive Load Validation
- 5 concepts ≤ B1 limit (10) ✓ Compliant
- Students analyze dependencies (higher-order thinking)
- Moderate scaffolding (AI generates tasks; student validates/refines)

#### Content Structure

**Section 1: What Does `/sp.tasks` Produce?**
- Takes plan.md → generates tasks.md
- Tasks are atomic work units with dependencies
- Example task structure:
  ```
  Task 1.1: Set up Playwright environment
    - Acceptance: Playwright runs `playwright --version`
    - Prerequisite: None

  Task 1.2: Set up session persistence for Gemini login
    - Acceptance: Session persists across 3 separate Playwright runs
    - Prerequisite: Task 1.1 complete
  ```
- Total: 8-12 tasks covering spec → video generation → download

**Section 2: Task Atomicity (Right Sizing)**
- Too big: "Implement video generation" (1-2 weeks of work, can't validate progress)
- Too small: "Click the Gemini UI button" (5 seconds, trivial)
- Right size: "Automate Gemini prompt input + video generation + download" (2-3 hours, produces tangible output)
- Guideline: "Can I complete this in 15-30 minutes? Can I validate it worked?"

**Section 3: Understanding Dependencies**
- Some tasks have prerequisites:
  - Task 1.1 (Setup Playwright) → Task 1.2 (Login) → Task 2.1 (Submit prompt to Gemini)
  - Can't do Task 2.1 until 1.1 + 1.2 complete
- Students trace dependency graph:
  - "Which tasks can run in parallel?"
  - "Which tasks block others?"
  - "What's the critical path (sequence of blocking tasks)?"

**Section 4: Acceptance Criteria (How Do We Know It's Done?)**
- Each task needs measurable completion criteria
- Bad: "Implement login" (too vague—did it work? Is it secure?)
- Good: "Automate Gemini login via Playwright; verify session persists for 2+ subsequent page loads"
- Student practice: "For each task, write 1-2 sentence acceptance criteria"

**Section 5: Validating Task Generation**
- Run `/sp.tasks`
- AI generates 10-12 tasks
- Your evaluation:
  - Are tasks sized correctly (15-30 min each)?
  - Do dependencies make sense?
  - Are acceptance criteria measurable?
  - Anything missing or unclear?
- This is **AI as Student**—you're teaching AI to generate better tasks

**Section 6: Refining Tasks Before Implementation**
- If a task is too vague, ask AI to clarify
  - You: "Task 2.1 says 'submit prompt to Gemini.' What exactly should the prompt be?"
  - AI: "Good catch. Here's a detailed prompt that Gemini will understand: '[specific prompt structure]'"
- If a task is too big, ask AI to split it
  - You: "Task 2.2 covers both video generation AND quality check. Can we split that?"
  - AI: "Absolutely. Let's make that two tasks with a checkpoint between them."
- This is **AI as Co-Worker**—collaborative refinement

#### Intelligence Accumulation
- Tasks become the **execution plan** for Lesson 5
- Checkpoint structure helps identify patterns → skills in Lesson 6

#### Three Roles Framework (Invisible)
- **AI as Teacher**: Generates task structure and dependency ordering (patterns student might not have considered)
- **AI as Student**: Adapts when student identifies gaps or over-sized tasks
- **AI as Co-Worker**: Iterative refinement makes tasks clearer and more executable

#### Prerequisites
- Lesson 3 (plan completed and refined)

#### Try With AI
**Activity: Task Dependency Analysis & Refinement**
1. Run `/sp.tasks` using your plan.md
2. Map task dependencies: "Which tasks block others? Which can run in parallel?"
3. Identify 2-3 tasks that feel unclear or too big
4. Ask Claude: "For task [X], I'm not clear on [specific uncertainty]. Can you provide more detail?"
5. Listen: AI provides clarification or suggests splitting task
6. Refine: Update tasks.md with clearer acceptance criteria
- Observe: How does task clarity affect your confidence to implement?
- Reflect: What's the relationship between clear specs → clear plans → clear tasks?

---

### LESSON 5: Generate Your First Video (Layer 2: AI Collaboration)

**Phase**: Application
**Layer**: 2 (AI Collaboration)
**CEFR Level**: B1 (Intermediate)
**Bloom's Level**: Apply + Create
**Estimated Time**: 3-4 hours (with breaks for AI processing time)

#### Learning Objectives
- Execute `/sp.implement` command to generate code that automates video generation
- Run generated code and validate it produces a video file
- Debug failures using error messages and AI guidance
- Practice iteration loops (specification → implementation → validation → refinement)
- Recognize convergence: "Neither student nor AI had perfect solution initially; iteration made it work"

#### New Concepts (Count: 6)
1. **Implementation from Specification** — AI generates code matching spec intent
2. **Checkpoint Validation** — Checking output at each task completion
3. **Error Debugging** — Interpreting errors and refining prompts
4. **Session Persistence Patterns** — Logging in once, automating multiple actions
5. **Video File Verification** — Confirming video quality and completeness
6. **Iteration Loops** — Refining when initial attempts don't match spec

#### Cognitive Load Validation
- 6 concepts ≤ B1 limit (10) ✓ Compliant
- Students execute code and validate output (applied, hands-on)
- Minimal scaffolding (mostly student problem-solving with AI support)

#### Content Structure

**Section 1: Running `/sp.implement` (Generating Code)**
- You now have: spec.md (intent), plan.md (approach), tasks.md (atomic steps)
- Run `/sp.implement` in Claude Code
- Output: Python scripts that automate Gemini video generation
- What you get: 3-5 files (playwright_automation.py, gemini_prompts.py, video_utils.py, etc.)

**Section 2: Setting Up Playwright & Session Persistence**
- Install Playwright if not already installed: `pip install playwright`
- Create a session directory for browser state
- First run: Manually log into Gemini (browser opens, you authenticate)
- Subsequent runs: Playwright uses persisted session (no manual login)
- Walkthrough: Step-by-step guide to verify session works (test 2 separate Python runs)

**Section 3: Running the First Implementation**
- Execute: `python playwright_automation.py --product "My Product" --message "Key feature to highlight"`
- What happens: Playwright opens browser, navigates Gemini, fills prompt, watches video generation
- Expected output: Video file saved locally (e.g., `~/videos/demo_product_v1.mp4`)
- Duration: 3-5 minutes (depends on Gemini processing time)
- Student role: Monitor process, watch browser automation in action

**Section 4: Validating the Video**
- Checklist:
  - ✓ Video file exists locally?
  - ✓ Video plays without errors?
  - ✓ Duration matches spec (30-60 seconds)?
  - ✓ Quality acceptable (1080p or better)?
  - ✓ Content matches intent (shows the feature clearly)?
- If all checks pass → Checkpoint complete → Move to next task
- If any check fails → Debug (next section)

**Section 5: Debugging & Iteration (AI as Co-Worker)**
- Common failures:
  - "Gemini didn't generate video—timed out" → Retry with longer timeout
  - "Video quality is low" → Refine prompt: "Use high-quality 1080p output"
  - "Selectors changed on Gemini UI" → Update CSS selectors in code
- Iteration loop:
  1. Observe failure: "Video quality is grainy; doesn't meet 1080p spec"
  2. Ask Claude: "In my implementation, the Gemini video output is grainy. How can I refine the prompt to request higher quality?"
  3. Listen: AI suggests specific prompt modifications
  4. Refine: Update prompt, re-run implementation
  5. Validate: Does refined version meet spec?
- This is true **convergence**: Neither student nor AI had perfect solution (student didn't know Gemini prompt patterns; AI didn't know your quality standards). Iteration combined both to produce correct output.

**Section 6: Checkpoints & Progress Tracking**
- After each task (upload, login, prompt submit, video generate, download), mark in tasks.md:
  - [ ] Task name — Not started
  - [x] Task name — Complete, video generated v1
  - [x] Task name — Refined, video meets 1080p quality
- This creates a record: "Here's what worked; here's what we iterated on"

#### Intelligence Accumulation
- Executing the workflow reveals **patterns**: Where do students make refinements? Where does AI add value?
- In Lesson 6, these patterns → recognizing what should become reusable skills
- Video file is tangible evidence of successful workflow

#### Three Roles Framework (Invisible but Prominent Here)
- **AI as Teacher**: Suggests better prompts for Gemini (student didn't know "use 1080p output" syntax)
- **AI as Student**: Learns from student feedback ("That didn't work; try this instead")
- **AI as Co-Worker**: "Neither of us had the perfect prompt on first try. Your feedback + my suggestions = video that works"

**Student Language**:
- "I ran the code → video didn't look right"
- "I asked Claude → got a better prompt suggestion"
- "I updated the code → ran again → video quality improved"
- (NO framework labels: no "Three Roles", no "AI as Teacher")

#### Prerequisites
- Lessons 1-4 (spec → plan → tasks complete)

#### Try With AI
**Activity: Full Implementation Cycle**
1. Run `/sp.implement` to generate Python scripts
2. Set up Playwright environment (installation, session persistence test)
3. Execute: `python playwright_automation.py --product "[Your Product]" --message "[Key Message]"`
4. Validate: Does video match your spec?
   - Duration: 30-60 seconds?
   - Quality: 1080p or better?
   - Content: Shows feature clearly?
5. If not meeting spec:
   - Ask Claude: "My video [specific issue]. How should I refine the prompt?"
   - Update code based on suggestion
   - Re-run; validate again
6. Success: Video downloaded locally, matches spec intent
- Observe: How many iterations did it take? What did each iteration improve?
- Reflect: How is this different from "just asking AI to generate a video"? Why does specification-first matter?

---

### LESSON 6: Recognize Patterns (Layer 3: Intelligence Design)

**Phase**: Integration
**Layer**: 3 (Intelligence Design)
**CEFR Level**: B1 (Intermediate)
**Bloom's Level**: Analyze + Evaluate
**Estimated Time**: 90 minutes

#### Learning Objectives
- Analyze the workflow from Lessons 1-5 to identify recurring patterns
- Distinguish between project-specific code and reusable intelligence patterns
- Recognize where two skills (`generate-video`, `upload-youtube`) naturally emerge
- Understand why skill extraction enables reuse in new contexts (Lesson 9)
- Develop meta-awareness of their own learning ("Where did I learn this pattern?")

#### New Concepts (Count: 5)
1. **Pattern Recognition** — Identifying recurring workflows across projects
2. **Skill vs. Project Code** — What's reusable? What's project-specific?
3. **Abstraction** — Generalizing specific (Gemini video) to generic (generate any video)
4. **Reusability Criteria** — "Will I use this pattern in future projects?"
5. **Skill Boundaries** — Clear inputs/outputs for reusable components

#### Cognitive Load Validation
- 5 concepts ≤ B1 limit (10) ✓ Compliant
- Students analyze own work (metacognitive awareness)
- Minimal scaffolding (guided discovery of patterns they created)

#### Content Structure

**Section 1: Reflection Prompt—What Did You Just Do?**
- Write 1-2 paragraphs answering:
  - "What was the hardest part of the video generation workflow?"
  - "Where did you iterate most? Why?"
  - "What decisions did you make that weren't obvious at the start?"
- Example reflection:
  - "The hardest part was figuring out the right Gemini prompt. I iterated 3-4 times before the video quality was acceptable."
  - "I made decisions about session persistence, retry logic, error handling—these aren't in the basic workflow; they're patterns I learned."

**Section 2: Pattern 1—Video Generation Workflow**
- Specific to this project: "Generate demo video for SaaS onboarding"
- Reusable pattern: "Given product + message + audience → generate video using Gemini + Playwright automation"
- Generalizations:
  - "Product" could be: SaaS feature, product feature, tutorial topic, marketing message
  - "Message" could be: feature benefit, use case, problem-solution
  - "Audience" could be: new users, existing customers, partners, students
- Decision points (Persona questions):
  - What are we showcasing?
  - Who are we showcasing it to?
  - What's the key message?
- This is the seeds of the **`generate-video` skill**

**Section 3: Pattern 2—Video Upload Workflow**
- Looking ahead to Lesson 8-9: After generating video, you'll upload to YouTube
- Reusable pattern: "Given video file + metadata (title, description, tags) → upload to YouTube + get URL"
- Generalizations:
  - "Video file" could be: marketing video, tutorial, webinar recording, product demo
  - "Metadata" could be: marketing-focused (SEO tags, description), tutorial-focused (timestamps, chapters), product-focused (feature list)
- Decision points (Persona questions):
  - What's the video title?
  - What's the description (SEO-optimized)?
  - What tags/categories?
  - Private or public?
- This is the seeds of the **`upload-youtube` skill**

**Section 4: Criteria for "This Should Be a Skill"**
- Recurring: "Will I use this pattern in future projects?" (YES for video generation and YouTube upload)
- Reusable: "Can I generalize from this specific project?" (YES—other products need videos; other videos need uploading)
- Complex: "Does it have 2-4 key decision points?" (YES—both patterns have clear decisions)
- Boundaries: "Can I define clear inputs and outputs?" (YES—product/message → video; video/metadata → URL)

**Section 5: Mapping Lessons to Skills**
- Lessons 1-5 execute the workflow: Many decisions, many iterations
- Lessons 6-7 extract patterns: "Here's the repeatable structure"
- Lessons 8-9 reuse skills: "Apply this pattern to new context (YouTube upload)"
- Show flow:
  ```
  Lesson 1-5: Execute specific project
    ↓ (Lessons 6-7: Extract patterns)
  Skill 1: generate-video (reusable structure)
  Skill 2: upload-youtube (reusable structure)
    ↓ (Lessons 8-9: Reuse in new context)
  Lesson 8-9: YouTube upload project
  ```

#### Intelligence Accumulation
- Recognizing patterns is a **meta-skill**: Students learn to extract intelligence from their own work
- Skills created in Lesson 7 will be reused in Lesson 9 (capstone)
- This is SDD-RI core: accumulate reusable intelligence → apply to future projects → work faster

#### Three Roles Framework (Invisible)
- Student reflects: "What did I learn from this project?"
- AI role (Lesson 7): "Help me structure this pattern into a reusable skill"
- No framework labels in student experience

#### Prerequisites
- Lesson 5 (video generation workflow complete with iterations)

#### Try With AI
**Activity: Pattern Extraction Dialogue**
1. Reflect: Write 1-2 paragraphs on the question: "If I had to build another product demo video in 3 months, what from my video generation project could I reuse? What would be different?"
2. Ask Claude: "I'm thinking about extracting a reusable 'video generation' skill from my project. Here's what I did: [summarize the workflow]. What parts are generic/reusable? What parts are specific to my product?"
3. Listen: AI identifies patterns and potential abstractions
4. Analyze: "Which patterns could I turn into a skill? What decision points would the skill guide?"
5. Document: Jot down the 2-4 key questions/decisions for each skill
- Observe: How does extraction help you see the workflow differently?
- Reflect: Why are reusable patterns valuable? How will they help in Lesson 9?

---

### LESSON 7: Create Reusable Skills (Layer 3: Intelligence Design)

**Phase**: Integration
**Layer**: 3 (Intelligence Design)
**CEFR Level**: B1 (Intermediate)
**Bloom's Level**: Create
**Estimated Time**: 120 minutes

#### Learning Objectives
- Apply Persona + Questions + Principles (P+Q+P) framework to design reusable skills
- Create two skills: `generate-video` and `upload-youtube` following SKILL.md template
- Write clear skill descriptions that activate reasoning (not just template filling)
- Understand why skills are different from code libraries (intent-capturing, not syntax)
- Prepare skills for reuse in Lesson 9 capstone

#### New Concepts (Count: 6)
1. **Persona in Skills** — Who would use this skill? What's their role?
2. **Questions Framework** — What decisions guide skill application?
3. **Principles Framework** — What values/heuristics constrain decisions?
4. **Skill vs. Code** — Skills capture intent; code implements mechanics
5. **SKILL.md Template** — Structure for documenting reusable intelligence
6. **Skill Composition** — How skills combine in future projects

#### Cognitive Load Validation
- 6 concepts ≤ B1 limit (10) ✓ Compliant
- Students create concrete artifacts (skill documents)
- Minimal scaffolding (template-guided but student-created content)

#### Content Structure

**Section 1: P+Q+P Framework Walkthrough**

**Persona: Who uses this skill?**
- Persona defines the **user context** and role
- Example for `generate-video`:
  - "Video Content Producer: Someone creating product demo videos for marketing purposes"
  - Implies: Thinks about audience, key message, brand consistency
  - Not: "Someone who codes" (focus is on role, not technical skill)

**Questions: What decisions does the skill guide?**
- Questions structure the decision-making process
- 2-4 key questions that users answer to apply the skill
- Example for `generate-video`:
  - "What product or feature are we showcasing?" (defines the subject)
  - "What's the primary message or benefit?" (defines the focus)
  - "Who's the target audience?" (defines tone and complexity)
  - "What call-to-action do we want?" (defines closure)
- Questions are **open-ended** (not yes/no), guiding creative thinking

**Principles: What values constrain the skill?**
- Principles are **heuristics**, not rules
- They reflect patterns from the project (Lesson 5)
- Example for `generate-video`:
  - "Visual clarity: The video should immediately show what we're demonstrating (not abstract or confusing)"
  - "Concise messaging: Assume viewer attention = 30 seconds; every second counts"
  - "Brand consistency: Tone and visual style align with company brand guidelines"
- Principles help users **reason** about applying the skill (not just follow steps)

**Section 2: Skill 1—`generate-video`**

**Structure**:
```markdown
# SKILL: generate-video

## Persona
Video Content Producer—someone creating product demo videos for marketing, onboarding, or feature showcases.

## Questions
1. What product or feature are you showcasing?
2. What's the primary message or benefit you want to communicate?
3. Who's your target audience (new users? existing customers? partners?)?
4. What call-to-action do you want at the end (sign up, try beta, learn more)?

## Principles
- **Visual Clarity**: The video should immediately show what you're demonstrating. Avoid abstract animations or lengthy introductions.
- **Concise Messaging**: Assume viewer attention is 30-45 seconds. Every second counts; cut unnecessary pauses.
- **Brand Consistency**: Tone of voice, color palette, and visual style should align with company brand guidelines.
- **Product Focus**: Lead with the feature/benefit, not the company name. Viewers want to know "what's in it for me," not "who made it."

## When to Use This Skill
- Creating product demo videos for marketing websites
- Generating onboarding videos for new product features
- Producing tutorial videos explaining software features
- Automating video generation at scale (batch process multiple videos)

## When NOT to Use This Skill
- Complex video editing (color grading, transitions, special effects)
- Narrative storytelling (brand documentaries, founder stories)
- Live-action filming (real people, real locations)
- Animated explainer videos (requires custom animation design)

## Implementation Notes
- Uses Gemini.google.com video generation feature (free tier)
- Automates via Playwright MCP (no manual UI interaction)
- Generates 30-60 second videos in MP4 format
- Session persistence allows batch generation without re-login
```

**Student Task**: Fill in the template with their own project's patterns. What questions guided their decisions in Lesson 5? What principles did they discover through iteration?

**Section 3: Skill 2—`upload-youtube`**

**Structure** (similar to above):
```markdown
# SKILL: upload-youtube

## Persona
Content Publisher—someone uploading videos to YouTube for marketing, education, or product promotion.

## Questions
1. What's a clear, descriptive title for your video (45 chars max)?
2. What's the video description? Include key benefits and CTAs.
3. What tags/categories describe the video content (comma-separated)?
4. Should the video be public, unlisted, or private?
5. When should the video publish (immediately or scheduled)?

## Principles
- **SEO Optimization**: Title and description should include searchable keywords (what would viewers search for?).
- **Thumbnail Selection**: YouTube auto-generates thumbnails; pick the one that best represents the video content.
- **Metadata Clarity**: Description should answer "What is this video?" in the first 2 sentences.
- **Call-to-Action**: Every video should direct viewers to a next step (website, email signup, feature access).

## When to Use This Skill
- Publishing marketing videos to YouTube
- Uploading tutorial/education videos at scale
- Automating video publishing in batch (multiple videos, different metadata)
- Publishing product demo videos with consistent metadata

## When NOT to Use This Skill
- Live streaming (different YouTube feature)
- Shorts creation (requires vertical aspect ratio, different workflow)
- Premium YouTube features (memberships, super chats)

## Implementation Notes
- Uses Playwright MCP to automate YouTube UI (browser automation)
- Session persistence for YouTube login (authenticate once, upload multiple)
- Generates SEO-optimized metadata from product/message input
- Returns published video URL
```

**Student Task**: Fill in template with their video project's metadata decisions.

**Section 4: Validation—Are These Real Skills?**
- Checklist:
  - ✓ Does Persona clearly define the user context?
  - ✓ Do Questions (2-4) guide decision-making?
  - ✓ Do Principles reflect patterns from Lesson 5?
  - ✓ Is "When to Use" section clear (reusable across projects)?
  - ✓ Are "When NOT to Use" boundaries realistic?
- If all pass → Skills are ready for reuse in Lesson 9

**Section 5: Why Skills > Code Libraries**
- Code libraries: "Here's syntax to upload videos"
- Skills: "Here's how to think about what makes a video uploadable; questions guide your decisions; principles help you reason"
- Skills activate reasoning in new contexts
- Example: "I'm uploading a tutorial video (different context than product demo). The `upload-youtube` skill still applies because it guides decision-making about metadata, timing, visibility—the mechanics change, but the reasoning framework doesn't"

#### Intelligence Accumulation
- Skills are the **reusable intelligence** extracted from the video generation project
- In Lesson 9, these skills will be applied to the YouTube upload project (new context)
- Demonstrating skill reuse proves the value of SDD-RI accumulation

#### Three Roles Framework (Invisible)
- Student role: "I designed these skills based on my project"
- Skills role: "My skills guide decisions (metadata, SEO, upload strategy)"
- AI role (coming in Lesson 9): "I'll use this spec + your skills to execute"

#### Prerequisites
- Lesson 6 (patterns identified and documented)

#### Try With AI
**Activity: Skill Design & Validation**
1. Create two SKILL.md files (use SKILL.md template from `.claude/skills/skill-creator/`):
   - `skills/generate-video/SKILL.md` — Persona, Questions, Principles for video generation
   - `skills/upload-youtube/SKILL.md` — Persona, Questions, Principles for YouTube upload
2. For each skill, fill in sections based on patterns from Lesson 5-6
3. Ask Claude: "I've created these two skills for video generation and YouTube upload. Are the Persona/Questions/Principles clear and reusable across different projects? Where could I improve?"
4. Listen: AI identifies vague sections or opportunities to generalize
5. Refine: Update skills based on feedback to ensure they're project-agnostic
- Observe: How does applying P+Q+P framework help you think differently about your workflow?
- Reflect: How will these skills help in Lesson 9 when you tackle YouTube upload?

---

### LESSON 8: Write YouTube Upload Specification (Layer 4: Spec-Driven)

**Phase**: Mastery
**Layer**: 4 (Spec-Driven Integration)
**CEFR Level**: B1 (Intermediate)
**Bloom's Level**: Apply
**Estimated Time**: 60 minutes

#### Learning Objectives
- Write a new specification (YouTube upload) from scratch using Spec-Kit Plus template
- Apply learnings from Lesson 2 (spec writing) and Lessons 6-7 (skill reuse thinking)
- Demonstrate that clear specification enables AI to orchestrate pre-built skills
- Recognize: "Spec-first in Layer 4 works because I have skills + experience from Layers 1-3"

#### New Concepts (Count: 4)
1. **Spec-Driven in Layer 4** — New projects start with specification (not code)
2. **Specification as Orchestration** — Spec tells AI which skills to compose
3. **Reuse Through Specification** — Skills handle domain logic; spec orchestrates them
4. **Iteration on Specifications** — Refining spec is faster than refining code

#### Cognitive Load Validation
- 4 concepts ≤ B1 limit (10) ✓ Compliant
- Students apply familiar template in new context
- Minimal scaffolding (guided by precedent from Lesson 2)

#### Content Structure

**Section 1: Shifting to Spec-First (Layer 4 Mindset)**
- Lessons 1-5: Spec → Plan → Tasks → Implementation (learning the workflow)
- Lesson 8-9: Spec → Compose Skills → Implementation (reusing workflow)
- Key insight: "In Layer 4, I write spec first, AI uses my skills to execute. No planning phase (I already have a plan: use my skills)."

**Section 2: YouTube Upload Specification Template**

**Intent: What are we uploading and why?**
- Which video are you uploading? (the one from Lesson 5)
- Why? (marketing goal, audience reach, business metric)
- What's success? (view count, engagement, traffic to product page)
- Example:
  ```
  We are uploading the product demo video generated in Lesson 5 to YouTube.
  Intent: Drive traffic to product signup page through viral marketing.
  Success: Video reaches 500+ views in first week; drives 50+ signups.
  ```

**Constraints: What are the boundaries?**
- Use YouTube browser automation (not API)
- Use my `upload-youtube` skill for metadata decisions
- Must work on Windows, macOS, Linux
- Publish within 24 hours
- Example:
  ```
  - Automate upload via Playwright MCP (no manual UI interaction)
  - Reuse upload-youtube skill for metadata consistency
  - Cross-platform compatibility required
  - Video must remain playable for 6+ months
  ```

**Success Evals: How do we measure success?**
- Video uploaded successfully (observable: video appears in YouTube channel)
- Metadata correct (title, description, tags visible in YouTube UI)
- Video plays without errors (observable: play video, no 404/failed errors)
- URL sharable and accessible (observable: share URL works in incognito browser)
- Example:
  ```
  - Video appears in "My Videos" within 30 minutes of upload
  - Title, description, tags match spec exactly
  - Video plays when clicked (not processing, not error)
  - YouTube watch URL is accessible from incognito browser
  ```

**Non-Goals: What are we NOT doing?**
- NOT creating custom thumbnail (use YouTube's auto-generated)
- NOT setting up super chats, memberships, or channel monetization
- NOT optimizing for YouTube Shorts (vertical video format)
- Example:
  ```
  - NOT creating a custom thumbnail (YouTube's auto-generated is acceptable)
  - NOT setting up advanced monetization features
  - NOT creating a Shorts version (standard horizontal video only)
  ```

**Section 3: Comparing Specs—Lesson 2 vs. Lesson 8**
- Lesson 2 (Specification): Very detailed (1-2 pages); explains video generation intent and constraints
- Lesson 8 (Specification): Shorter (0.5-1 page); focuses on orchestration intent
- Why shorter? "I'm not explaining how to generate—I'm explaining what to upload and how to measure success. The implementation details are in my skills."

**Section 4: Specification Validation**
- Checklist (reuse from Lesson 2):
  - ✓ Does intent answer: What? Why? Who? (who cares about the upload, not the initial project)
  - ✓ Are constraints realistic and boundary-setting?
  - ✓ Are evals measurable and observable?
  - ✓ Does "non-goals" prevent scope creep?
- New check:
  - ✓ Does spec rely on my skills correctly? (Is implementation clear enough that AI can use skills?)

#### Intelligence Accumulation
- Specification is now **orchestration blueprint** (compared to Lesson 2's detailed intent)
- Skills from Lesson 7 are the "executable intelligence" that this spec directs
- Demonstrates the SDD-RI value prop: "Specifications guide reusable intelligence"

#### Three Roles Framework (Invisible)
- Student role: "I write spec based on what I learned in Lessons 1-5"
- Skills role: "My skills guide decisions (metadata, SEO, upload strategy)"
- AI role (coming in Lesson 9): "I'll use this spec + your skills to execute"

#### Prerequisites
- Lessons 6-7 (skills created with P+Q+P framework)

#### Try With AI
**Activity: Specification Refinement for YouTube Upload**
1. Write your YouTube upload specification using Spec-Kit Plus template (0.5-1 page)
2. Ask Claude: "I'm uploading my product demo video to YouTube. Here's my specification: [paste spec]. Should I add anything? Are the success evals measurable? Can you identify any vagueness?"
3. Listen: AI identifies clarity improvements
4. Refine: Update spec based on feedback
5. Validate: Does it pass the spec checklist from Lesson 2?
- Observe: How much shorter is this spec compared to Lesson 2? Why?
- Reflect: Why can I write a shorter spec here? (Because skills + experience reduce need for detail)

---

### LESSON 9: YouTube Upload Capstone (Layer 4: Spec-Driven Integration)

**Phase**: Mastery
**Layer**: 4 (Spec-Driven Integration)
**CEFR Level**: B1 (Intermediate)
**Bloom's Level**: Create + Evaluate
**Estimated Time**: 2-3 hours (video upload execution + validation + reflection)

#### Learning Objectives
- Execute `/sp.implement` using YouTube upload specification and composed skills
- Verify uploaded video meets success evals from specification
- Demonstrate skill composition: "My skills + specification = working implementation"
- Reflect on SDD-RI value: "Accumulating reusable intelligence (skills) accelerated this second project"
- Articulate the relationship between specification clarity and execution speed
- Complete full SDD-RI cycle: Spec (L1) → Plan (L2) → Tasks (L2) → Implementation (L2) → Skills (L3) → Spec (L4) → Composition (L4) → Validation (L4)

#### New Concepts (Count: 5)
1. **Skill Composition** — Using pre-built skills in new context
2. **Spec-Driven Orchestration** — Specification as coordination layer
3. **Validation Against Evals** — Confirming implementation matches spec
4. **SDD-RI Loop** — Spec → Skills → Spec → Composition (recursive accumulation)
5. **Transfer Learning** — Recognizing how Lessons 1-5 patterns apply to new context

#### Cognitive Load Validation
- 5 concepts ≤ B1 limit (10) ✓ Compliant
- Students apply learned patterns to new context
- Minimal scaffolding (mostly execution with validation checkpoints)

#### Content Structure

**Section 1: Composing Your Skills in the Implementation**
- You have:
  - Specification (Lesson 8): YouTube upload intent, constraints, evals
  - Skills (Lesson 7): `generate-video`, `upload-youtube` (P+Q+P frameworks)
  - Experience (Lessons 1-5): Patterns that made video generation work
- Now: Run `/sp.implement` using specification + reference your skills

**Section 2: Running `/sp.implement` for YouTube Upload**
- Command: `/sp.implement` with your YouTube spec.md
- AI uses spec to generate code that orchestrates your skills
- Expected output: Python scripts (youtube_upload.py, metadata_generator.py, etc.)
- Key difference from Lesson 5: Code references your `upload-youtube` skill rather than implementing upload from scratch

**Section 3: Executing the Upload**
- Setup: Ensure Playwright session is ready (reuse session from Lesson 5 if possible)
- Execute: `python youtube_upload.py --video_file ~/videos/demo_product_v1.mp4 --metadata_file ~/metadata.yaml`
- What happens:
  - Playwright opens YouTube
  - Logs in (using persisted session)
  - Fills metadata (title, description, tags) using `upload-youtube` skill decisions
  - Uploads video file
  - Captures YouTube URL for verification
- Duration: 5-10 minutes (depends on upload speed and YouTube processing)

**Section 4: Validation Against Success Evals (Specification-Driven Acceptance)**
- Spec defined success evals in Lesson 8
- Now verify each eval:
  - ✓ **Eval 1**: "Video appears in 'My Videos' within 30 minutes of upload"
    - Check: Log into YouTube → My Channel → Videos → Is video there?
  - ✓ **Eval 2**: "Title, description, tags match spec exactly"
    - Check: Click video → Read metadata → Compare to spec
  - ✓ **Eval 3**: "Video plays without errors"
    - Check: Click play → Watch 10 seconds → No 404 or error messages
  - ✓ **Eval 4**: "YouTube URL is accessible from incognito browser"
    - Check: Copy URL → Open incognito window → Paste URL → Video loads
- All evals pass → Specification successfully executed
- Evals fail → Refine (similar to Lesson 5 iteration loop)

**Section 5: Reflecting on Skill Reuse & Acceleration**
- **First project (Lessons 1-5)**: Spec → Plan → Tasks → Implementation (lots of decisions, trial-and-error)
- **Second project (Lesson 8-9)**: Spec → Use skills → Implement (fewer decisions; skills guide thinking)
- Compare:
  - Lesson 5: "How should I structure the Gemini prompt? What retry logic?"
  - Lesson 9: "The `upload-youtube` skill tells me to optimize for SEO. I apply that principle."
  - Observation: Second project moved faster because patterns were codified in skills
- This is the **SDD-RI value proposition**: Accumulating reusable intelligence (skills) accelerates future work

**Section 6: Demonstrating Understanding—Final Reflection**
- Respond to:
  1. "What's the difference between Lesson 5 (video generation) and Lesson 9 (YouTube upload) in terms of planning effort?"
  2. "How did your skills from Lesson 7 guide your decisions in Lesson 9?"
  3. "If you had to do a third project (e.g., TikTok upload), what patterns would you extract as new skills?"
  4. "Why is SDD-RI (specification + reusable intelligence) better than plain SDD (specification + code)?"
- Student explains: Specifications capture intent; skills capture reasoning patterns. Together, they compound.

**Section 7: Capstone Evidence**
- By end of Lesson 9, students have:
  - ✓ Video file (from Lesson 5)
  - ✓ YouTube URL (from Lesson 9)
  - ✓ Two reusable skills (`generate-video`, `upload-youtube`)
  - ✓ Specifications for both projects
  - ✓ Understanding of SDD-RI workflow (Spec → Plan → Tasks → Implementation → Skills → Spec → Composition)
- This is a **portfolio piece** demonstrating SDD-RI mastery

#### Intelligence Accumulation
- Skills from Lesson 7 are reused in Lesson 9 (proven value)
- Capstone demonstrates that SDD-RI accumulation compounds (second project faster than first)
- These skills become part of student's **reusable toolkit** for Part 5 and beyond

#### Three Roles Framework (Invisible)
- All three roles are present in the orchestration:
  - **AI as Teacher** (Lesson 3): Taught planning approach
  - **AI as Student** (Lesson 4): Learned task structure from feedback
  - **AI as Co-Worker** (Lesson 5): Iterated toward working video
  - **Skills as Teacher** (Lesson 7): Skills guide decision-making
  - **Student as Co-Designer** (Lesson 8-9): Writes spec, uses skills, validates

#### Prerequisites
- Lessons 1-8 (complete SDD-RI + L1-L3 progression)

#### Try With AI
**Activity: Full SDD-RI Capstone Cycle**
1. Run `/sp.implement` using your YouTube upload spec (from Lesson 8)
2. Execute YouTube upload:
   - Set up Playwright environment
   - Run: `python youtube_upload.py --video_file [path] --metadata_file [path]`
3. Validate all success evals:
   - [ ] Video appears in My Videos
   - [ ] Metadata (title, description, tags) correct
   - [ ] Video plays without errors
   - [ ] URL accessible from incognito
4. Reflection:
   - Ask Claude: "I just completed my second SDD-RI project (YouTube upload). Compared to the first project (video generation), what accelerated my workflow? How did my skills help?"
   - Listen: AI reflects on your reuse and acceleration
5. Document:
   - Write 1-2 paragraph reflection: "How did accumulating reusable intelligence (skills) make this project faster than the first? What would I extract as skills for a third project?"
- Observe: Speed difference between first project (Lesson 5) and second (Lesson 9)
- Reflect: This is the core of SDD-RI—specifications guide reusable intelligence, which compounds value across projects

---

## Cross-Cutting Pedagogical Elements

### Cognitive Load Distribution

| Lesson | Layer | CEFR | Concepts | Scaffolding | Assessment |
|--------|-------|------|----------|------------|------------|
| 1 | L1 | A2 | 5 | Heavy | Reflection |
| 2 | L1 | A2 | 6 | Heavy | Spec checklist |
| 3 | L2 | A2-B1 | 6 | Moderate | Plan evaluation |
| 4 | L2 | B1 | 5 | Moderate | Task validation |
| 5 | L2 | B1 | 6 | Minimal | Video validation |
| 6 | L3 | B1 | 5 | Minimal | Pattern analysis |
| 7 | L3 | B1 | 6 | Minimal | Skill documentation |
| 8 | L4 | B1 | 4 | Minimal | Spec validation |
| 9 | L4 | B1 | 5 | Minimal | Capstone reflection |

### Modality Variation from Chapter 13

**Chapter 13 Modality**: Problem-discovery through "vague code" failure analysis
- Students analyzed faulty code
- Questions drove exploration: "Why does this code fail? What's the pattern?"
- Conceptual foundation

**Chapter 14 Modality**: Project-based hands-on execution with pattern recognition
- Students execute complete workflow (not analyze failure cases)
- Building artifacts: Spec → Video → Skills
- Practical accumulation of intelligence
- **Variation achieved**: Different cognitive approach (execution vs. analysis), different deliverables (code analysis vs. reusable skills), different progression (conceptual foundation vs. practical accumulation)

### Three Roles Framework Integration (Invisible in Student Language)

**Lesson 3**: AI as Teacher (suggesting planning approach students didn't consider)
**Lesson 4**: AI as Student (learning task structure from student feedback)
**Lesson 5**: AI as Co-Worker (iterative convergence toward working video)
**Lesson 7**: Skills as distilled "Teachers" (guiding decision-making in Lesson 9)
**Lesson 9**: Skills + Specification orchestrate (final synthesis)

**Student-Facing Language**:
- "AI suggested an approach; I evaluated if it fit my constraints"
- "I asked Claude to clarify the tasks; the clarification improved my understanding"
- "We iterated on the video prompt until quality met spec"
- "The `upload-youtube` skill guided my metadata decisions"
- (NO framework labels; just experience of collaboration)

### Anti-Convergence Verification

**Generic Pattern Avoided**:
- ❌ "All chapters have 9 lessons" → ✓ 9 lessons justified by 4-layer progression (2-3 per layer) + 8 core concepts
- ❌ "Lecture-style instruction" → ✓ Project-based execution with validation checkpoints
- ❌ "Topic-based organization" → ✓ Layer-based progression (L1 foundation → L2 collaboration → L3 design → L4 orchestration)
- ❌ "Passive AI tool presentation" → ✓ All three roles embedded (visible through action narratives, not labels)
- ❌ "Same modality as Chapter 13" → ✓ Varied from "problem-discovery analysis" to "project-based execution"

---

## Intelligence Accumulation Map

```
Lesson 1-2 (L1: Foundation)
├── Mental Model: Specification → Implementation flow
└── Artifact: spec.md (video generation intent)

Lesson 3-5 (L2: Collaboration)
├── Experience: Planning → Tasks → Iteration → Success
├── Artifact: plan.md, tasks.md, video file
└── Pattern Recognition Trigger: "Where did we iterate most?"

Lesson 6 (L3: Pattern Recognition)
├── Analysis: Identifying recurring workflows
├── Artifact: Pattern documentation
└── Skill Extraction Trigger: "What's reusable?"

Lesson 7 (L3: Skill Creation)
├── Design: P+Q+P framework application
├── Artifact: `generate-video` skill, `upload-youtube` skill
└── Reusability Proof: Skills guide Lesson 9

Lesson 8 (L4: Specification)
├── Application: New context spec writing
├── Artifact: YouTube upload spec.md
└── Orchestration Blueprint: "Which skills do I need?"

Lesson 9 (L4: Composition & Validation)
├── Execution: Skills compose to solve new problem
├── Artifact: YouTube URL + capstone reflection
└── Value Demonstration: "Second project was faster because of accumulated skills"
```

---

## Technical Implementation Notes

### Playwright MCP Usage Pattern

**Lesson 2**: Concepts only (no Playwright yet)
**Lesson 3**: Plan references Playwright approach (no code)
**Lesson 5**: Playwright automation executes video generation
- Session persistence: Login once, use session across multiple runs
- Selector abstraction: Isolate Gemini UI selectors in separate file (maintainability)
- Error handling: Retry logic for Gemini timeouts

**Lesson 9**: Playwright automation for YouTube upload
- Reuses session persistence pattern from Lesson 5
- References `upload-youtube` skill for metadata structure

### Gemini.google.com Integration

**Key Pattern**: Browser automation (not API calls)
- Advantages: Free tier, no authentication overhead, user-visible testing
- Challenges: UI selectors may change, processing time unpredictable
- Mitigation: Document selectors clearly, parameterize for flexibility

**Implementation Approach**:
1. Playwright navigates to Gemini.google.com
2. Fills prompt: "Generate a 45-second product demo video for [product]. Key message: [message]. Target audience: [audience]. Format: MP4, 1080p."
3. Waits for video generation
4. Downloads result locally
5. Validates file exists and plays

### YouTube Upload Automation

**Session Persistence** (from Lesson 5 pattern):
- First run: Manual YouTube login (browser opens)
- Subsequent runs: Playwright uses persisted session
- No credential sharing required

**Metadata Management**:
- `upload-youtube` skill defines questions: Title? Description? Tags? Visibility?
- Generated code fills YouTube form fields based on spec
- Error handling for missing or invalid metadata

**Validation**:
- Video appears in "My Videos" (observable)
- URL is sharable and accessible
- Metadata persists (no validation errors)

---

## Success Criteria Alignment

### FR Mapping

| Requirement | Lesson | Evidence |
|------------|--------|----------|
| FR-001: 7-9 lessons, pedagogical arc | All | 9 lessons: L1(2) → L2(3) → L3(2) → L4(2) |
| FR-004: SDD-RI workflow demo | L1-9 | `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` (L1-5), Skills (L6-7), Composition (L8-9) |
| FR-005: Write spec from scratch | L2, L8 | Lesson 2: video gen spec, Lesson 8: YouTube spec |
| FR-008-011: Skills creation | L6-7 | Two skills: `generate-video`, `upload-youtube` with P+Q+P |
| FR-016: Modality variation | All | Project-based execution (vs. Ch 13 problem-discovery) |
| FR-017: Three Roles invisible | L3-5, L7-9 | AI as Teacher/Student/Co-Worker; framework labels absent |
| FR-018: Cognitive load limits | All | Max 6 concepts/lesson ≤ A2-B1 limits |
| FR-019: "Try With AI" endings | All | Each lesson ends with activity, no "Summary" or "What's Next" |

### Success Criteria Mapping

| SC | Lesson(s) | Assessment |
|----|-----------|------------|
| SC-001-003: Workflow execution | L1-9 | All 4 Spec-Kit commands executed |
| SC-004: 90% video success | L5 | Video file generated locally |
| SC-005-006: Pattern ID + valid skills | L6-7 | Two skills with P+Q+P framework |
| SC-008: 80% capstone completion | L9 | YouTube URL + capstone reflection |

---

## Lesson Sequencing & Dependencies

```
Lesson 1 (Context)
  ↓
Lesson 2 (Spec Writing) ← Lesson 1 required
  ↓
Lesson 3 (Planning) ← Lesson 2 spec required
  ↓
Lesson 4 (Task Breakdown) ← Lesson 3 plan required
  ↓
Lesson 5 (Implementation) ← Lesson 4 tasks required
  ↓
Lesson 6 (Pattern Recognition) ← Lesson 5 execution required
  ↓
Lesson 7 (Skill Creation) ← Lesson 6 patterns required
  ↓
Lesson 8 (YouTube Spec) ← Lesson 7 skills required (contextual, not blocking)
  ↓
Lesson 9 (Capstone) ← Lesson 8 spec required
```

All lessons are **linear prerequisites** (each builds on previous). No branching or optional paths.

---

## Time Allocation

| Phase | Lessons | Total Hours | Rationale |
|-------|---------|------------|-----------|
| Foundation (L1) | 1-2 | 2.25 | Context + spec writing (manual effort) |
| Application (L2) | 3-5 | 4.5 | Planning + tasks + implementation (includes AI processing time) |
| Integration (L3) | 6-7 | 3 | Pattern recognition + skill design (student thinking, less execution) |
| Mastery (L4) | 8-9 | 2.5 | YouTube spec + capstone (composition + validation) |
| **Total** | 1-9 | **12.25 hours** | (Estimate: 6-8 hours of focused student time; 4-5 hours waiting for Gemini/YouTube processing) |

---

## Validation Checklist

**Pedagogical Alignment**:
- [x] 4-Layer progression (L1→L2→L3→L4) without skipping stages
- [x] Layer 1 establishes foundation without AI assistance
- [x] Layer 2 demonstrates three roles (invisible in student language)
- [x] Layer 3 creates reusable intelligence (P+Q+P framework)
- [x] Layer 4 orchestrates accumulated intelligence
- [x] Each lesson has clear learning objectives mapped to Bloom's

**Cognitive Load**:
- [x] No lesson exceeds A2-B1 limits (max 6 concepts)
- [x] Scaffolding decreases from Lesson 1 (heavy) → Lesson 9 (minimal)
- [x] Complexity tier validated per CEFR proficiency progression

**Content Quality**:
- [x] Each lesson ends with "Try With AI" activity (no "Summary" or "What's Next")
- [x] Three Roles framework integrated invisibly (no framework labels in student text)
- [x] Modality varies from Chapter 13 (execution vs. analysis)
- [x] All requirements (FR) addressed in lesson content
- [x] All success criteria (SC) have assessment points

**Technical Feasibility**:
- [x] Playwright MCP usage documented for each relevant lesson
- [x] Gemini.google.com integration explained (browser automation approach)
- [x] YouTube upload automation strategy defined (session persistence)
- [x] Session persistence addressed for cross-platform compatibility

**Intelligence Accumulation**:
- [x] Lessons 1-5 build foundational workflow execution
- [x] Lessons 6-7 extract patterns → create reusable skills
- [x] Lessons 8-9 reuse skills in new context → demonstrate value
- [x] Capstone includes reflection on acceleration (proving SDD-RI value)

---

## Next Steps (Content Implementation)

1. **Content-Implementer** creates lesson content (book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/)
   - Each lesson becomes standalone .md file with explanations, walkthroughs, Try With AI sections
   - Coordinate with pedagogical-designer for stage transitions

2. **Assessment-Architect** designs rubrics for each lesson
   - Formative assessments (spec checklist, plan evaluation, task validation)
   - Capstone assessment (YouTube upload + reflection)

3. **Validation-Auditor** verifies:
   - No meta-commentary exposing Three Roles framework (grep validation)
   - All lessons end with "Try With AI" (no "Summary" sections)
   - All success criteria met by lesson content

4. **Factual-Verifier** validates technical claims
   - Playwright MCP usage patterns
   - Gemini.google.com video generation (available feature)
   - YouTube upload automation feasibility

---

## Constitutional Compliance Summary

**Section IIa (4-Layer Progression)**: ✓ Full L1→L2→L3→L4 progression, 2-3 lessons per layer
**Section IIb (Progressive Complexity)**: ✓ A2-B1 tier, max 6 concepts/lesson, scaffolding decreases across arc
**Section IIc (Meta-Commentary Prohibition)**: ✓ Three Roles invisible; student experiences roles through narrative
**Section IIe (Minimal Content)**: ✓ Each lesson ends with "Try With AI"; no "Summary" or tangential content
**Section III (Intelligence Accumulation)**: ✓ Skills created in L3 reused in L4; capstone demonstrates acceleration
**Anti-Convergence (Principle 6)**: ✓ Modality varies from Ch 13 (problem-discovery → project-based execution)
**Principle 7 (Evals-First)**: ✓ Lesson structure maps to spec success criteria and evals

---

**Plan Status**: Ready for Lesson Content Implementation

**Constitutional Version**: 6.0.1 Compliance Verified
**Created**: 2025-11-25
**Feature Branch**: 036-chapter-14-sdd-ri-hands-on

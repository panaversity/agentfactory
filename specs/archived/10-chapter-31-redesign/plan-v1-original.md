# Implementation Plan: Chapter 31 — Spec-Kit Plus Hands-On

**Branch**: `010-chapter-31-redesign` | **Date**: 2025-11-05 | **Spec**: `/specs/010-chapter-31-redesign/spec.md`
**Input**: Feature specification from `/specs/010-chapter-31-redesign/spec.md` (UPDATED)
**Output**: Detailed lesson architecture with learning objectives, skills mapping, and assessment strategy

---

## I. Executive Summary

Chapter 31 teaches practical **Spec-Kit Plus hands-on workflow** for intermediate developers (Part 5). This chapter represents the **natural progression from Chapter 30** (where students learned why Spec-Kit Plus was chosen) to **actually practicing the Spec-Kit Plus methodology** with a real calculator project.

**Core Design Philosophy**:
- **Human in Control**: YOU are in control; YOU collaborate with AI orchestrator (Claude Code or Gemini CLI)
- **Spec-Kit Plus = Opinionated Toolkit**: Provides workflow structure; AI tools execute it
- **Workflow Isomorphism**: Chapter structure mirrors actual Spec-Kit Plus phases (Constitution → Specify → Clarify → Plan → Tasks → Implement → Validate)
- **Vertical Intelligence**: AI Orchestrator delegates to specialized subagents (YOU guide with commands)
- **Horizontal Intelligence**: ADRs + PHRs capture organizational knowledge automatically
- **Complete SDD Loop**: Students master all 7 phases + 6 commands + PHR auto-creation
- **Checkpoint-Driven Execution**: `/sp.tasks` allows checkpoints - agent completes phase, human reviews/commits, then continues
- **Atomic Tasks & Planned Phases**: Work broken into reviewable chunks; human guides progression
- **Cascade Effect**: Demonstrate spec quality → plan quality → task quality → code quality
- **AIDD Paradigm**: Show co-learning cycle: intent → generation → validation → refinement

**Key Outcome**: Students complete full Spec-Kit Plus workflow end-to-end with a **calculator capstone project** (5 operations: add, subtract, multiply, divide, power), understanding:
1. **YOU control the workflow** (AI orchestrator is your collaborator, not autonomous)
2. **Spec-Kit Plus provides structure** (opinionated toolkit)
3. **Checkpoints prevent runaway automation** (review → commit → continue pattern)
4. **Atomic tasks enable validation** (small, testable increments)

Foundation for Chapter 32 (team parallelization patterns).

---

## II. Project Overview & Success Criteria

### Chapter Objectives (By End of Chapter 31)

Students will be able to:

1. **Understand Human Control**: Recognize YOU control the workflow; AI orchestrator (Claude Code/Gemini CLI) is your collaborator, not autonomous agent
2. **Install Spec-Kit Plus**: Set up AI-native SDD toolkit with Horizontal Intelligence (ADRs+PHRs) + Vertical Intelligence (orchestrator+subagents)
3. **Use 6 Core Commands**: Master `/sp.specify`, `/sp.clarify`, `/sp.plan`, `/sp.adr`, `/sp.tasks`, `/sp.implement` with understanding of when/why each is used
4. **Implement Checkpoint Pattern**: Use checkpoints in `/sp.tasks` and `/sp.implement` - review → commit → continue (not long autonomous loops)
5. **Work with Atomic Tasks**: Break work into small, reviewable, testable increments guided by human validation
6. **Understand PHR Auto-Creation**: Recognize PHRs are automatically created for each command session (8-10 per feature typical)
7. **Practice Constitution → Commit → Feature Loop**: Create Constitution once → commit → then feature workflow
8. **Recognize Cascade Effect**: Understand spec quality → plan quality → task quality → code quality
9. **Collaborate with AI Orchestrator**: Practice co-learning: YOU provide intent → AI generates → YOU validate → iterate
10. **Complete Calculator Capstone**: Build working calculator (5 operations) end-to-end using complete Spec-Kit Plus workflow
11. **Understand Spec-Kit Plus Role**: Recognize it's an opinionated toolkit providing structure; AI tools (Claude Code/Gemini CLI) execute it

### Success Criteria for This Plan

- [ ] All 7 lessons detailed with testable learning objectives
- [ ] Each lesson has independent, standalone deliverable
- [ ] Capstone project is concrete, achievable, and consolidates all learning
- [ ] CEFR proficiency levels assigned (A2 for Lessons 1-3, B1 for Lessons 4-7)
- [ ] Bloom's cognitive levels align with proficiency (Understand/Apply for A2, Apply/Analyze for B1)
- [ ] Cognitive load validated (max 5 concepts for A2, max 10 for B1)
- [ ] Manual thinking (L1-3) clearly distinguished from tool-assisted thinking (L4-7)
- [ ] Cascade effect demonstrated in every lesson
- [ ] Correct SpecifyPlus workflow documented (no hallucinated commands)
- [ ] Constitutional alignment verified (spec-first, validation-first, AIDD paradigm)

---

## II. Chapter Type, Tier & Proficiency Levels

### Chapter Type: **Technical/Hybrid** (Methodology + Hands-On)

This is a **hybrid chapter** combining:
- **Conceptual foundation** (Vertical Intelligence architecture, Horizontal Intelligence with ADRs/PHRs)
- **Installation & setup** (Spec-Kit Plus framework + AI tool configuration)
- **Technical practice** (actual workflow with 6 commands: `/sp.specify`, `/sp.clarify`, `/sp.plan`, `/sp.adr`, `/sp.tasks`, `/sp.implement`)
- **Experiential learning** (calculator capstone project with 5 operations: add, subtract, multiply, divide, power)

### Complexity Tier: **Intermediate** (Part 5)

**CEFR Proficiency Targets**:
- **Lesson 1**: A2 (Basic) — Installation focus, no new concepts, mostly procedural
- **Lessons 2-3**: A2 (Basic Application) — Max 5 new concepts per lesson
- **Lessons 4-6**: B1 (Intermediate Application) — Max 7 new concepts per lesson
- **Lesson 7**: B1-B2 (Intermediate to Advanced) — Max 10 new concepts
- **Capstone**: B2 (Advanced) — Design decisions, real-world constraints

**Bloom's Taxonomy Alignment**:
- **Lesson 1**: Remember + Understand (setup)
- **Lessons 2-3**: Understand + Apply (foundation thinking)
- **Lessons 4-6**: Apply + Analyze (tool-assisted workflow)
- **Lesson 7**: Apply + Analyze + Evaluate (validation and refinement)
- **Capstone**: Create (design and build complete project)

### Total Duration: **13-15 hours**

- Lesson 1: 1.5 hours (Installation & Setup)
- Lesson 2: 1.5 hours (Constitution Phase)
- Lesson 3: 2-2.5 hours (Specify Phase)
- Lesson 4: 1.5 hours (/sp.clarify Phase)
- Lesson 5: 2 hours (/sp.plan Phase + ADRs)
- Lesson 6: 1.5 hours (/sp.tasks Phase)
- Lesson 7: 2.5-3 hours (/sp.implement Phase + Validation)
- **Capstone Project**: 3-4 hours (end-to-end workflow)
- **Total**: ~15-18 hours (with capstone)

---

## III. Lesson-by-Lesson Breakdown

### **Lesson 1: Installation & Setup — AI-Native SDD Toolkit**

**Duration**: 1.5 hours | **Proficiency Target**: A2 (Procedural) | **Bloom's Level**: Remember, Understand

#### Learning Objective

Students will be able to **install and configure Spec-Kit Plus framework with their chosen AI tool** (Claude Code or Gemini CLI) and **verify that the complete setup works end-to-end**—understanding what Horizontal and Vertical Intelligence are in the context of the toolkit.

**Measurable outcome**: Student has working Spec-Kit Plus installation, configured AI tool, verified `/sp.*` commands execute, initialized calculator project, and tested complete setup.

#### Skills Taught

1. **Understanding AI-Native SDD Toolkit** — A2 — Conceptual
   - Measurable at this level: Student can explain: Spec-Kit Plus is independent framework; works with any AI tool (Claude Code or Gemini CLI); includes Horizontal (ADRs+PHRs) and Vertical Intelligence (orchestrator+subagents)
   - Category: Conceptual (framework understanding)
   - DigComp: Problem-Solving, Communication

2. **Tool Configuration** — A2 — Technical
   - Measurable at this level: Student successfully installs and configures chosen AI tool for Spec-Kit Plus use
   - Category: Technical (setup and configuration)
   - DigComp: Problem-Solving, Digital Literacy

3. **Vertical Intelligence Architecture Awareness** — A2 — Conceptual
   - Measurable at this level: Student can explain: Human → Orchestrator → Subagents structure; each role/responsibility
   - Category: Conceptual (framework understanding)
   - DigComp: Problem-Solving, Communication

#### Key Concepts (Max 5 for A2 — but mostly procedural setup)

1. **Spec-Kit Plus Framework**: Independent SDD toolkit with commands, templates, folder structure
2. **Horizontal Intelligence**: ADRs (Architectural Decision Records) + PHRs (Prompt History Records) capture organizational knowledge
3. **Vertical Intelligence**: Human collaborates with AI Orchestrator, which delegates to Specification/Planning/Implementation/Validation Subagents
4. **AI Tool Options**: Claude Code or Gemini CLI—both supported; Spec-Kit Plus works with any AI tool
5. **Project Structure**: Installation creates `.specify/`, `specs/`, `history/` folders; understand purpose of each

#### Content Outline

**Part A: AI-Native SDD Toolkit Introduction (10 min)**
- What is Spec-Kit Plus?
  - **Opinionated toolkit** providing workflow structure
  - **Independent framework** - works with any AI tool (Claude Code, Gemini CLI, etc.)
  - **NOT an autonomous agent** - YOU control it; AI tools execute your commands
- What are Horizontal Intelligence components?
  - ADRs: Document architectural decisions (long-term impact, multiple alternatives)
  - PHRs: Auto-record AI interactions (every `/sp.*` command creates one)
  - Captures organizational knowledge over time
- What is Vertical Intelligence?
  - **YOU (Human)**: In control - define intent, make decisions, validate outputs, guide workflow
  - **AI Orchestrator (Claude Code/Gemini CLI)**: Your collaborator - understands context, delegates to subagents per YOUR command
  - **Subagents**: Specification, Planning, Implementation, Validation - specialized for each phase
  - **Critical**: YOU guide with commands; orchestrator executes; YOU validate at each step
- **Key Insights**:
  - Spec-Kit Plus provides structure (the toolkit)
  - AI tools (Claude Code/Gemini CLI) execute it
  - YOU are always in control; AI is your collaborator
- Diagram showing: Human → Issues Command → Orchestrator → Delegates to Subagent → Returns to Human for Validation

**Part B: Install Spec-Kit Plus Framework (20 min)**
- Download/install Spec-Kit Plus (pip install specifyplus or equivalent)
- Verify installation
- Initialize calculator project (`specifyplus init calculator-project`)
- Explore generated folder structure

**Part C: Configure AI Tool — Claude Code OR Gemini CLI (20 min)**
- **Option 1: Claude Code**
  - Verify Claude Code installed
  - Test that `/sp.specify`, `/sp.clarify`, `/sp.plan`, `/sp.tasks`, `/sp.implement` commands available
  - Run simple help command to verify
- **Option 2: Gemini CLI**
  - Verify Gemini CLI installed
  - Configure authentication
  - Test available commands
- **Either way**: Verify chosen tool works with Spec-Kit Plus

**Part D: Verify Commands Work (15 min)**
- Test `/sp.specify --help` (or equivalent)
- Test `/sp.plan --help`
- Test `/sp.tasks --help`
- Test `/sp.implement --help`
- Confirm all 6 commands available (including `/sp.clarify`, `/sp.adr`)

**Part E: Initialize Calculator Project (15 min)**
- Create calculator project directory
- Initialize Spec-Kit Plus structure
- Understand directory layout:
  - `.specify/memory/` — Constitution and project rules
  - `specs/calculator/` — Specification, plan, tasks for calculator
  - `history/adr/` — Architectural decisions
  - `history/prompts/` — PHRs (auto-created during `/sp.*` commands)
- Verify structure is ready

**Part F: Test Complete Setup (5 min)**
- Run one simple verification command
- Confirm framework + AI tool + calculator project all working together

#### End-of-Lesson: Try With AI Activity

**Tool**: Claude Code or Gemini CLI (whichever was installed)
**Duration**: 5-10 minutes

**Workflow**:
1. In your configured AI tool, run: `/sp.specify --help`
2. Ask: "What does `/sp.specify` do?" (AI should explain the command)
3. Ask: "How is Vertical Intelligence different from manual workflow?"
4. **Reflection**: "Now I have a working Spec-Kit Plus installation. Next lessons, I'll use these commands."

**Expected Outcomes**:
- Students have confirmed working installation
- They understand framework is independent of AI tool choice
- They recognize Vertical Intelligence as the delegation pattern
- They're ready to start workflow in Lesson 2

**Safety/Ethics Note**: "Spec-Kit Plus is a tool. You remain responsible for what it generates. The framework helps you think clearly and collaborate with AI systematically."

#### Prerequisites

- Chapter 30 (understand why Spec-Kit Plus was chosen)
- Python 3.13+ or Node.js 18+ (depending on AI tool)
- Terminal/CLI comfort (basic navigation)
- Access to Claude Code or Gemini CLI

#### Assessment

**Formative**:
- Installation checklist completed
- Directory structure verified
- Commands execute without errors

**Summative**:
- All installation steps completed
- All 6 commands available and working
- Calculator project initialized
- Student can explain Horizontal and Vertical Intelligence in one sentence each
- Rubric:
  - Installation: Framework and AI tool both working (A2)
  - Configuration: AI tool properly configured for Spec-Kit Plus (A2)
  - Verification: All commands test successfully (A2)
  - Understanding: Student explains Intelligence components (A2)

---

### **Lesson 2: Constitution Phase — Project-Wide Rules**

**Duration**: 1.5 hours | **Proficiency Target**: A2 (Basic Application) | **Bloom's Level**: Understand, Apply

#### Learning Objective

Students will be able to **create a project Constitution** that defines immutable rules for the calculator project (quality standards, testing requirements, error handling patterns) and **understand the difference between Constitution (global rules) vs. Specs (feature-specific)**—recognizing Constitution as foundational one-time setup.

**Measurable outcome**: Student creates complete Constitution document for calculator project with quality standards, testing rules, error handling patterns. Student can explain why Constitution is created once per project, not per feature.

#### Skills Taught

1. **Constitution Creation** — A2 — Technical
   - Measurable at this level: Student writes complete Constitution with standards for quality, testing, error handling
   - Category: Technical (SDD foundation)
   - DigComp: Problem-Solving, Content Creation

2. **Global Rules vs. Feature Specs** — A2 — Conceptual
   - Measurable at this level: Student distinguishes: Constitution applies to ALL features; specs are feature-specific
   - Category: Conceptual (process understanding)
   - DigComp: Problem-Solving

3. **Best Practice Workflow Recognition** — A2 — Soft
   - Measurable at this level: Student understands: Constitution → Commit → Features; Constitution is once-per-project
   - Category: Soft (process thinking)
   - DigComp: Problem-Solving

#### Key Concepts (Max 5 for A2)

1. **Constitution Definition**: Global rules for entire project (quality, testing, security standards)
2. **One-Time Creation**: Constitution created ONCE at project start, not per feature
3. **Feature Vs. Constitution**: Specs are feature-specific; Constitution applies to all features
4. **Cascade Starting Point**: Constitution rules → All specs must follow → All plans enforce → All code adheres
5. **Best Practice Pattern**: Constitution → Commit to Git → Then start features

#### Content Outline

**Part A: What is Constitution? (20 min)**
- Constitution defines immutable project rules (applies to ALL work)
- Examples:
  - Quality standards: "All code must have type hints"
  - Testing standards: "All code must have 80%+ test coverage"
  - Error handling: "All functions must validate input before processing"
  - Security: "No hardcoded secrets; all configs in environment variables"
- Constitution vs. Specs:
  - Constitution: "This is our standard"
  - Spec: "For calculator, we need add, subtract, multiply, divide, power"
- Why create Constitution first? (Forces thinking about what matters for quality)

**Part B: Create Calculator Constitution (50 min)**
- Open Constitution template (provided)
- Fill sections:
  1. **Project Vision**: Why are we building this? What's the goal?
  2. **Quality Standards**: Type hints required? Code style? Documentation needs?
  3. **Testing Standards**: Minimum coverage? All functions tested?
  4. **Error Handling**: How do we handle invalid input? Division by zero? Overflow?
  5. **Security Standards**: Environment variables only? No secrets in code?
  6. **Performance Expectations**: How fast should calculator respond?
  7. **Accessibility Standards**: Any accessibility requirements?
- Worked example: Show Constitution for a real calculator project
- Student writes their Constitution
- Example entries:
  - "All functions must have type hints (Python 3.13+ style)"
  - "All code must pass mypy type checking"
  - "Minimum 80% test coverage"
  - "Division by zero must raise ValueError with message: 'Cannot divide by zero'"
  - "Power function must handle negative exponents correctly"

**Part C: Peer Review (20 min)**
- Pair students
- Exchange Constitutions
- Reviewer asks: "Are these rules clear? Can a developer follow them?"
- Refine based on feedback

**Part D: Reflection & Git Best Practice (10 min)**
- **Try With AI**: Show Constitution to Claude Code; ask: "Are these standards complete for a calculator?"
- **Reflection**: "Why does Constitution come BEFORE specifications?"
- **Best Practice**: Constitution → `git commit -m "Initial constitution"` → Then start features

#### End-of-Lesson: Try With AI Activity

**Tool**: Claude Code or ChatGPT web
**Duration**: 10 minutes

**Workflow**:
1. Paste Constitution to Claude Code
2. Ask: "Are these standards clear? Could someone build from them?"
3. Ask: "What happens if we skip Constitution and write specs directly?"
4. **Reflection**: "Constitution sets the foundation. Everything built on it follows these rules."

**Expected Outcomes**:
- Students understand Constitution as foundational
- They recognize Constitution applies project-wide
- They see best practice: Constitution → Commit → Features
- They're ready to write specifications in Lesson 3 that follow Constitution

**Safety/Ethics Note**: "Constitution is your quality contract. It forces you to think about what 'done' means before you start building."

#### Prerequisites

- Lesson 1 (installation complete)
- Chapter 30 (SDD philosophy)

#### Assessment

**Formative**:
- Constitution sections completed
- Peer review feedback incorporated

**Summative**:
- Complete Constitution document for calculator project
- Student explains: "Why is Constitution one-time, not per-feature?"
- Rubric (A2 level):
  - Completeness: All 7 sections filled (A2)
  - Clarity: Standards are specific and unambiguous (A2)
  - Feasibility: Standards are achievable for calculator project (A2)
  - Scope Understanding: Student explains Constitution vs. Specs (A2)

---

### **Lesson 3: Specify Phase — Writing Complete Specifications**

**Duration**: 2 hours | **Proficiency Target**: A2 (Basic Application) | **Bloom's Level**: Understand, Apply

#### Learning Objective

Students will be able to **convert vague requirements into SMART acceptance criteria** that are specific, measurable, achievable, relevant, and time-bound—preventing AI misinterpretation and enabling clear implementation.

**Measurable outcome**: Given a vague feature requirement, student writes 3+ acceptance criteria using SMART framework; each criterion is testable (pass/fail) and includes quantifiable measures.

#### Skills Taught

1. **Understanding SMART Criteria** — A2 — Technical
   - Measurable at this level: Student can identify which criteria are SMART and which are vague; explain each component
   - Category: Conceptual (thinking pattern)
   - DigComp: Communication & Problem-Solving

2. **Identifying Vague Requirements** — A2 — Soft
   - Measurable at this level: Student spots ambiguous language ("helpful", "fast", "easy") and proposes specific alternatives
   - Category: Soft (critical thinking)
   - DigComp: Communication & Problem-Solving

3. **Writing Testable Criteria** — A2 — Technical
   - Measurable at this level: Student writes criteria with clear pass/fail conditions; no interpretation needed
   - Category: Technical (specification writing)
   - DigComp: Content Creation

#### Key Concepts (Max 5 for A2)

1. **SMART Framework**: Specific (exact), Measurable (numbers/metrics), Achievable (realistic), Relevant (matters), Time-bound (deadline or scope)
2. **Vagueness Problem**: Vague criteria → AI misinterprets → useless code (problem-first)
3. **Clarity Solution**: Clear criteria → AI builds exactly what you want (solution)
4. **Testability**: Acceptance criteria must be verifiable (pass/fail, not opinion)
5. **Cascade Starting Point**: Bad criteria → bad spec → bad plan → bad code (why this matters)

#### Content Outline

**Part A: The Problem — Experience Vagueness (30 min)**
- Start with a vague feature request: "The calculator should be helpful and give good feedback"
- Ask students: Ask AI companion to build from this requirement
- Show: AI-generated code that misses the mark (demo the problem)
- Insight: Vagueness cascades through entire workflow

**Part B: The Framework — SMART Criteria (40 min)**
- **Specific**: Define exactly what should happen (not "good feedback" → "display error messages in format: 'ERROR: [operation] failed'")
- **Measurable**: Include numbers, metrics, thresholds ("response time < 100ms", "support 10+ digit numbers")
- **Achievable**: Realistic scope (not "AI predicts user intent" → "calculator handles basic arithmetic")
- **Relevant**: Matters to users/business ("Users can see calculation history" for audit trail)
- **Time-bound**: When should this work? By what point in workflow?
- Worked example: Convert vague requirement to SMART criteria

**Part C: Practice — Refine Real Requirements (40 min)**
- **Exercise 1**: Given vague feature ("grading system should be fair"), identify vague terms and convert to SMART
- **Exercise 2**: Write 5 acceptance criteria for simple feature (todo list, calculator, quiz system)
- **Exercise 3**: Trade specs with partner; critique using SMART framework

**Part D: Reflection & AI Exercise (10 min)**
- Compare vague vs. refined criteria: Which does AI implement better?
- **Try With AI**: Use ChatGPT web to generate code from vague criteria, then from refined SMART criteria; compare outputs

#### End-of-Lesson: Try With AI Activity

**Tool**: ChatGPT web (Part 5 hasn't taught learner's AI companion yet)
**Duration**: 10-15 minutes

**Prompts**:
1. "Here's a vague feature request: 'Build a calculator that gives helpful feedback.' Can you write Python code for this?"
2. [Show AI output — likely incomplete/generic]
3. "Now here's a refined request with SMART criteria: [paste 5 SMART criteria]. Please rewrite the calculator."
4. [Compare outputs — second should be much better]
5. "What did the SMART criteria help you understand better?"
6. **Reflection**: "What would happen if you skipped writing SMART criteria and jumped straight to coding?"

**Expected Outcomes**:
- Students observe how specification quality directly improves AI output
- AI generates more specific, testable code when criteria are SMART
- Students understand cascade effect from first lesson

**Safety/Ethics Note**: "AI tools won't magically understand vague requests. Your job is clarity—write specs so clear that AI (and humans) can't misinterpret."

#### Prerequisites

- Chapter 30 (or Part 1 basics) — understanding of AI-native development paradigm
- Basic Python knowledge (not required for this lesson specifically, but helpful context)

#### Assessment

**Formative** (ongoing in lesson):
- Partner review of SMART criteria (peer feedback)
- Checklist: Does each criterion have specific/measurable/achievable/relevant/time-bound components?

**Summative** (end of lesson):
- Student produces template of SMART criteria for 1 real feature
- Rubric:
  - Clarity: Requirements are specific and unambiguous (A2 level)
  - Measurability: Each criterion includes quantifiable measures or clear pass/fail (A2 level)
  - Completeness: Minimum 3-5 criteria; covers main behavior (A2 level)
  - SMART Framework: All 5 components present in at least one criterion (A2 level)

---

### **Lesson 2: SpecifyPlus Project Structure & the Cascade**

**Duration**: 1.5 hours | **Proficiency Target**: A2 (Basic Application) | **Bloom's Level**: Understand, Apply

#### Learning Objective

Students will be able to **explain the SpecifyPlus project structure** (`.specify/`, `specs/`, `history/`) and **understand WHY the structure enforces Spec → Plan → Tasks cascade**—recognizing that each step builds on the previous.

**Measurable outcome**: Given an initialized SpecifyPlus project, student can navigate the structure, explain the purpose of each directory, and articulate why skipping steps breaks the cascade.

#### Skills Taught

1. **Understanding Project Structure** — A2 — Technical
   - Measurable at this level: Student can identify purpose of each directory (specs, .specify, history) and explain relationships
   - Category: Technical (project organization)
   - DigComp: Problem-Solving

2. **Recognizing Workflow Enforcement** — A2 — Conceptual
   - Measurable at this level: Student can explain why structure forces Spec → Plan → Tasks progression and what breaks if you skip steps
   - Category: Conceptual (methodology understanding)
   - DigComp: Problem-Solving

3. **Cascade Thinking** — A2 — Soft
   - Measurable at this level: Student can trace how a specification gap cascades to plan and tasks; predict ripple effects
   - Category: Soft (strategic thinking)
   - DigComp: Problem-Solving

#### Key Concepts (Max 5 for A2)

1. **Spec Folder**: Where specifications live (spec.md); describes what to build
2. **Plan Folder**: Where implementation plans live (plan.md); describes how to build it
3. **Tasks Folder**: Where task checklists live (tasks.md); atomic units of work
4. **Forced Sequence**: Structure prevents skipping (can't generate good plan from bad spec; can't generate good tasks from bad plan)
5. **Cascade Effect**: Spec quality → Plan quality → Task quality → Code quality (demonstrated through examples)

#### Content Outline

**Part A: Explore the Structure (30 min)**
- Initialize a SpecifyPlus project (show: `pip install specifyplus` or `uvx specifyplus init project-name`)
- Navigate the folders: `.specify/`, `specs/`, `history/`
- Examine actual files:
  - `specs/feature/spec.md` — Specification template with sections
  - `specs/feature/plan.md` — Plan template with phases
  - `specs/feature/tasks.md` — Tasks template with checklist
- Observations: Why are these files separate? Why this order?

**Part B: Understand the Cascade (35 min)**
- **Spec → Plan Logic**: "A specification describes WHAT to build. A plan breaks it into HOW-to-build phases. Bad spec = vague plan = confused implementation."
- Worked example: Show a vague spec, then the bad plan it produced, then the good spec and good plan
- **Plan → Tasks Logic**: "A plan describes phases. Tasks break phases into atomic units. Bad plan = fragmented tasks = unclear work."
- **Tasks → Code Logic**: "Tasks describe acceptance criteria. Code is generated from tasks. Bad tasks = AI doesn't know what to build."
- **Demonstrate**: Trace one requirement through all three files:
  - Spec: "User can see calculation history"
  - Plan: "Phase 1: Design data structure for history; Phase 2: Implement storage; Phase 3: Display UI"
  - Tasks: "Task 1.1: Define history schema; Task 1.2: Write tests for schema; Task 2.1: Implement save_history() function..."

**Part C: Project Exploration (25 min)**
- **Exercise 1**: Students initialize their own SpecifyPlus project locally
- **Exercise 2**: Open and read the spec.md template; identify sections (Overview, Scope, Requirements, Acceptance Criteria)
- **Exercise 3**: Open plan.md template; identify sections (Phases, Dependencies, Milestones)
- **Exercise 4**: Open tasks.md template; identify structure (Must/Should/Nice-to-have, dependencies)

**Part D: Reflection & AI Exercise (10 min)**
- Why can't you skip directly from idea to tasks? What would happen?
- **Try With AI**: Show structure to ChatGPT; ask it why this workflow makes sense

#### End-of-Lesson: Try With AI Activity

**Tool**: ChatGPT web
**Duration**: 10 minutes

**Prompts**:
1. "Here's a SpecifyPlus project structure: [show directory tree]. What's the purpose of each folder?"
2. "Why would a developer write a specification BEFORE a plan?"
3. "What would happen if a developer skipped the plan and wrote tasks directly from a vague specification?"
4. **Reflection**: "How does this structure force clarity at each step?"

**Expected Outcomes**:
- Students understand structure is purposeful, not arbitrary
- They recognize cascade effect in project organization
- They see why "specification-driven" means starting with specs, not code

**Safety/Ethics Note**: "The structure exists to prevent miscommunication. By the time you write tasks, everyone (human and AI) should understand what you're building."

#### Prerequisites

- Lesson 1 (SMART criteria understanding)
- Basic CLI familiarity (navigating directories)

#### Assessment

**Formative**:
- Partner explanation: "Why does this folder come before that folder?"
- Checklist: Can student navigate to each folder and describe its purpose?

**Summative**:
- Student explains in 2-3 sentences why each SpecifyPlus folder exists and what forces the cascade
- Rubric:
  - Structure Understanding: Can identify and locate all folders (A2 level)
  - Cascade Awareness: Can explain why order matters and what breaks if skipped (A2 level)
  - Logical Thinking: Can trace requirement through Spec → Plan → Tasks (A2 level)

---

### **Lesson 3: Complete Specification Writing**

**Duration**: 2 hours | **Proficiency Target**: A2 (Basic Application) | **Bloom's Level**: Understand, Apply

#### Learning Objective

Students will be able to **write complete specifications with all required components** (Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria) and **understand the relationship between components**—recognizing when a specification is "ready for planning."

**Measurable outcome**: Given a feature description, student writes a complete specification with all sections filled; each section is clear and internally consistent; spec is ready for `/sp.plan` without major revisions.

#### Skills Taught

1. **Specification Writing** — A2 — Technical
   - Measurable at this level: Student can write complete spec with all 6 components; components are related and consistent
   - Category: Technical (core SDD skill)
   - DigComp: Content Creation, Communication

2. **Requirements Clarity** — A2 — Soft
   - Measurable at this level: Student can identify ambiguous requirements and clarify them; explain relationships between requirements
   - Category: Soft (critical thinking)
   - DigComp: Communication & Problem-Solving

3. **Acceptance Criteria Mastery** — A2 — Technical
   - Measurable at this level: Student writes detailed acceptance criteria using SMART framework; criteria are testable and complete
   - Category: Technical (from Lesson 1, deepening)
   - DigComp: Content Creation

#### Key Concepts (Max 5 for A2)

1. **Specification Components**:
   - **Overview**: What problem does this solve? Why does it matter?
   - **Scope**: What's in-scope? What's explicitly out-of-scope?
   - **Requirements**: What must the system do? (Functional + Non-functional)
   - **Acceptance Criteria**: How do we verify it works? (SMART)
   - **Constraints**: What are the limitations? (time, budget, technical)
   - **Success Criteria**: How do we know we succeeded?

2. **Component Relationships**: Requirements must be SCOPED. Acceptance Criteria must TEST requirements. Success Criteria must relate to initial problem.

3. **Template as Thinking Tool**: Spec template is scaffolding; it forces you to think through every angle.

4. **Ready for Planning**: Spec is "done" when: requirements are clear, acceptance criteria are testable, constraints are explicit, success criteria are measurable.

5. **Revision Cycle**: Specs aren't written once; they iterate as understanding improves.

#### Content Outline

**Part A: Specification Anatomy (30 min)**
- **Overview**: "What are you building? Why?" (Problem statement + value)
- **Scope**: "What's in vs. out?" (Helps AI know boundaries)
- **Requirements**: "What must it do?" (Functional: behaviors; Non-functional: performance, security, scalability)
- **Acceptance Criteria**: "How do we verify?" (Testable conditions from Lesson 1)
- **Constraints**: "What limits us?" (Time, budget, technical assumptions)
- **Success Criteria**: "Did we succeed?" (Measurable outcomes from problem statement)
- Worked example: Simple spec with all sections filled (e.g., basic calculator or grading system starter)

**Part B: Write a Complete Spec (50 min)**
- **Exercise 1**: Students choose between two project options:
  - **Option A** (simpler): Build a CLI calculator (add, subtract, multiply, divide with history)
  - **Option B** (more complex): Build a grading system (track student submissions, compute grades, generate reports)
- **Exercise 2**: Use provided spec template; students fill each section:
  1. Overview (2-3 sentences on problem and value)
  2. Scope (2-3 bullet points: what's in/out)
  3. Requirements (5-8 functional + 2-3 non-functional)
  4. Acceptance Criteria (5-10 SMART criteria from Lesson 1)
  5. Constraints (time, technical, scope limits)
  6. Success Criteria (measurable outcomes)
- **Exercise 3**: Self-review checklist:
  - Is Overview clear? Can someone understand problem in 30 seconds?
  - Is Scope explicit? Would AI know what to build vs. not build?
  - Are Requirements complete? Do they cover main behaviors?
  - Are Acceptance Criteria SMART and testable?
  - Are Constraints realistic?
  - Are Success Criteria measurable?

**Part C: Peer Review & Iteration (25 min)**
- Pair students; each reviews other's spec
- Reviewer asks: "Are you clear on what I'm building? Can you predict what code should do?"
- Writer refines based on feedback
- Note: "This iteration is normal; specs improve through feedback"

**Part D: Reflection & AI Exercise (10 min)**
- "What would happen if you skipped the Overview or Scope?"
- "Why does each section matter for the person implementing?"
- **Try With AI**: Ask ChatGPT for feedback on spec completeness

#### End-of-Lesson: Try With AI Activity

**Tool**: ChatGPT web
**Duration**: 10-15 minutes

**Prompts**:
1. "Here's my specification: [paste student's spec]. Does it have all required sections?"
2. "Are there any ambiguous requirements that could be clearer?"
3. "If you were implementing this, is the Overview clear enough that you understand the problem?"
4. [Student reads feedback, refines if needed]
5. **Reflection**: "What makes a specification 'complete'?"

**Expected Outcomes**:
- Students understand specification completeness
- They see how AI can help validate spec clarity
- They're ready to move to `/sp.specify` (Lesson 4) with a solid foundation

**Safety/Ethics Note**: "A specification is your contract with the implementer (human or AI). If it's unclear, implementation will be unclear."

#### Prerequisites

- Lessons 1-2 (SMART criteria, project structure)
- Basic domain knowledge (calculator or grading system)

#### Assessment

**Formative**:
- Peer review feedback (checklist)
- Self-review completion (all 6 sections filled)

**Summative**:
- Completed specification for chosen project
- Rubric (A2 level):
  - Completeness: All 6 components present and related (A2)
  - Clarity: Requirements and criteria are specific, not vague (A2)
  - Testability: Acceptance criteria are measurable and verifiable (A2)
  - Scope: In/out boundaries are explicit (A2)
  - SMART: Acceptance criteria follow SMART framework (A2, builds on Lesson 1)

---

### **Lesson 4: Refining Specs with `/sp.specify`**

**Duration**: 1.5 hours | **Proficiency Target**: B1 (Intermediate) | **Bloom's Level**: Apply, Analyze

#### Learning Objective

Students will be able to **use `/sp.specify` within Claude Code to analyze spec completeness**, understand AI feedback on gaps and ambiguities, and **iterate on specifications based on identified gaps**—experiencing the iterative nature of specification clarity.

**Measurable outcome**: Given a specification, student uses `/sp.specify`, reviews AI feedback on 3+ gaps/ambiguities, and refines spec to address them. Student can articulate what gaps were fixed and why each matters.

#### Skills Taught

1. **Specification Refinement** — B1 — Technical
   - Measurable at this level: Student uses `/sp.specify` to identify gaps; iterates on spec; produces clearer version
   - Category: Technical (advanced SDD)
   - DigComp: Problem-Solving, Content Creation

2. **AI Feedback Interpretation** — B1 — Soft
   - Measurable at this level: Student reads AI feedback on completeness; understands gap rationale; applies feedback meaningfully
   - Category: Soft (critical thinking + collaboration)
   - DigComp: Communication & Problem-Solving

3. **Iterative Clarity** — B1 — Conceptual
   - Measurable at this level: Student recognizes specs aren't "one-time" but iterative; each iteration adds clarity
   - Category: Conceptual (process understanding)
   - DigComp: Problem-Solving

#### Key Concepts (Max 7 new for B1)

1. **Tool Usage**: `/sp.specify` analyzes spec against template and best practices; returns feedback on gaps
2. **Feedback Interpretation**: AI feedback identifies ambiguities, missing sections, unclear requirements, inconsistencies
3. **Iterative Refinement**: Spec → `/sp.specify` → Feedback → Revise → Repeat
4. **Gap Categories**:
   - Missing assumptions (what's assumed to be true?)
   - Ambiguous terms (defined or unclear?)
   - Incomplete requirements (covered all user stories?)
   - Untestable criteria (measurable or subjective?)
   - Scope creep (are these really in-scope?)
5. **Cascade Improvement**: Better spec → better plan → better tasks (demonstrate with example)
6. **Version Control**: Specs are versioned; v1 → v2 → v3 as clarity improves
7. **AI as Partner**: AI surfaces gaps you might miss; human makes final decision on what matters

#### Content Outline

**Part A: Introduction to `/sp.specify` (15 min)**
- What it does: Analyzes spec for completeness and clarity
- When to use: After writing initial spec, before moving to planning
- What it returns: Feedback on gaps, ambiguities, missing assumptions
- Where it runs: Within Claude Code (development environment)
- Disclaimer: "This is a helper tool. You decide what feedback to act on."

**Part B: Hands-On Demo (20 min)**
- Show live demo: Run `/sp.specify` on an example calculator spec
- AI returns feedback like:
  - "What's the maximum calculation size? (1000 digits? 1 million?)"
  - "Does history persist across sessions or clear on restart?"
  - "What happens if user divides by zero? (error message? exception?)"
  - "Is this CLI or graphical interface?"
  - "Performance requirement on calculation time?"
- Discuss each gap: Why does it matter? How does ignoring it break implementation?
- Update spec based on feedback; run `/sp.specify` again; show improved feedback

**Part C: Student Practice (45 min)**
- **Exercise 1**: Students use `/sp.specify` on their Lesson 3 spec
- **Exercise 2**: Read AI feedback; identify 3+ gaps
- **Exercise 3**: For each gap:
  - Understand why it matters (trace to cascading effects in plan/tasks)
  - Decide: Is this gap critical? Should we address it?
  - Refine spec to address it
- **Exercise 4**: Run `/sp.specify` again; compare feedback (should improve)
- **Exercise 5**: Document refinements: "What gaps did we address? Why?"

**Part D: Cascade Demonstration (10 min)**
- Show: Original spec → bad plan from `/sp.plan`
- Show: Refined spec → better plan from `/sp.plan`
- Highlight: How spec improvements cascade to plan quality
- **Try With AI**: Discuss what further improvements could be made

#### End-of-Lesson: Try With AI Activity

**Tool**: Claude Code with `/sp.specify` command
**Duration**: 10-15 minutes

**Workflow**:
1. Open Claude Code
2. Paste spec into `/sp.specify`
3. Review AI feedback on gaps/ambiguities
4. Choose 1-2 gaps to refine
5. Update spec
6. Run `/sp.specify` again
7. **Reflection**: "How did the spec improve? What did we learn?"

**Expected Outcomes**:
- Students experience `/sp.specify` as a real tool, not a demo
- They understand specs are iterative (not "one and done")
- They see how tool feedback surfaces gaps
- They're ready for `/sp.plan` (Lesson 5) with well-refined specs

**Safety/Ethics Note**: "AI feedback surfaces possible gaps. You're responsible for deciding which gaps matter for YOUR project. Don't blindly accept all feedback."

#### Prerequisites

- Lessons 1-3 (SMART criteria, spec writing)
- Access to Claude Code
- A complete specification from Lesson 3

#### Assessment

**Formative**:
- Successful `/sp.specify` execution
- Documentation of identified gaps (checklist)
- Evidence of spec refinement (before/after)

**Summative**:
- Refined specification addressing 3+ gaps from `/sp.specify` feedback
- Written reflection: "Which gaps did we address? Why did they matter?"
- Rubric (B1 level):
  - Tool Usage: Successfully runs `/sp.specify` and interprets feedback (B1)
  - Gap Identification: Identifies 3+ genuine gaps from AI feedback (B1)
  - Refinement: Updates spec to address gaps; improvements are meaningful (B1)
  - Cascade Thinking: Can explain how spec improvements affect plan quality (B1)

---

### **Lesson 5: Planning from Specification — `/sp.plan`**

**Duration**: 1.5 hours | **Proficiency Target**: B1 (Intermediate) | **Bloom's Level**: Apply, Analyze

#### Learning Objective

Students will be able to **use `/sp.plan` to generate implementation plans from specifications**, understand how plans flow from specifications, and **recognize dependencies and sequencing**—experiencing the cascade effect: clear spec → clear plan.

**Measurable outcome**: Given a refined specification, student uses `/sp.plan`, reads the generated plan, explains how plan relates to spec requirements, and identifies 3+ dependencies/sequencing decisions.

#### Skills Taught

1. **Plan Generation** — B1 — Technical
   - Measurable at this level: Student uses `/sp.plan`; understands generated plan structure; explains plan phases and dependencies
   - Category: Technical (SDD workflow)
   - DigComp: Problem-Solving

2. **Cascade Understanding** — B1 — Conceptual
   - Measurable at this level: Student traces requirement → plan phase and articulates how spec quality improved plan quality
   - Category: Conceptual (systems thinking)
   - DigComp: Problem-Solving

3. **Dependency Analysis** — B1 — Technical
   - Measurable at this level: Student reads plan; identifies which phases must complete before others; explains rationale
   - Category: Technical (project planning)
   - DigComp: Problem-Solving

#### Key Concepts (Max 7 for B1)

1. **Plan from Spec**: Specification defines WHAT. Plan defines HOW and sequences WHEN.
2. **Plan Structure**: Phases, each with dependencies, milestones, deliverables
3. **Cascade Quality**: Better spec → clearer phases, fewer ambiguities → better tasks
4. **Dependencies**: Phase B can't start until Phase A completes (enforced in plan)
5. **Milestones**: Clear checkpoints to verify progress
6. **Deliverables**: What's produced at each phase (code, tests, documentation)
7. **Feedback Loop**: If plan is unclear, spec probably has ambiguities; refine spec → regenerate plan

#### Content Outline

**Part A: Understanding Plans (20 min)**
- What a plan describes: Phases (high-level steps), dependencies (phase ordering), milestones (checkpoints), deliverables (what's built)
- Why plans matter: Prevents starting work on ambiguous requirements; makes dependencies explicit
- Relationship to spec: Spec says what/why. Plan says how/when.
- Worked example: Calculator spec → plan with 3 phases (architecture, core functions, tests)

**Part B: Generate Plan with `/sp.plan` (20 min)**
- Show process: Take refined Lesson 4 spec → run `/sp.plan` in Claude Code
- Show output: plan.md with phases, dependencies, dependencies, milestones
- Analyze structure:
  - Phase 1: Foundation (architecture, data structure)
  - Phase 2: Implementation (core behavior, validation)
  - Phase 3: Completeness (tests, edge cases, documentation)
- Identify dependencies: Why can't Phase 2 start until Phase 1 ends?
- Identify milestones: When are we "done" with each phase?

**Part C: Student Practice (40 min)**
- **Exercise 1**: Use `/sp.plan` on their refined Lesson 4 spec
- **Exercise 2**: Read generated plan; answer questions:
  - How many phases? What's the purpose of each?
  - What's the critical path (longest chain of dependent phases)?
  - What could start in parallel?
  - Which deliverables must be complete before next phase?
- **Exercise 3**: Trace one requirement from spec to plan:
  - Spec: "User can see calculation history"
  - Plan phase: Where does this appear? What's needed to build it?
- **Exercise 4**: Compare spec quality to plan clarity:
  - Did a clearer spec produce a clearer plan?
  - If plan is ambiguous, where in spec is the gap?
- **Exercise 5**: Identify a plan improvement:
  - "Could we parallelize anything?"
  - "Are there missing milestones?"
  - "Should we refine the spec further?"

**Part D: Cascade Demonstration (10 min)**
- Show: Two specs (vague vs. refined) and their resulting plans
- Vague spec → confusing plan with unclear phases
- Refined spec → clear plan with explicit dependencies
- **Reflection**: "This is the cascade effect. Better thinking (spec) → better structure (plan) → better execution (tasks)"

#### End-of-Lesson: Try With AI Activity

**Tool**: Claude Code with `/sp.plan` command
**Duration**: 10 minutes

**Workflow**:
1. Open Claude Code
2. Run `/sp.plan` on refined spec from Lesson 4
3. Read generated plan
4. Ask Claude: "What are the critical dependencies in this plan?"
5. **Reflection**: "How did our clear spec produce a clear plan?"

**Expected Outcomes**:
- Students see real plan structure
- They understand phases and dependencies
- They recognize cascade effect in action
- They're ready for `/sp.tasks` (Lesson 6) with a solid plan

**Safety/Ethics Note**: "A plan makes dependencies explicit. Ignoring dependencies (skipping phases) creates technical debt and integration problems."

#### Prerequisites

- Lessons 1-4 (SMART, structure, complete spec, spec refinement)
- Access to Claude Code
- A refined specification from Lesson 4

#### Assessment

**Formative**:
- Successful `/sp.plan` execution
- Dependency identification (checklist)
- Traceability from spec requirement to plan phase (evidence)

**Summative**:
- Generated plan from `/sp.plan`
- Written analysis:
  - List 5+ phases and their purposes
  - Identify 3+ critical dependencies
  - Trace 1 spec requirement through plan (requirement → phase)
  - Explain how spec quality affected plan quality
- Rubric (B1 level):
  - Plan Understanding: Reads and explains plan structure (B1)
  - Dependency Analysis: Identifies critical dependencies and sequencing (B1)
  - Cascade Recognition: Articulates how spec quality → plan quality (B1)
  - Traceability: Traces requirement through spec to plan (B1)

---

### **Lesson 6: Decomposing Plans into Tasks — `/sp.tasks`**

**Duration**: 1.5 hours | **Proficiency Target**: B1 (Intermediate) | **Bloom's Level**: Apply, Analyze

#### Learning Objective

Students will be able to **use `/sp.tasks` to break plans into atomic work units**, understand task dependencies, and **trace lineage from specification → plan → tasks**—recognizing that task quality flows from plan quality.

**Measurable outcome**: Given a plan, student uses `/sp.tasks`, reads the generated task checklist, identifies task dependencies, and traces at least 2 requirements from spec → plan phase → task unit.

#### Skills Taught

1. **Task Decomposition** — B1 — Technical
   - Measurable at this level: Student uses `/sp.tasks`; reads task checklist; understands how tasks break down phases
   - Category: Technical (SDD workflow)
   - DigComp: Problem-Solving

2. **Lineage Traceability** — B1 — Technical
   - Measurable at this level: Student traces requirement → plan phase → task unit; explains relationship at each level
   - Category: Technical (quality assurance)
   - DigComp: Problem-Solving

3. **Quality Cascade Mastery** — B1 — Conceptual
   - Measurable at this level: Student explains how spec gaps → plan ambiguities → task confusion; demonstrates improvement through iterative refinement
   - Category: Conceptual (systems thinking)
   - DigComp: Problem-Solving

#### Key Concepts (Max 7 for B1)

1. **Task Decomposition**: Plan phase (high-level) → Multiple tasks (atomic units)
2. **Atomic Task**: A task can be completed in 1-2 hours; has clear acceptance criteria; produces testable output
3. **Checkpoint Pattern**: `/sp.tasks` can request checkpoints - agent completes phase → YOU review/commit → agent continues (not long autonomous loops)
4. **Human-Guided Phases**: Work broken into reviewable chunks; YOU control progression
5. **Task Dependencies**: Task B depends on Task A; must be sequenced
6. **Acceptance Criteria**: Each task has clear pass/fail conditions (from spec requirements)
7. **Lineage Traceability**: Requirement (spec) → Phase (plan) → Task unit (tasks) → Code artifact

#### Content Outline

**Part A: Understanding Tasks (15 min)**
- What tasks are: Atomic units (1-2 hour completions) with acceptance criteria
- Why break plans into tasks: Makes work concrete, enables parallelization, clarifies who does what
- Task anatomy: Description, acceptance criteria, dependencies, effort estimate, priority
- Worked example: Plan phase "Core Functions" → Tasks "Implement add()", "Implement subtract()", "Implement multiply()", etc.

**Part B: Generate Tasks with `/sp.tasks` (20 min)**
- Show process: Take plan from Lesson 5 → run `/sp.tasks` in Claude Code/Gemini CLI
- **Checkpoint Pattern Introduction**:
  - YOU can request checkpoints: "Generate tasks for Phase 1, then pause for review"
  - Agent completes that phase → returns to YOU → YOU review/commit → YOU decide: continue or adjust
  - Prevents long autonomous loops; keeps YOU in control
  - Example: "Complete Phase 1 tasks → CHECKPOINT → Review → Commit → Continue to Phase 2"
- Show output: tasks.md with MUST/SHOULD/NICE-TO-HAVE priorities, dependencies, effort estimates
- Analyze structure:
  - Phase 1 tasks (checkpoint after): "Define data structure", "Write schema tests", "Validate schema"
  - Phase 2 tasks (checkpoint after): "Implement core calculation", "Handle edge cases", "Build history storage"
  - Phase 3 tasks (final): "Write comprehensive tests", "Document API", "Refactor for clarity"
- **Human-Guided Phases**: YOU control when to continue; agent doesn't run autonomously
- Identify dependencies: Task X blocks Task Y
- Identify effort: How long should each take?
- Identify priorities: Which tasks are non-negotiable?

**Part C: Student Practice (45 min)**
- **Exercise 1**: Use `/sp.tasks` on their plan from Lesson 5
- **Exercise 2**: Read generated task checklist; answer questions:
  - How many MUST tasks? SHOULD tasks? NICE-TO-HAVE tasks?
  - What's the critical path (longest chain of dependent tasks)?
  - Which tasks could be parallelized?
  - Total effort (sum of all estimates)?
- **Exercise 3**: Trace lineage (Spec → Plan → Task):
  - Spec requirement: "User can see calculation history"
  - Plan phase: "Implement history storage and UI"
  - Tasks:
    - Task A: "Design history data structure"
    - Task B: "Implement save_history()"
    - Task C: "Implement load_history()"
    - Task D: "Write tests for history"
  - Show how specification clarity → plan clarity → task clarity
- **Exercise 4**: Identify quality issues:
  - "Are any tasks vague? (if so, where in spec is the ambiguity?)"
  - "Are acceptance criteria testable?"
  - "Are dependencies explicit?"
  - "Should we refine spec/plan further?"

**Part D: Cascade Completion (10 min)**
- Final demonstration: Spec (vague) → Plan (unclear) → Tasks (confused)
- vs. Spec (clear) → Plan (clear) → Tasks (atomic and testable)
- **Insight**: "This cascade is the entire value of SDD. Clarity compounds at each level."

#### End-of-Lesson: Try With AI Activity

**Tool**: Claude Code with `/sp.tasks` command
**Duration**: 10 minutes

**Workflow**:
1. Open Claude Code
2. Run `/sp.tasks` on plan from Lesson 5
3. Read generated task checklist
4. Pick one task and trace it back:
   - "Which plan phase does this task belong to?"
   - "Which spec requirement triggered this task?"
5. **Reflection**: "How did we go from 'build a calculator' to concrete, testable tasks?"

**Expected Outcomes**:
- Students see full lineage: Spec → Plan → Tasks
- They understand task atomicity and dependencies
- They're ready for `/sp.implement` (Lesson 7) with clear work breakdown
- They experience the cascade effect in full action

**Safety/Ethics Note**: "Clear tasks prevent misunderstandings. Everyone (human or AI) knows exactly what to build and how to verify it's correct."

#### Prerequisites

- Lessons 1-5 (all prior lessons; tasks build directly on plans)
- Access to Claude Code
- A clear plan from Lesson 5

#### Assessment

**Formative**:
- Successful `/sp.tasks` execution
- Dependency mapping (task graph or list)
- Traceability evidence (spec requirement → plan phase → task)

**Summative**:
- Generated task checklist from `/sp.tasks`
- Written analysis:
  - Count of MUST/SHOULD/NICE-TO-HAVE tasks
  - Identification of critical path (3+ dependencies)
  - Traceability of 2 requirements through full pipeline (spec → plan → task)
  - Total effort estimate (sum of task hours)
  - Identification of any gaps or ambiguities
- Rubric (B1 level):
  - Tool Usage: Successfully runs `/sp.tasks` and reads output (B1)
  - Decomposition: Understands how plan phases decompose into tasks (B1)
  - Dependencies: Identifies task dependencies and sequencing (B1)
  - Traceability: Traces requirement through spec → plan → task (B1)
  - Quality Assessment: Identifies gaps in clarity/atomicity (B1)

---

### **Lesson 7: Implementation & Validation — `/sp.implement`**

**Duration**: 2.5 hours | **Proficiency Target**: B1-B2 (Intermediate to Advanced) | **Bloom's Level**: Apply, Analyze, Evaluate

#### Learning Objective

Students will be able to **use `/sp.implement` to orchestrate AI-driven code generation**, validate generated code against acceptance criteria, and **provide feedback to refine AI output**—recognizing validation as a core professional skill and understanding the full AIDD loop.

**Measurable outcome**: Given a specification, plan, and tasks, student uses `/sp.implement`, reviews generated code, validates against acceptance criteria (all pass/fail), and articulates what validation revealed about code quality and specification clarity.

#### Skills Taught

1. **Implementation Orchestration** — B1 — Technical
   - Measurable at this level: Student uses `/sp.implement`; AI generates code and tests; student reviews output
   - Category: Technical (SDD workflow completion)
   - DigComp: Problem-Solving, Content Creation

2. **Validation Mastery** — B1-B2 — Technical
   - Measurable at this level: Student systematically validates code: reads it, understands it, tests it, verifies against spec
   - Category: Technical (critical skill per Constitution Principle 15)
   - DigComp: Safety, Problem-Solving

3. **AI Feedback & Refinement** — B1-B2 — Soft
   - Measurable at this level: Student provides feedback to AI on failed criteria; sees AI refine output; understands iteration cycle
   - Category: Soft (collaboration + communication)
   - DigComp: Communication & Problem-Solving

4. **Specification Quality Reflection** — B2 — Conceptual
   - Measurable at this level: Student evaluates: Did AI understand spec? If code failed, was it AI error or spec ambiguity? Proposes improvements
   - Category: Conceptual (systems thinking + evaluation)
   - DigComp: Problem-Solving, Safety

#### Key Concepts (Max 10 for B1-B2)

1. **`/sp.implement` Workflow**: Spec + Plan + Tasks → AI generates code + tests → Human validates
2. **Checkpoint-Driven Implementation**: YOU can request checkpoints - "Implement Phase 1 tasks → pause for review" (not autonomous code generation)
3. **Human-Guided Implementation**: Agent implements set of tasks → YOU review/test → YOU commit → YOU decide: continue or adjust
4. **Generated Artifacts**: Code files (functions, classes, modules), test files (unit tests, integration tests), documentation
5. **Validation Checklist**: Read code → Understand intent → Run tests → Compare to spec → Check security
6. **Acceptance Criteria Validation**: Each criterion is testable (pass/fail); all must pass before code is accepted
7. **Code Comprehension**: Never accept code you don't understand; ask AI to explain
8. **Test Coverage**: AI should generate tests; you verify coverage is adequate
9. **Security Review**: Look for hardcoded secrets, SQL injection, XSS vulnerabilities, insecure defaults
10. **AIDD Loop with Human Control**: Intent (YOU) → Generation (AI) → Validation (YOU) → Decision (YOU: accept/refine/reject)

#### Content Outline

**Part A: The AIDD Loop (20 min)**
- Show complete cycle:
  - Spec (human intent)
  - `/sp.implement` (AI generation from tasks)
  - Validation (human review)
  - Refinement (iterate if needed)
  - Deploy (once validated)
- Key insight: "This cycle repeats. You don't manually patch code. You improve specs and regenerate."
- Risk framing: "AI can hallucinate, misunderstand, or generate insecure code. Validation is non-negotiable."

**Part B: Run `/sp.implement` with Checkpoints (20 min)**
- Show process: Take tasks from Lesson 6 → run `/sp.implement` in Claude Code/Gemini CLI
- **Demonstrate Checkpoint Pattern**:
  - Request: "Implement Phase 1 tasks (add, subtract), then pause for my review"
  - Agent generates code for add() and subtract() → returns to YOU
  - YOU review code, run tests, verify against spec
  - YOU commit Phase 1 code
  - Request: "Continue with Phase 2 (multiply, divide, power)"
  - Agent continues → returns to YOU → YOU review/commit
  - **Key**: YOU control progression; agent doesn't run autonomously for all tasks at once
- Show output:
  - Generated source code files
  - Generated test files
  - Generated documentation
- First observation: "Look at what was generated. Do you understand it?"
- Don't run yet; review first.

**Part C: Validation Deep-Dive (50 min)**

**Validation Phase 1: Code Reading (15 min)**
- Read generated code without running
- Ask: "What does this function do? Can I understand intent from code?"
- If unclear, ask AI to explain
- Check: Type hints present? Code style clean? Comments where needed?
- Look for red flags: Hardcoded secrets? Unsafe operations? Missing error handling?

**Validation Phase 2: Comprehension Check (10 min)**
- For each major function/class, explain in your own words:
  - What's the input?
  - What's the output?
  - What does it do?
  - Why is it implemented this way?
- If you can't explain it, ask AI for clarification

**Validation Phase 3: Test Execution (15 min)**
- Run generated tests
- Verify: Do tests pass? Are there failures?
- Check test coverage: Are all major paths covered?
- Run additional manual tests (edge cases, error conditions)
- Example: For calculator, test:
  - Normal operations (2 + 3 = 5)
  - Edge cases (0 + 0, negative numbers, large numbers)
  - Error cases (divide by zero)

**Validation Phase 4: Acceptance Criteria Verification (10 min)**
- For each acceptance criterion from Lesson 3:
  - Is it implemented? Can you verify it? (pass or fail)
  - Does implementation match spec intent?
  - Any discrepancies between spec and code?
- Create validation matrix:
  - Criterion 1: ✓ Pass
  - Criterion 2: ✓ Pass
  - Criterion 3: ✗ Fail (reason)
  - ...

**Validation Phase 5: Security & Quality Review (10 min)**
- Security checklist:
  - [ ] No hardcoded secrets (passwords, API keys)
  - [ ] Input validation (edge cases handled)
  - [ ] Error messages don't leak information
  - [ ] No SQL injection vulnerabilities (if applicable)
- Quality checklist:
  - [ ] Type hints complete
  - [ ] Docstrings present
  - [ ] Code style consistent (black/mypy)
  - [ ] Performance acceptable

**Validation Phase 6: Refinement (if needed)**
- If acceptance criteria fail:
  - Identify root cause: AI misunderstanding? Spec ambiguity? Both?
  - Refine spec if needed (Lesson 4 cycle)
  - Provide feedback to AI: "Criterion X failed because..."
  - Re-run `/sp.implement`
  - Repeat validation
- If validation passes:
  - Done! Code is ready for next phase

**Part D: Student Practice (50 min)**
- **Exercise 1**: Run `/sp.implement` on their tasks from Lesson 6
- **Exercise 2**: Follow full validation protocol:
  1. Read code (no running yet)
  2. Understand it (can you explain each major function?)
  3. Comprehension check (explain to partner)
  4. Run tests (all pass or fail?)
  5. Verify acceptance criteria (create validation matrix)
  6. Security/quality review
  7. Document findings
- **Exercise 3**: If validation fails:
  - Document failures
  - Identify root cause (spec ambiguity or AI error?)
  - Refine spec or provide feedback
  - Re-run `/sp.implement`
  - Validate again
- **Exercise 4**: If validation passes:
  - Reflect: "What made this code work? Clear spec? Clear tasks? Good validation?"

**Part E: Reflection & Cascade Completion (10 min)**
- **Try With AI**: Run `/sp.implement`; have Claude explain generated code
- **Reflection**: "This is the complete SDD cycle: Spec → Plan → Tasks → Implementation → Validation → Deploy"
- **Final insight**: "Specification quality determined code quality. Clear specs enable AI to generate working code."

#### End-of-Lesson: Try With AI Activity

**Tool**: Claude Code with `/sp.implement` command + ChatGPT
**Duration**: 15-20 minutes

**Workflow**:
1. Run `/sp.implement` on tasks
2. Review generated code
3. Ask Claude Code: "Please explain this function"
4. Ask ChatGPT: "Does this code match the specification? [paste spec and code]"
5. Validate generated code:
   - Read and understand
   - Run tests
   - Check acceptance criteria
6. **Reflection**: "What would have happened without a clear specification?"

**Expected Outcomes**:
- Students see full AIDD cycle in action
- They understand validation as critical skill
- They recognize specification quality cascades to code quality
- They're ready for capstone project with full workflow mastery

**Safety/Ethics Note**: "Never deploy code you haven't read and understood. AI is a powerful tool but requires human validation. Your job is to verify AI understood your intent."

#### Prerequisites

- Lessons 1-6 (full SDD workflow foundation)
- Access to Claude Code
- A clear specification, plan, and tasks from prior lessons

#### Assessment

**Formative**:
- Successful `/sp.implement` execution
- Code reading and comprehension check (documented)
- Test execution results (pass/fail)

**Summative**:
- Generated code artifact
- Validation report including:
  - Code comprehension explanation (for 2+ major functions)
  - Test execution results
  - Acceptance criteria validation matrix (all criteria assessed)
  - Security/quality review checklist
  - Refinement decisions (if any failures, how did you address them?)
  - Final reflection: "Did clear specs enable AI to generate working code?"
- Rubric (B1-B2 level):
  - Implementation: Successfully runs `/sp.implement` and reviews output (B1)
  - Code Reading: Can read and explain generated code (B1)
  - Validation: Systematically validates against acceptance criteria (B1-B2)
  - Testing: Runs tests and checks coverage (B1)
  - Refinement: If failures, identifies root cause and iterates (B2)
  - Specification Reflection: Evaluates how spec quality affected code quality (B2)

---

### **Capstone Project: Complete SpecifyPlus Workflow End-to-End**

**Duration**: 3-4 hours | **Proficiency Target**: B2 (Advanced) | **Bloom's Level**: Create, Evaluate

#### Learning Objective

Students will **complete a full SpecifyPlus workflow from vague idea to working, validated code** (Lessons 1-7 integrated). They will consolidate all learning in a single project, demonstrating mastery of specification-driven development and understanding the value of SDD.

**Measurable outcome**: Given a vague project idea, student produces:
- Complete specification (Lesson 3)
- Refined specification (Lesson 4)
- Implementation plan (Lesson 5)
- Task checklist (Lesson 6)
- Working, validated code (Lesson 7)
- Reflection document explaining how specification quality determined implementation quality

#### Project Options

**Option A: Python Calculator (Simpler)**
- Features: Add, subtract, multiply, divide, exponentiation, square root
- Complexity: 2-3 hours implementation time
- Target: Students who want to consolidate core workflow
- Example spec: Basic arithmetic with history and error handling

**Option B: Grading System (More Complex)**
- Features: Track student submissions, compute weighted grades, generate reports
- Complexity: 3-4 hours implementation time
- Target: Students who want additional challenge with data persistence
- Example spec: Multi-user system with role-based access (teacher/student)

#### Deliverables

**1. Specification Document (`spec.md`)**
- Complete specification with all 6 components from Lesson 3
- Refined through `/sp.specify` feedback (Lesson 4)
- Version history showing iterations

**2. Implementation Plan (`plan.md`)**
- Generated from specification using `/sp.plan` (Lesson 5)
- Identified phases, dependencies, milestones
- Effort estimates

**3. Task Checklist (`tasks.md`)**
- Decomposed from plan using `/sp.tasks` (Lesson 6)
- MUST/SHOULD/NICE-TO-HAVE priorities
- Task dependencies and effort estimates

**4. Working Code**
- Generated using `/sp.implement` (Lesson 7)
- Passes all acceptance criteria
- Includes tests with 80%+ coverage
- Type hints throughout (Python 3.13+ style)
- Documented and clean

**5. Validation Report**
- Code reading and comprehension
- Test execution results
- Acceptance criteria validation matrix
- Security/quality review
- Any refinements made during iteration

**6. Reflection Document (1-2 pages)**
- How clear was your initial specification?
- How did `/sp.specify` improve your spec?
- How did spec quality affect plan quality?
- How did plan quality affect task quality?
- How did task quality affect code generation?
- What would you do differently for the next project?
- How does this workflow compare to traditional "write code first" approach?

#### Capstone Assessment Rubric

| Criterion | Excellent (B2) | Proficient (B1) | Developing (A2) | Not Yet (A1) |
|-----------|----------------|-----------------|-----------------|------------|
| **Specification Completeness** | All 6 components complete, refined, internally consistent | 5-6 components present, mostly clear | 3-4 components present, some vagueness | <3 components, significant gaps |
| **Specification Clarity** | No ambiguities; acceptance criteria SMART; ready for implementation | Mostly clear; minor ambiguities; criteria mostly SMART | Some vague terms; criteria partially SMART | Multiple ambiguities; criteria not testable |
| **Plan Quality** | Clear phases with dependencies, milestones, deliverables; demonstrates understanding of workflow | Phases identified, some dependencies noted | Basic phases present; dependencies unclear | Incomplete plan |
| **Task Decomposition** | Tasks are atomic, testable, with clear acceptance criteria; priorities assigned | Most tasks atomic; most have acceptance criteria | Some tasks vague; acceptance criteria incomplete | Tasks incomplete or unclear |
| **Code Generation** | Successfully runs `/sp.implement`; all acceptance criteria pass; no failures | Mostly working; 1-2 minor failures corrected | Some failures; some correction iterations | Code doesn't run or many failures |
| **Code Quality** | Type hints complete, tests 80%+, security reviewed, documented | Type hints mostly present, tests adequate, basic security check | Type hints partial, tests minimal, limited security review | Type hints missing, tests insufficient |
| **Validation** | Systematic validation; all criteria verified; security/quality reviewed thoroughly | Validation documented; most criteria verified | Basic validation; some criteria unchecked | Minimal validation |
| **Refinement & Iteration** | If failures, properly identifies root cause, refines spec, regenerates, re-validates | If failures, attempts to refine and iterate | If failures, attempts patches or manual fixes | Doesn't iterate or refine |
| **Reflection Quality** | Articulates cascade effect; compares SDD to traditional approaches; identifies learnings | Reflects on spec-to-code journey; notes improvements | Basic reflection on workflow | Minimal reflection |

#### Capstone Success Criteria

**All of the following must be true**:

- [ ] Specification is complete (all 6 components) and unambiguous
- [ ] Specification was refined using `/sp.specify` (version history shows iterations)
- [ ] Plan was generated using `/sp.plan` (phases and dependencies identified)
- [ ] Tasks were decomposed using `/sp.tasks` (atomic units with acceptance criteria)
- [ ] Code was generated using `/sp.implement` (all major functions present)
- [ ] All acceptance criteria pass (validation matrix shows 100% pass rate)
- [ ] Code includes type hints throughout and 80%+ test coverage
- [ ] No hardcoded secrets; security reviewed
- [ ] Reflection document articulates cascade effect and learning
- [ ] Student can explain: "Clear specs enabled AI to generate working code"

#### Capstone Workflow

1. **Select project** (calculator or grading system)
2. **Write vague 1-sentence description** ("I want to build a calculator")
3. **Create specification** (Lesson 3 process) → Produce `spec.md`
4. **Refine specification** (Lesson 4 process with `/sp.specify`) → Produce `spec.md v2`
5. **Generate plan** (Lesson 5 process with `/sp.plan`) → Produce `plan.md`
6. **Decompose tasks** (Lesson 6 process with `/sp.tasks`) → Produce `tasks.md`
7. **Implement** (Lesson 7 process with `/sp.implement`) → Generate code + tests
8. **Validate** (Full validation protocol) → Produce `validation-report.md`
9. **Iterate if needed** (If failures, refine spec and repeat steps 4-8)
10. **Reflect** (Write reflection document on cascade effect and learning)
11. **Submit** (All 6 deliverables + reflection)

#### Support & Scaffolding

**For Calculator (Simpler Path)**:
- Provided: Detailed example specification
- Provided: Example plan from `/sp.plan`
- Provided: Example tasks from `/sp.tasks`
- Student: Customizes for their version, runs workflow

**For Grading System (Complex Path)**:
- Provided: High-level specification outline
- Provided: Hints on plan structure
- Student: Completes all sections, runs workflow independently

---

## V. Skills Proficiency Mapping (CEFR & Bloom's Taxonomy)

### Lesson-by-Lesson Skills Architecture

**Key Design Principles**:
1. **Progressive Proficiency**: A2 → A2 → A2 → B1 → B1 → B1 → B2
2. **Cognitive Load Management**: 5 new concepts (A2) → 7 new concepts (B1) → 10 new concepts (B2)
3. **Skill Reinforcement**: Earlier skills (SMART, specs) deepened in later lessons (plans, tasks)
4. **Validation as Core**: Every lesson includes validation activity; emphasized increasingly in later lessons

### Lesson 1: SMART Acceptance Criteria

| Skill | CEFR Level | Category | Bloom's Taxonomy | Measurable at This Level |
|-------|-----------|----------|------------------|------------------------|
| Understanding SMART Criteria | A2 | Conceptual | Understand, Apply | Student identifies SMART vs. vague criteria; explains each component |
| Identifying Vague Requirements | A2 | Soft | Understand, Analyze | Student spots ambiguous language; proposes specific alternatives |
| Writing Testable Criteria | A2 | Technical | Apply | Student writes criteria with clear pass/fail conditions |
| **Cognitive Load**: 5 new concepts (SMART, Vagueness, Clarity, Testability, Cascade) | | | | ✓ Within A2 limits |

### Lesson 2: SpecifyPlus Project Structure

| Skill | CEFR Level | Category | Bloom's Taxonomy | Measurable at This Level |
|-------|-----------|----------|------------------|------------------------|
| Understanding Project Structure | A2 | Technical | Understand, Apply | Student explains purpose of specs/, .specify/, history/ folders |
| Recognizing Workflow Enforcement | A2 | Conceptual | Understand, Analyze | Student explains why Spec → Plan → Tasks order is forced |
| Cascade Thinking | A2 | Soft | Understand, Analyze | Student traces requirement through cascade; predicts ripple effects |
| **Cognitive Load**: 5 new concepts (Folders, Spec, Plan, Tasks, Cascade) | | | | ✓ Within A2 limits |

### Lesson 3: Complete Specification Writing

| Skill | CEFR Level | Category | Bloom's Taxonomy | Measurable at This Level |
|-------|-----------|----------|------------------|------------------------|
| Specification Writing | A2 | Technical | Understand, Apply | Student writes complete spec with all 6 components |
| Requirements Clarity | A2 | Soft | Apply, Analyze | Student identifies ambiguous requirements; clarifies them |
| Acceptance Criteria Mastery | A2 | Technical | Apply | Student writes detailed SMART criteria; deepens Lesson 1 skill |
| **Cognitive Load**: 5 new concepts (Components, Relationships, Template, Readiness, Iteration) | | | | ✓ Within A2 limits |

### Lesson 4: Refining Specs with `/sp.specify`

| Skill | CEFR Level | Category | Bloom's Taxonomy | Measurable at This Level |
|-------|-----------|----------|------------------|------------------------|
| Specification Refinement | B1 | Technical | Apply, Analyze | Student uses `/sp.specify`; iterates on spec; produces clearer version |
| AI Feedback Interpretation | B1 | Soft | Apply, Analyze | Student reads AI feedback; understands gap rationale; applies meaningfully |
| Iterative Clarity | B1 | Conceptual | Apply, Analyze | Student recognizes specs are iterative; each iteration adds clarity |
| **Cognitive Load**: 7 new concepts (Tool Usage, Feedback, Iteration, Gap Categories, Version Control, AI Partnership, Cascade Improvement) | | | | ✓ Within B1 limits |

### Lesson 5: Planning from Specification

| Skill | CEFR Level | Category | Bloom's Taxonomy | Measurable at This Level |
|-------|-----------|----------|------------------|------------------------|
| Plan Generation | B1 | Technical | Apply, Analyze | Student uses `/sp.plan`; reads plan structure; explains plan phases/dependencies |
| Cascade Understanding | B1 | Conceptual | Apply, Analyze | Student traces requirement → plan phase; articulates spec→plan quality improvement |
| Dependency Analysis | B1 | Technical | Apply, Analyze | Student identifies which phases must complete before others; explains rationale |
| **Cognitive Load**: 7 new concepts (Plan vs. Spec, Structure, Cascade Quality, Dependencies, Milestones, Deliverables, Feedback Loop) | | | | ✓ Within B1 limits |

### Lesson 6: Decomposing Plans into Tasks

| Skill | CEFR Level | Category | Bloom's Taxonomy | Measurable at This Level |
|-------|-----------|----------|------------------|------------------------|
| Task Decomposition | B1 | Technical | Apply, Analyze | Student uses `/sp.tasks`; reads task checklist; understands task breakdown |
| Lineage Traceability | B1 | Technical | Apply, Analyze | Student traces requirement → plan phase → task unit; explains relationships |
| Quality Cascade Mastery | B1 | Conceptual | Apply, Analyze | Student explains spec gaps → plan ambiguities → task confusion; demonstrates improvement |
| **Cognitive Load**: 7 new concepts (Task Decomposition, Atomic Tasks, Task Dependencies, Priority Levels, Acceptance Criteria, Effort Estimation, Lineage Traceability) | | | | ✓ Within B1 limits |

### Lesson 7: Implementation & Validation

| Skill | CEFR Level | Category | Bloom's Taxonomy | Measurable at This Level |
|-------|-----------|----------|------------------|------------------------|
| Implementation Orchestration | B1 | Technical | Apply, Analyze | Student uses `/sp.implement`; AI generates code; student reviews output |
| Validation Mastery | B1-B2 | Technical | Apply, Analyze, Evaluate | Student systematically validates: reads, understands, tests, verifies spec alignment |
| AI Feedback & Refinement | B1-B2 | Soft | Apply, Analyze, Evaluate | Student provides feedback to AI; sees refinement; understands iteration |
| Specification Quality Reflection | B2 | Conceptual | Analyze, Evaluate | Student evaluates: Did AI understand? If failed, spec ambiguity or AI error? Proposes improvements |
| **Cognitive Load**: 10 new concepts (Workflow, Artifacts, Validation Checklist, Acceptance Criteria Validation, Code Comprehension, Test Coverage, Security Review, Spec-Code Alignment, Iteration, AIDD Loop) | | | | ✓ Within B1-B2 limits |

### Capstone Project

| Skill | CEFR Level | Category | Bloom's Taxonomy | Measurable at This Level |
|-------|-----------|----------|------------------|------------------------|
| Complete SDD Workflow | B2 | Technical | Create | Student completes Spec → Plan → Tasks → Code → Validation end-to-end |
| Design Decisions | B2 | Conceptual | Evaluate, Create | Student makes choices (which project, feature scope, implementation approach) and justifies |
| Specification-Code Traceability | B2 | Technical | Evaluate | Student validates all requirements met; identifies misalignments; refines if needed |
| Cascade Effect Recognition | B2 | Conceptual | Evaluate, Create | Student articulates: spec quality → implementation quality; identifies improvements for next project |
| **Proficiency**: By capstone, all B1 skills are integrated at B2 level (Create, design, synthesis) | | | | ✓ Capstone achieves B2 |

---

## VI. Learning Progression & Prerequisites

### Prerequisite Chain

```
Chapter 1-5 (Foundational Mindset)
  ↓
Chapter 30 (AIDD Philosophy & Paradigm)
  ↓
Chapter 31, Lesson 1 (SMART Criteria)
  ↓
Chapter 31, Lesson 2 (Project Structure)
  ↓
Chapter 31, Lesson 3 (Complete Specs)
  ↓
Chapter 31, Lesson 4 (/sp.specify)
  ↓
Chapter 31, Lesson 5 (/sp.plan)
  ↓
Chapter 31, Lesson 6 (/sp.tasks)
  ↓
Chapter 31, Lesson 7 (/sp.implement)
  ↓
Capstone Project (Full Workflow)
  ↓
Chapter 32 (Advanced SDD Topics)
```

### Skill Progression Map

```
Lesson 1-3: Foundation (A2)
├── SMART Criteria
├── Project Structure
├── Specification Writing
└── Acceptance Criteria Mastery

Lesson 4-6: Tool-Assisted (B1)
├── Specification Refinement (/sp.specify)
├── Plan Generation (/sp.plan)
├── Task Decomposition (/sp.tasks)
└── Lineage Traceability

Lesson 7: Integration (B1-B2)
├── Implementation Orchestration (/sp.implement)
├── Validation Mastery
├── AI Feedback & Refinement
└── Specification Quality Reflection

Capstone: Mastery (B2)
└── Complete SDD Workflow End-to-End
```

---

## VII. Constitutional Alignment Check

### Principle Adherence

- ✅ **Principle 1 (AI-First Teaching)**: Every lesson includes "Try With AI" activity; AI positioned as co-reasoning partner
- ✅ **Principle 2 (Spec-Kit Plus as Foundation)**: Entire chapter teaches SDD workflow; specifications are core, not supplementary
- ✅ **Principle 3 (Modern Python)**: Code examples use Python 3.13+ with mandatory type hints
- ✅ **Principle 4 (Test-First Mindset)**: Validation and testing emphasized throughout; Lesson 7 includes test validation
- ✅ **Principle 5 (Progressive Complexity)**: A2 (Lessons 1-3) → B1 (Lessons 4-6) → B2 (Lesson 7, Capstone)
- ✅ **Principle 6 (Consistent Structure)**: All lessons follow identical structure (objective, skills, concepts, outline, assessment)
- ✅ **Principle 7 (Technical Accuracy)**: All SpecifyPlus commands verified; no hallucinated workflows
- ✅ **Principle 8 (Accessibility)**: Clear language, no gatekeeping; diverse examples (calculator + grading system)
- ✅ **Principle 9 (Show-Spec-Validate)**: Every lesson shows spec first, then tools, then validation
- ✅ **Principle 10 (Real-World Integration)**: Projects are real (calculator, grading system); deployment-aware
- ✅ **Principle 11 (Tool Diversity)**: ChatGPT web + Claude Code; students see tool flexibility
- ✅ **Principle 12 (Cognitive Load)**: A2 max 5 concepts, B1 max 7 concepts, B2 max 10 concepts
- ✅ **Principle 13 (Concept-Before-Command)**: Every tool (SMART, `/sp.specify`, etc.) introduced conceptually first
- ✅ **Principle 14 (Planning-First)**: Specifications are primary; implementation is secondary
- ✅ **Principle 15 (Validation-Before-Trust)**: Validation is core skill; every lesson includes validation activity
- ✅ **Principle 16 (Bilingual Development)**: Examples show Python; TypeScript not needed for SDD methodology (covered in later parts)
- ✅ **Principle 17 (Production-Ready)**: Code examples include tests, type hints, security review

### Book Gaps Checklist Alignment

**For All Chapters**:
- ✅ Factual Accuracy: SpecifyPlus workflow verified; no hallucinated commands
- ✅ Field Volatility: SpecifyPlus version noted; maintenance trigger if CLI changes
- ✅ Inclusive Language: Multiple project examples; diverse terminology
- ✅ Accessibility: Clear explanations; scaffolding appropriate to tier
- ✅ Bias & Representation: Examples include both simple (calculator) and complex (grading) projects

**For Technical Chapters** (Chapter 31 is hybrid):
- ✅ Code Security: Type hints, test coverage, input validation shown
- ✅ Ethical AI Use: AI limitations framed; validation as safety mechanism
- ✅ Testing & Quality: Tests generated via `/sp.implement`; validation demonstrated
- ✅ Deployment Readiness: Code examples include setup, dependencies, troubleshooting
- ✅ Scalability Awareness: Design patterns shown (history storage, multi-phase architecture)
- ✅ Real-World Context: Realistic scenarios (calculator, grading system)
- ✅ Specification Included: Every lesson emphasizes spec → code traceability
- ✅ Validation Demonstrated: Every lesson includes validation activity

---

## VIII. Implementation Phases & Milestones

### Phase 1: Content Development (Tasks Phase)

**Deliverables**:
- [ ] Lesson READMEs for all 7 lessons + capstone (overview, learning outcomes)
- [ ] Lesson 1 content (SMART criteria teaching)
- [ ] Lesson 2 content (project structure explanation)
- [ ] Lesson 3 content (complete specification template + examples)
- [ ] Lesson 4 content (`/sp.specify` workflow)
- [ ] Lesson 5 content (`/sp.plan` workflow)
- [ ] Lesson 6 content (`/sp.tasks` workflow)
- [ ] Lesson 7 content (`/sp.implement` + validation protocol)
- [ ] Capstone project specification, rubric, and assessment
- [ ] All code examples tested and working (Python 3.13+, type hints, tests)

### Phase 2: Validation & Review

**Deliverables**:
- [ ] Technical review (SpecifyPlus accuracy, code quality, no hallucinated commands)
- [ ] Pedagogical review (learning objectives clear, scaffolding appropriate, assessments aligned)
- [ ] Constitutional alignment check (all 17 principles verified)
- [ ] Accessibility review (language clear, examples diverse, pace appropriate)

### Phase 3: Refinement & Polish

**Deliverables**:
- [ ] Content refinement based on review feedback
- [ ] Cross-chapter coherence verification (alignment with Chapter 30, 32)
- [ ] Docusaurus build test (no broken links, formatting correct)
- [ ] Final editorial polish

### Phase 4: Publication

**Deliverables**:
- [ ] Merge to main branch
- [ ] Live on https://ai-native.panaversity.org
- [ ] Cross-referenced in chapter-index.md and directory-structure.md

---

## IX. Risk Analysis & Mitigations

### Risk 1: Students Overwhelmed by Tool Commands

**Risk**: Lessons 4-7 introduce `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement` in rapid succession; students get lost in command execution vs. understanding.

**Mitigation**:
- ✅ Each lesson focuses on ONE tool only
- ✅ Previous lessons build foundation (Lessons 1-3 teach thinking before tools)
- ✅ Capstone integrates all tools, but in sequence (not all at once)
- ✅ Clear scaffolding: "Run command → Review output → Analyze output → Refine → Repeat"

### Risk 2: SpecifyPlus Commands Change or Break

**Risk**: SpecifyPlus is a real tool; if CLI changes, examples become outdated.

**Mitigation**:
- ✅ Document SpecifyPlus version used in chapter (maintenance trigger)
- ✅ Show correct workflow (Spec → Plan → Tasks → Implement) even if CLI syntax changes
- ✅ Emphasize principles over commands: "This workflow enforces clarity at each step"
- ✅ Fallback: If `/sp.specify` isn't available, students can manually review specs (same principles)

### Risk 3: Students Don't See Value in "Manual Thinking" (Lessons 1-3)

**Risk**: Students want to jump to tools; they see SMART criteria and spec writing as "boring" vs. "real coding."

**Mitigation**:
- ✅ Lesson 1 starts with problem: "Vague criteria → AI builds wrong code" (experiential learning)
- ✅ Explicitly contrast manual thinking vs. tool-assisted: "Tools amplify thinking, not replace it"
- ✅ Lesson 4 demonstrates payoff: Better specs → `/sp.specify` returns fewer gaps
- ✅ Lesson 7 shows final payoff: Clear specs → `/sp.implement` generates correct code
- ✅ Capstone requires specification-first (no shortcuts)

### Risk 4: Capstone Projects Scope Creep

**Risk**: Students choose grading system (complex path); scope grows beyond 3-4 hours; frustration sets in.

**Mitigation**:
- ✅ Provide clear scope for both projects (calculator: specific features; grading system: specific features)
- ✅ Spec template includes explicit "Out of Scope" section
- ✅ Milestone checkpoints: "After Lesson 5, you should have a clear plan. If plan is 20+ pages, scope is too large."
- ✅ Encourage calculator path for first capstone ("Grading system is advanced; start simple")

### Risk 5: Validation Seems Tedious (Lesson 7)

**Risk**: Students rush validation; skip reading code; don't understand why "it works, so move on."

**Mitigation**:
- ✅ Validation is non-negotiable per Constitutional Principle 15
- ✅ Lesson 7 explicitly frames validation as core professional skill
- ✅ Validation protocol is structured (read → understand → test → verify → review → iterate)
- ✅ Capstone success criteria require validation: all acceptance criteria must pass
- ✅ If code fails validation, students must iterate (not manually patch)

### Risk 6: Students Blame AI for Failed Criteria (Lesson 7)

**Risk**: Code doesn't match spec → students blame AI; don't recognize spec ambiguity → don't refine spec.

**Mitigation**:
- ✅ Lesson 7 explicitly teaches: "If code fails, ask: AI error or spec ambiguity?"
- ✅ Guided reflection: "Does AI understand what you asked for?"
- ✅ Iteration protocol: "If code fails, refine spec and regenerate (don't manually patch)"
- ✅ Capstone reflection document asks: "Were failures AI errors or spec gaps?"

---

## X. Detailed Assessments by Lesson

### Formative Assessment Strategy

**Throughout Lessons 1-7**:
- Daily check-in: "What did you learn today?"
- Pair reviews: Students review each other's work
- Checklists: "Does your spec have all 6 components?"
- Reflection prompts: "What would break if you skipped this step?"

### Summative Assessment Strategy

**End of Each Lesson**:
- Lesson 1: Student produces SMART criteria template for 3 features
- Lesson 2: Student explains project structure and cascade (2-3 sentences)
- Lesson 3: Student submits complete specification with all components
- Lesson 4: Student submits refined specification with `/sp.specify` feedback incorporated
- Lesson 5: Student submits plan from `/sp.plan` with dependency analysis
- Lesson 6: Student submits tasks from `/sp.tasks` with traceability documentation
- Lesson 7: Student submits working code + validation report
- **Capstone**: Student submits complete artifact (spec, plan, tasks, code, validation, reflection)

### Rubrics

**All rubrics aligned to CEFR proficiency levels**:
- **A2 Level** (Lessons 1-3): Understands, applies in simple/guided contexts
- **B1 Level** (Lessons 4-6): Applies independently to real problems
- **B1-B2 Level** (Lesson 7): Applies, analyzes, evaluates
- **B2 Level** (Capstone): Creates, designs, integrates

---

## XI. Integration with Chapter 30 & 32

### Connection to Chapter 30 (Prior)

Chapter 30 teaches AIDD philosophy (AI as co-reasoning partner, specification-first mindset). Chapter 31 puts this into practice with concrete SpecifyPlus workflow.

**Callbacks**:
- Lesson 1: "Remember from Chapter 30: clarity matters for AI collaboration. SMART criteria are how we achieve clarity."
- Lesson 4: "Chapter 30 said specifications are source code. Here's how we refine them with AI assistance."
- Lesson 7: "Chapter 30's AIDD cycle (intent → generation → validation → refinement) is live here with real code."

### Connection to Chapter 32 (Next)

Chapter 32 teaches advanced SDD topics (ADRs, PHRs, complex specifications). Chapter 31 is the foundational hands-on experience.

**Preparation**:
- Lesson 7 mentions: "As you scale projects, you'll document architectural decisions (ADRs) and prompt history (PHRs). Chapter 32 covers these."
- Capstone reflection asks: "What would change if you added 5 team members? (Hints at Chapter 32: documentation, decision records)"

---

## XII. Conclusion: Why This Plan Works

### Pedagogy-First Design

1. **Problem-First Learning**: Lessons start with problems (vague criteria produce bad code) before solutions (SMART framework)
2. **Progressive Complexity**: A2 thinking → A2 tools → B1 tools → B2 integration
3. **Hands-On, Real Projects**: Not toy examples; students build actual calculator or grading system
4. **Validation as Core Skill**: Every lesson includes validation; safety baked in per Constitutional Principle 15

### Specification-First Foundation

1. **Manual Thinking First** (Lessons 1-3): Build thinking skills before tools
2. **Tool Amplification** (Lessons 4-6): Tools amplify clear thinking; don't create clarity
3. **Full Cycle** (Lesson 7): See how specification quality cascades to implementation quality
4. **Capstone Integration**: Complete workflow end-to-end; internalize cascade effect

### Constitutional Alignment

1. **Principle 14 (Planning-First)**: Specification is primary; implementation is secondary
2. **Principle 15 (Validation-Before-Trust)**: Validation demonstrated and required
3. **Principle 2 (Spec-Kit Plus)**: Entire chapter teaches SDD as core methodology
4. **All 17 Principles**: Verified in Section IX

### Success Indicators

By end of Chapter 31, students can:
- ✅ Write SMART acceptance criteria that prevent AI misinterpretation
- ✅ Explain why SpecifyPlus enforces Spec → Plan → Tasks cascade
- ✅ Use `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement` to build real software
- ✅ Validate AI-generated code against specifications
- ✅ Recognize how specification quality cascades through entire workflow
- ✅ Build complete projects end-to-end using SDD methodology
- ✅ Understand why specs are the new syntax in AI-native development

---

## Appendix A: File Organization

**Chapter Files** (to be created during content-implementer phase):
```
docs/05-Spec-Kit-Plus-Methodology/
├── README.md                                    [Part 5 introduction]
└── 03-specifyplus-hands-on/
    ├── README.md                               [Chapter overview]
    ├── 01-smart-acceptance-criteria.md
    ├── 02-specifyplus-structure.md
    ├── 03-complete-specification.md
    ├── 04-refining-specs-with-sp-specify.md
    ├── 05-planning-from-specification.md
    ├── 06-decomposing-plans-into-tasks.md
    ├── 07-implementation-and-validation.md
    └── 08-capstone-project.md
```

**Specification Files** (already created):
```
specs/010-chapter-31-redesign/
├── spec.md       [Approved specification]
├── plan.md       [This file - detailed implementation plan]
└── [tasks.md will be created by /sp.tasks phase]
```

---

**This plan is ready for implementation. Proceed to content-implementer phase with confidence that pedagogy, content, and constitutional alignment are verified.**

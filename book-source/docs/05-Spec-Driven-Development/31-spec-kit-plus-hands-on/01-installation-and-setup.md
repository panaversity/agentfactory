---
title: "Installation & Setup - AI-Native SDD Toolkit"
chapter: 31
lesson: 1
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding Spec-Kit Plus Architecture"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain that Spec-Kit Plus is an independent framework working with Claude Code or Gemini CLI"

  - name: "Recognizing Vertical Intelligence Pattern"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can identify the three-tier architecture: Human → Orchestrator → Specialized Subagents"

  - name: "Understanding Horizontal Intelligence"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Remember, Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student recognizes that ADRs and PHRs capture organizational knowledge"

  - name: "Tool Configuration (Claude Code vs Gemini CLI)"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can choose and configure an AI tool for Spec-Kit Plus work"

  - name: "Project Structure Navigation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand, Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify and explain the purpose of `.specify/`, `specs/`, and `history/` directories"

learning_objectives:
  - objective: "Explain the difference between Spec-Kit Plus framework and AI tool, and why they are separate"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Verbal explanation or written response"

  - objective: "Install Spec-Kit Plus framework successfully on your development machine"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successful installation verification (run `/sp.*` commands)"

  - objective: "Configure Claude Code or Gemini CLI to work with Spec-Kit Plus"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Command execution in chosen AI tool"

  - objective: "Navigate and understand the Spec-Kit Plus project structure"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of folder hierarchy and artifact relationships"

  - objective: "Verify complete setup by running a test command"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successful command execution with expected output"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Spec-Kit Plus framework, Horizontal Intelligence, Vertical Intelligence, AI tool options, Project structure) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore multi-tool setup-configure both Claude Code and Gemini CLI; compare workflows between the two"
  remedial_for_struggling: "Step-by-step guided installation with video/visual aids; installation verification checklist"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Installation & Setup - AI-Native SDD Toolkit

Welcome to hands-on Spec-Kit Plus development! Chapter 30 taught you **why** specification-driven development matters. This chapter teaches you **how** to do it-by building a real calculator project from specification to working code using Spec-Kit Plus and your AI companion.

This lesson gets your toolkit ready. By the end, you'll have Spec-Kit Plus installed, your AI tool configured, and a test project initialized. More importantly, you'll understand the architecture behind Spec-Kit Plus-why it's independent of any single AI tool, and how the three-tier Vertical Intelligence pattern (You → Orchestrator → Specialized Subagents) enables efficient workflow management.

---

## Part A: What Is Spec-Kit Plus? 

Before installing anything, let's clarify what Spec-Kit Plus actually is.

### The Architecture: Three Independent Layers

Spec-Kit Plus is an Opiniated ToolKit for SDD. It has three independent but integrated components:

**1. The Framework** (The actual Spec-Kit Plus toolkit)
- File templates for specifications, plans, tasks
- Directory structure enforcing Spec → Plan → Tasks progression
- Slash commands (`/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`, etc.)
- Prompt templates and evaluation guides
- Storage for artifacts (ADRs, PHRs)

**2. The AI Orchestrator** (Your chosen AI tool)
- Claude Code (recommended for this book)
- Gemini CLI (alternative option)
- Or any AI tool that can execute slash commands
- Acts as the "main collaborator" who understands Spec-Kit Plus workflow

**3. The Vertical Intelligence Layer** (Delegated AI capabilities)

**The Critical Insight**: Spec-Kit Plus is **not** a tool that requires a specific AI service. It's an **opinionated methodology framework** that works with Claude Code, Gemini CLI, or any AI tool capable of understanding slash commands and specialized roles.

### Horizontal Intelligence vs. Vertical Intelligence

To understand how Spec-Kit Plus works, you need to distinguish two types of organizational intelligence:

**Horizontal Intelligence: Knowledge Across Time**

Horizontal intelligence captures decisions and learnings in a permanent, searchable form so future you (or future team members) can learn from past work.

- **ADRs** (Architectural Decision Records): Document the "why" behind significant decisions
  - Example: "Why did we choose error codes over exceptions? Because..."
  - Stored in: `history/adr/`
  - Created explicitly via `/sp.adr <title>` when architectural decisions are made

- **PHRs** (Prompt History Records): Automatically capture AI collaboration sessions
  - Example: "AI suggested this pattern for error handling. We chose it because..."
  - Stored in: `history/prompts/<feature>/`
  - Created automatically (you don't manually run a PHR command)

**Vertical Intelligence: Knowledge Through Hierarchy**

You can optionally build Vertical intelligence at start of each project. This is like onboarding the specialized skilled workers in your team. It is how YOU work with AI orchestrators and specialized subagents:

```
┌─────────────────────────────────────────────────────────┐
│  👤 YOU (Architect/Validator)                           │
│  Strategic Decisions & Quality Control                  │
│                                                          │
│  "What to build?" ──────────────┐                       │
│                                  ↓                       │
│                   ┌──────────────────────────┐          │
│                   │  🤖 AI ORCHESTRATOR      │          │
│                   │  Main Collaborator       │          │
│                   │  Routes work to experts  │          │
│                   └──────────────────────────┘          │
│                            ↓                             │
│         ┌──────────────────┼──────────────────┐         │
│         ↓                  ↓                  ↓          │
│  ┌────────────┐   ┌────────────┐   ┌────────────┐      │
│  │ 📝 Spec    │   │ 🏗️ Plan    │   │ ⚙️ Impl    │      │
│  │ Subagent   │   │ Subagent   │   │ Subagent   │      │
│  │            │   │            │   │            │      │
│  │ Writes     │   │ Creates    │   │ Generates  │      │
│  │ clear      │   │ plans      │   │ code +     │      │
│  │ specs      │   │            │   │ tests      │      │
│  └────────────┘   └────────────┘   └────────────┘      │
│         │                  │                  │          │
│         └──────────────────┼──────────────────┘         │
│                            ↓                             │
│                   ┌────────────────┐                     │
│                   │ ✅ Validation  │                     │
│                   │ Subagent       │                     │
│                   │ Reviews quality│                     │
│                   └────────────────┘                     │
│                            │                             │
│  ← Review & Approve ───────┘                            │
└─────────────────────────────────────────────────────────┘
```

**How Vertical Intelligence Works**:

1. **You describe intent** - "Build a calculator with 5 operations"
2. **Orchestrator delegates** - Routes to appropriate subagent (e.g., Specification Subagent for writing specs)
3. **Subagent executes** - Specification Subagent asks clarifying questions, identifies gaps, returns complete spec
4. **You validate** - Review spec and approve (or iterate)
5. **Orchestrator delegates next phase** - Routes to Planning Subagent
6. **Cycle repeats** through Plan → Tasks → Implementation

**Why This Matters**: You don't need to memorize specification templates, planning methodologies, or code patterns. The orchestrator knows which expert to consult for which task. Your job is **thinking clearly about intent and validating results**, not memorizing frameworks.

### Why Spec-Kit Plus Exists (Problem It Solves)

Chapter 30 introduced you to four SDD approaches:

- **Kiro** - Extremely simple (just folders, no automation)
- **GitHub Spec-Kit** - GitHub-based workflow with templates
- **Spec-Kit Plus** (Panaversity's approach) - Templates + ADRs + PHRs + Vertical Intelligence
- **Tessel** - Code-generation focused (spec as source of truth)

We chose Spec-Kit Plus for this book because:

- **Opinionated workflow**: Enforces Spec → Clarify → Plan → Tasks → Implement sequence (the cascade)
- **Knowledge capture**: ADRs preserve "why" decisions; PHRs capture AI collaboration history
- **Vertical Intelligence**: Orchestrator + subagents = efficient, scalable workflow
- **Flexible tooling**: Works with Claude Code, Gemini CLI, or any capable AI tool
- **Proven in practice**: Used by teams at Anthropic, Google, and OpenAI

---

## Part B: Install Spec-Kit Plus Framework

Now let's install the actual Spec-Kit Plus framework. This is independent of your AI tool choice.

### Installation Steps

**Step 1: Verify Python Version**

Spec-Kit Plus requires Python 3.12 or higher. Check your version first:

```bash
# Check Python version (must be 3.12+)
python --version

# If you see Python 3.11 or lower, upgrade Python first:
# - macOS: brew install python@3.12
# - Ubuntu: sudo apt install python3.12
# - Windows: Download from python.org
```

**Expected Output:**
```
Python 3.12.0  ✓ (or higher)
Python 3.11.5  ✗ (too old - upgrade needed)
```

**Step 2: Install Spec-Kit Plus**

With Python 3.12+ confirmed, install Spec-Kit Plus:

```bash
# Install the latest version
pip install specifyplus

# Verify installation
specifyplus --version
```

**Step 3: Initialize Your First Project**

```bash
# Create a new Spec-Kit Plus project
specifyplus init calculator-project
```

**Interactive Prompts:**

During initialization, you'll see these prompts:

```
? Select AI Tool:
  > Claude Code
    Gemini CLI

? Select Terminal:
  > bash
    powershell (Windows only)
```

**Recommendations:**
- **AI Tool**: Choose **Claude Code** (recommended for this book)
- **Terminal**: Choose **bash** (or powershell if on Windows without WSL)

**Step 4: Navigate to the project**
```bash
cd calculator-project
```

**Step 5: Verify Project Structure**

After initialization, you should see the following directory structure:

```
calculator-project/
├── .claude/
│   └── commands/                    # Slash commands for SDD workflow
│       ├── sp.adr.md                # Document architectural decisions
│       ├── sp.analyze.md            # Cross-artifact consistency checks
│       ├── sp.checklist.md          # Generate custom checklists
│       ├── sp.clarify.md            # Refine specifications
│       ├── sp.constitution.md       # Create project constitution
│       ├── sp.git.commit_pr.md      # Commit and create PRs
│       ├── sp.implement.md          # Generate code from tasks
│       ├── sp.phr.md                # Record prompt history
│       ├── sp.plan.md               # Generate implementation plans
│       ├── sp.specify.md            # Create specifications
│       └── sp.tasks.md              # Break plans into atomic tasks
│
├── .specify/
│   ├── memory/
│   │   └── constitution.md          # Project-wide rules and principles
│   │
│   ├── scripts/
│   │   └── bash/                    # Automation scripts
│   │       ├── check-prerequisites.sh
│   │       ├── common.sh
│   │       ├── create-adr.sh
│   │       ├── create-new-feature.sh
│   │       ├── create-phr.sh
│   │       ├── setup-plan.sh
│   │       └── update-agent-context.sh
│   │
│   └── templates/                   # Templates for specs, plans, tasks, ADRs, PHRs
│       ├── adr-template.md
│       ├── agent-file-template.md
│       ├── checklist-template.md
│       ├── phr-template.prompt.md
│       ├── plan-template.md
│       ├── spec-template.md
│       └── tasks-template.md
│
├── .git/                            # Git repository
├── CLAUDE.md                        # Agent instructions and guidelines
├── README.md                        # Project documentation
└── .gitignore                       # Git ignore rules
```

**Note**: The `specs/`, `history/prompts/`, and `history/adr/` directories will be created automatically when you start your first feature.

**Explanation of Key Directories**:

- **`.claude/commands/`** - Slash commands you'll use throughout the SDD workflow (/sp.specify, /sp.plan, etc.)
- **`.specify/memory/`** - Your project constitution (created once, referenced always)
- **`.specify/scripts/`** - Automation scripts for PHRs, ADRs, and feature setup
- **`.specify/templates/`** - Templates that guide spec, plan, task, ADR, and PHR creation
- **`CLAUDE.md`** - Agent instructions that guide your AI collaborator's behavior
- **`specs/`** - (Created later) Your feature specifications
- **`history/`** - (Created later) ADRs and PHRs for knowledge capture

---

## Part C: Verify Commands Work 

Now let's test that everything is connected.

### Test 1: Access Spec-Kit Plus Commands

Open Claude Code (or your chosen AI tool) in the `calculator-project` directory:

```bash
# In your terminal, from calculator-project directory
# Launch Claude Code interface
claude

# OR GEMINI
gemini
```

Inside Terminal, verify Spec-Kit Plus commands are available:

```
# Type
/sp.
```

You should see core Spec-Kit Plus commands:
- `/sp.constitution` - Build your constitution
- `/sp.specify` - Launch specification workflow
- `/sp.clarify` - Refine and validate specs
- `/sp.plan` - Generate implementation plan
- `/sp.adr` - Document architectural decisions
- `/sp.tasks` - Decompose plan into tasks
- `/sp.implement` - Generate code
- `/sp.phr` - Record prompt history

If the command is recognized, your orchestrator is configured correctly.

---

## Common Mistakes

### Mistake 1: Confusing Spec-Kit Plus with Claude Code

**The Error**: "I installed Claude Code, so I have Spec-Kit Plus now."

**Why It's Wrong**: Spec-Kit Plus is a separate framework. Claude Code is just the AI tool that executes Spec-Kit Plus commands.

**The Fix**: Install both: `pip install specifyplus` (framework) AND configure Claude Code/Gemini CLI (AI tool).

### Mistake 2: Skipping Project Initialization

**The Error**: Creating folders manually instead of running `specifyplus init`

**Why It's Wrong**: You miss critical infrastructure (.specify/ templates, configuration files, directory structure).

**The Fix**: Always run `specifyplus init <project-name>` to set up proper structure.

---

## Try With AI: Verify Your Complete Setup

Now let's use your newly configured Spec-Kit Plus to run a real test. This activity consolidates your learning about Spec-Kit Plus architecture and validates that everything is working.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)

**Context**: Your calculator-project directory with all infrastructure in place

:::tip ⚠️ Learning WITH AI (Not Generating FROM AI)

**What this exercise teaches:**
- ❌ **DON'T ask**: "Write this code for me"
- ❌ **DON'T ask**: "Generate the implementation"
- ✅ **DO ask**: "Explain why Spec-Kit Plus is separate from Claude Code"
- ✅ **DO ask**: "What's the difference between ADRs and PHRs?"

**Your role**: Understand concepts, validate setup, ask clarifying questions
**AI's role**: Explain architecture, verify configuration, answer questions

:::

### Prompt Set (Copy-Paste Ready)

**Prompt 1 - Framework Verification**

Copy and paste this into Claude Code:

```
I've installed Spec-Kit Plus and set up my calculator-project. Let me verify
the setup is correct by asking about the core concepts:

1. What is Spec-Kit Plus? (In one sentence, distinguish it from Claude Code)
2. What are ADRs and PHRs, and how do they differ?

Then, tell me: Am I ready to write my first specification, or do I need to
do anything else?
```

**Prompt 2 - Command Verification**

After you receive the response, ask:

```
Thanks for confirming. Now can you tell me:
1. What are the 7 main Spec-Kit Plus workflow commands?
   (Hint: /sp.specify, /sp.clarify, /sp.plan, /sp.adr, /sp.tasks, /sp.implement, /sp.phr)
2. In what order should I use them (from specification through implementation)?
3. Which commands are explicit (I run them) vs automatic (system runs them)?
```

**Prompt 3 - Architecture Confirmation**

Finally, ask:

```
One more question to confirm my mental model: In the Vertical Intelligence
architecture (You → Orchestrator → Subagents), what is MY job at each phase?

- Specification phase: What do I do?
- Planning phase: What do I do?
- Implementation phase: What do I do?

(I'm trying to understand that I'm architect/validator, not coder)
```

<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />


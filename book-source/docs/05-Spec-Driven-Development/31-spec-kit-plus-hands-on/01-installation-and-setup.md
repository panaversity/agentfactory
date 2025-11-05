---
title: "Installation & Setup — AI-Native SDD Toolkit"
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
  extension_for_advanced: "Explore multi-tool setup—configure both Claude Code and Gemini CLI; compare workflows between the two"
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

# Installation & Setup — AI-Native SDD Toolkit

Welcome to hands-on Spec-Kit Plus development! Chapter 30 taught you **why** specification-driven development matters. This chapter teaches you **how** to do it—by building a real calculator project from specification to working code using Spec-Kit Plus and your AI companion.

This lesson gets your toolkit ready. By the end, you'll have Spec-Kit Plus installed, your AI tool configured, and a test project initialized. More importantly, you'll understand the architecture behind Spec-Kit Plus—why it's independent of any single AI tool, and how the three-tier Vertical Intelligence pattern (You → Orchestrator → Specialized Subagents) enables efficient workflow management.

---

## Part A: What Is Spec-Kit Plus? 

Before installing anything, let's clarify what Spec-Kit Plus actually is.

### The Architecture: Three Independent Layers

Spec-Kit Plus is an Opiniated ToolKit for SDD. It have three independent but integrated components:

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
You (Architect/Validator)
  ↓
AI Orchestrator (Main Collaborator)
  ├─ Specification Subagent
  ├─ Planning Subagent
  ├─ Implementation Subagent
  └─ Validation Subagent
```

**How Vertical Intelligence Works**:

1. **You describe intent** — "Build a calculator with 5 operations"
2. **Orchestrator delegates** — Routes to appropriate subagent (e.g., Specification Subagent for writing specs)
3. **Subagent executes** — Specification Subagent asks clarifying questions, identifies gaps, returns complete spec
4. **You validate** — Review spec and approve (or iterate)
5. **Orchestrator delegates next phase** — Routes to Planning Subagent
6. **Cycle repeats** through Plan → Tasks → Implementation

**Why This Matters**: You don't need to memorize specification templates, planning methodologies, or code patterns. The orchestrator knows which expert to consult for which task. Your job is **thinking clearly about intent and validating results**, not memorizing frameworks.

### Why Spec-Kit Plus Exists (Problem It Solves)

Chapter 30 introduced you to four SDD approaches:

- **Kiro** — Extremely simple (just folders, no automation)
- **GitHub Spec-Kit** — GitHub-based workflow with templates
- **Spec-Kit Plus** (Panaversity's approach) — Templates + ADRs + PHRs + Vertical Intelligence
- **Tessel** — Code-generation focused (spec as source of truth)

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

**Step 1: Install Spec-Kit Plus**

Spec-Kit Plus is provided as a Python package:

```bash
# Install the latest version
pip install specifyplus

# Verify installation
specifyplus --version
```

**Step 2: Initialize Your First Project**

```bash
# Create a new Spec-Kit Plus project
specifyplus init calculator-project
```

It will prompt to Select AI Tool and Terminal. You can choose between **Claude Code** or **Gemini CLI**. For terminal prefer bash - if you are on windows without wsl there is option to use powershell.

**Step 3: Navigate to the project**
```bash
cd calculator-project
```
**Step 4: Verify Project Structure**

After initialization, you should see:

```
calculator-project/
├── .specify/
│   ├── memory/
│   │   └── constitution.md          # Project-wide rules
│   ├── templates/
│   └── scripts/
├── specs/                           # Specification Artifacts
├── history/
│   ├── adr/                         # Architectural Decision Records
│   └── prompts/                     # Prompt History Records (AI collaboration logs)
├── README.md
└── .gitignore
```

**Explanation of Directories**:

- **`.specify/`** — Framework infrastructure (read-only templates and tools)
- **`specs/`** — Your specification artifacts (what you write)
- **`history/`** — Knowledge capture (ADRs and PHRs for traceability)

Note: The `specs/` and `history/` directories will appear when we create our first specification.

---

For the rest of this chapter, all examples will show **Claude Code**. If you're using Gemini CLI or any other AI tool, the commands are identical.

---

## Part D: Verify Commands Work 

Now let's test that everything is connected.

### Test 1: Access Spec-Kit Plus Commands

Open Claude Code (or your chosen AI tool) in the `calculator-project` directory:

```bash
# In your terminal, from calculator-project directory
# Launch Claude Code interface
claude

# OR
gemini
```

Inside Terminal, verify Spec-Kit Plus commands are available:

```
# Type
/sp.
```

You should see core Spec-Kit Plus commands:
- `/sp.constitution` — Build your constitution
- `/sp.specify` — Launch specification workflow
- `/sp.clarify` — Refine and validate specs
- `/sp.plan` — Generate implementation plan
- `/sp.adr` — Document architectural decisions
- `/sp.tasks` — Decompose plan into tasks
- `/sp.implement` — Generate code
- `/sp.phr` — Record prompt history

If the command is recognized, your orchestrator is configured correctly.

---

## Try With AI: Verify Your Complete Setup

Now let's use your newly configured Spec-Kit Plus to run a real test. This activity consolidates your learning about Spec-Kit Plus architecture and validates that everything is working.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)

**Context**: Your calculator-project directory with all infrastructure in place

### Prompt Set (Copy-Paste Ready)

**Prompt 1 — Framework Verification**

Copy and paste this into Claude Code:

```
I've installed Spec-Kit Plus and set up my calculator-project. Let me verify
the setup is correct by asking about the core concepts:

1. What is Spec-Kit Plus? (In one sentence, distinguish it from Claude Code)
2. What are ADRs and PHRs, and how do they differ?

Then, tell me: Am I ready to write my first specification, or do I need to
do anything else?
```

**Prompt 2 — Command Verification**

After you receive the response, ask:

```
Thanks for confirming. Now can you tell me:
1. What are the 7 main Spec-Kit Plus workflow commands?
   (Hint: /sp.specify, /sp.clarify, /sp.plan, /sp.adr, /sp.tasks, /sp.implement, /sp.phr)
2. In what order should I use them (from specification through implementation)?
3. Which commands are explicit (I run them) vs automatic (system runs them)?
```

**Prompt 3 — Architecture Confirmation**

Finally, ask:

```
One more question to confirm my mental model: In the Vertical Intelligence
architecture (You → Orchestrator → Subagents), what is MY job at each phase?

- Specification phase: What do I do?
- Planning phase: What do I do?
- Implementation phase: What do I do?

(I'm trying to understand that I'm architect/validator, not coder)
```

### Expected Outcomes

After these prompts, you should understand:

✅ **Spec-Kit Plus is a methodology framework**, independent of any AI tool
✅ **ADRs** (explicit, long-term decisions) and **PHRs** (automatic, collaboration logs) capture knowledge
✅ **Your role is intent + validation**, not implementation
✅ **Six commands form the workflow**: specify → clarify → plan → adr → tasks → implement
✅ **You're ready for Lesson 2** (Constitution creation) with confidence

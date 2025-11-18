---
title: "Installation & Setup - Getting Spec-Kit Plus Running"
chapter: 31
lesson: 2
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
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

  - name: "Installation Verification"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can verify Spec-Kit Plus installation and run test commands successfully"

learning_objectives:
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
  new_concepts: 3
  assessment: "3 new concepts (Tool configuration, Project structure navigation, Installation verification) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore multi-tool setup-configure both Claude Code and Gemini CLI; compare workflows between the two"
  remedial_for_struggling: "Step-by-step guided installation with video/visual aids; installation verification checklist"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-18"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "2.0.0"
---

# Installation & Setup - Getting Spec-Kit Plus Running

In Lesson 1, you learned WHAT Spec-Kit Plus is (three-tier architecture, Horizontal/Vertical Intelligence) and WHY this book uses it (implements SDD-RI concepts). Now you'll install the framework and verify everything works.

By the end of this lesson, you'll have Spec-Kit Plus installed, your AI tool configured, and a test project initialized. You'll execute your first slash command and confirm the orchestrator can route work to specialized subagents.

---

## Install Spec-Kit Plus Framework

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

## Verify Commands Work

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

#### 🤝 Practice Exercise

> **Ask your AI**: "I've just installed Spec-Kit Plus in my calculator-project directory. Can you verify my project structure by listing what should be in `.specify/templates/` and `.claude/commands/`? Then explain what each directory's purpose is."

**Expected Outcome**: Your AI companion should confirm the directory structure matches the Spec-Kit Plus installation, explain that templates guide spec/plan/task creation, and clarify that commands are the slash commands you'll use throughout the workflow.

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

---
title: "Hooks and Extensibility"
sidebar_position: 7
chapter: 5
lesson: 7
duration_minutes: 6

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 2"
layer_progression: "L2 (AI Collaboration)"
layer_1_foundation: "N/A"
layer_2_collaboration: "Co-designing hooks for specific workflows (Step 5), AI as Teacher suggesting hook patterns, Student as Teacher providing workflow context, AI as Co-Worker refining hook commands"
layer_3_intelligence: "N/A"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA
skills:
  - name: "Implementing Event-Driven Automation with Hooks"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can understand hook event types, create SessionStart hooks in settings.json, recognize automation opportunities, and co-design hooks through AI collaboration"

learning_objectives:
  - objective: "Understand hooks as event-triggered automation"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of hook architecture and event-trigger-action pattern"
  - objective: "Create a simple SessionStart hook"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Creation of functional SessionStart hook that displays project context"
  - objective: "Recognize common hook events (PreToolUse, PostToolUse, SessionStart, SessionEnd)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Identification of appropriate hook events for different automation scenarios"
  - objective: "Test hooks in a real Claude Code session"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Verification that hook executes correctly during session start"

# Cognitive load tracking
cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (hooks definition, event types [PreToolUse/PostToolUse/SessionStart/SessionEnd], automation patterns, settings.json configuration, Three Roles co-design) - within B1 limit of 10 ✓"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Create PostToolUse hooks for code formatting and test automation; design hook chains that trigger sequentially"
  remedial_for_struggling: "Start with simple SessionStart echo message before adding project context; understand conceptually before implementing"

# Generation metadata
generated_by: "content-implementer v1.0.0 (029-chapter-5-refinement)"
source_spec: "specs/029-chapter-5-refinement/spec.md"
created: "2025-01-17"
last_modified: "2025-01-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"

# Legacy compatibility
prerequisites:
  - "Lessons 2-6: Claude Code, CLAUDE.md, MCP, Subagents, Skills"
  - "Understanding of event-driven automation"
---

# Hooks and Extensibility

You're working on a project. Every time you start a new Claude Code session, you manually:
1. Explain the project structure
2. Remind Claude about your naming conventions
3. Load environment variables
4. Set up project context

By the third session today, you're frustrated. **Why can't Claude Code just do this automatically when a session starts?**

**That's what hooks solve.**

---

## What Are Hooks?

**Definition**: Hooks are automated scripts that run when specific events occur in Claude Code—like when a session starts, a file is edited, or a tool runs.

**Think of hooks as**: Event listeners that trigger actions automatically.

A hook has three parts:
1. **Event**: What triggers the hook (like "SessionStart")
2. **Condition** (optional): Which tools or files match
3. **Action**: What command runs automatically

**Key benefit**: Automate repetitive tasks so you focus on creative work, not setup.

#### 💬 AI Colearning Prompt
> "Explain what hooks are in Claude Code. Give 2-3 real-world examples where a hook would save time by automating a repetitive task."

---

## Hook Events: When Hooks Trigger

Claude Code recognizes four main event types that can trigger hooks:

### PreToolUse
Fires **before** Claude Code runs a command or tool.

**Example use case**: Validation hook that checks requirements before running a build script.

### PostToolUse
Fires **after** Claude Code completes a command or tool.

**Example use case**: Format code immediately after edit, or run tests after saving.

### SessionStart
Fires **when you open a Claude Code session**.

**Example use case**: Load environment variables, initialize project context, or run startup checks.

### SessionEnd
Fires **when you close a Claude Code session**.

**Example use case**: Cleanup tasks, save session logs, or sync project state.

#### 🎓 Expert Insight
> In AI-driven development, hooks automate the routine follow-ups that developers used to do manually. Your value isn't in running `npm test` after every edit—it's in understanding WHEN testing matters and what the results tell you about your design.

---

## Real-World Hook Examples

Here are three practical scenarios showing hooks in action:

| **Scenario** | **Hook Type** | **Event Trigger** | **Action** | **Benefit** |
|---|---|---|---|---|
| **Code Formatting** | PostToolUse | After Claude edits a `.py` file | Run `black --line-length 88 file.py` | Consistent code style without manual intervention |
| **Test Validation** | PostToolUse | After any changes to `src/` directory | Run `pytest` and report results | Catch bugs immediately instead of later |
| **Environment Setup** | SessionStart | When new Claude Code session opens | Load variables from `.env` and run `source setup.sh` | Project context always ready without manual setup |

Each hook saves time by automating what you would otherwise do manually after specific events.

#### 💬 AI Colearning Prompt
> "Explain the difference between PreToolUse and PostToolUse hooks. Give 2 examples where each would be more appropriate than the other."

---

## Why Hooks Matter for Professional Workflows

Imagine you're part of a development team. Without hooks:
- After every edit, someone manually runs formatters: **repetitive, error-prone**
- Tests run on a schedule instead of immediately: **bugs discovered late**
- Setup requires manual steps: **onboarding takes longer**

With hooks:
- Code automatically formatted **immediately after edit**
- Tests run **automatically whenever code changes**
- Project environment **auto-loads on session start**

Hooks are about **automating the predictable parts of your workflow** so you focus on the strategic thinking—the parts only humans can do.

#### 🤝 Practice Exercise

> **Think of a repetitive task** you've done recently (running tests, formatting code, deploying, checking linting). **Ask your AI**: "In Claude Code, how would a hook help automate this repetitive task? What event would trigger it?"

**Expected Outcome**: You'll understand how hooks map to your real workflows and why automation matters for productivity.

---

## Hands-On: Create Your First Hook

Let's create a simple SessionStart hook that displays a welcome message when you open Claude Code.

### Step 1: Create Settings File

Hooks are configured in `.claude/settings.json`. Create this file in your project:

```bash
mkdir -p .claude
touch .claude/settings.json
```

### Step 2: Add a SessionStart Hook

Edit `.claude/settings.json` and add:

```json
{
  "hooks": [
    {
      "event": "SessionStart",
      "type": "command",
      "command": "echo 'Welcome to your project! Claude Code session started.'"
    }
  ]
}
```

**What this does**:
- **event**: Triggers when session starts
- **type**: "command" means run a bash command
- **command**: The actual command to run (echo a welcome message)

### Step 3: Test Your Hook

Close and restart Claude Code in this project:

```bash
exit  # if already in a session
claude
```

**What you should see**:
```
Welcome to your project! Claude Code session started.
```

The hook ran automatically when the session started!

### Step 4: Make It More Useful

Update `.claude/settings.json` to show project info:

```json
{
  "hooks": [
    {
      "event": "SessionStart",
      "type": "command",
      "command": "echo 'Project: $(basename $(pwd)) | Files: $(ls -1 | wc -l) | Last modified: $(ls -lt | head -2 | tail -1 | awk \"{print \\$6, \\$7, \\$8}\")'"
    }
  ]
}
```

Restart Claude Code:

```bash
exit
claude
```

**Now you see**:
```
Project: my-project | Files: 15 | Last modified: Nov 14 10:23
```

Useful project context **automatically** every session!

### Step 5: Co-Design a Hook for Your Workflow (Optional but Recommended)

Now that you understand hooks, let's collaborate with Claude Code to design one for YOUR specific needs.

**🤝 Practice Exercise: Three Roles Co-Design**

Ask Claude Code:

```
"I want to create a hook that automates [YOUR SPECIFIC TASK].
What event should trigger it? What command should run?
What should I watch out for?"
```

**What happens in this collaboration**:
1. **AI as Teacher**: Claude suggests appropriate hook design patterns you might not have considered
2. **AI as Student**: You provide context about your specific workflow that Claude doesn't know
3. **AI as Co-Worker**: Together you refine the design, converging on a hook that actually solves your problem

This is the Three Roles Framework in action—not just "asking AI to do something," but genuine collaboration where both you and Claude Code learn from each other.

**Try it now** with a real automation you need in your current project.

---

## Common Questions About Hooks

### "Aren't hooks the same as Skills or Plugins?"

No—each serves a different purpose:

- **Hooks**: Automate Claude Code's behavior in response to events (like "format code after edit")
- **Skills**: Extend Claude's knowledge and capabilities with new functions or custom commands (like "extract PDF forms")
- **MCP**: Connect external tools to Claude Code (like "access GitHub repositories")
- **Plugins**: Bundle all of the above together as complete customizations (marketplace packages)

Think of it this way: Skills and MCP add **new capabilities**. Hooks add **automation**. They're complementary, not overlapping.

### "Will I encounter hook errors?"

Possibly, but they're usually non-blocking. If a hook fails:
- Claude Code continues working (hook errors don't stop your session)
- The error is logged, not shown as a blocker
- You can investigate later

Debugging hooks is advanced content (Part 5). For now, just know hook errors won't stop you from working.

### "Should I enable hooks right now?"

Not necessary. Claude Code works perfectly fine without custom hooks. You can:
- Use Claude Code productively without any hooks
- Learn about hooks conceptually now
- Build hooks later when you're ready to optimize your workflow

**No pressure to customize right now—focus on mastering basic workflows first.**

---

## Try With AI

Open Claude Code and explore hooks conceptually with your AI companion.

### Prompt 1: Understand Hooks

```
claude "Explain hooks in Claude Code. Give 2-3 useful examples where a hook would save time by automating a repetitive task."
```

**Expected outcome**: AI explains hooks as event-triggered automation and provides concrete examples (format code, run tests, load environment).

### Prompt 2: Identify Opportunities

Think of a task you do repeatedly in your projects (running tests, formatting code, checking linting, deploying, loading configuration).

```
claude "I frequently [describe your repetitive task]. Could a Claude Code hook help automate this? If so, what would it do?"
```

**Expected outcome**: AI identifies how a hook could eliminate the repetitive step, discusses which event would trigger it, and explains the time savings.

### Prompt 3: Future Learning Path

```
claude "Can I build custom hooks in Claude Code? How hard is it? When would I learn this? What are the prerequisites?"
```

**Expected outcome**: AI clarifies that hook building is intermediate-to-advanced content, explains prerequisites (Part 5), and positions hooks in the larger Claude Code customization landscape.

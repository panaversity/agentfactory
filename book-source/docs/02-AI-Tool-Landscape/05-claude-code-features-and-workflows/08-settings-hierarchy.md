---
title: "Settings Hierarchy"
sidebar_position: 8
chapter: 5
lesson: 8
duration_minutes: 5

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 3"
layer_progression: "L3 (Intelligence Design - understanding configuration as organizational pattern)"
layer_1_foundation: "N/A"
layer_2_collaboration: "AI explains settings hierarchy conceptually, student verifies which settings exist on their system"
layer_3_intelligence: "Understanding three-level hierarchy as reusable organizational intelligence pattern (user/project/local), recognizing when to apply each level for team collaboration"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Configuration Precedence and Hierarchy"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain three-level settings hierarchy, identify precedence order (local > project > user), verify which settings files exist, and determine appropriate level for different configuration scenarios"

learning_objectives:
  - objective: "Understand Claude Code's three-level settings hierarchy"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of user/project/local levels and their scope differences"
  - objective: "Recognize precedence order: local > project > user"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Correct prediction of which setting wins in multi-level conflict scenarios"
  - objective: "Check which settings files exist on your system"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Verification of settings file existence using ls commands"
  - objective: "Know when to use each settings level"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Decision analysis matching configuration scenarios to appropriate hierarchy level"

# Cognitive load tracking
cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (three-level hierarchy, user/project/local scopes, precedence order, organizational intelligence pattern, team vs personal settings) - within B1 limit of 10 ✓"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Design multi-team configuration strategies with shared project standards and personal customizations; understand .gitignore patterns for settings.local.json"
  remedial_for_struggling: "Focus on visual hierarchy diagram; understand user-level first before project and local complexity"

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
  - "Lessons 2-7: All prior Claude Code features understood"
  - "Understanding of configuration management and team coordination"
---

# Settings Hierarchy

## What Are Settings?

Claude Code has a **settings system** that lets you customize how it behaves. These settings control things like:
- Permission modes (whether Claude asks before edits)
- Output preferences (how Claude formats responses)
- Project-specific defaults (which tools Claude prioritizes)
- Team standards (shared rules for collaborative work)

Instead of having one global settings file, Claude Code uses a **three-level hierarchy**. This design lets you have **personal preferences, project standards, and temporary overrides** all at the same time.

#### 💬 AI Colearning Prompt

> "Explain why applications use hierarchical configuration systems instead of a single global settings file. What problems does hierarchy solve?"

---

## The Three Settings Levels

Claude Code settings exist at three levels, from general to specific:

### Level 1: User Settings (Most General)

**Location**: `~/.claude/settings.json`

**Scope**: Applies to **all your Claude Code projects** on your machine

**Applies to**: Every project you work on, across your entire system

**When to use this level**:
- Your personal preferences (always use dark mode, prefer verbose output)
- Your coding style (consistent formatting choices)
- Your workflow defaults (always prefer plan mode for safety)

**Example content**:
```json
{
  "model": "claude-sonnet-4-5-20250929",
  "outputStyle": "Concise",
  "includeCoAuthoredBy": true
}
```

### Level 2: Project Settings (Middle)

**Location**: `.claude/settings.json` (inside your project directory)

**Scope**: Applies to **just this project**

**Applies to**: Only when you're working in this specific project

**When to use this level**:
- Team standards (your team agrees on permission settings)
- Project-specific customizations (this project uses a different framework)
- Temporary standards (during alpha testing, use stricter permissions)

**Example content**:
```json
{
  "permissions": {
    "defaultMode": "acceptEdits",
    "allow": ["Bash(npm run test:*)"],
    "deny": ["Read(./.env)"]
  },
  "env": {
    "PROJECT_ENV": "development"
  }
}
```

### Level 3: Local Settings (Most Specific)

**Location**: `.claude/settings.local.json` (inside your project directory)

**Scope**: Applies to **this project only, on your machine only**

**Applies to**: Just your local work in this project (not shared with team)

**When to use this level**:
- Temporary overrides (you need different settings just for today)
- Personal experiments (testing a new workflow locally)
- Machine-specific settings (your laptop needs different settings than your desktop)

#### 💬 AI Colearning Prompt
> "Why would you use three levels of settings (user/project/local) instead of just one global settings file? What flexibility does this hierarchy provide?"

**Example content**:
```json
{
  "outputStyle": "Verbose",
  "sandbox": {
    "enabled": true
  }
}
```

---

## Why Settings Hierarchy Matters for Team Intelligence

**Team Collaboration Without Conflicts**: Settings hierarchy enables teams to share standards (user-level defaults) while allowing personal customization (project-level overrides) and local experimentation (local-level testing)—all without stepping on each other's toes.

**Organizational Intelligence**: Think of this hierarchy as three layers of context:
- **User settings**: Your personal AI preferences that follow you across all projects
- **Project settings**: Team-agreed standards that everyone on the project shares (shared via git)
- **Local settings**: Your private testing and experiments (gitignored, never committed)

This connects to earlier lessons:
- **CLAUDE.md (Lesson 3)** provides content context at the project level
- **Skills (Lesson 6)** can be enabled at any hierarchy level
- **Plugins (Lesson 9)** will use this same hierarchy to configure bundled capabilities

**Real-World Impact**: Without this hierarchy, teams either enforce rigid standards (no personal customization) or descend into chaos (everyone's setup is different). The three-level system gives you both consistency AND flexibility.

#### 🎓 Expert Insight
> In AI-native development, configuration hierarchy mirrors organizational intelligence. User settings = your personal defaults. Project settings = team agreements. Local settings = safe experimentation space. Understanding WHEN to configure at each level is more valuable than memorizing settings syntax.

---

## Precedence: Which Settings Win?

When the same setting exists at multiple levels, Claude Code follows this **precedence order** (most specific wins):

**Local > Project > User**

This means:
- **Local settings** override both project and user settings
- **Project settings** override user settings
- **User settings** are the fallback when nothing more specific exists

### Visual Hierarchy

```
┌─────────────────────────────────┐
│   LOCAL SETTINGS                │
│   .claude/settings.local.json    │  ← Most Specific (Highest Priority)
│   (just your machine, temporary) │
└─────────────────┬───────────────┘
                  ↑ Overrides
┌─────────────────────────────────┐
│   PROJECT SETTINGS              │
│   .claude/settings.json          │  ← Team/Project Level
│   (shared with team)             │
└─────────────────┬───────────────┘
                  ↑ Overrides
┌─────────────────────────────────┐
│   USER SETTINGS                 │
│   ~/.claude/settings.json        │  ← Most General (Fallback)
│   (all projects on this machine) │
└─────────────────────────────────┘
```

---

## Example: Settings Precedence in Action

Let's say you have:

**User level** (`~/.claude/settings.json`):
```json
{
  "outputStyle": "Concise"
}
```

**Project level** (`.claude/settings.json` in your project):
```json
{
  "outputStyle": "Explanatory"
}
```

**Local level** (`.claude/settings.local.json` in your project):
```json
{
  // Empty or not set
}
```

**Result**: Claude Code uses `outputStyle: "Explanatory"` (from project level, since it overrides user level)

---

### What If Local Level Is Set?

Now you add a temporary local override:

**Local level** (`.claude/settings.local.json`):
```json
{
  "outputStyle": "Verbose"
}
```

**New Result**: Claude Code uses `outputStyle: "Verbose"` (from local level, which overrides both project and user)

**Why this matters**: You can temporarily change your workflow for this one session without affecting your project's standards or your personal preferences. Tomorrow, when you delete the local settings file, you're back to `"Explanatory"` (project level).

#### 🤝 Practice Exercise

> **Ask your AI**: "I have outputStyle set to 'Concise' at user level and 'Explanatory' at project level. I'm working in this project. Which style is active? If I create a .claude/settings.local.json file with outputStyle: 'Verbose', what happens?"

**Expected Outcome**: AI explains that project level is active (Explanatory), and creating a local override would switch to Verbose—helping you understand how to temporarily override settings without changing team standards.

---

## Why Settings Hierarchy Matters

### For Team Collaboration

Your team might decide: "All projects should deny access to .env files for security." They set `permissions.deny: ["Read(./.env)"]` at the **project level** (`.claude/settings.json`). Everyone on the team gets this standard automatically.

But you, personally, have different permission settings at your **user level**. Project settings win, so when working on this project, everyone (including you) uses the team's security standards. When you work on personal projects, your user-level preferences kick in again.

### For Personal Customization

You want special settings just for this project—maybe more verbose output or a specific framework preference. You set `.claude/settings.json` (project level) without touching user settings. Your other projects still use your personal defaults (user level).

### For Temporary Experiments

You want to test a new workflow today without affecting the team. You create `.claude/settings.local.json` (local level). Your changes stay on your machine, invisible to the team. Delete the file tomorrow, and everything reverts to project/user standards.

---

## The .claude/ Directory: Don't Delete It

You might see a `.claude/` directory in your project and wonder: "Is this important? Can I delete it?"

**Short answer**: Don't delete it.

**What it contains**:
- `settings.json` — Project-level settings
- `settings.local.json` — Your local, temporary overrides
- Other configuration files Claude Code needs

The `.claude/` directory is how Claude Code stores project customization. Deleting it would reset all your project settings to defaults.

**What you should do**: Treat `.claude/settings.json` like your `.gitignore` or `package.json`—it's part of your project configuration. Include it in version control (share with team). But `.claude/settings.local.json` should probably be in your `.gitignore` (keep it personal).

#### 💬 AI Colearning Prompt

> "Explain the difference between .claude/settings.json and .claude/settings.local.json. Which one should be in .gitignore? Why?"

---

## Not Configuring Yet—This Is Part 5 Content

This lesson teaches you that **settings exist and how the hierarchy works**. You don't need to configure them yet. Basic Claude Code usage works perfectly fine with defaults.

**Detailed settings configuration** (what specific settings do, how to change them, team policies) is **Part 5 content** (Spec-Driven Development, team workflows). For now, just know:

✅ Settings exist at three levels
✅ Precedence is: local > project > user
✅ This hierarchy enables team collaboration + personal customization

That's enough to understand when you encounter `.claude/settings.json` references in documentation.

---

## Try With AI

Now let's explore settings hierarchy with Claude Code.

**Setup**: Open Claude Code in your terminal and work through these prompts to understand how settings levels work together.

### Prompt 1: Explain Settings Hierarchy

```
In Claude Code, settings exist at three levels: user (~/.claude/settings.json),
project (.claude/settings.json), and local (.claude/settings.local.json).

Explain what each level is for and why having three levels is better than one global settings file.
```

**Expected Outcome**: AI explains that hierarchy enables team standards without overriding personal preferences, and allows temporary overrides without affecting shared configuration.

### Prompt 2: Verify Your Settings Hierarchy (Hands-On)

**First, check yourself which settings files exist**:

```bash
# Check user-level settings
ls -la ~/.claude/settings.json

# Check project-level settings (run from your project directory)
ls -la .claude/settings.json

# Check local-level settings
ls -la .claude/settings.local.json
```

**What you'll see**:
- "No such file or directory" = file doesn't exist (common for fresh installations)
- File details = file exists (you have settings at that level)

**Now ask Claude to explain what you found**:

```
I just checked my settings files. Here's what exists:
- User level (~/.claude/settings.json): [exists / doesn't exist]
- Project level (.claude/settings.json): [exists / doesn't exist]
- Local level (.claude/settings.local.json): [exists / doesn't exist]

Explain what this tells me about my current configuration and which level is active.
```

**Expected Outcome**: You understand hierarchy through **direct observation**, not just theory. Claude explains which settings are actually controlling your current session.

### Prompt 3: Precedence Scenario

```
Let's say:
- User level has: outputStyle = "Concise"
- Project level has: outputStyle = "Explanatory"
- Local level is: not set

Which outputStyle is active? Why?
Then: what happens if I add outputStyle = "Verbose" to local level?
```

**Expected Outcome**: AI correctly explains that project level wins initially (Explanatory), then local level overrides when you add it (Verbose)—demonstrating understanding of precedence order.

### Prompt 4 (Stretch): Plan for Part 5

```
When I learn team workflows in Part 5, what settings would I modify?
- User level (my personal preferences)?
- Project level (team standards)?
- Local level (temporary experiments)?

Explain when each level is the right choice for different scenarios.
```

**Expected Outcome**: AI explains use cases—user level for personal defaults, project level for team standards, local level for temporary changes. You'll understand why hierarchy matters for team collaboration.
---
title: "Settings Hierarchy"
sidebar_position: 9
chapter: 5
lesson: 9
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

![Pyramid showing three configuration levels—Global settings (base, system-wide defaults), Project settings (middle, .claude/config for repo), File settings (top, frontmatter overrides)—with precedence arrows showing file > project > global](/img/part-2/chapter-05/settings-hierarchy-global-project-file.png)

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

Let's understand how Claude Code's three-level settings hierarchy enables both team collaboration and personal customization.

**💡 Understand the Hierarchy:**

> "Claude Code has settings at three levels: user (~/.claude/settings.json), project (.claude/settings.json), and local (.claude/settings.local.json). Explain what each level is for and why having three levels is better than one global settings file. Give me concrete examples of what I'd put at each level."

**🔍 Verify Your Current Configuration:**

> "Help me check which settings files exist on my system. Walk me through the commands to check each level (user, project, local). Then, based on what exists, explain which settings are actually controlling my current Claude Code session and why."

**🧪 Test Precedence Rules:**

> "Let's test precedence with a scenario: User level has outputStyle='Concise', Project level has outputStyle='Explanatory', and Local level is not set. Which outputStyle is active and why? Then, if I create a .claude/settings.local.json file with outputStyle='Verbose', what happens? Walk me through the precedence logic."

**🚀 Plan for Team Workflows:**

> "When I learn team workflows in Part 5, help me understand which settings level to use for different scenarios: personal preferences (my editor style, my default verbosity), team standards (shared coding conventions, security policies), and temporary experiments (testing new configurations). Explain the decision framework for choosing the right level."
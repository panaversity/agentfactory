---
sidebar_position: 8
title: "Plugins: Composing Commands, Agents, Skills & Hooks"
duration: "40 min"
---

# Plugins: Composing Commands, Agents, Skills & Hooks

## The Problem: Isolated Automations Don't Scale

You've learned **four powerful extensions**:
- **Commands** (`/commit`, `/clear`, custom commands)
- **Agents** (Subagents for specialized tasks)
- **Skills** (Ambient expertise applied automatically)
- **Hooks** (Automation triggers before/after actions)

But they're scattered:
- One command here
- One hook there
- One skill scattered across projects
- One agent somewhere else

**What if you could package them together?** That's what **plugins** do.

**Without plugins**: Automations are isolated, hard to share, difficult to maintain.

**With plugins**: Everything needed for a workflow lives in one place—commands, agents, skills, hooks all working together.

---

## What Are Plugins?

**Definition**: A plugin is a **composable package** that bundles commands, agents, skills, and hooks into a single reusable workflow.

Instead of scattered automations, plugins organize everything needed for a workflow in one place:
- **Commands** — New `/command` interface
- **Agents** — Subagents that execute specialized tasks
- **Skills** — Ambient expertise applied automatically
- **Hooks** — Automation triggers before/after actions

### Plugins vs. Individual Extensions

You've learned each extension separately. Here's how they compare:

| Feature | Subagent | Agent Skill | Plugin |
|---------|----------|-------------|--------|
| **Purpose** | Specialized isolated task execution | Ambient expertise applied across work | Workflow automation & new commands |
| **Invocation** | Explicit (`claude agent run`) or auto-delegated | Autonomous (Claude discovers when relevant) | Explicit (`/command`) or auto-triggered via hooks |
| **Scope** | Single focused task | Refines outputs across multiple phases | Orchestrates multi-step workflows |
| **Example** | Code reviewer with custom standards | Auto-detect Python files → suggest docstrings | `/code-review` runs 4 review agents in parallel |
| **Best For** | Quality gates & specialized review | Embedded best practices | Automation & standardization |

### Key Insight: When to Use Each

- **Subagents**: "I need a specialized review from a code expert"
- **Skills**: "I always want type hints checked automatically"
- **Plugins**: "I want to run a complete code review workflow every time I create a PR"

---

## Plugin Architecture: How Plugins Compose Everything

Before learning built-in plugins, understand the architecture.

### Inside a Plugin: The Composition

Every plugin packages four things together:

```
my-plugin/
├── .claude-plugin/
│   └── plugin.json          # Metadata
├── commands/
│   └── my-command.md        # /my-command entry point
├── agents/
│   └── my-agent.md          # Specialized subagents
└── hooks/
    └── hooks.json           # Pre/post automation triggers
```

**How they work together:**

1. **Hook triggers** → Detects when to activate
2. **Command invokes** → User runs `/command`
3. **Agent executes** → Subagent handles specialized work
4. **Skills apply** → Ambient expertise throughout

**Real example: `/code-review` plugin**

```
User: /code-review
  ↓
Hook: Monitors for PR context
  ↓
Command: Parses PR metadata
  ↓
Agents: 4 parallel review agents run
  ↓
Skills: Each agent uses code-quality and security skills
  ↓
Output: Review comment posted to GitHub
```

---

## Built-In Plugins (Ready to Use)

Claude Code ships with powerful plugins that demonstrate composition. You can install and use them immediately.

### Plugin 1: Code Review (`code-review`)

**What it does**: Automatically reviews pull requests with confidence-based scoring. Analyzes the code, identifies issues, and posts review comments on GitHub.

**When to use**: Before merging a PR, want automated feedback

**How to enable**:
```bash
/plugin enable code-review
```

**How to run**:
```bash
/code-review
```

**What happens**:
- Claude launches 4 review agents in parallel
- Each agent scores confidence in issues (0-100)
- Only issues with ≥80 confidence are reported
- Filters out false positives automatically
- Posts GitHub comment with findings

**Example output**:
```
## Code Review (4 agents, 2 high-confidence issues)

1. Missing error handling for OAuth callback (confidence: 89)
   → Line 67-72 in src/auth.ts

2. Potential SQL injection in user query (confidence: 92)
   → Line 134 in src/database.ts
```

---

### Plugin 2: Feature Development (`feature-dev`)

**What it does**: Guides you through a structured 7-phase workflow for building features, automatically delegating to specialized agents at each phase.

**When to use**: Building a new feature from start to finish

**How to enable**:
```bash
/plugin enable feature-dev
```

**How to run**:
```bash
/feature-dev
```

**The 7 phases** (Claude automates):
1. **Exploration** - Understand existing codebase patterns
2. **Design** - Plan the feature architecture
3. **Specification** - Write formal spec
4. **Implementation** - Code the feature
5. **Testing** - Write and run tests
6. **Review** - Self-review before PR
7. **Documentation** - Generate docs

**What happens**: You describe what you want to build, Claude Code orchestrates specialized agents for each phase.

---

### Plugin 3: PR Review Toolkit (`pr-review-toolkit`)

**What it does**: Comprehensive PR analysis with 6 specialized agents, each checking different aspects.

**When to use**: Before PR creation, want deep analysis

**How to enable**:
```bash
/plugin enable pr-review-toolkit
```

**Agents** (run in parallel):
- **Comment Analyzer** - Verify comments are accurate & documentation is clear
- **Test Analyzer** - Evaluate test coverage quality
- **Silent Failure Hunter** - Find inadequate error handling
- **Type Design Analyzer** - Review type design and invariants
- **Code Reviewer** - General code quality check
- **Code Simplifier** - Suggest refactoring & simplifications

**Example**: Ask Claude Code to "Review error handling in this PR" and the Silent Failure Hunter automatically runs.

---

### Plugin 4: Commit Commands (`commit-commands`)

**What it does**: Git workflow automation. Handles committing, branch management, and cleanup.

**When to use**: Streamline git operations

**How to enable**:
```bash
/plugin enable commit-commands
```

**Available commands**:
- `/commit` - Create a commit with auto-generated message
- `/clean-gone` - Remove branches marked as [gone] and their worktrees
- `/branch-status` - Show all branches with status

**Example**:
```bash
/commit
# Claude auto-stages changed files and creates descriptive commit message
```

---

## Installing Plugins from Marketplace

Claude Code has a marketplace of additional plugins. You can browse and install them.

### How to Browse Plugins

```bash
/plugin marketplace
```

This shows available plugins with descriptions. Search for what you need.

### How to Install a Plugin

```bash
/plugin install plugin-name
```

**Example**:
```bash
/plugin install security-guidance
```

### How to Manage Plugins

```bash
/plugin list              # Show all installed plugins
/plugin enable name       # Activate a plugin
/plugin disable name      # Deactivate a plugin
/plugin validate          # Check if plugins are valid
```

---

## Creating Your Own Plugin (Optional)

If you want to automate a workflow specific to your team, you can create a custom plugin.

### Plugin Structure

Plugins live in `.claude-plugin/` directory:

```
my-plugin/
├── .claude-plugin/
│   └── plugin.json          # Plugin metadata
├── commands/
│   └── my-command.md        # Defines `/my-command`
├── agents/
│   └── my-agent.md          # Custom agent for this plugin
└── hooks/
    └── hooks.json           # Auto-triggers for workflows
```

### Simple Example: Creating a `/deploy` Command

**Step 1: Create `.claude-plugin/plugin.json`**

```json
{
  "name": "deploy-commands",
  "version": "1.0.0",
  "description": "Automate deployment workflows",
  "author": "Your Team"
}
```

**Step 2: Create `commands/deploy.md`**

```markdown
# Deploy Command

Automates deployment to production.

## Steps

1. Run tests
   - Execute `npm test`
   - If any test fails, stop here

2. Build
   - Execute `npm run build`

3. Deploy
   - Push to production branch
   - Trigger deployment pipeline

4. Verify
   - Run health checks
   - Report status
```

**Step 3: Enable the plugin**

```bash
/plugin validate
/plugin enable deploy-commands
```

**Step 4: Use your command**

```bash
/deploy
# Claude Code follows the steps you defined
```

### Advanced: Hooks for Automation

Hooks trigger automatically on certain events. Example hook config:

```json
{
  "hooks": {
    "before_commit": [
      {
        "command": "run tests before committing"
      }
    ],
    "before_pr_create": [
      {
        "command": "run code review before creating PR"
      }
    ]
  }
}
```

With this hook, every time you try to create a commit, the tests automatically run first. If tests fail, commit is blocked.

---

## Real Workflow: PR Review Automation

Let's see plugins in action with a realistic workflow.

**Scenario**: You've finished coding a feature and want to review it before pushing to GitHub.

### Without Plugins:
1. Manually run tests
2. Manually check code for style issues
3. Manually verify error handling
4. Manually write PR description
5. Manually review type safety
6. Finally push to GitHub

**Time: 45+ minutes**

### With Plugins:

```bash
# 1. Enable the toolkit
/plugin enable pr-review-toolkit
/plugin enable code-review

# 2. Run the review
/code-review

# 3. Fix issues found
# (Claude provides specific line numbers and suggestions)

# 4. Commit and push
/commit
git push

# 5. GitHub automatically runs the code-review plugin again on the PR
```

**Time: 15 minutes**

---

## Try With AI

Now put plugins into practice.

### Exercise 1: Enable and Run a Plugin

**Setup**:
- Choose one built-in plugin from the list above (`code-review`, `feature-dev`, `pr-review-toolkit`, or `commit-commands`)
- Have a recent piece of code or PR ready

**Prompt**:
```
Enable the [plugin-name] plugin and run it on my recent work.

Specifically:
1. Run /plugin enable [plugin-name]
2. Execute the appropriate command
3. Show me the results and explain what each finding means
4. Suggest 1-2 actions I should take based on the findings
```

**Expected outcome**: You see what a plugin does and get actionable feedback on your code.

### Exercise 2: Understand Plugin Scalability

**Prompt** (no code needed):
```
I'm learning about Claude Code plugins. If I had 5 different automated workflows
(code review, testing, documentation, security scan, performance analysis),
should I:

A) Create 5 separate plugins
B) Combine them into 1 large plugin
C) Use hooks to chain them together

Explain the tradeoffs and recommend an approach for a team of 5 developers.
```

**Expected outcome**: You understand when to separate vs. combine automations.

### Exercise 3: Design a Custom Plugin for Your Workflow

**Prompt**:
```
I want to create a custom plugin for my project that automates this workflow:

1. [Your workflow step 1]
2. [Your workflow step 2]
3. [Your workflow step 3]

Help me:
1. Design the plugin structure (what commands, what agents)
2. Write the plugin.json metadata
3. Outline the main command definition
4. Suggest hooks that would make this automatic

Keep it simple—3-4 steps maximum.
```

**Expected outcome**: A concrete blueprint for automating your team's most painful workflow.

---

## What's Next

You've now mastered **the full Claude Code stack**:

- ✅ **Installation** - Set up Claude Code on any platform
- ✅ **Commands** - Use daily workflows efficiently
- ✅ **Subagents** - Create specialized isolated assistants
- ✅ **Skills** - Encode team expertise for automatic application
- ✅ **MCP Servers** - Integrate external systems safely
- ✅ **Plugins** - Automate multi-step workflows

**In Part 5-6** (Advanced Orchestration), you'll learn:
- Git worktrees for parallel feature development
- Managing 3-10 AI agents simultaneously
- Coordinating complex decomposition strategies
- Building AI agent team orchestration

For now, **practice using what you've learned**. The real skill is recognizing when to use each tool (commands, subagents, skills, MCP, plugins) for maximum productivity.


<Quiz title="Chapter 8 Quiz" questions={[{"question":"Which of the following is NOT listed as a traditional \u0027gatekeeper\u0027 to learning software development that AI is now helping to remove?","options":{"a":"Syntax Memorization","b":"The need for creative problem-solving","c":"Debugging Cryptic Errors","d":"Configuration Hell"},"correct_answer":"b","explanation":"The chapter lists Syntax Memorization, Debugging Cryptic Errors, and Configuration Hell as barriers being removed. Creative problem-solving, however, is a skill that becomes more important."},{"question":"Why do experienced developers often see greater productivity gains with AI tools compared to beginners?","options":{"a":"Because they can type faster than the AI.","b":"Because their experience allows them to provide more precise specifications and better evaluate the AI\u0027s output.","c":"Because the AI tools are exclusively designed for them.","d":"Because they refuse to use the AI tools and work manually."},"correct_answer":"b","explanation":"The text states that experienced developers know \u0027What to ask for,\u0027 \u0027What to look for,\u0027 and \u0027What matters,\u0027 which allows them to leverage AI more effectively."},{"question":"According to the technology adoption curve shown in the chapter, where are AI coding tools currently positioned?","options":{"a":"Innovators","b":"Early Adopters","c":"Transitioning from Early Adopters to Early Majority","d":"Laggards"},"correct_answer":"c","explanation":"The chapter explicitly states: \u0027AI coding tools are currently transitioning from Early Adopters to Early Majority. → WE ARE HERE ←\u0027"},{"question":"For people with deep expertise in non-technical domains (like healthcare or finance), what is the key opportunity presented by AI coding tools?","options":{"a":"They can now easily switch to a career in AI research.","b":"Their domain expertise is no longer valuable.","c":"They can now build software solutions for their field without needing to become full-time programmers, as AI handles the coding.","d":"They must now get a traditional computer science degree."},"correct_answer":"c","explanation":"The chapter highlights this opportunity: \u0027AI handles coding; you provide domain expertise. The combination is incredibly valuable... AI doesn\u0027t replace domain expertise. It enables domain experts to build solutions...\u0027"},{"question":"What is the chapter\u0027s main argument for why \u0027this is the best time in three to four decades to be learning software development\u0027?","options":{"a":"Because developer salaries are the highest they have ever been.","b":"Because the barriers to entry have been dramatically lowered by AI, while the opportunity for growth is massive.","c":"Because there are fewer developers now, meaning less competition.","d":"Because universities have finally caught up and are teaching all the necessary skills."},"correct_answer":"b","explanation":"The central theme of the chapter is that the combination of falling barriers (syntax, debugging, etc.) and being in the \u0027Early Majority\u0027 phase of a massive technological shift creates a unique and powerful opportunity window."}]} />


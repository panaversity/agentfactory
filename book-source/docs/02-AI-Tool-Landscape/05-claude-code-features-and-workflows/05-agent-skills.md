---
sidebar_position: 5
title: "Creating and Using Agent Skills"
---

# Creating and Using Agent Skills

## The Competitive Advantage Hiding in Plain Sight

Skills are your team's reusable intelligence. Claude Code can auto‑detect opportunities to apply that intelligence and can also auto‑delegate to the right subagent when focused execution is needed.

Skills are **discovered and suggested autonomously by Claude Code** when relevant. Subagents handle focused, isolated execution; skills continuously inject shared standards and domain expertise.

In this lesson, you'll learn how skills work, create your first skill, and understand why building a skill library is a strategic competitive advantage for teams and companies.

---

## What Are Agent Skills?

**Definition**: An Agent Skill is a modular, discoverable capability that Claude Code can autonomously invoke when working on relevant tasks. Skills are defined by a `SKILL.md` file containing instructions and a **description** that helps Claude decide when to use them.

### Skills vs. Subagents vs. Slash Commands

You've now seen three ways to extend Claude Code. Here's how they differ:

| Feature | Subagent | Agent Skill | Slash Command |
|---------|----------|-------------|---------------|
| **Invocation** | Explicit or auto‑delegated | **Autonomous**: Claude discovers and suggests | Explicit: `/command-name` |
| **Discovery** | Manual—you decide when to run | **Automatic**—Claude decides based on context | Manual—you type the command |
| **Use Case** | Repetitive focused tasks | Domain expertise applied automatically | Predefined workflows |
| **Example** | `claude agent run code-reviewer` | Claude detects Python file, suggests docstrings | `/commit` to create git commit |
| **Competitive Advantage** | Medium (consistency) | **High (scales expertise)** | Low (simple automation) |

**Working together**
- Subagents: isolated context, task ownership (Claude may auto‑delegate)
- Skills: ambient capabilities that refine outputs across phases
> Claude discovers skills from `SKILL.md` descriptions and can delegate to subagents when your task clearly matches their description. See Subagents docs for details.

---

## Why Agent Skills Matter: The Strategic Value

Agent Skills aren't just a technical feature—they're a **strategic business advantage**. Here's why:

### 1. Expertise That Scales Automatically

**Without Skills**:
- Senior developer explains best practices to junior developer
- Junior developer forgets 60% within a week
- Process repeats for every new hire
- Knowledge stays locked in people's heads

**With Skills**:
- Senior developer encodes expertise once as a skill
- Claude Code applies it automatically across all projects
- Every developer benefits immediately
- Knowledge captured permanently in version control

**Example**: A security expert creates an `sql-injection-checker` skill. Now every developer—regardless of security expertise—gets automatic alerts when writing database queries that might be vulnerable.

### 2. Competitive Differentiation Through Domain Expertise

This is where skills become truly powerful: **Your unique domain knowledge becomes an automated advantage.**

**Example 1**: A fintech company builds a `compliance-checker` skill that validates financial calculations against regulatory requirements. Their developers ship compliant code faster than competitors who review regulations manually.

**Example 2**: A healthcare startup creates a `hipaa-privacy-auditor` skill that scans code for potential PHI (Protected Health Information) leaks. Their code is secure by default; competitors discover privacy issues in production.

**Example 3**: A machine learning team builds a `model-reproducibility-checker` skill that ensures experiments log hyperparameters and random seeds. Their models are reproducible; competitors waste weeks debugging non-deterministic results.

**The pattern**: Domain expertise encoded as skills compounds over time, creating organizational capabilities that competitors can't easily replicate.

---

## How Agent Skills Work: The Discovery Mechanism

Let's understand the magic behind autonomous discovery.

### Skill Anatomy: The SKILL.md File

Every skill is defined by a `SKILL.md` file with three critical sections:

**1. Discoverable Description** (most important):
- Clear trigger: when should Claude suggest this skill?
- Outcome: what does the skill produce?
- Scope and boundaries: what it will and will not do

**2. Skill Instructions**:
- Checklist of steps to follow
- Quality bar: what good output looks like
- Edge cases and constraints to respect

**3. Examples** (optional):
- Brief before/after descriptions (no code required)

---

## Skill Scopes: Where Skills Live

Skills can exist at three levels:

**1. Personal Skills** (`~/.claude/skills/`)
- Your personal toolkit
- Not shared with projects or team
- Use for personal workflow preferences

**2. Project Skills** (`.claude/skills/` in project directory)
- Specific to one project
- Committed to version control
- Team members inherit when they clone the repo
- **Most common for team collaboration**

**3. Plugin Skills** (installed from skill registry)
- Publicly available skills
- Installed with `claude skill install <name>`
- Maintained by community or vendors

**Best Practice**: Use **project skills** for team standards and domain expertise. This ensures everyone on the team benefits from shared knowledge.

---

## Quick Start: Add One Skill, See It Work

Goal: add a project skill that explains startup ideas.

Ask Claude:
```
Create a project skill named "idea-evaluator" to evaluate project ideas and decide on feasibility. Use docs to understand how to build skills: https://docs.claude.com/en/docs/claude-code/skills Store it project level in .claude/skills/.
```

You can now ask Claude "What skills do you have?" and it will list all the skills you have installed.

---

## ✓ Your Skill Is Working When:

**Quick check**:

1. **Skill is created** - Skill directory exists
2. **Skill is discovered** - When relevant, Claude suggests using it

**If this works**: 🎉 **Your collaborative skill is ready! Claude now automatically helps you understand errors as you encounter them.**

---

## Try With AI

Use Claude Code for this activity (preferred, since you just installed it). If you already have another AI companion tool set up (e.g., ChatGPT web, Gemini CLI), you may use that instead—the prompts are the same.

### Prompt 1: Skills vs. Subagents Comparison

```
I'm confused about skills vs. subagents vs. slash commands. Create a comparison table that shows: (a) when to use each, (b) a concrete example of each for MY work [describe what you do], (c) which one to learn FIRST as a beginner. Make it crystal clear.
```

**Expected outcome:** Clear differentiation between skills, subagents, and commands

### Prompt 2: First Skill Design

```
I want to create my FIRST skill. My team's biggest pain point is [describe: inconsistent code style / missing documentation / security vulnerabilities / etc.]. Design a skill for this: (a) What should I name it? (b) Write a 'discoverable description' that triggers when relevant, (c) List 5-7 instructions, (d) Give me one before/after example showing what it does.
```

**Expected outcome:** Complete design for your first team skill

### Prompt 3: Discovery Mechanism Explained

```
The lesson says skills are 'discovered autonomously' by Claude Code. I don't understand HOW that works. Explain the discovery mechanism step-by-step: (a) What does Claude read? (b) When does it decide to suggest a skill? (c) How do I write descriptions that trigger at the right time? (d) Give me 3 example descriptions with explanations.
```

**Expected outcome:** Deep understanding of autonomous discovery mechanism

### Prompt 4: Strategic ROI Analysis

```
The lesson talks about 'strategic competitive advantage' of building a skill library. Help me think strategically: If I invest time building skills for [my domain: Python / JavaScript / data science / etc.], what's the ROI? How much time will I SAVE in 6 months? Create a simple cost-benefit analysis.
```

**Expected outcome:** Strategic justification for investing time in skill creation

<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />


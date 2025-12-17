# Lesson 6 Summary: Building Your Own Skills

## Core Concept

Skills are folders with instructions that teach Claude how to execute specific tasks your way. They activate automatically based on context—you describe what you want, and Claude loads the right skill without explicit invocation.

## SKILL.md Structure

Every skill needs one file: `SKILL.md`

```yaml
---
name: "skill-name"
description: "What it does + when to use it"
version: "1.0.0"
---

# Skill Name

## When to Use
- Trigger conditions

## Procedure
1. Step one
2. Step two

## Output Format
- Expected structure

## Example
Input → Output demonstration
```

## The Description Formula

The most critical line. Determines when Claude activates your skill.

```
[Action verb] + [input type] + [into/for] + [output type] + [key features].
Use when [trigger conditions].
```

**Example**:
```yaml
description: "Transform meeting transcripts into structured summaries with action items and decisions. Use when user shares meeting content."
```

## Three-Level Loading

1. **Level 1 (Always)**: Brief descriptions—Claude knows skills exist
2. **Level 2 (On-demand)**: Full SKILL.md—loaded when triggered
3. **Level 3 (If needed)**: Scripts, references—accessed during execution

## Skills vs. Subagents

| Factor | Skill | Subagent |
|--------|-------|----------|
| Invocation | Automatic | Explicit |
| Context | Shared | Isolated |
| Best for | Repeated patterns | Complex workflows |
| Guarantee | Soft (might not trigger) | Hard (always runs) |

## Co-Learning Refinement

1. **AI as Teacher**: Ask Claude to review and suggest improvements
2. **You as Teacher**: Specify constraints Claude doesn't know
3. **Convergence**: Iterate until skill matches your workflow

```
Review my [skill-name] skill. What could be improved?
```

```
Good suggestions, but I have constraints: [your requirements].
Update the skill with these constraints.
```

## Skill Design Principles

1. **Be concise** — Claude is smart, don't over-explain
2. **Show examples** — Concrete demonstrations beat abstract rules
3. **Specify quality criteria** — Make your standards explicit
4. **Include "when to use"** — Clear activation triggers

## Skill-Creator Meta-Skill

Use Claude to create skills:

```bash
cd claude-code-skills-lab
claude
```

```
Use the skill-creator to help me build a skill for [your procedure].
```

The skill-creator guides you through:
- Clarifying questions
- Structure planning
- Description writing
- SKILL.md generation
- Testing guidance

## Your Workflow

1. **Identify** — What procedure do you repeat?
2. **Draft** — Write SKILL.md with frontmatter + instructions
3. **Test** — Try with real tasks
4. **Iterate** — Use co-learning to refine
5. **Share** — Skills are portable across Claude surfaces

## Common Mistakes

- **Vague descriptions** — "Helps with notes" won't trigger; be specific
- **Too narrow** — "Summarizes Zoom calls from marketing" misses edge cases
- **Over-explaining** — Claude is smart; trust it
- **No examples** — Concrete input/output pairs teach better than rules

## Skills Lab Reference

1. Go to [github.com/panaversity/claude-code-skills-lab](https://github.com/panaversity/claude-code-skills-lab)
2. Click green **Code** button → **Download ZIP**
3. Extract and open folder in terminal

**8 ready-to-use skills**: docx, pptx, xlsx, pdf, doc-coauthoring, internal-comms, theme-factory, skill-creator

## Connections

- **Lesson 04**: WHY skills exist (conceptual foundation)
- **Lesson 05**: WHERE skills fit (platform architecture)
- **Lesson 06 (this)**: HOW to build skills (hands-on creation)
- **Lesson 07**: CLAUDE.md for project context
- **Lesson 08**: MCP for external connectivity

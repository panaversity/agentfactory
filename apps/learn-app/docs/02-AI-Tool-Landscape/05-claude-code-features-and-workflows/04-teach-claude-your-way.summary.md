# Lesson 4 Summary: Teach Claude Your Way of Working

## Key Concepts

### 1. The Repetition Tax
Every time you re-explain preferences to Claude, you pay a cost in time and context. Skills eliminate this tax by encoding your procedures once, permanently.

### 2. Prompts vs. Procedures
- **Prompt**: Commands WHAT you want ("Write a blog post")
- **Procedure**: Encodes HOW you think about a task (your structure, preferences, quality criteria)

Prompts get you *a* result. Procedures get you *your* result.

### 3. Skills = Encoded Expertise
A skill is a folder of instructions that teaches Claude how to think about a specific type of task. Skills capture reasoning patterns and judgment—not just commands.

### 4. Automatic Activation
Skills are discovered automatically. You don't invoke them by name. Claude recognizes when your encoded expertise applies and loads it.

### 5. Skills vs. Alternatives

| Mechanism | Use When |
|-----------|----------|
| **Prompts** | One-time, exploratory tasks |
| **CLAUDE.md** | Project-specific context |
| **Subagents** | Complex tasks needing isolated context |
| **Skills** | Repeatable procedures, automatic application |

### 6. Cross-Platform Portability
One skill works across Claude.ai, Claude Code, and the API. No modifications needed.

### 7. Skills Beyond Code
Skills work for any repeated task: writing, research, accountability tracking, meeting notes, learning workflows—not just programming.

---

## Mental Models

### The Expertise Transfer Model
Skills compress the "learning curve" for working with you. Instead of Claude learning your preferences through months of interaction, skills transfer your judgment immediately.

### The Procedure Identification Test
If you've explained the same thing to Claude more than twice, and it's stable enough to document, it's a skill candidate.

---

## Common Mistakes to Avoid

1. **Thinking skills are "saved prompts"** — Skills encode reasoning patterns, not just text to paste
2. **Only considering coding tasks** — Skills work for any repeated procedure
3. **Overcomplicating the procedure** — "Meeting notes with action items first" is enough; simplicity wins
4. **Forgetting portability** — Define once, use across all Claude surfaces

---

## Hands-On: Skills Lab

Experience skills in action before creating your own:

1. Go to [github.com/panaversity/claude-code-skills-lab](https://github.com/panaversity/claude-code-skills-lab)
2. Click green **Code** button → **Download ZIP**
3. Extract and open folder in terminal
4. Run `claude`

**Available skills to try**:
- `docx` — Word document creation
- `pptx` — PowerPoint presentations
- `xlsx` — Spreadsheets with formulas
- `pdf` — PDF manipulation
- `doc-coauthoring` — Proposals and specs
- `internal-comms` — Newsletters and reports
- `theme-factory` — Professional styling
- `skill-creator` — Build new skills

**Test prompt**: "Create a project proposal document for a mobile app redesign"

---

## Preparation for Lesson 06

Before Lesson 06, identify one procedure you want to encode:

1. **What task triggers it?**
2. **What steps do you follow?**
3. **What makes your output distinctive?**
4. **What implicit knowledge does it require?**

Document your answers. This becomes raw material for your first skill.

---

## Quick Reference

**Skill**: Encoded expertise that Claude activates automatically when relevant

**Key insight**: Prompts encode WHAT. Skills encode HOW.

**Portability**: Same skill works in Claude.ai, Claude Code, and API

**Activation**: Automatic (Claude decides) vs. subagents (you invoke explicitly)

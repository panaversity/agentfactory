# Designing Reusable Intelligence

This lesson teaches students to create reusable skills through conversation with AI after good sessions. Students identify patterns worth encoding (frequency + complexity + value), have a conversation where they explain their approach, and AI structures it into a complete skill file with metadata, process steps, quality criteria, usage example, and self-check validation. Students save skills to `.claude/skills/`, test them on real tasks, and iterate until they work. The lesson also distinguishes skills (2-6 decision points, human-guided) from subagents (7+ decisions, AI autonomous).

### Mental Models

- **Skills from Good Sessions**: After a session that went well, ask: Will I do this again? Did it involve multiple decisions? Would I want the same quality next time? If 2+ YES → create a skill.

- **Conversation-Based Creation**: You describe your process naturally, AI asks clarifying questions (what makes output good? what's your process? what quality checks?), AI structures it into a complete skill file.

- **Test and Iterate Loop**: Skills need iteration. Save skill → test on real task → evaluate results → update skill → repeat until consistent results.

- **Reasoning vs Prediction Mode**: Good skills trigger reasoning mode (AI asks for clarification, applies each step explicitly). Bad skills trigger prediction mode (AI produces generic content ignoring the skill process).

- **Intelligence Compounds**: Project 1: 8-10 hours (from scratch). Project 2 with skill: 4 hours (50% faster). Project 3 with multiple skills: 3.5 hours. Each skill accelerates future work.

### Key Patterns

- **Start skill conversation**: After good session, say "I want to create a skill for [workflow]. During my work, I noticed my best [outputs] had [specific qualities]. Help me turn this into a reusable skill."

- **Complete skill file structure**: YAML frontmatter (`name`, `description`, `version`) → When to use → Process steps → Output format → Quality criteria → Example.

- **Save to `.claude/skills/<skill-name>/SKILL.md`**: Create skill directory with `mkdir -p .claude/skills/section-writer`, then save SKILL.md inside. Skills use directory structure (skill-name/SKILL.md), not flat files.

- **Test skill on real task**: Apply skill to actual work, evaluate: Did AI follow the process? Did output meet criteria? What's missing?

- **Iterate with specific feedback**: "The skill worked but [specific issue]. Update skill to [fix]. Save updated version."

- **Validate skill triggers reasoning**: Apply skill to vague request. Good skill asks for clarification and shows step-by-step reasoning. Bad skill produces generic content.

### Common Mistakes

- **Creating skills for trivial patterns**: "How to format headings" (1-2 decisions) doesn't justify a skill. Save skills for 5+ decision workflows.

- **Skipping testing**: Saving skill and assuming it works. Skills need iteration—first version misses edge cases.

- **Over-specific skills**: "AI-Education-Literature-Review-Writer" only works for one topic. Generalize to "Section-Writer" (any paper).

- **No quality criteria**: Skill describes process but not what "good" looks like. Every skill needs explicit ready/needs-work criteria.

### Progression Context

- **Builds on**: Lessons 04-08 (complete SDD workflow). Students have executed one full project cycle and understand where patterns recur.

- **Leads to**: Lesson 11 (Capstone) where students invoke created skills to accelerate new work, demonstrating intelligence compounding across projects.

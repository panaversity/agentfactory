### Core Concept
Skills are reusable capabilities that Claude discovers automatically based on context—you describe what you want, and Claude loads the right skill without explicit invocation. Unlike command automation, skills encode *reasoning patterns* that make Claude smarter by default.

### Key Mental Models
- **Three-Level Loading**: Brief summaries always loaded (Level 1), full instructions on-demand (Level 2), supporting files if needed (Level 3)—efficient context management
- **Autonomous Discovery**: Skills activate through context recognition (content type, task request), not explicit commands—shifting from "command what exists" to "describe what you want"
- **Reasoning Patterns vs Commands**: Skills teach "how to think about X" not "what commands to run"—making organizational intelligence transferable

### Critical Patterns
- Create `SKILL.md` with YAML frontmatter (`name`, `description`, `version`) plus instructions, workflows, and examples
- Write descriptions that specify WHEN to activate: "Use when user asks to plan or write blog content"
- Structure skills around workflows: research → outline → output format → examples
- Refine through co-learning: AI suggests improvements, you specify constraints, converge on optimized design

### Common Mistakes
- Using skills for complex tasks needing guaranteed execution—subagents provide isolated context and explicit invocation
- Writing vague descriptions that fail to trigger autonomous discovery
- Treating skills as script wrappers instead of reasoning frameworks

### Connections
- **Builds on**: CLAUDE.md (project intelligence), subagents (isolated task execution)
- **Leads to**: Plugins (bundling skills as shareable marketplace capabilities)

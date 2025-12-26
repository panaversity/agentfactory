### Core Concept
Agent Skills are folders containing instructions (SKILL.md) that encode your domain expertise into reusable packages any AI agent can load—transforming tacit knowledge (in your head) into explicit knowledge (in a file) that's portable, licensable, and monetizable.

### Key Mental Models
- **Matrix Skill Loading**: Like Trinity downloading helicopter skills, agents load expertise on-demand—years of experience, instantly available
- **Progressive Disclosure**: Three-level loading (names at startup ~100 tokens, full SKILL.md when activated <5K, supporting files on demand) achieves 80-98% token reduction
- **MCP = Connectivity, Skills = Expertise**: MCP server gives agent *access* to Stripe API; skill gives agent *expertise* in handling payment scenarios properly

### Critical Patterns
- SKILL.md format: YAML frontmatter (name, description for activation) + Markdown instructions (when to use, how to execute, output format)
- Adoption timeline: October 2025 (Claude Code), December 2025 (open standard at agentskills.io), OpenAI/Microsoft adoption
- Four monetization paths: license individual skills, sell skill bundles, deploy skill-enhanced Digital FTEs, consulting on skill development

### Common Mistakes
- Loading all skill content upfront (wastes tokens, bloats context—defeats the purpose)
- Confusing Skills with MCP (they're complementary: Skills = brain, MCP = hands)
- Writing vague descriptions that don't help agents decide when to activate
- Publishing proprietary methodology to public repositories without considering competitive advantage

### Connections
- **Builds on**: MCP for connectivity (Lesson 2), goose as reference agent showing skills in action (Lesson 4)
- **Leads to**: Part 3 Claude Code Mastery (using skills), Part 5-6 (building skills into Custom Agents)

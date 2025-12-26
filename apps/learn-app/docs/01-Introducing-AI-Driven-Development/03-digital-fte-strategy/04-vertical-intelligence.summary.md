### Core Concept
In the AI era, code is disposable (regenerated per application) while intelligence is permanent (reused across applications). The new architecture reuses system prompts, skills, and MCP connections—not code libraries.

### Key Mental Models
- **Five Components**: System prompt (persona), horizontal skills (infrastructure), vertical skills (domain expertise), horizontal MCPs (dev tools), vertical MCPs (industry APIs). Each layer adds defensibility.
- **Defensibility Hierarchy**: System prompts are easy to copy; vertical MCPs (Epic, Bloomberg integrations) are expensive to replicate because they require credentials, relationships, and maintenance.
- **Progressive Disclosure**: SKILL.md files load ~100 tokens at startup; detailed references load on-demand. This achieves 100:1 token efficiency versus embedding everything in prompts.

### Critical Patterns
- Organize skills by domain: `.claude/skills/healthcare/billing/SKILL.md`
- Vertical MCP connections are your moat—competitors must rebuild from scratch (6 months, $200K)
- Write skills once, license to many customers (5-10 hours per skill → 50x return across 50 clients)
- Use scripts for complex lookups (0 tokens—executed locally, not loaded into context)

### Common Mistakes
- Treating code reuse as the goal when AI can regenerate code in seconds
- Embedding all knowledge in system prompts (expensive, inflexible) instead of using SKILL.md progressive disclosure
- Focusing on system prompts (easy to copy) instead of vertical integrations (hard to replicate)

### Connections
- **Builds on**: Competitive layer positioning (Lesson 2) and FTE economics (Lesson 3)
- **Leads to**: Monetization models (Lessons 5-6) and PPP market entry (Lesson 7)

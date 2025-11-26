## Spec-Kit Plus Foundation

Spec-Kit Plus is an SDD-RI (Specification-Driven Development with Reusable Intelligence) framework that shifts how you build projects: every feature produces two deliverables—working code that solves the immediate problem, and intelligence artifacts (ADRs, PHRs) that document what worked. These artifacts stay in their project, but YOUR learning compounds across projects. This lesson establishes the two core intelligence architectures (Horizontal and Vertical) that enable compounding expertise over time.

### Mental Models

- **Two-Output Philosophy**: Every project generates both ephemeral working code (project-specific, technology-locked) and permanent intelligence artifacts (ADRs documenting decisions, PHRs logging what prompts worked). While code could be rewritten tomorrow, documented intelligence preserves the "why" and "how" for future reference.

- **Horizontal Intelligence (Learning Across Time)**: YOUR knowledge flows from past projects to future projects. ADRs and PHRs are automatically created within each project and stay there. The artifacts document your learning; the wisdom lives in YOU. Each project makes YOU smarter, and that expertise carries forward.

- **Vertical Intelligence (Your Reusable Components)**: After sessions that go well, YOU can create reusable intelligence components: Skills (structured prompts with P+Q+P), Subagents (specialized agents for complex autonomous tasks), or Tools/MCP Servers (custom capabilities). These are user-created (not automatic). You decide when a pattern is worth encoding.

- **Intelligence Compounding**: Unlike code libraries (which solve specific technology problems), YOUR learning compounds because artifacts capture reasoning. Each project makes you faster—not because artifacts transfer, but because YOU internalized what worked.

### Key Patterns

- **ADR (Architectural Decision Record)**: Automatically documents the "WHY" behind significant choices during a project: decision context, reasons for choosing this option, tradeoffs accepted. Stays in the project where created.

- **PHR (Prompt History Record)**: Automatically captures what prompts work vs. fail within a project. A generic prompt might yield 40% of needed detail, while a specific prompt yields production-ready output. PHRs log this difference. Stays in the project where created.

- **Reusable Components**: Skills, subagents, or tools that YOU create after sessions go well. Not automatic—YOU recognize patterns worth preserving, then design the appropriate component type based on complexity.

- **Learning Timeline**: Project 1 takes 60 hours (learning). Project 2 takes 35 hours (40% faster because YOU learned). By Project 10, YOUR accumulated expertise multiplies effectiveness.

### Common Mistakes

- **Expecting automatic cross-project transfer**: PHRs and ADRs are project-scoped. They stay in their project. The compounding benefit comes from YOUR learning, not artifact transfer.

- **Thinking artifacts are the value**: The artifacts document your learning, but the real value is YOUR growing expertise. Each project makes YOU better at decisions and prompts.

- **Creating components too early**: Reusable components should be created AFTER sessions go well, not preemptively. YOU recognize patterns worth preserving, then document them.

- **Making components too specific**: Good components work across similar problems. A Security Specification Skill works for authentication, payments, file uploads—because YOU made the principles general.

- **Confusing Horizontal and Vertical Intelligence**: Horizontal = YOUR learning across time, documented via ADRs/PHRs (auto-created, project-scoped). Vertical = YOUR reusable components (skills/subagents/tools—user-created after good sessions).

### Progression Context

- **Builds on**: Understanding of clear specifications and intent documentation from Chapters 12-13. Students understand that defining "what you want" before "how to build it" matters.

- **Leads to**: Lesson 2 (Setup), Lesson 3 (Constitution), Lessons 4-8 (Workflow phases where ADRs/PHRs are auto-created), Lesson 9 (Learning HOW to create reusable components after good sessions), Lessons 10-11 (Adoption and experiencing acceleration from accumulated expertise).

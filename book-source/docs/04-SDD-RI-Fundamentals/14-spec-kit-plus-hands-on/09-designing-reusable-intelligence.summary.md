## Lesson Title

Designing Reusable Intelligence

## Summary

This lesson teaches the Persona + Questions + Principles (P+Q+P) framework—the pattern that converts tacit knowledge into explicit, reasoning-activated components. Students learn to identify patterns worth encoding as reusable intelligence, design skills using the P+Q+P framework, understand skill file structure and location (.claude/skills/), and distinguish between when to create a skill (2-4 decision points) versus a subagent (5+ decision points). The lesson uses research paper writing as the concrete domain, building skills like section-writer (6 decisions, guides writing process) and research-validator (8+ decisions, autonomous judgment). This represents the Layer 3 transition—moving from executing individual SDD workflows (Lessons 04-08) to accumulating intelligence that compounds capability across future projects.

### Mental Models

- **Reusable Intelligence Over Reusable Code**: In AI-native development, skills and agent architectures are the units of reuse, not code libraries. A section-writer skill applied across 3+ papers compounds faster than writing each paper from scratch.

- **P+Q+P as Reasoning Activation**: Persona (specific cognitive stance, not generic expertise) + Questions (open-ended, forcing analysis) + Principles (concrete decision rules, not vague advice) activates reasoning mode. Contrasts with checklist mode ("Is it done?") that stops at surface-level validation.

- **Decision Point Density as Encoding Threshold**: Patterns with frequency (recurs 3+), complexity (5+ decisions), and organizational value justify intelligence encoding. Trivial patterns (1-2 decisions) don't warrant skills; complex patterns (8+) suggest subagent design instead.

- **Skill Velocity Multiplier**: With accumulated skills, Project 2 writing takes 50% less time than Project 1 (4 hours vs 8-10 hours). Project 3 with multiple composed skills takes 3.5 hours total—showing intelligence compounds across iterations.

- **Quality Gates Over Subjective Assessment**: Measurable quality gates (every paragraph has topic sentence, every claim has source, section reads smoothly, thesis-connected, stands alone) replace subjective "is it good?" judgment. Provides objective pass/fail criteria.

### Key Patterns

- **Persona Specificity Matters**: Effective personas establish a specific cognitive stance with analogy (journalist ensuring readers understand) rather than generic role labels (writing expert). The specific analogy activates domain-specific reasoning instead of generic advice prediction.

- **Open-Ended Questions vs Binary**: "Does this section clearly explain the concept? Is every claim supported? Could an unfamiliar reader understand? What iteration improves clarity?" activates analysis. Yes/no questions ("Is this section good?") do not force reasoning.

- **Principle Concreteness**: Decision principles must provide measurable rules: "Average 15-20 words per sentence," "Define technical terms on first use," "Quality Gate 2: Every factual claim needs source," not abstract guidance like "use best practices" or "write clearly."

- **Skill Reusability Requires Generalization**: Over-specific skills (ML-Healthcare-Section-Writer) reduce reuse value. Generalized skills (Section-Writer applicable to any research paper topic) compound across projects. Evaluate: "Would someone on my team use this immediately?"

- **Skill vs Subagent Boundary at Decision Count**: Skills guide human judgment across 2-4 decisions (writer controls iteration). Subagents operate autonomously across 5+ complex decisions (research validator evaluates source credibility, resolves contradictions without human input). The section-writer (6 decisions) sits on border but functions as skill because writer controls iteration.

- **Quality Gate Prioritization**: Iteration priorities order fixes: clarity first (Principles 1-3), then evidence (Principle 2), then thesis connection (Principle 4). Section ready when all 5 gates pass; needs iteration if 2+ gates fail.

### Common Mistakes

- **Creating Skills for Trivial Patterns**: Encoding "how to open a text editor" (1 decision) as a skill violates the 5+ decision requirement. Only patterns meeting frequency + complexity + organizational value justify encoding.

- **Vague Personas**: "You are a research expert" triggers generic prediction. Specific stances with analogy ("Think like a journalist ensuring readers understand stories—with evidence, clarity, audience awareness") activate reasoning.

- **Yes/No Questions**: Binary questions prevent analysis activation. Open-ended analytical questions (clarity, evidence, flow, audience, completeness dimensions) force deeper thinking.

- **Skills Without Quality Gates**: "Write section and you're done" provides no validation mechanism. Quality gates (topic sentence present, evidence provided, smooth reading, thesis-connected, stands alone) enable objective measurement.

- **Over-Specificity Reducing Reusability**: Domain-specific skills ("Machine-Learning-Healthcare-Section-Writer") only work for that context. Generalized skills ("Section-Writer") multiply value across papers, teams, projects.

- **Confusing Skill Metadata**: Skill files in .claude/skills/ need clear structure—metadata (name, category, complexity, first use, reusable across), persona, questions, principles, usage example, self-check validation. Missing structure reduces usability.

- **Skills as Checklists**: Lists of tasks ("Write intro, add examples, conclude") are workflows, not skills. Skills encode decision frameworks (P+Q+P) that guide judgment across complex situations.

### Progression Context

- **Builds on**: Lessons 04-08 (complete SDD workflow: Specification → Clarify → Plan → Tasks → Implement). Students have executed one full project cycle and understand where patterns recur. Layer 1-2 (Manual Foundation + AI Collaboration) provides foundation; Layer 3 (Intelligence Design) uses that foundation to create reusable components.

- **Leads to**: Layer 4 (Spec-Driven Integration) and capstone projects where multiple accumulated skills compose together. After creating section-writer, research-validator, outline-refiner skills, students orchestrate them in complex writing projects. Creates progression: Manual workflow (L1) → Collaborative workflow with AI (L2) → Reusable skill libraries (L3) → Orchestrated multi-skill projects (L4).

- **Proficiency Alignment**: B1 (Intermediate) level—students handle 4 new concepts (pattern identification criteria, P+Q+P framework, skill file structure, skill vs subagent distinction) with moderate scaffolding. Paper writing exercises provide concrete domain; decision frameworks guide judgment. Prerequisites: understanding SDD workflow and having executed one research paper project (Chapter 14 Lessons 04-08).

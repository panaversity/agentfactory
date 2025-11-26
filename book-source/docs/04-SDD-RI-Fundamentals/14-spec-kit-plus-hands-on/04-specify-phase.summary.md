# Specify Phase Summary

## Lesson Title: Specify Phase

The Specify Phase teaches students how to transform vague project ideas into clear, measurable specifications using the `/sp.specify` command. A specification answers the fundamental question "What are we building?" by defining Intent (what problem it solves and who benefits), Constraints (boundaries and technical requirements), Success Evals (SMART measurable outcomes), and Non-Goals (explicit scope boundaries). Students learn to distinguish specification (WHAT) from planning (HOW) and implementation (DO), mastering a skill that applies to software features, research papers, documentation, deployments, and any structured project.

### Mental Models

- **Specification as Foundation**: Specifications are the blueprint that prevents rework and misunderstanding. Clear specs → clear outputs. Vague specs → confusion. This mental model shifts students from reactive problem-solving ("Let me see what happens") to intentional design ("Here's what success looks like").

- **SMART Criteria Filter**: Success Evals must pass the SMART test (Specific, Measurable, Achievable, Relevant, Time-bound). This model prevents students from accepting vague criteria like "high quality" or "clear writing" and forces objective verification. Example: "Reader understands the topic" fails; "Reader can explain 3 concrete AI use cases" passes.

- **What vs How Boundary**: Specifications describe WHAT (intent, constraints, measurable outcomes). Plans describe HOW (steps, sequence, implementation strategy). Implementation does the work. This model prevents students from leaking implementation details into specifications, which causes confusion during planning.

- **Scope Creep Prevention**: Non-Goals are explicit scope boundaries that prevent feature creep. "Not a comprehensive literature review" stops debates about expanding the paper during execution. This model teaches students that saying "no" upfront is clearer than debating it later.

- **Intent-First Thinking**: Before defining constraints or success criteria, clarity about the problem and audience creates the foundation. A strong intent ("Research paper exploring AI's impact on K-12 classroom efficiency, target: school administrators, goal: provide evidence-based ROI analysis") prevents misalignment; weak intent ("Paper about AI and education") creates ambiguity.

### Key Patterns

- **SMART Success Evals Pattern**: Replace subjective criteria (vague, unmeasurable) with specific, testable outcomes. Weak: "Paper is well-written". Strong: "Paper cites 8+ peer-reviewed sources within past 10 years, uses APA formatting, 5000±500 words". Students learn to measure success through observable, countable criteria.

- **Four-Section Specification Structure**: Every specification contains Intent (what problem, why, audience) → Constraints (limits, format, scope, resources) → Success Evals (measurable done-ness) → Non-Goals (explicit out-of-scope items). This pattern applies to research papers, software features, documentation, APIs, and projects of any type.

- **Specification Checklist Pattern**: After drafting, verify with seven checks: Intent clear? Constraints specific? Success Evals SMART? Non-Goals explicit? No HOW leaked? Concise (1-2 pages)? This ensures specifications meet quality thresholds before moving to planning.

- **Audience-Aware Intent**: Intent answers "Who will use this?" and "What problem does it solve for them?" Specific audiences require specific solutions. Example: "School administrators evaluating AI adoption" differs from "Students learning about AI," creating different constraint and eval priorities.

- **Vague → Specific Transformation**: The `/sp.specify` command guides students through converting raw ideas into structured specifications. Input: "Write a paper on AI." Output: Clear 1-2 page specification with measurable success criteria and explicit scope boundaries.

### Common Mistakes

- **Leaking Implementation Into Specification**: Students write "Use Claude AI to research, outline, generate sections, refine" in the specification. This is IMPLEMENTATION (HOW). Correction: Specification states "5000-word paper with 8+ sources and 3+ AI applications"; planning describes the research and writing process.

- **Subjective Success Criteria**: Writing "Paper is high-quality," "Sources are credible," "Writing is clear," or "Reader finds it valuable" creates unmeasurable goals. Different people disagree. Correction: "Paper cites 8+ peer-reviewed sources," "Average source date is within 10 years," "APA formatting correct," "Paper word count 5000±500 words."

- **Vague Intent**: Specification states "Write a paper on AI and education" without audience, problem, or specific focus. This creates ambiguity during execution. Correction: "Analyze AI's measurable impact on K-12 classroom efficiency (teacher time savings, student engagement) for school administrators evaluating adoption."

- **Missing Constraints**: Students omit boundaries like word count, format, deadline, or resource limits. This causes scope creep and unmet expectations. Correction: List all limits explicitly (length, format, sources, deadline, visual elements, time investment).

- **Unclear Non-Goals**: Saying "No extra stuff" isn't clear. Correction: "Not a comprehensive literature review," "Not a how-to implementation guide," "No discussion of ethical concerns," "Not a quantitative study."

- **Over-Length Specifications**: Writing essay-length specifications that blur into planning. Correction: Keep specifications to 1-2 pages (concise, clear, bounded).

### Progression Context

- **Builds on**: Lesson 3 (Constitution Phase) established the global rules and principles for the project. Lesson 4 applies those principles to a specific feature. Students understand project identity before specifying individual features. Students also understand baseline communication skills from Parts 1-3.

- **Leads to**: Lesson 5 (Clarify Phase) uses the specification to ask targeted clarification questions that eliminate ambiguity. Lesson 6 (Plan Phase) takes the clarified specification and designs HOW to build it (steps, sequence, strategy). Lesson 7 (Tasks Phase) breaks the plan into executable tasks. Lesson 8 (Implement Phase) executes those tasks using the specification as validation criteria.

- **Cross-Cutting Connection**: The `/sp.specify` command is foundational to SDD-RI workflow. Every feature, chapter, lesson, and project in the later parts of the curriculum uses specifications as the starting point. Mastering this pattern now prevents rework throughout the book.

- **Capstone Integration**: Chapter 15 (Capstone) requires students to specify their own AI-native project. This lesson teaches the exact skill they'll need, applied to a concrete example (research paper) before applying it to their own choice.

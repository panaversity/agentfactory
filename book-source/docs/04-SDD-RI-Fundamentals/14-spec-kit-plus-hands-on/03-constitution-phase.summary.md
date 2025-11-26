## Constitution Phase — Project-Wide Quality Standards

This lesson teaches students how to create a Constitution using `/sp.constitution`—a document that defines immutable, testable quality standards applying to all features in a project. Unlike a Specification (which governs one feature), a Constitution establishes global rules that cascade through every downstream phase (Specification → Plan → Tasks → Implementation). Students learn to distinguish between vague aspirations ("papers should be good") and testable criteria ("all claims must cite primary sources, 50% peer-reviewed, APA format, zero plagiarism tolerance"), then practice creating a Constitution for a research paper project that will govern all papers written within that project.

## Mental Models

- **Constitution vs. Specification**: Constitution = global rules applying to ALL features in a project (e.g., "all papers must cite primary sources"); Specification = rules for ONE specific feature (e.g., "this paper explores AI development, due December 15"). Constitution written once; Specification written for each feature.

- **Quality Cascade**: A clear Constitution cascades downstream: Constitution → (ensures standards respected) → Specification → (includes quality gates) → Plan → (includes verification steps) → Tasks → (enforces constraints) → Implementation. Weak Constitution produces vague downstream work; strong Constitution produces precise downstream work.

- **Testability as Design Principle**: Standards must be testable, not subjective. "Good writing" is unprovable; "Flesch-Kincaid grade 10-12" is measurable. "Well-researched" is vague; "minimum 15 sources, 50% peer-reviewed, all claims cited" is verifiable.

- **Immutability and Leverage**: Constitution is written once per project and rarely changed, functioning as leverage—every specification and plan automatically inherits Constitution standards without re-specifying them. This eliminates repeating quality requirements for each feature.

## Key Patterns

- **Constitution Structure**: Comprises five sections: (1) Core Principles (non-negotiable values), (2) Quality Standards (testable criteria for evaluating work), (3) Technical Requirements (tools, formats, processes), (4) Constraints (limitations always applying), (5) Success Criteria (how to verify standards are met). Each section contains specific, testable statements—not vague aspirations.

- **Constitution-First Workflow**: (1) Initialize project, (2) Write Constitution once (quality standards for all features), (3) Commit Constitution to git, (4) FOR EACH FEATURE: Run `/sp.specify` → `/sp.clarify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` → commit. Constitution never rewritten per feature; Specification, Plan, Tasks rewritten for each.

- **Testability Criteria**: Replace vague standards with measurable ones. Vague: "Papers should be well-written." Testable: "Active voice 75% of time; Flesch-Kincaid grade 10-12." Vague: "Use recent sources." Testable: "Primary sources published within 10 years; seminal papers within 30 years."

- **Cascading Influence**: Constitution shapes Specification (which sources/formats are required), Plan (which verification steps must be included), Tasks (which constraints apply to work breakdown), and Implementation (which standards guide every AI interaction). Each downstream phase respects Constitution constraints without re-specifying them.

- **Review and Refinement Loop**: After `/sp.constitution` generates initial Constitution, review for: (1) Testability—are standards specific or vague? (2) Explicitness—are constraints documented or assumed? (3) Measurability—are success criteria objective or subjective? (4) Completeness—do all essential categories appear? (5) Request improvements if needed before committing.

## Common Mistakes

- **Writing Too General a Constitution**: "All papers must be good quality and well-researched." Problem: Subjective criteria cannot be verified or tested. Fix: Use testable criteria like "papers must pass plagiarism check; all claims verified against sources; Flesch-Kincaid grade 10-12; APA citations."

- **Constitution Too Specific to One Feature**: "This Constitution applies only to my AI research paper" or "AI research papers must cite these specific 5 researchers." Problem: Cannot reuse for future papers; defeats Constitution's leverage. Fix: Write Constitution general enough to apply to ANY paper in the project while still maintaining standards.

- **Forgetting to Commit Constitution**: Creating Constitution but starting `/sp.specify` without committing to git. Problem: Constitution becomes informal ("whatever I remember") instead of documented standard; no traceability. Fix: (1) Create Constitution, (2) Commit to git with clear message, (3) THEN start `/sp.specify` for specific features.

- **Confusing Constitution with Specification**: Treating Constitution as place to define specific paper details (thesis, length, deadline) when it should define general standards. Fix: Constitution specifies "word count range: 5,000-7,000" and "minimum 15 sources"; Specification specifies "this paper is 5,500 words with 18 sources."

- **Vague Success Criteria**: "Paper is well-researched" or "writing is clear." Problem: No objective way to evaluate success. Fix: Define measurable criteria like "all claims cited; minimum 50% peer-reviewed sources; zero plagiarism detected; passes fact-checking review."

## Progression Context

- **Builds on**: Chapter 13 conceptual introduction to what a Constitution is and its role in Specification-Driven Development. Students understand Constitution as foundational document before creating one.

- **Leads to**: Chapter 15 (Specification Phase) where students write a Specification for a specific feature, knowing that Specification must respect and inherit Constitution standards. Students apply Constitution principles by ensuring their Specification includes inherited quality standards without re-specifying them. Also leads to subsequent chapters on Plan, Tasks, and Implementation phases, where Constitution constraints cascade through each phase.

---
sidebar_position: 12
title: "Chapter 14: Spec-Kit Plus Hands-On Quiz"
chapter: 14
lesson: 12
duration_minutes: 20
proficiency_level: "B1"
---

# Chapter 14: Spec-Kit Plus Hands-On Quiz

Test your mastery of Spec-Kit Plus workflow. This assessment covers command recognition, artifact matching, workflow sequencing, and the P+Q+P framework through your research paper project.

<Quiz
  title="Chapter 14: Spec-Kit Plus Workflow Mastery"
  questions={[
    {
      question: "Which Spec-Kit Plus command creates project-wide quality standards that all subsequent work must satisfy?",
      options: [
        "/sp.specify — defines what you're building",
        "/sp.constitution — establishes non-negotiable project standards",
        "/sp.plan — documents how you'll build it",
        "/sp.clarify — identifies missing requirements"
      ],
      correctOption: 1,
      explanation: "The `/sp.constitution` command creates project-wide quality standards (formatting, style, citation rules, quality gates) that cascade through all phases. These standards are foundational and rarely change. `/sp.specify` defines WHAT you're building (scope, success criteria), not standards. `/sp.plan` documents HOW you'll build it (research approach, structure decisions). `/sp.clarify` refines existing specifications.",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "You're writing a research paper specification. Your first draft says 'Write a good paper about AI development.' During clarify phase, what makes this specification inadequate?",
      options: [
        "It doesn't specify which AI tools you'll use for research",
        "It lacks measurable success criteria and explicit constraints",
        "It focuses on AI development instead of a narrower topic",
        "It doesn't include the table of contents structure"
      ],
      correctOption: 1,
      explanation: "The specification lacks measurable success criteria ('good' is subjective) and constraints (page count, citation count, target audience, scope boundaries). A proper specification would say: 'Write 15-20 page research paper on AI development for computer science students, with 30+ citations, 5 main sections, demonstrating SDD-RI principles.' Tool choice belongs in planning, not specification. Topic scope might be refined but that's separate from measurability. Table of contents is an implementation detail, not specification.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "The `/sp.clarify` command identifies missing constraints in your specification. Your specification said 'Use academic sources' but didn't define academic. What should happen next?",
      options: [
        "Add the missing definition directly to the specification",
        "Accept the ambiguity since reviewers will understand 'academic'",
        "Document the ambiguity and update specification with concrete examples",
        "Skip clarification and proceed to planning phase"
      ],
      correctOption: 2,
      explanation: "The `/sp.clarify` workflow surfaces ambiguities (like 'academic sources'), then you resolve them with concrete definitions in the updated specification. Clear definition might be: 'Academic sources include peer-reviewed journals, conference proceedings, university research publications, and industry white papers.' Ambiguities not resolved now will cause planning and implementation delays. Skipping clarification compounds problems.",
      source: "Lesson 5: Clarify Phase"
    },
    {
      question: "Which file contains the completed research paper specification after `/sp.specify` command?",
      options: [
        "plan.md — documents your research and writing approach",
        "spec.md — describes what you're building with success criteria",
        "tasks.md — breaks work into atomic implementation units",
        "constitution.md — defines project-wide quality standards"
      ],
      correctOption: 1,
      explanation: "The `spec.md` file (specification) describes WHAT you're building: intent, success criteria, constraints, and non-goals. It answers 'What constitutes a completed research paper?' plan.md documents HOW you'll research and write. tasks.md breaks the work into atomic units. constitution.md defines project standards.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "Your plan.md describes research methodology: 'Review 40+ recent papers, organize by theme, synthesize findings.' When should this plan be completed?",
      options: [
        "During the specification phase when defining success criteria",
        "During the planning phase after understanding feasibility and approach",
        "During the implementation phase when actually researching papers",
        "During the clarification phase when resolving ambiguities"
      ],
      correctOption: 1,
      explanation: "The planning phase generates plan.md after you've clarified the specification and researched feasible approaches. Planning discovers constraints (only 2 weeks available, access limitations) and designs the strategy to meet spec requirements within constraints. Specification doesn't include implementation approach. Clarification refines the spec, not the plan. Implementation executes the plan, not creates it.",
      source: "Lesson 6: Plan Phase"
    },
    {
      question: "Which phase outputs define your P+Q+P (Persona, Questions, Principles) framework for a reusable 'academic-research' skill?",
      options: [
        "Specification phase — the spec IS the persona",
        "Planning phase — research approach informs persona design",
        "Tasks phase — task descriptions encode the framework",
        "Implementation phase — what you learn becomes the framework"
      ],
      correctOption: 1,
      explanation: "The planning phase generates understanding of 'how to research academically' that informs P+Q+P design. After completing the research paper project, you extract this into a reusable skill: Persona ('Academic Researcher focused on rigor and synthesis'), Questions ('What themes emerge?', 'Which sources best support each claim?'), Principles ('Prioritize peer review', 'Synthesize across sources'). The spec defines WHAT, not HOW to think. Tasks execute, they don't define frameworks. Learning happens throughout but becomes crystallized into a skill after implementation.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "During implementation, you discover that your literature search is taking 40% longer than planned. The checkpoint pattern says you should:",
      options: [
        "Accept the delay and continue without adjustment",
        "Stop and restart the entire workflow with revised timeline",
        "Document the delay and iterate with refined search strategy",
        "Skip literature review to meet original timeline"
      ],
      correctOption: 2,
      explanation: "The checkpoint pattern supports iteration: you have a working state (literature organized, some sources reviewed), so you iterate with refinement (more efficient search strategy, better database queries) while maintaining the checkpoint. This avoids losing completed work. Accepting arbitrary delays violates time constraints. Restarting wastes progress. Skipping deliverables violates the specification.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "Your constitution defines 'All citations must follow APA 7th edition.' During writing, you use Chicago style for a quote. What should happen?",
      options: [
        "Accept Chicago style since it's also an academic standard",
        "Update the constitution to allow multiple citation styles",
        "Fail the quality gate and convert to APA 7th edition",
        "Document the exception and continue writing"
      ],
      correctOption: 2,
      explanation: "The Constitution defines non-negotiable standards. If your writing violates this, you fail the quality gate and correct it—convert Chicago citations to APA 7th edition. Constitution standards cascade to all phases and catch errors before final delivery. Accepting non-compliant output violates the contract. Updating Constitution for one exception undermines its role as stable foundation. Exception documentation creates drift.",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "Arrange these Spec-Kit Plus phases in correct execution order: Tasks, Specify, Implement, Clarify, Plan",
      options: [
        "Specify → Clarify → Plan → Tasks → Implement",
        "Plan → Specify → Clarify → Tasks → Implement",
        "Clarify → Specify → Plan → Implement → Tasks",
        "Specify → Plan → Clarify → Tasks → Implement"
      ],
      correctOption: 0,
      explanation: "The standard SDD-RI workflow is: 1) Specify (define what you're building), 2) Clarify (resolve ambiguities in the spec), 3) Plan (research approach and design strategy), 4) Tasks (break work into atomic units), 5) Implement (execute tasks). This sequence is intentional: specification must be clear before planning, planning must be solid before tasking, tasks must be well-defined before implementation.",
      source: "Chapter 14: Foundation and Integration"
    },
    {
      question: "Which command-to-purpose matching is correct?",
      options: [
        "/sp.specify identifies missing constraints and edge cases",
        "/sp.clarify breaks work into atomic checkpoint-driven units",
        "/sp.plan researches feasibility and generates implementation strategy",
        "/sp.tasks generates project-wide quality standards"
      ],
      correctOption: 2,
      explanation: "`/sp.plan` researches approach feasibility and generates the implementation strategy. `/sp.specify` writes what you're building. `/sp.clarify` identifies missing constraints. `/sp.tasks` breaks work into atomic units. `/sp.constitution` (not listed) generates standards.",
      source: "Lessons 3-8: Spec-Kit Plus Phases"
    },
    {
      question: "Your research paper specification includes success criteria: 'Minimum 30 citations' and 'Cover 5 major themes.' What makes these effective success criteria?",
      options: [
        "They're ambitious goals that stretch your research effort",
        "They're measurable, objective, and provide clear pass/fail validation",
        "They're derived from academic publishing best practices",
        "They're negotiable based on research progress"
      ],
      correctOption: 1,
      explanation: "Effective success criteria are measurable ('30 citations' is countable, '5 themes' is verifiable) and objective (no subjective judgment needed). They enable automated quality gates: citation counter validates 30+, theme reviewer validates 5 major themes. Ambitious goals are motivating but not the defining feature of good criteria. Publishing standards might inform criteria but aren't the core principle. Success criteria must be non-negotiable once specified; renegotiation destroys the contract.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "After completing your research paper project, you create an 'academic-research' skill. What does the skill's Persona component primarily activate?",
      options: [
        "User profiles for which teams will use the skill",
        "AI reasoning mode with domain-specific decision frameworks",
        "Documentation for skill catalog listings",
        "Quality standards for skill implementation"
      ],
      correctOption: 1,
      explanation: "The Persona (e.g., 'Academic Researcher focused on synthesis and rigor') activates reasoning mode—when AI adopts this persona, it thinks about academic rigor, synthesis, and evidence-based claims rather than generic information gathering. Persona is a cognitive activation mechanism, not user documentation or team assignment. Quality standards belong in the skill's Principles, not Persona.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "Your tasks.md includes: 'Research foundational papers on AI development.' Why is this description useful for implementation?",
      options: [
        "It's a detailed step-by-step instruction that AI can execute directly",
        "It captures sufficient intent and context for AI and you to refine approach collaboratively",
        "It specifies exact databases and search terms to use",
        "It guarantees consistent results across multiple implementations"
      ],
      correctOption: 1,
      explanation: "Good task descriptions capture sufficient intent ('foundational papers on AI development') and context (from planning decisions about what 'foundational' means, which subtopics matter) to guide implementation without being prescriptive. They enable collaborative refinement: AI suggests databases and search terms, you confirm they fit the project. Over-specifying (option 3) creates brittle tasks; under-specifying creates ambiguity. The task doesn't guarantee results—it provides direction for intelligent execution.",
      source: "Lesson 7: Tasks Phase"
    },
    {
      question: "The `/sp.clarify` command surfaces the ambiguity: 'recent papers' in your spec. What does 'recent' mean for AI research papers?",
      options: [
        "Papers published within the last 6 months",
        "Papers that reference current LLM models",
        "Papers you haven't read yet",
        "Papers your clarification output must define specifically"
      ],
      correctOption: 3,
      explanation: "The clarification phase surfaces the ambiguity ('recent' is undefined) and requires you to define it explicitly in the updated specification. Your definition might be: 'Papers published between 2022-2025 that discuss transformer models or large language models.' This becomes non-negotiable for implementation. Options 1-3 are example definitions, not the principle—the principle is that clarification makes you define ambiguities.",
      source: "Lesson 5: Clarify Phase"
    },
    {
      question: "You've written your specification and completed clarification. Your stakeholder asks for new requirements: 'Add comparative analysis of AI frameworks.' How should this be handled?",
      options: [
        "Add it to the current specification since clarification is complete",
        "Create a separate feature specification since current scope is already defined",
        "Include it as an optional section to maintain flexibility",
        "Document it as a future enhancement and revisit after current paper is complete"
      ],
      correctOption: 1,
      explanation: "New requirements after specification are handled through separate feature specifications—not by expanding current scope. This prevents scope creep and maintains specification integrity. If comparative analysis is critical, create its own spec with its own clarification and planning. Adding it silently violates the specification contract. Optional sections and future enhancement documentation delay decisions that should be made explicitly. Clear scope boundaries enable focused delivery.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "During implementation, you write a section synthesizing three research papers. This section took longer than planned because you discovered additional nuances. The checkpoint pattern enables:",
      options: [
        "Automatic completion detection so you can move forward",
        "Resumption from your last stable state with refined approach",
        "Documentation of time estimate errors for future projects",
        "Skipping remaining sections to meet original timeline"
      ],
      correctOption: 1,
      explanation: "Checkpoints enable resumption from stable states with iterative refinement. You have a working synthesis (checkpoint), so you iterate with deeper nuance (refined approach) rather than starting over. This minimizes rework while improving quality. Automatic completion detection isn't what checkpoints do. Time tracking is secondary to capability. Skipping sections violates specification.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "What is the primary advantage of building reusable intelligence (skills) from your research paper project?",
      options: [
        "It documents what you learned for teaching purposes",
        "It enables future research projects to reuse decision frameworks without rediscovering them",
        "It creates portfolio items for job applications",
        "It ensures future papers are identical to the current one"
      ],
      correctOption: 1,
      explanation: "Reusable intelligence through skills encodes decision frameworks (how to identify foundational vs. current papers, synthesis patterns, quality validation approaches) that apply across future research projects. This is the core SDD-RI value: accumulate intelligence once, apply across projects. Skills accelerate subsequent projects because decisions are pre-made. Documentation is a side effect, not the purpose. Portfolio building is secondary. Skills don't enforce identical approaches; they provide frameworks that adapt to new contexts.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "Your specification's Non-Goals section states 'No original primary research.' A team member asks to add conducting interviews with AI researchers. What should happen?",
      options: [
        "Add interviews to the current specification as a significant enhancement",
        "Explain that Non-Goals explicitly excluded primary research; create separate feature spec if needed",
        "Include interviews only if time permits after current scope is complete",
        "Remove the Non-Goals section to allow flexibility"
      ],
      correctOption: 1,
      explanation: "Non-Goals explicitly exclude primary research. New requests for excluded work require separate feature specifications with their own planning and implementation. Non-Goals prevent scope creep by making exclusions explicit. When stakeholders request excluded features, the answer is 'that's a new feature, not part of this scope.' Including conditionally ('if time permits') creates ambiguity. Removing Non-Goals defeats their purpose.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "A colleague is starting their own research paper project. They ask: 'Should I use the 'academic-research' skill you created?' What guidance best supports their project success?",
      options: [
        "Use it directly since you built it to handle all research paper scenarios",
        "Use it as a decision framework to guide your own planning, then adapt for your specific project",
        "Skip the skill and follow the original workflow from Chapter 14 exactly",
        "Use it if you understand the Persona, Questions, and Principles driving it"
      ],
      correctOption: 3,
      explanation: "Skills are reusable decision frameworks, not rigid templates. Your colleague should use the skill if they understand the underlying Persona (what reasoning mode it activates), Questions (what decisions it guides), and Principles (what constraints it enforces). This understanding enables adaptation to their specific project. Option 2 is close but emphasizes adaptation separately from skill use. Direct reuse without understanding (option 1) violates the intelligence reuse principle. Skipping reusable intelligence wastes accumulated knowledge.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "After completing your research paper project (specification through implementation), you measure success. What demonstrates effective Spec-Kit Plus execution?",
      options: [
        "Your paper is the highest quality paper ever written",
        "Your specification criteria were met, and you understand the decision frameworks for reuse",
        "You completed the project faster than traditional writing approaches",
        "You used all six phases without skipping any steps"
      ],
      correctOption: 1,
      explanation: "Effective execution means specification criteria were met (30 citations, 5 themes, APA formatting, etc.) AND you captured reusable intelligence (decision frameworks) for future projects. Quality is relative (option 1 is unprovable). Speed varies by project (option 3). Using all phases is means, not ends (option 4). The outcome is: delivered specification + accumulated intelligence.",
      source: "Chapter 14: Integration and Learning Outcomes"
    }
  ]}
  questionsPerBatch={18}
/>

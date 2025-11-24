---
sidebar_position: 11
title: "Chapter 17: Spec-Kit Plus Hands-On Quiz"
---

# Chapter 17: Spec-Kit Plus Hands-On Quiz

Test your understanding of Spec-Driven Development with Reusable Intelligence (SDD-RI) through this comprehensive assessment covering the complete workflow from constitution to brownfield adoption.

<Quiz
  title="Chapter 17: Spec-Kit Plus Hands-On Assessment"
  questions={[
    {
      question: "Your team adopted Spec-Kit Plus but developers skip the constitution phase, arguing 'we already know our coding standards.' Six months later, specifications vary wildly in structure and quality. What does this reveal about the constitution's role?",
      options: [
        "Constitution primarily documents existing practices for reference only",
        "Constitution serves administrative purposes but doesn't affect technical work",
        "Constitution creates shared decision frameworks preventing drift over time",
        "Constitution replaces code reviews by automating quality checks automatically"
      ],
      correctOption: 2,
      explanation: "The constitution creates shared decision frameworks that prevent drift over time, which is exactly what happened when it was skipped. It's not merely documentation of existing practices—it actively shapes how teams make decisions through principles and validation criteria. Another option is incorrect because the constitution directly affects technical work by establishing how specifications are written, what quality means, and how decisions are made. Another option misunderstands the constitution's role entirely; it doesn't replace code reviews or automate checks, but rather provides the reasoning frameworks that guide both specification writing and review processes. The scenario demonstrates that without constitutional grounding, teams converge toward inconsistent patterns despite shared coding standards, because they lack the decision-making frameworks that keep specifications aligned.",
      source: "Lesson 1: Spec-Kit Plus Foundation"
    },
    {
      question: "A developer creates a detailed specification with clear acceptance criteria but receives feedback that it 'lacks constitutional alignment.' Reviewing the constitution reveals principles emphasizing progressive disclosure and minimal viable scope. What should the developer examine first?",
      options: [
        "Whether specification includes all possible edge cases and scenarios",
        "Whether specification starts simple and adds complexity incrementally only",
        "Whether specification uses correct markdown formatting throughout the document",
        "Whether specification matches the exact template structure precisely"
      ],
      correctOption: 1,
      explanation: "Constitutional principles about progressive disclosure and minimal viable scope directly point to examining whether the specification starts simple and adds complexity incrementally. Another option is actually the problem—including 'all possible edge cases' violates minimal viable scope and progressive disclosure principles. Another option misses the point; constitutional alignment concerns decision-making frameworks, not formatting details. Another option confuses structural compliance with constitutional alignment; templates provide structure, but constitutions provide reasoning principles. The feedback specifically mentioned constitutional principles (progressive disclosure, minimal viable scope), which means the specification likely tried to solve everything at once instead of defining a minimal first iteration with clear extension points. Constitutional alignment means applying the decision frameworks encoded in principles, not just following structural templates.",
      source: "Lesson 1: Spec-Kit Plus Foundation"
    },
    {
      question: "During Spec-Kit Plus installation, a team debates whether to use the default directory structure (.specify/) or customize it to match their existing project layout. What factor should drive this decision?",
      options: [
        "Team size determines structure complexity and customization needs always",
        "Default structure ensures tool compatibility and onboarding consistency primarily",
        "Custom structure prevents conflicts with legacy codebases automatically",
        "Directory location affects specification quality and completeness significantly"
      ],
      correctOption: 1,
      explanation: "The default structure ensures tool compatibility and onboarding consistency, which are the primary practical considerations. Spec-Kit Plus tools and workflows expect the .specify/ structure, and using it means new team members immediately recognize the pattern from documentation and other projects. Another option incorrectly assumes team size determines structure needs; even small teams benefit from standard structures for onboarding and tooling. Another option misunderstands the issue; customizing directory structure doesn't prevent legacy conflicts—those are addressed through .gitignore and project organization, not specification directory naming. Another option is incorrect because directory location is purely organizational; specification quality depends on content and process, not where files are stored. While customization is possible, the default structure provides immediate value through tool integration and pattern recognition.",
      source: "Lesson 2: Installation and Setup"
    },
    {
      question: "A project has Spec-Kit Plus installed, but git status shows .specify/ directory contents are being tracked when they should be temporary working files. What setup step was likely missed?",
      options: [
        "Adding .specify/ patterns to .gitignore to exclude working files",
        "Running npm install to configure package dependencies correctly",
        "Setting file permissions to restrict write access appropriately",
        "Creating symbolic links to shared specification templates properly"
      ],
      correctOption: 0,
      explanation: "Adding .specify/ patterns to .gitignore is the correct setup step to exclude temporary working files from version control. The .specify/ directory contains both permanent artifacts (like constitution.md) and temporary working files (like current feature specs, plans, tasks). Another option (npm install) handles dependency installation but doesn't affect git tracking—this is a version control configuration issue, not a package dependency issue. Another option is incorrect because file permissions control who can read/write files, not whether git tracks them; even read-only files are tracked if not in .gitignore. Another option misunderstands the directory structure; Spec-Kit Plus doesn't use symbolic links to templates—templates are copied or generated, and the issue is about version control, not file references. The lesson emphasizes configuring .gitignore properly to track only the artifacts that should be shared (constitution, completed ADRs) while excluding working files.",
      source: "Lesson 2: Installation and Setup"
    },
    {
      question: "Your project uses Spec-Kit Plus, and a new developer asks, 'Why do we need both a constitution AND coding standards?' How should you explain the distinction?",
      options: [
        "Constitution provides decision frameworks; coding standards define style conventions",
        "Constitution defines project-specific rules; coding standards define language syntax",
        "Constitution documents team agreements; coding standards enforce automated checks",
        "Constitution targets architects only; coding standards target all developers"
      ],
      correctOption: 0,
      explanation: "The constitution provides decision frameworks (how to think about problems), while coding standards define style conventions (how to format code). This is the fundamental distinction. Another option is incorrect because both can be project-specific, and coding standards cover far more than syntax—they include naming, organization, and patterns. The key difference isn't scope but purpose: decision-making versus formatting. Another option partially captures their different enforcement mechanisms but misses the core distinction; both document agreements, and both can have automated checks. The real difference is that constitutions guide reasoning (when to use patterns, how to evaluate trade-offs), while coding standards eliminate trivial decisions (tabs vs spaces, naming formats). Another option is wrong because constitutions target everyone who makes architectural decisions (including senior developers) and coding standards also target all developers. The lesson shows constitutions answering 'how do we decide?' while standards answer 'how do we format?'",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "A team creates a constitution with 23 principles covering every possible decision scenario they can imagine. Three months later, developers rarely reference it. What principle of constitutional design was violated?",
      options: [
        "Constitutions must cover all scenarios to provide complete guidance",
        "Constitutions need regular updates to stay current with technology",
        "Constitutions require legal review to ensure enforceability properly",
        "Constitutions should focus on core frameworks that apply broadly"
      ],
      correctOption: 3,
      explanation: "Constitutions should focus on core frameworks that apply broadly—typically 5-7 principles maximum. When constitutions try to cover every scenario, they become reference manuals that developers ignore because they're too detailed to internalize. The lesson emphasizes that constitutional principles should be memorable and applicable across many decisions, not exhaustive catalogs. Another option completely misunderstands the purpose; project constitutions aren't legal documents requiring legal review—they're shared decision frameworks for technical work. Another option misses the actual problem; the constitution failed because it had too many principles, not because technology changed. Technology-agnostic principles (like 'progressive disclosure' or 'explicit over implicit') remain relevant across tech shifts. The scenario shows a classic failure mode: teams create comprehensive constitutions that are technically correct but practically useless because no one can remember 23 principles when making daily decisions.",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "During constitution creation, a team debates whether to include 'Favor immutability in state management' as a principle. One developer argues this is too implementation-specific. How should this be resolved using constitutional thinking?",
      options: [
        "Include it because specific guidance prevents architectural drift always",
        "Exclude it because constitutions must avoid all technical details",
        "Reframe it as broader principle about predictability and side effects",
        "Create separate technical addendum for implementation-specific rules only"
      ],
      correctOption: 2,
      explanation: "Reframing as a broader principle about predictability and side effects makes it constitutional rather than implementation-specific. Good constitutional principles transcend specific technologies—'Favor predictable state changes with minimal side effects' applies whether you're using React, Vue, or backend services, while 'favor immutability' assumes specific language features. Another option is incorrect because specificity actually increases drift risk; when technologies change, specific principles become obsolete and ignored. Another option goes too far; constitutions can address technical concerns, but they should do so through frameworks, not prescriptions. Another option misses the point entirely—creating separate addendums defeats the purpose of having a single constitutional source of truth. The lesson shows that constitutional principles should be reusable across contexts; the underlying concern (predictability) is constitutional, while the implementation technique (immutability) is contextual.",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "You're writing a specification for a user authentication feature. The constitution emphasizes 'explicit over implicit.' How does this principle guide specification decisions?",
      options: [
        "State security requirements explicitly rather than assuming HTTPS defaults",
        "List all authentication methods even if some are standard",
        "Document every function signature with complete parameter definitions",
        "Specify exact line counts for each implementation file precisely"
      ],
      correctOption: 0,
      explanation: "Stating security requirements explicitly rather than assuming HTTPS defaults directly applies 'explicit over implicit' at the specification level. Specifications shouldn't assume developers will infer security requirements—they should state them clearly. Another option misapplies the principle; listing standard authentication methods adds no value if they're already implied by the chosen framework—this is being exhaustive, not explicit. Another option confuses specification with implementation; 'explicit over implicit' at the spec level means stating requirements and constraints, not dictating implementation details like function signatures. Another option completely misunderstands specification purpose; line counts are implementation details that violate specification abstraction. The constitutional principle guides what must be stated versus what can be assumed: state security requirements explicitly, but let implementation details remain implicit until the implementation phase.",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "A constitution includes the principle 'Optimize for change.' During specification review, someone flags a spec for violating this principle. What should reviewers look for?",
      options: [
        "Whether specification uses latest framework versions and modern patterns",
        "Whether specification includes automated testing for all components",
        "Whether specification isolates likely changes from unlikely changes clearly",
        "Whether specification follows agile methodology and sprint planning"
      ],
      correctOption: 2,
      explanation: "Isolating likely changes from unlikely changes clearly is how 'optimize for change' manifests in specifications. This means identifying extension points, defining stable interfaces, and separating core requirements from optional enhancements. Another option confuses change optimization with technology currency; using the latest framework doesn't optimize for change—it might even increase change risk if the framework is unstable. Another option addresses quality but not changeability; testing verifies correctness, but 'optimize for change' is about architectural flexibility. Another option completely misunderstands the principle; 'optimize for change' is an architectural guideline about designing for evolution, not a process methodology like agile. The lesson shows that constitutional principles guide architectural thinking: specifications optimized for change clearly distinguish core from periphery, stable from volatile, and required from optional.",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "Two teams use the same Spec-Kit Plus installation but develop constitutions with conflicting principles. Team A prioritizes 'comprehensive documentation,' while Team B prioritizes 'minimal sufficient documentation.' What does this reveal about constitutional design?",
      options: [
        "One team misunderstands Spec-Kit Plus and requires retraining",
        "Constitutions encode team-specific values and context appropriately",
        "Conflicting principles indicate installation errors needing correction",
        "Teams should merge constitutions to ensure organizational consistency"
      ],
      correctOption: 1,
      explanation: "Constitutions encode team-specific values and context appropriately—this is by design, not a problem. Different teams have different risk profiles, regulatory requirements, and maintenance contexts that justify different documentation strategies. Another option assumes there's one 'correct' way to use Spec-Kit Plus, but the framework explicitly supports diverse constitutional choices because different contexts demand different trade-offs. Another option completely misunderstands; this isn't a technical installation issue—it's teams making different architectural decisions based on their contexts. Another option assumes organizational consistency requires identical principles, but this violates the purpose of constitutions; a highly regulated team (comprehensive documentation) and an experimental team (minimal documentation) should have different constitutions reflecting their different constraints. The lesson emphasizes that constitutions are contextual, not universal—they should reflect each team's specific values, risks, and constraints.",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "When starting a specification, you write a detailed 10-page document covering architecture, implementation details, test strategies, and deployment procedures. During review, it's rejected for 'specification scope creep.' What should the specify phase primarily focus on?",
      options: [
        "Complete technical documentation covering architecture through deployment comprehensively",
        "High-level overview with links to separate detailed documents",
        "Intent, success criteria, constraints, and non-goals without implementation details",
        "User stories and acceptance tests using behavior-driven format"
      ],
      correctOption: 2,
      explanation: "The specify phase focuses on intent, success criteria, constraints, and non-goals—clarifying the problem and desired outcome without prescribing implementation. This is the core of specification thinking: what we're solving and how we'll know we've succeeded, not how we'll build it. Another option describes comprehensive documentation that mixes specification with implementation, architecture, and operations—this is exactly the 'scope creep' the review identified. Another option suggests splitting the specification into multiple documents, but the issue isn't document length—it's that implementation details don't belong in specifications at all. Another option confuses specification format with specification content; you can use user stories if helpful, but they don't change the fundamental purpose: defining intent and success criteria, not describing implementation. The lesson emphasizes that specifications answer 'what and why' while leaving 'how' to the planning and implementation phases.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "You're specifying a feature to 'improve application performance.' The specification template asks for success criteria. You write: 'Application should be fast enough for users.' What makes this success criterion inadequate?",
      options: [
        "It focuses on performance instead of user experience",
        "It lacks specific measurable thresholds and validation methods",
        "It doesn't reference specific performance testing tools",
        "It uses subjective language instead of technical terms"
      ],
      correctOption: 1,
      explanation: "The criterion lacks specific measurable thresholds and validation methods—'fast enough' is unmeasurable and untestable. Good success criteria are specific (what metric), measurable (what value), and verifiable (how to test). Another option misunderstands the issue; the problem isn't that it focuses on performance, but that 'fast enough' is undefined. A better criterion might be 'page load completes within 2 seconds for 95th percentile users measured via RUM.' one option confuses specification with implementation; success criteria should define the measurable outcome, not prescribe the tools used to measure it. Another option partially identifies the problem (subjective language) but misses the deeper issue; the real problem is unmeasurability, not tone. Even technical language can be vague ('low latency') without specific thresholds. The lesson emphasizes that success criteria must be falsifiable—someone should be able to definitively say whether they're met.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "A specification includes both a 'Goals' section and a 'Non-Goals' section. A reviewer suggests removing 'Non-Goals' to shorten the document. What value does the Non-Goals section provide that justifies keeping it?",
      options: [
        "Non-goals demonstrate thoroughness and attention to detail primarily",
        "Non-goals prevent scope creep by explicitly excluding related work",
        "Non-goals satisfy template requirements for complete specifications",
        "Non-goals provide future roadmap items for subsequent iterations"
      ],
      correctOption: 1,
      explanation: "Non-goals prevent scope creep by explicitly excluding related work that stakeholders might otherwise assume is included. When you specify 'user authentication,' stakeholders might assume you're also building user management, permissions, and audit logging unless you explicitly exclude them. Another option misses the functional value; non-goals aren't about demonstrating thoroughness—they actively prevent misunderstandings by making exclusions explicit. Another option suggests non-goals are merely ceremonial, but they serve a critical communication function: setting boundaries. Another option confuses non-goals with future work; non-goals are deliberately excluded from current scope (and possibly forever), while future work is deferred but intended. The lesson shows that non-goals answer the question 'what are we NOT building?' which is often as important as 'what are we building?' for aligning stakeholder expectations and preventing scope expansion.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "During specification review, stakeholders disagree about whether 'real-time notifications' is in scope. The specification doesn't mention notifications. What specification practice would have prevented this ambiguity?",
      options: [
        "Explicitly listing related features as non-goals when applicable",
        "Including comprehensive feature lists covering all possible functionality",
        "Adding detailed user stories for every interaction pattern",
        "Creating separate specifications for each technical component individually"
      ],
      correctOption: 0,
      explanation: "Explicitly listing 'real-time notifications' as a non-goal would have prevented this ambiguity by making the exclusion clear upfront. The lesson emphasizes that silence in a specification is ambiguous—stakeholders don't know if something was forgotten or deliberately excluded. Another option is impractical and misses the point; you can't list 'all possible functionality,' and the issue here is about what's excluded, not included. Another option confuses granularity with clarity; detailed user stories don't resolve scope boundaries—they describe in-scope features in more detail. Another option creates organizational overhead without solving the problem; splitting specifications doesn't clarify whether notifications are in or out of scope. The non-goals section exists specifically to resolve this type of ambiguity by making exclusions explicit rather than implicit.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "A specification defines constraints as: 'Must use Python. Must be maintainable. Must follow best practices.' A reviewer asks for constraint revision. What makes these constraints insufficient?",
      options: [
        "Constraints should be recommendations, not absolute requirements always",
        "Constraints should avoid technology choices to preserve flexibility",
        "Constraints must include detailed justifications for each restriction",
        "Constraints mix genuine limitations with vague quality aspirations incorrectly"
      ],
      correctOption: 3,
      explanation: "These constraints mix genuine limitations ('Must use Python'—a real technical constraint) with vague quality aspirations ('must be maintainable,' 'follow best practices'—these are goals, not constraints). Constraints are external limitations that restrict solution space, like technology requirements, budget limits, or compliance mandates. Another option is incorrect; constraints often are absolute requirements—that's what makes them constraints rather than preferences. Another option adds unnecessary burden; while justifications can be helpful, the core problem is that 'maintainable' and 'best practices' aren't constraints—they're quality attributes that should be addressed through success criteria, not constraints. Another option goes too far; technology choices can be legitimate constraints when dictated by existing systems, team skills, or organizational standards. The lesson distinguishes between constraints (external limitations), success criteria (measurable outcomes), and goals (desired results)—mixing them creates confusion.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "After completing a specification, you run the clarify phase and receive five targeted questions about ambiguities. You feel defensive because you thought the specification was clear. What does this reaction misunderstand about the clarify phase?",
      options: [
        "Clarify phase identifies specification quality issues requiring defensive responses",
        "Clarify phase generates questions to demonstrate reviewer diligence thoroughly",
        "Clarify phase tests whether specifications meet minimum completeness standards",
        "Clarify phase reveals implicit assumptions that seemed obvious to authors"
      ],
      correctOption: 3,
      explanation: "The clarify phase reveals implicit assumptions that seemed obvious to specification authors but aren't clear to others. This is the entire point—authors are too close to their work to see what they've left implicit. Another option frames clarification as criticism, but it's actually collaboration; questions aren't attacking specification quality, they're surfacing hidden assumptions before they cause implementation problems. Another option misunderstands the purpose; clarify isn't a quality gate testing 'minimum standards'—it's a collaborative process that makes implicit knowledge explicit. Another option cynically suggests questions are performative, but good clarification questions reveal genuine ambiguities that the author didn't see. The lesson emphasizes that the author's curse (knowing what you meant) makes it impossible to see your own ambiguities—clarification questions are a gift, not an attack, because they prevent downstream misunderstandings.",
      source: "Lesson 5: Clarify Phase"
    },
    {
      question: "The clarify phase generates eight questions, but you can answer all of them verbally in five minutes. The process requires encoding answers back into the specification. Why is this encoding step necessary rather than just documenting Q&A?",
      options: [
        "Encoding ensures specification becomes single source of truth eliminating ambiguity",
        "Encoding creates audit trail showing all clarification discussions comprehensively",
        "Encoding demonstrates compliance with specification methodology requirements fully",
        "Encoding prevents future clarification phases by addressing all questions"
      ],
      correctOption: 0,
      explanation: "Encoding ensures the specification becomes the single source of truth, eliminating the need to cross-reference Q&A discussions to understand intent. If answers remain in separate Q&A documents, readers must consult multiple sources and potentially encounter contradictions. Another option describes a side benefit (audit trail exists in version control) but misses the primary purpose: making the specification self-contained. Another option treats encoding as bureaucratic compliance, but it serves a functional purpose: future readers shouldn't need to reconstruct your reasoning from meeting notes. Another option is wrong on two counts: encoding doesn't prevent future clarifications (new questions will arise), and trying to address 'all questions' is impossible—specifications are refined iteratively. The lesson emphasizes that verbal answers are ephemeral and fragmented; encoding them into the specification centralizes knowledge and ensures everyone works from the same understanding.",
      source: "Lesson 5: Clarify Phase"
    },
    {
      question: "During clarification, someone asks: 'Should the notification system support SMS?' You realize this is a scope question, not a clarification. How should this be handled?",
      options: [
        "Answer the question and encode it into specification's features section",
        "Defer the question until implementation phase when details emerge",
        "Redirect to non-goals section or scope discussion as appropriate",
        "Flag as out-of-scope and document in separate requirements backlog"
      ],
      correctOption: 2,
      explanation: "Redirecting to the non-goals section or scope discussion is appropriate because this is a scope boundary question, not a clarification of existing specification content. If SMS is out of scope, it belongs in non-goals; if it's uncertain, it needs scope discussion with stakeholders before specification can be clarified. Another option treats scope expansion as clarification, but clarify phase shouldn't change scope—it should clarify already-defined scope. Adding SMS would be scope creep if it wasn't in the original specification. Another option defers the decision too long; scope must be clear before implementation begins, not discovered during implementation. Another option creates fragmentation by maintaining separate requirement backlogs; if SMS is in scope, it belongs in the specification; if it's out, it belongs in non-goals. The lesson distinguishes between clarification (making existing intent clearer) and scope negotiation (changing what's in scope)—they're different conversations.",
      source: "Lesson 5: Clarify Phase"
    },
    {
      question: "A clarification question asks: 'What database should we use?' You're tempted to answer 'PostgreSQL,' but the specification template suggests deferring this. Why would database choice be deferred from the specify phase?",
      options: [
        "Database choice should be made by implementation team",
        "Database choice requires performance testing not available during specification",
        "Database choice is implementation detail decided during planning phase",
        "Database choice isn't relevant to specification success criteria"
      ],
      correctOption: 2,
      explanation: "Database choice is an implementation detail decided during the planning phase after requirements are clear. The specification defines what data must be stored, queried, and how (consistency requirements, performance targets), but the specific database is a solution choice based on those requirements. Another option misidentifies the reason; you don't need performance testing to specify—you need clear requirements that planning can evaluate different databases against. Another option partially captures the workflow but misses the conceptual point; it's not about who decides, but when: implementation details are decided during planning, not specification. Another option is wrong because database choice is very relevant to success criteria (performance, scalability, consistency), but the specification defines those criteria without prescribing the database. The lesson shows the specification/planning boundary: specifications describe constraints and success criteria; planning evaluates solutions against those criteria and chooses implementations.",
      source: "Lesson 5: Clarify Phase"
    },
    {
      question: "After three clarification rounds, some questions remain unresolved because stakeholders disagree. What does this signal about moving to the planning phase?",
      options: [
        "Planning can proceed for agreed scope while unresolved parts stay",
        "Planning should be blocked until all stakeholder disagreements resolve completely",
        "Planning should begin because unresolved questions will clarify during implementation",
        "Planning phase includes conflict resolution so disagreements aren't blockers"
      ],
      correctOption: 0,
      explanation: "Planning can proceed for agreed scope while unresolved parts stay in specification—you don't need complete unanimity to start planning what is agreed. This allows progress on clear areas while stakeholders resolve disagreements on uncertain areas. Another option is dangerous; unresolved specification questions don't 'clarify during implementation'—they cause rework because implementers make different assumptions. Another option creates unnecessary blocking; requiring complete resolution before any planning begins wastes time when 80% of scope is clear. Another option misunderstands phase purposes; planning evaluates how to build what's specified, not what should be specified—conflict resolution about requirements belongs in the specify/clarify phases. The lesson shows that specifications can be partially complete if unresolved areas are clearly marked, allowing parallel progress on stable areas while unstable areas are finalized.",
      source: "Lesson 5: Clarify Phase"
    },
    {
      question: "During planning, you create a detailed 50-task breakdown with dependencies, estimates, and assigned owners. Review feedback says the plan 'over-constrains implementation.' What aspect of planning might be too prescriptive?",
      options: [
        "Including time estimates for each individual task",
        "Breaking work into small tasks instead of larger chunks",
        "Specifying exact implementation sequence for all dependent tasks",
        "Assigning specific developers to tasks before implementation starts"
      ],
      correctOption: 3,
      explanation: "Assigning specific developers to tasks before implementation starts over-constrains because it removes flexibility for the team to self-organize based on emerging context, availability, and expertise. Plans should define what needs to be done and in what order, but who does each task should be determined during implementation. Another option is actually good practice; small tasks are more predictable and easier to track than large chunks. Another option might seem constraining, but defining implementation sequence for dependent tasks is necessary—you can't build the UI before the API exists. Another option provides useful information for prioritization and risk assessment without over-constraining; estimates inform decisions but don't dictate how work happens. The lesson distinguishes between planning (defining work structure and dependencies) and work assignment (deciding who does what), emphasizing that plans should enable rather than constrain team self-organization.",
      source: "Lesson 6: Plan Phase"
    },
    {
      question: "A plan lists 30 tasks with no indicated dependencies or ordering. A reviewer asks: 'What's the critical path?' You can't answer. What planning element is missing?",
      options: [
        "Task priority rankings showing business value ordering",
        "Task dependency relationships showing sequencing constraints clearly",
        "Task effort estimates showing relative complexity comparisons",
        "Task risk assessments showing probability and impact"
      ],
      correctOption: 1,
      explanation: "Task dependency relationships showing sequencing constraints are missing—the critical path is the longest sequence of dependent tasks, which can't be identified without dependency information. Dependencies answer 'what must be done before what?' which reveals the critical path. Another option (priorities) is useful but doesn't define critical path; you might prioritize user-facing features highest, but critical path is about technical dependencies, not business value. Another option (estimates) contributes to critical path analysis by showing how long each sequence takes, but without dependencies, you don't know which tasks are sequential versus parallel. Another option (risk) is important for planning but unrelated to critical path; critical path is determined by dependencies and duration, not risk. The lesson emphasizes that plans must capture dependencies to enable resource allocation, risk identification, and timeline estimation—without dependencies, a plan is just a list of tasks.",
      source: "Lesson 6: Plan Phase"
    },
    {
      question: "Your plan includes 'Implement user authentication' as a single task. During review, you're asked to break it down further. What principle of task breakdown does this serve?",
      options: [
        "Smaller tasks create comprehensive documentation for future reference",
        "Smaller tasks ensure every developer has exactly equal workload",
        "Smaller tasks enable better progress tracking and risk identification",
        "Smaller tasks allow managers to monitor developer productivity closely"
      ],
      correctOption: 2,
      explanation: "Smaller tasks enable better progress tracking (you can see what's actually done versus vaguely 'in progress') and risk identification (you discover blockers earlier when tasks are granular). 'Implement authentication' might take two weeks and hide dependencies like 'set up OAuth provider,' 'implement token validation,' 'create user session management,' and 'add password reset flow.' one option misunderstands the goal; task breakdown isn't about equal workload distribution—it's about visibility and predictability. Equal workload comes from good assignment, not task size. Another option treats task breakdown as documentation exercise, but the real value is operational: can you track progress and identify blockers? one option frames task breakdown as surveillance, but the actual purpose is enabling the team to identify and resolve blockers early. The lesson shows that task granularity enables adaptive planning—you can't adjust course if you can't see where you are.",
      source: "Lesson 6: Plan Phase"
    },
    {
      question: "A plan includes both 'must-have' and 'nice-to-have' tasks, but they're intermixed in the task list. A reviewer suggests grouping them. Why does task categorization by priority improve plan usability?",
      options: [
        "Categorization creates visual organization making plans easier to read",
        "Categorization allows automatic task scheduling by priority level",
        "Categorization demonstrates planning thoroughness and attention to detail",
        "Categorization enables incremental delivery and scope negotiation explicitly"
      ],
      correctOption: 3,
      explanation: "Categorization enables incremental delivery (ship must-haves first) and scope negotiation (if timeline is tight, drop nice-to-haves). Making priority explicit in plan structure communicates what can flex and what can't. Another option identifies a side benefit (readability) but misses the strategic value: prioritization enables decision-making under constraints. Another option treats categorization as cosmetic demonstration of thoroughness, but it serves a functional purpose: guiding trade-off decisions when reality diverges from the plan. Another option misunderstands how task scheduling works; automatic scheduling by priority alone ignores dependencies, which are more important for sequencing. The lesson emphasizes that plans should support adaptive execution—when you're halfway through the timeline, you need to know what's essential versus optional to make informed trade-offs.",
      source: "Lesson 6: Plan Phase"
    },
    {
      question: "During planning, you identify a complex task with high uncertainty. The plan template has a 'risks' section. What makes a good risk entry versus just noting the uncertainty?",
      options: [
        "Good risks list all possible failure modes comprehensively",
        "Good risks reference historical project failures for context",
        "Good risks assign probability percentages to each scenario",
        "Good risks include mitigation strategies and trigger conditions"
      ],
      correctOption: 3,
      explanation: "Good risks include mitigation strategies (what you'll do to reduce likelihood or impact) and trigger conditions (what signals will tell you the risk is materializing). Just noting 'this is risky' doesn't help; you need actionable information. Another option creates overwhelming risk lists that don't guide action; good risk management focuses on significant risks with clear mitigations, not exhaustive possibility catalogs. Another option (probability percentages) adds false precision; most software risks don't have statistical probabilities—'high/medium/low' is usually sufficient. What matters is: what will we do about it? one option provides historical context but doesn't address the current risk; learning from past failures is valuable, but risk entries need forward-looking mitigation plans. The lesson shows that risk management is about preparedness and early detection, not just identification—your plan should answer 'how will we reduce this risk and how will we know if it's happening?'",
      source: "Lesson 6: Plan Phase"
    },
    {
      question: "You're converting a plan into a task list. The plan has 15 high-level tasks, but the task template asks for more granular entries. What level of granularity makes tasks actionable?",
      options: [
        "Tasks should be completable in one uninterrupted sitting ideally",
        "Tasks should align with git commit boundaries precisely",
        "Tasks should represent roughly one day of work typically",
        "Tasks should match specification success criteria exactly"
      ],
      correctOption: 2,
      explanation: "Tasks should represent roughly one day of work—small enough to show progress and identify blockers, but large enough to have meaningful completeness criteria. This 'one day' heuristic balances granularity with overhead. Another option (one sitting) is too granular; tasks like 'write function signature' or 'add import statement' create tracking overhead without adding value. Another option confuses task granularity with version control practices; tasks define logical work units, while commits define stable checkpoints—they're related but not identical. You might commit multiple times while completing one task, or complete one task in a single commit. Another option misunderstands the relationship between specifications and tasks; success criteria define when the entire feature is done, while tasks define incremental implementation steps. The lesson emphasizes practical actionability: can someone pick up this task, complete it in a day, and definitively mark it done?",
      source: "Lesson 7: Tasks Phase"
    },
    {
      question: "A task is written as: 'Make the authentication system work correctly.' During review, it's flagged as inadequate. What makes this task definition insufficient?",
      options: [
        "It doesn't assign a responsible developer or owner",
        "It doesn't specify which authentication method to use",
        "It lacks specific completion criteria and scope boundaries",
        "It doesn't include estimated hours for completion"
      ],
      correctOption: 2,
      explanation: "The task lacks specific completion criteria ('work correctly' is unmeasurable) and scope boundaries (what does 'authentication system' include?). Good tasks have clear done conditions: 'User can log in with email/password and receive JWT token that validates against protected endpoints.' one option confuses task definition with technical design; which authentication method to use should be determined in planning—tasks execute the plan, they don't make architectural decisions. Another option addresses task assignment, but that's separate from task definition; a well-defined task can be unassigned initially. Another option focuses on estimation, which is useful but not what makes a task actionable; you could estimate the vague task at 8 hours, but that doesn't clarify what 'work correctly' means. The lesson emphasizes that task definitions must be unambiguous: someone should be able to pick up the task, complete it, and definitively demonstrate it's done.",
      source: "Lesson 7: Tasks Phase"
    },
    {
      question: "Your task list includes: 'Task 12: Implement user profile update. Depends on: Tasks 3, 7, 11.' During standup, someone says they finished Tasks 3 and 7. Can they start Task 12?",
      options: [
        "Yes, because majority of dependencies are complete",
        "No, because all listed dependencies must complete first",
        "Yes, if Task 11 is in progress and nearing completion",
        "No, unless tasks are reassigned to balance workload"
      ],
      correctOption: 1,
      explanation: "No, all listed dependencies must complete first—that's what dependency means. Task 12 depends on 3, 7, AND 11, so starting Task 12 before Task 11 completes will likely cause rework or blocking. Another option misunderstands dependencies; they're not voting mechanisms where majority rules—each dependency represents a genuine prerequisite. Another option suggests starting dependent work before dependencies complete if they're 'nearing completion,' but this creates risk; if Task 11 encounters issues or changes direction, Task 12 might need rework. Another option introduces unrelated concerns (workload balancing); while workload balance matters, it doesn't override dependency constraints. The lesson shows that dependency relationships define the partial ordering of work—you can't safely skip them without risking rework.",
      source: "Lesson 7: Tasks Phase"
    },
    {
      question: "While implementing Task 15, you discover that Task 8 (already marked complete) needs modification. How should this be handled in task tracking?",
      options: [
        "Modify Task 8 directly and update its completion date",
        "Create new task for the modification referencing Task 8",
        "Reopen Task 8 and mark it incomplete until fixed",
        "Document the change in Task 15 without tracking separately"
      ],
      correctOption: 1,
      explanation: "Creating a new task for the modification (referencing Task 8) preserves the history of what was done when, and makes the additional work visible. Task 8 was completed according to its original definition; the new requirement is separate work. Another option (modifying Task 8) rewrites history and hides the fact that additional work was needed—this makes it harder to understand why estimates were wrong or where scope expanded. Another option (reopening Task 8) treats the modification as incomplete original work, but if Task 8 met its original criteria, reopening it is inaccurate. Another option (documenting in Task 15) hides the modification work; if someone reviews Task 8 later, they won't see that it was modified. The lesson emphasizes that task tracking provides historical record for learning—when 'completed' tasks need changes, that's valuable information about specification accuracy or planning quality that should be visible, not hidden.",
      source: "Lesson 7: Tasks Phase"
    },
    {
      question: "You're using a task tracking system that allows tasks to be 'blocked.' Task 18 is blocked waiting for API documentation. How does explicitly marking tasks 'blocked' improve workflow visibility?",
      options: [
        "Blocked status prevents developers from working on incomplete tasks",
        "Blocked status generates reports for management oversight purposes",
        "Blocked status automatically reassigns tasks to available developers",
        "Blocked status surfaces dependencies that need external action explicitly"
      ],
      correctOption: 3,
      explanation: "Blocked status surfaces dependencies that need external action—in this case, someone needs to provide API documentation. Making blocks explicit enables the team to escalate and resolve them rather than having developers silently wait. Another option misunderstands the purpose; 'blocked' doesn't prevent work—developers already can't work on it because dependencies aren't ready. The status makes the problem visible. Another option describes automation that doesn't exist in most systems; blocked status is about visibility, not automatic task routing. Another option treats blocking as reporting mechanism for management, but the real value is operational: the team sees where they're stuck and can take action. The lesson shows that workflow visibility enables active problem-solving—if blocked tasks are invisible, blockers persist; when visible, the team can work to remove them.",
      source: "Lesson 7: Tasks Phase"
    },
    {
      question: "During implementation, you're following the checkpoint pattern. After completing the authentication flow, you commit the code but tests are failing. What does the checkpoint pattern say about this situation?",
      options: [
        "Commit anyway because checkpoints capture progress regardless of state",
        "Document test failures in commit message and proceed",
        "Skip checkpoint and continue to next feature until tests pass",
        "Fix tests before committing because checkpoints must be stable"
      ],
      correctOption: 3,
      explanation: "Checkpoints must be stable—fix tests before committing because each checkpoint should be a working state you can return to. The checkpoint pattern creates safety through stable states, not just arbitrary progress markers. Another option misunderstands checkpoints; capturing 'progress' with failing tests creates a broken checkpoint that can't be used as a rollback point. Another option (skipping the checkpoint) means you lose the safety net—if the next feature introduces problems, you don't have a stable state to return to. Another option (documenting test failures) acknowledges the problem but doesn't fix it; committing with known test failures means your checkpoint is broken. The lesson emphasizes that checkpoints are stable milestones: all tests pass, functionality works, code is clean. This discipline ensures you can always return to a working state, which is the entire point of checkpoints.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "You complete a complex feature and want to create a checkpoint. The code works but has extensive TODO comments and some duplicated logic. Should you checkpoint now or refactor first?",
      options: [
        "Checkpoint now because checkpoints capture completion not quality",
        "Refactor first because checkpoints should represent production-ready code",
        "Checkpoint now then refactor in subsequent checkpoint for clarity",
        "Skip checkpoint because complex features need multiple incremental commits"
      ],
      correctOption: 2,
      explanation: "Checkpoint now (capturing working functionality) then refactor in a subsequent checkpoint—this creates two stable states and preserves the ability to roll back refactoring if it introduces issues. The first checkpoint proves the feature works; the second improves code quality. Another option is partially correct (checkpoints don't require perfection) but misses the strategic value of separating functional completion from quality improvement. Another option sets too high a bar; requiring production-ready code for every checkpoint means you checkpoint less frequently, reducing safety. Another option misunderstands checkpoint scope; complex features absolutely benefit from checkpoints—that's when you most need stable rollback points. The lesson shows that checkpoint granularity should balance progress capture with stability: checkpoint when functionality works, even if code needs improvement, then checkpoint again after improvement.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "During implementation, you discover that the specification's success criteria can't be met with the planned approach. You see two options: change the approach (adding two weeks) or change the success criteria (removing a requirement). What should you do first?",
      options: [
        "Change the approach immediately to meet original criteria",
        "Change success criteria after documenting the technical constraint",
        "Consult specification stakeholders before deciding either change alone",
        "Implement partial solution and revisit during next iteration"
      ],
      correctOption: 2,
      explanation: "Consult specification stakeholders before deciding—this is a specification-level decision, not an implementation decision. Either option (changing approach or criteria) affects scope, timeline, or requirements, which need stakeholder input. Another option assumes meeting original criteria is always correct, but if the requirement was based on faulty assumptions, adjusting it might be better than spending two extra weeks. Another option assumes the technical constraint justifies changing requirements, but stakeholders might prioritize the requirement and accept the timeline extension. Another option (partial solution) might violate the requirement while creating technical debt that makes the full solution harder later. The lesson emphasizes that implementation should execute the specification, not rewrite it; when implementation reveals that specification needs adjustment, loop back to specification stakeholders rather than unilaterally changing scope or approach.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "You're implementing a feature and realize you need a utility function that will be reused across multiple tasks. Should you create the utility now or wait until the second use case?",
      options: [
        "Create utility now to avoid duplication and future refactoring",
        "Duplicate code initially and extract utility during dedicated refactoring phase",
        "Create utility now but document as technical debt needing validation",
        "Wait until second use to verify reusability before abstracting"
      ],
      correctOption: 3,
      explanation: "Wait until the second use case to verify reusability before abstracting—this prevents premature abstraction based on speculative future needs. You might think you need a general utility, but the second use case might have different requirements that make the abstraction wrong. Another option creates abstraction risk; if you abstract too early, you might create the wrong interface, then struggle to retrofit it when real use cases emerge. Another option tries to have it both ways but creates unnecessary overhead; if you're not confident in the abstraction, don't create it yet—wait for evidence. Another option suggests always duplicating then refactoring, but this misses the point; the issue is when to abstract, not whether to refactor. The lesson follows the rule of three: first use, duplicate; second use, extract if patterns match; third use, refine the abstraction. This ensures abstractions are based on real needs, not speculation.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "During code review, a teammate suggests an implementation improvement that would require changing three completed checkpoints. The improvement adds no new functionality but makes the code cleaner. How should this be evaluated?",
      options: [
        "Accept improvement because code quality should be prioritized always",
        "Evaluate cost-benefit considering task status and remaining work explicitly",
        "Reject improvement because checkpoints represent committed stable states",
        "Defer improvement to technical debt backlog for future sprints"
      ],
      correctOption: 1,
      explanation: "Evaluate cost-benefit considering where you are in the task—if you're early with minimal subsequent work, the improvement might be worth it; if you're near completion with extensive subsequent work built on those checkpoints, it might not be. Another option makes code quality absolute regardless of context, but good engineering balances quality with delivery; rewriting working code has opportunity cost. Another option goes too far in the opposite direction; checkpoints provide stability but aren't immutable—you can refactor them if the value justifies the cost. Another option automatically defers improvements, which might be appropriate here, but shouldn't be automatic; some improvements are worth doing now. The lesson emphasizes pragmatic decision-making: checkpoints provide stable rollback points, but they're not permanent commitments. Evaluate refactoring proposals based on: how much subsequent work depends on current structure? How significant is the improvement? What's the timeline pressure?",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "You've created several project-specific prompt patterns that work well. A teammate suggests extracting them into a reusable skill. What factor most clearly indicates these patterns are ready for skill extraction?",
      options: [
        "Patterns have been used successfully in three different contexts",
        "Patterns include clear persona and question frameworks already",
        "Patterns have detailed documentation and usage examples currently",
        "Patterns solve general problems not specific implementations only"
      ],
      correctOption: 3,
      explanation: "Patterns solving general problems (not specific implementations) is the clearest indicator that they're ready for skill extraction. Skills should be broadly applicable; if your patterns are specific to your current project's architecture, they're not reusable. Another option (three uses) provides evidence of reusability but doesn't guarantee it—you might have used a project-specific pattern three times within the same project. Another option describes the structure skills should have, but patterns don't need this structure before extraction—you add it during extraction. Another option (documentation) is valuable but doesn't indicate reusability; well-documented project-specific patterns are still project-specific. The lesson emphasizes that skills should be context-independent: a skill for 'designing database schemas' is reusable, a skill for 'implementing our user service schema' is not. Generality, not usage count or documentation, determines skill readiness.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "When creating a subagent for test generation, you include detailed examples of your project's specific testing patterns. Review feedback suggests making examples more general. Why?",
      options: [
        "General examples are easier to understand for beginners",
        "General examples prevent intellectual property disclosure concerns",
        "General examples reduce subagent file size significantly",
        "General examples enable reuse across different technology stacks"
      ],
      correctOption: 3,
      explanation: "General examples enable reuse across different technology stacks—if your test generation subagent only shows pytest examples, it's less useful for developers using Jest, JUnit, or other frameworks. Another option misidentifies the issue; generality isn't about simplification for beginners—it's about transferability across contexts. A general principle like 'arrange-act-assert pattern' applies everywhere; a pytest-specific example only helps pytest users. Another option is trivial; file size is rarely a constraint for subagent definitions. Another option might occasionally be relevant but isn't the primary reason for generality. The lesson shows that reusable intelligence should be technology-agnostic when possible: instead of 'here's how to write pytest fixtures,' provide principles like 'isolate test setup,' 'make dependencies explicit,' and 'prefer composition over inheritance in test utilities'—these apply everywhere.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "You're designing a skill for API documentation generation. Should the skill include specific documentation tool configurations (like OpenAPI schemas) or focus on documentation principles?",
      options: [
        "Focus on principles allowing skill to apply across documentation tools",
        "Include tool configurations because developers need immediate actionable guidance",
        "Include both principles and tool-specific examples for comprehensive coverage",
        "Focus on principles initially and add tool configurations after validation"
      ],
      correctOption: 0,
      explanation: "Focus on principles allowing the skill to apply across documentation tools—a skill about 'documenting API contracts' should work whether you're using OpenAPI, GraphQL schemas, gRPC protos, or other formats. Another option prioritizes immediate utility but sacrifices reusability; tool-specific skills require maintenance as tools evolve and don't transfer to other contexts. Another option tries to serve both goals but creates maintenance burden; as documentation tools evolve, you constantly update examples. Another option suggests starting with principles then adding tool configurations, but the second step undermines reusability—once you add tool specifics, the skill is no longer tool-agnostic. The lesson emphasizes that skills should encode decision frameworks, not implementation recipes: instead of 'how to write OpenAPI schemas,' teach 'what makes API documentation useful' (completeness, examples, error cases, versioning)—these principles guide developers regardless of tooling.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "A developer creates an ADR documenting the decision to use PostgreSQL for a project. Another developer argues ADRs should capture architectural decisions, not technology choices. How should this be resolved?",
      options: [
        "Technology choices are implementation details not requiring ADR documentation",
        "Technology choices should be documented separately in technical specifications only",
        "Technology choices are architectural when they create long-term constraints significantly",
        "Technology choices become architectural only when affecting multiple services"
      ],
      correctOption: 2,
      explanation: "Technology choices are architectural when they create long-term constraints—choosing PostgreSQL constrains data modeling, affects scaling approaches, determines operational expertise needed, and influences what other technologies integrate easily. Another option treats technology choices as trivial, but some technology decisions have profound long-term consequences that deserve ADR treatment. Another option suggests separate documentation, but that misses the purpose of ADRs: capturing the reasoning behind significant decisions so future maintainers understand why. Another option makes the mistake of defining 'architectural' by scope (multiple services), but impact matters more than scope; even single-service technology choices can be architectural if they're difficult to reverse. The lesson shows that ADR-worthy decisions are those with long-term consequences or high reversal cost—choosing PostgreSQL likely qualifies, while choosing a logging library might not.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "You're adopting Spec-Kit Plus in a mature codebase with 100,000 lines and active development. A teammate suggests rewriting the entire codebase to match SDD-RI patterns. What's the brownfield adoption principle this violates?",
      options: [
        "Brownfield adoption requires comprehensive planning before any implementation changes",
        "Brownfield adoption should preserve existing code until new patterns prove valuable",
        "Brownfield adoption needs stakeholder buy-in before technical changes begin",
        "Brownfield adoption uses incremental adoption not wholesale replacement strategies"
      ],
      correctOption: 3,
      explanation: "Brownfield adoption uses incremental adoption, not wholesale replacement—you introduce SDD-RI patterns gradually to new features while leaving existing code alone unless you're already modifying it. Rewriting 100,000 lines is high-risk and disrupts ongoing development. Another option is partially true (planning helps), but the core issue is the wholesale replacement approach, not lack of planning. Another option addresses important change management but misses the technical strategy problem; even with stakeholder buy-in, rewriting everything is the wrong approach. Another option suggests preserving code until new patterns 'prove valuable,' but this creates an unnecessary proof burden; you can adopt new patterns incrementally without proving them first. The lesson emphasizes that brownfield adoption means strangler fig pattern: wrap or replace incrementally, let old and new coexist, gradually expand new patterns without big-bang rewrites.",
      source: "Lesson 10: Brownfield Adoption"
    },
    {
      question: "During brownfield adoption, developers want to start using the constitution immediately for all decisions, but existing code violates many constitutional principles. How should the constitution be applied during transition?",
      options: [
        "Apply constitution to new code while existing code remains as-is",
        "Suspend constitutional principles until codebase aligns with them fully",
        "Refactor existing code to match constitution before writing new features",
        "Create separate constitution for legacy code and new code"
      ],
      correctOption: 0,
      explanation: "Apply the constitution to new code while existing code remains as-is—this allows immediate value from constitutional decision-making without requiring expensive refactoring. The constitution guides forward progress; legacy code is addressed opportunistically when touched. Another option (suspending principles) defeats the purpose of brownfield adoption; you want to start getting value from SDD-RI immediately, not wait until you've refactored everything. Another option (refactoring first) creates the same big-bang problem; refactoring to constitutional alignment before adding features delays value and risks disruption. Another option (separate constitutions) creates fragmentation and confusion; having different decision frameworks for different code areas makes the codebase incoherent. The lesson shows that constitutional adoption is forward-looking: use it for new decisions immediately, let it influence refactoring priorities opportunistically, but don't let legacy code block adoption.",
      source: "Lesson 10: Brownfield Adoption"
    },
    {
      question: "A specification document is divided into sections: Context, Goals, Success Criteria, Constraints, and Non-Goals. During review, stakeholders keep adding items to every section. What specification practice prevents uncontrolled growth?",
      options: [
        "Limiting each section to maximum five items enforced strictly",
        "Requiring stakeholder consensus before any new items get added",
        "Applying minimal viable scope by questioning each addition's necessity",
        "Periodically reviewing and removing outdated or redundant entries"
      ],
      correctOption: 2,
      explanation: "Applying minimal viable scope means questioning whether each addition is necessary for the current iteration—can we defer it? Is it essential to the core problem? This prevents scope creep. Another option creates an arbitrary constraint that might exclude necessary items or include unnecessary ones just to fill space. The issue isn't section length but whether each item is justified. Another option addresses process but doesn't prevent growth; stakeholders might consensus-approve unnecessary additions. Another option helps manage existing content but doesn't prevent the initial additions that cause bloat. The lesson emphasizes that specifications should define the minimum necessary to solve the problem and achieve success—each specification element should be defensible as necessary, not just nice-to-have.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "During implementation checkpoint reviews, a teammate always creates checkpoints after completing large multi-day features. You create checkpoints after completing smaller increments within features. Who is applying the checkpoint pattern more effectively?",
      options: [
        "You apply checkpoints correctly by creating more frequent rollback points",
        "Teammate applies checkpoints correctly by aligning with feature boundaries",
        "Both approaches are valid depending on team workflow preferences",
        "Checkpoint frequency should match sprint boundaries for consistency"
      ],
      correctOption: 0,
      explanation: "You're applying checkpoints more effectively by creating frequent rollback points—checkpoints provide safety through granularity. If a checkpoint represents 3 days of work, you lose a lot when rolling back; if it represents half a day, rollback cost is minimal. Another option misunderstands checkpoint purpose; feature boundaries define deliverables, but checkpoints define safe states. Large features should have multiple checkpoints. Another option suggests both approaches are equally valid, but the lesson explicitly recommends frequent checkpoints to minimize rollback cost. Another option confuses checkpoints with sprint planning; checkpoints are technical safety mechanisms (stable working states), while sprints are project management time-boxes. The checkpoint pattern emphasizes creating stable states frequently—daily or more often—so you always have a recent working version to return to if problems arise.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "While implementing a feature, you discover a bug in framework code you're using. Fixing it would take 2 hours. Workaround would take 15 minutes but adds technical debt. The checkpoint pattern is your guide. What should you consider?",
      options: [
        "Evaluate impact and timeline to decide fix versus workaround pragmatically",
        "Use workaround and document as technical debt for future resolution",
        "Fix the framework bug because checkpoints require eliminating all debt",
        "Defer decision until discussing with team lead or architect"
      ],
      correctOption: 0,
      explanation: "Evaluate impact and timeline pragmatically—the checkpoint pattern doesn't dictate fixing everything immediately; it emphasizes maintaining working states while progressing. If the workaround is safe and you're under timeline pressure, it might be appropriate. Another option misunderstands checkpoints; they require stable working states, not zero technical debt. Technical debt is acceptable if documented and manageable. Another option automatically chooses the workaround, but sometimes the fix is better (if the bug affects multiple features or the workaround is fragile). Another option defers unnecessarily; this is an implementation decision developers can make if they understand the trade-offs. The lesson shows that checkpoint-driven development balances progress with stability: sometimes you fix, sometimes you workaround, depending on context. The key is creating stable checkpoints either way and documenting debt if you workaround.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "A developer creates a PHR (Prompt History Record) documenting an AI interaction that solved a tricky debugging problem. Another developer questions whether this is worth the overhead. What determines if an interaction deserves PHR documentation?",
      options: [
        "All AI interactions should be documented for comprehensive traceability",
        "Interactions revealing reusable patterns or solving novel problems warrant PHRs",
        "Only interactions producing production code require PHR documentation",
        "Interactions longer than certain message count threshold need PHRs"
      ],
      correctOption: 1,
      explanation: "Interactions revealing reusable patterns or solving novel problems warrant PHRs—the value is capturing knowledge that others (or future you) can learn from. Another option creates unsustainable overhead; documenting every interaction buries valuable insights in noise. PHRs should capture significant learning, not routine interactions. Another option is too narrow; PHRs document thinking processes and problem-solving approaches, not just code outputs. A valuable debugging insight might not produce any code but is still worth capturing. Another option uses an arbitrary metric (message count) that doesn't correlate with value; a short interaction might reveal a crucial insight, while a long interaction might be routine troubleshooting. The lesson emphasizes that PHRs are selective documentation: capture interactions that teach something reusable, not every conversation with AI.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "When creating a skill for error message design, you include examples from your Python codebase. A reviewer suggests removing language-specific examples. You argue that concrete examples help clarity. How should this tension be resolved?",
      options: [
        "Keep Python examples because concrete examples aid understanding significantly",
        "Use pseudocode examples that illustrate principles without language specificity",
        "Include Python examples but add examples from other languages too",
        "Remove all examples and focus on abstract principles only"
      ],
      correctOption: 1,
      explanation: "Using pseudocode examples illustrates principles without language specificity—you get clarity from concrete examples without sacrificing reusability. Pseudocode bridges the gap between abstract principles and concrete application. Another option prioritizes immediate clarity but sacrifices reusability; Python developers benefit, but JavaScript, Java, and Rust developers see less value. Another option (multi-language examples) creates maintenance burden and length; as languages evolve, examples need updating. Another option goes too far; pure abstraction makes skills hard to understand and apply. The lesson shows that skills should be technology-agnostic but not abstract—pseudocode, general patterns, and language-independent structures provide clarity while maintaining transferability across contexts.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "Your brownfield codebase has inconsistent patterns: some modules follow SDD-RI, others follow old approaches. A new developer is confused about which approach to use for new features. What should guide this decision?",
      options: [
        "Match existing patterns in each module to maintain local consistency",
        "Use SDD-RI for new features regardless of surrounding code",
        "Refactor surrounding code to SDD-RI before adding new features",
        "Create migration plan and wait until complete to use SDD-RI"
      ],
      correctOption: 1,
      explanation: "Use SDD-RI for all new features regardless of surrounding code—this is the forward-looking approach that gradually improves the codebase. Another option (matching existing patterns) perpetuates old approaches and prevents improvement; if you always match surrounding code, SDD-RI never expands. Another option (refactor first) creates unnecessary delays; you don't need to refactor old code before adding new features with better patterns. Another option (wait for migration) defeats the purpose of brownfield adoption; you'd never start because the migration plan might take months or years. The lesson emphasizes that brownfield adoption means applying new patterns to new work immediately—let old code coexist until you touch it, but don't let it dictate new feature approaches. This creates natural migration as the codebase evolves.",
      source: "Lesson 10: Brownfield Adoption"
    },
    {
      question: "During brownfield adoption, you implement a new feature using SDD-RI workflow, but it needs to integrate with legacy code that doesn't follow specifications. How should the integration boundary be managed?",
      options: [
        "Create adapter layer isolating new SDD-RI code from legacy",
        "Refactor legacy code to match SDD-RI patterns before integrating",
        "Compromise by using hybrid approach blending both patterns",
        "Document integration points thoroughly to clarify pattern transition"
      ],
      correctOption: 0,
      explanation: "Creating an adapter layer isolates new SDD-RI code from legacy, allowing both to coexist without contamination. The adapter translates between patterns, keeping new code clean while interfacing with legacy. Another option (refactor legacy first) creates the big-bang problem; you end up refactoring large amounts of code before delivering any new features. Another option (hybrid approach) is worst of both worlds; you lose the benefits of SDD-RI by contaminating it with legacy patterns, and you create inconsistent code that's harder to maintain. Another option (thorough documentation) helps but doesn't solve the structural problem; documentation can't prevent pattern contamination if new code directly depends on legacy patterns. The lesson shows the strangler fig pattern: wrap legacy with adapters, keep new code clean, gradually expand the new pattern without forcing wholesale rewrites.",
      source: "Lesson 10: Brownfield Adoption"
    },
    {
      question: "Your team is six months into brownfield adoption. About 30% of the codebase now follows SDD-RI patterns. Stakeholders ask when they'll see productivity improvements. What realistic timeline should you communicate?",
      options: [
        "Improvements emerge gradually as team internalizes patterns and reusable intelligence accumulates",
        "Immediate improvements as SDD-RI reduces implementation time significantly",
        "Improvements appear after majority of codebase converts to SDD-RI",
        "Improvements depend on team size and project complexity individually"
      ],
      correctOption: 0,
      explanation: "Improvements emerge gradually as the team internalizes patterns and reusable intelligence accumulates—this is realistic and evidence-based. Initially, SDD-RI might slow things down as team learns new workflows; productivity improves as patterns become habitual and reusable intelligence (skills, subagents, ADRs) reduces repeated decisions. Another option overpromises; immediate improvements are unlikely because learning new patterns has upfront cost. Another option delays expectations too long; you don't need majority conversion to see benefits—productivity improves incrementally as the team gets faster with SDD-RI and builds reusable intelligence. Another option is too vague; while team and project factors matter, the gradual improvement pattern is consistent across contexts. The lesson shows that SDD-RI is an investment: upfront learning cost, gradual return as patterns internalize and intelligence accumulates.",
      source: "Lesson 10: Brownfield Adoption"
    },
    {
      question: "A team member argues that brownfield adoption is failing because they spend more time writing specifications now than they used to spend on implementation. What does this criticism likely misunderstand about the SDD-RI workflow?",
      options: [
        "Specification time is upfront investment that reduces total implementation time",
        "Team needs more training to write specifications efficiently and quickly",
        "Specifications become faster with templates and reusable intelligence over time",
        "Specifications prevent costly rework making total project time lower overall"
      ],
      correctOption: 3,
      explanation: "Specifications prevent costly rework, making total project time lower—this addresses the criticism directly. Yes, specification takes time upfront, but it prevents the false starts, misunderstandings, and rework that cost more time overall. Another option describes the investment model but doesn't address the total time concern; the critic might still believe total time increased. Another option suggests the problem is execution (training), not the approach, but misses the point; even well-written specifications take time, and the value is in preventing rework. Another option correctly notes that specifications get faster with practice and tools, but this is a secondary benefit; the primary value is preventing rework. The lesson emphasizes that SDD-RI optimizes for total project time, not just implementation time—spending time on specifications reduces debugging, reduces feature rework, reduces integration problems, and reduces maintenance costs.",
      source: "Lesson 10: Brownfield Adoption"
    }
  ]}
  questionsPerBatch={18}
/>

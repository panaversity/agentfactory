---
sidebar_position: 9
title: "Chapter 16: SDD-RI Fundamentals Quiz"
---

# Chapter 16: Understanding Spec-Driven Development with Reusable Intelligence Quiz

Test your understanding of specification-driven development, reusable intelligence design, and the frameworks that enable AI-native software development. This comprehensive assessment covers all fundamental concepts from Chapter 16.

<Quiz
  title="Chapter 16: SDD-RI Fundamentals Assessment"
  questions={[
    {
      question: "A developer writes 'Build a login system' as their prompt to an AI agent. After 10 iterations addressing missing password reset, rate limiting, and email verification, they've spent 12 hours. What core problem does this scenario demonstrate?",
      options: [
        "AI agents cannot generate authentication systems reliably enough",
        "The developer should have used better prompting",
        "Login systems require too many features for AI",
        "Vague prompts force iteration cycles that specifications prevent"
      ],
      correctOption: 3,
      explanation: "This scenario illustrates the vague-code problem where unclear requirements force expensive iteration cycles. With a specification defining all requirements upfront (password reset, rate limiting, email verification), the AI could generate a complete system in one iteration, saving 8+ hours. The issue isn't AI capability or feature complexity, but the absence of explicit requirements. Better prompting won't fix inherently vague requirements—only specifications make implicit expectations explicit. The cost-benefit math shows: 1 hour spec + 3 hours refinement = 4 hours vs. 12 hours of iteration from vague prompts.",
      source: "Lesson 1: Vague Code and the AI Partner Problem"
    },
    {
      question: "Why do AI agents specifically need specifications more urgently than human developers do, according to the literal-mindedness principle?",
      options: [
        "AI agents require more detailed technical documentation overall",
        "Human developers always write better code than AI",
        "AI agents implement exactly specified requirements without inferring intent",
        "Specifications help AI agents learn programming patterns faster"
      ],
      correctOption: 2,
      explanation: "AI literal-mindedness is the critical difference: human colleagues can infer unstated intent ('Did you want password reset?') and ask clarifying questions during development, but AI agents implement EXACTLY what you specify—no more, no less. If you don't mention password reset, AI assumes you don't want it. This isn't a limitation but how AI works. Human code quality and AI learning speed are separate issues. The need for detailed technical documentation applies to both humans and AI. The distinctive factor is that AI cannot improvise around vague requirements, making specifications mandatory rather than optional for AI-native development.",
      source: "Lesson 1: Vague Code and the AI Partner Problem"
    },
    {
      question: "Three developers implement password reset features. Developer A uses bcrypt (secure), B uses MD5 (insecure), C doesn't hash (catastrophic). Each spec said 'use secure hashing.' What specification problem does this reveal?",
      options: [
        "Specifications cannot prevent security vulnerabilities in distributed teams",
        "Feature-specific specs lack system-wide rules enforcing consistent interpretation",
        "Developers need more security training before writing specifications",
        "The original specifications were written with insufficient detail"
      ],
      correctOption: 1,
      explanation: "This scenario demonstrates why Constitutions (system-wide rules) are essential. Each developer HAD a spec saying 'secure hashing,' but interpreted it differently because no Constitution defined what 'secure' means. A Constitution stating 'ALL passwords use bcrypt cost 12+' removes interpretation—everyone implements bcrypt, no debate. Security training doesn't solve interpretation variance. More detailed feature-specific specs don't scale (you'd repeat 'bcrypt' in 50 different specs). Specifications CAN prevent vulnerabilities when backed by constitutional enforcement. The problem isn't missing detail in individual specs, but missing global rules applying to ALL specs.",
      source: "Lesson 5: Your Team Needs Shared Rules"
    },
    {
      question: "A team discovers their password reset tokens don't expire, creating a critical security gap. They add a Constitution rule: 'All temporary access tokens MUST specify time-based expiry. Default: 30 minutes.' What pattern does this demonstrate?",
      options: [
        "Production bugs become constitutional rules preventing future recurrence",
        "Constitutional rules should address security issues after discovery",
        "Specifications need more comprehensive security testing from start",
        "Token expiry should have been in specification"
      ],
      correctOption: 0,
      explanation: "This demonstrates organizational learning through constitutional evolution: Production bug → Root cause analysis (spec didn't specify token expiry) → Constitutional rule added (all temporary tokens must specify expiry) → Future specs automatically include expiry. The pattern is: encountering a gap once, encoding it as a rule, preventing recurrence across 50 features and 10 teams. While token expiry SHOULD have been in the original spec, that's not the pattern demonstrated—the insight is how teams systematically prevent repeated mistakes. Saying Constitutions should 'address issues after discovery' misses the preventive value. More comprehensive testing doesn't prevent specification gaps—only better specifications do.",
      source: "Lesson 5: Your Team Needs Shared Rules"
    },
    {
      question: "Model-Driven Development (MDD) in the 2000s promised automatic code generation from UML models but mostly failed. SDD succeeds where MDD failed. What fundamental difference explains this?",
      options: [
        "SDD has better testing frameworks than MDD provided",
        "UML models were too complex for developers to maintain",
        "2025 LLMs understand natural language better than MDD understood UML",
        "MDD used proprietary tools causing lock-in; SDD uses LLMs"
      ],
      correctOption: 3,
      explanation: "The fundamental difference is MDD's tool lock-in problem versus SDD's flexibility. MDD required custom code generators creating vendor dependency, models sat at awkward abstraction levels, and models/code diverged. SDD uses natural language specs (markdown/prose, not proprietary formats) that ANY LLM can process—no custom tools, less lock-in, faster feedback (spec changes → regenerated code in minutes, not hours). While 2025 LLMs ARE better at understanding nuance than 2004 code generators, this is a consequence of the tool flexibility, not the root cause. UML complexity and testing frameworks are secondary issues. The core failure was architectural: MDD locked teams into specific tools, SDD doesn't.",
      source: "Lesson 2: Why Do Specs Matter NOW?"
    },
    {
      question: "A spec states: 'divide(10, 2) should return 5.0' (not 5). What specification concept does this example demonstrate?",
      options: [
        "Type preservation rules require explicit specification details",
        "Edge cases must be documented for operations",
        "Acceptance criteria are executable contracts defining correctness",
        "Division operations always return float types consistently"
      ],
      correctOption: 2,
      explanation: "This demonstrates that acceptance criteria are executable contracts: the spec isn't describing behavior, it's DEFINING what 'correct' means through a concrete test case. The specification that divide(10, 2) returns 5.0 (not 5) is a contract—it says division ALWAYS returns float, even when mathematically exact. While this relates to type preservation rules, the broader concept is that test scenarios ARE the specification in executable form. Edge cases are a different concept (boundary conditions like division by zero). Saying 'division operations always return float' describes behavior but misses the deeper insight: specifications define correctness through testable examples, not prose descriptions.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "During calculator spec collaboration, AI asks: 'For subtract(a, b), does subtract(5, 3) return 2 or -2?' The developer clarifies: 'a - b, so subtract(5, 3) = 2.' What specification development pattern does this illustrate?",
      options: [
        "AI agents identify ambiguities that humans overlook in specs",
        "Collaborative iteration where questions reveal gaps converging on clarity",
        "AI requires more explicit instructions than human developers",
        "Specifications must define function parameter order precisely"
      ],
      correctOption: 1,
      explanation: "This illustrates the convergence loop at specification level: AI identifies ambiguity → Developer clarifies → Spec improves through iteration. The pattern (AI asks questions → You clarify → Both improve spec together) is core to AI-native specification development. Neither could write the perfect spec alone—iteration produces production-ready specifications. While AI DOES identify ambiguities humans overlook, that's the mechanism, not the pattern. The statement about AI requiring explicit instructions is generally true but misses the collaborative convergence demonstrated. Parameter order precision is the specific issue resolved, not the general pattern shown. The key insight is iterative refinement through question-asking.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "A team writes specs with edge cases fully documented (division by zero, floating-point precision, type handling). Later, when AI generates code, it correctly handles all edge cases. What specification principle does this demonstrate?",
      options: [
        "Comprehensive specifications enable first-iteration correct implementation success",
        "AI agents can infer edge case handling from",
        "Edge cases should be documented for clarity",
        "Type preservation rules prevent runtime errors effectively"
      ],
      correctOption: 0,
      explanation: "This demonstrates specification primacy: when edge cases are explicit in specs, AI generates correct code in the first iteration because the specification removed ambiguity. The spec was clear enough that implementation became obvious and correct—AI didn't need to guess or ask questions during coding. While edge cases SHOULD be documented, this is a broader principle about specification completeness enabling success. AI cannot 'infer' edge case handling from context—that's precisely what specifications prevent. Type preservation rules are one aspect of edge cases, not the comprehensive principle. The key is that complete specifications (including edge cases) produce correct first implementations because clarity eliminates guesswork.",
      source: "Lesson 3: What is SDD"
    },
    {
      question: "Why did Specification-Driven Development emerge specifically in 2025 rather than in the 1970s (Formal Methods era), 2000s (MDD era), or 2010s (Agile era)?",
      options: [
        "Technology stack complexity required better documentation approaches now",
        "Developers finally recognized the value of writing specifications",
        "2025 AI agents became powerful and literal-minded simultaneously",
        "Formal Methods and MDD failed so teams tried"
      ],
      correctOption: 2,
      explanation: "SDD emerged in 2025 because three forces converged: (1) AI got good enough to generate production code from specs, (2) developers discovered specs save iteration time (cost-benefit math finally works: 1-hour spec + 3-hour refinement = 4 hours vs. 12-hour vague-prompt iteration), and (3) AI literal-mindedness made specs mandatory—AI can't improvise like humans. Earlier spec approaches failed for different reasons: Formal Methods required PhD-level math, MDD created tool lock-in, Agile minimized specs losing institutional knowledge. Developer recognition and documentation needs existed before 2025. Technology complexity alone doesn't explain timing. The distinctive 2025 factor: AI agents both NEED explicit specs (literal-minded) and CAN execute them (powerful enough).",
      source: "Lesson 2: Why Do Specs Matter NOW?"
    },
    {
      question: "A specification states: 'API response must be <100ms for user list endpoint.' What type of specification requirement is this?",
      options: [
        "Constraint limiting technical implementation approach choices",
        "Functional requirement describing what system should do",
        "Acceptance criteria for validating feature completion success",
        "Non-functional performance requirement defining system quality attributes"
      ],
      correctOption: 3,
      explanation: "This is a non-functional requirement (NFR) specifying a quality attribute—performance—rather than what the system does. Functional requirements describe behavior ('system returns user list'), while NFRs describe quality characteristics (latency, security, scalability). While this COULD be used as acceptance criteria for validation, that's the usage, not the type. Constraints limit implementation choices ('must use PostgreSQL'), but this NFR defines a performance target without prescribing how to achieve it—you could optimize queries, add caching, or use better data structures. The distinction matters: functional requirements tell AI WHAT to build, NFRs tell AI quality targets to achieve.",
      source: "Lesson 3: What is SDD"
    },
    {
      question: "During spec-first development, a developer writes: 'add(a, b) returns int | float (preserves input types)' and 'divide(a, b) always returns float (consistency over type preservation).' What do these contrasting decisions demonstrate?",
      options: [
        "Specifications must define explicit design decisions with reasoning",
        "Type handling should be consistent across arithmetic operations",
        "Division operations require special handling in specifications",
        "Python type hints allow flexible return types"
      ],
      correctOption: 0,
      explanation: "These contrasting decisions demonstrate that specifications force explicit design decisions with rationale. For addition, the designer chose type preservation (int + int = int); for division, the designer chose consistency (always float, even when exact). Both are valid designs, but the spec makes the choice explicit rather than leaving AI to guess. Consistency ACROSS operations isn't shown here—the decisions deliberately differ based on reasoning. Division doesn't universally require special handling; this spec made a specific choice. Python type hints enable flexibility, but that's the mechanism, not the lesson. The core insight: specifications aren't just documentation—they're where you encode design decisions that AI will implement. The rationale ('consistency over type preservation') shows thoughtful decision-making.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "A Constitution states: 'All API specs MUST define error response format.' A developer's API spec omits error responses. During review, what should happen?",
      options: [
        "Developer adds error responses during implementation instead",
        "Spec rejected until error responses added per rules",
        "Constitution rule updated to be more flexible",
        "Error responses added as optional enhancement later"
      ],
      correctOption: 1,
      explanation: "Constitutional compliance is enforced at spec time, not implementation time—the spec should be rejected until error responses are added per Constitution. This is the enforcement mechanism that makes Constitutions effective: quality rules apply to ALL specs, no exceptions. If developers can skip Constitution rules during spec-writing and 'add them during implementation,' the Constitution has no teeth—quality becomes optional. Updating Constitution rules to be more flexible defeats the purpose of having consistent quality standards. Making error responses 'optional enhancements' contradicts the Constitution's MUST requirement. The value of Constitutions is preventing quality gaps before they become code. Spec-time enforcement ensures every feature meets minimum quality standards from the start.",
      source: "Lesson 5: Your Team Needs Shared Rules"
    },
    {
      question: "A pattern appears in 30 different specifications: 'API endpoints require definition of endpoints, error handling, authentication, and rate limiting.' What does this recurrence suggest?",
      options: [
        "This pattern indicates need for API standardization",
        "API specifications are too complex and need",
        "Developers should use API specification template",
        "This pattern should become reusable intelligence component"
      ],
      correctOption: 3,
      explanation: "When a pattern recurs across 30 specs (80%+ of features), it should become Reusable Intelligence—either a Skill (guidance with 2-4 decision points) or a Subagent (autonomous reasoning with 5+ decision points). This encodes institutional knowledge once rather than rewriting it 30 times. Templates help but don't provide reasoning; they're static while RI components guide context-specific decisions. Saying 'API specs are too complex' misses the point—complexity isn't reduced by ignoring it, but by encoding expertise once and reusing it. Standardization is related but not the pattern demonstrated—the insight is recognizing when repeated patterns justify creating reusable intelligence. The threshold: patterns recurring 2+ times with 2+ decision points merit consideration.",
      source: "Lesson 6: Introduction to Reusable Intelligence"
    },
    {
      question: "What distinguishes a Skill (providing guidance) from a Subagent (autonomous reasoning) in the Reusable Intelligence framework?",
      options: [
        "Skills provide templates; Subagents generate complete code",
        "Skills are simpler; Subagents handle complex problems",
        "Skills have two to four decision points; Subagents five-plus",
        "Skills apply broadly; Subagents are domain-specific only"
      ],
      correctOption: 2,
      explanation: "The defining distinction is decision points: Skills handle 2-4 decisions (human decides with guidance), Subagents handle 5+ decisions (AI reasons autonomously). Example: Input Validation Skill (2-4 decisions: valid types? ranges? error handling? strict/lenient?) provides guidance; Performance Optimization Subagent (5+ decisions: volumes? latency? N+1? caching? memory? structures?) reasons autonomously. While Skills ARE typically simpler and Subagents handle complexity, this is a consequence of decision point count, not the definition. Skills don't just provide templates—they guide reasoning. Subagents don't generate code—they analyze specs and make recommendations. Skills CAN be domain-specific (not just broad), and Subagents CAN apply broadly (not just vertically). Decision point count is the architectural threshold.",
      source: "Lesson 6: Introduction to Reusable Intelligence"
    },
    {
      question: "In P+Q+P pattern design, a Persona states: 'You are an expert in programming.' Why is this considered a weak Persona?",
      options: [
        "Lacks specific cognitive stance activating particular thinking pattern",
        "Too generic to provide useful AI guidance",
        "Does not specify domain expertise level",
        "Fails to establish authority for recommendations"
      ],
      correctOption: 0,
      explanation: "This Persona is weak because it lacks cognitive stance—it doesn't activate a SPECIFIC thinking pattern. Compare: 'You are an expert' (generic) vs. 'You are a defensive programming specialist focused on attack surfaces. Think about validation the way a security auditor thinks about vulnerabilities' (activates defensive, security-first reasoning). The strong version establishes HOW to think, not just WHAT to know. While 'too generic' is related, the deeper issue is absence of cognitive framing. Domain expertise level matters less than thinking approach. Authority for recommendations isn't the Persona's purpose—establishing reasoning stance is. The goal: shift AI from pattern-matching mode ('retrieve generic expert knowledge') to reasoning mode ('analyze THIS context defensively').",
      source: "Lesson 7: Designing Skills and Subagents"
    },
    {
      question: "A Security Review Subagent's Questions section includes: 'What are actual threat actors? What's compliance requirement? What's deployment timeline?' Why are these questions effective?",
      options: [
        "Questions cover all security domains comprehensively",
        "Questions force context-specific analysis instead of generic patterns",
        "Questions help AI understand the problem",
        "Questions reduce need for human security expertise"
      ],
      correctOption: 1,
      explanation: "These questions are effective because they force context-specific analysis: 'What are YOUR actual threat actors? (script kiddies, targeted attackers, nation-states?)' demands reasoning about THIS situation, not retrieving generic security patterns. Weak questions like 'Is this secure?' encourage pattern-matching; strong questions like 'What's OUR compliance requirement? (PCI-DSS, HIPAA, GDPR, none?)' force situation-specific reasoning. While these questions DO cover important security domains, comprehensiveness isn't what makes them effective—contextual forcing is. Questions help AI understand the problem, but that's the goal, not the mechanism. Questions don't reduce human expertise—they activate it systematically. The design principle: questions should be answerable only with knowledge of YOUR specific context.",
      source: "Lesson 7: Designing Skills and Subagents"
    },
    {
      question: "An Input Validation Skill includes the Principle: 'Validate at system boundaries (prevent bad data from entering).' Why is this a strong Principle?",
      options: [
        "Prevents security vulnerabilities at input points reliably",
        "States best practice that all developers recognize",
        "Provides decision framework with clear reasoning about where",
        "Reduces validation code duplication across the system"
      ],
      correctOption: 2,
      explanation: "This is a strong Principle because it provides a decision framework with clear reasoning: 'WHERE should validation happen? At boundaries (API endpoints, user inputs, file uploads) to prevent bad data from entering.' It guides a SPECIFIC decision (location) with a REASON (prevention). Weak principles like 'Use best practices' or 'Make it secure' offer no guidance. While this IS a recognized best practice, recognition doesn't make it strong—actionability does. It DOES prevent vulnerabilities, but that's the outcome, not what makes it a strong Principle. Code duplication reduction is a benefit, not the framework's purpose. The pattern: strong Principles answer specific questions ('where to validate?') with reasoning ('because boundaries prevent contamination'), enabling consistent decisions across contexts.",
      source: "Lesson 7: Designing Skills and Subagents"
    },
    {
      question: "Spec-Kit Plus adds three features beyond GitHub's Spec-Kit: ADRs, PHRs, and Intelligence Templates. What is the primary purpose of Prompt History Records (PHRs)?",
      options: [
        "Document prompts for reuse across multiple projects",
        "Provide audit trail for regulatory compliance requirements",
        "Help debug AI-generated code when bugs occur",
        "Capture AI interactions to enable organizational learning"
      ],
      correctOption: 3,
      explanation: "PHRs' primary purpose is organizational learning: tracking AI interactions reveals which prompts produce better results, enabling teams to learn from patterns. Example: 'Prompts with explicit error handling: 95% success. Prompts without: 60% success → Team learns: always specify error handling.' While PHRs DO provide audit trails for compliance (secondary benefit), that's not the primary purpose. PHRs HELP debugging by showing what was asked and generated, but that's reactive use, not the design purpose. Documenting prompts for reuse is closer, but misses the learning aspect—it's not just reuse, but learning WHY some prompts work better. The pattern: PHR analysis over time reveals success factors, converting individual interactions into institutional knowledge.",
      source: "Lesson 8: SDD-RI Framework Landscape"
    },
    {
      question: "Why would a 50-person enterprise team choose Spec-Kit over Kiro for their development framework?",
      options: [
        "Kiro is too simple for enterprise complexity",
        "Spec-Kit provides constitutional governance that scales consistency",
        "Spec-Kit has better documentation and support",
        "Enterprise compliance requires Spec-Kit constitutional enforcement"
      ],
      correctOption: 1,
      explanation: "The 50-person team needs constitutional governance because Spec-Kit's Constitution enforces consistency across all features—'All passwords use bcrypt' applies to 50 developers without constant meetings. Kiro's simplicity is an advantage for 1-3 developers but doesn't scale to 50 (you get simplicity, sacrifice governance). While Kiro IS 'too simple' for enterprise scale, the specific mechanism is lack of governance, not general complexity. Documentation quality isn't the differentiator—governance is. Compliance doesn't universally require Spec-Kit (you could meet compliance with any framework), but constitutional enforcement makes compliance consistent and measurable. The decision factor: team size (50 developers) creates need for shared rules ensuring consistency. Context determines framework fit.",
      source: "Lesson 8: SDD-RI Framework Landscape"
    },
    {
      question: "A developer spends 30 minutes writing a vague prompt and then 10 hours iterating. An alternative: 1 hour writing a spec + 3 hours refinement. At what iteration count does the spec approach break even?",
      options: [
        "Spec approach is immediately faster with comprehensive savings",
        "Break-even occurs after approximately two complete iterations",
        "Requires three iterations before spec becomes worthwhile",
        "Only valuable for projects exceeding four iterations"
      ],
      correctOption: 0,
      explanation: "The spec approach is immediately faster: 0.5-hour vague prompt + 10-hour iteration = 10.5 hours total vs. 1-hour spec + 3-hour refinement = 4 hours total. Savings: 6.5 hours (62% faster) on the FIRST attempt—no break-even calculation needed. The cost-benefit math shows specifications save time from iteration 1, not after multiple iterations. This assumes the vague approach requires ~10 hours of iteration (realistic for complex features) while the spec approach needs only ~3 hours of refinement (because requirements were clear upfront). The break-even question misframes the analysis—specs aren't an investment that pays off after multiple uses, they're immediately cheaper because they prevent iteration waste from the start.",
      source: "Lesson 2: Why Do Specs Matter NOW?"
    },
    {
      question: "A specification's quality checklist includes: 'Clear Intent? Testable Success Criteria? Explicit Constraints? Edge Cases Covered? No Implementation Details?' What does the last criterion prevent?",
      options: [
        "AI agents cannot understand implementation-specific specification instructions",
        "Implementation details make specifications too long difficult",
        "Technical details confuse non-technical stakeholders reading specs",
        "Specifications prescribing HOW prevents alternative implementations working"
      ],
      correctOption: 3,
      explanation: "The 'No Implementation Details' criterion prevents specs from prescribing HOW to build, focusing instead on WHAT and WHY. Example: Spec saying 'use bcrypt with 12 rounds' (HOW) is too prescriptive—better: 'passwords must be securely hashed' (WHAT) + Constitution: 'secure hashing = bcrypt cost 12+' (HOW, system-wide). This lets different AI agents implement correctly using different approaches while meeting the intent. Implementation details don't make specs 'too long'—they make them inflexible. Non-technical stakeholder confusion is a side concern, not the primary issue. AI CAN understand implementation-specific instructions—the problem is those instructions prevent alternative valid implementations. Specifications should define INTENT ('secure authentication') not IMPLEMENTATION ('use JWT with RS256').",
      source: "Lesson 3: What is SDD"
    },
    {
      question: "During calculator spec development, AI generates code that converts all results to float (losing int type). The spec said 'preserve input types' but wasn't explicit. What does this failure reveal?",
      options: [
        "Type preservation is too complex for specifications",
        "AI agents require more explicit type instructions",
        "Ambiguous specifications let AI make wrong assumptions revealing gaps",
        "The developer should have tested implementation"
      ],
      correctOption: 2,
      explanation: "This failure reveals that ambiguous specs let AI make assumptions (AI interpreted 'preserve types' as 'use float for consistency'), exposing the specification gap. When AI generated wrong code, it wasn't AI's fault—the spec was ambiguous. The developer then refined the spec with examples: '❌ DON'T: return float(a + b) // Loses int type. ✅ DO: return a + b // Python preserves naturally.' This is the convergence loop: failure reveals ambiguity → refine spec → AI regenerates correctly. While AI DOES need explicit instructions, that's the general principle, not the specific pattern shown. Type preservation isn't too complex—it just needs clear specification. Testing would catch the error but doesn't prevent it; clearer specs do.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "A team of 5 developers builds features independently. Without a Constitution, what problem is most likely to emerge?",
      options: [
        "Inconsistent security and architecture decisions creating technical debt",
        "Slower development speed due to lack coordination",
        "Duplicated code across different features implemented",
        "Difficulty onboarding new developers to the codebase"
      ],
      correctOption: 0,
      explanation: "The most likely problem is inconsistent security and architecture decisions: each developer makes independent choices (bcrypt vs. MD5 vs. nothing; JWT vs. sessions; different error patterns) creating technical debt and security gaps. Constitutions solve this by establishing shared rules ('ALL passwords use bcrypt') everyone follows. Slower speed isn't the primary issue—inconsistency is. Code duplication is a symptom, not the core problem. Onboarding difficulty is a consequence of inconsistency. The pattern: without shared rules, 5 developers make 5 different security decisions for similar features, creating chaos. With Constitution, they make the SAME decision automatically. The scaling principle: more developers → greater need for shared rules. At 1 developer, no Constitution needed; at 50 developers, Constitution essential.",
      source: "Lesson 5: Your Team Needs Shared Rules"
    },
    {
      question: "An Architectural Decision Record (ADR) states: 'We chose JWT for password reset tokens instead of sessions because we need stateless scaling across multiple servers.' What distinguishes this from a Constitution entry?",
      options: [
        "ADRs explain technical choices; Constitutions enforce quality",
        "ADRs document mutable feature decisions; Constitutions define immutable principles",
        "ADRs are optional documentation; Constitutions are mandatory",
        "ADRs describe past decisions; Constitutions guide future"
      ],
      correctOption: 1,
      explanation: "ADRs document mutable decisions that might change ('JWT for password reset' could become 'sessions with Redis' later based on different needs), while Constitutions define immutable principles rarely changing ('ALL passwords use bcrypt cost 12+' never changes). ADRs answer 'Why did we choose THIS approach?' for feature-specific decisions. Constitutions answer 'What rules apply to EVERYTHING?' While ADRs DO explain technical choices and Constitutions enforce quality, this doesn't capture the mutability distinction. ADRs aren't optional—they're mandatory for significant architectural decisions that future developers will question. Both ADRs and Constitutions guide future decisions (not just document past ones). The key: Constitution = system-wide + immutable; ADR = feature-specific + mutable.",
      source: "Lesson 5: Your Team Needs Shared Rules"
    },
    {
      question: "A feature specification includes 'Password hashing: bcrypt cost 12+ (per Constitution).' What does this parenthetical reference demonstrate?",
      options: [
        "Feature specs cannot override constitutional system rules",
        "Constitutional requirements must be explicitly referenced every",
        "Specifications implement Constitution rules ensuring consistency across features",
        "Specifications depend on Constitution for security requirements"
      ],
      correctOption: 2,
      explanation: "The '(per Constitution)' reference demonstrates that feature-specific specs IMPLEMENT the Constitution's system-wide rules, ensuring consistency. The Constitution says 'ALL passwords use bcrypt,' and this feature spec shows compliance by explicitly using bcrypt cost 12+. It's not that references are required (though they help traceability), but that specs align with constitutional rules. Feature specs CAN'T override Constitution—that's implied by the hierarchy, but the pattern shown is implementation-of-rules, not hierarchy itself. Specs don't 'depend on' Constitution for security (security requirements exist independently)—the Constitution ENFORCES consistent interpretation. The mechanism: Constitution defines policy ('bcrypt always'), feature specs implement policy ('bcrypt cost 12+ here').",
      source: "Lesson 5: Your Team Needs Shared Rules"
    },
    {
      question: "Why does SDD recommend writing user stories in 'As a [user], I want to [action], so that [benefit]' format before defining technical acceptance criteria?",
      options: [
        "Stories provide context for AI code generation",
        "This format is standard in Agile practices",
        "User stories help non-technical stakeholders understand requirements",
        "User stories express intent and value before implementation"
      ],
      correctOption: 3,
      explanation: "User stories express intent (what users want to accomplish) and value (why they want it) BEFORE jumping to implementation details (how to build it). Starting with WHY (user needs) forces user-centric thinking before technical decisions. The format ('As a [user], I want [action], so that [benefit]') makes intent explicit: 'As a developer, I want to divide two numbers, so that I can calculate quotients' explains the purpose before defining divide(a, b) signature. While this IS Agile standard format, that's adoption, not reasoning. Stakeholder understanding is a benefit, not the primary purpose. AI context is helpful but not the design goal. The progression: user value → acceptance criteria → edge cases → spec ensures features serve actual needs, not just technical possibilities.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "A Performance Optimization Subagent's Persona states: 'Think about performance the way a systems architect thinks about scalability: What happens when data grows 1000x?' What does this analogy activate?",
      options: [
        "Systems architecture perspective on performance problems overall",
        "Scalability thinking preventing premature optimization for current data",
        "Growth mindset considering future data volumes proactively",
        "Mathematical analysis of algorithmic complexity scaling"
      ],
      correctOption: 1,
      explanation: "The '1000x growth' analogy activates scalability thinking that prevents premature optimization: it forces asking 'Is this optimization valuable (actual bottleneck at scale) or waste (non-problem even at 1000x)?' Example: optimizing for 10 users → 10,000 users requires different approach than 10 → 10 million. The Persona establishes a thinking pattern: analyze performance relative to expected scale, not absolute. While systems architecture perspective IS activated, scalability-focused thinking is the specific stance. Growth mindset is related but vaguer. Mathematical complexity analysis is one tool within scalability thinking, not the activated mindset. The design: Personas should establish HOW to think ('systems architect evaluating scale'), not just WHAT to know ('performance engineering').",
      source: "Lesson 7: Designing Skills and Subagents"
    },
    {
      question: "A specification includes GIVEN-WHEN-THEN formatted acceptance criteria. What advantage does this structured format provide?",
      options: [
        "Makes criteria testable by establishing context, action, outcome",
        "Follows standard behavior-driven development practices globally",
        "Helps AI agents understand expected behavior patterns",
        "Provides clear structure for non-technical stakeholders"
      ],
      correctOption: 0,
      explanation: "GIVEN-WHEN-THEN format makes criteria testable by establishing: GIVEN [initial context] → WHEN [action taken] → THEN [expected outcome]. Example: 'GIVEN calculator initialized, WHEN divide(10, 0) called, THEN raises ZeroDivisionError with message \"Cannot divide by zero\"' is directly testable. This removes ambiguity—you can write an automated test matching this exactly. While this IS BDD standard practice, that's convention, not the advantage. AI DOES understand this better, but that's a consequence of clarity, not the purpose. Stakeholder clarity helps, but testability is the design goal. The pattern: acceptance criteria should be specific enough that you could write tests proving success. GIVEN-WHEN-THEN forces that specificity.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "During specification quality iteration, a developer asks AI: 'Review my spec for ambiguities. What assumptions would you make if these aren't specified?' What role is the AI playing in this interaction?",
      options: [
        "AI as validator checking constitutional compliance",
        "AI as implementer preparing to generate code",
        "AI as teacher explaining specification best practices",
        "AI as co-reviewer identifying gaps before implementation"
      ],
      correctOption: 3,
      explanation: "In this interaction, AI plays co-reviewer role: identifying specification gaps by revealing what assumptions IT would make if things aren't specified. This catches ambiguities before implementation. Example: AI responds 'Spec doesn't say if 1 + 1.0 returns int or float—I'd assume float for consistency.' Developer then refines spec: 'Preserve input types: int + int = int, float + float = float, int + float = float.' The convergence loop: AI identifies gaps → Developer provides design decisions → Spec improves. AI isn't preparing to implement (that comes later). AI isn't teaching best practices (it's asking context-specific questions). AI isn't checking constitutional compliance (different validation type). The pattern: using AI to validate specification clarity without implementing yet.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "A Constitution includes: 'All API specs MUST define error response format.' An ADR explains: 'For this feature, we use {\"error\": string, \"code\": int} because our mobile app parses this format.' Why are both needed?",
      options: [
        "Constitution applies globally; ADR applies to this",
        "Constitution defines requirements; ADR provides implementation details",
        "Constitution enforces consistency; ADR explains feature-specific reasoning",
        "Constitution is immutable; ADR can change later"
      ],
      correctOption: 2,
      explanation: "Both are needed because they serve different purposes: Constitution enforces that ALL features define error responses (consistency requirement), while ADR explains WHY THIS feature uses a SPECIFIC format (feature-specific reasoning: mobile app compatibility). Constitution doesn't say which format, just that some format must exist. ADR documents the choice and reasoning for this context. While ADRs DO provide implementation-level details and Constitutions define requirements, this misses the consistency vs. feature-specific distinction. The global vs. project statement is close but incomplete—ADRs aren't just 'this project,' they're specific decisions within the constitutional framework. Mutability is a characteristic but not the purpose. The relationship: Constitution = quality standard (must have error format); ADR = design decision (why this error format).",
      source: "Lesson 5: Your Team Needs Shared Rules"
    },
    {
      question: "A pattern appears: 'API validation' recurs in 8 specs, each with 3 decision points (types? ranges? error handling?). Should this become a Skill or a Subagent?",
      options: [
        "Subagent, because pattern recurs across eight different specifications",
        "Skill, because three decision points falls within two-to-four",
        "Skill, because validation is simple pattern not complex",
        "Subagent, because pattern requires autonomous decision-making capability"
      ],
      correctOption: 1,
      explanation: "This should be a Skill because it has 3 decision points, falling within the 2-4 decision threshold for Skills. The decision matrix: Skills provide guidance for 2-4 decisions (human decides), Subagents provide autonomous reasoning for 5+ decisions (AI decides). Recurrence frequency (8 specs) justifies creating reusable intelligence but doesn't determine Skill vs. Subagent—decision complexity does. Pattern simplicity is related to decision count but isn't the architectural threshold. Autonomous decision-making is Subagent's characteristic, but this pattern doesn't require it (3 decisions manageable with human-guided framework). The design rule: count decision points → 2-4 = Skill, 5+ = Subagent. Here: types? ranges? error handling? = 3 decisions = Skill.",
      source: "Lesson 6: Introduction to Reusable Intelligence"
    },
    {
      question: "What is the primary difference between horizontal reusability (Skills) and vertical reusability (Subagents) in Reusable Intelligence design?",
      options: [
        "Skills provide guidance; Subagents make decisions autonomously",
        "Skills have simpler logic; Subagents have complex",
        "Skills are reusable templates; Subagents are autonomous",
        "Skills apply broadly across domains; Subagents specialize in domains"
      ],
      correctOption: 0,
      explanation: "The primary difference is autonomy: Skills provide guidance where humans decide (2-4 decision points manageable by human), Subagents make autonomous decisions (5+ decision points require AI reasoning). Example: Input Validation Skill (guidance: 'Consider types, ranges, error handling, strict/lenient') vs. Performance Optimization Subagent (autonomous: analyzes volumes, latency, N+1 risks, caching, memory, structures and makes recommendations). While Skills OFTEN apply broadly and Subagents OFTEN specialize vertically, this isn't definitional—Skills can be domain-specific, Subagents can be horizontal. Simplicity vs. complexity follows from decision count. Skills aren't just templates—they guide reasoning. The architectural distinction: decision point count determines whether human guidance suffices (Skill) or autonomous reasoning needed (Subagent).",
      source: "Lesson 6: Introduction to Reusable Intelligence"
    },
    {
      question: "Why does P+Q+P pattern (Persona + Questions + Principles) activate reasoning mode instead of pattern-matching mode in AI?",
      options: [
        "Pattern follows established AI prompting best practices",
        "Three-part structure provides comprehensive guidance for decisions",
        "Persona sets cognitive stance; Questions force context analysis systematically",
        "Components together cover all decision dimensions needed"
      ],
      correctOption: 2,
      explanation: "P+Q+P activates reasoning because: Persona establishes cognitive stance (defensive programming specialist thinking about attack surfaces, not generic 'expert'), Questions force context-specific analysis ('What are YOUR valid types? YOUR ranges? YOUR constraints?'), and Principles provide decision frameworks (not vague 'best practices'). Compare: 'Make it secure' (pattern-matching: retrieve generic security patterns) vs. P+Q+P: Persona (security auditor mindset), Questions (What threats? What data? What standards?), Principles (fail secure, least privilege). While three-part structure IS comprehensive, comprehensiveness alone doesn't activate reasoning—contextual forcing does. Following best practices is adoption, not mechanism. Covering dimensions is a result, not the cause. The shift: pattern-matching retrieves training data, reasoning analyzes YOUR specific context.",
      source: "Lesson 7: Designing Skills and Subagents"
    },
    {
      question: "Spec-Kit Plus includes 'Intelligence Templates' for domains like education, healthcare, fintech. What problem do these templates solve?",
      options: [
        "Accelerate onboarding of new team members",
        "Provide standardized formats for domain-specific specifications",
        "Ensure compliance with domain regulatory requirements automatically",
        "Prevent teams from rebuilding domain expertise from scratch repeatedly"
      ],
      correctOption: 3,
      explanation: "Intelligence Templates prevent rebuilding domain expertise repeatedly. Example: Healthcare template includes HIPAA rules, patient data handling, consent management—new healthcare teams start with this knowledge encoded rather than discovering it through mistakes. Education template includes Bloom's taxonomy, CEFR proficiency levels, cognitive load principles—new education teams don't reinvent learning science. Templates aren't just standardized formats (that's passive structure)—they encode domain expertise (active knowledge). They HELP with compliance but don't ensure it automatically (compliance still requires human judgment). They DO accelerate onboarding, but that's a consequence of encoded expertise, not the primary purpose. The value: institutional knowledge captured once, reused across teams, preventing each team from learning the same lessons independently.",
      source: "Lesson 8: SDD-RI Framework Landscape"
    },
    {
      question: "A solo developer building side projects asks: 'Should I use Kiro (simple) or Spec-Kit Plus (comprehensive governance)?' What context factor matters most?",
      options: [
        "Team size: solo developers rarely need constitutional governance",
        "Project complexity determines appropriate framework overhead trade-offs",
        "AI agent usage requires Spec-Kit Plus features",
        "Side projects don't justify specification-driven development overhead"
      ],
      correctOption: 0,
      explanation: "Team size matters most: solo developers don't need Constitutions enforcing consistency (no one to be inconsistent with), ADRs explaining decisions to future team members (you'll remember your own reasoning), or PHRs for organizational learning (you're the only learner). Kiro's simplicity (requirements → design → tasks → code) fits solo contexts. While project complexity DOES matter (complex projects might justify governance), team size is the primary factor. AI agent usage doesn't require Spec-Kit Plus—you could use AI with any framework. Side projects CAN justify SDD (learning, maintaining code over time), just not necessarily comprehensive governance. The scaling principle: 1 developer = minimal governance need; 5 developers = moderate governance; 50 developers = comprehensive governance essential. Context determines tool fit.",
      source: "Lesson 8: SDD-RI Framework Landscape"
    },
    {
      question: "During edge case specification, a developer documents: '0.1 + 0.2 returns 0.30000000000000004 (IEEE 754 floating-point precision).' Why document expected imprecision rather than requiring exact 0.3?",
      options: [
        "Floating-point imprecision is unavoidable in all languages",
        "Specifications should match actual implementation behavior not ideal",
        "Documentation prevents user confusion about calculated results",
        "AI agents need explicit precision expectations defined"
      ],
      correctOption: 1,
      explanation: "This documents actual behavior rather than ideal behavior because specifications define WHAT implementations should do, not what we wish they did. IEEE 754 floating-point arithmetic produces 0.30000000000000004—specifying 'must return exactly 0.3' would be impossible to implement without arbitrary-precision libraries (design tradeoff). The spec makes implementation reality explicit rather than hiding it. While floating-point imprecision IS unavoidable (in standard arithmetic), that's WHY we document it, not what makes documentation valuable. User confusion prevention is a benefit. AI needing explicit expectations is true but misses the deeper principle: specifications align with actual computational reality. If you need exact decimal arithmetic, specify: 'Use Decimal class for exact precision' (different design decision). Edge cases reveal where computational reality diverges from mathematical ideals.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "A Performance Optimization Subagent's Principles include: 'Optimize for actual use case, not theoretical worst case.' What does this principle prevent?",
      options: [
        "Worst-case performance scenarios that rarely occur practically",
        "Over-engineering systems beyond current requirements and scale",
        "Theoretical analysis without practical measurement and profiling",
        "Premature optimization wasting effort on non-problems creating complexity"
      ],
      correctOption: 3,
      explanation: "This principle prevents premature optimization: if your system handles 10 users, optimizing for 1 billion is wasted effort adding complexity without benefit. The principle says: profile first (measure actual bottlenecks), then optimize based on REAL constraints (not theoretical). Example: Don't optimize query complexity from O(n) to O(log n) if n never exceeds 10—the optimization costs more (development time, code complexity) than it saves. While this DOES prevent over-engineering, that's the general problem; premature optimization is the specific anti-pattern. Preventing 'theoretical analysis without measurement' is the mechanism (profile first). Preventing 'worst-case scenarios that rarely occur' is the application. The insight: optimization without data (profiling) is cargo cult engineering. The guidance: make performance decisions based on measured reality, not assumed scale.",
      source: "Lesson 7: Designing Skills and Subagents"
    },
    {
      question: "Why did Design by Contract (1980s) fail to gain widespread adoption despite promising specs embedded in code?",
      options: [
        "Contract syntax was too complex for teams",
        "Developers found embedded contracts too difficult understand",
        "Language-specific implementation prevented broad adoption across technology stacks",
        "Testing frameworks made contracts redundant in practice"
      ],
      correctOption: 2,
      explanation: "Design by Contract failed because it was language-specific (only Eiffel had native support), preventing adoption across technology stacks. Teams using Python, Java, C++ couldn't use Design by Contract without special libraries, fragmenting the approach. The lesson learned: specs need separation from implementation (not embedded in code) to work across languages and contexts. Embedded contracts weren't 'too difficult to understand'—the syntax wasn't the problem. Complexity wasn't the barrier (it was less complex than Formal Methods). Testing frameworks didn't make contracts redundant—they serve different purposes (tests verify behavior, contracts specify guarantees). The historical insight: language lock-in kills specification approaches. SDD succeeds where Design by Contract failed because natural language specs work with ANY language and ANY LLM.",
      source: "Lesson 2: Why Do Specs Matter NOW?"
    },
    {
      question: "What is the significance of the '2-4 decision points for Skills, 5+ for Subagents' threshold in RI design?",
      options: [
        "Simple patterns versus complex patterns requiring expertise",
        "Human-manageable decisions versus complexity requiring autonomous reasoning",
        "Reusable guidance versus domain-specific specialized knowledge",
        "Template-based approaches versus AI-powered decision-making"
      ],
      correctOption: 1,
      explanation: "The 2-4 vs. 5+ threshold distinguishes human-manageable decision complexity from complexity requiring autonomous AI reasoning. With 2-4 decisions (Input Validation: types? ranges? error handling? strict/lenient?), a human can evaluate tradeoffs using guidance (Skill). With 5+ decisions (Performance: volumes? latency? N+1? caching? memory? structures?), the decision space becomes too complex for simple guidance—autonomous reasoning needed (Subagent). While Skills ARE typically simpler and Subagents more complex, this follows from decision count. Skills CAN be domain-specific (not just reusable), Subagents CAN apply broadly (not just specialized). Skills aren't just templates—they guide reasoning. The architectural insight: cognitive load (decision count) determines whether human guidance suffices or autonomous AI reasoning required.",
      source: "Lesson 6: Introduction to Reusable Intelligence"
    },
    {
      question: "A Constitution states: 'All temporary access tokens MUST specify time-based expiry. Default: 30 minutes unless otherwise justified.' Why include the 'unless otherwise justified' clause?",
      options: [
        "Allows context-specific deviations with explicit reasoning documented",
        "Provides flexibility for features with special requirements",
        "Prevents Constitution from being too rigid and",
        "Enables teams to override rules when necessary"
      ],
      correctOption: 0,
      explanation: "The 'unless otherwise justified' clause allows context-specific deviations BUT requires explicit reasoning documented (typically in ADR). Example: 'Password reset tokens: 30 minutes (default). OAuth refresh tokens: 7 days (justified: user convenience vs. security tradeoff for this use case, documented in ADR-042).' The Constitution enforces (1) you MUST consider expiry, (2) default is 30 minutes, (3) deviations require documented justification. This isn't generic flexibility—it's controlled deviation with reasoning. Preventing rigidity is a consequence, not the purpose. 'Override rules when necessary' misses the 'documented justification' requirement—you can't just ignore the rule, you must explain why this context merits different approach. Constitutional governance with reasoned flexibility prevents both inconsistency (no rules) and inflexibility (rigid rules ignoring context).",
      source: "Lesson 5: Your Team Needs Shared Rules"
    },
    {
      question: "During collaborative spec-writing, AI suggests: 'Should I add runtime isinstance() checks for type validation, or rely on type hints with mypy?' What specification gap does this question reveal?",
      options: [
        "Error handling for invalid types was missing",
        "Type validation approach wasn't specified in requirements",
        "Spec didn't define validation strategy: static versus runtime",
        "Specification lacked technical implementation guidance details"
      ],
      correctOption: 2,
      explanation: "This question reveals the spec didn't define validation strategy: should validation happen at runtime (isinstance() checks executed during code) or statically (mypy checks during development)? This is a design decision with tradeoffs: runtime validation catches errors at execution (slower, catches dynamic issues), static validation catches errors during development (faster, catches type mismatches pre-deployment). The developer should clarify: 'Rely on Python's static type checking (mypy). No runtime isinstance() checks needed (trust type system).' This closes the gap. While 'type validation approach wasn't specified' is true, the specific gap is static vs. runtime strategy. Error handling is related but not the revealed gap. 'Lacking technical guidance' is too vague—the specific gap is validation timing strategy.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "A Reusable Intelligence component appears in your codebase. How do you determine if it's a Skill or a Subagent by examining its structure?",
      options: [
        "Determine if it uses templates or AI",
        "Check if it provides guidance versus generates",
        "Evaluate if it's domain-specific versus broadly",
        "Count decision points: two-to-four equals Skill; five-plus equals"
      ],
      correctOption: 3,
      explanation: "The architectural classification method: count decision points in the component's Questions section. If Questions cover 2-4 independent decisions requiring guidance, it's a Skill ('types? ranges? error handling? strict/lenient?' = 4 decisions = Skill). If Questions cover 5+ independent decisions requiring autonomous reasoning, it's a Subagent ('volumes? latency? N+1? caching? memory? structures?' = 6 decisions = Subagent). Checking if it 'provides guidance vs. generates code' misunderstands both—neither Skills nor Subagents generate code directly; both guide specifications. Domain-specificity doesn't determine classification (Skills can be domain-specific, Subagents can be horizontal). Template vs. AI reasoning isn't the distinction (both use AI reasoning via P+Q+P). The decision framework: decision complexity (counted as decision points) determines Skill vs. Subagent classification.",
      source: "Lesson 6: Introduction to Reusable Intelligence"
    },
    {
      question: "Why does Tesel (specs-as-source-of-truth) make generated code read-only, preventing developers from hand-editing implementations?",
      options: [
        "Prevents code-spec divergence ensuring specifications remain single truth",
        "Guarantees code quality through automated generation only",
        "Simplifies maintenance by eliminating manual code changes",
        "Enables rapid regeneration when specifications change later"
      ],
      correctOption: 0,
      explanation: "Tesel makes code read-only to prevent code-spec divergence: if developers can hand-edit generated code, code and spec will inevitably drift (spec says X, code does Y). Making code read-only enforces: ALL changes happen in spec → code regenerated → spec remains single source of truth. This is crucial for safety-critical systems (aerospace, medical devices) where spec-code divergence is unacceptable. While this DOES guarantee quality through automation and enable rapid regeneration, preventing divergence is the architectural principle. Simplifying maintenance is a benefit, not the purpose. The tradeoff: you sacrifice flexibility (can't quick-fix code) for certainty (code always matches spec). Tesel takes spec-driven development to logical extreme: specs aren't just primary, they're the ONLY source of truth.",
      source: "Lesson 8: SDD-RI Framework Landscape"
    },
    {
      question: "A specification includes: 'Out of Scope: Input validation beyond type hints. We rely on Python's static type checking (mypy). No runtime isinstance() checks.' Why explicitly state what's out of scope?",
      options: [
        "Clarifies scope boundaries for future developers reviewing",
        "Prevents AI from implementing features you deliberately excluded",
        "Reduces specification length by listing exclusions upfront",
        "Helps stakeholders understand what feature doesn't include"
      ],
      correctOption: 1,
      explanation: "Explicitly stating out-of-scope prevents AI from implementing features you deliberately excluded. Without 'Out of Scope' section, AI might see 'validate input types' and add runtime isinstance() checks thinking 'validation should be comprehensive.' The spec saying 'Out of Scope: runtime validation' tells AI: 'We consciously chose NOT to do this—don't add it.' This is especially important for decisions that seem incomplete: relying on static types without runtime checks LOOKS like a gap unless you explain it's intentional. While clarifying boundaries for future developers IS valuable, preventing incorrect implementation is the immediate purpose. Reducing length isn't a goal (clarity matters more than brevity). Stakeholder understanding helps, but the primary audience is AI generating code. Out-of-scope sections make INTENTIONAL exclusions explicit.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "During P+Q+P design, a Security Review Subagent's Questions include: 'What's the breach cost if this system is compromised? (Reputation loss? Revenue loss? Regulatory fines?)' Why ask about breach cost?",
      options: [
        "Financial impact guides compliance requirement interpretation clearly",
        "Understanding consequences helps prioritize security measures effectively",
        "Cost analysis enables risk-based security decisions systematically",
        "Breach cost determines appropriate security investment level justified"
      ],
      correctOption: 3,
      explanation: "Asking about breach cost determines appropriate security investment: if breach costs $100 (reputation hit to side project), you don't invest $10,000 in security infrastructure; if breach costs $10 million (HIPAA fines + patient data leak), you DO invest heavily. Security decisions are always tradeoffs—this question forces context-aware reasoning: 'Is this threat defense worth the cost given OUR breach consequences?' Example: solo developer building public blog → breach cost low → simple security suffices. Healthcare startup with patient data → breach cost catastrophic (HIPAA fines, trust loss, lawsuit risk) → comprehensive security essential. While breach cost DOES help prioritize, determine investment level, enable risk-based decisions, and guide compliance, the architectural purpose is establishing appropriate security investment justified by context. Prevents both under-securing (ignoring real risks) and over-securing (gold-plating against unlikely threats).",
      source: "Lesson 7: Designing Skills and Subagents"
    },
    {
      question: "What is the relationship between Constitutions and Reusable Intelligence components (Skills/Subagents)?",
      options: [
        "Constitution applies globally; RI components apply to contexts",
        "Constitution enforces quality; RI components implement patterns repeatedly",
        "Constitution defines system-wide rules; RI components encode domain expertise",
        "Constitution prevents inconsistency; RI components enable reusability"
      ],
      correctOption: 2,
      explanation: "The relationship: Constitution defines system-wide rules applying to ALL features ('All passwords use bcrypt,' 'All API specs define error responses'), while RI components encode domain expertise as reusable patterns (Input Validation Skill, Performance Optimization Subagent). Constitution is governance (what rules apply everywhere?), RI is expertise (what patterns recur, how do we encode them?). They're complementary: Constitution ensures consistency (everyone follows same security rules), RI provides efficiency (don't rebuild validation logic 30 times). While Constitution enforces quality and RI implements patterns, this misses the governance vs. expertise distinction. Global vs. specific contexts is related but incomplete—RI components can be broadly applicable (horizontal) or domain-specific (vertical). Preventing inconsistency vs. enabling reusability describes outcomes, not the fundamental relationship. The architecture: Constitution = mandatory rules, RI = encoded expertise.",
      source: "Lesson 6: Introduction to Reusable Intelligence"
    },
    {
      question: "A developer writes acceptance criteria: 'System should handle large numbers efficiently.' Why is this criterion weak?",
      options: [
        "Not testable: lacks specific threshold defining success measurably",
        "Too vague: doesn't specify what large means",
        "Missing context: doesn't explain why efficiency matters",
        "Incomplete: doesn't specify performance requirements explicitly"
      ],
      correctOption: 0,
      explanation: "This criterion is weak because it's not testable—you can't write a test proving success without a specific threshold. 'Handle large numbers efficiently' has no measurable definition: What's 'large' (1000? 1,000,000? 10^100?)? What's 'efficiently' (<100ms? <1s? <1hr?)? Strong acceptance criteria: 'GIVEN calculator initialized, WHEN multiply(999999, 999999), THEN result returned in <50ms.' This is testable—you can write a test proving success or failure. While vagueness is the problem, 'not testable' captures the specification failure most precisely. Missing context about why efficiency matters is a separate issue. Incomplete performance requirements is true but doesn't capture the testability issue specifically. The design principle: acceptance criteria should be specific enough that you can write automated tests proving them true or false.",
      source: "Lesson 3: What is SDD"
    },
    {
      question: "Spec-Kit Plus is described as 'Spec-Kit foundation + Intelligence Layers for AI-native teams.' What makes it specifically suited for AI-native development compared to traditional development?",
      options: [
        "Provides governance and traceability required for AI",
        "ADRs capture reasoning; PHRs log AI interactions for",
        "Intelligence Templates guide AI agent decision-making effectively",
        "Combines human reasoning with AI interaction tracking"
      ],
      correctOption: 1,
      explanation: "Spec-Kit Plus suits AI-native development because: ADRs capture architectural reasoning (why we chose THIS approach—important when AI agents implement multiple features and need to understand past decisions), PHRs log ALL AI interactions (what prompt → what response → accepted/modified/why—enables learning 'Prompt A produced vulnerable code, Prompt B fixed it, use B next time'), and Intelligence Templates encode domain expertise (AI agents read healthcare template, know HIPAA rules before coding). Traditional development doesn't need PHRs (no AI interactions to log) or reasoning-activated templates (humans apply domain knowledge intuitively). While governance and traceability ARE required, the AI-specific features are ADR-guided agent decisions and PHR-logged interaction learning. Templates DO guide AI, but the complete system (ADRs + PHRs + Templates together) is what makes it AI-native.",
      source: "Lesson 8: SDD-RI Framework Landscape"
    },
    {
      question: "Why is convergence at specification level (before implementation) more valuable than convergence at code level (after implementation)?",
      options: [
        "Collaboration at spec level produces better understanding",
        "Specifications are easier to change than implementations",
        "Earlier feedback reduces cost of changes later",
        "Prevents iteration waste by clarifying requirements before code generation"
      ],
      correctOption: 3,
      explanation: "Convergence at spec level prevents iteration waste: when AI asks 'Does subtract(5, 3) return 2 or -2?' BEFORE generating code, you clarify once and code generated correctly. Convergence at code level means: AI generates code implementing wrong assumption → you test → discover error → AI regenerates → test again. The vague prompt problem repeated. While specs ARE easier to change than code, that's a mechanism, not the value. Earlier feedback reducing cost is related but doesn't capture the iteration prevention specifically. Collaboration producing better understanding is true but not the efficiency advantage. The pattern: spec convergence (AI asks → you clarify → spec improves → code generated correctly) vs. code convergence (AI generates → you test → find error → regen iterate). Spec convergence is cheaper because clarification happens BEFORE expensive generation.",
      source: "Lesson 4: Build Your First Spec Together"
    },
    {
      question: "A Performance Optimization Subagent analyzes a feature and recommends: 'N+1 query risk detected. Expected impact: 1000+ queries per user. Recommend eager loading with join.' What demonstrates this is autonomous reasoning, not pattern matching?",
      options: [
        "Identification of N+1 problem through code review",
        "Recommendation based on feature-specific query analysis performed",
        "Context-specific analysis combining data volumes, query patterns, impact",
        "Performance impact calculation showing thousand-query problem scale"
      ],
      correctOption: 2,
      explanation: "This demonstrates autonomous reasoning because the Subagent: (1) analyzed context ('THIS feature's data volumes'), (2) identified pattern ('N+1 query risk in THIS code'), (3) calculated impact ('1000+ queries per user' based on expected scale), and (4) recommended solution ('eager loading with join' matching THIS context). Pattern matching would retrieve 'N+1 is bad, use eager loading' without analyzing whether it applies HERE. Reasoning evaluates: Is this ACTUALLY an N+1 problem? What's the impact given OUR volumes? Is optimization justified? Recommendation alone doesn't show reasoning (could be pattern-matching template). Identification alone doesn't show reasoning (could be pattern recognition). Impact calculation is part of reasoning but not the complete demonstration. The distinction: reasoning analyzes YOUR specific context; pattern matching retrieves generic advice.",
      source: "Lesson 7: Designing Skills and Subagents"
    }
  ]}
  questionsPerBatch={18}
/>

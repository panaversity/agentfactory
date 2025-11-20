---
sidebar_position: 10
title: "Chapter 12: Context Engineering Quiz"
---

# Chapter 12: Context Engineering for AI-Driven Development Quiz

Test your understanding of context window management, progressive loading strategies, compression techniques, memory file architecture, and tool selection frameworks covered in this chapter.

<Quiz
  title="Chapter 12: Context Engineering Assessment"
  questions={[    {
      question: "A developer notices Claude repeating previous suggestions and missing recent changes. The session has 150,000 tokens with 50,000 tokens in the most recent 20 messages. What does this symptom primarily indicate?",
      options: [
        "The context window has exceeded capacity entirely",
        "The AI model needs a complete restart",
        "Recent context is being deprioritized during processing",
        "Token counting tools are providing inaccurate estimates"
      ],
      correctOption: 2,
      explanation: "When Claude repeats previous suggestions despite recent changes, this indicates degradation where recent context is deprioritized. The lesson emphasizes that degradation manifests as the AI favoring older context over newer information, even when total tokens remain within capacity. Option A is incorrect because the session is still within the 200K limit—capacity hasn't been exceeded. Option C is wrong because restart isn't always necessary; compression or progressive loading may suffice. Option D misses the point—the symptom is about context processing priority, not measurement accuracy. Recognizing this degradation pattern helps developers intervene before session quality deteriorates further.",
      source: "Lesson 2: Degradation Symptoms and Manual Tracking"
    },
    {
      question: "You're implementing progressive loading for a large refactoring project. Which content should be loaded in the Foundation phase according to the three-phase strategy?",
      options: [
        "Project architecture, core conventions, and system constraints",
        "Current task requirements and recent code changes",
        "Detailed implementation code for specific features only",
        "Debugging logs and recent error messages exclusively"
      ],
      correctOption: 0,
      explanation: "The Foundation phase establishes the baseline understanding that persists throughout the session. The lesson specifies that Foundation content includes project architecture, coding conventions, system constraints, and high-level structure—information that doesn't change frequently. Option A describes Current phase content (task-specific requirements). Option C describes On-Demand phase content (detailed implementation loaded as needed). Option D is too narrow and specific, missing the architectural baseline. The Foundation→Current→On-Demand strategy ensures the AI always has architectural context while loading task-specific details progressively. This prevents context window overload while maintaining consistent understanding.",
      source: "Lesson 3: Progressive Loading Strategy"
    },
    {
      question: "During a debugging session, you've accumulated 180,000 tokens with multiple failed attempts documented. You've identified the root cause. What's the most effective next action?",
      options: [
        "Continue in current session to maintain context",
        "Switch to a different AI tool",
        "Delete only the most recent messages",
        "Create checkpoint summary and start fresh session"
      ],
      correctOption: 3,
      explanation: "Creating a checkpoint summary and starting fresh is the optimal strategy when you've identified a solution after extensive debugging. The lesson emphasizes that checkpoints preserve critical discoveries (root cause, attempted solutions, key insights) while eliminating noise from failed attempts. Option A is inefficient—continuing with 180K tokens of mostly irrelevant debugging history degrades performance. Option C (deleting recent messages) loses valuable context and doesn't address the underlying token bloat. Option D (switching tools) is unnecessary and loses all accumulated knowledge. Checkpoint creation is the bridge between learning from extensive debugging and executing the solution efficiently in a clean context.",
      source: "Lesson 4: Context Compression and Session Restart"
    },
    {
      question: "You're managing simultaneous documentation updates and API refactoring. Both tasks require AI assistance. How should you structure your context to prevent cross-contamination?",
      options: [
        "Use single session with clear markers",
        "Use separate isolated sessions for each task",
        "Alternate between tasks within same conversation thread",
        "Complete documentation first, then start API work"
      ],
      correctOption: 1,
      explanation: "Context isolation through separate sessions is essential when tasks have different requirements and vocabularies. The lesson emphasizes that documentation and API refactoring involve different contexts—mixing them causes the AI to conflate concerns, apply wrong patterns, or make inappropriate references. Option A (single session with markers) doesn't prevent context bleeding—the AI still sees all previous messages. Option C (alternating tasks) exacerbates contamination by constantly switching context. Option D (sequential completion) is unnecessarily rigid and doesn't address the isolation principle. Parallel isolated sessions ensure each task maintains its own clean context, preventing the AI from mixing documentation tone with code logic or vice versa.",
      source: "Lesson 5: Context Isolation and Parallel Tasks"
    },
    {
      question: "A new developer joins your project and starts an AI session. Which memory file should Claude load first to understand how the team collaborates with AI?",
      options: [
        "architecture.md describing technical system design patterns",
        "decisions.md documenting past architectural trade-off justifications",
        "CLAUDE.md explaining AI collaboration conventions and workflows",
        "README.md providing general project setup instructions"
      ],
      correctOption: 2,
      explanation: "CLAUDE.md is the collaboration contract that defines how humans and AI work together on this specific project. The lesson specifies that CLAUDE.md contains conventions, expectations, workflows, and project-specific interaction patterns—essential for establishing productive collaboration from the start. Option A (architecture.md) describes technical systems, not collaboration processes. Option B (decisions.md) documents past choices, not current workflows. Option D (README.md) is for general setup, not AI-specific collaboration patterns. Loading CLAUDE.md first ensures the AI understands team conventions before diving into architecture or decisions, preventing misaligned suggestions and establishing the collaboration foundation.",
      source: "Lesson 6: Memory Files and Persistent Intelligence"
    },
    {
      question: "You need to refactor a complex authentication system with extensive existing code. Which tool should you select and why based on the selection framework?",
      options: [
        "Claude Code for deep codebase reasoning capacity",
        "Gemini CLI for its superior multimodal capabilities",
        "Either tool works equally well for this",
        "Gemini CLI for its extended context window"
      ],
      correctOption: 0,
      explanation: "Claude Code excels at deep codebase reasoning, making it ideal for complex refactoring that requires understanding intricate relationships, dependencies, and architectural patterns across multiple files. The lesson emphasizes that Claude Code's strength is contextual understanding and logical reasoning through complex codebases. Option A (multimodal capabilities) is irrelevant—authentication refactoring doesn't require image/video processing. Option C ignores the framework's emphasis on matching tool strengths to task characteristics. Option D (extended context) misses the point—while Gemini has longer windows, Claude Code's reasoning depth matters more for refactoring complexity. The selection framework prioritizes task-tool alignment over generic capabilities.",
      source: "Lesson 7: Tool Selection Framework"
    },
    {
      question: "During API integration, Claude generates code that worked in isolation but fails in the full system. What does this symptom suggest about your context management?",
      options: [
        "The AI model lacks necessary programming knowledge",
        "Integration context was not loaded during generation",
        "Your code has fundamental architectural problems everywhere",
        "Token limits were exceeded during the session"
      ],
      correctOption: 1,
      explanation: "When code works in isolation but fails in integration, it indicates missing integration context during generation. The lesson emphasizes that successful integration requires loading system interfaces, dependencies, environment configuration, and interaction patterns—not just the isolated component. Option A (lacking knowledge) is unlikely; the issue is missing project-specific context, not general programming knowledge. Option C (architectural problems) is an overreach—isolated success suggests architecture isn't fundamentally broken. Option D (token limits) doesn't explain the specific isolation-vs-integration failure pattern. This diagnostic insight helps developers recognize when to use progressive loading to include integration context before generating code.",
      source: "Lesson 8: Hands-on Debugging and Optimization"
    },
    {
      question: "You're writing a specification for a new feature requiring multiple AI sessions coordinated over several days. What should your spec prioritize to ensure consistency?",
      options: [
        "Detailed step-by-step implementation instructions for AI",
        "Comprehensive timeline with specific completion deadlines imposed",
        "Complete code examples for every component needed",
        "Explicit constraints, context dependencies, and success criteria"
      ],
      correctOption: 3,
      explanation: "Effective specifications for multi-session orchestration prioritize constraints, context dependencies, and success criteria—these guide AI decision-making across sessions without micromanaging implementation. The lesson emphasizes that specs should define the 'what' and 'why' clearly while allowing AI flexibility in the 'how.' Option A (step-by-step instructions) creates brittle specs that break when AI encounters unexpected situations. Option C (complete code examples) defeats the purpose of AI generation and can't cover all scenarios. Option D (timelines with deadlines) addresses project management, not spec-driven orchestration quality. Good specifications enable consistent AI collaboration by establishing boundaries and goals, not dictating every decision.",
      source: "Lesson 9: Capstone Spec-Driven Orchestration"
    },
    {
      question: "A session reaches 195,000 tokens. Claude's responses are still accurate but slower. What does the lesson suggest about token counting in this scenario?",
      options: [
        "Token counting has become unreliable near capacity",
        "Manual tracking is more accurate than tools",
        "Token count is approaching critical degradation threshold",
        "The AI is intentionally slowing for accuracy"
      ],
      correctOption: 2,
      explanation: "At 195,000 tokens in a 200K window, you're approaching the threshold where degradation becomes severe. The lesson emphasizes that degradation symptoms (like slower responses) often emerge before hitting absolute capacity, signaling that intervention is needed soon. Option A (counting unreliable) is wrong—tools remain accurate; the issue is proximity to limits. Option B (manual vs tools) misses the point; both would show the same high count. Option D (intentional slowing) misinterprets the symptom—slowness indicates processing strain, not a deliberate strategy. Recognizing this pattern helps developers proactively create checkpoints or restart sessions before quality degrades significantly, rather than waiting for complete failure.",
      source: "Lesson 1: Context Windows and Token Counting"
    },
    {
      question: "You notice Claude forgetting architectural decisions made 100 messages ago despite those decisions being well within the 200K token window. What degradation pattern is this?",
      options: [
        "Effective attention degradation despite technical capacity remaining",
        "Absolute context window capacity has been exceeded",
        "The AI model requires updating its knowledge",
        "Token counting tools have miscalculated session size"
      ],
      correctOption: 0,
      explanation: "This scenario demonstrates effective attention degradation—the AI struggles to attend to older information even when it technically remains within the context window. The lesson distinguishes between absolute capacity (200K tokens) and effective attention (AI's ability to equally weight all context). Option A (capacity exceeded) is contradicted by the scenario stating tokens are within limits. Option C (model knowledge) confuses in-context learning with base model knowledge. Option D (miscalculation) misses the attention mechanism insight. This pattern is critical for understanding why sessions degrade before hitting token limits—attention mechanisms struggle with very long contexts even when capacity technically remains. Recognizing this prompts earlier intervention strategies.",
      source: "Lesson 2: Degradation Symptoms and Manual Tracking"
    },
    {
      question: "For a feature requiring rare database optimization, when should you load the database documentation according to progressive loading principles?",
      options: [
        "Foundation phase with core architecture and conventions",
        "On-Demand phase when optimization is actively needed",
        "Current phase with active task implementation details",
        "Never load it and rely on general knowledge"
      ],
      correctOption: 1,
      explanation: "Rare, specialized documentation like database optimization guides belongs in the On-Demand phase—loaded only when that specific task is active. The lesson emphasizes that On-Demand content is task-specific, detailed, and only relevant for particular moments in development. Option A (Foundation) is wrong because database optimization isn't core architecture needed throughout the session. Option B (Current) would load it too early if optimization isn't the immediate task. Option D (never load) ignores that project-specific optimization patterns may differ from general AI knowledge. The On-Demand strategy prevents context bloat by deferring specialized content until precisely when it's needed, maximizing context window efficiency.",
      source: "Lesson 3: Progressive Loading Strategy"
    },
    {
      question: "After 50 messages debugging a complex issue, you've tried multiple approaches and documented each failure. What should your compression checkpoint prioritize capturing?",
      options: [
        "Every attempted solution with complete error messages",
        "Only the final successful solution if found",
        "Just the initial problem statement and final state",
        "Root cause, key insights, and eliminated approaches"
      ],
      correctOption: 3,
      explanation: "Effective checkpoints balance comprehensiveness with conciseness by capturing root cause analysis, critical insights learned, and eliminated approaches—this prevents repeating failed attempts while preserving learning. The lesson emphasizes that checkpoints should distill extensive exploration into actionable knowledge. Option A (every attempt) reproduces the bloat you're trying to compress. Option B (only success) loses valuable learning from failures—understanding why approaches didn't work prevents future mistakes. Option D (initial and final only) omits the reasoning journey that explains why the solution works. Good compression preserves the intellectual progress while eliminating redundant discussion, enabling fresh sessions to benefit from accumulated learning.",
      source: "Lesson 4: Context Compression and Session Restart"
    },
    {
      question: "You're simultaneously developing a customer-facing API and internal admin tools. Why does the lesson recommend separate sessions rather than task interleaving?",
      options: [
        "Prevents vocabulary and pattern contamination across tasks",
        "Different token limits apply to different projects",
        "AI tools can only handle one task",
        "Security concerns require complete session isolation always"
      ],
      correctOption: 0,
      explanation: "Separate sessions prevent vocabulary and pattern contamination—customer-facing APIs emphasize security, validation, and clear documentation while admin tools prioritize efficiency and debugging access. The lesson emphasizes that context mixing causes the AI to inappropriately apply patterns from one domain to another. Option A (different token limits) is false—limits are per session, not per project. Option C (AI can't multitask) underestimates capability—the issue is quality, not ability. Option D (security) is too narrow; while security matters, the core reason is preventing conceptual contamination. Interleaving these tasks would cause the AI to suggest admin-style direct access in customer APIs or overly restrictive patterns in internal tools.",
      source: "Lesson 5: Context Isolation and Parallel Tasks"
    },
    {
      question: "Your team decides to switch from REST to GraphQL. Which memory file should document this decision and the reasoning behind it?",
      options: [
        "CLAUDE.md since it affects AI collaboration patterns",
        "architecture.md since it changes technical system design",
        "decisions.md since it records architectural trade-offs made",
        "README.md since it updates project setup requirements"
      ],
      correctOption: 2,
      explanation: "decisions.md is the architectural decision record (ADR) that captures significant choices, trade-offs considered, and rationale—exactly what's needed for documenting the REST→GraphQL transition. The lesson specifies that decisions.md preserves the reasoning behind choices, not just the choices themselves. Option A (CLAUDE.md) documents collaboration workflows, not technical decisions. Option B (architecture.md) describes the current state, not the decision history and rationale. Option D (README.md) provides setup instructions, not architectural justification. Recording decisions with rationale prevents future debates by preserving the context and trade-offs that informed the choice, helping AI understand why the current architecture exists.",
      source: "Lesson 6: Memory Files and Persistent Intelligence"
    },
    {
      question: "You're generating extensive documentation with many screenshots and diagrams. Which tool best matches this task according to the selection framework?",
      options: [
        "Claude Code for its superior reasoning capabilities",
        "Gemini CLI for its multimodal content processing",
        "Either tool handles this documentation task equally",
        "Claude Code for its extended memory features"
      ],
      correctOption: 1,
      explanation: "Gemini CLI's multimodal capabilities make it ideal for documentation involving screenshots, diagrams, and visual content—it can process and reference images directly. The lesson emphasizes matching tool strengths to task characteristics: visual content processing is Gemini's clear advantage. Option A (Claude Code reasoning) is valuable for complex logic but doesn't help with image processing. Option C (either works equally) ignores the framework's core principle of strategic tool selection. Option D (extended memory) is irrelevant—documentation with visuals benefits from multimodal processing, not memory capacity. The selection framework guides developers to leverage Gemini when visual content is central to the task.",
      source: "Lesson 7: Tool Selection Framework"
    },
    {
      question: "Claude suggests a performance optimization that contradicts a principle documented in your architecture.md. What does this indicate about your context management?",
      options: [
        "The architecture.md documentation is likely outdated completely",
        "Performance optimizations should always override architectural principles immediately",
        "Claude's base knowledge conflicts with your approach",
        "Memory files were not loaded in session"
      ],
      correctOption: 3,
      explanation: "When Claude contradicts documented architecture, it strongly suggests memory files weren't loaded—without architecture.md, Claude relies on general knowledge rather than project-specific principles. The lesson emphasizes that memory files establish project context; missing them causes generic suggestions. Option A (outdated docs) is possible but less likely than simply not loading them. Option C (base knowledge conflict) misses that loading architecture.md would inform Claude of project-specific approaches. Option D (optimizations override principles) is dangerous—architectural principles usually reflect deliberate trade-offs. This diagnostic helps identify when to reload memory files to align AI suggestions with project conventions.",
      source: "Lesson 8: Hands-on Debugging and Optimization"
    },
    {
      question: "Your specification for a multi-session feature includes 'use appropriate error handling.' Why is this problematic for spec-driven orchestration?",
      options: [
        "Specifications should never mention error handling details",
        "AI cannot implement error handling at all",
        "Error handling decisions need explicit constraints specified",
        "This instruction is perfectly clear for AI"
      ],
      correctOption: 2,
      explanation: "'Appropriate' is vague and subjective—different sessions might interpret it as try-catch blocks, error objects, logging, validation, or user notifications. The lesson emphasizes that effective specs provide explicit constraints and criteria rather than subjective judgments. Option B (AI can't implement) is false—AI handles error handling well with clear guidance. Option C (never mention) is wrong—error handling should be specified, just more precisely. Option D (perfectly clear) ignores that ambiguity causes inconsistency across sessions. Better specifications define error handling expectations explicitly: logging requirements, user-facing messages, retry logic, validation approaches. This ensures consistent implementation across multiple AI sessions.",
      source: "Lesson 9: Capstone Spec-Driven Orchestration"
    },
    {
      question: "You're estimating tokens for loading a 500-line Python file with extensive comments. Which factor most significantly affects the token count?",
      options: [
        "Character count drives estimation more than lines",
        "Programming language choice affects token density significantly",
        "Comments are excluded from token counting entirely",
        "File format determines token calculation methodology completely"
      ],
      correctOption: 0,
      explanation: "Character count is the primary driver of token estimation—roughly 4 characters per token for English text. The lesson emphasizes that token counting approximates based on character length, not line count or language syntax. Option A (language choice) has minor impact; character density matters more. Option C (comments excluded) is false—comments consume tokens just like code. Option D (file format) is wrong; tokens represent textual content regardless of format. A 500-line file with extensive comments likely has more characters (and thus tokens) than a 500-line file with minimal comments. Understanding character-based estimation helps developers accurately predict context window consumption.",
      source: "Lesson 1: Context Windows and Token Counting"
    },
    {
      question: "Claude starts generating overly verbose responses after 120 messages, though earlier responses were concise. What degradation symptom is this?",
      options: [
        "Token capacity exceeded causing random behavior patterns",
        "Lost track of conversation style and norms",
        "The AI model is intentionally being thorough",
        "Memory file instructions are now conflicting badly"
      ],
      correctOption: 1,
      explanation: "Increasing verbosity after many messages indicates lost attention to conversation style established early in the session—the AI no longer effectively attends to initial conciseness norms. The lesson identifies changing response patterns as a key degradation symptom. Option A (capacity exceeded) doesn't explain the specific verbosity pattern. Option C (intentional thoroughness) misinterprets degradation as strategy. Option D (memory file conflict) assumes files were loaded; degradation affects all context, not just files. This pattern signals that effective attention has degraded—the AI struggles to maintain consistency with early conversation norms. Recognizing this prompts intervention: reminders about conciseness or session restart with style guidelines reestablished.",
      source: "Lesson 2: Degradation Symptoms and Manual Tracking"
    },
    {
      question: "You're starting a new feature that builds on existing authentication code. According to progressive loading, which content belongs in the Current phase?",
      options: [
        "Complete authentication system architecture and all patterns",
        "Only the specific authentication endpoints being extended",
        "All previous features to maintain complete context",
        "Active feature requirements and relevant authentication interfaces"
      ],
      correctOption: 3,
      explanation: "The Current phase loads active task requirements plus relevant interfaces from existing systems—in this case, the new feature requirements and authentication interfaces being extended. The lesson specifies Current phase provides immediate task context. Option A (complete system) is too broad and belongs in Foundation (architecture) or On-Demand (detailed implementation). Option B (only endpoints) is too narrow and misses feature requirements. Option D (all previous features) causes unnecessary context bloat. The Current phase balances specificity and relevance: enough authentication context to understand integration points without loading the entire authentication system. This targeted approach maximizes context efficiency.",
      source: "Lesson 3: Progressive Loading Strategy"
    },
    {
      question: "You've completed a major debugging session with a successful solution. The session has 175,000 tokens including extensive exploration. Why compress rather than continue?",
      options: [
        "Clean context improves execution quality and speed",
        "Compression reduces future AI processing costs significantly",
        "Continuing risks exceeding token limits in moments",
        "AI requires fresh starts after solutions"
      ],
      correctOption: 0,
      explanation: "Clean context after compression significantly improves execution quality—without the noise of failed attempts, Claude focuses better on implementing the solution efficiently. The lesson emphasizes that compression optimizes cognitive load, not just token count. Option A (cost) is minor compared to quality benefits; cost savings are a bonus, not the primary reason. Option C (risk of exceeding) is a concern but not the main advantage—you could continue for a bit. Option D (requires fresh starts) is false; Claude doesn't need restarts, but benefits from them. The key insight is that extensive debugging history becomes noise during implementation, degrading focus and increasing errors.",
      source: "Lesson 4: Context Compression and Session Restart"
    },
    {
      question: "You're refactoring authentication (security-critical) and adding a fun Easter egg feature (creative). Why does context isolation matter here beyond just organizing work?",
      options: [
        "Security tasks require longer context windows always",
        "Easter eggs need specialized AI models entirely",
        "Different mindsets and risk tolerances prevent cross-contamination",
        "Isolation speeds up both tasks significantly"
      ],
      correctOption: 2,
      explanation: "Authentication refactoring requires security-focused, conservative thinking (validate everything, minimize attack surface) while Easter eggs encourage creativity and experimentation—mixing these mindsets causes problems. The lesson emphasizes that isolation prevents inappropriate pattern transfer between contexts with different priorities. Option A (longer windows) is irrelevant to the security vs creativity distinction. Option C (specialized models) is unnecessary; the same AI handles both, but separately. Option D (speeds up) is a secondary benefit, not the core reason. Context contamination here could manifest as overly cautious Easter eggs or, worse, creative experimentation in security-critical authentication code—a serious risk.",
      source: "Lesson 5: Context Isolation and Parallel Tasks"
    },
    {
      question: "Your project uses an unconventional testing strategy where integration tests run before unit tests. Where should this be documented?",
      options: [
        "README.md since it affects developer setup",
        "architecture.md since it describes technical system organization",
        "decisions.md since it records testing approach choices",
        "CLAUDE.md since it affects AI workflow expectations"
      ],
      correctOption: 3,
      explanation: "CLAUDE.md documents workflows and collaboration patterns—an unconventional testing sequence directly affects how AI should structure work, making it a workflow convention. The lesson specifies CLAUDE.md establishes AI collaboration expectations. Option B (architecture.md) describes system design, not process workflows. Option C (decisions.md) records why the choice was made but doesn't establish the current expectation. Option D (README.md) targets human developers, not AI collaboration patterns. Documenting this in CLAUDE.md ensures AI knows to suggest/implement integration tests before unit tests, aligning with team workflows and preventing confusion when AI follows typical unit-first patterns.",
      source: "Lesson 6: Memory Files and Persistent Intelligence"
    },
    {
      question: "You need to generate code that integrates with multiple complex third-party APIs. The selection framework suggests which tool and why?",
      options: [
        "Gemini CLI because multiple APIs need broad context",
        "Claude Code because deep reasoning about integration required",
        "Either tool handles multiple API integrations equally",
        "Gemini CLI because APIs require multimodal understanding"
      ],
      correctOption: 1,
      explanation: "Multiple complex API integrations require deep reasoning about error handling, rate limiting, authentication, data transformation, and failure scenarios—Claude Code's strength. The lesson emphasizes Claude Code excels at complex logical reasoning across interconnected systems. Option A (broad context) confuses context window size with reasoning depth; Gemini's longer window doesn't automatically mean better integration logic. Option C (either works) ignores the framework's guidance to match tool strengths to tasks. Option D (multimodal) is irrelevant unless APIs involve image/video processing. The selection framework guides you to Claude Code when the challenge is reasoning complexity, not context volume.",
      source: "Lesson 7: Tool Selection Framework"
    },
    {
      question: "Claude generates a database query that works in your development environment but fails in production. What context was likely missing during generation?",
      options: [
        "Production environment constraints and configuration differences from dev",
        "Complete production database schema and all tables",
        "AI lacks sufficient SQL knowledge base",
        "Development environment is incorrectly configured entirely"
      ],
      correctOption: 0,
      explanation: "Dev-vs-production failures typically indicate missing environment-specific context: production constraints, configuration differences, data scale, permissions, connection pooling, timeout settings. The lesson emphasizes that integration failures often reveal missing contextual information. Option A (complete schema) is too narrow; schema might be identical but constraints differ. Option C (AI knowledge) is unlikely; SQL generation works, but environment-specific factors weren't considered. Option D (dev misconfigured) doesn't explain why AI-generated code didn't account for production differences. This diagnostic helps identify when to explicitly load environment-specific constraints before code generation to prevent environment-specific failures.",
      source: "Lesson 8: Hands-on Debugging and Optimization"
    },
    {
      question: "Your spec says 'implement comprehensive validation for user input.' Why is this insufficient for multi-session orchestration?",
      options: [
        "Comprehensive validation is impossible to specify clearly",
        "AI cannot implement input validation correctly",
        "Validation requirements need specific rules and constraints",
        "This specification is sufficiently detailed already"
      ],
      correctOption: 2,
      explanation: "'Comprehensive' is subjective—one session might implement type checking, another regex validation, another database lookups, another sanitization. The lesson emphasizes specs must be explicit about expectations. Option B (AI can't validate) is false; AI validates well with clear requirements. Option C (impossible to specify) is defeatist; validation is very specifiable. Option D (sufficiently detailed) misses that ambiguity causes inconsistency. Better specs define validation explicitly: required fields, format rules, range constraints, sanitization approaches, error messages. This ensures different sessions (or different days) implement consistent validation, not each AI's interpretation of 'comprehensive.'",
      source: "Lesson 9: Capstone Spec-Driven Orchestration"
    },
    {
      question: "You manually count approximately 180K tokens in your session, but Claude's responses are still sharp and accurate. What does this indicate about degradation?",
      options: [
        "Degradation is purely a function of capacity",
        "Degradation has begun but is invisible currently",
        "Your manual count is certainly inaccurate significantly",
        "Degradation depends on token count and content patterns"
      ],
      correctOption: 3,
      explanation: "Degradation isn't purely about token count—content structure, redundancy, and attention patterns matter. Well-organized, non-repetitive content degrades slower than chaotic exploration even at similar token counts. The lesson emphasizes that degradation varies based on content characteristics, not just volume. Option A (purely capacity) oversimplifies; two sessions at 180K can have very different quality. Option C (inaccurate count) is possible but doesn't explain the absence of expected degradation symptoms. Option D (invisible degradation) contradicts the scenario stating responses are sharp. This insight helps developers understand that clean, structured sessions tolerate higher token counts than exploratory, repetitive ones.",
      source: "Lesson 1: Context Windows and Token Counting"
    },
    {
      question: "Claude begins asking questions it already answered 80 messages ago. What does this symptom reveal about context window health?",
      options: [
        "The AI's base knowledge needs updating",
        "Effective attention to earlier conversation has degraded",
        "Token limits have been completely exceeded now",
        "User needs to provide clearer answers"
      ],
      correctOption: 1,
      explanation: "Asking previously answered questions indicates degraded attention to earlier conversation—Claude struggles to attend to information from 80 messages ago. The lesson identifies repeated questions as a classic degradation symptom. Option A (base knowledge) confuses in-session context with model knowledge. Option C (limits exceeded) might be true but isn't diagnosed by repeated questions alone. Option D (clearer answers) misses that the issue is attention to existing answers, not answer quality. This pattern helps developers recognize when effective attention has degraded enough to warrant intervention—if Claude can't remember answers from earlier in the same session, compression or restart is needed.",
      source: "Lesson 2: Degradation Symptoms and Manual Tracking"
    },
    {
      question: "For implementing a new feature, you need architectural patterns (used throughout) and a specific legacy module interface (used once). How should you sequence loading?",
      options: [
        "Load both together in Foundation phase immediately",
        "Legacy interface in Current and patterns On-Demand",
        "Architectural patterns in Foundation and legacy interface On-Demand",
        "Load everything at start for completeness"
      ],
      correctOption: 2,
      explanation: "Architectural patterns used throughout belong in Foundation (persistent baseline), while one-time legacy interfaces belong in On-Demand (loaded when specifically needed). The lesson emphasizes the Foundation→Current→On-Demand progression based on usage frequency. Option A (both Foundation) wastes context on rarely-used legacy interfaces. Option C (patterns On-Demand) is backwards; patterns inform all development and should load early. Option D (everything at start) defeats progressive loading's purpose—efficient context management. This sequencing ensures frequently-referenced patterns are always available while deferring specialized, rarely-used interfaces until the moment they're needed, optimizing context window usage.",
      source: "Lesson 3: Progressive Loading Strategy"
    },
    {
      question: "After extensive refactoring exploration (170K tokens), you decide to take a different architectural approach entirely. What's the best compression strategy?",
      options: [
        "Checkpoint the new approach decision and reasoning only",
        "Start completely fresh without any checkpoint notes",
        "Checkpoint all explored approaches for future reference",
        "Continue in current session for context continuity"
      ],
      correctOption: 0,
      explanation: "When pivoting architectures, checkpoint the decision and reasoning but not the extensive exploration of the abandoned approach—preserve why you're changing direction without carrying forward irrelevant details. The lesson emphasizes strategic checkpoint content selection. Option A (all approaches) preserves too much noise from the discarded direction. Option B (completely fresh) loses the valuable insight about why the new approach is better. Option D (continue) carries 170K tokens of now-irrelevant exploration. The key is capturing the pivot rationale: why the original approach was insufficient, what the new approach solves, key constraints informing the decision. This enables the fresh session to benefit from the architectural learning.",
      source: "Lesson 4: Context Compression and Session Restart"
    },
    {
      question: "You're building a public API (strict versioning, backward compatibility) and a prototype feature (rapid iteration, breaking changes OK). Why are separate sessions valuable?",
      options: [
        "Prototypes require faster AI models than production",
        "Sessions must be isolated for compliance",
        "Public APIs need longer context windows always",
        "Different change management philosophies prevent inappropriate pattern transfer"
      ],
      correctOption: 3,
      explanation: "Public APIs require conservative change management (versioning, deprecation, backward compatibility) while prototypes benefit from aggressive iteration—mixing these philosophies causes problems. The lesson emphasizes isolation prevents inappropriate pattern application. Option A (faster models) is irrelevant; the same model handles both appropriately. Option C (longer windows) doesn't address the change management distinction. Option D (compliance) might be true but isn't the core pedagogical point about pattern contamination. Interleaving these tasks risks the AI suggesting breaking changes in the public API or overly cautious versioning in the prototype—opposite of what each needs.",
      source: "Lesson 5: Context Isolation and Parallel Tasks"
    },
    {
      question: "After several months, your team realizes the decision to use microservices was problematic and you're moving to a monolith. Where should this be documented?",
      options: [
        "Update CLAUDE.md with new development workflow patterns",
        "Document in all three files simultaneously",
        "Add entry to decisions.md explaining reversal reasoning",
        "Update architecture.md to reflect new monolithic structure"
      ],
      correctOption: 1,
      explanation: "This major architectural reversal affects all three memory files: decisions.md records the reversal reasoning, architecture.md describes the new monolithic structure, and CLAUDE.md updates development workflows. The lesson emphasizes memory files work together as a system. While option C (decisions.md) is the primary location for the reversal rationale, options A and B are also necessary. Option A updates workflows affected by the consolidation. Option B reflects the current architecture. Recording only in decisions.md would leave architecture.md outdated and CLAUDE.md misaligned with new practices. Comprehensive updates ensure all memory files stay synchronized, preventing AI confusion from contradictory information across files.",
      source: "Lesson 6: Memory Files and Persistent Intelligence"
    },
    {
      question: "You're analyzing large log files to identify performance bottlenecks. The selection framework suggests which tool and why?",
      options: [
        "Claude Code for deep reasoning about patterns",
        "Either tool since log analysis is straightforward",
        "Gemini CLI for its extended two million token context",
        "Claude Code for its superior coding capabilities"
      ],
      correctOption: 2,
      explanation: "Large log files benefit from Gemini CLI's extended two-million-token context window—it can ingest and analyze massive logs that exceed Claude Code's capacity. The lesson emphasizes matching tool capacity to task scale. Option A (deep reasoning) is valuable but irrelevant if logs don't fit in context. Option C (either works) ignores that log volume might exceed Claude Code's window. Option D (coding capabilities) misses that analysis, not code generation, is the primary task. The selection framework guides you to consider data volume: when context capacity is the constraint, Gemini CLI's larger window is the decisive factor.",
      source: "Lesson 7: Tool Selection Framework"
    },
    {
      question: "Your optimization makes code faster but Claude begins suggesting this pattern inappropriately in contexts where clarity matters more than speed. What went wrong?",
      options: [
        "Context isolation failed and pattern overgeneralized across concerns",
        "The optimization itself was implemented incorrectly fundamentally",
        "Claude's base model preferences performance over clarity",
        "Optimization patterns always override clarity concerns automatically"
      ],
      correctOption: 0,
      explanation: "When Claude overgeneralizes a pattern to inappropriate contexts, it indicates insufficient context isolation—the optimization discussion contaminated subsequent development where different trade-offs apply. The lesson emphasizes context management prevents inappropriate pattern transfer. Option A (incorrect implementation) doesn't explain why the pattern is being misapplied elsewhere. Option C (model preferences) ignores that the issue is context management, not base model behavior. Option D (always override) is false; context should guide trade-off decisions. This diagnostic reveals when to use session isolation: optimization discussions can create momentum toward performance at the expense of other concerns like readability, maintainability, or simplicity.",
      source: "Lesson 8: Hands-on Debugging and Optimization"
    },
    {
      question: "Your spec includes 'follow best practices for the framework.' What problem does this create for spec-driven orchestration across multiple sessions?",
      options: [
        "Best practices change too rapidly to specify",
        "Vague phrase allows inconsistent interpretation across sessions",
        "AI doesn't have knowledge of framework practices",
        "This instruction is appropriately specific already"
      ],
      correctOption: 1,
      explanation: "'Best practices' is vague—different sessions might interpret it as folder structure, naming conventions, component patterns, state management, testing approaches, or performance optimization. The lesson emphasizes specs need concrete guidance, not subjective terms. Option A (practices change) is true but misses the immediate problem: current vagueness. Option C (AI lacks knowledge) is false; AI knows frameworks but needs direction on which practices to prioritize. Option D (appropriately specific) ignores the consistency problem. Better specs enumerate specific practices: 'use composition over inheritance,' 'implement error boundaries for all async components,' 'colocate tests with components.' This ensures different sessions apply the same patterns.",
      source: "Lesson 9: Capstone Spec-Driven Orchestration"
    },
    {
      question: "Estimating tokens for a JSON file, you notice it's highly structured with lots of repeated keys. How does structure affect token count compared to prose?",
      options: [
        "Structured formats consume fewer tokens than prose",
        "JSON structure overhead increases token count significantly",
        "Repeated keys reduce tokens through compression automatically",
        "Structure has minimal impact compared to content"
      ],
      correctOption: 3,
      explanation: "Structure has minimal impact on token count compared to total character volume—tokens represent textual content regardless of format. The lesson emphasizes character count drives token estimation. Option A (fewer tokens) is wrong; structure doesn't compress content. Option B (overhead increases) overestimates structure impact; while braces and quotes add characters, the effect is proportional to content. Option D (automatic compression) misunderstands token counting; repeated keys aren't compressed in the token representation. Whether text is JSON, code, or prose, approximately 4 characters per token applies. Understanding this helps developers accurately estimate tokens for structured data without overcompensating for format.",
      source: "Lesson 1: Context Windows and Token Counting"
    },
    {
      question: "After 60 messages, Claude begins suggesting solutions you've already rejected with explanations. What degradation pattern is this and what does it indicate?",
      options: [
        "Attention to rejection explanations has degraded significantly",
        "AI cannot remember previous rejections ever",
        "Your explanations weren't clear enough initially",
        "This is normal behavior, not degradation"
      ],
      correctOption: 0,
      explanation: "Suggesting previously-rejected solutions indicates degraded attention to earlier explanations—Claude struggles to maintain awareness of what's been tried and dismissed. The lesson identifies this as a classic symptom of effective attention degradation. Option A (cannot remember) is too absolute; early in sessions Claude tracks this well. Option C (unclear explanations) misses that the issue is attention degradation, not initial clarity. Option D (normal behavior) fails to recognize the degradation symptom. This pattern is particularly concerning because it wastes effort on known-bad approaches, signaling that session quality has degraded enough to warrant compression or restart to restore effective attention.",
      source: "Lesson 2: Degradation Symptoms and Manual Tracking"
    },
    {
      question: "You're implementing error handling that touches many files. Which content belongs in the Current phase according to progressive loading?",
      options: [
        "All files that might eventually need changes",
        "Just the error handling specification documentation only",
        "Error handling requirements and files currently being modified",
        "Complete codebase for full context availability"
      ],
      correctOption: 2,
      explanation: "The Current phase loads active task requirements (error handling specs) plus the specific files currently being modified—not all files that might need changes eventually. The lesson emphasizes Current phase provides immediate task context. Option A (all potential files) creates unnecessary context bloat; load files as you reach them (On-Demand). Option C (just specification) omits the code being modified, preventing effective implementation. Option D (complete codebase) defeats progressive loading's purpose. The Current phase strategy loads just enough: the task requirements and actively-modified files. As you move to new files, you load them On-Demand, keeping context focused.",
      source: "Lesson 3: Progressive Loading Strategy"
    },
    {
      question: "During a 150-message architectural debate, your team reaches consensus on a hybrid approach. The session has 165K tokens. What compression approach makes sense?",
      options: [
        "Preserve entire debate for historical record value",
        "Continue session to maintain full discussion context",
        "Start fresh without any checkpoint documentation",
        "Checkpoint final consensus, key trade-offs, and rationale"
      ],
      correctOption: 3,
      explanation: "Architectural debates compress well into decision checkpoints: final consensus, considered alternatives, key trade-offs, and rationale—this preserves the intellectual conclusion without the back-and-forth. The lesson emphasizes distilling extensive discussion into actionable decisions. Option A (entire debate) preserves unnecessary conversational noise that doesn't improve implementation. Option C (no checkpoint) loses valuable architectural reasoning. Option D (continue) carries 165K tokens of debate that's no longer needed for implementation. The checkpoint should capture what was decided and why—enough context for future AI sessions to understand the architecture without relitigating the debate. This enables clean implementation in a fresh session.",
      source: "Lesson 4: Context Compression and Session Restart"
    },
    {
      question: "You're writing unit tests (isolated, deterministic) and integration tests (connected systems, external dependencies). Beyond organization, why does the lesson recommend separate sessions?",
      options: [
        "Unit tests and integration tests need different AI",
        "Different testing philosophies prevent pattern contamination across types",
        "Integration tests require significantly longer context windows always",
        "Test types must be isolated for correctness"
      ],
      correctOption: 1,
      explanation: "Unit tests emphasize isolation, mocking, and determinism while integration tests emphasize real connections, environment setup, and managing external dependencies—mixing these philosophies causes inappropriate pattern transfer. The lesson emphasizes that different testing mindsets benefit from isolation. Option A (different AI) is unnecessary; the same AI handles both, but in separate contexts. Option C (longer windows) isn't inherently true; complexity, not test type, drives context needs. Option D (must isolate for correctness) is too strong; technically they could share sessions, but quality suffers. Context contamination here might cause over-mocking in integration tests or inappropriate external dependencies in unit tests.",
      source: "Lesson 5: Context Isolation and Parallel Tasks"
    },
    {
      question: "Your team establishes a convention: always implement feature flags for new functionality. Where should this be documented?",
      options: [
        "decisions.md since it was a team decision",
        "architecture.md since feature flags are technical",
        "CLAUDE.md since it establishes development workflow",
        "README.md since it affects all development"
      ],
      correctOption: 2,
      explanation: "CLAUDE.md documents development workflows and conventions—the feature flag requirement directly affects how AI should structure new features, making it a workflow convention. The lesson specifies CLAUDE.md establishes AI collaboration patterns. Option A (decisions.md) would record why the convention was adopted but not the current expectation. Option C (architecture.md) describes system design, not development process. Option D (README.md) targets humans, not AI workflow integration. Documenting in CLAUDE.md ensures AI automatically includes feature flags when generating new features, aligning with team workflow without manual reminders each time.",
      source: "Lesson 6: Memory Files and Persistent Intelligence"
    },
    {
      question: "You need to refactor complex business logic with intricate conditional flows and edge cases. The selection framework suggests which tool and why?",
      options: [
        "Claude Code for deep logical reasoning capability",
        "Gemini CLI for extended context to hold logic",
        "Either tool handles business logic refactoring equally well",
        "Gemini CLI for its multimodal features"
      ],
      correctOption: 0,
      explanation: "Complex business logic with intricate conditionals and edge cases requires deep reasoning about logical relationships, implications, and edge case interactions—Claude Code's strength. The lesson emphasizes Claude Code excels at reasoning through complex logical systems. Option A (extended context) misses that reasoning depth, not context volume, is the challenge. Option C (either works) ignores the framework's guidance to match tool strengths to task demands. Option D (multimodal) is irrelevant; business logic doesn't involve visual content. The selection framework guides you toward Claude Code when the challenge is understanding and reasoning through complex logic, even if the code volume is modest.",
      source: "Lesson 7: Tool Selection Framework"
    },
    {
      question: "Claude generates a solution that works perfectly in your test cases but fails with real data. What context issue does this suggest?",
      options: [
        "Test cases are fundamentally flawed and incorrect",
        "Code generation was rushed and incomplete",
        "The AI cannot handle real-world scenarios",
        "Real data characteristics and constraints weren't in context"
      ],
      correctOption: 3,
      explanation: "Test-case success but real-data failure indicates missing context about real data characteristics: scale, edge cases, null handling, data quality issues, format variations. The lesson emphasizes integration failures reveal missing contextual information. Option A (flawed tests) might be true but doesn't explain why AI didn't account for real data. Option C (cannot handle) underestimates capability; the issue is missing context, not ability. Option D (rushed) doesn't diagnose the specific test-vs-real failure pattern. This diagnostic helps identify when to load real data samples, constraints, or characteristics before generation—giving Claude concrete examples prevents assumptions based on idealized test scenarios.",
      source: "Lesson 8: Hands-on Debugging and Optimization"
    },
    {
      question: "Your spec states 'implement proper database access patterns.' Why does this create inconsistency across multiple AI sessions?",
      options: [
        "Database access cannot be specified adequately",
        "Proper patterns need explicit definition and constraints",
        "AI lacks sufficient database knowledge entirely",
        "This specification is already sufficiently clear"
      ],
      correctOption: 1,
      explanation: "'Proper' is subjective—sessions might implement connection pooling, ORM usage, raw queries, transactions, caching, or read replicas differently. The lesson emphasizes explicit specification prevents interpretation variance. Option B (cannot specify) is defeatist; database patterns are very specifiable. Option C (lacks knowledge) is false; AI knows databases but needs guidance on project-specific patterns. Option D (sufficiently clear) misses that ambiguity causes inconsistency. Better specs enumerate specific patterns: 'use connection pool with max 20 connections,' 'wrap multi-step operations in transactions,' 'query read replicas for reports.' This ensures consistent implementation across sessions.",
      source: "Lesson 9: Capstone Spec-Driven Orchestration"
    },
    {
      question: "You notice token count climbing rapidly during an exploratory debugging session with many hypotheses tested. What does this pattern suggest about future context management?",
      options: [
        "Exploration requires eventual compression or checkpoint for efficiency",
        "Slow down exploration to control token growth",
        "Switch to a tool with larger window",
        "Token growth during exploration is never problematic"
      ],
      correctOption: 0,
      explanation: "Exploratory sessions naturally accumulate tokens from testing hypotheses, documenting failures, and iterating—this valuable exploration eventually requires compression to distill learning into actionable insights. The lesson emphasizes recognizing when context patterns necessitate future intervention. Option A (slow down) is counterproductive; exploration is valuable, just requires eventual checkpoint. Option C (larger window) delays but doesn't solve the issue; exploration still benefits from compression. Option D (never problematic) ignores that accumulated exploration becomes noise during implementation. Recognizing this pattern helps developers plan: explore freely now, but prepare to checkpoint findings before implementation to ensure clean execution context.",
      source: "Lesson 1: Context Windows and Token Counting"
    },
    {
      question: "Claude begins providing correct information but with decreasing confidence, hedging more than earlier in the session. What might this symptom indicate?",
      options: [
        "The AI's base knowledge is uncertain",
        "Earlier confidence was unjustified and problematic",
        "Effective attention to established context has degraded",
        "Increased hedging indicates improved accuracy actually"
      ],
      correctOption: 2,
      explanation: "Increasing hedging and decreasing confidence despite correct information suggests degraded attention to context that established certainty—Claude struggles to maintain confidence based on earlier conversation. The lesson identifies changing communication patterns as degradation symptoms. Option A (base knowledge) confuses in-session context with model knowledge. Option C (earlier confidence unjustified) doesn't explain the shift; if confidence was warranted then, why not now? Option D (improved accuracy) misinterprets hedging as accuracy improvement rather than context degradation. This subtle symptom helps identify degradation before obvious errors emerge—when Claude becomes less confident about established facts, context quality has declined.",
      source: "Lesson 2: Degradation Symptoms and Manual Tracking"
    },
    {
      question: "You're implementing a feature that references a design pattern documented in an old architectural decision. According to progressive loading, when should you load that decision document?",
      options: [
        "Foundation phase since it's architectural context",
        "On-Demand when specifically referencing the pattern",
        "Current phase since it informs active implementation",
        "Never needed if pattern is well-known"
      ],
      correctOption: 1,
      explanation: "Old architectural decision documents belong in On-Demand loading—loaded when you specifically reference that pattern, not throughout the session. The lesson emphasizes On-Demand is for specialized content needed at specific moments. Option A (Foundation) would load it unnecessarily if most features don't reference this pattern. Option B (Current) assumes it's always relevant to current work, which wastes context if not immediately applicable. Option D (never needed) ignores that project-specific decisions and rationale might differ from general knowledge. The On-Demand strategy loads this decision document at the precise moment you're applying the pattern, providing context exactly when needed without bloating the context window earlier.",
      source: "Lesson 3: Progressive Loading Strategy"
    },
    {
      question: "After resolving a critical production bug in a 190K-token emergency session, you need to implement the fix. What's the best approach?",
      options: [
        "Continue immediately while context is fresh",
        "Copy only the solution into new session",
        "Wait until next day for fresh perspective",
        "Create incident checkpoint, then implement in clean session"
      ],
      correctOption: 3,
      explanation: "Creating an incident checkpoint (root cause, solution approach, constraints) then implementing in a clean session optimizes both learning preservation and implementation quality. The lesson emphasizes checkpoint bridges discovery and execution. Option A (continue immediately) carries 190K tokens of crisis debugging that degrades implementation focus. Option C (wait until tomorrow) introduces unnecessary delay; the checkpoint preserves knowledge immediately. Option D (copy solution only) loses critical context about why this solution works and what was tried. The incident checkpoint distills the emergency session into actionable implementation context, enabling efficient, high-quality execution without the noise of crisis debugging.",
      source: "Lesson 4: Context Compression and Session Restart"
    },
    {
      question: "You're implementing a public-facing feature with strict security requirements and an internal admin dashboard with relaxed constraints. Why isolate these beyond just code organization?",
      options: [
        "Different security postures prevent inappropriate pattern transfer completely",
        "Security features require specialized AI models only",
        "Admin dashboards need longer context windows always",
        "Isolation speeds up development significantly"
      ],
      correctOption: 0,
      explanation: "Public features require defense-in-depth, input validation, rate limiting, and minimal exposed functionality while admin dashboards trust authenticated users more—mixing these security postures causes problems. The lesson emphasizes isolation prevents inappropriate pattern application. Option A (specialized models) is unnecessary; the same AI handles both in separate contexts. Option C (longer windows) is irrelevant to the security distinction. Option D (speeds up) is secondary; the primary concern is preventing security pattern confusion. Context contamination here could manifest as paranoid, user-hostile admin tools or, worse, relaxed security in the public-facing feature—a critical vulnerability risk.",
      source: "Lesson 5: Context Isolation and Parallel Tasks"
    },
    {
      question: "Your architecture.md describes a layered architecture, but recent rapid prototyping has bypassed layers. What memory file update is most important?",
      options: [
        "Update CLAUDE.md to reflect prototyping workflow",
        "Update decisions.md to record architecture deviation",
        "Update architecture.md to match actual implementation first",
        "All three files need synchronized updates"
      ],
      correctOption: 2,
      explanation: "When architecture.md diverges from reality, updating it to reflect actual implementation is most urgent—outdated architecture documentation causes AI to generate incompatible code. The lesson emphasizes memory files must reflect current reality. While option D (all three) is eventually correct, option B (architecture.md first) is most critical. Option A (CLAUDE.md) and C (decisions.md) are important but secondary—without accurate architecture, AI suggestions will conflict with codebase structure. Once architecture.md is updated, document the deviation decision in decisions.md and update CLAUDE.md workflows if needed. Prioritizing architecture accuracy prevents immediate AI confusion and integration failures.",
      source: "Lesson 6: Memory Files and Persistent Intelligence"
    },
    {
      question: "You're creating a comprehensive system design document with architecture diagrams, code examples, and API specifications. Which tool best matches this task?",
      options: [
        "Claude Code for superior technical writing quality",
        "Gemini CLI for multimodal content and large context",
        "Either tool creates documentation equally well",
        "Claude Code for better reasoning about systems"
      ],
      correctOption: 1,
      explanation: "Comprehensive documentation with diagrams benefits from Gemini CLI's multimodal capabilities (can reference/process diagrams) and extended context (can hold extensive specifications). The lesson emphasizes matching multiple tool strengths to task demands. Option A (technical writing) is valuable but doesn't address diagrams or volume. Option C (either works) ignores that visual content and document length favor Gemini. Option D (reasoning) is valuable for design decisions but doesn't address the full documentation scope. The selection framework guides you to consider multiple task dimensions: Gemini handles visual content and large documents effectively, making it ideal for comprehensive documentation projects.",
      source: "Lesson 7: Tool Selection Framework"
    }
  ]}
  questionsPerBatch={18}
/>

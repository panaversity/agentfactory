---
title: "Chapter 1 Quiz"
sidebar_position: 10
---

# Chapter 1 Quiz: The Agent Factory Paradigm

Test your understanding of the foundational concepts introduced in this chapter. Each question checks a specific learning objective from the lessons.

<Quiz
  title="Chapter 1: The Agent Factory Paradigm"
  questions={[
    {
      question: "Which of the following is concrete evidence that AI coding capability reached production quality in 2024-2025? (Lesson 1)",
      options: [
        "ChatGPT became more popular than other AI tools",
        "More blog posts were written about AI development",
        "OpenAI achieved a perfect score solving all 12 problems at the ICPC World Finals",
        "AI companies received more venture capital funding"
      ],
      correctOption: 2,
      explanation: "The ICPC World Finals breakthrough in 2025 demonstrated that AI can solve complex algorithmic problems at the highest competitive level—concrete evidence of production-quality capability, not just popularity or funding metrics.",
      source: "Lesson 1: The 2025 Inflection Point"
    },
    {
      question: "According to the 2025 Stack Overflow Developer Survey, what percentage of professional developers use or plan to use AI coding tools? (Lesson 2)",
      options: [
        "51%",
        "66%",
        "84%",
        "95%"
      ],
      correctOption: 2,
      explanation: "84% of developers are using or plan to use AI tools, with 51% using them daily. This mainstream adoption indicates AI tools have crossed from experimental to standard practice.",
      source: "Lesson 2: The Scale of the Shift"
    },
    {
      question: "According to the Y Combinator Winter 2025 batch data, what is notable about startup adoption of AI? (Lesson 2)",
      options: [
        "No startups are using AI yet because it's too experimental",
        "25% of startups incorporated AI-generated code as their primary development approach, with some reporting 95% AI-written code",
        "All startups now use AI exclusively and have eliminated human developers",
        "Only enterprise companies use AI; startups still rely on traditional coding"
      ],
      correctOption: 1,
      explanation: "A quarter of YC W25 startups used AI-generated code as their primary approach, with some reporting 95% AI-written code. These weren't non-technical founders—they were expert developers choosing to orchestrate AI instead of typing every line.",
      source: "Lesson 2: The Scale of the Shift"
    },
    {
      question: "What is the fundamental insight about the relationship between General Agents and Custom Agents? (Lesson 3)",
      options: [
        "General Agents and Custom Agents compete directly; you must choose one",
        "Custom Agents are outdated; General Agents have replaced them entirely",
        "General Agents don't compete with Custom Agents—General Agents BUILD Custom Agents",
        "Custom Agents and General Agents serve identical purposes but with different pricing models"
      ],
      correctOption: 2,
      explanation: "This is the core Agent Factory insight: General Agents (like Claude Code) are reasoning systems that help you explore, prototype, and ultimately BUILD purpose-specific Custom Agents for production deployment.",
      source: "Lesson 3: Two Paths to Building AI Products"
    },
    {
      question: "Which of the following is a General Agent tool? (Lesson 3)",
      options: [
        "OpenAI Agents SDK",
        "Anthropic Agents Kit",
        "Claude Code",
        "Google ADK"
      ],
      correctOption: 2,
      explanation: "Claude Code is a General Agent—a multi-purpose reasoning system designed for exploration, prototyping, and directing AI to build solutions. The SDKs (OpenAI, Claude SDK, Google ADK) are for building Custom Agents.",
      source: "Lesson 3: Two Paths to Building AI Products"
    },
    {
      question: "In the Agent Factory paradigm, what is your role when using a General Agent like Claude Code? (Lesson 3)",
      options: [
        "Implementer—you must write the actual code",
        "Director—you specify intent and let the agent handle tactical decisions",
        "Observer—you watch the agent work with minimal input",
        "Administrator—you manage security and deployment configurations"
      ],
      correctOption: 1,
      explanation: "When using General Agents, you're the Director: you specify intent clearly, let the agent handle tactical decisions, evaluate quality, and provide feedback to redirect when needed.",
      source: "Lesson 3: Two Paths to Building AI Products"
    },
    {
      question: "Which scenario would be MOST appropriate for using a General Agent rather than building a Custom Agent? (Lesson 3)",
      options: [
        "Building a system that will run 1000+ times daily in production",
        "Exploring an unknown problem to understand what a good solution looks like",
        "Deploying a customer-facing feature with strict reliability requirements",
        "Optimizing costs for a high-volume application that repeats the same workflow"
      ],
      correctOption: 1,
      explanation: "General Agents excel at exploration and prototyping when you don't yet understand the problem fully. Custom Agents are better for production, high-volume, reliability-critical scenarios.",
      source: "Lesson 3: Two Paths to Building AI Products"
    },
    {
      question: "What is the OODA loop? (Lesson 3)",
      options: [
        "A method for debugging code faster than traditional approaches",
        "A reasoning framework with Observe, Orient, Decide, Act—used by both General and Custom Agents",
        "A programming design pattern for asynchronous operations",
        "An acronym for four programming languages"
      ],
      correctOption: 1,
      explanation: "OODA (Observe, Orient, Decide, Act) is a reasoning framework from military strategy that describes how AI agents process information and take action. Good OODA loops are fast—Claude Code completes this cycle in seconds.",
      source: "Lesson 3: Two Paths to Building AI Products"
    },
    {
      question: "What is the most fundamental change in the developer role in the AI era? (Lesson 4)",
      options: [
        "Learning new programming syntax",
        "Mastering additional frameworks and libraries",
        "Shifting from implementation (typing code) to orchestration (directing AI systems)",
        "Understanding cloud computing and DevOps better"
      ],
      correctOption: 2,
      explanation: "The core shift is from typist to orchestrator. Your value is no longer in how fast you can type, but in the quality of your ideas and directions. The 10% humans contribute—problem understanding, decisions, quality judgments—becomes infinitely more valuable.",
      source: "Lesson 4: From Coder to Orchestrator"
    },
    {
      question: "Which tasks do orchestrators focus on rather than typists? (Lesson 4)",
      options: [
        "Writing boilerplate code and configuration files",
        "Implementing database queries and API endpoints",
        "Specification writing, requirement gathering, and validation of AI-generated work",
        "Remembering programming language syntax without references"
      ],
      correctOption: 2,
      explanation: "Orchestrators focus on the judgment work: writing clear specifications, gathering requirements, designing architecture, and validating AI output. AI handles the mechanical implementation.",
      source: "Lesson 4: From Coder to Orchestrator"
    },
    {
      question: "How does the role evolution from coder to orchestrator change the SDLC phases? (Lesson 5)",
      options: [
        "SDLC phases are eliminated entirely in the AI era",
        "The phases remain identical; only the tools change",
        "The phases shift emphasis from implementation to specification and validation; humans focus on judgment while AI handles execution",
        "SDLC becomes irrelevant because AI manages all phases autonomously"
      ],
      correctOption: 2,
      explanation: "Every SDLC phase is transformed but not eliminated. The emphasis shifts: humans handle specification, architecture, and validation (judgment work) while AI handles implementation (mechanical work).",
      source: "Lesson 5: Development Lifecycle Transformation"
    },
    {
      question: "Which generation of AI development tools represents the current state in 2025? (Lesson 6)",
      options: [
        "Generation 1: Simple code completion suggestions",
        "Generation 2: Chatbot interfaces disconnected from code editors",
        "Generation 3: AI-first IDEs with integrated reasoning",
        "Generation 4: Autonomous development agents with tools, structured reasoning, and multi-turn capability"
      ],
      correctOption: 3,
      explanation: "We're transitioning from Gen 3 to Gen 4. Generation 4 agents can autonomously execute multi-step workflows—reading code, running tests, making commits—without requiring step-by-step human approval.",
      source: "Lesson 6: The Autonomous Agent Era"
    },
    {
      question: "What are the Five Powers that enable autonomous agents? (Lesson 7)",
      options: [
        "Five programming languages you must master",
        "Five cloud providers for deploying AI systems",
        "See, Hear, Reason, Act, Remember—five capabilities that combine for autonomous orchestration",
        "Five types of machine learning models used in production"
      ],
      correctOption: 2,
      explanation: "The Five Powers are: See (visual understanding), Hear (audio processing), Reason (complex decision-making), Act (execute and orchestrate), Remember (maintain context and learn). Combined, they enable autonomous orchestration.",
      source: "Lesson 7: User Intent Replaces User Interface"
    },
    {
      question: "What is the primary advantage of a modular, three-layer AI stack compared to monolithic tool ecosystems? (Lesson 8)",
      options: [
        "It requires less training for developers to use",
        "It guarantees all tools are free and open-source",
        "It prevents vendor lock-in and enables faster evolution by composing independent layers",
        "It eliminates the need for AI models entirely"
      ],
      correctOption: 2,
      explanation: "The modular stack (Frontier Models → AI-First IDEs → Development Agents) prevents vendor lock-in. You can swap models via API, choose best-of-breed at each layer, and evolve your stack independently.",
      source: "Lesson 8: The Modern AI Stack"
    },
    {
      question: "What role does Model Context Protocol (MCP) play in the modern AI development stack? (Lesson 8)",
      options: [
        "It replaces frontier models as the reasoning engine",
        "It provides IDE editing capabilities similar to VS Code",
        "It acts as a universal standard enabling tools and services to interoperate without vendor lock-in",
        "It manages deployment and scaling of AI applications"
      ],
      correctOption: 2,
      explanation: "MCP is like USB for AI—before USB, every device had a custom cable. MCP creates a standard for AI tools to access your codebase, terminal, and other systems, enabling a true ecosystem.",
      source: "Lesson 8: The Modern AI Stack"
    },
    {
      question: "What distinguishes a development agent (Layer 3) from a frontier model (Layer 1)? (Lesson 8)",
      options: [
        "Development agents are cheaper but less capable than frontier models",
        "Development agents have tools to read codebases, test code, and iterate autonomously—frontier models only generate text",
        "Frontier models only work for code; development agents work for all tasks",
        "Development agents use older AI technology than frontier models"
      ],
      correctOption: 1,
      explanation: "Frontier models (GPT-5, Claude Opus) generate text/code. Development agents (Claude Code, Aider) wrap those models with tools—they can read your codebase, run tests, and iterate autonomously on multi-step tasks.",
      source: "Lesson 8: The Modern AI Stack"
    },
    {
      question: "Which best describes Spec-Driven Development as introduced in Lesson 9? (Lesson 9)",
      options: [
        "An approach where specifications are written after code is implemented",
        "A methodology focused entirely on AI generation without human involvement",
        "A workflow where clear specifications are written FIRST to drive AI implementation",
        "A traditional software development methodology that predates AI systems"
      ],
      correctOption: 2,
      explanation: "Spec-Driven Development is specification-first: write clear specs, then tests that encode the spec, then implement to pass tests. This discipline becomes MORE critical with AI—it amplifies good habits and bad habits alike.",
      source: "Lesson 9: Spec-Driven Development Preview"
    },
    {
      question: "Why does 'AI amplifies your habits' matter for development methodology? (Lesson 9)",
      options: [
        "AI makes all development approaches equally effective",
        "Vibe coding with AI leads to faster, more reliable software",
        "Spec-driven development becomes MORE critical because AI amplifies both good discipline and bad habits",
        "AI eliminates the need for any development methodology"
      ],
      correctOption: 2,
      explanation: "AI generates code instantly but won't write specs you didn't ask for or tests you didn't request. Vibe Coding + AI = Amplified Chaos. Spec-Driven + AI = Amplified Excellence. The discipline becomes more critical, not less.",
      source: "Lesson 9: Spec-Driven Development Preview"
    }
  ]}
  questionsPerBatch={18}
/>

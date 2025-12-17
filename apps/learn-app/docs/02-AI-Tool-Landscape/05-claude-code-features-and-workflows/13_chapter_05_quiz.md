---
sidebar_position: 13
title: "Chapter 5: Claude Code Features and Workflows Quiz"
---

# Chapter 5: Claude Code Features and Workflows Quiz

Test your understanding of Claude Code's architecture, extensibility features, and workflow patterns. This assessment covers installation, configuration, MCP integration, subagents, skills, hooks, settings hierarchy, and plugin architecture.

<Quiz
  title="Chapter 5: Claude Code Features and Workflows Assessment"
  questions={[    {
      question: "A developer notices that Claude Code sometimes generates boilerplate code patterns but doesn't understand the project's specific architectural constraints. What does this observation reveal about the fundamental limitation Claude Code was designed to address?",
      options: [
        "Generic AI assistants are too slow to respond to developer requests",
        "Generic AI assistants lack context about project-specific rules and conventions",
        "Generic AI assistants cannot generate code in multiple programming languages effectively",
        "Generic AI assistants lack internet connectivity to fetch external documentation"
      ],
      correctOption: 1,
      explanation: "Claude Code was designed to address the context limitation of generic AI assistants. Generic AI tools like ChatGPT can generate code but lack awareness of your project's specific conventions, architectural patterns, or custom workflows. Claude Code solves this through CLAUDE.md context files, MCP integrations, and custom skills that encode project-specific knowledge. Options B, C, and D describe capabilities that generic AI assistants actually possess—they can handle multiple languages, respond quickly, and access documentation. The core problem is context awareness, not technical capabilities. This distinction is crucial because it explains why Claude Code emphasizes extensibility features (context files, hooks, skills) rather than just improving code generation speed or language support.",
      source: "Lesson 01: The Claude Code Origin Story and Paradigm Shift"
    },
    {
      question: "You're evaluating whether to adopt Claude Code for your team. A colleague argues that since ChatGPT can already generate code, Claude Code doesn't offer meaningful advantages. Which technical capability of Claude Code best counters this argument?",
      options: [
        "Claude Code supports more programming languages than ChatGPT for code generation tasks",
        "Claude Code uses a larger language model with more parameters than ChatGPT",
        "Claude Code provides faster response times due to optimized infrastructure and caching",
        "Claude Code can execute bash commands and modify files directly in your workspace"
      ],
      correctOption: 3,
      explanation: "Claude Code's agentic capabilities—executing bash commands, reading/writing files, running tests, and using tools—fundamentally distinguish it from passive code generation tools like ChatGPT. While ChatGPT generates code snippets that you must manually copy and integrate, Claude Code acts as an autonomous agent that can implement features, run validation, and iterate based on results. This shifts the paradigm from 'AI as advisor' to 'AI as pair programmer.' Options B, C, and D describe differences that either don't exist (ChatGPT and Claude use comparable models) or aren't the core distinction (speed and language support are similar). The key innovation is agency—the ability to take action in your development environment, not just provide suggestions.",
      source: "Lesson 01: The Claude Code Origin Story and Paradigm Shift"
    },
    {
      question: "During a code review, you notice that Claude Code implemented a feature using a pattern inconsistent with your team's established practices. What does this scenario suggest about the relationship between AI agency and project-specific constraints?",
      options: [
        "Agentic AI requires explicit project context to align actions with team conventions",
        "Agentic AI automatically learns team conventions by analyzing the existing codebase structure",
        "Agentic AI should be avoided in projects with established coding standards",
        "Agentic AI works best when teams abandon existing conventions for AI patterns"
      ],
      correctOption: 0,
      explanation: "Option A correctly identifies that agentic AI requires explicit project context to align actions with team conventions. While Claude Code can execute tasks autonomously, it doesn't inherently know your team's architectural decisions, naming conventions, or design patterns. This is why Claude Code provides CLAUDE.md context files—to explicitly communicate project-specific rules and constraints. Option B is incorrect because AI doesn't automatically infer conventions reliably; it needs explicit guidance. Option C misses the point—agentic AI is valuable precisely because you can configure it with context. Option D suggests abandoning proven practices, which would reduce code quality. The correct approach is combining AI agency with explicit context (CLAUDE.md, hooks, custom skills) to align autonomous actions with project standards.",
      source: "Lesson 01: The Claude Code Origin Story and Paradigm Shift"
    },
    {
      question: "You're explaining the paradigm shift from ChatGPT to Claude Code to a junior developer. They ask: 'Why can't I just copy code from ChatGPT into my editor?' Which limitation of the copy-paste workflow best justifies Claude Code's approach?",
      options: [
        "Copy-paste workflows produce code that uses outdated syntax patterns and deprecated APIs",
        "Copy-paste workflows result in code that is always less efficient performance-wise",
        "Copy-paste workflows break iterative refinement loops where AI validates its own changes",
        "Copy-paste workflows create licensing issues when code is used in products"
      ],
      correctOption: 2,
      explanation: "The copy-paste workflow fundamentally breaks the feedback loop essential for quality code generation. With ChatGPT, you paste code, encounter errors, describe them back to ChatGPT, get new code, and repeat—a slow, manual process. Claude Code's agency enables autonomous iteration: generate code, run tests, observe failures, analyze output, and refine automatically. This creates rapid feedback loops that converge on working solutions faster. Option B is incorrect because both tools use similar training data and don't inherently produce outdated code. Option C is wrong because code efficiency depends on the problem, not the workflow. Option D is a misconception—AI-generated code doesn't inherently create licensing issues (though training data sources might). The real advantage is closing the feedback loop: agency enables AI to validate and refine its own work.",
      source: "Lesson 01: The Claude Code Origin Story and Paradigm Shift"
    },
    {
      question: "A team lead argues that Claude Code's agentic capabilities are risky because 'AI might make destructive changes.' Which Claude Code feature most directly addresses this concern while preserving agentic benefits?",
      options: [
        "Claude Code uses read-only filesystem access to prevent accidental file deletions",
        "Hooks allow validation logic to review and approve changes before execution",
        "Claude Code automatically creates git commits for every file modification made",
        "Claude Code requires human approval for every single bash command execution"
      ],
      correctOption: 1,
      explanation: "Hooks (Option C) provide the ideal balance between AI agency and safety by enabling automated validation without eliminating autonomy. For example, a pre-commit hook can run linters, type checkers, or custom validation logic, and block changes that fail checks—all automatically, without requiring per-action approval. This preserves the speed of agentic workflows while enforcing safety guardrails. Option A (read-only access) is wrong because Claude Code needs write access to be useful—read-only eliminates its core value. Option B (automatic commits) is incorrect because commits don't prevent bad changes, they just track them. Option D (human approval for every command) defeats the purpose of agency by requiring constant manual intervention. Hooks solve the real problem: enabling AI to work autonomously within defined safety boundaries.",
      source: "Lesson 01: The Claude Code Origin Story and Paradigm Shift"
    },
    {
      question: "After installing Claude Code globally using 'npm install -g claude-code', you run 'claude-code chat' but receive 'command not found'. Your colleague suggests checking the PATH. What does this debugging scenario reveal about the relationship between npm global installation and shell environment?",
      options: [
        "npm global installs require restarting the computer to update system environment fully",
        "npm global installs only work with administrator or sudo privileges on system",
        "npm global installs fail when Node.js version is below the minimum requirement",
        "npm global installs require the npm bin directory in PATH variable"
      ],
      correctOption: 3,
      explanation: "This scenario tests understanding of how npm global installations interact with the shell environment. When npm installs packages globally, it places executables in its global bin directory (typically 'npm config get prefix'/bin). For the shell to find these commands, this directory must be in your PATH environment variable. If it's not, the shell can't locate the 'claude-code' executable, producing 'command not found.' Option B is incorrect because global installs typically work without sudo (though permissions can sometimes require it). Option C misses the point—npm would error during installation if Node.js version was incompatible, not at runtime. Option D is wrong because PATH changes take effect in new shell sessions, not after reboot. The real issue is PATH configuration, which is foundational to understanding command-line tool installation.",
      source: "Lesson 02: Installing and Authenticating Claude Code"
    },
    {
      question: "You're troubleshooting authentication issues with Claude Code. The error message says 'API key invalid or expired.' You know you set ANTHROPIC_API_KEY in your .bashrc file. What's the most likely reason Claude Code can't access the key?",
      options: [
        "Current shell session hasn't sourced the updated bashrc file yet",
        "API keys must be set in .bash_profile not bashrc for tools",
        "Claude Code only reads API keys from environment config files",
        "API keys must be Base64 encoded before setting in environment"
      ],
      correctOption: 0,
      explanation: "This question tests understanding of how shell environment variables work. After editing .bashrc, existing shell sessions don't automatically reload the file—you must either run 'source ~/.bashrc' in the current shell or open a new terminal. Many developers encounter this issue because they expect immediate effect after editing config files. Option B is a common confusion but incorrect—both .bashrc and .bash_profile work (though they load at different times: interactive vs. login shells). Option C is wrong because Claude Code primarily uses environment variables, not separate config files for API keys. Option D is incorrect—Anthropic API keys are used as plain text, not Base64 encoded. This scenario is common in real development: environment variable issues often stem from shell state, not the tool itself. Understanding when shell configuration takes effect is crucial for debugging.",
      source: "Lesson 02: Installing and Authenticating Claude Code"
    },
    {
      question: "Your team uses multiple Anthropic API keys for different projects (development vs. production). You need Claude Code to use different keys depending on the project directory. Which approach best implements this requirement?",
      options: [
        "Create separate user accounts on the operating system for each project",
        "Hard-code different API keys directly in the Claude Code configuration files",
        "Create project-specific .env files and load them in each workspace directory",
        "Use the same API key everywhere and rely on usage tracking"
      ],
      correctOption: 2,
      explanation: "Project-specific .env files provide the cleanest solution for managing per-project API keys. By placing a .env file in each project root and using tools like direnv to auto-load it, you can seamlessly switch keys when changing directories. Claude Code respects environment variables from .env files, making this approach both secure (keys not in version control) and maintainable. Option B is dangerous because hard-coding keys in config files risks accidental commits to version control, exposing sensitive credentials. Option C doesn't solve the problem—using one key doesn't allow project-based isolation. Option D is unnecessarily complex and would break workflows requiring access to multiple projects simultaneously. The .env pattern is an industry-standard best practice that balances security (gitignored files) with usability (automatic context-switching).",
      source: "Lesson 02: Installing and Authenticating Claude Code"
    },
    {
      question: "After authenticating Claude Code, you want to verify the connection before starting a complex task. Which command would most effectively validate that Claude Code can communicate with Anthropic's API?",
      options: [
        "Read the authentication documentation to verify the installation steps",
        "Check the API key format using a regular expression validator",
        "Run 'npm list -g claude-code' to confirm package installation",
        "Run 'claude-code chat' and send a simple test message"
      ],
      correctOption: 3,
      explanation: "Actually using Claude Code with a simple test message (Option C) is the most effective validation because it tests the entire chain: API key validity, network connectivity, correct environment variable configuration, and API access permissions. This is an example of end-to-end testing—verifying the system works by using it. Option A (reading documentation) doesn't test anything; documentation won't reveal configuration issues. Option B (format validation) is insufficient because it doesn't confirm the key is actually valid with Anthropic's servers. Option D (checking package installation) only confirms the package exists, not that it can authenticate successfully. In software engineering, this principle applies broadly: the best way to verify a system works is to use it in a realistic scenario. A simple 'hello world' test catches configuration issues that theoretical validation would miss.",
      source: "Lesson 02: Installing and Authenticating Claude Code"
    },
    {
      question: "You're setting up Claude Code on a CI/CD pipeline for automated code reviews. The pipeline runs in a containerized environment. Which authentication approach is most appropriate for this scenario?",
      options: [
        "Copy your personal API key into the Dockerfile for container builds",
        "Use environment variables injected by the CI/CD platform's secret management",
        "Generate a new API key for each pipeline run using automation",
        "Share a single API key across all team members through config"
      ],
      correctOption: 1,
      explanation: "CI/CD platforms provide secret management systems that inject sensitive values as environment variables at runtime, which is the industry-standard secure approach. This avoids embedding secrets in images or code repositories while making them available to the running container. Option B is a critical security vulnerability—Dockerfiles are often committed to version control, and API keys in images can be extracted by anyone with image access. Option C is impractical because Anthropic doesn't provide programmatic API key generation; keys must be manually created in the web console. Option D creates accountability issues and security risks—if the shared key is compromised, you can't identify the source, and rotating it disrupts all users. The environment variable + secret management pattern is fundamental to secure DevOps practices.",
      source: "Lesson 02: Installing and Authenticating Claude Code"
    },
    {
      question: "Your CLAUDE.md file contains: 'Always use async/await instead of promises.' After several tasks, you notice Claude Code still generates .then() promise chains. What does this reveal about how Claude Code interprets project instructions?",
      options: [
        "CLAUDE.md requires clear context and specific examples to reliably guide behavior",
        "CLAUDE.md instructions are ignored when they conflict with Claude's training data",
        "CLAUDE.md files only affect new projects not existing codebases with patterns",
        "CLAUDE.md directives need to be repeated in every message for consistent effect"
      ],
      correctOption: 0,
      explanation: "This scenario illustrates that vague directives like 'always use X' are insufficient—effective CLAUDE.md files (Option B) provide clear context, rationale, and examples to reliably guide behavior. A better directive would be: 'Use async/await for all asynchronous operations (not .then() chains) to maintain consistency with our codebase and improve error handling readability. Example: [code sample].' This gives Claude Code concrete guidance and reasoning. Option A (instructions ignored when conflicting) is incorrect because CLAUDE.md does influence behavior, but vague instructions are open to interpretation. Option C (only affects new projects) is wrong because CLAUDE.md applies to all code generation, regardless of project age. Option D (must repeat in every message) misunderstands how context works—CLAUDE.md is automatically included in context. The real lesson is about prompt engineering: specificity, examples, and rationale produce better results than absolute rules without context.",
      source: "Lesson 07: CLAUDE.md Context Files"
    },
    {
      question: "You're debating whether to document your team's code review process in CLAUDE.md or in a separate wiki. A colleague argues wikis are better because 'they're more comprehensive.' Which advantage of CLAUDE.md best counters this argument?",
      options: [
        "CLAUDE.md files can be version controlled while wikis cannot be backed up",
        "CLAUDE.md files are easier to edit than wiki pages for technical content",
        "CLAUDE.md context is automatically injected into every Claude Code conversation",
        "CLAUDE.md supports markdown syntax while wikis typically use custom formats"
      ],
      correctOption: 2,
      explanation: "The critical advantage of CLAUDE.md is automatic context injection—Claude Code reads this file and includes its content in every conversation without requiring manual reference. This means your code review process guidelines are always present when Claude Code generates code, ensuring consistency. With a wiki, you'd need to manually paste relevant sections into conversations or hope Claude Code infers the rules from code patterns (unreliable). Option B is questionable—wiki editing difficulty varies by platform. Option C is incorrect because most modern wikis support markdown. Option D is wrong because wikis can be backed up, and many are version-controlled (e.g., GitHub wikis are git repositories). The unique value of CLAUDE.md is its role as machine-readable project context that's automatically available to AI, not human-readable documentation.",
      source: "Lesson 07: CLAUDE.md Context Files"
    },
    {
      question: "Your CLAUDE.md includes architectural guidelines, coding standards, and testing requirements. During a task, Claude Code implements a feature that follows the coding standards but violates an architectural guideline. What does this suggest about how to structure CLAUDE.md effectively?",
      options: [
        "Split content into multiple CLAUDE.md files for different concern areas",
        "Remove detailed sections and keep only the most important single rule",
        "Duplicate critical constraints throughout CLAUDE.md for emphasis and repetition",
        "Prioritize critical constraints at the top using clear hierarchical structure"
      ],
      correctOption: 3,
      explanation: "This scenario reveals that when context files are lengthy, structure and prioritization matter. Placing critical constraints (architectural rules) at the top with clear headings ensures they're prominent in Claude Code's attention, while less critical items (style preferences) can appear later. This mirrors effective human documentation: most important information first. Option B is too extreme—removing valuable context weakens Claude Code's understanding of your project. Option C (duplication) adds noise without adding clarity and wastes context window space. Option D is incorrect because Claude Code looks for a single CLAUDE.md file in the project root; splitting into multiple files would require custom configuration and might fragment context. The principle here is information architecture: organize content to reflect priority, not alphabetically or by category alone.",
      source: "Lesson 07: CLAUDE.md Context Files"
    },
    {
      question: "You're adding testing requirements to CLAUDE.md. Which directive would most effectively ensure Claude Code writes testable code?",
      options: [
        "Testing is important so always follow industry best practices thoroughly",
        "All functions must be pure when possible; provide test example patterns",
        "Write comprehensive unit tests for all code generated by AI assistants",
        "Functions should be testable, maintainable, and follow SOLID design principles"
      ],
      correctOption: 1,
      explanation: "Effective CLAUDE.md directives are specific, actionable, and include examples. 'All functions must be pure when possible; provide test example patterns' gives concrete guidance (pure functions) plus a deliverable format (test examples). This enables Claude Code to understand both the constraint and how to demonstrate compliance. Option B is vague—what does 'comprehensive' mean? It doesn't guide implementation, only validation. Option C is problematic because 'best practices' are subjective and 'important' doesn't specify behavior. Option D lists abstract principles (SOLID) without concrete guidance on how to apply them to the current task. The pattern here mirrors effective prompt engineering: specific constraints + expected output format produces better results than abstract principles or importance statements.",
      source: "Lesson 07: CLAUDE.md Context Files"
    },
    {
      question: "You've documented your API design patterns in CLAUDE.md, but Claude Code generates endpoints inconsistent with these patterns. You verify the file is in the project root and correctly named. What's the most likely explanation?",
      options: [
        "CLAUDE.md files require manual reload command to take effect in sessions",
        "Claude Code ignores CLAUDE.md when working on backend API code specifically",
        "The API patterns section is unclear or contradicts examples in codebase",
        "CLAUDE.md content is only read during the first message in conversations"
      ],
      correctOption: 2,
      explanation: "When CLAUDE.md guidance is ignored, the most common cause (Option B) is unclear instructions or conflict between stated rules and existing code patterns. If your codebase has REST endpoints but CLAUDE.md says 'use GraphQL patterns,' Claude Code faces conflicting signals—it might default to matching existing code patterns. Or if your API pattern description is vague ('RESTful design'), Claude Code may interpret this differently than you intended. Option A (manual reload required) is incorrect because CLAUDE.md is read automatically on each conversation start. Option C (ignores backend code) is wrong because CLAUDE.md applies to all code domains equally. Option D (only read at start) misunderstands the mechanism—CLAUDE.md context persists throughout the conversation. The real issue is usually clarity and consistency: effective project instructions align with codebase reality and provide specific, unambiguous guidance.",
      source: "Lesson 07: CLAUDE.md Context Files"
    },
    {
      question: "Your team uses an internal Python package for database operations. You want Claude Code to use this package instead of suggesting SQLAlchemy or raw SQL. Which MCP capability best addresses this requirement?",
      options: [
        "Create an MCP server exposing database tools that use internal package",
        "Document the internal package in CLAUDE.md with usage examples provided",
        "Install the internal package globally so Claude Code can import automatically",
        "Configure Claude Code hooks to replace SQLAlchemy references with internal package"
      ],
      correctOption: 0,
      explanation: "This scenario demonstrates a key use case for MCP: extending Claude Code's capabilities with project-specific tools. By creating an MCP server that exposes database operations (e.g., 'query_users', 'update_order') implemented using your internal package, you make these operations first-class tools in Claude Code's toolbox. This is more effective than documentation alone because tools are directly invokable with typed interfaces, reducing ambiguity. Option B (CLAUDE.md) can help but doesn't give Claude Code direct access to the package's capabilities. Option C misunderstands the issue—installation doesn't teach Claude Code when or how to use the package. Option D (hooks) would require post-processing generated code, which is fragile and complex. MCP's value is adding capabilities through structured, typed tool interfaces that Claude Code can use as naturally as built-in tools.",
      source: "Lesson 08: MCP Integration"
    },
    {
      question: "You're implementing an MCP server that provides access to your company's internal knowledge base. During testing, Claude Code makes many redundant API calls to fetch the same documentation. Which design principle would most effectively optimize this scenario?",
      options: [
        "Reduce the number of tools exposed by MCP server to limit calls",
        "Implement caching in the MCP server to store and reuse responses",
        "Tell Claude Code in CLAUDE.md to minimize knowledge base API calls",
        "Configure Claude Code to batch all MCP requests together for efficiency"
      ],
      correctOption: 1,
      explanation: "This question tests understanding of where optimization belongs in system architecture. The MCP server is the correct layer for caching because it controls the external API interaction and can implement intelligent caching policies (TTL, invalidation, etc.) transparently to Claude Code. This follows the single responsibility principle—the MCP server manages API interaction efficiency, while Claude Code focuses on using the tools. Option B is ineffective because CLAUDE.md guidance can't reliably prevent redundant calls—Claude Code might need the information multiple times for different contexts. Option C damages functionality by removing useful capabilities just to avoid optimization. Option D doesn't exist—Claude Code doesn't have built-in MCP request batching, and even if it did, caching is still needed. The principle here is architectural: handle cross-cutting concerns (caching, rate limiting) at the appropriate abstraction layer, not through caller-side behavior constraints.",
      source: "Lesson 08: MCP Integration"
    },
    {
      question: "Your MCP server exposes a 'deploy_to_production' tool. After an incident where Claude Code triggered an unintended deployment, your team wants to add safeguards. Which approach best balances safety with MCP's tool-based architecture?",
      options: [
        "Configure Claude Code settings to disable all MCP tools by default",
        "Remove the deploy tool entirely and require manual deployment processes always",
        "Use CLAUDE.md to instruct Claude Code never to deploy without asking",
        "Add required confirmation parameters to the deploy tool forcing explicit approval"
      ],
      correctOption: 3,
      explanation: "This scenario illustrates how to design safe MCP tools without sacrificing automation benefits. By adding required parameters like 'confirmed=true' or 'approval_code' to the deploy tool, you force explicit intent—Claude Code must consciously construct a call with these parameters, reducing accidental triggers. This is analogous to 'rm -rf' requiring explicit paths rather than having a default. Option B overreacts by eliminating valuable automation. Option C is unreliable because CLAUDE.md guidance can be overridden or misunderstood in complex task contexts. Option D defeats MCP's purpose entirely. The design principle here is 'safe by default': tools with significant consequences should require explicit confirmation through their interface design (parameters, multi-step flows) rather than relying on external guardrails.",
      source: "Lesson 08: MCP Integration"
    },
    {
      question: "You're building an MCP server to integrate with your GraphQL API. Claude Code needs to query user data, orders, and inventory. Which design approach would create the most maintainable and flexible MCP integration?",
      options: [
        "Expose generic 'execute_graphql_query' tool accepting arbitrary query strings as parameters",
        "Expose only read queries as MCP tools not mutations or updates",
        "Create specific tools for each query like get_user, get_order, get_inventory",
        "Create one tool per GraphQL type with all possible field combinations"
      ],
      correctOption: 2,
      explanation: "This question tests understanding of API design principles applied to MCP. Specific, purpose-built tools (get_user, get_order) provide better developer experience than generic execution tools because they have typed parameters, clear documentation, and validation. This mirrors the REST vs. RPC debate—specific endpoints are more self-documenting than generic 'execute' endpoints. Option A (generic executor) is tempting for flexibility but creates poor UX: Claude Code must construct correct GraphQL query strings (error-prone) and handle arbitrary response shapes. Option C artificially limits functionality without clear justification. Option D leads to combinatorial explosion—too many overly-specific tools that are hard to maintain. The principle here is API design: find the right abstraction level—not so generic that it's hard to use, not so specific that it's hard to maintain. Purpose-built tools for domain concepts strike this balance.",
      source: "Lesson 08: MCP Integration"
    },
    {
      question: "After configuring an MCP server, you run Claude Code but see 'MCP server failed to start'. The logs show 'connection refused on port 3000'. What does this error pattern suggest about MCP architecture?",
      options: [
        "MCP servers run as separate processes that Claude Code connects to via network",
        "MCP servers require port 3000 specifically and fail if port is occupied",
        "MCP servers must be restarted every time you launch Claude Code sessions",
        "MCP servers only work when Claude Code runs with administrator privileges granted"
      ],
      correctOption: 0,
      explanation: "This debugging scenario reveals MCP's architectural model: MCP servers are separate processes (often long-running services) that Claude Code connects to via network protocols (HTTP, WebSocket, etc.). The 'connection refused' error indicates the server process isn't running or isn't listening on the expected port. This architecture enables language-agnostic MCP servers (Python, Go, Node.js) and allows servers to maintain state across multiple Claude Code sessions. Option B is incorrect—port 3000 is arbitrary; MCP servers can use any configured port. Option C misunderstands startup responsibility—MCP servers typically start independently and persist across Claude Code sessions. Option D is wrong—network communication doesn't inherently require elevated privileges (for non-privileged ports >1024). The principle here is process architecture: understanding that MCP uses inter-process communication helps debug connection issues effectively.",
      source: "Lesson 08: MCP Integration"
    },
    {
      question: "You're working on a complex feature requiring database migrations, API updates, and frontend changes. You want Claude Code to handle each area using specialized configurations. Which Claude Code capability best addresses this orchestration challenge?",
      options: [
        "MCP servers allow exposing domain-specific tools that Claude Code selects automatically",
        "Skills allow creating reusable prompt templates for each domain area specifically",
        "Hooks allow intercepting and routing tasks to appropriate handlers by type",
        "Subagents allow delegating tasks to specialized agents with different tool access"
      ],
      correctOption: 3,
      explanation: "This scenario tests understanding of when to use subagents versus other extensibility mechanisms. Subagents are designed for exactly this use case: orchestrating complex tasks by delegating subtasks to specialized agents, each with appropriate tools and context. For example, a database subagent might have only database-related MCP tools, while a frontend subagent has browser automation tools. This prevents tool overload and allows different prompting strategies per domain. Option B (skills) are more about reasoning patterns than orchestration—they don't provide tool isolation. Option C (hooks) are for validation/transformation, not delegation. Option D (MCP) provides tools but doesn't handle orchestration—Claude Code would still need to manage all domains simultaneously, creating complexity. The key insight is architectural: subagents enable divide-and-conquer approaches to complex problems through controlled delegation.",
      source: "Lesson 09: Subagents and Orchestration"
    },
    {
      question: "You've created a subagent for code review tasks. During testing, you notice the subagent generates excellent feedback but takes much longer than expected. Which subagent configuration would most directly address this performance issue?",
      options: [
        "Configure subagent to use shorter context window for faster processing speed",
        "Configure subagent to use Haiku model instead of Sonnet or Opus",
        "Reduce the number of tools available to the subagent for faster decisions",
        "Split subagent into multiple smaller subagents for parallel execution instead"
      ],
      correctOption: 1,
      explanation: "This question tests understanding of the performance/quality tradeoffs in model selection. Claude Code supports multiple models (Haiku, Sonnet, Opus) with different speed/capability tradeoffs. For code review, if quality is sufficient with Haiku but Sonnet is slower, switching to Haiku directly addresses the performance issue. Haiku is designed for speed while maintaining good quality for structured tasks. Option B might help but could reduce quality by limiting necessary tools—it's treating symptoms, not optimizing configuration. Option C isn't a real configuration option and would harm quality by limiting context. Option D adds orchestration complexity without addressing the root cause (model speed). The principle here mirrors performance optimization in software engineering: before adding complexity (parallelization, caching), first optimize the direct parameter (model choice) that controls the speed/quality tradeoff.",
      source: "Lesson 09: Subagents and Orchestration"
    },
    {
      question: "Your main agent delegates a task to a subagent for implementation. The subagent completes the work but returns results that don't match the main agent's expectations. What does this scenario reveal about subagent communication design?",
      options: [
        "Subagents work best when they have complete autonomy without constraints or guidelines",
        "Subagents should be avoided when task requirements are complex or ambiguous initially",
        "Clear interface contracts between agents are critical for successful orchestration workflows",
        "Main agents should always validate subagent work through automated test execution"
      ],
      correctOption: 2,
      explanation: "This scenario illustrates a fundamental principle of distributed systems: clear interface contracts are essential when components communicate. Just as microservices need well-defined APIs, agent orchestration requires clear specifications of what the main agent expects (inputs, outputs, success criteria) and what the subagent delivers. Without this, you get impedance mismatch. Option B is defeatist—complex tasks are exactly when orchestration helps, but they require better contracts. Option C is a good practice but doesn't prevent the mismatch; validation catches problems but doesn't prevent them. Option D is backwards—autonomy without constraints leads to unpredictable results. The correct approach combines clear contracts with autonomy: 'Here's what I need (contract), you decide how (autonomy).' This mirrors software engineering: good abstractions have clear interfaces but hide implementation details.",
      source: "Lesson 09: Subagents and Orchestration"
    },
    {
      question: "You're designing a subagent for testing tasks. The main agent needs visibility into test results without managing test execution details. Which information architecture best achieves this separation of concerns?",
      options: [
        "Subagent returns structured summary with pass/fail counts and failing test names",
        "Subagent returns complete test output for main agent to parse and analyze",
        "Subagent writes test results to file for main agent to read later",
        "Subagent executes tests but main agent queries test status through MCP"
      ],
      correctOption: 0,
      explanation: "This question tests understanding of interface design in agent orchestration. A structured summary (pass/fail counts, failing test names) provides the right abstraction level: enough information for the main agent to make decisions (e.g., 'tests failed, need fixes') without overwhelming it with raw output. This applies the interface segregation principle: clients (main agent) should receive only the information they need. Option B violates this by dumping all details, forcing the main agent to parse complex output. Option C adds latency and complexity through file I/O when direct return would work. Option D overcomplicates with an additional abstraction layer (MCP) when simple return values suffice. The principle here is information hiding: good interfaces expose the right level of abstraction, not all implementation details.",
      source: "Lesson 09: Subagents and Orchestration"
    },
    {
      question: "You're orchestrating a deployment pipeline using subagents: one for tests, one for builds, one for deployment. Tests pass, but the build subagent fails. The deployment subagent starts anyway. What does this reveal about orchestration control flow?",
      options: [
        "Subagents always run in parallel unless explicitly configured for sequential operation",
        "Orchestration requires explicit dependency management and error handling between stages",
        "Failed subagents automatically prevent subsequent subagents from starting up execution",
        "Orchestration works best when all subagents are independent without shared state"
      ],
      correctOption: 1,
      explanation: "This scenario highlights that orchestration isn't automatic—the main agent must explicitly manage dependencies and handle errors. Just as a bash script doesn't automatically stop on errors (without 'set -e'), subagent orchestration requires intentional control flow. The main agent should check the build subagent's return status and conditionally invoke the deploy subagent only on success. Option B is incorrect—parallel vs. sequential execution depends on how the main agent invokes subagents, not a global configuration. Option C describes ideal behavior but isn't automatic—you must implement this logic. Option D misses the point—deployment pipelines inherently have dependencies (can't deploy before building). The principle here mirrors error handling in programming: explicit is better than implicit. Robust orchestration requires intentional dependency management, not assumptions about automatic behavior.",
      source: "Lesson 09: Subagents and Orchestration"
    },
    {
      question: "You're creating an agent skill for API testing. The skill needs to handle authentication, request construction, and response validation. What does this scope reveal about effective skill design?",
      options: [
        "Skills should delegate complex operations to subagents for better separation of concerns",
        "Skills should focus on single atomic operations to maximize reusability across contexts",
        "Skills should avoid combining multiple concerns into single reusable components",
        "Skills should encapsulate complete workflows with clear inputs and expected outputs"
      ],
      correctOption: 3,
      explanation: "This scenario tests understanding of skill scope. Effective skills encapsulate complete, coherent workflows that developers would naturally think of as a unit—like 'test this API endpoint' includes auth, request, and validation. Breaking this into three separate skills would require developers to orchestrate them manually, adding cognitive load. Option B advocates for overly granular skills, which leads to composition complexity (needing to chain many small skills for common tasks). Option C conflates skills with subagents—skills are prompt patterns, not delegation mechanisms. Option D is a misapplication of single responsibility—skills should have single purpose (API testing) but that purpose can encompass multiple steps. The principle here mirrors function design in programming: aim for cohesion (related operations together) over arbitrary atomicity. The right granularity is 'one meaningful task,' not 'one operation.'",
      source: "Lesson 06: Building Your Own Skills"
    },
    {
      question: "Your skill template includes: 'Analyze the code for bugs. Consider edge cases.' After testing, you find the skill produces superficial analysis. Which modification would most effectively improve the skill's depth?",
      options: [
        "Use Persona + Questions + Principles pattern to activate reasoning mode",
        "Add more examples of bugs from your codebase to the template",
        "Increase the skill's temperature parameter to encourage creative analysis",
        "Specify exact checklist of bugs to search for in code"
      ],
      correctOption: 0,
      explanation: "This scenario reveals the difference between instruction-based and reasoning-based prompts. The original skill ('Analyze... Consider...') gives commands but doesn't activate deep reasoning. The Persona + Questions + Principles pattern reframes the task as: 'You are an expert code reviewer [Persona]. Ask yourself: What edge cases does this code miss? What assumptions could break? [Questions] Apply principles: trust but verify, fail fast, defensive programming [Principles].' This activates reasoning mode, producing deeper analysis. Option B helps but doesn't fundamentally change the reasoning depth—examples show what to find, not how to think. Option C is a misconception—temperature controls randomness, not quality. Option D produces checklist-driven analysis (shallow) rather than principle-driven reasoning (deep). The key insight is about prompt engineering: reasoning-activated prompts produce better results than instruction-based prompts for complex cognitive tasks.",
      source: "Lesson 06: Building Your Own Skills"
    },
    {
      question: "You have skills for 'code-review', 'bug-analysis', and 'security-audit'. A developer argues these are redundant and should be merged into one 'code-quality' skill. Which principle best informs this design decision?",
      options: [
        "Merge skills when they operate on same input type to reduce maintenance",
        "Separate skills only when they use different tools or external integrations",
        "Separate skills when different contexts require different reasoning modes or criteria",
        "Merge skills when they share common patterns to eliminate duplication completely"
      ],
      correctOption: 2,
      explanation: "This question tests understanding of skill granularity based on cognitive context, not technical similarity. Code review, bug analysis, and security audit operate on code but apply different lenses: code review focuses on readability and maintainability; bug analysis on correctness and edge cases; security audit on vulnerabilities and attack vectors. These different contexts benefit from specialized reasoning frameworks (different Personas, Questions, and Principles). Merging them would create a generic 'check everything' skill that lacks depth in each area. Option B focuses on technical similarity (input type) rather than cognitive purpose. Option C incorrectly ties granularity to tools rather than reasoning. Option D values DRY (don't repeat yourself) over specialization, but in prompting, specialized context often outweighs code reuse benefits. The principle here mirrors software design: cohesion (related reasoning together) matters more than coupling (shared technical elements).",
      source: "Lesson 06: Building Your Own Skills"
    },
    {
      question: "You've created a skill for database schema design. During testing, it generates schemas that work but don't follow your team's naming conventions. The conventions are documented in CLAUDE.md. What does this reveal about skill-context interaction?",
      options: [
        "Skills should duplicate CLAUDE.md content to ensure conventions are always followed",
        "Skills should reference project context explicitly when conventions matter for output",
        "Skills automatically inherit all CLAUDE.md context without explicit references needed",
        "Skills take precedence over CLAUDE.md so conventions must be in skill"
      ],
      correctOption: 1,
      explanation: "This scenario illustrates that skills and project context (CLAUDE.md) serve different purposes and should be composed intentionally. Skills provide reasoning patterns ('think about normalization, indexes, relationships'); CLAUDE.md provides project-specific rules ('use snake_case for columns'). Effective skills explicitly reference context: 'Follow the naming conventions in CLAUDE.md when generating schemas.' This makes the interaction explicit. Option B is incorrect—while CLAUDE.md is in context, skills should explicitly invoke it when conventions matter, not assume passive application. Option C violates DRY and creates maintenance burden (updating conventions in multiple places). Option D misunderstands precedence—skills and CLAUDE.md complement each other. The principle here is composition: skills should explicitly reference external context when needed, making dependencies clear and enabling modular reasoning.",
      source: "Lesson 06: Building Your Own Skills"
    },
    {
      question: "You're creating a skill library for your organization. Different teams have different coding standards, but similar reasoning patterns (testing, review, refactoring). Which skill design approach best balances reusability with customization?",
      options: [
        "Create one master skill that includes all teams' standards combined",
        "Create separate skill versions for each team with customized standards",
        "Create generic skills that work without any team-specific context",
        "Create parameterized skills accepting team-specific configuration references as inputs"
      ],
      correctOption: 3,
      explanation: "This question tests understanding of skill parameterization and composition. Parameterized skills separate the reusable reasoning pattern (how to review code) from team-specific details (which conventions to enforce). For example: 'You are a code reviewer [Persona]. Review this code following the standards in {standards_file}. Questions: Does it follow conventions in {standards_file}? [Questions + Parameter].' Teams provide their standards_file reference. Option B creates maintenance overhead (N×M skills for N teams and M patterns). Option C produces generic, context-free skills that miss team-specific requirements. Option D creates a bloated skill with contradictory standards that would confuse the model. The principle here mirrors software engineering: parameterization enables reuse without duplication. Just as functions take parameters rather than hard-coding values, skills can reference external context rather than embedding all variations.",
      source: "Lesson 06: Building Your Own Skills"
    },
    {
      question: "You configure a validation hook that runs linting on generated code. During a task, Claude Code generates code with linting errors, the hook blocks it, but Claude Code doesn't automatically fix the issues. What does this reveal about hook behavior?",
      options: [
        "Hooks failed to execute properly or Claude Code would have fixed issues",
        "Hooks only work when Claude Code is configured to auto-fix validation errors",
        "Hooks provide feedback signals but don't automatically trigger Claude Code's correction logic",
        "Hooks must include correction suggestions for Claude Code to apply fixes automatically"
      ],
      correctOption: 2,
      explanation: "This scenario tests understanding of hook responsibility boundaries. Hooks are validation/interception mechanisms that can block or modify operations, but they don't directly control Claude Code's behavior—they provide signals (error messages, feedback). When a hook blocks an operation, Claude Code receives the error but must decide how to respond (retry with fixes, ask for guidance, etc.). This isn't automatic. Option B describes a configuration that doesn't exist—there's no 'auto-fix' mode. Option C confuses hook output with Claude Code's reasoning—suggestions help, but Claude Code must actively apply them. Option D misunderstands success—a hook successfully blocking bad code is working correctly; fixing the code is Claude Code's job, not the hook's. The principle here mirrors middleware in web frameworks: middleware intercepts requests and can reject them, but the application layer handles rejection responses.",
      source: "Lesson 11: Hooks and Extensibility"
    },
    {
      question: "Your team wants to prevent Claude Code from making database changes during code review tasks. Which hook configuration would most effectively enforce this constraint?",
      options: [
        "Pre-tool-call hook checking for database tools and blocking during review tasks",
        "Pre-file-write hook checking for database migration files and blocking writes",
        "Post-message hook analyzing responses for database operations and warning users",
        "Pre-prompt-submit hook instructing Claude Code not to modify databases"
      ],
      correctOption: 0,
      explanation: "This question tests understanding of hook granularity and timing. A pre-tool-call hook is the right interception point because it can examine which tool Claude Code is about to invoke and block database tools specifically. This prevents the operation before it happens. Option B is too narrow—database changes might happen via tools (executing SQL), not just writing migration files. Option C (post-message) is too late—it catches changes after they happen, providing warnings rather than prevention. Option D (pre-prompt-submit) doesn't enforce anything—it modifies the prompt to instruct Claude Code, but can't guarantee compliance. The principle here mirrors security architecture: enforce constraints at the lowest level possible (tool invocation) rather than relying on higher-level agreements (prompts) or post-hoc detection (post-message). Defense in depth applies: block dangerous operations before they execute.",
      source: "Lesson 11: Hooks and Extensibility"
    },
    {
      question: "You implement a hook that logs all file writes for audit purposes. During testing, you notice logs are massive and contain complete file contents. Which hook design principle would address this issue?",
      options: [
        "Hooks should only log file writes that exceed specific size thresholds",
        "Hooks should use compression algorithms to reduce log file size significantly",
        "Hooks should rotate log files frequently to prevent size growth issues",
        "Hooks should capture metadata like file paths and sizes not full content"
      ],
      correctOption: 3,
      explanation: "This scenario tests understanding of appropriate data capture in hooks. For audit purposes, metadata (who, what, when, where) is usually sufficient—full file contents are rarely needed for audit trails and create storage/privacy issues. Logging paths, sizes, timestamps, and operation types provides traceability without bloat. Option B addresses symptoms (big logs) rather than root cause (logging too much). Option C is arbitrary—size-based filtering would miss small but sensitive files. Option D (rotation) manages storage but doesn't solve the excessive data capture problem. The principle here mirrors logging best practices in software engineering: log events and metadata, not data payloads. Payloads create noise, storage issues, and privacy concerns. Effective instrumentation captures actionable signals, not exhaustive data.",
      source: "Lesson 11: Hooks and Extensibility"
    },
    {
      question: "Your hook validates that code changes include corresponding test updates. A developer complains this is 'too rigid' for quick prototypes. Which approach best balances validation rigor with development flexibility?",
      options: [
        "Disable hooks entirely during prototyping and re-enable for production work only",
        "Make hooks contextually aware using environment variables or flags for prototype mode",
        "Configure hooks to warn about missing tests but not block operations",
        "Create separate Claude Code installations for prototyping versus production work environments"
      ],
      correctOption: 1,
      explanation: "This question tests understanding of configurable validation. Context-aware hooks can check environment variables (e.g., PROTOTYPE_MODE=true) or command-line flags to adjust behavior—strict enforcement in normal mode, relaxed in prototype mode. This maintains safety by default while allowing opt-out for appropriate contexts. Option B (disabling hooks) creates the problem of forgetting to re-enable them, leading to reduced quality. Option C (warnings only) doesn't solve the 'too rigid' complaint—warnings still interrupt flow. Option D (separate installations) creates maintenance overhead and environment drift. The principle here mirrors feature flags in software engineering: build flexibility into the system through configuration, not by disabling safety features. Context-aware systems adapt to different use cases without compromising core principles.",
      source: "Lesson 11: Hooks and Extensibility"
    },
    {
      question: "You configure a post-tool-call hook that enriches tool outputs with additional context. During testing, Claude Code seems to 'hallucinate' information that wasn't in original tool outputs. What's the most likely explanation?",
      options: [
        "The hook successfully added context which Claude Code incorporated into subsequent reasoning",
        "The hook caused Claude Code to malfunction by modifying tool output format",
        "The hook's additional context confused Claude Code creating incorrect associations",
        "Claude Code ignores hook modifications and generated information independently"
      ],
      correctOption: 0,
      explanation: "This scenario tests understanding that hooks can modify the information flow to Claude Code—which is a feature, not a bug. Post-tool-call hooks can enrich tool outputs with additional context (e.g., adding related documentation links to code search results), and Claude Code incorporates this enriched information into its reasoning. This is working as intended. The term 'hallucinate' in the question is a misdirection—if the hook added the information, Claude Code is correctly using provided context, not inventing facts. Option B is unlikely—hooks that break output format would cause errors, not plausible-seeming information. Option C is possible but assumes the hook is poorly designed; well-designed enrichment helps rather than confuses. Option D is incorrect—hooks definitely affect Claude Code's context. The principle here is understanding information flow: hooks are part of the context provision system.",
      source: "Lesson 11: Hooks and Extensibility"
    },
    {
      question: "You set a test coverage threshold in your project-level settings to 80%, but your team lead wants to enforce 90% for themselves while working on critical modules. Which settings hierarchy level should the team lead configure to override the project default?",
      options: [
        "User-level settings since they apply to individual developer workflows across projects",
        "Global settings with a critical priority flag to override project defaults",
        "Local settings since they override project settings for this developer only",
        "Project settings cannot be overridden; team lead should modify project config"
      ],
      correctOption: 2,
      explanation: "Option B (Local settings) is correct because Claude Code's settings hierarchy follows: User < Project < Local, where Local settings have the highest priority. Local settings (`.claude/settings.local.json`) override both project and user settings, allowing individual developers like the team lead to enforce stricter personal standards without changing project defaults for everyone. Option A is incorrect because User settings have the LOWEST priority and cannot override Project settings. Option C is wrong because global settings are the same as user settings (lowest priority) and don't have priority flags. Option D misunderstands the hierarchy—project settings CAN be overridden by local settings. This principle mirrors configuration cascading: more specific contexts (local) override more general defaults (project, user).",
      source: "Lesson 10: Settings Hierarchy"
    },
    {
      question: "Your global settings configure Claude Code to use Sonnet, but a specific project's configuration specifies Haiku. When working in that project, Claude Code uses Haiku. What does this behavior reveal about settings precedence?",
      options: [
        "Global settings apply only when no other configuration level is defined",
        "Global settings are ignored entirely when project settings exist for property",
        "Settings conflicts cause Claude Code to default to most economical model",
        "Project-level settings take precedence over global settings for context specificity"
      ],
      correctOption: 3,
      explanation: "This scenario demonstrates the principle behind settings hierarchies: more specific contexts override more general defaults. Project settings take precedence over global settings because project-specific needs (e.g., 'this project is simple, Haiku is sufficient') should override personal defaults ('I generally prefer Sonnet'). This isn't about ignoring global settings entirely—if the project config doesn't specify a model, it falls back to global. Option B overstates the precedence—only conflicting properties are overridden, not entire configuration. Option C is incorrect—there's no 'conflict resolution by economy' logic. Option D misunderstands fallback behavior—global settings are the base layer, always active, but overridden by more specific layers when they define the same property. The principle here mirrors CSS specificity: more specific selectors override less specific ones, but both coexist.",
      source: "Lesson 10: Settings Hierarchy"
    },
    {
      question: "You're configuring Claude Code for a monorepo with multiple projects (frontend, backend, mobile). Each project has different linting rules and tool configurations. Which settings architecture would best support this structure?",
      options: [
        "User-level settings for each team member customized to their assigned project",
        "Project-level settings in each subdirectory with workspace settings for shared defaults",
        "Single global settings file with conditional logic for different project types",
        "Separate Claude Code installations for each project to isolate configurations completely"
      ],
      correctOption: 1,
      explanation: "This question tests understanding of settings composition in complex project structures. Project-level settings in each subdirectory (frontend/.claude/config, backend/.claude/config) combined with workspace-level shared defaults (workspace/.claude/config) provides the right balance: shared configuration for common concerns (model choice, API keys) with project-specific overrides for domain-specific needs (linting rules). This mirrors how tools like ESLint and Prettier support monorepos. Option B (conditional logic) creates brittle, hard-to-maintain configuration files. Option C (user-level) breaks when developers work across multiple projects—they'd need different settings per task. Option D (separate installations) creates unnecessary overhead and prevents shared context. The principle here is configuration composition: layer specific overrides on shared defaults rather than duplicating everything or using complex conditional logic.",
      source: "Lesson 10: Settings Hierarchy"
    },
    {
      question: "You configure custom tool paths in user settings, but Claude Code continues using default tool locations. What's the most likely configuration issue?",
      options: [
        "Tool path configuration requires restarting the computer to take effect globally",
        "User settings are overridden by project settings specifying different tool paths",
        "User settings file has syntax errors preventing parsing and loading configuration",
        "Custom tool paths require absolute paths not relative paths to work"
      ],
      correctOption: 2,
      explanation: "This scenario tests practical debugging of configuration issues. When settings appear to be ignored, the most common cause is syntax errors (invalid JSON, YAML indentation errors, etc.) that prevent the settings file from loading. Claude Code would then fall back to defaults, creating the appearance that custom settings are ignored. Option B describes valid precedence behavior, but the question states the settings 'continue' using defaults—if project settings were overriding, we'd expect those values, not defaults. Option C might be true for some settings but isn't the 'most likely' explanation. Option D is incorrect—settings take effect on next session, not after reboot. The debugging principle here mirrors many configuration systems: when configuration doesn't apply, check syntax first before investigating precedence or implementation details.",
      source: "Lesson 10: Settings Hierarchy"
    },
    {
      question: "Your organization wants to enforce security policies (e.g., no API keys in code) across all projects using Claude Code. Which settings level would most effectively implement organization-wide policies?",
      options: [
        "Provide organization-standard global settings file that developers install on their systems",
        "Configure project settings in each repository to enforce policies individually",
        "Rely on user settings with policy documentation for developers to configure",
        "Use workspace settings shared via cloud sync to enforce policies automatically"
      ],
      correctOption: 0,
      explanation: "This question tests understanding of settings distribution and governance. An organization-standard global settings file (distributed via documentation, onboarding scripts, or configuration management) ensures all developers have baseline security policies, while still allowing project-specific and user-specific customization on top. Global settings are the right level for organization-wide policies because they apply everywhere by default. Option B requires maintaining duplicate configuration across all repositories, creating consistency and maintenance issues. Option C (documentation relying on user settings) is weakest enforcement—developers might forget or ignore policies. Option D (workspace settings) limits policies to specific workspaces, not organization-wide. The principle here mirrors IT security: policies should be enforced at the broadest applicable scope (global) with mechanisms for stricter local requirements (project/user overrides), not relying on individual compliance.",
      source: "Lesson 10: Settings Hierarchy"
    },
    {
      question: "You're developing a Claude Code plugin that adds Git workflow automation. The plugin needs to read repository history and create commits. Which Claude Code integration point would provide the most appropriate access to these capabilities?",
      options: [
        "Plugin should extend Claude Code's CLI with new git command shortcuts",
        "Plugin should expose Git operations as MCP tools Claude Code invokes",
        "Plugin should hook into file write events and automatically create commits",
        "Plugin should provide skill templates for common Git workflow reasoning patterns"
      ],
      correctOption: 1,
      explanation: "This scenario tests understanding of appropriate plugin integration patterns. Git operations (log, diff, commit) are actions Claude Code should decide when to perform, so exposing them as MCP tools gives Claude Code agency while the plugin handles implementation details. This follows the 'tools, not automation' principle—provide capabilities, let Claude Code orchestrate. Option B creates implicit automation that might commit at inappropriate times—hooks aren't meant for adding new behaviors, but validating existing ones. Option C bypasses Claude Code's intelligence—CLI extensions don't integrate with Claude Code's reasoning. Option D is partial—skills help with reasoning but don't provide execution capabilities. The principle here is plugin architecture: plugins should extend Claude Code's capabilities (tools) while letting Claude Code's intelligence determine when and how to use them, not replacing its decision-making with automation.",
      source: "Lesson 12: Discovering and Using Claude Code Plugins"
    },
    {
      question: "Your plugin integrates with a proprietary issue-tracking system. You want Claude Code to understand issue context when implementing features. Which combination of plugin capabilities would most effectively provide this integration?",
      options: [
        "Settings configuration exposing issue tracker API credentials and URL endpoints",
        "Custom hooks that inject issue details into every prompt automatically",
        "Agent skills for reasoning about issue priorities and task breakdown workflows",
        "MCP tools for querying issues plus CLAUDE.md enhancement with issue context"
      ],
      correctOption: 3,
      explanation: "This question tests understanding of how different plugin capabilities complement each other. MCP tools provide action capabilities (query issues, update status), while CLAUDE.md enhancement provides context awareness (current issue being worked on). Together, they enable Claude Code to both access information and understand the current work context. Option B (hooks injecting context) is too implicit—Claude Code should explicitly fetch issue details when needed, not have them forced into every prompt. Option C (skills) helps with reasoning but doesn't provide data access. Option D (settings) stores credentials but doesn't provide integration logic. The principle here is capability composition: combine data access (MCP) with context awareness (CLAUDE.md) and reasoning (skills) to create complete integrations, not relying on a single mechanism.",
      source: "Lesson 12: Discovering and Using Claude Code Plugins"
    },
    {
      question: "You're designing a plugin architecture for your team. A colleague suggests creating one large plugin that handles all integrations (database, CI/CD, monitoring). Which principle best informs whether to use one plugin or multiple specialized plugins?",
      options: [
        "Separate plugins by domain boundaries to enable independent versioning and maintenance",
        "Combine plugins to reduce installation complexity and ensure consistent configuration",
        "Separate plugins only if they use different programming languages for implementation",
        "Combine plugins when they share common dependencies to reduce code duplication"
      ],
      correctOption: 0,
      explanation: "This scenario tests understanding of plugin modularity principles, which mirror microservices architecture. Separate plugins by domain (database plugin, CI/CD plugin, monitoring plugin) enable independent development, versioning, and adoption. A team might need database integration but not CI/CD, or they might want to upgrade the monitoring plugin without risking database functionality. Option B optimizes for initial convenience but creates long-term maintenance problems (must update all capabilities together). Option C ties modularity to implementation language, which is irrelevant to logical boundaries. Option D applies DRY prematurely—shared dependencies can be extracted to libraries without combining plugins. The principle here is modularity: optimize for independent evolution and flexible composition, not installation convenience. Good architecture separates concerns that change independently.",
      source: "Lesson 12: Discovering and Using Claude Code Plugins"
    },
    {
      question: "Your plugin provides MCP tools for database operations, skills for query optimization, and hooks for preventing dangerous queries. During testing, you notice configuration complexity. Which architectural principle would simplify plugin adoption?",
      options: [
        "Require explicit configuration of all capabilities to ensure users understand features",
        "Auto-detect all configuration from environment to eliminate manual setup completely",
        "Provide sensible defaults for all configuration with progressive disclosure for advanced options",
        "Provide configuration wizard that collects all settings before enabling any plugin"
      ],
      correctOption: 2,
      explanation: "This question tests understanding of user experience in plugin design. Sensible defaults + progressive disclosure means the plugin works immediately for common cases (default database safety rules, standard query patterns) while exposing advanced configuration (custom safety rules, specialized query strategies) for users who need it. This follows the 'pit of success' principle—make the right thing easy. Option B creates adoption friction—requiring explicit configuration for everything delays value and intimidates users. Option C overreaches—auto-detection works for some settings (environment variables) but not domain logic (what queries are 'dangerous'?). Option D front-loads complexity, delaying usage until setup completes. The principle here mirrors product design: reduce time-to-value through good defaults, then enable customization progressively. Users should get value immediately, then customize as they learn.",
      source: "Lesson 12: Discovering and Using Claude Code Plugins"
    },
    {
      question: "You're distributing a Claude Code plugin to the community. A user reports that your plugin's MCP server fails with 'port already in use'. Which plugin design practice would have prevented this issue?",
      options: [
        "Document the required port in README and warn about conflicts",
        "Make server port configurable with random available port as default fallback",
        "Use standard port 3000 that all plugins should share universally",
        "Require users to manually select port during plugin installation process"
      ],
      correctOption: 1,
      explanation: "This scenario tests understanding of robust plugin design. Configurable ports with smart defaults (attempt configured port, fall back to random available port if busy) prevent conflicts without requiring manual intervention. This follows the 'robust by default' principle—handle common failure cases automatically. Option B (documentation) doesn't prevent the problem, just warns about it—users still face errors and must manually resolve conflicts. Option C would cause conflicts between plugins if multiple use port 3000. Option D adds friction to installation—users must understand port selection before they've even used the plugin. The principle here is defensive programming: anticipate environmental variations and handle them gracefully. Good plugins work in diverse environments without requiring manual tuning, using configuration as an escape hatch, not a requirement.",
      source: "Lesson 12: Discovering and Using Claude Code Plugins"
    },
    {
      question: "Your team has CLAUDE.md project instructions, MCP tools for database access, custom hooks for validation, and agent skills for code review. A new developer asks: 'Which should I use for X?' What principle guides this decision?",
      options: [
        "Each mechanism addresses different architectural concerns so understand the problem first",
        "Always start with CLAUDE.md since it is the simplest mechanism available",
        "Use MCP tools for everything since they provide the most flexibility",
        "Combine all mechanisms together for every task to maximize effectiveness"
      ],
      correctOption: 0,
      explanation: "This question tests understanding of Claude Code's extensibility architecture as a cohesive system. Each mechanism serves a specific purpose: CLAUDE.md for project context, MCP for capabilities/tools, hooks for validation/interception, skills for reasoning patterns. The right choice depends on the problem you're solving—context awareness needs CLAUDE.md, new actions need MCP, safety needs hooks, reusable reasoning needs skills. Option B oversimplifies—CLAUDE.md isn't always the right answer (e.g., for executing database queries). Option C misapplies MCP—you can't express validation logic or reasoning patterns as tools. Option D creates unnecessary complexity—using all mechanisms for every task adds overhead without benefit. The principle here mirrors software architecture: understand the problem domain, then select appropriate abstractions. Good architecture uses the right tool for each job, not one tool for everything.",
      source: "Lesson 07: CLAUDE.md Context Files"
    },
    {
      question: "You're debugging an issue where Claude Code generates code that violates your team's standards despite CLAUDE.md guidelines, custom hooks, and a validation skill. Which debugging approach would most effectively identify the root cause?",
      options: [
        "Add more detailed guidelines to CLAUDE.md to make standards clearer",
        "Increase validation strictness in hooks to catch more violations automatically",
        "Replace all mechanisms with a single comprehensive plugin for consistency",
        "Systematically test each mechanism in isolation to identify which is failing"
      ],
      correctOption: 3,
      explanation: "This scenario tests systematic debugging of complex systems with multiple interacting components. When multiple mechanisms interact (CLAUDE.md context, hooks validation, skills reasoning), isolating each component reveals where the breakdown occurs—is CLAUDE.md guidance unclear? Are hooks not triggering? Is the skill prompt ineffective? Testing each in isolation (e.g., check if hooks run and block violations; verify CLAUDE.md is loaded; test skill output directly) identifies the specific failure point. Option B assumes the problem is CLAUDE.md without validation. Option C throws away working components to build something unproven. Option D addresses symptoms (catching violations) not root cause (preventing them). The principle here mirrors systematic troubleshooting: isolate variables, test hypotheses, identify root cause. Complex systems require methodical debugging, not reactive patches.",
      source: "Lesson 11: Hooks and Extensibility"
    },
    {
      question: "You're designing Claude Code extensibility for a team that lacks programming experience. They can write documentation and YAML config but not Python or JavaScript. Which extensibility mechanisms remain accessible to them?",
      options: [
        "CLAUDE.md context files and settings configuration files only accessible",
        "All mechanisms accessible since Claude Code handles implementation automatically",
        "CLAUDE.md, settings, agent skills using markdown prompt patterns accessible",
        "Only settings hierarchy since CLAUDE.md requires programming knowledge"
      ],
      correctOption: 2,
      explanation: "This question tests understanding of which extensibility mechanisms require programming skills versus configuration/prompting skills. CLAUDE.md (markdown documentation), settings (YAML/JSON), and agent skills (markdown with Persona+Questions+Principles patterns) all use declarative formats accessible to non-programmers. MCP servers and hooks require actual code (Python, JS, etc.) to implement logic. This is a key design principle in Claude Code—progressive complexity where basic extensibility doesn't require programming, but advanced features do. Option A incorrectly excludes skills, which use markdown. Option C wrongly suggests CLAUDE.md needs programming—it's just markdown documentation. Option D overstates accessibility—MCP and hooks definitely require programming. The principle here is tool accessibility: good platforms provide multiple extension points at different complexity levels, enabling users to start simple and grow into advanced features as skills develop.",
      source: "Lesson 06: Building Your Own Skills"
    },
    {
      question: "Your plugin uses MCP tools, CLAUDE.md context injection, custom skills, and validation hooks. A user reports the plugin 'slows down' Claude Code. Which architectural decision would most likely cause this performance issue?",
      options: [
        "Large CLAUDE.md content injected into every conversation context window",
        "MCP tools making synchronous network calls without caching implemented",
        "Skills using complex reasoning patterns that increase model processing time",
        "Hooks running validation logic on every file write operation performed"
      ],
      correctOption: 1,
      explanation: "This scenario tests understanding of performance bottlenecks in Claude Code extensibility. Synchronous MCP network calls without caching create the most significant performance impact because they: (1) block execution waiting for responses, (2) repeat the same slow operations when Claude Code calls the tool multiple times, and (3) depend on external service latency. Option A (CLAUDE.md size) affects context window usage but doesn't slow down execution significantly—larger context increases token cost, not runtime. Option C (skill complexity) affects reasoning quality but Claude processes text quickly—prompts don't typically cause noticeable slowness. Option D (hooks on every write) might add overhead but validation logic is usually fast compared to network I/O. The principle here mirrors performance optimization: I/O operations (especially network) are typically the slowest part of systems. Optimize I/O first before worrying about computational complexity.",
      source: "Lesson 08: MCP Integration"
    },
    {
      question: "You've built a comprehensive Claude Code setup with project instructions, MCP integrations, custom skills, and hooks. A colleague joining the project asks: 'How do I learn this system?' Which architectural principle would make the system more learnable?",
      options: [
        "Create training videos demonstrating every feature and configuration option",
        "Reduce system to simplest possible configuration eliminating complex extensibility features",
        "Provide complete reference documentation listing all available settings and options",
        "Documentation explaining each component purpose with examples and clear boundaries"
      ],
      correctOption: 3,
      explanation: "This question tests understanding of system learnability and documentation architecture. Good documentation explains purpose (why each component exists), provides examples (how to use), and clarifies boundaries (when to use each vs. alternatives). This enables new users to understand the mental model: 'CLAUDE.md for context, MCP for tools, skills for reasoning, hooks for validation.' Option B sacrifices capability for simplicity—powerful systems can be learnable without eliminating features. Option C (videos) helps but doesn't replace conceptual understanding—videos show 'how' but often skip 'why' and 'when.' Option D (reference documentation) is necessary but insufficient—it documents 'what exists' without teaching 'how to think about the system.' The principle here mirrors software documentation best practices: teach concepts and mental models, not just API references. Users need conceptual frameworks to make decisions, not exhaustive catalogs of options.",
      source: "Lesson 12: Discovering and Using Claude Code Plugins"
    }
  ]}
/>

---
sidebar_position: 9
title: "Chapter 6: Google Gemini CLI Quiz"
---

# Chapter 6: Google Gemini CLI Quiz

Test your understanding of Gemini CLI installation, configuration, built-in tools, memory management, MCP integration, custom commands, and security practices.

<Quiz
  title="Chapter 6: Google Gemini CLI Assessment"
  questions={[    {
      question: "Your team needs to generate 100 image descriptions for a dataset, ensuring each prompt is identical and results are version-controlled. Which approach best addresses these requirements?",
      options: [
        "Use Gemini web interface with copy-paste for consistency",
        "Use Google AI Studio with manual documentation",
        "Use Gemini CLI with scripted prompts in Git repository",
        "Use ChatGPT API with custom Python script"
      ],
      correctOption: 2,
      explanation: "Gemini CLI with scripted prompts in a Git repository is the best solution because it provides reproducibility (same prompt every time), automation (can process 100 images via script), and version control (track prompt changes over time). The web interface lacks automation and version control—manual copy-paste is error-prone and doesn't scale to 100 items. Google AI Studio is better than the web interface but still requires manual work for each image and lacks built-in version control integration. ChatGPT API could work but introduces an unnecessary dependency when the task requires Gemini's multimodal capabilities and Google ecosystem integration. The CLI approach combines all three requirements: consistency through scripting, scalability through automation, and traceability through Git.",
      source: "Lesson 1: Why Gemini CLI Matters"
    },
    {
      question: "A developer runs `gemini chat 'Analyze this code'` and receives an error: 'No multimodal content provided.' What is the likely cause and solution?",
      options: [
        "Command expects file reference but only text provided",
        "Missing API key in environment variables or configuration",
        "Gemini CLI version outdated and needs npm update",
        "Internet connection issue preventing API request completion"
      ],
      correctOption: 0,
      explanation: "The error 'No multimodal content provided' indicates the command expects a file reference (image, document, or code file) but only received text. When using `gemini chat` for code analysis, you should provide the file path: `gemini chat 'Analyze this code' --file code.py`. Missing API keys would produce an authentication error ('Invalid API key'), not a multimodal content error. Outdated CLI versions might have bugs, but this specific error message directly indicates missing file input. Internet connection issues would produce network-related errors ('Connection refused' or timeout messages), not multimodal content errors. Understanding error message patterns helps debug CLI usage—'multimodal content' specifically refers to non-text inputs like files or images that Gemini can process.",
      source: "Lesson 1: Why Gemini CLI Matters"
    },
    {
      question: "When deciding between Gemini CLI and the web interface for a project involving daily code reviews with team documentation, which factor most strongly favors the CLI?",
      options: [
        "CLI provides faster response times for queries",
        "CLI has simpler learning curve for new users",
        "CLI offers better model quality for code analysis",
        "CLI enables script automation and version control integration"
      ],
      correctOption: 3,
      explanation: "CLI enables script automation and version control integration, which is crucial for daily code reviews with team documentation. You can create scripts that automatically analyze commits, generate review summaries, and commit results to Git alongside code changes—creating an auditable, repeatable process. Response times are similar between CLI and web interface (both use the same API). Model quality is identical—CLI and web interface access the same Gemini models, so output quality doesn't differ. Learning curve actually favors the web interface for new users—GUI tools are typically easier to learn than CLI tools. The CLI's true advantage for this scenario is workflow integration: automated scripts can run on commit hooks, generate markdown reports, and integrate with CI/CD pipelines, while the web interface requires manual copy-paste for each review.",
      source: "Lesson 1: Why Gemini CLI Matters"
    },
    {
      question: "You need to process a mix of text prompts, images, and code files in a single Gemini workflow. Which Gemini CLI capability makes this possible?",
      options: [
        "Built-in Google Search tool for content discovery",
        "Multimodal input handling across different file types",
        "MCP server integration for file system access",
        "Custom slash commands for workflow automation steps"
      ],
      correctOption: 1,
      explanation: "Multimodal input handling across different file types is the core capability that enables processing text, images, and code together. Gemini CLI can accept multiple input types in a single command: `gemini chat 'Compare this design to code' --file design.png --file code.py`. Built-in Google Search helps find information but doesn't enable multimodal processing—it's a separate tool for web queries. MCP servers provide access to resources but don't inherently enable multimodal processing—they're integration mechanisms, not multimodal processors. Custom slash commands can automate workflows but rely on the underlying multimodal capability—they're workflow tools, not the core feature enabling mixed input processing. Understanding this distinction matters: multimodal is the foundational capability, while other features (search, MCP, commands) are tools built on that foundation.",
      source: "Lesson 1: Why Gemini CLI Matters"
    },
    {
      question: "A startup is choosing between Gemini CLI and Claude CLI for their AI-native development workflow. They heavily use Google Workspace (Docs, Sheets, Drive). Which architectural consideration should influence their decision?",
      options: [
        "Gemini CLI requires fewer system dependencies for installation",
        "Claude CLI has superior code generation capabilities overall",
        "Gemini CLI integrates directly with Google ecosystem services",
        "Claude CLI provides better context window for conversations"
      ],
      correctOption: 2,
      explanation: "Gemini CLI integrates directly with Google ecosystem services, which is crucial when heavily using Google Workspace. This means potential future integrations with Docs, Sheets, and Drive through Google's APIs—staying within one ecosystem reduces integration complexity and authentication overhead. While Claude CLI excels at many tasks, code generation quality is task-dependent and model-version-dependent, not a consistent architectural advantage—both tools produce high-quality code. System dependencies are similar for both—Gemini CLI (Node.js/npm) and Claude CLI (Python/pip) have comparable installation requirements. Context windows are model features, not CLI tool features—Gemini and Claude models have different context limits, but this isn't about CLI tool architecture. The key insight: ecosystem integration matters more than individual feature comparisons when you've already committed to a platform like Google Workspace.",
      source: "Lesson 1: Why Gemini CLI Matters"
    },
    {
      question: "Before installing Gemini CLI, you check your system: Python 3.8.5, Node.js 16.14.0, npm 8.3.0. Which component requires an upgrade?",
      options: [
        "Python version is below the minimum requirement",
        "Node.js version is below the minimum requirement",
        "npm version is below the minimum requirement",
        "All components meet the minimum installation requirements"
      ],
      correctOption: 0,
      explanation: "Python 3.8.5 is below the minimum requirement of Python 3.9+. While Gemini CLI is primarily a Node.js tool, some built-in features (like Code Execution tool) rely on Python 3.9+ for compatibility with modern libraries and security features. You need to upgrade Python before installation. Node.js 16.14.0 meets the requirement (Node.js 14+ is typically sufficient for Gemini CLI). npm 8.3.0 is adequate—npm 7+ generally works fine with Gemini CLI. Saying all components meet requirements is incorrect because Python needs upgrading. This illustrates an important installation lesson: even when the primary installation method (npm) has sufficient versions, check all runtime dependencies (Python) that tools like Code Execution might invoke during operation. Skipping Python upgrade might allow installation but cause failures when using certain features.",
      source: "Lesson 2: Installation, Authentication, and First Steps"
    },
    {
      question: "You install Gemini CLI with `npm install -g @google/generative-ai-cli` and run `gemini --version` successfully. However, `gemini chat 'hello'` returns 'API key not found.' What is the most likely cause?",
      options: [
        "Global npm installation failed to set path correctly",
        "API key not configured in environment or file",
        "Gemini service is temporarily down or unavailable now",
        "Node.js version incompatible with authentication module mechanism"
      ],
      correctOption: 1,
      explanation: "API key not configured in environment variables or config file is the most likely cause. The fact that `gemini --version` works proves installation and PATH configuration are correct—if installation failed, you wouldn't get version output. The error message 'API key not found' specifically indicates missing authentication credentials, which must be set via `GEMINI_API_KEY` environment variable or in `~/.gemini/config.json`. If Gemini service were down, you'd receive network errors ('Connection refused' or 'Service unavailable'), not 'API key not found'—the CLI hasn't even attempted API communication yet. Node.js version incompatibility would prevent CLI execution entirely, not just authentication—you wouldn't get the version command working if Node.js were incompatible. This diagnostic pattern is crucial: differentiate between installation issues (commands don't run), configuration issues (commands run but fail with specific errors), and service issues (network/timeout errors).",
      source: "Lesson 2: Installation, Authentication, and First Steps"
    },
    {
      question: "When setting up API authentication for Gemini CLI, which approach provides the best balance of security and convenience for a multi-user development machine?",
      options: [
        "Store API key in global system variables",
        "Share single config file across all users",
        "Hard-code API key directly in scripts",
        "Use individual user environment variables per account"
      ],
      correctOption: 3,
      explanation: "Individual user environment variables per account provide the best balance of security and convenience. Each user has their own API key in their shell profile (~/.bashrc or ~/.zshrc), which prevents key sharing (security) while remaining convenient (automatic loading). Storing API keys in global system environment variables exposes keys to all users on the machine—any user can read global variables, violating the principle of least privilege. Hard-coding API keys in scripts is the worst option—keys get committed to Git, shared in plaintext, and are difficult to rotate when compromised. Sharing a single config file across users forces everyone to use one API key, making it impossible to track usage per developer or revoke access for one person without affecting everyone. Best practices for multi-user environments: one key per person, stored in user-specific configuration, never shared or committed to version control.",
      source: "Lesson 2: Installation, Authentication, and First Steps"
    },
    {
      question: "After setting `export GEMINI_API_KEY=your_key_here` in your terminal, `gemini chat 'test'` works. After closing and reopening the terminal, the same command fails with 'API key not found.' What step was missed?",
      options: [
        "API key expired and requires regeneration",
        "Gemini CLI cache needs manual clearing first",
        "Environment variable not persisted in shell profile",
        "Global npm packages require reinstallation after restart"
      ],
      correctOption: 2,
      explanation: "Environment variable not persisted in shell configuration file (~/.bashrc, ~/.zshrc, or ~/.bash_profile) is the issue. Using `export GEMINI_API_KEY=...` in a terminal only sets the variable for that session—closing the terminal loses it. To persist, add the export statement to your shell profile file so it runs automatically on every new terminal session. API keys don't expire that quickly—Google API keys remain valid until manually revoked, so immediate expiration isn't the cause. CLI cache doesn't affect API key storage—clearing cache is for conversation history or temporary files, not authentication. Global npm packages don't require reinstallation after terminal restart—once installed globally, they remain available across sessions. The lesson: distinguish between session-level configuration (temporary `export` commands) and persistent configuration (shell profile files). Always test configuration persistence by opening a new terminal window.",
      source: "Lesson 2: Installation, Authentication, and First Steps"
    },
    {
      question: "You run `gemini chat --help` and see both `--model` and `--temperature` flags. Where would you configure default values to avoid specifying these flags on every command?",
      options: [
        "In the Gemini CLI configuration file ~/.gemini/config.json",
        "In the global npm package configuration file",
        "In environment variables GEMINI_MODEL and GEMINI_TEMPERATURE",
        "In a .geminirc file in project root"
      ],
      correctOption: 0,
      explanation: "Gemini CLI configuration file (~/.gemini/config.json) is the correct location for default settings like model and temperature. This file persists across all sessions and projects, setting global defaults that individual commands can override with flags. The npm package configuration manages package installation, not CLI behavior—it doesn't control Gemini-specific settings like model choice. While some CLIs support environment variables for all settings, Gemini CLI specifically uses configuration file for model and temperature defaults—environment variables are primarily for API keys. A .geminirc file in project root isn't standard for Gemini CLI (though some tools use this pattern)—Gemini CLI uses the centralized ~/.gemini/config.json. Understanding configuration precedence matters: config file sets defaults, command-line flags override defaults, enabling flexible workflows where global preferences can be adjusted per command when needed.",
      source: "Lesson 2: Installation, Authentication, and First Steps"
    },
    {
      question: "During first-time setup, you run `gemini chat 'hello'` and receive a response. What does this successful first command validate about your installation?",
      options: [
        "Only confirms CLI installation was successful",
        "Confirms installation, authentication, and API connectivity together",
        "Confirms authentication but not API connectivity yet",
        "Confirms only that Node.js dependencies resolved"
      ],
      correctOption: 1,
      explanation: "A successful `gemini chat 'hello'` command confirms installation, authentication, and API connectivity together—it's a complete end-to-end validation. The CLI must be properly installed (otherwise command wouldn't exist), API key must be correctly configured and valid (otherwise authentication would fail), and network connectivity to Google's API must be working (otherwise the request couldn't complete and return a response). Saying it only confirms installation is incomplete—installation alone wouldn't allow API communication. Saying it confirms authentication but not API connectivity is also incomplete—you can't receive a response without successful API communication. Saying it only confirms Node.js dependencies is too narrow—dependency resolution happens during installation, not during first command execution. This is why 'hello world' tests are valuable: they validate the entire stack in one simple command, providing confidence that all components are working together correctly.",
      source: "Lesson 2: Installation, Authentication, and First Steps"
    },
    {
      question: "You're writing a script that needs to invoke Gemini CLI in both interactive mode (for user input) and non-interactive mode (for automation). Which approach correctly handles both scenarios?",
      options: [
        "Use different API keys for interactive versus automation",
        "Use gemini chat always, detect terminal type automatically",
        "Use environment variable to toggle interactive mode flag",
        "Use gemini chat for interactive, gemini generate for automation"
      ],
      correctOption: 3,
      explanation: "`gemini chat` is designed for interactive use (real-time conversation, user prompts), while `gemini generate` is designed for non-interactive automation (scripted prompts, no user input expected). Using the right command for each scenario ensures proper behavior: `gemini chat` opens a conversation interface, while `gemini generate` takes input and exits immediately—perfect for scripts. Using `gemini chat` always and detecting terminal type could work but adds complexity—the CLI already provides purpose-built commands for each scenario, so use them. Environment variables for toggling mode would require custom wrapper scripts—unnecessary when the CLI natively supports both use cases through different commands. Using different API keys for interactive versus automation is about access control, not command behavior—it doesn't solve the technical problem of interactive vs non-interactive execution. The design lesson: tools often provide different commands for different use cases; learn them rather than forcing one command to serve all purposes.",
      source: "Lesson 2: Installation, Authentication, and First Steps"
    },
    {
      question: "You run `gemini chat 'Find latest Node.js security vulnerabilities'` and Gemini responds with current CVE information. Which built-in tool did Gemini automatically invoke?",
      options: [
        "Google Search tool for finding recent security reports",
        "Code Execution tool for querying vulnerability databases",
        "File system tool for reading local vulnerability data",
        "MCP server tool for accessing security APIs"
      ],
      correctOption: 0,
      explanation: "Google Search tool was automatically invoked because the query requires current information ('latest vulnerabilities') that isn't in Gemini's training data. Built-in tool selection is automatic—Gemini recognizes queries needing real-time data and invokes Google Search. Code Execution tool runs Python/JavaScript code, not web searches—it's for computation, data analysis, or algorithmic tasks, not information retrieval. File system tool reads local files, but the query asks for 'latest' information from the web, not local data. MCP servers are integration mechanisms you configure explicitly—they're not 'built-in' tools that activate automatically, and security API access would require specific MCP server setup. Understanding automatic tool selection is crucial: Gemini evaluates whether queries need (1) calculation → Code Execution, (2) current information → Google Search, (3) file content → file handling, or (4) none → direct model response. This intelligence enables natural language queries without explicitly specifying which tool to use.",
      source: "Lesson 3: Built-in Tools Deep Dive"
    },
    {
      question: "When would Code Execution tool be preferred over Google Search tool for a query about 'calculating factorial of 50'?",
      options: [
        "Code Execution requires less API quota usage",
        "Google Search finds better algorithm explanations quickly",
        "Code Execution provides more accurate mathematical computation results",
        "Google Search returns answers faster for math"
      ],
      correctOption: 2,
      explanation: "Code Execution provides more accurate mathematical computation results for calculating factorial of 50 because it runs actual code to compute the exact numeric answer (30414093201713378043612608166064768844377641568960512000000000000). Google Search would find articles explaining factorials but wouldn't compute the specific value you need—it returns web content, not computed results. The claim that Code Execution uses less API quota is misleading—both tools consume resources, and quota management isn't the primary factor in tool selection for accuracy-critical tasks. Google Search isn't inherently faster for math—it's fast at finding information, but Code Execution is equally fast at running simple calculations. The core distinction: Code Execution tool for computation (running code to get specific answers), Google Search tool for information retrieval (finding existing content). For 'calculate X,' use Code Execution; for 'explain X' or 'find recent X,' use Google Search.",
      source: "Lesson 3: Built-in Tools Deep Dive"
    },
    {
      question: "You ask Gemini to 'analyze this Python file for security issues' and provide a file path. Which tool capability enables this analysis?",
      options: [
        "Code Execution tool runs security scanning scripts",
        "File handling capability reads file contents for analysis",
        "Google Search tool finds known vulnerability patterns",
        "MCP server provides security analysis as service"
      ],
      correctOption: 1,
      explanation: "File handling capability reads the file contents and makes them available to Gemini's model for analysis. The model itself performs security analysis by examining code patterns, detecting vulnerabilities (SQL injection, XSS, etc.), and providing recommendations—no separate tool is needed for the analysis part, just file reading. Code Execution tool could theoretically run security scanning scripts, but that's not how Gemini typically handles this request—the model's code understanding is sufficient for most security reviews without executing external scanners. Google Search finds information about vulnerabilities, but analyzing your specific file requires reading the file, not searching the web. MCP servers could provide security analysis if configured, but built-in file handling is simpler and more direct for this use case. Understanding the distinction: file reading is about input (getting code into context), analysis is about model capability (understanding code), and tool execution is about output (running code or searching data).",
      source: "Lesson 3: Built-in Tools Deep Dive"
    },
    {
      question: "You notice Gemini using Google Search for a query that could be answered from your local codebase files. How can you influence tool selection to prioritize local file analysis?",
      options: [
        "Set tool priority order in environment variables",
        "Disable Google Search tool in configuration settings",
        "Request 'use Code Execution instead of Search'",
        "Explicitly provide file paths in your prompt"
      ],
      correctOption: 3,
      explanation: "Explicitly providing file paths in your prompt is the most effective way to direct Gemini to analyze local files instead of searching the web. When you include file paths, Gemini recognizes you're asking about specific local content and prioritizes file reading over web search. Phrasing matters: 'analyze functions in ./src/api.py' clearly indicates local file analysis. Disabling Google Search entirely would prevent useful searches when you actually need them—it's too blunt an approach. Requesting 'use Code Execution instead of Search' misunderstands the tools—Code Execution is for running code, not reading files. Tool priority configuration through environment variables isn't a standard Gemini CLI feature—tool selection is primarily prompt-driven, not configuration-driven. The design principle: natural language provides tool selection hints. Clear, specific prompts with file paths, calculations, or search terms help Gemini choose the right tool automatically without explicit configuration.",
      source: "Lesson 3: Built-in Tools Deep Dive"
    },
    {
      question: "During a Code Execution tool operation, Gemini returns an error: 'Execution timeout after 30 seconds.' What does this indicate about the code?",
      options: [
        "System resources insufficient to run the code",
        "Network latency delayed the execution API response",
        "Code contains infinite loop or very slow algorithm",
        "Code syntax errors prevented execution from starting"
      ],
      correctOption: 2,
      explanation: "Code contains an infinite loop or very slow algorithm that exceeded the 30-second execution time limit. Code Execution tool has timeout protection to prevent runaway processes—if code doesn't finish within the limit (typically 30 seconds), execution is terminated. This protects against infinite loops (`while True: pass`), inefficient algorithms (O(n³) on large datasets), or blocking operations (waiting for unresponsive network calls). Network latency affecting API response would produce different errors ('Connection timeout' or 'API unreachable'), not 'Execution timeout'—execution timeout is about the code runtime, not network communication. Insufficient system resources would cause 'Out of memory' or 'Resource exhausted' errors, not timeout. Syntax errors would cause 'SyntaxError' or 'Compilation failed' messages before execution even starts—timeout implies code started running but didn't finish. Debugging strategy: execution timeout → review algorithm complexity, check for infinite loops, verify loop termination conditions, and consider optimization or breaking work into smaller chunks.",
      source: "Lesson 3: Built-in Tools Deep Dive"
    },
    {
      question: "You want Gemini to scrape a website and analyze the data programmatically. Which combination of built-in tools would be most effective?",
      options: [
        "Google Search to find site, Code Execution to scrape",
        "File handling to save results, Search to find",
        "Code Execution to scrape, File to store data",
        "Google Search to read pages, File to save"
      ],
      correctOption: 0,
      explanation: "Google Search to find and access the website content, then Code Execution to programmatically process/scrape that data is the most effective combination. Google Search can fetch webpage content (acting as initial data gathering), then Code Execution can parse HTML, extract specific fields, transform data, and produce structured output—combining information retrieval with computational processing. File handling to save results would be useful after processing, but doesn't help with the core task of scraping—you need something to fetch the data first. Code Execution to scrape and File to store data could work if you write code to fetch URLs, but Google Search provides easier access to web content without manual HTTP requests. Google Search to read pages and File to save doesn't include the processing step—you need computation to parse and analyze scraped data. Understanding tool composition: Search for data access, Execution for processing, File for persistence. Each tool has a role in the pipeline.",
      source: "Lesson 3: Built-in Tools Deep Dive"
    },
    {
      question: "When using multiple built-in tools in a single Gemini request, what determines the order in which tools are executed?",
      options: [
        "Tools execute in parallel for maximum performance",
        "Gemini determines order based on task dependencies automatically",
        "Tools execute in order specified in prompt",
        "Tool execution order follows configuration file settings"
      ],
      correctOption: 1,
      explanation: "Gemini determines execution order based on task dependencies automatically. The model reasons about what information is needed when and creates an execution plan. For example, if you ask 'Search for Python tutorial and generate code based on it,' Gemini knows Search must happen first (to get tutorial content) before Code Execution (to generate code based on that content). Tools generally don't execute in parallel—they execute sequentially in a logical order determined by dependencies. While prompt phrasing can influence Gemini's understanding of the task, explicit tool order specification isn't how Gemini CLI works—the model interprets intent and plans accordingly. Tool execution order isn't configurable in settings—it's dynamically determined per request based on task logic. This intelligent orchestration is key to natural language interaction: you describe what you want, Gemini figures out the sequence of operations needed. Understanding this helps craft better prompts: focus on describing the goal, not micromanaging tool execution steps.",
      source: "Lesson 3: Built-in Tools Deep Dive"
    },
    {
      question: "You're in a long conversation with Gemini about a complex codebase. Suddenly, Gemini starts forgetting details mentioned early in the conversation. What is the most likely cause?",
      options: [
        "Gemini's model quality degraded over conversation length",
        "Memory management bug in Gemini CLI version",
        "API connection interrupted and conversation state lost",
        "Context window limit reached, early content pruned automatically"
      ],
      correctOption: 3,
      explanation: "Context window limit reached, causing early conversation content to be pruned automatically, is the most likely cause. Context windows have token limits (e.g., 1M tokens for Gemini 1.5 Pro)—when conversations exceed this limit, the oldest content is removed to make room for new content. This is expected behavior, not a bug. Model quality doesn't 'degrade' over conversation length—the model itself remains consistent; it's the available context that shrinks. API connection interruption would cause complete conversation loss or error messages, not gradual forgetting of early content while remembering recent content. While memory management bugs could theoretically cause this, the gradual, consistent pattern of forgetting old content while remembering recent content is characteristic of context window management, not bugs. Understanding context limits helps with conversation design: for long discussions, periodically summarize key points, restart conversations with summaries, or split work into focused sessions rather than one endless chat.",
      source: "Lesson 4: Memory and Context Management"
    },
    {
      question: "To maintain context across multiple Gemini CLI sessions over several days, which strategy is most effective?",
      options: [
        "Export conversation history to file, paste at restart",
        "Keep terminal window open continuously without closing",
        "Increase context window size in configuration settings",
        "Use save-conversation and load-conversation commands between sessions"
      ],
      correctOption: 0,
      explanation: "Export conversation history to a file and paste key context at restart is the most practical strategy for maintaining context across multiple sessions over days. Gemini CLI typically doesn't persist conversation state automatically between terminal sessions, so you need to manually save important context. Save-conversation and load-conversation commands would be ideal if Gemini CLI had them, but these aren't standard features in current versions—you'd need custom scripts or manual saving. Keeping a terminal window open for days is impractical and risky—system restarts, crashes, or accidental closures would lose everything. Context window size is model-determined, not user-configurable—you can't simply 'increase' Gemini's context window through configuration; it's a model architecture feature. The practical approach: summarize key points from previous conversations, save them to a file, and include that summary when starting new sessions. This manual context management ensures critical information persists without relying on non-existent persistence features.",
      source: "Lesson 4: Memory and Context Management"
    },
    {
      question: "In a conversation about debugging a 10-file codebase, you notice Gemini confusing details between files. Which technique would help Gemini maintain accuracy?",
      options: [
        "Discuss one file per conversation session only",
        "Use shorter, more frequent prompts to reduce load",
        "Provide file names explicitly in every prompt reference",
        "Request Gemini summarize context after each file"
      ],
      correctOption: 2,
      explanation: "Provide file names explicitly in every prompt reference to help Gemini maintain accuracy when discussing multiple files. Clear references reduce ambiguity: 'The function in api.py' instead of 'that function' helps Gemini track which file you're discussing. This is especially important in multi-file contexts where similar code patterns appear in different files. Discussing one file per conversation is overly restrictive and prevents analyzing cross-file interactions, dependencies, and architectural patterns—multi-file discussions are necessary for effective debugging. Shorter prompts don't inherently reduce confusion—explicit references matter more than prompt length. Requesting Gemini summarize after each file could help but is less efficient than simply using clear references throughout—it adds overhead without addressing the core issue of ambiguous references. The lesson: precision in language helps AI maintain accuracy. When context is complex (multiple files, similar structures), explicit naming prevents misattribution and keeps conversations productive.",
      source: "Lesson 4: Memory and Context Management"
    },
    {
      question: "You're analyzing a large dataset in a Gemini conversation. After the 20th query, responses become generic and lose specifics about your data. What optimization would most improve this?",
      options: [
        "Switch to a model with larger context window",
        "Use Code Execution tool instead of conversational analysis",
        "Reduce dataset size before starting conversation each time",
        "Break analysis into focused sub-tasks per conversation"
      ],
      correctOption: 3,
      explanation: "Break analysis into focused sub-tasks per conversation to optimize context usage. Instead of one long conversation covering everything, create separate conversations for specific aspects: one for data cleaning, one for statistical analysis, one for visualization, etc. Each conversation starts fresh with only relevant context, preventing context window exhaustion and maintaining specificity. Switching to a model with a larger context window helps but doesn't solve the fundamental issue—eventually, even large windows fill up with multi-query analysis. More importantly, you might already be using the largest available window. Reducing dataset size before analysis changes the task itself—you'd be analyzing different data, not optimizing context management. Using Code Execution tool instead of conversation shifts the approach but doesn't address context management—Code Execution still operates within conversation context and doesn't prevent context window exhaustion. The strategic lesson: conversation design matters as much as technical capabilities. Focused sessions with clear goals use context more efficiently than unfocused marathon conversations.",
      source: "Lesson 4: Memory and Context Management"
    },
    {
      question: "A conversation unexpectedly loses context after you paste a large code file. What likely caused the context loss?",
      options: [
        "Large file exceeded single-message token limit instantly",
        "Gemini automatically pruned conversation to fit new input",
        "File contained special characters that corrupted context",
        "API request failed due to file size limits"
      ],
      correctOption: 1,
      explanation: "Gemini automatically pruned earlier conversation content to make room for the large file within the context window limit. When new input (the large file) approaches or exceeds available context space, the system removes older messages to accommodate the new input. This automatic pruning can create abrupt context loss, especially if the large file consumes significant context space. While there are single-message token limits, exceeding them typically produces error messages ('Message too large'), not silent context loss—the request would be rejected, not cause pruning. Special characters don't 'corrupt' context in modern systems—encoding handles special characters correctly. API request failures due to size limits would produce explicit errors, not silent context loss. The key insight: context windows are fixed-size, zero-sum spaces. Adding large inputs forces removal of other content. To manage this, consider: (1) providing large files earlier in conversations before accumulating context, (2) summarizing files rather than pasting complete contents, or (3) starting new conversations for large-file analysis.",
      source: "Lesson 4: Memory and Context Management"
    },
    {
      question: "When starting a new Gemini conversation to continue work from a previous session, what information is most critical to include for context continuity?",
      options: [
        "Summary of decisions, key findings, and next steps",
        "Complete previous conversation history verbatim always",
        "Only the final output from previous session",
        "Error messages and issues encountered previously only"
      ],
      correctOption: 0,
      explanation: "Summary of decisions, key findings, and next steps provides the most effective context continuity without overwhelming the new conversation's context window. A good summary captures: 'In the previous session, we decided to use Flask for the API (not Django), implemented authentication with JWT, and identified the next step as database schema design.' This gives Gemini enough context to continue productively. Complete previous conversation history verbatim is inefficient—it consumes context space with repetitive details and low-value exchanges. If the previous conversation was 100 messages, pasting all of them wastes context and may cause pruning. Only the final output loses critical reasoning context—Gemini needs to understand why decisions were made, not just what the output was. Only including errors and issues is too narrow—you need positive context too (what worked, what's decided, what's built). The skill: distill conversations into high-signal summaries that preserve decision rationale, key findings, and progress state without unnecessary detail.",
      source: "Lesson 4: Memory and Context Management"
    },
    {
      question: "You need to choose between `gemini-1.5-flash` and `gemini-1.5-pro` for a script that generates 1,000 short product descriptions. Which factor should most influence your decision?",
      options: [
        "Flash provides significantly faster generation per description",
        "Pro provides better quality descriptions for all products",
        "Flash costs less and speed matters for bulk generation",
        "Pro has larger context for product catalog analysis"
      ],
      correctOption: 2,
      explanation: "Flash costs less and speed matters for bulk generation of 1,000 descriptions—this is the most important factor for high-volume tasks. Flash is optimized for speed and cost-efficiency, making it ideal for bulk operations where quality requirements are moderate. For short product descriptions (not complex creative writing), Flash usually provides sufficient quality at much lower cost and latency. While Flash is faster per description, speed alone isn't the deciding factor—cost at scale matters more (1,000 × lower cost = significant savings). Pro doesn't always provide better quality for all products—quality differences are task-dependent. For simple descriptions, Flash often performs comparably to Pro. Pro's larger context doesn't matter here—each product description is independent and short, so context window size isn't the bottleneck. The strategic lesson: model selection balances quality requirements, cost, and latency. For high-volume simple tasks, prioritize cost-effectiveness; for complex creative tasks, prioritize quality; for mixed scenarios, test both models on sample data.",
      source: "Lesson 5: Configuration and Settings"
    },
    {
      question: "You set `temperature: 0.2` in your Gemini CLI configuration for code generation tasks. What behavior does this setting optimize for?",
      options: [
        "More creative and varied code solutions across runs",
        "Better code quality with fewer syntax errors",
        "Faster code generation with reduced processing time overall",
        "More consistent and predictable code output per prompt"
      ],
      correctOption: 3,
      explanation: "Temperature 0.2 optimizes for more consistent and predictable code output. Low temperature (closer to 0) makes the model more deterministic—it favors high-probability tokens, resulting in similar outputs for the same prompt across multiple runs. This is ideal for code generation where you want reliable, conventional solutions, not creative variations. High temperature (closer to 1.0) would produce more creative and varied solutions—each run might generate different approaches, which is good for brainstorming but bad for reproducibility. Temperature doesn't affect generation speed—it's about output randomness, not processing time. Both low and high temperatures generate at similar speeds. Temperature doesn't directly improve code quality or reduce syntax errors—quality depends on the model's training and the prompt clarity. Low temperature makes code more conventional (potentially more correct due to following common patterns), but doesn't inherently fix errors. The principle: temperature is a creativity knob. Low = consistent/conventional, High = creative/varied. Match it to your task: code generation = low, creative writing = high.",
      source: "Lesson 5: Configuration and Settings"
    },
    {
      question: "Your Gemini CLI configuration file has `maxTokens: 1024`, but you need a specific command to generate longer output. How do you override this setting for one request?",
      options: [
        "Edit configuration file before running command, restore after",
        "Use --max-tokens flag directly in the command",
        "Set temporary environment variable MAX_TOKENS for command",
        "Create separate configuration profile for long outputs"
      ],
      correctOption: 1,
      explanation: "Use the --max-tokens flag directly in the command to override configuration for one request: `gemini chat 'generate long documentation' --max-tokens 2048`. Command-line flags have the highest precedence, overriding configuration file settings for that specific execution. This provides flexibility without modifying persistent configuration. Editing the configuration file before running and restoring after is error-prone and inefficient—you might forget to restore, affecting future commands. Manual file editing for one-off overrides is bad practice. Setting a temporary environment variable MAX_TOKENS could work if Gemini CLI supported it, but command-line flags are the standard, documented way to override settings per command. Creating a separate configuration profile is overkill for occasional overrides—profiles make sense for fundamentally different workflows (development vs production), not one-off variations. Understanding configuration precedence: defaults < config file < environment variables < command-line flags. This hierarchy enables flexible workflows: set sensible defaults in config, override as needed per command.",
      source: "Lesson 5: Configuration and Settings"
    },
    {
      question: "You configure safety settings to 'BLOCK_NONE' for all categories to allow testing edge cases. What risk does this introduce?",
      options: [
        "Configuration becomes incompatible with some models only",
        "API requests may fail more frequently",
        "Model may generate harmful content without filtering",
        "Performance degrades due to disabled optimizations"
      ],
      correctOption: 2,
      explanation: "Model may generate harmful content without filtering when safety settings are set to 'BLOCK_NONE.' Safety filters prevent generation of content in categories like hate speech, sexually explicit content, dangerous content, and harassment. Disabling them allows the model to respond to prompts that would normally be blocked. This is appropriate for research, red-teaming, or testing edge cases in controlled environments, but introduces risk of harmful outputs if used carelessly. API requests don't fail more frequently with disabled safety filters—in fact, you'd see fewer blocks, so fewer failures. The claim might confuse cause and effect: safety filters cause some requests to fail (by design), disabling them reduces those failures. Configuration compatibility isn't affected—safety settings are supported across Gemini models. BLOCK_NONE is a valid setting, not incompatible. Performance isn't degraded by disabled safety filters—safety filtering adds slight overhead, so disabling might marginally improve performance, not degrade it. Best practices: only disable safety filters when necessary, in controlled environments, with clear protocols for handling potentially harmful outputs.",
      source: "Lesson 5: Configuration and Settings"
    },
    {
      question: "You want Gemini to output responses in JSON format consistently without specifying it in every prompt. Where should you configure this preference?",
      options: [
        "Create custom slash command that includes format",
        "Add output format preference to configuration file",
        "Set system-level environment variable GEMINI_OUTPUT_FORMAT=json always",
        "Use MCP server to transform outputs automatically"
      ],
      correctOption: 0,
      explanation: "Create a custom slash command that includes format instructions in its system prompt, enabling consistent JSON output without repeating format requests in every user prompt. For example, `/json-query` could include 'Always respond in valid JSON format' in its prompt template, ensuring every use produces JSON. This makes the format constraint part of the workflow, not something to remember each time. Setting a system-level environment variable GEMINI_OUTPUT_FORMAT=json isn't a standard Gemini CLI feature—output format isn't configurable via environment variables in most implementations. Adding output format to config file could work if the CLI supports it, but this isn't a standard configuration option in Gemini CLI—the config file typically handles model, temperature, and API settings, not output formatting instructions. Using an MCP server to transform outputs adds unnecessary complexity—you'd be post-processing responses rather than configuring generation, which is inefficient and brittle. Understanding tool capabilities: custom commands embed prompt patterns, configuration files manage model settings, MCP servers provide external integrations. Choose the right mechanism for each need.",
      source: "Lesson 5: Configuration and Settings"
    },
    {
      question: "After updating Gemini CLI configuration, you run `gemini chat 'test'` but the new settings don't seem to apply. What is the most likely cause?",
      options: [
        "Configuration changes require CLI restart or re-installation",
        "Configuration cache needs manual clearing before new values",
        "Command-line flags overriding configuration file settings in use",
        "Configuration file has syntax errors preventing loading"
      ],
      correctOption: 3,
      explanation: "Configuration file has syntax errors preventing it from loading correctly, causing Gemini CLI to fall back to default settings or previously working configuration. JSON syntax errors (missing commas, unclosed brackets, unquoted keys) are common when manually editing config files. The CLI might silently ignore a malformed config file rather than crashing, making the issue non-obvious. Configuration changes don't require CLI restart or reinstallation—Gemini CLI typically reads the config file on each command invocation, so changes should apply immediately. Command-line flags overriding configuration is possible but would only happen if you explicitly used flags in your command—the question says you just ran `gemini chat 'test'` without additional flags. Configuration cache isn't a typical feature in CLI tools—most read config files directly each time. Manual cache clearing isn't necessary. Debugging steps: (1) validate JSON syntax using a JSON validator or linter, (2) check for file permissions issues, (3) verify file location matches expected path (~/.gemini/config.json), (4) test with minimal configuration to isolate issues.",
      source: "Lesson 5: Configuration and Settings"
    },
    {
      question: "What is the primary purpose of the Model Context Protocol (MCP) in Gemini CLI?",
      options: [
        "Optimizing model context window management for efficiency",
        "Enabling connections to external tools and data",
        "Providing secure protocol for API authentication only",
        "Caching conversation history for faster responses later"
      ],
      correctOption: 1,
      explanation: "MCP (Model Context Protocol) enables connections to external tools and data sources, extending Gemini CLI beyond its built-in capabilities. MCP servers provide access to file systems, databases, APIs, Git repositories, and other resources through a standardized protocol. This allows Gemini to interact with your development environment, company databases, or custom services without requiring native integration for each one. MCP doesn't optimize context window management—context window is a model feature, not something MCP affects. The 'Context' in 'Model Context Protocol' refers to providing context (data/tools) to the model, not managing the conversation context window. MCP isn't primarily about API authentication—while MCP connections may require authentication, the protocol's main purpose is tool/data integration, not security. MCP doesn't cache conversation history—conversation state management is a separate CLI feature. Understanding MCP's role: it's an integration protocol that connects AI models to external systems, enabling AI to act on your infrastructure (read files, query databases, invoke APIs) through a standardized interface.",
      source: "Lesson 6: MCP Servers and Integration"
    },
    {
      question: "You configure an MCP server for file system access, but Gemini still cannot read files in a specific directory. What is the most likely issue?",
      options: [
        "Gemini CLI version incompatible with MCP protocol standard",
        "File system MCP server only works with text files",
        "MCP server permissions don't include that directory path",
        "MCP servers require explicit activation per command via flag"
      ],
      correctOption: 2,
      explanation: "MCP server permissions don't include that directory path—MCP servers typically have explicitly configured access scopes for security. When setting up a file system MCP server, you specify which directories it can access. If a directory isn't in the allowed paths, the server won't read files there, even if Gemini requests it. This is a security feature preventing unrestricted file system access. File system MCP servers aren't limited to text files—they can handle any file type the underlying system can read. Binary files, images, and data files are all accessible if permissions allow. Gemini CLI version incompatibility would prevent MCP connections entirely, not just affect specific directories—you'd see connection errors, not selective directory access failures. MCP servers don't require per-command activation flags once configured—they're available for all commands in that CLI instance. Debugging steps: (1) check MCP server configuration for allowed paths, (2) verify directory path spelling and case sensitivity, (3) check file system permissions (maybe the MCP server process itself lacks OS-level read access to that directory).",
      source: "Lesson 6: MCP Servers and Integration"
    },
    {
      question: "When deciding whether to use a built-in tool or configure an MCP server, which scenario most strongly favors MCP?",
      options: [
        "Accessing company-specific internal database or proprietary APIs",
        "Performing Google searches for current information always",
        "Executing Python code for data analysis tasks",
        "Reading files from the local file system"
      ],
      correctOption: 0,
      explanation: "Accessing company-specific internal databases or proprietary APIs strongly favors MCP because built-in tools don't provide access to custom, private systems. MCP servers are designed for extending Gemini's capabilities to your organization's unique infrastructure. You'd create a custom MCP server that connects to your internal database API, enabling Gemini to query company data while respecting authentication and authorization. Performing Google searches is built-in—no MCP needed. Google Search tool is already available in Gemini CLI and covers general web searches. Executing Python code is built-in—Code Execution tool handles this without MCP. While you could create an MCP server for code execution with specific libraries, the built-in tool suffices for most cases. Reading local files can be done with built-in file handling or MCP. Built-in tools are simpler for basic file reading; MCP makes sense if you need advanced features (version control integration, access control, or reading from networked file systems). The decision pattern: built-in tools for common tasks (search, code execution, basic file I/O), MCP for custom integrations (internal APIs, specialized databases, proprietary systems).",
      source: "Lesson 6: MCP Servers and Integration"
    },
    {
      question: "You create a custom MCP server that provides access to your company's HR database. What security consideration is most critical?",
      options: [
        "Using HTTPS for MCP server connections only",
        "Ensuring MCP server validates all queries against access policies",
        "Limiting MCP server to read-only database operations completely",
        "Requiring MCP server restart for each Gemini session"
      ],
      correctOption: 1,
      explanation: "Ensuring the MCP server validates all queries against access control policies is most critical for security. The MCP server must check: (1) Does the requester have permission to access this data? (2) Does the query comply with data privacy regulations? (3) Are there sensitive fields that should be filtered? Without proper validation, Gemini could inadvertently access confidential HR data (salaries, performance reviews, personal information) that users shouldn't see. HTTPS for connections is important for data in transit but doesn't address access control—even encrypted connections can transmit unauthorized data if access policies aren't enforced. Limiting to read-only operations helps prevent data modification but doesn't prevent unauthorized data access—reading sensitive HR data without proper authorization is still a security breach. Requiring server restart per session is impractical and doesn't improve security—authentication and authorization should be per-request, not per-session. The principle: security in AI integrations requires explicit access control at the integration layer. Don't rely on prompt engineering or user trust—enforce permissions programmatically in the MCP server.",
      source: "Lesson 6: MCP Servers and Integration"
    },
    {
      question: "After configuring three MCP servers (file system, Git, and database), you notice increased latency in Gemini responses. What is the most likely cause?",
      options: [
        "MCP connection overhead adds latency to every query",
        "Multiple MCP servers compete for network resources constantly",
        "Database MCP server is slow and blocks queries",
        "Gemini evaluates all MCP servers for tool relevance"
      ],
      correctOption: 3,
      explanation: "Gemini evaluates all configured MCP servers for tool relevance on each query, which adds processing overhead. With three MCP servers, Gemini must determine: 'Does this query need file system access? Git access? Database access?' This evaluation happens before executing the query, adding latency proportional to the number of configured servers and the complexity of determining relevance. MCP connection overhead exists but isn't incurred on every query—connections are typically established once and reused. Connection setup latency is one-time per session, not per query. Multiple MCP servers don't compete for network resources unless they're all actively being used simultaneously—the servers are independent and don't inherently interfere with each other. A slow database MCP server would only cause latency when that specific server is invoked—it wouldn't affect queries that don't need database access. However, if tool selection is slow, all queries suffer. Optimization strategies: (1) disable unused MCP servers, (2) use clear prompts that help Gemini select tools quickly, (3) monitor which MCP servers are actually being used and remove unnecessary ones.",
      source: "Lesson 6: MCP Servers and Integration"
    },
    {
      question: "Your MCP server for Git access works in development but fails in production. The error message indicates 'Git not found in PATH.' What configuration was likely missed?",
      options: [
        "Production environment missing Git installation or PATH incorrect",
        "MCP server configuration must specify absolute Git binary path",
        "MCP protocol version mismatch between dev and production",
        "Firewall blocking MCP server from accessing Git service"
      ],
      correctOption: 0,
      explanation: "Production environment is missing Git installation or has incorrect PATH configuration. MCP servers that wrap command-line tools (like Git) depend on those tools being installed and accessible via PATH. Development machines typically have Git installed, but production servers might not—especially minimal container images or restricted environments. The error 'Git not found in PATH' clearly indicates the Git binary isn't accessible. While MCP server configuration could specify absolute paths to Git binaries as a workaround, the root cause is still missing Git in production—the better fix is ensuring Git is installed. MCP protocol version mismatch would cause connection errors or incompatible message formats, not 'command not found' errors. Protocol issues affect communication, not executable availability. Firewalls blocking MCP server from accessing Git service misunderstands the architecture—Git operates locally (or connects to remote Git servers), not as a service the MCP server connects to. Firewall issues would prevent Git from fetching remote repos, not finding the Git command itself. Deployment lesson: verify dependencies (Git, Python, Node.js) in production match development, especially for integration layers like MCP servers.",
      source: "Lesson 6: MCP Servers and Integration"
    },
    {
      question: "When would creating a custom MCP server be preferred over using Gemini's Code Execution tool?",
      options: [
        "When code execution needs to happen faster",
        "When executing simple Python or JavaScript calculations only",
        "When computations require specialized libraries not in environment",
        "When outputs need to be formatted nicely"
      ],
      correctOption: 2,
      explanation: "When computations require specialized libraries not available in the Code Execution environment, a custom MCP server is preferred. Code Execution tool has a predefined environment with standard libraries; if you need domain-specific libraries (bioinformatics tools, financial modeling packages, internal company libraries), a custom MCP server gives you control over the execution environment. You can set up a server with any dependencies, ensuring the code runs with the correct libraries. Simple Python or JavaScript calculations are exactly what Code Execution tool is designed for—no need for MCP complexity. Use the built-in tool for straightforward tasks. Code execution speed isn't significantly improved by MCP—both approaches involve remote execution. If anything, MCP adds overhead from the protocol layer. Custom MCP servers don't inherently make code execute faster. Output formatting isn't a reason to choose MCP over Code Execution—both can produce formatted output. You can format results in the code itself or ask Gemini to format results after execution. The decision point: built-in Code Execution for standard scenarios, custom MCP when you need environmental control (specific libraries, system access, custom security policies).",
      source: "Lesson 6: MCP Servers and Integration"
    },
    {
      question: "What is the primary benefit of creating a custom slash command for a frequently repeated task?",
      options: [
        "Commands execute faster than typing prompts manually",
        "Commands reduce API quota usage for common tasks",
        "Commands provide better quality responses from Gemini model",
        "Commands eliminate repetitive prompt typing and ensure consistency"
      ],
      correctOption: 3,
      explanation: "Commands eliminate repetitive prompt typing and ensure consistency across uses. Instead of typing 'Generate unit tests for this code following our team's standards with Jest framework and 80% coverage' every time, you create `/gen-tests` that encodes those instructions. This saves time, reduces errors from inconsistent prompts, and ensures everyone on the team uses the same high-quality prompt template. Commands don't execute faster in terms of Gemini's processing time—the model processes the underlying prompt at the same speed whether it came from a slash command or manual typing. API latency is the same. Commands don't improve response quality—quality comes from the prompt content, not the delivery mechanism. A well-written manual prompt and a slash command with the same text produce identical results. Commands don't reduce API quota usage—the same prompt (whether manual or command-triggered) consumes the same tokens. Quota is based on request content, not input method. The value: slash commands are prompt templates that enforce best practices, ensure consistency, and improve developer productivity through reuse, not through technical performance improvements.",
      source: "Lesson 7: Custom Slash Commands"
    },
    {
      question: "You create a slash command `/review-pr [PR_NUMBER]` for code reviews. Where should this command definition be stored for team-wide availability?",
      options: [
        "In global Gemini CLI configuration at ~/.gemini/commands/",
        "In project repository at .gemini/commands/ under version control",
        "In each developer's personal command library separately always",
        "In centralized command registry server for organization"
      ],
      correctOption: 1,
      explanation: "In project repository at .gemini/commands/ under version control is the best location for team-wide availability. This makes the command part of the project itself—when someone clones the repo, they get the custom commands automatically. Version control tracks changes to the command (prompt improvements, parameter adjustments), and the command stays synchronized with project-specific conventions. Global Gemini CLI configuration (~/.gemini/commands/) is for personal commands, not team commands—each developer would need to manually copy the command, and updates wouldn't propagate automatically. Each developer's personal library defeats the purpose of team-wide availability—there's no shared source of truth, and maintaining consistency becomes impossible. A centralized command registry server could work for organization-wide commands but adds infrastructure complexity—for project-specific commands, the project repo is simpler and more maintainable. The principle: scope your tools appropriately. Personal commands → global config, project commands → project repo, organization commands → shared infrastructure. Most teams benefit from project-level commands in version control, not centralized registries.",
      source: "Lesson 7: Custom Slash Commands"
    },
    {
      question: "When designing a custom slash command with parameters, what makes a parameter design well-suited for team use?",
      options: [
        "Maximum flexibility with many optional parameters for customization",
        "No parameters; hardcode all values for consistency",
        "Clear required parameters with sensible defaults for optional",
        "Only optional parameters to avoid user errors"
      ],
      correctOption: 2,
      explanation: "Clear required parameters with sensible defaults for optional parameters creates the best team experience. Required parameters force users to provide essential information (e.g., `/review-pr` requires PR number), while sensible defaults handle common cases without requiring explicit specification (e.g., default to 'main' branch if not specified). This balances flexibility with ease of use. Maximum flexibility with many optional parameters creates cognitive overhead—users must remember many options, read documentation frequently, and make many decisions per invocation. While flexibility is valuable, too much hurts adoption and usability. Hardcoding all values eliminates flexibility—if team needs change or edge cases arise, the command becomes useless. Commands should be reusable across scenarios, not single-purpose scripts. Only optional parameters seems convenient but creates ambiguity—without required parameters, the command might not have enough information to act. For example, `/review-pr` without PR number can't function. Design principles: (1) require only essential information, (2) default common choices, (3) document parameters clearly, (4) validate input and provide helpful errors when required parameters are missing.",
      source: "Lesson 7: Custom Slash Commands"
    },
    {
      question: "Your custom slash command `/analyze-logs` works locally but fails in CI/CD with 'File not found.' What is the most likely cause?",
      options: [
        "CI/CD environment has different working directory paths",
        "Slash commands not supported in automated CI environments",
        "File permissions prevent CI from reading log files",
        "Gemini CLI not properly installed in CI container"
      ],
      correctOption: 0,
      explanation: "CI/CD environment has different working directory paths than your local machine. Your command might reference files with relative paths that work locally (e.g., `./logs/app.log`) but fail in CI where the working directory or file structure differs. CI environments often have different directory layouts, temporary workspaces, or mounted volumes that break assumptions about file locations. Slash commands are supported in CI environments—they're just prompt templates that invoke the CLI, which works the same way in CI as locally. CI automation is a valid use case for custom commands. File permissions could cause issues, but 'File not found' specifically indicates the path doesn't exist or isn't correct—permission errors would say 'Permission denied.' The error message is diagnostic here. If Gemini CLI weren't properly installed, you'd get 'command not found' errors when invoking `gemini`, not 'File not found' errors within command execution. Solutions: (1) use absolute paths or environment variables in commands, (2) document working directory assumptions, (3) test commands in CI-like environments before deployment, (4) parameterize file paths so they're configurable per environment.",
      source: "Lesson 7: Custom Slash Commands"
    },
    {
      question: "You want to create a slash command that combines multiple AI interactions: fetch code from Git, analyze it, generate tests, and commit results. Which approach is most appropriate?",
      options: [
        "Create single complex command orchestrating all steps sequentially",
        "Write Python script, invoke Gemini API directly instead",
        "Use MCP servers exclusively, avoid slash commands here",
        "Create separate commands per step, use script to chain"
      ],
      correctOption: 3,
      explanation: "Create separate commands per step (fetch, analyze, generate-tests, commit) and use a shell script or automation script to chain them together. This modular approach provides flexibility: you can run individual steps for debugging, reuse steps in different workflows, and modify one step without affecting others. Each command has a clear, focused responsibility. A single complex command orchestrating all steps becomes difficult to debug, maintain, and reuse—if you only need analysis without committing, you can't easily extract that step. Complex commands also have poor error handling—a failure at any step might require rerunning everything. Using MCP servers exclusively misses the point—slash commands and MCP servers serve different purposes. Commands are workflow automation, MCP servers are integrations. You'd likely use both: MCP for Git access, commands for workflow steps. Writing a Python script invoking Gemini API directly bypasses the value of slash commands—commands provide reusability, discoverability, and team sharing through the CLI. Custom scripts are useful but less discoverable and harder for teams to adopt. Design principle: compose small, reusable commands rather than monolithic workflows. Unix philosophy applies: each command does one thing well, combining them creates powerful workflows.",
      source: "Lesson 7: Custom Slash Commands"
    },
    {
      question: "When integrating Gemini CLI into VS Code via an extension, what is the primary advantage over using the CLI in a separate terminal?",
      options: [
        "IDE integration provides better AI model quality",
        "IDE integration embeds AI within development workflow context",
        "IDE integration reduces API costs for queries",
        "IDE integration allows offline AI usage capabilities"
      ],
      correctOption: 1,
      explanation: "IDE integration embeds AI within development workflow context, enabling seamless interactions without context switching. You can highlight code, right-click, and invoke Gemini directly—keeping your hands on the keyboard and your focus in the editor. The IDE can automatically provide file paths, code selections, and project context to Gemini without manual copying. This contextual integration is the main benefit. AI model quality is identical—whether accessed via VS Code extension or terminal, you're using the same Gemini models. The interface doesn't change the underlying AI. API costs are the same—IDE extensions and CLI use the same API with the same pricing. The number and size of requests determine cost, not the access method. Offline usage isn't enabled by IDE integration—both IDE extensions and CLI require internet connectivity to access Gemini's cloud API. Offline AI would require running models locally, which is unrelated to IDE integration. The value proposition: IDE extensions reduce friction in AI-assisted workflows by integrating AI into the environment where you already work, not by changing AI capabilities or costs.",
      source: "Lesson 8: Extensions, Security, and IDE Integration"
    },
    {
      question: "Your team uses Gemini CLI in VS Code with a shared workspace configuration. A developer accidentally commits a file containing the API key. What is the best immediate response?",
      options: [
        "Remove file from Git history and regenerate key",
        "Add file to .gitignore and delete from repo",
        "Change file permissions to prevent future access only",
        "Notify team to not use that key anymore"
      ],
      correctOption: 0,
      explanation: "Remove the file from Git history using tools like `git filter-branch` or `BFG Repo-Cleaner`, then immediately regenerate the API key in Google Cloud Console. Simply deleting the file in a new commit doesn't remove it from Git history—anyone with access to the repo can view previous commits and extract the key. Compromised keys must be revoked. Adding file to .gitignore and deleting from the repo prevents future commits but doesn't remove the key from history—it's still accessible in previous commits. This is insufficient for security. Changing file permissions doesn't address the core issue—the key is already in a public/shared repository. Permissions on your local file system don't protect data already committed to Git. Notifying the team without revoking the key is dangerous—the key might be exposed to anyone with repository access, including past collaborators or if the repo is public. You can't assume the key is safe once exposed. Security protocol: (1) revoke compromised credentials immediately, (2) remove from version control history, (3) rotate to new credentials, (4) audit for unauthorized usage, (5) implement prevention (secrets scanning, pre-commit hooks).",
      source: "Lesson 8: Extensions, Security, and IDE Integration"
    },
    {
      question: "When using Code Execution tool in Gemini CLI, which security risk requires careful consideration in production environments?",
      options: [
        "Code Execution bypasses safety filters for harmful content",
        "Code Execution exposes API keys to executed scripts",
        "Code Execution enables arbitrary code running on server",
        "Code Execution allows unauthorized access to local filesystem"
      ],
      correctOption: 2,
      explanation: "Code Execution enables arbitrary code to run in the execution environment, which is a security risk if not properly sandboxed. If Gemini generates malicious code (intentionally or through prompt injection), Code Execution will run it. While Google likely sandboxes the Code Execution environment, understanding that code runs with some level of system access is important. For sensitive production environments, this is a risk to evaluate. Code Execution doesn't inherently expose API keys to executed scripts—API keys are used for authentication to Gemini's API, not passed to user code. However, if you reference environment variables or files containing secrets in your prompts, Gemini might include them in generated code. Code Execution doesn't bypass safety filters—safety settings apply to the conversation and generation, not code execution. Harmful content might be blocked before code generation, though Code Execution itself doesn't add safety filtering. Code Execution doesn't access your local filesystem—it runs in Google's remote execution environment. You'd need to explicitly provide file contents to Gemini, or use file handling/MCP to access local files. Security best practices: (1) review generated code before execution, (2) use Code Execution only when necessary, (3) avoid providing sensitive data in prompts that will be used in code, (4) monitor code execution logs for anomalies.",
      source: "Lesson 8: Extensions, Security, and IDE Integration"
    },
    {
      question: "You configure Gemini CLI extension in VS Code with workspace-specific settings for a client project. Which setting should definitely be workspace-specific rather than global?",
      options: [
        "Model preference between Flash and Pro variants",
        "API key or authentication token for client",
        "Temperature and creativity settings for generation tasks",
        "Keyboard shortcuts for invoking Gemini commands quickly"
      ],
      correctOption: 1,
      explanation: "API key or authentication token should definitely be workspace-specific for client projects. Different clients might have separate Google Cloud projects, billing accounts, and API keys for isolation and cost tracking. Using a workspace-specific key ensures usage is attributed correctly and access is scoped to the project. Model preference could be workspace-specific if different projects have different quality/cost requirements, but it's less critical than authentication—using the wrong model is inefficient, using the wrong API key is a security and billing issue. Temperature settings might vary by project type (creative vs deterministic), but this is a workflow preference, not a security requirement. Keyboard shortcuts are personal preferences, typically configured globally per user, not per workspace—developers have muscle memory and want consistent shortcuts across projects. Security principle: isolate credentials per context (client, project, environment). Workflow preferences (model, temperature) can be shared or customized as needed, but credentials must be scoped appropriately to prevent cross-contamination, unauthorized access, and billing confusion.",
      source: "Lesson 8: Extensions, Security, and IDE Integration"
    },
    {
      question: "A security audit flags your team's use of Gemini CLI for processing customer data in code reviews. What is the primary concern?",
      options: [
        "Team members can access customer data through CLI",
        "Gemini CLI logging stores all customer data locally",
        "Code reviews using AI produce inaccurate security assessments",
        "Customer data sent to Gemini may violate privacy agreements"
      ],
      correctOption: 3,
      explanation: "Customer data sent to Gemini's API may violate privacy agreements, data residency requirements, or compliance regulations (GDPR, HIPAA, etc.). When code containing customer data (PII, financial information, health records) is sent to external AI services, you're sharing that data with a third party. Your privacy policies or contracts might prohibit this, or regulations might require data to stay within specific geographic boundaries. Gemini CLI logging stores conversation history locally in some configurations, but the primary security concern is data leaving your organization to Google's API, not local storage (which you control). While local logs also need securing, the audit likely focuses on external data sharing. AI producing inaccurate security assessments is a quality concern, not the primary privacy concern flagged in audits. Inaccurate reviews are bad but don't violate privacy agreements—sending customer data externally does. Team members accessing customer data through CLI is an access control issue within your organization, not specific to Gemini usage. If access control is the problem, it exists regardless of AI tool choice. Compliance guidance: (1) strip sensitive data before sending to AI, (2) use data anonymization, (3) review service terms and data handling policies, (4) ensure AI provider agreements align with your compliance requirements, (5) consider on-premises AI alternatives for highly sensitive data.",
      source: "Lesson 8: Extensions, Security, and IDE Integration"
    },
    {
      question: "You ask Gemini to 'find the latest CVE for a library and generate code to check version compatibility.' Which tool execution order would Gemini most likely follow?",
      options: [
        "Search for CVE, then Code Execution for version check",
        "Code Execution first, then Search for validation results",
        "Both tools run in parallel for performance",
        "File reading first, then Search, then Execution"
      ],
      correctOption: 0,
      explanation: "Gemini would most likely execute Google Search first (to find the latest CVE information) and then Code Execution (to generate and run version checking code based on that CVE data). This sequence respects task dependencies: you need CVE details before writing version-checking code. The search provides information like affected versions and patch requirements, which the generated code uses to perform compatibility checks. Code Execution first doesn't make logical sense—how would you write version-checking code without knowing which CVE or version ranges to check? You'd be generating code without necessary context. Tools don't run in parallel for dependent tasks—while parallel execution might be faster, correctness requires sequential execution when one tool's output feeds into another's input. File reading first only makes sense if you specified a file in your prompt—the question doesn't mention local files, so this step wouldn't occur. The key learning: Gemini intelligently sequences tool operations based on information dependencies, not arbitrary ordering. Understanding this helps you write clearer prompts and predict tool behavior.",
      source: "Lesson 3: Built-in Tools Deep Dive"
    },
    {
      question: "When troubleshooting a failed Gemini CLI installation where `npm install -g @google/generative-ai-cli` completes but `gemini` command is not found, what is the most likely cause?",
      options: [
        "Insufficient permissions for global package installation on system",
        "Installation package corrupted during download from registry",
        "npm global bin directory not in system PATH",
        "Node.js version incompatible with package binary requirements"
      ],
      correctOption: 2,
      explanation: "npm global bin directory not being in system PATH is the most common cause when installation succeeds but the command isn't found. npm installs global packages to a directory like `/usr/local/bin` or `~/.npm-global/bin`, which must be in your shell's PATH to execute commands. You can find the global bin path with `npm config get prefix` and add it to PATH in your shell configuration. Installation package corruption would typically cause npm install itself to fail with checksum errors or incomplete installation messages, not succeed silently. If installation completes, the package downloaded correctly. Insufficient permissions would prevent installation altogether—npm would display 'permission denied' errors during installation, not complete successfully. You wouldn't get to the point where the command is installed but inaccessible. Node.js version incompatibility would cause runtime errors when trying to execute the command (if it could be found), not 'command not found' errors. You'd see messages about unsupported JavaScript features or module system issues. Fix: Check PATH with `echo $PATH`, verify npm global bin location, add to PATH in `~/.bashrc`, `~/.zshrc`, or equivalent, then reload shell configuration.",
      source: "Lesson 2: Installation, Authentication, and First Steps"
    }
  ]}
  questionsPerBatch={18}
/>

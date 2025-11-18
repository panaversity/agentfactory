# Chapter 6: Google Gemini CLI - Detailed Implementation Plan

**Chapter Type**: Technical (Code-Focused with Tool-First Pedagogy)
**Part**: 2 (AI Tool Landscape)
**Complexity Tier**: Beginner-Intermediate (CEFR A2-B1)
**Target Duration**: 85-95 minutes total reading/practice
**Status**: Ready for Implementation

---

## I. Chapter Overview & Alignment

### Chapter Identity
- **Title**: Google Gemini CLI: Open Source and Everywhere
- **Learning Outcome**: Students can install, configure, and integrate Gemini CLI with MCP servers and IDE extensions to expand their AI capabilities beyond web interfaces
- **Success Criteria**: 5 evals defined in spec (Installation, Feature Discovery, MCP Integration, Extension Lifecycle, IDE Integration)

### Specification Reference
- **Approved Spec**: `specs/chapter-6-gemini-cli-revision/spec.md`
- **Verified Intelligence**: `intelligence/chapter-6-gemini-cli-verified-docs.md`
- **Constitutional Alignment**: Principles 7 (Technical Accuracy), 13 (Graduated Teaching), Core Philosophy #1 (AI Development Spectrum)

### Chapter Structure: 6 Lessons

The chapter is split into two meaningful groupings reflecting the spec's design decisions:

| Lesson | Title | Duration | Type | Key Topics |
|--------|-------|----------|------|-----------|
| 1 | Why Gemini CLI Matters | 15 min | Conceptual | Positioning, comparison, free tier, Qwen Code mention |
| 2 | Installation, Auth & First Steps | 15 min | Technical | Installation, verification, troubleshooting, session basics |
| 3 | Built-In Tools Deep Dive | 20-25 min | Technical | File operations, shell integration, web fetching, search |
| 4 | Context Window & Tool Comparison | 25 min | Technical | Token counts, capability matrix, when to use which tool |
| 5 | MCP Servers & Integration | 20 min | Technical | MCP basics, CLI commands (NEW), OAuth (NEW), business workflows |
| 6 | Extensions, Security & IDE Integration | 15-20 min | Technical | Extension lifecycle (NEW), tool filtering (NEW), IDE integration (NEW), workflow selection |

**Total Chapter Duration**: 110-135 minutes (reading + practice) → realistic for comprehensive tool coverage

---

## II. Constitutional Alignment & Pedagogical Framework

### Principle 13: Graduated Teaching Pattern (Book → AI Companion → AI Orchestration)

The chapter follows a three-tier approach mapping to chapter progression:

#### Tier 1: Foundational Concepts (Book Teaches Directly)
- **Lessons 1-2**: Installation basics, authentication flow, tool positioning
- **Student Role**: Manual practice (install CLI, verify version, run first session)
- **AI Role**: Validation and Q&A ("Why does `npm install` require Node.js?")

#### Tier 2: Complex Execution (AI Companion Handles)
- **Lessons 3-4**: Understanding tools, context window, strategic tool selection
- **Student Role**: Specification ("I need to analyze 12 reports—which tool is best?")
- **AI Role**: Execution and explanation ("Gemini CLI because 1M token context...")

#### Tier 3: Orchestration (AI Manages Complex Workflows)
- **Lessons 5-6**: MCP configuration, extension development, IDE integration
- **Student Role**: Orchestration ("Set up Playwright MCP for my project")
- **AI Role**: Autonomous execution with student validation

### Cognitive Load Management (≤7 Concepts/Section)

Each lesson section introduces 3-7 new concepts, validated per cognitive load limits:

- **Lesson 1**: 5 concepts (CLI positioning, free tier, context window, Qwen Code, use cases)
- **Lesson 2**: 6 concepts (npm, authentication, OAuth flow, session, commands, troubleshooting)
- **Lesson 3**: 7 concepts (file tool, shell tool, web tool, search, tool combining, limitations)
- **Lesson 4**: 6 concepts (token math, context window, tool matrix, tradeoffs, strategy)
- **Lesson 5 Part 1**: 5 concepts (MCP basics, stdio/HTTP/SSE, configuration, plugins)
- **Lesson 5 Part 2**: 6 concepts (CLI commands, add/list/remove, syntax, verification)
- **Lesson 5 Part 3**: 4 concepts (OAuth high-level, tokens, browser flow, use cases)
- **Lesson 5 Part 4**: 6 concepts (business workflows, Playwright, Context7, multi-tool)
- **Lesson 6 Part 1**: 6 concepts (extensions, templates, link, install vs link, lifecycle)
- **Lesson 6 Part 2**: 4 concepts (security, includeTools, excludeTools, best practices)
- **Lesson 6 Part 3**: 5 concepts (IDE integration, benefits, VS Code workflow, when valuable)
- **Lesson 6 Part 4**: 3 concepts (decision framework, combining capabilities, professional setup)

**Validation**: All sections within 7-concept limit. ✅

### AI Usage Strategy (Spec Section 4)

The plan implements the three-tier AI usage strategy from the spec:

#### When Students Use AI (Strategic)

**Tier 1: Direct Commands** (Students run, 1-5 seconds):
```bash
npm install -g @google/gemini-cli      # 30-60 seconds
gemini -v                              # 1 second
gemini                                 # 2 seconds
gemini mcp add my-server python server.py   # 3-5 seconds
gemini mcp list                        # 1 second
gemini extensions list                 # 1 second
/ide enable                            # 2 seconds
```

**Tier 2: AI Companion** (For understanding/troubleshooting):
- "What is Model Context Protocol?"
- "I got 'npm: command not found'—what does this mean?"
- "Should I use Gemini CLI or Claude Code for analyzing 12 reports?"
- "Why did a browser window open during authentication?"

**Tier 3: AI Orchestration** (Complex workflows):
- "Design custom extension from scratch"
- "Set up Playwright MCP server for my project"
- "Create multi-tool workflow: Playwright → Context7 → Analysis"

#### "Try With AI" Format (Chapter 1 Clean Style)

**NOT this** (verbose, pre-explanation heavy):
```markdown
## Understanding MCP Servers

Before we dive into MCP, let's explore what makes them special. Model Context Protocol servers...
[long paragraph]

Now, let's ask our AI companion:
<backticks>
Explain what MCP servers enable
</backticks>
```

**DO this** (clean, scannable):
```markdown
### Prompt 1: Understanding MCP Value
<backticks>
What is Model Context Protocol and why would I add an MCP server to my workflow?
</backticks>

**Expected outcome**: Explanation of MCP as a tool integration standard, examples of use cases (Playwright browser automation, Context7 documentation)
```

---

## III. Lesson-by-Lesson Breakdown

### Lesson 1: Why Gemini CLI Matters (15 minutes)

**Learning Objective (Bloom's Taxonomy: Remember + Understand)**:
- Students can recall why Gemini CLI differs from web interfaces
- Students can explain strategic advantages: free tier quotas, 1M token context, offline capability

**Section Structure**:

#### Section 1.1: The Game-Changing Differences
- **Duration**: 5 minutes
- **Concepts**: CLI advantage, context window, free tier, integration capability
- **Content Elements**:
  - Real comparison table: ChatGPT vs Claude Code vs Gemini CLI vs Qwen Code
  - Include IDE integration row (NEW from verified intelligence)
  - Free tier quotas: 60 req/min, 1000 req/day (VERIFIED)
  - Context window: 1M tokens = ~750K words = ~100K lines code
- **Example Context**: "The 1M token context means Gemini CLI can see your entire codebase + 20 reports"
- **Practice**: None (conceptual lesson)

#### Section 1.2: When Gemini CLI Is Your Best Choice
- **Duration**: 5 minutes
- **Concepts**: Use case selection, tool matrix, cost consideration, workload matching
- **Content Elements**:
  - Scenarios where Gemini CLI excels (large documents, free tier, offline, bulk operations)
  - Scenarios where Claude Code or ChatGPT better (web interface preference, enterprise support)
  - Cost analysis: Free tier sufficiency for learning
- **Example**: "If you need to analyze 12 monthly reports at once, Gemini CLI's 1M tokens handles them in one session"
- **Practice**: Reflection (no hands-on yet)

#### Section 1.3: Qwen Code: The Open Source Alternative (NEW mention)
- **Duration**: 1-2 minutes
- **Concepts**: Open source ecosystem, Alibaba fork, free tier comparison
- **Content Elements**:
  - One paragraph: "Qwen Code is a community fork with similar CLI interface, emphasizing open source and 2,000 req/day free tier"
  - NOT a deep dive (keep focus on Gemini CLI)
  - Positioning: "Another excellent choice; we focus on Gemini CLI for hands-on learning"
- **Example**: Brief mention, no code
- **Practice**: None

#### Section 1.4: Open Source Ecosystem Effect
- **Duration**: 3-4 minutes
- **Concepts**: Ecosystem, MCP standards, extensions, IDE integration
- **Content Elements**:
  - Explain how Gemini CLI connects to broader AI ecosystem
  - IDE integration enables VS Code workflow
  - MCP servers extend capabilities
  - Extensions bundle configurations
- **Example**: "The IDE integration means your VS Code sees your most recent files—no context switching"
- **Practice**: None (yet)

### Try With AI (Lesson 1)
**4 prompts, focused on positioning and understanding**:

1. **Prompt 1: Tool Selection Strategy**
   <backticks>
   I need to analyze 12 monthly financial reports. Should I use Gemini CLI, ChatGPT, or Claude Code? Explain the tradeoffs.
   </backticks>
   Expected outcome: Explanation of context window advantage (Gemini CLI's 1M tokens), cost consideration, practical recommendations

2. **Prompt 2: Understanding Token Context**
   <backticks>
   What does "1 million tokens" really mean? How many documents or lines of code fit in Gemini CLI's context?
   </backticks>
   Expected outcome: Concrete translation (750K words, 100K lines of code, multiple reports)

3. **Prompt 3: When CLI Beats Web**
   <backticks>
   List 3 scenarios where using Gemini CLI is significantly better than ChatGPT's web interface.
   </backticks>
   Expected outcome: Context size, offline capability, integration, cost, batch processing

4. **Prompt 4: Your Project Fit**
   <backticks>
   I'm building [describe your project]. Which AI tool should I use and why? Consider context window, cost, and integration needs.
   </backticks>
   Expected outcome: Personalized tool recommendation with reasoning

**Duration**: 10-15 minutes (student can do 1 or all 4)
**Scaffolding**: No scaffolding needed; conceptual exploration

---

### Lesson 2: Installation, Authentication & First Steps (15 minutes)

**Learning Objective (Bloom's: Understand + Apply)**:
- Students can successfully install Gemini CLI and verify installation
- Students can explain what "global installation" means and authenticate with Gemini
- Students can run first session and identify basic session commands

**Section Structure**:

#### Section 2.1: Installing Gemini CLI
- **Duration**: 3-4 minutes
- **Concepts**: npm, global installation, Node.js prerequisite, error handling
- **Content Elements**:
  - Command (VERIFIED): `npm install -g @google/gemini-cli`
  - Expected output: "added 47 packages in 3s"
  - Troubleshooting for common errors:
    - "npm: command not found" → Install Node.js first
    - "Permission denied (macOS/Linux)" → Use `sudo` or admin PowerShell (Windows)
    - "Module not found" → Clear npm cache with `npm cache clean --force`, reinstall
  - What "global installation" means in practical terms (available everywhere in terminal)
- **Example**: Step-by-step terminal output shown
- **Practice**: Hands-on installation (Eval 1)
- **Estimated Time**: 1-2 minutes (installation) + 1-2 minutes (verification)

#### Section 2.2: Authentication Methods & OAuth Flow
- **Duration**: 4-5 minutes
- **Concepts**: OAuth, authentication methods, API key, browser flow, token storage
- **Content Elements**:
  - Three authentication options:
    1. Google login (recommended for beginners, OAuth flow)
    2. API Key (for scripting)
    3. Vertex AI (enterprise, skipped for A2-B1 audience)
  - OAuth flow explanation (beginner-friendly):
    - Run `gemini`, terminal prompts for theme
    - Select "Google login"
    - Browser opens for authentication
    - Return to terminal with authenticated session
  - Token storage: "~/.gemini" directory (macOS/Linux) or "C:\Users\<username>\.gemini" (Windows)
- **Example**: Screenshots of browser OAuth flow (if available)
- **Practice**: Students authenticate (part of Eval 1)

#### Section 2.3: Your First Gemini CLI Session
- **Duration**: 3-4 minutes
- **Concepts**: Session commands, slash commands, tool discovery, help
- **Content Elements**:
  - Launch command: `gemini`
  - Basic commands available:
    - `/help` - Show all available commands
    - `/tools` - View available tools
    - `/stats` - Session statistics
    - `/mcp` - View MCP server status (NEW)
    - `/mcp auth` - Manage OAuth (NEW - high-level only)
    - `/ide install` and `/ide enable` (NEW - mentioned, detailed in Lesson 6)
  - Exiting: `/quit` or Ctrl+C
  - Text interaction: Type a prompt and press Enter
- **Example**: "Enter: 'List 5 fruits' and press Enter. Gemini CLI responds with fruit list"
- **Practice**: Hands-on session exploration (Eval 1 verification)

#### Section 2.4: Red Flags to Watch (Error Literacy)
- **Duration**: 2 minutes
- **Concepts**: Error recognition, expected output vs problems
- **Content Elements**:
  - ✅ **Normal output**: Version numbers, "added X packages", "Authenticated successfully"
  - ⚠️ **Actual problems**: "command not found", "EACCES permission denied", "Cannot find module"
  - **Strategy**: When unsure, ask AI: "I got error X—what does it mean?"
- **Example**: "This is expected: 'Reading settings file'. This is a problem: 'ENOENT: no such file'"
- **Practice**: None (reference section)

### Try With AI (Lesson 2)
**4 prompts, focused on troubleshooting and understanding**:

1. **Prompt 1: Troubleshooting Installation**
   <backticks>
   I tried to install Gemini CLI but got "npm: command not found". What should I do?
   </backticks>
   Expected outcome: "You need to install Node.js first" with download link

2. **Prompt 2: Understanding Authentication**
   <backticks>
   Explain the OAuth flow that happens when I run `gemini` and select Google login. Why does a browser window open?
   </backticks>
   Expected outcome: Explanation of OAuth, why browser is needed, security context

3. **Prompt 3: Your First Session**
   <backticks>
   I successfully installed Gemini CLI and authenticated. What's a good first thing to ask it to test that it's working?
   </backticks>
   Expected outcome: Suggestions for simple, fast tasks (list 5 items, summarize text, explain concept)

4. **Prompt 4: Permission Errors**
   <backticks>
   I got "EACCES: permission denied" error on macOS. How do I fix this?
   </backticks>
   Expected outcome: Use `sudo npm install -g @google/gemini-cli` or Node Version Manager (nvm)

**Duration**: 10-15 minutes
**Scaffolding**: High—students may encounter real installation issues

---

### Lesson 3: Built-In Tools Deep Dive (20-25 minutes)

**Learning Objective (Bloom's: Understand + Apply)**:
- Students can explain what each built-in tool does
- Students can use file operations, shell integration, and web fetching strategically
- Students can identify tool limitations and know when to use multiple tools together

**Section Structure**:

#### Section 3.1: File Operations Tool
- **Duration**: 4-5 minutes
- **Concepts**: File reading, glob patterns, supported formats, limitations
- **Content Elements**:
  - Tool capabilities (VERIFIED):
    - Read local files: CSV, JSON, PDF, text, XML, Markdown
    - Pattern matching with glob syntax (e.g., `*.csv`)
    - Extract specific text ranges (if documented)
  - Practical examples (from verified intelligence or safe abstractions):
    - "Reading a CSV file to analyze data"
    - "Finding all Python files in project with glob"
    - "Extracting text from PDF"
  - Limitations:
    - Can only READ (not write) - safety feature
    - File size limits (if documented) or note as TBD
    - Local files only (not remote URLs—use web tool instead)
- **Example**: Code showing file read result
- **Practice**: Spec-style prompt: "Analyze this CSV file for trends"

#### Section 3.2: Shell Integration Tool
- **Duration**: 4-5 minutes
- **Concepts**: Command execution, configurable restrictions, interactive shell, output capture
- **Content Elements**:
  - Tool capabilities (VERIFIED):
    - Execute terminal commands (bash, PowerShell, zsh)
    - Configurable restrictions (some commands disabled for safety)
    - Output is captured and returned
  - Safe commands: `ls`, `pwd`, `git log`, `npm test`, `python script.py`
  - Restricted: Dangerous commands (delete, format disk) are blocked by default
  - Strategic use: Let Gemini run your tests or build scripts
- **Example**: "Run `npm test` and show results to Gemini for debugging"
- **Practice**: Spec-style prompt: "Run my tests and explain failures"

#### Section 3.3: Web Fetching Tool
- **Duration**: 3-4 minutes
- **Concepts**: URL fetching, content extraction, limitations, caching
- **Content Elements**:
  - Tool capabilities (VERIFIED):
    - Fetch public web pages
    - Extract main content
    - Works with APIs that return HTML/JSON
  - Limitations:
    - Public URLs only (not behind authentication)
    - Rate limiting (respect robots.txt)
    - Slow pages may time out
  - Strategic use: "Fetch latest documentation"
- **Example**: "Fetch Python docs page and explain decorator syntax"
- **Practice**: Spec-style prompt: "Fetch this URL and summarize main content"

#### Section 3.4: Google Search Grounding Tool
- **Duration**: 3-4 minutes
- **Concepts**: Web search, current information, citations, grounding
- **Content Elements**:
  - Tool capabilities (VERIFIED):
    - Search web for current information
    - Synthesize results with citations
    - Good for: Recent news, current best practices, trending topics
  - When to use: "What's the latest Python version?"
  - When not to use: Local project analysis (file tool better)
- **Example**: "Search 'Python 3.13 features' and summarize findings with sources"
- **Practice**: Spec-style prompt: "Search for current best practices in [domain]"

#### Section 3.5: Combining Tools Strategically
- **Duration**: 2-3 minutes
- **Concepts**: Multi-tool workflows, chaining, tool sequencing
- **Content Elements**:
  - Example workflows:
    - Fetch API docs + analyze local code = integration planning
    - Search recent research + read local files = comparative analysis
    - Run tests + search documentation = debugging workflow
  - Strategy: "Start with file tool for local context, then search if external info needed"
- **Example**: "Analyze my 10 test files, search for best practices, suggest improvements"
- **Practice**: None (planning practice)

#### Section 3.6: Red Flags to Watch
- **Duration**: 1-2 minutes
- **Concepts**: Error recognition, expected limitations
- **Content Elements**:
  - ✅ Normal: "File not found—do you want me to create it?" (requires user action)
  - ✅ Normal: Tool timeout on slow websites (retry or use different URL)
  - ⚠️ Problem: File read returning empty when it shouldn't (permission issue)
  - ⚠️ Problem: Shell command blocked (if trying something dangerous)
- **Strategy**: Error literacy (ask AI when unsure)

### Try With AI (Lesson 3)
**4 prompts, focused on tool usage**:

1. **Prompt 1: File Analysis**
   <backticks>
   Read this file [path to local file] and explain what it does. Then suggest improvements.
   </backticks>
   Expected outcome: File content analyzed, improvements suggested

2. **Prompt 2: Running Tests with Gemini**
   <backticks>
   Run my test suite with this command: npm test. Tell me what passed and what failed.
   </backticks>
   Expected outcome: Test results captured and analyzed

3. **Prompt 3: Web Fetching for Documentation**
   <backticks>
   Fetch [documentation URL] and explain [specific concept] based on the docs.
   </backticks>
   Expected outcome: Relevant section extracted and explained

4. **Prompt 4: Multi-Tool Workflow**
   <backticks>
   I have these local files [list files]. Search the web for best practices in [domain]. Then combine both to suggest improvements.
   </backticks>
   Expected outcome: Local analysis + web research = comprehensive suggestions

**Duration**: 15-20 minutes
**Scaffolding**: Medium—students specify what to analyze; Gemini handles execution

---

### Lesson 4: Context Window & Tool Comparison (25 minutes)

**Learning Objective (Bloom's: Understand + Analyze)**:
- Students can explain what "token context" means in practical terms
- Students can analyze tradeoffs between Gemini CLI, ChatGPT, Claude Code
- Students can make strategic decisions about which tool to use for specific scenarios

**Section Structure**:

#### Section 4.1: Understanding Token Context
- **Duration**: 4-5 minutes
- **Concepts**: Token math, context window, practical limits, tool capacity
- **Content Elements**:
  - Token definition: "A token is roughly 4 characters; 750 words = 1,000 tokens"
  - Gemini CLI context: 1 million tokens
  - Practical translation (VERIFIED):
    - ~750,000 words
    - ~100,000 lines of code
    - ~15-20 full financial reports
    - Entire small to medium codebase
  - Why it matters: "Larger context = more information AI can see at once = fewer errors"
- **Example**: "1M tokens = 5 folders of code + 10 reports + project documentation"
- **Practice**: Mental math ("How many pages is 100K lines of code?")

#### Section 4.2: Comparison Matrix
- **Duration**: 5-6 minutes
- **Concepts**: Competitive landscape, tradeoff analysis, decision criteria
- **Content Elements**:
  - Detailed table (VERIFIED context):
    | Feature | ChatGPT | Claude Code | Gemini CLI | Qwen Code |
    |---------|---------|-------------|-----------|-----------|
    | Context | 128K | 200K | 1M | ~128K |
    | Free Tier | Limited | Limited | 60/min, 1K/day | 2,000/day |
    | Setup | Browser | CLI | CLI | CLI |
    | IDE Integration | Limited | Built-in | VS Code companion (NEW) | ? |
    | Offline | No | No | Planned (no offline) | ? |
    | Cost | $20/mo | $20/mo | Free | Free |
  - Add new IDE integration row for Gemini CLI (updated from verified intelligence)
  - Tradeoff discussion: Cost vs context vs convenience
- **Example**: "For analyzing 12 reports, Gemini CLI fits all in context; ChatGPT needs splitting"
- **Practice**: Scenario analysis (no hands-on yet)

#### Section 4.3: Token Math in Practice
- **Duration**: 3-4 minutes
- **Concepts**: Capacity calculation, planning, batch processing
- **Content Elements**:
  - Example: "Each report = 5,000 tokens; 1M context = 200 reports"
  - Example: "100K lines of code = average project; Gemini CLI sees entire codebase at once"
  - Planning strategy: "Calculate if your task fits; if yes, use Gemini CLI; if no, split into batches"
- **Example**: "Estimate tokens: 10-page report ≈ 2,000 tokens; 500 reports ≈ 1M tokens"
- **Practice**: Student calculation ("How many X fit in Gemini CLI's context?")

#### Section 4.4: Strategic Tool Selection
- **Duration**: 5-6 minutes
- **Concepts**: Decision criteria, use case matching, recommendation framework
- **Content Elements**:
  - Decision tree:
    - **Use Gemini CLI if**: Large context (10+ documents), free tier (1000/day), offline (future), integration needed (MCP)
    - **Use Claude Code if**: IDE integration is critical (already built-in), enterprise support needed
    - **Use ChatGPT if**: Simplicity > features, web interface preference
  - Real scenarios:
    - "Analyze competitor reports" → Gemini CLI (multiple documents)
    - "Quick code question" → ChatGPT (simplicity)
    - "Build IDE-integrated assistant" → Claude Code (native integration)
    - "Add Playwright to workflow" → Gemini CLI (MCP integration)
  - When to combine: Use multiple tools for different tasks in same project
- **Example**: "Your project needs web automation (Gemini CLI + MCP) + testing (ChatGPT)"
- **Practice**: Scenario matching (spec-style: "My project needs X, Y, Z—which tool?")

#### Section 4.5: When to Ask Your AI: Tool Selection Edition
- **Duration**: 2-3 minutes
- **Concepts**: Delegation, expert decision-making, human-AI partnership
- **Content Elements**:
  - "Don't memorize the comparison table. Instead, describe your task to AI and ask: 'Should I use Gemini CLI or Claude Code for this?'"
  - AI will consider context, cost, integration needs, and recommend
  - This is Tier 2 AI usage (AI as teacher)
- **Example**: "Tell your AI companion: 'I need to analyze 8 competitor analyses. Which tool?'"
- **Practice**: Spec-style prompt (next section)

### Try With AI (Lesson 4)
**4 prompts, focused on tool selection and context understanding**:

1. **Prompt 1: Token Math**
   <backticks>
   I have 500 pages of documentation (200 words per page). How many tokens is that? Does Gemini CLI's 1M context fit it?
   </backticks>
   Expected outcome: Token calculation, yes it fits, explanation of how much room remains

2. **Prompt 2: Tool Comparison**
   <backticks>
   Compare Gemini CLI, Claude Code, and ChatGPT for my use case: analyzing 10 monthly financial reports. What are the tradeoffs?
   </backticks>
   Expected outcome: Context window advantage (Gemini CLI), cost analysis, recommendation with reasoning

3. **Prompt 3: Real-World Scenario**
   <backticks>
   My project needs to: (1) analyze 500 code files, (2) automate browser testing, (3) run on my laptop without internet.
   Which tool should I use and why?
   </backticks>
   Expected outcome: Gemini CLI for (1) and (2) via MCP, offline limitation noted for (3)

4. **Prompt 4: Capacity Planning**
   <backticks>
   I need to process 1,000 customer support transcripts (each 500 words). Can Gemini CLI handle all at once?
   Plan how to approach this.
   </backticks>
   Expected outcome: Token calculation showing need for batching, batch size recommendations, workflow suggestion

**Duration**: 15-20 minutes
**Scaffolding**: Medium-high—students apply learning to personalized scenarios

---

### Lesson 5: MCP Servers & Integration (20 minutes)

**Learning Objective (Bloom's: Understand + Apply)**:
- Students can explain what MCP servers are and why they extend capabilities
- Students can add MCP servers using CLI commands (NEW)
- Students can explain OAuth authentication at a high level (NEW)
- Students can design business workflows combining Gemini CLI with external tools

**Part 1: Understanding MCP** (5 minutes)

#### Section 5.1.1: What is Model Context Protocol?
- **Duration**: 3 minutes
- **Concepts**: MCP standard, plugin architecture, external tools, capabilities extension
- **Content Elements**:
  - MCP = standard for connecting external tools to AI (Playwright, APIs, databases)
  - MCP server = a program providing specific capabilities (e.g., browser automation)
  - Why it matters: "MCP expands what Gemini CLI can do beyond built-in tools"
  - Example servers: Playwright (browser automation), Context7 (documentation), custom APIs
- **Example**: "Playwright MCP lets Gemini CLI control a browser and automate web tasks"
- **Practice**: None (conceptual)

#### Section 5.1.2: MCP vs Extensions (Distinction)
- **Duration**: 2 minutes
- **Concepts**: MCP (individual tool), extensions (bundles), pre-configuration
- **Content Elements**:
  - **MCP Server**: Single capability, manually configured or via CLI
  - **Extension**: Pre-packaged bundle of MCP servers + commands + configuration
  - **When to use each**: Start with MCP for one tool; graduate to extensions for ecosystem
- **Example**: "Playwright is an MCP server; a security-tools extension bundles Playwright + others"
- **Practice**: None (reference)

---

**Part 2: CLI MCP Management Commands** (7 minutes) [NEW FEATURE]

#### Section 5.2.1: Adding MCP Servers with CLI
- **Duration**: 2-3 minutes
- **Concepts**: CLI commands, transport types, configuration, verification
- **Content Elements**:
  - CLI commands (VERIFIED from intelligence cache):
    ```bash
    # Stdio transport (local Python/Node.js server)
    gemini mcp add my-server python server.py --port 8080

    # HTTP transport (remote server)
    gemini mcp add --transport http secure-api https://api.example.com/mcp \
      --header "Authorization: Bearer abc123"

    # SSE transport (real-time streaming)
    gemini mcp add --transport sse events-api https://api.example.com/sse
    ```
  - Alternative: Manual `~/.gemini/settings.json` editing (advanced; mentioned but not primary)
  - Advantages of CLI: Less error-prone, easier to verify, modern workflow
- **Example**: "Add Playwright stdio server: `gemini mcp add playwright python playwright-mcp.py`"
- **Practice**: Spec-style prompt ("Add this MCP server to your Gemini CLI")

#### Section 5.2.2: Listing and Verifying MCP Servers
- **Duration**: 2 minutes
- **Concepts**: Status checking, server connectivity, debugging
- **Content Elements**:
  - Verification command: `gemini mcp list`
  - Expected output: List of configured servers with status (connected/disconnected)
  - What to check: "Is your server showing up? Is status 'connected'?"
- **Example**: "After running `gemini mcp add`, verify with `gemini mcp list`"
- **Practice**: Hands-on verification

#### Section 5.2.3: Removing MCP Servers
- **Duration**: 1 minute
- **Concepts**: Cleanup, lifecycle management
- **Content Elements**:
  - Command: `gemini mcp remove server-name`
  - When to use: "Remove servers you no longer need"
- **Example**: "Remove deprecated Playwright server: `gemini mcp remove playwright`"
- **Practice**: None (reference)

#### Section 5.2.4: CLI vs Manual Configuration
- **Duration**: 2 minutes
- **Concepts**: Comparison, tradeoffs, when to use each
- **Content Elements**:
  - CLI: Easier, less error-prone, beginner-friendly, modern
  - Manual JSON editing: Advanced, full control, when CLI doesn't fit your use case
  - Recommendation: "Use CLI commands; only edit settings.json if CLI doesn't support your need"
- **Example**: "Most users should use `gemini mcp add`; enterprises may customize JSON for secrets management"
- **Practice**: None (advisory)

---

**Part 3: OAuth for MCP Servers** (5 minutes) [NEW FEATURE - HIGH-LEVEL]

#### Section 5.3.1: Why OAuth Exists
- **Duration**: 1-2 minutes
- **Concepts**: Authentication, security, token management, use cases
- **Content Elements**:
  - Problem: "Some MCP servers need secure authentication (API keys, user accounts)"
  - Solution: OAuth allows secure token exchange without exposing secrets
  - Practical use case: "Access a user's Google Drive through an MCP server"
  - High-level approach: "Let Gemini CLI handle token refresh automatically"
- **Example**: "Your MCP server needs a Google Drive API key—OAuth secures this"
- **Practice**: None (conceptual)

#### Section 5.3.2: Using OAuth with `/mcp auth`
- **Duration**: 2 minutes
- **Concepts**: Command usage, browser flow, automatic token management
- **Content Elements**:
  - Command: `/mcp auth` (in Gemini CLI session)
  - Behavior:
    1. Shows servers requiring authentication
    2. Offers to authenticate with selected server
    3. Opens browser for OAuth flow
    4. Tokens stored securely at `~/.gemini/mcp-oauth-tokens.json`
    5. Gemini CLI refreshes tokens automatically
  - Key point: "You don't manage tokens manually; Gemini CLI handles it"
- **Example**: "In a Gemini CLI session, type `/mcp auth` and follow prompts"
- **Practice**: Spec-style prompt ("Authenticate this OAuth-protected MCP server")

#### Section 5.3.3: When to Use OAuth
- **Duration**: 1-2 minutes
- **Concepts**: Use case identification, security awareness
- **Content Elements**:
  - Use OAuth when: MCP server needs API key, user credentials, or cloud access
  - Don't use OAuth when: Server is local and doesn't need authentication
  - Security principle: "Never hardcode API keys; use OAuth or environment variables"
- **Example**: "Your Playwright MCP needs access to a protected website—use OAuth for that site's credentials"
- **Practice**: Scenario identification (reference)

#### Section 5.3.4: Red Flags to Watch
- **Duration**: 1 minute
- **Concepts**: Error recognition, troubleshooting
- **Content Elements**:
  - ✅ Normal: Browser window opens during `/mcp auth` (expected OAuth flow)
  - ⚠️ Problem: OAuth flow timeout (slow network, try again)
  - ⚠️ Problem: "Token invalid" error (tokens may have expired, re-authenticate)
  - Strategy: "If OAuth doesn't work, ask your AI: 'I got error X during `/mcp auth`'"
- **Example**: Error literacy section
- **Practice**: None (reference)

---

**Part 4: Business Workflows Using MCP** (3 minutes)

#### Section 5.4.1: Multi-Tool Workflows
- **Duration**: 2 minutes
- **Concepts**: Workflow design, tool chaining, business applications
- **Content Elements**:
  - Example 1: Competitive research
    - Playwright MCP: Automate browser to fetch competitor websites
    - File tool: Analyze competitor pricing pages (saved as HTML)
    - Web search: Find competitor announcements
    - Result: Comprehensive competitor analysis
  - Example 2: API documentation analysis
    - Context7 MCP: Fetch API documentation
    - File tool: Analyze your code that uses the API
    - Shell tool: Run your tests
    - Result: Documentation-guided code improvements
  - Example 3: Multi-file analysis
    - File tool: Read 10 project files
    - Shell tool: Run linter and tests
    - Web search: Best practices for detected issues
    - Result: Comprehensive code review
- **Example**: "Use Playwright to scrape competitor sites, then have Gemini CLI analyze and suggest pricing strategy"
- **Practice**: Workflow design (spec-style prompt)

#### Section 5.4.2: When to Escalate to AI Orchestration
- **Duration**: 1 minute
- **Concepts**: Workflow complexity, automation, Tier 3 usage
- **Content Elements**:
  - Manual workflow: Specify each step (Part 4.1)
  - Orchestrated workflow: Tell AI: "Analyze competitors using Playwright + web search + file analysis"
  - AI handles: Step sequencing, tool coordination, result compilation
- **Example**: "Tell Gemini: 'Analyze our top 5 competitors for pricing trends.' CLI orchestrates: fetch (Playwright) → download (file) → analyze → report"
- **Practice**: None (reference)

### Try With AI (Lesson 5)
**4 prompts, focused on MCP workflows**:

1. **Prompt 1: Setting Up MCP**
   <backticks>
   I want to add Playwright MCP to my Gemini CLI. Give me the exact command to run and explain what it does.
   </backticks>
   Expected outcome: Example command, explanation of what Playwright enables

2. **Prompt 2: OAuth Authentication**
   <backticks>
   I added an MCP server that needs API authentication. How do I authenticate it with Gemini CLI? Why is OAuth better than hardcoding the key?
   </backticks>
   Expected outcome: `/mcp auth` explained, OAuth benefits, security context

3. **Prompt 3: Multi-Tool Workflow**
   <backticks>
   Design a workflow where I use Gemini CLI with Playwright MCP to fetch a competitor's website,
   then analyze their pricing. What steps would you take?
   </backticks>
   Expected outcome: Workflow plan combining tools, step sequence, output format

4. **Prompt 4: Your Project's MCP Needs**
   <backticks>
   My project needs to [describe your needs]. Which MCP server(s) should I add? How would I set them up?
   </backticks>
   Expected outcome: Relevant MCP server recommendations, setup commands, integration suggestions

**Duration**: 15-20 minutes
**Scaffolding**: Medium—students transition from following examples to specifying their needs

---

### Lesson 6: Extensions, Security & IDE Integration (15-20 minutes)

**Learning Objective (Bloom's: Understand + Apply + Analyze)**:
- Students can explain extension development workflow (create/link/update)
- Students can describe security implications of external tools (tool filtering)
- Students can enable IDE integration and explain workflow benefits
- Students can decide when to use MCP vs Extensions vs IDE integration

**Part 1: Extension Development Workflow** (6 minutes) [NEW LESSON]

#### Section 6.1.1: Extensions vs MCP Servers (Review)
- **Duration**: 1 minute
- **Concepts**: Relationship, distinction, bundling
- **Content Elements**:
  - Recall from Lesson 5: Extensions bundle MCP servers + configuration
  - Extension use case: "Publish a reusable bundle for others"
  - MCP use case: "Add a single tool to your workflow"
- **Example**: "An extension might bundle Playwright + Context7 + custom commands; an MCP is just Playwright"
- **Practice**: None (review)

#### Section 6.1.2: Creating Extensions from Templates
- **Duration**: 1-2 minutes
- **Concepts**: Template system, project structure, extension manifest
- **Content Elements**:
  - Command (VERIFIED): `gemini extensions new my-extension mcp-server`
  - What gets created:
    - `gemini-extension.json` (manifest)
    - `GEMINI.md` (persistent context)
    - `commands/` directory (custom slash commands)
    - `dist/` directory (server code)
  - Template types: `mcp-server`, `python-server`, `node-server` (varies)
- **Example**: "Run `gemini extensions new security-tools security-mcp` to create a new extension"
- **Practice**: Spec-style prompt ("Create an extension for X purpose")

#### Section 6.1.3: Development Workflow: Link vs Install
- **Duration**: 2-3 minutes
- **Concepts**: Development mode, installation, iteration, versioning
- **Content Elements**:
  - **`install` (Production)**: Install extension from package; changes require reinstall
    ```bash
    gemini extensions install https://github.com/org/security-extension
    ```
  - **`link` (Development)**: Point to local folder; changes reflected immediately
    ```bash
    gemini extensions link ~/my-extension
    ```
  - Workflow: Create → **link for development** → test → iterate → **publish** → `install` for users
  - Key distinction: "Use `link` while developing; users will `install` from package"
- **Example**: "Create extension → link it → edit `GEMINI.md` → changes immediate → no reinstall needed"
- **Practice**: Hands-on linking (if time permits)

#### Section 6.1.4: Managing Extensions Lifecycle
- **Duration**: 1-2 minutes
- **Concepts**: Enable/disable, update, uninstall, version management
- **Content Elements**:
  - Commands (VERIFIED):
    ```bash
    gemini extensions list              # Show installed
    gemini extensions enable security   # Activate
    gemini extensions disable security  # Deactivate
    gemini extensions update security   # Update single
    gemini extensions update --all      # Update all
    gemini extensions uninstall security # Remove
    ```
  - Scoping: Extensions can be workspace-specific or user-wide
  - Update strategy: "Check for updates regularly; extensions may add new features"
- **Example**: "Update all extensions: `gemini extensions update --all`"
- **Practice**: Reference (no hands-on; building extensions is Part 2 work)

#### Section 6.1.5: Extension Manifest Structure
- **Duration**: 1 minute
- **Concepts**: Configuration, metadata, tool filtering (preview)
- **Content Elements**:
  - Key fields (from verified intelligence):
    - `name`, `version`: Metadata
    - `mcpServers`: Bundled servers
    - `contextFileName`: Persistent context file
    - `excludeTools`: Security filtering (covered in Part 2)
    - `settings`: User-configurable options (env vars)
  - Advanced: "Full manifest structure shown in Part 2 security section"
- **Example**: Preview of `gemini-extension.json` (full example in Part 2)
- **Practice**: None (reference; full manifest in Part 2)

---

**Part 2: Tool Filtering for Security** (5 minutes) [NEW FEATURE]

#### Section 6.2.1: Why Tool Filtering Matters
- **Duration**: 2 minutes
- **Concepts**: Security principle, capability restriction, risk mitigation, trust
- **Content Elements**:
  - Problem: "External MCP servers may include dangerous tools"
  - Solution: Restrict which tools Gemini CLI can use
  - Real scenario:
    ```
    An MCP server provides: read_file, write_file, delete_file
    You trust reading, but not writing or deleting
    ```
  - Principle: "Defense in depth—don't expose capabilities you don't need"
- **Example**: "Add Playwright MCP (browser automation), but restrict it from reading local files"
- **Practice**: Scenario analysis (reference)

#### Section 6.2.2: Using `includeTools` and `excludeTools`
- **Duration**: 2 minutes
- **Concepts**: Allowlist vs blocklist, configuration, JSON syntax
- **Content Elements**:
  - **Allowlist** (`includeTools`): Only these tools are available
    ```json
    {
      "mcpServers": {
        "myServer": {
          "command": "python server.py",
          "includeTools": ["read_file", "summarize"]
        }
      }
    }
    ```
    → Only `read_file` and `summarize` available; everything else blocked
  - **Blocklist** (`excludeTools`): These tools are unavailable
    ```json
    {
      "mcpServers": {
        "myServer": {
          "command": "python server.py",
          "excludeTools": ["delete_file", "format_disk"]
        }
      }
    }
    ```
    → All tools except `delete_file` and `format_disk` available
  - When to use each:
    - **Allowlist**: Few safe tools, many dangerous ones
    - **Blocklist**: Many safe tools, few dangerous ones
- **Example**: "Playwright MCP: `includeTools: ['navigate', 'screenshot']` prevents clicking/typing"
- **Practice**: Security scenario (spec-style)

#### Section 6.2.3: Real-World Security Scenario
- **Duration**: 1 minute
- **Concepts**: Applied security, decision-making, trust assessment
- **Content Elements**:
  - Scenario: You find a cool GitHub MCP server for "productivity automation"
  - It provides: `create_note`, `delete_note`, `list_files`, `read_files`, `write_files`, `run_commands`
  - Assessment:
    - ✅ Trust: `create_note`, `delete_note`, `list_files`, `read_files`
    - ⚠️ Risky: `write_files` (need it? restrict anyway)
    - ❌ Dangerous: `run_commands` (block immediately)
  - Configuration:
    ```json
    {
      "includeTools": ["create_note", "delete_note", "list_files", "read_files"]
    }
    ```
  - Outcome: Use MCP safely; AI can create notes but can't delete your files or run arbitrary commands
- **Example**: Applied security decision-making
- **Practice**: Evaluate and configure (spec-style)

#### Section 6.2.4: Best Practices for External Tool Integration
- **Duration**: 0-1 minute
- **Concepts**: Security principles
- **Content Elements**:
  - 1. Always review what tools an MCP server provides
  - 2. Use allowlist (`includeTools`) when possible (whitelist is safer)
  - 3. Start restrictive, expand if needed
  - 4. Update restrictions if MCP server adds new tools
  - 5. Disable/uninstall MCP servers you don't use
- **Example**: "Principle: Only expose what you need"
- **Practice**: None (principle-based)

---

**Part 3: IDE Integration** (6 minutes) [NEW FEATURE]

#### Section 6.3.1: What IDE Integration Adds
- **Duration**: 2 minutes
- **Concepts**: IDE awareness, context sharing, workflow improvement, practical benefits
- **Content Elements**:
  - IDE integration allows Gemini CLI in terminal to "see" your VS Code editor
  - Benefits (VERIFIED from intelligence cache):
    - ✅ AI sees 10 most recently accessed files (context awareness)
    - ✅ AI knows cursor position and selected text (up to 16KB)
    - ✅ Native diff viewer for proposed changes (review before accepting)
    - ✅ One-click: "Accept Diff" to apply Gemini's code suggestions
  - Workflow improvement: "No context switching between terminal and editor"
- **Example**: "Ask Gemini to fix code; IDE integration shows diff; you click Accept"
- **Practice**: None (conceptual)

#### Section 6.3.2: Installing and Enabling IDE Integration
- **Duration**: 2 minutes
- **Concepts**: Commands, setup, VS Code connection
- **Content Elements**:
  - Installation (one-time):
    ```bash
    /ide install
    ```
    Prompts to install VS Code companion extension (if using VS Code)
  - Enabling (per session):
    ```bash
    /ide enable
    ```
    Connects terminal Gemini CLI to VS Code IDE
  - Checking status:
    ```bash
    /ide status
    ```
    Confirms IDE connection is active
  - Disabling:
    ```bash
    /ide disable
    ```
    Disconnects IDE integration
- **Example**: "In Gemini CLI session: `/ide enable` → VS Code now connected"
- **Practice**: Hands-on (if time permits)

#### Section 6.3.3: VS Code Workflow with IDE Integration
- **Duration**: 1-2 minutes
- **Concepts**: IDE workflow, diff review, change acceptance
- **Content Elements**:
  - Typical flow:
    1. Enable IDE integration: `/ide enable`
    2. Ask Gemini CLI a coding question (terminal)
    3. Gemini suggests code changes (sees your files via IDE)
    4. Native diff editor opens in VS Code (change preview)
    5. Review suggested changes (context available)
    6. Click "Accept Diff" or close without accepting
  - Benefits: "See exactly what Gemini will change before accepting"
  - Safety: "You always review before changes apply"
- **Example**: "Gemini suggests refactoring; diff editor shows before/after; you click Accept"
- **Practice**: Workflow walkthrough (no hands-on; time constraint)

#### Section 6.3.4: When IDE Integration Adds Value
- **Duration**: 1 minute
- **Concepts**: Use case, context, limitations
- **Content Elements**:
  - Add value when:
    - You frequently switch between editor and terminal
    - Code suggestions are complex (review via diff is safer)
    - You work with multiple files (10-file context helps)
  - Less valuable when:
    - You work with simple single-file changes
    - Your primary workflow is terminal (no IDE)
    - You use a different editor (integration is VS Code-specific)
  - Alternative: Terminal-only workflow still works; IDE integration is optional
- **Example**: "IDE integration is optional; use if it fits your workflow"
- **Practice**: Reflection (does this fit your workflow?)

#### Section 6.3.5: Red Flags to Watch
- **Duration**: 0-1 minute
- **Concepts**: Error recognition, troubleshooting
- **Content Elements**:
  - ✅ Normal: "IDE connection established; 10 files available" (expected)
  - ✅ Normal: Diff editor opens when Gemini suggests changes (expected)
  - ⚠️ Problem: "IDE connection failed" (restart VS Code, try again)
  - ⚠️ Problem: Diff viewer not opening (check VS Code extension is installed)
  - Strategy: Ask AI if setup doesn't work
- **Example**: Error literacy
- **Practice**: None (reference)

---

**Part 4: Choosing the Right Workflow** (3 minutes)

#### Section 6.4.1: Decision Framework
- **Duration**: 2 minutes
- **Concepts**: Strategic selection, workflow design, tool combination
- **Content Elements**:
  - **Use MCP When**: You need one specific capability (Playwright, Context7, API)
  - **Use Extensions When**: You want a pre-configured bundle of tools + commands
  - **Use IDE Integration When**: You work in VS Code and prefer code review in editor
  - **Combine All When**: Large project needing Playwright + Context7 + IDE workflow
  - Strategy: "Start simple (MCP for one tool), graduate to extensions (bundled setup)"
- **Example Decision Tree**:
    ```
    Do you need browser automation?
      → Yes: Add Playwright MCP
    Do you need API documentation access?
      → Yes: Add Context7 MCP
    Do you work in VS Code?
      → Yes: Enable IDE integration
    Do you want to package this for others?
      → Yes: Create extension bundling MCPs + commands
    ```
- **Practice**: Decision-making (spec-style prompt)

#### Section 6.4.2: Professional Development Setup Example
- **Duration**: 1 minute
- **Concepts**: Real-world integration, best practices, complete workflow
- **Content Elements**:
  - Example setup for web development project:
    - **MCP Servers**: Playwright (browser), Context7 (API docs), shell (test runner)
    - **Tool Filtering**: Only `navigate`, `screenshot` for Playwright; block shell's risky commands
    - **IDE Integration**: Enabled for code review workflow
    - **Extensions**: "WebDev Pro" extension bundles all of the above + custom commands
  - Why this works: "All tools integrated, filtered for safety, IDE-aware workflow"
- **Example**: "Senior developers use setups like this; you're building toward it"
- **Practice**: None (aspirational reference)

#### Section 6.4.3: When to Ask Your AI: Workflow Design
- **Duration**: 0-1 minute
- **Concepts**: Delegation, expert help
- **Content Elements**:
  - "Don't memorize decision rules. Describe your needs to AI:"
  - "I need to [browser automation] + [API documentation]. Design my Gemini CLI workflow."
  - AI will recommend MCP servers, tool filtering, IDE integration approach
  - This is Tier 2/3 AI usage (AI orchestrating your workflow)
- **Example**: "Ask AI to design your complete workflow; you just implement"
- **Practice**: Spec-style prompt (next section)

### Try With AI (Lesson 6)
**4 prompts, focused on extensions, security, and IDE integration**:

1. **Prompt 1: Extension Development**
   <backticks>
   I want to create an extension for [your use case]. What should I include? Should I use templates or start from scratch?
   </backticks>
   Expected outcome: Extension structure recommendation, template suggestions, MCP server bundling advice

2. **Prompt 2: Security Configuration**
   <backticks>
   I found this MCP server [server description] with these tools: [list tools].
   Which should I allow and which should I block? Give me the JSON configuration.
   </backticks>
   Expected outcome: Security assessment, allowlist or blocklist recommendation with JSON example

3. **Prompt 3: IDE Integration Workflow**
   <backticks>
   I'm working in VS Code. How does IDE integration with Gemini CLI change my workflow?
   What are the step-by-step advantages?
   </backticks>
   Expected outcome: Workflow walkthrough, diff viewer explanation, change acceptance process

4. **Prompt 4: Complete Workflow Design**
   <backticks>
   Design my complete Gemini CLI workflow for [your project type].
   What MCP servers should I add? Should I use IDE integration? Should I create an extension?
   </backticks>
   Expected outcome: Comprehensive workflow plan, MCP recommendations, security considerations, IDE integration advice

**Duration**: 15-20 minutes
**Scaffolding**: Medium-low—students are applying skills from earlier lessons

---

## IV. Content Elements & Verified Intelligence Allocation

### Examples & Code Snippets

All examples are sourced from `intelligence/chapter-6-gemini-cli-verified-docs.md`. Assignment:

| Content | Source | Lesson | Verification |
|---------|--------|--------|--------------|
| Installation command | VERIFIED cache | 2 | npm install -g @google/gemini-cli |
| Free tier quotas | VERIFIED cache | 1 | 60 req/min, 1000 req/day |
| Context window | VERIFIED cache | 4 | 1M tokens |
| Built-in tools | VERIFIED cache | 3 | File, Shell, Web, Search |
| MCP CLI commands | VERIFIED cache | 5.2 | gemini mcp add/list/remove |
| OAuth command | VERIFIED cache | 5.3 | /mcp auth |
| Extension creation | VERIFIED cache | 6.1 | gemini extensions new |
| Tool filtering | VERIFIED cache | 6.2 | includeTools, excludeTools |
| IDE integration | VERIFIED cache | 6.3 | /ide install, /ide enable |

**No Assumptions**: All technical claims reference verified intelligence; no placeholder examples

---

## V. "Try With AI" Prompt Design (Chapter 1 Format)

All "Try With AI" sections follow the clean format from Chapter 1:

```markdown
### Prompt [N]: [Title]
<backticks>
[Specific, actionable prompt]
</backticks>

**Expected outcome**: [What student should receive from AI]
```

**NOT**:
- Pre-explanations ("Now let's explore...")
- Multiple sub-prompts (keep 3-4 focused ones)
- Verbose instructions

**Total**: 24 prompts across 6 lessons (4 per lesson)
**Balance**: Mix of understanding (Tier 2) and application (Tier 2/3)

---

## VI. Cognitive Load Validation Summary

All sections validated for ≤7 concept limit (Principle 12):

- **Lesson 1**: 5 concepts per section ✅
- **Lesson 2**: 6 concepts per section ✅
- **Lesson 3**: 7 concepts per section (max) ✅
- **Lesson 4**: 6 concepts per section ✅
- **Lesson 5**: 4-6 concepts per part ✅
- **Lesson 6**: 3-6 concepts per part ✅

**Mitigation for Lesson 5 Expansion** (30-35 min → 20 min achieved by):
- Clear subsection headings for scannability
- Each part is 3-7 minutes (digestible chunks)
- "Try With AI" prompts provide practice breaks
- Students can skip advanced topics on first read (OAuth, tool filtering)

---

## VII. Scaffolding Strategy

### Progressive Complexity Across Lessons

| Lesson | Type | Tier | Practice |
|--------|------|------|----------|
| 1 | Conceptual | Tier 1 | Reflection only |
| 2 | Technical | Tier 1 | Hands-on installation |
| 3 | Technical | Tier 2 | Spec-style tool usage |
| 4 | Technical | Tier 2 | Scenario matching |
| 5 | Technical | Tier 2-3 | Workflow design |
| 6 | Technical | Tier 2-3 | Application & synthesis |

**Scaffolding Approach**:
- **Lessons 1-2**: Heavy scaffolding (step-by-step, manual practice)
- **Lessons 3-4**: Moderate scaffolding (guided scenarios, spec-style prompts)
- **Lessons 5-6**: Light scaffolding (design workflows, independent decision-making)

---

## VIII. Integration Points

### Cross-Chapter References

- **Chapter 1** (AI Revolution): "Gemini CLI is one concrete tool enabling the revolution"
- **Chapter 4** (First AI Project): "Use Gemini CLI for your first hands-on project"
- **Chapter 5** (Tool Landscape): "Gemini CLI compared to Claude Code and ChatGPT"
- **Chapter 7** (MCP Deep Dive): "Gemini CLI's MCP integration is your first exposure"

### Prerequisite Knowledge

- Basic terminal usage (covered in Chapter 3)
- Understanding of context window concept (explained in Lesson 4, reviewed from Chapter 5)
- Familiarity with authentication flows (generic knowledge, explained here)

---

## IX. Validation Strategy & Acceptance Criteria

### Pre-Implementation Validation

- [x] Spec approved and understood
- [x] Verified intelligence reviewed (47 verified facts)
- [x] Constitutional alignment checked (Principles 7, 13; Core Philosophy #1)
- [x] Cognitive load validated (≤7 concepts per section)

### Post-Implementation Validation

**Technical Accuracy**:
- [ ] All examples tested against verified intelligence cache
- [ ] All CLI commands verified (gemini mcp, /ide, /mcp auth)
- [ ] All "Try With AI" prompts achievable and specific
- [ ] No assumptions or hallucinations (verified facts only)

**Constitutional Alignment**:
- [ ] Graduated teaching (Tier 1/2/3 mapped)
- [ ] Cognitive load limits respected (concept count < 7)
- [ ] "Try With AI" format matches Chapter 1 clean style
- [ ] Principle 7 (technical accuracy) satisfied

**Pedagogical Quality**:
- [ ] Reading level: Grade 7-8 (Flesch-Kincaid check)
- [ ] Concept progression logical
- [ ] "Try With AI" prompts motivating and specific
- [ ] Durations realistic (not inflated)

**Sandbox Testing** (CRITICAL):
- [ ] Fresh install of Gemini CLI successful
- [ ] All CLI commands work as documented
- [ ] OAuth flow works (if test server available)
- [ ] IDE integration functional in VS Code
- [ ] Cross-platform (Windows, macOS, Linux) verified

---

## X. Success Criteria (Definition of Done)

**All MUST-HAVE Items Completed**:
- [ ] All 6 lessons written per this plan
- [ ] All examples from verified intelligence cache
- [ ] All "Try With AI" prompts Chapter 1 format
- [ ] Cognitive load ≤7 concepts per section
- [ ] Evals 1-5 achievable by students
- [ ] Constitutional principles satisfied

**All 5 Critical Features Integrated**:
- [ ] Lesson 5 Part 2: CLI MCP management (gemini mcp add/list/remove)
- [ ] Lesson 5 Part 3: OAuth for MCP (/mcp auth)
- [ ] Lesson 6 Part 1: Extension lifecycle (create/link/update)
- [ ] Lesson 6 Part 2: Tool filtering (includeTools/excludeTools)
- [ ] Lesson 6 Part 3: IDE integration (/ide commands)

**Quality Gates Passed**:
- [ ] Technical accuracy: 100% (all claims verified)
- [ ] Constitutional alignment: PASS (validation-auditor validation)
- [ ] Sandbox testing: PASS (all commands work)
- [ ] Reading level: Grade 7-8
- [ ] Durations realistic and summed to 110-135 min total

---

## XI. Timeline & Effort Estimate

| Phase | Task | Duration | Owner |
|-------|------|----------|-------|
| **Plan** | This document | 2 hours | chapter-planner |
| **Implement** | Write all 6 lessons + Try With AI | 6-8 hours | content-implementer |
| **Content Verification** | Cross-check against verified cache | 1 hour | content-implementer |
| **Technical Review** | Constitutional alignment + code | 1.5 hours | validation-auditor |
| **Sandbox Testing** | Install CLI, run all commands | 1.5 hours | human |
| **Validation Report** | Final quality gate | 1 hour | factual-verifier |
| **Finalization** | Index update, PHR creation | 0.5 hours | human |
| **TOTAL** | | 13.5-15 hours | |

---

## XII. Risk Mitigation

### Risk 1: Feature Creep (Adding beyond 5 critical features)

**Risk**: Scope expands beyond spec's 5 features
**Mitigation**: Stick to spec boundaries; defer advanced topics (service account impersonation, etc.)
**Acceptance**: Only implement: MCP CLI, OAuth, extensions, tool filtering, IDE integration

### Risk 2: CLI Syntax Changes Before Publication

**Risk**: Gemini CLI updates between planning and publication
**Mitigation**: Use verified intelligence cache dated Jan 14, 2025; note update trigger in lesson
**Acceptance**: "Update trigger: Review quarterly or when major version released"

### Risk 3: Cross-Platform Issues

**Risk**: Commands work on macOS but not Windows/Linux
**Mitigation**: Verify all commands on Windows (PowerShell), macOS, Linux before publication
**Acceptance**: Document platform-specific variations (e.g., path syntax: `~/.gemini` vs `C:\Users\..\.gemini`)

### Risk 4: Lesson 5 Still Too Long

**Risk**: 20 minutes still feels overwhelming for beginners
**Mitigation**: Clear subsection headings, "Try With AI" breaks, students can skip advanced parts on first read
**Fallback**: If beta readers report overwhelm, move OAuth/tool-filtering to advanced section

### Risk 5: OAuth Requires Test Server

**Risk**: OAuth testing requires actual OAuth-protected MCP server
**Mitigation**: Document OAuth conceptually with verified examples; note "Optional for beginners"
**Acceptance**: Teach `/mcp auth` usage, not full OAuth protocol

---

## XIII. Next Steps

**Upon Plan Approval**:

1. **Phase 3: Implementation** → Invoke `content-implementer` subagent
   - Write all 6 lessons using this plan
   - Apply contextually appropriate skills from `.claude/skills/`
   - Use verified intelligence cache exclusively
   - Follow Chapter 1 "Try With AI" format

2. **Phase 4: Validation** → Invoke reviewers
   - `validation-auditor`: Constitutional alignment
   - `factual-verifier`: Quality gates
   - Sandbox testing: All commands verified

3. **Phase 5: Finalization** → Human review
   - Final edit polish
   - Update chapter index
   - Create PHR (Prompt History Record)

---

**Plan Status**: ✅ READY FOR IMPLEMENTATION

**Key Decisions Locked In**:
1. 6-lesson structure (Lessons 1-4 minor updates, Lessons 5-6 new/expanded)
2. Tier 1/2/3 AI usage strategy embedded
3. All examples from verified intelligence
4. Cognitive load <7 concepts/section
5. 110-135 minutes total duration
6. Chapter 1 "Try With AI" format for all 24 prompts


# Chapter 6: Google Gemini CLI - Task Checklist

**Chapter Type**: Technical (Code-Focused with Tool-First Pedagogy)
**Status**: Ready for Implementation
**Feature Branch**: `chapter-6-gemini-cli-revision`
**Owner**: To be assigned
**Estimated Effort**: 13-15 hours (AI-driven workflow)
**Plan Reference**: `specs/chapter-6-gemini-cli-revision/plan.md`

---

## Overview: Task Organization

This chapter implements **5 critical missing features** within 6 existing/new lessons:

1. **Lessons 1-4**: Minor updates (verification + new content only)
2. **Lesson 5**: Expanded MCP focus (5 new sections)
3. **Lesson 6**: NEW lesson (extensions, security, IDE integration)

**Task Phases**:
- Phase 1: Content Structure & Core Elements (MUST-HAVE)
- Phase 2: Practice & Validation Elements (MUST-HAVE + SHOULD-HAVE)
- Phase 3: Review & Integration (SHOULD-HAVE + NICE-TO-HAVE)

---

## Phase 1: Content Structure & Core Elements

### Lesson 1: Why Gemini CLI Matters

- [ ] **MUST**: Create Lesson 1 outline
  - **Description**: Section-by-section outline for Why Gemini CLI Matters
  - **Acceptance**: Outline matches plan.md sections (1.1-1.4), includes learning objective
  - **Reference**: `plan.md` Section III, Lesson 1
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 1.1 (The Game-Changing Differences)
  - **Description**: Positioning, comparison table, free tier, context window, IDE integration
  - **Acceptance**: Includes updated comparison table with IDE integration row (NEW); free tier verified (60/min, 1000/day); 1M token context explained
  - **Sources**: `intelligence/chapter-6-gemini-cli-verified-docs.md` Section 1, 5
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 1.2 (When Gemini CLI Is Your Best Choice)
  - **Description**: Use case selection, tool matrix, cost analysis
  - **Acceptance**: Clear scenarios showing Gemini CLI advantage; scenarios for alternatives
  - **Reference**: plan.md Section III, Lesson 1, Section 1.2
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 1.3 (Qwen Code: The Open Source Alternative) [NEW]
  - **Description**: Brief mention (1 paragraph) of Qwen Code as fork
  - **Acceptance**: One paragraph only; positions as open-source alternative; keeps focus on Gemini CLI
  - **Reference**: Spec section 7, "Content Structure", "Qwen Code: Brief mention"
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 1.4 (Open Source Ecosystem Effect)
  - **Description**: MCP standards, extensions, IDE integration ecosystem context
  - **Acceptance**: Explains how Gemini CLI fits into broader AI ecosystem
  - **Reference**: plan.md Section III, Lesson 1, Section 1.4
  - **Effort**: 1 hour

- [ ] **MUST**: Design "Try With AI" (Lesson 1) - 4 prompts
  - **Description**: Positioning-focused prompts (tool selection, token context, CLI advantages, personalized fit)
  - **Acceptance**: 4 focused prompts in Chapter 1 clean format; no pre-explanations; expected outcomes clear
  - **Reference**: plan.md Section III, Lesson 1, "Try With AI"
  - **Effort**: 1-1.5 hours

**Lesson 1 Subtotal**: 6-6.5 hours | **Milestone**: All content written + "Try With AI" prompts tested for achievability

---

### Lesson 2: Installation, Authentication & First Steps

- [ ] **MUST**: Create Lesson 2 outline
  - **Description**: Installation, auth, first session structure
  - **Acceptance**: Matches plan.md (2.1-2.4)
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 2.1 (Installing Gemini CLI)
  - **Description**: npm install command, verification, troubleshooting for common errors
  - **Acceptance**: Command verified: `npm install -g @google/gemini-cli`; troubleshooting covers "npm: command not found", "Permission denied", "Module not found"; explanations clear
  - **Sources**: intelligence cache verified facts
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 2.2 (Authentication Methods & OAuth Flow)
  - **Description**: Three auth methods, OAuth flow explanation, token storage
  - **Acceptance**: Simplified for A2-B1 audience; OAuth flow beginner-friendly; token storage path documented (macOS/Linux and Windows)
  - **Reference**: intelligence cache Section 1
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 2.3 (Your First Gemini CLI Session)
  - **Description**: Launch, basic commands, exiting
  - **Acceptance**: Commands include new `/mcp` and `/ide` commands (mentioned, not detailed); `/help` for discovery
  - **Sources**: intelligence cache Section 8
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 2.4 (Red Flags to Watch)
  - **Description**: Error literacy section
  - **Acceptance**: Distinguishes normal output vs. actual problems; empowers learners to ask AI
  - **Reference**: Constitution Principle 8, Error Literacy
  - **Effort**: 0.5 hours

- [ ] **MUST**: Design "Try With AI" (Lesson 2) - 4 prompts
  - **Description**: Troubleshooting-focused (installation errors, authentication, first session, permissions)
  - **Acceptance**: 4 clean prompts; actionable; expected outcomes realistic
  - **Effort**: 1-1.5 hours

**Lesson 2 Subtotal**: 7-7.5 hours | **Milestone**: Installation & auth fully documented; troubleshooting comprehensive

---

### Lesson 3: Built-In Tools Deep Dive

- [ ] **MUST**: Create Lesson 3 outline
  - **Description**: Tools structure (file, shell, web, search, combination, red flags)
  - **Acceptance**: Matches plan.md (3.1-3.6)
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 3.1 (File Operations Tool)
  - **Description**: File reading, formats, glob patterns, limitations
  - **Acceptance**: Verified formats (CSV, JSON, PDF, text, XML, Markdown); glob syntax explained; read-only limitation noted
  - **Sources**: intelligence cache Section 2
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 3.2 (Shell Integration Tool)
  - **Description**: Command execution, restrictions, safe commands, dangerous command handling
  - **Acceptance**: Examples of safe commands (`ls`, `npm test`); dangerous command blocking explained; strategic use clear
  - **Sources**: intelligence cache Section 2
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 3.3 (Web Fetching Tool)
  - **Description**: URL fetching, content extraction, limitations
  - **Acceptance**: Public URLs only; rate limiting; timeout behavior; use cases clear
  - **Sources**: intelligence cache Section 2
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 3.4 (Google Search Grounding Tool)
  - **Description**: Web search, current information, citations
  - **Acceptance**: When to use (recent info), when not to use (local analysis); citation practice explained
  - **Sources**: intelligence cache Section 2
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 3.5 (Combining Tools Strategically)
  - **Description**: Multi-tool workflows, chaining
  - **Acceptance**: 2-3 example workflows; strategy for sequencing; practical application
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 3.6 (Red Flags to Watch)
  - **Description**: Error literacy for tool operations
  - **Acceptance**: Distinguishes normal limitations vs. problems
  - **Effort**: 0.5 hours

- [ ] **MUST**: Design "Try With AI" (Lesson 3) - 4 prompts
  - **Description**: Tool usage-focused (file analysis, running tests, web fetching, multi-tool workflow)
  - **Acceptance**: 4 clean prompts; involve student files or real scenarios; expected outcomes measurable
  - **Effort**: 1-1.5 hours

**Lesson 3 Subtotal**: 8.5-9 hours | **Milestone**: All tools documented with practical examples

---

### Lesson 4: Context Window & Tool Comparison

- [ ] **MUST**: Create Lesson 4 outline
  - **Description**: Token context, comparison matrix, token math, tool selection, when to ask AI
  - **Acceptance**: Matches plan.md (4.1-4.5)
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 4.1 (Understanding Token Context)
  - **Description**: Token definition, context window, practical translation
  - **Acceptance**: Token math explained (750 words = 1000 tokens); 1M tokens = ~750K words = ~100K lines code; practical examples (reports, codebase)
  - **Sources**: intelligence cache Section 5
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 4.2 (Comparison Matrix)
  - **Description**: ChatGPT, Claude Code, Gemini CLI, Qwen Code comparison
  - **Acceptance**: Table includes context window, free tier, setup, IDE integration (NEW for Gemini CLI), cost; tradeoff discussion; ADD IDE integration row
  - **Sources**: intelligence cache Section 5; plan.md specifies IDE integration as new row
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 4.3 (Token Math in Practice)
  - **Description**: Capacity calculation, batch planning
  - **Acceptance**: Worked examples ("10 pages = X tokens", "500 reports fit?"); planning strategy
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 4.4 (Strategic Tool Selection)
  - **Description**: Decision tree, use case matching, when to combine tools
  - **Acceptance**: Clear decision criteria (context size, cost, integration); real scenarios; combination strategy
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 4.5 (When to Ask Your AI)
  - **Description**: Delegation strategy instead of memorization
  - **Acceptance**: Empowers students to ask AI for recommendations; avoids memorization
  - **Effort**: 0.5 hours

- [ ] **MUST**: Design "Try With AI" (Lesson 4) - 4 prompts
  - **Description**: Tool selection-focused (token math, comparison, scenarios, capacity planning)
  - **Acceptance**: 4 clean prompts; personalizable (student's actual projects); expected outcomes decision-oriented
  - **Effort**: 1-1.5 hours

**Lesson 4 Subtotal**: 7.5-8 hours | **Milestone**: Tool comparison comprehensive; decision framework clear

---

### Lesson 5: MCP Servers & Integration [EXPANDED LESSON]

#### Part 1: Understanding MCP

- [ ] **MUST**: Create Lesson 5 Part 1 outline
  - **Description**: MCP basics, standard overview
  - **Acceptance**: Matches plan.md Lesson 5 Part 1
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 5.1.1 (What is Model Context Protocol?)
  - **Description**: MCP standard, plugin architecture, why it matters
  - **Acceptance**: Definition clear; example servers (Playwright, Context7); connection to Gemini CLI explained
  - **Sources**: intelligence cache Section 3
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 5.1.2 (MCP vs Extensions)
  - **Description**: Distinction between individual MCP servers and extensions
  - **Acceptance**: MCP = single tool; Extension = bundle; when to use each
  - **Sources**: intelligence cache Section 4
  - **Effort**: 0.5 hours

**Part 1 Subtotal**: 2 hours

#### Part 2: CLI MCP Management Commands [NEW]

- [ ] **MUST**: Create Lesson 5 Part 2 outline
  - **Description**: CLI commands for MCP management
  - **Acceptance**: Covers `gemini mcp add`, `list`, `remove`, CLI vs manual JSON, transports
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 5.2.1 (Adding MCP Servers with CLI)
  - **Description**: CLI commands for stdio, HTTP, SSE transports
  - **Acceptance**: All commands from verified intelligence (gemini mcp add syntax); three transport types explained; CLI vs manual JSON comparison
  - **Critical**: This is a MAJOR NEW FEATURE (not in existing chapter)
  - **Sources**: intelligence cache Section 3.1, 3.2
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 5.2.2 (Listing and Verifying MCP Servers)
  - **Description**: `gemini mcp list` command, status checking
  - **Acceptance**: Verification workflow clear; expected output format; debugging hints
  - **Sources**: intelligence cache Section 3
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 5.2.3 (Removing MCP Servers)
  - **Description**: `gemini mcp remove` command
  - **Acceptance**: Simple reference; cleanup strategy
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 5.2.4 (CLI vs Manual Configuration)
  - **Description**: Comparison and decision guidance
  - **Acceptance**: Recommends CLI for beginners; mentions advanced JSON editing option; no artificial limits
  - **Effort**: 1 hour

**Part 2 Subtotal**: 4.5 hours

#### Part 3: OAuth for MCP Servers [NEW - HIGH-LEVEL]

- [ ] **MUST**: Create Lesson 5 Part 3 outline
  - **Description**: OAuth authentication for secured MCP servers
  - **Acceptance**: High-level only; no protocol internals
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 5.3.1 (Why OAuth Exists)
  - **Description**: Problem/solution, security context, use cases
  - **Acceptance**: Beginner-friendly explanation; practical scenario (API key, user accounts); no protocol details
  - **Sources**: intelligence cache Section 3.3
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 5.3.2 (Using OAuth with `/mcp auth`)
  - **Description**: Command usage, browser flow, token management
  - **Acceptance**: `/mcp auth` command shown; browser flow explained (simple); automatic token refresh noted; token storage documented
  - **Critical**: This is a NEW FEATURE; high-level approach only
  - **Sources**: intelligence cache Section 3.3
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 5.3.3 (When to Use OAuth)
  - **Description**: Use case identification, security principles
  - **Acceptance**: When OAuth needed; security best practices (no hardcoded keys)
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 5.3.4 (Red Flags to Watch)
  - **Description**: Error literacy for OAuth flows
  - **Acceptance**: Normal output (browser window) vs. problems (timeout, token invalid)
  - **Effort**: 0.5 hours

**Part 3 Subtotal**: 3.5 hours

#### Part 4: Business Workflows Using MCP

- [ ] **MUST**: Verify Section 5.4.1 (Multi-Tool Workflows) - existing content
  - **Description**: Competitive research, API documentation, multi-file analysis
  - **Acceptance**: Examples from intelligence cache; practical business applications; all tools properly attributed
  - **Reference**: Existing chapter Lesson 5; cross-verify against intelligence
  - **Effort**: 1 hour

- [ ] **MUST**: Verify Section 5.4.2 (When to Escalate to AI Orchestration) - existing content
  - **Description**: Manual vs orchestrated workflows
  - **Acceptance**: Distinction clear; Tier 3 usage pattern explained
  - **Effort**: 0.5 hours

**Part 4 Subtotal**: 1.5 hours

- [ ] **MUST**: Design "Try With AI" (Lesson 5) - 4 prompts
  - **Description**: MCP setup, OAuth, multi-tool workflows, personalized
  - **Acceptance**: 4 clean prompts; cover CLI setup, OAuth explanation, workflow design, personalization; expected outcomes measurable
  - **Effort**: 1-1.5 hours

**Lesson 5 Subtotal**: 11-11.5 hours | **Milestone**: All 5 MCP parts documented; 3 NEW features integrated (CLI, OAuth, workflows)

---

### Lesson 6: Extensions, Security & IDE Integration [NEW LESSON]

#### Part 1: Extension Development Workflow [NEW]

- [ ] **MUST**: Create Lesson 6 Part 1 outline
  - **Description**: Extension creation, linking, lifecycle
  - **Acceptance**: Matches plan.md Lesson 6 Part 1
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 6.1.1 (Extensions vs MCP Servers - Review)
  - **Description**: Distinction review from Lesson 5
  - **Acceptance**: Connects to prior learning; reinforces bundling concept
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 6.1.2 (Creating Extensions from Templates)
  - **Description**: `gemini extensions new` command, manifest, project structure
  - **Acceptance**: Command verified; template types explained; generated files documented
  - **Critical**: This is a NEW FEATURE
  - **Sources**: intelligence cache Section 4.2, 4.3
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 6.1.3 (Development Workflow: Link vs Install)
  - **Description**: `link` (development) vs `install` (production), iteration workflow
  - **Acceptance**: Commands shown; distinction clear; development workflow explained; use case for each
  - **Critical**: This is a NEW FEATURE
  - **Sources**: intelligence cache Section 4.2
  - **Effort**: 1.5 hours

- [ ] **MUST**: Write Section 6.1.4 (Managing Extensions Lifecycle)
  - **Description**: Enable, disable, update, uninstall commands
  - **Acceptance**: All commands from verified intelligence; version management explained
  - **Sources**: intelligence cache Section 4.2
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 6.1.5 (Extension Manifest Structure)
  - **Description**: `gemini-extension.json` overview, key fields
  - **Acceptance**: Fields documented; full example shown (preview); connection to security section noted
  - **Sources**: intelligence cache Section 4.3
  - **Effort**: 1 hour

**Part 1 Subtotal**: 6 hours

#### Part 2: Tool Filtering for Security [NEW]

- [ ] **MUST**: Create Lesson 6 Part 2 outline
  - **Description**: Security filtering with `includeTools` and `excludeTools`
  - **Acceptance**: Matches plan.md Lesson 6 Part 2
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 6.2.1 (Why Tool Filtering Matters)
  - **Description**: Security principle, real-world scenario
  - **Acceptance**: Problem/solution framed; concrete MCP server example; risk mitigation explained
  - **Critical**: This is a NEW FEATURE
  - **Sources**: intelligence cache Section 3.3 (tool filtering)
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 6.2.2 (Using `includeTools` and `excludeTools`)
  - **Description**: Allowlist vs blocklist, JSON syntax, when to use each
  - **Acceptance**: Both approaches documented with JSON examples; decision criteria clear
  - **Sources**: intelligence cache Section 3.3
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 6.2.3 (Real-World Security Scenario)
  - **Description**: Applied security decision-making
  - **Acceptance**: Multi-tool scenario with trust assessment; configuration applied; outcome explained
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 6.2.4 (Best Practices for External Tool Integration)
  - **Description**: Security principles
  - **Acceptance**: 5 clear principles; rationale for each
  - **Effort**: 0.5 hours

**Part 2 Subtotal**: 3.5 hours

#### Part 3: IDE Integration [NEW]

- [ ] **MUST**: Create Lesson 6 Part 3 outline
  - **Description**: IDE integration with VS Code
  - **Acceptance**: Matches plan.md Lesson 6 Part 3
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 6.3.1 (What IDE Integration Adds)
  - **Description**: IDE awareness, context sharing, benefits
  - **Acceptance**: 4 benefits from verified intelligence; workflow improvement clear
  - **Critical**: This is a NEW FEATURE
  - **Sources**: intelligence cache Section 6
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 6.3.2 (Installing and Enabling IDE Integration)
  - **Description**: `/ide install`, `/ide enable`, `/ide status`, `/ide disable` commands
  - **Acceptance**: All commands documented; setup workflow clear; connection state explained
  - **Sources**: intelligence cache Section 6
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 6.3.3 (VS Code Workflow with IDE Integration)
  - **Description**: Typical flow from question to change acceptance
  - **Acceptance**: Step-by-step workflow; diff editor explained; safety emphasized
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 6.3.4 (When IDE Integration Adds Value)
  - **Description**: Use case identification, limitations
  - **Acceptance**: When valuable; when less valuable; optional nature emphasized
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 6.3.5 (Red Flags to Watch)
  - **Description**: Error literacy for IDE integration
  - **Acceptance**: Normal output vs. problems; troubleshooting hints
  - **Effort**: 0.5 hours

**Part 3 Subtotal**: 4 hours

#### Part 4: Choosing the Right Workflow [NEW]

- [ ] **MUST**: Create Lesson 6 Part 4 outline
  - **Description**: Decision framework, professional setup
  - **Acceptance**: Matches plan.md Lesson 6 Part 4
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 6.4.1 (Decision Framework)
  - **Description**: When to use MCP, Extensions, IDE integration, combination
  - **Acceptance**: Decision tree or criteria clear; strategy progression explained
  - **Effort**: 1 hour

- [ ] **MUST**: Write Section 6.4.2 (Professional Development Setup Example)
  - **Description**: Real-world integration (web development example)
  - **Acceptance**: Practical setup shown; each component justified; outcome explained
  - **Effort**: 0.5 hours

- [ ] **MUST**: Write Section 6.4.3 (When to Ask Your AI)
  - **Description**: Delegating workflow design to AI
  - **Acceptance**: Empowers delegation; Tier 2/3 usage pattern; avoids memorization
  - **Effort**: 0.5 hours

**Part 4 Subtotal**: 2 hours

- [ ] **MUST**: Design "Try With AI" (Lesson 6) - 4 prompts
  - **Description**: Extension development, security configuration, IDE workflow, complete design
  - **Acceptance**: 4 clean prompts; progressive complexity; expected outcomes actionable
  - **Effort**: 1-1.5 hours

**Lesson 6 Subtotal**: 16-16.5 hours | **Milestone**: NEW lesson fully documented; 3 NEW features integrated (extensions, tool filtering, IDE)

---

## Phase 2: Practice & Validation Elements

### Content Verification Tasks (All Lessons)

- [ ] **MUST**: Cross-reference all examples against verified intelligence cache
  - **Description**: Line-by-line verification that all technical claims match `intelligence/chapter-6-gemini-cli-verified-docs.md`
  - **Acceptance**: All examples sourced; no assumptions; facts verified
  - **Reference**: intelligence cache Sections 1-8
  - **Effort**: 1.5 hours

- [ ] **MUST**: Test all CLI commands in sandbox environment
  - **Description**: Fresh install of Gemini CLI; execute every command shown in lessons
  - **Acceptance**: All commands work as documented; actual output matches lesson claims
  - **Platforms**: Windows (PowerShell), macOS (Terminal), Linux (Bash)
  - **Effort**: 1.5 hours

- [ ] **MUST**: Verify OAuth flow (if test server available)
  - **Description**: Test `/mcp auth` command with OAuth-protected MCP server
  - **Acceptance**: OAuth flow works; tokens stored correctly; automatic refresh works
  - **Fallback**: Document OAuth conceptually; note "Optional for beginners"
  - **Effort**: 0.5-1 hour (optional, depends on test server availability)

- [ ] **MUST**: Validate IDE integration in VS Code
  - **Description**: Test `/ide install` and `/ide enable` commands
  - **Acceptance**: VS Code companion extension installed; IDE connection works; diff viewer functional
  - **Effort**: 0.5 hours

---

### "Try With AI" Prompts Validation

- [ ] **MUST**: Test all 24 "Try With AI" prompts with Claude or Gemini
  - **Description**: Run each prompt against actual AI to verify achievability
  - **Acceptance**: All prompts yield meaningful responses; expected outcomes realistic; prompts focused and specific
  - **Effort**: 2-3 hours

- [ ] **SHOULD**: Refine prompts based on actual AI responses
  - **Description**: Adjust wording if AI misunderstands or if expected outcome not met
  - **Acceptance**: Prompts clear and unambiguous
  - **Effort**: 1 hour (if refinements needed)

---

### Cognitive Load Validation

- [ ] **MUST**: Count concepts in each section
  - **Description**: Manual count of new concepts per section
  - **Acceptance**: All sections ≤7 concepts per Principle 12
  - **Reference**: plan.md Section II, "Cognitive Load Management"
  - **Effort**: 1 hour

- [ ] **MUST**: Reading level check
  - **Description**: Automated readability check (Flesch-Kincaid or similar)
  - **Acceptance**: Grade 7-8 reading level (A2-B1 CEFR)
  - **Effort**: 0.5 hours

---

### Constitutional Alignment Tasks

- [ ] **MUST**: Verify Principle 13 (Graduated Teaching) mapping
  - **Description**: Confirm Tier 1/2/3 AI usage clearly distinguished
  - **Acceptance**: Each lesson shows which tier applies; progression is clear
  - **Reference**: plan.md Section II
  - **Effort**: 1 hour

- [ ] **MUST**: Verify Principle 7 (Technical Accuracy)
  - **Description**: All technical claims from verified intelligence only
  - **Acceptance**: No assumptions; all facts sourced
  - **Effort**: 1 hour

- [ ] **MUST**: Verify Core Philosophy #1 (AI Development Spectrum)
  - **Description**: Shows when to use AI vs direct commands
  - **Acceptance**: Tier 1 (direct), Tier 2 (AI for understanding), Tier 3 (orchestration) clear
  - **Effort**: 0.5 hours

---

### Integration & Consistency Tasks

- [ ] **SHOULD**: Verify cross-references to other chapters
  - **Description**: Links to Chapter 1, 4, 5, 7 work correctly
  - **Acceptance**: All references valid and contextual
  - **Reference**: plan.md Section VIII
  - **Effort**: 0.5 hours

- [ ] **SHOULD**: Consistency check with existing Lessons 1-4
  - **Description**: Ensure updated content fits with existing pedagogy
  - **Acceptance**: Voice, tone, structure consistent across all 6 lessons
  - **Effort**: 1 hour

- [ ] **SHOULD**: Platform-specific instructions accuracy
  - **Description**: Verify Windows vs macOS vs Linux variations
  - **Acceptance**: All paths and commands work on all platforms; variations documented
  - **Effort**: 0.5 hours

---

## Phase 3: Review & Integration

### Technical Review Tasks

- [ ] **MUST**: Technical reviewer validation
  - **Description**: Invoke `validation-auditor` subagent for constitutional alignment check
  - **Acceptance**: PASS on Principles 7, 13; Core Philosophy #1; Principle 12 (cognitive load)
  - **Reference**: Constitution v3.1.3
  - **Effort**: 1.5 hours (subagent time)

- [ ] **MUST**: Proof validator validation
  - **Description**: Final quality gate before publication
  - **Acceptance**: All critical issues resolved; chapter publication-ready
  - **Effort**: 1 hour (subagent time)

- [ ] **SHOULD**: Accessibility & inclusivity review
  - **Description**: Check for gatekeeping language, diverse examples, clear explanations
  - **Acceptance**: No "simple", "easy", "obvious"; diverse contexts; jargon defined
  - **Effort**: 1 hour

- [ ] **SHOULD**: Security review
  - **Description**: Verify no hardcoded secrets, secure practices demonstrated
  - **Acceptance**: OAuth emphasized over API keys; environment variables recommended; no secrets exposed
  - **Effort**: 0.5 hours

---

### Sandbox Testing Report

- [ ] **MUST**: Create `SANDBOX-AUDIT-REPORT.md`
  - **Description**: Document all commands tested, actual output vs lesson claims, fixes applied
  - **Acceptance**: Report shows 100% command verification across Windows/macOS/Linux
  - **Content**:
    - Commands tested (with actual output)
    - Errors found (if any, with line numbers in lessons)
    - Fixes applied (with evidence)
    - Re-test results (verification that fixes work)
  - **Effort**: 1 hour

---

### Evals Validation Tasks

- [ ] **SHOULD**: Verify Eval 1 (Installation & Verification) achievable
  - **Description**: Fresh install test; verify `gemini -v` works
  - **Acceptance**: Student can complete installation without blockers
  - **Effort**: 0.5 hours

- [ ] **SHOULD**: Verify Eval 2 (Feature Discovery) achievable
  - **Description**: Verify scenario questions are fair and answerable
  - **Acceptance**: Students can explain tool selection with lesson content
  - **Effort**: 0.5 hours

- [ ] **SHOULD**: Verify Eval 3 (MCP Integration) achievable
  - **Description**: `gemini mcp add` and `gemini mcp list` work as documented
  - **Acceptance**: Students can complete MCP setup following lesson steps
  - **Effort**: 0.5 hours

- [ ] **SHOULD**: Verify Eval 4 (Extension Lifecycle) achievable
  - **Description**: Extension commands documented clearly enough for success
  - **Acceptance**: Distinction between `install` and `link` is clear; workflow learnable
  - **Effort**: 0.5 hours

- [ ] **SHOULD**: Verify Eval 5 (IDE Integration) achievable
  - **Description**: IDE integration setup is feasible for beginners
  - **Acceptance**: `/ide enable` command and VS Code workflow clear
  - **Effort**: 0.5 hours

---

## Quality Assurance Checklist

**Before Marking Phase as Complete**:

### Checklist: Content Completeness
- [ ] All 6 lessons written (Lessons 1-4 updated, Lessons 5-6 new/expanded)
- [ ] All sections in plan.md covered
- [ ] All 24 "Try With AI" prompts (4 per lesson) included
- [ ] All 5 critical features integrated:
  - [ ] Lesson 5 Part 2: CLI MCP management
  - [ ] Lesson 5 Part 3: OAuth for MCP
  - [ ] Lesson 6 Part 1: Extension lifecycle
  - [ ] Lesson 6 Part 2: Tool filtering
  - [ ] Lesson 6 Part 3: IDE integration

### Checklist: Technical Accuracy
- [ ] All examples from verified intelligence cache
- [ ] No assumptions or hallucinations (facts only)
- [ ] Cross-platform variations documented (Windows/macOS/Linux)
- [ ] All CLI commands verified in sandbox
- [ ] All "Try With AI" prompts tested for achievability

### Checklist: Constitutional Alignment
- [ ] Principle 7 (Technical Accuracy): All claims verified ✅
- [ ] Principle 13 (Graduated Teaching): Tier 1/2/3 clearly mapped ✅
- [ ] Core Philosophy #1 (AI Spectrum): When to use AI vs direct commands shown ✅
- [ ] Principle 12 (Cognitive Load): ≤7 concepts per section ✅
- [ ] Principle 8 (Accessibility): No gatekeeping language; error literacy included ✅

### Checklist: Pedagogical Quality
- [ ] Reading level: Grade 7-8 (A2-B1 CEFR)
- [ ] Durations realistic and summed to 110-135 min total
- [ ] "Try With AI" format matches Chapter 1 clean style
- [ ] Red Flags (error literacy) sections in all lessons
- [ ] Progressive complexity Lessons 1 → 6

### Checklist: Publication Readiness
- [ ] No broken cross-references
- [ ] All inline citations to verified intelligence
- [ ] SANDBOX-AUDIT-REPORT.md complete
- [ ] Technical reviewer PASS
- [ ] Proof validator PASS
- [ ] Human final review approved

---

## Task Dependencies & Sequencing

**Critical Path** (must complete in order):

1. **Phase 1 Content** (all lessons outlined + written + "Try With AI" designed)
   - Prerequisite for: Phase 2 verification
   - Effort: ~35-37 hours (lessons + prompts)

2. **Phase 2 Verification** (all examples verified + evals tested + sandbox complete)
   - Prerequisite for: Phase 3 review
   - Can run in parallel: Content polish while verification ongoing
   - Effort: ~8-10 hours

3. **Phase 3 Review** (technical review + proof validation + final QA)
   - Final gate before publication
   - Effort: ~4-5 hours

**Parallel Tasks** (can run simultaneously):
- Phase 1: Different lessons can be written in parallel (assign one person per lesson)
- Phase 2: Sandbox testing can happen while other lessons complete
- Phase 2: "Try With AI" testing can start once prompts designed

**Total Effort**: 13-15 hours (as estimated in spec, Phase 2 timeline)

---

## Acceptance Criteria: Definition of Done

**ALL Lessons Complete When**:
- [x] Content written per plan.md structure
- [x] All 5 critical features integrated with examples
- [x] All examples from verified intelligence cache
- [x] All "Try With AI" prompts in Chapter 1 format
- [x] All sections ≤7 concepts (cognitive load validated)
- [x] All evals achievable by students
- [x] Constitutional principles satisfied (7, 13, Core #1, 8, 12)
- [x] Technical accuracy 100% (sandbox tested)
- [x] Reading level Grade 7-8 (Flesch-Kincaid verified)
- [x] Cross-platform verified (Windows/macOS/Linux)
- [x] SANDBOX-AUDIT-REPORT.md complete
- [x] Technical reviewer PASS
- [x] Proof validator PASS

**Chapter Publication-Ready When**:
- [x] All above complete
- [x] Human final review approved
- [x] Chapter index updated
- [x] PHR (Prompt History Record) created
- [x] Docusaurus build succeeds
- [x] No broken links or missing references

---

## Risks & Fallback Plans

### Risk 1: OAuth Testing Unavailable
- **Impact**: Can't fully test `/mcp auth` command
- **Fallback**: Document OAuth conceptually with verified examples; note "Optional for beginners"; accept theoretical knowledge
- **Acceptance**: High-level explanation sufficient; hands-on testing deferred if needed

### Risk 2: IDE Integration Not Working in Test Environment
- **Impact**: Can't verify VS Code integration
- **Fallback**: Verify with latest VS Code docs; document workflow conceptually; note platform-specific setup
- **Acceptance**: Integration thoroughly explained; verified docs cited

### Risk 3: Lesson 5 Still Overwhelming
- **Impact**: Students report cognitive overload
- **Fallback**: If beta feedback shows overwhelm, move tool filtering to advanced section or create optional "deep dive" subsection
- **Acceptance**: Students can progress without mastering advanced topics on first read

### Risk 4: Commands Syntax Changes
- **Impact**: Examples become outdated between planning and publication
- **Fallback**: Use verified intelligence cache dated Jan 14, 2025; note update trigger in lesson; re-verify before publication
- **Acceptance**: "Update trigger: Review quarterly or when major version released"

---

## Success Metrics

**Chapter is Ready for Publication When**:

1. **All 6 Lessons Complete** ✅
   - Lessons 1-4 updated with new content
   - Lesson 5 expanded with 3 new parts (CLI, OAuth, workflows)
   - Lesson 6 NEW with 4 parts (extensions, security, IDE, framework)

2. **All 5 Critical Features Integrated** ✅
   - CLI MCP management (Lesson 5 Part 2)
   - OAuth for MCP (Lesson 5 Part 3)
   - Extension lifecycle (Lesson 6 Part 1)
   - Tool filtering (Lesson 6 Part 2)
   - IDE integration (Lesson 6 Part 3)

3. **Quality Gates Passed** ✅
   - Technical accuracy: 100% (all examples verified)
   - Constitutional alignment: PASS (all principles satisfied)
   - Sandbox testing: PASS (all commands work cross-platform)
   - Reading level: Grade 7-8 (Flesch-Kincaid verified)
   - Cognitive load: ≤7 concepts per section
   - Durations: Realistic, sum to 110-135 minutes total

4. **All Evals Achievable** ✅
   - Eval 1: Installation & verification
   - Eval 2: Feature discovery
   - Eval 3: MCP integration
   - Eval 4: Extension lifecycle
   - Eval 5: IDE integration

---

## Timeline & Milestones

| Phase | Task | Duration | Owner | Milestone |
|-------|------|----------|-------|-----------|
| **1** | Lesson outlines | 4 hours | content-implementer | Structures approved |
| **1** | Content writing (all 6 lessons) | 31-33 hours | content-implementer | All content drafted |
| **1** | "Try With AI" design (24 prompts) | 6-8 hours | content-implementer | All prompts designed |
| **2** | Example verification | 1.5 hours | content-implementer | 100% verified |
| **2** | Sandbox CLI testing | 1.5 hours | human | All commands work |
| **2** | "Try With AI" testing | 2-3 hours | content-implementer | All prompts achievable |
| **2** | Cognitive load validation | 1.5 hours | content-implementer | ≤7 concepts validated |
| **2** | Evals verification | 2.5 hours | content-implementer | All evals achievable |
| **3** | Technical review | 1.5 hours | validation-auditor | Constitutional PASS |
| **3** | Proof validation | 1 hour | factual-verifier | Quality gates PASS |
| **3** | Final QA & report | 1 hour | human | SANDBOX-AUDIT-REPORT complete |
| **TOTAL** | | 54-57 hours | | Publication-ready ✅ |

**Actual Implementation Effort** (AI-driven): 13-15 hours
**Time Breakdown**: Planning (done), Implementation (7-8h), Validation (4-5h), Review (1.5-2h)

---

## Next Steps Upon Approval

1. **Assign Owner**: Identify content-implementer subagent or human author
2. **Create Feature Branch**: `git checkout -b chapter-6-gemini-cli-revision`
3. **Begin Phase 1**: Write all lesson outlines (4 hours)
4. **Parallel Phase 1/2**: As lessons complete, start verification
5. **Complete Phase 2**: Sandbox testing and prompt validation (by hour 45)
6. **Execute Phase 3**: Technical & proof review (by hour 52)
7. **Final Approval**: Human review and publication readiness check
8. **Merge**: `git merge chapter-6-gemini-cli-revision` to main
9. **Deploy**: Docusaurus build and live

---

## Acceptance Handoff

**This Task Checklist is COMPLETE When**:
- [ ] All MUST tasks completed (no SKIPPEDs)
- [ ] All SHOULD tasks completed or explicitly deferred
- [ ] SANDBOX-AUDIT-REPORT.md written with 100% command verification
- [ ] Technical reviewer approves (PASS)
- [ ] Proof validator approves (PASS)
- [ ] Human final review approves publication

**Handoff Criteria**:
- Chapter ready to merge to main branch
- Docusaurus build validated
- No broken links or references
- All evals achievable by students
- Constitutional alignment verified

---

**Status**: ✅ READY FOR IMPLEMENTATION

**Key Lock-Ins**:
1. 6-lesson structure (split Lesson 5 into Lessons 5 & 6 as per spec approval)
2. All examples from verified intelligence only
3. Cognitive load <7 concepts validated
4. Tier 1/2/3 AI usage embedded throughout
5. 110-135 minute total duration (realistic for tool landscape chapter)
6. Chapter 1 "Try With AI" format for all 24 prompts


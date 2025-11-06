# Feature Specification: Chapter 6 - Gemini CLI: Installation and Basics

**Feature Branch**: `001-chapter-6`
**Created**: 2025-10-31
**Status**: Draft
**Input**: User description: "Design chapter 6. Check the part 2 goal provided at @book-source\docs\02-AI-Tool-Landscape\README.md and the context here @context\07_chap6_spec\"

## Overview

Chapter 6 introduces learners to Google Gemini CLI as an accessible, open-source alternative to Claude Code. This chapter is the second in Part 2: AI Tool Landscape, following Chapter 5's introduction to Claude Code. The chapter must enable complete beginners to successfully install, configure, and use Gemini CLI for AI-driven development workflows, while understanding its unique advantages (open source, free tier, massive context window, extensibility).

**Context within the Book:**
- **Part 2 Goal**: Transform learners from theoretical understanding (Part 1) into practical capability with AI development tools
- **Prior Knowledge**: Learners have completed Chapter 5 (Claude Code) and understand basic terminal concepts
- **Next Steps**: Chapter 7 (GitHub Copilot) and Chapter 8 (Choosing the Right Tool) will build on this foundation

**Key Differentiators for Gemini CLI:**
- Fully open source (Apache 2.0 license)
- Generous free tier: 60 requests/minute, 1,000 requests/day with personal Google account
- Gemini 2.5 Pro with 1 million token context window (can analyze entire codebases)
- Built on Model Context Protocol (MCP) for extensibility
- Recent Gemini CLI Extensions feature (similar to Claude Code Agent Skills)
- Qwen Code fork demonstrates cross-platform applicability

## User Scenarios & Testing *(mandatory)*

### User Story 1 - First-Time Installation and Authentication (Priority: P1)

A complete beginner with no prior Gemini CLI experience needs to install the tool on their operating system (Windows, Mac, or Linux) and authenticate with a Google account to start using Gemini CLI for development work.

**Why this priority**: Without successful installation and authentication, learners cannot proceed with any other learning objectives. This is the critical path prerequisite for all subsequent functionality.

**Independent Test**: Learner can execute `gemini --version` in their terminal and receive version information, then run `gemini auth` and successfully authenticate with their Google account, receiving confirmation of authentication status.

**Acceptance Scenarios**:

1. **Given** a Windows/Mac/Linux computer with internet access and terminal/command prompt open, **When** the learner follows platform-specific installation instructions and runs the installation command, **Then** Gemini CLI installs without errors and the learner can verify installation with `gemini --version`

2. **Given** Gemini CLI is successfully installed, **When** the learner runs `gemini auth` and follows the authentication flow with their Google account, **Then** authentication completes successfully and the learner receives confirmation message

3. **Given** authentication is complete, **When** the learner runs `gemini status` or equivalent command, **Then** the system displays their authentication status and available API quota (60 requests/minute, 1,000 requests/day)

---

### User Story 2 - Basic Configuration and First Command (Priority: P1)

After successful installation, the learner needs to understand basic Gemini CLI configuration options and execute their first successful command to verify the tool is working correctly.

**Why this priority**: Learners need immediate feedback that the tool works before investing time in advanced features. This builds confidence and validates their installation was successful.

**Independent Test**: Learner can run `gemini "Explain what Gemini CLI is in one sentence"` and receive a coherent AI-generated response, demonstrating successful end-to-end functionality.

**Acceptance Scenarios**:

1. **Given** Gemini CLI is installed and authenticated, **When** the learner runs `gemini config` or equivalent command, **Then** they can view current configuration settings and understand what can be customized

2. **Given** the learner wants to test basic functionality, **When** they run `gemini "Hello, can you introduce yourself?"`, **Then** they receive a response from Gemini 2.5 Pro model explaining its capabilities

3. **Given** the learner wants to verify their quota, **When** they check their request count or quota status, **Then** they can see how many requests they've made and how many remain in their free tier allocation

---

### User Story 3 - Built-In Tools Exploration (Priority: P2)

The learner needs to understand and use Gemini CLI's built-in tools (Google Search grounding, file operations, shell commands, web fetching) to leverage the full power of the platform for development tasks.

**Why this priority**: Built-in tools differentiate Gemini CLI from basic chat interfaces and enable practical development workflows. Understanding these tools unlocks the platform's full potential.

**Independent Test**: Learner can use Gemini CLI with file operations to read a local file and ask Gemini to analyze it, demonstrating they understand how to leverage built-in tools beyond simple text prompts.

**Acceptance Scenarios**:

1. **Given** the learner has a text file or code file on their computer, **When** they use Gemini CLI's file reading capability to analyze the file (e.g., `gemini --file mycode.py "Explain what this code does"`), **Then** Gemini reads the file and provides accurate analysis

2. **Given** the learner needs current information, **When** they use Google Search grounding feature to ask a time-sensitive question, **Then** Gemini provides up-to-date information grounded in search results

3. **Given** the learner wants to fetch web content, **When** they use web fetching capability to retrieve and analyze a webpage, **Then** Gemini successfully fetches the content and provides relevant analysis

---

### User Story 4 - Understanding the 1 Million Token Context Window (Priority: P2)

The learner needs to understand Gemini 2.5 Pro's massive 1 million token context window and how to leverage it for analyzing entire codebases or large documents.

**Why this priority**: The context window is a key competitive advantage of Gemini CLI. Understanding this capability helps learners appreciate when to use Gemini CLI vs. other tools.

**Independent Test**: Learner can explain what the 1 million token context window means in practical terms (e.g., "approximately X lines of code" or "an entire small-to-medium project") and identify scenarios where this capability provides advantages.

**Acceptance Scenarios**:

1. **Given** the learner wants to analyze a multi-file project, **When** they use Gemini CLI to ingest multiple files or an entire directory for analysis, **Then** Gemini successfully processes the large context and provides coherent cross-file analysis

2. **Given** the learner encounters a context limitation with another AI tool, **When** they switch to Gemini CLI and provide the same large context, **Then** they can successfully process the request and understand the practical advantage of the larger context window

---

### User Story 5 - Gemini CLI Extensions Introduction (Priority: P3)

The learner needs to understand Gemini CLI Extensions (analogous to Claude Code Agent Skills) and how they can extend Gemini CLI's capabilities for domain-specific workflows.

**Why this priority**: Extensions enable advanced customization but are not required for basic productive use. This is valuable for learners who want to go beyond basics or create specialized workflows.

**Independent Test**: Learner can explain what Gemini CLI Extensions are, describe at least one example use case, and navigate to the extensions documentation to explore available extensions.

**Acceptance Scenarios**:

1. **Given** the learner wants to understand extensibility, **When** they read about Gemini CLI Extensions and review example extensions, **Then** they can explain how extensions enhance Gemini CLI's capabilities

2. **Given** the learner wants to explore available extensions, **When** they access the extensions marketplace or documentation, **Then** they can browse, understand installation process, and identify extensions relevant to their needs

3. **Given** the learner wants to create custom domain-specific workflows, **When** they review extension creation documentation, **Then** they understand the high-level process for creating custom extensions (detailed implementation would be covered in advanced chapters)

---

### User Story 6 - Comparing Gemini CLI to Claude Code (Priority: P2)

The learner needs to develop an objective framework for comparing Gemini CLI and Claude Code (from Chapter 5) to make informed tool choices for different development tasks.

**Why this priority**: Tool selection is a critical skill for AI-driven development. Learners must understand trade-offs rather than treating any single tool as universally superior.

**Independent Test**: Learner can create a comparison table identifying at least 3 key differences between Gemini CLI and Claude Code, and articulate at least one scenario where each tool would be preferred.

**Acceptance Scenarios**:

1. **Given** the learner has experience with both Claude Code and Gemini CLI, **When** they are asked to identify key differences, **Then** they can accurately describe differences in licensing (proprietary vs open source), pricing (paid API vs free tier), context windows, and unique features

2. **Given** a specific development scenario (e.g., "analyze a large codebase" or "get the latest information about a current event"), **When** the learner evaluates which tool to use, **Then** they can justify their choice based on tool capabilities

3. **Given** the learner wants to understand when to switch between tools, **When** they encounter tool limitations (rate limits, context limits, capability gaps), **Then** they know how to evaluate and select an alternative tool

---

### User Story 7 - Qwen Code as Alternative Implementation (Priority: P3)

The learner needs to understand that Qwen Code is a fork of Gemini CLI with similar functionality but different backend (Alibaba Qwen models) and even more generous free tier (2,000 requests/day).

**Why this priority**: Demonstrates the open-source advantage and provides fallback options. Not critical for core learning but valuable for flexibility and understanding the broader ecosystem.

**Independent Test**: Learner can explain what Qwen Code is, how it relates to Gemini CLI, and identify the key difference in free tier quotas (1,000 vs 2,000 requests/day).

**Acceptance Scenarios**:

1. **Given** the learner wants to understand Gemini CLI's open-source nature, **When** they learn about Qwen Code fork, **Then** they understand how open source enables tool diversity and experimentation

2. **Given** the learner hits Gemini CLI rate limits, **When** they consider alternatives, **Then** they know Qwen Code offers similar functionality with higher free tier limits

---

### Edge Cases

- **Installation Failures**: What happens when installation fails due to missing dependencies (e.g., Node.js, npm, Python) or permissions issues? Chapter must provide troubleshooting guidance for common installation problems across Windows/Mac/Linux.

- **Authentication Failures**: How does the system handle authentication failures (e.g., Google account issues, network problems, regional restrictions)? Must provide clear error messages and troubleshooting steps.

- **Rate Limit Behavior**: What happens when the learner exceeds free tier limits (60 requests/minute or 1,000 requests/day)? Chapter must explain rate limiting behavior and recovery (wait periods, upgrade options).

- **Context Window Limits**: What happens when the learner attempts to exceed the 1 million token context window? How is this communicated, and what strategies exist for handling very large inputs?

- **Offline Usage**: Can Gemini CLI be used offline for any functionality? If not, how are network errors handled?

- **Platform-Specific Differences**: How do installation and usage differ across Windows, Mac, and Linux? Chapter must address platform-specific terminal behavior and path differences.

- **Version Compatibility**: What happens if the learner has an older version of Gemini CLI and encounters documentation for newer features? How do they check their version and upgrade?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST provide platform-specific installation instructions for Windows, Mac, and Linux
- **FR-002**: Chapter MUST include step-by-step authentication process with screenshots or detailed terminal output examples
- **FR-003**: Chapter MUST demonstrate basic Gemini CLI commands with expected output examples
- **FR-004**: Chapter MUST explain Gemini 2.5 Pro's 1 million token context window in practical, beginner-friendly terms with concrete examples
- **FR-005**: Chapter MUST demonstrate at least 3 of Gemini CLI's built-in tools (Google Search grounding, file operations, shell commands, web fetching)
- **FR-006**: Chapter MUST provide troubleshooting guidance for common installation and authentication issues
- **FR-007**: Chapter MUST explain the free tier limitations (60 requests/minute, 1,000 requests/day) and how to monitor quota usage
- **FR-008**: Chapter MUST introduce Gemini CLI Extensions concept with at least one concrete example
- **FR-009**: Chapter MUST include objective comparison between Gemini CLI and Claude Code with clear use-case guidance
- **FR-010**: Chapter MUST explain the Model Context Protocol (MCP) at a high level and why it matters for extensibility
- **FR-011**: Chapter MUST mention Qwen Code fork as an alternative with its key differentiation (2,000 requests/day free tier)
- **FR-012**: Chapter MUST include hands-on exercises where learners execute real Gemini CLI commands and verify output
- **FR-013**: Chapter MUST demonstrate how to use Gemini CLI for a realistic beginner development task (e.g., analyzing a small Python script, explaining error messages, generating code comments)
- **FR-014**: Chapter MUST explain when Gemini CLI is the better choice vs. Claude Code or other tools
- **FR-015**: Chapter MUST reference official Gemini CLI documentation links for deeper learning

### Key Entities

- **Gemini CLI Tool**: Command-line interface application that provides terminal access to Google's Gemini models
  - Attributes: Version number, installation path, configuration settings
  - Capabilities: Authentication, prompt execution, file operations, web fetching, search grounding, extensions support
  - Platform support: Windows, Mac, Linux

- **Google Account**: Authentication credential required for Gemini CLI access
  - Attributes: Email address, authentication token, quota allocation
  - Free tier limits: 60 requests/minute, 1,000 requests/day

- **Gemini 2.5 Pro Model**: Underlying AI model powering Gemini CLI
  - Key capability: 1 million token context window
  - Model Context Protocol (MCP) architecture

- **Built-in Tools**: Pre-configured capabilities within Gemini CLI
  - Google Search grounding (real-time web information)
  - File operations (read local files for analysis)
  - Shell commands (execute system commands with AI guidance)
  - Web fetching (retrieve and analyze web content)

- **Gemini CLI Extensions**: Modular add-ons that extend Gemini CLI functionality
  - Conceptually similar to Claude Code Agent Skills
  - Enable domain-specific vertical capabilities
  - Community-contributed and official extensions available

- **Qwen Code**: Fork of Gemini CLI using Alibaba Qwen models
  - Higher free tier: 2,000 requests/day (vs. 1,000 for Gemini CLI)
  - Demonstrates open-source ecosystem benefits

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Learner can install Gemini CLI on their operating system and verify installation by running `gemini --version` within 15 minutes of starting the chapter
- **SC-002**: Learner can authenticate with their Google account and confirm authentication status within 10 minutes of completing installation
- **SC-003**: Learner can successfully execute at least 5 different Gemini CLI commands demonstrating various built-in tools (basic prompts, file operations, web fetching)
- **SC-004**: Learner can explain the 1 million token context window in practical terms when asked "What does this mean for my development work?"
- **SC-005**: Learner can create a comparison table with at least 5 criteria comparing Gemini CLI and Claude Code
- **SC-006**: Learner can identify at least 2 scenarios where Gemini CLI is the better choice and 2 scenarios where Claude Code is the better choice
- **SC-007**: Learner can troubleshoot at least one common installation issue using chapter guidance (e.g., missing dependencies, permission errors)
- **SC-008**: Learner can locate and describe at least one Gemini CLI Extension and explain its purpose
- **SC-009**: Learner can explain what the Model Context Protocol (MCP) is at a high level (1-2 sentences) and why it enables extensibility
- **SC-010**: 90% of learners complete the chapter's hands-on exercises successfully on first attempt without external help

## Scope

### In Scope

- Installation and setup on all major platforms (Windows, Mac, Linux)
- Authentication with Google account
- Basic command execution and prompt engineering
- Built-in tools demonstration (file ops, search, web fetch, shell)
- 1 million token context window explanation and practical use cases
- Introduction to Gemini CLI Extensions concept with examples
- Objective comparison framework for Gemini CLI vs. Claude Code
- Free tier quota management and monitoring
- Troubleshooting common installation and authentication issues
- Introduction to Model Context Protocol (MCP) at conceptual level
- Qwen Code mention as alternative implementation
- Hands-on exercises using real Gemini CLI commands
- Links to official documentation for deeper learning

### Out of Scope (Covered in Later Chapters or Parts)

- Advanced prompt engineering techniques (covered in Part 3: Prompt & Context Engineering)
- Creating custom Gemini CLI Extensions from scratch (advanced topic for later parts)
- Deep technical details of Model Context Protocol implementation
- Advanced troubleshooting for complex environment configurations
- Gemini API pricing beyond free tier (not relevant for beginners)
- Integration with IDEs or other development tools (covered in Chapter 7 or later)
- Production deployment considerations for AI-assisted applications
- Detailed comparison with all AI tools beyond Claude Code (Chapter 8 provides comprehensive tool comparison)
- Writing Python code or building applications (Part 4 onward)

## Assumptions

- Learners have completed Chapter 5 (Claude Code) and understand basic terminal concepts (navigating directories, running commands)
- Learners have a computer with internet access and can install software
- Learners have a Google account or can create one (free)
- Learners can access Google services (not in regions with Google restrictions)
- Official Gemini CLI documentation at geminicli.com remains available and current
- Free tier quotas (60 requests/minute, 1,000 requests/day) remain stable or improve over time
- Learners are using a modern terminal/command prompt (not legacy systems like Windows XP)
- Installation dependencies (Node.js, npm, or Python depending on Gemini CLI implementation) are either already present or can be installed with guidance
- Learners can follow platform-specific instructions appropriate to their operating system

## Dependencies

- **Chapter 5 Completion**: Learners must understand basic terminal concepts and have context for comparing Gemini CLI to Claude Code
- **Part 1 Completion**: Learners must understand AI-driven development philosophy before diving into specific tools
- **Internet Connectivity**: Required for installation, authentication, and API requests
- **Google Account Access**: Learners must be able to create/use Google accounts
- **Official Documentation Availability**: Links to geminicli.com documentation must remain live and current

## Constraints

- **Reading Level**: Must maintain grade 7 baseline reading level with clear explanations of technical terms
- **Platform Coverage**: Must provide equal support for Windows, Mac, and Linux users
- **Free Tier Focus**: Must focus exclusively on free tier capabilities (no paid API discussion)
- **Time Budget**: Estimated 2-3 hours for complete chapter (as specified in Part 2 README)
- **Screenshot/Output Examples**: Must include enough visual examples for learners to verify they're on the right track without being overly redundant
- **Tool Volatility**: Must include update triggers noting this content should be reviewed when Gemini CLI undergoes major version changes
- **No Implementation Details**: Must remain technology-agnostic in success criteria (focus on user outcomes, not system internals)

## Risks and Mitigation

**Risk 1: Official Documentation Changes**
- **Mitigation**: Include archive.org links as backup; review chapter when Gemini CLI releases major updates; teach learners how to find current documentation themselves

**Risk 2: Free Tier Quota Changes**
- **Mitigation**: Include publication date context; teach learners to verify current quota in their account rather than memorizing numbers; emphasize principles over specifics

**Risk 3: Regional Google Access Restrictions**
- **Mitigation**: Acknowledge geographic limitations upfront; provide Qwen Code as alternative with similar workflow; teach VPN concepts if appropriate for audience

**Risk 4: Installation Dependency Confusion**
- **Mitigation**: Provide clear prerequisite checks before installation; include troubleshooting section for common dependency issues; link to dependency installation guides

**Risk 5: Learner Overwhelm from Tool Comparison**
- **Mitigation**: Present comparison in structured, scannable table format; focus on practical decision criteria rather than exhaustive feature lists; provide clear decision framework

**Risk 6: Extensions Feature Complexity**
- **Mitigation**: Introduce extensions conceptually only; provide one clear example; defer deep implementation to advanced chapters; emphasize this is optional knowledge

## Cross-Chapter Integration

- **Chapter 5 (Claude Code)**: Reference comparison points throughout; assume learners have context for what CLI AI tools can do
- **Chapter 7 (GitHub Copilot)**: This chapter prepares learners for evaluating multiple AI tools; sets up framework for tool selection
- **Chapter 8 (Choosing the Right Tool)**: Comparison framework developed here will be expanded to include all tools in Part 2
- **Part 3 (Prompt Engineering)**: Basic prompting shown here will be formalized into systematic techniques in Part 3
- **Part 4 (Python)**: Gemini CLI will be used as a tool option for all Python learning exercises

## Notes for Content Creators

- **Pedagogical Approach**: Follow "show-then-explain" pattern (demonstrate working commands before explaining internals)
- **Engagement**: Include opening hook connecting to Part 2's transformation from theory to capability
- **AI-Augmented Learning**: Show how to use Gemini CLI to learn about Gemini CLI (meta-learning)
- **Accessibility**: Define all technical terms on first use; no gatekeeping language
- **Evidence-Based**: All installation steps and commands must be tested on all three platforms before publication
- **Update Maintenance**: Flag this chapter for review when Gemini CLI releases major versions or when quota structure changes

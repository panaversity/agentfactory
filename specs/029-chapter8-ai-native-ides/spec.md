# Feature Specification: Chapter 8 - AI-Native IDEs

**Feature Branch**: `029-chapter8-ai-native-ides`
**Created**: 2025-11-20
**Status**: Draft
**Input**: User description: "We need a new Chapter for AI-Native IDEs. The summary of this chapter is available. The context for antigravity google's IDE is available. Use context7 mcp server for research and then create specs."

## Overview

This specification defines the learning objectives, content structure, and pedagogical approach for Chapter 8: "AI-Native IDEs - Zed, Cursor, Antigravity, and the Future of Development" in Part 2: AI Tool Landscape. This chapter introduces students to modern development environments designed from the ground up for AI collaboration, covering three distinct approaches: Zed (performance-focused native), Cursor (VS Code evolution), and Antigravity (agent control plane architecture).

**Educational Context**:
- **Part**: Part 2 (AI Tool Landscape)
- **Proficiency Level**: A2-B1 (Beginner to Intermediate)
- **Prerequisites**: Chapter 5 (Claude Code), Chapter 6 (Gemini CLI - optional), Chapter 7 (Bash Essentials)
- **Pedagogical Layer**: Layer 1 (Manual Foundation) transitioning to Layer 2 (AI Collaboration)
- **Student Knowledge State**: Students have CLI proficiency, understand AI collaboration via Claude Code/Gemini, but have not yet used AI-integrated code editors

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students can explain what makes an IDE "AI-native" in 2-3 sentences without referencing specific brands (tests conceptual understanding independent of tool)
- **SC-002**: 90% of students successfully install and authenticate at least one AI-native IDE (Zed, Cursor, or Antigravity) on their system within 30 minutes of starting installation lesson
- **SC-003**: Students can guide AI to complete a simple coding task (e.g., "write a function to calculate factorial") in under 10 minutes using natural language prompts, demonstrating basic IDE proficiency through AI collaboration
- **SC-004**: Given three project scenarios with different characteristics (e.g., performance-critical, team-based, agent-driven), students select appropriate IDE (from Zed, Cursor, Antigravity) and justify with at least two specific criteria
- **SC-005**: Students complete the "Try With AI" capstone activity by observing AI implement the same feature (temperature converter) in 2-3 different IDEs and produce a structured comparison (300+ words) analyzing AI output differences, IDE interaction patterns, response speed, and code quality
- **SC-006**: By chapter end, students can configure at least one advanced IDE feature (multi-model setup, custom rules, or local model integration) beyond basic installation
- **SC-007**: Students demonstrate critical evaluation of AI suggestions by identifying at least one instance where AI-generated code needed correction/improvement during practice exercises
- **SC-008**: 80% of students report increased confidence in using AI-integrated development tools (measured via end-of-chapter self-assessment survey)

### Qualitative Outcomes

- Students develop informed preferences for IDE selection based on experience rather than hype or brand loyalty
- Students understand that tool choice is contextual—no single "best" IDE exists for all scenarios
- Students experience the paradigm shift from "writing all code manually" to "collaborating with AI to write code faster" in a familiar editor context (vs CLI-only tools from earlier chapters)
- Students gain vocabulary to discuss AI-native development tools critically with peers and in professional contexts

## Learning Objectives

By the end of this chapter, students will be able to:

1. **LO-001** (Conceptual): Explain the architectural distinction between AI-native IDEs and traditional editors with AI plugins, citing at least three key differences (maps to SC-001)
2. **LO-002** (Installation): Install and authenticate an AI-native IDE (Zed, Cursor, or Antigravity) on their system and verify functionality with a simple AI-assisted task (maps to SC-002, SC-003)
3. **LO-003** (Feature Usage): Use at least three AI collaboration features (inline assistant, tab autocomplete, git integration, multi-model selection, agent mode) within their chosen IDE (maps to SC-003, SC-006)
4. **LO-004** (Critical Evaluation): Apply selection criteria (performance, AI features, collaboration, ecosystem, agent architecture) to choose appropriate IDE for different project scenarios (maps to SC-004)
5. **LO-005** (Comparative Analysis): Compare AI-native IDEs through hands-on implementation of the same task in multiple tools (minimum 2 of 3: Zed, Cursor, Antigravity), documenting workflow differences (maps to SC-005)
6. **LO-006** (Integration): Integrate AI-native IDE with existing development toolchain (git, terminal, formatters) without disrupting established workflows (maps to SC-006)
7. **LO-007** (Critical Thinking): Evaluate AI-generated code suggestions for correctness, security, and quality, understanding when to accept/modify/reject AI output (maps to SC-007, SC-008)

## Clarifications

### Session 2025-11-20

- Q: How should Google Antigravity be integrated into Chapter 8 given its public launch on Nov 18, 2025? → A: Add Antigravity as third major IDE alongside Zed and Cursor (Option C - comprehensive coverage of all three distinct AI-native approaches)
- Q: How should the 3-IDE structure be organized across lessons? → A: Sequential deep-dive approach (Option B - Lessons 2-3: Zed, Lessons 4-5: Cursor, Lessons 6-7: Antigravity, Lesson 8: Comparative capstone)
- Q: What should the capstone activity focus on given students have NO programming experience yet? → A: Observational comparison - students provide identical prompt to all 3 IDEs ("Create temperature converter Celsius/Fahrenheit"), compare AI-generated outputs, response speed, code structure, understandability (no manual coding required)
- Q: How should Success Criteria be adjusted to reflect the observational (non-coding) approach? → A: Update SC-005 only - change "implement feature" to "observe AI implement feature" focusing comparison on AI output differences, IDE interaction patterns, critical evaluation (300+ words structured analysis)
- Q: Given Antigravity launched Nov 18, 2025, should we update the "risk" assumption about it going public? → A: Update Assumptions & Risks - change Assumption #7 to reflect public availability (free download as of Nov 2025), remove/reframe Risk section about public launch

## User Scenarios & Testing

### User Story 1 - Understanding What Makes an IDE "AI-Native" (Priority: P1)

A student who has only used traditional text editors (VS Code, Sublime) or Claude Code CLI needs to understand the conceptual difference between AI-native IDEs and traditional editors with AI plugins bolted on. They should be able to explain why architecture matters and identify key characteristics that distinguish AI-native tools.

**Why this priority**: Foundational understanding required before any hands-on work. Without this mental model, students cannot make informed tool selection decisions or understand trade-offs.

**Independent Test**: Student can explain (in writing or verbally) three architectural differences between AI-native IDEs and traditional editors, and provide one concrete example of how this affects their workflow.

**Acceptance Scenarios**:

1. **Given** a student has read the conceptual lesson, **When** asked to define "AI-native IDE", **Then** they articulate that it's "an editor architecturally designed for AI collaboration from inception" (not just plugins added later)
2. **Given** examples of Cursor, Zed, and VS Code with Copilot, **When** asked to categorize them, **Then** student correctly identifies Cursor and Zed as AI-native, VS Code + Copilot as plugin-based
3. **Given** a scenario describing inline AI suggestions integrated with git workflows, **When** asked to identify the benefit of native integration, **Then** student explains reduced context switching and tighter feedback loops

---

### User Story 2 - Installing and Configuring Zed IDE (Priority: P2)

A student needs to successfully install Zed IDE on their system (macOS, Linux, or Windows), authenticate with their preferred AI provider (Anthropic, OpenAI, Google, or Ollama), configure basic settings, and verify the installation by completing a simple AI-assisted coding task.

**Why this priority**: Hands-on installation provides concrete experience with one AI-native IDE. Zed chosen for speed, simplicity, and official Anthropic backing. This establishes baseline competency.

**Independent Test**: Student completes installation, demonstrates working AI assistant by generating a "Hello World" function using Zed's inline assistant, and shows their settings configuration file.

**Acceptance Scenarios**:

1. **Given** platform-specific installation instructions, **When** student follows steps for their OS, **Then** Zed launches successfully and displays welcome screen
2. **Given** API key configuration instructions, **When** student adds their Anthropic/OpenAI/Google API key to settings, **Then** AI assistant responds to prompts without authentication errors
3. **Given** a blank Python file, **When** student uses inline assistant (Ctrl/Cmd+I) to request "write a function that prints hello world", **Then** Zed generates working code with AI attribution visible
4. **Given** Zed's settings.json file, **When** student modifies theme and AI model preferences, **Then** changes persist across restarts

---

### User Story 3 - Exploring Zed's AI Features and Workflows (Priority: P3)

A student with Zed installed needs to experience core AI collaboration features: inline assistant, multi-model selection, tab autocomplete, AI-powered refactoring, and git commit message generation. They should understand when to use each feature and how they integrate into coding workflows.

**Why this priority**: Moves beyond installation to practical usage patterns. Students learn the "vocabulary" of AI-native editing before comparing tools.

**Independent Test**: Student completes a guided mini-project (e.g., "Build a temperature converter CLI") using at least three different Zed AI features, documenting which features they used and why.

**Acceptance Scenarios**:

1. **Given** an existing Python function with repetitive code, **When** student highlights the code and requests AI refactoring, **Then** Zed suggests DRY (Don't Repeat Yourself) improvements inline
2. **Given** multiple AI models configured (e.g., Claude Sonnet and GPT-4), **When** student toggles between models during inline assist, **Then** they observe different suggestion styles and can articulate differences
3. **Given** uncommitted changes in a git repository, **When** student requests AI commit message generation, **Then** Zed analyzes diffs and suggests contextually appropriate commit message
4. **Given** typing `def calc_` in a Python file, **When** Zed's tab autocomplete activates, **Then** student sees ghost text suggestions and can accept/reject/partially accept them

---

### User Story 4 - Installing and Comparing Cursor IDE (Priority: P4)

A student already familiar with Zed needs to install Cursor IDE, understand its VS Code heritage, configure its unique features (Chat mode, Agent mode, .cursorrules), and compare its approach to AI collaboration against Zed's philosophy.

**Why this priority**: Provides contrast and teaches tool selection criteria. Cursor represents "AI-native evolution of familiar tool" while Zed represents "built from scratch for AI." Both valid, different trade-offs.

**Independent Test**: Student installs Cursor, completes the same mini-project from Story 3 using Cursor instead, and writes a 200-word comparison highlighting one strength of each tool.

**Acceptance Scenarios**:

1. **Given** Cursor installation package, **When** student installs on their system, **Then** Cursor launches successfully and imports their VS Code settings/extensions
2. **Given** a `.cursorrules` file in project root, **When** student writes custom AI instructions (e.g., "always use TypeScript strict mode"), **Then** Cursor's AI follows those rules in generated code
3. **Given** Cursor's Chat pane (Ctrl/Cmd+L), **When** student asks "explain this function" while highlighting code, **Then** Chat provides contextual explanation with references to codebase
4. **Given** Cursor's Agent mode, **When** student requests "add error handling to all API calls", **Then** Agent proposes multi-file changes in diff view before applying
5. **Given** the same coding task in both Zed and Cursor, **When** student reflects on experience, **Then** they identify at least one scenario where each tool excels

---

### User Story 5 - Applying Selection Criteria to Choose the Right IDE (Priority: P5)

A student preparing to start a new project needs a framework for choosing between AI-native IDEs based on objective criteria: performance requirements, AI model preferences, collaboration needs, extension ecosystem, and learning curve. They should be able to justify their choice with specific reasoning.

**Why this priority**: Metacognitive skill - teaches critical evaluation rather than "one tool fits all" thinking. Prepares students for professional decision-making.

**Independent Test**: Given three hypothetical project scenarios (e.g., "solo Python data science project", "team web development with React", "embedded systems programming"), student selects appropriate IDE for each and justifies with 2-3 specific criteria.

**Acceptance Scenarios**:

1. **Given** a comparison matrix of Zed, Cursor, and traditional VS Code, **When** student evaluates based on startup time, **Then** they correctly rank Zed > Cursor > VS Code and explain why this matters for certain workflows
2. **Given** a project requiring local AI models (no cloud API), **When** student reviews IDE capabilities, **Then** they identify which IDEs support Ollama integration (Zed, Cursor via custom config)
3. **Given** a team project requiring real-time collaboration, **When** student assesses collaboration features, **Then** they note Zed's built-in multiplayer vs Cursor/VS Code's Live Share extensions
4. **Given** preference for Claude Sonnet 4 as primary model, **When** evaluating IDE AI integrations, **Then** student confirms both Zed and Cursor support it natively but via different configuration methods

---

### User Story 6 - Integrating AI-Native IDE with Existing Toolchain (Priority: P6)

A student with established workflows (git, terminal, linters, formatters, testing frameworks) needs to integrate an AI-native IDE without disrupting existing tools. They should configure IDE to work with external tools and understand when to use IDE-integrated vs standalone tools.

**Why this priority**: Real-world adoption challenge. Students often have existing setups and need migration/integration strategies, not "start from scratch."

**Independent Test**: Student demonstrates AI-native IDE working seamlessly with: (1) existing git workflow, (2) external terminal tasks, (3) at least one language-specific tool (e.g., Black formatter for Python, Prettier for JavaScript).

**Acceptance Scenarios**:

1. **Given** an existing git repository, **When** student opens it in Zed/Cursor, **Then** IDE detects git status, shows branch info, and integrates with existing commit history
2. **Given** Zed's integrated terminal, **When** student runs npm/pip/cargo commands, **Then** commands execute in project context and IDE reflects changes (e.g., new dependencies)
3. **Given** a Python project with Black formatter configured, **When** student saves a file in Zed, **Then** format-on-save respects existing .black config without manual reconfiguration
4. **Given** VS Code extensions student previously used, **When** switching to Cursor, **Then** student identifies which extensions transfer directly vs which require Cursor-specific alternatives

---

### User Story 7 - Hands-On IDE Comparison Activity (Priority: P7)

A student completing the chapter needs a capstone "Try With AI" activity where they implement the same small feature in multiple IDEs, document their experience, and reflect on how AI assistance differs across tools. This consolidates learning and builds personal preference grounded in experience.

**Why this priority**: Experiential learning finale. Transforms conceptual knowledge into embodied understanding through direct comparison.

**Independent Test**: Student completes the "Try With AI" challenge: implement a feature (e.g., "Add input validation to a form") in both Zed and Cursor, documents workflow differences, and writes a 300-word reflection on which tool they prefer for different scenarios.

**Acceptance Scenarios**:

1. **Given** a starter codebase with incomplete feature, **When** student implements feature in Zed using inline assistant, **Then** they document time taken, number of AI iterations, and subjective experience (1-5 scale)
2. **Given** the same task in Cursor, **When** student uses Cursor's Agent mode for implementation, **Then** they compare agent-driven vs inline-driven workflows and note differences
3. **Given** both implementations complete, **When** student reviews code quality, **Then** they evaluate differences (if any) in AI-generated code patterns between IDEs
4. **Given** reflection prompt, **When** student writes comparison, **Then** they reference specific features (not vague impressions) and identify one scenario where each tool excels

---

### Edge Cases

- **What happens when a student's platform isn't supported?** (e.g., Zed on older Windows versions): Provide alternative paths using Cursor (broader platform support) or web-based options, with explanation that not all tools support all platforms equally.

- **How does the chapter handle students without API keys/credits?** Include instructions for free tiers (Anthropic/OpenAI free trials) and local model alternatives (Ollama integration) so lack of budget doesn't block learning.

- **What if a student's existing VS Code setup conflicts with Cursor?** Provide troubleshooting section on isolating Cursor settings, managing extension conflicts, and when to use separate profiles.

- **How does the chapter address performance issues on low-spec machines?** Include system requirements upfront, discuss lightweight configurations (disabling certain AI features), and provide guidance on optimizing for available hardware.

- **What happens when AI models produce incorrect/unsafe code?** Explicitly teach critical evaluation: "AI suggestions are starting points, not gospel." Include examples of reviewing AI output, testing thoroughly, and understanding when to override suggestions.

- **How are accessibility considerations addressed?** Ensure lessons include keyboard shortcuts, discuss screen reader compatibility, and note that AI assistance can help developers with disabilities write code more efficiently.

## Requirements

### Functional Requirements

#### Conceptual Understanding

- **FR-001**: Chapter MUST explain the architectural distinction between AI-native IDEs (designed for AI collaboration from inception) and traditional IDEs with AI plugins (retrofitted capabilities)
- **FR-002**: Chapter MUST define at least three key characteristics of AI-native IDEs (e.g., context-aware completions, multi-model support, agent-driven workflows, integrated artifact review)
- **FR-003**: Students MUST be able to articulate trade-offs between different AI-native IDE approaches (e.g., performance vs features, local vs cloud models, multiplayer vs single-user)

#### Installation & Configuration

- **FR-004**: Chapter MUST provide platform-specific installation instructions for Zed (macOS, Linux, Windows)
- **FR-005**: Chapter MUST provide platform-specific installation instructions for Cursor (macOS, Linux, Windows)
- **FR-006**: Chapter MUST include authentication setup for at least three AI providers (Anthropic, OpenAI, Google)
- **FR-007**: Chapter MUST demonstrate local model configuration using Ollama for students preferring offline/private AI
- **FR-008**: Installation lessons MUST include verification steps (e.g., "test AI assistant with simple prompt to confirm setup")

#### Feature Exploration (Zed)

- **FR-009**: Chapter MUST demonstrate Zed's inline assistant with concrete examples (code generation, refactoring, explanation)
- **FR-010**: Chapter MUST explain multi-model configuration in Zed (assigning different models to different features: inline assist, commit messages, thread summaries)
- **FR-011**: Chapter MUST show tab autocomplete functionality with accept/reject/partial-accept interactions
- **FR-012**: Chapter MUST demonstrate AI-powered git commit message generation from diff analysis
- **FR-013**: Chapter MUST explain Zed's extension system and how to install language-specific tooling

#### Feature Exploration (Cursor)

- **FR-014**: Chapter MUST demonstrate Cursor's Chat mode (Ctrl/Cmd+L) with codebase-aware queries
- **FR-015**: Chapter MUST explain Cursor's Agent mode and when to use agent-driven vs manual workflows
- **FR-016**: Chapter MUST show `.cursorrules` configuration for project-specific AI instructions
- **FR-017**: Chapter MUST demonstrate Cursor's diff-based change review before applying AI suggestions
- **FR-018**: Chapter MUST explain how Cursor inherits VS Code settings/extensions and potential migration paths

#### Comparison & Selection

- **FR-019**: Chapter MUST provide a decision matrix comparing Zed, Cursor, and traditional VS Code across dimensions: performance, AI features, collaboration, extension ecosystem, learning curve
- **FR-020**: Students MUST practice applying selection criteria to hypothetical project scenarios
- **FR-021**: Chapter MUST acknowledge Google's Anti-gravity IDE (from context research) as emerging alternative, noting its current beta/limited availability status

#### Integration

- **FR-022**: Chapter MUST demonstrate git integration (viewing diffs, staging, committing) within AI-native IDEs
- **FR-023**: Chapter MUST show integrated terminal usage for running build tools, package managers, and tests
- **FR-024**: Chapter MUST explain format-on-save configuration respecting existing formatter configs (Black, Prettier, rustfmt)
- **FR-025**: Chapter MUST address extension/plugin compatibility when migrating from VS Code to Cursor

#### Hands-On Practice

- **FR-026**: Chapter MUST include a "Try With AI" capstone activity requiring students to implement the same feature in multiple IDEs
- **FR-027**: Practice activity MUST provide starter code, clear feature requirements, and structured reflection prompts
- **FR-028**: Chapter MUST include mini-exercises throughout (not just final capstone) for incremental skill building

### Key Entities

**Note**: This is educational content, not a software system, but we define key "entities" as conceptual building blocks students must understand.

- **AI-Native IDE**: Development environment architecturally designed for AI collaboration from inception; characterized by context-aware AI, multi-model support, integrated artifacts, and agent capabilities
  - **Attributes**: Name, platform support, AI providers, collaboration features, extension ecosystem, performance profile
  - **Examples**: Zed, Cursor, Anti-gravity (emerging)

- **AI Provider Configuration**: Connection setup between IDE and AI service
  - **Attributes**: Provider name (Anthropic, OpenAI, Google, Ollama), API key/auth method, model selection, rate limits
  - **Relationships**: One IDE supports multiple providers; one provider may offer multiple models

- **IDE Feature**: Specific AI-assisted capability within the IDE
  - **Attributes**: Feature name (inline assistant, chat mode, agent mode, tab autocomplete), activation method (keyboard shortcut), model assignment
  - **Relationships**: Features map to specific models in multi-model configurations

- **Selection Criterion**: Dimension for evaluating IDE fitness for a project
  - **Attributes**: Criterion name (performance, AI features, collaboration, ecosystem), measurement method, importance weighting
  - **Relationships**: Different projects prioritize different criteria

- **Lesson**: Instructional unit within the chapter
  - **Attributes**: Lesson number, title, learning objectives, estimated time, prerequisites
  - **Relationships**: Lessons build sequentially; later lessons reference earlier concepts

## Assumptions

1. **Platform Distribution**: Assume majority of students use macOS (40%), Windows (40%), Linux (20%) based on typical developer demographics; provide instructions for all three
2. **Prior Tool Experience**: Assume most students have used at least one traditional code editor (VS Code, Sublime, Atom) even if not professionally
3. **API Access**: Assume students can obtain free-tier API keys from at least one major provider (Anthropic, OpenAI, Google) or are willing to use local models via Ollama
4. **Internet Connectivity**: Assume stable internet for cloud-based AI models during practice; provide offline alternatives for bandwidth-constrained scenarios
5. **Hardware Baseline**: Assume minimum 8GB RAM, modern CPU (within 5 years) for acceptable performance; note lighter configurations for lower-spec machines
6. **Learning Pace**: Assume chapter completion in 6-8 hours across one week for typical student (increased from 4-6 hours due to third IDE); faster for those with strong existing IDE experience
7. **Antigravity Availability**: Google Antigravity publicly launched November 18, 2025, with free download for macOS, Linux, and Windows; assume stable availability throughout chapter lifespan though as newly launched product may have more frequent updates/changes than mature Zed/Cursor
8. **Non-Programmer Audience**: Students have ZERO programming experience at this stage (Part 2, before Python in Part 4); chapter focuses on observing and evaluating AI-generated code rather than manual coding

## Dependencies

### Prerequisite Knowledge (from earlier chapters)

- **Chapter 5 (Claude Code)**: Understanding of AI collaboration via natural language prompts, experience with subagents/skills
- **Chapter 7 (Bash Essentials)**: Command-line proficiency for installing tools, running scripts, managing environments
- **Optional Chapter 6 (Gemini CLI)**: Exposure to alternative AI tools builds comparative mindset

### External Tools/Services

- **AI Providers**: Anthropic API, OpenAI API, Google AI API, or Ollama (local models)
- **Package Managers**: Homebrew (macOS), apt/dnf (Linux), winget/chocolatey (Windows) for installation
- **Git**: Required for demonstrating IDE git integrations (assumed installed from Chapter 7 context)

### System Requirements

- **Zed**: macOS 10.15+, Linux (glibc 2.29+), Windows 10+ (Windows support improving but less mature as of 2025)
- **Cursor**: macOS 10.11+, Linux (Ubuntu 18.04+), Windows 10+
- **Disk Space**: ~500MB per IDE, plus additional for extensions/models
- **Network**: Broadband recommended for cloud AI; offline possible with Ollama

## Constraints

### Technical Constraints

- **Platform Limitations**: Zed's Windows support is newer and may have fewer features than macOS/Linux versions; acknowledge in lessons
- **API Costs**: Cloud AI providers charge per token; students must manage usage to stay within free tiers or budget
- **Model Availability**: Specific models (e.g., Claude Sonnet 4, GPT-4) may require paid plans; provide free-tier alternatives (Claude Haiku, GPT-3.5)
- **Extension Ecosystem**: Cursor benefits from VS Code extension compatibility; Zed has smaller (but growing) extension library—acknowledge trade-off

### Pedagogical Constraints

- **Cognitive Load**: A2-B1 proficiency means limiting to 2-3 main IDEs (Zed, Cursor, VS Code reference) rather than overwhelming with 10+ tools
- **Time Budget**: Chapter should fit within 4-6 hours; cannot provide exhaustive coverage of every IDE feature
- **Hands-On Priority**: Must balance conceptual understanding with practical exercises; favor concrete practice over theoretical deep-dives
- **Layer 1→Layer 2 Transition**: Students are learning manual IDE operation (Layer 1) while simultaneously using AI collaboration features (Layer 2)—requires careful scaffolding

### Content Constraints

- **Anti-gravity Limited Access**: Can reference Google's Anti-gravity from research context but cannot provide hands-on tutorial due to beta restrictions
- **Fast-Moving Tools**: IDE features evolve rapidly (especially AI capabilities); content must focus on enduring concepts (architecture, selection criteria) alongside current feature snapshots
- **Brand Neutrality**: Avoid appearing as promotional material for specific vendors; maintain critical, comparative stance
- **Accessibility**: Must work for students using screen readers, keyboard-only navigation; ensure all workflows have non-mouse alternatives

## Out of Scope

The following topics are explicitly excluded from this chapter:

### Advanced IDE Features
- **Plugin/Extension Development**: Creating custom Zed/Cursor extensions requires separate advanced tutorial
- **LSP Server Configuration**: Low-level Language Server Protocol setup better suited for advanced IDE customization chapter
- **Regex Search/Replace Mastery**: Power-user IDE features beyond beginner scope
- **Vim Motions/Modal Editing**: Zed supports Vim mode but teaching modal editing is orthogonal to AI-native concepts

### Alternative AI Development Tools
- **Windsurf**: Another emerging AI-native IDE; excluded to limit scope (Zed + Cursor sufficient for comparison)
- **JetBrains AI Assistant**: Plugin for IntelliJ/PyCharm; not AI-native architecture
- **GitHub Copilot in VS Code**: Mentioned as contrast but not hands-on tutorial (plugin vs native architecture)
- **Replit/Codespaces**: Cloud IDEs with AI features; different category (cloud-first vs local-first)

### Deep AI Customization
- **Fine-tuning Models**: Training custom AI models for code completion out of scope
- **Prompt Engineering for IDEs**: Basic usage covered, advanced prompt optimization separate topic
- **Custom AI Providers**: Building your own AI provider integration requires SDK knowledge

### Production Workflows
- **CI/CD Integration**: Connecting IDE to deployment pipelines separate DevOps topic
- **Docker/Container Development**: IDE support for containers mentioned but not tutorial focus
- **Remote Development**: SSH/remote server editing separate from local AI-native focus
- **Team Collaboration Workflows**: Multiplayer/Live Share features mentioned but not deep team workflow tutorial

### Language-Specific Deep Dives
- **Language Ecosystem Setup**: Chapter shows installation, not comprehensive TypeScript/Python/Rust toolchain setup
- **Debugger Mastery**: Using IDE debuggers separate from AI collaboration features
- **Test Framework Integration**: IDE test runners mentioned but not TDD tutorial

### Non-IDE AI Tools
- **Specialized AI Tools**: Tools like Cursor's Agent mode vs autonomous agents (e.g., AutoGPT) are different categories
- **AI Code Review Services**: SaaS products for code review (e.g., CodeRabbit) separate from IDE features

## Risks & Mitigations

### Risk: Rapid Tool Evolution Outdates Content
- **Likelihood**: High
- **Impact**: Medium (screenshots/specific features change, core concepts remain valid)
- **Mitigation**: Focus lessons on enduring architectural concepts and selection criteria rather than ephemeral UI details; use versioned screenshots with "as of [date]" notation; maintain errata document for known changes

### Risk: Students Cannot Afford API Credits
- **Likelihood**: Medium
- **Impact**: High (blocks hands-on practice for budget-constrained learners)
- **Mitigation**: Prioritize free-tier providers; provide comprehensive Ollama (local model) setup as first-class alternative, not afterthought; include cost estimation guidance upfront

### Risk: Platform-Specific Installation Failures
- **Likelihood**: Medium (especially Windows for Zed)
- **Impact**: High (blocked students cannot proceed)
- **Mitigation**: Include detailed troubleshooting sections per platform; provide alternative installation methods (binary download vs package manager); create video walkthroughs for common failure scenarios; establish community support channel for installation help

### Risk: Students Overwhelmed by Tool Choices
- **Likelihood**: High (increased with third IDE)
- **Impact**: Medium (decision paralysis delays learning)
- **Mitigation**: Provide clear decision flowchart based on student priorities (speed→Zed, familiarity→Cursor, agent-driven→Antigravity); use sequential deep-dive structure (master one IDE before next) rather than parallel comparison; normalize that choosing "wrong" tool isn't permanent; emphasize skills transfer between tools

### Risk: Antigravity Instability as Newly Launched Product
- **Likelihood**: Medium (launched Nov 18, 2025, only 2 days before spec writing)
- **Impact**: Medium (bugs, frequent updates, breaking changes could disrupt lessons)
- **Mitigation**: Design Antigravity lessons with version-agnostic concepts; focus on agent architecture principles over specific UI details; provide troubleshooting guidance for common early-adopter issues; maintain errata document for breaking changes; consider making Antigravity optional if severe stability issues emerge

### Risk: AI-Generated Code Contains Security Vulnerabilities
- **Likelihood**: Medium (inherent to AI code generation)
- **Impact**: High (students learn unsafe practices)
- **Mitigation**: Include explicit "Critical Evaluation" sidebar in every hands-on lesson emphasizing AI suggestions are starting points; require code review step in practice exercises; provide examples of common AI mistakes (SQL injection, XSS) and how to spot them

### Risk: Accessibility Barriers for Non-Visual Users
- **Likelihood**: Low-Medium (depends on student population)
- **Impact**: High (excludes certain learners entirely)
- **Mitigation**: Ensure all workflows document keyboard shortcuts; test lessons with screen reader; provide text descriptions for all screenshots/visual examples; verify AI chat/inline features work with assistive tech

### Risk: Students Skip Conceptual Lessons, Jump to Installation
- **Likelihood**: High (common student behavior)
- **Impact**: Medium (shallow tool usage without understanding trade-offs)
- **Mitigation**: Design installation lessons to fail gracefully without conceptual foundation (e.g., "If you're unsure which AI provider to choose, review Lesson 1 section on model selection"); include reflection prompts in hands-on exercises that require conceptual knowledge; make capstone "Try With AI" explicitly require comparative analysis

## Success Metrics (Post-Launch)

These metrics would be collected after the chapter is published to validate effectiveness:

### Completion Metrics
- **Completion Rate**: % of students who start Chapter 8 and complete final "Try With AI" activity (target: >75%)
- **Time to Complete**: Median time from chapter start to finish (target: 4-6 hours)
- **Drop-off Points**: Identify specific lessons where students abandon chapter (investigate high drop-off lessons)

### Learning Outcomes
- **Conceptual Quiz Score**: Average score on "What Makes an IDE AI-Native?" assessment (target: >80% correct)
- **Installation Success Rate**: % of students who successfully install at least one IDE (target: >90%)
- **Selection Criteria Application**: % of capstone submissions that correctly apply 2+ selection criteria (target: >85%)

### Engagement Metrics
- **Hands-On Exercise Completion**: % of students who complete optional mini-exercises vs only mandatory capstone (target: >60% do optionals)
- **Tool Adoption**: Survey of which IDE students continue using 1 month post-chapter (goal: >50% still using Zed or Cursor)
- **Community Discussions**: Number of student forum posts sharing IDE tips/preferences (higher = better engagement)

### Quality Indicators
- **Error Report Rate**: Number of content errors/outdated info reported per 100 students (target: <5)
- **Accessibility Feedback**: Student reports of accessibility barriers (target: 0 blockers, <3 friction points)
- **Student Satisfaction**: Net Promoter Score for chapter (target: >40)

## Related Work

- **Chapter 5 (Claude Code)**: Establishes AI collaboration via CLI; Chapter 8 extends to IDE context
- **Chapter 9 (Git & GitHub)**: Git workflows referenced in IDE integration lessons
- **Part 4 (Python Fundamentals)**: Students will apply Chapter 8 IDE skills to Python projects in later chapters
- **Part 6 (AI-Native Software Development)**: Advanced AI-assisted development patterns build on IDE proficiency from Chapter 8

## Appendix: Research Sources

### Context7 MCP Research Conducted

1. **Zed IDE** (`/zed-industries/zed` and `/websites/zed_dev`):
   - AI feature configuration (multi-model setup, inline assistant, commit messages)
   - Installation methods (Homebrew, package managers)
   - Local collaboration setup (LiveKit, PostgreSQL)
   - Extension system and language support
   - Performance characteristics (Rust-based, GPU-accelerated)

2. **Cursor IDE** (`/getcursor/docs`):
   - Project-specific AI rules (`.cursorrules`)
   - Chat mode (Ctrl/Cmd+L) and long context chat
   - Tab completion and partial accepts
   - AI Review feature
   - Diff-based change review
   - VS Code compatibility

3. **Anti-gravity** (from `context/16_chapter8__ides/antigravity.md`):
   - Google DeepMind product in beta
   - Agent-assisted development paradigm
   - Three surfaces: Agent Manager, Editor, Integrated Browser
   - Artifact system (task lists, implementation plans, walkthroughs)
   - Parallel task execution
   - Context-aware editing
   - Noted as emerging tool with limited public access

### Additional Research Needed (for lesson implementation)

- Platform-specific installation edge cases and troubleshooting
- Free-tier limits and pricing for Anthropic/OpenAI/Google APIs as of 2025
- Ollama model recommendations for code generation (CodeLlama, DeepSeek Coder, etc.)
- Accessibility testing results for Zed and Cursor with screen readers
- Performance benchmarks (startup time, memory usage) on representative hardware

## Notes for Implementation

1. **Lesson Breakdown (Sequential Deep-Dive)**: Restructured to 8 lessons covering 3 IDEs:
   - Lesson 1: AI-Native Concepts (conceptual foundation)
   - Lesson 2: Installing Zed IDE (first hands-on experience)
   - Lesson 3: Zed AI Features & Workflows (master Zed deeply)
   - Lesson 4: Installing Cursor IDE (second IDE introduction)
   - Lesson 5: Cursor AI Features & Workflows (master Cursor deeply, compare with Zed)
   - Lesson 6: Installing Antigravity IDE (third IDE, newest/most cutting-edge)
   - Lesson 7: Antigravity Agent Architecture & Features (agent control plane paradigm)
   - Lesson 8: Comparative Capstone - "Try With AI" (observational comparison across all 3)

2. **Antigravity Treatment**: Full hands-on coverage as third major IDE (publicly launched Nov 18, 2025); emphasize agent-first architecture as distinct from Zed (performance) and Cursor (familiarity); acknowledge newness and potential instability

3. **Three Roles Framework (Layer 2)**: Emphasize in Lessons 3, 5, and 7 that students are collaborating WITH AI (student teaches AI via `.cursorrules` in Cursor, Antigravity's agent surfaces, AI teaches student better patterns, convergence toward better code)—framework must remain invisible, experienced through action

4. **Assessment Strategy**: Lesson 1 includes conceptual quiz; Lessons 2-7 include mini hands-on exercises focusing on AI prompting (NOT manual coding); Lesson 8 is observational capstone where students compare AI-generated outputs across IDEs; consider end-of-chapter reflection survey for qualitative feedback

5. **Visuals Needed**: Screenshots of Zed/Cursor/Antigravity interfaces, decision flowchart for IDE selection (3-way comparison), comparison matrix table (3 columns), step-by-step installation sequences for all 3 IDEs (consider video alternatives)

6. **Observational Learning Approach**: Since students have ZERO programming experience, all "coding" activities involve students providing natural language prompts to AI and observing/evaluating outputs; focus on critical thinking about AI capabilities, not manual code writing

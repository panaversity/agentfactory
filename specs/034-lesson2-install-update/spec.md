# Feature Specification: Update Chapter 5 Lesson 2 Claude Code Installation

**Feature Branch**: `034-lesson2-install-update`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Update Chapter 5 Lesson 2 with new simplified Claude Code installation methods (platform-specific recommendations)"

## Context

**Current State**: Chapter 5 Lesson 2 (`apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md`) contains outdated Claude Code installation instructions that primarily focus on npm installation and present 4 methods without platform-specific guidance.

**Problem**:

- Installation methods are outdated (npm-centric approach no longer primary)
- No platform-specific recommendations (students see all 4 methods regardless of OS)
- Cognitive load violation for B1 learners (6+ installation options presented simultaneously)
- New official installation methods are simpler (curl/bash, Homebrew, PowerShell) and don't require Node.js

**Source of Truth**: Official Claude Code documentation at https://code.claude.com/docs/en/setup

## User Scenarios & Testing _(mandatory)_

### User Story 1 - Windows Student Installation (Priority: P1)

A Windows student reads the lesson and needs to install Claude Code on their system using the recommended method for Windows.

**Why this priority**: Windows is a primary platform for students, and incorrect platform guidance causes installation failures and support burden.

**Independent Test**: Student on Windows 10+ can identify Windows section, follow PowerShell installation steps, verify with `claude --version`, and complete authentication without consulting external documentation.

**Acceptance Scenarios**:

1. **Given** student is on Windows 10+ with PowerShell, **When** they follow the recommended PowerShell installation method, **Then** Claude Code installs successfully and `claude --version` shows version number
2. **Given** student is on Windows with Git Bash, **When** they see PowerShell isn't available and check alternative methods, **Then** they find curl/bash method for Git Bash clearly documented
3. **Given** student encounters installation failure, **When** they check troubleshooting guidance, **Then** they find platform-specific troubleshooting steps (not generic npm errors)

---

### User Story 2 - macOS Student Installation (Priority: P1)

A macOS student reads the lesson and needs to install Claude Code using the recommended method for macOS.

**Why this priority**: macOS is a primary development platform, and Homebrew is the de facto package manager for macOS developers.

**Independent Test**: Student on macOS 10.15+ can identify macOS section, choose between Homebrew (if installed) or curl/bash, complete installation, and verify working setup.

**Acceptance Scenarios**:

1. **Given** student has Homebrew installed on macOS, **When** they follow the Homebrew installation method, **Then** Claude Code installs via `brew install --cask claude-code` and is immediately available
2. **Given** student does not have Homebrew, **When** they use the curl/bash method, **Then** installation completes without requiring Homebrew installation first
3. **Given** student wants to understand which method to choose, **When** they read the decision guidance, **Then** they see clear recommendation: "Have Homebrew? Use it. Otherwise use curl/bash"

---

### User Story 3 - Linux/WSL Student Installation (Priority: P1)

A Linux or WSL student reads the lesson and needs to install Claude Code using the recommended method for their environment.

**Why this priority**: Linux and WSL are common development environments, especially for students following cloud-native curriculum.

**Independent Test**: Student on Ubuntu 20.04+/Debian 10+/WSL can identify Linux section, use curl/bash installation, and verify working setup.

**Acceptance Scenarios**:

1. **Given** student is on WSL 1 or WSL 2, **When** they follow the Linux/WSL installation section, **Then** they use the same curl/bash method as native Linux
2. **Given** student is on native Linux, **When** they execute the curl installation command, **Then** Claude Code installs to their user directory without requiring sudo
3. **Given** student wants to verify installation type, **When** they run `claude doctor`, **Then** they see installation method, version, and system compatibility

---

### User Story 4 - Cross-Platform npm Alternative (Priority: P2)

A student on any platform already has Node.js 18+ installed and prefers npm-based tools.

**Why this priority**: npm installation is still valid but no longer the primary recommendation. Students familiar with npm should still have this option.

**Independent Test**: Student with Node.js 18+ can find npm installation as an alternative method on any platform and successfully install using npm.

**Acceptance Scenarios**:

1. **Given** student has Node.js 18+ installed, **When** they check installation methods for their platform, **Then** they see npm listed as an alternative (not primary) method
2. **Given** student uses `npm install -g @anthropic-ai/claude-code`, **When** installation completes, **Then** they see warning NOT to use `sudo npm install -g` (security best practice)
3. **Given** student encounters npm permission errors, **When** they check troubleshooting, **Then** they find guidance on fixing npm permissions without sudo

---

### User Story 5 - Authentication Setup (Priority: P1)

A student completes installation and needs to authenticate Claude Code using one of three authentication methods.

**Why this priority**: Installation without authentication is incomplete. Students need clear guidance on which authentication path matches their access (Console API, Claude.ai Pro/Max, or Enterprise).

**Independent Test**: Student can identify which authentication method applies to them (Console API, Claude.ai subscription, or Enterprise), complete the authentication flow, and verify with a test command.

**Acceptance Scenarios**:

1. **Given** student has Claude Console account with billing, **When** they run `claude` for first time and select Console option, **Then** OAuth flow completes and creates "Claude Code" workspace for usage tracking
2. **Given** student has Claude.ai Pro or Max subscription, **When** they run `claude` and select Claude App option, **Then** they authenticate with Claude.ai credentials and can use unified subscription
3. **Given** student needs to verify authentication, **When** they run `claude "Hello! Can you confirm Claude Code is working?"`, **Then** Claude responds, confirming authentication success

---

### User Story 6 - Lesson Summary Update (Priority: P2)

A student uses the lesson summary file to quickly review installation steps without reading the full lesson.

**Why this priority**: Summary files enable quick reference and review. Outdated summaries cause confusion when they contradict the main lesson.

**Independent Test**: Summary file (`02-installation-and-authentication.md.summary.md`) accurately reflects the updated installation methods and can be used standalone for quick review.

**Acceptance Scenarios**:

1. **Given** student opens the summary file, **When** they scan the installation section, **Then** they see platform-specific recommendations matching the main lesson
2. **Given** student used the summary for installation, **When** they encounter issues, **Then** they know to check the full lesson for detailed troubleshooting
3. **Given** summary file is updated, **When** compared to main lesson, **Then** key steps, platform recommendations, and authentication paths are consistent

---

### Edge Cases

- **What happens when student is on unsupported OS (e.g., Alpine Linux)?** Lesson includes "Special Considerations" section mentioning Alpine Linux requirements (`libgcc`, `libstdc++`, `ripgrep`) and environment variable (`USE_BUILTIN_RIPGREP=0`)
- **What if student has Node.js < 18 and tries npm installation?** Lesson prerequisites section clearly states "Node.js 18+ (only required for NPM installation)" so students without it will use platform-native methods
- **What if student is on Windows but doesn't have PowerShell or Git Bash?** Lesson documents Windows CMD method as third option for Windows
- **How does student know which authentication method to choose if they have multiple accounts?** Decision tree provided: "Have Console account with billing? Use Console. Have Pro/Max subscription? Use Claude App. Enterprise setup? Use third-party integrations."
- **What if student's country isn't in Anthropic supported countries list?** Prerequisites section mentions "Location: Anthropic supported countries" with link to check eligibility
- **What happens when auto-updates download but student wants to disable them?** Lesson documents `export DISABLE_AUTOUPDATER=1` and settings.json configuration

## Requirements _(mandatory)_

### Functional Requirements

- **FR-001**: Lesson MUST organize installation methods by platform (Windows, macOS, Linux/WSL) with clear visual hierarchy
- **FR-002**: Each platform section MUST present recommended installation method FIRST, followed by alternative methods
- **FR-003**: Windows section MUST document three installation methods in priority order: (1) PowerShell, (2) curl/bash (Git Bash), (3) CMD, (4) npm
- **FR-004**: macOS section MUST document three installation methods in priority order: (1) Homebrew, (2) curl/bash, (3) npm
- **FR-005**: Linux/WSL section MUST document two installation methods in priority order: (1) curl/bash, (2) npm
- **FR-006**: Each installation method MUST include exact command with code syntax highlighting
- **FR-007**: Each installation method MUST explain "What this does" in plain language for B1 comprehension
- **FR-008**: Lesson MUST include verification step (`claude --version`) after installation instructions
- **FR-009**: Authentication section MUST document three authentication paths: (1) Claude Console (default), (2) Claude App (Pro/Max), (3) Enterprise platforms
- **FR-010**: Each authentication path MUST include decision criteria ("Use this if...") to help students choose
- **FR-011**: Lesson MUST remove outdated npm-centric language (e.g., "Method 4: npm (Cross-Platform)" becomes "npm Alternative (Cross-Platform)")
- **FR-012**: Lesson MUST add auto-update documentation (update checks, automatic downloads, disabling auto-updates)
- **FR-013**: Lesson MUST update "Try With AI" section prompts to reflect new installation methods
- **FR-014**: Summary file (`02-installation-and-authentication.md.summary.md`) MUST be updated to match main lesson structure and content
- **FR-015**: Lesson MUST maintain existing pedagogical layer metadata (Layer 1 foundation, B1 proficiency, learning objectives)
- **FR-016**: Lesson MUST preserve existing "Why This Matters" and "Prerequisites" sections (update only if installation prerequisites changed)
- **FR-017**: Lesson MUST include platform-specific troubleshooting guidance in "Security and Best Practices" or new troubleshooting section
- **FR-018**: All installation commands MUST be tested for accuracy against official documentation (https://code.claude.com/docs/en/setup)

### Content Quality Requirements

- **CQR-001**: Platform-specific recommendations MUST reduce cognitive load (max 3 options per platform vs current 4 options globally)
- **CQR-002**: Decision trees MUST use clear visual format (e.g., "Do you have Homebrew? ├─ Yes → Use Homebrew │ └─ No → Use curl/bash")
- **CQR-003**: Installation commands MUST be copy-paste ready (no placeholder values requiring student editing)
- **CQR-004**: "What this does" explanations MUST avoid jargon and use B1-appropriate vocabulary
- **CQR-005**: Authentication decision criteria MUST be unambiguous (no student should wonder which auth method applies to them)

### Key Entities _(lesson content structure)_

- **Installation Method Block**: Consists of platform name (heading), recommended badge, command block, "What this does" explanation, expected output (if applicable)
- **Authentication Path**: Consists of auth method name, decision criteria ("Use this if..."), command/flow steps, verification command, expected output
- **Platform Decision Tree**: Visual hierarchy showing: Platform identification → Recommended method → Alternative methods → Troubleshooting
- **Verification Step**: Standard pattern of command + expected output + troubleshooting if output doesn't match

## Success Criteria _(mandatory)_

### Measurable Outcomes

- **SC-001**: Students can identify their platform and recommended installation method within 30 seconds of reading the installation section
- **SC-002**: 90% of students successfully install Claude Code on first attempt using platform-specific recommended method (measured via lesson completion surveys)
- **SC-003**: Installation support requests decrease by 60% due to clearer platform-specific guidance (measured via Discord/support channel analysis)
- **SC-004**: Students complete installation and authentication within 10 minutes (down from current 15+ minutes with npm-centric approach)
- **SC-005**: Summary file can be used standalone for quick installation (verified by external reviewer following summary-only instructions)
- **SC-006**: All installation commands execute successfully on target platforms: Windows 10+, macOS 10.15+, Ubuntu 20.04+/Debian 10+, WSL 1/2
- **SC-007**: Zero factual inaccuracies when compared to official Claude Code documentation (verified via factual-verifier agent)
- **SC-008**: Cognitive load remains within B1 limits (8 concepts per section max, currently measured at 8 in metadata, must not increase)

### Qualitative Outcomes

- **SC-009**: Student confidence increases ("I know which method is right for MY system" vs "I'll try all methods until one works")
- **SC-010**: Lesson maintains pedagogical consistency with Chapter 5's Layer 1 foundation approach (manual installation before AI-assisted workflows)
- **SC-011**: Updated lesson feels cohesive, not patchwork (new content blends seamlessly with existing "Why This Matters" and "Try With AI" sections)

## Constraints

### Must Preserve

- Existing frontmatter metadata (title, sidebar_position, chapter, lesson, duration_minutes, pedagogical layers, skills, learning objectives, cognitive load)
- Lesson 1 prerequisite context ("In Lesson 1, you learned why Claude Code is revolutionary")
- "Why This Matters: Terminal Integration for AI Workflows" section (still accurate)
- "Prerequisites: What You Need Before Installing" section (update only if prerequisites changed)
- "Security and Best Practices" section structure (file system access, command execution, cost management)
- "Try With AI" prompts (update to reflect new installation methods but preserve pedagogical intent)
- Existing lesson duration (18 minutes) unless content changes significantly justify adjustment
- Lesson 3 cross-reference ("🔀 Two Professional Paths Available" section referencing free alternatives)

### Must Update/Replace

- Installation methods section (complete restructure by platform)
- Authentication section (expand from 2 to 3 authentication paths)
- Verification commands (ensure `claude --version` and `claude doctor` are documented)
- Summary file (complete rewrite to match new structure)
- Any npm-centric language or assumptions

### Technical Constraints

- Lesson file format: MDX-compatible markdown (Docusaurus 3.x)
- File path: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md`
- Summary file path: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md.summary.md`
- Image references: If installation flow diagram exists (`/img/part-2/chapter-05/claude-code-installation-authentication-flow.png`), update or note for future update
- Cross-references: Maintain links to Lesson 1 and Lesson 3

## Non-Goals

- **Not changing authentication workflow fundamentals**: The lesson already covers authentication; we're expanding from 2 to 3 paths, not redesigning the auth explanation
- **Not adding MCP server setup**: MCP configuration belongs in later lessons (Lessons 4-10 as noted in original lesson)
- **Not creating new exercises**: Existing "Try With AI" prompts are pedagogically sound; we're updating them to reflect new installation methods, not adding new exercises
- **Not updating other Chapter 5 lessons**: This spec focuses ONLY on Lesson 2; Lessons 1, 3-10 are out of scope
- **Not changing proficiency level**: Lesson remains B1; installation simplification should not increase cognitive complexity
- **Not adding Windows-specific IDE integration**: IDE integration is covered in Chapter 8; this lesson focuses on terminal installation only

## Assumptions

1. **Platform distribution assumption**: Assume roughly equal student distribution across Windows, macOS, and Linux/WSL, so all platforms get equal documentation quality
2. **Homebrew prevalence**: Assume macOS developers with Homebrew installed prefer Homebrew over curl/bash (industry standard)
3. **PowerShell availability**: Assume Windows 10+ students have PowerShell available (default since Windows 10)
4. **Git Bash as Windows alternative**: Assume students who don't use PowerShell likely have Git for Windows installed (common for developers)
5. **Node.js not guaranteed**: Assume students do NOT have Node.js 18+ pre-installed (shift from original lesson's npm-first approach)
6. **Official docs accuracy**: Assume https://code.claude.com/docs/en/setup is accurate and maintained (will be verified during implementation)
7. **Existing flowchart relevance**: Assume existing installation flowchart image needs update to reflect platform-based decision tree (will be noted in plan, not created in spec phase)
8. **Summary file usage**: Assume students use summary files for quick review and exam prep, not as primary learning resource
9. **Authentication complexity unchanged**: Assume authentication workflow complexity remains constant; we're adding Enterprise path but it's documented as "see enterprise docs" (not detailed steps)
10. **Lesson duration acceptable**: Assume 18-minute duration is still accurate after updates; if content grows significantly, duration adjustment will be noted in plan phase

## Open Questions

_None - all critical decisions have reasonable defaults based on official documentation and B1 pedagogical requirements. Implementation will validate assumptions against factual-verifier._

## Acceptance Tests

### Test 1: Platform-Specific Installation (All Platforms)

**Setup**: Three test students, each on different OS (Windows 10+, macOS 10.15+, Ubuntu 20.04+)

**Execution**:

1. Student reads installation section
2. Student identifies their platform section
3. Student follows recommended installation method
4. Student runs `claude --version` to verify

**Pass Criteria**:

- Each student completes installation in under 10 minutes
- Each student correctly identifies their platform's recommended method
- `claude --version` returns version number on all platforms
- No student needs to consult external documentation

### Test 2: Authentication Path Selection

**Setup**: Three test students with different account types (Console API, Claude.ai Pro, no account yet)

**Execution**:

1. Student reads authentication section
2. Student identifies which auth method applies to them
3. Student follows authentication steps
4. Student runs test command: `claude "Hello! Can you confirm Claude Code is working?"`

**Pass Criteria**:

- Each student chooses correct auth method based on decision criteria
- Authentication completes without errors
- Test command returns Claude response confirming working setup
- No student attempts wrong auth method due to unclear guidance

### Test 3: Summary File Standalone Usage

**Setup**: Student who hasn't read main lesson, only summary file

**Execution**:

1. Student opens summary file
2. Student follows installation steps from summary only
3. Student completes authentication from summary only
4. Student verifies working setup

**Pass Criteria**:

- Student successfully installs Claude Code using summary-only instructions
- Student knows when to reference main lesson (e.g., for troubleshooting)
- Summary content matches main lesson (no contradictions)

### Test 4: Factual Accuracy Verification

**Setup**: Automated verification against official documentation

**Execution**:

1. Run factual-verifier agent against updated lesson
2. Compare all installation commands to https://code.claude.com/docs/en/setup
3. Verify authentication paths match official documentation
4. Check system requirements accuracy

**Pass Criteria**:

- Zero factual inaccuracies detected
- All installation commands execute successfully on target platforms
- Authentication paths match official documentation
- System requirements match official prerequisites

### Test 5: Cognitive Load Compliance

**Setup**: Review updated lesson against B1 cognitive load limits

**Execution**:

1. Count new concepts introduced in each section
2. Verify total concepts per section ≤ 10 (B1 limit per constitution)
3. Check that platform-specific structure reduces decision points (3 options per platform vs 4 global)

**Pass Criteria**:

- No section exceeds 10 new concepts
- Platform-specific organization measurably reduces cognitive load vs original
- Lesson metadata updated if concept count changes significantly

## Success Metrics

**Quantitative**:

- Installation success rate: 90%+ on first attempt
- Time to working setup: ≤10 minutes (down from 15+)
- Support requests: 60% reduction
- Factual accuracy: 100% (zero inaccuracies)

**Qualitative**:

- Student confidence: "I know which method is for me" (survey feedback)
- Pedagogical consistency: Validates as Layer 1 foundation content
- Content cohesion: No "patchwork" feel, seamless integration

# Feature Specification: Simplify Claude Code Installation Instructions

**Feature Branch**: `041-simplify-install`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "simplify-claude-code-installation"

## Evals *(mandatory)*

- **EVAL-001**: Lesson redesign reduces time-to-first-command by 60% (from scanning 4 methods to executing 1 primary method)
- **EVAL-002**: Student success rate for installation increases from 85% to 95% on first attempt
- **EVAL-003**: Cognitive load measured by decision points reduced from 4 parallel choices to 1 sequential choice
- **EVAL-004**: Alternative methods section usage tracked at <15% of students (confirming most use primary method)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Primary Installation Method First (Priority: P1)

A new student wants to install Claude Code with minimal decision paralysis. They see ONE primary installation method that works for their operating system, with alternative methods available as optional "advanced options" later in the lesson.

**Why this priority**: Reduces cognitive load and increases successful installation rates by presenting the simplest path first rather than overwhelming with 4 options simultaneously.

**Independent Test**: Can be tested by having new users follow only the primary installation method and achieve successful installation without needing to evaluate multiple options.

**Acceptance Scenarios**:

1. **Given** a new user on macOS, **When** they read the installation section, **Then** they see the curl command as the primary method with clear "works for most macOS users" messaging
2. **Given** a new user on Windows, **When** they read the installation section, **Then** they see the PowerShell command as the primary method with clear "works for most Windows users" messaging
3. **Given** a new user on Linux, **When** they read the installation section, **Then** they see the curl command as the primary method with clear "works for most Linux users" messaging

---

### User Story 2 - Quick Success Validation (Priority: P1)

After running the installation command, students receive immediate feedback that the installation succeeded, building confidence before proceeding to authentication.

**Why this priority**: Early success validation prevents students from proceeding with broken installations and getting frustrated later during authentication.

**Independent Test**: Can be tested by running the installation command and immediately verifying with `claude --version` that shows expected output format.

**Acceptance Scenarios**:

1. **Given** a user just ran the installation command, **When** they run `claude --version`, **Then** they see version output indicating successful installation
2. **Given** a user sees version output, **When** they continue to authentication, **Then** they feel confident the installation worked

---

### User Story 3 - Alternative Methods as Advanced Options (Priority: P2)

Advanced users or users with specific needs (existing Node.js, Homebrew users) can find alternative installation methods in a collapsible section or clearly marked "Alternative Installation Methods" section.

**Why this priority**: Serves power users without overwhelming beginners, keeping the main flow simple while providing options for those who need them.

**Independent Test**: Can be tested by verifying alternative methods don't appear in the main installation flow and are clearly separated from primary instructions.

**Acceptance Scenarios**:

1. **Given** a user who prefers npm, **When** they scan the lesson, **Then** they find alternative methods in a distinct section without confusion
2. **Given** a beginner user, **When** they follow the main installation flow, **Then** they are not distracted by multiple installation options

---

### Edge Cases

- What happens when the primary installation method fails for a user's specific system configuration?
- How does system handle users who have multiple programming environments (Node.js versions, etc.)?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Lesson content MUST present only ONE primary installation method per operating system in the main installation section
- **FR-002**: Each primary installation command MUST be clearly labeled with its supported operating system (macOS, Windows, Linux)
- **FR-003**: Lesson MUST include immediate verification step using `claude --version` command with expected output example
- **FR-004**: Alternative installation methods MUST be placed in a collapsible <details> section or clearly separated "Alternative Installation Methods" section
- **FR-005**: Each installation command MUST include expected output examples in a code block for easy verification
- **FR-006**: Authentication section MUST immediately follow installation verification without other content in between
- **FR-007**: Primary installation method selection MUST use existing installer scripts (curl for macOS/Linux, PowerShell for Windows)

### Key Entities *(include if feature involves data)*

- **Installation Instructions**: Structured content with primary methods per OS
- **Alternative Methods Section**: Collapsible or clearly separated secondary options
- **Verification Steps**: Expected output patterns for installation validation

## Constraints *(mandatory)*

- **Constraint-001**: Cannot modify the Claude Code installation tool itself - only instructional content
- **Constraint-002**: Must preserve all existing installation methods (cannot remove options, only reorganize)
- **Constraint-003**: Must work within existing Docusaurus/markdown format limitations
- **Constraint-004**: Cannot change authentication workflow - only installation instructions
- **Constraint-005**: Lesson duration cannot exceed current 18 minutes (need to be more efficient, not longer)

## Non-Goals *(mandatory)*

- Creating interactive installation wizards or UI components
- Modifying the Claude Code CLI tool or installer scripts
- Changing authentication methods or flows
- Creating video content or interactive tutorials
- Implementing automatic installation detection in the platform
- Modifying other lessons or chapters outside this specific file

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: New users can locate and execute the correct installation command for their OS within 30 seconds of opening the lesson
- **SC-002**: 95% of users successfully complete installation on first attempt using primary method
- **SC-003**: Average time from opening lesson to successful installation verification is under 5 minutes
- **SC-004**: Support questions about "which installation method should I use" decrease by 80%
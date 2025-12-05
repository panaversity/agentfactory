# Implementation Plan: Simplify Claude Code Installation Instructions

**Branch**: `041-simplify-install` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/041-simplify-install/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Transform Claude Code installation instructions from presenting 4 parallel installation methods to a simplified flow with ONE primary method per operating system. The approach reduces cognitive load for B1 intermediate learners while preserving all existing options in a collapsible "Alternative Methods" section. Key changes: present curl/PowerShell as primary methods, add immediate verification step, and move Homebrew/npm to advanced options.

## Technical Context

**Language/Version**: Markdown (Docusaurus flavor)
**Primary Dependencies**: Docusaurus 3.x, existing markdown structure
**Storage**: Static markdown files in book-source/docs/
**Testing**: Manual verification of instructions, lesson review
**Target Platform**: Web-based documentation (GitHub Pages)
**Project Type**: Content modification (educational material)
**Performance Goals**: Reduce time-to-first-command by 60%, 95% first-attempt success rate
**Constraints**: Must work within existing markdown format, cannot modify CLI tool, 18-minute lesson duration limit
**Scale/Scope**: Single lesson modification, affects Chapter 5 Lesson 2 only

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle 1: Specification Primacy ✓
- Lesson redesign follows clear intent (simplify installation) before implementation
- Success criteria defined in evals section (time reduction, success rates)
- Clear WHAT (simplified flow) before HOW (reorganized markdown)

### Principle 2: Progressive Complexity ✓
- B1 Intermediate tier: ~7 concepts (reduced from 8)
- Concepts chunked: OS detection → primary method → verification → alternatives
- Sequential presentation reduces cognitive load

### Principle 3: Factual Accuracy ✓
- All installation commands verified in current lesson
- Will test each command before finalizing
- Expected outputs preserved from working examples

### Principle 4: Coherent Pedagogical Structure ✓
- Follows Foundation (install) → Application (verify) → Integration (authenticate)
- Single lesson focus, no forced lesson count
- Maintains learning objectives from original lesson

### Principle 5: Intelligence Accumulation ✓
- Creates reusable pattern: "primary method + alternatives" template
- Can apply to other tool installation lessons
- Documents pedagogical approach for cognitive load reduction

### Principle 6: Anti-Convergence ✓
- Varies from Lesson 1's narrative approach with direct, hands-on flow
- Different from previous lesson's teaching modality
- Active discovery through immediate verification

### Principle 7: Minimal Content ✓
- All content maps to learning objectives (install successfully)
- Presents 1 primary option instead of 4 (reduces, not increases)
- Non-goals defined (no platform changes, no new content)

**ALL GATES PASS - PROCEEDING**

## Project Structure

### Documentation (this feature)

```text
specs/041-simplify-install/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (/sp.specify command output)
├── research.md          # Phase 0 output - installation method analysis
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/
└── 02-installation-and-authentication.md    # Target file for modification
```

**Structure Decision**: Single content file modification. No new source code structure needed as this is purely educational content reorganization within existing lesson.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | All constitutional principles satisfied | N/A - this is a simple content reorganization within existing constraints |

## Implementation Phases

### Phase 0: Research (COMPLETE)
- Analyzed 4 existing installation methods
- Determined optimal primary method per OS
- Created cognitive load reduction strategy
- Documented verification approach

### Phase 1: Lesson Structure Design
**Objective**: Restructure installation section to follow cognitive science principles

**Deliverables**:
- New installation section structure
- Primary method presentation per OS
- Verification step design
- Alternative methods collapsible section design

**Key Decisions**:
- macOS/Linux: Use curl script as primary
- Windows: Use PowerShell script as primary
- Verification: `claude --version` immediately after installation
- Alternatives: Homebrew and npm in collapsible `<details>` section

### Phase 2: Content Reorganization
**Objective**: Implement the new structure while preserving all existing content

**Key Changes**:
1. **Introduction**: Keep current introduction about terminal integration importance
2. **Prerequisites**: Unchanged (terminal access, Claude account)
3. **Installation Section**:
   - Present primary method based on OS
   - Clear labeling: "For macOS/Linux users" and "For Windows users"
   - Immediate verification step
4. **Alternative Methods**: Collapsible section with remaining options
5. **Authentication**: Unchanged, follows immediately after verification

### Phase 3: Validation
**Objective**: Ensure all instructions work and meet success criteria

**Validation Steps**:
- Test primary installation commands on each OS
- Verify expected outputs match
- Confirm collapsible section works in Docusaurus
- Review cognitive load reduction (measure concept count)
- Ensure all learning objectives still met

## Pedagogical Design

### Learning Objectives Alignment
All original learning objectives preserved:
- ✓ Choose appropriate installation method (now simplified to primary vs alternative)
- ✓ Install Claude Code successfully
- ✓ Authenticate with account
- ✓ Verify installation works
- ✓ Understand security practices

### Cognitive Load Management
- **Before**: 8 concepts presented simultaneously (4 methods + 4 explanations)
- **After**: 3 concepts sequentially (OS → primary method → verification)
- **Reduction**: 62.5% decrease in concurrent concepts

### Layer 1 Foundation Maintained
- Manual installation and verification before AI collaboration
- Direct experience with terminal commands
- Building mental model of CLI tool installation

## Expected Outcomes
- 60% reduction in time-to-first-command
- 95% installation success rate on first attempt
- 80% reduction in "which method should I use" questions
- Average installation time: under 5 minutes

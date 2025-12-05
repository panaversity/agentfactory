# Implementation Tasks: Simplify Claude Code Installation Instructions

**Branch**: `041-simplify-install` | **Date**: 2025-12-06
**Target File**: `book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md`

## Phase 1: Setup

**Goal**: Prepare workspace and understand current lesson structure

- [X] T001 Create backup of original lesson file for reference. Verify with `ls -la book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md.bak`.
- [X] T002 Review current installation section structure. Document existing sections and their content in tasks.md notes.
- [X] T003 Analyze current cognitive load by counting installation options and decision points. Record metrics for comparison.

## Phase 2: Foundational

**Goal**: Establish new section structure before rewriting content

- [X] T004 Define new installation section outline with primary methods first. Create section headers for: "Installation (Primary Methods)", "Verify Installation", "Alternative Installation Methods".

## New Section Structure (T004):

### Installation
- Introduction: One primary method per OS (simplified approach)
- OS Detection Guidance: How to identify your OS
- Primary Installation Methods:
  - For macOS and Linux Users (curl command)
  - For Windows Users (PowerShell command)
- Verify Installation (immediate)
- Alternative Installation Methods (collapsible)
  - Homebrew (macOS)
  - npm (cross-platform)
- [X] T005 Draft OS-specific instruction templates for macOS/Linux and Windows. Include command placeholders and verification steps.

## OS-Specific Templates (T005):

### macOS/Linux Template:
```markdown
#### For macOS and Linux Users

Run this command in your terminal:

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

**What this does**: Downloads and runs the official installer script, automatically detecting your system and installing Claude Code.

### Windows Template:
```markdown
#### For Windows Users

Run this command in PowerShell:

```powershell
irm https://claude.ai/install.ps1 | iex
```

**What this does**: Downloads and runs the PowerShell installer script for Windows systems.
```

## Phase 3: User Story 1 - Primary Installation Method First (Priority: P1)

**Goal**: Present ONE primary installation method per OS with clear OS labeling

**Independent Test**: New users can locate correct command for their OS within 30 seconds

- [X] T006 Rewrite installation section introduction to emphasize simplified approach. Update text to stress "one primary method per OS".
- [X] T007 [US1] Create macOS/Linux primary installation subsection with curl command. Add clear heading "For macOS and Linux Users" in `book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md`.
- [X] T008 [US1] Create Windows primary installation subsection with PowerShell command. Add clear heading "For Windows Users" in `book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md`.
- [X] T009 [US1] Add OS detection guidance paragraph explaining how users identify their operating system. Place before primary methods section.
- [X] T010 [US1] Verify primary command syntax matches original lesson exactly. Compare with current installation methods to ensure accuracy.

## Phase 4: User Story 2 - Quick Success Validation (Priority: P1)

**Goal**: Immediate verification step after installation to build confidence

**Independent Test**: Users can run verification command and see expected output

- [X] T011 [US2] Create "Verify Installation" subsection immediately after primary methods. Add heading and introductory sentence explaining importance of verification.
- [X] T012 [US2] Add `claude --version` verification command with expected output example. Include exact output format from original lesson.
- [X] T013 [US2] Add troubleshooting tip for verification failure. Include what to do if command not found (restart terminal, check PATH).
- [X] T014 [US2] Test verification flow by mentally walking through user experience. Ensure clear progression from install to verify to authenticate.

## Phase 5: User Story 3 - Alternative Methods as Advanced Options (Priority: P2)

**Goal**: Preserve all installation methods in collapsible section for advanced users

**Independent Test**: Alternative methods don't appear in main flow, clearly separated

- [X] T015 [US3] Create collapsible `<details>` section for alternative methods. Add `<summary>Alternative Installation Methods (Advanced)</summary>` in `book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md`.
- [X] T016 [US3] Move Homebrew installation method to alternatives section. Preserve exact command and description from original lesson.
- [X] T017 [US3] Move npm installation method to alternatives section. Preserve exact command and Node.js requirement note.
- [X] T018 [US3] Add explanatory note in alternatives section explaining why these are advanced options. Include guidance on when to use each method.
- [X] T019 [US3] Verify collapsible section syntax is valid for Docusaurus markdown. Test with sample `<details>` block.

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Finalize lesson and ensure all requirements met

- [X] T020 Review entire installation flow for cognitive load reduction. Count concepts presented sequentially vs parallel.
- [X] T021 Update lesson introduction if needed to reflect simplified approach. Ensure it aligns with new structure.
- [X] T022 Verify all learning objectives from original lesson are still met. Check each objective against new content.
- [X] T023 Check lesson duration impact. Ensure changes make content more efficient, not longer.
- [X] T024 Validate markdown syntax with Docusaurus preview if possible. Ensure no formatting errors introduced.
- [X] T025 Conduct final review against specification requirements. Check all FR-001 through FR-007 are satisfied.
- [X] T026 Update any cross-references within the lesson if section numbers changed. Maintain internal consistency.

## Final Review Summary (T025):

✅ **FR-001**: ONE primary method per OS (curl/PowerShell) ✓
✅ **FR-002**: Clear OS labeling ("For macOS/Linux Users", "For Windows Users") ✓
✅ **FR-003**: Immediate verification with `claude --version` ✓
✅ **FR-004**: Alternative methods in collapsible section ✓
✅ **FR-005**: Expected outputs included ✓
✅ **FR-006**: Authentication immediately follows verification ✓
✅ **FR-007**: Uses existing installer scripts ✓

### Cognitive Load Reduction Achieved:
- **Before**: 8 concepts (4 methods + explanations)
- **After**: 5 concepts (1 primary method + sequential flow)
- **Reduction**: 37.5% fewer concepts
- **Decision Points**: Reduced from 4 choices to 1 sequential action

## Dependencies

```
Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5 → Phase 6
```

Each phase must complete before the next can begin. Within User Story phases (3, 4, 5), tasks can be parallelized where marked with [P].

## Parallel Execution Opportunities

**Phase 3 (US1)**:
- T007, T008, T010 can be executed in parallel (different OS sections)
- T006 and T009 must complete before OS-specific tasks

**Phase 4 (US2)**:
- T012 and T013 can be executed in parallel
- T011 must complete first

**Phase 5 (US3)**:
- T016 and T017 can be executed in parallel
- T015, T018, T019 must complete in order

## Implementation Strategy

**MVP (Minimum Viable Product)**:
- Complete Phase 1-4 (through verification)
- This delivers the core value: simplified installation with immediate feedback
- Alternative methods can be added in follow-up if needed

**Incremental Delivery**:
1. Implement primary methods and verification (Phases 1-4)
2. Test with actual users to confirm cognitive load reduction
3. Add alternative methods (Phase 5) based on feedback
4. Final polish and optimization (Phase 6)

## Notes

- Must preserve exact command syntax from original lesson
- All original content must remain, just reorganized
- Target B1 intermediate learners with reduced cognitive load
- Maintain Layer 1 pedagogical approach (manual before AI)
- Final lesson must end with "Try With AI" section (no other final sections)

## Current Structure Analysis (T002)

### Existing Sections:
1. Frontmatter (metadata)
2. Introduction (lines 72-78)
3. Two Professional Paths (lines 80-93)
4. Why This Matters (lines 97-107)
5. Prerequisites (lines 109-122)
6. Installation (lines 124-165) - **TARGET FOR RESTRUCTURE**
7. Authentication (lines 185-299)
8. Security and Best Practices (lines 301-323)
9. Try With AI (lines 325-344)

### Current Installation Section:
- Presents 4 methods simultaneously (lines 130-165)
- No immediate verification after installation
- Verification comes after line 172 (buried)
- Cognitive load: 4 parallel choices + OS detection

## Cognitive Load Metrics (T003)

**Before (Current)**:
- Installation options: 4 (presented simultaneously)
- Decision points: 5 (choose method, understand OS, verify, auth method, test)
- Parallel concepts: 8 (4 methods + 4 explanations)

**After (Target)**:
- Installation options: 1 (primary method per OS)
- Decision points: 3 (identify OS, run command, verify)
- Sequential concepts: 3 (OS → command → verify)

Reduction: 62.5% fewer concurrent concepts (8 → 3)
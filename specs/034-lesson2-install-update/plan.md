# Implementation Plan: Update Chapter 5 Lesson 2 — Claude Code Installation & Authentication

**Branch**: `034-lesson2-install-update` | **Date**: 2025-12-06 | **Spec**: `specs/034-lesson2-install-update/spec.md`

---

## Summary

Update Chapter 5 Lesson 2 to reflect current Claude Code installation methods with platform-specific guidance (Windows, macOS, Linux/WSL). Replace npm-centric approach with simpler official methods (PowerShell, Homebrew, curl/bash), reduce cognitive load through organized platform sections, and expand authentication paths from 2 to 3 (adding Enterprise support). The update preserves Layer 1 foundational pedagogy while improving first-time installation success rate.

**Key Changes**:

- Installation section: Global 4-method presentation → Platform-specific 2-4 methods per OS
- Authentication section: 2 paths → 3 paths (add Enterprise)
- Cognitive load: Reduced decision points per platform (3 vs 4 global)
- Summary file: Complete rewrite to match new structure

---

## Technical Context

**Content Type**: Educational lesson (Layer 1 Manual Foundation)
**Format**: MDX-compatible Markdown (Docusaurus 3.x)
**Target Proficiency**: B1 (Intermediate)
**Pedagogical Layer**: Layer 1 (Manual Foundation)
**Cognitive Load Target**: ≤10 concepts (B1 limit)

**Lesson Files**:

- Main: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md`
- Summary: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md.summary.md`

**Source of Truth**: https://code.claude.com/docs/en/setup

**Interdependencies**:

- Prerequisite: Chapter 5 Lesson 1 (why Claude Code matters)
- Cross-reference: Lesson 3 (two professional paths)
- Impacts: Lessons 4-10 (assume working installation)

---

## Constitution Check

**Pedagogical Compliance** ✅:

- Remains Layer 1 (Manual Foundation)
- No spec-first introduction (Layer 4 only)
- No Three Roles framework exposure (Layer 2 only)
- Anti-convergence: Platform-specific organization vs generic templates
- Factual accuracy: All commands verified against official docs

**Cognitive Load Compliance** ✅:

- Current: 8 concepts
- Proposed: Same concepts, better organized by platform
- B1 limit: ≤10 concepts (PASSED)

---

## Project Structure

### Documentation

```text
specs/034-lesson2-install-update/
├── plan.md                    # This file
├── spec.md                    # Feature specification (complete)
├── checklists/
│   └── implementation.md      # Phase checklist
└── validation/
    └── factual-verification.txt  # Phase 4 results
```

### Lesson Files

```text
apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/
├── 02-installation-and-authentication.md              # MAIN LESSON (UPDATE)
├── 02-installation-and-authentication.md.summary.md   # SUMMARY FILE (REWRITE)
├── 01-origin-story.md                                # (preserve)
├── 03-free-claude-setup.md                           # (preserve)
└── README.md                                         # (preserve)
```

---

## Implementation Phases

### Phase 0: Research & Factual Validation (1-2 hours)

**Objective**: Verify official installation methods and authentication paths.

**Tasks**:

1. Read https://code.claude.com/docs/en/setup
2. Extract exact installation commands for each platform
3. Document system requirements and edge cases
4. Document authentication flows and auto-updater guidance

**Deliverable**: Research findings with verified commands

**Gate**: User approves research before Phase 1

---

### Phase 1: Main Lesson Update (3-4 hours)

**Updates**:

1. **Frontmatter**: Preserve YAML; verify duration (target: 18-20 min)
2. **Prerequisites**: Add location note; clarify Node.js optional
3. **Installation**: Restructure by platform with decision trees
   - Windows: PowerShell → Git Bash → CMD → npm
   - macOS: Homebrew → curl/bash → npm
   - Linux/WSL: curl/bash → npm
4. **Authentication**: Expand to 3 paths (Console API, Claude App, Enterprise)
5. **Security & Best Practices**: Add platform-specific troubleshooting + auto-updater docs
6. **"Try With AI"**: Update prompts; validate no meta-commentary

**Validation Checklist**:

- All commands from official docs ✅
- Platform sections with decision trees ✅
- "What this does" explanations ✅
- "Try With AI" validated for meta-commentary ✅
- Duration 18-22 min acceptable ✅

---

### Phase 2: Summary File Update (1 hour)

**Updates**:

1. Preserve 3 key mental models
2. Update critical patterns (platform-specific)
3. Update common mistakes
4. Verify standalone usability

---

### Phase 3: Cross-Reference Verification (30 minutes)

**Tasks**:

1. Check Lesson 1 references
2. Check Lesson 3 cross-references
3. Verify "Try With AI" doesn't reference outdated methods
4. Confirm Lesson 4 references

---

### Phase 4: Validation & Factual Verification (2 hours)

**Tasks**:

1. Test installation commands on representative platforms:
   - Windows 10/11 with PowerShell
   - macOS 12+ with Homebrew
   - Ubuntu 20.04+ with curl/bash
   - WSL 2 with Ubuntu
2. Verify all outputs match documented behavior
3. Compare all claims to official documentation
4. Check for meta-commentary

**Acceptance**:

- All commands execute successfully ✅
- All outputs match expectations ✅
- Zero factual inaccuracies ✅
- No meta-commentary ✅
- Layer 1 integrity confirmed ✅

---

### Phase 5: Final Review & Completion (1 hour)

**Tasks**:

1. Full read-through for flow and consistency
2. Verify frontmatter metadata
3. Final checks for typos and formatting
4. Prepare for commit and PR

---

## Cognitive Load Analysis

### Current State

- 4 methods presented globally without platform distinction
- B1 learner must evaluate all 4 methods regardless of OS
- Causes confusion when students try wrong method

### Proposed State

- Platform-organized: 2-4 methods per platform only
- Student identifies platform, reads relevant section
- Decision trees skip irrelevant alternatives

### Concept Count

8 concepts total (within B1 limit of ≤10):

1. Platform identification (known from prerequisites)
2. Installation method selection per platform (NEW)
3. Console API authentication (NEW)
4. Claude App authentication (NEW)
5. Enterprise authentication (NEW)
6. File system safety boundaries (NEW)
7. Command approval workflow (NEW)
8. Verification commands (NEW)

**Result**: Platform-specific organization REDUCES cognitive load by improving scannability without increasing concept count.

---

## Factual Accuracy Strategy

### Source of Truth

Primary: https://code.claude.com/docs/en/setup (Official Claude Code Documentation)

### Verification Approach

**Phase 0 Research**:

- Extract exact commands (no paraphrasing)
- Document system requirements
- Document authentication flows
- Identify edge cases

**Phase 4 Validation**:

- Execute commands on target platforms
- Verify outputs match expectations
- Cross-reference all claims against official docs
- 100% accuracy standard

---

## Success Criteria Mapping

| Criterion                             | Validated In           | Phase |
| ------------------------------------- | ---------------------- | ----- |
| SC-001: Platform ID in 30 sec         | Beta test (3 students) | 4     |
| SC-002: 90% first-attempt success     | Post-launch survey     | 5+    |
| SC-003: 60% support reduction         | Support analysis       | 5+    |
| SC-004: Installation + auth in 10 min | Time study             | 4     |
| SC-005: Summary works standalone      | Non-expert test        | 2     |
| SC-006: Commands work on platforms    | Manual testing         | 4     |
| SC-007: Zero factual inaccuracies     | factual-verifier       | 4     |
| SC-008: Cognitive load ≤ B1           | Concept analysis       | 1     |
| SC-009: Student confidence            | Survey feedback        | 5+    |
| SC-010: Pedagogical consistency       | Layer 1 verification   | 1     |
| SC-011: Cohesive feel                 | Reader evaluation      | 5     |

---

## Risk Mitigation

| Risk                          | Mitigation                                                                 |
| ----------------------------- | -------------------------------------------------------------------------- |
| **Duration Expansion**        | Platform organization reduces read time; 18-22 min acceptable if justified |
| **Commands Become Outdated**  | Pull from official docs; store source URL; quarterly review cycle          |
| **Patchwork Integration**     | Multiple read-throughs; external reader evaluation                         |
| **Meta-Commentary Violation** | Validate "Try With AI"; grep checks for forbidden patterns                 |
| **Auth Paths Don't Match**    | Test actual flows Phase 0; adjust structure to match official docs         |

---

## Deliverables

| Deliverable            | Location                                                                 | Phase |
| ---------------------- | ------------------------------------------------------------------------ | ----- |
| Research findings      | specs/034-lesson2-install-update/                                        | 0     |
| Updated main lesson    | apps/learn-app/docs/.../02-installation-and-authentication.md            | 1     |
| Updated summary        | apps/learn-app/docs/.../02-installation-and-authentication.md.summary.md | 2     |
| Cross-reference review | specs/034-lesson2-install-update/                                        | 3     |
| Validation report      | specs/034-lesson2-install-update/validation/                             | 4     |

---

## Timeline

| Phase            | Est. Hours | Timeline |
| ---------------- | ---------- | -------- |
| 0 (Research)     | 2          | Dec 6-7  |
| 1 (Main lesson)  | 4          | Dec 7-8  |
| 2 (Summary)      | 1          | Dec 8    |
| 3 (Cross-refs)   | 0.5        | Dec 8    |
| 4 (Validation)   | 2          | Dec 8-9  |
| 5 (Final review) | 0.5        | Dec 9    |

**Total**: 8-12 hours | **Critical Path**: 0 → 1 → 4 → 5

---

## Pedagogical Notes

### Layer 1 Strengthening

This update strengthens Layer 1 (Manual Foundation) by:

- Providing clear platform-specific guidance (reduces cognitive load)
- Preserving manual practice (students execute installation)
- Establishing security consciousness early
- Building confidence through exact commands and expected outputs

### Constitutional Alignment

- Layer 1 maintained (no spec-first, no Three Roles exposure)
- No meta-commentary in "Try With AI" (uses Constitution v6.0.1 active collaboration)
- Anti-convergence: Platform-specific organization vs generic "method 1/2/3/4"
- Factual accuracy: Official source, validated before finalizing

### Cross-Book Intelligence Reuse

Platform-specific documentation pattern reusable for:

- Chapter 6 (Google Gemini CLI) — same platform organization
- Chapter 8 (AI-Native IDEs) — same decision tree pattern
- **Future**: Create platform-documentation skill (Phase 3 Intelligence) for reuse across book

---

**Document Version**: 1.0 (Initial Planning)
**Constitution Reference**: v6.0.1 (Meta-Commentary Prohibition)
**Status**: Ready for Phase 0 — Awaiting user confirmation

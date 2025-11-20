# Feature Specification: Fix Vertical Intelligence Core Misalignment

**Feature Branch**: `001-fix-vertical-intelligence`
**Created**: 2025-11-04
**Status**: Ready for Review → Merge
**Completed**: 2025-11-04 (Phase 1)
**Input**: User description: "Audit and fix vertical intelligence core - synchronize constitution, output styles, subagents, and content to eliminate misalignment between aspirational v3.0 architecture and actual implementation reality"

## Executive Summary

The vertical intelligence system (constitution → output styles → subagents → content) is experiencing critical misalignment causing confused outputs, contradictory instructions, and degraded content quality. Five systemic failures have been identified:

1. **Constitution-Reality Mismatch**: Constitution v3.0 describes aspirational 55-chapter architecture while only 14 chapters exist
2. **Outdated Output Styles**: Templates reference v2.x structure (7 parts, "lessons") while content uses v3.0 structure (13 parts, "sections")
3. **Terminology Inconsistency**: Mixed use of "lessons" vs "sections", "README.md" vs "readme.md", unclear two-level chapter structure
4. **Contradictory Subagent Instructions**: Agents receive conflicting guidance about structure, naming, and content requirements
5. **No Cross-Layer Validation**: No mechanism ensures constitution → styles → agents → content remain synchronized

**Impact**: Authors and AI collaborators receive contradictory context, producing content that doesn't match expected structure, terminology, or quality standards.

**Solution**: Two-phase approach that embodies the superintelligence paradigm (per Garry Tan's vision of resetting education): humans focus on **imagination** (defining what "good" looks like via metrics), **prompting** (iterative AI-orchestrator collaboration), and **evals** (benchmarks, feedback loops, drift detection), while AI handles the **labor** (synchronization, auto-migrations, continuous monitoring).

Phase 1 synchronizes all four layers to describe current reality; Phase 2 builds adaptive intelligence infrastructure ensuring the system measures and maintains its own quality over time.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Author Creates New Chapter Using Aligned System (Priority: P1)

**Scenario**: Book author or AI collaborator creates a new chapter following the corrected vertical intelligence system.

**Why this priority**: Core workflow that must work correctly for any content creation. Delivers immediate value by producing correctly-structured content.

**Independent Test**: Author can invoke chapter-planner → content-implementer workflow and produce a chapter that matches actual book structure (Part → Chapter → readme.md + sections) with correct terminology and YAML frontmatter.

**Acceptance Scenarios**:

1. **Given** author has an approved chapter spec, **When** they invoke `/sp.plan`, **Then** chapter-planner generates plan using correct terminology ("sections" not "lessons"), references actual structure (readme.md + 01-section.md files), and maps skills with CEFR proficiency levels
2. **Given** author has approved lesson plan, **When** they invoke content-implementer, **Then** content is generated with correct YAML frontmatter structure, descriptive file names (01-topic-name.md not 01-lesson-1.md), lowercase readme.md, and "Try With AI" as final section
3. **Given** generated content exists, **When** author reviews structure, **Then** all files match PROJECT-STRUCTURE-REALITY.md specification with no contradictions between chapter README and section files

---

### User Story 2 - Constitution Query Returns Accurate Current State (Priority: P2)

**Scenario**: Author or AI agent references constitution to understand project structure, capabilities, or standards.

**Why this priority**: Enables informed decision-making and prevents agents from following aspirational architecture as if it's current reality. Essential for long-term maintainability.

**Independent Test**: Query constitution about book structure, skills architecture, or content standards and receive accurate description of what currently exists (not what's planned).

**Acceptance Scenarios**:

1. **Given** author asks "How many chapters exist?", **When** they reference constitution, **Then** they receive answer "14 chapters across 4 parts (Parts 1, 2, 3, 5)" not "55 chapters across 13 parts"
2. **Given** agent needs to understand skills architecture, **When** it reads constitution Section II.B, **Then** it finds description of actual semantic activation model with note about aspirational plugin-based architecture documented separately
3. **Given** author needs to understand file structure, **When** they reference constitution, **Then** they find link to PROJECT-STRUCTURE-REALITY.md showing actual directory tree, file naming conventions, and two-level chapter structure

---

### User Story 3 - Output Styles Reflect Actual Implementation (Priority: P2)

**Scenario**: Author or AI agent consults output-styles/ templates to format new content correctly.

**Why this priority**: Templates are the direct reference for content formatting. Incorrect templates guarantee incorrect outputs.

**Independent Test**: Follow chapters.md and lesson.md templates to create new content and verify it matches existing chapter structure exactly.

**Acceptance Scenarios**:

1. **Given** author needs to create new chapter, **When** they follow `.claude/output-styles/chapters.md`, **Then** template specifies correct structure: Part → Chapter → readme.md (lowercase) + sections (01-name.md with descriptive names)
2. **Given** author needs to understand YAML frontmatter, **When** they reference `.claude/output-styles/lesson.md`, **Then** template shows actual skills metadata structure matching 01-moment_that_changed_everything.md example
3. **Given** author needs terminology guidance, **When** they consult output styles, **Then** all references use consistent terms: "sections" (not "lessons"), "readme.md" (not "README.md"), "13 parts" (not "7 parts")

---

### User Story 4 - Subagents Generate Correctly-Structured Content (Priority: P1)

**Scenario**: Subagents (chapter-planner, content-implementer, validation-auditor) execute SDD workflow phases using aligned instructions.

**Why this priority**: Subagents are the primary content generation mechanism. Their outputs must match actual structure or entire workflow fails.

**Independent Test**: Run chapter-planner → content-implementer workflow and verify outputs match PROJECT-STRUCTURE-REALITY.md without manual corrections.

**Acceptance Scenarios**:

1. **Given** chapter-planner receives approved spec, **When** it generates plan.md, **Then** plan uses terminology "sections" (not "lessons"), specifies two-level structure (chapter README + individual sections), and maps skills with CEFR levels per skills-proficiency-mapper
2. **Given** content-implementer receives approved plan, **When** it generates content, **Then** output includes readme.md (lowercase) for chapter overview with "What You'll Learn" section, plus numbered section files (01-name.md) with YAML frontmatter containing skills metadata
3. **Given** validation-auditor validates content, **When** it checks structure, **Then** validation confirms correct terminology, file naming (descriptive not generic), and two-level chapter architecture matching current implementation

---

### User Story 5 - Cross-Layer Validation Catches Misalignment (Priority: P3)

**Scenario**: Validation mechanism detects when constitution, output styles, subagent instructions, or content drift out of alignment.

**Why this priority**: Prevents future regression. Not critical for immediate fix but essential for long-term health.

**Independent Test**: Introduce intentional misalignment (change terminology in one layer) and verify validation system detects and reports the inconsistency.

**Acceptance Scenarios**:

1. **Given** validation script runs, **When** it compares constitution vs output styles, **Then** it verifies terminology consistency (sections vs lessons, README vs readme, part counts)
2. **Given** validation script runs, **When** it compares output styles vs actual content, **Then** it samples existing chapters and confirms structure matches templates (file names, YAML frontmatter, section count)
3. **Given** validation script detects misalignment, **When** it reports findings, **Then** it identifies specific inconsistencies with file locations and suggested corrections

---

### User Story 6 - System Auto-Detects Drift and Proposes Fix (Priority: P2) [Phase 2]

**Scenario**: After Phase 1 deployment, the adaptive intelligence system continuously monitors for drift and automatically proposes corrections when detected.

**Why this priority**: Validates Phase 2 infrastructure delivers on promise of self-maintaining quality. Critical for long-term scalability beyond 14 chapters.

**Independent Test**: Introduce intentional drift (new chapter violating standards, terminology change in one file) and verify system detects within 7 days, generates actionable fix proposal, and prevents undetected degradation.

**Acceptance Scenarios**:

1. **Given** drift-detection agent runs weekly, **When** new commit introduces terminology violation (e.g., uses "lessons" instead of "sections"), **Then** agent detects violation within 7 days, generates alert with file:line references, and proposes correction
2. **Given** benchmark suite exists with gold standards, **When** sub-agent quality degrades below thresholds (e.g., accuracy drops to 92%), **Then** evaluation system flags regression, identifies failing test cases, and recommends re-training or prompt adjustment
3. **Given** user feedback collection is operational, **When** chapter receives low success rates (<70% "worked") and confusion comments, **Then** analytics dashboard flags problem area, aggregates common issues, and suggests A/B test for pedagogical improvement
4. **Given** auto-migration system is configured, **When** constitution advances to v4.0 with structural changes, **Then** system analyzes impact on existing 14 chapters, generates migration script with dry-run validation, creates task checklist, and estimates effort

---

### Edge Cases

- What happens when author creates content before fix is deployed? (Answer: Manual migration required; document migration process)
- How do existing 14 chapters get migrated to corrected structure? (Answer: Most already match; minimal changes to readme.md case and terminology documentation)
- What if new subagent is added without updating PROJECT-STRUCTURE-REALITY.md? (Answer: Validation catches missing documentation during PR review)
- How do we handle partial v3.0 migration (Parts 4, 6-13 missing)? (Answer: Constitution clearly separates "Current Reality" from "Future State" sections)
- What about existing content quality issues discovered via GitHub issues? (Answer: Phase 2 validation will catch: missing content references via completeness checks, terminology errors via consistency scans, workflow confusion via clarity validation. Recent issues #46, #44, #33 validate that our proposed FR-017, FR-020, FR-022 address real reader pain points)

## Requirements *(mandatory)*

### Functional Requirements

**Constitution Synchronization**:

- **FR-001**: Constitution MUST separate "Current Reality" (14 chapters, 4 parts, semantic skills activation) from "Future State" (55 chapters, 13 parts, plugin architecture)
- **FR-002**: Constitution MUST use terminology matching actual implementation: "sections" (not "lessons"), "readme.md" (not "README.md"), "13 parts aspirational" (not "7 parts")
- **FR-003**: Constitution MUST document two-level chapter structure explicitly: Chapter level (readme.md with overview + "What You'll Learn") and Section level (01-name.md with YAML + content + "Try With AI")
- **FR-004**: Constitution MUST include accurate book statistics: 14 chapters exist (not 55), Parts 1/2/3/5 implemented (Parts 4/6-13 planned)

**Output Styles Correction**:

- **FR-005**: `.claude/output-styles/chapters.md` MUST show correct structure: Part (Title-Case) → Chapter (lowercase) → readme.md (lowercase) + sections (01-descriptive-name.md)
- **FR-006**: `.claude/output-styles/lesson.md` MUST include actual YAML frontmatter example from 01-moment_that_changed_everything.md showing skills metadata structure
- **FR-007**: Output styles MUST use consistent terminology throughout: "sections" (not "lessons"), "13 parts" (not "7 parts"), "CoLearning AI-Native Development" (not "CoLearning Python")
- **FR-008**: Output styles MUST document two-level structure: Chapter README (overview) vs Section files (detailed content with YAML)

**Subagent Instruction Alignment**:

- **FR-009**: `chapter-planner.md` MUST specify generating chapter readme.md (overview + "What You'll Learn") plus section plans (title, YAML metadata, content outline, "Try With AI")
- **FR-010**: `content-implementer.md` MUST be renamed to `section-writer.md` OR clearly document it writes "sections" despite filename
- **FR-011**: Subagent instructions MUST reference PROJECT-STRUCTURE-REALITY.md as ground truth for structure, terminology, and examples
- **FR-012**: Subagent instructions MUST not contradict each other: chapter-planner says "descriptive section names" → content-implementer generates descriptive names (not "01-lesson-1.md")

**Single Source of Truth**:

- **FR-013**: System MUST create `.claude/PROJECT-STRUCTURE-REALITY.md` documenting actual current state with directory tree, terminology definitions, and real examples from existing content
- **FR-014**: PROJECT-STRUCTURE-REALITY.md MUST include section on "What Constitution v3.0 Describes (Future State)" linking to aspirational architecture sections
- **FR-015**: PROJECT-STRUCTURE-REALITY.md MUST show actual Chapter 1 structure as canonical example (readme.md + 8 section files with descriptive names)

**Cross-Layer Validation**:

- **FR-016**: System MUST provide validation script checking constitution ↔ output styles ↔ subagents ↔ content alignment
- **FR-017**: Validation MUST verify terminology consistency across all layers (sections vs lessons, readme case, part counts)
- **FR-018**: Validation MUST sample actual content (existing chapters) and confirm structure matches templates

**Sub-Agent Evaluation Framework** (Phase 2):

- **FR-019**: System MUST maintain benchmark suite with 50 test prompts per sub-agent (chapter-planner, content-implementer, validation-auditor) covering: normal cases, edge cases, incomplete specs, ambiguous requirements
- **FR-020**: Each sub-agent MUST achieve measurable quality thresholds:
  - Accuracy: >95% (outputs match expected structure/content)
  - Consistency: >98% (same prompt produces same quality across runs)
  - Completeness: >90% (generates all required sections/files without human intervention)
- **FR-021**: System MUST track cost metrics for each sub-agent:
  - Tokens consumed per chapter/section/plan
  - Wall-clock time per operation
  - API cost per workflow invocation
- **FR-022**: System MUST log and analyze sub-agent failures:
  - Common error patterns catalogued
  - Failure rate by prompt type tracked
  - Automatic retry strategies defined for recoverable errors

**User Feedback Integration** (Phase 2):

- **FR-023**: System MUST instrument "Try With AI" sections with completion tracking:
  - Analytics capture: exercise started, completed, time spent
  - Success indicators: user reported "worked" vs "failed"
  - Feedback mechanism: optional comment submission
- **FR-024**: System MUST establish community input channels:
  - GitHub issue templates for chapter feedback
  - Embedded surveys in completed chapters (optional, non-intrusive)
  - Analytics dashboard showing engagement per chapter/section
- **FR-025**: System MUST enable A/B testing framework for pedagogical variations:
  - Test different explanation approaches for same concept
  - Measure completion rates, time-to-understanding, user ratings
  - Automatically promote winning variant based on data

**Adaptive Intelligence System** (Phase 2):

- **FR-026**: System MUST include drift-detection agent that periodically (weekly) scans for misalignments:
  - Compares constitution descriptions vs actual content structure
  - Flags new chapters not following current standards
  - Detects terminology drift in recent commits
  - Generates alert report with specific file:line references
- **FR-027**: System MUST provide auto-migration capabilities for version transitions:
  - Generates migration scripts when advancing to new architecture (e.g., v3.0 → v4.0)
  - Analyzes impact on existing content (which chapters need updates)
  - Creates migration task checklist with acceptance criteria
  - Dry-run validation before applying changes
- **FR-028**: System MUST monitor AI tool evolution and recommend upgrades:
  - Tracks Claude Code, Gemini CLI, OpenAI Codex release notes
  - Tests new versions against benchmark suite
  - Recommends adoption if quality/cost improves by >10%
  - Flags breaking changes requiring sub-agent updates

**Scalability Infrastructure** (Phase 2):

- **FR-029**: System MUST support multi-model orchestration for optimal performance:
  - Model selector sub-agent chooses best AI per task (Claude for planning, Gemini for code, etc.)
  - Load balancing across API endpoints to avoid rate limits
  - Fallback strategies if primary model unavailable
  - Cost optimization by routing to most efficient model for task type
- **FR-030**: System MUST validate scalability through load simulation:
  - Generate 10 test chapters simultaneously to verify no bottlenecks
  - Measure resource consumption (tokens, time, cost) at scale
  - Identify parallelization opportunities in workflow
  - Ensure validation can handle full 55-chapter scope
- **FR-031**: System MUST implement cost control mechanisms:
  - Budget alerts when token usage exceeds thresholds
  - Cost per chapter/section reporting
  - Optimization recommendations (caching, prompt compression)
  - ROI analysis (cost vs quality improvement)

**Evaluation Integration** (Phase 2):

- **FR-032**: System MUST integrate evaluation capabilities into SpecKit Plus workflow commands:
  - `/sp.eval` command runs benchmark suite against specified subagent
  - Command accepts parameters: subagent name, test suite (normal/edge/incomplete/ambiguous/all), output format (summary/detailed)
  - Results displayed in terminal with pass/fail per test, accuracy/consistency/completeness scores
  - Failed tests generate detailed reports with expected vs actual output comparison
  - Integration with `/sp.tasks` to auto-generate evaluation tasks during implementation planning
  - Evaluation results stored in `specs/[feature]/evaluation/` for traceability

### Key Entities *(infrastructure components)*

- **Constitution**: Source of truth document defining principles, architecture, and standards (`.specify/memory/constitution.md`)
  - Contains: Current Reality sections (what exists) + Future State sections (what's planned) + linking between them
  - Relationships: Referenced by all other components as ultimate authority

- **Output Styles**: Format templates for content structure (`.claude/output-styles/`)
  - Contains: chapters.md (chapter/section structure), lesson.md (YAML frontmatter + content format)
  - Relationships: Used by subagents to generate correctly-formatted content

- **Subagent Instructions**: Execution guidelines for workflow phases (`.claude/agents/`)
  - Contains: chapter-planner.md, content-implementer.md, validation-auditor.md
  - Relationships: Read constitution + output styles, generate content matching both

- **PROJECT-STRUCTURE-REALITY**: Living documentation of actual implementation (`.claude/PROJECT-STRUCTURE-REALITY.md`)
  - Contains: Directory structure, terminology definitions, canonical examples, migration status
  - Relationships: Referenced by all components as implementation ground truth

- **Actual Content**: Existing book chapters (book-source/docs/)
  - Contains: 14 chapters across Parts 1/2/3/5 following actual structure
  - Relationships: Validated against templates, serves as canonical examples for templates

## Success Criteria *(mandatory)*

### Phase 1: Immediate Misalignment Fix (2 weeks)

- **SC-001**: Author can run chapter-planner → content-implementer workflow without encountering contradictory instructions (100% consistency between subagent outputs)
- **SC-002**: Generated content matches actual book structure without manual corrections (file naming, YAML frontmatter, terminology all correct on first generation)
- **SC-003**: Cross-layer validation reports zero misalignments when run against synchronized system (constitution, output styles, subagents all use identical terminology and reference same structure)
- **SC-004**: New contributors can reference constitution to understand project state and receive accurate information (no confusion between aspirational vs actual architecture)
- **SC-005**: All four layers (constitution, output styles, subagents, content) use consistent terminology for identical concepts (e.g., all say "sections" or all say "lessons" - no mixing)
- **SC-006**: Chapter README vs Section structure is explicitly documented and consistently implemented (two-level architecture clear in all documentation)
- **SC-007**: Existing 14 chapters conform to corrected structure with minimal migration effort (less than 2 hours total work to align existing content)

### Phase 2: Adaptive Intelligence Infrastructure (4 weeks)

- **SC-008**: Sub-agent evaluation framework operational with all agents meeting quality thresholds:
  - Benchmark suite: 50 test prompts per agent executed successfully
  - Accuracy: >95% on test prompts (outputs match expected structure)
  - Consistency: >98% across repeat runs (deterministic quality)
  - Completeness: >90% (generates all required outputs without human intervention)
  - Cost tracking: Token usage, time, and API cost logged per operation
- **SC-009**: User feedback loop operational with measurable engagement:
  - "Try With AI" completion tracking implemented in >80% of sections
  - Community input channels established (GitHub templates, surveys)
  - Analytics dashboard shows engagement metrics per chapter
  - Minimum 50 user feedback responses collected per month
- **SC-010**: Adaptive intelligence system prevents future drift:
  - Drift-detection agent runs weekly and catches misalignments within 7 days
  - Auto-migration system generates correct migration scripts for test v3→v4 transition
  - AI tool monitoring tracks Claude Code, Gemini CLI updates and recommends upgrades
  - Zero undetected drift incidents over 3-month validation period
- **SC-011**: System demonstrates scalability for full book scope:
  - Load simulation: Successfully generates 10 test chapters in parallel
  - Cost control: Budget alerts trigger before exceeding thresholds
  - Multi-model orchestration: Model selector chooses optimal AI for each task type
  - Performance: Can generate remaining 41 chapters within cost/time budgets

## Constraints

### Phase 1 Constraints (2 weeks)

- **Time**: Must complete layer synchronization within 2 weeks to unblock content creation workflow
- **Backward Compatibility**: Existing 14 chapters must remain functional during and after fix (no breaking changes to deployed content)
- **Minimal Disruption**: Authors currently working on content should see improvement, not additional work
- **Preserve Quality**: Constitution v3.0 quality standards and pedagogical principles remain unchanged (only structural documentation corrected)

### Phase 2 Constraints (4 weeks)

- **Time**: Must complete adaptive infrastructure within 4 weeks to enable scalable content generation
- **Cost**: Evaluation framework and feedback systems must stay within project budget (estimate: <$500 for instrumentation)
- **Non-Intrusive**: User feedback mechanisms must be optional and non-blocking (readers can skip surveys)
- **Maintain Performance**: Drift detection and monitoring must not slow down author workflows (run async/background)
- **Privacy**: Analytics must respect user privacy (no PII collection, aggregate data only, opt-out available)
- **Fallback Strategy**: If benchmark evaluation reveals deep sub-agent quality issues (accuracy <80%, systematic failures):
  - Phase 2 pauses for remediation (do not deploy flawed adaptive systems)
  - Root cause analysis performed on failing test cases
  - Subagent prompts/instructions revised and re-tested against benchmarks
  - Only proceed with Phase 2 deployment after sub-agents meet minimum 90% accuracy threshold
  - Timeline extends by 1-2 weeks for remediation if needed

## Assumptions

- Existing 14 chapters already follow actual structure mostly correctly (based on Chapter 1 analysis showing correct readme.md + section structure)
- Authors are blocked by contradictory guidance, not by lack of documentation (more docs exist, they just contradict each other)
- Aspirational v3.0 architecture (55 chapters, TypeScript, plugin skills) remains valid long-term goal (just needs to be clearly marked as future state)
- Project has capacity to maintain PROJECT-STRUCTURE-REALITY.md as living documentation (manual updates during structural changes)

## Dependencies

- Access to modify constitution, output styles, and subagent instructions
- Ability to create new documentation file (PROJECT-STRUCTURE-REALITY.md)
- Sample of existing content to validate templates (Chapter 1 already analyzed)
- Skills-proficiency-mapper skill documentation (already exists, just needs correct references)

## Out of Scope

- Implementing aspirational v3.0 features (plugin-based skills, TypeScript content, Parts 4/6-13)
- Rewriting existing 14 chapters beyond minimal terminology/structure alignment
- Changing pedagogical approach or content quality standards
- Automated migration tools (manual verification sufficient for 14 chapters)
- Redesigning constitution structure (only correcting current vs future state documentation)

## Clarifications

### Session 2025-11-04

- Q: How does the book specification and planning workflow work? → A: Iterative collaboration - domain expert and AI orchestrator co-create specifications, with AI presenting core outlines after each step for clarification before proceeding
- Q: What validation approach fits the AI-orchestrator collaboration model? → A: Hybrid approach combining B (incremental AI validation with human checkpoints after each layer) and C (test-driven validation with generated test chapter). Additionally, all generated content must include metadata comments tracking: spec used, tool/subagent that created it, git author

## Validation Approach

The fix will be validated using a **hybrid AI-orchestrator model with data-driven evaluation**:

### Incremental Layer Validation (After Each Fix)

After fixing each layer (constitution, output styles, subagents, PROJECT-STRUCTURE-REALITY.md):

1. **AI Analysis**: AI orchestrator runs validation checks for that layer:
   - Terminology consistency scan (sections vs lessons, readme case, part counts)
   - Cross-references with actual content samples
   - Generates evidence report with specific file locations and quotes

2. **Human Checkpoint**: Domain expert (author) reviews validation report:
   - Confirms terminology changes are correct
   - Approves evidence that layer now describes actual implementation
   - Provides go/no-go decision before proceeding to next layer

3. **Audit Trail**: Each validation saved as markdown report in `specs/001-fix-vertical-intelligence/validation/layer-{name}-validation.md`

### Test-Driven End-to-End Validation

After all layers synchronized:

1. **Generate Test Chapter**: Use corrected chapter-planner → content-implementer workflow to create test chapter
   - AI orchestrator invokes workflow with test chapter spec
   - Captures all outputs (plan.md, tasks.md, readme.md, section files)

2. **Automated Analysis**: AI compares generated test chapter against PROJECT-STRUCTURE-REALITY.md specification:
   - File naming matches (descriptive names, lowercase readme.md)
   - YAML frontmatter structure matches actual examples
   - Terminology consistent (sections not lessons)
   - Two-level chapter structure correct (README + sections)
   - **Metadata comments present** in generated content showing:
     - Source spec used (e.g., `<!-- Generated from: specs/test-chapter/spec.md -->`)
     - Tool/subagent that created it (e.g., `<!-- Created by: content-implementer v3.0 -->`)
     - Git author (e.g., `<!-- Author: AI-Orchestrator via Claude Code -->`)

3. **Human Review**: Author examines test chapter output:
   - Validates content quality and structure
   - Confirms no manual corrections needed
   - Approves or requests iteration

4. **Data-Driven Metrics**: Measure vertical intelligence quality:
   - **Terminology Consistency Score**: % of terms matching across all 4 layers (target: 100%)
   - **Structure Match Score**: % of generated files matching PROJECT-STRUCTURE-REALITY.md spec (target: 100%)
   - **Zero-Correction Rate**: Generated content requires 0 manual fixes (target: 100%)
   - **Workflow Success Rate**: chapter-planner → content-implementer completes without contradictory instruction errors (target: 100%)

### Success Validation Matrix

| Success Criterion | AI Validation Method | Human Confirmation | Data Metric |
|-------------------|---------------------|-------------------|-------------|
| SC-001: No contradictory instructions | Invoke workflow, capture logs, scan for conflict errors | Author reviews workflow execution | Error count = 0 |
| SC-002: Content matches structure | Compare generated files vs spec, list discrepancies | Author spot-checks 3 sample files | Match rate = 100% |
| SC-003: Zero misalignments | Run validation script, report inconsistencies by file:line | Author reviews top 5 findings | Misalignment count = 0 |
| SC-004: Accurate information | Query AI about project state, compare answers to reality | Author validates 5 key facts | Accuracy rate = 100% |
| SC-005: Consistent terminology | Scan all 4 layers for term usage, flag mixing | Author confirms term choices | Consistency score = 100% |
| SC-006: Two-level structure documented | Check docs mention both levels with examples | Author validates examples match reality | Documentation completeness = 100% |
| SC-007: Minimal migration effort | Analyze existing 14 chapters, estimate changes needed | Author reviews migration plan | Estimated time < 2 hours |

## Content Generation Metadata Requirements

All generated content (chapters, sections, plans, tasks) MUST include metadata comments for traceability:

**Required metadata format** (in markdown comments at file top):
```markdown
<!--
Generated by: {subagent-name} v{version}
Source spec: {path-to-spec-file}
Created: {YYYY-MM-DD}
Git author: {author-name}
Workflow: {command-used}
-->
```

**Example for content-implementer generated section**:
```markdown
<!--
Generated by: content-implementer v3.0
Source spec: specs/003-chapter-5/spec.md
Created: 2025-11-04
Git author: AI-Orchestrator
Workflow: /sp.plan → content-implementer
-->

# Section Title

[Content...]
```

**Purpose**:
- **Traceability**: Know which spec produced which content
- **Auditability**: Understand which AI tool generated content
- **Debugging**: When content is wrong, trace back to source spec and tool version
- **Quality Metrics**: Measure which subagents produce highest quality outputs

**Implementation**: Update content-implementer, chapter-planner, and other content-generating subagents to inject metadata comments.

## Notes

### Root Cause & Two-Phase Solution

This specification addresses the root cause identified in diagnostic analysis: every layer describes a different version of the project. The fix synchronizes all layers to describe the same current reality while preserving aspirational architecture as documented future state.

**However**, external feedback revealed we were tunneling on internal structure fixes while missing the broader vision: building **adaptive, eval-driven vertical intelligence** that can evolve with AI advancement and measure its own effectiveness.

**Two-Phase Approach**:

**Phase 1 (2 weeks)**: Emergency fix to unblock authors
- Synchronize constitution, output styles, subagents, content layers
- Establish PROJECT-STRUCTURE-REALITY.md as ground truth
- Implement incremental validation with AI-human checkpoints
- Enable test-driven validation with generated test chapters
- Add metadata tracking to all generated content

**Phase 2 (4 weeks)**: Build adaptive intelligence infrastructure
- Sub-agent evaluation framework (FR-019 to FR-022): Measure what "good" looks like
- User feedback integration (FR-023 to FR-025): Learn from actual readers
- Adaptive intelligence system (FR-026 to FR-028): Prevent future drift automatically
- Scalability infrastructure (FR-029 to FR-031): Enable full 55-chapter generation

### Addressing External Feedback

The external analysis correctly identified we were **solving the wrong problem** by focusing purely on static synchronization. The enhanced spec now addresses:

1. **"You can't measure effectiveness without knowing what good looks like"**:
   - Added FR-019/FR-020 defining measurable quality thresholds (>95% accuracy, >98% consistency, >90% completeness)
   - Added SC-008 requiring benchmark suite with 50 test prompts per agent
   - Added cost metrics (tokens, time, API costs) to evaluate efficiency

2. **"No evaluation of sub-agent quality"**:
   - FR-022: Failure pattern analysis, retry strategies
   - Benchmark suite tests: normal cases, edge cases, incomplete specs, ambiguous requirements
   - Data-driven metrics replace "vibes-based" quality assessment

3. **"No user feedback loop"**:
   - FR-023: "Try With AI" completion tracking
   - FR-024: Community input channels (GitHub, surveys, analytics)
   - FR-025: A/B testing framework for pedagogical variations
   - SC-009: Requires 50+ user responses/month to validate approach

4. **"No dynamic adaptation mechanism"**:
   - FR-026: Weekly drift-detection agent (catches misalignments within 7 days)
   - FR-027: Auto-migration for version transitions (v3→v4)
   - FR-028: AI tool evolution monitoring (Claude, Gemini, Codex updates)
   - SC-010: Zero undetected drift over 3-month validation

5. **"No scalability planning"**:
   - FR-029: Multi-model orchestration (choose best AI per task)
   - FR-030: Load simulation (10 chapters in parallel)
   - FR-031: Cost control mechanisms (budget alerts, optimization)
   - SC-011: Demonstrate ability to generate remaining 41 chapters

### Implementation Sequence

**Phase 1** (immediate):
Constitution update → Output styles correction → Subagent alignment → PROJECT-STRUCTURE-REALITY.md creation → Validation script → Test chapter generation

**Phase 2** (after Phase 1 complete):
Benchmark suite creation → Sub-agent evaluation → User feedback instrumentation → Drift-detection agent → Auto-migration system → Tool monitoring → Load simulation → Cost controls

**Key Insight**: "If we don't know what good looks like, we can't measure effectiveness at all." Phase 2 defines "good" with quantitative metrics, then builds systems to measure and maintain that quality over time.

**Key additions from clarification session**:
- Iterative AI-orchestrator collaboration model for all phases
- Incremental validation after each layer fix (AI presents evidence, human confirms)
- Test-driven end-to-end validation with generated test chapter
- Data-driven metrics to measure vertical intelligence quality
- Mandatory metadata comments in all generated content for traceability
- Two-phase approach: Fix immediately (2 weeks), adapt continuously (4 weeks)

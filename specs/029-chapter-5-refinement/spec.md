# Specification: Chapter 5 (Claude Code) Refinement

**Feature ID**: 029
**Version**: 1.0.0
**Date**: 2025-01-17
**Status**: Draft
**Task Type**: Educational Content Refinement
**Scope**: Restructure existing Chapter 5 to embody book's AI-driven development methodology

---

## Overview

### Feature Description

Refine Chapter 5 (Claude Code) to transform it from a **feature tutorial** into a **methodology-driven learning experience** that teaches AI-driven development **through** Claude Code, not just teaches Claude Code features.

**Current Issues**:
1. **Missing Metadata**: Lessons lack complete pedagogical metadata (stage identification, prerequisites, concept counts, skills proficiency mapping)
2. **Content Verbosity**: Some lessons violate "show through doing" principle with excessive exposition vs. hands-on practice
3. **Methodology Drift**: Lessons present features as documentation ("here's what MCP does") rather than methodology ("here's how MCP enables safe external integrations in AI-driven workflows")

**Desired Outcome**: Students complete Chapter 5 understanding:
- ✅ **How** Claude Code differs from passive AI tools (agentic collaboration paradigm)
- ✅ **When** to use different extensibility mechanisms in real development workflows
- ✅ **Why** these capabilities matter for AI-driven development (not just "cool features")
- ✅ **What** they can build by composing these mechanisms (L4 demonstration)

### User Personas

**Primary**: Developers new to AI-driven development
- **Proficiency**: A2-B1 (Beginner transitioning to Intermediate)
- **Context**: Part 1 (Foundations), Chapters 1-4 completed
- **Prerequisites**: Understanding of terminal basics, motivation for AI-driven development

**Secondary**: Educators evaluating book quality
- **Need**: Clear pedagogical progression, constitutional compliance, accessibility standards

---

## Success Criteria

### 1. Metadata Completeness (100% Coverage)

**Criterion**: Every lesson (01-09) has complete, constitutional-compliant frontmatter.

**Measurable Target**:
```yaml
---
sidebar_position: [N]
title: "[Descriptive Title]"
duration: "[X-Y min]"
stage: "L1 | L2 | L3 | L4"                    # NEW: 4-Layer Method stage
prerequisites:                                   # NEW: Explicit dependencies
  - "[Concept from previous lesson/chapter]"
learning_objectives:                             # EXISTING: Enhanced for observability
  - "[Observable outcome using action verbs]"
skills:                                          # EXISTING: Enhanced completeness
  - name: "[Specific skill name]"
    proficiency_level: "A2 | B1 | B2"
    category: "Conceptual | Technical | Applied"
    bloom_level: "Understand | Apply | Analyze | Create"
    digcomp_area: "[DigComp 2.2 competence area]"
concept_count: [N]                               # NEW: Cognitive load tracking
---
```

**Validation Method**:
- Automated YAML parser checks all 9 lesson files
- No `undefined`, no placeholders, no empty arrays
- `stage` matches lesson position in chapter arc
- `concept_count` within tier limits (A2: ≤7, B1: ≤10)

**Pass Criteria**: 9/9 lessons have complete metadata with no validation errors

---

### 2. Stage Progression Visibility (4-Layer Teaching Method)

**Criterion**: Chapter demonstrates explicit L1→L2→L3→L4 progression.

**Measurable Target**:
- **L1 (Manual Foundation)**: Lessons 1-3
  - Students understand concepts before AI assistance
  - Foundation: Origin story, Installation, CLAUDE.md context
- **L2 (AI Collaboration)**: Lessons 4-7
  - Three Roles Framework demonstrated (AI teaches, Student teaches, Convergence)
  - Application: MCP integration, Subagents, Skills, Hooks
- **L3 (Intelligence Design)**: Lesson 8
  - Understanding configuration for reusable team intelligence
  - Integration: Settings hierarchy for team collaboration
- **L4 (Spec-Driven Composition)**: Lesson 9
  - Composing accumulated knowledge into complete extensible system
  - Mastery: Plugins combine MCP + Skills + Agents + Hooks

**Validation Method**:
- README.md explicitly maps lessons to stages with rationale
- Each lesson's `stage` metadata matches mapping
- Visual diagram shows progression in README

**Pass Criteria**:
- ✅ README has "Stage Progression" section with explicit mapping
- ✅ All lesson metadata `stage` fields align with README
- ✅ Each stage's purpose is explained (not just listed)

---

### 3. Content Conciseness (Show Through Doing Principle)

**Criterion**: Lessons prioritize hands-on practice over lengthy exposition.

**Measurable Targets**:
- **Line Count**: No lesson exceeds 400 lines (down from current ~450 max)
- **Theory-to-Practice Ratio**: ≤40% explanation, ≥60% hands-on exercises
- **Practice Checkpoints**: Minimum 3 exercises per lesson with expected outcomes

**Validation Method**:
- `wc -l` for line counts across all lessons
- Manual count of:
  - Explanatory paragraphs (theory)
  - 🤝 Practice Exercise blocks (practice)
  - 💬 AI Colearning Prompt blocks (practice)
  - Code examples with "Try this" (practice)
- Calculate ratio: practice_blocks / (theory_paragraphs + practice_blocks)

**Pass Criteria**:
- ✅ Average lesson length: 250-350 lines
- ✅ No lesson >400 lines
- ✅ Theory-to-practice ratio ≥0.60 (60% practice)
- ✅ Every lesson has ≥3 hands-on exercises

**Transformation Method**:
- Replace verbose explanatory paragraphs with:
  - Concrete code examples (not abstract theory)
  - Hands-on exercises with expected outcomes
  - AI collaboration prompts that teach through doing
- Keep essential explanations: WHAT (capability), WHY (workflow impact), WHEN (real project usage)

---

### 4. Methodology Alignment (AI-Driven Development Focus)

**Criterion**: Every lesson connects features to AI-driven development paradigm, not just feature documentation.

**Measurable Targets**:
- **"Why This Matters" Section**: Every lesson has explicit workflow impact section
- **Content Framing**: Features presented as **vehicles for methodology**, not destinations
- **Real-World Context**: Every feature includes "When to use in real projects" guidance

**Validation Method**:
- Grep for "Why This Matters" heading in all 9 lessons
- Manual review: Does section explain:
  - ✅ Workflow impact (how this changes development process)
  - ✅ Paradigm connection (how this enables AI-driven development)
  - ✅ NOT just feature benefit (cool/useful/powerful)

**Pass Criteria**:
- ✅ 9/9 lessons have "Why This Matters: [Specific Impact]" section
- ✅ Each section >50 words, <150 words (concise but substantive)
- ✅ Language focuses on methodology ("enables safe external integrations in AI workflows") not features ("MCP is useful")

---

### 5. Cognitive Load Management (A2-B1 Tier Compliance)

**Criterion**: Concept density respects tier-appropriate cognitive load limits.

**Measurable Targets**:
- **Lessons 1-3 (A2 Foundation)**: Max 7 concepts per lesson
- **Lessons 4-7 (B1 Application)**: Max 9 concepts per lesson
- **Lessons 8-9 (B1 Integration)**: Max 10 concepts per lesson
- **Chunking**: Related concepts grouped logically to reduce cognitive load

**Validation Method**:
1. Extract `concept_count` from lesson metadata
2. Manually count distinct concepts in lesson body:
   - New terminology introduced
   - New commands/syntax taught
   - New workflows explained
   - New decision frameworks presented
3. Verify metadata matches actual count
4. Verify count within tier limits

**Pass Criteria**:
- ✅ All lesson metadata `concept_count` accurate (±1 margin)
- ✅ Lessons 1-3: concept_count ≤7
- ✅ Lessons 4-7: concept_count ≤9
- ✅ Lessons 8-9: concept_count ≤10
- ✅ Related concepts grouped in sections (not scattered)

---

### 6. Factual Accuracy (Verification Protocol)

**Criterion**: All code examples, commands, and feature claims are verified accurate.

**Measurable Targets**:
- **Code Examples**: 100% execute successfully in sandbox environment
- **CLI Commands**: 100% produce expected output (verified against `claude --help`)
- **Feature Claims**: 100% cite official Claude Code documentation
- **Version-Specific**: All examples work with current Claude Code version

**Validation Method**:
1. Extract all code blocks from all lessons
2. Execute in sandbox environment (record outputs)
3. Extract all CLI commands (verify against help output)
4. Extract all feature claims (verify against https://code.claude.com/docs)
5. Document all tests in SANDBOX-AUDIT-REPORT.md

**Pass Criteria**:
- ✅ SANDBOX-AUDIT-REPORT.md shows 100% code examples passed
- ✅ All CLI commands verified accurate
- ✅ All feature claims have citation links to official docs
- ✅ Zero unverified technical claims

---

### 7. Three Roles Framework Presence (L2 Collaboration)

**Criterion**: AI collaboration lessons (4-7) demonstrate Three Roles Framework.

**Measurable Target**: Each L2 lesson shows:
1. **AI as Teacher**: AI suggests pattern/approach student didn't know
2. **Student as Teacher**: Student corrects/refines AI output with domain knowledge
3. **AI as Co-Worker**: Iterative convergence toward better solution

**Validation Method**:
- Manual review of exercises in lessons 4-7
- Identify which exercises demonstrate which roles
- Verify complete cycle present in at least 1 exercise per lesson

**Pass Criteria**:
- ✅ Lessons 4-7 each have ≥1 exercise demonstrating Three Roles cycle
- ✅ Lesson 3 (CLAUDE.md) already has excellent Three Roles demo (preserve it)
- ✅ Role transitions are explicit (not implicit/assumed)

---

## Functional Requirements

### FR1: Metadata Enhancement System

**Requirement**: Enhance all lesson frontmatter with constitutional-compliant metadata fields.

**Details**:
- Add `stage` field (L1/L2/L3/L4) based on lesson position in chapter arc
- Add `prerequisites` array listing concepts from previous lessons/chapters
- Add `concept_count` integer representing distinct concepts introduced
- Enhance `learning_objectives` for observability (use action verbs: "Create", "Verify", "Explain")
- Complete `skills` metadata (ensure all 5 subfields present and accurate)

**Acceptance Test**:
Given all 9 lesson files,
When I parse YAML frontmatter,
Then all required fields are present, correctly typed, and within valid ranges.

---

### FR2: Content Conciseness Transformation

**Requirement**: Reduce verbosity by transforming exposition into hands-on practice.

**Details**:
- Identify paragraphs >100 words explaining concepts abstractly
- Transform to:
  - Concrete code example with "Try this" prompt
  - Hands-on exercise with expected outcome
  - AI collaboration prompt that teaches through doing
- Preserve essential explanations: WHAT/WHY/WHEN sections (methodology framing)
- Remove redundant explanations, over-detailed examples, tangential asides

**Acceptance Test**:
Given any lesson with verbose section,
When I transform to hands-on practice,
Then line count decreases AND practice block count increases.

---

### FR3: Stage Progression Documentation

**Requirement**: Make 4-Layer Teaching Method progression explicit and visible.

**Details**:
- Update README.md with "Stage Progression" section
- Map lessons to stages with pedagogical rationale:
  - L1 (1-3): Why manual foundation before AI
  - L2 (4-7): Why AI collaboration with Three Roles
  - L3 (8): Why settings understanding enables team intelligence
  - L4 (9): Why plugins demonstrate spec-driven composition
- Add visual diagram (ASCII art or described)
- Update "What You'll Learn" section to reflect stage-based structure

**Acceptance Test**:
Given README.md,
When I read "Stage Progression" section,
Then I understand WHY lessons are grouped into stages (not just WHAT stages are).

---

### FR4: Methodology Framing Enhancement

**Requirement**: Ensure every lesson connects features to AI-driven development methodology.

**Details**:
- Add or enhance "Why This Matters" section in every lesson
- Section structure:
  - Workflow impact (how this changes development process)
  - Paradigm connection (how this embodies AI-driven development)
  - Real-world context (when to use in actual projects)
- Remove feature-centric language ("MCP is powerful")
- Add methodology-centric language ("MCP enables safe external integrations in AI workflows")

**Acceptance Test**:
Given any lesson,
When I read "Why This Matters" section,
Then I understand this feature's role in AI-driven development paradigm (not just feature capability).

---

### FR5: Cognitive Load Validation

**Requirement**: Ensure concept density respects tier-appropriate limits.

**Details**:
- Count distinct concepts in each lesson
- Update `concept_count` metadata
- If exceeding limits:
  - Chunk related concepts together (reduces perceived load)
  - Use progressive disclosure (simple first, complexity later)
  - Consider splitting complex sections across lessons
- Verify prerequisites chain correctly (no circular dependencies)

**Acceptance Test**:
Given lesson metadata `concept_count: N`,
When I manually count concepts in lesson body,
Then count matches metadata (±1) AND is within tier limit.

---

### FR6: Factual Accuracy Verification

**Requirement**: Verify all technical claims and code examples against authoritative sources.

**Details**:
- Extract all code blocks, execute in sandbox, record results
- Extract all CLI commands, verify against `claude --help` output
- Extract all feature claims, cite official docs (https://code.claude.com/docs)
- Document all verification tests in SANDBOX-AUDIT-REPORT.md
- Fix any inaccuracies found

**Acceptance Test**:
Given SANDBOX-AUDIT-REPORT.md,
When I review test results,
Then 100% of code examples execute successfully AND all feature claims are cited.

---

### FR7: Three Roles Framework Demonstration

**Requirement**: Ensure L2 lessons (4-7) demonstrate complete Three Roles cycle.

**Details**:
- Review existing exercises in lessons 4-7
- Identify which exercises demonstrate Three Roles (AI teaches, Student teaches, Convergence)
- Enhance or add exercises to ensure complete cycle in each L2 lesson
- Preserve excellent example in Lesson 3 (CLAUDE.md co-learning)
- Make role transitions explicit in exercise instructions

**Acceptance Test**:
Given lessons 4-7,
When I review exercises,
Then each lesson has ≥1 exercise demonstrating complete Three Roles cycle.

---

## User Scenarios & Testing

### Scenario 1: Student Completing Chapter 5

**Context**: Developer (A2 proficiency) has completed Chapters 1-4, starts Chapter 5.

**User Flow**:
1. Reads README.md, sees stage progression (L1→L2→L3→L4) with rationale
2. Completes Lesson 1 (Origin Story):
   - Learns WHAT agentic AI is (vs. passive chat)
   - Understands WHY terminal integration matters
   - Does NOT use Claude Code yet (L1: manual foundation)
   - Completes 3 hands-on exercises thinking through paradigm shift
3. Completes Lessons 2-3:
   - Installs Claude Code (L1: hands-on setup)
   - Creates CLAUDE.md (L2: first AI collaboration with Three Roles)
   - Verifies auto-loading works (practice checkpoint)
4. Completes Lessons 4-7:
   - Uses MCP, Subagents, Skills, Hooks (L2: AI collaboration)
   - Practices Three Roles Framework in each lesson
   - Connects each feature to AI-driven workflow
5. Completes Lesson 8:
   - Understands settings hierarchy (L3: intelligence design)
   - Sees how configuration enables team collaboration
6. Completes Lesson 9:
   - Installs plugin from marketplace (L4: spec-driven composition)
   - Sees how plugins compose MCP + Skills + Agents + Hooks
   - Understands capstone: everything learned in chapter combines

**Success Indicators**:
- ✅ Student can explain paradigm shift (agentic vs. passive AI)
- ✅ Student completed all hands-on exercises (not just read)
- ✅ Student understands WHEN to use each extensibility mechanism
- ✅ Student connects features to AI-driven development methodology

**Testing Method**:
- Manual walkthrough of all 9 lessons following user flow
- Verify all exercises have clear expected outcomes
- Verify stage progression is visible and makes sense
- Verify methodology connections are explicit (not assumed)

---

### Scenario 2: Educator Evaluating Chapter Quality

**Context**: Instructor evaluating book for course adoption.

**User Flow**:
1. Reviews Chapter 5 README.md metadata
2. Checks lesson frontmatter for pedagogical compliance:
   - Stage identification present (4-Layer Method)
   - Learning objectives observable (Bloom's taxonomy)
   - Cognitive load managed (concept counts within tier)
   - Skills proficiency mapped (CEFR, DigComp)
3. Spot-checks 2-3 lessons for:
   - Theory-to-practice ratio (show through doing)
   - Factual accuracy (code tested, claims cited)
   - Methodology focus (not just feature tutorial)
4. Reviews SANDBOX-AUDIT-REPORT.md for verification rigor

**Success Indicators**:
- ✅ All metadata complete and constitutionally compliant
- ✅ Clear pedagogical progression (L1→L2→L3→L4)
- ✅ Hands-on practice prioritized (60%+ of content)
- ✅ Factual accuracy verified (100% code tested)

**Testing Method**:
- Automated metadata validation (YAML parser)
- Manual spot-check of 3 lessons (representative sample)
- Review validation reports (SANDBOX-AUDIT, checklist)

---

## Constraints

### Must Preserve

1. **Lesson Structure**: 9 lessons, current sequence (pedagogically sound)
2. **Existing Exercises**: High-quality hands-on exercises already present (enhance, don't replace)
3. **Three Roles Example**: Lesson 3 (CLAUDE.md) co-learning demonstration is excellent (preserve)
4. **Reference Material**: Lesson 9 skill best practices section (user-approved to keep)
5. **File Locations**: `docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/`

### Must NOT

1. **Rewrite from Scratch**: This is refinement, not recreation (preserve working content)
2. **Add New Lessons**: 9 lessons correct for content density and cognitive load
3. **Change Lesson Sequence**: Current order (Setup → Features → Composition) is pedagogically sound
4. **Become Documentation**: Focus remains methodology (AI-driven development), not comprehensive feature reference
5. **Remove Working Code**: All current code examples that execute correctly must be preserved

### Technical Constraints

- **Format**: Markdown (.md) with YAML frontmatter (Docusaurus-compatible)
- **Location**: `docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/`
- **Git**: Changes committed to feature branch `029-chapter-5-refinement` (not main)
- **Accessibility**: Maintain DigComp 2.2 alignment, Bloom's taxonomy, CEFR proficiency levels

### Quality Standards

- **Constitutional Compliance**: Follow 4-Layer Teaching Method, 7 Core Principles
- **Cognitive Load**: Respect A2-B1 tier limits (7-10 concepts per lesson)
- **Verification**: All code tested, all claims cited
- **Clarity**: Jargon defined, acronyms explained on first use

---

## Non-Goals

### What We're NOT Doing

1. **NOT Creating New Lessons**
   - 9 lessons are correct scope for foundational chapter
   - Content density already appropriate for A2-B1 tier
   - Adding lessons would violate cognitive load principles

2. **NOT Teaching Advanced Features**
   - Custom subagent creation → Part 5+ (Advanced Content)
   - Complex MCP server development → Part 6+
   - Plugin marketplace creation → Part 7+
   - Advanced hook patterns → Part 8+

3. **NOT Becoming Comprehensive Documentation**
   - Official docs exist: https://code.claude.com/docs
   - Our focus: **Methodology** (how AI-driven development works through Claude Code)
   - Not focus: **Reference** (every flag, every config option, every edge case)

4. **NOT Changing Chapter's Foundational Role**
   - Chapter 5 remains **introduction** to agentic AI tools
   - Deep dives and advanced patterns come in later chapters
   - Appropriate depth for Part 1 (Foundations)

5. **NOT Optimizing for Speed Readers**
   - Hands-on exercises are mandatory (cannot be skipped)
   - Practice checkpoints enforce active learning
   - Pedagogical choice: depth > speed, mastery > coverage

6. **NOT Teaching Claude Code in Isolation**
   - Every feature connected to AI-driven development methodology
   - Claude Code is vehicle, paradigm shift is destination
   - Students learn methodology through tool, not tool alone

---

## Assumptions

1. **Students Have Prerequisites**
   - Completed Chapters 1-4 (motivation, context, basic terminal usage)
   - A2-B1 proficiency level (beginner transitioning to intermediate)
   - Access to computer for hands-on exercises (Windows, macOS, or Linux)

2. **Claude Code Version Stability**
   - Examples verified against current version (January 2025)
   - Core features (MCP, Skills, Hooks, Plugins) stable in API
   - If breaking changes occur, lessons will need updates

3. **Educational Context**
   - Book used for self-study or instructor-led courses
   - Students have 8-12 minutes per lesson (duration metadata guides)
   - Active learning environment (students actually DO exercises)

4. **Docusaurus Compatibility**
   - Frontmatter format remains Docusaurus-compatible
   - Sidebar navigation unaffected by metadata changes
   - Build process handles enhanced YAML fields

---

## Acceptance Criteria

### AC1: Metadata Validation Passes

**Given**: All 9 lesson files (01-09)
**When**: I run YAML parser validation script
**Then**:
- ✅ All required fields present (no `undefined`, no empty arrays)
- ✅ `stage` values are L1/L2/L3/L4 (no invalid values)
- ✅ `concept_count` within tier limits (L1-3: ≤7, L4-7: ≤9, L8-9: ≤10)
- ✅ `prerequisites` array not empty for lessons 2-9
- ✅ `learning_objectives` use action verbs (Create, Verify, Explain)
- ✅ `skills` metadata has all 5 subfields complete

---

### AC2: Stage Progression Is Explicit

**Given**: Chapter README.md
**When**: I read "Stage Progression" section
**Then**:
- ✅ Lessons 1-3 explicitly labeled "L1: Manual Foundation" with rationale
- ✅ Lessons 4-7 explicitly labeled "L2: AI Collaboration" with Three Roles explanation
- ✅ Lesson 8 explicitly labeled "L3: Intelligence Design" with team intelligence context
- ✅ Lesson 9 explicitly labeled "L4: Spec-Driven Composition" with capstone explanation
- ✅ Visual representation (diagram or clear description) of progression

---

### AC3: Content Meets Conciseness Targets

**Given**: All lesson files
**When**: I measure line counts and practice ratios
**Then**:
- ✅ No lesson exceeds 400 lines
- ✅ Average lesson length: 250-350 lines
- ✅ Each lesson has ≥3 hands-on exercises
- ✅ Theory-to-practice ratio ≥60% practice
- ✅ Verbose sections transformed to exercises (not just deleted)

---

### AC4: Methodology Framing Present

**Given**: Any lesson (random sample)
**When**: I search for "Why This Matters" section
**Then**:
- ✅ Section exists in all 9 lessons
- ✅ Section explains **workflow impact** (not just feature benefit)
- ✅ Section connects to AI-driven development paradigm explicitly
- ✅ Section is 50-150 words (concise but substantive)

---

### AC5: Cognitive Load Compliance Verified

**Given**: Lesson metadata `concept_count: N`
**When**: I manually count distinct concepts in lesson body
**Then**:
- ✅ Actual count matches metadata (±1 margin)
- ✅ Count within tier limits (A2: ≤7, B1: ≤10)
- ✅ Related concepts grouped in sections (logical chunking)
- ✅ Progressive disclosure used (simple→complex within lesson)

---

### AC6: Factual Accuracy Verified

**Given**: SANDBOX-AUDIT-REPORT.md
**When**: I review verification test results
**Then**:
- ✅ 100% code examples execute successfully (with logs)
- ✅ 100% CLI commands produce expected output
- ✅ 100% feature claims cite official docs (URLs provided)
- ✅ Zero unverified technical claims

---

### AC7: Three Roles Framework Demonstrated

**Given**: Lessons 4-7 (L2: AI Collaboration)
**When**: I review exercises
**Then**:
- ✅ Each lesson has ≥1 exercise demonstrating Three Roles cycle:
  - AI teaches student (suggests pattern student didn't know)
  - Student teaches AI (corrects/refines with domain knowledge)
  - Convergence loop (iterate toward better solution)
- ✅ Role transitions are explicit in exercise instructions
- ✅ Lesson 3 (CLAUDE.md) Three Roles example preserved

---

## Implementation Notes

### Phase 1: Metadata Enhancement (Lesson-by-Lesson)

**For each lesson (01-09)**:
1. Read current frontmatter
2. Add `stage` field based on lesson position
3. Add `prerequisites` array (concepts from previous lessons)
4. Add `concept_count` (manual count, then verify)
5. Enhance `learning_objectives` (use action verbs)
6. Complete `skills` metadata (verify all 5 subfields)
7. Update `duration` if content length changes significantly

**Estimated Effort**: ~20 minutes per lesson = 3 hours total

---

### Phase 2: Content Refinement (Targeted Edits)

**For each lesson**:
1. **Identify verbose sections** (paragraphs >100 words, abstract explanations)
2. **Transform to practice**:
   - Code example with "Try this" → Expected outcome
   - Hands-on exercise → Checkpoint validation
   - AI collaboration prompt → Three Roles demonstration
3. **Verify methodology framing**:
   - Check "Why This Matters" section exists and is substantive
   - Ensure it explains workflow impact, paradigm connection, real-world context
4. **Validate cognitive load**:
   - Count concepts manually
   - Verify ≤ tier limit
   - Update `concept_count` metadata

**Estimated Effort**: ~45 minutes per lesson = 6.75 hours total

---

### Phase 3: Chapter-Level Coherence

1. **Update README.md**:
   - Add "Stage Progression" section (L1→L2→L3→L4 with rationale)
   - Update "What You'll Learn" to reflect stage-based structure
   - Add visual representation of progression
2. **Cross-reference validation**:
   - Verify prerequisites chain correctly (no circular dependencies)
   - Ensure stage labels consistent (README ↔ lesson metadata)
3. **Verify capstone composition**:
   - Lesson 9 explicitly shows how plugins compose earlier lessons' concepts
   - Three Roles demonstrated in L2 (4-7), composition in L4 (9)

**Estimated Effort**: ~2 hours

---

### Phase 4: Validation & Verification

1. **Run acceptance tests** (AC1-AC7)
2. **Sandbox audit**:
   - Extract all code examples
   - Execute in clean environment
   - Document results in SANDBOX-AUDIT-REPORT.md
3. **Verify all CLI commands** against `claude --help`
4. **Verify all feature claims** against official docs (cite URLs)
5. **Generate validation report**
6. **Fix any failures** (iterate until all tests pass)

**Estimated Effort**: ~4 hours

---

**Total Estimated Effort**: ~16 hours (spread across implementation phase)

---

## Dependencies

### Upstream Dependencies
- **Chapters 1-4 Content**: Students must have completed prerequisites
- **Claude Code Installation**: Students need working Claude Code (Lesson 2 teaches this)
- **Docusaurus Build**: Frontmatter changes must be build-compatible

### Downstream Dependencies
- **Later Chapters**: Advanced features (custom subagents, MCP development) taught in Part 5+
- **Assessment Design**: Quizzes/exercises in later sections may reference Chapter 5 concepts

### External Dependencies
- **Claude Code Official Docs**: https://code.claude.com/docs (for verification)
- **Anthropic Skills Repository**: https://github.com/anthropics/skills (referenced in Lesson 9)
- **Constitutional Framework**: `.specify/memory/constitution.md` (v6.0.0) — may not exist in this repo

---

## Risks & Mitigations

### Risk 1: Metadata Changes Break Docusaurus Build

**Likelihood**: Low
**Impact**: High (blocks site deployment)

**Mitigation**:
- Test frontmatter changes in local Docusaurus build before committing
- Validate YAML syntax with parser
- Keep sidebar_position, title intact (known working fields)

---

### Risk 2: Content Reduction Loses Essential Explanations

**Likelihood**: Medium
**Impact**: Medium (students miss key concepts)

**Mitigation**:
- Transform, don't just delete (verbose paragraph → concise example)
- Preserve WHAT/WHY/WHEN sections (methodology framing)
- Run user scenario test: can student complete exercises with reduced content?

---

### Risk 3: Cognitive Load Counts Subjective

**Likelihood**: Medium
**Impact**: Low (metadata slightly inaccurate)

**Mitigation**:
- Use consistent definition: "distinct concept = new term + new command + new workflow + new decision framework"
- Allow ±1 margin in acceptance criteria
- Document counting methodology in implementation notes

---

### Risk 4: Claude Code Features Change Post-Verification

**Likelihood**: Low
**Impact**: Medium (examples break for students)

**Mitigation**:
- Document Claude Code version used for verification
- Add note in README: "Verified for Claude Code vX.Y (Jan 2025)"
- Plan periodic re-verification (quarterly?)

---

## Success Metrics

### Quantitative
- ✅ **Metadata**: 9/9 lessons with complete frontmatter (100%)
- ✅ **Conciseness**: Average lesson ≤350 lines (target: 250-350)
- ✅ **Practice Ratio**: ≥60% hands-on exercises per lesson
- ✅ **Cognitive Load**: 9/9 lessons within tier limits (100%)
- ✅ **Verification**: 100% code tested, 100% claims cited

### Qualitative
- ✅ **Stage Progression**: Explicit and visible (README + lesson metadata)
- ✅ **Methodology Framing**: Every lesson connects to AI-driven development paradigm
- ✅ **Three Roles**: Demonstrated in L2 lessons (4-7)
- ✅ **Constitutional Compliance**: All 7 principles applied

### Pedagogical
- ✅ **Accessibility**: DigComp 2.2 + Bloom's taxonomy + CEFR alignment
- ✅ **Active Learning**: Show through doing principle enforced
- ✅ **Progressive Complexity**: L1→L2→L3→L4 progression clear
- ✅ **Factual Accuracy**: All verification protocols passed

---

## References

### Constitutional Foundation
- **Constitution**: `.specify/memory/constitution.md` (v6.0.0 — if exists)
- **4-Stage Teaching Framework**: L1 (Manual) → L2 (AI Collaboration) → L3 (Intelligence Design) → L4 (Spec-Driven)
- **7 Core Principles**: Specification Primacy, Progressive Complexity, Factual Accuracy, Coherent Structure, Intelligence Accumulation, Anti-Convergence, Minimal Content

### Official Documentation
- **Claude Code Docs**: https://code.claude.com/docs
- **MCP Specification**: https://modelcontextprotocol.io
- **Anthropic Skills**: https://github.com/anthropics/skills

### Project Context
- **Current Chapter**: `docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/`
- **9 Lessons**: Already have good structure, need refinement not rewrite
- **User Decisions**: Enhanced metadata approved, skill reference material stays in L09

### Accessibility Standards
- **DigComp 2.2**: Digital Competence Framework for Citizens
- **Bloom's Taxonomy**: Knowledge dimension + Cognitive process
- **CEFR**: Common European Framework of Reference (proficiency levels)

---

## Approval Criteria

**Specification is ready for planning when**:
- ✅ Success criteria are measurable and falsifiable (can objectively test pass/fail)
- ✅ Intent clearly defines WHAT (refine chapter) and WHY (methodology over documentation)
- ✅ Constraints define boundaries (must preserve, must NOT do)
- ✅ Non-goals prevent scope creep (not new lessons, not advanced features, not comprehensive docs)
- ✅ Acceptance tests provide objective validation methods
- ✅ Implementation strategy outlines phases (not detailed line-by-line edits)
- ✅ Reasoning-activated: spec explains WHY each refinement matters for learning outcomes

**Next Phase**: Create lesson-by-lesson plan mapping refinement actions to specific lessons with pedagogical rationale.

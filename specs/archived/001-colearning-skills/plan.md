# Implementation Plan: Colearning Skills for Python Book Agent

**Branch**: `001-colearning-skills` | **Date**: 2025-10-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/001-colearning-skills/spec.md`

---

## Summary

Build 6 specialized Claude Code skills for educational content development, enabling educators and authors to create pedagogically sound Python learning materials. Skills use Anthropic's progressive disclosure architecture (3 layers: metadata → instructions → resources) and apply evidence-based teaching strategies (Bloom's taxonomy, Cognitive Load Theory, scaffolding patterns).

**Primary Requirement**: Create model-invoked skills that automatically activate based on educator context, providing targeted guidance for learning objectives, concept scaffolding, code examples, exercises, technical clarity, and assessments.

**Technical Approach**: Implement skills as SKILL.md files with supporting resources (reference/, templates/, scripts/) following Anthropic's official patterns. Use Python 3.13+ for validation scripts, YAML for structured data, Markdown for documentation. Validate code examples via isolated subprocess sandbox with security constraints.

---

## Technical Context

**Language/Version**: Python 3.13+ (for validation scripts and sandbox execution)
**Primary Dependencies**:
- Claude Code (model-invoked skills support, Claude 3.5 Sonnet+)
- Python standard library (subprocess, ast, pathlib, tempfile, typing)
- No external packages required (standard library only)

**Storage**: Local filesystem (`.claude/skills/` directory, stateless operation)
**Testing**:
- Unit tests: pytest for validation scripts
- Integration tests: Skill activation scenarios
- Validation: YAML schema validation, sandbox execution tests

**Target Platform**: Cross-platform (Windows/macOS/Linux) - Claude Code environment
**Project Type**: Claude Code Skills (not standalone application)
**Performance Goals**:
- learning-objectives: <30s for 3-5 objectives
- concept-scaffolding: <45s for 3-7 steps
- code-example-generator: <20s per example (includes sandbox validation)
- exercise-designer: <60s for 3-5 exercises
- technical-clarity: <30s for 500-word analysis
- assessment-builder: <60s for 5-10 questions

**Constraints**:
- Hard failure mode (skills halt on errors, require user intervention per clarification #1)
- No external API/database dependencies (stateless, local only)
- Sandbox execution must prevent file system escape, network access, resource overconsumption
- Context efficiency via progressive disclosure (only load needed files)
- Mutually exclusive skill descriptions (prevent activation conflicts)

**Scale/Scope**: 6 core skills, each with 3-5 supporting files, estimated 30-40 total files across `.claude/skills/` directory

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Project Constitution

**Principle 1: AI-First Teaching Philosophy** ✅
- Skills ARE the AI-first tooling for educators
- Demonstrates Claude Code skills as example of AI collaboration
- Aligns: Skills help educators create AI-augmented learning materials

**Principle 2: Spec-Kit Methodology** ✅
- This feature itself follows SDD: spec → plan → tasks → implementation
- Aligns: Meta-demonstration of the methodology

**Principle 3: Modern Python Standards (3.13+)** ✅
- All validation scripts use Python 3.13+ with mandatory type hints
- code-example-generator enforces PEP 8, type hints, modern syntax
- Aligns: Skills enforce modern standards in generated content

**Principle 4: Test-First Mindset** ✅
- exercise-designer skill creates test cases for exercises
- code-example-generator validates via execution
- Assessment-builder includes test cases for coding questions
- Aligns: Skills embed testing in workflow

**Principle 5: Progressive Complexity with Clear Scaffolding** ✅
- concept-scaffolding skill implements CLT-based progressive introduction
- learning-objectives identifies prerequisites
- Aligns: Skills help educators apply this principle

**Principle 6: Consistent Structure** ✅
- All 6 skills follow identical directory structure
- Shared validation script patterns
- Consistent SKILL.md format
- Aligns: Infrastructure enables consistency

**Principle 7: Technical Accuracy (Always Verified)** ✅
- Sandbox execution validates code examples run correctly
- technical-clarity checks for accuracy
- Hard failure prevents inaccurate output
- Aligns: Verification is automated, not optional

**Principle 8: Accessibility and Inclusivity** ✅
- technical-clarity detects gatekeeping language
- Readability metrics ensure appropriate complexity
- Aligns: Skills help enforce accessibility standards

**Principle 9: Show-Then-Explain Pedagogy** ✅
- code-example-generator provides working code FIRST, then explanation (what/how/why)
- Aligns: Output structure enforces this pattern

**Principle 10: Real-World Project Integration** ✅
- Skills are real Claude Code skills (not toy examples)
- Used in actual book development workflow
- Aligns: Meta-example of professional tooling

**Principle 11: Tool Diversity** ⚠️ DEFERRED
- Skills support educators using ANY AI tool for content creation
- No tool lock-in (skills are Claude Code-specific but outputs are universal)
- Partial alignment: Skills demonstrate ONE AI tool ecosystem (Claude Code skills)

### Gates

| Gate | Status | Notes |
|------|--------|-------|
| **Spec Clarity** | ✅ PASS | All clarifications resolved (5 questions answered) |
| **Constitution Alignment** | ✅ PASS | All relevant principles satisfied |
| **Technical Feasibility** | ✅ PASS | Subprocess sandbox confirmed implementable |
| **Resource Constraints** | ✅ PASS | No external dependencies, local execution only |
| **Performance Targets** | ✅ PASS | Targets achievable with progressive disclosure |

**Proceed to Phase 0 Research** ✅

---

## Project Structure

### Documentation (this feature)

```text
specs/001-colearning-skills/
├── spec.md                   # Feature requirements (COMPLETE)
├── plan.md                   # This file (/sp.plan output - IN PROGRESS)
├── research.md               # Phase 0 output (COMPLETE)
├── data-model.md             # Phase 1 output (COMPLETE)
├── quickstart.md             # Phase 1 output (COMPLETE)
├── contracts/                # Phase 1 output (COMPLETE)
│   ├── skill-metadata.schema.yml
│   ├── learning-objective.schema.yml
│   └── code-example.schema.yml
├── prompts/                  # PHR records
│   ├── 001-refine-skills-spec-architecture.spec.prompt.md
│   ├── 002-clarify-spec-ambiguities.spec.prompt.md
│   └── [003-plan-skills-architecture.plan.prompt.md - PENDING]
└── tasks.md                  # Phase 2 output (/sp.tasks - NOT YET CREATED)
```

### Source Code (repository root)

```text
.claude/skills/               # All skills live here (created during implementation)
├── learning-objectives/
│   ├── SKILL.md                          # Core instructions (Layer 2)
│   ├── reference/                         # Documentation (Layer 3)
│   │   ├── blooms-taxonomy-programming.md
│   │   ├── prerequisite-analysis.md
│   │   └── assessment-methods.md
│   ├── templates/                         # Output structures (Layer 3)
│   │   └── learning-objective-template.yml
│   └── scripts/                           # Validation (Layer 3)
│       └── validate-objectives.py
│
├── concept-scaffolding/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── cognitive-load-theory.md
│   │   ├── scaffolding-strategies.md
│   │   └── worked-examples.md
│   ├── templates/
│   │   └── scaffolding-plan-template.yml
│   └── scripts/
│       └── assess-cognitive-load.py
│
├── code-example-generator/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── python-best-practices.md
│   │   ├── example-patterns.md
│   │   └── pep8-summary.md
│   ├── templates/
│   │   └── code-example-template.md
│   └── scripts/
│       ├── validate-syntax.py
│       └── execute-sandbox.py
│
├── exercise-designer/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── exercise-types.md
│   │   ├── evidence-based-strategies.md
│   │   ├── difficulty-progression.md
│   │   └── spaced-repetition.md
│   ├── templates/
│   │   ├── exercise-template.yml
│   │   └── rubric-template.yml
│   └── scripts/
│       └── generate-test-cases.py
│
├── technical-clarity/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── readability-metrics.md
│   │   ├── analogy-patterns.md
│   │   ├── clarity-checklist.md
│   │   └── gatekeeping-language.md
│   ├── templates/
│   │   └── clarity-report-template.yml
│   └── scripts/
│       ├── analyze-readability.py
│       └── check-completeness.py
│
└── assessment-builder/
    ├── SKILL.md
    ├── reference/
    │   ├── question-types.md
    │   ├── distractor-design.md
    │   ├── rubric-guidelines.md
    │   └── blooms-assessment-alignment.md
    ├── templates/
    │   ├── assessment-template.yml
    │   └── rubric-template.yml
    └── scripts/
        └── validate-assessment.py

tests/                        # Test suite for skills
├── unit/                     # Script unit tests
│   ├── test_sandbox.py
│   ├── test_syntax_validator.py
│   └── test_readability_analyzer.py
├── integration/              # Skill activation tests
│   ├── test_learning_objectives_activation.py
│   ├── test_code_example_generator.py
│   └── test_sequential_skills.py
└── fixtures/                 # Test data
    ├── sample_concepts.yml
    ├── sample_code_examples.py
    └── sample_explanations.md
```

**Structure Decision**: Single project structure with skills in `.claude/skills/` (standard Claude Code location). Tests in project root `tests/` directory following standard Python conventions.

---

## Complexity Tracking

> **No violations detected** - Constitution check passed without justification needed.

---

## Architecture Decisions

### 1. Progressive Disclosure Implementation

**Decision**: Use Anthropic's 3-layer architecture with filesystem-based progressive loading

**Rationale**:
- Official Anthropic pattern (October 2025)
- Unbounded context capacity (only load what's needed)
- Reduces token cost and improves performance

**Implementation**:
- **Layer 1**: SKILL.md frontmatter (loaded at startup for all skills) - includes `name`, `description` (semantic, contextual), `allowed-tools`
- **Layer 2**: Full SKILL.md body (loaded when skill activates) - contains instructions that explicitly reference Layer 3 files
- **Layer 3**: Supporting files (loaded ONLY when SKILL.md references them) - includes reference docs, templates, and scripts

**Key Pattern**: SKILL.md instructions explicitly direct Claude:
- When to load reference docs: "Read [reference/blooms-taxonomy.md](reference/blooms-taxonomy.md) for guidance"
- When to invoke scripts via Bash tool: `python scripts/validate-objectives.py <file>`
- How to interpret script outputs and proceed

**Alternatives Rejected**:
- Monolithic SKILL.md: Would bloat context, violate progressive disclosure principle
- External API calls: Violates "no external dependencies" constraint (FR-033)
- Auto-executing scripts: Not supported by Claude Code skills architecture

**Source**: research.md §1, Anthropic engineering blog, Claude Code skills documentation

---

### 2. Sandbox Execution Strategy

**Decision**: Python subprocess with security constraints (timeout, isolated temp directory, no network)

**Rationale**:
- User explicitly requested sandbox execution (clarification #3)
- Simpler than Docker for educational code snippets
- Sufficient security for validation use case (not running untrusted user code)
- Cross-platform compatible

**Implementation**:
```python
subprocess.run(
    ["python3", code_file],
    capture_output=True,
    timeout=5,  # Hard timeout
    cwd=temp_dir,  # Isolated directory
    env=minimal_env  # No credentials
)
```

**Security Constraints**:
- Timeout: 5 seconds maximum
- File system: Restricted to temp directory (auto-deleted after validation)
- Network: No network access (subprocess inherits no credentials)
- Resource limits: OS-level (subprocess doesn't set additional limits initially; enhancement for future)

**Alternatives Rejected**:
- AST-only validation: My recommendation, but user chose execution
- Docker containers: Over-engineered for initial implementation; could be future enhancement
- Manual review only: Violates FR-011 requirement for automated validation

**Source**: research.md §5, clarification #3

---

### 3. Semantic Skill Activation (NOT Keyword-Based)

**Decision**: Skills use semantic activation based on contextual understanding, NOT keyword/trigger phrase matching

**Rationale**:
- Claude Code skills are model-invoked: Claude understands context semantically
- Descriptions should explain WHEN and WHY to use skill (contextual scenarios)
- Keyword lists don't work and create brittle activation patterns
- Aligns with Anthropic's official skills documentation

**Implementation**:
- Description field contains semantic explanation: "Generate measurable learning outcomes aligned with Bloom's taxonomy for educational content. Use when educators need to define what students will achieve..."
- NO keyword trigger lists: ❌ "Triggers: 'define objectives', 'learning goals', 'prerequisites'"
- Claude evaluates entire request context against semantic description
- Integration tests validate semantic activation with varied phrasings

**Alternatives Rejected**:
- Keyword/trigger phrase matching: Not how Claude Code skills work
- Explicit user invocation only: Loses model-invoked convenience
- Regex patterns: Over-engineered and doesn't leverage LLM semantic understanding

**Source**: Claude Code skills documentation (https://docs.claude.com/en/history/claude-code/skills)

---

### 4. Skill Conflict Resolution

**Decision**: Mutually exclusive semantic descriptions + sequential activation when multiple skills serve different purposes

**Rationale**:
- User chose A + D (clarification #5)
- Aligns with Anthropic guidance on specific contextual differentiation
- Enables powerful multi-skill workflows

**Implementation**:
- Each SKILL.md description uses distinct contextual scenarios
- Example: "generate runnable teaching examples" (code-example-generator) vs "review explanations for clarity and accessibility" (technical-clarity)
- Claude's model-invocation logic handles sequential activation automatically when multiple skills address different aspects
- Outputs combined coherently in single response

**Alternatives Rejected**:
- User prompt for disambiguation: Interrupts workflow
- Last-activated wins: Loses valuable multi-skill capabilities
- All skills always run: Performance penalty, context bloat

**Source**: research.md §6, clarification #5

---

### 5. Output Format Flexibility

**Decision**: No prescribed format - each skill uses structure optimal for its purpose

**Rationale**:
- User clarified no format prescription (clarification #4)
- Different output types benefit from different structures:
  - learning-objectives: YAML (machine-readable, structured)
  - code-example-generator: Fenced code blocks + markdown explanation
  - technical-clarity: Structured feedback with sections

**Implementation**:
- No cross-skill format enforcement
- Consistency WITHIN each skill
- Clear documentation of each skill's output format in SKILL.md

**Alternatives Rejected**:
- Uniform YAML across all skills: Would force awkward structures for some outputs
- Uniform markdown: Loses structured data benefits for machine-readable outputs
- JSON only: Poor readability for educators

**Source**: research.md §7, clarification #4

---

### 6. Pedagogical Foundation Selection

**Decision**: Evidence-based strategies from Bloom's Taxonomy and Cognitive Load Theory

**Rationale**:
- Research-backed, not opinion-based
- Direct application to programming education (not general theory)
- Measurable outcomes (aligns with success criteria SC-005 to SC-015)

**Key Frameworks**:
1. **Bloom's Taxonomy**: 6 cognitive levels (Remember → Create)
2. **Cognitive Load Theory**: Intrinsic + Extraneous + Germane load
3. **Scaffolding Patterns**: Worked examples, faded guidance, chunking

**Implementation**:
- learning-objectives: Maps objectives to Bloom's levels
- concept-scaffolding: Applies CLT principles (max 7 steps, 3 concepts per step)
- exercise-designer: Uses retrieval practice, spaced repetition, interleaving
- assessment-builder: Targets higher-order thinking (not just recall)

**Evidence**:
- ACL 2025: LLMs underserve Metacognitive/Create/Evaluate levels
- ResearchGate 2020: Scaffolding reduces cognitive load, improves scores
- British Journal of Ed Tech 2025: AI-enabled hints optimize cognitive load

**Source**: research.md §4

---

### 7. File Organization Pattern

**Decision**: Type-based directories (reference/, templates/, scripts/) within each skill

**Rationale**:
- Clear separation of concerns
- Easy discovery (know where to add new resources)
- Scalable (6-8 skills × multiple files each)
- Aligns with clarification #2 (markdown/Python/YAML formats)

**Structure**:
```
skill-name/
├── SKILL.md            # Instructions
├── reference/          # Markdown docs
├── templates/          # YAML/Markdown templates
└── scripts/            # Python validation
```

**Alternatives Rejected**:
- Flat structure: All files in root → poor scalability
- Function-based: Group by purpose not type → unclear discovery pattern
- Deep hierarchy: reference/concepts/, reference/patterns/ → over-engineered

**Source**: research.md §3

---

## Implementation Strategy

### Phase 0: Research ✅ COMPLETE

**Artifacts Created**:
- `research.md`: Consolidated findings from Anthropic docs, pedagogy research
- Decisions documented: architecture, sandbox, pedagogy, file organization

**Key Findings**:
- Progressive disclosure is the core pattern
- Bloom's + CLT provide evidence-based pedagogy foundation
- Subprocess sandbox balances security and simplicity
- Mutually exclusive descriptions prevent conflicts

---

### Phase 1: Design ✅ COMPLETE

**Artifacts Created**:
- `data-model.md`: Entity structures for all 6 skills
- `contracts/`: YAML schemas for validation
  - `skill-metadata.schema.yml` (Layer 1 frontmatter)
  - `learning-objective.schema.yml` (learning-objectives output)
  - `code-example.schema.yml` (code-example-generator output)
- `quickstart.md`: User-facing usage guide

**Key Deliverables**:
- Detailed data models for 6 skill outputs
- Validation rules and constraints
- File system structure defined
- Example interactions documented

---

### Phase 2: Task Decomposition (NEXT STEP)

**Command**: `/sp.tasks`

**Expected Artifacts**:
- `tasks.md`: Dependency-ordered task list with acceptance criteria

**Scope**:
1. Create base directory structure (`.claude/skills/`)
2. Implement SKILL.md files for each of 6 skills
3. Create supporting reference documentation
4. Build validation scripts (Python 3.13+)
5. Create templates (YAML, Markdown)
6. Write unit tests for scripts
7. Write integration tests for skill activation
8. Document activation patterns and troubleshooting

**Estimated Task Count**: 35-45 tasks across 6 skills

---

### Phase 3: Implementation (FUTURE)

**Execution Method**: `/sp.implement` command

**Implementation Order** (dependency-aware):
1. **Foundational**: shared utilities, sandbox script (used by other skills)
2. **Core skills**: learning-objectives, code-example-generator (most foundational)
3. **Advanced skills**: concept-scaffolding, exercise-designer, technical-clarity, assessment-builder
4. **Testing**: unit tests → integration tests → validation
5. **Documentation**: finalize quickstart, add troubleshooting

---

## Risks & Mitigations

### Risk 1: Sandbox Execution Performance

**Risk**: Subprocess execution adds latency, might exceed 20s target for code-example-generator

**Mitigation**:
- Optimize code examples to run quickly (<5s execution)
- Cache validation results for identical code
- Flag complex examples for manual review instead of auto-validation
- Monitor SC-007 metric (>95% success rate)

**Contingency**: Add Docker option for complex validation if subprocess insufficient

---

### Risk 2: Skill Activation Accuracy

**Risk**: Claude doesn't activate skills when expected (poor description matching)

**Mitigation**:
- Use very specific trigger phrases in descriptions
- Include domain terminology that appears in user requests
- Test activation with varied phrasings
- Iterate on descriptions based on real usage

**Contingency**: Allow explicit skill invocation ("Use learning-objectives skill...")

---

### Risk 3: Context Window Overflow

**Risk**: Progressive disclosure fails, too many files loaded simultaneously

**Mitigation**:
- Keep SKILL.md concise (reference other files, don't inline everything)
- Limit reference file size (<2000 lines each)
- Test with multiple skills activating sequentially
- Monitor token usage during integration tests

**Contingency**: Split large reference files into smaller topic-specific files

---

### Risk 4: Cross-Platform Compatibility

**Risk**: Subprocess behavior differs on Windows vs Unix-based systems

**Mitigation**:
- Use `pathlib` for cross-platform paths
- Test on Windows, macOS, Linux
- Use `python3` explicitly (not `python`)
- Handle platform-specific line endings (CRLF vs LF)

**Contingency**: Add platform-specific handling in sandbox script if needed

---

### Risk 5: Hard Failure Mode Impact

**Risk**: Hard failures frustrate users, block workflows

**Mitigation**:
- Provide extremely clear error messages with suggested fixes
- Validate aggressively before reaching user-facing failure
- Include helpful troubleshooting in quickstart.md
- Log errors for analysis and improvement

**Contingency**: Add "soft failure" mode as optional config if hard failure too disruptive

---

## Success Criteria Alignment

**From spec.md SC-001 to SC-015:**

| Criterion | Target | Plan Alignment |
|-----------|--------|----------------|
| **SC-001** | 6 skills with SKILL.md frontmatter | ✅ All 6 designed with proper metadata |
| **SC-002** | 2-3 supporting files per skill | ✅ reference/, templates/, scripts/ structure |
| **SC-003** | >80% activation accuracy | ✅ Mutually exclusive descriptions designed |
| **SC-004** | Skills discoverable | ✅ quickstart.md provides guidance |
| **SC-005** | learning-objectives <30s | ✅ Minimal computation, template-driven |
| **SC-006** | concept-scaffolding <45s | ✅ CLT algorithm designed, lightweight |
| **SC-007** | code-example-generator >95% success | ✅ Sandbox validation ensures correctness |
| **SC-008** | exercise-designer <60s | ✅ Template-based generation, bounded complexity |
| **SC-009** | technical-clarity >80% issue detection | ✅ Readability scripts + heuristics |
| **SC-010** | assessment-builder meaningful distractors | ✅ Distractor design reference included |
| **SC-011** | Reduced time defining objectives | ✅ Automation via skill |
| **SC-012** | Code examples >4/5 rating | ✅ Validation ensures quality baseline |
| **SC-013** | ≥2 evidence-based strategies per exercise | ✅ exercise-designer enforces |
| **SC-014** | Improved readability scores | ✅ technical-clarity measures Flesch-Kincaid |
| **SC-015** | Improved educator confidence | ✅ Consistent, validated outputs build trust |

---

## Open Questions for Phase 2 (Tasks)

1. **Shared Utilities**: Create common `utils/` directory for code reused across skills (e.g., YAML loading, validation helpers)?
   - **Recommendation**: Yes, create `.claude/skills/_shared/` for common utilities

2. **Skill Testing Framework**: How to systematically test skill activation accuracy?
   - **Recommendation**: Integration tests with varied prompt phrasings, measure activation rate

3. **Version Management**: Track skill versions for iteration and A/B testing?
   - **Recommendation**: Add `version` field to SKILL.md frontmatter (semver format)

4. **Docker Enhancement**: When to add Docker support for sandbox execution?
   - **Recommendation**: Defer to post-MVP; assess if subprocess sufficient first

These will be resolved during task decomposition (/sp.tasks).

---

## Next Steps

1. **Review this plan**: Validate architecture decisions, ask questions, suggest changes
2. **Run `/sp.tasks`**: Generate dependency-ordered task list with acceptance criteria
3. **Implement incrementally**: Start with foundational tasks (sandbox, shared utilities)
4. **Test continuously**: Unit tests → integration tests → real-world validation
5. **Iterate on descriptions**: Refine skill activation based on usage patterns

---

**Plan Complete** ✅
**Phase**: Phase 1 Design Complete → Ready for Phase 2 (Task Decomposition)
**Command to Continue**: `/sp.tasks`

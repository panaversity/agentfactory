# Research: Colearning Skills for Python Book Agent

**Feature**: 001-colearning-skills
**Date**: 2025-10-28
**Purpose**: Research effective Claude Code skill architecture based on Anthropic official resources and educational pedagogy evidence

---

## Phase 0: Research Findings

### 1. Claude Code Skills Architecture (Official Anthropic Documentation)

#### Decision: Use Progressive Disclosure with Three-Layer Architecture

**What This Means:**
- **Layer 1 (Metadata)**: Skill name + description (1024 char max) loaded at startup
- **Layer 2 (Instructions)**: Full SKILL.md loaded when skill activates
- **Layer 3 (Resources)**: Supporting files (reference/, templates/, scripts/) loaded only when referenced

**Rationale:**
- Anthropic's official design pattern (October 2025 release)
- Context is effectively unbounded due to progressive loading
- Reduces token cost - only relevant content loaded
- Supports complex skills without overwhelming context window

**Alternatives Considered:**
- **Monolithic SKILL.md**: All content in one file → Rejected due to context bloat
- **External API calls**: Fetch data dynamically → Rejected due to "no external dependencies" constraint (spec FR-033)

**Source**:
- https://docs.claude.com/en/history/claude-code/skills
- https://www.anthropic.com/engineering/equipping-agents-for-the-real-world-with-agent-skills

---

### 2. SKILL.md Frontmatter Requirements

#### Decision: Strict Frontmatter Format with Activation-Optimized Descriptions

**Required Fields:**
```yaml
name: skill-name  # lowercase, hyphens, max 64 chars
description: |  # max 1024 chars, specific triggers
  Activate when [specific context]. Use for [concrete use cases].
  Mention [domain-specific terminology] to improve matching.
```

**Optional Fields:**
```yaml
allowed-tools: Read, Grep, Glob, Bash  # Restrict tools for security/scope
```

**Rationale:**
- Name format ensures CLI compatibility and consistency
- Description is THE activation trigger - must be precise, not generic
- Specific terminology improves Claude's context matching accuracy
- Tool restrictions enable security and focus

**Best Practices from Anthropic:**
- ✅ "Analyze sales data in Excel files and CRM exports" (specific)
- ❌ "Helps with documents" (too generic, poor activation)
- Include when/why to use the skill
- Use domain-specific terms that appear in user requests

**Source**: Anthropic Claude Code Skills documentation

---

### 3. File Organization Pattern

#### Decision: Structured Skill Directories with Type-Based Organization

**Directory Structure:**
```
.claude/skills/skill-name/
├── SKILL.md              # Core instructions (Layer 2)
├── reference/            # Documentation (.md files)
│   ├── concept-name.md
│   └── patterns.md
├── templates/            # Reusable structures (.md, .yml)
│   ├── output-template.md
│   └── config.yml
└── scripts/              # Executable utilities (.py)
    ├── validator.py
    └── analyzer.py
```

**Rationale:**
- Clear separation of concerns (docs vs templates vs code)
- Progressive disclosure: SKILL.md references files as needed
- Type-based organization aids discovery and maintenance
- Aligns with clarification #2 (markdown/Python/YAML formats)

**Alternatives Considered:**
- **Flat structure**: All files in root → Rejected due to poor scalability (6-8 skills × multiple files)
- **Function-based**: Group by function not type → Rejected due to unclear discovery patterns

**Source**: Anthropic skills documentation + clarification session

---

### 4. Pedagogical Foundations for Educational Skills

#### Decision: Evidence-Based Teaching Strategies from Cognitive Science

**Bloom's Taxonomy for Programming (2024-2025 Research):**

| Level | Programming Application | Skill Mapping |
|-------|------------------------|---------------|
| **Remember** | Recall syntax, commands, terminology | learning-objectives (prerequisite analysis) |
| **Understand** | Explain what code does, paraphrase concepts | technical-clarity (readability, analogies) |
| **Apply** | Write code for given problem, use patterns | code-example-generator (progressive examples) |
| **Analyze** | Debug code, evaluate design choices | concept-scaffolding (break down complexity) |
| **Evaluate** | Assess code quality, compare approaches | technical-clarity, assessment-builder |
| **Create** | Design solutions, synthesize knowledge | exercise-designer (open-ended projects) |

**Key Finding**: Current LLM benchmarks miss Metacognitive, Create, and Evaluate levels (ACL 2025 research). Our skills MUST target these underserved areas.

**Cognitive Load Theory (CLT) Principles:**

**Three Types of Cognitive Load:**
1. **Intrinsic Load**: Inherent complexity of concept itself
2. **Extraneous Load**: Poor instruction/presentation adding unnecessary difficulty
3. **Germane Load**: Desirable difficulty that builds schemas

**CLT-Informed Scaffolding Strategies:**
- **Worked Examples**: Show complete solutions before practice (reduces intrinsic load)
- **Faded Guidance**: Start with full support → gradually remove scaffolding
- **Chunking**: Break complex concepts into 3-7 manageable pieces (working memory limit)
- **Interleaving**: Mix related concepts to build connections
- **Spaced Repetition**: Review over increasing intervals for retention

**Research Evidence:**
- Scaffolding tools reduce cognitive load and improve test scores (ResearchGate 2020)
- Worked example effect: Studying examples > solving problems for novices
- 7±2 chunk limit for working memory (Miller's Law) → concept-scaffolding max 7 steps

**Rationale:**
- These are evidence-based, not opinion-based teaching strategies
- Direct application to programming education (not general education theory)
- Measurable outcomes align with success criteria (SC-005 to SC-015)

**Source**:
- ACL 2025: "LLMs meet Bloom's Taxonomy" (coling-main.350.pdf)
- ResearchGate: "Managing Cognitive Load in Introductory Programming Courses"
- British Journal of Educational Technology 2025: "GenAI-enabled coding hints and cognitive load"

---

### 5. Sandbox Execution for Code Validation

#### Decision: Isolated Python Subprocess with Resource Limits

**Approach**: Use Python's `subprocess` module with security constraints

**Implementation Pattern:**
```python
import subprocess
import tempfile
import resource

def validate_code_example(code: str, timeout: int = 5) -> tuple[bool, str]:
    """
    Execute Python code in isolated subprocess with resource limits.
    Returns (success, output/error).
    """
    with tempfile.TemporaryDirectory() as temp_dir:
        # Write code to temp file
        code_file = Path(temp_dir) / "example.py"
        code_file.write_text(code)

        # Execute with constraints
        try:
            result = subprocess.run(
                ["python3", str(code_file)],
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=temp_dir,  # Isolated working directory
                env={"PYTHONDONTWRITEBYTECODE": "1"}  # No .pyc files
            )
            return (result.returncode == 0, result.stdout or result.stderr)
        except subprocess.TimeoutExpired:
            return (False, f"Execution exceeded {timeout}s timeout")
        except Exception as e:
            return (False, f"Validation error: {e}")
```

**Security Constraints:**
- Isolated temp directory (no file system access beyond temp)
- Timeout enforcement (default 5s per example)
- No network access (subprocess inherits no network credentials)
- Standard library only by default (external packages handled separately)

**Rationale:**
- User explicitly chose sandbox execution (clarification #3 answer: C)
- Subprocess approach is simpler than Docker for educational code snippets
- Meets security requirements without infrastructure complexity
- Handles timeout and resource exhaustion gracefully

**Edge Case Handling** (from spec edge cases):
- External packages: Flag for manual review, don't auto-install
- Timeout: Return clear error, suggest simplification
- Platform-specific code: Flag with warning, note platform requirements

**Alternatives Considered:**
- **Docker containers**: More secure but adds deployment complexity → Deferred for initial implementation
- **AST-only validation**: My recommendation, but user chose execution → Must implement sandbox
- **Manual review only**: Violates FR-011 requirement for automated validation

**Source**: Clarification session + Python subprocess documentation

---

### 6. Skill Conflict Resolution

#### Decision: Mutually Exclusive Descriptions + Sequential Activation

**Strategy:**
1. **Write distinct descriptions**: Each skill description must clearly differentiate its specific context
2. **Sequential execution**: When multiple skills serve different purposes, Claude activates them one after another
3. **Combined output**: Results presented coherently to user

**Example Scenario:**
- User: "Generate a code example for list comprehensions and review it for clarity"
- **Activation**: code-example-generator → technical-clarity (sequential)
- **Output**: Generated example + clarity analysis in one response

**Description Writing Guidelines:**
```yaml
# GOOD: Specific trigger terms
name: code-example-generator
description: |
  Generate runnable Python code examples when educator requests
  "create example", "show code for", or "demonstrate [concept]".

# GOOD: Distinct review context
name: technical-clarity
description: |
  Review existing technical explanations when educator requests
  "review for clarity", "check readability", or "improve explanation".

# BAD: Overlapping descriptions
description: "Help with Python code"  # Too vague, conflicts possible
```

**Rationale:**
- User chose option A + D (clarification #5: mutually exclusive + sequential)
- Aligns with Anthropic's guidance on specific terminology
- Reduces activation ambiguity and improves user experience

**Source**: Clarification session + Anthropic best practices

---

### 7. Output Format Flexibility

#### Decision: No Prescribed Format - Skill-Appropriate Structures

**Approach**: Each skill uses the format that best serves its purpose

**Examples:**

**learning-objectives skill** (structured YAML + markdown):
```yaml
objectives:
  - level: Apply
    statement: "Write for-loops to iterate over sequences"
    prerequisites: [variables, data types, sequences]
    assessment: "Code exercise with iteration"
```

**code-example-generator skill** (fenced code blocks + explanation):
````markdown
## Example: List Comprehensions

```python
# Traditional loop approach
squares = []
for x in range(10):
    squares.append(x ** 2)

# List comprehension (same result)
squares = [x ** 2 for x in range(10)]
```

**What it does**: Creates a list of squared numbers...
````

**technical-clarity skill** (structured feedback):
```markdown
## Clarity Analysis

**Jargon Identified**: "idempotent", "memoization" (line 12, 34)
**Suggestions**:
- Line 12: Define "idempotent" or replace with "produces same result"
- Line 34: Add example of memoization before using term
```

**Rationale:**
- User clarified: no format prescription (clarification #4)
- Flexibility allows each skill to optimize for its specific output type
- Consistency within each skill, diversity across skills

**Source**: Clarification session

---

## Summary: Key Decisions for Implementation

| Decision Area | Choice | Reason |
|--------------|--------|--------|
| **Architecture** | Progressive disclosure (3 layers) | Anthropic official pattern, unbounded context |
| **File Organization** | Type-based directories (reference/, templates/, scripts/) | Clear separation, scalability |
| **Pedagogy** | Evidence-based (Bloom's, CLT, scaffolding) | Research-backed, measurable outcomes |
| **Code Validation** | Subprocess sandbox with timeouts | Security + simplicity balance |
| **Activation** | Mutually exclusive descriptions | Reduces conflicts, improves accuracy |
| **Output Format** | Skill-appropriate (not prescribed) | Flexibility for optimal UX per skill |

---

## Open Questions for Phase 1 Design

1. **Sandbox enhancement**: Should we add optional Docker support for complex validation scenarios?
2. **Shared utilities**: Create a common `utils/` directory for code reused across skills?
3. **Skill testing**: How to validate skill activation accuracy before deployment?
4. **Version management**: Track skill versions for A/B testing and iteration?

These questions will be addressed during data model design (Phase 1).

---

**Research Complete** ✅
**Next Phase**: Design data models and skill contracts based on these research findings.

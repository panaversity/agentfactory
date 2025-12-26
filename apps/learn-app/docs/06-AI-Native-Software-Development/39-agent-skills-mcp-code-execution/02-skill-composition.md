---
sidebar_position: 2
title: "Skill Composition & Dependencies"
description: "Design skills that work together seamlessly. Learn to compose multiple skills into coordinated workflows by declaring dependencies, managing data flow, and handling errors when skills interact."
keywords: ["skill composition", "dependencies", "data flow", "error handling", "skill testing", "integration patterns"]
chapter: 39
lesson: 2
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Skill Dependency Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design skill dependency graphs without circular dependencies, declare dependencies in YAML, and understand how skills reference each other for reuse"

  - name: "Composition Patterns and Data Flow"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can compose sequential, parallel, and conditional skill workflows; understand data contracts between skills; validate data flow through composition"

  - name: "Error Recovery in Skill Composition"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design error propagation strategies that balance robustness with clarity; implement fallback and retry logic for composed workflows"

learning_objectives:
  - objective: "Design skill dependency graphs that avoid circular dependencies and enable modular skill reuse across projects"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Dependency graph design exercise; validation of acyclic structure"

  - objective: "Compose multiple skills into coordinated workflows using sequential, parallel, and conditional patterns"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Skill composition exercise; data flow diagram validation"

  - objective: "Implement skill-to-skill communication patterns that validate data contracts and handle transformation between skill outputs and inputs"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Data contract definition; integration testing with mock skills"

  - objective: "Test skill composition for integration issues including error propagation, partial failure recovery, and state management"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Testing strategy design; validation of error scenarios"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (dependency declaration, composition patterns, data flow, error propagation, testing, versioning, circular detection, capstone integration) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design a three-skill pipeline where skill C depends on both A and B (parallel then sequential). Implement version constraints (skill A requires skill B >= 1.2). Design error recovery that rolls back partial changes when a composed workflow fails mid-execution."
  remedial_for_struggling: "Start with a two-skill composition: data-fetch-skill → data-process-skill. Focus on one composition pattern (sequential). Practice declaring dependencies in YAML. Test with one error scenario (first skill fails)."
---

# Skill Composition & Dependencies

You've designed individual execution skills in Lesson 1. But real-world problems rarely fit a single skill. Consider this scenario:

Your data pipeline needs to:
1. **Fetch data** from an API (via one skill)
2. **Transform the data** (via another skill)
3. **Validate results** against acceptance criteria (via a third skill)

Each skill works individually. But when you combine them, new challenges emerge: What happens if step 2 receives malformed data from step 1? How do they share context? When should the pipeline stop vs retry? How do you prevent skill A from depending on skill B which depends on skill A (circular dependency)?

This is skill composition—the art and science of making skills work together reliably.

## From Individual Skills to Skill Ecosystems

Lesson 1 taught you to design personas, questions, and principles that make a single skill autonomous. This lesson teaches the next level: designing systems of skills that compose into increasingly sophisticated workflows.

**Why composition matters for Digital FTE production:**

When you build a shippable agent (Digital FTE), you're not building from scratch—you're composing existing skills into a coordinated system. The difference between a mediocre agent and a production-grade one often comes down to how well skills integrate. Poor composition means:

- **Cascading failures**: One skill fails, the entire pipeline breaks
- **Silent data corruption**: Skill A's output doesn't match Skill B's expectations
- **Maintenance nightmares**: Changing Skill A breaks dependent Skill C
- **Wasted capability**: 70% of Skill B's functionality unused because data flows wrong

Smart composition means reliability, reusability, and rapid scaling.

## Pattern 1: Dependency Declaration (Preventing Circular Dependencies)

Before writing any code, you declare: "This skill depends on that skill." This declaration has three critical functions:

1. **Makes dependencies visible**: Future developers (including you) see the whole ecosystem
2. **Prevents circular dependencies**: Declares that Skill A → B → C, never A → B → A
3. **Enables versioning**: Allows skills to evolve independently when dependencies are clear

### The Spec-First Example: Data Processing Pipeline

Let's look at a specification for a skill that composes three sub-skills:

**Specification: Data Processing Pipeline Skill**

```yaml
# Skill intent
name: "data-processing-pipeline-skill"
description: "Orchestrates a three-step data pipeline: fetch raw data, transform, validate results"

# Explicitly declare what this skill depends on
dependencies:
  - skill: "data-fetch-skill"
    version: ">=1.0"
    purpose: "Retrieve raw data from source"

  - skill: "transform-skill"
    version: ">=2.1"
    purpose: "Apply business logic transformations"

  - skill: "validate-skill"
    version: ">=1.5"
    purpose: "Verify output meets acceptance criteria"

# How data flows between dependencies
data_contracts:
  fetch_to_transform:
    source: "data-fetch-skill.output.records"
    target: "transform-skill.input.data"
    validation: "Must be list of dicts with 'id' and 'value' keys"

  transform_to_validate:
    source: "transform-skill.output.results"
    target: "validate-skill.input.records"
    validation: "Must include 'status' and 'processed_at' fields"
```

This specification declares:

- **What dependencies exist**: Three skills this pipeline requires
- **Version compatibility**: Versions that work together (not "I'll break if dependency updates")
- **Data contracts**: What format each skill expects vs produces

**What this prevents:**

- Circular dependencies (data-fetch depends on data-transform which depends on... data-fetch — invalid!)
- Silent data mismatches (transform-skill receives data without required fields)
- Brittle version coupling (skill breaks when dependency updates)

### How to Declare Dependencies in YAML

In your skill's `SKILL.md` file, add a `dependencies` section:

```yaml
---
name: "data-processing-pipeline-skill"
version: "1.0"

# Declare what this skill depends on
dependencies:
  - skill: "data-fetch-skill"
    version: ">=1.0"  # This skill requires data-fetch at version 1.0 or higher
    purpose: "Retrieve data from API"

# Declare the data contracts between skills
data_contracts:
  fetch_output_to_transform_input:
    source: "data-fetch-skill output"
    target: "transform-skill input"
    schema: "Must be list[dict] with keys: id, value, timestamp"
---
```

**Why versioning matters:**

Imagine `data-fetch-skill` updates from version 1.0 to 2.0. The new version adds an optional `retry_count` field. If your pipeline isn't version-constrained, it might work fine. But what if the update changes the output format entirely? Your pipeline breaks silently.

With explicit version constraints (`>=1.0, <2.0`), you declare "I've tested with version 1.x and that's what I need." When version 2.0 releases, you consciously update and re-test before using it.

### Detecting Circular Dependencies (What to Avoid)

A circular dependency looks like this:

```
❌ CIRCULAR (INVALID):
Skill A depends on Skill B
Skill B depends on Skill C
Skill C depends on Skill A  ← Loop! This is a bug.
```

This is invalid because you can't initialize the skills—Skill A needs Skill B, which needs Skill C, which needs Skill A.

**Valid acyclic pattern:**

```
✓ ACYCLIC (VALID):
Skill A (data-fetch)
  ↓ depends on nothing
Skill B (transform)
  ↓ depends on A only
Skill C (validate)
  ↓ depends on B only
```

You can initialize A first, then B (which has A), then C (which has B). No loops.

## Pattern 2: Composition Patterns (Sequential, Parallel, Conditional)

Now that dependencies are declared, how do skills actually execute together?

### Sequential Composition: One Skill Feeds the Next

**Pattern**: Skill A completes → output becomes input to Skill B → Skill B completes → output to Skill C

This is the simplest and most common pattern.

```python
# Pseudocode showing sequential composition
result_a = fetch_skill.execute(source="API")
# Skill A outputs: {"records": [{"id": 1, "value": 100}, ...]}

result_b = transform_skill.execute(data=result_a["records"])
# Skill B outputs: {"transformed": [{"id": 1, "value": 200}, ...]}

result_c = validate_skill.execute(records=result_b["transformed"])
# Skill C outputs: {"valid": true, "summary": "100 records processed"}
```

**When to use**: Most data pipelines follow this pattern. Fetch → Process → Validate.

### Parallel Composition: Multiple Skills Run Simultaneously

**Pattern**: Skill A and Skill B run at the same time (if they don't depend on each other), their outputs combine later

```python
# Pseudocode showing parallel composition
# Both run simultaneously
result_fetch_csv = fetch_csv_skill.execute(source="file.csv")
result_fetch_api = fetch_api_skill.execute(source="https://api.example.com")

# Wait for both to complete, then combine results
combined = merge_skill.execute(
    csv_data=result_fetch_csv["records"],
    api_data=result_fetch_api["records"]
)
```

**When to use**: When you need data from multiple sources and sources are independent (CSV file AND API both have data you need).

### Conditional Composition: Choose Skills Based on Context

**Pattern**: If condition X, run Skill A; else run Skill B

```python
# Pseudocode showing conditional composition
result_fetch = fetch_skill.execute(source="data_source")

# Choice: which skill comes next depends on data
if result_fetch["format"] == "csv":
    result_process = csv_processor_skill.execute(data=result_fetch["raw"])
else:
    result_process = json_processor_skill.execute(data=result_fetch["raw"])

result_validate = validate_skill.execute(records=result_process["results"])
```

**When to use**: When the next step depends on what the previous step found (e.g., "if data is CSV, use CSV parser; if JSON, use JSON parser").

## Pattern 3: Data Flow and Contracts

The most common bug in skill composition happens at the boundaries: Skill A produces JSON with fields `{id, name}`. Skill B expects `{user_id, full_name}`. They don't match. The pipeline breaks silently (data corruption) or loudly (error).

### Data Contracts: Preventing Mismatches

A **data contract** specifies: "When Skill A finishes, its output will have this structure. Skill B expects this structure for input."

```yaml
# In the pipeline skill's SKILL.md
data_contracts:
  # Contract between fetch and transform
  fetch_to_transform:
    source: "data-fetch-skill"
    source_field: "output.records"
    target: "transform-skill"
    target_field: "input.data"

    # Explicitly define what format must be passed
    schema: |
      List of dictionaries:
      [
        {
          "id": string,
          "value": number,
          "timestamp": ISO8601 datetime
        },
        ...
      ]

    # Transformation rule if format doesn't match exactly
    transform: "extract records list from fetch output"
```

### Testing Data Flow

Before assuming data flows correctly, test it:

```python
# Test that skills' output/input contracts align

def test_fetch_to_transform_contract():
    """Verify fetch output matches transform input expectations"""

    # Step 1: Run fetch skill
    fetch_result = fetch_skill.execute(source="test_source")

    # Step 2: Verify fetch output has required fields
    assert "records" in fetch_result, "Fetch missing 'records' field"
    assert isinstance(fetch_result["records"], list), "Records must be list"

    for record in fetch_result["records"]:
        assert "id" in record, "Record missing 'id' field"
        assert "value" in record, "Record missing 'value' field"

    # Step 3: Pass fetch output to transform
    transform_result = transform_skill.execute(data=fetch_result["records"])

    # Step 4: Verify transform received and processed data
    assert "transformed" in transform_result
    assert len(transform_result["transformed"]) == len(fetch_result["records"])
```

This test proves that Skill A's output actually works as Skill B's input.

## Pattern 4: Error Propagation and Recovery

When Skill A succeeds but Skill B fails, what should happen?

**Three strategies, each with tradeoffs:**

### Strategy 1: Fail Immediately (Simple, Breaks Early)

```python
# If any skill fails, stop the entire pipeline

result_a = fetch_skill.execute(source="API")  # Succeeds

result_b = transform_skill.execute(data=result_a["records"])  # FAILS

# Pipeline stops here. User sees: "Transform skill failed: Invalid data"
# Advantage: Clear error, easy to debug
# Disadvantage: No partial results, might waste work already done
```

### Strategy 2: Log and Continue (Optimistic, Loses Data)

```python
# If skill fails, log it and keep going

result_a = fetch_skill.execute(source="API")  # Succeeds
results = [result_a]

result_b = transform_skill.execute(data=result_a["records"])  # FAILS
results.append({"error": "Transform failed", "reason": str(error)})

result_c = validate_skill.execute(records=[])  # Validates nothing!
results.append(result_c)

# Pipeline finishes but with incomplete data. Subtle bugs.
# Advantage: Doesn't crash
# Disadvantage: Data corruption, silent failures
```

### Strategy 3: Rollback on Partial Failure (Safe, Explicit)

```python
# If any skill fails, undo work already done

try:
    result_a = fetch_skill.execute(source="API")  # Succeeds
    checkpoint_a = save_state(result_a)  # Save intermediate state

    result_b = transform_skill.execute(data=result_a["records"])  # FAILS

except SkillError as e:
    # Rollback: Delete any data written, restore previous state
    rollback_state(checkpoint_a)

    # Then either retry with different parameters or fail cleanly
    return {
        "status": "failed",
        "last_successful_step": "fetch",
        "error": str(e),
        "recoverable": True  # User can retry
    }
```

**The right strategy depends on your domain:**

- **Financial transactions**: Strategy 3 (rollback) — never partial updates
- **Data analysis**: Strategy 1 (fail immediately) — clear error, retry with different approach
- **Log aggregation**: Strategy 2 (continue) — collect what you can, report gaps

The key: **Choose explicitly, document it, test all three branches** (success, failure with recovery, irrecoverable failure).

## Bringing It Together: Composing Skills With Intelligence

Now let's see how a full composition works, demonstrating the iterative refinement process where intelligence emerges from collaborative effort.

### Initial Composition: Hardcoded References

A developer writes their first version:

```python
# FIRST ATTEMPT: Hardcoded skill references (rigid, breaks easily)

class DataPipelineSkill:
    def execute(self):
        # Hard-coded to use specific skill versions
        fetch_skill = DataFetchSkill(version="1.0")
        transform_skill = TransformSkill(version="2.1")
        validate_skill = ValidateSkill(version="1.5")

        # Execute in sequence
        fetch_result = fetch_skill.execute(source="api.example.com")
        transform_result = transform_skill.execute(data=fetch_result["records"])
        validate_result = validate_skill.execute(records=transform_result["transformed"])

        return validate_result
```

**Problems with this version:**
- Hard-coded skill names (can't reuse with different skills)
- Hard-coded versions (breaks when dependencies update)
- No error handling (one skill fails, whole pipeline fails)
- No data contract validation (silent corruption possible)

### Refined Composition: Flexible Dependencies & Error Recovery

Through collaborative iteration, the design improves:

**AI Suggests**: "Here's a pattern used in production systems. Instead of hardcoding skills, accept them as parameters. This lets the same pipeline skill work with different dependencies."

**Developer Responds**: "Good, but we also need error recovery. What if the fetch succeeds but transform fails? How should we handle that?"

**AI Refines**: "Add checkpoints between skills. If a skill fails, log which step failed and offer rollback. For your domain, rollback makes sense—don't process partial data."

**Iteration Result:**

```python
# REFINED VERSION: Parameterized skills, error recovery, data contracts

class DataPipelineSkill:
    def __init__(self, fetch_skill=None, transform_skill=None, validate_skill=None):
        # Accept skill dependencies as parameters (flexible)
        self.fetch_skill = fetch_skill or DataFetchSkill()
        self.transform_skill = transform_skill or TransformSkill()
        self.validate_skill = validate_skill or ValidateSkill()

        # Track checkpoints for rollback
        self.checkpoints = {}

    def execute(self, source, validation_rules=None):
        try:
            # Step 1: Fetch
            fetch_result = self.fetch_skill.execute(source=source)
            self.checkpoints["fetch"] = fetch_result

            # Validate fetch output has required fields
            self._validate_data_contract(
                fetch_result,
                expected_schema={"records": list}
            )

            # Step 2: Transform
            transform_result = self.transform_skill.execute(
                data=fetch_result["records"]
            )
            self.checkpoints["transform"] = transform_result

            # Validate transform output
            self._validate_data_contract(
                transform_result,
                expected_schema={"transformed": list}
            )

            # Step 3: Validate
            validate_result = self.validate_skill.execute(
                records=transform_result["transformed"],
                rules=validation_rules
            )

            return {
                "status": "success",
                "result": validate_result,
                "steps_completed": ["fetch", "transform", "validate"]
            }

        except Exception as e:
            # Rollback: Report which step failed
            return {
                "status": "failed",
                "failed_step": self._identify_failed_step(),
                "error": str(e),
                "recovered": False,
                "last_successful": list(self.checkpoints.keys())[-1]
            }

    def _validate_data_contract(self, data, expected_schema):
        """Ensure data matches contract before passing to next skill"""
        for key, expected_type in expected_schema.items():
            if key not in data:
                raise ValueError(f"Data missing required field: {key}")
            if not isinstance(data[key], expected_type):
                raise TypeError(
                    f"Field {key} has wrong type. "
                    f"Expected {expected_type}, got {type(data[key])}"
                )
```

**What improved through iteration:**

1. **AI taught flexibility**: Parameterized dependencies enable reuse
2. **Developer taught constraints**: Error recovery with rollback necessary for their domain
3. **Convergence**: Composition pattern emerged that balances flexibility with safety

## Hands-On Exercise: Compose Existing Skills

Now it's your turn to compose skills. You'll create a simple pipeline that combines two existing skills.

### Your Task

Design (don't implement yet) a skill composition that combines:

- **Skill A**: `python-api-fetcher` (retrieves Python library documentation via API)
- **Skill B**: `documentation-summarizer` (extracts key concepts from documentation)

**Required outputs:**

1. **Dependency declaration** (YAML)
   - List the two dependencies
   - Specify versions (use >=1.0)
   - Declare purpose of each dependency

2. **Data contracts**
   - What does Skill A output? (structure, fields)
   - What does Skill B expect as input?
   - How does Skill A's output become Skill B's input?

3. **Error handling strategy**
   - What should happen if Skill A (fetch) fails? (fail immediately, retry, fallback?)
   - What should happen if Skill B (summarize) fails?
   - Should there be rollback or just logging?

4. **Composition pattern**
   - Is this sequential, parallel, or conditional?
   - Why that pattern?

## Try With AI: Collaborative Composition Design

In this section, you'll work with AI to design a real-world skill composition. The collaboration will show how constraints you bring (domain knowledge) combine with patterns AI suggests (technical expertise) to create something better than either starting point.

### Part 1: Initial Composition Request

Ask AI to help design a skill composition for your domain. Share your constraints:

```
I need to build a data-processing pipeline skill that:
1. Fetches raw data from a CSV file
2. Transforms the data (adds calculated fields, normalizes dates)
3. Validates output against business rules
4. Stores results in a database

What skill composition pattern would work for this?
```

**What you're learning**: You're seeing what composition patterns AI suggests when given a clear specification.

### Part 2: Critical Evaluation

Review AI's suggestion. Ask yourself:

- Does the pattern match my constraints? (If my data needs real-time processing, is the suggested pattern too batch-oriented?)
- What assumptions did AI make? (Does it assume small datasets? Does it assume errors are recoverable?)
- What did AI miss? (Database connection pooling? Partial failure recovery?)

Document your evaluation:

```
Pattern AI suggested: Sequential composition with rollback
What matches my needs: Clear error recovery, rollback prevents partial updates
What assumption I question: AI assumes rollback means deleting written data.
  But I want to preserve fetch/transform steps, only skip validation if it fails
What's missing: Error logging strategy (where do failure details go?)
```

### Part 3: Constraint Teaching

Now tell AI your specific constraints. This teaches AI what your domain actually needs:

```
Let me clarify a few things about my constraints:

1. If fetch fails: Retry up to 3 times with exponential backoff.
   If all retries fail, stop (this is critical data, we can't proceed with missing data)

2. If transform fails: Log the failure and continue.
   We'll manually fix malformed records later

3. If validate fails: Reject the batch but keep the transformed data.
   We want to know what passed validation partially

So error handling isn't uniform—each step has different failure modes.
```

**What emerged**: AI now understands that your domain needs step-specific error handling, not generic rollback. This is domain knowledge you brought that AI wouldn't know without you teaching it.

### Part 4: Refinement Through Testing

AI and you iterate on the design:

**AI suggests**: "Based on your constraints, here's the updated composition:

```yaml
error_strategies:
  fetch:
    on_failure: "retry 3 times, exponential backoff"
    final_action: "fail_entire_pipeline"

  transform:
    on_failure: "log and continue"
    final_action: "mark_partial_batch"

  validate:
    on_failure: "skip but preserve"
    final_action: "return_unvalidated_results"
```

**You respond**: "Good structure, but there's one thing: when fetch retries, it might be a temporary network issue OR a malformed URL. Should we distinguish?"

**AI adapts**: "Yes. Check error type:
- Network error → retry
- Malformed URL → fail immediately (retrying won't help)
- Authentication error → log and escalate to human

This makes retry logic smarter."

**What emerged**: An error handling strategy that neither you nor AI had at the start. You brought domain context (some errors are recoverable, some aren't). AI brought the pattern (conditional error handling based on error type). Together: production-grade robustness.

### Part 5: Final Validation

Compare your original composition request to the refined design:

- What changed from first attempt to final version?
- What constraints changed your thinking?
- What patterns did AI suggest that you hadn't considered?
- Is the result better than what either of you would have designed alone?

This reflection locks in the learning.

---

**Takeaway**: Skill composition isn't about writing perfect code on the first try. It's about making dependencies visible, testing data flow between skills, and designing error recovery that matches your domain's reality. When you get this right, the same composition skill can work with different skill implementations (different fetchers, different processors, same pipeline logic).

In Lesson 3, you'll see how existing skills like `fetching-library-docs` handle composition patterns. You'll reverse-engineer production patterns so you understand what makes composition robust.


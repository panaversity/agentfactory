---
sidebar_position: 7
title: "Full Workflow Orchestration"
description: "Compose MCP-wrapping and script-execution skills into orchestrated workflows. Design state management, error recovery, and convergence criteria. Implement multi-step processes that handle failures gracefully and iterate toward robust solutions."
keywords: [workflow orchestration, skill composition, state management, error recovery, convergence criteria, multi-step workflows, DAG-like execution, fallback strategies, workflow testing]
chapter: 39
lesson: 7
duration_minutes: 50

# HIDDEN SKILLS METADATA
skills:
  - name: "Workflow Orchestration Architecture"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design workflow specifications that compose multiple skills (MCP-wrapping + script-execution) into coordinated sequences with explicit data contracts between steps"

  - name: "State Management for Multi-Step Processes"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement state tracking that enables workflow resumption, intermediate result persistence, and step-by-step validation"

  - name: "Error Recovery Strategy Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design error recovery patterns: identify failure modes, design fallbacks for each, implement retry logic with backoff, and escalate gracefully when recovery is impossible"

  - name: "Convergence Criteria Validation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can specify convergence criteria (when workflow succeeds/fails), implement iteration limits to prevent infinite loops, and validate that workflows reach terminal states"

learning_objectives:
  - objective: "Design workflow specifications that compose skills into orchestrated sequences with explicit step definitions and data contracts"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Students write workflow spec defining intent, step sequence, input/output contracts, success criteria, and error handling strategy"

  - objective: "Implement skill composition patterns that pass data from one skill output to next skill input"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Students build workflow that chains MCP-wrapping skill output as input to script-execution skill, with validation between steps"

  - objective: "Design and test error recovery for each step, including detection, fallback execution, and escalation"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Students document failure mode per step and demonstrate recovery behavior through testing (inject failures, verify recovery)"

  - objective: "Validate workflow convergence through acceptance testing across happy path and diverse error scenarios"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Students run workflow against clean input, malformed input, timeout scenarios, and verify all paths reach defined terminal states"

cognitive_load:
  new_concepts: 9
  assessment: "9 concepts (workflow specification, skill composition, data contracts, state management, error modes, recovery strategies, fallback logic, iteration limits, convergence detection) within B2 limit (7-10) ✓"

differentiation:
  extension_for_advanced: "Implement conditional branching: if Step A produces result type X, follow path 1; if type Y, follow path 2. Design parallel execution for independent steps (fetch documentation in parallel with initial script generation). Implement adaptive retry with exponential backoff and circuit breakers."
  remedial_for_struggling: "Start with a simpler two-step workflow (fetch → analyze) before attempting multi-step orchestration. Use the provided workflow template and walk through one complete execution manually. Focus on understanding state transitions first—what data moves between steps? Then practice error handling for one failure mode."

---

# Full Workflow Orchestration

You've built two types of skills:

1. **MCP-wrapping skills** (Lesson 4) that fetch data from external sources intelligently
2. **Script-execution skills** (Lesson 6) that write, execute, and iterate on code autonomously

But real-world problems rarely fit into single skills. They require composition—using the output of one skill as input to another, handling failures at each step, managing state across the workflow, and knowing when you've succeeded.

This is workflow orchestration.

In this lesson, you'll design workflows that compose skills into coordinated, robust multi-step processes. You'll learn to think about workflows the way production engineers do: as directed acyclic graphs (DAGs) where each step depends on previous steps, each connection has a data contract, and each step can fail in predictable ways that require recovery.

By the end of this lesson, you'll build a complete workflow that fetches data via MCP, analyzes it with scripts, handles failures at each step, and knows when it's done.

## Understanding Workflow Composition

### The Pattern: From Single Skills to Orchestrated Workflows

When you build an individual skill, you design it to solve one problem completely. But most real-world problems involve multiple complementary operations:

**Single Skill**: "Fetch documentation for this library"
**Workflow**: "Fetch documentation → Extract code examples → Generate test cases → Validate test syntax → Return results"

Each step in the workflow is a skill. The workflow orchestrates them.

### Why Workflow Orchestration Matters

Consider a concrete example: A domain expert wants to analyze a codebase and generate documentation. The workflow looks like:

```
Step 1: Fetch code files (MCP)
         ↓
Step 2: Analyze patterns (Script)
         ↓
Step 3: Generate documentation (Script)
         ↓
Step 4: Validate markdown (Script)
         ↓
Success
```

Each step depends on the previous one. Step 2 takes Step 1's output as input. If Step 1 fails, Step 2 can't run. If Step 3 produces invalid markdown, Step 4 detects it and either fixes or escalates.

**Without orchestration**: You'd manually manage each step, handle failures ad-hoc, lose intermediate results when something breaks.

**With orchestration**: The workflow knows the sequence, preserves state, recovers from failures automatically, and reports what succeeded and what didn't.

This is what separates prototype scripts from production systems.

## Step 1: Workflow Specification

Before writing any code or building skill logic, specify what your workflow should do. Use the same principle that guided skill design: **specification first, implementation second**.

### Your Workflow Specification Template

Create a file (workflow-spec.md) with this structure:

```markdown
# [Workflow Name] Specification

## Intent
[What problem does this workflow solve? Why is it valuable?]

## Workflow Steps

### Step 1: [Name]
- Skill used: [MCP-wrapping or script-execution skill]
- Input: [What data/parameters does this step receive?]
- Output: [What does this step produce?]
- Success criteria: [How do you know this step succeeded?]
- Failure modes: [What can go wrong? List 3-5 scenarios]

### Step 2: [Name]
[repeat structure]

### Step N: [Name]
[repeat structure]

## Data Contracts

### Step 1 → Step 2
- Output of Step 1: [exact format, fields, constraints]
- Expected Input to Step 2: [what format does Step 2 expect?]
- Validation: [how do you verify compatibility?]

[repeat for each step transition]

## State Management

- What state must persist across steps? [intermediate results, configuration]
- How is state stored? [memory, file, database?]
- How long must state persist? [during workflow execution, or longer?]

## Convergence Criteria

### Success Path
- When does the workflow succeed? [define terminal success state]
- What must be true at the end? [all steps completed? outputs validated?]

### Failure Paths
- When should the workflow fail? [unrecoverable error?]
- What constitutes "too many retries"? [iteration limit?]
- How do you escalate when automatic recovery fails?

## Error Recovery Strategy

For each failure mode identified in Step definitions:

| Step | Failure Mode | Detection | Recovery | Fallback |
|------|--------------|-----------|----------|----------|
| Step 1 | Network timeout | Timeout exception | Retry with backoff | Escalate |
| Step 2 | Syntax error | Parse failure | Regenerate code | Manual intervention |
| [continue for each failure mode] |

## Testing Plan

- **Happy Path**: Clean input → all steps succeed
- **Graceful Degradation**: Data issue in Step 2 → Step 2 recovers → workflow continues
- **Escalation**: Unrecoverable error in Step 3 → workflow fails cleanly with diagnostic info
- **Edge Cases**: [specific scenarios relevant to your workflow]
```

### Example: Fetch-Analyze-Document Workflow

To make this concrete, here's what a complete specification looks like:

```markdown
# Fetch-Analyze-Document Workflow

## Intent
Automatically generate API documentation for a Python module by:
1. Fetching existing documentation (if any)
2. Analyzing the module structure with script execution
3. Generating markdown documentation
4. Validating output format

Result: A domain expert writes a module; this workflow generates comprehensive documentation without manual effort.

## Workflow Steps

### Step 1: Fetch Documentation
- Skill: MCP-wrapping skill wrapping docs repository
- Input: module_name (string)
- Output: existing_docs (string, markdown), docs_found (boolean)
- Success: Output file contains valid markdown (possibly empty if no existing docs)
- Failure modes:
  - Docs repository unreachable (network error)
  - Module name doesn't exist (returns empty)
  - Docs repository returns malformed data
  - Timeout (docs too large)

### Step 2: Analyze Module Structure
- Skill: Script-execution skill
- Input: module_name, existing_docs
- Output: analysis (JSON): functions, classes, methods, parameters, return types
- Success: JSON parses without error, all public APIs documented
- Failure modes:
  - Module doesn't exist (import error)
  - Module has syntax errors (can't import)
  - Analysis script syntax error
  - Analysis script timeout (module too complex)

### Step 3: Generate Documentation
- Skill: Script-execution skill
- Input: module_name, analysis (JSON)
- Output: generated_docs (string, markdown)
- Success: Markdown is valid (Step 4 validates)
- Failure modes:
  - Script generation failure (can't create code)
  - Script execution timeout (takes >10 seconds)
  - Output is not markdown (generation bug)

### Step 4: Validate Output
- Skill: Script-execution skill (markdown validator)
- Input: generated_docs
- Output: validation_result (boolean), issues (list of strings)
- Success: validation_result = true, issues = []
- Failure modes:
  - Markdown parser fails (generated docs invalid)
  - Required sections missing

## Data Contracts

### Step 1 → Step 2
- Output: { existing_docs: string, docs_found: boolean }
- Step 2 Input: module_name, existing_docs
- Validation: module_name is non-empty string; existing_docs is valid markdown or empty string

### Step 2 → Step 3
- Output: { functions: [...], classes: [...] }
- Step 3 Input: module_name, analysis
- Validation: JSON is valid; has expected keys

### Step 3 → Step 4
- Output: generated_docs (string)
- Step 4 Input: generated_docs
- Validation: String is non-empty; contains markdown syntax

## State Management
- Persisted across steps: module_name (input), analysis (output of Step 2), generated_docs (output of Step 3)
- Stored in: in-memory dictionary during execution
- Duration: Cleared after workflow completes (success or failure)

## Convergence Criteria

### Success
- All 4 steps complete without escalation
- validation_result (Step 4) = true
- Output: generated_docs (string) + validation report

### Failure
- Step 1 reaches retry limit (3 retries)
- Step 2 can't import module (unrecoverable)
- Step 3 timeout (>10 sec means module too complex for automated docs)
- Step 4 validation fails after Step 3 regeneration (2 attempts max)

## Error Recovery Strategy

| Step | Failure | Detection | Recovery | Fallback |
|------|---------|-----------|----------|----------|
| 1 | Network timeout | No response after 5s | Retry with backoff (1s, 2s, 4s) | Assume no existing docs; continue |
| 1 | Module not found | Empty response | Return empty docs; continue to Step 2 | Continue |
| 1 | Malformed docs | Parse fails | Log error; use empty docs | Continue |
| 2 | Import error | ImportError exception | Invalid module; escalate | FAIL workflow |
| 2 | Syntax error in analysis script | Parse error | Regenerate script (1 attempt) | Escalate |
| 2 | Timeout | >30 seconds | Module too complex | FAIL workflow |
| 3 | Generation timeout | >10 seconds | Use template fallback | Continue to validation |
| 3 | Output not markdown | Parse fails | Regenerate (1 attempt) | Escalate |
| 4 | Invalid markdown | Validation fails | If Step 3 hasn't been retried, regenerate | FAIL workflow |

## Testing Plan
1. Happy path: Existing library (e.g., simple module with docstrings)
2. Graceful degradation: Module with no existing docs → Steps 1 returns empty → continue
3. Error recovery: Network timeout in Step 1 → retry → succeed
4. Escalation: Module doesn't exist → Step 2 detects import error → workflow fails cleanly
5. Edge case: Generated docs with missing sections → Step 4 detects → mark for manual review
```

Notice the structure: **specification is explicit about what each step does, how steps connect, and what failures look like before any implementation happens**. This is spec-first thinking applied to workflows.

## Step 2: Skill Composition Design

Once you've specified the workflow, you design how skills compose. This is about data contracts—explicitly defining how output from one skill becomes input to the next.

### The Composition Pattern

When Skill A outputs data that Skill B consumes, you need:

1. **Output Format Contract** (Skill A defines what it produces)
   ```
   Skill A output: {
     data: [...],
     metadata: { success: true, timestamp: ... },
     errors: []
   }
   ```

2. **Input Format Contract** (Skill B defines what it accepts)
   ```
   Skill B input: {
     data: [...],  // from Skill A.output.data
     configuration: { ... }
   }
   ```

3. **Validation Layer** (workflow validates compatibility)
   ```
   Before calling Skill B:
   - Check Skill A succeeded (metadata.success = true)
   - Validate Skill A output format matches Skill B input format
   - If mismatch, either transform or fail
   ```

### Example: MCP-Wrapping → Script-Execution

Let's say your workflow is:

**Step 1 (MCP-Wrapping)**: Fetch API documentation
**Step 2 (Script-Execution)**: Analyze the documentation for patterns

**Skill A Output** (from MCP-wrapping skill):
```python
{
  "status": "success",
  "documentation": "# React useState\n\nThe useState hook...",
  "library": "react",
  "topic": "useState",
  "token_count": 342,
  "errors": []
}
```

**Skill B Input** (what script-execution skill expects):
```python
{
  "source_text": "...",  # documentation to analyze
  "analysis_type": "code_patterns"  # what kind of analysis?
}
```

**Composition Logic** (workflow orchestrates the handoff):
```python
# Step 1: Fetch documentation via MCP-wrapping skill
result_step1 = mcp_wrapping_skill(
  library="react",
  topic="useState"
)

# Validate Step 1 success
if not result_step1["status"] == "success":
  fail_workflow("Fetch step failed", result_step1)

# Step 2: Analyze documentation via script-execution skill
result_step2 = script_execution_skill(
  source_text=result_step1["documentation"],  # Pass Step 1 output
  analysis_type="code_patterns"
)

# Validate Step 2 success
if result_step2["status"] != "success":
  recover_or_escalate(result_step2)
```

The key insight: **The workflow orchestrator is responsible for validating that one skill's output matches the next skill's input requirements**. If they don't match, the workflow either transforms the data or fails gracefully.

## Step 3: Error Recovery Design

Every step can fail. The question is: how does the workflow recover?

### The Recovery Hierarchy

Design error recovery as a decision tree:

```
Step fails
├─ Is this error recoverable?
│  ├─ YES: Can we retry?
│  │  ├─ YES: Retry with backoff (up to N times)
│  │  │  ├─ Succeeds → continue workflow
│  │  │  └─ Fails N times → use fallback
│  │  └─ NO: Use fallback strategy
│  │     ├─ Fallback succeeds → continue workflow
│  │     └─ Fallback fails → escalate
│  └─ NO: This step is essential
│     └─ Escalate (workflow terminates, reports error)
```

### Implementing Recovery Patterns

**Pattern 1: Retry with Backoff**
```python
max_retries = 3
backoff_times = [1, 2, 4]  # seconds

for attempt in range(max_retries):
  try:
    result = perform_step()
    return result
  except TemporaryError:
    if attempt < max_retries - 1:
      wait(backoff_times[attempt])
      continue
    else:
      # All retries exhausted
      use_fallback_strategy()
```

Why backoff? To avoid overwhelming a failing service. First retry in 1 second, next in 2, next in 4. Often the service recovers in that time.

**Pattern 2: Fallback Strategy**
```python
try:
  result = fetch_from_primary_source()
except Timeout:
  # Primary source failed, try fallback
  result = fetch_from_cached_copy()
  if result is None:
    escalate("Both primary and fallback failed")
  else:
    log("Using fallback result", result)
```

**Pattern 3: Graceful Degradation**
```python
# Step 1: Fetch full documentation
docs = fetch_docs()
if docs is None:
  # Continue with minimal docs instead of failing
  docs = ""  # Empty, but workflow continues

# Step 2: Analyze whatever we have
analysis = analyze(docs)  # Works with empty docs
```

### Testing Error Recovery

Effective error recovery requires testing failures, not just happy paths. For each failure mode:

1. **Inject the failure**: Call the step with bad input or simulate network error
2. **Verify detection**: The workflow recognizes the failure occurred
3. **Verify recovery**: The recovery strategy executes as designed
4. **Verify state**: Workflow is in correct state after recovery (ready to retry, or escalated, or using fallback)

Example test case:
```python
def test_step1_network_timeout_recovery():
  # Inject network timeout
  mock_mcp_server.timeout_after(0.1)  # Timeout after 100ms

  # Execute workflow
  result = workflow.execute(input_data)

  # Verify recovery: workflow detected timeout
  assert result["recovery_used"] == "retry"
  # Verify state: workflow made multiple attempts
  assert result["attempts"] > 1
  # Verify fallback: workflow continued despite timeout
  assert result["step_1_fallback_docs"] == ""
```

## Step 4: Convergence Strategy

Every workflow must reach a terminal state: either success or failure. If you implement poorly, workflows can get stuck in infinite loops, consuming resources indefinitely.

Convergence strategy defines when to stop.

### Defining Convergence

**Success Convergence**:
- All steps completed without escalation
- All outputs validated against acceptance criteria
- Workflow terminator reports success and outputs results

**Failure Convergence**:
- A step reached max retries without recovery
- Fallback strategy failed
- Critical step determined to be unrecoverable
- Workflow terminates with error report

### Iteration Limits

Never allow infinite iteration. For each step, define:

```python
step_config = {
  "max_retries": 3,           # How many times to retry on failure?
  "retry_backoff": [1, 2, 4], # Wait times between retries (seconds)
  "timeout_seconds": 10,       # Maximum time step can run
  "max_regenerations": 2       # For script-execution: max code rewrites
}
```

When limits are reached, escalate.

### Convergence Validation

To ensure workflows converge, implement a convergence validator:

```python
def validate_convergence(workflow_state):
  """Verify workflow is progressing toward terminal state."""

  # Check 1: No infinite loops
  if workflow_state["step_retries"] > MAX_RETRIES_TOTAL:
    return False, "Excessive retries, possible infinite loop"

  # Check 2: Steps are progressing
  if workflow_state["current_step"] == workflow_state["last_step"]:
    elapsed = time.now() - workflow_state["step_start_time"]
    if elapsed > workflow_state["timeout"]:
      return False, "Step timeout exceeded"

  # Check 3: Success criteria achievable
  if workflow_state["failures_to_recovery"] > MAX_FAILURES:
    return False, "Too many failures, recovery unlikely"

  return True, "Workflow converging normally"
```

## Step 5: Implementation & Testing

Now that you've specified the workflow, designed composition, recovery, and convergence, implementation becomes straightforward. You're not figuring out *what* to do—the specification defines that. You're writing code that faithfully executes the specification.

### Build Your Workflow

Starting with your specification:

1. **Define the workflow orchestrator** (Python class or function that manages the sequence)
2. **Initialize skill instances** (MCP-wrapping skills and script-execution skills)
3. **Implement step execution** (call each skill, validate outputs)
4. **Implement state management** (track intermediate results, logs)
5. **Implement error recovery** (retry logic, fallbacks, escalation)
6. **Implement convergence detection** (terminal state validation)

Here's a skeleton:

```python
class Workflow:
  def __init__(self, spec: WorkflowSpec):
    self.spec = spec
    self.state = {
      "current_step": 0,
      "results": {},
      "errors": [],
      "attempt_count": 0
    }

  def execute(self, input_data):
    """Run workflow from start to terminal state."""

    self.state["input"] = input_data

    for step_num, step_spec in enumerate(self.spec.steps):
      self.state["current_step"] = step_num

      try:
        # Execute step with recovery
        result = self.execute_step_with_recovery(step_spec)
        self.state["results"][step_spec.name] = result

      except UnrecoverableError as e:
        self.state["errors"].append(e)
        return self.fail(f"Step {step_num} failed: {e}")

    # Validate convergence
    if not self.validate_convergence():
      return self.fail("Workflow did not converge")

    return self.succeed()

  def execute_step_with_recovery(self, step_spec):
    """Execute single step with retry and fallback."""

    for attempt in range(step_spec.max_retries):
      try:
        # Get input from previous step output
        step_input = self.prepare_step_input(step_spec)

        # Execute the skill
        result = step_spec.skill.execute(step_input)

        # Validate output
        if not self.validate_step_output(step_spec, result):
          raise ValueError(f"Step output invalid: {result}")

        return result

      except Exception as e:
        if attempt < step_spec.max_retries - 1:
          # Retry with backoff
          wait_time = step_spec.retry_backoff[attempt]
          time.sleep(wait_time)
          continue
        else:
          # All retries exhausted, try fallback
          return self.apply_fallback(step_spec, e)

  def prepare_step_input(self, step_spec):
    """Build step input from previous outputs."""
    # Map previous step outputs to this step's inputs
    return {
      key: self.state["results"][source]
      for key, source in step_spec.input_mapping.items()
    }

  def validate_step_output(self, step_spec, result):
    """Verify step output matches expected format."""
    # Check result has required fields
    return all(field in result for field in step_spec.output_fields)

  def apply_fallback(self, step_spec, error):
    """Execute fallback strategy for failed step."""
    if step_spec.fallback_strategy == "use_empty":
      return {}
    elif step_spec.fallback_strategy == "escalate":
      raise error
    else:
      # Custom fallback logic
      pass

  def validate_convergence(self):
    """Verify workflow reached terminal state."""
    return len(self.state["errors"]) == 0

  def succeed(self):
    return {
      "status": "success",
      "results": self.state["results"],
      "state_log": self.state
    }

  def fail(self, message):
    return {
      "status": "failed",
      "message": message,
      "errors": self.state["errors"],
      "state_log": self.state
    }
```

### Testing the Workflow

Test all paths: happy, degradation, recovery, escalation.

```python
def test_workflow_happy_path():
  """All steps succeed."""
  workflow = Workflow(spec)
  result = workflow.execute(clean_input)
  assert result["status"] == "success"

def test_workflow_step1_timeout_recovery():
  """Step 1 times out, retries, succeeds."""
  mock_step1.timeout_after(0.1)
  workflow = Workflow(spec)
  result = workflow.execute(input_data)
  assert result["status"] == "success"
  assert result["state_log"]["results"]["step1"]["retry_count"] == 1

def test_workflow_step2_unrecoverable():
  """Step 2 has unrecoverable error, workflow escalates."""
  mock_step2.raise_error(ImportError("Module not found"))
  workflow = Workflow(spec)
  result = workflow.execute(input_data)
  assert result["status"] == "failed"
  assert "unrecoverable" in result["message"]

def test_workflow_convergence():
  """Workflow doesn't enter infinite loop."""
  workflow = Workflow(spec)
  # Set very tight timeout
  workflow.max_execution_time = 5
  result = workflow.execute(pathological_input)
  # Should converge (succeed or fail) before timeout
  assert result["status"] in ["success", "failed"]
```

---

## Try With AI

Now you're ready to design and build a complete orchestrated workflow. You've learned the specification-first approach, skill composition, error recovery, and convergence validation. Use your knowledge to guide AI in building a workflow that actually works.

### Prompt 1: Workflow Design

**What you're learning**: Translating domain problems into workflow specifications; designing step sequences that compose existing skills.

```
Design a workflow specification for this problem:

"A team maintains a codebase with outdated documentation. They want to automatically update documentation for any module that changes. The workflow should:
1. Detect which modules changed (given a git commit)
2. Fetch existing documentation for changed modules
3. Analyze the module code to extract current API
4. Generate updated documentation
5. Validate the generated documentation is valid markdown
6. Return the updated docs (or report if generation failed)

Write the specification following this structure:
- Intent: [Problem solved, value provided]
- Workflow Steps: [For each step: name, input, output, success criteria, failure modes]
- Data Contracts: [How Step N output maps to Step N+1 input]
- Error Recovery: [For each failure mode, recovery strategy]
- Convergence Criteria: [When workflow succeeds/fails]

Be specific about what each step receives and produces. Define at least 3 failure modes per step."
```

**What you're learning**: How to compose MCP-wrapping skills with script-execution skills; explicit data contracts between steps.

### Prompt 2: Composition & Error Recovery

```
Given this workflow specification, design the error recovery strategy:

[Paste your workflow spec from Prompt 1]

For each step, answer:
1. What's the most likely failure for this step?
2. Is it recoverable (can we retry, or use fallback)?
3. If recoverable: how many retries? What backoff timing?
4. If fallback: what's the fallback action?
5. If unrecoverable: what does escalation look like?

Create a table mapping each failure mode to recovery:
| Step | Failure | Type | Recovery | Fallback | Max Attempts |
|------|---------|------|----------|----------|--------------|

Also describe:
- What state must persist across steps (intermediate results)?
- How long does state persist (during workflow execution, or permanently)?
- What iteration limits prevent infinite loops?"
```

**What you're learning**: How to think about error recovery like a production engineer—distinguishing recoverable from unrecoverable errors, designing fallbacks, and preventing infinite loops.

### Prompt 3: Implementation & Testing Strategy

```
Now design how you'd test this workflow to ensure it converges properly:

[Paste your workflow spec and error recovery strategy]

For this workflow, define test cases covering:

1. **Happy Path**: [Specific scenario where all steps succeed]
2. **Graceful Degradation**: [Scenario where one step can't complete normally but workflow continues using fallback]
3. **Error Recovery**: [Specific failure mode that should be recovered via retry]
4. **Escalation**: [Unrecoverable error that terminates workflow]
5. **Convergence Validation**: [Confirm workflow doesn't infinite loop]

For each test, specify:
- Input data
- What should happen at each step
- Expected output or error
- How you'll verify recovery/failure/convergence"
```

**What you're learning**: How orchestrated workflows differ from single skills—you're not just testing that each step works, you're testing the whole system reaches a terminal state (success or failure) without getting stuck.

### Prompt 4: Build the Orchestrator (Optional)

```
Build a Python implementation of this workflow orchestrator:

[Paste your spec and test cases]

Create a Workflow class that:
1. Takes the workflow specification as input
2. Executes steps in sequence, passing output from step N as input to step N+1
3. Implements error recovery: for each step, catch exceptions, retry with backoff, apply fallback if needed
4. Validates convergence: ensures workflow reaches terminal state (success or failure)
5. Logs state throughout execution (for debugging)

Include:
- __init__(spec): Initialize workflow with specification
- execute(input_data): Run workflow from start to terminal state
- execute_step_with_recovery(step_spec): Execute single step with retry and fallback
- validate_convergence(): Check workflow reached terminal state
- succeed() / fail(): Return terminal state results

Also provide one test case that demonstrates recovery behavior (e.g., first attempt fails, retry succeeds)."
```

**What you're learning**: How to implement workflows so that error recovery, state management, and convergence happen automatically rather than manually. This is the difference between a script (sequential commands) and a robust workflow (fault-tolerant orchestration).

